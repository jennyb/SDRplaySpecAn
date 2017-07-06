/*
 * play_lib, update to function as a shared library
 * 
 * play_sdr, an update to work with SDRplay RSP devices
 *
 * rtl-sdr, turns your Realtek RTL2832 based DVB dongle into a SDR receiver
 * Copyright (C) 2012 by Steve Markgraf <steve@steve-m.de>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>
#include <pthread.h>

#include "mailbox.h"
#include "gpu_fft.h"
#include "mirsdrapi-rsp.h"

#define MAGNITUDE(z) sqrt(z.re * z.re + z.im * z.im)
#define SKIP_N  2

unsigned int fftSize = 4096;
unsigned int log2_fftSize = 12;

typedef struct {
    int gRdB;
    double fsMHz;
    double rfMHz;
    mir_sdr_Bw_MHzT bwType;
    mir_sdr_If_kHzT ifType;
    double ppmOffset;
    mir_sdr_SetGrModeT setGrMode;
    mir_sdr_AgcControlT agcControl;
    int setPoint_dBfs;
    int lnaState;

    int gRdBsystem;
    int samplesPerPacket;
    int init_r;
    bool reinit;
} SDR_CONFIG;

typedef enum {
    PlayLib_CONTINUE = 0,
    PlayLib_REINIT   = 1,
    PlayLib_EXIT     = 2
} CallbackReturnT;

typedef CallbackReturnT (*CallbackFn)(unsigned int gRdB, unsigned int lnaGRdB, mir_sdr_GainValuesT *gains, double *levels);

typedef struct {
    SDR_CONFIG *sdr;                // pointer to SDR config struct passed in to sdr_main()
    bool bhWindow;                  // whether to apply the Blackman-Harris window
    int devModel;                   // the dev model determined from the antenna

    int skip;                       // don't call the callback until we get to skip

    int mb;                         // 'mailbox' handle used by GPU FFT
    struct GPU_FFT *fft;            // input/output for GPU FFT
    unsigned int pos;               // current position into fft->in

    unsigned int gRdB;              // updated by gainCallback()
    unsigned int lnaGRdB;           // updated by gainCallback()

    CallbackFn fn;
    mir_sdr_GainValuesT gains;
    int action;
    double *levels;
    pthread_mutex_t mutex;
    pthread_cond_t cv;
} CB_CONTEXT;

// initialise callback context including RAM for IQ samples
int initCbContext(CB_CONTEXT *cbContext, SDR_CONFIG *sdr, bool bhWindow, int devModel, CallbackFn callback) {
    cbContext->mb = mbox_open();
        
    int ret = gpu_fft_prepare(cbContext->mb, log2_fftSize, GPU_FFT_FWD, 1, &cbContext->fft);
    switch (ret) {
        case -1: fprintf(stderr, "Unable to enable V3D. Please check your firmware is up to date.\n"); return -1;
        case -2: fprintf(stderr, "log2_N=%d not supported.  Try between 8 and 22.\n", log2_fftSize);   return -1;
        case -3: fprintf(stderr, "Out of memory.  Try a smaller batch or increase GPU memory.\n");     return -1;
        case -4: fprintf(stderr, "Unable to map Videocore peripherals into ARM memory space.\n");      return -1;
        case -5: fprintf(stderr, "Can't open libbcm_host.\n");                                         return -1;
    }

    cbContext->sdr = sdr;
    cbContext->bhWindow = bhWindow;
    cbContext->devModel = devModel;
    cbContext->pos = 0;
    cbContext->gRdB = 0;
    cbContext->lnaGRdB = 0;
    cbContext->fn = callback;
    cbContext->action = PlayLib_CONTINUE;
    cbContext->levels = calloc(fftSize, sizeof(double));
    pthread_mutex_init(&cbContext->mutex, NULL);
    pthread_cond_init(&cbContext->cv, NULL);

    return 0;
}

void destroyCbContext(CB_CONTEXT *cbContext) {
    pthread_cond_destroy(&cbContext->cv);
    pthread_mutex_destroy(&cbContext->mutex);
    free(cbContext->levels);
    gpu_fft_release(cbContext->fft);
    mbox_close(cbContext->mb);
}

void processFft(CB_CONTEXT *cbContext) {
    // apply Blackman-Harris window?
    if (cbContext->bhWindow) {
        for (int i = 0; i < fftSize; i++) {
            double bhtmp1 = (double)(2 * 3.1415 * i) / fftSize;
            double bhtmp2 = (double)(0.35875 - (0.48829*cos(bhtmp1)) + (0.14128 * cos(2*bhtmp1)) - (0.01168*cos(3*bhtmp1)));
            cbContext->fft->in[i].re *= bhtmp2;                 
            cbContext->fft->in[i].im *= bhtmp2;
        }
    }
    
    // ask the GPU to compute the FFT
    gpu_fft_execute(cbContext->fft);
    
    // re-order the FFT output
    int j = 0;
    for (int i = fftSize / 2; i < fftSize; i++) {
        cbContext->levels[j++] = MAGNITUDE(cbContext->fft->out[i]);
    }
    for (int i = 0; i < fftSize / 2; i++) {
        cbContext->levels[j++] = MAGNITUDE(cbContext->fft->out[i]);
    }
}

void gainCallback(unsigned int gRdB, unsigned int lnaGRdB, void *_cbContext) {
    CB_CONTEXT *cbContext = (CB_CONTEXT *)_cbContext;
    cbContext->gRdB = gRdB;
    cbContext->lnaGRdB = lnaGRdB;
}

void streamCallback(short *xi, short *xq, unsigned int firstSampleNum,
    int grChanged, int rfChanged, int fsChanged, unsigned int numSamples,
    unsigned int reset, void *_cbContext) {
    CB_CONTEXT *cbContext = (CB_CONTEXT *)_cbContext;
    SDR_CONFIG *sdr = cbContext->sdr;

    if (cbContext->action == PlayLib_EXIT) return;

    if (grChanged > 0 || rfChanged > 0 || fsChanged > 0 || reset > 0) {
        // if system parameters changed, we can't use currently buffered iq data
        cbContext->pos = 0;
        sdr->reinit = true;
    }

    if (grChanged > 0) {
        // packets are unreliable after a gain change - skip next few packets
        cbContext->skip = firstSampleNum + SKIP_N * numSamples;
    }

    for (int i = 0; i < numSamples; i++) {
        cbContext->fft->in[cbContext->pos].re = xi[i];
        cbContext->fft->in[cbContext->pos].im = xq[i];

        if (++cbContext->pos == fftSize) {
            cbContext->pos = 0;

            processFft(cbContext);

            if (firstSampleNum < cbContext->skip) {
                // skip after processing the FFT
                return;
            }

            mir_sdr_GetCurrentGain(&cbContext->gains);

            // notify waiter on the condition variable if callback says to exit
            pthread_mutex_lock(&cbContext->mutex);

            cbContext->action = cbContext->fn(cbContext->gRdB, cbContext->lnaGRdB, &cbContext->gains, cbContext->levels);

            if (cbContext->action == PlayLib_REINIT) {
                sdr->reinit = false;
                sdr->init_r = mir_sdr_Reinit(&sdr->gRdB, sdr->fsMHz, sdr->rfMHz,
                    sdr->bwType, sdr->ifType, mir_sdr_LO_Undefined, sdr->lnaState, &sdr->gRdBsystem,
                    sdr->setGrMode, &sdr->samplesPerPacket, mir_sdr_CHANGE_RF_FREQ);
            } else if (cbContext->action == PlayLib_EXIT) {
                pthread_cond_signal(&cbContext->cv);
            }

            pthread_mutex_unlock(&cbContext->mutex);
        }
    }
}

// initialise SDRPlay device - return -1 on error, device model (>=0) on success
int initDevice(int deviceArg, int antenna) {
    mir_sdr_DeviceT devices[4];
    unsigned int numDevs;
    int device = deviceArg - 1;

    if (antenna < 0 || antenna > 2) {
        fprintf(stderr, "ERROR: antenna value not supported (0 = Ant A, 1 = Ant B, 2 = HiZ)z\n");
        return -1;
    }

    mir_sdr_ErrT r = mir_sdr_GetDevices(&devices[0], &numDevs, 4);
    if (r != mir_sdr_Success) {
        fprintf(stderr, "ERROR (%d): Could not get devices\n", r);
        return -1;
    }

    int devAvail = 0;
    for (int i = 0; i < numDevs; i++) {
        if (devices[i].devAvail == 1) {
            devAvail++;
        }
    }

    if (devAvail == 0) {
        fprintf(stderr, "ERROR: No RSP devices available.\n");
        return -1;
    }

    if (devices[device].devAvail != 1) {
        fprintf(stderr, "ERROR: RSP selected (%d) is not available.\n", (device + 1));
        return -1;
    }

    r = mir_sdr_SetDeviceIdx(device);
    if (r != mir_sdr_Success) {
        fprintf(stderr, "ERROR (%d): Could not set device index\n", r);
        return -1;
    }

    int devModel = devices[device].hwVer;

    if (devModel == 2) {
        if (antenna == 0) {
            mir_sdr_RSPII_AntennaControl(mir_sdr_RSPII_ANTENNA_A);
        } else if (antenna == 1) {
            mir_sdr_RSPII_AntennaControl(mir_sdr_RSPII_ANTENNA_B);
        } else {
            mir_sdr_AmPortSelect(1);
        }
    }

    return devModel;
}

// open SDR Play for streaming data - cbContext is an output
int sdr_open(int antenna, int device, SDR_CONFIG *sdr, bool bhWindow, CallbackFn callback, bool verbose, CB_CONTEXT **cbContext) {
    if (verbose) {
        mir_sdr_DebugEnable(1);
    }

    int devModel = initDevice(device, antenna);
    if (devModel == -1) {
        return -1;
    }

    mir_sdr_ErrT r = mir_sdr_SetPpm(sdr->ppmOffset);
    if (r != mir_sdr_Success) {
        fprintf(stderr, "ERROR (%d): Could not set ppm\n", r);
        return -1;
    }

    *cbContext = (CB_CONTEXT *)malloc(sizeof(CB_CONTEXT));
    int s = initCbContext(*cbContext, sdr, bhWindow, devModel, callback);
    if (s != 0) {
        free(*cbContext);
    }

    return r;
}

// start a streaming thread, calling the callback; wait for it to complete
void sdr_main(CB_CONTEXT *cbContext) {
    SDR_CONFIG *sdr = cbContext->sdr;
    sdr->init_r = mir_sdr_StreamInit(&sdr->gRdB, sdr->fsMHz, sdr->rfMHz,
        sdr->bwType, sdr->ifType, sdr->lnaState, &sdr->gRdBsystem,
        sdr->setGrMode, &sdr->samplesPerPacket, streamCallback,
        gainCallback, cbContext);

    if (sdr->init_r == mir_sdr_Success) {
        mir_sdr_AgcControl(sdr->agcControl, sdr->setPoint_dBfs, 0, 0, 0, 0, sdr->lnaState);

        if (cbContext->devModel == 2) {
            mir_sdr_RSPII_ExternalReferenceControl(0);
            mir_sdr_RSPII_RfNotchEnable(0);
            mir_sdr_RSPII_BiasTControl(0);
        }

        pthread_mutex_lock(&cbContext->mutex);
        while (cbContext->action != PlayLib_EXIT) {
            pthread_cond_wait(&cbContext->cv, &cbContext->mutex);
        }
        pthread_mutex_unlock(&cbContext->mutex);

        mir_sdr_StreamUninit();
    } else {
        fprintf(stderr, "Failed to start SDRplay RSP device.\n");
        fprintf(stderr, "Use verbose mode to see the issue.\n");
    }
}

// close SDR Play: free resources, let go of device, etc.
void sdr_close(CB_CONTEXT *cbContext) {
    destroyCbContext(cbContext);
    mir_sdr_ReleaseDeviceIdx();
}
