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

#define MAXIMAL_BUF_LENGTH  (256 * 16384)
#define FFT_SIZE            4096  // FFT size
#define LOG2_N              12    // FFT length

#define RSP_LNA             0
#define IF_KHZ              0
#define SET_POINT           -30
#define PPM_OFFSET          0.0f
#define AGC_CONTROL         1

#define MAGNITUDE(z) sqrt(z.re * z.re + z.im * z.im)

typedef bool (*CallbackFn)(double *levels);

typedef struct {
    int mb;
    struct GPU_FFT *fft;
    struct GPU_FFT_COMPLEX *buffer;
    int pos;
    double levels[FFT_SIZE];
    CallbackFn fn;
    bool do_exit;
    pthread_mutex_t mutex;
    pthread_cond_t cv;
} CB_CONTEXT;

// initialise callback context including RAM for IQ samples
int initCbContext(CB_CONTEXT *cbContext, CallbackFn callback)
{
    cbContext->mb = mbox_open();
        
    int ret = gpu_fft_prepare(cbContext->mb, LOG2_N, GPU_FFT_FWD, 1, &cbContext->fft);
    switch (ret) {
        case -1: fprintf(stderr, "Unable to enable V3D. Please check your firmware is up to date.\n"); return -1;
        case -2: fprintf(stderr, "log2_N=%d not supported.  Try between 8 and 22.\n", LOG2_N);         return -1;
        case -3: fprintf(stderr, "Out of memory.  Try a smaller batch or increase GPU memory.\n");     return -1;
        case -4: fprintf(stderr, "Unable to map Videocore peripherals into ARM memory space.\n");      return -1;
        case -5: fprintf(stderr, "Can't open libbcm_host.\n");                                         return -1;
    }

    cbContext->buffer = calloc(MAXIMAL_BUF_LENGTH, sizeof(struct GPU_FFT_COMPLEX));
    cbContext->pos = 0;
    cbContext->fn = callback;
    cbContext->do_exit = false;
    pthread_mutex_init(&cbContext->mutex, NULL);
    pthread_cond_init(&cbContext->cv, NULL);

    return 0;
}

void destroyCbContext(CB_CONTEXT *cbContext)
{
    pthread_cond_destroy(&cbContext->cv);
    pthread_mutex_destroy(&cbContext->mutex);
    free(cbContext->buffer);
    gpu_fft_release(cbContext->fft);
    mbox_close(cbContext->mb);
}

void processBuffer(CB_CONTEXT *cbContext)
{
    // apply blackmanHarris
    for (int i = 0; i < FFT_SIZE; i++) {
        double bhtmp1 = (double)(2 * 3.1415 * i / FFT_SIZE);
        double bhtmp2 = (double)(0.35875 - (0.48829*cos(bhtmp1)) + (0.14128 * cos(2*bhtmp1)) - (0.01168*cos(3*bhtmp1)));
        cbContext->fft->in[i].re = cbContext->buffer[i].re * bhtmp2;                 
        cbContext->fft->in[i].im = cbContext->buffer[i].im * bhtmp2;
    }
    
    gpu_fft_execute(cbContext->fft);
    
    //re-order the FFT output.
    int j = 0;
    for (int i = FFT_SIZE / 2; i < FFT_SIZE; i++) {
        cbContext->levels[j++] = MAGNITUDE(cbContext->fft->out[i]);
    }
    for (int i = 0; i < FFT_SIZE / 2; i++) {
        cbContext->levels[j++] = MAGNITUDE(cbContext->fft->out[i]);
    }

    // notify waiter on the condition variable if callback says to exit
    pthread_mutex_lock(&cbContext->mutex);
    cbContext->do_exit = cbContext->fn(cbContext->levels);
    if (cbContext->do_exit) pthread_cond_signal(&cbContext->cv);
    pthread_mutex_unlock(&cbContext->mutex);
}

void gainCallback(unsigned int gRdB, unsigned int lnaGRdB, void *cbContext)
{
    return;
}

void streamCallback(short *xi, short *xq, unsigned int firstSampleNum,
    int grChanged, int rfChanged, int fsChanged, unsigned int numSamples,
    unsigned int reset, void *_cbContext)
{
    CB_CONTEXT *cbContext = (CB_CONTEXT *)_cbContext;

    for (int i = 0; i < numSamples; i++) {
        cbContext->buffer[cbContext->pos].re = xi[i];
        cbContext->buffer[cbContext->pos].im = xq[i];
        if (++cbContext->pos == FFT_SIZE) {
            processBuffer(cbContext);
            cbContext->pos = 0;
        }
    }
}

int initDevice(int deviceArg, int antenna) {
    mir_sdr_DeviceT devices[4];
    unsigned int numDevs;
    int device = deviceArg - 1;

    if (antenna < 0 || antenna > 2) {
        fprintf(stderr, "ERROR: antenna value not supported (0 = Ant A, 1 = Ant B, 2 = HiZ)z\n");
        return -1;
    }

    mir_sdr_GetDevices(&devices[0], &numDevs, 4);

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

    mir_sdr_SetDeviceIdx(device);
    
    int devModel = devices[device].hwVer;

    mir_sdr_SetPpm(PPM_OFFSET);

    if (devModel == 2) {
        if (antenna == 1) {
            mir_sdr_RSPII_AntennaControl(mir_sdr_RSPII_ANTENNA_A);
        } else {
            mir_sdr_RSPII_AntennaControl(mir_sdr_RSPII_ANTENNA_B);
        }

        if (antenna == 2) {
            mir_sdr_AmPortSelect(1);
        }
    }

    return devModel;
}

bool check_bw(int bwkHz) {
    switch (bwkHz) {
        case 200:
        case 300:
        case 600:
        case 1536:
        case 5000:
        case 6000:
        case 7000:
        case 8000:
            return true;
        default:
            return false;
    }
}

unsigned int fft_size() {
    return FFT_SIZE;
}

int sdr_main(int antenna, int bwkHz, int device, uint32_t frequency, uint32_t samp_rate, CallbackFn callback, int verbose) {
    int gainR = 50;
    int gRdBsystem;
    int samplesPerPacket;

    if (! check_bw(bwkHz)) {
        fprintf(stderr, "ERROR: IF bandwidth (%d kHz) not valid.\n", bwkHz);
        fprintf(stderr, "Valid values: 200, 300, 600, 1536, 5000, 6000, 7000, 8000\n\n");
        return -1;
    }

    if (verbose > 0) {
        mir_sdr_DebugEnable(1);
    }

    int devModel = initDevice(device, antenna);
    if (devModel == -1) {
        return -1;
    }

    CB_CONTEXT cbContext;
    int r = initCbContext(&cbContext, callback);
    if (r == 0) {
        r = mir_sdr_StreamInit(&gainR, (samp_rate/1e6), (frequency/1e6),
            (mir_sdr_Bw_MHzT)bwkHz, (mir_sdr_If_kHzT)IF_KHZ, RSP_LNA, &gRdBsystem,
            mir_sdr_USE_SET_GR_ALT_MODE, &samplesPerPacket, streamCallback,
            gainCallback, &cbContext) == mir_sdr_Success ? 0 : 1;

        if (r == 0) {
            mir_sdr_AgcControl(AGC_CONTROL, SET_POINT, 0, 0, 0, 0, RSP_LNA);

            if (devModel == 2) {
                mir_sdr_RSPII_ExternalReferenceControl(0);
                mir_sdr_RSPII_RfNotchEnable(0);
                mir_sdr_RSPII_BiasTControl(0);
            }

            pthread_mutex_lock(&cbContext.mutex);
            while (! cbContext.do_exit) {
                pthread_cond_wait(&cbContext.cv, &cbContext.mutex);
            }
            pthread_mutex_unlock(&cbContext.mutex);

            mir_sdr_StreamUninit();
        } else {
            fprintf(stderr, "Failed to start SDRplay RSP device.\n");
            fprintf(stderr, "Use verbose mode to see the issue.\n");
        }

        destroyCbContext(&cbContext);
    }

    mir_sdr_ReleaseDeviceIdx();

    return r;
}
