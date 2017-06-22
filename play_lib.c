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

#define DEFAULT_SAMPLE_RATE       2000000
#define DEFAULT_BUF_LENGTH        (336 * 2) // (16 * 16384)
#define MINIMAL_BUF_LENGTH        672 // 512
#define MAXIMAL_BUF_LENGTH        (256 * 16384)

uint8_t *buf8;
struct GPU_FFT_COMPLEX *buf16;
unsigned int firstSample;
unsigned int gNumSamples = 0;
unsigned int fftSize = 4096;
unsigned int tBefore = 0, elapsedTime=0;
int samplesPerPacket, grChanged, fsChanged, rfChanged;
int devModel = 1;
int outputRes = 16;

struct GPU_FFT_COMPLEX *base; 
struct GPU_FFT *fft;

typedef bool (*CallbackFn)(double *levels);

typedef struct {
    CallbackFn fn;
    bool do_exit;
    pthread_mutex_t mutex;
    pthread_cond_t cv;
} CB_CONTEXT;

void gainCallback(unsigned int gRdB, unsigned int lnaGRdB, void *cbContext)
{
    return;
}

void streamCallback(short *xi, short *xq, unsigned int firstSampleNum,
    int grChanged, int rfChanged, int fsChanged, unsigned int numSamples,
    unsigned int reset, void *_cbContext)
{
    gNumSamples = numSamples;
    int i, j = 0;
    for (i = 0; i < numSamples; i++) {
        if (outputRes == 16) {
            buf16[i].re = xi[i];
            buf16[i].im = xq[i];
        } else {
            buf8[j++] = (unsigned char) (xi[i] >> 8);
            buf8[j++] = (unsigned char) (xq[i] >> 8);
        }
    }    

    // apply blackmanHarris
    for( i=0; i< fftSize; i++ ){
        double bhtmp1 = (double)(2 * 3.1415 * i / fftSize);
        double bhtmp2 = (double)(0.35875 - (0.48829*cos(bhtmp1)) + (0.14128 * cos(2*bhtmp1)) - (0.01168*cos(3*bhtmp1)));
        fft->in[i].re = buf16[i].re * bhtmp2;                 
        fft->in[i].im = buf16[i].im * bhtmp2;
    }    
    
    gpu_fft_execute(fft);
    
    //re-order the FFT output.
    //FIXME check fft properties - are they doubles or floats?
    double levels[fftSize];
    j = 0;
    for( i=fftSize/2; i< fftSize; i++ ){
        levels[j++] = sqrt(fft->out[i].re * fft->out[i].re + fft->out[i].im * fft->out[i].im);                
    }    
    for( i=0; i< fftSize/2; i++ ){
        levels[j++] = sqrt(fft->out[i].re * fft->out[i].re + fft->out[i].im * fft->out[i].im);                
    }

    CB_CONTEXT *cbContext = (CB_CONTEXT *)cbContext;
    pthread_mutex_lock(&cbContext->mutex);
    cbContext->do_exit = cbContext->fn(levels);
    if (cbContext->do_exit) pthread_cond_signal(&cbContext->cv);
    pthread_mutex_unlock(&cbContext->mutex);
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

bool check_if(int ifkHz) {
    switch (ifkHz) {
        case 0:
        case 450:
        case 1620:
        case 2048:
            return true;
        default:
            return false;
    }
}

bool check_res(int res) {
    switch (res) {
        case 8:
        case 16:
            return true;
        default:
            return false;
    }
}

//FIXME have a function to compute this separetely
//write header for results table - calculate frequency per bin
//    for ( i =0; i < fftSize; i++ ) 
//    {
//        fprintf(file,"%f,",(float)((frequency - (samp_rate/2)) + (float)(i * (float)(samp_rate/fftSize)))/1e6);
//    }

//int antenna = 0; // 0 = Ant A, 1 = Ant B, 2 = HiZ

//            if (!check_bw(bwkHz)) {
//                fprintf(stderr, "\nERROR: IF bandwidth (%d kHz) not valid.\n", bwkHz);
//                fprintf(stderr, "Valid values: 200, 300, 600, 1536, 5000, 6000, 7000, 8000\n\n");

//    uint32_t samp_rate = DEFAULT_SAMPLE_RATE;

//            if (!check_res(outputRes)) {
//                fprintf(stderr, "\nERROR: Only 8 or 16 bit output supported.\n");


//    int bwkHz = 1536;

int sdr_main(int antenna, int bwkHz, int deviceArg, uint32_t frequency, uint32_t samp_rate, CallbackFn callback, int verbose) {
    mir_sdr_ErrT r;
    uint32_t out_block_size = DEFAULT_BUF_LENGTH;
    int rspLNA = 0;
    int i;
    int ifkHz = 0;
    int device = deviceArg - 1;
    int gainR = 50;
    int notchEnable = 0;
    int biasT = 0;
    long setPoint = -30;
    int refClk = 0;
    mir_sdr_SetGrModeT grMode = mir_sdr_USE_SET_GR_ALT_MODE;
    int gRdBsystem;
    float ppmOffset = 0.0;
    mir_sdr_DeviceT devices[4];
    unsigned int numDevs;
    int devAvail = 0;
    mir_sdr_RSPII_AntennaSelectT ant = mir_sdr_RSPII_ANTENNA_A;
    int agcControl = 1;
    
    int ret, log2_N, jobs;

    int mb = mbox_open();
    
    //allocate RAM for IQ samples
    if (outputRes == 16) {
        buf16 = malloc(MAXIMAL_BUF_LENGTH * sizeof(struct GPU_FFT_COMPLEX));
    } else {
        buf8 = malloc(MAXIMAL_BUF_LENGTH * 2 * sizeof(uint8_t));
    }
        
    log2_N = 12; // FFT length
    jobs = 1;  // transforms per batch

    ret = gpu_fft_prepare(mb, log2_N, GPU_FFT_FWD, jobs, &fft); // call once
    switch(ret) {
        case -1: printf("Unable to enable V3D. Please check your firmware is up to date.\n"); return -1;
        case -2: printf("log2_N=%d not supported.  Try between 8 and 22.\n", log2_N);         return -1;
        case -3: printf("Out of memory.  Try a smaller batch or increase GPU memory.\n");     return -1;
        case -4: printf("Unable to map Videocore peripherals into ARM memory space.\n");      return -1;
        case -5: printf("Can't open libbcm_host.\n");                                         return -1;
    }


    if(out_block_size < MINIMAL_BUF_LENGTH ||
       out_block_size > MAXIMAL_BUF_LENGTH ){
        fprintf(stderr, "Output block size wrong value, falling back to default\n");
        fprintf(stderr, "Minimal length: %u\n", MINIMAL_BUF_LENGTH);
        fprintf(stderr, "Maximal length: %u\n", MAXIMAL_BUF_LENGTH);
        out_block_size = DEFAULT_BUF_LENGTH;
    }

    if (verbose > 0) {
        mir_sdr_DebugEnable(1);
    }

    mir_sdr_GetDevices(&devices[0], &numDevs, 4);

    for(i = 0; i < numDevs; i++) {
        if(devices[i].devAvail == 1) {
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

    devModel = devices[device].hwVer;

    mir_sdr_SetPpm(ppmOffset);

    if (devModel == 2) {
        if (antenna == 1) {
            ant = mir_sdr_RSPII_ANTENNA_A;
            mir_sdr_RSPII_AntennaControl(ant);
        } else {
            ant = mir_sdr_RSPII_ANTENNA_B;
            mir_sdr_RSPII_AntennaControl(ant);
        }

        if (antenna == 2) {
            mir_sdr_AmPortSelect(1);
        }
    }

    // initialise callback context
    CB_CONTEXT cbContext;
    cbContext.fn = callback;
    cbContext.do_exit = false;
    pthread_mutex_init(&cbContext.mutex, NULL);
    pthread_cond_init(&cbContext.cv, NULL);

    r = mir_sdr_StreamInit(&gainR, (samp_rate/1e6), (frequency/1e6),
        (mir_sdr_Bw_MHzT)bwkHz, (mir_sdr_If_kHzT)ifkHz, rspLNA, &gRdBsystem,
        grMode, &samplesPerPacket, streamCallback, gainCallback, &cbContext);

    if (r == mir_sdr_Success) {
        mir_sdr_AgcControl(agcControl, setPoint, 0, 0, 0, 0, rspLNA);

        if (devModel == 2) {
            if (refClk > 0) {
                mir_sdr_RSPII_ExternalReferenceControl(1);
            } else {
                mir_sdr_RSPII_ExternalReferenceControl(0);
            }

            if (notchEnable > 0) {
                mir_sdr_RSPII_RfNotchEnable(1);
            } else {
                mir_sdr_RSPII_RfNotchEnable(0);
            }

            if (biasT > 0) {
                mir_sdr_RSPII_BiasTControl(1);
            } else {
                mir_sdr_RSPII_BiasTControl(0);
            }
        }

        pthread_mutex_lock(&cbContext.mutex);
        while (! cbContext.do_exit) {
            pthread_cond_wait(&cbContext.cv, &cbContext.mutex);
        }
        pthread_mutex_unlock(&cbContext.mutex);

        mir_sdr_StreamUninit();

    pthread_cond_destroy(&cbContext.cv);
    pthread_mutex_destroy(&cbContext.mutex);
    mir_sdr_ReleaseDeviceIdx();
    gpu_fft_release(fft);
    mbox_close(mb);

    if (outputRes == 16) {
        free (buf16);
    } else {
        free (buf8);
    }

    return r == mir_sdr_Success ? 0 : 1;
}
