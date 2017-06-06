/*
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
#include <errno.h>
#include <signal.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include  <math.h>

#include "mailbox.h"
#include "gpu_fft.h"

#ifndef _WIN32
#include <stdbool.h>
#include <unistd.h>
#include "mirsdrapi-rsp.h"
#else
#include <windows.h>
#include <io.h>
#include <fcntl.h>
#include "mir_sdr.h"
#endif

#define DEFAULT_SAMPLE_RATE		2000000
#define DEFAULT_BUF_LENGTH		(336 * 2) // (16 * 16384)
#define MINIMAL_BUF_LENGTH		672 // 512
#define MAXIMAL_BUF_LENGTH		(256 * 16384)

static int do_exit = 0;

void *cbContext = NULL;

FILE *file,*iqFile;
uint8_t *buf8;
struct GPU_FFT_COMPLEX *buf16;
unsigned int firstSample;
unsigned int processingData = 0;  // set when a buffer is filled with IQ data, reset when FFT completes
unsigned int gNumSamples = 0;
unsigned int fftSize = 4096;
unsigned int tBefore = 0, elapsedTime=0;
int samplesPerPacket, grChanged, fsChanged, rfChanged;
int devModel = 1;
int outputRes = 16;

struct GPU_FFT_COMPLEX *base; 
struct GPU_FFT *fft;

unsigned Microseconds(void) {
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    return ts.tv_sec*1000000 + ts.tv_nsec/1000;
}

double atofs(char *s)
/* standard suffixes */
{
    char last;
    int len;
    double suff = 1.0;
    len = strlen(s);
    last = s[len-1];
    s[len-1] = '\0';
    switch (last) {
        case 'g':
        case 'G':
            suff *= 1e3;
        case 'm':
        case 'M':
            suff *= 1e3;
        case 'k':
        case 'K':
            suff *= 1e3;
            suff *= atof(s);
            s[len-1] = last;
            return suff;
    }
    s[len-1] = last;
    return atof(s);
}

void usage(void)
{
	fprintf(stderr,
		"play_sdr, an I/Q recorder for SDRplay RSP receivers\n\n"
		"Usage:\t[-f frequency_to_tune_to [Hz]\n"
        "\t[-d RSP device to use (default: 1, first found)]\n"
		"\t[-s samplerate (default: 2000000 Hz)]\n"
		"\t[-R gain reduction (default: 50)]\n"
        "\t[-l RSP LNA (default: 0, disabled)]\n"
        "\t[-b Bandwidth in kHz (default: 1536)]\n"
        "\t[-i IF bandwidth in kHz (default: 0)]\n"
        "\t[-p ADC set point in dBfs (default: -30)]\n"
        "\t[-n Broadcast Notch enable* (default: 0, disabled)]\n"
        "\t[-r Refclk output enable* (default: 0, disabled)]\n"
        "\t[-t Bias-T enable* (default: 0, disabled)]\n"
        "\t[-a Antenna select* (0/1/2, default: 0, Port A)]\n"
        "\t[-o PPM offset (default 0.0)]\n"
        "\t[-A IF AGC enable (default: 1, enabled)]\n"
        "\t[-x Output bitresolution (8/16, default: 16)]\n"
        "\t[-v Verbose output (debug) enable (default: 0, disabled)]\n\n"
		"\tfilename (a '-' dumps samples to stdout)\n\n"
        "\tNote: options with * are only availale for the RSP2\n\n");
	exit(1);
}

#ifdef _WIN32
BOOL WINAPI
sighandler(int signum)
{
	if (CTRL_C_EVENT == signum) {
		fprintf(stderr, "Signal caught, exiting!\n");
		processingData = 0;
		do_exit = 1;
		mir_sdr_Uninit();
		return TRUE;
	}
	return FALSE;
}
#else
static void sighandler(int signum)
{
	fprintf(stderr, "Signal caught, exiting!\n");
	do_exit = 1;
	mir_sdr_Uninit();
}
#endif

void gainCallback(unsigned int gRdB, unsigned int lnaGRdB, void *cbContext)
{
    return;
}

void streamCallback(short *xi, short *xq, unsigned int firstSampleNum,
    int grChanged, int rfChanged, int fsChanged, unsigned int numSamples,
    unsigned int reset, void *cbContext)
{
	int i = 0;
	static int j = 0;
    if ( ! processingData ) {
		gNumSamples = numSamples;
		for (i = 0; i < numSamples; i++) {
			if (j >= fftSize) {

				processingData = 1 ;
				j = 0 ;
				break;
			} 
			if (outputRes == 16) {
				buf16[j].re = xi[i];
				buf16[j].im = xq[i];
				j++;
			} else {
				buf8[j++] = (unsigned char) (xi[i] >> 8);
				buf8[j++] = (unsigned char) (xq[i] >> 8);
			}
		}	
	}
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

int main(int argc, char **argv)
{
#ifndef _WIN32
	struct sigaction sigact;
#endif
	char *filename = NULL;
	mir_sdr_ErrT r;
    int opt;
	uint32_t frequency = 100000000;
	uint32_t samp_rate = DEFAULT_SAMPLE_RATE;
	uint32_t out_block_size = DEFAULT_BUF_LENGTH;
    int rspLNA = 0;
    int i;
    int bwkHz = 1536;
    int ifkHz = 0;
    int device = 0;
    int gainR = 50;
    int notchEnable = 0;
    int biasT = 0;
    int antenna = 0; // 0 = Ant A, 1 = Ant B, 2 = HiZ
    long setPoint = -30;
    int refClk = 0;
    mir_sdr_SetGrModeT grMode = mir_sdr_USE_SET_GR_ALT_MODE;
    int gRdBsystem;
    int verbose = 0;
    float ppmOffset = 0.0;
    mir_sdr_DeviceT devices[4];
    unsigned int numDevs;
    int devAvail = 0;
    mir_sdr_RSPII_AntennaSelectT ant = mir_sdr_RSPII_ANTENNA_A;
    int agcControl = 1;
    float signalLevel;
    
    int ret, log2_N, jobs, mb = mbox_open();
    
    //allocate RAM for IQ samples
    if (outputRes == 16) {
		buf16 = malloc(MAXIMAL_BUF_LENGTH * sizeof(struct GPU_FFT_COMPLEX));
	} else {
		buf8 = malloc(MAXIMAL_BUF_LENGTH * 2 * sizeof(uint8_t));
	}
		
    log2_N = 12; // FFT length
    jobs = 1;  // transforms per batch



	ret = gpu_fft_prepare(mb, log2_N, GPU_FFT_REV, jobs, &fft); // call once
    switch(ret) {
        case -1: printf("Unable to enable V3D. Please check your firmware is up to date.\n"); return -1;
        case -2: printf("log2_N=%d not supported.  Try between 8 and 22.\n", log2_N);         return -1;
        case -3: printf("Out of memory.  Try a smaller batch or increase GPU memory.\n");     return -1;
        case -4: printf("Unable to map Videocore peripherals into ARM memory space.\n");      return -1;
        case -5: printf("Can't open libbcm_host.\n");                                         return -1;
    }

	while ((opt = getopt(argc, argv, "a:A:b:d:f:i:l:n:o:p:r:R:s:t:v:x:")) != -1) {
		switch (opt) {
        case 'a':
            antenna = atoi(optarg);
            break;
        case 'A':
            agcControl = atoi(optarg);
            break;
        case 'b':
            bwkHz = atoi(optarg);
            if (!check_bw(bwkHz)) {
                fprintf(stderr, "\nERROR: IF bandwidth (%d kHz) not valid.\n", bwkHz);
                fprintf(stderr, "Valid values: 200, 300, 600, 1536, 5000, 6000, 7000, 8000\n\n");
                usage();
            }
            break;
        case 'd':
            device = atoi(optarg) - 1;
            break;
		case 'f':
			frequency = (uint32_t)atofs(optarg);
			break;
        case 'i':
            ifkHz = atoi(optarg);
            if (!check_if(ifkHz)) {
                fprintf(stderr, "\nERROR: IF frequency (%d kHz) not valid.\n", ifkHz);
                fprintf(stderr, "Valid values: 0, 450, 1620, 2048\n\n");
                usage();
            }
            break;
        case 'l':
            rspLNA = atoi(optarg);
            break;
		case 'n':
			notchEnable = atoi(optarg);
			break;
        case 'o':
            ppmOffset = atof(optarg);
            break;
        case 'p':
            setPoint = atol(optarg);
            break;
        case 'r':
            refClk = atoi(optarg);
            break;
		case 'R':
			gainR = atoi(optarg); 
			break;
		case 's':
			samp_rate = (uint32_t)atofs(optarg);
			break;
        case 't':
            biasT = atoi(optarg);
            break;
        case 'v':
            verbose = atoi(optarg);
            break;
        case 'x':
            outputRes = atoi(optarg);
            if (!check_res(outputRes)) {
                fprintf(stderr, "\nERROR: Only 8 or 16 bit output supported.\n");
                usage();
            }
            break;
		default:
			usage();
			break;
		}
	}

	if (argc <= optind) {
		usage();
	} else {
		filename = argv[optind];
	}

	if(out_block_size < MINIMAL_BUF_LENGTH ||
	   out_block_size > MAXIMAL_BUF_LENGTH ){
		fprintf(stderr,
			"Output block size wrong value, falling back to default\n");
		fprintf(stderr,
			"Minimal length: %u\n", MINIMAL_BUF_LENGTH);
		fprintf(stderr,
			"Maximal length: %u\n", MAXIMAL_BUF_LENGTH);
		out_block_size = DEFAULT_BUF_LENGTH;
	}


#ifndef _WIN32
	sigact.sa_handler = sighandler;
	sigemptyset(&sigact.sa_mask);
	sigact.sa_flags = 0;
	sigaction(SIGINT, &sigact, NULL);
	sigaction(SIGTERM, &sigact, NULL);
	sigaction(SIGQUIT, &sigact, NULL);
	sigaction(SIGPIPE, &sigact, NULL);
#else
	SetConsoleCtrlHandler( (PHANDLER_ROUTINE) sighandler, TRUE );
#endif

	if(strcmp(filename, "-") == 0) { /* Write samples to stdout */
		file = stdout;
#ifdef _WIN32
		_setmode(_fileno(stdin), _O_BINARY);
#endif
	} else {
		file = fopen(filename, "wb");
		if (!file) {
			fprintf(stderr, "Failed to open %s\n", filename);
			goto out;
		}
	}

    iqFile = fopen("IQdata.csv", "wb");
        if (!iqFile) {
            fprintf(stderr, "Failed to open file:IQdata.csv\n");
            goto out;
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
        exit(1);
    }

    if (devices[device].devAvail != 1) {
        fprintf(stderr, "ERROR: RSP selected (%d) is not available.\n", (device + 1));
        exit(1);
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

    r = mir_sdr_StreamInit(&gainR, (samp_rate/1e6), (frequency/1e6),
        (mir_sdr_Bw_MHzT)bwkHz, (mir_sdr_If_kHzT)ifkHz, rspLNA, &gRdBsystem,
        grMode, &samplesPerPacket, streamCallback, gainCallback, &cbContext);

	if (r != mir_sdr_Success) {
		fprintf(stderr, "Failed to start SDRplay RSP device.\n");
        fprintf(stderr, "Use -v 1 (for verbose mode) to see the issue.\n");
		exit(1);
	}

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

	fprintf(stderr, "Writing samples...");
    if (verbose > 0) {
        fprintf(stderr, "\n");
    }
	while (!do_exit) {
		if ( processingData )
		{
			
            //outputIQ data for ananth
            for (i=0; i< fftSize; i++)
            { 
                fprintf(iqFile,("%f,%f\n"),buf16[i].re,buf16[i].im);
            }
            // apply blackmanHarris
			for( i=0; i< fftSize; i++ ){
			    double bhtmp1 = (double)(2 * 3.1415 * i / fftSize);
			    double bhtmp2 = (double)(0.35875 - (0.48829*cos(bhtmp1)) + (0.14128 * cos(2*bhtmp1)) - (0.01168*cos(3*bhtmp1)));
				fft->in[i].re = buf16[i].re * bhtmp2; 				
				fft->in[i].im = buf16[i].im * bhtmp2;
			}	
			gpu_fft_execute(fft);
			for( i=0; i< fftSize; i++ ){
				signalLevel = sqrt (fft->out[i].re * fft->out[i].re + fft->out[i].im * fft->out[i].im);				
				fprintf(file,"%f,",signalLevel);   
			}	
			fprintf(file,"\n,");  
			elapsedTime = tBefore - Microseconds();
			printf("elapsedTime = %dus\n",elapsedTime);
			tBefore = Microseconds();
			processingData = 0;
			
		}
        if (verbose < 1) {
            fprintf(stderr,".");
        }
        //sleep(1);
	}

	if (do_exit) {
		fprintf(stderr, "\nUser cancel, exiting...\n");
	} else {
		fprintf(stderr, "\nLibrary error %d, exiting...\n", r);
    }
	if (file != stdout)
		fclose(file);

    fclose(iqFile);

	mir_sdr_StreamUninit();

    mir_sdr_ReleaseDeviceIdx();
    
    gpu_fft_release(fft);

    if (outputRes == 16) {
	    free (buf16);
    } else {
        free (buf8);
    }

    out:
    return r >= 0 ? r : -r;
}
