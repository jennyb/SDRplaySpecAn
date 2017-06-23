import sys
import time
import math
from ctypes import *

# int sdr_main(int antenna, int bwkHz, int deviceArg, uint32_t frequency, uint32_t samp_rate, CallbackFn callback, int verbose)
# typedef int (*CallbackFn)(double *levels)

class SdrPlay(object):
    def __init__(self, antenna, bwkHz, deviceArg, frequency, samp_rate, verbose):
        self.lib = cdll.LoadLibrary('libplaysdr.so')
        self.lib.fft_size.restype = c_uint
#        self.CallbackFunc = CFUNCTYPE(c_int, c_int, c_int, c_int, c_int, c_double * self.lib.fft_size())
        self.CallbackFunc = CFUNCTYPE(c_int, POINTER(c_double))
        self.lib.sdr_main.argtypes = c_int, c_int, c_int, c_uint32, c_uint32, self.CallbackFunc, c_int
        self.lib.sdr_main.restype = c_int
        self.antenna = antenna
        self.bwkHz = bwkHz
        self.deviceArg = deviceArg
        self.frequency = frequency
        self.samp_rate = samp_rate
        self.verbose = verbose

    def main(self, callback):
        self.lib.sdr_main(self.antenna, self.bwkHz, self.deviceArg, self.frequency, self.samp_rate, self.CallbackFunc(callback), self.verbose)

    # (float)((frequency - (samp_rate/2)) + (float)(i * (float)(samp_rate/fftSize)))/1e6)
    def iter_freq(self):
        fftSize = self.lib.fft_size()
        for i in xrange(fftSize):
            yield (self.frequency - self.samp_rate / 2.0 + i * self.samp_rate / float(fftSize)) / 1e6

t0 = time.time()

def callback(levels):
    try:
        #print len(list(levels))
    #    assert len(list(sdr.iter_freq())) == len(levels)
        v = zip(sdr.iter_freq(), levels)
        print '\n'.join(['%f,%f' % (f, l) for f, l in v])
    except Exception as e:
        print e
    finally:
        return True #time.time() > t0 + 10

sdr = SdrPlay(0, 8000, 1, 94500000, 10000000, 1 if 'debug' in sys.argv else 0)

#print ','.join([str(f) for f in sdr.iter_freq()])

sdr.main(callback)

