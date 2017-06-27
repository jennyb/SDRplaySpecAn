import sys
import time
import math
from ctypes import *

# int sdr_main(int antenna, int gainR, int bwType, int device, double rfMHz, double fsMHz, CallbackFn callback, int verbose)
# typedef int (*CallbackFn)(double *levels)

class SdrPlay(object):
    def __init__(self, antenna, gainR, bwkHz, deviceArg, frequency, samp_rate, verbose):
        self.lib = cdll.LoadLibrary('libplaysdr.so')
        self.lib.fft_size.restype = c_uint
        self.CallbackFunc = CFUNCTYPE(c_int, POINTER(c_double))
        self.lib.sdr_main.argtypes = c_int, c_int, c_int, c_int, c_double, c_double, self.CallbackFunc, c_int
        self.lib.sdr_main.restype = c_int
        self.antenna = antenna
        self.gainR = gainR
        self.bwkHz = bwkHz
        self.deviceArg = deviceArg
        self.frequency = frequency
        self.samp_rate = samp_rate
        self.verbose = verbose

    def main(self, callback):
        rfMHz = self.frequency / 1e6
        fsMHz = self.samp_rate / 1e6
        sampleN = self.lib.sdr_main(self.antenna, self.gainR, self.bwkHz, self.deviceArg, rfMHz, fsMHz, self.CallbackFunc(callback), self.verbose)

    # (float)((frequency - (samp_rate/2)) + (float)(i * (float)(samp_rate/fftSize)))/1e6)
    def iter_freq(self):
        fftSize = self.lib.fft_size()
        for i in xrange(fftSize):
            yield (self.frequency - self.samp_rate / 2.0 + i * self.samp_rate / float(fftSize)) / 1e6

t0 = time.time()

def callback(levels):
    try:
        v = zip(sdr.iter_freq(), levels)
        print ','.join(['%f' % (math.log(l), ) for f, l in v[2020:2076]])
        print >>sys.stderr, max(v, key=lambda x: x[1]), time.time()
    except BaseException as e:
        print e
    finally:
        return time.time() > t0 + 60

sdr = SdrPlay(0, 70, 8000, 1, 94500000, 10000000, 1 if 'debug' in sys.argv else 0)

#print ','.join([str(f) for f in sdr.iter_freq()])

sdr.main(callback)

