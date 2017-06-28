import sys
import time
import math
import traceback
from ctypes import *

class SdrConfig(Structure):
    _fields_ = [('gRdB', c_int),
                ('fsMHz', c_double),
                ('rfMHz', c_double),
                ('rfMHz_min', c_double),
                ('rfMHz_max', c_double),
                ('dfMHz', c_double), 
                ('bwType', c_int),
                ('ifType', c_int),
                ('ppmOffset', c_double),
                ('setGrMode', c_int),
                ('agcControl', c_int),
                ('setPoint_dBfs', c_int),
                ('lnaState', c_int),
                ('gRdBsystem', c_int),
                ('samplesPerPacket', c_int),
                ('init_r', c_int)]

class GainValues(Structure):
    _fields_ = [('curr', c_float), ('max', c_float), ('min', c_float)]

CallbackFunc = CFUNCTYPE(c_int, POINTER(GainValues), POINTER(c_double))

Callback_Continue = 0
Callback_Reinit = 1
Callback_StepFreq = 2
Callback_Exit = 3

_lib = cdll.LoadLibrary('libplaysdr.so')
_lib.sdr_main.argtypes = c_int, c_int, POINTER(SdrConfig), c_bool, CallbackFunc, c_bool
_lib.sdr_main.restype = c_int

FftSize = c_uint.in_dll(_lib, 'fftSize').value

class SdrPlay(object):
    def __init__(self, antenna, gRdB, bwkHz, device, rfMHz, rfMHz_range, fsMHz, verbose):
        self.antenna = antenna
        self.device = device
        rfMHz_min, rfMHz_max, dfMHz = rfMHz_range
        self.sdr_config = SdrConfig(gRdB, fsMHz, rfMHz, rfMHz_min, rfMHz_max, dfMHz, bwkHz, 0, 0, 1, 1, -30, 0)
        self.verbose = verbose

    def main(self, callback):
        if _lib.sdr_main(self.antenna, self.device, self.sdr_config, False, CallbackFunc(callback), self.verbose) != 0:
            pass

    def iter_freq(self):
        for i in xrange(FftSize):
            yield self.sdr_config.rfMHz - 0.5 * self.sdr_config.fsMHz + i * self.sdr_config.fsMHz / float(FftSize)

t0 = time.time()
t1 = t0

def log(x):
    try:
        return math.log(x)
    except ValueError:
        return 0.0

def callback(gains_p, levels):
    global t0, t1 

    action = Callback_Continue
    try:
        if sdr.sdr_config.init_r != 0:
            raise Exception("Bad Reinit({})".format(sdr.sdr_config.init_r))

        v = zip(sdr.iter_freq(), levels)

        # channelise
        width = 0.1
        v2 = []
        f0 = 0
        for f, l in v:
            if f > f0 + width:
                if f0 > 0:
                    v2.append((f_s / n, l_m))
                f0 = f
                f_s = 0
                l_m = 0
                n = 0
            n += 1
            f_s += f
            if l > l_m:
                l_m = l

        print ','.join(['%f' % (log(l), ) for f, l in v2])
        gains = gains_p.contents
        peak = max(v2, key=lambda x: x[1])
        print >>sys.stderr, 'Tuner freq {:.2f}    Peak freq {:.2f}    gain {:d}    level {:.2f}    time {:.2f}'.format(sdr.sdr_config.rfMHz, peak[0], int(gains.curr), log(peak[1]), time.time() - t0)
        if time.time() > t1 + 5:
            action = Callback_StepFreq
            t1 = time.time()
            print >>sys.stderr, "Change frequency"
    except BaseException as e:
        traceback.print_exc(sys.stderr)
        action = Callback_Exit
    finally:
        return Callback_Exit if time.time() > t0 + 60 else action

sdr = SdrPlay(0, 30, 8000, 1, 94.5, (93.0, 97.0, 0.5), 10, 'debug' in sys.argv)

#print ','.join([str(f) for f in sdr.iter_freq()])

sdr.main(callback)

