import sys
import time
import math
from itertools import islice
import traceback
from ctypes import *

def log(x):
    try:
        return math.log(x)
    except ValueError:
        return 0.0

class SdrConfig(Structure):
    _fields_ = [('gRdB', c_int),
                ('fsMHz', c_double),
                ('rfMHz', c_double),
                ('bwType', c_int),
                ('ifType', c_int),
                ('ppmOffset', c_double),
                ('setGrMode', c_int),
                ('agcControl', c_int),
                ('setPoint_dBfs', c_int),
                ('lnaState', c_int),
                ('gRdBsystem', c_int),
                ('samplesPerPacket', c_int),
                ('init_r', c_int),
                ('reinit', c_bool)]

class GainValues(Structure):
    _fields_ = [('curr', c_float), ('max', c_float), ('min', c_float)]

CallbackFunc = CFUNCTYPE(c_int, c_uint, c_uint, POINTER(GainValues), POINTER(c_double))

Callback_Continue = 0
Callback_Reinit = 1
Callback_Exit = 2

_lib = cdll.LoadLibrary('libplaysdr.so')
_lib.sdr_open.argtypes = c_int, c_int, POINTER(SdrConfig), c_bool, CallbackFunc, c_bool, POINTER(c_void_p)
_lib.sdr_open.restype = c_int
_lib.sdr_main.argtypes = c_void_p,
_lib.sdr_main.restype = None
_lib.sdr_close.argtypes = c_void_p,
_lib.sdr_close.restype = None

FftSize = c_uint.in_dll(_lib, 'fftSize').value

class SdrPlay(object):
    """ API for spectrum scanning with an SDR Play device.

        antenna         Which antenna to use, 0=A, 1=B, 2=HiZ (default A)
        gRdB            Gain reduction dB (default 30dB)
        bwkHz           Bandwidth (default 8MHz)
        device          SDR Play device (default 1)
        rfMHz           Tuner frequency
        fsMHz           Sample frequency (default 10MHz)
        cwMHz           If not None, channelise to this channel width
        verbose         If True, output SDR Play debug to stderr
    """
    def __init__(self, antenna=0, gRdB=30, bwkHz=8000, device=1, rfMHz=None, fsMHz=10, cwMHz=None, verbose=False, **args):
        self.antenna = antenna
        self.device = device
        if rfMHz is None:
            raise ValueError("Tuner frequency (rfMHz) must be specified")
        self.sdr_config = SdrConfig(gRdB, fsMHz, rfMHz, bwkHz, 0, 0, 1, 1, -30, 0, True)
        self.cwMHz = cwMHz
        self.verbose = verbose

    def main(self, callback):
        def _callback(gRdB, lnaGRdB, gains_p, levels_p):
            if self.sdr_config.init_r != 0:
                print >>sys.stderr, "Bad Reinit({})".format(self.sdr_config.init_r)
                return Callback_Exit
            gains = gains_p.contents
            levels = [log(l) for l in list(islice(levels_p, FftSize))]
            if self.cwMHz is not None:
                levels = self._channelise(levels)
            return callback(self.sdr_config.reinit, [gRdB, lnaGRdB, gains.curr, gains.max, gains.min], levels)
        p = c_void_p()
        r = _lib.sdr_open(self.antenna, self.device, self.sdr_config, False, CallbackFunc(_callback), self.verbose, p)
        if r == 0:
            _lib.sdr_main(p)
        _lib.sdr_close(p)

    def n_channels(self):
        if self.cwMHz is None:
            return None
        return int(math.ceil(self.sdr_config.fsMHz / self.cwMHz))

    def freq_0(self):
        if self.cwMHz is not None:
            return self.sdr_config.rfMHz - math.ceil(0.5 * self.sdr_config.fsMHz / self.cwMHz) * self.cwMHz
        else:
            return self.sdr_config.rfMHz - 0.5 * self.sdr_config.fsMHz

    def iter_freq(self):
        if self.cwMHz is None:
            for i in xrange(FftSize):
                yield self.sdr_config.rfMHz - 0.5 * self.sdr_config.fsMHz + i * self.sdr_config.fsMHz / float(FftSize)
        else:
            freq = self.freq_0()
            for _ in xrange(self.n_channels()):
                yield freq
                freq += self.cwMHz

    def _channelise(self, levels):
        levels_in = iter(levels)
        levels_out = []
        level_max = 0
        f0 = self.sdr_config.rfMHz - math.ceil(0.5 * self.sdr_config.fsMHz / self.cwMHz) * self.cwMHz - 0.5 * self.cwMHz
        for i in xrange(FftSize):
            freq = self.sdr_config.rfMHz - 0.5 * self.sdr_config.fsMHz + i * self.sdr_config.fsMHz / float(FftSize)
            if freq >= f0 + self.cwMHz:
                levels_out.append(level_max)
                f0 = freq
                level_max = 0
            level = levels_in.next()
            if level > level_max:
                level_max = level
        return levels_out

if __name__ == "__main__":
    t0 = time.time()

    def callback(reinit, gains, levels):
        global rf
        action = Callback_Continue
        try:
            if time.time() > t0 + 60.0:
                print >>sys.stderr, "Time out"
                action = Callback_Exit

            elif sdr.sdr_config.reinit: # True at start and when a requested frequency change has occurred
                rf = sdr.sdr_config.rfMHz
                sdr.sdr_config.rfMHz += 5.0
                if sdr.sdr_config.rfMHz > 108.0 + 5.0:
                    print >>sys.stderr, "Exit"
                    action = Callback_Exit
                else:
                    print >>sys.stderr, "Change frequency"
                    action = Callback_Reinit

            print >>sys.stderr, sdr.sdr_config.init_r, rf, len(levels), max(levels)
            #print ','.join([str(rf)] + [str(l) for l in levels])
        except BaseException as e:
            traceback.print_exc(sys.stderr)
            action = Callback_Exit
        finally:
            return action

    sdr = SdrPlay(rfMHz=88.0, cwMHz=1.0, verbose='debug' in sys.argv)
    sdr.main(callback)

