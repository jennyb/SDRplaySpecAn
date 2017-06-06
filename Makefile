CFLAGS?=-O2 -g -Wall
LDLIBS+= -lpthread -lm -lmirsdrapi-rsp -lrt -ldl
CC?=gcc
PROGNAME=play_sdr
OBJ=mailbox.o gpu_fft.o gpu_fft_base.o gpu_fft_twiddles.o gpu_fft_shaders.o

all: play_sdr

%.o: %.c
	$(CC) $(CFLAGS) -c $<

play_sdr: play_sdr.o $(OBJ)
	$(CC) -g -o play_sdr play_sdr.o $(OBJ) $(LDFLAGS) $(LDLIBS)

clean:
	rm -f *.o play_sdr
