CFLAGS?=-O2 -g -Wall -fpic
LDLIBS+= -lpthread -lm -lmirsdrapi-rsp -lrt -ldl
CC?=gcc
PROGNAME=play_sdr
LIBNAME=libplaysdr.so
OBJ=mailbox.o gpu_fft.o gpu_fft_base.o gpu_fft_twiddles.o gpu_fft_shaders.o

all: play_sdr play_lib 

%.o: %.c
	$(CC) $(CFLAGS) -c $<

play_sdr: play_sdr.o $(OBJ)
	$(CC) -g -o play_sdr play_sdr.o $(OBJ) $(LDFLAGS) $(LDLIBS)

play_lib: play_lib.o $(OBJ)
	$(CC) -shared -o $(LIBNAME) $(OBJ) play_lib.o $(LDFLAGS) $(LDLIBS)

install: play_lib
	cp $(LIBNAME) /usr/lib
	ldconfig

clean:
	rm -f *.o play_sdr
