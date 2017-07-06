CFLAGS?=-O2 -g -Wall -fpic -std=gnu99
LDLIBS+= -lpthread -lm -lmirsdrapi-rsp -lrt -ldl
CC?=gcc
PROGNAME=play_sdr
LIBNAME=libplaysdr.so
OBJ=mailbox.o gpu_fft.o gpu_fft_base.o gpu_fft_twiddles.o gpu_fft_shaders.o

all: $(PROGNAME) $(LIBNAME) 

%.o: %.c
	$(CC) $(CFLAGS) -c $<

$(PROGNAME): play_sdr.o $(OBJ)
	$(CC) -o $(PROGNAME) play_sdr.o $(OBJ) $(LDFLAGS) $(LDLIBS)

$(LIBNAME): play_lib.o $(OBJ)
	$(CC) -shared -o $(LIBNAME) $(OBJ) play_lib.o $(LDFLAGS) $(LDLIBS)

install: $(LIBNAME)
	cp $(LIBNAME) /usr/lib
	ldconfig

.PHONY: clean
clean:
	rm -f *.o $(PROGNAME) $(LIBNAME)

