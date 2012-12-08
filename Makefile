CC=gcc
LD=gcc

CFLAGS=-Wall -Wno-format -g -I/opt/vc/include/IL -I/opt/vc/include -I/opt/vc/include/interface/vcos/pthreads -DSTANDALONE -D__STDC_CONSTANT_MACROS -D__STDC_LIMIT_MACROS -DTARGET_POSIX -D_LINUX -D_REENTRANT -D_LARGEFILE64_SOURCE -D_FILE_OFFSET_BITS=64 -U_FORTIFY_SOURCE -DHAVE_LIBOPENMAX=2 -DOMX -DOMX_SKIP64BIT -ftree-vectorize -pipe -DUSE_EXTERNAL_OMX -DHAVE_LIBBCM_HOST -DUSE_EXTERNAL_LIBBCM_HOST -DUSE_VCHIQ_ARM
LDFLAGS=-Xlinker -L/opt/vc/lib/ # -Xlinker --verbose
LIBS=-lavformat -lopenmaxil -lbcm_host -lvcos -lpthread
OFILES=omxtx.o

.PHONY: all clean install dist

all: omxtx

.c.o:
	$(CC) $(CFLAGS) -c $<

omxtx: omxtx.o
	$(CC) $(LDFLAGS) $(LIBS) -o omxtx $(OFILES)

remuxer: remuxer.o
	$(CC) $(LDFLAGS) $(LIBS) -o remuxer remuxer.o

clean:
	rm -f *.o omxtx
	rm -rf dist

dist: clean
	mkdir dist
	cp omxtx.c Makefile dist
	FILE=omxtx-`date +%Y%m%dT%H%M%S`.tar.bz2 && tar cvf - --exclude='.*.sw[ponml]' dist | bzip2 > $$FILE && echo && echo $$FILE
