#PREFIX=x86_64-w64-mingw32-
#PLATFORM=WIN=1
#SUFFIX=.exe

PREFIX=
PLATFORM=LINUX=1
SUFFIX=

CC=$(PREFIX)gcc

mmm8x8$(SUFFIX): main.o serial.o command.o pattern.o crc16.o
	$(CC) -o mmm8x8$(SUFFIX)  main.o serial.o command.o pattern.o crc16.o 

main.o: main.c serial.h command.h pattern.h crc16.h
	$(CC) -c main.c -I. -D$(PLATFORM)

serial.o: serial.c serial.h
	$(CC) -c serial.c -I. -D$(PLATFORM)

command.o: command.c command.h
	$(CC) -c command.c -I. -D$(PLATFORM)

pattern.o: pattern.c pattern.h
	$(CC) -c pattern.c -I. -D$(PLATFORM)

crc16.o: crc16.c crc16.h 
	$(CC) -c crc16.c -I. -D$(PLATFORM)

clean:
	rm -f mmm8x8$(SUFFIX) *.o
