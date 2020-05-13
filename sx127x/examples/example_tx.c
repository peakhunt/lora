#include <stdio.h>
#include <fcntl.h>
#include <stdlib.h>
#include <unistd.h>
#include "sx127x.h"
#include "common.h"

int main(int argc, char** argv) {
	int fd = open("/dev/sx127x0", O_RDWR);
	//char helloworld[] = "hello, world\n";
  char data1[] = { 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xaa, 0xbb, 0xcc, 0xdd };
  char data2[] = { 0x01, 0x20, 0x03, 0x40, 0x05, 0x60, 0x07, 0x80, 0x09, 0xa0, 0x0b, 0xc0, 0x0d };
  int ret;

	if (fd < 0)
		printf("failed to open device\n");

	if (setupradio(fd, SX127X_OPMODE_RXCONTINUOS))
		return 1;


  while(1)
  {
    //ret = write(fd, helloworld, sizeof(helloworld));
    ret = write(fd, data1, sizeof(data1));
    if(ret <= 0)
    {
      printf("failed to tx %d\n", ret);
      return -1;
    }
    printf("sent %d bytes\n", sizeof(data1));
    //printf("sent %d bytes\n", sizeof(helloworld));
    sleep(1);

    ret = write(fd, data2, sizeof(data2));
    if(ret <= 0)
    {
      printf("failed to tx %d\n", ret);
      return -1;
    }
    printf("sent %d bytes\n", sizeof(data2));
    //printf("sent %d bytes\n", sizeof(helloworld));
    sleep(1);
  }
	return 0;
}

