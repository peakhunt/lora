obj-m += sx127x.o

KERNEL=/home/hawk/sandbox/OrangePiH2_mainline/kernel

all:
	make ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- -C $(KERNEL) M=$(PWD) modules
clean:
	make ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- -C $(KERNEL) M=$(PWD) clean
