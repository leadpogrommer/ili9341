obj-m += ili9341.o
ccflags-ili9341.o := -Wno-error=incompatible-pointer-types -Wno-incompatible-pointer-types

all:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules

clean:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean
