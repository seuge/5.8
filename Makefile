ifneq  ($(KERNELRELEASE),)
obj-m:=mv190.o
mv190-y += chip.o global1.o global2.o
else
KDIR := /lib/modules/$(shell uname -r)/build
PWD:=$(shell pwd)
all:
	make -C $(KDIR) M=$(PWD) modules
clean:
	rm -f *.ko *.o *.symvers *.cmd *.cmd.o
endif
