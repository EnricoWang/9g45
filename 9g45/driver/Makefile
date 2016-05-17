obj-m += sam9g45_ebi_dev.o
obj-m += sam9g45_ebi_drv.o
KERNEL_DIR = /home/enrico/topsys/src/kernel/linux-3.6.9
PWD = $(shell pwd)
module:
	make -C $(KERNEL_DIR) M=$(PWD) modules
clean:
	rm -rf *.o *.mod.c *.mod.o *.ko *.order *.symvers

