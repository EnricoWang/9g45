obj-m:= can_drv.o

KERNEL_DIR ?= /home/enrico/prj/Linux_Kernel/linux-kernel-2.6.32-ls3a2h
PWD := $(shell pwd)
module:
	make -C $(KERNEL_DIR) M=$(PWD) modules
clean:
	-rm *.o *.mod.c *.ko *.order *.symvers

