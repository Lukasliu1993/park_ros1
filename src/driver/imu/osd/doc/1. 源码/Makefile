KERNELDIR :=  /home/lyd/imx6ul/kernel/linux-imx-4.1.15
CURRENT_PATH := $(shell pwd)

obj-m := imu102n.o

build: kernel_modules

kernel_modules:
	$(MAKE) -C $(KERNELDIR) M=$(CURRENT_PATH) modules

clean:
	$(MAKE) -C $(KERNELDIR) M=$(CURRENT_PATH) clean
