PWD := $(CURDIR)
MOD_NAME := ssm3582a
KERNEL=kernel
ARCH=arm
CROSS_COMPILE=/opt/gcc-linaro-11.3.1-2022.06-x86_64_arm-linux-gnueabihf/bin/arm-linux-gnueabihf-

obj-m := $(MOD_NAME).o

all:
	make -C $(KRN_SRC) M=$(PWD) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) modules

build:
	make -C $(KRN_SRC) M=$(PWD) modules

clean:
	make -C $(KRN_SRC) M=$(PWD) clean

install:
	make -C $(KRN_SRC) M=$(PWD) modules_install


