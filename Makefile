#!makefile

KERNEL_DIR ?= ../linux-hdmi-dongle

.PHONY : all clean
all:
	@$(MAKE) -C $(KERNEL_DIR) M=$$PWD modules

clean:
	@$(MAKE) -C $(KERNEL_DIR) M=$$PWD clean
