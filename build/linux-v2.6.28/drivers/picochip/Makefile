# BSP Version: 3.2.4, RevisionID: ac30f57, Date: 20100223 17:42:05
#
# Makefile for pico driver
# HDP102 CPLD driver
obj-$(CONFIG_PICOHDP_CPLD) 	+= cpld_hdp102.o

# CPE20x FPGA driver
obj-$(CONFIG_PICOCPE20X_FPGA) 	+= fpga_cpe20x.o
fpga_cpe20x-y := fpga_cpe20x_main.o

obj-$(CONFIG_PICOIF)	      	+= picoif.o
picoif-y := picoif_main.o resource.o \
	    debug.o soft_reset.o gpr_interrupt.o dma.o \
	    dma_fifo.o hwif.o hwif2.o utilities_internal.o

picoifktest-y := kerneltest.o

picoif-$(CONFIG_PICOIF_PC203)	+= pc203.o procif.o
picoif-$(CONFIG_PICOIF_PC202)	+= pc202.o procif.o
picoif-$(CONFIG_PICOIF_PC302)	+= pc302.o axi2cfg.o
