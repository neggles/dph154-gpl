# BSP Version: 3.2.4, RevisionID: ac30f57, Date: 20100223 17:42:05
#
# Makefile for the linux kernel.
#

obj-y						:= firecracker_core.o firecracker_dma.o firecracker_emac.o firecracker_reset.o
obj-$(CONFIG_MACH_PC72052_I10_REVB)		+= firecracker_cpe.o

obj-$(CONFIG_FIRECRACKER_DMA_TEST)		+= dma_test.o
