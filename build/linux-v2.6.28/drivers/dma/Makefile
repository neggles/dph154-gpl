# BSP Version: 3.2.4, RevisionID: ac30f57, Date: 20100223 17:42:05
#
obj-$(CONFIG_DMA_ENGINE) += dmaengine.o
obj-$(CONFIG_NET_DMA) += iovlock.o
obj-$(CONFIG_DMATEST) += dmatest.o
obj-$(CONFIG_INTEL_IOATDMA) += ioatdma.o
ioatdma-objs := ioat.o ioat_dma.o ioat_dca.o
obj-$(CONFIG_INTEL_IOP_ADMA) += iop-adma.o
obj-$(CONFIG_FSL_DMA) += fsldma.o
obj-$(CONFIG_MV_XOR) += mv_xor.o
obj-$(CONFIG_DW_DMAC) += dw_dmac.o

# PC302 DMA driver
obj-$(CONFIG_PC302_DMA) += dmac.o
dmac-y :=  pc302_dmac.o

# PC302 DMA driver tester
obj-$(CONFIG_PC302_DMA_TEST) += dmacdebug.o
dmacdebug-y := pc302_dmac_test.o

