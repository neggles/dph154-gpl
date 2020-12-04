/*
 * spi_pc302.c - PC302 SPI controller driver.
 *
 * Based on linux/device/spi/spi_txx9.c
 * Copyright (C) 2009 ip.access Ltd
 * Copyright (C) 2000-2001 Toshiba Corporation
 *
 * 2003-2005 (c) MontaVista Software, Inc. This file is licensed under the
 * terms of the GNU General Public License version 2. This program is
 * licensed "as is" without any warranty of any kind, whether express
 * or implied.
 *
 * Support for TX4938 in 2.6 - Manish Lachwani (mlachwani@mvista.com)
 *
 * Convert to generic SPI framework - Atsushi Nemoto (anemo@mba.ocn.ne.jp)
 */
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>
#include <linux/spi/spi.h>
#include <linux/err.h>
#include <linux/io.h>

#include <mach/spi-gpio.h>

#ifdef CONFIG_ARCH_PC302

#include <mach/gpio.h>
#include <mach/pc302/axi2cfg.h>
#include <mach/pc302/ssi.h>
#include <mach/pc302/pc302.h>

#else

/* Test version for PC202 with printk to indicate HW access */

#define SSI_CTRL_REG_0_REG_OFFSET                   0x00
#define SSI_CTRL_REG_1_REG_OFFSET                   0x04
#define SSI_ENABLE_REG_REG_OFFSET                   0x08
#define SSI_MW_CTRL_REG_OFFSET                      0x0C
#define SSI_SLAVE_ENABLE_REG_OFFSET                 0x10
#define SSI_BAUD_RATE_SEL_REG_OFFSET                0x14
#define SSI_TX_FIFO_THRESHOLD_REG_OFFSET            0x18
#define SSI_RX_FIFO_THRESHOLD_REG_OFFSET            0x1C
#define SSI_IMR_REG_OFFSET                          0x2C
#define SSI_STATUS_REG_OFFSET                       0x28
#define SSI_ISR_REG_OFFSET                          0x30
#define SSI_DATA_REG_OFFSET                         0x60                        

#define PC302_TIMER_FREQ        200000000   /* 200 MHz */

#define AXI2CFG_DECODE_MUX_3_IDX           11
#define AXI2CFG_DECODE_MUX_2_IDX           10
#define AXI2CFG_DECODE_MUX_1_IDX            9
#define AXI2CFG_DECODE_MUX_0_IDX            8

#define AXI2CFG_DECODE_MUX_3                1 << AXI2CFG_DECODE_MUX_3_IDX
#define AXI2CFG_DECODE_MUX_2                1 << AXI2CFG_DECODE_MUX_2_IDX
#define AXI2CFG_DECODE_MUX_1                1 << AXI2CFG_DECODE_MUX_1_IDX
#define AXI2CFG_DECODE_MUX_0                1 << AXI2CFG_DECODE_MUX_0_IDX

#define AXI2CFG_SYS_CONFIG_SSI_EBI_SEL_MASK   \
      (AXI2CFG_DECODE_MUX_0 | AXI2CFG_DECODE_MUX_1 | AXI2CFG_DECODE_MUX_2 | AXI2CFG_DECODE_MUX_3)

#endif

#define PC302_SPI_FIFO_SIZE                   (8)

/* Baud rate divisor is 16 bits, but must be even */
#define PC302_SPI_MIN_CLK_DIVIDER             (2)
#define PC302_SPI_MAX_CLK_DIVIDER             (65534)

/* SSI_CTRL_REG_0_REG_OFFSET bites */
#define PC302_SPI_MW_CTRL_WORD_8_BIT          (7 << 12)
#define PC302_SPI_MW_CTRL_WORD_7_BIT          (6 << 12)
#define PC302_SPI_NORMAL_MODE                 (0 << 11)
#define PC302_SPI_LOOPBACK_MODE               (1 << 11)
#define PC302_SPI_TMOD_TX_RX                  (0 <<  8)
#define PC302_SPI_TMOD_TX                     (1 <<  8)
#define PC302_SPI_TMOD_RX                     (2 <<  8)
#define PC302_SPI_TMOD_EEPROM_RX              (3 <<  8)
#define PC302_SPI_SCPOL                       (1 <<  7)
#define PC302_SPI_SCPH                        (1 <<  6)
#define PC302_SPI_MOTO_SPI_FORMAT             (0x0 << 4)
#define PC302_SPI_TI_SSP_FORMAT               (0x1 << 4)
#define PC302_SPI_NI_MW_FORMAT                (0x2 << 4)
#define PC302_SPI_DATA_FRM_16_BIT             (15)
#define PC302_SPI_DATA_FRM_8_BIT              (7)

/* SSI_MW_CTRL_REG_OFFSET bits */
#define PC302_SPI_MICROWIRE_HANDSHAKE         (1 << 2)
#define PC302_SPI_MICROWIRE_READ              (0 << 1)
#define PC302_SPI_MICROWIRE_WRITE             (1 << 1)
#define PC302_SPI_MICROWIRE_DISCONTINUOUS     (0 << 0)
#define PC302_SPI_MICROWIRE_CONTINUOUS        (1 << 0)

/* SSI_ENABLE_REG_REG_OFFSET bits */
#define PC302_SPI_ENABLE                      (1)
#define PC302_SPI_DISABLE                     (0)

/* SSI_SLAVE_ENABLE_REG_OFFSET bits */
#define PC302_SPI_SLAVES_DISABLE              (0)

/* SSI_STATUS_REG_OFFSET bits */
#define PC302_SPI_STATUS_TX_ERROR             (1 << 5)
#define PC302_SPI_STATUS_RX_FIFO_FULL         (1 << 4)
#define PC302_SPI_STATUS_RX_FIFO_NOT_EMPTY    (1 << 3)
#define PC302_SPI_STATUS_TX_FIFO_EMPTY        (1 << 2)
#define PC302_SPI_STATUS_TX_FIFO_NOT_FULL     (1 << 1)
#define PC302_SPI_STATUS_BUSY                 (1 << 0)

/* SSI_IMR_REG_RESET bits */
#define PC302_SPI_MASK_ALL_INTS               (0)
#define PC302_SPI_TX_FIFO_EMPTY_INT           (1 << 0)
#define PC302_SPI_RX_FIFO_FULL_INT            (1 << 4)


struct pc302spi {
	struct workqueue_struct*    workqueue;
	struct work_struct          work;
	spinlock_t                  lock; /* protect 'queue' */
	struct list_head            queue;
	wait_queue_head_t           waitq;
	void __iomem*               membase_p;
	int                         baseclk;
	u32                         max_speed_hz;
	u32                         min_speed_hz;
	struct pc302_spi_info*      info;
	const u8*                   activeTxBuf8_p;
	const u16*                  activeTxBuf16_p;
	s32                         activeTxLen;
	u8*                         activeRxBuf8_p;
	u16*                        activeRxBuf16_p;
	s32                         activeRxLen;
};

static inline u16 pc302spi_rd(struct pc302spi *c, int reg)
{
#ifdef CONFIG_ARCH_PC302
	return __raw_readw((u16*)((u8*)c->membase_p + reg));
#else
	printk("Read from SPI reg %p+%x (Always 0)\n", c->membase_p, reg);
	return 0;
#endif
}

static inline void pc302spi_wr(struct pc302spi *c, u16 val, int reg)
{
#ifdef CONFIG_ARCH_PC302
	__raw_writew(val, (u16*)((u8*)c->membase_p + reg));
#else
	printk("Write 0x%08X to SPI reg %p+%x\n", val, c->membase_p, reg);
#endif
}

static void pc302spi_wr_n(struct pc302spi *c, u16 *val_p, int reg, int count)
{
	u16*  addr = (u16*)((u8*)c->membase_p + reg);

	while (count > 0)
	{
		count--;
#ifdef CONFIG_ARCH_PC302
		__raw_writew(*val_p++, addr);
#else
		printk("Write(%d) 0x%08X to SPI reg %p\n", count, *val_p++, addr);
#endif
	}
}

static int pc302spi_setup(struct spi_device *spi)
{
	struct pc302spi *c = spi_master_get_devdata(spi->master);
	u8 bits_per_word;
	
	if (spi->mode & ~(SPI_CPOL|SPI_CPHA|SPI_3WIRE|SPI_CS_HIGH))
		return -EINVAL;
	
	if (!spi->max_speed_hz
			|| spi->max_speed_hz > c->max_speed_hz
			|| spi->max_speed_hz < c->min_speed_hz)
		return -EINVAL;
	
	bits_per_word = spi->bits_per_word ? : 8;
	if ((bits_per_word != 8) && (bits_per_word != 16))
		return -EINVAL;
	
	return 0;
}

#ifdef CONFIG_ARCH_PC302
static irqreturn_t pc302spi_interrupt(int irq, void *dev_id)
{
	struct pc302spi *c = dev_id;
	u16              isrStatus;
	
	isrStatus = pc302spi_rd(c, SSI_ISR_REG_OFFSET);
	
	while (isrStatus & PC302_SPI_RX_FIFO_FULL_INT)
	{
		/* There's data available in the RX FIFO */
		u16  data = pc302spi_rd(c, SSI_DATA_REG_OFFSET);

		/* The activeRxLen is either the number of bytes or words left
		 * to read.  Only one of the pointers (8 vs 16-bit) will be
		 * active at any time, and it's possible that neither is active
		 */
		if (c->activeRxLen > 0)
		{
			c->activeRxLen--;
			
			if (c->activeRxBuf8_p)
			{
				*(c->activeRxBuf8_p)++ = data;
			}
			else if (c->activeRxBuf16_p)
			{
				*(c->activeRxBuf16_p)++ = data;
			}
		}
		
		if (c->activeRxLen <= 0)
		{
			/* Disable further Rx interrupts */
			u16  mask;
				
			mask = pc302spi_rd(c, SSI_IMR_REG_OFFSET);
			mask &= ~PC302_SPI_RX_FIFO_FULL_INT;
			pc302spi_wr(c, mask, SSI_IMR_REG_OFFSET);
			
			/* Inform the upper layers when we've finished */
			if (c->activeTxLen <= 0)
			{
				wake_up(&c->waitq);
			}
		}
	
		isrStatus = pc302spi_rd(c, SSI_ISR_REG_OFFSET);
	}
	
	while (isrStatus & PC302_SPI_TX_FIFO_EMPTY_INT)
	{
		/* Transmit FIFO can accept more data */
		
		/* The activeTxLen is either the number of bytes or words left
		 * to write.  Only one of the pointers (8 vs 16-bit) will be
		 * active at any time, and it's possible that neither is active
		 */
		if (c->activeTxLen > 0)
		{
			u16  data = 0xFFFF;
			
			c->activeTxLen--;
			
			if (c->activeTxBuf8_p)
			{
				data = *(c->activeTxBuf8_p)++;
			}
			else if (c->activeTxBuf16_p)
			{
				data = *(c->activeTxBuf16_p)++;
			}
			
			pc302spi_wr(c, data, SSI_DATA_REG_OFFSET);
		}
		
		if (c->activeTxLen <= 0)
		{
			/* Disable further Tx interrupts */
			u16  mask;
			
			mask = pc302spi_rd(c, SSI_IMR_REG_OFFSET);
			mask &= ~PC302_SPI_TX_FIFO_EMPTY_INT;
			pc302spi_wr(c, mask, SSI_IMR_REG_OFFSET);
			
			if (c->activeRxLen <= 0)
			{
				wake_up(&c->waitq);
			}
		}
	
		isrStatus = pc302spi_rd(c, SSI_ISR_REG_OFFSET);
	}
	
	return IRQ_HANDLED;
}
#endif

static void pc302spi_work_one(struct pc302spi *c, struct spi_message *m)
{
	struct spi_device    *spi = m->spi;
	struct spi_transfer  *t;
	u16                   ctrlr0;
	u16                   ctrlr1;
	u16                   mwcr;
	u32                   prev_speed_hz = 0;
	u16                   status;
	unsigned long         flags;
	
	/* Wait until any previous transfer has completed. */
	do
	{
		status = pc302spi_rd(c, SSI_STATUS_REG_OFFSET);
		status &= PC302_SPI_STATUS_BUSY;
		if (status)
		{
			cpu_relax();
		}
	}
	while (status);
	
	pc302spi_wr(c, PC302_SPI_DISABLE, SSI_ENABLE_REG_REG_OFFSET);
	
	/* Enable the appropriate chip select */
	pc302spi_wr(c, 1 << spi->chip_select, SSI_SLAVE_ENABLE_REG_OFFSET);
		
	/* Perform each transfer in the message */
	list_for_each_entry (t, &m->transfers, transfer_list) {
	
		u8    bits_per_word = t->bits_per_word ? : spi->bits_per_word;
		u32   speed_hz      = t->speed_hz      ? : spi->max_speed_hz;
		u16   intMask;
		int   i;
		int   n;
		int   ret;
		u32   timeout;
		u16   data[PC302_SPI_FIFO_SIZE];
		int   shiftCmd = 0;   /* We might have to fiddle with the command we send */

		bits_per_word = bits_per_word ? : 8;
		
		/* Ensure interrupts are disabled to begin with */
		pc302spi_wr(c, 0, SSI_IMR_REG_OFFSET);
		
		/* Set baud rate */    
		if (prev_speed_hz != speed_hz)
		{
			u32 clkDiv;
			
			/* Calculate the divisor.  This must be even and we must always
			 * round the divisor up to avoid exceeding the maximum requested
			 */
			clkDiv = (c->baseclk + speed_hz - 1) / speed_hz;
			clkDiv += clkDiv & 1;
			
			if (clkDiv < PC302_SPI_MIN_CLK_DIVIDER)
			{
				clkDiv = PC302_SPI_MIN_CLK_DIVIDER;
			}
			else if (clkDiv > PC302_SPI_MAX_CLK_DIVIDER)
			{
				clkDiv = PC302_SPI_MAX_CLK_DIVIDER;
			}
			
			pc302spi_wr(c, clkDiv, SSI_BAUD_RATE_SEL_REG_OFFSET);
			
			prev_speed_hz = speed_hz;
		}

		/* Configure the transfer mode */
		ctrlr0 = PC302_SPI_NORMAL_MODE | PC302_SPI_TMOD_TX_RX;
		ctrlr1 = 0;
		
		if (spi->mode & SPI_CPOL)
		{
			ctrlr0 |= PC302_SPI_SCPOL;
		}
		if (spi->mode & SPI_CPHA)
		{
			ctrlr0 |= PC302_SPI_SCPH;
		}

		/* Get local copies of the lengths and pointers to the buffers for use
		 * by the interrupt service routine.
		 */
		c->activeTxBuf8_p  = NULL;
		c->activeRxBuf8_p  = NULL;
		c->activeTxBuf16_p = NULL;
		c->activeRxBuf16_p = NULL;

		/* We can perform 8-bit or 16-bit transfers.  The t->len field
		 * is the number of bytes in either case.  If not specified then
		 * default to 8-bit words.
		 */
		if (bits_per_word == 16)
		{
			c->activeTxLen   = t->len / 2;
			c->activeRxLen   = t->len / 2;
		
			c->activeTxBuf16_p = t->tx_buf;
			c->activeRxBuf16_p = t->rx_buf;
			
			ctrlr0 |= PC302_SPI_DATA_FRM_16_BIT;
		}
		else
		{
			c->activeTxLen   = t->len;
			c->activeRxLen   = t->len;
		
			c->activeTxBuf8_p  = t->tx_buf;
			c->activeRxBuf8_p  = t->rx_buf;
			
			ctrlr0 |= PC302_SPI_DATA_FRM_8_BIT;
		}
		
		mwcr = 0;

		/* There have been problems running the SSI block in Mode 0, the chip
		 * select is deselected between words so, as a workaround, Microwire
		 * mode is being used instead for now.  This is not documented yet and
		 * it also has issues, described below.
		 */
		if (spi->mode & SPI_3WIRE)
		{
			/* Send an 8-bit control word.  Then send or receive 16-bits.
			 * The length passed in is the combined Tx and Rx, so adjust
			 * the Rx buffer length and pointer to allow a single buffer
			 * for both Tx and Rx.
			 */
			
			/* The SSI block inserts an extra clock pulse between the cmd and data
			 * on a read, but not on a write.  That extra clock corresponds to the
			 * first bit of the reply from the MAX6662, so we lose that.  To avoid
			 * missing data the command has to be reduced from 8 to 7 bits.  Then the
			 * MAX6662 sees the extra clock as the final command bit.  Now, the final
			 * command bit is always 1, so it's not needed to decide what the command
			 * is.  However, reducing the command to 7-bits causes it to be sent as
			 * a 0.  There's no reason to believe that this will work, but it appears
			 * to.  If we are sending a 7-bit command we have to right justify the
			 * command by shifting it right one bit.
			 */
			if (c->activeRxBuf8_p)
			{
				/* 8-bit data transfers, but always a single 8-bit command. */
				c->activeTxLen = 1;
				c->activeRxLen--;
				c->activeRxBuf8_p++;
				mwcr = PC302_SPI_MICROWIRE_READ | PC302_SPI_MICROWIRE_CONTINUOUS | PC302_SPI_MICROWIRE_HANDSHAKE;
				ctrlr0 |= PC302_SPI_NI_MW_FORMAT | PC302_SPI_MW_CTRL_WORD_7_BIT;
				shiftCmd = 1;
			}
			else if (c->activeRxBuf16_p)
			{
				/* 16-bit data transfers, but always a single 8-bit command. */
				c->activeTxLen = 1;
				c->activeRxLen--;
				c->activeRxBuf16_p++;
				mwcr = PC302_SPI_MICROWIRE_READ | PC302_SPI_MICROWIRE_CONTINUOUS | PC302_SPI_MICROWIRE_HANDSHAKE;
				ctrlr0 |= PC302_SPI_NI_MW_FORMAT | PC302_SPI_MW_CTRL_WORD_7_BIT;
				shiftCmd = 1;
			}
			else
			{
				/* Don't read any data, but we have to wait till the transmit is complete,
				 * so set the activeRxLen to -1.  That will prevent the wait condition
				 * later on from passing immediately (since it's not 0) and force us to
				 * timeout.  If we didn't wait for the timeout in this case, then the
				 * SSI block would be disabled before the transfer was completed.
				 */
				c->activeRxLen=-1;
				mwcr = PC302_SPI_MICROWIRE_WRITE | PC302_SPI_MICROWIRE_CONTINUOUS;
				ctrlr0 |= PC302_SPI_NI_MW_FORMAT | PC302_SPI_MW_CTRL_WORD_8_BIT;
			}
		}
		else
		{
			ctrlr0 |= PC302_SPI_MOTO_SPI_FORMAT;
		}
		
		/* Configure the transfer */
		pc302spi_wr(c, ctrlr0, SSI_CTRL_REG_0_REG_OFFSET);
		pc302spi_wr(c, ctrlr1, SSI_CTRL_REG_1_REG_OFFSET);
		pc302spi_wr(c, mwcr,   SSI_MW_CTRL_REG_OFFSET);
		
		/* Interrupt when the TX FIFO has at least one space. */
		pc302spi_wr(c, PC302_SPI_FIFO_SIZE-1, SSI_TX_FIFO_THRESHOLD_REG_OFFSET);
		
		/* Interrupt when the RX FIFO has at least one word. PC302 Programmers Guide
		 * section G.5.3 says this triggers an interrupt when the number of FIFO entries
		 * is >= to the value written here + 1
		 */
		pc302spi_wr(c, 0, SSI_RX_FIFO_THRESHOLD_REG_OFFSET);
				
		/* RX interrupts will fill the RX buffer */
		intMask = PC302_SPI_RX_FIFO_FULL_INT;
		
		/* How many words should we write to the FIFO to kick things off */
		n = c->activeTxLen;

		if (n > PC302_SPI_FIFO_SIZE)
		{
			n = PC302_SPI_FIFO_SIZE;

			/* We'll still have data to send.  We must use Tx interrupts to
			 * keep the FIFO fed.
			 */
			intMask |= PC302_SPI_TX_FIFO_EMPTY_INT;
		}

		c->activeTxLen -= n;

		/* To avoid stalling during the attempt to write to the FIFO.  We
		 * will get a local copy of the the data and perform the initial
		 * Tx writes with interrupts off, that should ensure the FIFO doesn't
		 * empty and turn off the chip select in the middle of the transfer.
		 */
		spin_lock_irqsave(&c->lock, flags);

		/* If we have enough to send, then fill the Tx FIFO */
		for (i=0; i<n; i++)
		{
			if (c->activeTxBuf8_p)
			{
				if (shiftCmd)
				{
					shiftCmd = 0;
					data[i] = (*(c->activeTxBuf8_p)++) >> 1;
				}
				else
				{
					data[i] = *(c->activeTxBuf8_p)++;
				}
			}
			else if (c->activeTxBuf16_p)
			{
				if (shiftCmd)
				{
					shiftCmd = 0;
					data[i] = (*(c->activeTxBuf16_p)++) >> 1;
				}
				else
				{
					data[i] = *(c->activeTxBuf16_p)++;
				}
			}
			else
			{
				data[i] = 0xFFFF;
			}
		}
		
		pc302spi_wr(c, PC302_SPI_ENABLE, SSI_ENABLE_REG_REG_OFFSET);
	
#ifdef CONFIG_ARCH_PC302
		if (spi->dev.platform_data)
		{
			gpio_set_value((unsigned)(spi->dev.platform_data), (spi->mode & SPI_CS_HIGH) ? 1 : 0);
		}
#endif

		pc302spi_wr_n(c, data, SSI_DATA_REG_OFFSET, n);

		pc302spi_wr(c, intMask, SSI_IMR_REG_OFFSET);
		
		spin_unlock_irqrestore(&c->lock, flags);
		
#ifdef CONFIG_ARCH_PC302
		/* wait until all tx data has been sent and all rx data has been read */
		timeout = 2 + (2 * t->len * 8 * HZ) / speed_hz;
		ret = wait_event_timeout(c->waitq, (c->activeTxLen == 0) && (c->activeRxLen == 0), timeout);
		
		if ((c->activeTxLen > 0) || (c->activeRxLen > 0))
		{
			printk("SPI timeout after %d jiffies tx=%d rx=%d\n", timeout, c->activeTxLen, c->activeRxLen);
		}

		if (spi->dev.platform_data)
		{
			gpio_set_value((unsigned)(spi->dev.platform_data), (spi->mode & SPI_CS_HIGH) ? 0 : 1);
		}
#else
		/* Pretend to write the remaining data */
		while (c->activeTxLen > 0)
		{
			u8 data = 0;
			
			if (c->activeTxBuf8_p)
			{
				data = *(c->activeTxBuf8_p)++;
			}
			else if (c->activeTxBuf16_p)
			{
				data = *(c->activeTxBuf16_p)++;
			}
			
			pc302spi_wr(c, data, SSI_DATA_REG_OFFSET);
			
			c->activeTxLen--;
		}
		/* Pretend to read the data */
		while (c->activeRxLen > 0)
		{
			u16  data = pc302spi_rd(c, SSI_DATA_REG_OFFSET);
			
			data = 0x5555;
			
			if (c->activeRxBuf8_p)
			{
				*(c->activeRxBuf8_p)++ = data;
			}
			else if (c->activeRxBuf16_p)
			{
				*(c->activeRxBuf16_p)++ = data;
			}
			
			c->activeRxLen--;
		}
#endif
		
		/* Ensure interrupts are disabled from the SPI block */
		pc302spi_wr(c, PC302_SPI_MASK_ALL_INTS, SSI_IMR_REG_OFFSET);
		
		pc302spi_wr(c, PC302_SPI_DISABLE, SSI_ENABLE_REG_REG_OFFSET);
	
		m->actual_length += t->len;
		
		if (t->delay_usecs)
			udelay(t->delay_usecs);
	
		if (t->transfer_list.next == &m->transfers)
			break;
	}
	
	m->status = 0;
	m->complete(m->context);
	
	pc302spi_wr(c, PC302_SPI_DISABLE, SSI_ENABLE_REG_REG_OFFSET);
	pc302spi_wr(c, PC302_SPI_SLAVES_DISABLE, SSI_SLAVE_ENABLE_REG_OFFSET);
}

static void pc302spi_work(struct work_struct *work)
{
	struct pc302spi *c = container_of(work, struct pc302spi, work);
	unsigned long flags;

	spin_lock_irqsave(&c->lock, flags);
	while (!list_empty(&c->queue)) {
		struct spi_message *m;

		m = container_of(c->queue.next, struct spi_message, queue);
		list_del_init(&m->queue);
		spin_unlock_irqrestore(&c->lock, flags);

		pc302spi_work_one(c, m);

		spin_lock_irqsave(&c->lock, flags);
	}
	spin_unlock_irqrestore(&c->lock, flags);
}

static int pc302spi_transfer(struct spi_device *spi, struct spi_message *m)
{
	struct spi_master    *master = spi->master;
	struct pc302spi      *c = spi_master_get_devdata(master);
	struct spi_transfer  *t;
	unsigned long         flags;
	u32                   enabled = 0;
	
	m->actual_length = 0;
	
	switch (spi->chip_select)
	{
	case 0:
		enabled = c->info->cs0_active;
		break;
	case 1:
		enabled = c->info->cs1_active;
		break;
	case 2:
		enabled = c->info->cs2_active;
		break;
	case 3:
		enabled = c->info->cs3_active;
		break;
	default:
		enabled = 0;
		break;
	}
	
	if ( ! enabled)
	{
		return -EINVAL;
	}
	
	/* check the parameters for each transfer in the message */
	list_for_each_entry (t, &m->transfers, transfer_list) {
		u32 speed_hz     = t->speed_hz      ? : spi->max_speed_hz;
		u8 bits_per_word = t->bits_per_word ? : spi->bits_per_word;
	
		bits_per_word = bits_per_word ? : 8;
		if (!t->tx_buf && !t->rx_buf && t->len)
			return -EINVAL;
		if ((bits_per_word != 8) && (bits_per_word != 16))
			return -EINVAL;
		if (speed_hz < c->min_speed_hz)
			return -EINVAL;
//TODO: We could always just run slower, it's unlikely to be a problem.
//        if (speed_hz > c->max_speed_hz)
//			return -EINVAL;
	}

	spin_lock_irqsave(&c->lock, flags);
	list_add_tail(&m->queue, &c->queue);
	queue_work(c->workqueue, &c->work);
	spin_unlock_irqrestore(&c->lock, flags);

	return 0;
}

static int __init pc302spi_probe(struct platform_device *dev)
{
	struct spi_master *master;
	struct pc302spi   *c;
	struct resource   *res;
	u32                system_config;
	int                ret = -ENODEV;
	int                irq;
	int                i;
	
	master = spi_alloc_master(&dev->dev, sizeof(*c));
	if (!master)
	{
		return ret;
	}
	
	c = spi_master_get_devdata(master);
	platform_set_drvdata(dev, master);
	
	/* copy in the platform data */
	c->info = dev->dev.platform_data;
	
	INIT_WORK(&c->work, pc302spi_work);
	spin_lock_init(&c->lock);
	INIT_LIST_HEAD(&c->queue);
	init_waitqueue_head(&c->waitq);
	
	c->baseclk      = PC302_TIMER_FREQ;
	c->min_speed_hz = (PC302_TIMER_FREQ + PC302_SPI_MAX_CLK_DIVIDER - 1)/ PC302_SPI_MAX_CLK_DIVIDER;
	c->max_speed_hz = PC302_TIMER_FREQ / PC302_SPI_MIN_CLK_DIVIDER;
	
	res = platform_get_resource(dev, IORESOURCE_MEM, 0);
	if (!res)
	{
		goto exit_busy;
	}
	
#ifdef CONFIG_ARCH_PC302
	if (!devm_request_mem_region(&dev->dev, res->start, res->end - res->start + 1, "spi_pc302"))
	{
		goto exit_busy;
	}
	
	c->membase_p = devm_ioremap(&dev->dev, res->start, res->end - res->start + 1);
	if (!c->membase_p)
	{
		goto exit_busy;
	}
#else
	c->membase_p = (void*)res->start;
#endif

	/* Disable the SPI and ensure all chip selects are inactive */
	pc302spi_wr(c, PC302_SPI_DISABLE, SSI_ENABLE_REG_REG_OFFSET);
	pc302spi_wr(c, PC302_SPI_SLAVES_DISABLE, SSI_SLAVE_ENABLE_REG_OFFSET);
	
	/* Give the SPI bus control of the required chip select pins */
	system_config = 0;
	if ( ! c->info->cs0_active )
	{
		system_config |= AXI2CFG_DECODE_MUX_0;
	}
	if ( ! c->info->cs1_active )
	{
		system_config |= AXI2CFG_DECODE_MUX_1;
	}
	if ( ! c->info->cs2_active )
	{
		system_config |= AXI2CFG_DECODE_MUX_2;
	}
	if ( ! c->info->cs3_active )
	{
		system_config |= AXI2CFG_DECODE_MUX_3;
	}
#ifdef CONFIG_ARCH_PC302
	syscfg_update( AXI2CFG_SYS_CONFIG_SSI_EBI_SEL_MASK, system_config );
#else
	printk("Updating SYS_CONFIG mask=0x%08X, value=0x%08X\n", AXI2CFG_SYS_CONFIG_SSI_EBI_SEL_MASK, system_config);
#endif

	irq = platform_get_irq(dev, 0);
	if (irq < 0)
	{
		goto exit_busy;
	}
#ifdef CONFIG_ARCH_PC302
	ret = devm_request_irq(&dev->dev, irq, pc302spi_interrupt, 0, "spi_pc302", c);
	if (ret)
	{
		goto exit;
	}
#else
	printk("Register irq %d for spi_pc302 with pc302spi_interrupt\n", irq);
#endif
	
	c->workqueue = create_singlethread_workqueue(master->dev.parent->bus_id);
	if (!c->workqueue)
	{
		goto exit_busy;
	}
	
	master->bus_num        = dev->id;
	master->setup          = pc302spi_setup;
	master->transfer       = pc302spi_transfer;
	master->num_chipselect = 4;
	
	ret = spi_register_master(master);
	if (ret)
	{
		goto exit;
	}
	
	/* register the chips to go with the board */
	for (i = 0; i < c->info->board_size; i++) {
		dev_info(&dev->dev, "registering %p: %s\n",
		         &c->info->board_info[i],
		          c->info->board_info[i].modalias);
	
		c->info->board_info[i].controller_data = c;
		spi_new_device(master, c->info->board_info + i);
	}
	
	return 0;
	
exit_busy:
	ret = -EBUSY;
	
exit:
	if (c->workqueue)
	{
		destroy_workqueue(c->workqueue);
	}
	
	platform_set_drvdata(dev, NULL);
	spi_master_put(master);
	
	return ret;
}

static int __exit pc302spi_remove(struct platform_device *dev)
{
	struct spi_master *master = spi_master_get(platform_get_drvdata(dev));
	struct pc302spi   *c      = spi_master_get_devdata(master);

	spi_unregister_master(master);
	platform_set_drvdata(dev, NULL);
	destroy_workqueue(c->workqueue);
	spi_master_put(master);
	return 0;
}

/* work with hotplug and coldplug */
//MODULE_ALIAS("platform:spi_pc302");

static struct platform_driver pc302spi_driver = {
	.remove = __exit_p(pc302spi_remove),
	.driver = {
		.name = "pc302-spi",
		.owner = THIS_MODULE,
	},
};

static int __init pc302spi_init(void)
{
	return platform_driver_probe(&pc302spi_driver, pc302spi_probe);
}
subsys_initcall(pc302spi_init);

static void __exit pc302spi_exit(void)
{
	platform_driver_unregister(&pc302spi_driver);
}
module_exit(pc302spi_exit);

MODULE_DESCRIPTION("PC302 SPI Driver");
MODULE_LICENSE("GPL");
