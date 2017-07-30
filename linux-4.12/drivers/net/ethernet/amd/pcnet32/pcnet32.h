#ifndef AMD_PCNET32_H_
#define AMD_PCNET32_H_

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/string.h>
#include <linux/errno.h>
#include <linux/ioport.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/pci.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/ethtool.h>
#include <linux/mii.h>
#include <linux/crc32.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/if_ether.h>
#include <linux/skbuff.h>
#include <linux/spinlock.h>
#include <linux/moduleparam.h>
#include <linux/bitops.h>
#include <linux/io.h>
#include <linux/uaccess.h>

#include <asm/dma.h>
#include <asm/irq.h>

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#define DRV_NAME "pcnet32"
#define DRV_VERSION "1.35"
#define PFX DRV_NAME ": "

#define PCNET32_PORT_AUI      0x00
#define PCNET32_PORT_10BT     0x01
#define PCNET32_PORT_GPSI     0x02
#define PCNET32_PORT_MII      0x03

#define PCNET32_PORT_PORTSEL  0x03
#define PCNET32_PORT_ASEL     0x04
#define PCNET32_PORT_100      0x40
#define PCNET32_PORT_FD	      0x80

#define PCNET32_DMA_MASK 0xffffffff

#define PCNET32_WATCHDOG_TIMEOUT (jiffies + (2 * HZ))
#define PCNET32_BLINK_TIMEOUT	(jiffies + (HZ/4))

#define PCNET32_TEST_LEN	ARRAY_SIZE(pcnet32_gstrings_test)

#define PCNET32_NUM_REGS 136

#define MAX_UNITS 8		/* More are supported, limit only on options */

/*
 * Set the number of Tx and Rx buffers, using Log_2(# buffers).
 * Reasonable default values are 4 Tx buffers, and 16 Rx buffers.
 * That translates to 2 (4 == 2^^2) and 4 (16 == 2^^4).
 */
#ifndef PCNET32_LOG_TX_BUFFERS
#define PCNET32_LOG_TX_BUFFERS		4
#define PCNET32_LOG_RX_BUFFERS		5
#define PCNET32_LOG_MAX_TX_BUFFERS	9	/* 2^9 == 512 */
#define PCNET32_LOG_MAX_RX_BUFFERS	9
#endif

#define TX_RING_SIZE		(1 << (PCNET32_LOG_TX_BUFFERS))
#define TX_MAX_RING_SIZE	(1 << (PCNET32_LOG_MAX_TX_BUFFERS))

#define RX_RING_SIZE		(1 << (PCNET32_LOG_RX_BUFFERS))
#define RX_MAX_RING_SIZE	(1 << (PCNET32_LOG_MAX_RX_BUFFERS))

#define PKT_BUF_SKB		1544
/* actual buffer length after being aligned */
#define PKT_BUF_SIZE		(PKT_BUF_SKB - NET_IP_ALIGN)
/* chip wants twos complement of the (aligned) buffer length */
#define NEG_BUF_SIZE		(NET_IP_ALIGN - PKT_BUF_SKB)

/* Offsets from base I/O address. */
#define PCNET32_WIO_RDP		0x10
#define PCNET32_WIO_RAP		0x12
#define PCNET32_WIO_RESET	0x14
#define PCNET32_WIO_BDP		0x16

#define PCNET32_DWIO_RDP	0x10
#define PCNET32_DWIO_RAP	0x14
#define PCNET32_DWIO_RESET	0x18
#define PCNET32_DWIO_BDP	0x1C

#define PCNET32_TOTAL_SIZE	0x20

#define CSR0		0
#define CSR0_INIT	0x1
#define CSR0_START	0x2
#define CSR0_STOP	0x4
#define CSR0_TXPOLL	0x8
#define CSR0_INTEN	0x40
#define CSR0_IDON	0x0100
#define CSR0_NORMAL	(CSR0_START | CSR0_INTEN)
#define PCNET32_INIT_LOW	1
#define PCNET32_INIT_HIGH	2
#define CSR3		3
#define CSR4		4
#define CSR5		5
#define CSR5_SUSPEND	0x0001
#define CSR15		15
#define PCNET32_MC_FILTER	8

#define PCNET32_79C970A	0x2621

#define MAX_INTERRUPT_WORK (2)
#define RX_COPYBREAK (200)

#define PCNET32_REGS_PER_PHY	32
#define PCNET32_MAX_PHYS	32
#define PCNET32_MSG_DEFAULT (NETIF_MSG_DRV | NETIF_MSG_PROBE | NETIF_MSG_LINK)

/* The PCNET32 Rx and Tx ring descriptors. */
struct pcnet32_rx_head {
	__le32 base;
	__le16 buf_length; /* two`s complement of length */
	__le16 status;
	__le32 msg_length;
	__le32 reserved;
};

struct pcnet32_tx_head {
	__le32 base;
	__le16 length; /* two`s complement of length */
	__le16 status;
	__le32 misc;
	__le32 reserved;
};

/* The PCNET32 32-Bit initialization block, described in databook. */
struct pcnet32_init_block {
	__le16 mode;
	__le16 tlen_rlen;
	u8 phys_addr[6];
	__le16 reserved;
	__le32 filter[2];
	/* Receive and transmit ring base, along with extra bits. */
	__le32 rx_ring;
	__le32 tx_ring;
};

/* PCnet32 access functions */
struct pcnet32_access {
	u16 (*read_csr)(unsigned long, int);
	void (*write_csr)(unsigned long, int, u16);
	u16 (*read_bcr)(unsigned long, int);
	void (*write_bcr)(unsigned long, int, u16);
	u16 (*read_rap)(unsigned long);
	void (*write_rap)(unsigned long, u16);
	void (*reset)(unsigned long);
};

/*
 * The first field of pcnet32_private is read by the ethernet device
 * so the structure should be allocated using pci_alloc_consistent().
 */
struct pcnet32_private {
	struct pcnet32_init_block *init_block;
	/* The Tx and Rx ring entries must be aligned on 16-byte boundaries in 32bit mode. */
	struct pcnet32_rx_head *rx_ring;
	struct pcnet32_tx_head *tx_ring;
	dma_addr_t init_dma_addr;/* DMA address of beginning of the init block,
	 returned by pci_alloc_consistent */
	struct pci_dev *pci_dev;
	const char *name;
	/* The saved address of a sent-in-place packet/buffer, for skfree(). */
	struct sk_buff **tx_skbuff;
	struct sk_buff **rx_skbuff;
	dma_addr_t *tx_dma_addr;
	dma_addr_t *rx_dma_addr;
	const struct pcnet32_access *a;
	spinlock_t lock; /* Guard lock */
	unsigned int cur_rx, cur_tx; /* The next free ring entry */
	unsigned int rx_ring_size; /* current rx ring size */
	unsigned int tx_ring_size; /* current tx ring size */
	unsigned int rx_mod_mask; /* rx ring modular mask */
	unsigned int tx_mod_mask; /* tx ring modular mask */
	unsigned short rx_len_bits;
	unsigned short tx_len_bits;
	dma_addr_t rx_ring_dma_addr;
	dma_addr_t tx_ring_dma_addr;
	unsigned int dirty_rx, /* ring entries to be freed. */
	dirty_tx;

	struct net_device *dev;
	struct napi_struct napi;
	char tx_full;
	char phycount; /* number of phys found */
	int options;
	unsigned int shared_irq :1, /* shared irq possible */
	dxsuflo :1, /* disable transmit stop on uflo */
	mii :1, /* mii port available */
	autoneg :1, /* autoneg enabled */
	port_tp :1, /* port set to TP */
	fdx :1; /* full duplex enabled */
	struct net_device *next;
	struct mii_if_info mii_if;
	struct timer_list watchdog_timer;
	u32 msg_enable; /* debug message level */

	/* each bit indicates an available PHY */
	u32 phymask;
	unsigned short chip_version; /* which variant this is */

	/* saved registers during ethtool blink */
	u16 save_regs[4];
};

extern int pcnet32_debug;
int pcnet32_probe1(unsigned long, int, struct pci_dev *);

/*
 * pcnet32-probe-tool
 */
void pcnet32_wio_reset(unsigned long addr);
u16 pcnet32_wio_read_csr(unsigned long addr, int index);
int pcnet32_wio_check(unsigned long addr);
void pcnet32_dwio_reset(unsigned long addr);
u16 pcnet32_dwio_read_csr(unsigned long addr, int index);
int pcnet32_dwio_check(unsigned long addr);

/*
 * pcnet32-common
 */
int mdio_read(struct net_device *dev, int phy_id, int reg_num);
void mdio_write(struct net_device *dev, int phy_id, int reg_num, int val);
void pcnet32_purge_rx_ring(struct net_device *dev);
void pcnet32_restart(struct net_device *dev, unsigned int csr0_bits);
void pcnet32_free_ring(struct net_device *dev);
void pcnet32_purge_tx_ring(struct net_device *dev);
int pcnet32_init_ring(struct net_device *);
void pcnet32_check_media(struct net_device *dev, int verbose);
void pcnet32_clr_suspend(struct pcnet32_private *lp, ulong ioaddr);
int pcnet32_suspend(struct net_device *dev, unsigned long *flags, int can_sleep);
int pcnet32_open(struct net_device *);
int pcnet32_close(struct net_device *);

#endif /* AMD_PCNET32_H_ */
