/* Linux PRO/1000 Ethernet Driver main header file */

#ifndef _E1000_H_
#define _E1000_H_

#include <linux/stddef.h>
#include <linux/module.h>
#include <linux/types.h>
#include <asm/byteorder.h>
#include <linux/mm.h>
#include <linux/errno.h>
#include <linux/ioport.h>
#include <linux/pci.h>
#include <linux/kernel.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>
#include <linux/delay.h>
#include <linux/timer.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/interrupt.h>
#include <linux/string.h>
#include <linux/pagemap.h>
#include <linux/dma-mapping.h>
#include <linux/bitops.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <linux/capability.h>
#include <linux/in.h>
#include <linux/ip.h>
#include <linux/ipv6.h>
#include <linux/tcp.h>
#include <linux/udp.h>
#include <net/pkt_sched.h>
#include <linux/list.h>
#include <linux/reboot.h>
#include <net/checksum.h>
#include <linux/mii.h>
#include <linux/ethtool.h>
#include <linux/if_vlan.h>

#define BAR_0		0
#define BAR_1		1
#define BAR_5		5

#define INTEL_E1000_ETHERNET_DEVICE(device_id) {\
	PCI_DEVICE(PCI_VENDOR_ID_INTEL, device_id)}

struct e1000_adapter;

#include "e1000_hw.h"

#define E1000_MAX_INTR			10

/*
 * Count for polling __E1000_RESET condition every 10-20msec.
 */
#define E1000_CHECK_RESET_COUNT	50

/* TX/RX descriptor defines */
#define E1000_DEFAULT_TXD		256
#define E1000_MAX_TXD			256
#define E1000_MIN_TXD			48
#define E1000_MAX_82544_TXD		4096

#define E1000_DEFAULT_RXD		256
#define E1000_MAX_RXD			256
#define E1000_MIN_RXD			48
#define E1000_MAX_82544_RXD		4096

#define E1000_MIN_ITR_USECS		10 /* 100000 irq/sec */
#define E1000_MAX_ITR_USECS		10000 /* 100    irq/sec */

/* this is the size past which hardware will drop packets when setting LPE=0 */
#define MAXIMUM_ETHERNET_VLAN_SIZE	1522

/* Supported Rx Buffer Sizes */
#define E1000_RXBUFFER_128		128    /* Used for packet split */
#define E1000_RXBUFFER_256		256    /* Used for packet split */
#define E1000_RXBUFFER_512		512
#define E1000_RXBUFFER_1024		1024
#define E1000_RXBUFFER_2048		2048
#define E1000_RXBUFFER_4096		4096
#define E1000_RXBUFFER_8192		8192
#define E1000_RXBUFFER_16384		16384

/* SmartSpeed delimiters */
#define E1000_SMARTSPEED_DOWNSHIFT	3
#define E1000_SMARTSPEED_MAX		15

/* Packet Buffer allocations */
#define E1000_PBA_BYTES_SHIFT		0xA
#define E1000_TX_HEAD_ADDR_SHIFT	7
#define E1000_PBA_TX_MASK		0xFFFF0000

/* Flow Control Watermarks */
#define E1000_FC_HIGH_DIFF	0x1638 /* High: 5688 bytes below Rx FIFO size */
#define E1000_FC_LOW_DIFF	0x1640 /* Low:  5696 bytes below Rx FIFO size */

#define E1000_FC_PAUSE_TIME	0xFFFF /* pause for the max or until send xon */

/* How many Tx Descriptors do we need to call netif_wake_queue ? */
#define E1000_TX_QUEUE_WAKE	16
/* How many Rx Buffers do we bundle into one write to the hardware ? */
#define E1000_RX_BUFFER_WRITE	16 /* Must be power of 2 */

#define AUTO_ALL_MODES		0
#define E1000_EEPROM_82544_APM	0x0004
#define E1000_EEPROM_APME	0x0400

#ifndef E1000_MASTER_SLAVE
/* Switch to override PHY master/slave setting */
#define E1000_MASTER_SLAVE	e1000_ms_hw_default
#endif

#define E1000_MNG_VLAN_NONE	(-1)

/* wrapper around a pointer to a socket buffer,
 * so a DMA handle can be stored along with the buffer
 */
struct e1000_tx_buffer {
	struct sk_buff *skb;
	dma_addr_t dma;
	unsigned long time_stamp;
	u16 length;
	u16 next_to_watch;
	bool mapped_as_page;
	unsigned short segs;
	unsigned int bytecount;
};

struct e1000_rx_buffer {
	union {
		struct page *page; /* jumbo: alloc_page */
		u8 *data; /* else, netdev_alloc_frag */
	} rxbuf;
	dma_addr_t dma;
};

struct e1000_tx_ring {
	/* pointer to the descriptor ring memory */
	void *desc;
	/* physical address of the descriptor ring */
	dma_addr_t dma;
	/* length of descriptor ring in bytes */
	unsigned int size;
	/* number of descriptors in the ring */
	unsigned int count;
	/* next descriptor to associate a buffer with */
	unsigned int next_to_use;
	/* next descriptor to check for DD status bit */
	unsigned int next_to_clean;
	/* array of buffer information structs */
	struct e1000_tx_buffer *buffer_info;

	u16 tdh;
	u16 tdt;
	bool last_tx_tso;
};

struct e1000_rx_ring {
	/* pointer to the descriptor ring memory */
	void *desc;
	/* physical address of the descriptor ring */
	dma_addr_t dma;
	/* length of descriptor ring in bytes */
	unsigned int size;
	/* number of descriptors in the ring */
	unsigned int count;
	/* next descriptor to associate a buffer with */
	unsigned int next_to_use;
	/* next descriptor to check for DD status bit */
	unsigned int next_to_clean;
	/* array of buffer information structs */
	struct e1000_rx_buffer *buffer_info;
	struct sk_buff *rx_skb_top;

	/* cpu for rx queue */
	int cpu;

	u16 rdh;
	u16 rdt;
};

#define E1000_DESC_UNUSED(R)						\
({									\
	unsigned int clean = smp_load_acquire(&(R)->next_to_clean);	\
	unsigned int use = READ_ONCE((R)->next_to_use);			\
	(clean > use ? 0 : (R)->count) + clean - use - 1;		\
})

#define E1000_RX_DESC_EXT(R, i)						\
	(&(((union e1000_rx_desc_extended *)((R).desc))[i]))
#define E1000_GET_DESC(R, i, type)	(&(((struct type *)((R).desc))[i]))
#define E1000_RX_DESC(R, i)		E1000_GET_DESC(R, i, e1000_rx_desc)
#define E1000_TX_DESC(R, i)		E1000_GET_DESC(R, i, e1000_tx_desc)
#define E1000_CONTEXT_DESC(R, i)	E1000_GET_DESC(R, i, e1000_context_desc)

/* board specific private data structure */
struct e1000_adapter {
	unsigned long active_vlans[BITS_TO_LONGS(VLAN_N_VID)];
	u16 mng_vlan_id;
	u32 bd_number;
	u32 rx_buffer_len;
	u32 wol;
	u32 smartspeed;
	u32 en_mng_pt;
	u16 link_speed;
	u16 link_duplex;
	spinlock_t stats_lock;
	unsigned int total_tx_bytes;
	unsigned int total_tx_packets;
	unsigned int total_rx_bytes;
	unsigned int total_rx_packets;
	/* Interrupt Throttle Rate */
	u32 itr;
	u32 itr_setting;
	u16 tx_itr;
	u16 rx_itr;

	u8 fc_autoneg;

	/* TX */
	struct e1000_tx_ring *tx_ring; /* One per active queue */
	unsigned int restart_queue;
	u32 txd_cmd;
	u32 tx_int_delay;
	u32 tx_abs_int_delay;
	u32 gotcl;
	u64 gotcl_old;
	u64 tpt_old;
	u64 colc_old;
	u32 tx_timeout_count;
	u32 tx_fifo_head;
	u32 tx_head_addr;
	u32 tx_fifo_size;
	u8 tx_timeout_factor;
	atomic_t tx_fifo_stall;
	bool pcix_82544;
	bool detect_tx_hung;
	bool dump_buffers;

	/* RX */
	bool (*clean_rx)(struct e1000_adapter *adapter, struct e1000_rx_ring *rx_ring, int *work_done, int work_to_do);
	void (*alloc_rx_buf)(struct e1000_adapter *adapter, struct e1000_rx_ring *rx_ring, int cleaned_count);
	struct e1000_rx_ring *rx_ring; /* One per active queue */
	struct napi_struct napi;

	int num_tx_queues;
	int num_rx_queues;

	u64 hw_csum_err;
	u64 hw_csum_good;
	u32 alloc_rx_buff_failed;
	u32 rx_int_delay;
	u32 rx_abs_int_delay;
	bool rx_csum;
	u32 gorcl;
	u64 gorcl_old;

	/* OS defined structs */
	struct net_device *netdev;
	struct pci_dev *pdev;

	/* structs defined in e1000_hw.h */
	struct e1000_hw hw;
	struct e1000_hw_stats stats;
	struct e1000_phy_info phy_info;
	struct e1000_phy_stats phy_stats;

	u32 test_icr;
	struct e1000_tx_ring test_tx_ring;
	struct e1000_rx_ring test_rx_ring;

	int msg_enable;

	/* to not mess up cache alignment, always add to the bottom */
	bool tso_force;
	bool smart_power_down; /* phy smart power down */
	bool quad_port_a;
	unsigned long flags;
	u32 eeprom_wol;

	/* for ioport free */
	int bars;
	int need_ioport;

	bool discarding;

	struct work_struct reset_task;
	struct delayed_work watchdog_task;
	struct delayed_work fifo_stall_task;
	struct delayed_work phy_info_task;
};

enum e1000_state_t {
	__E1000_TESTING, __E1000_RESETTING, __E1000_DOWN
};

#undef pr_fmt
#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

struct net_device *e1000_get_hw_dev(struct e1000_hw *hw);
#define e_dbg(format, arg...) \
	netdev_dbg(e1000_get_hw_dev(hw), format, ## arg)
#define e_err(msglvl, format, arg...) \
	netif_err(adapter, msglvl, adapter->netdev, format, ## arg)
#define e_info(msglvl, format, arg...) \
	netif_info(adapter, msglvl, adapter->netdev, format, ## arg)
#define e_warn(msglvl, format, arg...) \
	netif_warn(adapter, msglvl, adapter->netdev, format, ## arg)
#define e_notice(msglvl, format, arg...) \
	netif_notice(adapter, msglvl, adapter->netdev, format, ## arg)
#define e_dev_info(format, arg...) \
	dev_info(&adapter->pdev->dev, format, ## arg)
#define e_dev_warn(format, arg...) \
	dev_warn(&adapter->pdev->dev, format, ## arg)
#define e_dev_err(format, arg...) \
	dev_err(&adapter->pdev->dev, format, ## arg)

#define COPYBREAK_DEFAULT 256
#define DRV_VERSION "7.3.21-k8-NAPI"
#define DRV_NAME "e1000"

int e1000_set_spd_dplx(struct e1000_adapter *adapter, u32 spd, u8 dplx);
bool e1000_has_link(struct e1000_adapter *adapter);
void e1000_power_up_phy(struct e1000_adapter *);
void e1000_set_ethtool_ops(struct net_device *netdev);
void e1000_check_options(struct e1000_adapter *adapter);
char *e1000_get_hw_dev_name(struct e1000_hw *hw);

int e1000_up(struct e1000_adapter *adapter);
void e1000_down(struct e1000_adapter *adapter);
void e1000_reinit_locked(struct e1000_adapter *adapter);
void e1000_reset(struct e1000_adapter *adapter);
int e1000_setup_all_tx_resources(struct e1000_adapter *adapter);
int e1000_setup_all_rx_resources(struct e1000_adapter *adapter);
void e1000_free_all_tx_resources(struct e1000_adapter *adapter);
void e1000_free_all_rx_resources(struct e1000_adapter *adapter);
static int e1000_setup_tx_resources(struct e1000_adapter *adapter, struct e1000_tx_ring *txdr);
static int e1000_setup_rx_resources(struct e1000_adapter *adapter, struct e1000_rx_ring *rxdr);
static void e1000_free_tx_resources(struct e1000_adapter *adapter, struct e1000_tx_ring *tx_ring);
static void e1000_free_rx_resources(struct e1000_adapter *adapter, struct e1000_rx_ring *rx_ring);
void e1000_update_stats(struct e1000_adapter *adapter);

static int e1000_init_module(void);
static void e1000_exit_module(void);
int e1000_probe(struct pci_dev *pdev, const struct pci_device_id *ent);
void e1000_remove(struct pci_dev *pdev);
int e1000_open(struct net_device *netdev);
int e1000_close(struct net_device *netdev);
static void e1000_configure_tx(struct e1000_adapter *adapter);
static void e1000_configure_rx(struct e1000_adapter *adapter);
static void e1000_setup_rctl(struct e1000_adapter *adapter);
static void e1000_clean_all_tx_rings(struct e1000_adapter *adapter);
static void e1000_clean_all_rx_rings(struct e1000_adapter *adapter);
static void e1000_clean_tx_ring(struct e1000_adapter *adapter, struct e1000_tx_ring *tx_ring);
static void e1000_clean_rx_ring(struct e1000_adapter *adapter, struct e1000_rx_ring *rx_ring);
static void e1000_set_rx_mode(struct net_device *netdev);
static netdev_tx_t e1000_xmit_frame(struct sk_buff *skb, struct net_device *netdev);
static int e1000_change_mtu(struct net_device *netdev, int new_mtu);
static int e1000_set_mac(struct net_device *netdev, void *p);
static irqreturn_t e1000_intr(int irq, void *data);
static bool e1000_clean_rx_irq(struct e1000_adapter *adapter, struct e1000_rx_ring *rx_ring, int *work_done, int work_to_do);
static bool e1000_clean_jumbo_rx_irq(struct e1000_adapter *adapter, struct e1000_rx_ring *rx_ring, int *work_done, int work_to_do);
static void e1000_alloc_dummy_rx_buffers(struct e1000_adapter *adapter, struct e1000_rx_ring *rx_ring, int cleaned_count) {
}
static void e1000_alloc_rx_buffers(struct e1000_adapter *adapter, struct e1000_rx_ring *rx_ring, int cleaned_count);
static void e1000_alloc_jumbo_rx_buffers(struct e1000_adapter *adapter, struct e1000_rx_ring *rx_ring, int cleaned_count);
static int e1000_ioctl(struct net_device *netdev, struct ifreq *ifr, int cmd);
static int e1000_mii_ioctl(struct net_device *netdev, struct ifreq *ifr, int cmd);
static void e1000_enter_82542_rst(struct e1000_adapter *adapter);
static void e1000_leave_82542_rst(struct e1000_adapter *adapter);
static void e1000_tx_timeout(struct net_device *dev);
static int e1000_82547_fifo_workaround(struct e1000_adapter *adapter, struct sk_buff *skb);

static bool e1000_vlan_used(struct e1000_adapter *adapter);
static void e1000_vlan_mode(struct net_device *netdev, netdev_features_t features);
void e1000_vlan_filter_on_off(struct e1000_adapter *adapter, bool filter_on);
static int e1000_vlan_rx_add_vid(struct net_device *netdev, __be16 proto, u16 vid);
static int e1000_vlan_rx_kill_vid(struct net_device *netdev, __be16 proto, u16 vid);
static void e1000_restore_vlan(struct e1000_adapter *adapter);
int e1000_suspend(struct pci_dev *pdev, pm_message_t state);
int e1000_resume(struct pci_dev *pdev);
void e1000_shutdown(struct pci_dev *pdev);

#ifdef CONFIG_NET_POLL_CONTROLLER
/* for netdump / net console */
static void e1000_netpoll(struct net_device *netdev);
#endif

static pci_ers_result_t e1000_io_error_detected(struct pci_dev *pdev, pci_channel_state_t state);
static pci_ers_result_t e1000_io_slot_reset(struct pci_dev *pdev);
static void e1000_io_resume(struct pci_dev *pdev);
void e1000_irq_enable(struct e1000_adapter *adapter);
void e1000_irq_disable(struct e1000_adapter *adapter);
void e1000_unmap_and_free_tx_resource(struct e1000_adapter *adapter, struct e1000_tx_buffer *buffer_info);
void e1000_down_and_stop(struct e1000_adapter *adapter);
void e1000_release_manageability(struct e1000_adapter *adapter);
int __e1000_shutdown(struct pci_dev *pdev, bool *enable_wake);
int e1000_request_irq(struct e1000_adapter *adapter);
void e1000_init_manageability(struct e1000_adapter *adapter);

void e1000_diag_test(struct net_device *netdev, struct ethtool_test *eth_test, u64 *data);

#endif /* _E1000_H_ */
