/* sanremo.c: An ethernet driver for the IBM 10/100 Mbps TX MCA Ethernet (9-K)
              Micro Channel card, code name "San Remo",
              with Adaptec ASIC-9060R as an MCA-PCI bridge
              and the AMD PCnet-FAST Am79C971 PCI Ethernet controller */
/*
 *     Copyright 2023 Christian Holzapfel, Ryan Alswede
 *
 *     Derived from the pcnet32 driver written 1996-1999 by Thomas Bogendoerfer.
 *
 *     Derived from the lance driver written 1993,1994,1995 by Donald Becker.
 *
 *     Copyright 1993 United States Government as represented by the
 *     Director, National Security Agency.
 *
 *     This software may be used and distributed according to the terms
 *     of the GNU Public License, incorporated herein by reference.
 *
 *     This driver is for the IBM 9-K ethernet card only.
 */

#define DRV_NAME	"sanremo"
#define DRV_VERSION	"1.04"
#define DRV_RELDATE	"20.02.2024"

static const char *version =
DRV_NAME ".c:v" DRV_VERSION " " DRV_RELDATE " christian@holzapfel.biz\n";

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/string.h>
#include <linux/ptrace.h>
#include <linux/errno.h>
#include <linux/ioport.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/pci.h>
#include <linux/mca.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/ethtool.h>
#include <linux/mii.h>
#include <asm/bitops.h>
#include <asm/io.h>
#include <asm/dma.h>
#include <asm/uaccess.h>

#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>
#include <linux/spinlock.h>


static int sanremo_debug = 1;
static int tx_start = 1; /* Mapping -- 0:20, 1:64, 2:128, 3:~220 (depends on chip vers) */

static struct net_device *sanremo_dev;

static const int max_interrupt_work = 80;
static const int rx_copybreak = 200;

#define PCNET32_PORT_AUI      0x00
#define PCNET32_PORT_10BT     0x01
#define PCNET32_PORT_GPSI     0x02
#define PCNET32_PORT_MII      0x03

#define PCNET32_PORT_PORTSEL  0x03
#define PCNET32_PORT_ASEL     0x04
#define PCNET32_PORT_100      0x40
#define PCNET32_PORT_FD	      0x80

#define PCNET32_DMA_MASK 0xffffffff

/*
 * table to translate option values from tulip
 * to internal options
 */
static unsigned char options_mapping[] = {
    PCNET32_PORT_ASEL,			   /*  0 Auto-select	  */
    PCNET32_PORT_AUI,			   /*  1 BNC/AUI	  */
    PCNET32_PORT_AUI,			   /*  2 AUI/BNC	  */ 
    PCNET32_PORT_ASEL,			   /*  3 not supported	  */
    PCNET32_PORT_10BT | PCNET32_PORT_FD,	   /*  4 10baseT-FD	  */
    PCNET32_PORT_ASEL,			   /*  5 not supported	  */
    PCNET32_PORT_ASEL,			   /*  6 not supported	  */
    PCNET32_PORT_ASEL,			   /*  7 not supported	  */
    PCNET32_PORT_ASEL,			   /*  8 not supported	  */
    PCNET32_PORT_MII,			   /*  9 MII 10baseT	  */
    PCNET32_PORT_MII | PCNET32_PORT_FD,		   /* 10 MII 10baseT-FD	  */
    PCNET32_PORT_MII,			   /* 11 MII (autosel)	  */
    PCNET32_PORT_10BT,			   /* 12 10BaseT	  */
    PCNET32_PORT_MII | PCNET32_PORT_100,   /* 13 MII 100BaseTx	  */
    PCNET32_PORT_MII | PCNET32_PORT_100 | PCNET32_PORT_FD, /* 14 MII 100BaseTx-FD */
    PCNET32_PORT_ASEL			   /* 15 not supported	  */
};

#define MAX_UNITS 1
static int options[MAX_UNITS];
static int full_duplex[MAX_UNITS];

/*
 *                 Theory of Operation
 *
 * This driver uses the same software structure as the normal pcnet32
 * driver. The difference is in the presence of the Adaptec ASIC-9060R
 * PCI-to-MCA bridge chip that needs to be initialized, configured,
 * and all I/O traffic to the PCnet-FAST Am79C971 tunneled through.
 */

/*
 * History:
 * v1.00:  Initial version
 * v1.01:  Strange access in sanremo_init_asic() clarified
 * v1.02:  VPD access updated and commented out
 * v1.03:  Updated from pcnet32 v1.26p
 * v1.04:  ASIC initialization corrected
 */

/*
 * Set the number of Tx and Rx buffers, using Log_2(# buffers).
 * Reasonable default values are 4 Tx buffers, and 16 Rx buffers.
 * That translates to 2 (4 == 2^^2) and 4 (16 == 2^^4).
 */
#ifndef PCNET32_LOG_TX_BUFFERS
#define PCNET32_LOG_TX_BUFFERS 4
#define PCNET32_LOG_RX_BUFFERS 5
#endif

#define TX_RING_SIZE			(1 << (PCNET32_LOG_TX_BUFFERS))
#define TX_RING_MOD_MASK		(TX_RING_SIZE - 1)
#define TX_RING_LEN_BITS		((PCNET32_LOG_TX_BUFFERS) << 12)

#define RX_RING_SIZE			(1 << (PCNET32_LOG_RX_BUFFERS))
#define RX_RING_MOD_MASK		(RX_RING_SIZE - 1)
#define RX_RING_LEN_BITS		((PCNET32_LOG_RX_BUFFERS) << 4)

#define PKT_BUF_SZ		1544

/* Offsets from base I/O address. */
#define PCNET32_DWIO_RDP	0x10
#define PCNET32_DWIO_RAP	0x14
#define PCNET32_DWIO_RESET	0x18
#define PCNET32_DWIO_BDP	0x1C

#define SANREMO_TOTAL_SIZE	0x400

#define CRC_POLYNOMIAL_LE 0xedb88320UL	/* Ethernet CRC, little endian */

/* IBM MCA 10/100 Mbps Ethernet (9-K) "San Remo" specific */

/* The Micro Channel Adapter ID */
#define SANREMO_ADAPTER_ID              0x8F62
#define SANREMO_ADAPTER_NAME            "IBM 10/100 Mbps Ethernet (9-K)"

/* The ASIC puts the PCnet at an offset of
   0x1000 above the ASICs slave IO space */
#define ASIC_IO_OFFSET                  0x00001000

/* The ASIC's register to write the PCI_CONFIG_CMD
   to to access the daughter card's PCI config space */
#define ASIC_PCI_CONFIG_CMD_REGISTER    0x04

/* Magic bit to access the daughter card's PCI
   configuration space */
#define ASIC_PCI_CONFIG_CMD             0x01

/* The ASIC's daughter card IO tunnel address
   register to write the PCnet's register address to */
#define ASIC_IO_ADDRESS_REGISTER        0x08

/* The ASIC's daughter card IO tunnel data
   register to read/write the PCnet's selected register */
#define ASIC_IO_DATA_REGISTER           0x0C

/* The ASIC's interrupt enable/disable register */
#define ASIC_IRQ_ENABLE_REGISTER        0x1C

/* The ASIC's index register for accessing
    the VPD data, byte by byte */
#define ASIC_VPD_INDEX_REGISTER         0x24

/* The ASIC's <unknown> register, related to
   the VPD data. "VPD EEPROM operation ok" indicator? */
#define ASIC_VPD_STATUS_REGISTER        0x25

/* The ASIC's data register for reading(/writing?)
   the VPD data, byte by byte */
#define ASIC_VPD_DATA_REGISTER          0x26

/* The PCNET32 Rx and Tx ring descriptors. */
struct pcnet32_rx_head {
    u32 base;
    s16 buf_length;
    s16 status;	   
    u32 msg_length;
    u32 reserved;
};
	
struct pcnet32_tx_head {
    u32 base;
    s16 length;
    s16 status;
    u32 misc;
    u32 reserved;
};

/* The PCNET32 32-Bit initialization block, described in databook. */
struct pcnet32_init_block {
    u16 mode;
    u16 tlen_rlen;
    u8	phys_addr[6];
    u16 reserved;
    u32 filter[2];
    /* Receive and transmit ring base, along with extra bits. */    
    u32 rx_ring;
    u32 tx_ring;
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
 * The first three fields of sanremo_private are read by the ethernet device
 * so we allocate the structure should be allocated by pci_alloc_consistent().
 */
struct sanremo_private {
    /* The Tx and Rx ring entries must be aligned on 16-byte boundaries in 32bit mode. */
    struct pcnet32_rx_head   rx_ring[RX_RING_SIZE];
    struct pcnet32_tx_head   tx_ring[TX_RING_SIZE];
    struct pcnet32_init_block	init_block;
    dma_addr_t dma_addr;		/* DMA address of beginning of this object, returned by pci_alloc_consistent */
    struct pci_dev *pci_dev;		/* Pointer to the associated pci device structure */
    const char *name;
    /* The saved address of a sent-in-place packet/buffer, for skfree(). */
    struct sk_buff *tx_skbuff[TX_RING_SIZE];
    struct sk_buff *rx_skbuff[RX_RING_SIZE];
    dma_addr_t tx_dma_addr[TX_RING_SIZE];
    dma_addr_t rx_dma_addr[RX_RING_SIZE];
    struct pcnet32_access a;
    spinlock_t lock;					/* Guard lock */
    unsigned int cur_rx, cur_tx;		/* The next free ring entry */
    unsigned int dirty_rx, dirty_tx;	/* The ring entries to be free()ed. */
    struct net_device_stats stats;
    char tx_full;
    int	 options;
    int	 shared_irq:1,			/* shared irq possible */
	ltint:1,
      dxsuflo:1,			    /* disable transmit stop on uflo */
	mii:1;					/* mii port available */
	int slot;
    struct net_device *next;
    struct mii_if_info mii_if;
};

static int  sanremo_probe_mca(void);
static int  sanremo_init_asic(unsigned long ioaddr);
static int  sanremo_read_pci_config (unsigned long, int);
static void sanremo_write_pci_config (unsigned long, int, int);
static void sanremo_enable_interrupt (unsigned long);
static void sanremo_disable_interrupt (unsigned long);
static int  sanremo_probe1(unsigned long, unsigned char irq_line, int, struct pci_dev *, int);
static int  sanremo_open(struct net_device *);
static int  sanremo_init_ring(struct net_device *);
static int  sanremo_start_xmit(struct sk_buff *, struct net_device *);
static int  sanremo_rx(struct net_device *);
static void sanremo_tx_timeout (struct net_device *dev);
static void sanremo_interrupt(int, void *, struct pt_regs *);
static int  sanremo_close(struct net_device *);
static struct net_device_stats *sanremo_get_stats(struct net_device *);
static void sanremo_set_multicast_list(struct net_device *);
static int  sanremo_ioctl(struct net_device *, struct ifreq *, int);
static int mdio_read(struct net_device *dev, int phy_id, int reg_num);
static void mdio_write(struct net_device *dev, int phy_id, int reg_num, int val);


static u16 pcnet32_dwio_read_csr (unsigned long addr, int index)
{
    int val = 0;
    outl(addr + ASIC_IO_OFFSET + PCNET32_DWIO_RAP, addr + ASIC_IO_ADDRESS_REGISTER);
    outl(index, addr + ASIC_IO_DATA_REGISTER);
    outl(addr + ASIC_IO_OFFSET + PCNET32_DWIO_RDP, addr + ASIC_IO_ADDRESS_REGISTER);
    val = (inl(addr + ASIC_IO_DATA_REGISTER) & 0xFFFF);
    return (val);
}

static void pcnet32_dwio_write_csr (unsigned long addr, int index, u16 val)
{
    outl(addr + ASIC_IO_OFFSET + PCNET32_DWIO_RAP, addr + ASIC_IO_ADDRESS_REGISTER);
    outl(index, addr + ASIC_IO_DATA_REGISTER);
    outl(addr + ASIC_IO_OFFSET + PCNET32_DWIO_RDP, addr + ASIC_IO_ADDRESS_REGISTER);
    outl(val, addr + ASIC_IO_DATA_REGISTER);
}

static u16 pcnet32_dwio_read_bcr (unsigned long addr, int index)
{
    outl(addr + ASIC_IO_OFFSET + PCNET32_DWIO_RAP, addr + ASIC_IO_ADDRESS_REGISTER);
    outl(index, addr + ASIC_IO_DATA_REGISTER);
    outl(addr + ASIC_IO_OFFSET + PCNET32_DWIO_BDP, addr + ASIC_IO_ADDRESS_REGISTER);
    return (inl(addr + ASIC_IO_DATA_REGISTER) & 0xFFFF);
}

static void pcnet32_dwio_write_bcr (unsigned long addr, int index, u16 val)
{
    outl(addr + ASIC_IO_OFFSET + PCNET32_DWIO_RAP, addr + ASIC_IO_ADDRESS_REGISTER);
    outl(index, addr + ASIC_IO_DATA_REGISTER);
    outl(addr + ASIC_IO_OFFSET + PCNET32_DWIO_BDP, addr + ASIC_IO_ADDRESS_REGISTER);
    outl(val, addr + ASIC_IO_DATA_REGISTER);
}

static u16 pcnet32_dwio_read_rap (unsigned long addr)
{
    outl(addr + ASIC_IO_OFFSET + PCNET32_DWIO_RAP, addr + ASIC_IO_ADDRESS_REGISTER);
    return (inl (addr + ASIC_IO_DATA_REGISTER) & 0xFFFF);
}

static void pcnet32_dwio_write_rap (unsigned long addr, u16 val)
{
    outl(addr + ASIC_IO_OFFSET + PCNET32_DWIO_RAP, addr + ASIC_IO_ADDRESS_REGISTER);
    outl(val, addr + ASIC_IO_DATA_REGISTER);
}

static void pcnet32_dwio_reset (unsigned long addr)
{
    outl(addr + ASIC_IO_OFFSET + PCNET32_DWIO_RESET, addr + ASIC_IO_ADDRESS_REGISTER);
    inl(addr + ASIC_IO_DATA_REGISTER);
}

static int pcnet32_dwio_check (unsigned long addr)
{
    outl(addr + ASIC_IO_OFFSET + PCNET32_DWIO_RAP, addr + ASIC_IO_ADDRESS_REGISTER);
    outl(88, addr + ASIC_IO_DATA_REGISTER);
    return (inl(addr + ASIC_IO_DATA_REGISTER) == 88);
}

static struct pcnet32_access pcnet32_dwio = {
    pcnet32_dwio_read_csr,
    pcnet32_dwio_write_csr,
    pcnet32_dwio_read_bcr,
    pcnet32_dwio_write_bcr,
    pcnet32_dwio_read_rap,
    pcnet32_dwio_write_rap,
    pcnet32_dwio_reset

};


static int __init sanremo_probe_mca(void)
{
    unsigned long ioaddr = 0;
    unsigned int  irq_line = 0;
    unsigned int  dma = 0;
    int slot = MCA_NOTFOUND;
    int err = -ENODEV;

    slot = mca_find_unused_adapter(SANREMO_ADAPTER_ID, 0);
    if (slot != MCA_NOTFOUND)
    {
        /* Upper 6 bits of POS[2] contain the IO base * 0x100) */
        ioaddr = ((mca_read_pos(slot, 2) & 0xFC)) << 8;

        /* Lower 2 bits of POS[5] encode the IRQ */
        switch (mca_read_pos(slot, 5) & 0x03)
        {
            case 0x00: irq_line = 15; break;
            case 0x01: irq_line = 12; break;
            case 0x02: irq_line = 11; break;
            default:   irq_line = 10; break;
        }

        /* Upper 4 bits of POS[3] contain the DMA arbitration level.
           Unused from CPU side, but reserved by POS for the adapter's
           busmaster to use */
        dma = mca_read_pos(slot, 3) >> 4;

        printk("%s found in MCA slot %d using I/O 0x%4.4lX, IRQ %u, DMA %d\n",
            SANREMO_ADAPTER_NAME, slot, ioaddr, irq_line, dma);

        sanremo_init_asic(ioaddr);

        err = sanremo_probe1(ioaddr, irq_line, 1, NULL, slot);    // MCA = Shared IRQ
    }
    
    return err;
}

/* sanremo_init_asic */
static int __devinit
sanremo_init_asic(unsigned long ioaddr)
{
    int eepromInitDone;
    int temp;

    /* This sequence of writes goes out to the San Remo ASIC
       and is required to start it up.
       IBM only knows what they mean. */
    outb(0x00,       ioaddr + 0x1D);
    outb(0x0F,       ioaddr + 0x1E);
    outb(0x04,       ioaddr + 0x1F);
    outl(0x00000000, ioaddr + 0x28);
    outw(0x0006,     ioaddr + 0x00);
    outl(0x00000000, ioaddr + 0x10);
    outl(0x00000000, ioaddr + 0x14);
    outw(0x0FFF,     ioaddr + 0x1A);
    outb(0x3F,       ioaddr + 0x22);
    outw(0x03FF,     ioaddr + 0x20);

    /* Set up the PCnet's PCI Configuration Space through the ASIC */
    temp = sanremo_read_pci_config(ioaddr, 0x0C);                        // Read Latency and Header Type
    sanremo_write_pci_config(ioaddr, 0x0C, 0x0000FF00);                  // Write Latency and Header Type
    sanremo_write_pci_config(ioaddr, 0x10, ASIC_IO_OFFSET + ioaddr);     // Write I/O Base Address
    sanremo_write_pci_config(ioaddr, 0x04, 0x00000145);                  // Write Control: SERREN, PERREN, BMEN, IOEN
    temp = sanremo_read_pci_config(ioaddr, 0x08);                        // Read PCI Revision ID

    /* The following 32-bit accesses will switch the PCnet from 16-bit WIO address mode to the
       32-bit DWIO mode. Maybe DWIO is the only one supported by the ASIC, I have never tested WIO.
       The original AIX driver shifts gears into DWIO mode as first action between the driver and
       the PCnet, and in Linux we do the same. Guaranteed to work fine.
       From the Am79C971 datasheet:
          "The Software can invoke the DWIO mode by performing a DWord write
           access to the I/O location at offset 10h (RDP)"                     */
    outl(ioaddr + ASIC_IO_OFFSET + PCNET32_DWIO_RDP, ioaddr + ASIC_IO_ADDRESS_REGISTER);      
    temp = inl(ioaddr + ASIC_IO_DATA_REGISTER); 
    outl(ioaddr + ASIC_IO_OFFSET + PCNET32_DWIO_RDP, ioaddr + ASIC_IO_ADDRESS_REGISTER);      
    outl(temp, ioaddr + ASIC_IO_DATA_REGISTER);

    /* Check BDP19 = EECAS = EEPROM Control and Status for bit 0x8000 = PVALID,
       that indicates an EEPROM has been read and found valid. */
    eepromInitDone = pcnet32_dwio_read_bcr(ioaddr, 19) & 0x8000;
    if (!eepromInitDone)
    {
        int eepromValid;

        /* Start EEPROM read.
           This will trigger the PCnet to read the EEPROM
           and initialize some registers from the data. */
        pcnet32_dwio_write_bcr(ioaddr, 19, 0x4000);

        /* Delay until EEPROM is read */
        mdelay(1000);

        /* Check BDP19 = EECAS = EEPROM Control and Status for bit 0x8000 = PVALID set,
        that indicates the EEPROM has been read and is checksum-correct */
        eepromValid = pcnet32_dwio_read_bcr(ioaddr, 19) & 0x8000;

        if (!eepromValid)
            return EIO;
    }

    /* Read out the VPD data. We don't need it here, outside of AIX,
       but who knows if it is required to make the ASIC happy.
       And we keep it here for reference, to show how it could be done.
       It could probably also get written through this interface.
    for (i = 0; i < 256; i += 2)
    {
        short data = 0;
        char status = 0;

        outb(i, ioaddr + ASIC_VPD_INDEX_REGISTER);
        mdelay(1);
        status = inb(ioaddr + ASIC_VPD_STATUS_REGISTER);
        data = inw(ioaddr + ASIC_VPD_DATA_REGISTER);

        if (status == 1)
        {
            printk("%c%c", data & 0xFF, data >> 8);
        }
    }
    */

    return 0;
}

/* sanremo_read_pci_config */
static int sanremo_read_pci_config (unsigned long ioaddr, int index)
{
    outb(ASIC_PCI_CONFIG_CMD, ioaddr + ASIC_PCI_CONFIG_CMD_REGISTER);
    outl(index, ioaddr + ASIC_IO_ADDRESS_REGISTER);
    return (inl(ioaddr + ASIC_IO_DATA_REGISTER));
}

/* sanremo_write_pci_config */
static void sanremo_write_pci_config (unsigned long ioaddr, int index, int val)
{
    outb(ASIC_PCI_CONFIG_CMD, ioaddr + ASIC_PCI_CONFIG_CMD_REGISTER);
    outl(index, ioaddr + ASIC_IO_ADDRESS_REGISTER);
    outl(val, ioaddr + ASIC_IO_DATA_REGISTER);
}

/* sanremo_enable_interrupt */
static void sanremo_enable_interrupt (unsigned long ioaddr)
{
    outb(1, ioaddr + ASIC_IRQ_ENABLE_REGISTER);
}

/* sanremo_disable_interrupt */
static void sanremo_disable_interrupt (unsigned long ioaddr)
{
    outb(0, ioaddr + ASIC_IRQ_ENABLE_REGISTER);
}

/* sanremo_probe1 */
static int __devinit
sanremo_probe1(unsigned long ioaddr, unsigned char irq_line, int shared, struct pci_dev *pdev, int slot)
{
    struct sanremo_private *lp;
    struct resource *res;
    dma_addr_t lp_dma_addr;
    int i,fdx = 0, mii = 0, fset = 0;
    int dxsuflo = 0;
    int ltint = 0;
    int chip_version;
    char *chipname;
    struct net_device *dev;
    struct pcnet32_access *a = NULL;
    u8 promaddr[6];

    /* reset the chip */
    pcnet32_dwio_reset(ioaddr);

    if (pcnet32_dwio_read_csr (ioaddr, 0) == 4 && pcnet32_dwio_check(ioaddr))
    {
        a = &pcnet32_dwio;
	} else
	    return -ENODEV;

    chip_version = a->read_csr(ioaddr, 88) | (a->read_csr(ioaddr,89) << 16);
    if (sanremo_debug > 2)
	printk(KERN_INFO "  PCnet chip version is %#x.\n", chip_version);
    if ((chip_version & 0xfff) != 0x003)
	return -ENODEV;
    chip_version = (chip_version >> 12) & 0xffff;

    if (chip_version == 0x2623) {
	chipname = "PCnet/FAST 79C971"; /* PCI */
	fdx = 1; mii = 1; fset = 1;
	ltint = 1;
    }
    else {
	printk(KERN_INFO "sanremo: PCnet version %#x, no PCnet32 chip.\n",chip_version);
	return -ENODEV;
    }

    /*
     *	On selected chips turn on the BCR18:NOUFLO bit. This stops transmit
     *	starting until the packet is loaded. Strike one for reliability, lose
     *	one for latency - although on PCI this isnt a big loss. Older chips
     *	have FIFO's smaller than a packet, so you can't do this.
     *	Turn on BCR18:BurstRdEn and BCR18:BurstWrEn.
     */

    if (fset) 
	{
	a->write_bcr(ioaddr, 18, (a->read_bcr(ioaddr, 18) | 0x0860));
	a->write_csr(ioaddr, 80, (a->read_csr(ioaddr, 80) & 0x0C00) | 0x0c00);
	dxsuflo = 1;
	ltint = 1;
    }
    
    dev = init_etherdev(NULL, 0);
    if(dev==NULL)
	return -ENOMEM;

    printk(KERN_INFO "%s: %s at %#3lx,", dev->name, chipname, ioaddr);

    /* read PROM address and compare with CSR address */
    {
        int val;
        outl(ASIC_IO_OFFSET + ioaddr + 0x00, ioaddr + ASIC_IO_ADDRESS_REGISTER);

        /* Through the ASIC we're in word-only access mode, so we read 4 bytes at once */
        val = inl(ioaddr + ASIC_IO_DATA_REGISTER);
        promaddr[0] = val & 0xFF;
        promaddr[1] = (val >> 8) & 0xFF;
        promaddr[2] = (val >> 16) & 0xFF;
        promaddr[3] = (val >> 24) & 0xFF;

        outl(ASIC_IO_OFFSET + ioaddr + 0x04, ioaddr + ASIC_IO_ADDRESS_REGISTER);
        val = inl(ioaddr + ASIC_IO_DATA_REGISTER);
        promaddr[4] = val & 0xFF;
        promaddr[5] = (val >> 8) & 0xFF;
	    memcpy(dev->dev_addr, promaddr, 6);
    }

    /* if the ethernet address is not valid, force to 00:00:00:00:00:00 */
    if( !is_valid_ether_addr(dev->dev_addr) )
	for (i = 0; i < 6; i++)
	    dev->dev_addr[i]=0;

    for (i = 0; i < 6; i++)
	printk(" %2.2x", dev->dev_addr[i] );

    if (((chip_version + 1) & 0xfffe) == 0x2624) { /* Version 0x2623 or 0x2624 */
	i = a->read_csr(ioaddr, 80) & 0x0C00;  /* Check tx_start_pt */
	printk("\n" KERN_INFO "    tx_start_pt(0x%04x):",i);
	switch(i>>10) {
	    case 0: printk("  20 bytes,"); break;
	    case 1: printk("  64 bytes,"); break;
	    case 2: printk(" 128 bytes,"); break;
	    case 3: printk("~220 bytes,"); break;
	}
	i = a->read_bcr(ioaddr, 18);  /* Check Burst/Bus control */
	printk(" BCR18(%x):",i&0xffff);
	if (i & (1<<5)) printk("BurstWrEn ");
	if (i & (1<<6)) printk("BurstRdEn ");
	if (i & (1<<7)) printk("DWordIO ");
	if (i & (1<<11)) printk("NoUFlow ");
	i = a->read_bcr(ioaddr, 25);
	printk("\n" KERN_INFO "    SRAMSIZE=0x%04x,",i<<8);
	i = a->read_bcr(ioaddr, 26);
	printk(" SRAM_BND=0x%04x,",i<<8);
	i = a->read_bcr(ioaddr, 27);
	if (i & (1<<14)) printk("LowLatRx");
    }

    dev->base_addr = ioaddr;
    res = request_region(ioaddr, SANREMO_TOTAL_SIZE, chipname);
    if (res == NULL)
    {
        printk(KERN_ERR "sanremo: Error requesting region.\n");
        return -EBUSY;
    }
    
    /* pci_alloc_consistent returns page-aligned memory, so we do not have to check the alignment */
    if ((lp = pci_alloc_consistent(pdev, sizeof(*lp), &lp_dma_addr)) == NULL) {
    printk(KERN_ERR "sanremo: Error allocating memory.\n");
	release_resource(res);
	return -ENOMEM;
    }

    memset(lp, 0, sizeof(*lp));
    lp->dma_addr = lp_dma_addr;
    lp->pci_dev = pdev;
    printk("\n" KERN_INFO "sanremo: sanremo_private lp=%p lp_dma_addr=%#08x", lp, (unsigned int)lp_dma_addr);

    spin_lock_init(&lp->lock);

    mca_set_adapter_name(slot, SANREMO_ADAPTER_NAME);
    mca_mark_as_used(slot);

    dev->priv = lp;
    lp->name = chipname;
    lp->shared_irq = shared;
    lp->slot = slot;
    lp->mii_if.full_duplex = fdx;
    lp->dxsuflo = dxsuflo;
    lp->ltint = ltint;
    lp->mii = mii;
    if (options[0] > sizeof (options_mapping))
	lp->options = PCNET32_PORT_ASEL;
    else
	lp->options = options_mapping[options[0]];
    lp->mii_if.dev = dev;
    lp->mii_if.mdio_read = mdio_read;
    lp->mii_if.mdio_write = mdio_write;
    
    if (fdx && !(lp->options & PCNET32_PORT_ASEL) && full_duplex[0])
	lp->options |= PCNET32_PORT_FD;
    
    if (a == NULL) {
      printk(KERN_ERR "pcnet32: No access methods\n");
      pci_free_consistent(lp->pci_dev, sizeof(*lp), lp, lp->dma_addr);
      release_resource(res);
      return -ENODEV;
    }
    lp->a = *a;
    
    /* detect special T1/E1 WAN card by checking for MAC address */
    if (dev->dev_addr[0] == 0x00 && dev->dev_addr[1] == 0xe0 && dev->dev_addr[2] == 0x75)
	lp->options = PCNET32_PORT_FD | PCNET32_PORT_GPSI;

    lp->init_block.mode = le16_to_cpu(0x0003);	/* Disable Rx and Tx. */
    lp->init_block.tlen_rlen = le16_to_cpu(TX_RING_LEN_BITS | RX_RING_LEN_BITS); 
    for (i = 0; i < 6; i++)
	lp->init_block.phys_addr[i] = dev->dev_addr[i];
    lp->init_block.filter[0] = 0x00000000;
    lp->init_block.filter[1] = 0x00000000;
    lp->init_block.rx_ring = (u32)le32_to_cpu(lp->dma_addr + offsetof(struct sanremo_private, rx_ring));
    lp->init_block.tx_ring = (u32)le32_to_cpu(lp->dma_addr + offsetof(struct sanremo_private, tx_ring));
    
    /* switch pcnet32 to 32bit mode */
    a->write_bcr (ioaddr, 20, 2);

    a->write_csr (ioaddr, 1, (lp->dma_addr + offsetof(struct sanremo_private, init_block)) & 0xffff);
    a->write_csr (ioaddr, 2, (lp->dma_addr + offsetof(struct sanremo_private, init_block)) >> 16);
    
    if (irq_line) {
	dev->irq = irq_line;
    }
    
    if (dev->irq >= 2)
        printk(" assigned IRQ %d.\n", dev->irq);
	else
	{
	    printk(", failed to detect IRQ line.\n");
	    pci_free_consistent(lp->pci_dev, sizeof(*lp), lp, lp->dma_addr);
	    mca_mark_as_unused(slot);
	    release_resource(res);
		return -ENODEV;
	}
    if (sanremo_debug > 0)
	printk(KERN_INFO "%s", version);
    /* The PCNET32-specific entries in the device structure. */
    dev->open = &sanremo_open;
    dev->hard_start_xmit = &sanremo_start_xmit;
    dev->stop = &sanremo_close;
    dev->get_stats = &sanremo_get_stats;
    dev->set_multicast_list = &sanremo_set_multicast_list;
    dev->do_ioctl = &sanremo_ioctl;
    dev->tx_timeout = sanremo_tx_timeout;
    dev->watchdog_timeo = (HZ >> 1);

    lp->next = sanremo_dev;
    sanremo_dev = dev;

    /* Fill in the generic fields of the device structure. */
    ether_setup(dev);
    
    return 0;
}


static int
sanremo_open(struct net_device *dev)
{
    struct sanremo_private *lp = dev->priv;
    unsigned long ioaddr = dev->base_addr;
    u16 val;
    int i;

    if (dev->irq == 0 ||
	request_irq(dev->irq, &sanremo_interrupt,
		    lp->shared_irq ? SA_SHIRQ : 0, lp->name, (void *)dev)) {
    printk(KERN_ERR "sanremo: Error requesting IRQ.\n");
	return -EAGAIN;
    }

    /* Check for a valid station address */
    if( !is_valid_ether_addr(dev->dev_addr) )
    {
        printk(KERN_ERR "sanremo: Error: invalid station address.\n");
        return -EINVAL;
    }

    /* Reset the PCNET32 */
    lp->a.reset (ioaddr);

    sanremo_enable_interrupt(ioaddr);

    /* switch pcnet32 to 32bit mode */
    lp->a.write_bcr (ioaddr, 20, 2);

    if (sanremo_debug > 1)
	printk(KERN_DEBUG "%s: sanremo_open() irq %d tx/rx rings %#x/%#x init %#x.\n",
	       dev->name, dev->irq,
	       (u32) (lp->dma_addr + offsetof(struct sanremo_private, tx_ring)),
	       (u32) (lp->dma_addr + offsetof(struct sanremo_private, rx_ring)),
	       (u32) (lp->dma_addr + offsetof(struct sanremo_private, init_block)));

    /* set/reset autoselect bit */
    val = lp->a.read_bcr (ioaddr, 2) & ~2;
    if (lp->options & PCNET32_PORT_ASEL)
	val |= 2;
    lp->a.write_bcr (ioaddr, 2, val);
    
    /* handle full duplex setting */
    if (lp->mii_if.full_duplex) {
	val = lp->a.read_bcr (ioaddr, 9) & ~3;
	if (lp->options & PCNET32_PORT_FD) {
	    val |= 1;
	    if (lp->options == (PCNET32_PORT_FD | PCNET32_PORT_AUI))
		val |= 2;
	}
	lp->a.write_bcr (ioaddr, 9, val);
    }
    
    /* set/reset GPSI bit in test register */
    val = lp->a.read_csr (ioaddr, 124) & ~0x10;
    if ((lp->options & PCNET32_PORT_PORTSEL) == PCNET32_PORT_GPSI)
	val |= 0x10;
    lp->a.write_csr (ioaddr, 124, val);

    /* 24 Jun 2004 according AMD, in order to change the PHY,
     * DANAS (or DISPM for 79C976) must be set; then select the speed,
     * duplex, and/or enable auto negotiation, and clear DANAS */
    if (lp->mii && !(lp->options & PCNET32_PORT_ASEL)) {
	lp->a.write_bcr(ioaddr, 32, lp->a.read_bcr(ioaddr, 32) | 0x0080);
	/* disable Auto Negotiation, set 10Mpbs, HD */
	val = lp->a.read_bcr(ioaddr, 32) & ~0xb8;
	if (lp->options & PCNET32_PORT_FD)
	    val |= 0x10;
	if (lp->options & PCNET32_PORT_100)
	    val |= 0x08;
	lp->a.write_bcr (ioaddr, 32, val);
    } else {
	if (lp->options & PCNET32_PORT_ASEL) {
	    lp->a.write_bcr(ioaddr, 32, lp->a.read_bcr(ioaddr, 32) | 0x0080);
	    /* enable auto negotiate, setup, disable fd */
	    val = lp->a.read_bcr(ioaddr, 32) & ~0x98;
	    val |= 0x20;
	    lp->a.write_bcr(ioaddr, 32, val);
	}
    }

    if (lp->dxsuflo) { /* Disable transmit stop on underflow */
	val = lp->a.read_csr (ioaddr, 3);
	val |= 0x40;
	lp->a.write_csr (ioaddr, 3, val);
    }
    if (lp->ltint) { /* Enable TxDone-intr inhibitor */
	val = lp->a.read_csr (ioaddr, 5);
	val |= (1<<14);
	lp->a.write_csr (ioaddr, 5, val);
    }
   
    lp->init_block.mode = le16_to_cpu((lp->options & PCNET32_PORT_PORTSEL) << 7);
    lp->init_block.filter[0] = 0x00000000;
    lp->init_block.filter[1] = 0x00000000;
    if (sanremo_init_ring(dev))
    {
        printk(KERN_ERR "sanremo: Error init_ring().\n");
        return -ENOMEM;
    }
    
    /* Re-initialize the PCNET32, and start it when done. */
    lp->a.write_csr (ioaddr, 1, (lp->dma_addr + offsetof(struct sanremo_private, init_block)) &0xffff);
    lp->a.write_csr (ioaddr, 2, (lp->dma_addr + offsetof(struct sanremo_private, init_block)) >> 16);

    lp->a.write_csr (ioaddr, 4, 0x0915);
    lp->a.write_csr (ioaddr, 0, 0x0001);

    netif_start_queue(dev);

    i = 0;
    while (i++ < 100)
	if (lp->a.read_csr (ioaddr, 0) & 0x0100)
	    break;
    /* 
     * We used to clear the InitDone bit, 0x0100, here but Mark Stockton
     * reports that doing so triggers a bug in the '974.
     */
    lp->a.write_csr (ioaddr, 0, 0x0042);

    if (sanremo_debug > 2)
	printk(KERN_DEBUG "%s: pcnet32 open after %d ticks, init block %#x csr0 %4.4x.\n",
	       dev->name, i, (u32) (lp->dma_addr + offsetof(struct sanremo_private, init_block)),
	       lp->a.read_csr (ioaddr, 0));


    MOD_INC_USE_COUNT;
    
    return 0;	/* Always succeed */
}

/*
 * The LANCE has been halted for one reason or another (busmaster memory
 * arbitration error, Tx FIFO underflow, driver stopped it to reconfigure,
 * etc.).  Modern LANCE variants always reload their ring-buffer
 * configuration when restarted, so we must reinitialize our ring
 * context before restarting.  As part of this reinitialization,
 * find all packets still on the Tx ring and pretend that they had been
 * sent (in effect, drop the packets on the floor) - the higher-level
 * protocols will time out and retransmit.  It'd be better to shuffle
 * these skbs to a temp list and then actually re-Tx them after
 * restarting the chip, but I'm too lazy to do so right now.  dplatt@3do.com
 */

static void 
pcnet32_purge_tx_ring(struct net_device *dev)
{
    struct sanremo_private *lp = dev->priv;
    int i;

    for (i = 0; i < TX_RING_SIZE; i++) {
	if (lp->tx_skbuff[i]) {
            pci_unmap_single(lp->pci_dev, lp->tx_dma_addr[i], lp->tx_skbuff[i]->len, PCI_DMA_TODEVICE);
	    dev_kfree_skb_any(lp->tx_skbuff[i]); 
	    lp->tx_skbuff[i] = NULL;
            lp->tx_dma_addr[i] = 0;
	}
    }
}


/* Initialize the PCNET32 Rx and Tx rings. */
static int
sanremo_init_ring(struct net_device *dev)
{
    struct sanremo_private *lp = dev->priv;
    int i;

    lp->tx_full = 0;
    lp->cur_rx = lp->cur_tx = 0;
    lp->dirty_rx = lp->dirty_tx = 0;

    for (i = 0; i < RX_RING_SIZE; i++) {
        struct sk_buff *rx_skbuff = lp->rx_skbuff[i];
	if (rx_skbuff == NULL) {
	    if (!(rx_skbuff = lp->rx_skbuff[i] = dev_alloc_skb (PKT_BUF_SZ))) {
		/* there is not much, we can do at this point */
		printk(KERN_ERR "%s: sanremo_init_ring dev_alloc_skb failed.\n",dev->name);
		return -1;
	    }
	    skb_reserve (rx_skbuff, 2);
	}
        lp->rx_dma_addr[i] = pci_map_single(lp->pci_dev, rx_skbuff->tail, rx_skbuff->len, PCI_DMA_FROMDEVICE);
	lp->rx_ring[i].base = (u32)le32_to_cpu(lp->rx_dma_addr[i]);
	lp->rx_ring[i].buf_length = le16_to_cpu(-PKT_BUF_SZ);
	lp->rx_ring[i].status = le16_to_cpu(0x8000);
    }
    /* The Tx buffer address is filled in as needed, but we do need to clear
       the upper ownership bit. */
    for (i = 0; i < TX_RING_SIZE; i++) {
	lp->tx_ring[i].base = 0;
	lp->tx_ring[i].status = 0;
        lp->tx_dma_addr[i] = 0;
    }

    lp->init_block.tlen_rlen = le16_to_cpu(TX_RING_LEN_BITS | RX_RING_LEN_BITS);
    for (i = 0; i < 6; i++)
	lp->init_block.phys_addr[i] = dev->dev_addr[i];
    lp->init_block.rx_ring = (u32)le32_to_cpu(lp->dma_addr + offsetof(struct sanremo_private, rx_ring));
    lp->init_block.tx_ring = (u32)le32_to_cpu(lp->dma_addr + offsetof(struct sanremo_private, tx_ring));
    return 0;
}

/* the pcnet32 has been issued a stop or reset.  Wait for the stop bit
 * then flush the pending transmit operations, re-initialize the ring,
 * and tell the chip to initialize.
 */
static void
sanremo_restart(struct net_device *dev, unsigned int csr0_bits)
{
    struct sanremo_private *lp = dev->priv;
    unsigned long ioaddr = dev->base_addr;
    int i;
    
    pcnet32_purge_tx_ring(dev);
    if (sanremo_init_ring(dev))
	return;

    /* ReInit Ring */
    lp->a.write_csr (ioaddr, 0, 1);
    i = 0;
    while (i++ < 1000)
	if (lp->a.read_csr (ioaddr, 0) & 0x0100)
	    break;

    lp->a.write_csr (ioaddr, 0, csr0_bits);
}


static void
sanremo_tx_timeout (struct net_device *dev)
{
    struct sanremo_private *lp = dev->priv;
    unsigned long ioaddr = dev->base_addr;

    /* Transmitter timeout, serious problems. */
	printk(KERN_ERR "%s: transmit timed out, status %4.4x, resetting.\n",
		dev->name, lp->a.read_csr(ioaddr, 0));
    lp->a.write_csr (ioaddr, 0, 0x0004);
    lp->stats.tx_errors++;
    if (sanremo_debug > 2) {
	int i;
	printk(KERN_DEBUG " Ring data dump: dirty_tx %d cur_tx %d%s cur_rx %d.",
	   lp->dirty_tx, lp->cur_tx, lp->tx_full ? " (full)" : "",
	   lp->cur_rx);
	for (i = 0 ; i < RX_RING_SIZE; i++)
	printk("%s %08x %04x %08x %04x", i & 1 ? "" : "\n ",
		lp->rx_ring[i].base, -lp->rx_ring[i].buf_length,
		lp->rx_ring[i].msg_length, (unsigned)lp->rx_ring[i].status);

	for (i = 0 ; i < TX_RING_SIZE; i++)
	printk("%s %08x %04x %08x %04x", i & 1 ? "" : "\n ",
		   lp->tx_ring[i].base, -lp->tx_ring[i].length,
		   lp->tx_ring[i].misc, (unsigned)lp->tx_ring[i].status);
	printk("\n");
    }
    sanremo_restart(dev, 0x0042);

	dev->trans_start = jiffies;
	netif_start_queue(dev);
}


static int
sanremo_start_xmit(struct sk_buff *skb, struct net_device *dev)
{
    struct sanremo_private *lp = dev->priv;
    unsigned int ioaddr = dev->base_addr;
    u16 status;
    int entry;
    unsigned long flags;

    if (sanremo_debug > 3) {
	printk(KERN_DEBUG "%s: sanremo_start_xmit() called, csr0 %4.4x.\n",
	       dev->name, lp->a.read_csr (ioaddr, 0));
    }

    spin_lock_irqsave(&lp->lock, flags);

    /* Default status -- will not enable Successful-TxDone
     * interrupt when that option is available to us.
     */
    status = 0x8300;
    if ((lp->ltint) &&
	((lp->cur_tx - lp->dirty_tx == TX_RING_SIZE/2) ||
	 (lp->cur_tx - lp->dirty_tx >= TX_RING_SIZE-2)))
    {
	/* Enable Successful-TxDone interrupt if we have
	 * 1/2 of, or nearly all of, our ring buffer Tx'd
	 * but not yet cleaned up.  Thus, most of the time,
	 * we will not enable Successful-TxDone interrupts.
	 */
	status = 0x9300;
    }
  
    /* Fill in a Tx ring entry */
  
    /* Mask to ring buffer boundary. */
    entry = lp->cur_tx & TX_RING_MOD_MASK;

    /* Caution: the write order is important here, set the status
     * with the "ownership" bits last. */

    lp->tx_ring[entry].length = le16_to_cpu(-skb->len);

    lp->tx_ring[entry].misc = 0x00000000;

    lp->tx_skbuff[entry] = skb;
    lp->tx_dma_addr[entry] = pci_map_single(lp->pci_dev, skb->data, skb->len, PCI_DMA_TODEVICE);
    lp->tx_ring[entry].base = (u32)le32_to_cpu(lp->tx_dma_addr[entry]);
    lp->tx_ring[entry].status = le16_to_cpu(status);

    lp->cur_tx++;
    lp->stats.tx_bytes += skb->len;

    /* Trigger an immediate send poll. */
    lp->a.write_csr (ioaddr, 0, 0x0048);

    dev->trans_start = jiffies;

    if (lp->tx_ring[(entry+1) & TX_RING_MOD_MASK].base == 0)
	netif_start_queue(dev);
    else {
	lp->tx_full = 1;
	netif_stop_queue(dev);
    }
    spin_unlock_irqrestore(&lp->lock, flags);
    return 0;
}

/* The PCNET32 interrupt handler. */
static void
sanremo_interrupt(int irq, void *dev_id, struct pt_regs * regs)
{
    struct net_device *dev = dev_id;
    struct sanremo_private *lp;
    unsigned long ioaddr;
    u16 csr0,rap;
    int boguscnt =  max_interrupt_work;
    int must_restart;
    int io18, io02;

    if (dev == NULL) {
	printk (KERN_DEBUG "sanremo_interrupt(): irq %d for unknown device.\n", irq);
	return;
    }

    ioaddr = dev->base_addr;
    lp = dev->priv;
    
    spin_lock(&lp->lock);

    /* Read ASIC interrupt flags?
       ASIC is not documented. */
    io18 = inw(ioaddr + 0x18);
    io02 = inw(ioaddr + 0x02);

    rap = lp->a.read_rap(ioaddr);
    while ((csr0 = lp->a.read_csr (ioaddr, 0)) & 0x8600 && --boguscnt >= 0) {
	/* Acknowledge all of the current interrupt sources ASAP. */
	lp->a.write_csr (ioaddr, 0, csr0 & ~0x004f);

	must_restart = 0;

	if (sanremo_debug > 5)
	    printk(KERN_DEBUG "%s: interrupt  csr0=%#2.2x new csr=%#2.2x.\n",
		   dev->name, csr0, lp->a.read_csr (ioaddr, 0));

	if (csr0 & 0x0400)		/* Rx interrupt */
	    sanremo_rx(dev);

	if (csr0 & 0x0200) {		/* Tx-done interrupt */
	    unsigned int dirty_tx = lp->dirty_tx;

	    while (dirty_tx != lp->cur_tx) {
		int entry = dirty_tx & TX_RING_MOD_MASK;
		int status = (short)le16_to_cpu(lp->tx_ring[entry].status);
			
		if (status < 0)
		    break;		/* It still hasn't been Txed */

		lp->tx_ring[entry].base = 0;

		if (status & 0x4000) {
		    /* There was an major error, log it. */
		    int err_status = le32_to_cpu(lp->tx_ring[entry].misc);
		    lp->stats.tx_errors++;
		    if (err_status & 0x04000000) lp->stats.tx_aborted_errors++;
		    if (err_status & 0x08000000) lp->stats.tx_carrier_errors++;
		    if (err_status & 0x10000000) lp->stats.tx_window_errors++;
		    if (err_status & 0x40000000) {
			lp->stats.tx_fifo_errors++;
			if (! lp->dxsuflo) {  /* If controller doesn't recover ... */
			    /* Ackk!  On FIFO errors the Tx unit is turned off! */
			    /* Remove this verbosity later! */
			    printk(KERN_ERR "%s: Tx FIFO error! CSR0=%4.4x\n",
				   dev->name, csr0);
			    must_restart = 1;
			}
		    }
		} else {
		    if (status & 0x1800)
			lp->stats.collisions++;
		    lp->stats.tx_packets++;
		}

		/* We must free the original skb */
		if (lp->tx_skbuff[entry]) {
                    pci_unmap_single(lp->pci_dev, lp->tx_dma_addr[entry], lp->tx_skbuff[entry]->len, PCI_DMA_TODEVICE);
		    dev_kfree_skb_irq(lp->tx_skbuff[entry]);
		    lp->tx_skbuff[entry] = 0;
                    lp->tx_dma_addr[entry] = 0;
		}
		dirty_tx++;
	    }

#ifndef final_version
	    if (lp->cur_tx - dirty_tx >= TX_RING_SIZE) {
		printk(KERN_ERR "out-of-sync dirty pointer, %d vs. %d, full=%d.\n",
		       dirty_tx, lp->cur_tx, lp->tx_full);
		dirty_tx += TX_RING_SIZE;
	    }
#endif
	    if (lp->tx_full &&
		netif_queue_stopped(dev) &&
		dirty_tx > lp->cur_tx - TX_RING_SIZE + 2) {
		/* The ring is no longer full, clear tbusy. */
		lp->tx_full = 0;
		netif_wake_queue (dev);
	    }
	    lp->dirty_tx = dirty_tx;
	}

	/* Log misc errors. */
	if (csr0 & 0x4000) lp->stats.tx_errors++; /* Tx babble. */
	if (csr0 & 0x1000) {
	    /*
	     * this happens when our receive ring is full. This shouldn't
	     * be a problem as we will see normal rx interrupts for the frames
	     * in the receive ring. But there are some PCI chipsets (I can reproduce
	     * this on SP3G with Intel saturn chipset) which have sometimes problems
	     * and will fill up the receive ring with error descriptors. In this
	     * situation we don't get a rx interrupt, but a missed frame interrupt sooner
	     * or later. So we try to clean up our receive ring here.
	     */
	    sanremo_rx(dev);
	    lp->stats.rx_errors++; /* Missed a Rx frame. */
	}
	if (csr0 & 0x0800) {
	    printk(KERN_ERR "%s: Bus master arbitration failure, status %4.4x.\n",
		   dev->name, csr0);
	    /* unlike for the lance, there is no restart needed */
	}

	if (must_restart) {
	    /* stop the chip to clear the error condition, then restart */
	    lp->a.write_csr (ioaddr, 0, 0x0004);
	    sanremo_restart(dev, 0x0002);
	}
    }

    /* Clear any other interrupt, and set interrupt enable. */
    lp->a.write_csr (ioaddr, 0, 0x7940);
    lp->a.write_rap(ioaddr,rap);
    
    if (sanremo_debug > 4)
	printk(KERN_DEBUG "%s: exiting interrupt, csr0=%#4.4x.\n",
	       dev->name, lp->a.read_csr (ioaddr, 0));

    /* Acknowledge ASIC interrupt flags and re-prepare?
       ASIC is not documented. */
    outw(io18, ioaddr + 0x18);
    outw(io02, ioaddr + 0x02);
    outw(0x0FFF, ioaddr + 0x1A);

    spin_unlock(&lp->lock);
}

static int
sanremo_rx(struct net_device *dev)
{
    struct sanremo_private *lp = dev->priv;
    int entry = lp->cur_rx & RX_RING_MOD_MASK;

    /* If we own the next entry, it's a new packet. Send it up. */
    while ((short)le16_to_cpu(lp->rx_ring[entry].status) >= 0) {
	int status = (short)le16_to_cpu(lp->rx_ring[entry].status) >> 8;

	if (status != 0x03) {			/* There was an error. */
	    /* 
	     * There is a tricky error noted by John Murphy,
	     * <murf@perftech.com> to Russ Nelson: Even with full-sized
	     * buffers it's possible for a jabber packet to use two
	     * buffers, with only the last correctly noting the error.
	     */
	    if (status & 0x01)	/* Only count a general error at the */
		lp->stats.rx_errors++; /* end of a packet.*/
	    if (status & 0x20) lp->stats.rx_frame_errors++;
	    if (status & 0x10) lp->stats.rx_over_errors++;
	    if (status & 0x08) lp->stats.rx_crc_errors++;
	    if (status & 0x04) lp->stats.rx_fifo_errors++;
	    lp->rx_ring[entry].status &= le16_to_cpu(0x03ff);
	} else {
	    /* Malloc up new buffer, compatible with net-2e. */
	    short pkt_len = (le32_to_cpu(lp->rx_ring[entry].msg_length) & 0xfff)-4;
	    struct sk_buff *skb;
			
	    if(pkt_len < 60) {
		printk(KERN_ERR "%s: Runt packet!\n",dev->name);
		lp->stats.rx_errors++;
	    } else {
		int rx_in_place = 0;

		if (pkt_len > rx_copybreak) {
		    struct sk_buff *newskb;
				
		    if ((newskb = dev_alloc_skb (PKT_BUF_SZ))) {
			skb_reserve (newskb, 2);
			skb = lp->rx_skbuff[entry];
			skb_put (skb, pkt_len);
			lp->rx_skbuff[entry] = newskb;
			newskb->dev = dev;
                        lp->rx_dma_addr[entry] = pci_map_single(lp->pci_dev, newskb->tail, newskb->len, PCI_DMA_FROMDEVICE);
			lp->rx_ring[entry].base = le32_to_cpu(lp->rx_dma_addr[entry]);
			rx_in_place = 1;
		    } else
			skb = NULL;
		} else {
		    skb = dev_alloc_skb(pkt_len+2);
                }
			    
		if (skb == NULL) {
                    int i;
		    printk(KERN_ERR "%s: Memory squeeze, deferring packet.\n", dev->name);
		    for (i = 0; i < RX_RING_SIZE; i++)
			if ((short)le16_to_cpu(lp->rx_ring[(entry+i) & RX_RING_MOD_MASK].status) < 0)
			    break;

		    if (i > RX_RING_SIZE -2) {
			lp->stats.rx_dropped++;
			lp->rx_ring[entry].status |= le16_to_cpu(0x8000);
			lp->cur_rx++;
		    }
		    break;
		}
		skb->dev = dev;
		if (!rx_in_place) {
		    skb_reserve(skb,2); /* 16 byte align */
		    skb_put(skb,pkt_len);	/* Make room */
		    eth_copy_and_sum(skb,
				     (unsigned char *)(lp->rx_skbuff[entry]->tail),
				     pkt_len,0);
		}
		lp->stats.rx_bytes += skb->len;
		skb->protocol=eth_type_trans(skb,dev);
		netif_rx(skb);
		lp->stats.rx_packets++;
	    }
	}
	/*
	 * The docs say that the buffer length isn't touched, but Andrew Boyd
	 * of QNX reports that some revs of the 79C965 clear it.
	 */
	lp->rx_ring[entry].buf_length = le16_to_cpu(-PKT_BUF_SZ);
	lp->rx_ring[entry].status |= le16_to_cpu(0x8000);
	entry = (++lp->cur_rx) & RX_RING_MOD_MASK;
    }

    return 0;
}

static int
sanremo_close(struct net_device *dev)
{
    unsigned long ioaddr = dev->base_addr;
    struct sanremo_private *lp = dev->priv;
    int i;

    netif_stop_queue(dev);

    lp->stats.rx_missed_errors = lp->a.read_csr (ioaddr, 112);

    if (sanremo_debug > 1)
	printk(KERN_DEBUG "%s: Shutting down ethercard, status was %2.2x.\n",
	       dev->name, lp->a.read_csr (ioaddr, 0));

    /* We stop the PCNET32 here -- it occasionally polls memory if we don't. */
    lp->a.write_csr (ioaddr, 0, 0x0004);

    /*
     * Switch back to 16bit mode to avoid problems with dumb 
     * DOS packet driver after a warm reboot
     */
    lp->a.write_bcr (ioaddr, 20, 4);

    sanremo_disable_interrupt(ioaddr);

    free_irq(dev->irq, dev);

    /* free all allocated skbuffs */
    for (i = 0; i < RX_RING_SIZE; i++) {
	lp->rx_ring[i].status = 0;			    
	if (lp->rx_skbuff[i]) {
            pci_unmap_single(lp->pci_dev, lp->rx_dma_addr[i], lp->rx_skbuff[i]->len, PCI_DMA_FROMDEVICE);
	    dev_kfree_skb(lp->rx_skbuff[i]);
        }
	lp->rx_skbuff[i] = NULL;
        lp->rx_dma_addr[i] = 0;
    }
    
    for (i = 0; i < TX_RING_SIZE; i++) {
	if (lp->tx_skbuff[i]) {
            pci_unmap_single(lp->pci_dev, lp->tx_dma_addr[i], lp->tx_skbuff[i]->len, PCI_DMA_TODEVICE);
	    dev_kfree_skb(lp->tx_skbuff[i]);
        }
	lp->tx_skbuff[i] = NULL;
        lp->tx_dma_addr[i] = 0;
    }
    
    MOD_DEC_USE_COUNT;

    return 0;
}

static struct net_device_stats *
sanremo_get_stats(struct net_device *dev)
{
    struct sanremo_private *lp = dev->priv;
    unsigned long ioaddr = dev->base_addr;
    u16 saved_addr;
    unsigned long flags;

    spin_lock_irqsave(&lp->lock, flags);
    saved_addr = lp->a.read_rap(ioaddr);
    lp->stats.rx_missed_errors = lp->a.read_csr (ioaddr, 112);
    lp->a.write_rap(ioaddr, saved_addr);
    spin_unlock_irqrestore(&lp->lock, flags);

    return &lp->stats;
}

/* taken from the sunlance driver, which it took from the depca driver */
static void sanremo_load_multicast (struct net_device *dev)
{
    struct sanremo_private *lp = dev->priv;
    volatile struct pcnet32_init_block *ib = &lp->init_block;
    volatile u16 *mcast_table = (u16 *)&ib->filter;
    struct dev_mc_list *dmi=dev->mc_list;
    char *addrs;
    int i, j, bit, byte;
    u32 crc, poly = CRC_POLYNOMIAL_LE;
	
    /* set all multicast bits */
    if (dev->flags & IFF_ALLMULTI){ 
	ib->filter [0] = 0xffffffff;
	ib->filter [1] = 0xffffffff;
	return;
    }
    /* clear the multicast filter */
    ib->filter [0] = 0;
    ib->filter [1] = 0;

    /* Add addresses */
    for (i = 0; i < dev->mc_count; i++){
	addrs = dmi->dmi_addr;
	dmi   = dmi->next;
	
	/* multicast address? */
	if (!(*addrs & 1))
	    continue;
	
	crc = 0xffffffff;
	for (byte = 0; byte < 6; byte++)
	    for (bit = *addrs++, j = 0; j < 8; j++, bit >>= 1) {
		int test;
		
		test = ((bit ^ crc) & 0x01);
		crc >>= 1;
		
		if (test) {
		    crc = crc ^ poly;
		}
	    }
	
	crc = crc >> 26;
	mcast_table [crc >> 4] |= 1 << (crc & 0xf);
    }
    return;
}


/*
 * Set or clear the multicast filter for this adaptor.
 */
static void sanremo_set_multicast_list(struct net_device *dev)
{
    unsigned long ioaddr = dev->base_addr;
    struct sanremo_private *lp = dev->priv;

    if (dev->flags&IFF_PROMISC) {
	/* Log any net taps. */
	printk(KERN_INFO "%s: Promiscuous mode enabled.\n", dev->name);
	lp->init_block.mode = le16_to_cpu(0x8000 | (lp->options & PCNET32_PORT_PORTSEL) << 7);
    } else {
	lp->init_block.mode = le16_to_cpu((lp->options & PCNET32_PORT_PORTSEL) << 7);
	sanremo_load_multicast (dev);
    }
    
    lp->a.write_csr (ioaddr, 0, 0x0004); /* Temporarily stop the lance. */

    sanremo_restart(dev, 0x0042); /*  Resume normal operation */
}

/* This routine assumes that the lp->lock is held */
static int mdio_read(struct net_device *dev, int phy_id, int reg_num)
{
    struct sanremo_private *lp = dev->priv;
    unsigned long ioaddr = dev->base_addr;
    u16 val_out;
	int phyaddr;

	if (!lp->mii)
		return 0;
		
	phyaddr = lp->a.read_bcr(ioaddr, 33);

	lp->a.write_bcr(ioaddr, 33, ((phy_id & 0x1f) << 5) | (reg_num & 0x1f));
	val_out = lp->a.read_bcr(ioaddr, 34);
	lp->a.write_bcr(ioaddr, 33, phyaddr);
	
	return val_out;
}

/* This routine assumes that the lp->lock is held */
static void mdio_write(struct net_device *dev, int phy_id, int reg_num, int val)
{
    struct sanremo_private *lp = dev->priv;
    unsigned long ioaddr = dev->base_addr;
	int phyaddr;

	if (!lp->mii)
		return;
		
	phyaddr = lp->a.read_bcr(ioaddr, 33);

	lp->a.write_bcr(ioaddr, 33, ((phy_id & 0x1f) << 5) | (reg_num & 0x1f));
	lp->a.write_bcr(ioaddr, 34, val);
	lp->a.write_bcr(ioaddr, 33, phyaddr);
}

static int sanremo_ioctl(struct net_device *dev, struct ifreq *rq, int cmd)
{
    unsigned long ioaddr = dev->base_addr;
    struct sanremo_private *lp = dev->priv;	 
    struct mii_ioctl_data *data = (struct mii_ioctl_data *)&rq->ifr_data;
    int phyaddr = lp->a.read_bcr (ioaddr, 33);

    if (lp->mii) {
	switch(cmd) {
	case SIOCGMIIPHY:		/* Get address of MII PHY in use. */
	case SIOCDEVPRIVATE:		/* for binary compat, remove in 2.5 */
	    data->phy_id = (phyaddr >> 5) & 0x1f;
	    /* Fall Through */
	case SIOCGMIIREG:		/* Read MII PHY register. */
	case SIOCDEVPRIVATE+1:		/* for binary compat, remove in 2.5 */
	    lp->a.write_bcr (ioaddr, 33, ((data->phy_id & 0x1f) << 5) | (data->reg_num & 0x1f));
	    data->val_out = lp->a.read_bcr (ioaddr, 34);
	    lp->a.write_bcr (ioaddr, 33, phyaddr);
	    return 0;
	case SIOCSMIIREG:		/* Write MII PHY register. */
	case SIOCDEVPRIVATE+2:		/* for binary compat, remove in 2.5 */
	    if (!capable(CAP_NET_ADMIN))
		return -EPERM;
	    lp->a.write_bcr (ioaddr, 33, ((data->phy_id & 0x1f) << 5) | (data->reg_num & 0x1f));
	    lp->a.write_bcr (ioaddr, 34, data->val_in);
	    lp->a.write_bcr (ioaddr, 33, phyaddr);
	    return 0;
	default:
	    return -EOPNOTSUPP;
	}
    }
    return -EOPNOTSUPP;
}

MODULE_PARM(debug, "i");
MODULE_PARM(max_interrupt_work, "i");
MODULE_PARM(rx_copybreak, "i");
MODULE_PARM(tx_start_pt, "i");
MODULE_PARM(options, "1-" __MODULE_STRING(MAX_UNITS) "i");
MODULE_PARM(full_duplex, "1-" __MODULE_STRING(MAX_UNITS) "i");
MODULE_AUTHOR("Christian Holzapfel");
MODULE_DESCRIPTION("Driver for IBM 10/00 Mbps TX MCA Ethernet Adapter");
MODULE_LICENSE("GPL");

/* An additional parameter that may be passed in... */
static int debug = -1;
static int tx_start_pt = -1;

static int __init sanremo_init_module(void)
{
    if (debug > 0)
	sanremo_debug = debug;
    if ((tx_start_pt >= 0) && (tx_start_pt <= 3))
	tx_start = tx_start_pt;

	return sanremo_probe_mca();
}

static void __exit sanremo_cleanup_module(void)
{
    struct net_device *next_dev;

    /* No need to check MOD_IN_USE, as sys_delete_module() checks. */
    while (sanremo_dev) {
	struct sanremo_private *lp = sanremo_dev->priv;
	next_dev = lp->next;
	unregister_netdev(sanremo_dev);
	release_region(sanremo_dev->base_addr, SANREMO_TOTAL_SIZE);
    mca_mark_as_unused(lp->slot);
	pci_free_consistent(lp->pci_dev, sizeof(*lp), lp, lp->dma_addr);
	kfree(sanremo_dev);
	sanremo_dev = next_dev;
    }
}

module_init(sanremo_init_module);
module_exit(sanremo_cleanup_module);

/*
 * Local variables:
 *  c-indent-level: 4
 *  tab-width: 8
 * End:
 */
