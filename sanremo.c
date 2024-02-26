/* sanremo.c: An ethernet driver for the IBM 10/100 Mbps Ethernet (9-K)
              Micro Channel card, code name "San Remo", 
              with Adaptec ASIC-9060R as an MCA-PCI bridge
              and the AMD PCnet-FAST Am79C971 PCI Ethernet controller */
/*
 *      Copyright 2023 Christian Holzapfel, Ryan Alswede
 *
 *  Derived from the pcnet32 driver written 1996-1999 by Thomas Bogendoerfer.
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

static const char *version = "sanremo.c:v1.03 26.01.2024 christian@holzapfel.biz\n";

#include <linux/config.h>
#include <linux/module.h>
#ifdef MODVERSIONS
#include <linux/modversions.h>
#endif

#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/string.h>
#include <linux/ptrace.h>
#include <linux/errno.h>
#include <linux/ioport.h>
#include <linux/malloc.h>
#include <linux/interrupt.h>
#include <linux/mca.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <asm/bitops.h>
#include <asm/io.h>
#include <asm/dma.h>

#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>
#include <asm/spinlock.h>

static int sanremo_debug = 0;
static int tx_start = 1; /* Mapping -- 0:20, 1:64, 2:128, 3:~220 (depends on chip vers) */

#ifdef MODULE
static struct device *sanremo_dev = NULL;
#endif

static const int max_interrupt_work = 80;
static const int rx_copybreak = 200;

#define PORT_AUI      0x00
#define PORT_10BT     0x01
#define PORT_GPSI     0x02
#define PORT_MII      0x03

#define PORT_PORTSEL  0x03
#define PORT_ASEL     0x04
#define PORT_100      0x40
#define PORT_FD       0x80


/*
 * table to translate option values from tulip
 * to internal options
 */
static unsigned char options_mapping[] = {
    PORT_ASEL,              /*  0 Auto-select      */
    PORT_AUI,               /*  1 BNC/AUI          */
    PORT_AUI,               /*  2 AUI/BNC          */ 
    PORT_ASEL,              /*  3 not supported    */
    PORT_10BT | PORT_FD,    /*  4 10baseT-FD       */
    PORT_ASEL,              /*  5 not supported    */
    PORT_ASEL,              /*  6 not supported    */
    PORT_ASEL,              /*  7 not supported    */
    PORT_ASEL,              /*  8 not supported    */
    PORT_MII,               /*  9 MII 10baseT      */
    PORT_MII | PORT_FD,     /* 10 MII 10baseT-FD   */
    PORT_MII,               /* 11 MII (autosel)    */
    PORT_10BT,              /* 12 10BaseT      */
    PORT_MII | PORT_100,    /* 13 MII 100BaseTx    */
    PORT_MII | PORT_100 | PORT_FD, /* 14 MII 100BaseTx-FD */
    PORT_ASEL               /* 15 not supported    */
};

#define MAX_UNITS 8
static int options[MAX_UNITS] = {0, };
static int full_duplex[MAX_UNITS] = {0, };

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
 * v1.03:  ASIC initialization sequence updated
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

#define TX_RING_SIZE            (1 << (PCNET32_LOG_TX_BUFFERS))
#define TX_RING_MOD_MASK        (TX_RING_SIZE - 1)
#define TX_RING_LEN_BITS        ((PCNET32_LOG_TX_BUFFERS) << 12)

#define RX_RING_SIZE            (1 << (PCNET32_LOG_RX_BUFFERS))
#define RX_RING_MOD_MASK        (RX_RING_SIZE - 1)
#define RX_RING_LEN_BITS        ((PCNET32_LOG_RX_BUFFERS) << 4)

#define PKT_BUF_SZ        1544

/* Offsets from base I/O address. */
#define PCNET32_DWIO_RDP    0x10
#define PCNET32_DWIO_RAP    0x14
#define PCNET32_DWIO_RESET  0x18
#define PCNET32_DWIO_BDP    0x1C

#define PCNET32_TOTAL_SIZE  0x30

#define CRC_POLYNOMIAL_LE 0xedb88320UL  /* Ethernet CRC, little endian */

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

/* Setting DXSUFLO to 1 enables the Am79C971 controller to gracefully 
recover from an underflow error. The device will scan the transmit 
descriptor ring until it finds either the start of a new frame or a
TDTE it does not own. */
#define DO_DXSUFLO (1)

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
    u8  phys_addr[6];
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

struct sanremo_private {
    /* The Tx and Rx ring entries must be aligned on 16-byte boundaries in 32bit mode. */
    struct pcnet32_rx_head   rx_ring[RX_RING_SIZE];
    struct pcnet32_tx_head   tx_ring[TX_RING_SIZE];
    struct pcnet32_init_block    init_block;
    const char *name;
    /* The saved address of a sent-in-place packet/buffer, for skfree(). */
    struct sk_buff *tx_skbuff[TX_RING_SIZE];
    struct sk_buff *rx_skbuff[RX_RING_SIZE];
    struct pcnet32_access a;
    void *origmem;
    spinlock_t lock;                    /* Guard lock */
    unsigned int cur_rx, cur_tx;        /* The next free ring entry */
    unsigned int dirty_rx, dirty_tx;    /* The ring entries to be free()ed. */
    struct net_device_stats stats;
    char tx_full;
    int  options;
    int  shared_irq:1,                  /* shared irq possible */
    ltint:1,
#ifdef DO_DXSUFLO
     dxsuflo:1,                         /* disable transmit stop on uflo */
#endif
    full_duplex:1,                      /* full duplex possible */
    mii:1;                              /* mii port available */
#ifdef MODULE
    struct device *next;
#endif    
    int slot;
};

int         sanremo_probe(struct device *);
static int  sanremo_init_asic(unsigned long);
static int  sanremo_read_pci_config (unsigned long, int);
static void sanremo_write_pci_config (unsigned long, int, int);
static void sanremo_enable_interrupt (unsigned long);
static void sanremo_disable_interrupt (unsigned long);
static int  sanremo_probe1(struct device *, unsigned long, unsigned char, int, int, int);
static int  sanremo_open(struct device *);
static int  sanremo_init_ring(struct device *);
static int  sanremo_start_xmit(struct sk_buff *, struct device *);
static int  sanremo_rx(struct device *);
static void sanremo_interrupt(int, void *, struct pt_regs *);
static int  sanremo_close(struct device *);
static struct net_device_stats *sanremo_get_stats(struct device *);
static void sanremo_set_multicast_list(struct device *);
#ifdef HAVE_PRIVATE_IOCTL
static int  sanremo_mii_ioctl(struct device *, struct ifreq *, int);
#endif

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



int __init sanremo_probe (struct device *dev)
{
    unsigned long ioaddr = dev ? dev->base_addr: 0;
    unsigned int  irq_line = dev ? dev->irq : 0;
    unsigned int  dma = 0;
    int cards_found = 0;
    int slot = MCA_NOTFOUND;    
    
    if(ioaddr != 0)
        return ENXIO;
        
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
        
        if (sanremo_probe1(dev, ioaddr, irq_line, 1, 0, slot) == 0)    // MCA = Shared IRQ
        {                
            cards_found++;
        }
    }

    return cards_found ? 0: ENODEV;
}

/* sanremo_init_asic */
static int __init
sanremo_init_asic(unsigned long ioaddr)    
{
    int eepromInitDone;
    int i, temp;
    
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
    sanremo_write_pci_config(ioaddr, 0x0C, 0x0000FF00);                   // Write Latency and Header Type  
    sanremo_write_pci_config(ioaddr, 0x10, ASIC_IO_OFFSET + ioaddr);    // Write I/O Base Address
    sanremo_write_pci_config(ioaddr, 0x04, 0x00000145);                    // Write Control: SERREN, PERREN, BMEN, IOEN
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
static int sanremo_read_pci_config (unsigned long addr, int index)
{
    outb(ASIC_PCI_CONFIG_CMD, addr + ASIC_PCI_CONFIG_CMD_REGISTER);
    outl(index, addr + ASIC_IO_ADDRESS_REGISTER);
    return (inl(addr + ASIC_IO_DATA_REGISTER));
}

/* sanremo_write_pci_config */
static void sanremo_write_pci_config (unsigned long addr, int index, int val)
{
    outb(ASIC_PCI_CONFIG_CMD, addr + ASIC_PCI_CONFIG_CMD_REGISTER);
    outl(index, addr + ASIC_IO_ADDRESS_REGISTER);
    outl(val, addr + ASIC_IO_DATA_REGISTER);
}

/* sanremo_enable_interrupt */
static void sanremo_enable_interrupt (unsigned long addr)
{
    outb(1, addr + ASIC_IRQ_ENABLE_REGISTER);
}

/* sanremo_disable_interrupt */
static void sanremo_disable_interrupt (unsigned long addr)
{
    outb(0, addr + ASIC_IRQ_ENABLE_REGISTER);
}

/* sanremo_probe1 */
static int __init
sanremo_probe1(struct device *dev, unsigned long ioaddr, unsigned char irq_line, int shared, int card_idx, int slot)
{
    struct sanremo_private *lp;
    int i,fdx = 0, mii = 0;
#ifdef DO_DXSUFLO
    int dxsuflo = 0;
#endif
    int ltint = 0;
    int chip_version;
    char *chipname;
    char *priv;
    struct pcnet32_access *a;

    /* reset the chip */
    pcnet32_dwio_reset(ioaddr);

    if (pcnet32_dwio_read_csr (ioaddr, 0) == 4 && pcnet32_dwio_check(ioaddr)) 
    {
        a = &pcnet32_dwio;
    } 
    else
    {
        return ENODEV;
    }

    chip_version = a->read_csr (ioaddr, 88) | (a->read_csr (ioaddr,89) << 16);
    if (sanremo_debug > 2)
    printk("  PCnet chip version is %#x.\n", chip_version);
    if ((chip_version & 0xfff) != 0x003)
    {
        return ENODEV;
    }
    chip_version = (chip_version >> 12) & 0xffff;

    if (chip_version == 0x2623) 
    {
        chipname = "PCnet/FAST 79C971";
        /* To prevent Tx FIFO underflows ... (may increase Tx latency) */
        /* Set BCR18:NOUFLO to not start Tx until reach Tx start point */
        /* Looks like EEPROM sets BCR18:5/6 for BurstWrite/Read */
        a->write_bcr(ioaddr, 18, (a->read_bcr(ioaddr, 18) | 0x0800));
        /* Set CSR80:XMTSP, Tx start point = 20|64|128|248 bytes or size of frame */
        i = a->read_csr(ioaddr, 80) & ~0x0C00; /* Clear bits we are touching */
        a->write_csr(ioaddr, 80, i | (tx_start << 10));
        fdx = 1; mii = 1;
#ifdef DO_DXSUFLO
        dxsuflo = 1;
#endif
        /* Another mode, which is enabled by setting LTINTEN
        (CSR5, bit 14) to 1, allows suppression of interrupts for
        successful transmissions for all but the last frame in a
        sequence. */
        ltint = 1;
    } 
    else 
    {
        printk("sanremo: PCnet version %#x, no PCnet32 chip.\n",chip_version);
        return ENODEV;
    }
    
    /* Enable PCI/MCA busmaster Write and Read burst modes for throughput */
    a->write_bcr(ioaddr, 18, (a->read_bcr(ioaddr, 18) | (1 << 6)));
    a->write_bcr(ioaddr, 18, (a->read_bcr(ioaddr, 18) | (1 << 5)));
        
    dev = init_etherdev(dev, 0);

    printk(KERN_INFO "%s: %s at %#3lx,", dev->name, chipname, ioaddr);

    /* There is a 16 byte station address PROM at the base address.
     The first six bytes are the station address. */
    {
        int val;
        outl(ASIC_IO_OFFSET + ioaddr + 0x00, ioaddr + ASIC_IO_ADDRESS_REGISTER);
        
        /* Through the ASIC we're in word-only access mode, so we read 4 bytes at once */
        val = inl(ioaddr + ASIC_IO_DATA_REGISTER);
        dev->dev_addr[0] = val & 0xFF;
        dev->dev_addr[1] = (val >> 8) & 0xFF;
        dev->dev_addr[2] = (val >> 16) & 0xFF;
        dev->dev_addr[3] = (val >> 24) & 0xFF;
        printk(" %2.2x %2.2x %2.2x %2.2x", dev->dev_addr[0], 
                                           dev->dev_addr[1], 
                                           dev->dev_addr[2], 
                                           dev->dev_addr[3]); 
        
        outl(ASIC_IO_OFFSET + ioaddr + 0x04, ioaddr + ASIC_IO_ADDRESS_REGISTER); 
        val = inl(ioaddr + ASIC_IO_DATA_REGISTER);
        dev->dev_addr[4] = val & 0xFF;
        dev->dev_addr[5] = (val >> 8) & 0xFF;
        printk(" %2.2x %2.2x", dev->dev_addr[4], dev->dev_addr[5]); 
    }

    if (((chip_version + 1) & 0xfffe) == 0x2624) { /* Version 0x2623 or 0x2624 */
        i = a->read_csr(ioaddr, 80) & 0x0C00;  /* Check tx_start_pt */
        printk("\n    tx_start_pt(0x%04x):",i);
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
        printk("\n    SRAMSIZE=0x%04x,",i<<8);
        i = a->read_bcr(ioaddr, 26);
        printk(" SRAM_BND=0x%04x,",i<<8);
        i = a->read_bcr(ioaddr, 27);
        if (i & (1<<14)) printk("LowLatRx,");
    }

    dev->base_addr = ioaddr;
    request_region(ioaddr, PCNET32_TOTAL_SIZE, chipname);
    
    if ((priv = kmalloc(sizeof(*lp)+15,GFP_KERNEL)) == NULL)
        return ENOMEM;

    /*
     * Make certain the data structures used by
     * the PCnet32 are 16byte aligned
      */
    lp = (struct sanremo_private *)(((unsigned long)priv+15) & ~15);
      
    memset(lp, 0, sizeof(*lp));
    
    spin_lock_init(&lp->lock);
    
     mca_set_adapter_name(slot, SANREMO_ADAPTER_NAME);
     mca_mark_as_used(slot);    
    
    dev->priv = lp;
    lp->name = chipname;
    lp->shared_irq = shared;
    lp->slot = slot;
    lp->full_duplex = fdx;
#ifdef DO_DXSUFLO
    lp->dxsuflo = dxsuflo;
#endif
    lp->ltint = ltint;
    lp->mii = mii;
    if (options[card_idx] > sizeof (options_mapping))
    lp->options = PORT_ASEL;
    else
    lp->options = options_mapping[options[card_idx]];
    
    if (fdx && !(lp->options & PORT_ASEL) && full_duplex[card_idx])
    lp->options |= PORT_FD;
    
    lp->origmem = priv;
    lp->a = *a;
    
    /* detect special T1/E1 WAN card by checking for MAC address */
    if (dev->dev_addr[0] == 0x00 && dev->dev_addr[1] == 0xe0 && dev->dev_addr[2] == 0x75)
    lp->options = PORT_FD | PORT_GPSI;

    lp->init_block.mode = le16_to_cpu(0x0003);     /* Disable Rx and Tx. */
    lp->init_block.tlen_rlen = le16_to_cpu(TX_RING_LEN_BITS | RX_RING_LEN_BITS); 
    for (i = 0; i < 6; i++)
      lp->init_block.phys_addr[i] = dev->dev_addr[i];
    lp->init_block.filter[0] = 0x00000000;
    lp->init_block.filter[1] = 0x00000000;
    lp->init_block.rx_ring = (u32)le32_to_cpu(virt_to_bus(lp->rx_ring));
    lp->init_block.tx_ring = (u32)le32_to_cpu(virt_to_bus(lp->tx_ring));
    
    /* switch pcnet32 to 32bit mode */
    a->write_bcr (ioaddr, 20, 2);

    a->write_csr (ioaddr, 1, virt_to_bus(&lp->init_block) & 0xffff);
    a->write_csr (ioaddr, 2, virt_to_bus(&lp->init_block) >> 16);
     
    if (irq_line) 
    {
        dev->irq = irq_line;
    }    
    
    if (dev->irq >= 2)
        printk(" assigned IRQ %d.\n", dev->irq);
    else 
    {
        unsigned long irq_mask = probe_irq_on();    
        
        sanremo_enable_interrupt(ioaddr);
        
        /*
        * To auto-IRQ we enable the initialization-done and DMA error
        * interrupts. For ISA boards we get a DMA error, but VLB, PCI
        * and of course MCA boards will work.
        */
        /* Trigger an initialization just for the interrupt. */
        a->write_csr (ioaddr, 0, 0x41);
        mdelay (1);
        
        dev->irq = probe_irq_off (irq_mask);
        
        sanremo_disable_interrupt(ioaddr);
            
        if (dev->irq)
            printk(", probed IRQ %d.\n", dev->irq);
        else 
        {
            printk(", failed to detect IRQ line.\n");
            return ENODEV;
        }
    }    

    if (sanremo_debug > 0)
    printk(version);
    
    /* The PCNET32-specific entries in the device structure. */
    dev->open = &sanremo_open;
    dev->hard_start_xmit = &sanremo_start_xmit;
    dev->stop = &sanremo_close;
    dev->get_stats = &sanremo_get_stats;
    dev->set_multicast_list = &sanremo_set_multicast_list;
#ifdef HAVE_PRIVATE_IOCTL
    dev->do_ioctl = &sanremo_mii_ioctl;
#endif

    
#ifdef MODULE
    lp->next = sanremo_dev;
    sanremo_dev = dev;
#endif    

    /* Fill in the generic fields of the device structure. */
    ether_setup(dev);
    return 0;
}


static int
sanremo_open(struct device *dev)
{
    struct sanremo_private *lp = (struct sanremo_private *)dev->priv;
    unsigned long ioaddr = dev->base_addr;
    u16 val;
    int i;

    if (dev->irq == 0 ||
    request_irq(dev->irq, &sanremo_interrupt,
            lp->shared_irq ? SA_SHIRQ : 0, lp->name, (void *)dev)) {
    return -EAGAIN;
    }

    /* Reset the PCNET32 */
    lp->a.reset (ioaddr);
    
    sanremo_enable_interrupt(ioaddr);

    /* switch pcnet32 to 32bit mode */
    lp->a.write_bcr (ioaddr, 20, 2);

    if (sanremo_debug > 1)
    printk("%s: sanremo_open() irq %d tx/rx rings %#x/%#x init %#x.\n",
           dev->name, dev->irq,
           (u32) virt_to_bus(lp->tx_ring),
           (u32) virt_to_bus(lp->rx_ring),
           (u32) virt_to_bus(&lp->init_block));
    
    /* set/reset autoselect bit */
    val = lp->a.read_bcr (ioaddr, 2) & ~2;
    if (lp->options & PORT_ASEL)
    val |= 2;
    lp->a.write_bcr (ioaddr, 2, val);
    
    /* handle full duplex setting */
    if (lp->full_duplex) {
    val = lp->a.read_bcr (ioaddr, 9) & ~3;
    if (lp->options & PORT_FD) {
        val |= 1;
        if (lp->options == (PORT_FD | PORT_AUI))
        val |= 2;
    }
    lp->a.write_bcr (ioaddr, 9, val);
    }
    
    /* NOOP ??? set/reset GPSI bit in test register */
    val = lp->a.read_csr (ioaddr, 124) & ~0x10;
    if ((lp->options & PORT_PORTSEL) == PORT_GPSI)
    val |= 0x10;
    lp->a.write_csr (ioaddr, 124, val);
    
    if (lp->mii & !(lp->options & PORT_ASEL)) {
    val = lp->a.read_bcr (ioaddr, 32) & ~0x38; /* disable Auto Negotiation, set 10Mpbs, HD */
    if (lp->options & PORT_FD)
        val |= 0x10;
    if (lp->options & PORT_100)
        val |= 0x08;
    lp->a.write_bcr (ioaddr, 32, val);
    }

#ifdef DO_DXSUFLO 
    if (lp->dxsuflo) { /* Disable transmit stop on underflow */
        val = lp->a.read_csr (ioaddr, 3);
    val |= 0x40;
        lp->a.write_csr (ioaddr, 3, val);
    }
#endif
    if (lp->ltint) { /* Enable TxDone-intr inhibitor */
        val = lp->a.read_csr (ioaddr, 5);
    val |= (1<<14);
        lp->a.write_csr (ioaddr, 5, val);
    }
    
    /* Enable SINT System Error Interrupt */
    lp->a.write_csr (ioaddr, 5, lp->a.read_csr(ioaddr, 5) | (1 << 10));
    
    /* Enable EXDINT Excessive Deferral Interrupt */
    lp->a.write_csr (ioaddr, 5, lp->a.read_csr(ioaddr, 5) | (1 << 6));
    
    lp->init_block.mode = le16_to_cpu((lp->options & PORT_PORTSEL) << 7);
    lp->init_block.filter[0] = 0x00000000;
    lp->init_block.filter[1] = 0x00000000;
    if (sanremo_init_ring(dev))
    return -ENOMEM;
    
    /* Re-initialize the PCNET32, and start it when done. */
    lp->a.write_csr (ioaddr, 1, virt_to_bus(&lp->init_block) &0xffff);
    lp->a.write_csr (ioaddr, 2, virt_to_bus(&lp->init_block) >> 16);

    lp->a.write_csr (ioaddr, 4, 0x0915);
    lp->a.write_csr (ioaddr, 0, 0x0001);

    dev->tbusy = 0;
    dev->interrupt = 0;
    dev->start = 1;
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
    printk("%s: PCNET32 open after %d ticks, init block %#x csr0 %4.4x.\n",
           dev->name, i, (u32) virt_to_bus(&lp->init_block),
           lp->a.read_csr (ioaddr, 0));

    MOD_INC_USE_COUNT;
    
    return 0;    /* Always succeed */
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
sanremo_purge_tx_ring(struct device *dev)
{
    struct sanremo_private *lp = (struct sanremo_private *)dev->priv;
    int i;

    for (i = 0; i < TX_RING_SIZE; i++) {
        if (lp->tx_skbuff[i]) {
            dev_kfree_skb(lp->tx_skbuff[i]);
            lp->tx_skbuff[i] = NULL;
        }
    }
}


/* Initialize the PCNET32 Rx and Tx rings. */
static int
sanremo_init_ring(struct device *dev)
{
    struct sanremo_private *lp = (struct sanremo_private *)dev->priv;
    int i;

    lp->tx_full = 0;
    lp->cur_rx = lp->cur_tx = 0;
    lp->dirty_rx = lp->dirty_tx = 0;

    for (i = 0; i < RX_RING_SIZE; i++) {
        if (lp->rx_skbuff[i] == NULL) {
            if (!(lp->rx_skbuff[i] = dev_alloc_skb (PKT_BUF_SZ))) {
                /* there is not much, we can do at this point */
                printk ("%s: sanremo_init_ring dev_alloc_skb failed.\n",dev->name);
                return -1;
            }
            skb_reserve (lp->rx_skbuff[i], 2);
        }
        lp->rx_ring[i].base = (u32)le32_to_cpu(virt_to_bus(lp->rx_skbuff[i]->tail));
        lp->rx_ring[i].buf_length = le16_to_cpu(-PKT_BUF_SZ);
        lp->rx_ring[i].status = le16_to_cpu(0x8000);
    }
    /* The Tx buffer address is filled in as needed, but we do need to clear
     the upper ownership bit. */
    for (i = 0; i < TX_RING_SIZE; i++) {
        lp->tx_ring[i].base = 0;
        lp->tx_ring[i].status = 0;
    }

    lp->init_block.tlen_rlen = le16_to_cpu(TX_RING_LEN_BITS | RX_RING_LEN_BITS); 
    for (i = 0; i < 6; i++)
        lp->init_block.phys_addr[i] = dev->dev_addr[i];
    lp->init_block.rx_ring = (u32)le32_to_cpu(virt_to_bus(lp->rx_ring));
    lp->init_block.tx_ring = (u32)le32_to_cpu(virt_to_bus(lp->tx_ring));
    return 0;
}

static void
sanremo_restart(struct device *dev, unsigned int csr0_bits)
{
    struct sanremo_private *lp = (struct sanremo_private *)dev->priv;
    unsigned long ioaddr = dev->base_addr;
    int i;
    
    sanremo_purge_tx_ring(dev);
    if (sanremo_init_ring(dev))
    return;
    
    /* ReInit Ring */
    lp->a.write_csr (ioaddr, 0, 1);
    i = 0;
    while (i++ < 100)
    if (lp->a.read_csr (ioaddr, 0) & 0x0100)
        break;

    lp->a.write_csr (ioaddr, 0, csr0_bits);
}

static int
sanremo_start_xmit(struct sk_buff *skb, struct device *dev)
{
    struct sanremo_private *lp = (struct sanremo_private *)dev->priv;
    unsigned int ioaddr = dev->base_addr;
    u16 status;
    int entry;
    unsigned long flags;

    /* Transmitter timeout, serious problems. */
    if (dev->tbusy) {
    int tickssofar = jiffies - dev->trans_start;
    if (tickssofar < HZ/2)
    {
        return 1;
    }
    printk("%s: transmit timed out, status %4.4x, resetting.\n",
           dev->name, lp->a.read_csr (ioaddr, 0));
    lp->a.write_csr (ioaddr, 0, 0x0004);
    lp->stats.tx_errors++;
    if (sanremo_debug > 2) {
        int i;
        printk(" Ring data dump: dirty_tx %d cur_tx %d%s cur_rx %d.",
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

    dev->tbusy = 0;
    dev->trans_start = jiffies;
    dev_kfree_skb(skb);
    return 0;
    }

    if (sanremo_debug > 3) {
        printk("%s: sanremo_start_xmit() called, csr0 %4.4x.\n",
               dev->name, lp->a.read_csr (ioaddr, 0));
    }

    /* Block a timer-based transmit from overlapping.  This could better be
       done with atomic_swap(1, dev->tbusy), but set_bit() works as well. */
    if (test_and_set_bit(0, (void*)&dev->tbusy) != 0) {
        printk("%s: Transmitter access conflict.\n", dev->name);
        return 1;
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

    /* Caution: the write order is important here, set the base address
       with the "ownership" bits last. */

    lp->tx_ring[entry].length = le16_to_cpu(-skb->len);

    lp->tx_ring[entry].misc = 0x00000000;

    lp->tx_skbuff[entry] = skb;
    lp->tx_ring[entry].base = (u32)le32_to_cpu(virt_to_bus(skb->data));

    lp->tx_ring[entry].status = le16_to_cpu(status);

    lp->cur_tx++;
    lp->stats.tx_bytes += skb->len;

    /* Trigger an immediate send poll. */
    lp->a.write_csr (ioaddr, 0, 0x0048);

    dev->trans_start = jiffies;

    if (lp->tx_ring[(entry+1) & TX_RING_MOD_MASK].base == 0)
    clear_bit (0, (void *)&dev->tbusy);
    else
    lp->tx_full = 1;
    spin_unlock_irqrestore(&lp->lock, flags);
    return 0;
}

/* The PCNET32 interrupt handler. */
static void
sanremo_interrupt(int irq, void *dev_id, struct pt_regs * regs)
{
    struct device *dev = (struct device *)dev_id;
    struct sanremo_private *lp;
    unsigned long ioaddr;
    u16 csr0,csr5,rap;
    int boguscnt =  max_interrupt_work;
    int must_restart;
    int io18, io02;

    if (dev == NULL) {
    printk ("sanremo_interrupt(): irq %d for unknown device.\n", irq);
    return;
    }

    ioaddr = dev->base_addr;
    lp = (struct sanremo_private *)dev->priv;

    spin_lock(&lp->lock);

    if (dev->interrupt)
    printk("%s: Re-entering the interrupt handler.\n", dev->name);

    dev->interrupt = 1;
    
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
        printk("%s: interrupt  csr0=%#2.2x new csr=%#2.2x.\n",
           dev->name, csr0, lp->a.read_csr (ioaddr, 0));

    if (csr0 & 0x0400)        /* Rx interrupt */
    {
        sanremo_rx(dev);
    }

    if (csr0 & 0x0200) {        /* Tx-done interrupt */
        unsigned int dirty_tx = lp->dirty_tx;

        while (dirty_tx < lp->cur_tx) {
            int entry = dirty_tx & TX_RING_MOD_MASK;
            int status = (short)le16_to_cpu(lp->tx_ring[entry].status);
                
            if (status < 0)
                break;        /* It still hasn't been Txed */
    
            lp->tx_ring[entry].base = 0;
    
            if (status & 0x4000) {
                /* There was an major error, log it. */
                int err_status = le32_to_cpu(lp->tx_ring[entry].misc);
                lp->stats.tx_errors++;
                if (err_status & 0x04000000) lp->stats.tx_aborted_errors++;
                if (err_status & 0x08000000) lp->stats.tx_carrier_errors++;
                if (err_status & 0x10000000) lp->stats.tx_window_errors++;
#ifndef DO_DXSUFLO
                if (err_status & 0x40000000) {
                    lp->stats.tx_fifo_errors++;
                    /* Ackk!  On FIFO errors the Tx unit is turned off! */
                    /* Remove this verbosity later! */
                    printk("%s: Tx FIFO error! CSR0=%4.4x\n", dev->name, csr0);
                    must_restart = 1;
                }
#else    
                if (err_status & 0x40000000) {
                    printk("%s: Tx error! CSR0=%4.4x\n", dev->name, csr0);
                    lp->stats.tx_fifo_errors++;
                    if (! lp->dxsuflo) {  /* If controller doesn't recover ... */
                        /* Ackk!  On FIFO errors the Tx unit is turned off! */
                        /* Remove this verbosity later! */
                        printk("%s: Tx FIFO error! CSR0=%4.4x\n",
                                dev->name, csr0);
                        must_restart = 1;
                    }
                }
#endif    
            } else {
                if (status & 0x1800)
                lp->stats.collisions++;
                lp->stats.tx_packets++;
            }
    
            /* We must free the original skb */
            if (lp->tx_skbuff[entry]) {
                dev_kfree_skb(lp->tx_skbuff[entry]);
                lp->tx_skbuff[entry] = 0;
            }
            dirty_tx++;
        }

#ifndef final_version
        if (lp->cur_tx - dirty_tx >= TX_RING_SIZE) {
            printk("out-of-sync dirty pointer, %d vs. %d, full=%d.\n",
               dirty_tx, lp->cur_tx, lp->tx_full);
            dirty_tx += TX_RING_SIZE;
        }
#endif
        if (lp->tx_full && dev->tbusy
        && dirty_tx > lp->cur_tx - TX_RING_SIZE + 2) {
            /* The ring is no longer full, clear tbusy. */
            lp->tx_full = 0;
            clear_bit(0, (void *)&dev->tbusy);
            mark_bh(NET_BH);
        }
        lp->dirty_tx = dirty_tx;
    }
    
    /* SINT System Error Interrupt */
    csr5 = lp->a.read_csr (ioaddr, 5);
    if (csr5 & (1 << 11))
    {
        lp->a.write_csr (ioaddr, 5, csr5 | (1 << 11));    /* Acknowledge */
        printk("%s: Systen Interrupt error!\n", dev->name);
    }
    
    /* EXDINT Excessive Deferral Interrupt */
    if (csr5 & (1 << 7))
    {
        lp->a.write_csr (ioaddr, 5, csr5 | (1 << 7));    /* Acknowledge */
        printk("%s: Excessive Deferral Interrupt!\n", dev->name);
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
        printk("%s: Bus master arbitration failure, status %4.4x.\n",
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
        printk("%s: exiting interrupt, csr0=%#4.4x.\n",
           dev->name, lp->a.read_csr (ioaddr, 0));


    /* Acknowledge ASIC interrupt flags and re-prepare?
       ASIC is not documented. */
    outw(io18, ioaddr + 0x18);
    outw(io02, ioaddr + 0x02);
    outw(0x0FFF, ioaddr + 0x1A);
    
    dev->interrupt = 0;

    spin_unlock(&lp->lock);
    return;
}

static int
sanremo_rx(struct device *dev)
{
    struct sanremo_private *lp = (struct sanremo_private *)dev->priv;
    int entry = lp->cur_rx & RX_RING_MOD_MASK;
    int i;

    /* If we own the next entry, it's a new packet. Send it up. */
    while ((short)le16_to_cpu(lp->rx_ring[entry].status) >= 0) {
    int status = (short)le16_to_cpu(lp->rx_ring[entry].status) >> 8;

    if (status != 0x03) {            /* There was an error. */
        /* 
         * There is a tricky error noted by John Murphy,
         * <murf@perftech.com> to Russ Nelson: Even with full-sized
         * buffers it's possible for a jabber packet to use two
         * buffers, with only the last correctly noting the error.
         */
        if (status & 0x01)    /* Only count a general error at the */
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
        printk("%s: Runt packet!\n",dev->name);
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
            lp->rx_ring[entry].base = le32_to_cpu(virt_to_bus(newskb->tail));
            rx_in_place = 1;
            } else
            skb = NULL;
        } else
            skb = dev_alloc_skb(pkt_len+2);
                
        if (skb == NULL) {
            printk("%s: Memory squeeze, deferring packet.\n", dev->name);
            for (i=0; i < RX_RING_SIZE; i++)
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
            skb_reserve(skb,2);    /* 16 byte align */
            skb_put(skb,pkt_len);    /* Make room */
            eth_copy_and_sum(skb,
                     (unsigned char *)bus_to_virt(le32_to_cpu(lp->rx_ring[entry].base)),
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
sanremo_close(struct device *dev)
{
    unsigned long ioaddr = dev->base_addr;
    struct sanremo_private *lp = (struct sanremo_private *)dev->priv;
    int i;

    dev->start = 0;
    set_bit (0, (void *)&dev->tbusy);

    lp->stats.rx_missed_errors = lp->a.read_csr (ioaddr, 112);

    if (sanremo_debug > 1)
    printk("%s: Shutting down ethercard, status was %2.2x.\n",
           dev->name, lp->a.read_csr (ioaddr, 0));

    /* We stop the PCNET32 here -- it occasionally polls memory if we don't. */
    lp->a.write_csr (ioaddr, 0, 0x0004);

    sanremo_disable_interrupt(ioaddr);

    free_irq(dev->irq, dev);
    
    /* free all allocated skbuffs */
    for (i = 0; i < RX_RING_SIZE; i++) {
    lp->rx_ring[i].status = 0;                        
    if (lp->rx_skbuff[i])
        dev_kfree_skb(lp->rx_skbuff[i]);
    lp->rx_skbuff[i] = NULL;
    }
    
    for (i = 0; i < TX_RING_SIZE; i++) {
    if (lp->tx_skbuff[i])
        dev_kfree_skb(lp->tx_skbuff[i]);
    lp->rx_skbuff[i] = NULL;
    }
       
    MOD_DEC_USE_COUNT;    

    return 0;
}

static struct net_device_stats *
sanremo_get_stats(struct device *dev)
{
    struct sanremo_private *lp = (struct sanremo_private *)dev->priv;
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
static void pcnet32_load_multicast (struct device *dev)
{
    struct sanremo_private *lp = (struct sanremo_private *) dev->priv;
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
static void sanremo_set_multicast_list(struct device *dev)
{
    unsigned long ioaddr = dev->base_addr;
    struct sanremo_private *lp = (struct sanremo_private *)dev->priv;    

    if (dev->flags&IFF_PROMISC) {
    /* Log any net taps. */
    printk("%s: Promiscuous mode enabled.\n", dev->name);
    lp->init_block.mode = le16_to_cpu(0x8000 | (lp->options & PORT_PORTSEL) << 7);
    } else {
    lp->init_block.mode = le16_to_cpu((lp->options & PORT_PORTSEL) << 7);
    pcnet32_load_multicast (dev);
    }
    
    lp->a.write_csr (ioaddr, 0, 0x0004); /* Temporarily stop the lance. */

    sanremo_restart(dev, 0x0042); /*  Resume normal operation */
}

#ifdef HAVE_PRIVATE_IOCTL
static int sanremo_mii_ioctl(struct device *dev, struct ifreq *rq, int cmd)
{
    unsigned long ioaddr = dev->base_addr;
    struct sanremo_private *lp = (struct sanremo_private *)dev->priv;    
    u16 *data = (u16 *)&rq->ifr_data;
    int phyaddr = lp->a.read_bcr (ioaddr, 33);

    if (lp->mii) {
    switch(cmd) {
     case SIOCDEVPRIVATE:            /* Get the address of the PHY in use. */
        data[0] = (phyaddr >> 5) & 0x1f;
        /* Fall Through */
     case SIOCDEVPRIVATE+1:          /* Read the specified MII register. */
        lp->a.write_bcr (ioaddr, 33, ((data[0] & 0x1f) << 5) | (data[1] & 0x1f));
        data[3] = lp->a.read_bcr (ioaddr, 34);
        lp->a.write_bcr (ioaddr, 33, phyaddr);
        return 0;
     case SIOCDEVPRIVATE+2:          /* Write the specified MII register */
        if (!suser())
        return -EPERM;
        lp->a.write_bcr (ioaddr, 33, ((data[0] & 0x1f) << 5) | (data[1] & 0x1f));
        lp->a.write_bcr (ioaddr, 34, data[2]);
        lp->a.write_bcr (ioaddr, 33, phyaddr);
        return 0;
     default:
        return -EOPNOTSUPP;
    }
    }
    return -EOPNOTSUPP;
}
#endif  /* HAVE_PRIVATE_IOCTL */
                        
#ifdef MODULE
MODULE_PARM(debug, "i");
MODULE_PARM(max_interrupt_work, "i");
MODULE_PARM(rx_copybreak, "i");
MODULE_PARM(tx_start_pt, "i");
MODULE_PARM(options, "1-" __MODULE_STRING(MAX_UNITS) "i");
MODULE_PARM(full_duplex, "1-" __MODULE_STRING(MAX_UNITS) "i");
                         

/* An additional parameter that may be passed in... */
static int debug = -1;
static int tx_start_pt = -1;

int
init_module(void)
{
    if (debug > 0)
    sanremo_debug = debug;
    if ((tx_start_pt >= 0) && (tx_start_pt <= 3))
    tx_start = tx_start_pt;
    
    sanremo_dev = NULL;
    return sanremo_probe(NULL);
}

void
cleanup_module(void)
{
    struct device *next_dev;

    /* No need to check MOD_IN_USE, as sys_delete_module() checks. */
    while (sanremo_dev) {
    struct sanremo_private *lp = (struct sanremo_private *)sanremo_dev->priv;
    next_dev = ((struct sanremo_private *) sanremo_dev->priv)->next;
    unregister_netdev(sanremo_dev);
    release_region(sanremo_dev->base_addr, PCNET32_TOTAL_SIZE);
    mca_mark_as_unused(lp->slot);
    kfree(((struct sanremo_private *)sanremo_dev->priv)->origmem);
    kfree(sanremo_dev);
    sanremo_dev = next_dev;
    }
}
#endif /* MODULE */



/*
 * Local variables:
 *  compile-command: "gcc -D__KERNEL__ -I/usr/src/linux/net/inet -Wall -Wstrict-prototypes -O6 -m486 -c sanremo.c"
 *  c-indent-level: 4
 *  tab-width: 4
 * End:
 */
