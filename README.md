# sanremo-linux

A Linux ethernet driver module (Kernel 2.2.17) for the IBM 10/100 Mbps Ethernet (9-K)
Micro Channel card, code name "San Remo", with Adaptec ASIC-9060R as an MCA-PCI bridge
and the AMD PCnet-FAST Am79C971 PCI Ethernet controller.

The difference between a regular PCnet-FAST card and the San Remo is the onboard ASIC,
which is the only component that can be directly addressed from within the driver,
and that needs to be passed in order to configure the PCnet chip.

Since the PCnet is a PCI busmastering chipset, the ASIC is needed in order to transform
bus accesses through the card's MCA interface to the PCnet. It acts as a MCA-to-PCI bridge.

## Driver compilation

If you are running a vintage 2.2.17 Linux kernel on your favorite Micro Channel machine,
you can copy the sanremo.c to /usr/src/kernel-src/drivers/net, copy the sanremo.patch to
/usr/src/kernel-src, navigate there and

`patch -p0 < sanremo.patch`

Then 

`make menuconfig`

and select the new driver from the network section.
MCA bus support must be enabled.

Then rebuild and install your custom kernel like you always do.

## The card's resources

Through Reference Disk configuration and the ADF, you can choose from the following resources:

I/O 0x1C00..1FFF / 0x2000..23FF / 0x2400..27FF / 0x2800..2BFF

IRQ 10 / 11 / 12 / 15

DMA arb level 3 / 4 / 5 / 6 / 7 
    
## The ASIC's I/O window registers

The ASIC opens up a space of 0x400 bytes in the system's I/O port range, of which only
0x30 bytes are actually used, while the rest is repetitions of the initial 0x30 wide area.
I call this the "ASIC I/O space registers", because the ASIC places some internal registers
here. The following ASIC registers were identified:

```				
Address        Size Access   Use           Value
======================================================================================
iobase + 0x00  2    WO       Init          0x0006          Written to on Init only
iobase + 0x02  2    RW       ISR           0x0000          ASIC interrupt flags
iobase + 0x04  1    WO       I/O Tunnel    0x01            Daughter card PCI config space access register
iobase + 0x05  1    -        -             -               <unused>
iobase + 0x06  2    -        -             -               <unused>
iobase + 0x08  4    RW       I/O Tunnel                    Daughter card I/O address register
iobase + 0x0C  4    RW       I/O Tunnel                    Daughter card I/O data register
iobase + 0x10  4    WO       Init          0x00000000      Written to on Init only
iobase + 0x14  4    WO       Init          0x00000000      Written to on Init only
iobase + 0x18  2    RW       ISR           0x8008          ASIC interrupt flags
iobase + 0x1A  2    WO       Init/ISR      0x0FFF          Written to on Init (0x0FFF) and every ISR exit
iobase + 0x1C  1    WO       Interrupt                     Daughter card IRQ enable register
iobase + 0x1D  1    WO       Init          0x00            Written to on Init only
iobase + 0x1E  1    WO       Init          0x0F            Written to on Init only
iobase + 0x1F  1    WO       Init          0x04            Written to on Init only
iobase + 0x20  2    WO       Init          0x03FF          Written to on Init only
iobase + 0x22  1    WO       Init          0x3F            Written to on Init only
iobase + 0x23  1    -        -             -               <unused>
iobase + 0x24  1    WO       VPD           0x00..0xFF      VPD index register
iobase + 0x25  1    RO       VPD           0x01            VPD state/valid Register?
iobase + 0x26  2    RO       VPD           Var             VPD data register
iobase + 0x28  4    WO       Init          0x00000000      Written to on Init only
iobase + 0x2C  4    -	     -             -               <unused>
```

## Initializing the ASIC

After setting up the POS registers by using the ADF, the card needs a few accesses through the
I/O window to initialize.
The purpose of those accesses is still unclear, but replaying the noted values seems enough to
set the card in operation:

```
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
```
	
I will not go into details of all the initialization sequnces, as you can find them inside the
source code, hopefully commented sufficiently.

## Tunneling I/O to the PCnet

Generally, the PCnet is able to work in two different addressing modes: WIO (16-bit) and DWIO (32-bit).
On the San Remo card, it is forced to work in "DWIO addressing mode" only, and this seems to be the only
mode the ASIC supports tunneling.
Usually the PCnet starts up in WIO mode and is shifted up to DWIO by the driver; in this case, the PCnet
initializes itself from the onboard EEPROM which contains initialization data that sets it into DWIO
mode immediately.
If you want to talk to the PCnet from your driver, you can't just read and write to the card's I/O space
directly, because you have to get past the ASIC.
On the PCI side of the ASIC (the PCnet side), the PCnet chip has an I/O window too, at 0x1000 offset.
So if your card's I/O base address sits at 0x1C00, the *internal* PCnet I/O base is 0x2C00, and so on.
Tunneling works like this:

1) You have to write to the ASIC's Daughter card I/O address register at iobase + 0x08 the address of
   the PCnet you would like to access, e.g. the RAP register at offset 0x14 inside the PCnet's range.
   -> Write 0x2C14 to 0x1C08
2) Then write to or read from the ASIC's Daughter card I/O data register at iobase + 0x0C:
   -> Read/write from/to 0x1C0C
   
So for each usually single port access to the PCnet, in our case we have to do two, to get through the ASIC.

## Accessing the PCI Configuration Space

If you issue writing a '1' to the Daughter card PCI config space access register at iobase + 0x04,
the PCnet's PCI Config Space is available for 32-bit wide access through the standard I/O tunnel registers
iobase + 0x08 and iobase + 0x0C for exactly one immediately following access.
The PCI Config Space can be read and written this way.

## Interrupt handling

On the San Remo card, we have a PCI bus between the ASIC and the PCnet, and an MCA bus between our CPU
and the ASIC. When the PCnet raises an interrupt, the ASIC picks it up, sets a bit in one of its registers
and raises an interrupt on the MCA bus for the CPU to pick it up.

The ASIC's interrupts are enabled by setting the first bit in iobase + 0x1C.
Clearing this bit inhibits MCA interrupts, no matter what's pending on the PCI side.

So in order to clear a pending interrupt, the driver's ISR would have to 

1) Read the ASIC's interrupt flags at iobase + 0x18 and iobase + 0x02
2) Service the PCnet's needs
3) Acknowledge the PCnet's pending interrupt flags
4) Acknowledge the ASIC's pending interrupt flags by writing the read
   values back to iobase + 0x18, iobase + 0x02 and 0x0FFF to 0x1A.
   
## DMA transfers

Usually, the PCnet is a supreme busmaster on it's PCI bus.
The ASIC gracefully picks up busmastering requests from the PCnet, and translates them into 
Micro Channel DMA requests using the PREEMPT#, ARB/GNT# lines, along with DMA slave arbitration.
All DMA handling is invisible to the user or driver.
Reserving a DMA arb level in the Reference Disk configuration is just for allocating the resource
for the ASIC, and avoiding conflicts with other installed cards.
   
## Accessing the VPD area

In the AIX / RS/6000 world, each adapter must provide a storage area where the so called 
VPD = Vital Product Data can be stored and altered.
This is supposed to hold information about the card, it's revisions, and optional space
about customer information about where the card is installed into, among other things.
This data is stored inside a 256 byte EEPROM type 24C02 next to the ASIC.
It may not prove important in PS/2 systems, but accessing that storage space works like this:

1) Write the byte index (0..255) to the VPD index register at iobase + 0x24
2) Read the stored one or two bytes at a time from the VPD data register at iobase + 0x26.
3) Writing to that VPD data register does not work.
4) The VPD state/valid register at iobase + 0x25 probably holds a "valid" flag for the checksum
   or an error/success flag for the EEPROM read operation.

## Performance measurement

I used netio v1.11 to benchmark the adapter against a random Core2Duo system running Ubuntu 22.04.
Adapter in full-duplex, 100 mbit mode.

Measurements were taken on an IBM PC 750 PCI/MCA machine with Debian "Potato" 2.2 and various CPUs.
With the stock Pentium 133 CPU, the card achieved about 3.200 kb/sec.
With an AMD K6-III 400 CPU, the card achieved over 7.500 kb/sec.
With a newer 2.4 kernel, the AMD CPU achieved around 9.300 kb/sec.

An IBM 8595 with Type 4 Y-Complex, a 66 MHz crystal and Linux 2.2 does around 5.000 kb/sec.
With a Pentium Overdrive at 200 MHz you can expect around 8.600 kb/sec.

The driver hits CPU saturation, but the results are still impressive for an MCA system.
This may change with future revisions of the driver.	

## Credits

Thanks to Ryan Alswede for having the vision of reverse enginering the card for so long, 
poking me at it over and over again, creating the ADF, asking all the right questions and
all the day long rubber ducking.
