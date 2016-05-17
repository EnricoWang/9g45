/* This program is free software; you can redistribut it and/or
 * modify it under the term of version 2 of the GNU General Public 
 * license as published by the free software foundation.
 * This program is based on plx_pci.c and 8250_pci.c.
 * Author: Xi'an Hangpu Electrical lt. co
 * date: 2016-05-09
 * */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/netdevice.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/pci.h>
#include <linux/can/dev.h>
#include <linux/io.h>
#include <linux/tty.h>
#include <linux/serial_core.h>
#include <linux/8250_pci.h>
#include <linux/bitops.h>
#include <asm/byteorder.h>
#include <asm/io.h>
#include "/home/enrico/prj/Linux_Kernel/linux-kernel-2.6.32-ls3a2h/drivers/serial/8250.h"
#include "/home/enrico/prj/Linux_Kernel/linux-kernel-2.6.32-ls3a2h/drivers/net/can/sja1000/sja1000.h"

#define DRV_NAME  "multi_board_of_pci"

#define PLX_PCI_MAX_CHAN   4           // double can channels
#define PCI_NUM_BAR_RESOURCES   6      // This defination is for 16c554, define bar number is 6                                                                                                                                                                
struct plx_pci_card {
	int channels;
	struct net_device *net_dev[PLX_PCI_MAX_CHAN];
	void __iomem *conf_addr;
	void (*reset_func)(struct pci_dev *pdev);
};

#define PLX_PCI_CAN_CLOCK (16000000 / 2)
/* PLX9030/9050/9052 registers */
#define PLX_INTCSR	0x4c		/* Interrupt Control/Status */
#define PLX_CNTRL	0x50		/* User I/O, Direct Slave Response,
					             * Serial EEPROM, and Initialization
					             * Control register
					             */
#define PLX_LINT1_EN	0x1		/* Local interrupt 1 enable */
#define PLX_LINT2_EN	(1 << 3)	/* Local interrupt 2 enable */
#define PLX_PCI_INT_EN	(1 << 6)	/* PCI Interrupt Enable */
#define PLX_PCI_RESET	(1 << 30)	/* PCI Adapter Software Reset */
#define PLX_PCI_OCR	(OCR_TX0_PUSHPULL | OCR_TX1_PUSHPULL)
#define PLX_PCI_CDR			(CDR_CBP | CDR_CLKOUT_MASK)
/* SJA1000 Control Register in the BasicCAN Mode */
#define REG_CR				0x00

/* States of some SJA1000 registers after hardware reset in the BasicCAN mode*/
#define REG_CR_BASICCAN_INITIAL		0x21
#define REG_CR_BASICCAN_INITIAL_MASK	0xa1
#define REG_SR_BASICCAN_INITIAL		0x0c
#define REG_IR_BASICCAN_INITIAL		0xe0

/* States of some SJA1000 registers after hardware reset in the PeliCAN mode*/
#define REG_MOD_PELICAN_INITIAL		0x01
#define REG_SR_PELICAN_INITIAL		0x3c
#define REG_IR_PELICAN_INITIAL		0x00

#define HP_PCI_VENDOR_ID		0x4850
#define HP_PCI_DEVICE_ID	    0x3341

static void plx_pci_reset_common(struct pci_dev *pdev);

struct plx_pci_channel_map {
	u32 bar;
	u32 offset;
	u32 size;		/* 0x00 - auto, e.g. length of entire bar */
};

struct plx_pci_card_info {
	const char *name;
	int channel_count;
	u32 can_clock;
	u8 ocr;			/* output control register */
	u8 cdr;			/* clock divider register */

	/* Parameters for mapping local configuration space */
	struct plx_pci_channel_map conf_map;

	/* Parameters for mapping the SJA1000 chips */
	struct plx_pci_channel_map chan_map_tbl[PLX_PCI_MAX_CHAN];

	/* Pointer to device-dependent reset function */
	void (*reset_func)(struct pci_dev *pdev);
};

// register 4 CAN device, start at bar0 0x400, and offset is 0x100
static struct plx_pci_card_info plx_pci_card_info_hangpu __devinitdata = {
	"hangpu_pci_can", 4,
	PLX_PCI_CAN_CLOCK, PLX_PCI_OCR, PLX_PCI_CDR,
	{0, 0x100, 0x00}, { {0, 0x400, 0x100}, {0, 0x500, 0x100},{0, 0x600, 0x100},{0, 0x700, 0x100}},
	&plx_pci_reset_common
};

// get sja1000 register status
static u8 plx_pci_read_reg(const struct sja1000_priv *priv, int port)
{
	return ioread8(priv->reg_base + port);
}

static void plx_pci_write_reg(const struct sja1000_priv *priv, int port, u8 val)
{
	iowrite8(val, priv->reg_base + port);
}

/*
 * Check if a CAN controller is present at the specified location
 * by trying to switch 'em from the Basic mode into the PeliCAN mode.
 * Also check states of some registers in reset mode.
 */
static inline int plx_pci_check_sja1000(const struct sja1000_priv *priv)
{
	int flag = 0;

	/*
	 * Check registers after hardware reset (the Basic mode)
	 * See states on p. 10 of the Datasheet.
	 */
	if ((priv->read_reg(priv, REG_CR) & REG_CR_BASICCAN_INITIAL_MASK) ==
			REG_CR_BASICCAN_INITIAL &&
			(priv->read_reg(priv, REG_SR) == REG_SR_BASICCAN_INITIAL) &&
			(priv->read_reg(priv, REG_IR) == REG_IR_BASICCAN_INITIAL))
		flag = 1;

	/* Bring the SJA1000 into the PeliCAN mode*/
	priv->write_reg(priv, REG_CDR, CDR_PELICAN);

	/*
	 * Check registers after reset in the PeliCAN mode.
	 * See states on p. 23 of the Datasheet.
	 */
	if (priv->read_reg(priv, REG_MOD) == REG_MOD_PELICAN_INITIAL &&
			priv->read_reg(priv, REG_SR) == REG_SR_PELICAN_INITIAL &&
			priv->read_reg(priv, REG_IR) == REG_IR_PELICAN_INITIAL)
		return flag;

	return 0;
}

/*
 * PLX9030/50/52 software reset
 * Also LRESET# asserts and brings to reset device on the Local Bus (if wired).
 * For most cards it's enough for reset the SJA1000 chips.
 */
static void plx_pci_reset_common(struct pci_dev *pdev)
{
	struct plx_pci_card *card = pci_get_drvdata(pdev);
	u32 cntrl;

	cntrl = ioread32(card->conf_addr + PLX_CNTRL);
	cntrl |= PLX_PCI_RESET;
	iowrite32(cntrl, card->conf_addr + PLX_CNTRL);
	udelay(100);
	cntrl ^= PLX_PCI_RESET;
	iowrite32(cntrl, card->conf_addr + PLX_CNTRL);
};

// define a structure to find serial device
struct pci_serial_quirk {                                                                                                                                                                               
	u32 vendor;                 
	u32 device;
	u32 subvendor;
	u32 subdevice;
	int (*init)(struct pci_dev *dev);
	int (*setup)(struct serial_private *,
			const struct pciserial_board *,
			struct uart_port *, int);
	void    (*exit)(struct pci_dev *dev);
}; 

// define serial private
struct serial_private {
	struct pci_dev      *dev;
	unsigned int        nr;
	void __iomem        *remapped_bar[PCI_NUM_BAR_RESOURCES];
	struct pci_serial_quirk *quirk;
	int         line[0];
}; 

// When executing broken call this function
static void moan_device(const char *str, struct pci_dev *dev)
{
	printk(KERN_WARNING
			"%s: %s\n"
			"Please send the output of lspci -vv, this\n"
			"message (0x%04x,0x%04x,0x%04x,0x%04x), the\n"
			"manufacturer and name of serial board or\n"
			"modem board to rmk+serial@arm.linux.org.uk.\n",
			pci_name(dev), str, dev->vendor, dev->device,
			dev->subsystem_vendor, dev->subsystem_device);
}

	static int
setup_port(struct serial_private *priv, struct uart_port *port,
		int bar, int offset, int regshift)
{
	struct pci_dev *dev = priv->dev;
	unsigned long base, len;
	unsigned int flag;
    printk(KERN_DEBUG"setup_port bar = %d\n", bar);

	flag = pci_resource_flags(dev, 0);

	if (bar >= PCI_NUM_BAR_RESOURCES)
		return -EINVAL;

	base = pci_resource_start(dev, bar);

	if (pci_resource_flags(dev, bar) & IORESOURCE_MEM) {
		len =  0x100;//pci_resource_len(dev, bar);

		if (!priv->remapped_bar[bar])
			priv->remapped_bar[bar] = ioremap_nocache(base, len);
		if (!priv->remapped_bar[bar])
			return -ENOMEM;

		port->iotype = UPIO_MEM;
		port->iobase = 0;
		port->mapbase = base + offset;
		port->membase = priv->remapped_bar[bar] + offset;
		port->regshift = regshift;
	} else {
		port->iotype = UPIO_PORT;
		port->iobase = base + offset;
		port->mapbase = 0;
		port->membase = NULL;
		port->regshift = 0;
	}
	return 0;
}

static int pci_plx9050_init(struct pci_dev *dev)
{
	u8 irq_config;
	void __iomem *p;

	if ((pci_resource_flags(dev, 0) & IORESOURCE_MEM) == 0) {
		moan_device("no memory in bar 0", dev);
		return 0;
	}
	irq_config = 0x41;
	if (dev->vendor == PCI_VENDOR_ID_PANACOM ||
			dev->subsystem_vendor == PCI_SUBVENDOR_ID_EXSYS)
		irq_config = 0x43;

	if ((dev->vendor == HP_PCI_VENDOR_ID) &&
			(dev->device == HP_PCI_DEVICE_ID))
		/*
		 * As the megawolf cards have the int pins active
		 * high, and have 2 UART chips, both ints must be
		 * enabled on the 9050. Also, the UARTS are set in
		 * 16450 mode by default, so we have to enable the
		 * 16C950 'enhanced' mode so that we can use the
		 * deep FIFOs
		 */
		irq_config = 0x43;
	/*
	 * enable/disable interrupts
	 */
	p = ioremap_nocache(pci_resource_start(dev, 0), 0x80);
	if (p == NULL)
		return -ENOMEM;
	writel(0x41, p + 0x4c);
	/*
	 * Read the register back to ensure that it took effect.
	 */
	readl(p + 0x4c);
	iounmap(p);

	return 0;
}

static void __devexit pci_plx9050_exit(struct pci_dev *dev)
{
	u8 __iomem *p;

	if ((pci_resource_flags(dev, 0) & IORESOURCE_MEM) == 0) {
		printk(KERN_DEBUG"pci_plx9050_exit\n");
		return;
	}
	/*
	 * disable interrupts
	 */
	p = ioremap_nocache(pci_resource_start(dev, 0), 0x80);
	if (p != NULL) {
		writel(0, p + 0x4c);

		/*
		 * Read the register back to ensure that it took effect.
		 */
		readl(p + 0x4c);
		iounmap(p);
	}
}

	static int
pci_default_setup(struct serial_private *priv,
		const struct pciserial_board *board,
		struct uart_port *port, int idx)
{
	unsigned int bar, offset = board->first_offset, maxnr;

	bar = FL_GET_BASE(board->flags);
    printk(KERN_ALERT"pci_default_setup bar = %d\n", bar);

	if (board->flags & FL_BASE_BARS)
		bar += idx;
	else
		offset += idx * board->uart_offset;

	maxnr = (pci_resource_len(priv->dev, bar) - board->first_offset) >>
		(board->reg_shift + 3);

	if (board->flags & FL_REGION_SZ_CAP && idx >= maxnr)
		return 1;

	return setup_port(priv, port, bar, offset, board->reg_shift);
}

static struct pci_serial_quirk pci_serial_quirks[] __refdata = {
	/*
	 * PLX
	 */
	{
		.vendor		= HP_PCI_VENDOR_ID,
		.device		= HP_PCI_DEVICE_ID,
		.subvendor	= PCI_ANY_ID,
		.subdevice	= PCI_ANY_ID,
		.init		= pci_plx9050_init,
		.setup		= pci_default_setup,
		.exit		= __devexit_p(pci_plx9050_exit),
	},
};

static inline int quirk_id_matches(u32 quirk_id, u32 dev_id)
{
	return quirk_id == PCI_ANY_ID || quirk_id == dev_id;
}

static struct pci_serial_quirk *find_quirk(struct pci_dev *dev)
{
	struct pci_serial_quirk *quirk;

	quirk = pci_serial_quirks;
	if (quirk_id_matches(quirk->vendor, dev->vendor) &&
			quirk_id_matches(quirk->device, dev->device) &&
			quirk_id_matches(quirk->subvendor, dev->subsystem_vendor) &&
			quirk_id_matches(quirk->subdevice, dev->subsystem_device))
		return quirk;
	return quirk;
}

static inline int get_pci_irq(struct pci_dev *dev, 
		const struct pciserial_board *board) 
{                  
	if (board->flags & FL_NOIRQ) 
		return 0;  
	else           
		return dev->irq; 
}  

enum pci_board_num_t {
	pbn_plx_romulus,
};

static struct pciserial_board pci_boards[] __devinitdata = {
	/* I think this entry is broken - the first_offset looks wrong --rmk */
	[pbn_plx_romulus] = {
		.flags		= FL_BASE0,
		.num_ports	= 8,
		.base_baud	= 921600,
		.uart_offset	= 0x10,
		.reg_shift	= 0,
		.first_offset	= 0,
	},
};

// mark: private data is not same, unknown how to handle
static struct pci_device_id plx_pci_tbl[] = {
	// add by hangpu
	{   HP_PCI_VENDOR_ID, HP_PCI_DEVICE_ID,
		PCI_ANY_ID, PCI_ANY_ID, 0, 0,
		(kernel_ulong_t)&plx_pci_card_info_hangpu
	},
	{   HP_PCI_VENDOR_ID, HP_PCI_DEVICE_ID,
		PCI_ANY_ID, PCI_ANY_ID, 0, 0,
		pbn_plx_romulus 
	},
	{ 0,}
};
MODULE_DEVICE_TABLE(pci, plx_pci_tbl);

	struct serial_private *
pciserial_init_ports(struct pci_dev *dev, const struct pciserial_board *board)
{
	struct uart_port serial_port;
	struct serial_private *priv;
	struct pci_serial_quirk *quirk;
	int rc, nr_ports, i;

	nr_ports = board->num_ports;

	/*
	 * Find an init and setup quirks.
	 */
	quirk = find_quirk(dev);

#if 0
	/*
	 * Run the new-style initialization function.
	 * The initialization function returns:
	 *  <0  - error
	 *   0  - use board->num_ports
	 *  >0  - number of ports
	 */
	if (quirk->init) {
		rc = quirk->init(dev);
		if (rc < 0) {
			priv = ERR_PTR(rc);
			goto err_out;
		}
		if (rc)
			nr_ports = rc;
	}
#endif

	priv = kzalloc(sizeof(struct serial_private) +
			sizeof(unsigned int) * nr_ports,
			GFP_KERNEL);
	if (!priv) {
		printk(KERN_ALERT"kzalloc fialed!\n");
		priv = ERR_PTR(-ENOMEM);
		goto err_deinit;
	}

	priv->dev = dev;
	priv->quirk = quirk;

	memset(&serial_port, 0, sizeof(struct uart_port));
	serial_port.flags = UPF_SKIP_TEST | UPF_BOOT_AUTOCONF | UPF_SHARE_IRQ;
	serial_port.uartclk = board->base_baud * 16;
	serial_port.irq = get_pci_irq(dev, board);
	serial_port.dev = &dev->dev;

	printk(KERN_ALERT"hello! i'm serial, i'm ready to register!");
	for (i = 0; i < nr_ports; i++) {
		if (quirk->setup(priv, board, &serial_port, i)) {
			printk(KERN_ALERT"byebye! said by serial!!");
			break;
		}
		/*
#ifdef SERIAL_DEBUG_PCI
printk(KERN_DEBUG "Setup PCI port: port %lx, irq %d, type %d\n",
serial_port.iobase, serial_port.irq, serial_port.iotype);
#endif
		 */
		priv->line[i] = serial8250_register_port(&serial_port);
		printk(KERN_ALERT"%d\n", i);
		if (priv->line[i] < 0) {
			printk(KERN_WARNING "Couldn't register serial port %s: %d\n", pci_name(dev), priv->line[i]);
			break;
		}
	}
	priv->nr = i;
	return priv;

err_deinit:
	if (quirk->exit)
		quirk->exit(dev);
err_out:
	return priv;
}

static void plx_pci_del_card(struct pci_dev *pdev)
{
	struct plx_pci_card *card = pci_get_drvdata(pdev);
	struct net_device *dev;
	struct sja1000_priv *priv;
	int i = 0;

	struct serial_private *priv_uart = pci_get_drvdata(pdev);

	for (i = 0; i < card->channels; i++) {
		dev = card->net_dev[i];
		if (!dev)
			continue;

		dev_info(&pdev->dev, "Removing %s\n", dev->name);
		unregister_sja1000dev(dev);
		priv = netdev_priv(dev);
		if (priv->reg_base)
			pci_iounmap(pdev, priv->reg_base);
		free_sja1000dev(dev);
	}

	card->reset_func(pdev);

	pci_set_drvdata(pdev, NULL);

	pciserial_remove_ports(priv_uart);

	/*
	 * Disable interrupts from PCI-card and disable local
	 * interrupts
	 */

	if (card->conf_addr)
		pci_iounmap(pdev, card->conf_addr);
	kfree(card);
	pci_disable_device(pdev);
	pci_set_drvdata(pdev, NULL);
}

/*
 * Probe PLX90xx based device for the SJA1000 chips and register each
 * available CAN channel to SJA1000 Socket-CAN subsystem.
 */
static int __devinit plx_pci_add_card(struct pci_dev *pdev,
		const struct pci_device_id *ent)
{
	struct sja1000_priv *priv;
	struct net_device *dev;
	struct plx_pci_card *card;
	struct plx_pci_card_info *ci;
	int err, i;
	u32 val;
	void __iomem *addr;

	struct serial_private *priv_uart;
	const struct pciserial_board *board;
	struct pciserial_board tmp;
	int rc;

	ci = (struct plx_pci_card_info *)ent->driver_data;

	if (pci_enable_device(pdev) < 0) {
		dev_err(&pdev->dev, "Failed to enable PCI device\n");
		return -ENODEV;
	}

	dev_info(&pdev->dev, "Detected \"%s\" card at slot #%i\n",
			ci->name, PCI_SLOT(pdev->devfn));

	/* Allocate card structures to hold addresses, ... */
	card = kzalloc(sizeof(*card), GFP_KERNEL);
	if (!card) {
		dev_err(&pdev->dev, "Unable to allocate memory\n");
		pci_disable_device(pdev);
		return -ENOMEM;
	}

	pci_set_drvdata(pdev, card);
	card->channels = 0;
	//end if
	
#if 0
	/* Remap PLX90xx configuration space */
	addr = pci_iomap(pdev, ci->conf_map.bar, ci->conf_map.size);
	if (!addr) {
		err = -ENOMEM;
		dev_err(&pdev->dev, "Failed to remap configuration space "
				"(BAR%d)\n", ci->conf_map.bar);
		goto failure_cleanup;
	}
	card->conf_addr = addr + ci->conf_map.offset;

	ci->reset_func(pdev);
	card->reset_func = ci->reset_func;
#endif

	/* Detect available channels */
	for (i = 0; i < ci->channel_count; i++) {
		struct plx_pci_channel_map *cm = &ci->chan_map_tbl[i];

		dev = alloc_sja1000dev(0);
		if (!dev) {
			err = -ENOMEM;
			goto failure_cleanup;
		}

		card->net_dev[i] = dev;
		priv = netdev_priv(dev);
		priv->priv = card;
		priv->irq_flags = IRQF_SHARED;

		dev->irq = pdev->irq;

		/*
		 * Remap IO space of the SJA1000 chips
		 * This is device-dependent mapping
		 */
		addr = pci_iomap(pdev, cm->bar, cm->size);
		if (!addr) {
			err = -ENOMEM;
			dev_err(&pdev->dev, "Failed to remap BAR%d\n", cm->bar);
			goto failure_cleanup;
		}

		priv->reg_base = addr + cm->offset;
		priv->read_reg = plx_pci_read_reg;
		priv->write_reg = plx_pci_write_reg;

		/* Check if channel is present */
		if (plx_pci_check_sja1000(priv)) {
			priv->can.clock.freq = ci->can_clock;
			priv->ocr = ci->ocr;
			priv->cdr = ci->cdr;

			SET_NETDEV_DEV(dev, &pdev->dev);

			/* Register SJA1000 device */
			err = register_sja1000dev(dev);
			if (err) {
				dev_err(&pdev->dev, "Registering device failed "
						"(err=%d)\n", err);
				free_sja1000dev(dev);
				goto failure_cleanup;
			}

			card->channels++;

			dev_info(&pdev->dev, "Channel #%d at 0x%p, irq %d "
					"registered as %s\n", i + 1, priv->reg_base,
					dev->irq, dev->name);
		} else {
			dev_err(&pdev->dev, "Channel #%d not detected\n",
					i + 1);
			free_sja1000dev(dev);
		}
	}

	if (!card->channels) {
		err = -ENODEV;
		goto failure_cleanup;
	}

#if 0
	/*
	 * Enable interrupts from PCI-card (PLX90xx) and enable Local_1,
	 * Local_2 interrupts from the SJA1000 chips
	 */
	if (pdev->device != PCI_DEVICE_ID_PLX_9056) {
		val = ioread32(card->conf_addr + PLX_INTCSR);
		if (pdev->subsystem_vendor == PCI_VENDOR_ID_ESDGMBH)
			val |= PLX_LINT1_EN | PLX_PCI_INT_EN;
		else
			val |= PLX_LINT1_EN | PLX_LINT2_EN | PLX_PCI_INT_EN;
		iowrite32(val, card->conf_addr + PLX_INTCSR);
	} 
#endif

	board = &pci_boards[pbn_plx_romulus];
	memcpy(&tmp, &pci_boards[pbn_plx_romulus], sizeof(struct pciserial_board));
	board = &tmp;
	priv_uart = pciserial_init_ports(pdev, board);
	if (!IS_ERR(priv_uart)) {
		pci_set_drvdata(pdev, priv_uart);
		return 0;
	}

	rc = PTR_ERR(priv_uart);

	return 0;

failure_cleanup:
	printk(KERN_DEBUG"I'm sorry to meet you! motherfuck!!\n");
	dev_err(&pdev->dev, "Error: %d. Cleaning Up.\n", err);

	plx_pci_del_card(pdev);

	return err;
}

static struct pci_driver plx_pci_driver = {
	.name = DRV_NAME,
	.id_table = plx_pci_tbl,
	.probe = plx_pci_add_card,
	.remove = plx_pci_del_card,
};

static int __init plx_pci_init(void)
{
	return pci_register_driver(&plx_pci_driver);
}

static void __exit plx_pci_exit(void)
{
	pci_unregister_driver(&plx_pci_driver);
}

module_init(plx_pci_init);
module_exit(plx_pci_exit);

MODULE_AUTHOR("HANGPU ELECTRICAL");
MODULE_DESCRIPTION("driver for PLX9052 PCI-bridge cards with sja1000 and 16c554a");
MODULE_LICENSE("GPL v2");
