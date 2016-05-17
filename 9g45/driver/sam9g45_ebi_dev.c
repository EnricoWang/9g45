/* ***************************************************************************************************************************
 **************   ____________   _______   _________  _________ __       __    _________ ************************************
 **************  /____  _____/ / _____  / / ______ / / _______/ \ \     / /   / _______/ ************************************ 
 **************      / /      / /    / / / /     // / /          \ \   / /   / /         ************************************
 **************     / /      / /    / / / /_____// / /______      \_\ /_/   / /______    ************************************
 **************    / /      / /    / / /  ______/ /______  /        | |    / _____  /    ************************************
 **************   / /      / /____/ / / /        _______/ /         | |  ________/ /     ************************************       
 **************  /_/      /________/ /_/        /________/          |_| /_________/      ************************************
 ************************************************************************************************************************** */          
#include <linux/types.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/atmel-mci.h>
#include <linux/delay.h>
#include <linux/smsc911x.h>

#include <mach/hardware.h>
#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/irq.h>
#include <asm/gpio.h>
#include <mach/at91_pmc.h>
#include <mach/at91_aic.h>

#include <asm/mach/arch.h>
#include <asm/mach/irq.h>

#include <mach/board.h>
#include <mach/at91sam9_smc.h>
#include <mach/at91_shdwc.h>
#include <mach/system_rev.h>

/* new add */
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <mach/cpu.h>

#include "/home/enrico/topsys/src/kernel/linux-3.6.9/arch/arm/mach-at91/soc.h"
#include "/home/enrico/topsys/src/kernel/linux-3.6.9/arch/arm/mach-at91/sam9_smc.h"
#include "/home/enrico/topsys/src/kernel/linux-3.6.9/arch/arm/mach-at91/generic.h"

#define AT91_SMC_CS(id, n)  (smc_base_addr[id] + ((n) * 0x10))

static void __iomem *smc_base_addr[2];

static struct resource	sam9g45_resource[] = {
	[0] = 	{ 
			.start = 0x10000000 ,                //EBI0  base address   is 0x10000000
			.end   = 0x10000000 + 64*1024 -1,
			.flags = IORESOURCE_MEM, 
		},
	[1] =   {
			.start = 167,//gpio_to_irq(AT91_PIN_PD23),
			.end   = 167,//gpio_to_irq(AT91_PIN_PD23),
			.flags = IORESOURCE_IRQ,
		},
};

static struct smsc911x_platform_config  sam9g45_pdata = {
	.irq_polarity = SMSC911X_IRQ_POLARITY_ACTIVE_LOW,
	.irq_type = SMSC911X_IRQ_TYPE_OPEN_DRAIN,	
	.flags	  = SMSC911X_USE_16BIT | SMSC911X_FORCE_INTERNAL_PHY,
	.phy_interface	 = PHY_INTERFACE_MODE_MII,
};

static struct platform_device  sam9g45_device = {
	.name		= "sam9g45",
	.id		= 0,
	.dev 		= {
				.platform_data = &sam9g45_pdata,

			  }, 	
	.resource 	= sam9g45_resource,
	.num_resources	= ARRAY_SIZE(sam9g45_resource),
};

static struct sam9_smc_config sam9g45_smc_config = {
	.ncs_read_setup		= 4,
	.nrd_setup		= 4,
	.ncs_write_setup	= 4,
	.nwe_setup		= 4,

	.ncs_read_pulse		= 16,
	.nrd_pulse		= 16,
	.ncs_write_pulse	= 16,
	.nwe_pulse		= 16,

	.read_cycle		= 64,
	.write_cycle		= 64,

	.mode			= AT91_SMC_READMODE | AT91_SMC_WRITEMODE | AT91_SMC_EXNWMODE_DISABLE | AT91_SMC_DBW_16,
	.tdf_cycles		= 12,
};

static void sam9_smc_cs_write_mode(void __iomem *base, struct sam9_smc_config *config)
{
	__raw_writel(config->mode | AT91_SMC_TDF_(config->tdf_cycles),
		base + AT91_SMC_MODE);
} 

#if 1
static void sam9_smc_cs_configure(void __iomem *base, struct sam9_smc_config *config)
{
	/* Setup register */
	__raw_writel(AT91_SMC_NWESETUP_(config->nwe_setup)
		| AT91_SMC_NCS_WRSETUP_(config->ncs_write_setup)
		| AT91_SMC_NRDSETUP_(config->nrd_setup)
		| AT91_SMC_NCS_RDSETUP_(config->ncs_read_setup),
		base + AT91_SMC_SETUP);
	/* Pulse register */
	__raw_writel(AT91_SMC_NWEPULSE_(config->nwe_pulse)
		| AT91_SMC_NCS_WRPULSE_(config->ncs_write_pulse)
		| AT91_SMC_NRDPULSE_(config->nrd_pulse)
		| AT91_SMC_NCS_RDPULSE_(config->ncs_read_pulse),
		base + AT91_SMC_PULSE);
	/* Cycle register  */
	__raw_writel(AT91_SMC_NWECYCLE_(config->write_cycle)
		| AT91_SMC_NRDCYCLE_(config->read_cycle),
		base + AT91_SMC_CYCLE);

	/* Mode register  */
	sam9_smc_cs_write_mode(base, config);
	
#if 0
	/* setup register */
	__raw_write((config->nwe_setup << 0)
		| (config->ncs_write_setup << 8)
		| (config->nrd_setup << ))
#endif
}

void sam9_smc_configure(int id, int cs, struct sam9_smc_config *config)
{
	sam9_smc_cs_configure(AT91_SMC_CS(id, cs), config);
}

#endif

void at91sam9_ioremap_smc(int id, u32 addr)
{
	if (id > 1) {
		pr_warn("%s: id > 2\n", __func__);
		return;
	}
	
	smc_base_addr[id] = ioremap(addr, 512);
	if (!smc_base_addr[id])
	pr_warn("Impossible to ioremap smc. %d 0x%x\n", id, addr);
}

static int __init sam9g45_init(void)
{
	at91_pmc_write(AT91_PMC_PCER,1<<AT91SAM9G45_ID_PIODE);   //input irq function require the gpio clock enable 
	
	at91_set_gpio_input(AT91_PIN_PD23,1);                    
	at91_set_deglitch(AT91_PIN_PD23,0);
	at91sam9_ioremap_smc(0, AT91SAM9G45_BASE_SMC);
	sam9_smc_configure(0, 0, &sam9g45_smc_config);
	platform_device_register(&sam9g45_device);
	return 0;
}

static void __exit sam9g45_exit(void)
{
	platform_device_unregister(&sam9g45_device);
}

module_init(sam9g45_init);
module_exit(sam9g45_exit);

MODULE_LICENSE("GPL");
