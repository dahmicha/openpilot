/**
 * @file	startup.c
 * @brief	Generic CortexM3 startup code for PiOS.
 */

#include <string.h>

/* prototype for main() that tells us not to worry about it possibly returning */
extern int main(void) __attribute__((noreturn));

/* prototype our _main() to avoid prolog/epilog insertion and other related junk */
static void _main(void) __attribute__((noreturn, naked, no_instrument_function));

/** default handler for CPU exceptions */
static void		default_cpu_handler(void) __attribute__((noreturn, naked));

/** BSS symbols XXX should have a header that defines all of these */
extern char		_sbss, _ebss;

/** DATA symbols XXX should have a header that defines all of these */
extern char		_sidata, _sdata, _edata;

/** The bootstrap/IRQ stack XXX should define size somewhere else */
char			irq_stack[512] __attribute__((section(".irqstack")));

/** exception handler */
typedef const void	(vector)(void);

/** CortexM3 CPU vectors */
struct cm3_vectors {
	void	*initial_stack;
	vector	*entry;
	vector	*vectors[14];
};

/**
 * Initial startup code.
 */
void
_main(void)
{
	asm volatile ("mov r10, %0" : : "r" (&irq_stack[0]) : );

	/* copy initialised data from flash to RAM */
	memcpy(&_sdata, &_sidata, &_edata - &_sdata);

	/* zero the BSS */
	memset(&_sbss, 0, &_ebss - &_sbss);

	/* fill most of the IRQ/bootstrap stack with a watermark pattern so we can measure how much is used */
	/* leave a little space at the top in case memset() isn't a leaf with no locals */
	memset(&irq_stack, 0xa5, sizeof(irq_stack) - 64);

	/* call main */
	(void)main();
}

/**
 * Default handler for CPU exceptions.
 */
static void
default_cpu_handler(void)
{
	for (;;) ;
}

/** Prototype for optional exception vector handlers */
#define HANDLER(_name)	extern vector _name __attribute__((weak, alias("default_cpu_handler")))

/* standard CMSIS vector names */
HANDLER(NMI_Handler);
HANDLER(HardFault_Handler);
HANDLER(MemManage_Handler);
HANDLER(BusFault_Handler);
HANDLER(UsageFault_Handler);
HANDLER(DebugMon_Handler);

/* these vectors point directly to the relevant FreeRTOS functions if they are defined */
HANDLER(vPortSVCHandler);
HANDLER(xPortPendSVHandler);
HANDLER(xPortSysTickHandler);

/** CortexM3 vector table */
struct cm3_vectors cpu_vectors __attribute((section(".cpu_vectors"))) = {
		.initial_stack = &irq_stack[sizeof(irq_stack)],
		.entry = (vector *)_main,
		.vectors = {
				NMI_Handler,
				HardFault_Handler,
				MemManage_Handler,
				BusFault_Handler,
				UsageFault_Handler,
				0,
				0,
				0,
				0,
				vPortSVCHandler,
				DebugMon_Handler,
				0,
				xPortPendSVHandler,
				xPortSysTickHandler,
		}
};
