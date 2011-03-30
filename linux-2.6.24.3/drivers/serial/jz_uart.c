/*
 *  linux/drivers/serial/jz_uart.c
 *
 *  Driver for Ingenic serial ports
 *
 *  Based on drivers/char/serial.c, by Linus Torvalds, Theodore Ts'o.
 *
 *  Copyright (C) 2009 INGENIC.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * A note about mapbase / membase
 *
 *  mapbase is the physical address of the IO port.
 *  membase is an 'ioremapped' cookie.
 */
#if defined(CONFIG_SERIAL_INGENIC_CONSOLE) && defined(CONFIG_MAGIC_SYSRQ)
#define SUPPORT_SYSRQ
#endif

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/ioport.h>
#include <linux/init.h>
#include <linux/console.h>
#include <linux/sysrq.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/serial_core.h>
#include <linux/serial.h>
#include <linux/nmi.h>
#include <linux/mutex.h>
#include <linux/version.h>

#include <asm/jzsoc.h>
#include <asm/io.h>
#include <asm/irq.h>

#include "jz_uart.h"

#define UART_DEBUG
#ifdef UART_DEBUG
#define uprintk(fmt...) printk(fmt)
#else
#define uprintk(fmt...)
#endif

#ifdef UART_DEBUG
//#define DUMP_RX
//#define DUMP_TX
//#define UART_DUMP
//#define PORT_DUMP
//#define FUNCTION_DUMP
#endif

#ifdef FUNCTION_DUMP
#define FUN_INFO() uprintk("%s, %d\n", __FUNCTION__, __LINE__)
#else
#define FUN_INFO()
#endif

#define SERIAL_TYPE  "jz_uart"

static unsigned int nr_uarts = CONFIG_SERIAL_INGENIC_RUNTIME_UARTS;

#define UART_NR	CONFIG_SERIAL_INGENIC_NR_UARTS

#define __wr(reg, val) uart_wr(port, reg, val)
#define __rd(reg)      uart_rd(port, reg)
#define __set(reg, val) uart_set(port, reg, val)
#define __clr(reg, val) uart_clr(port, reg, val)

static DEFINE_MUTEX(serial_mutex);

static unsigned int uart_rd(struct uart_port *port, int reg){
    return readb(port->membase + reg);
}

static void uart_wr(struct uart_port *port, int reg, int val){
    writeb(val, port->membase + reg);
}

static void uart_set(struct uart_port *port, int reg, int val){
    unsigned int temp = readb(port->membase + reg);
    writeb(temp | val, port->membase + reg);
}

static void uart_clr(struct uart_port *port, int reg, int val){
    unsigned int temp = readb(port->membase + reg);
    writeb(temp & ~val, port->membase + reg);
}

#ifdef UART_DUMP
void uart_dump(struct uart_port *port){
    int lcr = __rd(UART_LCR);
    
    uprintk("^^^^^ +uart dump ^^^^^^\n");
    __set(UART_LCR, UARTLCR_DLAB);
    uprintk("line(%d)\n", port->line);
    uprintk("UART_DLLR:%#x\n", __rd(UART_DLLR));
    uprintk("UART_DLHR:%#x\n", __rd(UART_DLHR));
    __wr(UART_LCR, lcr);
    uprintk("UART_IER:%#x\n", __rd(UART_IER));
    uprintk("UART_IIR:%#x\n", __rd(UART_IIR));
    uprintk("UART_LCR:%#x\n", __rd(UART_LCR));
    //    uprintk("UART_FCR:%#x\n", __rd(UART_FCR));
    uprintk("UART_MCR:%#x\n", __rd(UART_MCR));
    uprintk("UART_LSR:%#x\n", __rd(UART_LSR));
    uprintk("UART_MSR:%#x\n", __rd(UART_MSR));
    uprintk("UART_SPR:%#x\n", __rd(UART_SPR));
    uprintk("UART_UMR:%#x\n", __rd(UART_UMR));
    uprintk("UART_UACR:%#x\n", __rd(UART_UACR));                            
    uprintk("^^^^^ -uart dump ^^^^^^\n");
}
#endif
#ifdef PORT_DUMP
void port_dump(struct uart_port *port){
    uprintk("------ port dump ------\n");
    uprintk("irq(%d)\n", port->irq);
    uprintk("type(%d)\n", port->type);
    uprintk("clk(%d)\n", port->uartclk);
    uprintk("membase(%#x)\n", (unsigned int)port->membase);
    uprintk("mapbase(%#x)\n", port->mapbase);    
    uprintk("line(%d)\n", port->line);
    uprintk("fifosize(%d)\n", port->fifosize);
    uprintk("regshift(%d)\n", port->regshift);        
    uprintk("suspended(%d)\n", port->suspended);    
    uprintk("------ port dump ------\n");
}
#endif

static unsigned int serial47xx_tx_empty(struct uart_port *port){
    unsigned int lsr = __rd(UART_LSR);
    return (lsr & UARTLSR_TDRQ) ? TIOCSER_TEMT : 0;
}

static void serial47xx_set_mctrl(struct uart_port *port, unsigned int mctrl){
    
}

static unsigned int serial47xx_get_mctrl(struct uart_port *port){
    return (0);
}

static void serial47xx_stop_tx(struct uart_port *port){
    __clr(UART_IER, UARTIER_TIE);
}

static void transmit_chars(struct uart_port *port);
static void serial47xx_start_tx(struct uart_port *port)
{
	if (!(__rd(UART_IER) & UARTIER_TIE)) {
        __set(UART_IER, UARTIER_TIE);
        transmit_chars(port);
    }

}

static void serial47xx_stop_rx(struct uart_port *port){
    __clr(UART_IER, UARTIER_RLIE | UARTIER_RIE);
}

static void serial47xx_enable_ms(struct uart_port *port){
    
}

static void serial47xx_break_ctl(struct uart_port *port, int break_state)
{
    unsigned long flags;

    uprintk("%s, brk(%d)\n", __FUNCTION__, break_state);

    spin_lock_irqsave(&port->lock, flags);
	if (break_state == -1)
        __set(UART_LCR, UARTLCR_SBRK);
	else
        __clr(UART_LCR, UARTLCR_SBRK);
    spin_unlock_irqrestore(&port->lock, flags);
}


static void
receive_chars(struct uart_port *port, unsigned int *status)
{
#if (LINUX_VERSION_CODE == KERNEL_VERSION(2,6,24))
	struct tty_struct *tty = port->info->tty;    
#else
	struct tty_struct *tty = port->info->port.tty;
#endif    
	unsigned char ch, lsr = *status;
	int max_count = 256;
	char flag;

    do {
		if (likely(lsr & UARTLSR_DR))
			ch = __rd(UART_RDR);
		else
			ch = 0;

		flag = TTY_NORMAL;
		port->icount.rx++;

        if (unlikely(lsr & UART_LSR_BRK_ERROR_BITS)) {
			/*
			 * For statistics only
			 */
			if (lsr & UARTLSR_BRK) {
				lsr &= ~(UARTLSR_FER | UARTLSR_PER);
				port->icount.brk++;
				/*
				 * We do the SysRQ and SAK checking
				 * here because otherwise the break
				 * may get masked by ignore_status_mask
				 * or read_status_mask.
				 */
				if (uart_handle_break(port))
					goto ignore_char;
			} else if (lsr & UARTLSR_PER)
				port->icount.parity++;
			else if (lsr & UARTLSR_FER)
				port->icount.frame++;
			if (lsr & UARTLSR_ORER)
				port->icount.overrun++;

			/*
			 * Mask off conditions which should be ignored.
			 */

			lsr &= port->read_status_mask;
            if (lsr & UARTLSR_BRK) {
                flag = TTY_BREAK;
			} else if (lsr & UARTLSR_PER)
				flag = TTY_PARITY;
			else if (lsr & UARTLSR_FER)
				flag = TTY_FRAME;
		}
		if (uart_handle_sysrq_char(port, ch))
			goto ignore_char;

#ifdef DUMP_RX
        uprintk("%c", ch);
#endif        
        
		uart_insert_char(port, lsr, UARTLSR_ORER, ch, flag);

ignore_char:
		lsr = __rd(UART_LSR);
	} while ((lsr & (UARTLSR_DR | UARTLSR_BRK)) && (max_count-- > 0));
	spin_unlock(&port->lock);
	tty_flip_buffer_push(tty);
	spin_lock(&port->lock);
	*status = lsr;
}

static void transmit_chars(struct uart_port *port)
{
	struct circ_buf *xmit = &port->info->xmit;
	//int count = 8;

	if (port->x_char) {
		__wr(UART_TDR, port->x_char);
		port->icount.tx++;
		port->x_char = 0;
		return;
	}
	if (uart_tx_stopped(port) || uart_circ_empty(xmit)) {
		serial47xx_stop_tx(port);
		return;
	}

    while  (__rd(UART_LSR) & UARTLSR_TDRQ){
        __wr(UART_TDR, xmit->buf[xmit->tail]);
        
#ifdef DUMP_TX
        uprintk("%c", xmit->buf[xmit->tail]);
#endif        
		xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
		port->icount.tx++;
		if (uart_circ_empty(xmit))
			break;
	};

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(port);

    if (uart_circ_empty(xmit))
		serial47xx_stop_tx(port);		
}

#define DEVICE_CLOCK   __cpm_get_extalclk()
#define DEFAULT_UART_BAUDRATE  57600
#if defined(CONFIG_JZSOC) && !defined(CONFIG_SOC_JZ4730)
static unsigned short *serial47xx_get_divisor(struct uart_port *port, unsigned int baud);
#else
static unsigned int serial47xx_get_divisor(struct uart_port *port, unsigned int baud);
#endif
static inline void serial_dl_write(struct uart_port *port, int value);
static void serial47xx_set_baudrate(struct uart_port *port, int baud)
{
#if defined(CONFIG_JZSOC) && !defined(CONFIG_SOC_JZ4730)
    unsigned short *quot;
    
	quot = serial47xx_get_divisor(port, baud);
	serial_dl_write(port, quot[0]);
    __wr(UART_UMR, quot[1]);
    __wr(UART_UACR, quot[2]);
#else
    unsigned int quot;
    
	quot = serial47xx_get_divisor(port, baud);
	serial_dl_write(port, quot);
#endif
}

static inline void
serial47xx_handle_port(struct uart_port *port)
{
	unsigned int status;
	unsigned long flags;

    spin_lock_irqsave(&port->lock, flags);

	status = __rd(UART_LSR);

    if (status & (UARTLSR_DR | UARTLSR_BRK))
		receive_chars(port, &status);
	//check_modem_status(up);
	if (status & UARTLSR_TDRQ)
		transmit_chars(port);

	spin_unlock_irqrestore(&port->lock, flags);

}

static irqreturn_t serial47xx_interrupt(int irq, void *dev_id)
{
    struct uart_port *port = dev_id;
    unsigned int iir;
    
    iir = __rd(UART_IIR);

    if (!(iir & UARTIIR_IP)) {
        serial47xx_handle_port(port);
    } 

    return IRQ_HANDLED;
}

static int serial47xx_startup(struct uart_port *port){
    int ret = 0;
    
    __set(UART_FCR, UARTFCR_FE);
    __set(UART_FCR, UARTFCR_FE | UARTFCR_RFLS | UARTFCR_TFLS);
    __set(UART_FCR, UARTFCR_UUE);
    __wr(UART_IER, 0);
    __wr(UART_ISR, 0);
    __wr(UART_LCR, UARTLCR_WLEN_8 | UARTLCR_STOP1);

    serial47xx_set_baudrate(port, DEFAULT_UART_BAUDRATE);
    
    ret = request_irq(port->irq, serial47xx_interrupt,
				  0, "serial", port);
    if (ret){
        return ret;                
    }

    __wr(UART_IER, UARTIER_RLIE | UARTIER_RIE | UARTIER_RTIE);
    
    __wr(UART_FCR, UARTFCR_FE | UARTFCR_RFLS | UARTFCR_TFLS | UARTFCR_UUE);
   
#ifdef UART_DUMP    
    uart_dump(port);
#endif
#ifdef PORT_DUMP    
    port_dump(port);
#endif    

    return ret;
}

static void serial47xx_shutdown(struct uart_port *port){
    __wr(UART_IER, 0);
    __wr(UART_LCR, __rd(UART_LCR) & ~UARTLCR_SBRK);
    __wr(UART_FCR, UARTFCR_FE);
    __wr(UART_FCR, UARTFCR_FE | UARTFCR_RFLS | UARTFCR_TFLS);
    __set(UART_FCR, UARTFCR_UUE);

    free_irq(port->irq, port);
}

#if defined(CONFIG_JZSOC) && !defined(CONFIG_SOC_JZ4730)
static unsigned short quot1[3] = {0}; /* quot[0]:baud_div, quot[1]:umr, quot[2]:uacr */
static unsigned short *serial47xx_get_divisor(struct uart_port *port, unsigned int baud)
{
	int err, sum, i, j;
	int a[12], b[12];
	unsigned short div, umr, uacr;
	unsigned short umr_best, div_best, uacr_best;
	long long t0, t1, t2, t3;

	sum = 0;
	umr_best = div_best = uacr_best = 0;
	div = 1;

    if ((port->uartclk % (16 * baud)) == 0) {
		quot1[0] = port->uartclk / (16 * baud);
		quot1[1] = 16;
		quot1[2] = 0;
		return quot1;
	}

	while (1) {
		umr = port->uartclk / (baud * div);
  		if (umr > 32) {
			div++;
			continue;
		}
		if (umr < 4) {
			break;
		}
		for (i = 0; i < 12; i++) {
			a[i] = umr;
			b[i] = 0;
			sum = 0;
			for (j = 0; j <= i; j++) {
				sum += a[j];
			}

                        /* the precision could be 1/2^(36) due to the value of t0 */
			t0 = 0x1000000000LL;
			t1 = (i + 1) * t0;
			t2 = (sum * div) * t0;
			t3 = div * t0;
			do_div(t1, baud);
			do_div(t2, port->uartclk);
			do_div(t3, (2 * port->uartclk));
			err = t1 - t2 - t3;

			if (err > 0) {
				a[i] += 1;
				b[i] = 1;
			}
		}

		uacr = 0;
		for (i = 0; i < 12; i++) {
			if (b[i] == 1) {
				uacr |= 1 << i;
			}
		}

                /* the best value of umr should be near 16, and the value of uacr should better be smaller */
		if (abs(umr - 16) < abs(umr_best - 16) || (abs(umr - 16) == abs(umr_best - 16) && uacr_best > uacr)) {
			div_best = div;
			umr_best = umr;
			uacr_best = uacr;
		}
		div++;
	}

	quot1[0] = div_best;
	quot1[1] = umr_best;
	quot1[2] = uacr_best;

    return quot1;
}
#else
static unsigned int serial47xx_get_divisor(struct uart_port *port, unsigned int baud)
{
	unsigned int quot;

	/*
	 * Handle magic divisors for baud rates above baud_base on
	 * SMSC SuperIO chips.
	 */
	if ((port->flags & UPF_MAGIC_MULTIPLIER) &&
	    baud == (port->uartclk/4))
		quot = 0x8001;
	else if ((port->flags & UPF_MAGIC_MULTIPLIER) &&
		 baud == (port->uartclk/8))
		quot = 0x8002;
	else
		quot = uart_get_divisor(port, baud);

	return quot;
}
#endif


/* Uart divisor latch read */
static inline int serial_dl_read(struct uart_port *port)
{
    int lcr = __rd(UART_LCR), temp;
    __set(UART_LCR, UARTLCR_DLAB);
    temp = __rd(UART_DLLR) | __rd(UART_DLHR) << 8;
    __wr(UART_LCR, lcr);
    return temp;
}

/* Uart divisor latch write */
static inline void serial_dl_write(struct uart_port *port, int value)
{
    int lcr = __rd(UART_LCR);
    __set(UART_LCR, UARTLCR_DLAB);
	__wr(UART_DLLR, value & 0xff);
	__wr(UART_DLHR, value >> 8 & 0xff);
    __wr(UART_LCR, lcr);
}

static void
serial47xx_set_termios(struct uart_port *port, struct ktermios *termios,
		       struct ktermios *old){
    unsigned char cval;
	unsigned long flags;
	unsigned int baud, quot;
    unsigned short *quot1;

    switch (termios->c_cflag & CSIZE) {
	case CS5:
		cval = UARTLCR_WLEN_5;
		break;
	case CS6:
		cval = UARTLCR_WLEN_6;
		break;
	case CS7:
		cval = UARTLCR_WLEN_7;
		break;
	default:
	case CS8:
		cval = UARTLCR_WLEN_8;
		break;
	}

	if (termios->c_cflag & CSTOPB)
		cval |= UARTLCR_STOP;
	if (termios->c_cflag & PARENB)
		cval |= UARTLCR_PE;
	if (!(termios->c_cflag & PARODD))
		cval |= UARTLCR_PROE;
#ifdef CMSPAR
	if (termios->c_cflag & CMSPAR)
		cval |= UARTLCR_SPAR;
#endif

	/*
	 * Ask the core to calculate the divisor for us.
	 */
	baud = uart_get_baud_rate(port, termios, old, 0, port->uartclk/16);
#if defined(CONFIG_JZSOC) && !defined(CONFIG_SOC_JZ4730)
	quot1 = serial47xx_get_divisor(port, baud);
	quot = quot1[0]; /* not usefull, just let gcc happy */
#else
	quot = serial47xx_get_divisor(port, baud);
#endif

	spin_lock_irqsave(&port->lock, flags);

	/*
	 * Update the per-port timeout.
	 */
	uart_update_timeout(port, termios->c_cflag, baud);

	port->read_status_mask = UARTLSR_ORER | UARTLSR_DR | UARTLSR_TDRQ;
	if (termios->c_iflag & INPCK)
		port->read_status_mask |= UARTLSR_FER | UARTLSR_PER;

    if (termios->c_iflag & (BRKINT | PARMRK))
		port->read_status_mask |= UARTLSR_BRK;

    __set(UART_IER, UARTIER_RTIE);

#if defined(CONFIG_JZSOC) && !defined(CONFIG_SOC_JZ4730)
	serial_dl_write(port, quot1[0]);
    __wr(UART_UMR, quot1[1]);
    __wr(UART_UACR, quot1[2]);
#else
	serial_dl_write(port, quot);
#endif

    __wr(UART_LCR, cval);		/* reset DLAB */

	spin_unlock_irqrestore(&port->lock, flags);
	/* Don't rewrite B0 */
	if (tty_termios_baud_rate(termios))
		tty_termios_encode_baud_rate(termios, baud, baud);

}

static void serial47xx_pm(struct uart_port *port, unsigned int state,
	      unsigned int oldstate)
{
    FUN_INFO();
}

static const char *serial47xx_type(struct uart_port *port)
{
    FUN_INFO();    
	return SERIAL_TYPE;
}

static void serial47xx_release_port(struct uart_port *port)
{
    FUN_INFO();    
}

static int serial47xx_request_port(struct uart_port *port)
{
    FUN_INFO();    
    return 0;
}

static void serial47xx_config_port(struct uart_port *port, int flags)
{
    FUN_INFO();
}

static int
serial47xx_verify_port(struct uart_port *port, struct serial_struct *ser)
{
    FUN_INFO();
	return 0;
}

static struct uart_ops serial47xx_pops = {
	.tx_empty	= serial47xx_tx_empty,
	.set_mctrl	= serial47xx_set_mctrl,
	.get_mctrl	= serial47xx_get_mctrl,
	.stop_tx	= serial47xx_stop_tx,
	.start_tx	= serial47xx_start_tx,
	.stop_rx	= serial47xx_stop_rx,
	.enable_ms	= serial47xx_enable_ms,
	.break_ctl	= serial47xx_break_ctl,
	.startup	= serial47xx_startup,
	.shutdown	= serial47xx_shutdown,
	.set_termios	= serial47xx_set_termios,
	.pm		= serial47xx_pm,
	.type		= serial47xx_type,
	.release_port	= serial47xx_release_port,
	.request_port	= serial47xx_request_port,
	.config_port	= serial47xx_config_port,
	.verify_port	= serial47xx_verify_port,
};

static struct uart_port serial47xx_ports[UART_NR] = {
    [0] = {
        .line = 0,
        .irq = IRQ_UART0,
        .type = PORT_47XX,
        .iotype		= UPIO_MEM,
        .fifosize	= 16,
        //        .flags		= UPF_BOOT_AUTOCONF,        
        .membase = (u8 *)UART0_BASE,
    },
#if (CONFIG_SERIAL_INGENIC_NR_UARTS >= 2)
    [1] = {
        .line = 1,
        .irq = IRQ_UART1,
        .type = PORT_47XX,
        .iotype		= UPIO_MEM,
        .fifosize	= 16,
        //        .flags		= UPF_BOOT_AUTOCONF,        
        .membase = (u8 *)UART1_BASE,
    },
#endif
#if (CONFIG_SERIAL_INGENIC_NR_UARTS >= 3)
    [2] = {
        .line = 2,
        .irq = IRQ_UART2,
        .type = PORT_47XX,
        .iotype		= UPIO_MEM,
        .fifosize	= 16,
        //        .flags		= UPF_BOOT_AUTOCONF,        
        .membase = (u8 *)UART2_BASE,
    },
#endif
#if (CONFIG_SERIAL_INGENIC_NR_UARTS >= 4)
    [3] = {
        .line = 3,
        .irq = IRQ_UART3,
        .type = PORT_47XX,
        .iotype		= UPIO_MEM,
        .fifosize	= 16,
        //        .flags		= UPF_BOOT_AUTOCONF,        
        .membase = (u8 *)UART3_BASE,
    },
#endif
};

static void __init serial47xx_init_ports(void)
{
    static int first = 1;
	int i;

    FUN_INFO();

    if (!first)
		return;
	first = 0;

	for (i = 0; i < nr_uarts; i++) {
		struct uart_port *up = &serial47xx_ports[i];

		//up->line = i;
		spin_lock_init(&up->lock);
        /*
		 * ALPHA_KLUDGE_MCR needs to be killed.
		 */
		//up->mcr_mask = ~ALPHA_KLUDGE_MCR;
		//up->mcr_force = ALPHA_KLUDGE_MCR;
        up->uartclk = DEVICE_CLOCK;
		up->ops = &serial47xx_pops;
	}

    FUN_INFO();    
}

static void __init
serial47xx_register_ports(struct uart_driver *drv, struct device *dev)
{
	int i, ret = 0;

	serial47xx_init_ports();

	for (i = 0; i < nr_uarts; i++) {
		struct uart_port *up = &serial47xx_ports[i];

		up->dev = dev;
		ret = uart_add_one_port(drv, up);
        if (ret){
            uprintk("uart add port failed!\n");
            return;
        }
	}
}


#ifdef CONFIG_SERIAL_INGENIC_CONSOLE

#define putchar(port, ch) do {                     \
    while (!(__rd(UART_LSR) & UARTLSR_TDRQ));      \
    __wr(UART_TDR, ch);                            \
} while (0)

static inline void
serial_write(struct uart_port *port, const char *s, unsigned int count){
	unsigned int i;

	for (i = 0; i < count; i++, s++) {
		if (*s == '\n')
			putchar(port, '\r');
		putchar(port, *s);
	}
}

/*
 *	Print a string to the serial port trying not to disturb
 *	any possible real use of the port...
 *
 *	The console_lock must be held when we get here.
 */
static void
serial47xx_console_write(struct console *co, const char *s, unsigned int count)
{
	struct uart_port *port = &serial47xx_ports[co->index];
	unsigned long flags;
	unsigned int ier;

    spin_lock_irqsave(&port->lock, flags);

    /*
	 *	First save the IER then disable the interrupts
	 */
	ier = __rd(UART_IER);

    __wr(UART_IER, 0);

	serial_write(port, s, count);

	/*
	 *	Finally, wait for transmitter to become empty
	 *	and restore the IER
	 */
    while (!(__rd(UART_LSR) & UARTLSR_TDRQ));
	__wr(UART_IER, ier);

    spin_unlock_irqrestore(&port->lock, flags);
}

static int __init serial47xx_console_setup(struct console *co, char *options)
{
	struct uart_port *port;
	int baud = 57600;
	int bits = 8;
	int parity = 'n';
	int flow = 'n';

    FUN_INFO();
	
	/*
	 * Check whether an invalid uart number has been specified, and
	 * if so, search for the first available port that does have
	 * console support.
	 */
	if (co->index >= nr_uarts)
		co->index = 0;
	port = &serial47xx_ports[co->index];
	if (!port->iobase && !port->membase)
		return -ENODEV;
    /*
	ret = update_console_cmdline("uart", jz47xx,
			     "ttyS", port->line, options);
	if (ret < 0)
		ret = update_console_cmdline("uart", 0,
    		     "ttyS", port->line, options);
    */
    if (options)
		uart_parse_options(options, &baud, &parity, &bits, &flow);

    FUN_INFO();
    
    return uart_set_options(port, co, baud, parity, bits, flow);
}

static int serial47xx_console_early_setup(void)
{
    printk("%s,line->%d\n", __FUNCTION__, __LINE__);
	return 0;
}


static struct uart_driver serial47xx_uart_driver;
static struct console serial47xx_console = {
	.name		= "ttyS",
	.write		= serial47xx_console_write,
	.device		= uart_console_device,
	.setup		= serial47xx_console_setup,
	.early_setup	= serial47xx_console_early_setup,    
    .flags		= CON_PRINTBUFFER,
	.index		= -1,
	.data		= &serial47xx_uart_driver,
};

static int __init serial47xx_console_init(void)
{
    if (nr_uarts > UART_NR)
        nr_uarts = UART_NR;

    FUN_INFO();

    serial47xx_init_ports();
    register_console(&serial47xx_console);

    FUN_INFO();
    
	return 0;
}
console_initcall(serial47xx_console_init);
 
#define SERIAL_47XX_CONSOLE &serial47xx_console
#else
#define SERIAL_47XX_CONSOLE NULL
#endif //CONFIG_SERIAL_INGENIC_CONSOLE

static struct uart_driver serial47xx_uart_driver = {
	.owner			= THIS_MODULE,
	.driver_name		= "serial",
	.dev_name		= "ttyS",
	.major			= TTY_MAJOR,
	.minor			= 64,
	.nr			= UART_NR,
	.cons			= SERIAL_47XX_CONSOLE,
};

/**
 *	serial47xx_suspend_port - suspend one serial port
 *	@line:  serial line number
 *
 *	Suspend one serial port.
 */
void serial47xx_suspend_port(int line)
{
    FUN_INFO();    
	uart_suspend_port(&serial47xx_uart_driver, &serial47xx_ports[line]);
}

/**
 *	serial47xx_resume_port - resume one serial port
 *	@line:  serial line number
 *
 *	Resume one serial port.
 */
void serial47xx_resume_port(int line)
{
	struct uart_port *port = &serial47xx_ports[line];
	uart_resume_port(&serial47xx_uart_driver, port);
    FUN_INFO();    
}

/*
 * Register a set of serial devices attached to a platform device.  The
 * list is terminated with a zero flags entry, which means we expect
 * all entries to have at least UPF_BOOT_AUTOCONF set.
 */
static int __devinit serial47xx_probe(struct platform_device *dev)
{
    FUN_INFO();
    return 0;
}

/*
 * Remove serial ports registered against a platform device.
 */
static int __devexit serial47xx_remove(struct platform_device *dev)
{
    FUN_INFO();    
    return 0;
}

static int serial47xx_suspend(struct platform_device *dev, pm_message_t state)
{
    FUN_INFO();    
	return 0;
}

static int serial47xx_resume(struct platform_device *dev)
{
    FUN_INFO();
	return 0;
}

static struct platform_driver serial47xx_platform_driver = {
	.probe		= serial47xx_probe,
	.remove		= __devexit_p(serial47xx_remove),
	.suspend	= serial47xx_suspend,
	.resume		= serial47xx_resume,
	.driver		= {
		.name	= "serial_jz",
		.owner	= THIS_MODULE,
	},
};
 
static struct platform_device *serial47xx_platform_devs;
#define PLAT47xx_DEV_ID     -1
static int __init serial47xx_init(void)
{
	int ret;

    FUN_INFO();
    
	if (nr_uarts > UART_NR)
		nr_uarts = UART_NR;

    ret = uart_register_driver(&serial47xx_uart_driver);
	if (ret){
        uprintk("uart_driver register failed!\n");
        goto out;        
    }

    serial47xx_platform_devs = platform_device_alloc("serial47xx",
						    PLAT47xx_DEV_ID);
	if (!serial47xx_platform_devs) {
        uprintk("platform device allo failed!\n");
		ret = -ENOMEM;
		goto unreg_uart_drv;
	}

	ret = platform_device_add(serial47xx_platform_devs);
	if (ret){
        uprintk("platform add failed!\n");
		goto put_dev;        
    }

    serial47xx_register_ports(&serial47xx_uart_driver, &serial47xx_platform_devs->dev);

	ret = platform_driver_register(&serial47xx_platform_driver);
	if (ret == 0)
		goto out;

    uprintk("register platform driver failed!\n");
    
	platform_device_del(serial47xx_platform_devs);
 put_dev:
	platform_device_put(serial47xx_platform_devs);
 unreg_uart_drv:
	uart_unregister_driver(&serial47xx_uart_driver);
 out:
    FUN_INFO();
    uprintk("%s, ret:%d\n", __FUNCTION__, ret);
    return ret;
}

static void __exit serial47xx_exit(void)
{
	struct platform_device *dev = serial47xx_platform_devs;

	/*
	 * This tells serial47xx_unregister_port() not to re-register
	 * the ports (thereby making serial47xx_platform_driver permanently
	 * in use.)
	 */
    FUN_INFO();
    
	serial47xx_platform_devs = NULL;

	platform_driver_unregister(&serial47xx_platform_driver);
	platform_device_unregister(dev);

	uart_unregister_driver(&serial47xx_uart_driver);

    FUN_INFO();
}

module_init(serial47xx_init);
module_exit(serial47xx_exit);

EXPORT_SYMBOL(serial47xx_suspend_port);
EXPORT_SYMBOL(serial47xx_resume_port);
