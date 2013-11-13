/* ########################################################################

   nullmodem - linux null modem emulator (module)

   ########################################################################

   Copyright (c) : 2012 Peter Remmers

	Based on tty0tty driver -  Copyright (c) : 2010  Luis Claudio Gambôa Lopes
	Based on Tiny TTY driver -  Copyright (C) 2002-2004 Greg Kroah-Hartman (greg@kroah.com)

   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 2, or (at your option)
   any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program; if not, write to the Free Software
   Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.

   For e-mail suggestions :  pitti98@googlemail.com
   ######################################################################## */


#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/wait.h>
#include <linux/tty.h>
#include <linux/tty_driver.h>
#include <linux/tty_flip.h>
#include <linux/serial.h>
#include <linux/sched.h>
#include <linux/kfifo.h>
#include <asm/uaccess.h>

#define DRIVER_VERSION "v1.1"
#define DRIVER_AUTHOR "Peter Remmers <pitti98@googlemail.com>"
#define DRIVER_DESC "nullmodem driver"

/* Module information */
MODULE_AUTHOR( DRIVER_AUTHOR );
MODULE_DESCRIPTION( DRIVER_DESC );
MODULE_LICENSE("GPL");

#ifdef SCULL_DEBUG
#define dprintf(fmt, args...) printk(KERN_DEBUG fmt, ##args)
#else
#define dprintf(fmt, args...)
#endif

#define NULLMODEM_MAJOR		240	/* experimental range */
#define NULLMODEM_PAIRS		4

#define TIMER_INTERVAL (HZ/20)
//#define TIMER_INTERVAL HZ
#define TX_BUF_SIZE 4096
#define WAKEUP_CHARS 256

struct nullmodem_pair;
struct nullmodem_end
{
	struct tty_struct		*tty;		/* pointer to the tty for this device */
	struct nullmodem_end	*other;
	struct nullmodem_pair	*pair;
	struct async_icount		icount;
	struct serial_struct	serial;
	unsigned char			xchar;
	unsigned char			char_length;
	unsigned				nominal_bit_count;
	unsigned				actual_bit_count;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,0,0)	
	struct kfifo			fifo;
#else
	struct kfifo			*fifo;
#endif
};
struct nullmodem_pair
{
	spinlock_t				spin;		/* locks this structure */
	struct nullmodem_end	a;
	struct nullmodem_end	b;
	int						control_lines;	/* control lines as seen from end a */
	wait_queue_head_t		control_lines_wait;
};
static struct nullmodem_pair pair_table[NULLMODEM_PAIRS];

static struct timer_list nullmodem_timer;

static int switch_pin_view(int pins)
{
	int out = 0;
	if (pins & TIOCM_RTS) out |= TIOCM_CTS;
	if (pins & TIOCM_DTR) out |= TIOCM_DSR;
	if (pins & TIOCM_CTS) out |= TIOCM_RTS;
	if (pins & TIOCM_DSR) out |= TIOCM_DTR;
	return out;
}
static int get_pins(struct nullmodem_end *end)
{
	int pins = end->pair->control_lines;
	if (end == &end->pair->b)
		pins = switch_pin_view(pins);
	if (pins&TIOCM_DSR)
		pins |= TIOCM_CD;
	return pins;
}
static void change_pins(struct nullmodem_end *end, unsigned int set, unsigned int clear)
{
	int is_end_b = (end == &end->pair->b);
	int old_pins = end->pair->control_lines;
	if (is_end_b)
		old_pins = switch_pin_view(old_pins);

	int new_pins = (old_pins & ~clear) | set;
	int change = old_pins ^ new_pins;

	if (is_end_b)
		new_pins = switch_pin_view(new_pins);

	end->pair->control_lines = new_pins;

	if (change & TIOCM_RTS)
	{
		end->other->icount.cts++;
	}
	if (change & TIOCM_DTR)
	{
		end->other->icount.dsr++;
		end->other->icount.dcd++;
	}

	if (end->other->tty
	&& (end->other->tty->termios->c_cflag & CRTSCTS)
	&& (change&TIOCM_RTS))
	{
		if (!(new_pins&TIOCM_RTS))
			end->other->tty->hw_stopped = 1;
		else
		{
			end->other->tty->hw_stopped = 0;
			tty_wakeup(end->other->tty);
		}
	}

	if (change)
		wake_up_interruptible(&end->pair->control_lines_wait);
}

static unsigned char drain[TX_BUF_SIZE];
static unsigned long last_timer_jiffies;
static unsigned long delta_jiffies;
#define FACTOR 10
static inline void handle_end(struct nullmodem_end *end)
{
	if (!end->tty)
		return;
	if (end->tty->hw_stopped)
	{
		//dprintf("%s - #%d: hw_stopped\n", __FUNCTION__, end->tty->index);
		return;
	}
	unsigned nominal_bits = end->tty->termios->c_ospeed * FACTOR * delta_jiffies / HZ;
	unsigned add_bits = end->nominal_bit_count - end->actual_bit_count;
	unsigned chars = (nominal_bits+add_bits) / end->char_length;
	unsigned actual_bits = chars * end->char_length;

	end->nominal_bit_count += nominal_bits;
	end->actual_bit_count  += actual_bits;

//	dprintf("%s - #%d: nb %u add %u ab %u ch %u nbc %u abc %u\n", __FUNCTION__,
//			end->tty->index, nominal_bits, add_bits, actual_bits, chars,
//			end->nominal_bit_count, end->actual_bit_count);

	if (chars == 0)
		return;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,0,0)	
	int cnt = kfifo_out(&end->fifo, drain, chars);
#else
	int cnt = __kfifo_get(end->fifo, drain, chars);
#endif
	if (cnt < chars)
	{
		end->nominal_bit_count = 0;
		end->actual_bit_count = 0;
	}
	if (cnt <= 0)
	{
		//dprintf("%s - #%d: fifo empty\n", __FUNCTION__, end->tty->index);
		return;
	}

//	dprintf("%s - #%d: drained %d bytes\n", __FUNCTION__, end->tty->index, cnt);

	if (end->other->tty)
	{
		if (end->tty->termios->c_ospeed == end->other->tty->termios->c_ispeed
		&& (end->tty->termios->c_cflag & (CSIZE|PARENB|CSTOPB))
		 ==(end->other->tty->termios->c_cflag & (CSIZE|PARENB|CSTOPB)))
		{
			tcflag_t csize = (end->tty->termios->c_cflag&CSIZE);
			if (csize != CS8)
			{
				int i;
				unsigned char mask = 0xFF;
				switch (csize)
				{
				case CS7: mask = 0x7F; break;
				case CS6: mask = 0x3F; break;
				case CS5: mask = 0x1F; break;
				}
				for (i=0; i<cnt; ++i)
					drain[i] &= mask;
			}
			int written = tty_insert_flip_string(end->other->tty, drain, cnt);
			if (written > 0)
			{
				//dprintf("%s - #%d -> #%d: copied %d bytes\n", __FUNCTION__, end->tty->index, end->other->tty->index, written);
				tty_flip_buffer_push(end->other->tty);
			}
		}
	}

//	if (kfifo_len(&end->fifo) < WAKEUP_CHARS)
		tty_wakeup(end->tty);
}
static void nullmodem_timer_proc(unsigned long data)
{
	int i;
	unsigned long flags;
	//dprint("%s jiffies: %lu\n", __FUNCTION__, jiffies);

	unsigned long current_jiffies = jiffies;
	delta_jiffies = current_jiffies - last_timer_jiffies;
	last_timer_jiffies = current_jiffies;

	for (i=0; i<NULLMODEM_PAIRS; ++i)
	{
		struct nullmodem_pair *pair = &pair_table[i];

		spin_lock_irqsave(&pair->spin, flags);
		handle_end(&pair->a);
		handle_end(&pair->b);
		spin_unlock_irqrestore(&pair->spin, flags);
	}

	nullmodem_timer.expires += TIMER_INTERVAL;
	add_timer(&nullmodem_timer);
}

static void handle_termios(struct tty_struct *tty)
{
	struct nullmodem_end *end = tty->driver_data;

	speed_t speed = tty_get_baud_rate(tty);
	if (speed == 0)
		change_pins(end, 0, TIOCM_DTR|TIOCM_RTS);
	else
		change_pins(end, TIOCM_DTR|TIOCM_RTS, 0);

	unsigned int cflag = tty->termios->c_cflag;
	end->char_length = 2;
	switch (cflag & CSIZE)
	{
	case CS5:	end->char_length +=5;	break;
	case CS6:	end->char_length +=6;	break;
	case CS7:	end->char_length +=7;	break;
	default:
	case CS8:	end->char_length +=8;	break;
	}
	if (cflag & PARENB) end->char_length += 1;
	if (cflag & CSTOPB) end->char_length += 1;
	end->char_length *= FACTOR;

	tty->hw_stopped = (tty->termios->c_cflag&CRTSCTS)
					&& !(get_pins(end) & TIOCM_CTS);
}
static int nullmodem_open(struct tty_struct *tty, struct file *file)
{
	struct nullmodem_pair *pair = &pair_table[tty->index/2];
	struct nullmodem_end *end = ((tty->index&1) ? &pair->b : &pair->a);
	unsigned long flags;
	int err = -ENOMEM;

	dprintf("%s - #%d c:%d\n", __FUNCTION__, tty->index, tty->count);

	if (tty->count > 1)
		return 0;

	spin_lock_irqsave(&end->pair->spin, flags);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,0,0)	
	if (kfifo_alloc(&end->fifo, TX_BUF_SIZE, GFP_KERNEL))
		goto exit;
#else
	end->fifo = kfifo_alloc(TX_BUF_SIZE, GFP_KERNEL, NULL);
	if (!end->fifo)
		goto exit;
#endif
	tty->driver_data = end;
	end->tty = tty;
	end->nominal_bit_count = 0;
	end->actual_bit_count = 0;
	handle_termios(tty);
	err = 0;
exit:
	spin_unlock_irqrestore(&end->pair->spin, flags);
	return err;
}

static void nullmodem_close(struct tty_struct *tty, struct file *file)
{
	struct nullmodem_end *end = tty->driver_data;
	unsigned long flags;

	dprintf("%s - #%d c:%d\n", __FUNCTION__, tty->index, tty->count);

	if (tty->count > 1)
		return;

	spin_lock_irqsave(&end->pair->spin, flags);
	end->tty = NULL;
	change_pins(end, 0, TIOCM_RTS|TIOCM_DTR);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,0,0)	
	kfifo_free(&end->fifo);
#else
	kfifo_free(end->fifo);
	end->fifo = NULL;
#endif
	tty->hw_stopped = 1;
	spin_unlock_irqrestore(&end->pair->spin, flags);

	wake_up_interruptible(&tty->read_wait);
	wake_up_interruptible(&tty->write_wait);
}

static int nullmodem_write(struct tty_struct *tty, const unsigned char *buffer, int count)
{
	struct nullmodem_end *end = tty->driver_data;
	unsigned long flags;
	int written = 0;

	if (tty->stopped)
	{
		dprintf("%s - #%d %d bytes --> 0 (tty stopped)\n", __FUNCTION__, tty->index, count);
		return 0;
	}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,0,0)
	written = kfifo_in(&end->fifo, buffer, count);
#else
	written = __kfifo_put(end->fifo, buffer, count);
#endif
	//dprintf("%s - #%d %d bytes --> %d written\n", __FUNCTION__, tty->index, count, written);
	return written;
}

static int nullmodem_write_room(struct tty_struct *tty)
{
	struct nullmodem_end *end = tty->driver_data;
	int room = 0;

	if (tty->stopped)
	{
		dprintf("%s - #%d --> %d (tty stopped)\n", __FUNCTION__, tty->index, room);
		return 0;
	}
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,0,0)	
	room = kfifo_avail(&end->fifo);
#else
	room = TX_BUF_SIZE - __kfifo_len(end->fifo);
#endif
	//dprintf("%s - #%d --> %d\n", __FUNCTION__, tty->index, room);
	return room;
}

#define RELEVANT_IFLAG(iflag) ((iflag) & (IGNBRK|BRKINT|IGNPAR|PARMRK|INPCK))

static void nullmodem_set_termios(struct tty_struct *tty, struct ktermios *old_termios)
{
	struct nullmodem_end *end = tty->driver_data;
	unsigned long flags;
	unsigned int cflag;

	dprintf("%s - #%d\n", __FUNCTION__, tty->index);

	cflag = tty->termios->c_cflag;

	/* check that they really want us to change something */
	if (old_termios)
	{
		if (cflag == old_termios->c_cflag
		&& RELEVANT_IFLAG(tty->termios->c_iflag) == RELEVANT_IFLAG(old_termios->c_iflag))
		{
			dprintf(" - nothing to change...\n");
			return;
		}
	}
	spin_lock_irqsave(&end->pair->spin, flags);
	handle_termios(tty);
	spin_unlock_irqrestore(&end->pair->spin, flags);

#ifdef SCULL_DEBUG
	speed_t speed = tty_get_baud_rate(tty);
	dprintf(" - baud = %u", speed);
	dprintf(" - ispeed = %u", tty->termios->c_ispeed);
	dprintf(" - ospeed = %u", tty->termios->c_ospeed);

	/* get the byte size */
	switch (cflag & CSIZE)
	{
	case CS5:	dprintf(" - data bits = 5\n");	break;
	case CS6:	dprintf(" - data bits = 6\n");	break;
	case CS7:	dprintf(" - data bits = 7\n");	break;
	default:
	case CS8:	dprintf(" - data bits = 8\n");	break;
	}

	/* determine the parity */
	if (cflag & PARENB)
		if (cflag & PARODD)
			dprintf(" - parity = odd\n");
		else
			dprintf(" - parity = even\n");
	else
		dprintf(" - parity = none\n");

	/* figure out the stop bits requested */
	if (cflag & CSTOPB)
		dprintf(" - stop bits = 2\n");
	else
		dprintf(" - stop bits = 1\n");

	/* figure out the hardware flow control settings */
	if (cflag & CRTSCTS)
		dprintf(" - RTS/CTS is enabled\n");
	else
		dprintf(" - RTS/CTS is disabled\n");

	/* determine software flow control */
	/* if we are implementing XON/XOFF, set the start and
	 * stop character in the device */
	/* if we are implementing INBOUND XON/XOFF */
	if (I_IXOFF(tty))
		dprintf(" - INBOUND XON/XOFF is enabled, "
			"XON = %2x, XOFF = %2x\n", START_CHAR(tty), STOP_CHAR(tty));
	else
		dprintf(" - INBOUND XON/XOFF is disabled\n");

	/* if we are implementing OUTBOUND XON/XOFF */
	if (I_IXON(tty))
		dprintf(" - OUTBOUND XON/XOFF is enabled, "
			"XON = %2x, XOFF = %2x\n", START_CHAR(tty), STOP_CHAR(tty));
	else
		dprintf(" - OUTBOUND XON/XOFF is disabled\n");
#endif
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,0,0)
static int nullmodem_tiocmget(struct tty_struct *tty)
#else
static int nullmodem_tiocmget(struct tty_struct *tty, struct file *filp)
#endif
{
	struct nullmodem_end *end = tty->driver_data;
	unsigned long flags;
	int retval = -EINVAL;

	spin_lock_irqsave(&end->pair->spin, flags);
	retval = get_pins(end);
	spin_unlock_irqrestore(&end->pair->spin, flags);

	//dprintf("%s - #%d --> 0x%x\n", __FUNCTION__, tty->index, retval);
	return retval;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,0,0)
static int nullmodem_tiocmset(struct tty_struct *tty, unsigned int set, unsigned int clear)
#else
static int nullmodem_tiocmset(struct tty_struct *tty, struct file *filp, unsigned int set, unsigned int clear)
#endif
{
	struct nullmodem_end *end = tty->driver_data;
	unsigned long flags;

	dprintf("%s - #%d set:0x%x clear:0x%x\n", __FUNCTION__,
			tty->index, set, clear);

	spin_lock_irqsave(&end->pair->spin, flags);
	change_pins(end, set, clear);
	spin_unlock_irqrestore(&end->pair->spin, flags);
	return 0;
}


static int nullmodem_ioctl_tiocgserial(struct tty_struct *tty, unsigned long arg)
{
	struct nullmodem_end *end = tty->driver_data;
	unsigned long flags;
	struct serial_struct tmp;
	struct serial_struct *serial;

	dprintf("%s - #%d\n", __FUNCTION__, tty->index);

	if (!arg)
		return -EFAULT;

	serial = &end->serial;
	memset(&tmp, 0, sizeof(tmp));

	spin_lock_irqsave(&end->pair->spin, flags);
	tmp.type			= serial->type;
	tmp.line			= serial->line;
	tmp.port			= serial->port;
	tmp.irq				= serial->irq;
	tmp.flags			= ASYNC_SKIP_TEST | ASYNC_AUTO_IRQ;
	tmp.xmit_fifo_size	= serial->xmit_fifo_size;
	tmp.baud_base		= serial->baud_base;
	tmp.close_delay		= 5*HZ;
	tmp.closing_wait	= 30*HZ;
	tmp.custom_divisor	= serial->custom_divisor;
	tmp.hub6			= serial->hub6;
	tmp.io_type			= serial->io_type;
	spin_unlock_irqrestore(&end->pair->spin, flags);

	if (copy_to_user((void __user *)arg, &tmp, sizeof(struct serial_struct)))
		return -EFAULT;
	return 0;
}

static int nullmodem_ioctl_tiocmiwait(struct tty_struct *tty, unsigned long arg)
{
	struct nullmodem_end *end = tty->driver_data;
	unsigned long flags;
	DECLARE_WAITQUEUE(wait, current);
	int pins;
	int prev;
	int changed;
	int ret;

	dprintf("%s - #%d\n", __FUNCTION__, tty->index);

	if ((tty->index&1) == 0)
	{
		if (arg & TIOCM_CD) arg |= TIOCM_DSR;
	}
	else
	{
		int t = 0;
		if (arg & TIOCM_CTS) t |= TIOCM_RTS;
		if (arg & TIOCM_DSR) t |= TIOCM_DTR;
		if (arg & TIOCM_CD)  t |= TIOCM_DTR;
		arg = t;
	}

	spin_lock_irqsave(&end->pair->spin, flags);
	prev = end->pair->control_lines;
	add_wait_queue(&end->pair->control_lines_wait, &wait);
	set_current_state(TASK_INTERRUPTIBLE);
	spin_unlock_irqrestore(&end->pair->spin, flags);

	while (1)
	{
		schedule();

		/* see if a signal woke us up */
		if (signal_pending(current))
		{
			ret = -ERESTARTSYS;
			break;
		}

		spin_lock_irqsave(&end->pair->spin, flags);
		pins = end->pair->control_lines;
		set_current_state(TASK_INTERRUPTIBLE);
		spin_unlock_irqrestore(&end->pair->spin, flags);

		changed = pins ^ prev;
		if (changed & arg)
		{
			ret = 0;
			break;
		}

		prev = pins;
	}
	remove_wait_queue(&end->pair->control_lines_wait, &wait);
	set_current_state(TASK_RUNNING);
	return ret;
}

static int nullmodem_ioctl_tiocgicount(struct tty_struct *tty, unsigned long arg)
{
	struct nullmodem_end *end = tty->driver_data;
	unsigned long flags;
	struct serial_icounter_struct icount;

	dprintf("%s - #%d\n", __FUNCTION__, tty->index);

	spin_lock_irqsave(&end->pair->spin, flags);
	icount.cts			= end->icount.cts;
	icount.dsr			= end->icount.dsr;
	icount.rng			= end->icount.rng;
	icount.dcd			= end->icount.dcd;
	icount.rx			= end->icount.rx;
	icount.tx			= end->icount.tx;
	icount.frame		= end->icount.frame;
	icount.overrun		= end->icount.overrun;
	icount.parity		= end->icount.parity;
	icount.brk			= end->icount.brk;
	icount.buf_overrun	= end->icount.buf_overrun;
	spin_unlock_irqrestore(&end->pair->spin, flags);

	if (copy_to_user((void __user *)arg, &icount, sizeof(icount)))
		return -EFAULT;
	return 0;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,0,0)
static int nullmodem_ioctl(struct tty_struct *tty, unsigned int cmd, unsigned long arg)
#else
static int nullmodem_ioctl(struct tty_struct *tty, struct file *filp, unsigned int cmd, unsigned long arg)
#endif
{
	if (cmd == TCGETS || cmd == TCSETS)
		return -ENOIOCTLCMD;

	dprintf("%s - #%d cmd:0x%x\n", __FUNCTION__, tty->index, cmd);

	switch (cmd)
	{
	case TIOCGSERIAL:
		return nullmodem_ioctl_tiocgserial(tty, arg);
	case TIOCMIWAIT:
		return nullmodem_ioctl_tiocmiwait(tty, arg);
	case TIOCGICOUNT:
		return nullmodem_ioctl_tiocgicount(tty, arg);
	}

	return -ENOIOCTLCMD;
}

static void nullmodem_send_xchar(struct tty_struct *tty, char ch)
{
	struct nullmodem_end *end = tty->driver_data;
	unsigned long flags;

	dprintf("%s - #%d\n", __FUNCTION__, tty->index);

	end->xchar = ch;
}

static void nullmodem_throttle(struct tty_struct * tty)
{
	struct nullmodem_end *end = tty->driver_data;
	unsigned long flags;

	dprintf("%s - #%d\n", __FUNCTION__, tty->index);

	if (I_IXOFF(tty))
		nullmodem_send_xchar(tty, STOP_CHAR(tty));

	if (tty->termios->c_cflag & CRTSCTS)
	{
		spin_lock_irqsave(&end->pair->spin, flags);
		change_pins(end, 0, TIOCM_RTS);
		spin_unlock_irqrestore(&end->pair->spin, flags);
	}
}

static void nullmodem_unthrottle(struct tty_struct * tty)
{
	struct nullmodem_end *end = tty->driver_data;
	unsigned long flags;

	dprintf("%s - #%d\n", __FUNCTION__, tty->index);

	if (tty->termios->c_cflag & CRTSCTS)
	{
		spin_lock_irqsave(&end->pair->spin, flags);
		change_pins(end, TIOCM_RTS, 0);
		spin_unlock_irqrestore(&end->pair->spin, flags);
	}
	if (I_IXOFF(tty))
		nullmodem_send_xchar(tty, START_CHAR(tty));
}

static struct tty_operations serial_ops =
{
	.open = nullmodem_open,
	.close = nullmodem_close,
	.throttle = nullmodem_throttle,
	.unthrottle = nullmodem_unthrottle,
	.write = nullmodem_write,
	.write_room = nullmodem_write_room,
	.set_termios = nullmodem_set_termios,
	//.send_xchar = nullmodem_send_xchar,
	.tiocmget = nullmodem_tiocmget,
	.tiocmset = nullmodem_tiocmset,
	.ioctl = nullmodem_ioctl,
};

static struct tty_driver *nullmodem_tty_driver;

static int __init nullmodem_init(void)
{
	int retval;
	int i;
	dprintf("%s - \n", __FUNCTION__);

	init_timer(&nullmodem_timer);
	setup_timer(&nullmodem_timer, nullmodem_timer_proc, 0);

	for (i = 0; i < NULLMODEM_PAIRS; ++i)
	{
		struct nullmodem_pair *pair = &pair_table[i];
		memset(pair, 0, sizeof(*pair));
		pair->a.other = &pair->b;
		pair->a.pair = pair;
		pair->b.other = &pair->a;
		pair->b.pair = pair;
		pair->a.char_length = 10 * FACTOR;
		pair->b.char_length = 10 * FACTOR;
		spin_lock_init(&pair->spin);
		init_waitqueue_head(&pair->control_lines_wait);
		dprintf("%s - initialized pair %d -> %p\n", __FUNCTION__, i, pair);
	}

	/* allocate the tty driver */
	nullmodem_tty_driver = alloc_tty_driver(NULLMODEM_PAIRS*2);
	if (!nullmodem_tty_driver)
		return -ENOMEM;

	/* initialize the tty driver */
	nullmodem_tty_driver->owner = THIS_MODULE;
	nullmodem_tty_driver->driver_name = "nullmodem";
	nullmodem_tty_driver->name = "nmp";
	/* no more devfs subsystem */
	nullmodem_tty_driver->major = NULLMODEM_MAJOR;
	nullmodem_tty_driver->type = TTY_DRIVER_TYPE_SERIAL;
	nullmodem_tty_driver->subtype = SERIAL_TYPE_NORMAL;
	nullmodem_tty_driver->flags = TTY_DRIVER_RESET_TERMIOS | TTY_DRIVER_REAL_RAW ;
	/* no more devfs subsystem */
	nullmodem_tty_driver->init_termios = tty_std_termios;
	nullmodem_tty_driver->init_termios.c_iflag = 0;
	nullmodem_tty_driver->init_termios.c_oflag = 0;
	nullmodem_tty_driver->init_termios.c_cflag = B38400 | CS8 | CREAD;
	nullmodem_tty_driver->init_termios.c_lflag = 0;
	nullmodem_tty_driver->init_termios.c_ispeed = 38400;
	nullmodem_tty_driver->init_termios.c_ospeed = 38400;

	tty_set_operations(nullmodem_tty_driver, &serial_ops);

	/* register the tty driver */
	retval = tty_register_driver(nullmodem_tty_driver);
	if (retval)
	{
		printk(KERN_ERR "failed to register nullmodem tty driver\n");
		put_tty_driver(nullmodem_tty_driver);
		return retval;
	}

	last_timer_jiffies = jiffies;
	nullmodem_timer.expires = last_timer_jiffies + TIMER_INTERVAL;
	add_timer(&nullmodem_timer);

	printk(KERN_INFO DRIVER_DESC " " DRIVER_VERSION "\n");
	return retval;
}

static void __exit nullmodem_exit(void)
{
	int i;

	dprintf("%s - \n", __FUNCTION__);

	del_timer_sync(&nullmodem_timer);

	for (i = 0; i < NULLMODEM_PAIRS*2; ++i)
		tty_unregister_device(nullmodem_tty_driver, i);
	tty_unregister_driver(nullmodem_tty_driver);

	/* shut down all of the timers and free the memory */
	for (i = 0; i < NULLMODEM_PAIRS; ++i)
	{
		pair_table[i].a.tty = NULL;
		pair_table[i].b.tty = NULL;
	}
}

module_init(nullmodem_init);
module_exit(nullmodem_exit);
