/*
 * I2C Bit Banging Algorithm + Adapter 
 * Using Raspberry Pi GPIO
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/sched.h>
#include <linux/i2c.h>
#include <linux/i2c-algo-bit.h>
#include <linux/gpio.h>

#define GPIO1 17 
#define GPIO2 27

static int bit_test;	/* see if the line-setting functions work	*/
module_param(bit_test, int, S_IRUGO);
MODULE_PARM_DESC(bit_test, "lines testing - 0 off; 1 report; 2 fail if stuck");

#ifdef DEBUG
static int i2c_debug = 1;
module_param(i2c_debug, int, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(i2c_debug,
		 "debug level - 0 off; 1 normal; 2 verbose; 3 very verbose");
#endif

struct gpio_data {
	u8 addr;
	u8 reg;
};


/* --- setting states on the bus with the right timing: ---------------	*/

#define UDELAY 10
#define TIMEOUT 500

static void setsda(int val)
{
	gpio_direction_output(GPIO1, val);
}

static void setscl(int val)
{
	gpio_direction_output(GPIO2, val);
}

static int getsda(void)
{
	int ret;
	gpio_direction_input(GPIO1);
	ret = gpio_get_value(GPIO1);
	return ret;
}

/*
static int getscl(void)
{
	int ret;
	gpio_direction_input(GPIO2);
	ret = gpio_get_value(GPIO2);
	return ret;
}
*/

static inline void sdalo(void)
{
	setsda(0);
	udelay((UDELAY + 1) / 2);
}

static inline void sdahi(void)
{
	setsda(1);
	udelay((UDELAY + 1) / 2);
}

static inline void scllo(void)
{
	setscl(0);
	udelay(UDELAY / 2);
}

/*
 * Raise scl line, and do checking for delays. This is necessary for slower
 * devices.
 */
static int sclhi(void)
{
	setscl(1);

	udelay(UDELAY);
	return 0;
}


/* --- other auxiliary functions --------------------------------------	*/
static void i2c_start(void)
{
	/* assert: scl, sda are high */
	setsda(0);
	udelay(UDELAY);
	scllo();
}

static void i2c_repstart(void)
{
	/* assert: scl is low */
	sdahi();
	sclhi();
	setsda(0);
	udelay(UDELAY);
	scllo();
}


static void i2c_stop(void)
{
	/* assert: scl is low */
	sdalo();
	sclhi();
	setsda(1);
	udelay(UDELAY);
}



/* send a byte without start cond., look for arbitration,
   check ackn. from slave */
/* returns:
 * 1 if the device acknowledged
 * 0 if the device did not ack
 * -ETIMEDOUT if an error occurred (while raising the scl line)
 */
static int i2c_outb(struct i2c_adapter *i2c_adap, unsigned char c)
{
	int i;
	int sb;
	int ack;
	
	/* assert: scl is low */
	for (i = 7; i >= 0; i--) {
		
		sb = (c >> i) & 1;
		setsda(sb);
		udelay((UDELAY + 1) / 2);
		sclhi();
		scllo();
	}
	sdahi();
	sclhi();
	/* read ack */
	ack = !getsda();

	scllo();
	return ack;
}


static int i2c_inb(struct i2c_adapter *i2c_adap)
{
	/* read byte via i2c port, without start/stop sequence	*/
	/* acknowledge is sent in i2c_read.			*/
	int i;
	unsigned char indata = 0;
	struct i2c_algo_bit_data *adap = i2c_adap->algo_data;

	/* assert: scl is low */
	sdahi();
	for (i = 0; i < 8; i++) {
		indata *= 2;
		if (getsda())
			indata |= 0x01;
		setscl(0);
		udelay(i == 7 ? UDELAY / 2 : adap->udelay);
	}
	/* assert: scl is low */
	return indata;
}

/* ----- Utility functions
 */

/* try_address tries to contact a chip for a number of
 * times before it gives up.
 * return values:
 * 1 chip answered
 * 0 chip did not answer
 * -x transmission error
 */
static int try_address(struct i2c_adapter *i2c_adap,
		       unsigned char addr, int retries)
{
	int i, ret = 0;

	for (i = 0; i <= retries; i++) {
		ret = i2c_outb(i2c_adap, addr);
		if (ret == 1 || i == retries)
			break;
		i2c_stop();
		udelay(UDELAY);
		yield();
		i2c_start();
	}
	return ret;
}

static int sendbytes(struct i2c_adapter *i2c_adap, struct i2c_msg *msg)
{
	const unsigned char *temp = msg->buf;
	int count = msg->len;

	unsigned short nak_ok = msg->flags & I2C_M_IGNORE_NAK;
	int retval;
	int wrcount = 0;
	while (count > 0) {
		retval = i2c_outb(i2c_adap, *temp);

		/* OK/ACK; or ignored NAK */
		if ((retval > 0) || (nak_ok && (retval == 0))) {
			count--;
			temp++;
			wrcount++;

		/* A slave NAKing the master means the slave didn't like
		 * something about the data it saw.  For example, maybe
		 * the SMBus PEC was wrong.
		 */
		} else if (retval == 0) {
			dev_err(&i2c_adap->dev, "sendbytes: NAK bailout.\n");
			return -EIO;

		/* Timeout; or (someday) lost arbitration
		 *
		 * FIXME Lost ARB implies retrying the transaction from
		 * the first message, after the "winning" master issues
		 * its STOP.  As a rule, upper layer code has no reason
		 * to know or care about this ... it is *NOT* an error.
		 */
		} else {
			dev_err(&i2c_adap->dev, "sendbytes: error %d\n",
					retval);
			return retval;
		}
	}
	return wrcount;
}

static int acknak(struct i2c_adapter *i2c_adap, int is_ack)
{
	/* assert: sda is high */
	if (is_ack)		/* send ack */
		setsda(0);
	udelay((UDELAY + 1) / 2);
	if (sclhi() < 0) {	/* timeout */
		dev_err(&i2c_adap->dev, "readbytes: ack/nak timeout\n");
		return -ETIMEDOUT;
	}
	scllo();
	return 0;
}

static int readbytes(struct i2c_adapter *i2c_adap, struct i2c_msg *msg)
{
	int inval;
	int rdcount = 0;	/* counts bytes read */
	unsigned char *temp = msg->buf;
	int count = msg->len;
	const unsigned flags = msg->flags;

	while (count > 0) {
		inval = i2c_inb(i2c_adap);
		if (inval >= 0) {
			*temp = inval;
			rdcount++;
		} else {   /* read timed out */
			break;
		}

		temp++;
		count--;

		/* Some SMBus transactions require that we receive the
		   transaction length as the first read byte. */
		if (rdcount == 1 && (flags & I2C_M_RECV_LEN)) {
			if (inval <= 0 || inval > I2C_SMBUS_BLOCK_MAX) {
				if (!(flags & I2C_M_NO_RD_ACK))
					acknak(i2c_adap, 0);
				dev_err(&i2c_adap->dev, "readbytes: invalid "
					"block length (%d)\n", inval);
				return -EPROTO;
			}
			/* The original count value accounts for the extra
			   bytes, that is, either 1 for a regular transaction,
			   or 2 for a PEC transaction. */
			count += inval;
			msg->len += inval;
		}


		if (!(flags & I2C_M_NO_RD_ACK)) {
			inval = acknak(i2c_adap, count);
			if (inval < 0)
				return inval;
		}
	}
	return rdcount;
}

/* doAddress initiates the transfer by generating the start condition (in
 * try_address) and transmits the address in the necessary format to handle
 * reads, writes as well as 10bit-addresses.
 * returns:
 *  0 everything went okay, the chip ack'ed, or IGNORE_NAK flag was set
 * -x an error occurred (like: -ENXIO if the device did not answer, or
 *	-ETIMEDOUT, for example if the lines are stuck...)
 */
static int bit_doAddress(struct i2c_adapter *i2c_adap, struct i2c_msg *msg)
{
	unsigned short flags = msg->flags;
	unsigned short nak_ok = msg->flags & I2C_M_IGNORE_NAK;

	unsigned char addr;
	int ret, retries;

	retries = nak_ok ? 0 : i2c_adap->retries;

	if (flags & I2C_M_TEN) {
		/* a ten bit address */
		addr = 0xf0 | ((msg->addr >> 7) & 0x06);
		/* try extended address code...*/
		ret = try_address(i2c_adap, addr, retries);
		if ((ret != 1) && !nak_ok)  {
			dev_err(&i2c_adap->dev,
				"died at extended address code\n");
			return -ENXIO;
		}
		/* the remaining 8 bit address */
		ret = i2c_outb(i2c_adap, msg->addr & 0xff);
		if ((ret != 1) && !nak_ok) {
			/* the chip did not ack / xmission error occurred */
			dev_err(&i2c_adap->dev, "died at 2nd address code\n");
			return -ENXIO;
		}
		if (flags & I2C_M_RD) {
			i2c_repstart();
			/* okay, now switch into reading mode */
			addr |= 0x01;
			ret = try_address(i2c_adap, addr, retries);
			if ((ret != 1) && !nak_ok) {
				dev_err(&i2c_adap->dev,
					"died at repeated address code\n");
				return -EIO;
			}
		}
	} else {		/* normal 7bit address	*/
		addr = msg->addr << 1;
		if (flags & I2C_M_RD)
			addr |= 1;
		if (flags & I2C_M_REV_DIR_ADDR)
			addr ^= 1;
		ret = try_address(i2c_adap, addr, retries);
		if ((ret != 1) && !nak_ok)
			return -ENXIO;
	}

	return 0;
}

static int bit_xfer(struct i2c_adapter *i2c_adap,
		    struct i2c_msg msgs[], int num)
{
	struct i2c_msg *pmsg;
	int i, ret;
	unsigned short nak_ok;

	i2c_start();
	for (i = 0; i < num; i++) {
		pmsg = &msgs[i];

		nak_ok = pmsg->flags & I2C_M_IGNORE_NAK;
		if (!(pmsg->flags & I2C_M_NOSTART)) {
			if (i) {
				i2c_repstart();
			}
			ret = bit_doAddress(i2c_adap, pmsg);
			if ((ret != 0) && !nak_ok) {
				goto bailout;
			}
		}
		if (pmsg->flags & I2C_M_RD) {
			/* read bytes into buffer*/
			ret = readbytes(i2c_adap, pmsg);
			if (ret < pmsg->len) {
				if (ret >= 0)
					ret = -EIO;
				goto bailout;
			}
		} else {
			/* write bytes from buffer */
			ret = sendbytes(i2c_adap, pmsg);
			if (ret < pmsg->len) {
				if (ret >= 0)
					ret = -EIO;
				goto bailout;
			}
		}
	}
	ret = i;

bailout:
	i2c_stop();

	return ret;
}

static u32 bit_func(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C | I2C_FUNC_NOSTART | I2C_FUNC_SMBUS_EMUL |
	       I2C_FUNC_SMBUS_READ_BLOCK_DATA |
	       I2C_FUNC_SMBUS_BLOCK_PROC_CALL |
	       I2C_FUNC_10BIT_ADDR | I2C_FUNC_PROTOCOL_MANGLING;
}

/*------------ SYSFS Interface -------------*/

static ssize_t show_addr(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_adapter *adap = to_i2c_adapter(dev);
	struct gpio_data *data = i2c_get_adapdata(adap);
	return sprintf(buf, "%d\n", data->addr);
}

static ssize_t set_addr(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_adapter *adap = to_i2c_adapter(dev);
	struct gpio_data *data = i2c_get_adapdata(adap);
	
	u8 addr;
	int error;
	
	error = kstrtou8(buf, 10, &addr);
	data->addr = addr;
	return count;
}

static DEVICE_ATTR(addr, S_IWUSR | S_IRUGO, show_addr, set_addr);

static ssize_t show_reg(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_adapter *adap = to_i2c_adapter(dev);
	struct gpio_data *data = i2c_get_adapdata(adap);
	return sprintf(buf, "%d\n", data->reg);
}

static ssize_t set_reg(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_adapter *adap = to_i2c_adapter(dev);
	struct gpio_data *data = i2c_get_adapdata(adap);
	
	u8 reg;
	int error;
	
	error = kstrtou8(buf, 10, &reg);
	data->reg = reg;
	return count;
}

static DEVICE_ATTR(reg, S_IWUSR | S_IRUGO, show_reg, set_reg);

static ssize_t show_data(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_adapter *adap = to_i2c_adapter(dev);
	struct gpio_data *data = i2c_get_adapdata(adap);
	unsigned char i2c_buf[1];
	u8 regaddr = data->reg;	
	struct i2c_msg msgs[2] = {
		{
			.addr	= data->addr,
			.len	= 1,
			.buf	= &regaddr,
		}, {
			.addr	= data->addr,
			.flags	= I2C_M_RD,
			.len	= 1,
			.buf	= i2c_buf,
		}
	};

	int ret;	
	ret = i2c_transfer(adap, msgs, 2);
	if (ret != 2) {
		return sprintf(buf, "Read Error\n");
	}
	else return sprintf(buf, "Read: %d\n", i2c_buf[0]);	
}

static ssize_t set_data(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_adapter *adap = to_i2c_adapter(dev);
	struct gpio_data *data = i2c_get_adapdata(adap);
	u8 i2c_buf[2];
	u8 val;
	int error, ret;	
	struct i2c_msg msg = {
		.addr	= data->addr,
		.flags	= I2C_M_IGNORE_NAK,
		.len	= 2,
		.buf	= i2c_buf,
	};


	error = kstrtou8(buf, 10, &val);
	if (error)
		return error;
	
	i2c_buf[0] = data->reg;
	i2c_buf[1] = val;

	ret = i2c_transfer(adap, &msg, 1);
	if (ret != 1) {
		return -EIO;
	}
	return count;
}

static DEVICE_ATTR(data, S_IWUSR | S_IRUGO, show_data, set_data);
		

/* -----exported algorithm data: ------------------------------	*/

const struct i2c_algorithm i2c_bit_algo = {
	.master_xfer	= bit_xfer,
	.functionality	= bit_func,
};


/*-----Adapter code---------------------------------------------*/

static struct i2c_adapter gpio_adapter = {
	.owner	= THIS_MODULE,
	.algo	= &i2c_bit_algo,
	.name	= "gpio adapter",
};

static int __init gpio_init(void)
{
	struct gpio_data *data;
	i2c_add_adapter(&gpio_adapter);
	data = devm_kzalloc(&gpio_adapter.dev, sizeof(struct gpio_data), GFP_KERNEL);
	if(!data)
		return -ENOMEM;
	i2c_set_adapdata(&gpio_adapter, data);
	device_create_file(&gpio_adapter.dev, &dev_attr_addr);
	device_create_file(&gpio_adapter.dev, &dev_attr_reg);
	device_create_file(&gpio_adapter.dev, &dev_attr_data);

	return 0;
}

static void __exit gpio_exit(void)
{
	i2c_del_adapter(&gpio_adapter);
}

module_init(gpio_init);
module_exit(gpio_exit);

MODULE_AUTHOR("Kellen Yamamoto");
MODULE_DESCRIPTION("I2C Bit Banging Algorithm/Adapter for GPIO");
MODULE_LICENSE("GPL");
