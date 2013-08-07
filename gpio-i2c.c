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
#include <linux/mutex.h>

#define GPIO1 17
#define GPIO2 27
#define GPIO3 23
#define GPIO4 24

#define GPIO_SDA GPIO3
#define GPIO_SCL GPIO4
#define GPIO_RESERVED1 GPIO1
#define GPIO_RESERVED2 GPIO2

#define MAX_I2C 32

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
    u8 numbytes;
    u8 extcmd;
    struct mutex lock;
};


/* --- setting states on the bus with the right timing: ---------------	*/

#define UDELAY 2
#define TIMEOUT 500

static void setsda(int val)
{
    if (val)
        gpio_direction_input(GPIO_SDA);
    else
        gpio_direction_output(GPIO_SDA, 0);
}

static void setscl(int val)
{
    if (val)
        gpio_direction_input(GPIO_SCL);
    else
	    gpio_direction_output(GPIO_SCL, 0);
}

static int getsda(void)
{
	int ret;
//	gpio_direction_input(GPIO_SDA);
	ret = gpio_get_value(GPIO_SDA);
	return ret;
}

/*
static int getscl(void)
{
	int ret;
	gpio_direction_input(GPIO_SCL);
	ret = gpio_get_value(GPIO_SCL);
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
	//unsigned long start;

	setscl(1);
/*
	start = jiffies;
	while (!getscl()) {
		if (time_after(jiffies, start + TIMEOUT)) {
			if (getscl())
				break;
			return -ETIMEDOUT;
		}
		cpu_relax();
	}
    */
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
    udelay(2*UDELAY);
    /* read ack */
	ack = !getsda();
   // ack = 1;
	scllo();
	return ack;
}


static int i2c_inb(struct i2c_adapter *i2c_adap)
{
	/* read byte via i2c port, without start/stop sequence	*/
	/* acknowledge is sent in i2c_read.			*/
	int i;
	unsigned char indata = 0;

	/* assert: scl is low */
	sdahi();
	for (i = 0; i < 8; i++) {
		sclhi();
        indata *= 2;
		if (getsda())
			indata |= 0x01;
		setscl(0);
		udelay(UDELAY/2);
	}
	/* assert: scl is low */
	return indata;
}

/* ----- Utility functions
 */

/* try_address tries to contact a chip for a num of
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
		//yield();
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

	retries = 0;

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
static ssize_t show_test(struct device *dev, struct device_attribute *attr, char *buf)
{
    int ret;
    struct i2c_adapter *adap = to_i2c_adapter(dev);
    struct gpio_data *data = i2c_get_adapdata(adap);
	u8 regaddr = data->reg;	

	struct i2c_msg msg = {
		.addr	= data->addr,
		.len	= 1,
		.buf	= &regaddr,
	};

    mutex_lock(&data->lock);

	ret = sprintf(buf, "%d\n", i2c_transfer(adap, &msg, 1));

    mutex_unlock(&data->lock);

    return ret;
}

static DEVICE_ATTR(test, S_IRUGO, show_test, NULL);

static ssize_t show_addr(struct device *dev, struct device_attribute *attr, char *buf)
{
    int ret;
	struct i2c_adapter *adap = to_i2c_adapter(dev);
	struct gpio_data *data = i2c_get_adapdata(adap);

    mutex_lock(&data->lock);

    ret = sprintf(buf, "0x%.02x\n", data->addr);

    mutex_unlock(&data->lock);

	return ret;
}

static ssize_t set_addr(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_adapter *adap = to_i2c_adapter(dev);
	struct gpio_data *data = i2c_get_adapdata(adap);
	u8 addr;
	int error;
	
    mutex_lock(&data->lock);
	
	error = kstrtou8(buf, 0, &addr);
	data->addr = addr;

    mutex_unlock(&data->lock);

	return count;
}

static DEVICE_ATTR(addr, S_IWUSR | S_IRUGO, show_addr, set_addr);

static ssize_t show_extcmd(struct device *dev, struct device_attribute *attr, char *buf)
{
    int ret;
    struct i2c_adapter *adap = to_i2c_adapter(dev);
    struct gpio_data *data = i2c_get_adapdata(adap);

    mutex_lock(&data->lock);

    ret = sprintf(buf, "0x%.02x\n", data->extcmd);
    
    mutex_unlock(&data->lock);
    
    return ret;
}

static ssize_t set_extcmd(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    struct i2c_adapter *adap = to_i2c_adapter(dev);
    struct gpio_data *data = i2c_get_adapdata(adap);
    u8 addr;
    int error;

    mutex_lock(&data->lock);

    error = kstrtou8(buf, 0, &addr);
    data->addr = addr;

    mutex_unlock(&data->lock);
    
    return count;
}

static DEVICE_ATTR(extcmd, S_IWUSR | S_IRUGO, show_extcmd, set_extcmd);

static ssize_t show_reg(struct device *dev, struct device_attribute *attr, char *buf)
{
    int ret;
	struct i2c_adapter *adap = to_i2c_adapter(dev);
	struct gpio_data *data = i2c_get_adapdata(adap);

    mutex_lock(&data->lock);
   
    ret = sprintf(buf, "0x%.02x\n", data->reg);
    
    mutex_unlock(&data->lock);

	return ret;
}

static ssize_t set_reg(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_adapter *adap = to_i2c_adapter(dev);
	struct gpio_data *data = i2c_get_adapdata(adap);
	u8 reg;
	int error;
	
    mutex_lock(&data->lock);

	error = kstrtou8(buf, 0, &reg);
	data->reg = reg;

    mutex_unlock(&data->lock);

	return count;
}

static DEVICE_ATTR(reg, S_IWUSR | S_IRUGO, show_reg, set_reg);

static ssize_t show_data(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_adapter *adap = to_i2c_adapter(dev);
	struct gpio_data *data = i2c_get_adapdata(adap);
	unsigned char i2c_buf[1];
	u8 regaddr = data->reg;	
	int ret;	
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

    mutex_lock(&data->lock);

	ret = i2c_transfer(adap, msgs, 2);

    mutex_unlock(&data->lock);

	if (ret < 2) {
		return sprintf(buf, "-1");
	}
	else return sprintf(buf, "0x%.02x\n", i2c_buf[0]);	
}

static ssize_t set_data(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_adapter *adap = to_i2c_adapter(dev);
	struct gpio_data *data = i2c_get_adapdata(adap);
	u8 val, i2c_buf[2];
	int error, ret;	
	struct i2c_msg msg = {
		.addr	= data->addr,
		//.flags	= I2C_M_IGNORE_NAK,
		.len	= 2,
		.buf	= i2c_buf,
	};

    mutex_lock(&data->lock);

	error = kstrtou8(buf, 0, &val);
	if (error) {
        mutex_unlock(&data->lock);
		return error;
    }

	i2c_buf[0] = data->reg;
	i2c_buf[1] = val;

	ret = i2c_transfer(adap, &msg, 1);

    mutex_unlock(&data->lock);

	if (ret != 1) {
		return -EIO;
	}
	return count;
}

static DEVICE_ATTR(data, S_IWUSR | S_IRUGO, show_data, set_data);

static ssize_t show_srbytes(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_adapter *adap = to_i2c_adapter(dev);
	struct gpio_data *data = i2c_get_adapdata(adap);
	int ret, i, index = 0;	
    unsigned char i2c_buf[MAX_I2C];
    unsigned char strbuf[256];
	
    struct i2c_msg msgs[1] = {
		{
			.addr	= data->addr,
			.flags	= I2C_M_RD,
			.len	= data->numbytes,
			.buf	= i2c_buf,
		}
	};

    mutex_lock(&data->lock);
    
	ret = i2c_transfer(adap, msgs, ARRAY_SIZE(msgs));

    mutex_unlock(&data->lock);
    for (i = 0; i < data->numbytes; i++) 
	    index += sprintf(strbuf+index, "0x%.02x ", i2c_buf[i]);
    if (ret < ARRAY_SIZE(msgs)) {
		return sprintf(buf, "-1\n");
	}
	else return sprintf(buf, "%s\n", strbuf);	
}

static ssize_t set_srbytes(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_adapter *adap = to_i2c_adapter(dev);
	struct gpio_data *data = i2c_get_adapdata(adap);
	u8 val, i2c_buf[MAX_I2C];
    char byte[9];
	int error, ret, i, n;

	struct i2c_msg msg = {
		.addr	= data->addr,
		.flags	= I2C_M_IGNORE_NAK,
		.len	= data->numbytes+1,
		.buf	= i2c_buf,
	};
	
    mutex_lock(&data->lock);

    for (i = 0; i < data->numbytes; i++) {
        sscanf(buf, " %s%n", byte, &n);
        buf += n;
        error = kstrtou8(byte, 0, &val);
        if (error) {
            mutex_unlock(&data->lock);
            return error;
        }
        i2c_buf[0] = val;
    }
	
	ret = i2c_transfer(adap, &msg, 1);

    mutex_unlock(&data->lock);

	if (ret != 1) {
		return -EIO;
	}
    return count;
}

static DEVICE_ATTR(srbytes, S_IWUSR | S_IRUGO, show_srbytes, set_srbytes);

static ssize_t show_wrbytes(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_adapter *adap = to_i2c_adapter(dev);
	struct gpio_data *data = i2c_get_adapdata(adap);
	int ret, i, index = 0;	
    unsigned char i2c_buf[MAX_I2C];
    unsigned char strbuf[256];
	u8 regaddr = data->reg;	
	
    struct i2c_msg msgs[2] = {
        {
            .addr   = data->addr,
            .len    = 1,
            .buf    = &regaddr,
        }, {
			.addr	= data->addr,
			.flags	= I2C_M_RD,
			.len	= data->numbytes,
			.buf	= i2c_buf,
		}
	};

    mutex_lock(&data->lock);
    
	ret = i2c_transfer(adap, msgs, ARRAY_SIZE(msgs));

    mutex_unlock(&data->lock);
    for (i = 0; i < data->numbytes; i++) 
	    index += sprintf(strbuf+index, "0x%.02x ", i2c_buf[i]);
    if (ret < ARRAY_SIZE(msgs)) {
		return sprintf(buf, "-1\n");
	}
	else return sprintf(buf, "%s\n", strbuf);	
}

static ssize_t set_wrbytes(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_adapter *adap = to_i2c_adapter(dev);
	struct gpio_data *data = i2c_get_adapdata(adap);
	u8 val, i2c_buf[MAX_I2C+1];
    char byte[9];
	int error, ret, i, n;

	struct i2c_msg msg = {
		.addr	= data->addr,
		.flags	= I2C_M_IGNORE_NAK,
		.len	= data->numbytes+1,
		.buf	= i2c_buf,
	};
	
    mutex_lock(&data->lock);

    i2c_buf[0] = data->reg;

    for (i = 0; i < data->numbytes; i++) {
        sscanf(buf, " %s%n", byte, &n);
        buf += n;
        error = kstrtou8(byte, 0, &val);
        if (error) {
            mutex_unlock(&data->lock);
            return error;
        }
        i2c_buf[i+1] = val;
    }
	
	ret = i2c_transfer(adap, &msg, 1);

    mutex_unlock(&data->lock);

	if (ret != 1) {
		return -EIO;
	}
	return count;
}

static DEVICE_ATTR(wrbytes, S_IWUSR | S_IRUGO, show_wrbytes, set_wrbytes);

static ssize_t show_extwrbytes(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_adapter *adap = to_i2c_adapter(dev);
	struct gpio_data *data = i2c_get_adapdata(adap);
	int ret, i, index = 0;	
    unsigned char i2c_buf[MAX_I2C];
    unsigned char strbuf[256];
    u8 reg_buf[2];
	

    struct i2c_msg msgs[2] = {
        {
            .addr   = data->addr,
            .len    = 2,
            .buf    = reg_buf,
        }, {
			.addr	= data->addr,
			.flags	= I2C_M_RD,
			.len	= data->numbytes,
			.buf	= i2c_buf,
		}
	};

    reg_buf[0] = data->reg;
    reg_buf[1] = data->extcmd;
    
    mutex_lock(&data->lock);
    
	ret = i2c_transfer(adap, msgs, ARRAY_SIZE(msgs));

    mutex_unlock(&data->lock);
    for (i = 0; i < data->numbytes; i++) 
	    index += sprintf(strbuf+index, "0x%.02x ", i2c_buf[i]);
    if (ret < ARRAY_SIZE(msgs)) {
		return sprintf(buf, "-1\n");
	}
	else return sprintf(buf, "%s\n", strbuf);	
}

static ssize_t set_extwrbytes(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    struct i2c_adapter *adap = to_i2c_adapter(dev);
    struct gpio_data *data = i2c_get_adapdata(adap);
    u8 val, i2c_buf[MAX_I2C+1];
    char byte[9];
    int error, ret, i, n;

    struct i2c_msg msg = {
        .addr   = data->addr,
        .flags  = I2C_M_IGNORE_NAK,
        .len    = data->numbytes+2,
        .buf    = i2c_buf,
    };

    mutex_lock(&data->lock);

    i2c_buf[0] = data->reg;
    i2c_buf[1] = data->extcmd;

    for (i = 0; i < data->numbytes; i++) {
        sscanf(buf, " %s%n", byte, &n);
        buf += n;
        error = kstrtou8(byte, 0, &val);
        if (error) {
            mutex_unlock(&data->lock);
            return error;
        }
        i2c_buf[i+2] = val;
    }

    ret = i2c_transfer(adap, &msg, 1);

    mutex_unlock(&data->lock);

    if (ret != 1) {
        return -EIO;
    }
    return count;
}

static DEVICE_ATTR(extwrbytes, S_IWUSR | S_IRUGO, show_extwrbytes, set_extwrbytes);

static const struct gpio i2c_gpios[] __initconst_or_module = {
    {
        .gpio   = GPIO_SDA,
        .flags  = GPIOF_OUT_INIT_HIGH,
        .label  = "gpio-sda",
    },
    {
        .gpio   = GPIO_SCL,
        .flags  = GPIOF_OUT_INIT_HIGH,
        .label  = "gpio-scl",
    },
    {
        .gpio   = GPIO_RESERVED1,
        .label  = "Reserved-i2c",
    },
    {
        .gpio   = GPIO_RESERVED2,
        .label  = "Reserved-i2c",
    },
};

static ssize_t show_lock(struct device *dev, struct device_attribute *attr, char *buf)
{
    int ret;
    struct i2c_adapter *adap = to_i2c_adapter(dev);
    struct gpio_data *data = i2c_get_adapdata(adap);

    mutex_lock(&data->lock);

    if (gpio_request_array(i2c_gpios, ARRAY_SIZE(i2c_gpios)))
        ret = sprintf(buf, "%d\n", 0);
    else ret = sprintf(buf, "%d\n", 1);

    mutex_unlock(&data->lock);

    return ret;
}

static ssize_t set_lock(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    struct i2c_adapter *adap = to_i2c_adapter(dev);
    struct gpio_data *data = i2c_get_adapdata(adap);

    mutex_lock(&data->lock);

    gpio_free_array(i2c_gpios, ARRAY_SIZE(i2c_gpios));

    mutex_unlock(&data->lock);

    return count;
}

static DEVICE_ATTR(lock, S_IWUSR | S_IRUGO, show_lock, set_lock);

static ssize_t set_numbytes(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    struct i2c_adapter *adap = to_i2c_adapter(dev);
    struct gpio_data *data = i2c_get_adapdata(adap);
    u8 val;
    int error;

    mutex_lock(&data->lock);
    
    error = kstrtou8(buf, 0, &val);
    if (error) {
        mutex_unlock(&data->lock);
        return error;
    }

    data->numbytes = val;

    mutex_unlock(&data->lock);

    return count;
}

static DEVICE_ATTR(numbytes, S_IWUSR | S_IRUGO, NULL, set_numbytes);

static struct attribute *gpio_attributes[] = {
    &dev_attr_test.attr,
    &dev_attr_addr.attr,
    &dev_attr_reg.attr,
    &dev_attr_data.attr,
    &dev_attr_wrbytes.attr,
    &dev_attr_lock.attr,
    &dev_attr_numbytes.attr,
    &dev_attr_srbytes.attr,
    &dev_attr_extwrbytes.attr,
    &dev_attr_extcmd.attr,
    NULL
};

static const struct attribute_group gpio_group = {
    .attrs = gpio_attributes,
};

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
    int err;
	struct gpio_data *data;
   
    printk("gpio-i2c init\n");
	
    i2c_add_adapter(&gpio_adapter);
	data = devm_kzalloc(&gpio_adapter.dev, sizeof(struct gpio_data), GFP_KERNEL);
	if(!data)
		return -ENOMEM;

    mutex_init(&data->lock);

	i2c_set_adapdata(&gpio_adapter, data);

    err = sysfs_create_group(&gpio_adapter.dev.kobj, &gpio_group);
    if (err < 0)
        return err;

	return 0;
}

static void __exit gpio_exit(void)
{
    printk("gpio-i2c exit\n");
	i2c_del_adapter(&gpio_adapter);
}

module_init(gpio_init);
module_exit(gpio_exit);

MODULE_DESCRIPTION("I2C GPIO Bitbanging driver for RPi");
MODULE_LICENSE("GPL");
