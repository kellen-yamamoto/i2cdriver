#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>


/* 
 * Need to dynamically add "clients" to expose
 * All I2C addresses to be written to without device 
 * drivers needed.  This keeps the module loadable.
 */
static struct i2c_client *client3;
static struct i2c_client *client4;

static struct i2c_board_info __initdata gpio_i2c_board_info3[] = {
	{
		I2C_BOARD_INFO("addr_0x03", 0x03),
	},
};

static struct i2c_board_info __initdata gpio_i2c_board_info4[] = {
	{
		I2C_BOARD_INFO("addr_0x04", 0x04),
	},
};

/* Client Data */
struct gpio_data {
	struct mutex	update_lock;
	u8		orig_conf;
	char		valid;
	unsigned long	last_updated;
	u8		addr;
};

/* Access Functions */
static int gpio_read_value(struct i2c_client *client, uint8_t reg, uint8_t *data, size_t n);
static int gpio_write_value(struct i2c_client *client, uint8_t reg, uint8_t value);

/*------------ Sysfs attribute interface ------------*/

static ssize_t show_addr(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct gpio_data *data = i2c_get_clientdata(client);
	return sprintf(buf, "%d\n", data->addr);
}

static ssize_t set_addr(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct gpio_data *data = i2c_get_clientdata(client);
	
	u8 addr;
	int error;
	
	error = kstrtou8(buf, 10, &addr);
	if(error)
		return error;
	data->addr = addr;
	printk("Wrote %d to addr\n", addr);
	return count;
}

static DEVICE_ATTR(addr, S_IWUSR | S_IRUGO, show_addr,  set_addr);

static ssize_t show_data(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct gpio_data *data = i2c_get_clientdata(client);
	
	uint8_t i2cdata;
  	
	gpio_read_value(client, data->addr, &i2cdata, 1);
	
	return sprintf(buf, "%d\n", i2cdata);
}

static ssize_t set_data(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct gpio_data *data = i2c_get_clientdata(client);
	
	u8 val;
	int error;
	
	error = kstrtou8(buf, 10, &val);
	if(error)
		return error;
	
	gpio_write_value(client, data->addr, val);
	return count;
}

static DEVICE_ATTR(data, S_IWUSR | S_IRUGO, show_data, set_data);


/*--------------- register access ----------------*/

static int gpio_read_value(struct i2c_client *client, uint8_t reg, uint8_t *data, size_t n)
{
	struct i2c_msg msgs[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = 1,
			.buf = data,
		}, {
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = n,
			.buf = data,
		}
	};
	
	int ret;
	
	data[0] = reg;
	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret != ARRAY_SIZE(msgs)) {
		dev_err(&client->dev, "%s: read error, ret=%d\n",
			__func__, ret);
		return -EIO;
	}
		
	return 0;	
	
}

static int gpio_write_value(struct i2c_client *client, uint8_t reg, uint8_t value)
{
	uint8_t data[2] = { reg, value };
	int err;
		
	err = i2c_master_send(client, data, sizeof(data));
	if (err != sizeof(data)) {
		dev_err(&client->dev, "%s: err=%d addr=%02x, data=%02x\n",
				__func__, err, data[0], data[1]);
		return -EIO;
	}
	
	return 0;	
}


/*---------------- Driver Setup and Cleanup -------------------*/

static int __init gpio_init(void)
{
	struct i2c_adapter *adapter;
	struct gpio_data *data;
	
  /* Find adapter at runtime */
  /* 
   * Using new adapter, so it should be 2 because the default 
   * Is adapter 1 for BCM2708 built in I2C hardware
   */
	adapter = i2c_get_adapter(2);

  /* 
   * Register available addresses to expose their sysfs interface for 
   * Use in userspace.  Also generate the sysfs attribute files to engage
   * I2C communication through file access userspace functions.
   */ 
	client3 = i2c_new_device(adapter, gpio_i2c_board_info3);
	data = devm_kzalloc(&client3->dev, sizeof(struct gpio_data), GFP_KERNEL);
	if(!data)
		return -ENOMEM;
	i2c_set_clientdata(client3, data);
	device_create_file(&client3->dev, &dev_attr_addr);
	device_create_file(&client3->dev, &dev_attr_data);
	
	client4 = i2c_new_device(adapter, gpio_i2c_board_info4);
	data = devm_kzalloc(&client4->dev, sizeof(struct gpio_data), GFP_KERNEL);
	if(!data)
		return -ENOMEM;
	i2c_set_clientdata(client4, data);
	device_create_file(&client4->dev, &dev_attr_addr);
	device_create_file(&client4->dev, &dev_attr_data);
	
	i2c_put_adapter(adapter);

	return 0;
}


static void __exit gpio_cleanup(void)
{
	i2c_unregister_device(client3);
}

module_init(gpio_init);
module_exit(gpio_cleanup);

MODULE_AUTHOR("Kellen Yamamoto");
MODULE_DESCRIPTION("GPIO i2c driver");
MODULE_LICENSE("GPL");


