/*
* created by wwj
*
*
*
*/
#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/string.h>
#include <linux/memory.h>
#include <linux/shenonmxc.h>

#define MAC_ID_ADDR_START 0x01
#define MAC_ID_SIZE		6

static struct i2c_client* ac24cxx_client;
static unsigned char mac_addr[6];

#ifdef CONFIG_RII_DDR_CLL
#include <linux/byteorder/generic.h>
#include <linux/io.h>
#define DEBUG(...)
#define EEPROM_DDR_START 0x30
static unsigned long ddr3_calibration_default[][2] = {
  {0x021b083c, 0x42180218},
  {0x021b0840, 0x02030203},
  {0x021b483c, 0x41670173},
  {0x021b4840, 0x0167016f},
  {0x021b0848, 0x484b4d49},
  {0x021b4848, 0x4a4b4d47},
  {0x021b0850, 0x3f3f3137},
  {0x021b4850, 0x33373930},
};

static void calibration_mmc (void)
{
	DEBUG("WWJ==%s====in\n", __FUNCTION__);
	int ret = 0, i, j;
	u8 data = 0;
	unsigned long parm = 0;
	int size = ARRAY_SIZE(ddr3_calibration_default);
	
	//printk("%s:array size is %d\n", __func__, size);
	for (i = 0; i < size; ++i){
		parm = 0;
			ret = i2c_smbus_read_i2c_block_data(ac24cxx_client, (EEPROM_DDR_START + i * 4), 4, &parm);
			//printk("%s: read data = 0x%x from 0x%x\n", __func__, parm, (EEPROM_DDR_START + i * 4));

			parm = ntohl(parm);

			//printk("%s:write %x to %x\n", __func__, parm, ddr3_calibration_default[i][0]);
		if(0xffffffff == parm){
			printk("%s:use default ddr params\n");
		}else
			__raw_writel(parm, ioremap(ddr3_calibration_default[i][0], 4));
		msleep(10);
#if 0
		if (data == 0xffffffff){
   			/* code */
   			DEBUG("write default ddr calibration reg:%x value:%x\n", ddr3_calibration_default[i][0], ddr3_calibration_default[i][1]);
   			//__raw_writel(ddr3_calibration_default[i][1], ioremap(ddr3_calibration_default[i][0], 4));
		} else {
			DEBUG("write new ddr calibration reg:%x value:%x\n",ddr3_calibration_default[i][0], data);
			DEBUG("  default ddr calibration reg:%x value:%x\n", ddr3_calibration_default[i][0], ddr3_calibration_default[i][1]);
			//__raw_writel(data, ioremap(ddr3_calibration_default[i][0], 4));
			printk("check-- %x:%x\n", data,  __raw_readl(ioremap(ddr3_calibration_default[i][0], 4)));
		}
#endif
	}
	/*
	for(i = 0; i < size; i++){
		printk("check-- %x\n",  __raw_readl(ioremap(ddr3_calibration_default[i][0], 4)));
	}
	*/
	DEBUG("MQ==%s====out\n", __FUNCTION__);

}
#endif

static int read_macid(struct i2c_client* client);
static ssize_t set_macid(struct device* dev, struct device_attribute* attr, const char* buf, size_t count)
{
    char macid[18];
    char* p = macid;
    char* mac;
    int i = 0;
    s32 ret;
    u8 addr;
    char tmp[5];

    memset(macid, 0, 18);
    strncpy(macid, buf, 18);
    macid[17] = '\0';

    i = 0;
    printk("macid = %s\n", p);
    while(mac = strsep(&p, ":")){
    	//printk("WWJ======mac = %s\n", mac);
    	mac_addr[i] = simple_strtoul(mac, NULL, 16);
    	i++;
    }
    printk("i = %d, i2c_addr = 0x%x\n", i, ac24cxx_client->addr);

    i = 0;
    for(addr = MAC_ID_ADDR_START; addr < (MAC_ID_ADDR_START + MAC_ID_SIZE); addr++){
    	printk("WWJ======== set MAC[%d]:0x%x; addr = 0x%x\n", i, mac_addr[i], addr);
    	ret = i2c_smbus_write_byte_data(ac24cxx_client, addr, mac_addr[i]);
    	if(ret < 0)
    		printk("i2c_smbus_write_byte_data failed ret = %d dev addr = 0x%x\n", ret, ac24cxx_client->addr);
    	msleep(100);
    	i++;
    }

    return count;
}

static ssize_t show_macid(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret;
	read_macid(ac24cxx_client);
		
        ret = sprintf(buf, "%02x:%02x:%02x:%02x:%02x:%02x", mac_addr[0], mac_addr[1],
        	mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
	return ret;
}

static DEVICE_ATTR(MAC_ADDR, 0600, show_macid, set_macid);

static int read_macid(struct i2c_client* client)
{
	u32 index = 0, i;
	int dat;
	for(i = MAC_ID_ADDR_START; i < (MAC_ID_ADDR_START+MAC_ID_SIZE); i++){
		//mac_addr[index] = i2c_smbus_read_byte_data(client, i);
		dat = i2c_smbus_read_byte_data(client, i);
		if(dat < 0)
			printk("error read dat = %d\n", dat);
		else
			mac_addr[index] = dat;
		msleep(100);
		printk("WWJ=======addr 0x%x = 0x%x\n", i, dat);
		index++;
	}
}

static int ac24cxx_probe(struct i2c_client *client,
				       const struct i2c_device_id *id)
{
	int ret;
	//int value;
	//int i;
	int retry;
	ac24cxx_client = client;
	//printk("WWJ=========%s start, i2c addr = 0x%x\n", __func__, client->addr);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_err(&client->dev, "i2c bus does not support the powermcu\n");
		return -ENODEV;
	}

	//check the eeprom existing
	retry = 3;
	while(retry--){
		if(i2c_smbus_read_byte_data(client, 0x00) >= 0)
			break;
		msleep(50);
	}
	if(retry <= 0)
		return -1;

#ifdef CONFIG_RII_DDR_CLL
	//calibration_mmc();
#endif

	ret = sysfs_create_file(&client->dev.kobj, &dev_attr_MAC_ADDR.attr);
    if(ret)
        printk("------------sysfs_create_file failed---------\n");

	read_macid(client);

	return 0;
}

static int ac24cxx_remove(struct i2c_client *client)
{

	//printk("WWJ=========%s start\n", __func__);

	return 0;
}

static const struct i2c_device_id ac24cxx_id[] = {
	{ "ac24cxx", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ac24cxx_id);

static struct of_device_id ac24cxx_ids[] = {
	{ .compatible = "ac24cxx" },
	{ /* sentinel */ }
};

static struct i2c_driver ac24cxx_driver = {
	.driver = {
		.name	= "ac24cxx",
		.owner	= THIS_MODULE,
		.of_match_table	= of_match_ptr(ac24cxx_ids),
	},
	.id_table	= ac24cxx_id,
	.probe		= ac24cxx_probe,
	.remove		= ac24cxx_remove,
};
module_i2c_driver(ac24cxx_driver);

MODULE_AUTHOR("ShenCloudTech, Inc.");
MODULE_DESCRIPTION("EEPROM for MACID and MEM parameters");
MODULE_LICENSE("GPL");
