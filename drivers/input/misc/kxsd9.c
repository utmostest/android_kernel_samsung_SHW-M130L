/*
 * kxsd9 accelerometer driver
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA  02110-1301, USA.
 */
#include <linux/fs.h>

#include <linux/irq.h>
#include <asm/system.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/types.h>
#include <linux/workqueue.h>
#include <linux/slab.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>

#define kxsd9_NAME "kxsd9"

#define MSEXT_WAIT_MEASURE_KXSD9 1 /* 200 micro-seconds */

#define MSEXT_KXSD9_RESOLUTION 819 /* [count/G] */

#define MSEXT_KXSD9_I2C_SLAVE_ADDRESS 0x18 /* 0x19 can be also assigned by ADDR */

#define MSEXT_KXSD9_CENTER 2048

#define MSEXT_KXSD9_CTRL_REGB (0x0d)
#define MSEXT_KXSD9_RESET_REG (0x0a)
#define MSEXT_KXSD9_RESET_KEY (0xca)

/* Default parameters */
#define kxsd9_DEFAULT_DELAY            100
#define kxsd9_MAX_DELAY                2000

#define kxsd9_BANDWIDTH_25HZ           0
#define kxsd9_BANDWIDTH_50HZ           1
#define kxsd9_BANDWIDTH_100HZ          2
#define kxsd9_BANDWIDTH_190HZ          3
#define kxsd9_BANDWIDTH_375HZ          4
#define kxsd9_BANDWIDTH_750HZ          5
#define kxsd9_BANDWIDTH_1500HZ         6

/* filter setting */
#define KXSD9_FILTER_LEN		4
#define KXSD9_STABLE_TH		10      // s1*3 (256:819)
/* ioctl commnad */
#define DCM_IOC_MAGIC			's'
#define KXSD9_CALIBRATION		_IOWR(DCM_IOC_MAGIC, 48, short)

/* Acceleration measurement */
struct acceleration {
	short x;
	short y;
	short z;
};

//////////////////
#define GSCALC_CFGFILE "/data/system/ms3c_yamaha_gc.cfg"
#define GSCALC_DATA_NUM           (20)
static int gCalculatorIndex = 0;
static int gCalculatorNum = 0;
static struct acceleration gCalculatorData[GSCALC_DATA_NUM] ;
static int FirstCal = 0;
static int unReadCnt = 0;

 short GB_offset_x = 0;
 short GB_offset_y = 0;
 short GB_offset_z = 0;

int kxsd9_readCalcData(short *offset_x, short *offset_y, short *offset_z);
/////////////////

/* Output data rate  */
struct kxsd9_odr {
	unsigned long delay;	/* min delay (msec) in the range of ODR */
	u8 odr;			/* bandwidth register value */
};

static const struct kxsd9_odr kxsd9_odr_table[] = {
/*	{  1, kxsd9_BANDWIDTH_1500HZ }, */
	{  2, kxsd9_BANDWIDTH_750HZ },
	{  3, kxsd9_BANDWIDTH_375HZ },
	{  6, kxsd9_BANDWIDTH_190HZ },
	{ 10, kxsd9_BANDWIDTH_100HZ },
	{ 20, kxsd9_BANDWIDTH_50HZ },
	{ 40, kxsd9_BANDWIDTH_25HZ },
};

struct kxsd9_fir_filter {
	int num;
	int filter_len;
	int index;
	int32_t sequence[KXSD9_FILTER_LEN];
};

/* driver private data */
struct kxsd9_data {
	atomic_t enable;                /* attribute value */
	atomic_t delay;                 /* attribute value */
	struct mutex enable_mutex;
	struct mutex data_mutex;
	struct i2c_client *client;
	struct input_dev *input;
	struct delayed_work work;
	struct miscdevice kxsd9_device;
	struct kxsd9_fir_filter filter[3];
};

#define delay_to_jiffies(d) ((d) ? msecs_to_jiffies(d) : 1)
#define actual_delay(d)     (jiffies_to_msecs(delay_to_jiffies(d)))

/* register access functions */
#define kxsd9_read_bits(p, r) \
	((i2c_smbus_read_byte_data((p)->client, r##_REG) \
	& r##_MASK) >> r##_SHIFT)

#define kxsd9_update_bits(p, r, v) \
	i2c_smbus_write_byte_data((p)->client, r##_REG, \
	((i2c_smbus_read_byte_data((p)->client, r##_REG) \
	& ~r##_MASK) | ((v) << r##_SHIFT)))

static void fir_filter_init(struct kxsd9_fir_filter *filter, int len)
{
	int i;

	filter->num = 0;
	filter->index = 0;
	filter->filter_len = len;

	for (i = 0; i < filter->filter_len; ++i)
		filter->sequence[i] = 0;
}

static s16 fir_filter_filter(struct kxsd9_fir_filter *filter, s16 in)
{
	int out = 0;
	int i;

	if (filter->filter_len == 0)
		return in;
	if (filter->num < filter->filter_len) {
		filter->sequence[filter->index++] = in;
		filter->num++;
		return in;
	} else {
		if (filter->filter_len <= filter->index)
			filter->index = 0;
		filter->sequence[filter->index++] = in;

		for (i = 0; i < filter->filter_len; i++)
			out += filter->sequence[i];
		return out / filter->filter_len;
	}
}

static void filter_init(struct kxsd9_data *kxsd9)
{
	int i;

	for (i = 0; i < 3; i++)
		fir_filter_init(&kxsd9->filter[i], KXSD9_FILTER_LEN);
}

static void filter_filter(struct kxsd9_data *kxsd9, s16 *orig, s16 *filtered)
{
	int i;

	for (i = 0; i < 3; i++)
		filtered[i] = fir_filter_filter(&kxsd9->filter[i], orig[i]);
}

static void filter_stabilizer(struct kxsd9_data *kxsd9,
					s16 *orig, s16 *stabled)
{
	int i;
	static s16 buffer[3] = { 0, };

	for (i = 0; i < 3; i++) {
		if ((buffer[i] - orig[i] >= KXSD9_STABLE_TH)
			|| (buffer[i] - orig[i] <= -KXSD9_STABLE_TH)) {
			stabled[i] = orig[i];
			buffer[i] = stabled[i];
		} else
			stabled[i] = buffer[i];
	}
}


/* Device dependant operations */
static int kxsd9_power_up(struct kxsd9_data *kxsd9)
{
	int err;
	unsigned char data;

	data = i2c_smbus_read_byte_data(kxsd9->client, MSEXT_KXSD9_CTRL_REGB);
    data |= 0x40;	
	/* wait 10us after soft_reset to start I2C transaction */
	msleep(1);
	err = i2c_smbus_write_byte_data(kxsd9->client, MSEXT_KXSD9_CTRL_REGB, data);
	/* wait 10us after soft_reset to start I2C transaction */
	msleep(1);
	
	return 0;
}

static int kxsd9_power_down(struct kxsd9_data *kxsd9)
{
	int err;
	unsigned char data;

	data = i2c_smbus_read_byte_data(kxsd9->client, MSEXT_KXSD9_CTRL_REGB);
    data &= 0xBF;	
	/* wait 10us after soft_reset to start I2C transaction */
	msleep(1);
	
	err = i2c_smbus_write_byte_data(kxsd9->client, MSEXT_KXSD9_CTRL_REGB, data);
	/* wait 10us after soft_reset to start I2C transaction */
	msleep(1);
	
	return 0;
}

static int kxsd9_hw_init(struct kxsd9_data *kxsd9)
{
	int err;
	
	kxsd9_power_up(kxsd9);
	err = i2c_smbus_write_byte_data(kxsd9->client, MSEXT_KXSD9_RESET_REG, MSEXT_KXSD9_RESET_KEY);
	/* wait 10us after soft_reset to start I2C transaction */
	msleep(1);
	
	kxsd9_power_down(kxsd9);
	
	return 0;
}

static int kxsd9_get_enable(struct device *dev)
{
	struct kxsd9_data *kxsd9 = dev_get_drvdata(dev);
	return atomic_read(&kxsd9->enable);
}

static void kxsd9_set_enable(struct device *dev, int enable)
{
	struct kxsd9_data *kxsd9 = dev_get_drvdata(dev);
	int delay = atomic_read(&kxsd9->delay);

	mutex_lock(&kxsd9->enable_mutex);
	if (enable) { /* enable if state will be changed */
		if (!atomic_cmpxchg(&kxsd9->enable, 0, 1)) {
			kxsd9_power_up(kxsd9);
			schedule_delayed_work(&kxsd9->work,
					      delay_to_jiffies(delay) + 1);
		}
	} else { /* disable if state will be changed */
		if (atomic_cmpxchg(&kxsd9->enable, 1, 0)) {
			cancel_delayed_work_sync(&kxsd9->work);
			kxsd9_power_down(kxsd9);
		}
	}
	atomic_set(&kxsd9->enable, enable);
	mutex_unlock(&kxsd9->enable_mutex);
}

static int kxsd9_get_delay(struct device *dev)
{
	struct kxsd9_data *kxsd9 = dev_get_drvdata(dev);
	return atomic_read(&kxsd9->delay);
}

static void kxsd9_set_delay(struct device *dev, int delay)
{
	struct kxsd9_data *kxsd9 = dev_get_drvdata(dev);
	int i;
	u8 odr;
#if 1
	/* determine optimum ODR */
	for (i = 1; (i < ARRAY_SIZE(kxsd9_odr_table)) &&
		     (actual_delay(delay) >= kxsd9_odr_table[i].delay); i++)
		;
	odr = kxsd9_odr_table[i-1].odr;
	atomic_set(&kxsd9->delay, delay);

	mutex_lock(&kxsd9->enable_mutex);
	if (kxsd9_get_enable(dev)) {
		cancel_delayed_work_sync(&kxsd9->work);
		//kxsd9_update_bits(kxsd9, kxsd9_BANDWIDTH, odr);
		schedule_delayed_work(&kxsd9->work,
				      delay_to_jiffies(delay) + 1);
	} else {
		kxsd9_power_up(kxsd9);
		//kxsd9_update_bits(kxsd9, kxsd9_BANDWIDTH, odr);
		kxsd9_power_down(kxsd9);
	}
	mutex_unlock(&kxsd9->enable_mutex);
#endif
}

static int kxsd9_measure(struct kxsd9_data *kxsd9,
				struct acceleration *accel)
{
	struct i2c_client *client = kxsd9->client;
	int err;
	int i;
	s16 raw_data[3];
	s16 filtered_data[3];
	s16 stabled_data[3];
	u8 buf[6];
    short v[3];

    if((!FirstCal)&&(unReadCnt++<5))
    {
        // Load Calc data
        if(!kxsd9_readCalcData(&GB_offset_x,&GB_offset_y,&GB_offset_z))
        {
            FirstCal = 1;
        }
    }
    
	/* read acceleration raw data */
	err = i2c_smbus_read_i2c_block_data(client,
			0, sizeof(buf), buf) != sizeof(buf);
	if (err < 0) {
		pr_err("%s: i2c read fail addr=0x%02x, len=%d\n",
			__func__, 0x00, sizeof(buf));
		for (i = 0; i < 3; i++)
			raw_data[i] = 0;
	} 
	else
	{
		raw_data[0] = (buf[0] << 4) | (buf[1] >> 4);
		raw_data[1] = (buf[2] << 4) | (buf[3] >> 4);
		raw_data[2] = (buf[4] << 4) | (buf[5] >> 4);
	}
	/* filter out sizzle values */
	/* filter_filter(kxsd9, raw_data, filtered_data); */
	/* filter_stabilizer(kxsd9, filtered_data, stabled_data); */
	filter_stabilizer(kxsd9, raw_data, stabled_data);

	accel->x  = -(stabled_data[0] - MSEXT_KXSD9_CENTER - GB_offset_x);
	accel->y  = -(stabled_data[1] - MSEXT_KXSD9_CENTER - GB_offset_y);
	accel->z  = (stabled_data[2] - MSEXT_KXSD9_CENTER - GB_offset_z);

	/*
	v[0] = (((data[0]) - MSEXT_KXSD9_CENTER)*0x1000)/MSEXT_KXSD9_RESOLUTION;
	v[1] = (((data[1]) - MSEXT_KXSD9_CENTER)*0x1000)/MSEXT_KXSD9_RESOLUTION;
	v[2] = (((data[2]) - MSEXT_KXSD9_CENTER)*0x1000)/MSEXT_KXSD9_RESOLUTION;

	//v /= MSEXT_KXSD9_RESOLUTION;
	accel->x  = v[0];
	accel->y  = v[1];
	accel->z  = v[2];
	*/
	return err;
}

static int kxsd9_measure_cal(struct kxsd9_data *kxsd9,
				struct acceleration *accel)
{
	struct i2c_client *client = kxsd9->client;
	int err;
	int i;
	s16 raw_data[3];
	s16 filtered_data[3];
	s16 stabled_data[3];
	u8 buf[6];
    short v[3];

	/* read acceleration raw data */
	err = i2c_smbus_read_i2c_block_data(client,
			0, sizeof(buf), buf) != sizeof(buf);
	if (err < 0) {
		pr_err("%s: i2c read fail addr=0x%02x, len=%d\n",
			__func__, 0x00, sizeof(buf));
		for (i = 0; i < 3; i++)
			raw_data[i] = 0;
	} 
	else
	{
		raw_data[0] = (buf[0] << 4) | (buf[1] >> 4);
		raw_data[1] = (buf[2] << 4) | (buf[3] >> 4);
		raw_data[2] = (buf[4] << 4) | (buf[5] >> 4);
	}
	/* filter out sizzle values */
	filter_filter(kxsd9, raw_data, filtered_data);
	filter_stabilizer(kxsd9, filtered_data, stabled_data);

	accel->x  = (stabled_data[0] - MSEXT_KXSD9_CENTER);
	accel->y  = (stabled_data[1] - MSEXT_KXSD9_CENTER);
	accel->z  = (stabled_data[2] - MSEXT_KXSD9_CENTER);

	return err;
}

static void kxsd9_work_func(struct work_struct *work)
{
	struct kxsd9_data *kxsd9 = container_of((struct delayed_work *)work,
						  struct kxsd9_data, work);
	struct acceleration accel;
	unsigned long delay = delay_to_jiffies(atomic_read(&kxsd9->delay));

	kxsd9_measure(kxsd9, &accel);

	input_report_rel(kxsd9->input, REL_X, accel.x);
	input_report_rel(kxsd9->input, REL_Y, accel.y);
	input_report_rel(kxsd9->input, REL_Z, accel.z);
	input_sync(kxsd9->input);

	schedule_delayed_work(&kxsd9->work, delay);
}

/* Input device interface */
static int kxsd9_input_init(struct kxsd9_data *kxsd9)
{
	struct input_dev *dev;
	int err;

	dev = input_allocate_device();
	if (!dev)
		return -ENOMEM;
   	dev->name = "accelerometer_sensor";
	dev->id.bustype = BUS_I2C;

	input_set_capability(dev, EV_REL, REL_RY);
	input_set_capability(dev, EV_REL, REL_X);
	input_set_capability(dev, EV_REL, REL_Y);
	input_set_capability(dev, EV_REL, REL_Z);
	input_set_drvdata(dev, kxsd9);

	err = input_register_device(dev);
	if (err < 0) {
		input_free_device(dev);
		return err;
	}
	kxsd9->input = dev;

	return 0;
}

static void kxsd9_input_fini(struct kxsd9_data *kxsd9)
{
	struct input_dev *dev = kxsd9->input;

	input_unregister_device(dev);
	input_free_device(dev);
}

/* sysfs device attributes */
static ssize_t kxsd9_enable_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", kxsd9_get_enable(dev));
}

static ssize_t kxsd9_enable_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	int err;
	unsigned long enable;
	err = strict_strtoul(buf, 10, &enable);

	if ((enable == 0) || (enable == 1))
		kxsd9_set_enable(dev, enable);

	return count;
}

static ssize_t kxsd9_delay_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", kxsd9_get_delay(dev));
}

static ssize_t kxsd9_delay_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	int err;
	long delay;

	err = strict_strtoul(buf, 10, &delay);
	if (err < 0)
		return count;

	if (delay > kxsd9_MAX_DELAY)
		delay = kxsd9_MAX_DELAY;

	kxsd9_set_delay(dev, delay);

	return count;
}

static ssize_t kxsd9_wake_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct input_dev *input = to_input_dev(dev);
	static atomic_t serial = ATOMIC_INIT(0);

	input_report_rel(input, REL_RY, atomic_inc_return(&serial));

	return count;
}

static ssize_t kxsd9_data_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct kxsd9_data *kxsd9 = input_get_drvdata(input);
	struct acceleration accel;
	int on;

	mutex_lock(&kxsd9->data_mutex);
	on = kxsd9_get_enable(dev);
	if (!on)
		kxsd9_set_enable(dev, 1);
	kxsd9_measure(kxsd9, &accel);
	if (!on)
		kxsd9_set_enable(dev, 0);
	mutex_unlock(&kxsd9->data_mutex);

	return sprintf(buf, "%d,%d,%d\n", accel.x, accel.y, accel.z);
}

static DEVICE_ATTR(enable, S_IRUGO|S_IWUSR|S_IWGRP,
		   kxsd9_enable_show, kxsd9_enable_store);
static DEVICE_ATTR(delay, S_IRUGO|S_IWUSR|S_IWGRP,
		   kxsd9_delay_show, kxsd9_delay_store);
static DEVICE_ATTR(wake, S_IWUSR|S_IWGRP,
		   NULL, kxsd9_wake_store);
static DEVICE_ATTR(data, S_IRUGO,
		   kxsd9_data_show, NULL);

static struct attribute *kxsd9_attributes[] = {
	&dev_attr_enable.attr,
	&dev_attr_delay.attr,
	&dev_attr_wake.attr,
	&dev_attr_data.attr,
	NULL
};

static struct attribute_group kxsd9_attribute_group = {
	.attrs = kxsd9_attributes
};

#if 0
static int kxsd9_detect(struct i2c_client *client, struct i2c_board_info *info)
{
	int id;

	id = i2c_smbus_read_byte_data(client, kxsd9_CHIP_ID_REG);
	if (id != kxsd9_CHIP_ID)
		return -ENODEV;

	return 0;
}
#endif
/////////////////////////////////

void gsStableData_reset()
{
    gCalculatorIndex = 0;
    gCalculatorNum = 0;
}

int gsStableData_add(struct acceleration *pdata)
{
    if (pdata == 0) {
        return -1;
    }
    gCalculatorData[gCalculatorIndex++] = *pdata;
    gCalculatorNum++;

    if (gCalculatorIndex >= GSCALC_DATA_NUM) {
        gCalculatorIndex = 0;
    }
    if (gCalculatorNum >= GSCALC_DATA_NUM) {
        gCalculatorNum = GSCALC_DATA_NUM;
    }
    return 0;
}

int gsStableData_get(struct acceleration *pdata)
{
    int avg[3], sum[3], sigma[3];
    int i;

    if (pdata == 0) {
        return 0;
    }
    if (gCalculatorNum < GSCALC_DATA_NUM) {
        return 0;
    }
    avg[0] = avg[1] = avg[2] = 0;
    for (i = 0; i < GSCALC_DATA_NUM; i++) {
        avg[0] += gCalculatorData[i].x;
        avg[1] += gCalculatorData[i].y;
        avg[2] += gCalculatorData[i].z;
    }
    avg[0] /= (int)GSCALC_DATA_NUM;
    avg[1] /= (int)GSCALC_DATA_NUM;
    avg[2] /= (int)GSCALC_DATA_NUM;

    sum[0] = sum[1] = sum[2] = 0;
    for (i = 0; i < GSCALC_DATA_NUM; i++) {
        sum[0] = ((int)gCalculatorData[i].x - avg[0])
            *((int)gCalculatorData[i].x - avg[0]);
        sum[1] = ((int)gCalculatorData[i].y - avg[1])
            *((int)gCalculatorData[i].y - avg[1]);
        sum[2] = ((int)gCalculatorData[i].z - avg[2])
            *((int)gCalculatorData[i].z - avg[2]);
    }
    sigma[0] = (int)int_sqrt((int)(sum[0] / ((int)GSCALC_DATA_NUM - 1)));
    sigma[1] = (int)int_sqrt((int)(sum[1] / ((int)GSCALC_DATA_NUM - 1)));
    sigma[2] = (int)int_sqrt((int)(sum[2] / ((int)GSCALC_DATA_NUM - 1)));

    if (sigma[0] < 5 && sigma[1] < 5 && sigma[2] < 5) {
printk("gsStableData_get OK! %d %d %d\n",sigma[0],sigma[1],sigma[2]);        
        pdata->x = (short) avg[0];
        pdata->y = (short) avg[1];
        pdata->z = (short) avg[2];
        return 1;
    }
    return 0;
}

int kxsd9_readCalcData(short *offset_x, short *offset_y, short *offset_z)
{
	struct file *fp      = NULL;
    char buffer[50];
    int tx=0, ty=0, tz=0;
    
    fp = filp_open(GSCALC_CFGFILE, O_RDONLY, 0);
    if(IS_ERR(fp)||(fp==NULL))
    {
        printk("accel kxsd9_readCalcData error\n");
        return 1;
    }
    else
    {
        kernel_read(fp, fp->f_pos, buffer, 49);
		sscanf(buffer,"%d,%d,%d",&tx,&ty,&tz);
        *offset_x = (short)tx;
        *offset_y = (short)ty;
        *offset_z = (short)tz;
        printk("accel cal saved data %d, %d, %d\n",*offset_x,*offset_y,*offset_z);
    }

    if(fp)
		filp_close(fp, NULL);

    return 0;

}


int kxsd9_saveCalcData(short offset_x, short offset_y, short offset_z)
{
	struct file *fp      = NULL;
    int  xx, yy, zz;
	mm_segment_t oldfs;
  
    fp = filp_open(GSCALC_CFGFILE, O_WRONLY, 0);
    if(IS_ERR(fp))// the file is not exist
    {
        fp = filp_open(GSCALC_CFGFILE, O_RDWR | O_CREAT, 0660);
    }

    oldfs = get_fs();
	set_fs(get_ds());
    
    if(IS_ERR(fp)||(fp==NULL))
    {
        printk("accel kxsd9_saveCalcData error\n");
    }
    else
    {
        if(fp->f_mode & FMODE_WRITE)
    	{
    	    int sz;
            char buffer[50]   = {1};
			sprintf(buffer,"%d,%d,%d",offset_x,offset_y,offset_z);
			sz = fp->f_op->write(fp, (const char *)buffer, sizeof(buffer), &fp->f_pos);
            printk("accel cal save data %d, %d, %d\n",offset_x,offset_y,offset_z);

		}
    }
    
    set_fs(oldfs);
    
    if(fp)
		filp_close(fp, NULL);

            kxsd9_readCalcData(&xx,&yy,&zz);

    return 0;
    
}


int kxsd9_calibrate(struct kxsd9_data *kxsd9,
			struct acceleration orientation, int *tries)
{
	int need_calibration = 0;
	int on;
    int index;
    struct acceleration gRawdata;
    short savX, savY, savZ;

    gsStableData_reset();
    savX = GB_offset_x;
    savY = GB_offset_y;
    savZ = GB_offset_z;
	on = kxsd9_get_enable(&kxsd9->client->dev);
	if (!on)
		kxsd9_set_enable(&kxsd9->client->dev, 1);

    for (index = 0; index < GSCALC_DATA_NUM+1; index++) {
        if (kxsd9_measure_cal(kxsd9, &gRawdata) < 0) 
		{			
			printk("Ms3_DrvGsMeasure return fail \n");
			return 11;
        }
        GB_offset_x -= (savX-gRawdata.x)/GSCALC_DATA_NUM;//2048 - gRawdata.x;
	    GB_offset_y -= (savY-gRawdata.y)/GSCALC_DATA_NUM;//2048 - gRawdata.y;
    	GB_offset_z -= (savZ-(gRawdata.z-819)/GSCALC_DATA_NUM);//(2048+819) - gRawdata.z; 

		//LOGD("Ms3_MeasureAndSave_GsOffset()  gRawdata = (%d, %d, %d)\n", gRawdata.x[0],gRawdata.x[1],gRawdata.x[2]);
        gsStableData_add(&gRawdata);
        if (gsStableData_get(&gRawdata)) {
            break;
        }
        msleep(50);
    }
	printk("Ms3_MeasureAndSave_GsOffset() sum Rawdata => gRawdata = (%d, %d, %d)\n", gRawdata.x,gRawdata.y,gRawdata.z);
	/*
    if (Ms3_GsCalibGetGsOffset(&gdata, &goffset) < 0) {		
		LOGD("Ms3_GsCalibGetGsOffset return fail \n");
		return 12;
    }*/
	GB_offset_x = gRawdata.x;//2048 - gRawdata.x;
	GB_offset_y = gRawdata.y;//2048 - gRawdata.y;
	GB_offset_z = gRawdata.z-819;//(2048+819) - gRawdata.z;
printk("offset (%d, %d, %d)\n", GB_offset_x,GB_offset_y,GB_offset_z);
    
	if (!on)
		kxsd9_set_enable(&kxsd9->client->dev, 0);

    kxsd9_saveCalcData(GB_offset_x, GB_offset_y, GB_offset_z);
    
	return !need_calibration;
}

////////////////////////////////////

static int kxsd9_open(struct inode *inode, struct file *file)
{
	struct kxsd9_data *kxsd9 = container_of(file->private_data,
						struct kxsd9_data,
						kxsd9_device);
	file->private_data = kxsd9;
    
	return 0;
}

static int kxsd9_close(struct inode *inode, struct file *file)
{
	return 0;
}

static int kxsd9_ioctl(struct inode *inode, struct file *file,
			unsigned int cmd, unsigned long arg)
{
	struct kxsd9_data *kxsd9 = file->private_data;
	int try;
	int err = 0;
	unsigned char data[6];

	switch (cmd) {
	case KXSD9_CALIBRATION:
		if (copy_from_user((struct acceleration *)data,
			(struct acceleration *)arg, 6) != 0) {
			pr_err("copy_from_user error\n");
			return -EFAULT;
		}
		/* iteration time = 20 */
		try = GSCALC_DATA_NUM;
		err = kxsd9_calibrate(kxsd9,
				*(struct acceleration *)data, &try);
		break;
	default:
		break;
	}
	return 0;
}

static const struct file_operations kxsd9_fops = {
	.owner = THIS_MODULE,
	.open = kxsd9_open,
	.release = kxsd9_close,
	.ioctl = kxsd9_ioctl,
};

static int kxsd9_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct kxsd9_data *kxsd9;
	int err;

	/* setup private data */
	kxsd9 = kzalloc(sizeof(struct kxsd9_data), GFP_KERNEL);
	if (!kxsd9)
		return -ENOMEM;
	mutex_init(&kxsd9->enable_mutex);
	mutex_init(&kxsd9->data_mutex);

	/* setup i2c client */
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err = -ENODEV;
		goto err_i2c_fail;
	}
	i2c_set_clientdata(client, kxsd9);
	kxsd9->client = client;

#if 0
	/* detect and init hardware */
	err = kxsd9_detect(client, NULL);
	if (err)
		goto err_id_read;
	dev_info(&client->dev, "%s found\n", id->name);
	dev_info(&client->dev, "al_version=%d, ml_version=%d\n",
		 kxsd9_read_bits(kxsd9, kxsd9_AL_VERSION),
		 kxsd9_read_bits(kxsd9, kxsd9_ML_VERSION));
#endif
	kxsd9_hw_init(kxsd9);
	kxsd9_set_delay(&client->dev, kxsd9_DEFAULT_DELAY);

	/* setup driver interfaces */
	INIT_DELAYED_WORK(&kxsd9->work, kxsd9_work_func);

	err = kxsd9_input_init(kxsd9);
	if (err < 0)
		goto err_input_allocate;

	err = sysfs_create_group(&kxsd9->input->dev.kobj,
					&kxsd9_attribute_group);
	if (err < 0)
		goto err_sys_create;

	kxsd9->kxsd9_device.minor = MISC_DYNAMIC_MINOR;
	kxsd9->kxsd9_device.name = "accelerometer";
	kxsd9->kxsd9_device.fops = &kxsd9_fops;

	err = misc_register(&kxsd9->kxsd9_device);
	if (err) {
		pr_err("%s: misc_register failed\n", __FILE__);
		goto err_misc_register;
	}

	/* filter init */
	filter_init(kxsd9);

	return 0;

err_misc_register:
	sysfs_remove_group(&kxsd9->input->dev.kobj,
				&kxsd9_attribute_group);
err_sys_create:
	kxsd9_input_fini(kxsd9);
err_input_allocate:
err_id_read:
err_i2c_fail:
	kfree(kxsd9);
	return err;
}

static int kxsd9_remove(struct i2c_client *client)
{
	struct kxsd9_data *kxsd9 = i2c_get_clientdata(client);

	kxsd9_set_enable(&client->dev, 0);

	sysfs_remove_group(&kxsd9->input->dev.kobj, &kxsd9_attribute_group);
	kxsd9_input_fini(kxsd9);
	kfree(kxsd9);

	return 0;
}

static int kxsd9_suspend(struct device *dev)
{
	struct kxsd9_data *kxsd9 = dev_get_drvdata(dev);

	mutex_lock(&kxsd9->enable_mutex);
	if (kxsd9_get_enable(dev)) {
		cancel_delayed_work_sync(&kxsd9->work);
		kxsd9_power_down(kxsd9);
	}
	mutex_unlock(&kxsd9->enable_mutex);

	return 0;
}

static int kxsd9_resume(struct device *dev)
{
	struct kxsd9_data *kxsd9 = dev_get_drvdata(dev);
	int delay = atomic_read(&kxsd9->delay);

	kxsd9_hw_init(kxsd9);
	kxsd9_set_delay(dev, delay);

	mutex_lock(&kxsd9->enable_mutex);
	if (kxsd9_get_enable(dev)) {
		kxsd9_power_up(kxsd9);
		schedule_delayed_work(&kxsd9->work,
				      delay_to_jiffies(delay) + 1);
	}
	mutex_unlock(&kxsd9->enable_mutex);

	return 0;
}

static const struct dev_pm_ops kxsd9_pm_ops = {
	.suspend = kxsd9_suspend,
	.resume = kxsd9_resume,
};

static const struct i2c_device_id kxsd9_id[] = {
	{kxsd9_NAME, 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, kxsd9_id);

struct i2c_driver kxsd9_driver = {
	.driver = {
		.name = "kxsd9",
		.owner = THIS_MODULE,
		.pm = &kxsd9_pm_ops,
	},
	.probe = kxsd9_probe,
	.remove = kxsd9_remove,
	.id_table = kxsd9_id,
};

static int __init kxsd9_init(void)
{
	return i2c_add_driver(&kxsd9_driver);
}
module_init(kxsd9_init);

static void __exit kxsd9_exit(void)
{
	i2c_del_driver(&kxsd9_driver);
}
module_exit(kxsd9_exit);

MODULE_DESCRIPTION("kxsd9 accelerometer driver");
MODULE_AUTHOR("Samsung Electronics");
MODULE_LICENSE("GPL");
MODULE_VERSION(1.0.0);
