#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/ide.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/gpio.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/of_gpio.h>
#include <linux/semaphore.h>
#include <linux/timer.h>
#include <linux/i2c.h>
#include <linux/spi/spi.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/irq.h>
#include <linux/wait.h>
#include <linux/poll.h>

#include <asm/mach/map.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#include "imu102nreg.h"


/********************************************************************
Copyright © BDStar Navigation Co.,Ltd. 1998-2029. All rights reserved.
文件名		: imu102n.c
作者	  	: 李彦东
版本	   	: V1.0
描述	   	: IMU102N SPI驱动程序
硬件环境   	: IMU6ULL(arm Cortex-A7)
日志	   	: 初版V1.0 2020/01/13 
*********************************************************************/
#define IMU102N_CNT		1
#define IMU102N_NAME	"imu102n"


/* 中断IO描述结构体 */
struct imu_irqdesc {
	int gpio;								/* gpio */
	int irqnum;								/* 中断号     */
	atomic_t value;							/* 中断发生标志 */
	char name[10];							/* 名字 */
	irqreturn_t (*handler)(int, void *);	/* 中断服务函数 */
};

struct imu102n_dev {
	dev_t devid;							/* 设备号 	 */
	struct cdev cdev;						/* cdev 	*/
	struct class *class;					/* 类 		*/
	struct device *device;					/* 设备 	 */
	struct device_node	*nd; 				/* 设备节点 */
	struct device_node	*irq_nd; 
	int major;								/* 主设备号 */
	void *private_data;						/* 私有数据 		*/
	int cs_gpio;							/* 片选所使用的GPIO编号		*/
	int irq_gpio;
	struct imu_irqdesc imu_irqdesc;		


	u16 ID;
	u16 temp_low;				/* 温度低地址原始值 		*/
	u16 temp_high;				/* 温度高地址原始值 		*/

	u16 gyro_x_low;  			/* 陀螺仪X轴低地址原始值 	 */
	u16 gyro_x_high;   			/* 陀螺仪X轴高地址原始值 	 */
	u16 gyro_y_low;    	
	u16 gyro_y_high;   	
	u16 gyro_z_low;    	
	u16 gyro_z_high;   	

	u16 accel_x_low;   			/* 加速度计X轴低地址原始值 	*/
	u16 accel_x_high;  			/* 加速度计X轴高地址原始值 	*/
	u16 accel_y_low;   	
	u16 accel_y_high;  	
	u16 accel_z_low;   	
	u16 accel_z_high;  
	
	wait_queue_head_t r_wait;	/* 读等待队列头 */
};

static struct imu102n_dev imu102ndev;

/*
 * @description		: 打开设备
 * @param - inode 	: 传递给驱动的inode
 * @param - filp 	: 设备文件，file结构体有个叫做pr似有ate_data的成员变量
 * 					  一般在open的时候将private_data似有向设备结构体。
 * @return 			: 0 成功;其他 失败
 */
static int imu102n_open(struct inode *inode, struct file *filp)
{
	filp->private_data = &imu102ndev; /* 设置私有数据 */
	return 0;
}

/*
 * @description	: 从imu102n读取多个寄存器数据
 * @param - dev:  imu102n设备
 * @param - reg:  要读取的寄存器首地址
 * @param - val:  读取到的数据
 * @param - len:  要读取几个Byte
 * @return 		: 操作结果
 */
static u16 imu102n_read_regs(struct imu102n_dev *dev, u16 reg, void *buf, int len)
{
	int ret;
	u16 txdata[len];
	struct spi_message m;
	struct spi_transfer *t;
	struct spi_device *spi = (struct spi_device *)dev->private_data;

	gpio_set_value(dev->cs_gpio, 0);						/* 片选拉低，选中imu102n */
	t = kzalloc(sizeof(struct spi_transfer), GFP_KERNEL);	/* 申请内存 */

	/* 第1次，发送要读取的寄存地址 */
    txdata[0]  =  reg;	

	t->tx_buf = txdata;					
	t->len = 2;							/* 2个字节 */
	spi_message_init(&m);				/* 初始化spi_message */
	spi_message_add_tail(t, &m);		/* 将spi_transfer添加到spi_message队列 */
	ret = spi_sync(spi, &m);			/* 同步发送 */

	/* 第2次，读取数据 */
	txdata[0] = 0xffff;					/* 随便一个值，此处无意义 */
	t->rx_buf = buf;					/* 读取到的数据 */
	t->len = len;						/* 要读取的字节数 */
	spi_message_init(&m);				/* 初始化spi_message */
	spi_message_add_tail(t, &m);		/* 将spi_transfer添加到spi_message队列 */
	ret = spi_sync(spi, &m);			/* 同步发送 */

	kfree(t);							/* 释放内存 */
	gpio_set_value(dev->cs_gpio, 1);	/* 片选拉高，释放imu102n */

	return ret;
}


/*
 * @description	: 读取imu102n指定寄存器值，读取一个寄存器
 * @param - dev:  imu102n设备
 * @param - reg:  要读取的寄存器
 * @return 	  :   读取到的寄存器值
 */
static u16 imu102n_read_onereg(struct imu102n_dev *dev, u16 reg)
{
	u16 data = 0;
	u16 data_h, data_l;
	
	imu102n_read_regs(dev, reg, &data, 2);  //2表示2个Byte, 数据长度为16位

	// 高位和低位交换位置（MSB和LSB的问题）
	data_h =  data  & 0xff00;
	data_l =  data  & 0x00ff;	
	data =  (data_h >> 8) | (data_l << 8);	 
	return data;
}

/*
 * @description	: 读取imu102n的数据，读取原始数据，包括三轴陀螺仪、
 * 				: 三轴加速度计和内部温度。
 * @param - dev	: imu102n设备
 * @return 		: 无。
 */
void imu102n_readdata(struct imu102n_dev *dev)
{
	imu102n_read_onereg(&imu102ndev, PROD_ID);
	dev->ID = imu102n_read_onereg(&imu102ndev, PROD_ID);
	
	dev->temp_low = imu102n_read_onereg(&imu102ndev, TEMP_H);
	dev->temp_high = imu102n_read_onereg(&imu102ndev, TEMP_L);
	dev->gyro_x_low = imu102n_read_onereg(&imu102ndev, X_GYRO_H);
	dev->gyro_x_high = imu102n_read_onereg(&imu102ndev, X_GYRO_L);
	dev->gyro_y_low = imu102n_read_onereg(&imu102ndev, Y_GYRO_H);
	dev->gyro_y_high = imu102n_read_onereg(&imu102ndev, Y_GYRO_L);
	dev->gyro_z_low = imu102n_read_onereg(&imu102ndev, Z_GYRO_H);
	dev->gyro_z_high = imu102n_read_onereg(&imu102ndev, Z_GYRO_L);
	
	dev->accel_x_low  = imu102n_read_onereg(&imu102ndev, X_ACCL_H);
	dev->accel_x_high  = imu102n_read_onereg(&imu102ndev, X_ACCL_L);
	dev->accel_y_low  = imu102n_read_onereg(&imu102ndev, Y_ACCL_H);
	dev->accel_y_high  = imu102n_read_onereg(&imu102ndev, Y_ACCL_L);
	dev->accel_z_low  = imu102n_read_onereg(&imu102ndev, Z_ACCL_H);
	dev->accel_z_high  = imu102n_read_onereg(&imu102ndev, Z_ACCL_L);
}

/*
 * @description		: 从设备读取数据 
 * @param - filp 	: 要打开的设备文件(文件描述符)
 * @param - buf 	: 返回给用户空间的数据缓冲区
 * @param - cnt 	: 要读取的数据长度
 * @param - offt 	: 相对于文件首地址的偏移
 * @return 			: 读取的字节数，如果为负值，表示读取失败
 */
static ssize_t imu102n_read(struct file *filp, char __user *buf, size_t cnt, loff_t *off)
{
	u_int16_t data[15];
	long err = 0;
	int ret = 0;
	unsigned char value = 0;
	wait_queue_t wait; 						//分配等待队列
	
	struct imu102n_dev *dev = (struct imu102n_dev *)filp->private_data;

	/* 非阻塞访问 */ 
	if(filp->f_flags & O_NONBLOCK)		
	{  
		if(atomic_read(&imu102ndev.imu_irqdesc.value) == 0)
			goto read_data;
		else
			return -EAGAIN;
	}
	else
	{
		init_waitqueue_entry(&wait, current); 			/* 将当前进程添加到容器中 */
		//DECLARE_WAITQUEUE(wait, current); 			/* 定义等待队列 */
		if(atomic_read(&imu102ndev.imu_irqdesc.value) == 0)  /* 没有数据准备就绪，切换任务 */
		{	
			add_wait_queue(&dev->r_wait, &wait);		/* 将等待队列添加到队头 */
			__set_current_state(TASK_INTERRUPTIBLE);
			schedule(); 							
			set_current_state(TASK_RUNNING);	
			if(signal_pending(current)) {				/* 是否为信号引起的唤醒？*/
				ret = -ERESTARTSYS;
				goto wait_error;
			}

			
			value = atomic_read(&imu102ndev.imu_irqdesc.value); /* 读取数据就绪标志 */
			if (value) 	/* SPI数据准备就绪 */	
			{					
				remove_wait_queue(&dev->r_wait, &wait); 		/* 唤醒以后将等待队列移除 */
			
				ret = copy_to_user(buf, &value, sizeof(value));
				if(ret != 0)
					printk("copy_to_user error.\r\n");
				
				atomic_set(&dev->imu_irqdesc.value, 0);
			} else {
				goto data_error;
			}
			goto read_data;

			return 0;
			wait_error:
				printk("second remove_wait_queue.\r\n");
				remove_wait_queue(&dev->r_wait, &wait); /* 将等待队列移除 */
				return ret;
			
			data_error:
				return -EINVAL;
		}
		
	}	

read_data:
	imu102n_readdata(dev);
	data[0] = dev->temp_low;
	data[1] = dev->temp_high;
	data[2] = dev->gyro_x_low;
	data[3] = dev->gyro_x_high;
	data[4] = dev->gyro_y_low;
	data[5] = dev->gyro_y_high;
	data[6] = dev->gyro_z_low;
	data[7] = dev->gyro_z_high;

	data[8] = dev->accel_x_low;
	data[9] = dev->accel_x_high;
	data[10] = dev->accel_y_low;
	data[11] = dev->accel_y_high;
	data[12] = dev->accel_z_low;
	data[13] = dev->accel_z_high;
	data[14] = dev->ID;
	err = copy_to_user(buf, data, sizeof(data));
	return 0;		
}

/*
 * @description  : poll 函数，用于处理非阻塞访问
 * @param - filp : 要打开的设备文件(文件描述符) 
 * @param - wait : 等待列表(poll_table)
 * @return : 设备或者资源状态，
 */
static unsigned int imx6uirq_poll(struct file *filp, struct poll_table_struct *wait)
{
	unsigned int mask = 0;
	
	struct imu102n_dev *dev = (struct imu102n_dev *)filp->private_data;
	
	poll_wait(filp, &dev->r_wait, wait);

	if(atomic_read(&imu102ndev.imu_irqdesc.value)) 	/* 中断发生 */
	{ 
		atomic_set(&(imu102ndev.imu_irqdesc.value), 0);
		mask = POLLIN | POLLRDNORM; /* 可读 */
	}

	return mask;
}

/*
 * @description		: 关闭/释放设备
 * @param - filp 	: 要关闭的设备文件(文件描述符)
 * @return 			: 0 成功;其他 失败
 */
static int imu102n_release(struct inode *inode, struct file *filp)
{
	return 0;
}

/* imu102n操作函数 */
static const struct file_operations imu102n_ops = {
	.owner = THIS_MODULE,
	.open = imu102n_open,
	.read = imu102n_read,
	.poll = imx6uirq_poll,
	.release = imu102n_release,
};


/* @description		: 中断服务函数，	
 * @param - irq 	: 中断处理函数要相应的中断号 
 * @param - dev_id	: 可以为void*通用指针，需要与 request_irq 函数的 dev 参数保持一致。用于区分共享中断的不同设备。
 * @return 			: 中断执行结果
 */
static irqreturn_t imu102n_handler(int irq, void *dev_id)
{
	struct imu102n_dev *dev = (struct imu102n_dev *)dev_id;

	atomic_set(&(imu102ndev.imu_irqdesc.value), 1);
	wake_up_interruptible(&dev->r_wait);			/* 唤醒进程，参数为要唤醒的等待队列头 */

	return IRQ_RETVAL(IRQ_HANDLED);
}

static int imu102n_irqinit(void )
{
	char name[10];
	int ret = 0;

	printk("imu102n_irqinit. \r\n");

	imu102ndev.irq_nd = of_find_node_by_path("/imu102nirq");
	if (imu102ndev.irq_nd== NULL){
		printk("imu102nirq node not find!\r\n");
		return -EINVAL;
	} 

	/* 提取GPIO */
	imu102ndev.imu_irqdesc.gpio = of_get_named_gpio(imu102ndev.irq_nd ,"irq-gpio", 0);
	if (imu102ndev.imu_irqdesc.gpio < 0) {
		printk("can't get irq-gpio\r\n");
	}

	/* 初始化SPI中断所使用的IO，并且设置成中断模式 */
	memset(imu102ndev.imu_irqdesc.name, 0, sizeof(name));	/* 缓冲区清零 */
	sprintf(imu102ndev.imu_irqdesc.name, "imu102nirq");
	gpio_request(imu102ndev.imu_irqdesc.gpio, name);
	gpio_direction_input(imu102ndev.imu_irqdesc.gpio);	
	
	imu102ndev.imu_irqdesc.irqnum = irq_of_parse_and_map(imu102ndev.irq_nd, 0);  	/* 从设备树中获取设备号 */
	imu102ndev.imu_irqdesc.handler = imu102n_handler;

	ret = request_irq(imu102ndev.imu_irqdesc.irqnum, imu102ndev.imu_irqdesc.handler, 
	                 IRQF_TRIGGER_RISING, imu102ndev.imu_irqdesc.name, &imu102ndev);
	if(ret < 0)
	{
		printk("imu102n irq request failed!\r\n");
		return -EFAULT;
	}

	/* 初始化等待队列头 */
	init_waitqueue_head(&imu102ndev.r_wait);
	printk("imu102n_irqinit finished. \r\n");
	return 0;

}

/*
 * imu102n内部寄存器初始化函数 
 * @param  	: 无
 * @return 	: 无
 */
void imu102n_reginit(void)
{
	u16 value = 0;

	mdelay(200);
	imu102n_read_onereg(&imu102ndev, PROD_ID);
	value = imu102n_read_onereg(&imu102ndev, PROD_ID);
	printk("imu102n ID = %#X\r\n", value);	

	imu102n_read_onereg(&imu102ndev, PROD_ID);
	value = imu102n_read_onereg(&imu102ndev, PROD_ID);
	printk("imu102n ID = %#X\r\n", value);
}

 /*
  * @description     : spi驱动的probe函数，当驱动与
  *                    设备匹配以后此函数就会执行
  * @param - client  : spi设备
  * @param - id      : spi设备ID
  * 
  */	
static int imu102n_probe(struct spi_device *spi)
{
	int ret = 0;

	printk("imu102n_probe.\r\n");

	/* 1、构建设备号 */
	if (imu102ndev.major) {
		imu102ndev.devid = MKDEV(imu102ndev.major, 0);
		register_chrdev_region(imu102ndev.devid, IMU102N_CNT, IMU102N_NAME);
	} else {
		alloc_chrdev_region(&imu102ndev.devid, 0, IMU102N_CNT, IMU102N_NAME);
		imu102ndev.major = MAJOR(imu102ndev.devid);
	}

	/* 2、注册设备 */
	cdev_init(&imu102ndev.cdev, &imu102n_ops);
	cdev_add(&imu102ndev.cdev, imu102ndev.devid, IMU102N_CNT);

	/* 3、创建类 */
	imu102ndev.class = class_create(THIS_MODULE, IMU102N_NAME);
	if (IS_ERR(imu102ndev.class)) {
		return PTR_ERR(imu102ndev.class);
	}

	/* 4、创建设备 */
	imu102ndev.device = device_create(imu102ndev.class, NULL, imu102ndev.devid, NULL, IMU102N_NAME);
	if (IS_ERR(imu102ndev.device)) {
		return PTR_ERR(imu102ndev.device);
	}

	/* 获取设备树中cs片选信号，即匹配设备树中的节点 ecspi3:ecspi@02010000 */
	imu102ndev.nd = of_find_node_by_path("/soc/aips-bus@02000000/spba-bus@02000000/ecspi@02010000");
	if(imu102ndev.nd == NULL) {
		printk("ecspi3 node not find!\r\n");
		return -EINVAL;
	} 

	/* 5、 获取名字为cs-gpio(片选)的属性 */
	imu102ndev.cs_gpio = of_get_named_gpio(imu102ndev.nd, "cs-gpio", 0);
	if(imu102ndev.cs_gpio < 0) {
		printk("can't get cs-gpio");
		return -EINVAL;
	}

	/* 6、设置GPIO1_IO24为输出，并且输出高电平 */
	ret = gpio_direction_output(imu102ndev.cs_gpio, 1);
	if(ret < 0) {
		printk("can't set gpio!\r\n");
	}

	/*初始化spi_device */
	spi->mode = SPI_MODE_3;	/*MODE3*/
	spi_setup(spi);
	imu102ndev.private_data = spi; 		/* 设置私有数据 */

	imu102n_reginit();					/* 初始化imu102n内部寄存器 */
	
	ret = imu102n_irqinit();
	if(ret != 0)
	{
		printk("imu102n_irqinit failed.\r\n");	
	}
	return 0;
}


/*
 * @description     : spi驱动的remove函数，移除spi驱动的时候此函数会执行
 * @param - client 	: spi设备
 * @return          : 0，成功;其他负值,失败
 */
static int imu102n_remove(struct spi_device *spi)
{
	/* 删除设备 */
	cdev_del(&imu102ndev.cdev);
	unregister_chrdev_region(imu102ndev.devid, IMU102N_CNT);

	/* 注销掉类和设备 */
	device_destroy(imu102ndev.class, imu102ndev.devid);
	class_destroy(imu102ndev.class);
	return 0;
}

/* 传统匹配方式ID列表 */
static const struct spi_device_id imu102n_id[] = {
	{"BDStar-imu102n", 0},
	{}
};

/* 设备树匹配列表 */
static const struct of_device_id imu102n_of_match[] = {
	{ .compatible = "BDStar-imu102n" },
	{ }
};

/* SPI驱动结构体 */	
static struct spi_driver imu102n_driver = {
	.probe = imu102n_probe,
	.remove = imu102n_remove,
	.driver = {
			.owner = THIS_MODULE,
		   	.name = "imu102n",
		   	.of_match_table = imu102n_of_match, 
		   },
	.id_table = imu102n_id,
};
		   
/*
 * @description	: 驱动入口函数
 */
static int __init imu102n_init(void)
{
	return spi_register_driver(&imu102n_driver);
}

/*
 * @description	: 驱动出口函数
 */
static void __exit imu102n_exit(void)
{
	free_irq(imu102ndev.imu_irqdesc.irqnum, &imu102ndev);

	spi_unregister_driver(&imu102n_driver);
}

module_init(imu102n_init);
module_exit(imu102n_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("liyandong");



