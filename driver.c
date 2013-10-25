/* http://www.linuxchix.org/content/courses/kernel_hacking/lesson5
 * http://www.linuxjournal.com/article/8110?page=0,1
 * http://tuomasnylund.fi/drupal6/content/making-embedded-linux-kernel-device-driver
 * http://www.linuxjournal.com/article/6930
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/proc_fs.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/sched.h>
#include <asm/uaccess.h>
#include <linux/io.h>
#include <linux/errno.h>
#include <linux/ioport.h>
#include <asm/io.h>
#include <mach/hardware.h>
#include <linux/spi/spi.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/fcntl.h>
#include <linux/syscalls.h>

MODULE_LICENSE("Dual BSD/GPL");
#define MODULE_NAME "mcp2515-fen"
// ------------------------
// NEW SPI PART

typedef struct{
	struct spi_device *spidev;
} spi_data;

spi_data spidata;

static int spidriver_resume(struct spi_device *spi){
	// No implementation
	return 0;
}

static int spidriver_suspend(struct spi_device *spi, pm_message_t state){
	// No implementation
	return 0;
}

static int spidriver_remove(struct spi_device *spi){
	// No implementation
	return 0;
}

static int __init spidriver_probe(struct spi_device *dev){
	int ret = 0;


	printk(KERN_INFO "SPI: Probe started\n");

	spidata.spidev = dev;
	dev_set_drvdata(&dev->dev, &spidata); // gebruikt om data in te laden en te gebruiken in de spi struct functies
	dev_info(&dev->dev, "spi registered");

	return ret;
} 

static struct spi_driver spi_mcp2515_driver = {
	.driver = {
		.name = "spi-mcp2515",
		.bus = &spi_bus_type, // line 87
		.owner = THIS_MODULE,
	},
	.probe = spidriver_probe,
	.remove = spidriver_remove,
	.suspend = spidriver_suspend,
	.resume = spidriver_resume,
};

int initSPI(void){
	int ret = 0;
	ret = spi_register_driver(&spi_mcp2515_driver);
	if(ret)
		printk(KERN_ALERT "SPI: Problem with spi_register_driver\n");
	return ret;
}

int exitSPI(void){

}

// -------------------------
// CAN PART

// ----- Register
#define EXIDE 3
#define TXB0SIDH 0x31
#define TXB0SIDL 0x32
#define CANSTATUS 0x0E
#define TXB0EID0 0x34
#define RXF0SIDH 0x00
#define TXB0EID8 0x3
#define CNF1 0x2A
#define CNF2 0x29
#define CNF3 0x28
// ----- Commando's
#define RESET 0b11000000
#define WRITE 0x02
#define READ 0x03
#define GARBAGE 0x00
#define BIT_MODIFY 0x05

int writeCANRegister(__u8 reg, __u8 value){
	__u8 transfer[3] = {WRITE, reg, value};
	if(spi_write(spidata.spidev, transfer, 3) == -1) return 0;
	return 1;
}

__u8 readCANRegister(__u8 reg){
	__u8 transfer[3] = {READ, reg, GARBAGE};
	__u8 register_value[3] = {0x00, 0x00, 0x00};
	spi_write(spidata.spidev, transfer, 3);
	spi_read(spidata.spidev, register_value, 3);
	return register_value[2];
}

void initCAN(){
	writeSPIData(RESET, 1);
	msleep(10);
	__u8 can_status = readCANRegister(CANSTATUS) >> 5;
	if(can_status != 0b100)
		printk("CAN: Failed CAN INIT\n");
	else
		printk("CAN: Succeeded CAN INIT\n");

	return;
}
// -------------------------
// COMBINED PART

static struct proc_dir_entry *procfs_dir, *procfs_status_entry;

int init(void){
	printk("Init module mcp2515-fen\n");

	//aanmaken van een map in procfs
	procfs_dir = proc_mkdir(MODULE_NAME, NULL);
	if(procfs_dir == NULL){
		printk(KERN_ALERT "Could not make procfs_dir\n");
		return -1;
	}

	initSPI();
	initCAN();

	return;
}

void exit(void){
	printk(KERN_INFO "Remov module mcp2515-fen\n");
	remove_proc_entry("status", procfs_dir);
	remove_proc_entry(MODULE_NAME, NULL);

	exitSPI();
}

module_init(init);
module_exit(exit);