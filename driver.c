/* http://www.linuxchix.org/content/courses/kernel_hacking/lesson5
 *
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
#include <linux/spi/spidev.h>
#include <linux/types.h>
#include <linux/delay.h>

MODULE_LICENSE("Dual BSD/GPL");
#define MODULE_NAME "mcp2515-fen"
// ------------------------
// SPI PART

#define MODE 0
#define DELAY 0
#define BPW 8
#define SPEED 5000
#define DEVICE "/dev/spidev0.0"

typedef struct{
	int device = 0;
	__u8 mode = 0xff;
	__u16 bitsPerWord = 0xffff;
	__u16 delay_usecs = 0x0000;
	__u32 speed = 0xffffffff;
} spi_device;

typedef struct{
	__u8* rx = NULL;
	__u8* tx = NULL;
	__u64 rx_size = 0;
} spi_data;

spi_data spidata;
spi_device spi;

int openSPI(char* devicename = DEVICE){
	if(spi.device > 0){
		printk(KERN_ALERT "SPI: SPI device already open, please close it.\n");
		return 2;
	}
	spi.device = open(devicename, O_RDWR);
	if(spi.device < 1){
		printk(KERN_ALERT "SPI: Couldn't open spi device\n");
		return 1;
	}
	return 0;
}

int setSPIDelay(__u16 usecs = DELAY){
	spi.delay_usecs = usecs;
	return 0;
}

int setSPISpeed(__u32 speed = SPEED){
	if(spi.device < 1){
		printk(KERN_ALERT "SPI: SPI Device not open yet.\n");
		return 1;
	}
	if(ioctl(spi.device, SPI_IOC_WR_MAX_SPEED_HZ, &speed) < 0){
		printk(KERN_ALERT "SPI: Failed setting Speed (WR)\n");
		return 2;
	}
	if(ioctl(spi.device, SPI_IOC_RD_MAX_SPEED_HZ, &speed) < 0){
		printk(KERN_ALERT "SPI: Failed setting Speed (RD)\n");
		return 2;
	}linux/types.h
	spi.speed = speed;
	return 0;
}

int setSPIBitsPerWord(__u16 bitsPerWord = BPW){
	if(spi.device < 1){
		printk(KERN_ALERT "SPI: SPI Device not open yet.\n");
		return 1;
	}
	if(ioctl(spi.device, SPI_IOC_WR_BITS_PER_WORD, &bitsPerWord) < 0){
		printk(KERN_ALERT "SPI: Failed setting Bits Per Word (WR)\n");
		return 2;
	}
	if(ioctl(spi.device, SPI_IOC_RD_BITS_PER_WORD, &bitsPerWord) < 0){
		printk(KERN_ALERT "SPI: Failed setting Bits Per Word (RD)\n");
		return 2;
	}
	spi.bitsPerWord = bitsPerWord;
	return 0;
}

int setSPIMode(__u8 mode = MODE){
	if(spi.device < 1){
		printk(KERN_ALERT "SPI: SPI Device not open yet.\n");
		return 1;
	}
	if(ioctl(spi.device, SPI_IOC_WR_MODE, &mode) < 0){
		printk(KERN_ALERT "SPI: Failed setting Mode (WR)\n");
		return 2;
	}
	if(ioctl(spi.device, SPI_IOC_RD_MODE, &mode) < 0){
		printk(KERN_ALERT "SPI: Failed setting Mode (RD)\n");
		return 2;
	}
	spi.mode = mode;
	return 0;
}

int closeSPI(void){
	if(spi.device < 1){
		printk(KERN_ALERT "SPI: SPI Device not open yet.\n");
		return 0; // Returns 0 because SPI is closed
	}
	close(spi.device);
	spi.device = 0;
	return 0;
}

int initSPI(void){
	if(openSPI() == 0){
		return (setSPIMode() | setSPISpeed() | setSPIBitsPerWord() | setSPIDelay());
	}else{
		return 1;
	}
}

int exitSPI(void){
	closeSPI();
	if(spidata.rx != NULL){
		kfree(spidata.rx);
		spidata.rx = NULL;
	}
}

__s64 writeSPIData(const char* data, __s64 maxSize){
// FULL DUPLEX
	if(spi.device < 1){
		printk(KERN_ALERT "SPI: SPI Device not open yet.\n");
		return -1;
	}
	if(maxSize == -1){
		const char* cur_ptr = data;
		maxSize++; // maxSize = 0
		while(*(cur_ptr) != '\0'){
			maxSize++;
			cur_ptr++;
		}
		if(maxSize == 0) return -1;
	}else if(maxSize < 0) return -1;

	spidata.tx = (__u8*) data;

	if(spidata.rx != NULL){
		kfree(spidata.rx);
		spidata.rx = NULL;
	}

	spidata.rx = (__u8*) kmalloc(maxSize, 1); // 1 = sizeof(__u8)
	spidata.rx_size = maxSize;

	struct spi_ioc_transfer tr;

	tr.tx_buf = (unsigned long)spidata.tx;
	tr.rx_buf = (unsigned long)spidata.rx;
	tr.len = maxSize;
	tr.delay_usecs = spi.delay_usecs;
	tr.speed_hz = spi.speed;
	tr.bits_per_word = spi.bitsPerWord;

	if((ioctl(spi.device, SPI_IOC_MESSAGE(1), &tr)) < 0){
		printk(KERN_ALERT "SPI: Failed sending.\n");
		return -1;
	}
	return maxSize;
}

__s64 readSPIData(char* data, __s64 maxSize){
	if(spi.device < 1){
		printk(KERN_ALERT "SPI: SPI Device not open yet.\n");
		return -1;
	}
	if( maxSize < 1) return -1; 
	if( spidata.rx_size == 0) return -1;

	__u64 size;
	if( spidata.rx_size > maxSize){
		size = maxSize;
		spidata.rx_size -= maxSize;
	}else{
		size = spidata.rx_size;
		spidata.rx_size = 0;
	}

	__u64 i;
	for(i = 0; i < size; i++){
		data[i] = *(rx + i);
	}

	if(spidata.rx_size != 0){
		__u8* new_rx = (__u8*) kmalloc(spidata.rx_size, 1); // sizeof(__u8) = 1
		__u64 teller = 0;

		for(i = size; i < spidata.rx_size; i++){
			*(new_rx + teller) = *(spidata.rx + i);
			teller++;
		}
		kfree(spidata.rx);<
		spidata.rx = new_rx;
	}else{
		kfree(spidata.rx);
		spidata.rx = NULL;
	}
	return size;
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
	if(writeSPIData(transfer, 3) == -1) return 0;
	return 1;
}

int readCANRegister(__u8 reg){
	__u8 transfer[3] = {READ, reg, GARBAGE};
	__u8 register_value[3] = {0x00, 0x00, 0x00};
	writeSPIData(transfer, 3);
	readSPIData(register_value, 3);
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