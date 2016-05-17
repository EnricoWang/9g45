/* This is a demo for fpga's analog, digital and I/O, please follow the tips
 * to operate.
 * company: Xi'an Topsys Electronic co,. Ltd.
 * E-mail:wb_926@163.com
 * TEL:029-85732573
 */

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/types.h>
#include <unistd.h>
#include <sys/stat.h>
#include <string.h>
#include <sys/ioctl.h>

#define WRITE 1
#define READCMD	3
#define DEVNAME "/dev/sam9g45"

#define KEY_0   0xfffe
#define KEY_1   0xfffd
#define KEY_2   0xfffb
#define KEY_3   0xfff7
#define KEY_4   0xffef
#define KEY_5   0xffdf
#define KEY_6   0xffbf
#define KEY_7   0xff7f
#define KEY_8   0xfeff
#define KEY_9   0xfdff
#define KEY_10  0xfbff
#define KEY_11  0xf7ff

#define AD_0    18
#define AD_1    19
#define AD_2    20 

typedef struct __fpga_pdata {
	int offset;
	unsigned short  dataBuf;
}pdata;

int fpga_read_data(int fd, pdata *datatransmit)
{
	unsigned short buf;
	int ret;
	ret = ioctl(fd, READCMD, (unsigned long)datatransmit);
	if (ret < 0) {
		puts("ioctl error!\n");
		return -1;
	}

	ret = read(fd, &buf, 2);	
	if (ret < 0) {
		puts("read error!\n");
		return -1;
	}

	return buf;
}

int fpga_write_data(int fd, pdata *datatransmit)
{
	int ret;
	ret = ioctl(fd, WRITE, (unsigned long)datatransmit);
	if (ret < 0) {
		puts("ioctl error!!\n");
		return -1;
	}	

	return 0;
}

int main(int argc, char *argv[])
{
	int fd;
	int input_num;
	pdata dataTest;
	int in_offset;
	unsigned short in_data;
	unsigned short buff;
	int i;	

	if (2 != argc) {
		printf("Please input the Option: AD IO or Write\n");
		return -1;
	}

	fd = open(DEVNAME, O_RDWR);
	if (fd < 0) {
		puts("open error");
	}

	if (!strncmp(argv[1], "AD", 2)) {
		printf("Please input which AD port you want to read\n");
		scanf("%d", &input_num);
		switch (input_num) {
			case 0:
				for (i = 0; i<10; i++) {
					dataTest.offset = AD_0;
					dataTest.dataBuf = 0;
					buff = fpga_read_data(fd, &dataTest);
					printf("%3f\n", (float)buff/32768*164);
					sleep(1);
				}
				break;
			case 1:
				for (i = 0; i<10; i++) {
					dataTest.offset = AD_1;
					dataTest.dataBuf = 0;
					buff = fpga_read_data(fd, &dataTest);
					printf("%3f\n", (float)buff/32768*164);
					sleep(1);
				}
				break;
			case 2:
				for (i = 0; i<10; i++) {
					dataTest.offset = AD_2;
					dataTest.dataBuf = 0;
					buff = fpga_read_data(fd, &dataTest);
					printf("%3f\n", (float)buff/32768*164);
					sleep(1);
				}
				break;
			default: break;
		}

	}

	else if (!strncmp(argv[1], "Write", 5)) {
//		printf("Please input offset and data\n");
//		scanf("%d, %x", &in_offset, &in_data);
//		dataTest.offset = in_offset;
//		dataTest.dataBuf = in_data;
		while(1) {
			dataTest.offset = 17;
			dataTest.dataBuf = 0;
			fpga_write_data(fd, &dataTest);
			sleep(2);
			dataTest.offset = 17;
			dataTest.dataBuf = 0xffff;
			fpga_write_data(fd, &dataTest);
			sleep(2);
			dataTest.offset = 17;
			dataTest.dataBuf = 0x5555;
			fpga_write_data(fd, &dataTest);
			sleep(2);
			dataTest.offset = 17;
			dataTest.dataBuf = 0xaaaa;
			fpga_write_data(fd, &dataTest);
			sleep(2);
			dataTest.offset = 17;
			dataTest.dataBuf = 0xfffe;
			fpga_write_data(fd, &dataTest);
			sleep(2);
			dataTest.offset = 17;
			dataTest.dataBuf = 0xfffd;
			fpga_write_data(fd, &dataTest);
			sleep(2);
			dataTest.offset = 17;
			dataTest.dataBuf = 0xfffb;
			fpga_write_data(fd, &dataTest);
			sleep(2);
			dataTest.offset = 17;
			dataTest.dataBuf = 0xfff7;
			fpga_write_data(fd, &dataTest);
			sleep(2);
			dataTest.offset = 17;
			dataTest.dataBuf = 0xffef;
			fpga_write_data(fd, &dataTest);
			sleep(2);
			dataTest.offset = 17;
			dataTest.dataBuf = 0xffdf;
			fpga_write_data(fd, &dataTest);
			sleep(2);
			dataTest.offset = 17;
			dataTest.dataBuf = 0xffbf;
			fpga_write_data(fd, &dataTest);
			sleep(2);
			dataTest.offset = 17;
			dataTest.dataBuf = 0xff7f;
			fpga_write_data(fd, &dataTest);
			sleep(2);
			dataTest.offset = 17;
			dataTest.dataBuf = 0xfeff;
			fpga_write_data(fd, &dataTest);
			sleep(2);
			dataTest.offset = 17;
			dataTest.dataBuf = 0xfdff;
			fpga_write_data(fd, &dataTest);
			sleep(2);
			dataTest.offset = 17;
			dataTest.dataBuf = 0xfbff;
			fpga_write_data(fd, &dataTest);
			sleep(2);
			dataTest.offset = 17;
			dataTest.dataBuf = 0xf7ff;
			fpga_write_data(fd, &dataTest);
			sleep(2);
			dataTest.offset = 17;
			dataTest.dataBuf = 0xefff;
			fpga_write_data(fd, &dataTest);
			sleep(2);
			dataTest.offset = 17;
			dataTest.dataBuf = 0xdfff;
			fpga_write_data(fd, &dataTest);
			sleep(2);
			dataTest.offset = 17;
			dataTest.dataBuf = 0xbfff;
			fpga_write_data(fd, &dataTest);
			sleep(2);
			dataTest.offset = 17;
			dataTest.dataBuf = 0x7fff;
			fpga_write_data(fd, &dataTest);
			sleep(2);
		}
	}

	else if (!strncmp(argv[1], "IO", 2)) {
		dataTest.offset = 16;
		dataTest.dataBuf = 0;
			do {
				buff = fpga_read_data(fd, &dataTest);
				switch (buff) {
					case KEY_0: printf("the key your press is KEY_0\n");
								break;
					case KEY_1: printf("the key your press is KEY_1\n");
								break;
					case KEY_2: printf("the key your press is KEY_2\n");
								break;
					case KEY_3: printf("the key your press is KEY_3\n");
								break;
					case KEY_4: printf("the key your press is KEY_4\n");
								break;
					case KEY_5: printf("the key your press is KEY_5\n");
								break;
					case KEY_6: printf("the key your press is KEY_6\n");
								dataTest.offset = 17;
								dataTest.dataBuf = 0xfffe;
								fpga_write_data(fd, &dataTest);
								sleep(1);
								break;
					case KEY_7: printf("the key your press is KEY_7\n");
								dataTest.offset = 17;
								dataTest.dataBuf = 0xfffd;
								fpga_write_data(fd, &dataTest);
								sleep(1);
								break;
					case KEY_8: printf("the key your press is KEY_8\n");
								break;
					case KEY_9: printf("the key your press is KEY_9\n");
								break;
					case KEY_10: printf("the key your press is KEY_10\n");
								 break;
					case KEY_11: printf("the key your press is KEY_11\n");
								 break;
					default: break;
				}
			}while (0xffff == buff);
			dataTest.offset = 17;
			dataTest.dataBuf = 0xffff;
			fpga_write_data(fd, &dataTest);
	}

	return 0;
}
