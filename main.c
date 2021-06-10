#include <stdio.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <sys/mman.h>

#define QBIT_DMA 0

int main (void)
{
	int fd;
	int i;
	unsigned long* virtual_addr = 10;
	int* ptr = NULL;
	char buf[400] ="123456789abcdef0123456789dabcdef0123456789abcdabcd1234";
	
	printf("virt addr shoudl be null  0x%x \n", virtual_addr);

	fd = open("/dev/virtual_pci", O_RDWR);
	if (fd < 0)
	{
		perror("open: ");
		return 1;
	}
	if (write(fd, &buf,400,0) < 0);
	{
		perror("write: ");
//		return 1;
	}

//	if(ioctl(fd, QBIT_DMA, &virtual_addr) == -1)
	{
//		perror("ioctl: ");
	}
//	printf("virt addr 0x%x &virt 0x%x\n", virtual_addr,&virtual_addr);
 
	ptr = mmap(NULL, 0x1000, PROT_READ|PROT_WRITE, MAP_SHARED|MAP_LOCKED, fd,0);
//	return 0;
//	memset(ptr, 0xbeef, 0x1000);
 
	printf(" shared buffer value 0x%x addr 0x%x \n",*ptr, ptr);
//return 0;
	for (i = 0; i< 0x1000/4; i++)
	{
	*ptr = i;
	ptr++;
	}

	printf(" shared buffer value 0x%x addr 0x%x \n", *ptr, ptr);

	 if(ioctl(fd, QBIT_DMA, &virtual_addr) == -1)
        {
               perror("ioctl: ");
        }
   //     printf("virt addr 0x%x &virt 0x%x\n", virtual_addr,&virtual_addr);


//while(1)

{}
close(fd);

	return 0;
}

