#include <stdio.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <errno.h>
#define QBIT_DMA 0
int main (void)
{
	int fd;
	int* virtual_addr;
	char buf[400] ="1234abcdabcdabc1234dabcdabcdabcd1234";

	fd = open("/dev/virtual_pci", O_RDWR);
	if (fd < 0)
	{
		perror("open: ");
		return 1;
	}
	if (write(fd, &buf,400,0) < 0);
	{
		perror("write: ");
		return 1;
	}
	
	if(ioctl(fd, QBIT_DMA, virtual_addr);
	   printf("virt addr 0x%x \n", virtual_addr);
	   
	   
	
	close(fd);

	return 0;
}

