#include <stdio.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <sys/mman.h>
#include <poll.h>

#define QBIT_DMA 0

int main (void)
{
	int fd;
	int i,j;
	unsigned long* virtual_addr = 10;
	int* ptr[5] ;
	char buf[400] ="123456789abcdef0123456789dabcdef0123456789abcdabcd1234";

	static struct pollfd fds;

	
	
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


	fds.fd = fd;
	fds.events = POLLIN | POLLWRNORM;
	fds.revents = 0;
	poll(&fds, 2, 5000);

//	if(ioctl(fd, QBIT_DMA, &virtual_addr) == -1)
	{
//		perror("ioctl: ");
	}
//	printf("virt addr 0x%x &virt 0x%x\n", virtual_addr,&virtual_addr);
//
//
//
//
 for (i = 0; i < 4; i++)
 {

	ptr[i] = mmap(NULL, 0x1000, PROT_READ|PROT_WRITE, MAP_SHARED|MAP_LOCKED, fd,0);


//	memset(ptr, 0xbeef, 0x1000);
 
	printf(" shared buffer value 0x%x addr 0x%x \n",*ptr[i], ptr[i]);
//return 0;
	for (j = 0; j< 0x1000/4; j++)
	{
	*ptr[i] = j;
	ptr[i]++;
	}

	printf(" shared buffer value 0x%x addr 0x%x \n", *ptr[i], ptr[i]);
 
	 if(ioctl(fd, QBIT_DMA, &virtual_addr) == -1)
        {
               perror("ioctl: ");
        }
   //     printf("virt addr 0x%x &virt 0x%x\n", virtual_addr,&virtual_addr);

 }
//while(1)

{}
close(fd);

	return 0;
}

