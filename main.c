#include <stdio.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <errno.h>

int main (void)
{
	int fd;
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
	close(fd);

	return 0;
}

