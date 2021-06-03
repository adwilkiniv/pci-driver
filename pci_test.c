/***************************************************************************
 *   Copyright (C) 2021 Alfred alfredw@qti.qualcomm.com                    *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNES/.S FOR A PARTICULAR PURPOSE.  See the       *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/

/**
 * This program is a Linux  pci driver for the QBIT 6300 SOC.
 * 
 * This is a driver which can R/W and DMA to/from the QBIT device.
 *
  * 
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/fs.h>
#include <linux/cdev.h>		/* for cdev_ */
#include <asm/uaccess.h>        /* for put_user */

#define MAX_DEVICE		8
#define DEVICE_NAME 		"virtual_pci"
#define BAR_IO			2
#define BAR_MEM			3

MODULE_DESCRIPTION("QBIT PCI driver");
MODULE_AUTHOR("Alfred (alfredw@qti.qualcomm.com)");
MODULE_LICENSE("GPL");

/**
 * This table holds the list of (VendorID,DeviceID) supported by this driver
 *
 */
static struct pci_device_id pci_ids[] = {
	{ PCI_DEVICE( 0x1b21, 0x0612) },
	{ 0, }
};

/**
 * This macro ...
 *
 */
MODULE_DEVICE_TABLE(pci, pci_ids);
static dev_t devno;
static int major;

/**
 *  This structure is used to link a pci_dev to a cdev 
 *
 */
struct pci_cdev {
	int minor;
	struct pci_dev *pci_dev;
	struct cdev *cdev;
};

/* This is a "private" data structure */
/* You can store there any data that should be passed between driver's functions */
struct my_driver_priv {
    u8 __iomem *hwmem;
    u8 __iomem *regmem;
};

static struct pci_cdev pci_cdev[MAX_DEVICE];

/* this function initialize the table with all struct pci_cdev */
static void pci_cdev_init(struct pci_cdev pci_cdev[], int size, int first_minor)
{
	int i;

	for(i=0; i<size; i++) {
		pci_cdev[i].minor   = first_minor++;
		pci_cdev[i].pci_dev = NULL;
		pci_cdev[i].cdev    = NULL;
	}
}

/*
	-1 	=> failed
	 others => succes
*/
static int pci_cdev_add(struct pci_cdev pci_cdev[], int size, struct pci_dev *pdev)
{
	int i, res = -1;

	for(i=0; i<size; i++) {
		if (pci_cdev[i].pci_dev == NULL) {
			pci_cdev[i].pci_dev = pdev;
			res = pci_cdev[i].minor;
			break;
		}
	}
	
	return res;
}

static void pci_cdev_del(struct pci_cdev pci_cdev[], int size, struct pci_dev *pdev)
{
	int i;

	for(i=0; i<size; i++) {
		if (pci_cdev[i].pci_dev == pdev) {
			pci_cdev[i].pci_dev = NULL;
		}
	}
}

static struct pci_dev *pci_cdev_search_pci_dev(struct pci_cdev pci_cdev[], int size, int minor)
{
	int i;
	struct pci_dev *pdev = NULL;

	for(i=0; i<size; i++) {
		if (pci_cdev[i].minor == minor) {
			pdev = pci_cdev[i].pci_dev;
			break;
		}
	}

	return pdev;	
}

static struct cdev *pci_cdev_search_cdev(struct pci_cdev pci_cdev[], int size, int minor)
{
	int i;
	struct cdev *cdev = NULL;

	for(i=0; i<size; i++) {
		if (pci_cdev[i].minor == minor) {
			cdev = pci_cdev[i].cdev;
			break;
		}
	}

	return cdev;	
}

/* 
 	-1 	=> not found
	others	=> found
*/
static int pci_cdev_search_minor(struct pci_cdev pci_cdev[], 
		int size, struct pci_dev *pdev)
{
	int i, minor = -1;

	for(i=0; i<size; i++) {
		if (pci_cdev[i].pci_dev == pdev) {
			minor = pci_cdev[i].minor;
			break;
		}
	}

	return minor;
}
/* Release the PCI resources */
void release_device(struct pci_dev *pdev)
{

    /* Free memory region */
    pci_release_region(pdev, pci_select_bars(pdev, IORESOURCE_MEM));
    /* And disable device */
    pci_disable_device(pdev);
}


/**
 * This function is called when the device node is opened
 *
 */
static int pci_open(struct inode *inode, struct file *file)
{
	int minor = iminor(inode);
	file->private_data = (void *)pci_cdev_search_pci_dev(pci_cdev, MAX_DEVICE, minor);
	return 0;
}

/**
 * This function is called when the device node is closed
 *
 */
static int pci_release(struct inode *inode, struct file *file)
{
	return 0;
}

/**
 * This function is called when the device node is read
 *
 */
static ssize_t pci_read(struct file *file,	/* see include/linux/fs.h   */
			   char *buffer,	/* buffer to fill with data */
			   size_t length,	/* length of the buffer     */
			   loff_t * offset)
{
	int byte_read = 0;
	unsigned char value;
	struct pci_dev *pdev = (struct pci_dev *)file->private_data;
	unsigned long pci_io_addr = 0;

	pci_io_addr = pci_resource_start(pdev,2);

	while (byte_read < length) {
		/* read a byte from the input */
		value = inb(pci_io_addr + 1);
		/* write the value in the user buffer */
		put_user(value, &buffer[byte_read])
		byte_read++;
	}

	return byte_read;
}

/**
 * This function is called when the device node is read
 *
 */
static ssize_t pci_write(struct file *filp, const char *buffer, size_t len, loff_t * off) 
{
	int i;
	unsigned char value;
	struct pci_dev *pdev = (struct pci_dev *)filp->private_data;
        struct my_driver_priv *drv_priv = (struct my_driver_priv *) pci_get_drvdata(pdev);
      
	if (!drv_priv) {
	    printk(KERN_INFO "no private data \n");
            return -1; 
      	}

	for(i=0; i<len; i++) {
		/* read value on the buffer */
		value = (unsigned char)buffer[i];

		/* write data to the device */
		iowrite32(value, drv_priv->hwmem +(i*4));//+ (int)off);//pci_io_addr+i);
		value = ioread32(drv_priv->hwmem +(i*4));//+ (int)off);
                printk(KERN_INFO " data 0x%X offset 0x%X \n", value, off);
	}

	return len;
}


/* */
static irqreturn_t irq_handler_1(int irq, void *cookie)
{
   (void) cookie;
   printk("RDMA interrupt Handle IRQ #%d\n", irq);
   return IRQ_HANDLED;
}


static irqreturn_t irq_handler_2(int irq, void *cookie)
{
   (void) cookie;
   printk("MSI interrupt Handle IRQ #%d\n", irq);
   return IRQ_HANDLED;

}

/* Reqest interrupt and setup handler */
int set_interrupts(struct pci_dev *pdev)
{
    int irq;

    /* We want to obtain MSI interrupts we are requesting 3 intrrupts and will fail if we are not successful */
    int ret = pci_alloc_irq_vectors(pdev, 3, 3, PCI_IRQ_MSI);

    if (ret < 0) {
        return ret;
    }

    irq = pci_irq_vector(pdev,1 );//'2 correspoinds to the interrupt vector

    /* Request IRQ # */
    request_irq(irq, irq_handler_1,0, "TEST IRQ", pdev);

    printk(KERN_INFO "MSI 1 irq %d \n", irq);

    irq = pci_irq_vector(pdev, 2);
    //'2 correspoinds to the interrupt vector in the QBIT XXX_VEC1
    // where XXX is RDMA WDMA completed Mailbox etc. they must match

    printk(KERN_INFO "MSI 2 irq %d \n", irq);


    /* Request IRQ # */
    return request_irq(irq, irq_handler_2,0, "TEST IRQ", pdev);


}
/**
 * This structure holds informations about the pci node
 *
 */
static struct file_operations pci_ops = {
	.owner		= THIS_MODULE,
	.read 		= pci_read,
	.write 		= pci_write,
	.open 		= pci_open,
	.release 	= pci_release,
	.unlocked_ioctl = pci_ioctl
};
/*  This is the driver IOCTL.  This is the QBIT specific API to user space
 *  it provides user space access to DMA to/from the QBIT PCI device, and to 
 *  status the board
 *
 */



/*
 * This function is called when a new pci device is associated with a driver
 *
 * return: 1 => driver install failed
 *         0 => driver install was successful
 *
 */
static int pci_probe(struct pci_dev *dev, const struct pci_device_id *id)
{
	int ret,err, minor, bar;
	struct cdev *cdev;
	dev_t devno;
	unsigned long mmio_start, mmio_len;
	struct my_driver_priv *drv_priv;



	/* add this pci device in pci_cdev */
	if ((minor = pci_cdev_add(pci_cdev, MAX_DEVICE, dev)) < 0)
		goto error;

	/* compute major/minor number */
	devno = MKDEV(major, minor);

	/* allocate struct cdev */
	cdev = cdev_alloc();

	/* initialise struct cdev */
	cdev_init(cdev, &pci_ops);
	cdev->owner = THIS_MODULE;

	/* register cdev */
	ret = cdev_add(cdev, devno, 1);
	if (ret < 0) {
		dev_err(&(dev->dev), "Can't register character device\n");
		goto error;
	}
	pci_cdev[minor].cdev = cdev;

	dev_info(&(dev->dev), "%s The major device number is %d (%d).\n",
	       "Registeration is a success", MAJOR(devno), MINOR(devno));
	dev_info(&(dev->dev), "If you want to talk to the device driver,\n");
	dev_info(&(dev->dev), "you'll have to create a device file. \n");
	dev_info(&(dev->dev), "We suggest you use:\n");
	dev_info(&(dev->dev), "mknod %s c %d %d\n", DEVICE_NAME, MAJOR(devno), MINOR(devno);

	bar = pci_select_bars(dev, IORESOURCE_MEM);
	/* enable the device */
	pci_enable_device(dev);

	/* 'alloc' IO to talk with the card */
	err = pci_request_region(dev, bar,  "pci");
	
	if (err) {
        	pci_disable_device(dev);
		dev_err(&(dev->dev), "Can't request BAR2\n");
                cdev_del(cdev);
        	goto error;
	}

	/* check that BAR_IO is *really* IO region */
	if ((pci_resource_flags(dev, 2) & IORESOURCE_MEM) != IORESOURCE_MEM) {
		dev_err(&(dev->dev), "BAR2 isn't an IO region\n");
		cdev_del(cdev);
		goto error;
	}

    /* Get start and stop memory offsets for BAR2 set to hwmem2*/
    mmio_start = pci_resource_start(dev, 2);
    mmio_len = pci_resource_len(dev, 2);

    printk(KERN_INFO "mmio bar 2 start 0x%X mmio len bar 2 0x%X \n", mmio_start, mmio_len);

    /* Allocate memory for the driver private data */
    drv_priv = kzalloc(sizeof(struct my_driver_priv), GFP_KERNEL);

    if (!drv_priv) {
        release_device(dev);
        return -ENOMEM;
    }

    /* Remap BAR to the local pointer */
    drv_priv->hwmem = ioremap(mmio_start, mmio_len);
    printk(KERN_INFO " hwmem 0x%x \n", drv_priv->hwmem);

    if (!drv_priv->hwmem) {
       release_device(dev);
       return -EIO;
    }
  /* Get start and stop memory offsets and set BAR4 to hwmem_bar4 */
    mmio_start = pci_resource_start(dev, 0);
    mmio_len = pci_resource_len(dev, 0);

    printk(KERN_INFO "mmio bar 0 start 0x%X mmio le bar 0 0x%X \n", mmio_start, mmio_len);

    /* Allocate memory for the driver private data */
   /* 
    * drv_priv = kzalloc(sizeof(struct my_driver_priv), GFP_KERNEL);

    if (!drv_priv) {
        release_device(dev);
        return -ENOMEM;
    }
    */

    /* Remap BAR to the local pointer */
    drv_priv->regmem = ioremap(mmio_start, mmio_len);
     printk(KERN_INFO " regmem 0x%x \n", drv_priv->regmem);
	     
	     
	     
	     
	     
	     
	     
	     
	  


    if (!drv_priv->regmem) {
       release_device(dev);
       return -EIO;
    }
 
    /* Set driver private data */
    /* Now we can access mapped "hwmem" from the any driver's function */
    pci_set_drvdata(dev, drv_priv);

 // Set the Qbit device to be a bus master for DMA
   pci_set_master(dev);
   pci_set_dma_mask(dev, DMA_BIT_MASK(64));

 //  qbit_DMA(pdev, data );


   return set_interrupts(dev);

//	return 0;

error:
	return 1;
}

/**
 * This function is called when the driver is removed
 *
 */
static void pci_remove(struct pci_dev *dev)
{
	int minor;
	struct cdev *cdev;
	int irq;
        struct my_driver_priv *drv_priv = pci_get_drvdata(dev);

	/* remove associated cdev */
	minor = pci_cdev_search_minor(pci_cdev, MAX_DEVICE, dev);
	cdev = pci_cdev_search_cdev(pci_cdev, MAX_DEVICE, minor);
	if (cdev != NULL) 
		cdev_del(cdev);
		
	/* remove this device from pci_cdev */
	pci_cdev_del(pci_cdev, MAX_DEVICE, dev);

	  
	if (drv_priv) {
        printk(KERN_INFO "remove private data \n");
		if (drv_priv->hwmem) {
            
			iounmap(drv_priv->hwmem);
        
		}
	kfree(drv_priv);
	}
	/* free the irqs*/
        irq = pci_irq_vector(dev, 1);
        free_irq(irq, dev);
        irq = pci_irq_vector(dev, 2);
        free_irq(irq, dev);

        pci_free_irq_vectors(dev);

        

    release_device(dev);


	/* release the IO region */
//	pci_release_region(dev, BAR_IO);
}

/**
 * This structure holds informations about the pci driver
 *
 */
static struct pci_driver pci_driver = {
	.name 		= "pci",
	.id_table 	= pci_ids,
	.probe 		= pci_probe,
	.remove 	= pci_remove,
};


/**
 * This function is called when the module is loaded
 *
 */
static int __init pci_init_module(void)
{
	int ret;

	printk(KERN_DEBUG "Module pci init\n");

	/* allocate (several) major number */
	ret = alloc_chrdev_region(&devno, 0, MAX_DEVICE, "buffer");
	if (ret < 0) {
		printk(KERN_ERR "Can't get major\n");
		return ret;
	}

	/* get major number and save it in major */
	major = MAJOR(devno);

	/* initialise pci_cdev */
	pci_cdev_init(pci_cdev, MAX_DEVICE, MINOR(devno));

	/* register pci driver */
	ret = pci_register_driver(&pci_driver);
	if (ret < 0) {
		/* free major/minor number */
		unregister_chrdev_region(devno, 1);

		printk(KERN_ERR "pci-driver: can't register pci driver\n");
		return ret;
	}


	return 0;
}

/**
 * This function is called when the module is unloaded
 *
 */
static void pci_exit_module(void)
{
	int i;

	/* unregister pci driver */
	pci_unregister_driver(&pci_driver);

	/* unregister character device */
	for(i=0; i< MAX_DEVICE; i++) {
		if (pci_cdev[i].pci_dev != NULL) {
			cdev_del(pci_cdev[i].cdev);
		}
	}

	/* free major/minor number */
	unregister_chrdev_region(devno, MAX_DEVICE);

	printk(KERN_DEBUG "Module pci exit\n");
}

module_init(pci_init_module);
module_exit(pci_exit_module);
