/*
 * lunix-chrdev.c
 *
 * Implementation of character devices
 * for Lunix:TNG
 *
 * Nick Papadis  <nikpapadis@gmail.com>
 * Chris Markou  <chrs.markx86@gmail.com>
 */

#include <linux/mm.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/cdev.h>
#include <linux/poll.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/ioctl.h>
#include <linux/types.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/mmzone.h>
#include <linux/vmalloc.h>
#include <linux/spinlock.h>

#include "lunix.h"
#include "lunix-chrdev.h"
#include "lunix-lookup.h"

/*
 * Global data
 */
struct cdev lunix_chrdev_cdev;

/*
 * Just a quick [unlocked] check to see if the cached
 * chrdev state needs to be updated from sensor measurements.
 */
static int lunix_chrdev_state_needs_refresh(struct lunix_chrdev_state_struct *state)
{
	struct lunix_sensor_struct *sensor;
	
	WARN_ON ( !(sensor = state->sensor));

	if(state->buf_timestamp < (sensor->msr_data[state->type]->last_update)){
		debug("Refresh: needs to be updated, return 1");
		return 1;
	 }
	else{
		debug("Refresh: doesn't need to be updated, return 0");
		return 0;
	 }

}

/*
 * Updates the cached state of a character device
 * based on sensor data. Must be called with the
 * character device state lock held.
 */
static int lunix_chrdev_state_update(struct lunix_chrdev_state_struct *state)
{
	struct lunix_sensor_struct *sensor;
	uint32_t new_value; 
	long fixed_value;
	unsigned long cpu_flags;

	debug("entering Update\n");
	debug("Nick Update: type is  = %d\n", state->type);
	/*
	 * Grab the raw data quickly, hold the
	 * spinlock for as little as possible.
	 */
	/* ? */

	 /* Code add start */
	 
	if(!lunix_chrdev_state_needs_refresh(state)){
	 	return -EAGAIN;
	}
	
	/*
	 * Any new data available?
	 */
	
	
	sensor = state->sensor;
	WARN_ON(!sensor);

	debug(" Update : before the spinLock lock!\n");
	/* Spin and enter without interrupts enable because you don't want to lose any data */
	spin_lock_irqsave(&sensor->lock, cpu_flags);

	new_value = sensor->msr_data[state->type]->values[0];

	spin_unlock_irqrestore(&sensor->lock, cpu_flags);
	debug(" Update : after the spinLock unlock!\n");


	/*
	 * Now we can take our time to format them,
	 * holding only the private state semaphore
	 */

	
	// (2)Here we have to do the trick on the floats using lookup tables!!!
	switch(state->type) {
		case BATT:
			fixed_value = lookup_voltage[new_value];
			break;
		case TEMP:
			fixed_value = lookup_temperature[new_value];
			break;
		case LIGHT:
			fixed_value = lookup_light[new_value];
			break;
		default:
			debug("lookup: #$^&$#@&$&@");
	}

	debug("update: fixed_value = %ld", fixed_value);

	int integer_part = fixed_value / 1000;
	int decimal_part = fixed_value % 1000;

	debug("update: final_value = %d.%d\n", integer_part,decimal_part);

	/* functions returns the number of characters written to a buffer */
	//state->buf_lim = sprintf(state->buf_data, "%d.%d\n", integer_part,decimal_part);

	state->buf_lim = snprintf(state->buf_data, LUNIX_CHRDEV_BUFSZ, "%d.%d\n", integer_part, decimal_part);

	if (state->buf_lim >= LUNIX_CHRDEV_BUFSZ)
		debug("Update: snprintf returned string is truncated \n");

    // This is the new timeStamp!!!
	state->buf_timestamp = get_seconds();

	
	

    /* Code add end */


	debug("leaving Update\n");
	return 0;
}

/*************************************
 * Implementation of file operations
 * for the Lunix character device
 *************************************/

static int lunix_chrdev_open(struct inode *inode, struct file *filp)
{
	/* Declarations */
	/* ? */

	/* Code add-start*/
	struct lunix_chrdev_state_struct *new_state;  
	int type;
	int sensor_id;
        /* Code add-end*/
	int ret;

	debug("Nick: entering\n");
	ret = -ENODEV;
	if ((ret = nonseekable_open(inode, filp)) < 0)
		goto out;

	/*
	 * Associate this open file with the relevant sensor based on
	 * the minor number of the device node [/dev/sensor<NO>-<TYPE>]
	 */
	

	/* Code add-start*/
	sensor_id = iminor(inode) >> 3;
	type = iminor(inode) % 8;  
	debug("Nick: sensorId is  = %d\n", sensor_id);
	debug("Nick: type is  = %d\n", type);
	if(type>= N_LUNIX_MSR){ //type>=3 isn't a valid type-device
		ret = -ENODEV; // return no such device!!!
		goto out;
	}

	/* Code add-end*/

	
	/* Allocate a new Lunix character device private state structure */
	

  	/* Code add-start*/
	new_state = kmalloc(sizeof(struct lunix_chrdev_state_struct), GFP_KERNEL);
        /* Error handling */
	if(!new_state){
		printk(KERN_ERR "Nick: Lunix_Open: Error with allocationg private state structure\n");	
		/* return bad-address */	 	
		ret = -EFAULT;
		goto out; 
	}		  

	/* Initialization */
	
	new_state->type = type;
	new_state->sensor = &lunix_sensors[sensor_id]; 
	new_state->buf_lim = 0; 
	new_state->buf_timestamp = 0;  

	sema_init(&new_state->lock,1);

        /* Attach the new state to private_data so as to be reachable from other methods */
	filp->private_data = new_state;
        
        /* Code add-end*/

out:
	debug("Nick: leaving, with ret = %d\n", ret);
	return ret;
}

static int lunix_chrdev_release(struct inode *inode, struct file *filp)
{
	/* Code add-start */
	/* Free the allocated memory of the opened state structure */
	// kfree() handles NULL pointers fine - checking is redundant.
	kfree(filp->private_data);
	/* Code add-end */
	return 0;
}

static long lunix_chrdev_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	/* Why? */
	return -EINVAL;
}

static ssize_t lunix_chrdev_read(struct file *filp, char __user *usrbuf, size_t cnt, loff_t *f_pos)
{
	ssize_t ret;

	struct lunix_sensor_struct *sensor;
	struct lunix_chrdev_state_struct *state;

	state = filp->private_data;
	WARN_ON(!state);

	sensor = state->sensor;
	WARN_ON(!sensor);


	debug("Hey Nick i am in read function!\n") ;
	/* Lock? */

	if (down_interruptible(&state->lock))
		return -ERESTARTSYS; 	/* Lock because other procs may have the same state with you */


	/*
	 * If the cached character device state needs to be
	 * updated by actual sensor data (i.e. we need to report
	 * on a "fresh" measurement), do so.
	 */
	if (*f_pos == 0) {
		while (lunix_chrdev_state_update(state) == -EAGAIN) { // -> Try again 
			
			/* The process needs to sleep */

			up(&state->lock); 

			/*The condition is an arbitrary boolean expression that is evaluated by the
				macro before and after sleeping; until condition evaluates to a true value, the pro-
				cess continues to sleep.*/
			debug(" Read : No new data, i'm going to sleep!!!\n");
			if (wait_event_interruptible(sensor->wq, lunix_chrdev_state_needs_refresh(state))){
				debug(" Read : Wake up from an interrupt!!!\n");
				return -ERESTARTSYS; /* signal: tell the fs layer to handle it */
			}
			debug(" Read : New data has arrived, waked up!!!\n");
			/*This version returns an
              integer value that you should check; a nonzero value means your sleep was inter-
              rupted by some sort of signal, and your driver should probably return -ERESTARTSYS.
              */

            if (down_interruptible(&state->lock))
				return -ERESTARTSYS; 	/* Lock because other procs may have the same state with you */

		}
	}

	/* ok, data is there, do what you want now! */

	/* End of file */
	
	/* If the value is 0 , end-of-file was reached (and no data was read). Can this really happen?*/
	if (state->buf_lim == 0) {
        ret = 0;
        goto out;
    		}

	
	/* Determine the number of cached bytes to copy to userspace */

	int temp = state->buf_lim - *f_pos;

	/* now check if we really can transfer that data. See in LDD3 p.65 the schema */
	/* is the space the user gives us enough to copy the new_data? What if the space is not enough? */
	if (temp < cnt) cnt = temp; 


	debug(" Read : I am ready, i am going to copy the data to user!!!\n");
    /* copy buf into dst, which is in the user’s address space */ 
    if (copy_to_user(usrbuf, state->buf_data + *f_pos, cnt)) {
        ret = -EFAULT;
        goto out;
    }
    debug(" Read : Copy the data to user completed successfully!!!\n");


	/* Auto-rewind on EOF mode? */
	/* This happens when the read is trimed in more than one "accesses" because the buffer the user
	   is giving us is not enough. The second/third/... time we are in read() the update doesn't happen
	   because *f_pos!=0, so we continue from where we have stopped before!. */ 

	*f_pos += cnt;

	if (*f_pos >= state->buf_lim){
		*f_pos = 0;
		ret = cnt;
		goto out;
	}
		
	ret = cnt;
out:
	/* Unlock */
	up(&state->lock);
	return ret;
}

static int lunix_chrdev_mmap(struct file *filp, struct vm_area_struct *vma)
{
	return -EINVAL;
}

static struct file_operations lunix_chrdev_fops = 
{
	.owner          = THIS_MODULE,
	.open           = lunix_chrdev_open,
	.release        = lunix_chrdev_release,
	.read           = lunix_chrdev_read,
	.unlocked_ioctl = lunix_chrdev_ioctl,
	.mmap           = lunix_chrdev_mmap
};

int lunix_chrdev_init(void)
{
	/*
	 * Register the character device with the kernel, asking for
	 * a range of minor numbers (number of sensors * 8 measurements / sensor)
	 * beginning with LINUX_CHRDEV_MAJOR:0
	 */
	int ret;
	dev_t dev_no;
	unsigned int lunix_minor_cnt = lunix_sensor_cnt << 3; //16*8 nums
	
	debug("Nick: initializing character device\n");
	cdev_init(&lunix_chrdev_cdev, &lunix_chrdev_fops);
	lunix_chrdev_cdev.owner = THIS_MODULE;
	
	dev_no = MKDEV(LUNIX_CHRDEV_MAJOR, 0);
	

	/* register_chrdev_region */
        /* Add CODE-START */
	/* register the range of device numbers */
	ret = register_chrdev_region(dev_no, lunix_minor_cnt, "lunixTNG");
        /* Add CODE-END   */
	
	if (ret < 0) {
		debug("Nick: failed to register region, ret = %d\n", ret);
		goto out;
	}	
	
	/* cdev_add */
	/* Add CODE-START */
	/*  add the char device to the system */
	ret = cdev_add(&lunix_chrdev_cdev, dev_no, lunix_minor_cnt);
        /* Add CODE-END   */
	
	if (ret < 0) {
		debug("Nick: failed to add character device\n");
		goto out_with_chrdev_region;
	}
	debug("Nick: completed successfully\n");
	return 0;

out_with_chrdev_region:
	unregister_chrdev_region(dev_no, lunix_minor_cnt);
out:
	return ret;
}

void lunix_chrdev_destroy(void)
{
	dev_t dev_no;
	unsigned int lunix_minor_cnt = lunix_sensor_cnt << 3;
		
	debug("Nick: entering\n");
	dev_no = MKDEV(LUNIX_CHRDEV_MAJOR, 0);
	cdev_del(&lunix_chrdev_cdev);
	unregister_chrdev_region(dev_no, lunix_minor_cnt);
	debug("Nick: leaving\n");
}
