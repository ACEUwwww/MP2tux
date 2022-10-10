/* tuxctl-ioctl.c
 *
 * Driver (skeleton) for the mp2 tuxcontrollers for ECE391 at UIUC.
 *
 * Mark Murphy 2006
 * Andrew Ofisher 2007
 * Steve Lumetta 12-13 Sep 2009
 * Puskar Naha 2013
 */

#include <asm/current.h>
#include <asm/uaccess.h>

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/file.h>
#include <linux/miscdevice.h>
#include <linux/kdev_t.h>
#include <linux/tty.h>
#include <linux/spinlock.h>

#include "tuxctl-ld.h"
#include "tuxctl-ioctl.h"
#include "mtcp.h"

#define debug(str, ...) \
	printk(KERN_DEBUG "%s: " str, __FUNCTION__, ## __VA_ARGS__)

int ack;
static unsigned long LED_condition;
static unsigned long BUTTON_condition;
static spinlock_t button_lock = SPIN_LOCK_UNLOCKED;


void handle_button(struct tty_struct * tty, char b, char c);
void handle_ack(void);
int tux_initial(struct tty_struct* tty);
int tux_button(struct tty_struct* tty,unsigned long arg);
int tux_led(struct tty_struct* tty,unsigned long arg);
void handle_reset(struct tty_struct * tty);

/************************ Protocol Implementation *************************/

/* tuxctl_handle_packet()
 * IMPORTANT : Read the header for tuxctl_ldisc_data_callback() in 
 * tuxctl-ld.c. It calls this function, so all warnings there apply 
 * here as well.
 */
void tuxctl_handle_packet (struct tty_struct* tty, unsigned char* packet)
{
    unsigned a, b, c;
    a = packet[0]; /* Avoid printk() sign extending the 8-bit */
    b = packet[1]; /* values when printing them. */
    c = packet[2];

	switch (a)
	{
		case MTCP_ACK:
			handle_ack();
			break;
		case MTCP_BIOC_EVENT:
			handle_button(tty,b,c);
			break;
		case MTCP_RESET:
			handle_reset(tty);
			break;
		default:
			return;
	}
	return;
    /*printk("packet : %x %x %x\n", a, b, c); */
}

void handle_ack(void)
{
	ack = 1;
	return;
}


void handle_button(struct tty_struct * tty, char b, char c)
{
	char RDLU;			/*MASK binary 1111*/
	char D_value; /*MASK binary 0100*/
	char L_value ;
	char RLDU ; 
	char CBAS  ;
	unsigned long flag;

	RDLU = (~c) & 0x0F;
	D_value = (RDLU & 0x04) >> 1 ; 
	L_value = (RDLU & 0x02) << 1 ;
	RLDU = ((RDLU & 0x09) | D_value | L_value) << 4 ;
	CBAS = (~b) & 0x0F ;

	spin_lock_irqsave(&button_lock,flag) ;
	BUTTON_condition = RLDU|CBAS ;
	spin_unlock_irqrestore(&button_lock,flag) ;
	return;
}

void handle_reset(struct tty_struct * tty)
{
	tux_initial(tty) ;
	tux_led(tty,LED_condition) ;
	return;
}



/******** IMPORTANT NOTE: READ THIS BEFORE IMPLEMENTING THE IOCTLS ************
 *                                                                            *
 * The ioctls should not spend any time waiting for responses to the commands *
 * they send to the controller. The data is sent over the serial line at      *
 * 9600 BAUD. At this rate, a byte takes approximately 1 millisecond to       *
 * transmit; this means that there will be about 9 milliseconds between       *
 * the time you request that the low-level serial driver send the             *
 * 6-byte SET_LEDS packet and the time the 3-byte ACK packet finishes         *
 * arriving. This is far too long a time for a system call to take. The       *
 * ioctls should return immediately with success if their parameters are      *
 * valid.                                                                     *
 *                                                                            *
 ******************************************************************************/
int 
tuxctl_ioctl (struct tty_struct* tty, struct file* file, 
	      unsigned cmd, unsigned long arg)
{
    switch (cmd) {
	case TUX_INIT:
		return tux_initial(tty);
	case TUX_BUTTONS:
		return tux_button(tty,arg);
	case TUX_SET_LED:
		return tux_led(tty,arg);
	case TUX_LED_ACK:
		return 0;
	case TUX_LED_REQUEST:
		return 0;
	case TUX_READ_LED:
		return 0;
	default:
	    return -EINVAL;
    }
}

/*
 * tux_initial
 *   DESCRIPTION: initialize the tux condition 
 *   INPUTS: tty-struct tty_struct*
 *   OUTPUTS: none
 *   RETURN VALUE:0 for success and -EINVAL for fail
 *   SIDE EFFECTS: set the initial value to all the variable of TUX
 */ 

int tux_initial(struct tty_struct* tty){
	/* Initialize the LED and BUTTON condition to 0*/
	unsigned long flag;
	char command1 = MTCP_BIOC_ON;
	char command2 = MTCP_LED_USR;
	LED_condition = 0;
	ack = 0;

	spin_lock_irqsave(&button_lock,flag);
	BUTTON_condition = 0;
	spin_unlock_irqrestore(&button_lock,flag);

	if(tuxctl_ldisc_put(tty,&command1,1) || tuxctl_ldisc_put(tty,&command2,1)){
	 	return -EINVAL;
	}
	return 0;
}

/*
 * tux_button
 *   DESCRIPTION: initialize the tux condition 
 *   INPUTS: tty -struct tty_struct*
 * 			 arg - unsigned long, a pointer to a 32-bit integer, which represents the state of button
 *   OUTPUTS: none
 *   RETURN VALUE:0 for success and -EINVAL for fail
 *   SIDE EFFECTS: set the tux_buttons for the later use
 */ 
int tux_button(struct tty_struct* tty, unsigned long arg){
	int ret;
	unsigned long flag;

	/* If the pointer is NULL, return -EINVAL immidiately*/
	if((uint32_t*)arg == NULL){return -EINVAL;}

	/* Enter the critcal section*/
	spin_lock_irqsave(&button_lock,flag);
	ret = copy_to_user((uint32_t*)arg,(uint32_t*)(&BUTTON_condition),sizeof(uint32_t));
	spin_unlock_irqrestore(&button_lock,flag);

	/* If the return value of copy_to_user >0, fail and return -EINVAL. */
	if(ret){
		return -EINVAL;
	}else{
		return 0;
	}
}

/*0 , 1 , 2 , 3 , 4 , 5 , 6 , 7 , 8 , 9 , A , B, C , D , E , F */
unsigned char SEGMENTS[16] = {0xE7, 0x06, 0xCB, 0x8F, 0x2E, 0xAD, 0xED, 0x86, 0xEF, 0xAF, 0xEE, 0x6D, 0xE1, 0x4F, 0xE9, 0xE8};


int tux_led(struct tty_struct* tty, unsigned long arg)
{
	unsigned int seg_value;
	unsigned int start_point;
	unsigned int offset;
	unsigned int decimal;
	unsigned int index ;
	unsigned char buffer[6];
	if (ack == 0)
	{
		return 0;
	}
	else{
		ack = 0;
	}
 /* 6 stands for the size of led set */
	buffer[0] = MTCP_LED_USR;
	tuxctl_ldisc_put(tty,buffer,1);
	buffer[0] = MTCP_LED_SET;
	buffer[1] = (arg >> 16) & 0x000F; /*16 stands for the high 2 bytes and 0x000F for the mask of last four bits*/

	/* initialize the decimal check part for the LED */
	/* 24 stands for the highest byte and get the last four bits*/
	decimal = (arg >> 24) & 0x000F ;

	start_point =2 ;

	index = 0;

	/* loop through the whole buffer and get each value for LED */

	for (index=0;index<4;index++)
	{
		if (((buffer[1]>>index) & 0x0001) != 0)	
		{
			offset = index * 4;
			seg_value = SEGMENTS[(arg >> offset) & 0x000F]; /*get the last four bits*/
			buffer[start_point] = seg_value | (((decimal >> index)&0x0001) << 4 );
			start_point ++ ; 
		}
	}
	
	if (tuxctl_ldisc_put(tty,buffer,start_point))
	{
		return -EINVAL;
	}
	LED_condition = arg;
	return 0;
}
