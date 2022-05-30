/*
 * Bego Giacomo, Longo Mattia 2022/05
 * Assignment 4
 * 
 * Implementation of three task by semaphors and shared memory. Only the first one is periodic. 
 *
 */


#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/gpio.h>
#include <drivers/adc.h>
#include <drivers/pwm.h>
#include <sys/printk.h>
#include <sys/__assert.h>
#include <string.h>
#include <timing/timing.h>
#include <stdlib.h>
#include <stdio.h>

/* PWM pin initialization */
#define PWM0_NID DT_NODELABEL(pwm0)
#define BOARDLED_PIN DT_PROP(PWM0_NID, ch0_pin) /* Define output pin */
          
/* Size of stack area used by each thread (can be thread specific, if necessary)*/
#define STACK_SIZE 1024

/* Thread scheduling priority */
#define thread_A_prio 1
#define thread_B_prio 1
#define thread_C_prio 1

/* First therad periodicity (in ms)*/
#define thread_A_period 15

/* Create thread stack space */
K_THREAD_STACK_DEFINE(thread_A_stack, STACK_SIZE);
K_THREAD_STACK_DEFINE(thread_B_stack, STACK_SIZE);
K_THREAD_STACK_DEFINE(thread_C_stack, STACK_SIZE);
  
/* Create variables for thread data */
struct k_thread thread_A_data;
struct k_thread thread_B_data;
struct k_thread thread_C_data;

/* Create task IDs */
k_tid_t thread_A_tid;
k_tid_t thread_B_tid;
k_tid_t thread_C_tid;

/* Create fifos */
struct k_fifo fifo_ab;
struct k_fifo fifo_bc;

/* Create fifo data structure and variables */
struct data_item_t {
    void *fifo_reserved;    /* 1st word reserved for use by FIFO */
    uint16_t data;          /* Actual data */
};

/* Thread code prototypes */
void thread_A_code(void *argA, void *argB, void *argC);
void thread_B_code(void *argA, void *argB, void *argC);
void thread_C_code(void *argA, void *argB, void *argC);

/*ADC definitions and includes*/
#include <hal/nrf_saadc.h>
#define ADC_NID DT_NODELABEL(adc) 
#define ADC_RESOLUTION 10
#define ADC_GAIN ADC_GAIN_1_4
#define ADC_REFERENCE ADC_REF_VDD_1_4
#define ADC_ACQUISITION_TIME ADC_ACQ_TIME(ADC_ACQ_TIME_MICROSECONDS, 40)
#define ADC_CHANNEL_ID 1 
#define ADC_CHANNEL_INPUT NRF_SAADC_INPUT_AIN1
#define BUFFER_SIZE 1

/* Size of the vector in thread B */
#define SIZE 10 

/* ADC channel configuration */
static const struct adc_channel_cfg my_channel_cfg = {
	.gain = ADC_GAIN,
	.reference = ADC_REFERENCE,
	.acquisition_time = ADC_ACQUISITION_TIME,
	.channel_id = ADC_CHANNEL_ID,
	.input_positive = ADC_CHANNEL_INPUT
};
struct k_timer my_timer;
const struct device *adc_dev = NULL;
static uint16_t adc_sample_buffer[BUFFER_SIZE];
static uint16_t output;

/* Takes one sample */
static int adc_sample(void)
{
	int ret;
	const struct adc_sequence sequence = {
		.channels = BIT(ADC_CHANNEL_ID),
		.buffer = adc_sample_buffer,
		.buffer_size = sizeof(adc_sample_buffer),
		.resolution = ADC_RESOLUTION,
	};

	if (adc_dev == NULL) {
            printk("adc_sample(): error, must bind to adc first \n\r");
            return -1;
	}

	ret = adc_read(adc_dev, &sequence);
	if (ret) {
            printk("adc_read() failed with code %d\n", ret);
	}	

	return ret;
}

/* Main function */
void main(void) {
    
    int err=0;

    /* ADC setup: bind and initialize */
    adc_dev = device_get_binding(DT_LABEL(ADC_NID));
	if (!adc_dev) {
        printk("ADC device_get_binding() failed\n");
    } 
    err = adc_channel_setup(adc_dev, &my_channel_cfg);
    if (err) {
        printk("adc_channel_setup() failed with error code %d\n", err);
    }


    /* Create/Init fifos */
    k_fifo_init(&fifo_ab);
    k_fifo_init(&fifo_bc);
    
    /* Create tasks */
    thread_A_tid = k_thread_create(&thread_A_data, thread_A_stack,
        K_THREAD_STACK_SIZEOF(thread_A_stack), thread_A_code,
        NULL, NULL, NULL, thread_A_prio, 0, K_NO_WAIT);

    thread_B_tid = k_thread_create(&thread_B_data, thread_B_stack,
        K_THREAD_STACK_SIZEOF(thread_B_stack), thread_B_code,
        NULL, NULL, NULL, thread_B_prio, 0, K_NO_WAIT);

    thread_B_tid = k_thread_create(&thread_C_data, thread_C_stack,
        K_THREAD_STACK_SIZEOF(thread_C_stack), thread_C_code,
        NULL, NULL, NULL, thread_C_prio, 0, K_NO_WAIT);

    
    return;
} 

/* Thread code implementation */
void thread_A_code(void *argA , void *argB, void *argC)
{
    /* Timing variables to control task periodicity */
    int64_t fin_time=0, release_time=0;
    
    struct data_item_t data_ab;
    
    printk("Thread A init (periodic)\n");

    /* Compute next release instant */
    release_time = k_uptime_get() + thread_A_period;

    /* Thread loop */
    while(1) {

        printk("\n\nTask A at time: %lld ms" ,k_uptime_get());
        /*****************************************/
        /******** Read data input from ADC *******/
        /*****************************************/

        int err=0;
        /* Get one sample, checks for errors and prints the values */
        err=adc_sample();
        if(err) {
            printk("adc_sample() failed with error code %d\n\r",err);
        }
        else {
            if(adc_sample_buffer[0] > 1023) {
                printk("\t adc reading out of range\r"); 
                data_ab.data = 0;  /* Safety value */
        	
            }
            else {
                data_ab.data=adc_sample_buffer[0];
                printk("\t sample is : %4u", data_ab.data);
            }
        }

	/* Put FIFO data */
        k_fifo_put(&fifo_ab, &data_ab);


        /* Wait for next release instant */ 
        fin_time = k_uptime_get();
        if( fin_time < release_time) {
            k_msleep(release_time - fin_time);
            release_time += thread_A_period;
        }
    } 

}

void thread_B_code(void *argA , void *argB, void *argC)
{
    struct data_item_t *data_ab;
    struct data_item_t data_bc;
    
    uint16_t samples[SIZE]={0,0,0,0,0,0,0,0,0,0};
    uint16_t filteredSamples[SIZE]={0,0,0,0,0,0,0,0,0,0};
    int8_t index=-1;
    uint16_t average=0;
    uint16_t upperLevel=0;
    uint16_t lowerLevel=0;

    while(1) {

        data_ab = k_fifo_get(&fifo_ab, K_FOREVER);

        printk("\nTask B at time: %lld ms",k_uptime_get());

        /*
         * 1- Read sample value from FIFO queues
         * 2- Filtering the samples
         * 3- Provide the right value that drive PWM led
         *
         */
        
         if(index<SIZE-1){
		index++;
         }
         else{
		index=0;
	 }
          
         /* Return data from the pointer data_ab */
	 samples[index]=data_ab->data;
	
	 /* Average calculation */
	 int32_t sum=0;
	
	 for(int8_t i=0;i<10;i++){
		 sum+=samples[i];
	 }
	 average=sum/10;
	
	 /* Samples value limits */
	 upperLevel=average*1.1;
	 lowerLevel=average*0.9;

         uint16_t j=0;
	
	 for(uint8_t i=0;i<10;i++){
		 if(samples[i]>=lowerLevel && samples[i]<=upperLevel){
		 	filteredSamples[j]=samples[i];
		 	j++;
		 }
	 }
         
	 uint16_t sum2=0;
	
	 for(uint16_t a=0;a<j;a++){
	 	sum2+=filteredSamples[a];
	 }
          
         if(j>0){
            data_bc.data=sum2/j;
         }

	 /* Put value in the FIFO quesues */
         k_fifo_put(&fifo_bc, &data_bc);

  }
}

void thread_C_code(void *argA , void *argB, void *argC)
{
    struct data_item_t *data_bc;
    
    /* Variables to PWM */
    const struct device *pwm0_dev;          /* Pointer to PWM device structure */
    unsigned int pwmPeriod_us = 1000;       /* PWM priod in us */
    int ret=0;                              /* Generic return value variable */

    /* Return pointer to device structure with the given name */
    pwm0_dev = device_get_binding(DT_LABEL(PWM0_NID));
    if (pwm0_dev == NULL) {
	printk("\nError: Failed to bind to PWM0 r");
	return;
    }

    printk("\nThread C init");

    while(1) {

        data_bc = k_fifo_get(&fifo_bc, K_FOREVER);

        printk("\nTask C at time: %lld ms",k_uptime_get());

        /*
         * 1- Read filtered value from FIFO queues
         * 2- Apply PWM signals to output LED
         *
         */

         ret = pwm_pin_set_usec(pwm0_dev, BOARDLED_PIN,
		      pwmPeriod_us,(unsigned int)((pwmPeriod_us*(data_bc->data))/1023), PWM_POLARITY_NORMAL);
         if (ret) {
          printk("Error %d: failed to set pulse width\n", ret);
            return;
         }

  }
}