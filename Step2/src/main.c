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
//#define BOARDLED_PIN 0x0e
#define BOARDLED_PIN DT_PROP(PWM0_NID, ch0_pin)
/* Pin at which LED is connected. Addressing is direct (i.e., pin number)                  */
                /* Note 1: The PMW channel must be associated with the SAME pin in the DTS file            */
                /*         See the overlay file in this project to see how to change the assignment        */
                /*         *** Note: RUN CMAKE (Project -> Run Cmake) after editing the overlay file***    */
                /* Note 2: the pin can (and should) be obtained automatically from the DTS file.           */
                /*         I'm doing it manually to avoid entering in (cryptic) DT macros and to force     */ 
                /*         you to read the dts file.                                                       */
                /*         This line would do the trick: #define BOARDLED_PIN DT_PROP(PWM0_NID, ch0_pin)   */          

/* Size of stack area used by each thread (can be thread specific, if necessary)*/
#define STACK_SIZE 1024

/* Thread scheduling priority */
#define thread_A_prio 1
#define thread_B_prio 1
#define thread_C_prio 1

/* First therad periodicity (in ms)*/
#define thread_A_period 10

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

/* Semaphores for task synch */
struct k_sem sem_ab;
struct k_sem sem_bc;

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
#define SIZE 10 /* Size of the vector in thread B*/

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
static uint16_t sample;
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


    /* Create and init semaphores */
    k_sem_init(&sem_ab, 0, 1);
    k_sem_init(&sem_bc, 0, 1);
    
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

    /* Other variables */
    long int nact = 0;
    
    printk("Thread A init (periodic)\n");

    /* Compute next release instant */
    release_time = k_uptime_get() + thread_A_period;

    /* Thread loop */
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
                printk("adc reading out of range\n\r");
                sample=0;  /* Safety value */
            }
            else {
                sample=adc_sample_buffer[0];
                printk(": sample is : %4u",sample);
            }
        }

        /*semaphore*/
        k_sem_give(&sem_ab);


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
    uint16_t samples[SIZE]={0,0,0,0,0,0,0,0,0,0};
    uint16_t filteredSamples[SIZE]={0,0,0,0,0,0,0,0,0,0};
    int8_t index=-1; /* From 1 to 10*/
    uint16_t average=0;
    uint16_t upperLevel=0;
    uint16_t lowerLevel=0;

    while(1) {

        k_sem_take(&sem_ab,  K_FOREVER);

        printk("\nTask B at time: %lld ms",k_uptime_get());

        /*
         * 1- Read sample value from the semaphore A-B shared memory
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

	 samples[index]=sample;
	
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
            output=sum2/j;
         }

         /*semaphore*/
         k_sem_give(&sem_bc);

  }
}

void thread_C_code(void *argA , void *argB, void *argC)
{
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

        k_sem_take(&sem_bc, K_FOREVER);

        printk("\nTask C at time: %lld ms",k_uptime_get());

        /*
         * 1- Read filtered value from the vector
         * 2- Apply PWM signals to output LED
         *
         */

         ret = pwm_pin_set_usec(pwm0_dev, BOARDLED_PIN,
		      pwmPeriod_us,(unsigned int)((pwmPeriod_us*output)/1023), PWM_POLARITY_NORMAL);
         if (ret) {
          printk("Error %d: failed to set pulse width\n", ret);
            return;
         }

  }
}





