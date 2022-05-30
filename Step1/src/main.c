/** @file main.c
 * @brief Contains the the implementation of the Assignment4
 *
 * The goal of this code is to read an input voltage signal,
 * filter it and show the result to the output 
 * by a PWM signal to a LED.
 * In particular, this file contains the main() function
 * 
 * @author Mattia Longo and Giacomo Bego
 * @date 31 May 2022
 * @bug No known bugs
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
#define BOARDLED_PIN DT_PROP(PWM0_NID, ch0_pin)
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
void thread_A_code(void *,void *,void *);
void thread_B_code(void *,void *,void *);
void thread_C_code(void *,void *,void *);

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

/* Size of the vector in thread B*/
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
static uint16_t sample; /**< shared memory between task A and B */
static uint16_t output; /**< shared memory between task B and C */

/**
 * @brief adc_sample function read the input voltage
 *
 * ADC setting and acquisition of the input voltage.
 * In this case the resolution has been setted to 10 bit (0-1023).
 * 
 * \author Mattia Longo and Giacomo Bego
 * \return integer value, representing the acquired sample
 */

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

/**
 * @brief main function run project
 *
 * main function sets the ADC, 
 * creates the tasks and
 * initilizes the semaphores.
 * 
 */

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

/**
 * @brief thread_A_code function implement the acquisition task
 *
 * Timing variables have been declared and calculated.
 * This function occurs every 10 ms (periodic task).
 * If the acquisition by the ADC gets a correct value, 
 * it pass the sample by a semaphore to task B, otherwise
 * it set it to a "safety value" equal to zero. 
 *
 * \param[*argA, *argB, *argC] void pointer parameters (not used in this project)
 * \return void function-> it does not return anything
 * 
 */

void thread_A_code(void *argA,void *argB,void *argC)
{
    /* Timing variables to control task periodicity */
    int64_t fin_time=0, release_time=0;

    printk("Thread A init (periodic)\n");

    /* Compute next release instant */
    release_time = k_uptime_get() + thread_A_period;

    /* Thread loop */
    while(1) {

        printk("\n\nTask A at time: %lld ms" ,k_uptime_get());

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

/**
 * @brief thread_B_code function implement the filtering task
 *
 * Every new sample is taken from the shared memory and put into a vector
 * from which we extract the average value of the last 10 samples.
 * Then the goal of the task is to copy the starting vector into another 
 * vector except for the samples "more than 10% far" form the average.
 * At the end, a new average on the final vector has been done.
 * The result is saved in another shared memory (global variable)
 * and passed to the output task by another semaphore.
 *
 * \param[*argA, *argB, *argC] void pointer parameters (not used in this project)
 * \return void function-> it does not return anything
 * 
 */

void thread_B_code(void *argA,void *argB,void *argC)
{
    uint16_t samples[SIZE]={0,0,0,0,0,0,0,0,0,0};
    uint16_t filteredSamples[SIZE]={0,0,0,0,0,0,0,0,0,0};
    int8_t index=-1;
    uint16_t average=0;
    uint16_t upperLevel=0;
    uint16_t lowerLevel=0;

    while(1) {

        k_sem_take(&sem_ab,  K_FOREVER);

        printk("\nTask B at time: %lld ms",k_uptime_get());

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
         
         //condition to avoid a zero denominator
         if(j>0){
            output=sum2/j;
         }

         /*semaphore*/
         k_sem_give(&sem_bc);

  }
}

/**
 * @brief thread_C_code function shows the result by a LED
 *
 * This function gets the filtered result from task B and show
 * the result so that it is proportional to a PWM duty cicle of a LED.
 *
 * \param[*argA, *argB, *argC] void pointer parameters (not used in this project)
 * \return void function-> it does not return anything 
 *
 */

void thread_C_code(void *argA,void *argB,void *argC)
{
    /* Variables to PWM */
    const struct device *pwm0_dev;          /* Pointer to PWM device structure */
    unsigned int pwmPeriod_us = 1000;       /* PWM period in us */
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

         ret = pwm_pin_set_usec(pwm0_dev, BOARDLED_PIN,
		      pwmPeriod_us,(unsigned int)((pwmPeriod_us*output)/1023), PWM_POLARITY_NORMAL);
         if (ret) {
          printk("Error %d: failed to set pulse width\n", ret);
            return;
         }
  }
}





