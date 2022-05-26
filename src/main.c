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
<<<<<<< HEAD
#include <drivers/adc.h>
=======
#include <drivers/pwm.h>
>>>>>>> PWMImplementation
#include <sys/printk.h>
#include <sys/__assert.h>
#include <string.h>
#include <timing/timing.h>
#include <stdlib.h>
#include <stdio.h>

<<<<<<< HEAD
=======


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


>>>>>>> PWMImplementation
/* Size of stack area used by each thread (can be thread specific, if necessary)*/
#define STACK_SIZE 1024

/* Thread scheduling priority */
#define thread_A_prio 1
#define thread_B_prio 1
#define thread_C_prio 1

/* First therad periodicity (in ms)*/
#define thread_A_period 1000

/* PWM step */
#define step 25

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

/* Global variables (shared memory between tasks A/B and B/C) */
int32_t samples[];
int32_t filtered_samples[];
int ab = 0;
int bc = 0;

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

/* ADC channel configuration */
static const struct adc_channel_cfg my_channel_cfg = {
	.gain = ADC_GAIN,
	.reference = ADC_REFERENCE,
	.acquisition_time = ADC_ACQUISITION_TIME,
	.channel_id = ADC_CHANNEL_ID,
	.input_positive = ADC_CHANNEL_INPUT
};

/* Main function */
void main(void) {
    
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
    while(1) {
        
        /*
         * 1- Read data input from ADC
         * 2- Insert samples in the vector
         *
         */
        
        /*Briefly semaphores test*/
        if(nact<100-step)
          nact+=step;
        else
          nact=-step;

        printk("\n\nTask A %ld at time: %lld ms", nact ,k_uptime_get());
        ab=nact;
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
    /* Other variables */
    long int nact = 0;

    while(1) {
        k_sem_take(&sem_ab,  K_FOREVER);

        /*
         * 1- Read samples from the vector
         * 2- Filtering the samples
         * 3- Write the righ samples in a new vector
         *
         */
        
        /*Briefly semaphores test*/
        nact=ab;
        nact+=step;
        printk("\nTask B read %ld and write %ld at time: %lld ms",ab ,nact ,k_uptime_get());
        bc=nact;
        k_sem_give(&sem_bc);

  }
}

void thread_C_code(void *argA , void *argB, void *argC)
{
    /* Other variables */
    long int nact = 0;

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

        /*
         * 1- Read filtered samples from the vector
         * 2- Apply PWM signals to output LED
         *
         */

         /*Briefly semaphores test*/
         printk("\nTask C read %ld at time: %lld ms", bc ,k_uptime_get());

         ret = pwm_pin_set_usec(pwm0_dev, BOARDLED_PIN,
		      pwmPeriod_us,(unsigned int)((pwmPeriod_us*bc)/100), PWM_POLARITY_NORMAL);
         if (ret) {
          printk("Error %d: failed to set pulse width\n", ret);
            return;
         }

  }
}





