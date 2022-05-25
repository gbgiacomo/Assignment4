/*
 * Bego Giacomo, Longo Mattia 2022/05
 * Assignment 4
 * 
 * Implementation of three task by semaphors and shared memory. Only the first one is periodic. 
 *
 */


#include <zephyr.h>
#include <device.h>
#include <drivers/gpio.h>
#include <sys/printk.h>
#include <sys/__assert.h>
#include <string.h>
#include <timing/timing.h>
#include <stdlib.h>
#include <stdio.h>


/* Size of stack area used by each thread (can be thread specific, if necessary)*/
#define STACK_SIZE 1024

/* Thread scheduling priority */
#define thread_A_prio 1
#define thread_B_prio 1
#define thread_C_prio 1

/* First therad periodicity (in ms)*/
#define thread_A_period 1000


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
int ab = 100;
int bc = 200;

/* Semaphores for task synch */
struct k_sem sem_ab;
struct k_sem sem_bc;

/* Thread code prototypes */
void thread_A_code(void *argA, void *argB, void *argC);
void thread_B_code(void *argA, void *argB, void *argC);
void thread_C_code(void *argA, void *argB, void *argC);


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
        nact++;
        /*Briefly semaphores test*/
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
        printk("\nTask B read %ld and write %ld at time: %lld ms",ab ,++nact ,k_uptime_get());
        bc=nact;
        k_sem_give(&sem_bc);

  }
}

void thread_C_code(void *argA , void *argB, void *argC)
{
    /* Other variables */
    long int nact = 0;

    
    while(1) {
        k_sem_take(&sem_bc, K_FOREVER);

        /*
         * 1- Read filtered samples from the vector
         * 2- Apply PWM signals to output LED
         *
         */

        /*Briefly semaphores test*/
        printk("\nTask C read %ld at time: %lld ms", bc ,k_uptime_get());
  }
}
