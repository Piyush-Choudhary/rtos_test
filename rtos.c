// RTOS Framework - Spring 2020
// J Losh

// Student Name:
// TO DO: Add your name on this line.  Do not include your ID number in the file.

// Add xx_ prefix to all files in your project
// xx_rtos.c
// xx_tm4c123gh6pm_startup_ccs.c
// xx_other files (except uart0.x and wait.x)
// (xx is a unique number that will be issued in class)
// Please do not change any function name in this code or the thread priorities

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL Evaluation Board
// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz

// Hardware configuration:
// 6 Pushbuttons and 5 LEDs, UART
// LEDs on these pins:
// Blue:   PF2 (on-board)
// Red:    PA2
// Orange: PA3
// Yellow: PA4
// Green:  PE0
// PBs on these pins
// PB0:    PC4
// PB1:    PC5
// PB2:    PC6
// PB3:    PC7
// PB4:    PD6
// PB5:    PD7
// UART Interface:
//   U0TX (PA1) and U0RX (PA0) are connected to the 2nd controller
//   The USB on the 2nd controller enumerates to an ICDI interface and a virtual COM port
//   Configured to 115,200 baud, 8N1

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <stdint.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"
#include "wait.h"
#include <string.h>

// REQUIRED: correct these bitbanding references for the off-board LEDs


#define BLUE_LED (*((volatile uint32_t *)(0x42000000 + (0x400253FC - 0x40000000) * 32 + 2 * 4))) //PF2
#define GREEN_LED (*((volatile uint32_t *)(0x42000000 + (0x400243FC - 0x40000000) * 32 + 0 * 4))) //PE0
#define YELLOW_LED (*((volatile uint32_t *)(0x42000000 + (0x400043FC - 0x40000000) * 32 + 4 * 4))) //PA4
#define ORANGE_LED (*((volatile uint32_t *)(0x42000000 + (0x400043FC - 0x40000000) * 32 + 3 * 4))) //PA3
#define RED_LED (*((volatile uint32_t *)(0x42000000 + (0x400043FC - 0x40000000) * 32 + 2 * 4))) //PA2
#define POWER (*((volatile uint32_t *)(0x42000000 + (0x400253FC - 0x40000000) * 32 + 3 * 4))) //PF3

#define PB0 (*((volatile uint32_t *)(0x42000000 + (0x400063FC - 0x40000000) * 32 + 4 * 4))) //Push Button-1
#define PB1 (*((volatile uint32_t *)(0x42000000 + (0x400063FC - 0x40000000) * 32 + 5 * 4))) //Push Button-2
#define PB2 (*((volatile uint32_t *)(0x42000000 + (0x400063FC - 0x40000000) * 32 + 6 * 4))) //Push Button-3
#define PB3 (*((volatile uint32_t *)(0x42000000 + (0x400063FC - 0x40000000) * 32 + 7 * 4))) //Push Button-4
#define PB4 (*((volatile uint32_t *)(0x42000000 + (0x400073FC - 0x40000000) * 32 + 6 * 4))) //Push Button-5
#define PB5 (*((volatile uint32_t *)(0x42000000 + (0x400073FC - 0x40000000) * 32 + 7 * 4))) //Push Button-6


#define BLUE_LED_MASK 4
#define GREEN_LED_MASK 1
#define YELLOW_LED_MASK 16
#define ORANGE_LED_MASK 8
#define RED_LED_MASK 4
#define POWER_LED_MASK 8

#define PB0_MASK 16
#define PB1_MASK 32
#define PB2_MASK 64
#define PB3_MASK 128
#define PB4_MASK 64
#define PB5_MASK 128



//-----------------------------------------------------------------------------
// RTOS Defines and Kernel Variables
//-----------------------------------------------------------------------------


// task
//#define STATE_INVALID    0 // no task
//#define STATE_UNRUN      1 // task has never been run
//#define STATE_READY      2 // has run, can resume at any time
//#define STATE_DELAYED    3 // has run, but now awaiting timer
//#define STATE_BLOCKED    4 // has run, but now blocked by semaphore


typedef enum _TaskState
{
    STATE_INVALID = 1,
    STATE_UNRUN   = 2,
    STATE_READY   = 3,
    STATE_DELAYED = 4,
    STATE_BLOCKED = 5,

}TaskState;

typedef enum _ServiceCalls
{
    CALL_YIELD = 10,

}ServiceCalls;


#define MAX_TASKS 12       // maximum number of valid tasks


extern void setPsp(uint32_t *PSP);
extern uint32_t *getPsp(void);
extern void pushR4_11(void);
extern void popR4_11(void);

// function pointer
typedef void (*_fn)();

// semaphore
#define MAX_SEMAPHORES 5
#define MAX_QUEUE_SIZE 5

struct semaphore
{
    uint16_t count;
    uint16_t queueSize;
    uint32_t processQueue[MAX_QUEUE_SIZE]; // store task index here
} semaphores[MAX_SEMAPHORES];

uint8_t semaphoreCount = 0;

struct semaphore *keyPressed, *keyReleased, *flashReq, *resource;


uint8_t taskCurrent = 0;   // index of last dispatched task

// REQUIRED: add store and management for the memory used by the thread stacks
//           thread stacks must start on 1 kiB boundaries so mpu can work correctly

typedef struct _TCB
{
    uint8_t state;                 // see STATE_ values above
    void *pid;                     // used to uniquely identify thread
    void *spInit;                  // location of original stack pointer
    void *sp;                      // location of stack pointer for thread
    int8_t priority;               // 0=highest to 15=lowest
    int8_t currentPriority;        // used for priority inheritance
    uint32_t ticks;                // ticks until sleep complete
    char name[16];                 // name of task used in ps command
    void *semaphore;               // pointer to the semaphore that is blocking the thread

} TCB;


TCB tcb[MAX_TASKS];

uint32_t stack[MAX_TASKS][512];

//-----------------------------------------------------------------------------
// RTOS Kernel Functions
//-----------------------------------------------------------------------------

// REQUIRED: initialize systick for 1ms system timer
void initRtos()
{
    uint8_t i;

    // clear out tcb records
    for (i = 0; i < MAX_TASKS; i++)
    {
        tcb[i].state = STATE_INVALID;
        tcb[i].pid = 0;
    }
}

// REQUIRED: Implement prioritization to 16 levels
int rtosScheduler()
{
    bool ok;

    static uint8_t task = 0xFF;

    ok = false;

    while (!ok)
    {
        task++;

        if (task >= MAX_TASKS)
            task = 0;

        ok = (tcb[task].state == STATE_READY || tcb[task].state == STATE_UNRUN);
    }
    return task;
}

bool createThread(_fn fn, const char name[], uint8_t priority, uint32_t stackBytes)
{
    bool ok             = false;
    uint8_t i           = 0;
    bool found          = false;

    static uint8_t taskCount   = 0;     // total number of valid tasks

    // REQUIRED: store the thread name
    // add task if room in task list
    // allocate stack space for a thread and assign to sp below
    if (taskCount < MAX_TASKS)
    {
        // make sure fn not already in list (prevent reentrancy)
        while (!found && (i < MAX_TASKS))
        {
            found = (tcb[i++].pid ==  fn);
        }
        if (!found)
        {
            // find first available tcb record
            i = 0;
            while (tcb[i].state != STATE_INVALID) {i++;}
            tcb[i].state = STATE_UNRUN;
            tcb[i].pid = fn;
            tcb[i].sp = &stack[i][511];
            tcb[i].priority = priority;
            tcb[i].currentPriority = priority;
            // increment task count
            taskCount++;
            ok = true;
        }
    }
    // REQUIRED: allow tasks switches again
    return ok;
}

// REQUIRED: modify this function to restart a thread
void restartThread(_fn fn)
{
}

// REQUIRED: modify this function to destroy a thread
// REQUIRED: remove any pending semaphore waiting
void destroyThread(_fn fn)
{
}

// REQUIRED: modify this function to set a thread priority
void setThreadPriority(_fn fn, uint8_t priority)
{
}

struct semaphore* createSemaphore(uint8_t count)
{
    struct semaphore *pSemaphore = 0;
    if (semaphoreCount < MAX_SEMAPHORES)
    {
        pSemaphore = &semaphores[semaphoreCount++];
        pSemaphore->count = count;
    }
    return pSemaphore;
}

// REQUIRED: modify this function to start the operating system, using all created tasks
void startRtos()
{
    _fn thread;

    taskCurrent = rtosScheduler();

    setPsp(tcb[taskCurrent].sp);

    thread = (_fn)tcb[taskCurrent].pid;

    tcb[taskCurrent].state = STATE_READY;

    (*thread)();

}

// REQUIRED: modify this function to yield execution back to scheduler using pendsv
// push registers, call scheduler, pop registers, return to new function
void yield()
{
    __asm(" SVC #10");
}

// REQUIRED: modify this function to support 1ms system timer
// execution yielded back to scheduler until time elapses using pendsv
// push registers, set state to delayed, store timeout, call scheduler, pop registers,
// return to new function (separate unrun or ready processing)
void sleep(uint32_t tick)
{
}

// REQUIRED: modify this function to wait a semaphore with priority inheritance
// return if avail (separate unrun or ready processing), else yield to scheduler using pendsv
void wait(struct semaphore *pSemaphore)
{
}

// REQUIRED: modify this function to signal a semaphore is available using pendsv
void post(struct semaphore *pSemaphore)
{
}

// REQUIRED: modify this function to add support for the system timer
// REQUIRED: in preemptive code, add code to request task switch
void systickIsr()
{
}

uint8_t getSvcValue()
{
    uint32_t *pc   = NULL;
    uint8_t  *temp = NULL;

    pc = getPsp();
    pc += 6;

    temp = (uint8_t*)(*pc - 2);

    return *temp;
}


// REQUIRED: modify this function to add support for the service call
// REQUIRED: in preemptive code, add code to handle synchronization primitives
void svCallIsr()
{
   uint8_t svcNumber = 0;

   svcNumber = getSvcValue();

   switch(svcNumber)
   {

   case CALL_YIELD:

       NVIC_INT_CTRL_R= NVIC_INT_CTRL_PEND_SV;

       break;

   default:
       __asm(" NOP");
       break;
   }

}


// REQUIRED: in coop and preemptive, modify this function to add support for task swit
// REQUIRED: process UNRUN and READY tasks differently
void pendSvIsr()
{
    pushR4_11();

    tcb[taskCurrent].sp = getPsp();

    taskCurrent = rtosScheduler();

    uint32_t *newStack = NULL;

    if (tcb[taskCurrent].state == STATE_READY)
    {
        setPsp(tcb[taskCurrent].sp);
        popR4_11();
    }
    else
    {
        setPsp(tcb[taskCurrent].sp);
        newStack = getPsp();                //psp
        newStack--;                         //xpsr
        *newStack = 16777216;               //put value on xpsr
        newStack--;                         //pc
        *newStack = (uint32_t)tcb[taskCurrent].pid;   //set value to pc
        newStack--;                         //lr
        newStack--;                         //r12
        newStack--;                         //r3
        newStack--;                         //r2
        newStack--;                         //r1
        newStack--;                         //r0

        setPsp(newStack);
        tcb[taskCurrent].state = STATE_READY;

    }

    __asm(" MOVW LR,#0xFFFD");
    __asm(" MOVT LR,#0xFFFF");

}

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

// Initialize Hardware
// REQUIRED: Add initialization for blue, orange, red, green, and yellow LEDs
//           6 pushbuttons, and uart
void initHw()
{
    SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);

    // Set GPIO ports to use APB (not needed since default configuration -- for clarity)
    // Note UART on port A must use APB
    SYSCTL_GPIOHBCTL_R = 0;


    //enable clk on timer 1
    //    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1;

    SYSCTL_RCGC2_R = SYSCTL_RCGC2_GPIOA | SYSCTL_RCGC2_GPIOF | SYSCTL_RCGC2_GPIOE |SYSCTL_RCGC2_GPIOD | SYSCTL_RCGC2_GPIOC ;

    GPIO_PORTD_LOCK_R = GPIO_LOCK_KEY;
    GPIO_PORTD_CR_R = PB5_MASK;

    // Configure LED pins
    //PORTf
    GPIO_PORTF_DIR_R = POWER_LED_MASK | BLUE_LED_MASK;  // bits 1 and 3 are outputs
    GPIO_PORTF_DR2R_R = POWER_LED_MASK | BLUE_LED_MASK; // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTF_DEN_R = POWER_LED_MASK | BLUE_LED_MASK;  // enable LEDs

    //PORTA
    GPIO_PORTA_DIR_R = RED_LED_MASK | ORANGE_LED_MASK | YELLOW_LED_MASK;
    GPIO_PORTA_DR2R_R = RED_LED_MASK | ORANGE_LED_MASK | YELLOW_LED_MASK; // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTA_DEN_R = RED_LED_MASK | ORANGE_LED_MASK | YELLOW_LED_MASK;  // enable LEDs

    //PORT E
    GPIO_PORTE_DIR_R = GREEN_LED_MASK;
    GPIO_PORTE_DR2R_R = GREEN_LED_MASK; // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTE_DEN_R = GREEN_LED_MASK;  // enable LEDs

    //PORTC
    GPIO_PORTC_DEN_R = PB0_MASK | PB1_MASK | PB2_MASK | PB3_MASK;  // enable LEDs and pushbuttons
    GPIO_PORTC_PUR_R = PB0_MASK | PB1_MASK | PB2_MASK | PB3_MASK; // enable internal pull-up for push button

    //PORT D
    GPIO_PORTD_DEN_R = PB4_MASK | PB5_MASK;  // enable LEDs and pushbuttons
    GPIO_PORTD_PUR_R = PB4_MASK | PB5_MASK; // enable internal pull-up for push button

}

// REQUIRED: add code to return a value from 0-63 indicating which of 6 PBs are pressed
uint8_t readPbs()
{
    return 0;
}

//-----------------------------------------------------------------------------
// YOUR UNIQUE CODE
// REQUIRED: add any custom code in this space
//-----------------------------------------------------------------------------

// ------------------------------------------------------------------------------
//  Task functions
// ------------------------------------------------------------------------------

// one task must be ready at all times or the scheduler will fail
// the idle task is implemented for this purpose
void idle()
{
    while(true)
    {
        ORANGE_LED = 1;
        waitMicrosecond(100000);
        ORANGE_LED = 0;
        yield();
    }
}


void idle2()
{
    while(true)
    {
        RED_LED = 1;
        waitMicrosecond(100000);
        RED_LED = 0;
        yield();
    }
}



void flash4Hz()
{
    while(true)
    {
        GREEN_LED ^= 1;
        sleep(125);
    }
}

void oneshot()
{
    while(true)
    {
        wait(flashReq);
        YELLOW_LED = 1;
        sleep(1000);
        YELLOW_LED = 0;
    }
}

void partOfLengthyFn()
{
    // represent some lengthy operation
    waitMicrosecond(990);
    // give another process a chance to run
    yield();
}

void lengthyFn()
{
    uint16_t i;
    while(true)
    {
        wait(resource);
        for (i = 0; i < 5000; i++)
        {
            partOfLengthyFn();
        }
        RED_LED ^= 1;
        post(resource);
    }
}

void readKeys()
{
    uint8_t buttons;
    while(true)
    {
        wait(keyReleased);
        buttons = 0;
        while (buttons == 0)
        {
            buttons = readPbs();
            yield();
        }
        post(keyPressed);
        if ((buttons & 1) != 0)
        {
            YELLOW_LED ^= 1;
            RED_LED = 1;
        }
        if ((buttons & 2) != 0)
        {
            post(flashReq);
            RED_LED = 0;
        }
        if ((buttons & 4) != 0)
        {
            restartThread(flash4Hz);
        }
        if ((buttons & 8) != 0)
        {
            destroyThread(flash4Hz);
        }
        if ((buttons & 16) != 0)
        {
            setThreadPriority(lengthyFn, 4);
        }
        yield();
    }
}

void debounce()
{
    uint8_t count;
    while(true)
    {
        wait(keyPressed);
        count = 10;
        while (count != 0)
        {
            sleep(10);
            if (readPbs() == 0)
                count--;
            else
                count = 10;
        }
        post(keyReleased);
    }
}

void uncooperative()
{
    while(true)
    {
        while (readPbs() == 32)
        {
        }
        yield();
    }
}

void important()
{
    while(true)
    {
        wait(resource);
        BLUE_LED = 1;
        sleep(1000);
        BLUE_LED = 0;
        post(resource);
    }
}

// REQUIRED: add processing for the shell commands through the UART here
//           your solution should not use C library calls for strings, as the stack will be too large
void shell()
{
    while (true)
    {
    }
}

//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------

int main(void)
{
    bool ok;

    // Initialize hardware
    initHw();
    initRtos();

    // Power-up flash
    GREEN_LED = 1;
    waitMicrosecond(250000);
    GREEN_LED = 0;
    waitMicrosecond(250000);

    // Initialize semaphores
    //    keyPressed = createSemaphore(1);
    //    keyReleased = createSemaphore(0);
    //    flashReq = createSemaphore(5);
    //    resource = createSemaphore(1);

    // Add required idle process at lowest priority
    ok =  createThread(idle, "Idle", 15, 1024);
    ok &=  createThread(idle2, "Idle2", 15, 1024);


    // Add other pr&ocesses
    //    ok &= createThread(lengthyFn, "LengthyFn", 12, 1024);
    //    ok &= createThread(flash4Hz, "Flash4Hz", 8, 1024);
    //    ok &= createThread(oneshot, "OneShot", 4, 1024);
    //    ok &= createThread(readKeys, "ReadKeys", 12, 1024);
    //    ok &= createThread(debounce, "Debounce", 12, 1024);
    //    ok &= createThread(important, "Important", 0, 1024);
    //    ok &= createThread(uncooperative, "Uncoop", 10, 1024);
    //    ok &= createThread(shell, "Shell", 8, 1024);

    // Start up RTOS
    if (ok)
        startRtos(); // never returns
    else
        RED_LED = 1;

    return 0;
}
