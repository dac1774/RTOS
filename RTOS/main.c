// RTOS Framework - Fall 2019
// J Losh

// Student Name: David Chavez
// TO DO: Add your name on this line.  Do not include your ID number in the file.

// got 5a WORKING!!! yay!!!! 6pm 11/27/19 - DC
// 5b complete, now onto sleep 6pm 11/29/19 - DC

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL Evaluation Board
// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz

// Hardware configuration:
// 6 Pushbuttons and 5 LEDs, UART
// LEDS on these pins:
// Blue:   PF2 (on-board)
// Red:    PE1
// Orange: PE2
// Yellow: PE3
// Green:  PE4
// PBs on these pins
// PB0:    PA2
// PB1:    PA3
// PB2:    PA4
// PB3:    PA5
// PB4:    PA6
// PB5:    PA7

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <stdint.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"

// REQUIRED: correct these bitbanding references for the off-board LEDs
#define BLUE_LED     (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 2*4)))
#define RED_LED      (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 1*4))) // off-board red LED
#define ORANGE_LED   (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 2*4))) // off-board green LED
#define YELLOW_LED   (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 3*4))) // off-board yellow LED
#define GREEN_LED    (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 4*4))) // off-board orange LED

#define PB0         (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 2*4)))  //push button switches
#define PB1         (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 3*4)))
#define PB2         (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 4*4)))
#define PB3         (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 5*4)))
#define PB4         (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 6*4)))
#define PB5         (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 7*4)))

#define MAX_CHARS   80
//-----------------------------------------------------------------------------
// RTOS Defines and Kernel Variables
//-----------------------------------------------------------------------------

// function pointer
typedef void (*_fn)();

// semaphore
#define MAX_SEMAPHORES 5
#define MAX_QUEUE_SIZE 5
struct semaphore
{
    uint16_t count;
    uint16_t queueSize;
    uint32_t current;
    uint32_t processQueue[MAX_QUEUE_SIZE]; // store task index here
} semaphores[MAX_SEMAPHORES];
struct args
{
    uint16_t address;
    char StringInput[MAX_CHARS + 1];
    char type[4];
    uint8_t pos[4];
    uint8_t fieldCount;
    char result[50];
    uint8_t trash;
};
uint8_t semaphoreCount = 0;

struct semaphore *keyPressed, *keyReleased, *flashReq, *resource;

// task
#define STATE_INVALID    0 // no task
#define STATE_UNRUN      1 // task has never been run
#define STATE_READY      2 // has run, can resume at any time
#define STATE_DELAYED    3 // has run, but now awaiting timer
#define STATE_BLOCKED    4 // has run, but now blocked by semaphore
#define ON 1
#define OFF 0
#define MAX_TASKS 12       // maximum number of valid tasks
uint8_t taskCurrent = 0;   // index of last dispatched task
uint8_t taskCount = 0;     // total number of valid tasks
uint8_t priorityinheritance = 0;   // index of last dispatched task
uint8_t preemption= 1;     // total number of valid tasks
uint8_t scheduler= 1;
int ticks=0;
// REQUIRED: add store and management for the memory used by the thread stacks
//           thread stacks must start on 1 kiB boundaries so mpu can work correctly

struct _tcb
{
    uint8_t state;                 // see STATE_ values above
    void *pid;                     // used to uniquely identify thread
    void *spInit;                  // location of original stack pointer
    void *sp;                      // location of stack pointer for thread
    int8_t priority;               // 0=highest to 15=lowest
    int8_t currentPriority;        // used for priority inheritance
    uint32_t ticks;                // ticks until sleep complete
    int lowerpercentage;
    int upperpercentage;
    uint32_t countticks;
    char name[16];                 // name of task used in ps command
    void *semaphore;               // pointer to the semaphore that is blocking the thread
} tcb[MAX_TASKS];
#define STACK_NUM 24                // number of 1 kiB stacks
void* openHeapRegions[STACK_NUM];    // open memory regions
#pragma DATA_ALIGN(heap, 1024);     // align memory with 1 kiB
uint8_t heap[STACK_NUM*1024];
uint32_t offset = 0;
void modePSP();
extern void popR4R11();
extern void pushR4R11();
extern void * getPSP();
extern void setMSP(void * MSP);
extern void pushfake(void * xPSR, void * PC);
extern void setPSP(void * PSP);
extern uint32_t getR0();
//-----------------------------------------------------------------------------
// RTOS Kernel Functions
//-----------------------------------------------------------------------------
void putcUart0(char c)
{


    while (UART0_FR_R & UART_FR_TXFF);               // wait if uart0 tx fifo full
    UART0_DR_R = c;                                  // write character to fifo
}
void putsUart0(char* str)
{

    uint8_t i = 0;
    while (str[i]!='\0')
      putcUart0(str[i++]);
}

// Blocking function that returns with serial data once the buffer is not empty
char getcUart0()
{
    while (UART0_FR_R & UART_FR_RXFE)// wait if uart0 rx fifo empty
    {
        yield();
    }
    return UART0_DR_R & 0xFF;                        // get character from fifo
}
//BW
void toint(long long value){
    int temp=0, digits=1;
    long long bit=0x100000000;
    putsUart0("0x");
    while(digits!=8)
    {
        temp=(int)value/bit;
        if(temp>9)
            temp= temp+7;
        putcUart0(temp +48);
        value=value- (bit* temp);
        bit=bit/16;
        digits++;
    }

}

void tohex(long long value){
    int temp=0, digits=1;
    long long bit=0x100000000;
    putsUart0("0x");
    while(digits!=8)
    {
        temp=(int)value/bit;
        if(temp>9)
            temp= temp+7;
        putcUart0(temp +48);
        value=value- (bit* temp);
        bit=bit/16;
        digits++;
    }

}
// REQUIRED: initialize systick for 1ms system timer
void initRtos()
{
    uint8_t i;
    // no tasks running
    taskCount = 0;
    // clear out tcb records
    for (i = 0; i < MAX_TASKS; i++)
    {
        tcb[i].state = STATE_INVALID;
        tcb[i].pid = 0;
    }
}

// REQUIRED: Implement prioritization to 16 levels
// Priority is implemented by using the ticks variable on ready and unrun programs
int rtosScheduler()
{
    bool ok;
    static uint8_t task = 0xFF;
    ok = false;
    if(scheduler)
    {   //priority
        while (!ok)
        {
            task++;
            if (task >= MAX_TASKS)
                task = 0;
            if(tcb[task].state == STATE_READY || tcb[task].state == STATE_UNRUN)
            {
                if(tcb[task].ticks == 0)
                {
                    ok = (tcb[task].state == STATE_READY || tcb[task].state == STATE_UNRUN);
                    tcb[task].ticks = tcb[task].currentPriority;
                }
                else
                {
                    tcb[task].ticks -=1;
                }
            }
        }
    }
    else
    {   //round robin
        while (!ok)
        {
            task++;
            if (task >= MAX_TASKS)
                task = 0;
            ok = (tcb[task].state == STATE_READY || tcb[task].state == STATE_UNRUN);
        }
    }
    tcb[task].countticks++;
    return task;
}

bool createThread(_fn fn, const char nam[], uint8_t priority, uint32_t stackBytes)
{
    bool ok = false;
    int t=0;
    uint8_t i = 0;
    bool found = false;
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
            for(t=0;t<16;t++)
                tcb[i].name[t] = nam[t];
            //sram= sram + stackBytes;
            tcb[i].spInit= &heap[offset];
            tcb[i].sp= &heap[offset + stackBytes];
            offset += stackBytes;
            tcb[i].priority = priority;
            tcb[i].currentPriority = priority;
            tcb[i].ticks = 0;
            // increment task count
            // set sp to psp
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
// NOTE: see notes in class for strategies on whether stack is freed or not
void destroyThread(_fn fn)
{
}

// REQUIRED: modify this function to set a thread priority
void setThreadPriority(_fn fn, uint8_t priority)
{   //find thread
    uint32_t i;
    for(i = 0; i<MAX_TASKS;i++)
    {
        if(tcb[i].pid == fn)
        {
            tcb[i].currentPriority = priority;
            break;
        }
    }
}

struct semaphore* createSemaphore(uint8_t count)
{
    struct semaphore *pSemaphore = 0;
    if (semaphoreCount < MAX_SEMAPHORES)
    {
        pSemaphore = &semaphores[semaphoreCount++];
        pSemaphore->count = count;
        pSemaphore->queueSize = 0;
    }
    return pSemaphore;
}

// REQUIRED: modify this function to start the operating system, using all created tasks
void startRtos()
{


    taskCurrent=rtosScheduler();
    setPSP(tcb[taskCurrent].sp);
    tcb[taskCurrent].state = STATE_READY;

    modePSP();  //use the PSP register for threads

    _fn f=(_fn)tcb[taskCurrent].pid;
    NVIC_ST_CTRL_R |= NVIC_ST_CTRL_ENABLE;//start Systick timer
    TIMER1_CTL_R |= TIMER_CTL_TAEN;
    f();
}

// REQUIRED: modify this function to yield execution back to scheduler using pendsv
// push registers, call scheduler, pop registers, return to new function
void yield()
{
    //putsUart0("We are in yield\n");
    __asm("   MOV R1, #4");     //temp, passing svc number over r1
    __asm("   SVC #4");

}


// REQUIRED: modify this function to support 1ms system timer
// execution yielded back to scheduler until time elapses using pendsv
// push registers, set state to delayed, store timeout, call scheduler, pop registers,
// return to new function (separate unrun or ready processing)
void sleep(uint32_t tick)
{
    __asm("    MOV R1, #8");    //temp, passing svc number over r1
    __asm("    SVC #8");

}

// REQUIRED: modify this function to wait a semaphore with priority inheritance
// return if avail (separate unrun or ready processing), else yield to scheduler using pendsv
void wait(struct semaphore *pSemaphore)
{
    __asm("     MOV R1, #12");
    __asm("     SVC #12");
}

// REQUIRED: modify this function to signal a semaphore is available using pendsv
void post(struct semaphore *pSemaphore)
{
    __asm("     MOV R1, #14");
    __asm("     SVC #14");
}

// REQUIRED: modify this function to add support for the system timer
// REQUIRED: in preemptive code, add code to request task switch
void systickIsr()
{
    //sleep(getRO());
    //needs to run at 1kHz rate
    uint8_t i;
    for(i = 0; i < taskCount;i++)
    {
        if(tcb[i].state == STATE_DELAYED)
        {
            tcb[i].ticks -= 1;
            if(tcb[i].ticks == 0)
            {
                tcb[i].state = STATE_READY;
            }
        }
    }
    ///HERE INSERT PRE-EMPTION
    if(preemption)
    {
        NVIC_INT_CTRL_R |=  NVIC_INT_CTRL_PEND_SV ;
    }

}

// REQUIRED: in coop and preemptive, modify this function to add support for task switching
// REQUIRED: process UNRUN and READY tasks differently
void pendSvIsr()
{
    void* sp;
    NVIC_INT_CTRL_R &= ~NVIC_INT_CTRL_VEC_PEN_PNDSV;
    pushR4R11();
    sp=getPSP();
    tcb[taskCurrent].sp=sp;
    taskCurrent=rtosScheduler();
    setPSP(tcb[taskCurrent].sp);
    if(tcb[taskCurrent].state==STATE_READY)
    {
        popR4R11();
    }
    if(tcb[taskCurrent].state == STATE_UNRUN)
    {
        pushfake((void*)0x01000000,tcb[taskCurrent].pid);  //magic number is the value of the xPSR to enable thumb instructions
        tcb[taskCurrent].state=STATE_READY;
    }

}

// REQUIRED: modify this function to add support for the service call
// REQUIRED: in preemptive code, add code to handle synchronization primitives
void svCallIsr()
{
    void * data = getR0();    //getting argument 0
    __asm("    MOV R0, R1");    //getting svc number in r1
    uint32_t svcNum = getR0();
    NVIC_SYS_HND_CTRL_R &= ~NVIC_SYS_HND_CTRL_SVC;//clear SVC flag bit
    //clearing the bit is done afterwords, since it modifies r1 and r0
    //find which switch case function to equal the wanted phase function
    struct semaphore* pSemaphore =0;
    uint32_t pid = 0;
    uint32_t i = 0;
    switch(svcNum)
    {
    case 4: //yield
        NVIC_INT_CTRL_R |=  NVIC_INT_CTRL_PEND_SV ;
        break;
    case 8: //sleep
        tcb[taskCurrent].ticks = data;
        tcb[taskCurrent].state = STATE_DELAYED;
        NVIC_INT_CTRL_R |=  NVIC_INT_CTRL_PEND_SV ;
        break;
    case 12: //wait
        pSemaphore = data;
        data = pSemaphore->count;
        if(data>0)  //note DATA here is equal to the count in the semaphore struct
        {
            pSemaphore->count -= 1;
            pSemaphore->current = tcb[taskCurrent].pid;
            break;
        }
        else
        {
            pSemaphore->processQueue[pSemaphore->queueSize] = tcb[taskCurrent].pid;
            pSemaphore->queueSize += 1;
            tcb[taskCurrent].state = STATE_BLOCKED;
            tcb[taskCurrent].semaphore = pSemaphore;
            NVIC_INT_CTRL_R |=  NVIC_INT_CTRL_PEND_SV ;
            //Priority Inheretance
            if(priorityinheritance)
            {
                for(i = 0; i<MAX_TASKS;i++)
                {
                    if(tcb[i].pid == pSemaphore->current)
                    {
                        if(tcb[i].currentPriority<tcb[taskCurrent].priority)
                        {
                            tcb[i].currentPriority = tcb[taskCurrent].priority;
                            break;
                        }
                    }
                }
            }
        }
        break;
    case 14: //post
        pSemaphore = data;
        pSemaphore->count +=1;
        tcb[taskCurrent].currentPriority = tcb[taskCurrent].priority;   //returning to normal priority after PI
        if(pSemaphore->queueSize>0)
        {
            pSemaphore->queueSize -=1;
            pSemaphore->count -=1;
            pid = pSemaphore->processQueue[pSemaphore->queueSize];
            //find process from pid
            for(i = 0; i<MAX_TASKS;i++)
            {
                if(tcb[i].pid == pid)
                {
                    tcb[i].state = STATE_READY;
                    break;
                }
            }
            for(i=0; i<MAX_QUEUE_SIZE; i++)
            {
                if (i==0)
                {
                    pSemaphore->current = pSemaphore->processQueue[i];

                    pSemaphore->processQueue[i]=pSemaphore->processQueue[i+1];
                }
                else if((i!=0)&&(i!=4))
                {
                    pSemaphore->processQueue[i]=pSemaphore->processQueue[i+1];
                }
                else
                {
                    pSemaphore->processQueue[i] = 0;
                }
            }

        }
         break;
    case 20:
            //pidof
            while(tcb[i].name !=data &&i<10){
                i++;
            }
            //display number
            putsUart0(tcb[i].pid);
            break;
       case 24:
            //sched ON
           scheduler=ON;
           break;
        case 28:
            //sched OFF
            scheduler=OFF;
            break;
        case 32:
            //preempt ON
            preemption=ON;
            break;
        case 36:
            //preempt OFF
            preemption=OFF;
            break;
        case 40:
            // kill send to destroyThread
            break;
        case 44:
            priorityinheritance=ON;
            break;
        case 48:
            priorityinheritance=OFF;
            break;
        case 52:
            tohex(tcb[0].pid);
            putsUart0("   ");
            while(tcb[0].name[i]!=0){
            putcUart0(tcb[0].name[i]);
            i++;
            }
            toint(tcb[0].upperpercentage);
                                putcUart0('.');
                                toint(tcb[0].lowerpercentage);
             putcUart0('\n');

            break;
        case 53:
            tohex(tcb[1].pid);
            putsUart0("   ");
            while(tcb[1].name[i]!=0){
                       putcUart0(tcb[1].name[i]);
                       i++;
                       }
            toint(tcb[1].upperpercentage);
                                putcUart0('.');
                                toint(tcb[1].lowerpercentage);
               putcUart0('\n');

            break;
        case 54:
            tohex(tcb[2].pid);
            putsUart0("   ");
            while(tcb[2].name[i]!=0){
                       putcUart0(tcb[2].name[i]);
                       i++;
                       }
            toint(tcb[2].upperpercentage);
                                putcUart0('.');
                                toint(tcb[2].lowerpercentage);
                        putcUart0('\n');

           break;
        case 55:
            tohex(tcb[3].pid);
            putsUart0("   ");
            while(tcb[3].name[i]!=0){
                       putcUart0(tcb[3].name[i]);
                       i++;
                       }
            toint(tcb[0].upperpercentage);
                  putcUart0('.');
                                toint(tcb[0].lowerpercentage);
                        putcUart0('\n');
            break;
        case 56:
            tohex(tcb[4].pid);
             putsUart0("   ");
            while(tcb[4].name[i]!=0){
                       putcUart0(tcb[4].name[i]);
                       i++;
                       }
            toint(tcb[4].upperpercentage);
                                putcUart0('.');
                                toint(tcb[4].lowerpercentage);
                        putcUart0('\n');

            break;
       case 57:
           tohex(tcb[5].pid);
           putsUart0("   ");
           while(tcb[5].name[i]!=0){
                      putcUart0(tcb[5].name[i]);
                      i++;
                      }
           toint(tcb[5].upperpercentage);
                               putcUart0('.');
                               toint(tcb[5].lowerpercentage);
                       putcUart0('\n');
           break;
       case 58:
           tohex(tcb[6].pid);
           putsUart0("   ");
           while(tcb[6].name[i]!=0){
                      putcUart0(tcb[6].name[i]);
                      i++;
                      }
                      toint(tcb[6].upperpercentage);
                      putcUart0('.');
                      toint(tcb[6].lowerpercentage);
                      putcUart0('\n');

          break;
       case 59:
           tohex(tcb[7].pid);
           putsUart0("   ");
           while(tcb[7].name[i]!=0){
                      putcUart0(tcb[7].name[i]);
                      i++;
                      }
           toint(tcb[7].upperpercentage);
                               putcUart0('.');
                               toint(tcb[7].lowerpercentage);
                       putcUart0('\n');
          break;
       case 60:
           tohex(tcb[8].pid);
           putsUart0("   ");
           while(tcb[8].name[i]!=0){
                      putcUart0(tcb[8].name[i]);
                      i++;
                      }
           toint(tcb[8].upperpercentage);
                               putcUart0('.');
                               toint(tcb[8].lowerpercentage);
                      putcUart0('\n');
          break;
        case 61:
            tohex(tcb[9].pid);
            putsUart0("   ");
            while(tcb[9].name[i]!=0){
                       putcUart0(tcb[9].name[i]);
                       i++;
                       }
            toint(tcb[9].upperpercentage);
                                putcUart0('.');
                                toint(tcb[9].lowerpercentage);
                        putcUart0('\n');
          break;
       case 62:
          break;
       case 63:
             break;
        case 66:
            NVIC_APINT_R |= 0x04 | (0x05FA << 16);

        break;
    }



}

// REQUIRED: code this function
void mpuFaultIsr()
{
}

// REQUIRED: code this function
void hardFaultIsr()
{
}

// REQUIRED: code this function
void busFaultIsr()
{
}

// REQUIRED: code this function
void usageFaultIsr()
{
}

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

// Initialize Hardware
// REQUIRED: Add initialization for blue, orange, red, green, and yellow LEDs
//           5 pushbuttons, and uart
void initHw()
{

    // Configure HW to work with 16 MHz XTAL, PLL enabled, system clock of 40 MHz
        SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);

        // Set GPIO ports to use APB (not needed since default configuration -- for clarity)
        // Note UART on port A must use APB
        SYSCTL_GPIOHBCTL_R = 0;

        // Enable GPIO port A and F peripherals
        SYSCTL_RCGC2_R = SYSCTL_RCGC2_GPIOA | SYSCTL_RCGC2_GPIOF|SYSCTL_RCGC2_GPIOE;

        // Configure LED pins
        GPIO_PORTF_DIR_R = 0x04;  // bits 1 and 3 are outputs, other pins are inputs 1010
        GPIO_PORTF_DR2R_R = 0x04; // set drive strength to 2mA (not needed since defaultconfiguration -- for clarity) 0011
        GPIO_PORTF_DEN_R = 0x4;  //

        GPIO_PORTE_DIR_R = 0x1E;  // bits 1 and 3 are outputs, other pins are inputs 1010
        GPIO_PORTE_DR2R_R = 0x1E; // set drive strength to 2mA (not needed since defaultconfiguration -- for clarity) 0011
        GPIO_PORTE_DEN_R = 0x1E;  // enable LEDs and pushbuttons 1010

        // Configure UART0 and pushbuttons pins
        SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R0;         // turn-on UART0, leave other uarts in same status
        GPIO_PORTA_DEN_R |= 0xFF;               // default, added for clarity
        GPIO_PORTA_AFSEL_R |= 0xFF;                    // default, added for clarity
        GPIO_PORTA_PUR_R = 0xFC;  // enable internal pull-up for push button
        GPIO_PORTA_PCTL_R = GPIO_PCTL_PA1_U0TX | GPIO_PCTL_PA0_U0RX;

        // Configure UART0 to 115200 baud, 8N1 format (must be 3 clocks from clock enable and config writes)
        UART0_CTL_R = 0;                                 // turn-off UART0 to allow safe programming
        UART0_CC_R = UART_CC_CS_SYSCLK;                  // use system clock (40 MHz)
        //from the  divison
        UART0_IBRD_R = 21;                               // r = 40 MHz / (Nx115.2kHz), set floor(r)=21, where N=16
        UART0_FBRD_R = 45;                               // round(fract(r)*64)=45
        UART0_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN; // configure for 8N1 w/ 16-level FIFO
        UART0_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN; // enable TX, RX, and module
        SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R0;         // turn-on UART0, leave other UARTs in same status

        //systick timer
        NVIC_ST_RELOAD_R = 0x9C40;  //1KHz @ 40MHz
        NVIC_ST_CURRENT_R = 0x0;
        NVIC_ST_CTRL_R = NVIC_ST_CTRL_CLK_SRC|NVIC_ST_CTRL_INTEN;//use 40MHz clock, enable systick interrupt
        NVIC_EN0_R = 0x2000;    //enable Systick ISr

        SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1;
        TIMER1_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer before reconfiguring
        TIMER1_CFG_R = TIMER_CFG_32_BIT_TIMER;           // configure as 32-bit timer (A+B)
        TIMER1_TAMR_R = TIMER_TAMR_TAMR_PERIOD;          // configure for periodic mode (count down
        TIMER1_TAILR_R = 40000;                      // set load value to 40e6 for 1 Hz interrupt rate
        TIMER1_IMR_R = TIMER_IMR_TATOIM;                 // turn-on interrupts
        NVIC_EN0_R |= 1 << (INT_TIMER1A-16);

}

void TimerISR(){

    int i;
    for(i=0;i<taskCount;i++){
        tcb[i].upperpercentage= (tcb[i].countticks*100)/4000;
        tcb[i].lowerpercentage=(tcb[i].countticks%4000)*100;
        tcb[i].countticks=0;
    }
    TIMER1_ICR_R = TIMER_ICR_TATOCINT;
}

// Approximate busy waiting (in units of microseconds), given a 40 MHz system clock
void waitMicrosecond(uint32_t us)
{
                                              // Approx clocks per us
    __asm("WMS_LOOP0:   MOV  R1, #6");          // 1
    __asm("WMS_LOOP1:   SUB  R1, #1");          // 6
    __asm("             CBZ  R1, WMS_DONE1");   // 5+1*3
    __asm("             NOP");                  // 5
    __asm("             B    WMS_LOOP1");       // 5*3
    __asm("WMS_DONE1:   SUB  R0, #1");          // 1
    __asm("             CBZ  R0, WMS_DONE0");   // 1
    __asm("             B    WMS_LOOP0");       // 1*3
    __asm("WMS_DONE0:");                        // ---
                                                // 40 clocks/us + error
}

// REQUIRED: add code to return a value from 0-64 indicating which of 5 PBs are pressed
uint8_t readPbs()
{

    uint8_t value=0;
    if(!PB0){
        //RED_LED^=1;   //these commented out sections were used to test HW
        value= value + 1;
    }if(!PB1){
        //BLUE_LED^=1;
        value= value + 2;
    }if(!PB2){
        //RED_LED^=1;
        //YELLOW_LED^=1;
        value = value + 4;
    }if(!PB3){
        //GREEN_LED^=1;
        value =value + 8;
    }if(!PB4){
        //YELLOW_LED^=1;
        value= value + 16;
    }if(!PB5){
        //ORANGE_LED^=1;
        value=value + 32;
    }
    return value;
}

//-----------------------------------------------------------------------------
// YOUR UNIQUE CODE
// REQUIRED: add any custom code in this space
//-----------------------------------------------------------------------------
void modePSP(){

    __asm(" MOV R0, #2");
    __asm(" MSR CONTROL, R0");
    __asm(" BX LR");

}
void ipcs(){
    putsUart0("Semaphore #          Name              \n");
    putsUart0("==============================================\n");
    char line[45];
    __asm(" MOV R1, #62");
    __asm(" SVC #62");
    putsUart0(line);
    __asm(" MOV R1, #63");
    __asm(" SVC #63");
   putsUart0(line);
   __asm(" MOV R1, #64");
   __asm(" SVC #64");
   putsUart0(line);
   __asm(" MOV R1, #65");
   __asm(" SVC #65");
   putsUart0(line);
   __asm(" MOV R1, #66");
   __asm(" SVC #66");
    putsUart0(line);

}
void ps(){
    // will put the actual names from the svc
    putsUart0("PID#          Name            % of CPU Time  \n");
    putsUart0("==============================================\n");
    char line[45];

    __asm(" MOV R1, #52");
    __asm(" SVC #52");
    putsUart0(line);
    __asm(" MOV R1, #53");
     __asm(" SVC #53");
     putsUart0(line);
     __asm(" MOV R1, #54");
     __asm(" SVC #54");
     putsUart0(line);
     __asm(" MOV R1, #55");
     __asm(" SVC #55");
     putsUart0(line);
      __asm(" MOV R1, #56");
      __asm(" SVC #56");
      putsUart0(line);
      __asm(" MOV R1, #57");
      __asm(" SVC #57");
      putsUart0(line);
      __asm(" MOV R1, #58");
      __asm(" SVC #58");
      putsUart0(line);
      __asm(" MOV R1, #59");
      __asm(" SVC #59");
      putsUart0(line);
      __asm(" MOV R1, #60");
      __asm(" SVC #60");
      putsUart0(line);
      __asm(" MOV R1, #61");
      __asm(" SVC #61");
      putsUart0(line);
}
void pi(bool on){
    if(on){
        __asm(" MOV R1, #44");
               __asm(" SVC #44");
        putsUart0("pi ON\n");
    }
    else{
        __asm(" MOV R1, #48");
               __asm(" SVC #48");
        putsUart0("pi OFF\n");
    }
}
void sched(bool on){
    if(on){
        __asm(" MOV R1, #24");
               __asm(" SVC #24");
        putsUart0("sched PRIO\n");
    }
    else{
        __asm(" MOV R1, #28");
               __asm(" SVC #28");
        putsUart0("sched RR\n");
    }
}
void preempt(bool on){
    if(on){
        __asm(" MOV R1, #32");
        __asm(" SVC #32");
        putsUart0("preempt ON\n");
    }
    else{
        __asm(" MOV R1, #36");
        __asm(" SVC #36");
        putsUart0("preempt OFF\n");
    }
}
void pidof(char *name, struct args a){
    __asm(" MOV R1, #20");

     __asm(" SVC #20");

}
void kill(int pid){

   __asm(" MOV R1, #24");
   __asm("  SVC #24 ");
   char result[9];
   int i=0, digits=0, temp=1;
   while(temp<pid){
           temp=temp*10;
           digits++;


   }
   temp=temp/10;

    while(pid>0){
            result[i]=((int)(pid/temp)+48);

            i++;
            pid= pid-((int)(pid/temp)*temp);
            temp=temp/10;
    }
    putsUart0(result);
    putsUart0(" killed \n\n");

}
void clearScreen()
{
    putcUart0(27);  //esc
    putsUart0("[2J");
    putcUart0(27);  //esc
    putsUart0("[H");
}

char toLower(char c)
{
    if((c>='A')&&(c<='Z'))
    {
        c = c + 32;//offset to change ascii capital letter to lowercase
    }
    return c;
}

void blinkGreenLED()
{
    GREEN_LED ^= 1;
}

/* Step 2
 * This function will take character input from uart0 (incoming from the terminal)
 * and will return a string with acquired values. It does basic character validation
 * and has backspace support
 * Warnings: calls a Blocking Function
 */
//Warning, uses c lib tolower() FIXED
void getsUart0(uint8_t maxChars, char *inputstring)
{
    uint8_t count = 0;
    putcUart0('>');
    while(1)
    {
        char c = getcUart0();   //Blocking func
        if(c == 13) //carriage return
        {
            inputstring[count] = 0;

            putsUart0("\r\n");
            return;
        }
        else if(c == 8) //backspace
        {
            if(count > 0)
            {
                count--;
                putcUart0(8);
                putcUart0(' ');
                putcUart0(8);
            }
        }
        else if(c >= ' ') //valid character
        {
            inputstring[count] = toLower(c);
            putcUart0(inputstring[count]);
            count++;

        }
        if(count == maxChars)
        {
            inputstring[count] = 0;
            blinkGreenLED();
            putsUart0("\r\n");
            return;
        }
    }

}

/* Step 3
 * lightweight parser, will be used to create pointers to main string to where
 * the useful data is located, will replace all other characters in the string
 * with null
 */
void parceStr(struct args* a)
{
    uint8_t i = 0;
    uint8_t d = 1;  //Delimiter flag
    a->fieldCount = 0;

    while(true)
    {
        if(a->fieldCount == 4) // This is to avoid overflowing the size 4 matrixes
        {
            return;
        }
        if(a->StringInput[i] == 0) //end of command string <null>
        {
            return;
        }
        if(((a->StringInput[i]>= 45)&&(a->StringInput[i]<=57))&&(a->StringInput[i]!=47)) //number type
        {
            if(d == 1)
            {
                d = 0;
                a->type[a->fieldCount] = 'n';
                a->pos[a->fieldCount++] = i;
            }
        }
        else if((a->StringInput[i]>= 97)&&(a->StringInput[i]<=122))    //alpha type
        {
            if(d == 1)
            {
                d = 0;
                a->type[a->fieldCount] = 'a';
                a->pos[a->fieldCount++] = i;
            }
        }
        else if(a->StringInput[i]=='_'&&a->fieldCount==1 )    //alpha type
                {
                    if(d == 1)
                    {
                        d = 0;
                        a->type[a->fieldCount] = 'a';
                        a->pos[a->fieldCount++] = i;
                    }
                }
        else    // Assume delimiter
        {
            a->StringInput[i] = 0;
            d = 1;
        }
        i++;
    }
}

/*Step 4
 * helper functions for step 5
 * isCommand returns true when the first value of the string equals the passed
 * command string, and has the number of minimum arguments
 *
 * getValue returns the raw bit value of a number in a string
 *
 * getString returns the substring requested
 *
 * NOTE: that both get helper functions are offset by one, to ignore the command
 * that is in position zero
 */
// Acquired from http://www.strudel.org.uk/itoa/
char * itoa (int value, char *result, int base){
    // check that the base if valid
    if (base < 2 || base > 36) { *result = '\0'; return result; }

    char* ptr = result, *ptr1 = result, tmp_char;
    int tmp_value;

    do {
        tmp_value = value;
        value /= base;
        *ptr++ = "zyxwvutsrqponmlkjihgfedcba9876543210123456789abcdefghijklmnopqrstuvwxyz" [35 + (tmp_value - value * base)];
    } while ( value );

    // Apply negative sign
    if (tmp_value < 0) *ptr++ = '-';
    *ptr-- = '\0';
    while (ptr1 < ptr) {
        tmp_char = *ptr;
        *ptr--= *ptr1;
        *ptr1++ = tmp_char;
    }
    return result;
}
//Warning, uses c lib strcmp()
bool isCommand(char* strVerb, uint8_t minArgs, struct args a)
{
    int i=0;
    if(a.fieldCount < minArgs ) return false;
    while(a.StringInput[i]!='\0'){
        i++;
        strVerb++;
        if(a.StringInput[i]!=*strVerb){
                    return false;
                }
        if(a.StringInput[i]=='\0'&& *strVerb!='\0'){
            return false;
        }
        if(a.StringInput[i]!='\0'&& *strVerb=='\0'){
                    return false;
                }

    }
    return true;
}
//Warning, uses c lib, atoi()
uint16_t getValue(int8_t argNum, struct args a){
    return atoi(&a.StringInput[a.pos[argNum+1]]);
}
char* getString(uint8_t argNum, struct args a){
    return &a.StringInput[a.pos[argNum +1]];
}
void MPUinit(){

     //3 = full accesss

    NVIC_SYS_HND_CTRL_R |= NVIC_SYS_HND_CTRL_USAGE | NVIC_SYS_HND_CTRL_BUS | NVIC_SYS_HND_CTRL_MEM;

     //Create a full-access MPU aperature for RAM, peripherals, and bitbanded
        //addresses with RW access for privileged and unprivileged access.
    NVIC_MPU_NUMBER_R = 0x0;
        NVIC_MPU_ATTR_R = NVIC_MPU_ATTR_XN | 0x03000000 | 0x0002003F;
        NVIC_MPU_BASE_R = 0;

        // Set MPU 1 for whole 256 KB Flash with enabled all sub-regions, full access, and executable for privilege and un-privilege
        NVIC_MPU_NUMBER_R = 0x1;
        NVIC_MPU_ATTR_R = 0x03000000 | 0x00020023;
        NVIC_MPU_BASE_R = 0;
     // 4 MPUs eachNVIC_MPU_BASE_R =  0x000000; one is 1kB
     //Use 4 MPUs regions to cover the 32kiB SRAM (each MPU region covers 8kiB, with 8 subregions
     //of 1kiB each. With RW access for privileged mode and no access for unprivileged mode.
     //Disable all sub- regions.
    /// 7000-7FFF

        NVIC_MPU_NUMBER_R = 0x2;
        NVIC_MPU_ATTR_R = NVIC_MPU_ATTR_XN | 0x01000000 | 0x0002019;
        NVIC_MPU_BASE_R = 0x20000000;

        // Set MPU 3 for region 0x20000000 to 0x20001FFF with enabled all sub-regions, full access for privilege and no-access for un-privilege
        NVIC_MPU_NUMBER_R = 0x3;
        NVIC_MPU_ATTR_R = NVIC_MPU_ATTR_XN | 0x01000000| 0x00020019;
        NVIC_MPU_BASE_R = 0x20002000;

        // Set MPU 4 for region 0x20000000 to 0x20001FFF with enabled all sub-regions, full access for privilege and no-access for un-privilege
        NVIC_MPU_NUMBER_R = 0x4;
        NVIC_MPU_ATTR_R = NVIC_MPU_ATTR_XN | 0x01000000 | 0x00020019;
        NVIC_MPU_BASE_R = 0x20004000;

        // Set MPU 5 for region 0x20000000 to 0x20001FFF with enabled all sub-regions, full access for privilege and no-access for un-privilege
        NVIC_MPU_NUMBER_R = 0x5;
        NVIC_MPU_ATTR_R = NVIC_MPU_ATTR_XN | 0x01000000 | 0x00020019;
        NVIC_MPU_BASE_R = 0x20006000;
    NVIC_MPU_CTRL_R = 0x4 | 0x1;

}
void MPUtest(int number){
    int fault;
    uint32_t status;
    char* address= 0x20000000;
    char c='3';
    if(number== 1){
            putsUart0("\nWe are testing the flash write\n");
            fault = 0;


            *address= 33;

            if((fault == 1) && (status == 0x82))
            {
                putsUart0("OK\n");
            }
            else
            {

                putsUart0("NOK\n");
            }
    }
    else if(number==2){
        putsUart0("\nWe are testing the SRAM subregion write\n");
                   fault = 0;
                   //status=NVIC_FAULT_STAT_R;

                   c=*address;


                   if((fault == 1) && (status == 0x82))
                   {
                       putsUart0("OK\n");
                   }
                   else
                   {

                       putsUart0("NOK\n");
                   }
    }
}
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
        waitMicrosecond(1000);
        ORANGE_LED = 0;
        yield();
    }
}
void idle2()
{
    while(true)
    {
        BLUE_LED = 1;
        waitMicrosecond(1000);
        BLUE_LED = 0;
        yield();
    }
}

void flash4Hz()
{
    while(true)
    {
        GREEN_LED ^= 1;
        sleep(125);
        //yield();
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
        while (readPbs() == 8)
        {
        }
        yield();
    }
}

void errant()
{
    uint32_t* p = (uint32_t*)0x20000000;
    while(true)
    {
        while (readPbs() == 32)
        {
            *p = 0;
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
void shell()
{
    struct args a;
    while (true)
    {


        a.address=1;
        a.trash=0;
        a.fieldCount = 0;


       // get the string and process its info

       getsUart0(MAX_CHARS,a.StringInput);
       parceStr(&a);

       // find the command
        if(isCommand("reboot", 0, a)){

            __asm(" MOV R1, #22");
           __asm(" SVC #22");


        }
        else if(isCommand("kill", 1,a)){

            kill(getValue(0,a));



        }
        else if(isCommand("ps", 0,a)){
                   ps();


               }
        else if(isCommand("ipcs", 0,a)){
            ipcs();


        }
        else if(isCommand("pidof", 1,a)){

                   pidof( getString(0,a),a);

               }
        else if(isCommand("pi", 1,a)){

             pi(getValue(0, a));

                    }

        else if(isCommand("preempt", 1,a)){

            preempt(getValue(0, a));

        }
        else if(isCommand("sched", 1,a)){

            sched(getValue(0, a));

        }

        else if(isCommand("proc_name", 1,a)){
            RED_LED ^=1;// ***** this needds to be done********

        }
        else{
             putsUart0("Command not found\n");

        }
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

    GREEN_LED ^= 1;
    waitMicrosecond(250000);
    GREEN_LED ^= 1;
    waitMicrosecond(250000);

    // Initialize semaphores
    keyPressed = createSemaphore(1);
    keyReleased = createSemaphore(0);
    flashReq = createSemaphore(5);
    resource = createSemaphore(1);

    // Add required idle process at lowest priority
    ok =  createThread(idle, "Idle", 15, 1024);
    //ok &=  createThread(idle2, "Idle 2", 15, 1024);

    // Add other processes

    ok &= createThread(lengthyFn, "LengthyFn", 12, 1024);
    ok &= createThread(flash4Hz, "Flash4Hz", 8, 1024);
    ok &= createThread(oneshot, "OneShot", 4, 1024);
    ok &= createThread(readKeys, "ReadKeys", 12, 1024);
    ok &= createThread(debounce, "Debounce", 12, 1024);
    ok &= createThread(important, "Important", 0, 1024);
    ok &= createThread(uncooperative, "Uncoop", 10, 1024);
    ok &= createThread(errant, "Errant", 8, 1024);
    ok &= createThread(shell, "Shell", 8, 1024);

    MPUinit();  //enable MPU

    // Start up RTOS
    if (ok)
        startRtos(); // never returns
    else
        RED_LED = 1;

    return 0;
}
