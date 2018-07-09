// ***** 0. Documentation Section *****
// TableTrafficLight.c for Lab 10
// Runs on LM4F120/TM4C123
// Index implementation of a Moore finite state machine to operate a traffic light.  
// Daniel Valvano, Jonathan Valvano
// January 15, 2016

// east/west red light connected to PB5
// east/west yellow light connected to PB4
// east/west green light connected to PB3
// north/south facing red light connected to PB2
// north/south facing yellow light connected to PB1
// north/south facing green light connected to PB0
// pedestrian detector connected to PE2 (1=pedestrian present)
// north/south car detector connected to PE1 (1=car present)
// east/west car detector connected to PE0 (1=car present)
// "walk" light connected to PF3 (built-in green LED)
// "don't walk" light connected to PF1 (built-in red LED)

// ***** 1. Pre-processor Directives Section *****
#include "TExaS.h"
#include "tm4c123gh6pm.h"

#define GoS			0
#define WaitS		1
#define GoW			2
#define WaitW		3
#define GoP			4
#define RedP1		5
#define FlashP1	6
#define RedP2		7	
#define FlashP2	8	
#define RedP3		9
#define FlashP3	10

#define Sgreen	0x21
#define Syellow	0x22
#define Wgreen	0x0C
#define Wyellow	0x14
#define WSred		0x24
#define Pgreen	0x08
#define Pred		0x02
#define Poff		0x00

#define HALF_SECOND 500
#define SECOND 			1000
// ***** 2. Global Declarations Section *****

// FUNCTION PROTOTYPES: Each subroutine defined
void DisableInterrupts(void); // Disable interrupts
void EnableInterrupts(void);  // Enable interrupts
void PortsInit(void);                            // ports initialization
void SystickInit(void);                          // SysTick initialization
void SystickWait(unsigned long delay);           // SysTick wait function
void SystickWait1ms(unsigned long wait_time_ms); // SysTick delay function

 struct State{
	int B_out;
	int F_out;
	int wait;
	int next[8];
};
 typedef const struct State state_t;

state_t light[11]={
	{Sgreen,Pred,HALF_SECOND ,{GoS,WaitS,GoS,WaitS,WaitS,WaitS,WaitS,WaitS}},	//0
	{Syellow,Pred,HALF_SECOND,{GoS,GoW,WaitS,GoW,GoP,GoW,GoP,GoW}},	//1
	{Wgreen,Pred,HALF_SECOND ,{GoW,GoW,WaitW,WaitW,WaitW,WaitW,WaitW,WaitW}},	//2
	{Wyellow,Pred,HALF_SECOND,{GoW,GoW,GoS,GoS,GoP,GoP,GoS,GoP}},	//3
	{WSred,Pgreen,HALF_SECOND ,{GoP,RedP1,RedP1,RedP1,GoP,RedP1,RedP1,RedP1}},	//4
	{WSred,Pred,HALF_SECOND   ,{FlashP1,FlashP1,FlashP1,FlashP1,FlashP1,FlashP1,FlashP1,FlashP1}},	//5
	{WSred,Poff,HALF_SECOND   ,{RedP2,RedP2,RedP2,RedP2,RedP2,RedP2,RedP2,RedP2}},	//6
	{WSred,Pred,HALF_SECOND   ,{FlashP2,FlashP2,FlashP2,FlashP2,FlashP2,FlashP2,FlashP2,FlashP2}},	//7
	{WSred,Poff,HALF_SECOND   ,{RedP3,RedP3,RedP3,RedP3,RedP3,RedP3,RedP3,RedP3}},	//8
	{WSred,Pred,HALF_SECOND   ,{FlashP3,FlashP3,FlashP3,FlashP3,FlashP3,FlashP3,FlashP3,FlashP3}},	//9
	{WSred,Poff,HALF_SECOND   ,{GoP,GoW,GoS,GoS,GoP,GoW,GoS,GoS}}						//10
};
// ***** 3. Subroutines Section *****
int main(void){ 
	int state=GoS; //initial state is Green for south
	int input=0;
  TExaS_Init(SW_PIN_PE210, LED_PIN_PB543210,ScopeOff); // activate grader and set system clock to 80 MHz
  SystickInit();
  PortsInit();
  EnableInterrupts();
  while(1){
    GPIO_PORTB_DATA_R = light[state].B_out;
		GPIO_PORTF_DATA_R = light[state].F_out ; //PF1 and PF3
		SystickWait1ms(light[state].wait);	//delay
		input=GPIO_PORTE_DATA_R & 0x07;

		state=light[state].next[input];		
  }
}
void PortsInit(void) {
    // clock
    volatile unsigned long delay;

    // 1) activate clock for Port F, Port B, and Port E
    SYSCTL_RCGC2_R |= 0x00000032;
    delay = SYSCTL_RCGC2_R;         // allow time for clock to start
    // Port F
    GPIO_PORTF_LOCK_R = 0x4C4F434B; // 2) unlock GPIO Port F
    GPIO_PORTF_CR_R |= 0x0A;        // allow changes to PF3, PF1
    GPIO_PORTF_AMSEL_R = 0x00;      // 3) disable analog function
    GPIO_PORTF_PCTL_R = 0x00;       // 4) PCTL GPIO on PF3, PF1
    GPIO_PORTF_DIR_R |= 0x0A;       // 5) PF3, PF1 are outputs
    GPIO_PORTF_AFSEL_R = 0x00;      // 6) disable alternate function
    GPIO_PORTF_PUR_R = 0x00;        // disable pull-up resistor
    GPIO_PORTF_DEN_R |= 0x0A;       // 7) enable digital I/O on PF3, PF1
    // Port B
    GPIO_PORTB_LOCK_R = 0x4C4F434B; // 2) unlock GPIO Port B
    GPIO_PORTB_CR_R |= 0x3F;        // allow changes to PB5-PB0
    GPIO_PORTB_AMSEL_R = 0x00;      // 3) disable analog function
    GPIO_PORTB_PCTL_R = 0x00;       // 4) PCTL GPIO on PB5-PB0
    GPIO_PORTB_DIR_R |= 0x3F;       // 5) PB5-PB0 are outputs
    GPIO_PORTB_AFSEL_R = 0x00;      // 6) disable alternate function
    GPIO_PORTB_PUR_R = 0x00;        // disable pull-up resistor
    GPIO_PORTB_DEN_R |= 0x3F;       // 7) enable digital I/O on PB5-PB0
    // Port E
    GPIO_PORTE_LOCK_R = 0x4C4F434B; // 2) unlock GPIO Port E
    GPIO_PORTE_CR_R |= 0x07;        // allow changes to PE2-PE0
    GPIO_PORTE_AMSEL_R = 0x00;      // 3) disable analog function
    GPIO_PORTE_PCTL_R = 0x00;       // 4) PCTL GPIO on PE2-PE0
    GPIO_PORTE_DIR_R = 0x00;        // 5) PE2-PE0 are inputs
    GPIO_PORTE_AFSEL_R = 0x00;      // 6) disable alternate function
    GPIO_PORTE_PUR_R = 0x00;        // disable pull-up resistor
    GPIO_PORTE_DEN_R |= 0x07;       // 7) enable digital I/O on PE2-PE0
}

/**
 * Initializes SysTick timer
 */
void SystickInit(void) {
    NVIC_ST_CTRL_R = 0;          // disable SysTick during set up
    NVIC_ST_RELOAD_R = 0xFFFFFF; // maximum value to RELOAD register
    NVIC_ST_CURRENT_R = 0;       // overwrite to CURRENT to clear it
    NVIC_ST_CTRL_R = 0x05;       // enable CLK_SRC bit and ENABLE bit
}

/**
 * Delays the program
 *
 * @param  delay  count value
 *
 * @assumption    80-MHz clock
 *
 * @notes         delay = Time_To_Delay_In_Seconds / 12.5 / 0.000000001
 */
void SystickWait(unsigned long delay) {
    NVIC_ST_RELOAD_R = delay - 1; // number of counts to wait
    NVIC_ST_CURRENT_R = 0;        // overwrite the CURRENT register

    // wait until COUNT is flagged:
    while ((NVIC_ST_CTRL_R & 0x00010000) == 0) {}
}

/**
 * Delays the program some milliseconds
 *
 * @param  wait_time_ms  milliseconds to delay
 *
 * @assumption          80-MHz clock
 */
void SystickWait1ms(unsigned long wait_time_ms) {
    unsigned long i;

    for (i = 0; i < wait_time_ms; i++) {
        // count = 0.001 / 12.5 / 0.000000001 = 80000
        // equivalent to 1 millisecond
        SystickWait(80000);
    }
}
