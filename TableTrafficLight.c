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

#define Sgreen	0x61
#define Syellow	0x62
#define Wgreen	0x4C
#define Wyellow	0x54
#define Pgreen	0xA4
#define Pred		0x64
#define Poff		0x24

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
	int out;
	int wait;
	int next[8];
};
 typedef const struct State state_t;

state_t light[11]={
	{Sgreen,HALF_SECOND ,{0,1,0,1,1,1,1,1}},	//0
	{Syellow,HALF_SECOND,{0,2,1,2,4,2,4,2}},	//1
	{Wgreen,HALF_SECOND ,{2,2,3,3,3,3,3,3}},	//2
	{Wyellow,HALF_SECOND,{2,2,0,0,4,4,0,4}},	//3
	{Pgreen,HALF_SECOND ,{4,5,5,5,5,5,5,5}},	//4
	{Pred,HALF_SECOND   ,{6,6,6,6,6,6,6,6}},	//5
	{Poff,HALF_SECOND   ,{7,7,7,7,7,7,7,7}},	//6
	{Pred,HALF_SECOND   ,{8,8,8,8,8,8,8,8}},	//7
	{Poff,HALF_SECOND   ,{9,9,9,9,9,9,9,9}},	//8
	{Pred,HALF_SECOND   ,{10,10,10,10,10,10,10,10}},	//9
	{Poff,HALF_SECOND   ,{4,2,0,0,4,2,0,0}}
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
    GPIO_PORTB_DATA_R = (light[state].out) & 0x3F;
		GPIO_PORTF_DATA_R = (((light[state].out) & 0x40)>>5) | (((light[state].out) & 0x80)>>3); //PF1 and PF3
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
