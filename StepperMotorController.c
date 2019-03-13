// StepperMotorController.c starter file EE319K Lab 5
// Runs on TM4C123
// Finite state machine to operate a stepper motor.  
// Jonathan Valvano
// January 18, 2019

// Hardware connections (External: two input buttons and four outputs to stepper motor)
//  PA5 is Wash input  (1 means pressed, 0 means not pressed)
//  PA4 is Wiper input  (1 means pressed, 0 means not pressed)
//  PE5 is Water pump output (toggle means washing)
//  PE4-0 are stepper motor outputs 
//  PF1 PF2 or PF3 control the LED on Launchpad used as a heartbeat
//  PB6 is LED output (1 activates external LED on protoboard)

#include "SysTick.h"
#include "TExaS.h"
#include <stdint.h>
#include "../inc/tm4c123gh6pm.h"

void EnableInterrupts(void);
// edit the following only if you need to move pins from PA4, PE3-0      
// logic analyzer on the real board
#define PA4       (*((volatile unsigned long *)0x40004040))
#define PE50      (*((volatile unsigned long *)0x400240FC))
//NW - only wipe, YW - wipe and wash. F/R - foward/reverse ## - angle from home
#define NW00	0
#define YW00	1
#define NWF04	2
#define YWF04	3
#define NWF08	4
#define YWF08	5
#define NWF12	6
#define	YWF12	7
#define	NWF16	8
#define	YWF16	9
#define	NWF20	10
#define	YWF20	11
#define	NWF24	12
#define	YWF24	13	
#define	NWF28	14
#define	YWF28	15
#define	NWF32	16	
#define	YWF32	17
#define	NWF36	18
#define	YWF36	19
#define	NW40	20
#define	YW40	21	
#define	NWR36	22
#define	YWR36	23	
#define	NWR32	24
#define	YWR32	25
#define	NWR28	26
#define	YWR28	27
#define	NWR24	28
#define	YWR24	29
#define	NWR20	30
#define	YWR20	31
#define	NWR16	32
#define	YWR16	33
#define	NWR12	34
#define	YWR12	35
#define	NWR08	36
#define	YWR08	37
#define	NWR04	38	
#define	YWR04	39

void SendDataToLogicAnalyzer(void){
  UART0_DR_R = 0x80|(PA4<<2)|PE50;
}

struct state{
		uint32_t output;
		uint32_t dwell;
		uint8_t next[4];
};
typedef struct state stype_t;

stype_t FSM[40] = {
 {0x01, 50, {NW00,NWF04,YWF04,YWF04}}, {0x21, 50, {NW00,NW00,NWF04,YWF04}},
 {0x02, 50, {NWF08,NWF08,YWF08,YWF08}},{0x22, 50, {NWF08,NWF08,NWF08,YWF08}},
 {0x04, 50, {NWF12,NWF12,YWF12,YWF12}},{0x24, 50, {NWF12,NWF12,NWF12,YWF12}},
 {0x08, 50, {NWF16,NWF16,YWF16,YWF16}},{0x28, 50, {NWF16,NWF16,NWF16,YWF16}},
 {0x10, 50, {NWF20,NWF20,YWF20,YWF20}},{0x30, 50, {NWF20,NWF20,NWF20,YWF20}},
 {0x01, 50, {NWF24,NWF24,YWF24,YWF24}},{0x21, 50, {NWF24,NWF24,NWF24,YWF24}},
 {0x02, 50, {NWF28,NWF28,YWF28,YWF28}},{0x22, 50, {NWF28,NWF28,NWF28,YWF28}},
 {0x04, 50, {NWF32,NWF32,YWF32,YWF32}},{0x24, 50, {NWF32,NWF32,NWF32,YWF32}},
 {0x08, 50, {NWF36,NWF36,YWF36,YWF36}},{0x28, 50, {NWF36,NWF36,NWF36,YWF36}},
 {0x10, 50, {NW40,NW40,YW40,YW40}},{0x30, 50, {NW40,NW40,NW40,YW40}},
 {0x08, 50, {NWR36,NWR36,YWR36,YWR36}},{0x28, 50, {NWR36,NWR36,NWR36,YWR36}},
 {0x04, 50, {NWR32,NWR32,YWR32,YWR32}},{0x24, 50, {NWR32,NWR32,NWR32,YWR32}},
 {0x02, 50, {NWR28,NWR28,YWR28,YWR28}},{0x22, 50, {NWR28,NWR28,NWR28,YWR28}},
 {0x01, 50, {NWR24,NWR24,YWR24,YWR24}},{0x21, 50, {NWR24,NWR24,NWR24,YWR24}},
 {0x10, 50, {NWR20,NWR20,YWR20,YWR20}},{0x30, 50, {NWR20,NWR20,NWR20,YWR20}},
 {0x08, 50, {NWR16,NWR16,YWR16,YWR16}},{0x28, 50, {NWR16,NWR16,NWR16,YWR16}},
 {0x04, 50, {NWR12,NWR12,YWR12,YWR12}},{0x24, 50, {NWR12,NWR12,NWR12,YWR12}},
 {0x02, 50, {NWR08,NWR08,YWR08,YWR08}},{0x22, 50, {NWR08,NWR08,NWR08,YWR08}},
 {0x01, 50, {NWR04,NWR04,YWR04,YWR04}},{0x21, 50, {NWR04,NWR04,NWR04,YWR04}},
 {0x10, 50, {NW00,NW00,YW00,YW00}},{0x30, 50, {NW00,NW00,NW00,YW00}}
 };
stype_t currentState;
uint32_t input;
 
int main(void){ 
  volatile uint32_t delay;
	TExaS_Init(&SendDataToLogicAnalyzer);    // activate logic analyzer and set system clock to 80 MHz
  SysTick_Init();   
	// you initialize your system here
	SYSCTL_RCGCGPIO_R |= 0x13;
	delay = 420;
	GPIO_PORTA_DIR_R &= ~0x30;
	GPIO_PORTA_DEN_R |= 0x30;
	GPIO_PORTE_DIR_R |= 0x3F;
	GPIO_PORTE_DEN_R |= 0x3F;
	GPIO_PORTB_DIR_R |= 0x40;
	GPIO_PORTB_DEN_R |= 0x40;	
	
  EnableInterrupts();   
	
	currentState = FSM[0];
  while(1){
// output
		GPIO_PORTE_DATA_R = currentState.output;
		GPIO_PORTB_DATA_R = (currentState.output & 0x20) << 1;
// wait
		SysTick_Wait1ms(currentState.dwell);
// input
		input = (GPIO_PORTA_DATA_R & 0x30) >> 4;
// next	
		currentState = FSM[currentState.next[input]];
  }
}


