/********************* (C) COPYRIGHT 2016 STMicroelectronics ********************
* File Name          : Finite state machine
* Author             : ILYAS ERAY TUNCAN - 040130303
* Version            : V1
* Date               : 1-November-2016
* Description        : A finite state machine
*											 Led cahnges its state according to buttons state.
*											 First state LED remains close. If button pressed,
*											 Program turns second state and led turn on. During timeout
*											 cycle if button not pressed program turns first state. If 
*											 button pressed program changes third state and led blinks.
*******************************************************************************/

#include "stm32f0xx.h"// Device header
#include "stdbool.h"

/*	
		Definition of LED_GPIO address. 
		Both leds connected to GPIOC so we can define LED port as GPIOC
*/
#define LED_GPIO GPIOC
#define BUTTON_GPIO GPIOA

/*	
		In mode register there are 2 bits for each output.
		These values can fill the mode register as general purpose output for specific pins.
*/
#define LED1_MODER_OUTPUT 	(0x00010000u)
#define LED2_MODER_OUTPUT 	(0x00040000u)
#define BUTTON_MODER_INPUT	~(0x00000003u)


/*
		In output type register user can set pins as open drain or push-pull.
		These values can fill the output type register as open drain for specific pins.
*/
#define LED1_OTYPER_OD (0x0100u)
#define LED2_OTYPER_OD (0x0200u)
#define BUTTON_OTYPER_OD (0x0001u)

/*
		In output speed register user can set pins maximum frequency.
		Lower clock speeds provides less energy usage.
		These values can fill the output speed register as low for specific pins.
*/
#define LED1_OSPEEDR_LOW (0x00000000u)
#define LED2_OSPEEDR_LOW (0x00000000u)
#define BUTTON_OSPEEDR_LOW (0x00000000u)

/*
		In bit set/reset register user can set pins logic high or low.
		These values can fill the bit set/reset register as high or low for sepcific pins.
*/
#define LED1_BSRR_HIGH (0x00000100u)
#define LED1_BSRR_LOW (0x01000000u)
#define LED2_BSRR_HIGH (0x00000200u)
#define LED2_BSRR_LOW (0x02000000u)

#define LED_GPIO_RCC_EN (0x00080000u)
#define BUTTON_GPIO_RCC_EN (0x00020000u)

/*
		Some program constants which can be adjusted by changing defined values
*/
#define WHILE_TIMEOUT_CYCLE 4000
#define BUTTON_DELAY 2000
#define BLINK_SPEED 50

/*
		Created an enumerator type which indicates LED state
		HIGH - Logic high for LED pin.
		LOW - Logic low for LED pin
*/
typedef enum{
	HIGH,
	LOW
}state_t;

/*
		Create an enumerator type which indicates LED's
		LED1 - Discovery board blue led
		LED2 - DIscovery board green led
*/
typedef enum{
	LED1,
	LED2
}output_t;
/*
		Create an enumerator type which indicates system state
		STATE_OFF - First state of program led stays close
		STATE_ON - Second state of program led stays open
		if there is no button change in timeout duration program turns off state
		STATE_BLINK - Last state of prograö led blinks periodicly respect to BLINK_SPEED value
		if there is no button change in timeout duration program turns off state
*/
typedef enum{
	STATE_OFF,
	STATE_ON,
	STATE_BLINK
}led_state_t;

/*
		Function declerations 
*/
void configure_gpio(void);
static void delay(uint32_t cycle);
void digitalWrite(output_t out, state_t state);
bool read_button(void);

/*
		Global variables which can be use in whole program
*/
led_state_t led_state = STATE_OFF;
unsigned int time_out;
unsigned int time_out_old;

/*
		Main program funciton
*/
int main()
{
	uint8_t led_high;
	
	/* Configured gpio */
	configure_gpio();
	
	/* Continious loop */
	while(1){
		
		/* Execute program according to initial state */
		switch(led_state)
		{
			case STATE_OFF:
				digitalWrite(LED1, LOW);
				led_high = 0;
				while(!read_button())
				{
					delay(BUTTON_DELAY);
				}
				while(read_button()){};
				led_state = STATE_ON;
					
				break;
					
			case STATE_ON:
				time_out = WHILE_TIMEOUT_CYCLE;
				digitalWrite(LED1, HIGH);
				led_high = 1;
				while(!read_button())
				{
						time_out--;
						if(!time_out)
						{
							led_state = STATE_OFF;
							break;
						}
						else
						{
							led_state = STATE_BLINK;
						}
						delay(BUTTON_DELAY);
				}
				while(read_button()){};
				break;
				
			case STATE_BLINK:
				time_out = WHILE_TIMEOUT_CYCLE;
				time_out_old = time_out;
				while(!read_button())
				{
						time_out--;
						if(!time_out)
						{
							led_state = STATE_OFF;
							break;
						}
						if(time_out_old - time_out > BLINK_SPEED)
						{
							time_out_old = time_out;
							if(led_high)
							{
								digitalWrite(LED1, LOW);
								led_high = 0;
							}
							else
							{
								digitalWrite(LED1, HIGH);
								led_high = 1;
							}					
						}
						delay(BUTTON_DELAY);
				}
				while(read_button()){};
				led_state = STATE_OFF;
				break;
		}
	}
	
}

/*
		This funciton configures gpio for BUTTON and LED usage according to discovery board.
		Takes no parameter returns nothing.
*/
void configure_gpio(void)
{
	/* Enables BUTTON and LED's clocs */
	RCC->AHBENR |= (LED_GPIO_RCC_EN | BUTTON_GPIO_RCC_EN);
	
	/* Set LED1 and LED2 pins as output */
	LED_GPIO->MODER 	|=  ((LED1_MODER_OUTPUT | LED2_MODER_OUTPUT) & BUTTON_MODER_INPUT);
	
	/* Set LED1 and LED2 pins as open drain output. */	
	LED_GPIO->OTYPER 	&= 	~(LED1_OTYPER_OD | LED2_OTYPER_OD);
	
	/* Set LED1 and LED2 pins  maximum speed low */
	LED_GPIO->OSPEEDR |= 	(LED1_OSPEEDR_LOW | LED2_OSPEEDR_LOW | BUTTON_OSPEEDR_LOW);
}

/*
		Delay system in defined cycle
		Takes parameter cycle for cycle counter returns nothing
*/
void delay(uint32_t cycle)
{
	/* 32 bit unsigned intiger for cycle counter */
	uint32_t i;
	
	/* Count 'i' until reaching cycle value */
	for(i = 0; i < cycle; i++){};
}

/*
		This function changes the output pins state
		Parameter out indicates which pin wants to change
		and user can change state of pin via parameter state
		Returns nothing
*/
void digitalWrite(output_t out, state_t state)
{
	if(out == LED1)
	{
		if(state == HIGH)
		{
			LED_GPIO->BSRR |= (LED1_BSRR_HIGH);
		}
		else
		{
			LED_GPIO->BSRR |= (LED1_BSRR_LOW);
		}
	}
	else if(out == LED2)
	{
		if(state == HIGH)
		{
			LED_GPIO->BSRR |= (LED2_BSRR_HIGH);
		}
		else
		{
			LED_GPIO->BSRR |= (LED2_BSRR_LOW);
		}		
	}
}

/*
		This function reads only user button in discovery board
		Takes no parameter returns state of button
*/
bool read_button(void)
{
	 return (GPIOA->IDR & (0x00000001u));
}

