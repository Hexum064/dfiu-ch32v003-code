

/* Small example showing how to use the SWIO programming pin to
   do printf through the debug interface */

#include <ch32v003fun.h>
#include <stdio.h>
#include "audio.h"
#include "ch32v003_touch.h"

#define LED_D		PD3
#define LED_F		PD4
#define LED_I		PD5
#define LED_U		PD6

#define AUDIO_EN	PA1

#define PWM_OUT		PD0

#define TOUCH_P		PD2

#define DISP_LEDS_P GPIOC
#define LED_0		PC0
#define LED_1		PC1
#define LED_2		PC2
#define LED_3		PC3
#define LED_4		PC4
#define LED_5		PC5
#define LED_6		PC6
#define LED_7		PC7

#define SAMPLE_DELAY 6000		// 48MHz / 6000 = 8KHz
#define TOUCH_SAMPLE_bm 0b100000000
#define AUDIO_LOOP_DELAY 3500
#define BASE_DELAY 50
#define INPUT_DELAY_MULT 2
#define LED_UPDATE_DELAY_MULT 5

#define LED_D_ON 100
#define LED_D_OFF (LED_D_ON + 3150)

#define LED_F_ON (LED_D_OFF + 400)
#define LED_F_OFF (LED_F_ON + 2600)

#define LED_I_ON (LED_F_OFF + 200)
#define LED_I_OFF (LED_I_ON + 2000)

#define LED_U_ON (LED_I_OFF + 200)
#define LED_U_OFF (AUDIO_SIZE - 100)
// #define DEBUG
// #define TOUCH_DEBUG

uint16_t idx = 0;
uint8_t button_down = 0;
uint8_t audio_loop_delaying = 0;
uint8_t display_led_reg = 0x7F;

#ifdef TOUCH_DEBUG
uint32_t touch_val;
#endif

#define DISP_INIT_VAL_LEN 8
#define DISP_UPDATE 5
#define DISP_UPDATE_NEXT_INIT 87

uint8_t display_init_vals[] = {
	0x01,
	0x55,
	0x7F,
	0x70,
	0x11,
	0x0F,
	0x33,
	0x03
};

void all_dfiu_leds_off()
{
	funDigitalWrite(LED_D, FUN_LOW);
	funDigitalWrite(LED_F, FUN_LOW);
	funDigitalWrite(LED_I, FUN_LOW);
	funDigitalWrite(LED_U, FUN_LOW);
}

void audio_start()
{

	all_dfiu_leds_off();
	idx = 0;
	// turn offf audio disable pin
	funDigitalWrite(AUDIO_EN, FUN_LOW);
	// Enable TIM1
	TIM1->CTLR1 |= TIM_CEN;

	TIM2->CTLR1 |= TIM_CEN;
}

void audio_stop()
{
	all_dfiu_leds_off();
	// turn on audio disable pin. disabled high
	funDigitalWrite(AUDIO_EN, FUN_HIGH);
	TIM1->CTLR1 &= ~TIM_CEN;
	TIM2->CTLR1 &= ~TIM_CEN;
}

void dfiu_leds_update()
{
	switch (idx)
	{
	case LED_D_ON:
		funDigitalWrite(LED_D, FUN_HIGH);
		break;
	case LED_D_OFF:
		funDigitalWrite(LED_D, FUN_LOW);
		break;
	case LED_F_ON:
		funDigitalWrite(LED_F, FUN_HIGH);
		break;
	case LED_F_OFF:
		funDigitalWrite(LED_F, FUN_LOW);
		break;
	case LED_I_ON:
		funDigitalWrite(LED_I, FUN_HIGH);
		break;
	case LED_I_OFF:
		funDigitalWrite(LED_I, FUN_LOW);
		break;
	case LED_U_ON:
		funDigitalWrite(LED_U, FUN_HIGH);
		break;
	case LED_U_OFF:
		funDigitalWrite(LED_U, FUN_LOW);
		break;
	}
}

void audio_update()
{
	if (idx >= (AUDIO_SIZE + AUDIO_LOOP_DELAY))
	{

		if (button_down)
		{
			idx = 0;
		}
		else
		{
			audio_stop();
		}
	}
	else
	{
		TIM2->INTFR = TIM_CC1IF;
		if (idx >= AUDIO_SIZE)
		{
			TIM1->CH1CVR = audio_dfiu[AUDIO_SIZE - 1];
		}
		else
		{
			TIM1->CH1CVR = audio_dfiu[idx];
		}
	}
}

void TIM2_IRQHandler(void) __attribute__((interrupt));
void TIM2_IRQHandler(void)
{
	idx++;

	audio_update();
	dfiu_leds_update();
}

void gpios_init()
{
	funPinMode(AUDIO_EN, GPIO_Speed_10MHz | GPIO_CNF_OUT_PP); // Audio Enable
	funPinMode(LED_D, GPIO_Speed_10MHz | GPIO_CNF_OUT_PP); // D LED
	funPinMode(LED_F, GPIO_Speed_10MHz | GPIO_CNF_OUT_PP); // F LED
	funPinMode(LED_I, GPIO_Speed_10MHz | GPIO_CNF_OUT_PP); // I LED
	funPinMode(LED_U, GPIO_Speed_10MHz | GPIO_CNF_OUT_PP); // U LED
	funPinMode(PWM_OUT, GPIO_Speed_10MHz | GPIO_CNF_OUT_PP_AF); //PWM Pin
	funPinMode(LED_0, GPIO_Speed_10MHz | GPIO_CNF_OUT_PP); // display LED
	funPinMode(LED_1, GPIO_Speed_10MHz | GPIO_CNF_OUT_PP); // display LED
	funPinMode(LED_2, GPIO_Speed_10MHz | GPIO_CNF_OUT_PP); // display LED
	funPinMode(LED_3, GPIO_Speed_10MHz | GPIO_CNF_OUT_PP); // display LED
	funPinMode(LED_4, GPIO_Speed_10MHz | GPIO_CNF_OUT_PP); // display LED
	funPinMode(LED_5, GPIO_Speed_10MHz | GPIO_CNF_OUT_PP); // display LED
	funPinMode(LED_6, GPIO_Speed_10MHz | GPIO_CNF_OUT_PP); // display LED
	funPinMode(LED_7, GPIO_Speed_10MHz | GPIO_CNF_OUT_PP); // display LED

	// PC4 is T1CH4, 10MHz Output alt func, push-pull
	// GPIOA->CFGLR &= ~(0xf << (4 * PWM_OUT));
	// GPIOA->CFGLR |= (GPIO_Speed_10MHz | GPIO_CNF_OUT_PP_AF) << (4 * PWM_OUT);
}

void pwm_init()
{
	// Reset TIM1 to init all regs
	RCC->APB2PRSTR |= RCC_APB2Periph_TIM1;
	RCC->APB2PRSTR &= ~RCC_APB2Periph_TIM1;

	// Reset TIM2 to init all regs
	RCC->APB1PRSTR |= RCC_APB1Periph_TIM2;
	RCC->APB1PRSTR &= ~RCC_APB1Periph_TIM2;

	// CTLR1: default is up, events generated, edge align
	// SMCFGR: default clk input is CK_INT

	// Prescaler
	TIM1->PSC = 0x0000;

	// Auto Reload - sets period. Also means we have 8 bits of resolution
	TIM1->ATRLR = 255;

	// Reload immediately
	TIM1->SWEVGR |= TIM_UG;

	// Enable CH1N output, positive pol
	TIM1->CCER |= TIM_CC1NE | TIM_CC1NP;

	// Enable CH4 output, positive pol
	//TIM1->CCER |= TIM_CC2E | TIM_CC2P;

	// CH1 Mode is output, PWM1 (CC1S = 00, OC1M = 110)
	TIM1->CHCTLR1 |= TIM_OC1M_2 | TIM_OC1M_1;

	// CH2 Mode is output, PWM1 (CC1S = 00, OC1M = 110)
	//TIM1->CHCTLR1 |= TIM_OC2M_2 | TIM_OC2M_1;

	// Set the Capture Compare Register value to 50% initially
	 TIM1->CH1CVR = 128;
	//TIM1->CH2CVR = 128;

	// Enable TIM1 outputs
	TIM1->BDTR |= TIM_MOE;
}

void sample_timer_init()
{
	TIM2->CTLR1 = TIM_ARPE;

	TIM2->PSC = 0x0000;

	TIM2->ATRLR = SAMPLE_DELAY;

	TIM2->SWEVGR |= TIM_UG;

	TIM2->DMAINTENR = TIM_UIE;

	NVIC_EnableIRQ(TIM2_IRQn);

	TIM2->CTLR1 &= ~TIM_CEN;
}

void sao_init()
{
	// Enable GPIOs and timers
	RCC->APB2PCENR |= RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD | RCC_APB2Periph_TIM1 | RCC_APB2Periph_ADC1;
	RCC->APB1PCENR |= RCC_APB1Periph_TIM2;

	gpios_init();
	pwm_init();
	sample_timer_init();

	InitTouchADC();

	// turn on audio disable pin. disabled high
	funDigitalWrite(AUDIO_EN, FUN_HIGH);
}

void display_leds_update()
{
	uint8_t temp = display_led_reg & 0x01;
	display_led_reg >>= 1;
	display_led_reg |= (temp << 7);

#ifdef TOUCH_DEBUG
	DISP_LEDS_P->OUTDR = (touch_val >> 2);
#else
	DISP_LEDS_P->OUTDR = display_led_reg;
#endif

}




void button_state_update()
{

#ifdef DEBUG
	uint32_t cnt = 0;
#endif

#ifndef TOUCH_DEBUG
uint32_t touch_val;
#endif
	
	touch_val = ReadTouchPin(GPIOD, 2, 3, 3);

#ifdef DEBUG
	printf("touch_val %d\n", touch_val);
#endif
	
	if ((touch_val & TOUCH_SAMPLE_bm) && !button_down)
	{
#ifdef DEBUG
		printf("touch %d\n", cnt);
#endif
		audio_start();
		button_down = 1;
	}
	
	if (!(touch_val & TOUCH_SAMPLE_bm) && button_down)
	{
#ifdef DEBUG
		printf("release %d\n", cnt);
		cnt++;
#endif
		button_down = 0;
	}
}

int main()
{
	SystemInit();
	sao_init();

	uint8_t input_delay = 0;
	uint8_t update_delay = 0;
	uint8_t disp_next_init_delay = 0;
	uint8_t disp_next_init_cnt = 0;
	uint32_t tmp;

	display_led_reg = display_init_vals[0];

	//A little pause to let the voltage stabilize 
	Delay_Ms(100);

	while (1)
	{
		Delay_Ms(BASE_DELAY);
		

		update_delay++;	
		input_delay++;

		if (input_delay >= INPUT_DELAY_MULT) 
		{
			button_state_update();
			input_delay = 0;
		}

		if (update_delay >= LED_UPDATE_DELAY_MULT)
		{
			disp_next_init_delay++;
			update_delay = 0;
			display_leds_update();
		}
		
		if (disp_next_init_delay >= DISP_UPDATE_NEXT_INIT)
		{
			disp_next_init_delay = 0;
			disp_next_init_cnt++;
			display_led_reg = display_init_vals[disp_next_init_cnt % DISP_INIT_VAL_LEN];
		}

	}
}
