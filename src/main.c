/* Small example showing how to use the SWIO programming pin to
   do printf through the debug interface */

#include <ch32v003fun.h>
#include <stdio.h>
#include "audio.h"
#include "ch32v003_touch.h"

#define SAMPLE_DELAY 6000		// 48MHz / 6000 = 8KHz
#define TOUCH_THRESHHOLD 6000	// Has to be over this to register as a touch
#define RELEASE_THRESHHOLD 5750 // Has to be under this to register as a release
#define AUDIO_LOOP_DELAY 3500

#define LED_D_ON 100
#define LED_D_OFF (LED_D_ON + 3150)

#define LED_F_ON (LED_D_OFF + 400)
#define LED_F_OFF (LED_F_ON + 2600)

#define LED_I_ON (LED_F_OFF + 200)
#define LED_I_OFF (LED_I_ON + 2000)

#define LED_U_ON (LED_I_OFF + 200)
#define LED_U_OFF (AUDIO_SIZE - 100)
// #define DEBUG

uint16_t idx = 0;
uint8_t button_down = 0;
uint8_t audio_loop_delaying = 0;

void all_dfiu_leds_off()
{
	funDigitalWrite(PC0, FUN_LOW);
	funDigitalWrite(PC1, FUN_LOW);
	funDigitalWrite(PC2, FUN_LOW);
	funDigitalWrite(PC3, FUN_LOW);
}

void audio_start()
{

	all_dfiu_leds_off();
	idx = 0;
	// turn offf audio disable pin
	funDigitalWrite(PD0, FUN_LOW);
	// Enable TIM1
	TIM1->CTLR1 |= TIM_CEN;

	TIM2->CTLR1 |= TIM_CEN;
}

void audio_stop()
{
	all_dfiu_leds_off();
	// turn on audio disable pin. disabled high
	funDigitalWrite(PD0, FUN_HIGH);
	TIM1->CTLR1 &= ~TIM_CEN;
	TIM2->CTLR1 &= ~TIM_CEN;
}

void dfiu_leds_update()
{
	switch (idx)
	{
	case LED_D_ON:
		funDigitalWrite(PC0, FUN_HIGH);
		break;
	case LED_D_OFF:
		funDigitalWrite(PC0, FUN_LOW);
		break;
	case LED_F_ON:
		funDigitalWrite(PC1, FUN_HIGH);
		break;
	case LED_F_OFF:
		funDigitalWrite(PC1, FUN_LOW);
		break;
	case LED_I_ON:
		funDigitalWrite(PC2, FUN_HIGH);
		break;
	case LED_I_OFF:
		funDigitalWrite(PC2, FUN_LOW);
		break;
	case LED_U_ON:
		funDigitalWrite(PC3, FUN_HIGH);
		break;
	case LED_U_OFF:
		funDigitalWrite(PC3, FUN_LOW);
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
			TIM1->CH4CVR = audio_dfiu[AUDIO_SIZE - 1];
		}
		else
		{
			TIM1->CH4CVR = audio_dfiu[idx];
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
	funPinMode(PD0, GPIO_Speed_10MHz | GPIO_CNF_OUT_PP); // Audio Enable
	funPinMode(PC0, GPIO_Speed_10MHz | GPIO_CNF_OUT_PP); // D LED
	funPinMode(PC1, GPIO_Speed_10MHz | GPIO_CNF_OUT_PP); // F LED
	funPinMode(PC2, GPIO_Speed_10MHz | GPIO_CNF_OUT_PP); // I LED
	funPinMode(PC3, GPIO_Speed_10MHz | GPIO_CNF_OUT_PP); // U LED
	// funPinMode( PC4, GPIO_Speed_10MHz | GPIO_CNF_OUT_PP ); //PWM

	// PC4 is T1CH4, 10MHz Output alt func, push-pull
	GPIOC->CFGLR &= ~(0xf << (4 * 4));
	GPIOC->CFGLR |= (GPIO_Speed_10MHz | GPIO_CNF_OUT_PP_AF) << (4 * 4);
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
	// TIM1->CCER |= TIM_CC1NE | TIM_CC1NP;

	// Enable CH4 output, positive pol
	TIM1->CCER |= TIM_CC4E | TIM_CC4P;

	// CH1 Mode is output, PWM1 (CC1S = 00, OC1M = 110)
	// TIM1->CHCTLR1 |= TIM_OC1M_2 | TIM_OC1M_1;

	// CH2 Mode is output, PWM1 (CC1S = 00, OC1M = 110)
	TIM1->CHCTLR2 |= TIM_OC4M_2 | TIM_OC4M_1;

	// Set the Capture Compare Register value to 50% initially
	// TIM1->CH1CVR = 128;
	TIM1->CH4CVR = 128;

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
	RCC->APB2PCENR |= RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOC | RCC_APB2Periph_TIM1 | RCC_APB2Periph_ADC1;
	RCC->APB1PCENR |= RCC_APB1Periph_TIM2;

	gpios_init();
	pwm_init();
	sample_timer_init();

	InitTouchADC();

	// turn on audio disable pin. disabled high
	funDigitalWrite(PD0, FUN_HIGH);
}

void button_state_update()
{

#ifdef DEBUG
	uint32_t cnt = 0;
#endif
	uint32_t touch_val;

	touch_val = ReadTouchPin(GPIOD, 4, 7, 3);

#ifdef DEBUG
	printf("touch_val %d\n", touch_val);
#endif

	// Don't need to debounce this with the delay
	if (touch_val > TOUCH_THRESHHOLD && !button_down)
	{
#ifdef DEBUG
		printf("touch %d\n", cnt);
#endif
		audio_start();
		button_down = 1;
	}

	if (touch_val < RELEASE_THRESHHOLD && button_down)
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

	while (1)
	{
		Delay_Ms(50);
		button_state_update();
	}
}
