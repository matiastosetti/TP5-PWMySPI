#include <stdint.h>
#include "stm32f4xx.h"			// Header del micro
#include "stm32f4xx_gpio.h"		// Perifericos de E/S
#include "stm32f4xx_rcc.h"		// Para configurar el (Reset and clock controller)
#include "stm32f4xx_tim.h"		// Modulos Timers
#include "stm32f4xx_exti.h"		// Controlador interrupciones externas
#include "stm32f4xx_syscfg.h"	// configuraciones Generales
#include "misc.h"				// Vectores de interrupciones (NVIC)
#include "bsp.h"
#include "LIS3DSH.h"
#include "stm32f4xx_adc.h"//Paso 1 continuaciòn. como en este archivo usamos funciones de la librería ADC hay que incluir el .h del ADC
#define LED_V GPIO_Pin_12
#define LED_N GPIO_Pin_13
#define LED_R GPIO_Pin_14
#define LED_A GPIO_Pin_15

//Paso 6: defino pines con respecto a numero de led
#define LED_0 GPIO_Pin_0
#define LED_1 GPIO_Pin_1
#define LED_2 GPIO_Pin_2
#define LED_3 GPIO_Pin_3
#define LED_4 GPIO_Pin_6
#define LED_5 GPIO_Pin_7
#define LED_6 GPIO_Pin_10
#define LED_7 GPIO_Pin_11

#define BOTON GPIO_Pin_0

/* Puertos de los leds disponibles */
GPIO_TypeDef* leds_port[] = { GPIOD, GPIOD, GPIOD, GPIOD, GPIOD, GPIOD, GPIOD,
		GPIOD, GPIOD, GPIOD, GPIOD, GPIOD };
/* Leds disponibles */
const uint16_t leds[] = { LED_V, LED_R, LED_N, LED_A };

//Paso 7: creo arreglo para los 8 leds de la placa de expansiòn que se usarà como bumetro
const uint16_t bumetro[] = { LED_0, LED_1, LED_2, LED_3, LED_4, LED_5, LED_6,
		LED_7 };

uint32_t* const leds_pwm[] = { &TIM4->CCR1, &TIM4->CCR3, &TIM4->CCR2,
		&TIM4->CCR4 };

extern void APP_ISR_sw(void);
extern void APP_ISR_1ms(void);

volatile uint16_t bsp_contMS = 0;

void bumetroSet(uint8_t NumeroLedsOn) { //funcion de manejo bumetro

	uint8_t i;

	for (i = 0; i < NumeroLedsOn; i++) {
		GPIO_SetBits(leds_port[i + 5], bumetro[i]);
	}
	for (i; i < 8; i++) {
		GPIO_ResetBits(leds_port[i + 5], bumetro[i]);
	}
}

void led_on(uint8_t led) {
	GPIO_SetBits(leds_port[led], leds[led]);
}

void led_off(uint8_t led) {
	GPIO_ResetBits(leds_port[led], leds[led]);
}

void led_toggle(uint8_t led) {
	GPIO_ToggleBits(leds_port[led], leds[led]);
}

uint8_t sw_getState(void) {
	return GPIO_ReadInputDataBit(GPIOA, BOTON);
}

void led_setBright(uint8_t led, uint8_t value) {

	*leds_pwm[led] = 10000 * value / 100;
}

void bsp_delayMs(uint16_t x) {
	bsp_contMS = x;

	while (bsp_contMS)
		;

}

//Paso 5: creo función para recoger valor del ADC. //En este mismo paso se completa agregar el header de la funcón en el bsp.h
float bsp_getPote() { //devuelve el valor convertido en el ADC leido del pote en porcentaje float (con coma)

	uint16_t valorPorcentaje = 0;
	// Selecciono el canal a convertir
	ADC_RegularChannelConfig(ADC1, 12, 1, ADC_SampleTime_15Cycles);
	ADC_SoftwareStartConv(ADC1);

	// Espero a que la conversión termine
	while (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) != SET)
		;

	// Guardo el valor leido
	valorPorcentaje = ADC_GetConversionValue(ADC1);

	return (valorPorcentaje * 100 / 4095); //como el adc es de 12 bits, tiene 4096 posibles valores (deltas v)

}

/**
 * @brief Interrupcion llamada cuando se preciona el pulsador
 */
//void EXTI0_IRQHandler(void) {
//
//	if (EXTI_GetITStatus(EXTI_Line0) != RESET) //Verificamos si es la del pin configurado
//			{
//		EXTI_ClearFlag(EXTI_Line0); // Limpiamos la Interrupcion
//		// Rutina:
//		APP_ISR_sw();
//	}
//}
/**
 * @brief Interrupcion llamada al pasar 1ms
 */
//void TIM2_IRQHandler(void) {
//
//	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) {
//		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
//
//		APP_ISR_1ms();
//
//		if (bsp_contMS) {
//			bsp_contMS--;
//		}
//	}
//}
void bsp_led_init();
void bsp_sw_init();
void bsp_timer_config();
void bsp_ADC_config(); //PASO 2

//PASO 3
void bsp_init() {
	// No sucede nada si dejo las siguientes funciones sin comentar si andan y no los uso.
	bsp_led_init(); //leds de la placa discovery
//	bsp_pwm_config();
//	bsp_sw_init();
	//bsp_timer_config();
//	LIS3DSH_Init();
//	LIS3DSH_Set_Output(0X47);
	bsp_ADC_config();
}

float bsp_get_acc(char eje) {

	switch (eje) { //sólo se lepuede pasar una variable entera, char, etc pero no punteros ni strings 'x'=caracter
	case 'x': //se agregan dos casos para una única acción. En este caso por si x e smayúscula o minúscula.
	case 'X':
		return LIS3DSH_Get_X_Out(LIS3DSH_Sense_2g);
		break; //no haría falta agregar break ya que el return hace salir del switch. pero por costumbre luego de una acción hay que poner un break.
	case 'y':
		return LIS3DSH_Get_Y_Out(LIS3DSH_Sense_2g);
		break;
	case 'Z':
		return LIS3DSH_Get_Z_Out(LIS3DSH_Sense_2g);
		break;

	default:
		return -999, 9; //se pasa un número que indica error ya que es un imposible este nivel de aceleración
	}
}

/**
 * @brief Inicializa Leds
 */
void bsp_led_init() {
	GPIO_InitTypeDef GPIO_InitStruct;

	// Arranco el clock del periferico
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_15 | GPIO_Pin_14;
	GPIO_InitStruct.GPIO_Pin |= GPIO_Pin_13 | GPIO_Pin_12;
	GPIO_InitStruct.GPIO_Pin |= GPIO_Pin_11 | GPIO_Pin_10; //Paso 4: agrego los demas pines que usan los leds de la placa de expansión (que segun el esquemàtico de la placa tambièn estàn en el puerto D)para que también se configuren.
	GPIO_InitStruct.GPIO_Pin |= GPIO_Pin_7 | GPIO_Pin_6;
	GPIO_InitStruct.GPIO_Pin |= GPIO_Pin_3 | GPIO_Pin_2;
	GPIO_InitStruct.GPIO_Pin |= GPIO_Pin_1 | GPIO_Pin_0;

	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP; // (Push/Pull)
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOD, &GPIO_InitStruct);
}

/**
 * @brief Inicializa SW
 */
void bsp_sw_init() {
	GPIO_InitTypeDef GPIO_InitStruct;

	NVIC_InitTypeDef NVIC_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;

	// Arranco el clock del periferico
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init(GPIOA, &GPIO_InitStruct);

	// Configuro interrupcion

	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource0);

	/* Configuro EXTI Line */
	EXTI_InitStructure.EXTI_Line = EXTI_Line0;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	/* Habilito la EXTI Line Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

/**
 * @brief Inicializa TIM2
 */
void bsp_timer_config(void) {
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStruct;
	NVIC_InitTypeDef NVIC_InitStructure;
	/* Habilito la interrupcion global del  TIM2 */
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* TIM2 habilitado */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	/* Configuracion de la base de tiempo */
	TIM_TimeBaseStruct.TIM_Period = 1000; // 1 MHz bajado a 1 KHz (1 ms)
	TIM_TimeBaseStruct.TIM_Prescaler = (2 * 8000000 / 1000000) - 1; // 8 MHz bajado a 1 MHz
	TIM_TimeBaseStruct.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStruct);
	/* TIM habilitado */
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
	/* TIM2 contador habilitado */
	TIM_Cmd(TIM2, ENABLE);

}

void bsp_pwm_config(void) {
	TIM_TimeBaseInitTypeDef TIM_config;
	GPIO_InitTypeDef GPIO_config;
	TIM_OCInitTypeDef TIM_OC_config;

	/* Habilito el clock */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

	/* Configuro leds como Segunda Funcion */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

	GPIO_config.GPIO_Mode = GPIO_Mode_AF;
	GPIO_config.GPIO_Pin = GPIO_Pin_15 | GPIO_Pin_14 | GPIO_Pin_13 | GPIO_Pin_12;
	GPIO_config.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_config.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_config.GPIO_OType = GPIO_OType_PP;

	GPIO_Init(GPIOD, &GPIO_config);

	GPIO_PinAFConfig(GPIOD, GPIO_PinSource15, GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource14, GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource13, GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource12, GPIO_AF_TIM4);

	TIM_config.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_config.TIM_ClockDivision = 0;
	TIM_config.TIM_Period = 10000;
	TIM_config.TIM_Prescaler = 16 - 1;
	TIM_TimeBaseInit(TIM4, &TIM_config);

	TIM_OC_config.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OC_config.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OC_config.TIM_Pulse = 0;
	TIM_OC_config.TIM_OCPolarity = TIM_OCPolarity_High;

	// CH1 del pwm
	TIM_OC1Init(TIM4, &TIM_OC_config);
	TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);

	//CH2 del pwm
	TIM_OC_config.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OC_config.TIM_Pulse = 0;

	TIM_OC2Init(TIM4, &TIM_OC_config);
	TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);

	//CH3 del pwm
	TIM_OC_config.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OC_config.TIM_Pulse = 0;

	TIM_OC3Init(TIM4, &TIM_OC_config);
	TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);

	//CH4 del pwm
	TIM_OC_config.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OC_config.TIM_Pulse = 0;

	TIM_OC4Init(TIM4, &TIM_OC_config);
	TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);

	TIM_ARRPreloadConfig(TIM4, ENABLE);

	TIM_Cmd(TIM4, ENABLE);

}

//PASO 1
void bsp_ADC_config() {
// Estructuras de configuración
	GPIO_InitTypeDef GPIO_InitStruct;
	ADC_CommonInitTypeDef ADC_CommonInitStruct;
	ADC_InitTypeDef ADC1_InitStruct;

// Habilito los clock a los periféricos
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

// Configuro el pin en modo analógico
	GPIO_StructInit(&GPIO_InitStruct); // Reseteo la estructura
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AIN; // Modo Analógico
	GPIO_Init(GPIOC, &GPIO_InitStruct);

// Configuro el prescaler del ADC
	ADC_CommonStructInit(&ADC_CommonInitStruct);
	ADC_CommonInitStruct.ADC_Prescaler = ADC_Prescaler_Div4;
	ADC_CommonInit(&ADC_CommonInitStruct);

	/* Configuro el ADC  */
	ADC_StructInit(&ADC1_InitStruct);
	ADC1_InitStruct.ADC_Resolution = ADC_Resolution_12b;
	ADC_Init(ADC1, &ADC1_InitStruct);
	ADC_Cmd(ADC1, ENABLE);
}

