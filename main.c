#include <stdint.h>
#include "bsp/bsp.h"

/**
 * @brief Se encarga de prender un led y apagarlo luego de un tiempo
 *
 * @param led    Numero de led a pulsar
 * @param tiempo Numero de ciclos del delay entre prendido y apagado
 */
void ledPulso(uint8_t led, uint32_t tiempo);

/**
 * @brief Aplicacion principal
 */
int main(void) {
	bsp_init();
	int brillo = 0;
	int flag = 0;
	float acc_x;
	float acc_y;
	float acc_z;

	while (1) {
		bsp_delayMs(100);

//		led_setBright(0,brillo);
//		led_setBright(1,brillo);
//		led_setBright(2,brillo);
//		led_setBright(3,brillo);
//
//		if(brillo >= 100)
//			flag = 0;
//		if(brillo <=0)
//			flag = 1;
//
//		if(flag)
//			brillo++;
//		else
//			brillo--;

		acc_x = bsp_get_acc('x');
		acc_y = bsp_get_acc('y');

		if (acc_x > 0) {
			led_setBright(0, acc_x * 1000);
			led_setBright(2, 0);
		} else {
			led_setBright(0, 0);
			led_setBright(2, acc_x * -100);
		}
		if (acc_y > 0) {
			led_setBright(1, acc_y * 100);
			led_setBright(3, 0);
		} else {
			led_setBright(1, 0);
			led_setBright(3, acc_y * -100);
		}
		acc_z = bsp_get_acc('z');
	}
}

/**
 * @brief Se preciono el pulsador
 *
 */
void APP_ISR_sw(void) {

}

/**
 * @brief Interrupcion cada 1ms
 *
 */
void APP_ISR_1ms(void) {
	static uint16_t count_1s = 1000;
	count_1s--;
	if (!count_1s) {
		led_toggle(0);
		count_1s = 1000;
	}
}

void ledPulso(uint8_t led, uint32_t tiempo) {
	led_on(led);
	Delay(tiempo);
	led_off(led);
}
