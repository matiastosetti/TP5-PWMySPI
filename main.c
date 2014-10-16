#include <stdint.h>
#include "bsp/bsp.h"
#include <stdio.h>

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

//	int brillo = 0;
//	int flag = 0;
//	float acc_x;
//	float acc_y;
//	float acc_z;
	uint8_t CantLedsOn = 0; //Paso 7 empiezo con aplicaci�n.
	float PorcentajePote = 0;
	uint8_t memoriaCantLedsTemporal = 0;
	char txbuffer[20];

	while (1) {
		//bsp_delayMs(100);

		PorcentajePote = bsp_getPote();
		CantLedsOn = (uint8_t)(0.08 * PorcentajePote); //Se guarda un entero que indica la cant de leds a encenderse en base a una ecuaci�n de la recta. Ver apuntes luego de ADC
		bumetroSet(CantLedsOn);

		if (memoriaCantLedsTemporal != CantLedsOn) {

			uint8_t i;

			sprintf(txbuffer, "Valor Pote = %f\n", PorcentajePote); //funci�n de librer�a de manejo de string que carga en txbuffer, que es un arreglo de chars, los car�cteres entre doble comillas como chars individuales. el %f indica que adem�s se va a pasar una variable float. (tabi�n se puede decir %d si es decimal y asi)

			memoriaCantLedsTemporal = CantLedsOn;

			for (i = 0; txbuffer[i] != 0; i++) //mandamos cada caracter a la funci�n de envio de datos que usa la uart
					{
				sendData(txbuffer[i]);
			}
		}

//Uso del pwm para que con el paso del tiempo los leds brillen mas o menos
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
//

//	Uso Aceler�metro para prender led en base a la posici�n de la placa
//		acc_x = bsp_get_acc('x');
//		acc_y = bsp_get_acc('y');
//
//		if (acc_x > 0) {
//			led_setBright(1, acc_x * 100); //acc_x es un valor entre cero y uno. Como el duty cicle debe ser ingresado como % se lo multiplica por 100
//			led_setBright(3, 0);
//		} else {
//			led_setBright(1, 0);
//			led_setBright(3, acc_x * -100);
//		}
//		if (acc_y > 0) {
//			led_setBright(0, acc_y * 100);
//			led_setBright(2, 0);
//		} else {
//			led_setBright(0, 0);
//			led_setBright(2, acc_y * -100);
//		}
//		acc_z = bsp_get_acc('z');
//	}

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
}
