/*
 * key.c
 *
 *  Created on: Dec 8, 2020
 *      Author: bruce
 */

#include "main.h"
#include "cmsis_os.h"

#include "key.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim4;

const int key_check_count = 20;

void KeyTask(void *argument)
{
	GPIO_PinState value;
	int key_pressed = 0;
	uint8_t event_count = 0;
	int event;

	for (;;) {
		value = HAL_GPIO_ReadPin(SWITCH_S2_GPIO_Port, SWITCH_S2_Pin);
		if (value == GPIO_PIN_SET)
			continue;
		key_pressed = 1;
		for (int i = 0; i < key_check_count; i++) {
			value = HAL_GPIO_ReadPin(SWITCH_S2_GPIO_Port, SWITCH_S2_Pin);
			if (value == GPIO_PIN_SET) {
				key_pressed = 0;
				break;
			}
			osDelay(1);
		}
		if (key_pressed == 0)
			continue;

		event_count++;
		event = event_count % 5;
		switch (event) {
		case 0: {
			HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1);
			HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_2);
			HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_3);
			HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_4);

			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 75); // peroid 1.5ms
			break;
		}
		case 1: {
			HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
			HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
			break;
		}
		case 2: {
			HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_2);
			HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_4);
			HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
			HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
			break;
		}
		case 3: {
			HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1);
			HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_3);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 25); // 0.5ms
			HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
			break;
		}
		case 4: {
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 125); // 2.5ms
			break;
		}
		}

		// check key event after 500ms
		osDelay(500);
	}
}
