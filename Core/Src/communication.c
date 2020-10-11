
#include <communication.h>
#include <string.h>
#include "checksum.h"
#include "main.h"

uint8_t tx_array[NORMAL_RESPONSE_LENGTH] = {0};
extern uint8_t b_state;

bool parse_normal_package(esc_settings *esc_struct,  uint8_t  *message)
{
	if  (IsChecksumm8bCorrect(message, NORMAL_REQUEST_LENGTH))  {

		struct Request req;

		memcpy((void*)&req,  (void*)message,  NORMAL_REQUEST_LENGTH);

        if  (req.address  ==  esc_struct->device_adress)  {
        	esc_struct->PWM_duty  =  req.velocity;
        	update_esc_settings(esc_struct);
            return true;
        }
	}
    return false;
}

void normal_response(UART_HandleTypeDef *huart, esc_settings *esc_struct)
{
	struct Response resp;

	resp.AA            = 0xAA;
	resp.type          = 1;
	resp.address       = esc_struct->device_adress;
	resp.state         = 0x55;
	resp.current       = 0x55;
	resp.speed_period  = 0x55;
	resp.button_state  = b_state;

	memcpy((void*)tx_array,  (void*)&resp,  NORMAL_RESPONSE_LENGTH - 1);

	AddChecksumm8b(tx_array,  NORMAL_RESPONSE_LENGTH);

    HAL_GPIO_WritePin(RS_485_DIR_GPIO_Port,  RS_485_DIR_Pin,  SET);

    HAL_UART_Transmit_IT(huart, tx_array, NORMAL_RESPONSE_LENGTH);

    //HAL_UART_Transmit_DMA(huart, tx_array, NORMAL_RESPONSE_LENGTH);

    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
}

bool parse_config_package(esc_settings *esc_struct,  uint8_t  *message)
{
	if  (IsChecksumm8bCorrect(message, CONFIG_REQUEST_LENGTH)) {

        struct ConfigRequest req;

        memcpy((void*)&req,  (void*)message,  CONFIG_REQUEST_LENGTH);

        if  (req.old_address  ==  esc_struct->device_adress)  {
        	esc_struct->device_adress  =  req.new_address;
        	return true;
        }
	}
	return false;
}

