#include<jy62.h>

struct vec acc,angle,vel;
uint32_t height;

uint8_t jy62_buff[JY62_BUFFSIZE];

UART_HandleTypeDef *jy62_uart;

uint8_t initAngle_pack[3] = { 0xFF, 0xAA, 0x52 };
uint8_t calibrateAcce_pack[3] = { 0xFF, 0xAA, 0x67 };
uint8_t setBaud115200_pack[3] = { 0xFF, 0xAA, 0x63 };
uint8_t setBaud9600_pack[3] = { 0xFF, 0xAA, 0x64 };
uint8_t setHorizontal_pack[3] = { 0xFF, 0xAA, 0x65 };
uint8_t setVertical_pack[3] = { 0xFF, 0xAA, 0x66 };
uint8_t sleepAndAwake_pack[3] = { 0xFF, 0xAA, 0x60 };

float g=9.8f;
void jy62_init(UART_HandleTypeDef *huart){
	jy62_uart=huart;
}

void calibrate(){
	HAL_UART_Transmit(jy62_uart, calibrateAcce_pack, 3, HAL_MAX_DELAY);
}

void setVertical(){
	HAL_UART_Transmit(jy62_uart, setVertical_pack, 3, HAL_MAX_DELAY);
}

void setHorizontal() {
	HAL_UART_Transmit(jy62_uart, setHorizontal_pack, 3, HAL_MAX_DELAY);
}

void initAngle() {
	HAL_UART_Transmit(jy62_uart, initAngle_pack, 3, HAL_MAX_DELAY);
}
