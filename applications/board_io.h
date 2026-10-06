#ifndef BOARD_IO_H
#define BOARD_IO_H

#include "stm32f4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

extern SPI_HandleTypeDef hspi1;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;
extern CAN_HandleTypeDef hcan1;

void board_io_init(void);

#ifdef __cplusplus
}
#endif

#endif  // BOARD_IO_H
