/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.h
  * @brief   This file contains the USART2 initialization and basic UART I/O.
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __USART_H
#define __USART_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

typedef struct
{
  USART_TypeDef *Instance;
} UART_HandleTypeDef;

extern UART_HandleTypeDef huart2;

void MX_USART2_UART_Init(void);
HAL_StatusTypeDef UART_Transmit(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef UART_Receive(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size, uint32_t Timeout);

#define HAL_UART_Transmit UART_Transmit
#define HAL_UART_Receive UART_Receive

#ifdef __cplusplus
}
#endif

#endif /* __USART_H */
