/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration of the USART2.
  ******************************************************************************
  */
/* USER CODE END Header */

#include "usart.h"

UART_HandleTypeDef huart2;

static void USART2_MspInit(void)
{
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_USART2_CLK_ENABLE();

  GPIOA->CRL &= ~((GPIO_CRL_MODE2 | GPIO_CRL_CNF2) | (GPIO_CRL_MODE3 | GPIO_CRL_CNF3));
  GPIOA->CRL |= (GPIO_CRL_MODE2_1 | GPIO_CRL_CNF2_1);
  GPIOA->CRL |= GPIO_CRL_CNF3_0;
}

void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;
  USART2_MspInit();

  huart2.Instance->CR1 = USART_CR1_TE | USART_CR1_RE;
  huart2.Instance->CR2 = 0;
  huart2.Instance->CR3 = 0;
  huart2.Instance->BRR = 0x1388; // 115200 @ PCLK1 = 36 MHz
  huart2.Instance->CR1 |= USART_CR1_UE;
}

HAL_StatusTypeDef UART_Transmit(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
  uint32_t tickstart = HAL_GetTick();
  for (uint16_t i = 0; i < Size; ++i)
  {
    while (!(huart->Instance->SR & USART_SR_TXE))
    {
      if ((Timeout != HAL_MAX_DELAY) && ((HAL_GetTick() - tickstart) > Timeout))
      {
        return HAL_TIMEOUT;
      }
    }
    huart->Instance->DR = pData[i];
  }
  while (!(huart->Instance->SR & USART_SR_TC))
  {
    if ((Timeout != HAL_MAX_DELAY) && ((HAL_GetTick() - tickstart) > Timeout))
    {
      return HAL_TIMEOUT;
    }
  }
  return HAL_OK;
}

HAL_StatusTypeDef UART_Receive(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
  uint32_t tickstart = HAL_GetTick();
  for (uint16_t i = 0; i < Size; ++i)
  {
    while (!(huart->Instance->SR & USART_SR_RXNE))
    {
      if ((Timeout != HAL_MAX_DELAY) && ((HAL_GetTick() - tickstart) > Timeout))
      {
        return HAL_TIMEOUT;
      }
    }
    pData[i] = (uint8_t)(huart->Instance->DR & 0xFF);
  }
  return HAL_OK;
}
