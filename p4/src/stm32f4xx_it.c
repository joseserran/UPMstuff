/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @author  Ac6
  * @version V1.0
  * @date    02-Feb-2015
  * @brief   Default Interrupt Service Routines.
  ******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#ifdef USE_RTOS_SYSTICK
#include <cmsis_os.h>
#endif
#include "stm32f4xx_it.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
extern USART_HandleTypeDef UartHandle;
extern DMA_HandleTypeDef dma1Stream5Handle;
extern DMA_HandleTypeDef dma1Stream6Handle;
extern DMA_HandleTypeDef dma1Stream0Handle;
extern DMA_HandleTypeDef dma1Stream7Handle;
extern I2C_HandleTypeDef I2cHandle;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            	  	    Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles SysTick Handler, but only if no RTOS defines it.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
	HAL_IncTick();
	HAL_SYSTICK_IRQHandler();
#ifdef USE_RTOS_SYSTICK
	osSystickHandler();
#endif
}

void EXTI0_IRQHandler() {
	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
}

void USART2_IRQHandler()
{
	HAL_UART_IRQHandler(&UartHandle);
}

void DMA1_Stream5_IRQHandler()
{
    HAL_DMA_IRQHandler(&dma1Stream5Handle);
}

void DMA1_Stream6_IRQHandler()
{
    HAL_DMA_IRQHandler(&dma1Stream6Handle);
}

void DMA1_Stream0_IRQHandler()
{
    HAL_DMA_IRQHandler(&dma1Stream0Handle);
}

void DMA1_Stream7_IRQHandler()
{
    HAL_DMA_IRQHandler(&dma1Stream7Handle);
}

void I2C1_EV_IRQHandler()
{
	HAL_I2C_EV_IRQHandler(&I2cHandle);
}

void I2C1_ER_IRQHandler()
{
	HAL_I2C_ER_IRQHandler(&I2cHandle);
}

void HardFault_Handler()
{
	while(1);
}
