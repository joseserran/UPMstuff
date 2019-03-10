/**
  ******************************************************************************
  * @file    main.c
  * @author  Ac6
  * @version V1.0
  * @date    01-December-2013
  * @brief   Default main function.
  ******************************************************************************
*/


#include "stm32f4xx.h"
#include "stm32f411e_discovery.h"
#include "stm32f4xx_hal.h"
#include "stm32f411e_discovery_accelerometer.h"
//#include "lsm303dlhc.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include <string.h>

#define MAX_PERIOD 8

enum {
  M_UART_POLL = 0,
  M_UART_IT,
  M_UART_DMA,
  M_UART_MODES
};

typedef struct {
  uint32_t timestamp;
  int16_t val;
  uint8_t type;
  uint8_t subtype;
  float sensitivity;
} msg_t;

QueueHandle_t txQueue;
QueueHandle_t rxQueue;

SemaphoreHandle_t txSemaphoreDone;
SemaphoreHandle_t buttonSemaphore;
SemaphoreHandle_t i2cSemaphoreDone;
SemaphoreHandle_t i2cMutex;

UART_HandleTypeDef UartHandle;
DMA_HandleTypeDef dma1Stream5Handle;
DMA_HandleTypeDef dma1Stream6Handle;
DMA_HandleTypeDef dma1Stream0Handle;
DMA_HandleTypeDef dma1Stream7Handle;

I2C_HandleTypeDef hi2c1;

volatile int uart_mode = 0;
volatile int acc_period = 0;
volatile int gyro_period = 0;
volatile int temp_period = 7;
volatile int button_period = 0;
volatile uint8_t i2cDone = 0;


/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  while(1)
  {
  }
}

/****************************
 * 
 * UART TX task and auxiliary functions and callbacks
 * 
 ****************************/
void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
	GPIO_InitTypeDef gpio;

	__HAL_RCC_GPIOD_CLK_ENABLE();
	gpio.Mode = GPIO_MODE_AF_PP;
	gpio.Alternate = GPIO_AF7_USART2;
	gpio.Pin = GPIO_PIN_5 | GPIO_PIN_6;
	gpio.Pull = GPIO_NOPULL;
	gpio.Speed = GPIO_SPEED_MEDIUM;

	HAL_GPIO_Init(GPIOD, &gpio);

	  __HAL_RCC_DMA1_CLK_ENABLE();
	  //UART Configuration
	  dma1Stream6Handle.Instance = DMA1_Stream6;
	  dma1Stream6Handle.Init.Channel = DMA_CHANNEL_4;
	  dma1Stream6Handle.Init.Direction = DMA_MEMORY_TO_PERIPH;
	  dma1Stream6Handle.Init.Mode = DMA_NORMAL;
	  dma1Stream6Handle.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
	  dma1Stream6Handle.Init.MemBurst = DMA_MBURST_SINGLE;
	  dma1Stream6Handle.Init.MemInc = DMA_MINC_ENABLE;
	  dma1Stream6Handle.Init.PeriphBurst = DMA_PBURST_SINGLE;
	  dma1Stream6Handle.Init.PeriphInc = DMA_PINC_DISABLE;
	  dma1Stream6Handle.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
	  dma1Stream6Handle.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
	  dma1Stream6Handle.Init.Priority = DMA_PRIORITY_MEDIUM;

	  HAL_DMA_Init(&dma1Stream6Handle);

	  dma1Stream5Handle.Instance = DMA1_Stream5;
	  dma1Stream5Handle.Init.Channel = DMA_CHANNEL_4;
	  dma1Stream5Handle.Init.Direction = DMA_PERIPH_TO_MEMORY;
	  dma1Stream5Handle.Init.Mode = DMA_NORMAL;
	  dma1Stream5Handle.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
	  dma1Stream5Handle.Init.MemBurst = DMA_MBURST_SINGLE;
	  dma1Stream5Handle.Init.MemInc = DMA_MINC_ENABLE;
	  dma1Stream5Handle.Init.PeriphBurst = DMA_PBURST_SINGLE;
	  dma1Stream5Handle.Init.PeriphInc = DMA_PINC_DISABLE;
	  dma1Stream5Handle.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
	  dma1Stream5Handle.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
	  dma1Stream5Handle.Init.Priority = DMA_PRIORITY_MEDIUM;

	  HAL_DMA_Init(&dma1Stream5Handle);

	  __HAL_LINKDMA(huart, hdmarx, dma1Stream5Handle);
	  __HAL_LINKDMA(huart, hdmatx, dma1Stream6Handle);

	  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, configLIBRARY_LOWEST_INTERRUPT_PRIORITY, 0);
	  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

	  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, configLIBRARY_LOWEST_INTERRUPT_PRIORITY, 0);
	  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

}

void uart_config()
{
  __HAL_RCC_USART2_CLK_ENABLE();


  UartHandle.Instance          = USART2;

  UartHandle.Init.BaudRate     = 115200;
  UartHandle.Init.WordLength   = UART_WORDLENGTH_8B;
  UartHandle.Init.StopBits     = UART_STOPBITS_1;
  UartHandle.Init.Parity       = UART_PARITY_NONE;
  UartHandle.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
  UartHandle.Init.Mode         = UART_MODE_TX_RX;
  UartHandle.Init.OverSampling = UART_OVERSAMPLING_16;

  //GPIO configured in MSP Callback
  if(HAL_UART_Init(&UartHandle) != HAL_OK)
  { 
    _Error_Handler(__FILE__, __LINE__);
  }
  HAL_NVIC_EnableIRQ(USART2_IRQn);
  HAL_NVIC_SetPriority(USART2_IRQn, configLIBRARY_LOWEST_INTERRUPT_PRIORITY, 0); // min priority
}

void uart_tx(void* param)
{
  //Ping pong buffer
  static char buf[2][256];
  static uint8_t buf_index = 0;
  msg_t msg;
  uint8_t len;

  uart_config();

  while(1) {
    // Get element from the queue and send it!
    xQueueReceive(txQueue, &msg, portMAX_DELAY);
    if (msg.sensitivity) {
      sprintf(buf[buf_index], "%c%c, %d, %d\r\n", msg.type, msg.subtype, msg.timestamp, msg.val);
    } else {
      sprintf(buf[buf_index], "%c%c, %d, %d\r\n", msg.type, msg.subtype, msg.timestamp, msg.val);
    }
    len = strlen(buf[buf_index]);
    switch(uart_mode) {
      case M_UART_POLL:
        if(HAL_UART_Transmit(&UartHandle, (uint8_t *)buf[buf_index], len, 500)!= HAL_OK) {
        //if(HAL_UART_Transmit(&UartHandle, msg.ptr, msg.len, 500)!= HAL_OK) {
          _Error_Handler(__FILE__, __LINE__);
          break;
        }
        break;
      case M_UART_IT:
        if (xSemaphoreTake(txSemaphoreDone, portMAX_DELAY) != pdPASS) {
          _Error_Handler(__FILE__, __LINE__);
          break;
        }
        if(HAL_UART_Transmit_IT(&UartHandle, (uint8_t*)buf[buf_index], len)!= HAL_OK) {
        //if(HAL_UART_Transmit_IT(&UartHandle, msg.ptr, msg.len)!= HAL_OK) {
          _Error_Handler(__FILE__, __LINE__);
          break;
        }
        break;
      case M_UART_DMA:
        if (xSemaphoreTake(txSemaphoreDone, portMAX_DELAY) != pdPASS) {
          _Error_Handler(__FILE__, __LINE__);
          break;
        }
        if(HAL_UART_Transmit_DMA(&UartHandle, (uint8_t*)buf[buf_index], len)!= HAL_OK) {
        //if(HAL_UART_Transmit_DMA(&UartHandle, msg.ptr, msg.len)!= HAL_OK) {
          _Error_Handler(__FILE__, __LINE__);
          break;
        }
        break;
    }
    buf_index = !buf_index;
  }
}

/**
  * @brief  Tx Transfer completed callback
  * @param  UartHandle: UART handle.
  * @retval None
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle)
{
  /* Set transmission flag: transfer complete*/
  //Generate Token
  xSemaphoreGiveFromISR(txSemaphoreDone, NULL);
}

/****************************
 * 
 * UART RX task and auxiliary functions and callbacks
 * 
 ****************************/


/****************************
 * 
 * BUTTON task and auxiliary functions and callbacks
 * 
 ****************************/
void configButton()
{
  GPIO_InitTypeDef gpio;
  __HAL_RCC_GPIOA_CLK_ENABLE();

  gpio.Mode = GPIO_MODE_IT_RISING;
  gpio.Pin = GPIO_PIN_0;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_MEDIUM;

  HAL_GPIO_Init(GPIOA, &gpio);

  HAL_NVIC_EnableIRQ(EXTI0_IRQn);
  HAL_NVIC_SetPriority(EXTI0_IRQn, configLIBRARY_LOWEST_INTERRUPT_PRIORITY, 0); // min priority
}

void button(void* param)
{
  static int period[MAX_PERIOD] = {1000/portTICK_RATE_MS, 800/portTICK_RATE_MS, 700/portTICK_RATE_MS, 600/portTICK_RATE_MS, 500/portTICK_RATE_MS, 400/portTICK_RATE_MS, 300/portTICK_RATE_MS, 200/portTICK_RATE_MS};

  msg_t button_msg;
  configButton();
  button_msg.type = 'B';
  button_msg.subtype = '0';
  button_msg.val = 1;
  button_msg.sensitivity = 0;

  while (1) {
    if (xSemaphoreTake(buttonSemaphore, portMAX_DELAY) != pdPASS) {
      _Error_Handler(__FILE__, __LINE__);
      break;
    }

    button_msg.timestamp = xTaskGetTickCount();
    button_msg.val++;

    if (!(button_msg.val % 5))
    {
    	uart_mode = (uart_mode + 1) % M_UART_MODES;
    }

    HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_15);

    xQueueSendToBack(txQueue, &button_msg, 0);
    vTaskDelay(200/portTICK_RATE_MS);
    // Clear EXTI interrupt, just in case there was another pending interrupt.
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_0);
    HAL_NVIC_EnableIRQ(EXTI0_IRQn);
  }
}

/****************************
 * 
 * ACCELEROMETER task and auxiliary functions and callbacks
 * 
 ****************************/
void i2cInit() {

	__HAL_RCC_I2C1_CLK_ENABLE();

	hi2c1.Instance = I2C1;
	hi2c1.Init.OwnAddress1 =  0x43;
	hi2c1.Init.ClockSpeed = 100000;
	hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLED;
	hi2c1.Init.OwnAddress2 = 0x00;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLED;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLED;
	HAL_I2C_Init(&hi2c1);

	__HAL_RCC_DMA1_CLK_ENABLE();

	dma1Stream0Handle.Instance = DMA1_Stream0;
	dma1Stream0Handle.Init.Channel = DMA_CHANNEL_1;
	dma1Stream0Handle.Init.Direction = DMA_PERIPH_TO_MEMORY;
	dma1Stream0Handle.Init.Mode = DMA_NORMAL;
	dma1Stream0Handle.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
	dma1Stream0Handle.Init.MemBurst = DMA_MBURST_SINGLE;
	dma1Stream0Handle.Init.MemInc = DMA_MINC_ENABLE;
	dma1Stream0Handle.Init.PeriphBurst = DMA_PBURST_SINGLE;
	dma1Stream0Handle.Init.PeriphInc = DMA_PINC_DISABLE;
	dma1Stream0Handle.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
	dma1Stream0Handle.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
	dma1Stream0Handle.Init.Priority = DMA_PRIORITY_MEDIUM;

	HAL_DMA_Init(&dma1Stream0Handle);

	dma1Stream7Handle.Instance = DMA1_Stream7;
	dma1Stream7Handle.Init.Channel = DMA_CHANNEL_1;
	dma1Stream7Handle.Init.Direction = DMA_MEMORY_TO_PERIPH;
	dma1Stream7Handle.Init.Mode = DMA_NORMAL;
	dma1Stream7Handle.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
	dma1Stream7Handle.Init.MemBurst = DMA_MBURST_SINGLE;
	dma1Stream7Handle.Init.MemInc = DMA_MINC_ENABLE;
	dma1Stream7Handle.Init.PeriphBurst = DMA_PBURST_SINGLE;
	dma1Stream7Handle.Init.PeriphInc = DMA_PINC_DISABLE;
	dma1Stream7Handle.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
	dma1Stream7Handle.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
	dma1Stream7Handle.Init.Priority = DMA_PRIORITY_MEDIUM;

	HAL_DMA_Init(&dma1Stream7Handle);

	__HAL_LINKDMA(&hi2c1, hdmarx, dma1Stream0Handle);
	__HAL_LINKDMA(&hi2c1, hdmatx, dma1Stream7Handle);

	HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, configLIBRARY_LOWEST_INTERRUPT_PRIORITY, 0);
	HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);

	HAL_NVIC_SetPriority(DMA1_Stream7_IRQn, configLIBRARY_LOWEST_INTERRUPT_PRIORITY, 0);
	HAL_NVIC_EnableIRQ(DMA1_Stream7_IRQn);

	// Init SDA SCL
	__HAL_RCC_GPIOB_CLK_ENABLE();

	GPIO_InitTypeDef gpio;
	gpio.Pin = GPIO_PIN_9 | GPIO_PIN_6;
	gpio.Mode = GPIO_MODE_AF_OD;
	gpio.Pull = GPIO_NOPULL;
	gpio.Speed = GPIO_SPEED_FAST;
	gpio.Alternate = GPIO_AF4_I2C1;

	HAL_GPIO_Init(GPIOB, &gpio);
}

void initAccelerometer()
{
	ACCELERO_InitTypeDef         LSM303DLHC_InitStructure;

    LSM303DLHC_InitStructure.Power_Mode = LSM303DLHC_NORMAL_MODE;
    LSM303DLHC_InitStructure.AccOutput_DataRate = LSM303DLHC_ODR_50_HZ;
    LSM303DLHC_InitStructure.Axes_Enable = LSM303DLHC_AXES_ENABLE;
    LSM303DLHC_InitStructure.AccFull_Scale = LSM303DLHC_FULLSCALE_2G;
    LSM303DLHC_InitStructure.BlockData_Update = LSM303DLHC_BlockUpdate_Continous;
    LSM303DLHC_InitStructure.Endianness = LSM303DLHC_BLE_LSB;
    LSM303DLHC_InitStructure.High_Resolution = LSM303DLHC_HR_ENABLE;

    /* Configure MEMS: data rate, power mode, full scale and axes */
    uint8_t ctrl = (LSM303DLHC_InitStructure.Power_Mode | LSM303DLHC_InitStructure.AccOutput_DataRate | \
                       LSM303DLHC_InitStructure.Axes_Enable);

    uint8_t ctrl4 = (LSM303DLHC_InitStructure.BlockData_Update | LSM303DLHC_InitStructure.Endianness | \
                      LSM303DLHC_InitStructure.AccFull_Scale | LSM303DLHC_InitStructure.High_Resolution);


	HAL_I2C_Mem_Write(&hi2c1, ACC_I2C_ADDRESS, (uint16_t)LSM303DLHC_CTRL_REG1_A, I2C_MEMADD_SIZE_8BIT, &ctrl, 1, 500);
	HAL_I2C_Mem_Write(&hi2c1, ACC_I2C_ADDRESS, (uint16_t)LSM303DLHC_CTRL_REG4_A, I2C_MEMADD_SIZE_8BIT, &ctrl4, 1, 500);
}

void readAcc(int16_t *xyz)
{
    xSemaphoreTake(i2cSemaphoreDone, portMAX_DELAY);
    i2cDone = 0;
	HAL_I2C_Mem_Read_DMA(&hi2c1, ACC_I2C_ADDRESS, LSM303DLHC_OUT_X_L_A | 0x80, I2C_MEMADD_SIZE_8BIT, xyz, 6);
	while (!i2cDone);
}

void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    xSemaphoreGiveFromISR(i2cSemaphoreDone, NULL);
    i2cDone = 1;
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    xSemaphoreGiveFromISR(i2cSemaphoreDone, NULL);
    i2cDone = 1;
}

void accelerometer(void* param) {
  msg_t msg;
  static int period[MAX_PERIOD] = {1000/portTICK_RATE_MS, 500/portTICK_RATE_MS, 250/portTICK_RATE_MS, 100/portTICK_RATE_MS, 40/portTICK_RATE_MS, 20/portTICK_RATE_MS, 10/portTICK_RATE_MS, 5/portTICK_RATE_MS};
  int16_t xyz[3];

  initAccelerometer();

  TickType_t prevWakeTime = xTaskGetTickCount();

  msg.type = 'A';
  msg.sensitivity = 1;

  while (1) {
    xSemaphoreTake(i2cMutex, portMAX_DELAY);
    msg.timestamp = xTaskGetTickCount();
    readAcc(xyz);
    xSemaphoreGive(i2cMutex);
    msg.subtype = 'X';
    msg.val = xyz[0];
    xQueueSendToBack(txQueue, &msg, 0);
    msg.subtype = 'Y';
    msg.val = xyz[1];
    xQueueSendToBack(txQueue, &msg, 0);
    msg.subtype = 'Z';
    msg.val = xyz[2];
    xQueueSendToBack(txQueue, &msg, 0);

    vTaskDelayUntil(&prevWakeTime, period[acc_period]);
  }
}

/****************************
 * 
 * GYROSCOPE task and auxiliary functions and callbacks
 * 
 ****************************/
void gyroscope_config() {
  //BSP_ACCELERO_Init();
}

void gyroscope(void* param) {
  msg_t msg;
  static int period[MAX_PERIOD] = {1000/portTICK_RATE_MS, 500/portTICK_RATE_MS, 250/portTICK_RATE_MS, 100/portTICK_RATE_MS, 40/portTICK_RATE_MS, 20/portTICK_RATE_MS, 10/portTICK_RATE_MS, 5/portTICK_RATE_MS};
  int16_t xyz[3];

  TickType_t prevWakeTime = xTaskGetTickCount();

  msg.type = 'G';

  while (1) {
    //xSemaphoreTake(i2cMutex, portMAX_DELAY); SPI not I2C
    msg.timestamp = xTaskGetTickCount();
    //Read XYZ. Check l3gd20.c, which is the device used in stm32f411_discovery_giroscope.c
    //xSemaphoreGive(i2cMutex);
    msg.subtype = 'X';
    msg.val = xyz[0];
    xQueueSendToBack(txQueue, &msg, 0);
    msg.subtype = 'Y';
    msg.val = xyz[1];
    xQueueSendToBack(txQueue, &msg, 0);
    msg.subtype = 'Z';
    msg.val = xyz[2];
    xQueueSendToBack(txQueue, &msg, 0);

    vTaskDelayUntil(&prevWakeTime, period[gyro_period]);
  }
}

/****************************
 * 
 * COMPASS task and auxiliary functions and callbacks
 * 
 ****************************/
void compass_config() {
    /* Configure MEMS: data rate, power mode, full scale and axes */
    uint8_t ctrlA = 0x94;
	HAL_I2C_Mem_Write(&hi2c1, MAG_I2C_ADDRESS, (uint16_t)LSM303DLHC_CRA_REG_M, I2C_MEMADD_SIZE_8BIT, &ctrlA, 1, 500);

	uint8_t ctrlB = 0xA0;
	HAL_I2C_Mem_Write(&hi2c1, MAG_I2C_ADDRESS, (uint16_t)LSM303DLHC_CRB_REG_M, I2C_MEMADD_SIZE_8BIT, &ctrlB, 1, 500);

	uint8_t mr = 0x00;
	HAL_I2C_Mem_Write(&hi2c1, MAG_I2C_ADDRESS, (uint16_t)LSM303DLHC_MR_REG_M, I2C_MEMADD_SIZE_8BIT, &mr, 1, 500);
}

void readCompass(int16_t *xyz)
{
	uint8_t buf[6];
	HAL_I2C_Mem_Read(&hi2c1, MAG_I2C_ADDRESS, LSM303DLHC_OUT_X_H_M | 0x80, I2C_MEMADD_SIZE_8BIT, buf, 6, 500);

	for (uint32_t i = 0; i < 3; i++)
	{
		xyz[i] = buf[2*i] << 8 | buf[2*i+1];
	}
}

void compass(void* param) {
  msg_t msg;
  static int period[MAX_PERIOD] = {5000/portTICK_RATE_MS, 2500/portTICK_RATE_MS, 1000/portTICK_RATE_MS, 500/portTICK_RATE_MS, 200/portTICK_RATE_MS, 100/portTICK_RATE_MS, 50/portTICK_RATE_MS, 20/portTICK_RATE_MS};
  int16_t xyz[3];

  TickType_t prevWakeTime = xTaskGetTickCount();

  msg.type = 'C';

  while (1) {
    xSemaphoreTake(i2cMutex, portMAX_DELAY);
    msg.timestamp = xTaskGetTickCount();
    readCompass(xyz);
    xSemaphoreGive(i2cMutex);
    msg.subtype = 'X';
    msg.val = xyz[0];
    xQueueSendToBack(txQueue, &msg, 0);
    msg.subtype = 'Y';
    msg.val = xyz[1];
    xQueueSendToBack(txQueue, &msg, 0);
    msg.subtype = 'Z';
    msg.val = xyz[2];
    xQueueSendToBack(txQueue, &msg, 0);

    vTaskDelayUntil(&prevWakeTime, 1000/portTICK_RATE_MS);
  }
}

/****************************
 * 
 * TEMPERATURE task and auxiliary functions and callbacks
 * 
 ****************************/
int16_t getTemp() {
	uint8_t tempBytes[2];
	HAL_I2C_Mem_Read(&hi2c1, MAG_I2C_ADDRESS, LSM303DLHC_TEMP_OUT_H_M | 0x80, I2C_MEMADD_SIZE_8BIT, tempBytes, 2, 500);

	return tempBytes[0] << 8 | tempBytes[1];
}

void temperature(void* param) {
  msg_t msg;
  static int period[MAX_PERIOD] = {600000/portTICK_RATE_MS, 300000/portTICK_RATE_MS, 120000/portTICK_RATE_MS, 60000/portTICK_RATE_MS, 30000/portTICK_RATE_MS, 10000/portTICK_RATE_MS, 5000/portTICK_RATE_MS, 1000/portTICK_RATE_MS};
  int16_t temp;

  TickType_t prevWakeTime = xTaskGetTickCount();

  msg.type = 'T';
  msg.subtype = '0';

  while (1) {
    xSemaphoreTake(i2cMutex, portMAX_DELAY);
    msg.timestamp = xTaskGetTickCount();
    temp = getTemp();
    xSemaphoreGive(i2cMutex);
    msg.val = temp;
    xQueueSendToBack(txQueue, &msg, 0);

    vTaskDelayUntil(&prevWakeTime, /*period[temp_period]*/1000/portTICK_RATE_MS);
  }
}



void SystemClock_Config(void)
{
	RCC_ClkInitTypeDef RCC_ClkInitStruct;
	RCC_OscInitTypeDef RCC_OscInitStruct;

	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLN = 192;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
	RCC_OscInitStruct.PLL.PLLM = 4;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLQ = 8;
	HAL_RCC_OscConfig(&RCC_OscInitStruct);

	RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_HCLK |
								   RCC_CLOCKTYPE_PCLK1 |
								   RCC_CLOCKTYPE_PCLK2 |
								   RCC_CLOCKTYPE_SYSCLK);
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;

	HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);
}

void config_leds() {
	__HAL_RCC_GPIOD_CLK_ENABLE();
	GPIO_InitTypeDef gpio;
	gpio.Mode = GPIO_MODE_OUTPUT_PP;
	gpio.Pin = GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
	gpio.Pull = GPIO_NOPULL;
	gpio.Speed = GPIO_SPEED_LOW;

	HAL_GPIO_Init(GPIOD, &gpio);
}

void config_queue() {
	txQueue = xQueueCreate(20, sizeof(msg_t));
}

int main(void)
{

    SystemClock_Config();
    HAL_Init();

    config_leds();
    config_queue();
	i2cInit();
	compass_config();

    if(! (buttonSemaphore = xSemaphoreCreateBinary()) ){
        _Error_Handler(__FILE__, __LINE__);
    }
    if (! (i2cMutex = xSemaphoreCreateMutex() ) )
    {
        _Error_Handler(__FILE__, __LINE__);
    }
    if(! (txSemaphoreDone = xSemaphoreCreateBinary() ) )
    {
    	_Error_Handler(__FILE__, __LINE__);
    }
    xSemaphoreGive(txSemaphoreDone);
    if(! (i2cSemaphoreDone = xSemaphoreCreateBinary() ) )
    {
    	_Error_Handler(__FILE__, __LINE__);
    }
    xSemaphoreGive(i2cSemaphoreDone);

    if (xTaskCreate(button, "button", 512, NULL, 2, NULL) != pdPASS) {
      _Error_Handler(__FILE__, __LINE__);
    }
    if (xTaskCreate(uart_tx, "uart_tx", 512, NULL, 3, NULL) != pdPASS) {
      _Error_Handler(__FILE__, __LINE__);
    }
    if (xTaskCreate(accelerometer, "accelerometer", 512, NULL, 2, NULL) != pdPASS) {
      _Error_Handler(__FILE__, __LINE__);
    }
    if (xTaskCreate(compass, "compass", 512, NULL, 2, NULL) != pdPASS) {
      _Error_Handler(__FILE__, __LINE__);
    }
    if (xTaskCreate(temperature, "temperature", 512, NULL, 2, NULL) != pdPASS) {
      _Error_Handler(__FILE__, __LINE__);
    }

	vTaskStartScheduler();

	return 0;
}

void HAL_GPIO_EXTI_Callback(uint16_t pin) {
	// Disable interrupts to prevent button bounces from giving another semaphore
	HAL_NVIC_DisableIRQ(EXTI0_IRQn);
	//Generate Token
	xSemaphoreGiveFromISR(buttonSemaphore, NULL);
}

