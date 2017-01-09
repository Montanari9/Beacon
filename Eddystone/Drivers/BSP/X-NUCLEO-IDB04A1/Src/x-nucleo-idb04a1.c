/**
  ******************************************************************************
  * @file    x-nucleo-idb04a1.c
  * @author  MCD Application Team
  * @version
  * @date    8/17/2015
  * @brief
  *
  @verbatim
  ==============================================================================
                        ##### How to use this driver #####
  ==============================================================================
  @endverbatim
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2015 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

#include "stm32l0xx_hal.h"
#include "spi.h"

#include "hal_types.h"
#include "hci_const.h"
#include "osal.h"
#include "bluenrg_gap.h"
#include "bluenrg_gap_aci.h"
#include "bluenrg_gatt_aci.h"
#include "bluenrg_hal_aci.h"
#include "eddystone_beacon.h"
#include "string.h"
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
#define BLUENRG_SPI_TIMEOUT   (100uL)
#define HEADER_SIZE 5
#define MAX_BUFFER_SIZE 255
#define TIMEOUT_DURATION 15

/* Private macros ------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
int32_t BlueNRG_SPI_Write(SPI_HandleTypeDef *hspi, uint8_t* data1,
                          uint8_t* data2, uint8_t Nb_bytes1, uint8_t Nb_bytes2);
int32_t BlueNRG_SPI_Read_All(SPI_HandleTypeDef *hspi, uint8_t *buffer, uint8_t buff_size);
void Enable_SPI_IRQ(void);
void Disable_SPI_IRQ(void);
void Clear_SPI_IRQ(void);
void Clear_SPI_EXTI_Flag(void);
void set_irq_as_output(void);
void set_irq_as_input(void);
void us150Delay(void);

/* Exported functions --------------------------------------------------------*/

/**
 * @brief  Writes data to a serial interface.
 * @param  data1   :  1st buffer
 * @param  data2   :  2nd buffer
 * @param  n_bytes1: number of bytes in 1st buffer
 * @param  n_bytes2: number of bytes in 2nd buffer
 * @retval None
 */
void Hal_Write_Serial(const void* data1, const void* data2, int32_t n_bytes1,
                      int32_t n_bytes2)
{
  uint32_t tickstart = 0;

  /* Get timeout */
  tickstart = HAL_GetTick();

  while (1)
  {
    if(BlueNRG_SPI_Write(&hspi1, (uint8_t *)data1,(uint8_t *)data2, n_bytes1, n_bytes2) == 0)
    {
      break;
    }

    /* Check for the Timeout */
    if(BLUENRG_SPI_TIMEOUT != HAL_MAX_DELAY)
    {
      if((HAL_GetTick() - tickstart ) > BLUENRG_SPI_TIMEOUT)
      {
        return;
      }
    }
  }
}

/**
 * @brief  Reads data from a serial interface.
 * @param  data   :  buffer
 * @param  n_bytes: number of bytes in buffer
 * @retval None
 */
int32_t Hal_Read_Serial(const void* data, int32_t n_bytes)
{
   return BlueNRG_SPI_Read_All(&hspi1, (uint8_t *)data, n_bytes);
}

/**
 * @brief  Resets the BlueNRG.
 * @param  None
 * @retval None
 */
void BlueNRG_RST(void)
{
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);

  HAL_Delay(5);

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);

  HAL_Delay(5);
}

void BlueNRG_Init(void)
{
  
  tBleStatus ret = BLE_STATUS_SUCCESS;
  const char *name = "BlueNRG";
  uint16_t service_handle, dev_name_char_handle, appearance_char_handle;
  uint8_t SERVER_BDADDR[] = { MAC_ADDRESS };

  ret = aci_hal_write_config_data(CONFIG_DATA_PUBADDR_OFFSET,
                                  CONFIG_DATA_PUBADDR_LEN,
                                  SERVER_BDADDR);

  if (ret != BLE_STATUS_SUCCESS)
  {
    __asm("nop");
  }

  ret = aci_gatt_init();

  if (ret != BLE_STATUS_SUCCESS)
  {
    __asm("nop");
  }

  ret = aci_gap_init(GAP_PERIPHERAL_ROLE, &service_handle, &dev_name_char_handle, &appearance_char_handle);

  if (ret != BLE_STATUS_SUCCESS)
  {
    __asm("nop");
  }
  
  aci_gatt_update_char_value(service_handle, dev_name_char_handle, 0, strlen(name), (uint8_t *)name);

  ret = aci_hal_set_tx_power_level(1,4);

  if (ret != BLE_STATUS_SUCCESS)
  {
    __asm("nop");
  }

}

/**
 * @brief  Callback processing the ACI events.
 * @note   Inside this function each event must be identified and correctly
 *         parsed.
 * @param  void* Pointer to the ACI packet
 * @retval None
 */
void HCI_Event_CB(void *pckt)
{
  hci_uart_pckt *hci_pckt = pckt;

  /* obtain event packet */
  hci_event_pckt *event_pckt = (hci_event_pckt*)hci_pckt->data;

  if(hci_pckt->type != HCI_EVENT_PKT)
  {
    return;
  }

  switch(event_pckt->evt)
  {

  case EVT_DISCONN_COMPLETE:
    {
      ;
    }
    break;

  case EVT_LE_META_EVENT:
    {
      evt_le_meta_event *evt = (void *)event_pckt->data;

      switch(evt->subevent)
      {
      case EVT_LE_CONN_COMPLETE:
        {
          ;
        }
        break;
      }
    }
    break;

  case EVT_VENDOR:
    {
      ;
    }
    break;
  }
}

void EnterStopMode(void)
{
  __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
  HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
}

/**
 * @brief  Reports if the BlueNRG has data for the host micro.
 * @param  None
 * @retval 1 if data are present, 0 otherwise
 */
uint8_t BlueNRG_DataPresent(void)
{
  if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET)
  {
    return 1;
  }
  else
  {
    return 0;
  }
}

/**
 * @brief  Reads from BlueNRG SPI buffer and store data into local buffer.
 * @param  hspi     : SPI handle
 * @param  buffer   : Buffer where data from SPI are stored
 * @param  buff_size: Buffer size
 * @retval int32_t  : Number of read bytes
 */
int32_t BlueNRG_SPI_Read_All(SPI_HandleTypeDef *hspi, uint8_t *buffer, uint8_t buff_size)
{
  uint16_t byte_count;
  uint8_t len = 0;
  volatile uint8_t read_char;

  uint8_t header_master[HEADER_SIZE] = {0x0b, 0x00, 0x00, 0x00, 0x00};
  uint8_t header_slave[HEADER_SIZE];

  /* CS reset */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);

  /* Read the header */
  HAL_SPI_TransmitReceive_DMA(hspi, header_master, header_slave, HEADER_SIZE);

  while (hspi->State != HAL_SPI_STATE_READY)
  {
    ;
  }

  if (hspi->ErrorCode != HAL_SPI_ERROR_NONE)
  {
    /* Release CS line */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);

    // Add a small delay to give time to the BlueNRG to set the IRQ pin low
    // to avoid a useless SPI read at the end of the transaction
    for(volatile int i = 0; i < 2; i++)
    {
      __NOP();
    }

    return 0;
  }

  if (header_slave[0] == 0x02)
  {
    /* device is ready */
    byte_count = (header_slave[4]<<8)|header_slave[3];

    if (byte_count > 0) {

      /* avoid to read more data that size of the buffer */
      if (byte_count > buff_size){
        byte_count = buff_size;
      }

      for (len = 0; len < byte_count; len++)
      {
        buffer[len] = 0xff;
      }

      HAL_SPI_TransmitReceive_DMA(hspi, buffer, buffer, byte_count);

      while (hspi->State != HAL_SPI_STATE_READY)
      {
        ;
      }

      if (hspi->ErrorCode != HAL_SPI_ERROR_NONE)
      {
        /* Release CS line */
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);

        // Add a small delay to give time to the BlueNRG to set the IRQ pin low
        // to avoid a useless SPI read at the end of the transaction
        for(volatile int i = 0; i < 2; i++)
        {
          __NOP();
        }

        return 0;
      }
    }
  }

  /* Release CS line */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);

  // Add a small delay to give time to the BlueNRG to set the IRQ pin low
  // to avoid a useless SPI read at the end of the transaction
  for(volatile int i = 0; i < 2; i++)
  {
    __NOP();
  }

  return len;
}

/**
 * @brief  Writes data from local buffer to SPI.
 * @param  hspi     : SPI handle
 * @param  data1    : First data buffer to be written
 * @param  data2    : Second data buffer to be written
 * @param  Nb_bytes1: Size of first data buffer to be written
 * @param  Nb_bytes2: Size of second data buffer to be written
 * @retval Number of read bytes
 */
int32_t BlueNRG_SPI_Write(SPI_HandleTypeDef *hspi, uint8_t* data1, uint8_t* data2,
                          uint8_t Nb_bytes1, uint8_t Nb_bytes2)
{
  int32_t result = 0;

  unsigned char header_master[HEADER_SIZE] = {0x0a, 0x00, 0x00, 0x00, 0x00};
  unsigned char header_slave[HEADER_SIZE]  = {0xaa, 0x00, 0x00, 0x00, 0x00};

  unsigned char read_char_buf[MAX_BUFFER_SIZE];

  Disable_SPI_IRQ();

  /*
  The IRQ is set in Output mode, then it is pulled
  high and, after a delay of at least 112us, the CS line is asserted and the
  header transmit/receive operations are started.
  After these transmit/receive operations the IRQ is reset in input mode.
  */
  set_irq_as_output();

  /* Assert CS line after at least 112us */
  us150Delay();

  /* CS reset */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);

  /* Exchange header */
  HAL_SPI_TransmitReceive_DMA(hspi, header_master, header_slave, HEADER_SIZE);

  while (hspi->State != HAL_SPI_STATE_READY)
  {
    ;
  }

  if (hspi->ErrorCode != HAL_SPI_ERROR_NONE)
  {
    set_irq_as_input();

    /* Release CS line */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);

    Enable_SPI_IRQ();

    return -1;
  }

  set_irq_as_input();

  if (header_slave[0] == 0x02)
  {
    /* SPI is ready */
    if (header_slave[1] >= (Nb_bytes1+Nb_bytes2))
    {
      /*  Buffer is big enough */
      if (Nb_bytes1 > 0)
      {
        HAL_SPI_TransmitReceive_DMA(hspi, data1, read_char_buf, Nb_bytes1);

        while (hspi->State != HAL_SPI_STATE_READY)
        {
          ;
        }

        if (hspi->ErrorCode != HAL_SPI_ERROR_NONE)
        {
          /* Release CS line */
          HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);

          Enable_SPI_IRQ();

          return -1;
        }
      }
      if (Nb_bytes2 > 0)
      {
        HAL_SPI_TransmitReceive_DMA(hspi, data2, read_char_buf, Nb_bytes2);

        while (hspi->State != HAL_SPI_STATE_READY)
        {
          ;
        }

        if (hspi->ErrorCode != HAL_SPI_ERROR_NONE)
        {
          /* Release CS line */
          HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);

          Enable_SPI_IRQ();

          return -1;
        }
      }

    }
    else
    {
      /* Buffer is too small */
      result = -2;
    }
  }
  else
  {
    /* SPI is not ready */
    result = -1;
  }

  /* Release CS line */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);

  Enable_SPI_IRQ();

  return result;
}

/**
 * @brief  Clear EXTI (External Interrupt) line for SPI IRQ.
 * @param  None
 * @retval None
 */
void Clear_SPI_EXTI_Flag(void)
{
  __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_0);
}

/* Private functions ---------------------------------------------------------*/

/**
 * @brief  Set in Output mode the IRQ.
 * @param  None
 * @retval None
 */
void set_irq_as_output(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;

  /* Pull IRQ high */
  GPIO_InitStructure.Pin = GPIO_PIN_0;
  GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
}

/**
 * @brief  Set the IRQ in input mode.
 * @param  None
 * @retval None
 */
void set_irq_as_input(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;

  /* IRQ input */
  GPIO_InitStructure.Pin = GPIO_PIN_0;
  GPIO_InitStructure.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStructure.Pull = GPIO_PULLDOWN;
  GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);

  GPIO_InitStructure.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);
}

/**
 * @brief  Utility function for delay
 * @param  None
 * @retval None
 * NOTE: TODO: implement with clock-independent function.
 */
void us150Delay(void)
{
#if SYSCLK_FREQ == 4000000
  for(volatile int i = 0; i < 35; i++)__NOP();
#elif SYSCLK_FREQ == 32000000
  for(volatile int i = 0; i < 420; i++)__NOP();
#elif SYSCLK_FREQ == 84000000
  for(volatile int i = 0; i < 1125; i++)__NOP();
#else
  for(volatile int i = 0; i < 420; i++)__NOP();
#endif
}

/**
 * @brief  Enable SPI IRQ.
 * @param  None
 * @retval None
 */
void Enable_SPI_IRQ(void)
{
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);
}

/**
 * @brief  Disable SPI IRQ.
 * @param  None
 * @retval None
 */
void Disable_SPI_IRQ(void)
{
  HAL_NVIC_DisableIRQ(EXTI0_1_IRQn);
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
