/**
  ******************************************************************************
  * @file    eddystone_tlm_service.c
  * @author  MCD Application Team
  * @version
  * @date    8/24/2015
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
#include <string.h>
#include <time.h>

#include "adc.h"
#include "rtc.h"
#include "ble_status.h"
#include "bluenrg_itf.h"
#include "eddystone_beacon.h"
#include "x-nucleo-idb04a1.h"

/* Private types -------------------------------------------------------------*/
enum
{
  IDLE,
  TLM_UPDATE_UPTIME,
  TLM_UPDATE_ADCOUNT,
  TLM_UPDATE_VOLTAGE,
  TLM_UPDATE_VOLTAGE_STARTED,
  TLM_UPDATE_TEMP,
  TLM_UPDATE_TEMP_STARTED,
  TLM_UPDATE_BROADCAST
};

/* Private variables ---------------------------------------------------------*/
static uint8_t update = IDLE;

static uint16_t vrefint;
static int16_t tsense;
static uint32_t uptime;
static uint32_t adcount;

/* Private constants ---------------------------------------------------------*/
#define ADC_BUF_LEN               10uL

#define VDD_CALIB       (3000uL)
#define VREFINT_CAL     (* (uint16_t *) 0x1FF80078)
#define TS_CAL1         (* (uint16_t *) 0x1FF8007A)
#define TS_CAL2         (* (uint16_t *) 0x1FF8007E)
#define TS_CAL1_TEMP    (30uL)
#define TS_CAL2_TEMP    (130uL)

#define VREF_ADC_RDYF_TIMEOUT     15
#define SENSOR_ADC_RDYF_TIMEOUT   15

/* Private macros ------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

void ComputeVoltage(void);
void ComputeTemperature(void);

/**
  * @brief  Calculate the voltage of the beacon.
  *         The value is expressed as 1 mV/bit.
  * @param  None
  * @retval None
  */
void EddystoneTLM_GetVoltage(void);
/**
  * @brief  Calculate the temperature of the processor using the ADC.
  *         The value is expressed in FP8.8 format.
  * @param  None
  * @retval None
  */
void EddystoneTLM_GetTemperature(void);

/**
  * @brief  Calculate the number of advertising frames emitted by the beacon.
  * @param  None
  * @retval None
  */
void EddystoneTLM_GetAdcount(void);

/**
  * @brief  Calculate the uptime of the beacon.
  *         The value is expressed in 0.1 second resolution.
  * @param  None
  * @retval None
  */
void EddystoneTLM_GetUptime(void);

/* Exported functions --------------------------------------------------------*/
uint16_t adc_buf[ADC_BUF_LEN];

void EddystoneTLM_InitService(void)
{
	RTC_DateTypeDef sDate;

  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 1;
  sDate.Year = 70;

	HAL_RTC_SetDate(&hrtc, &sDate, FORMAT_BIN);

  HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, ADVERTISING_INTERVAL_IN_MS / 1000 - 1, RTC_WAKEUPCLOCK_CK_SPRE_16BITS);
  update = TLM_UPDATE_UPTIME;
}

void EddystoneTLM_Process(void)
{
  static uint8_t state = NOT_CONNECTED;

  if (update == TLM_UPDATE_UPTIME)
  {
    EddystoneTLM_GetUptime();
  }

  if (update == TLM_UPDATE_ADCOUNT)
  {
    EddystoneTLM_GetAdcount();
  }

  if (update == TLM_UPDATE_VOLTAGE)
  {
    EddystoneTLM_GetVoltage();
  }

  if (update == TLM_UPDATE_TEMP)
  {
    EddystoneTLM_GetTemperature();
  }

  if (update == TLM_UPDATE_BROADCAST)
  {
    update = IDLE;

    if (state == NOT_CONNECTED)
    {
      state = CONNECTED;

      EddystoneTLM_InitTypeDef EddystoneTLM_InitStruct =
      {
        .AdvertisingInterval = ADVERTISING_INTERVAL_IN_MS,
        .TLM_Version = 0x00,
        .BatteryVoltage = vrefint,
        .BeaconTemperature = tsense,
        .Uptime = uptime,
        .AdvertisingCount = adcount
      };

      EddystoneTLM_Init(&EddystoneTLM_InitStruct);
    }
    else
    {
      EddystoneTLM_Update(vrefint, tsense, adcount, uptime);
    }
  }

  if ((update == IDLE) && (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_SET))
  {
    EnterStopMode();
  }
}

void EddystoneTLM_VoltageCallback(void)
{
  ComputeVoltage();

  update = TLM_UPDATE_TEMP;
}

void EddystoneTLM_TemperatureCallback(void)
{
  ComputeTemperature();

  update = TLM_UPDATE_BROADCAST;
}

void EddystoneTLM_TimerCallback(void)
{
  update = TLM_UPDATE_UPTIME;
}

/* Private functions ---------------------------------------------------------*/
uint32_t ComputeSeconds(RTC_DateTypeDef * sDate, RTC_TimeTypeDef * sTime)
{
  uint32_t seconds = 0;

	struct tm cal =
	{
		.tm_sec = sTime->Seconds,
		.tm_min = sTime->Minutes,
		.tm_hour = sTime->Hours,
		.tm_mday = sDate->Date,
		.tm_mon = sDate->Month - 1,
		.tm_year = sDate->Year
	};

	seconds = mktime(&cal);

  return seconds;
}

void ComputeVoltage(void)
{
  uint32_t temp = 0;

  for (uint32_t i = 0; i < ADC_BUF_LEN; ++i)
  {
    temp += adc_buf[i];
  }

  vrefint = ADC_BUF_LEN * VDD_CALIB * VREFINT_CAL / temp;
}

void ComputeTemperature(void)
{
  uint32_t temp = 0;

  for (int i = 0; i < ADC_BUF_LEN; ++i)
  {
    temp += adc_buf[i];
  }

  temp /= ADC_BUF_LEN;

  static volatile float ftsense = 0.0f;

  ftsense = (((float) temp * vrefint / VDD_CALIB) - TS_CAL1) * (TS_CAL2_TEMP - TS_CAL1_TEMP) / (TS_CAL2 - TS_CAL1) + TS_CAL1_TEMP;

  tsense = (uint16_t) (ftsense * 256.0f);
}

void EddystoneTLM_GetTemperature(void)
{
    /**Disable the VREFINT channel and VREFINT
    */
  hadc.Instance->CHSELR &= ~(ADC_CHANNEL_VREFINT);
  ADC->CCR &= ~(ADC_CCR_VREFEN);

  HAL_ADC_EnableBuffer_Cmd(DISABLE);

    /**Deinit the ADC to reconfigure it for the temperature sensor
    */
  HAL_ADC_DeInit(&hadc);

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
    */
  hadc.Instance = ADC1;
  hadc.Init.OversamplingMode = DISABLE;
  hadc.Init.ClockPrescaler = ADC_CLOCKPRESCALER_PCLK_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION12b;
  hadc.Init.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  hadc.Init.ScanDirection = ADC_SCAN_DIRECTION_UPWARD;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ContinuousConvMode = ENABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIG_EDGE_NONE;
  hadc.Init.DMAContinuousRequests = ENABLE;
  hadc.Init.EOCSelection = EOC_SINGLE_CONV;
  hadc.Init.Overrun = OVR_DATA_PRESERVED;
  hadc.Init.LowPowerAutoWait = ENABLE;
  hadc.Init.LowPowerFrequencyMode = DISABLE;
  hadc.Init.LowPowerAutoOff = ENABLE;
  HAL_ADC_Init(&hadc);

  ADC_ChannelConfTypeDef sConfig;

    /**Configure for the selected ADC regular channel to be converted.
    */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  HAL_ADC_ConfigChannel(&hadc, &sConfig);

  HAL_ADC_EnableBufferSensor_Cmd(ENABLE);

  HAL_ADCEx_Calibration_Start(&hadc, ADC_SINGLE_ENDED);

  uint32_t tickstart = 0;

  /* Get timeout */
  tickstart = HAL_GetTick();

  while (__HAL_SYSCFG_GET_FLAG(SYSCFG_FLAG_SENSOR_ADC) == RESET)
  {
    /* Check for the Timeout */
    if((HAL_GetTick() - tickstart ) > SENSOR_ADC_RDYF_TIMEOUT)
    {
      EddystoneTLM_GetVoltage();

      return;
    }
  }

  HAL_ADC_Start_DMA(&hadc, (uint32_t *) adc_buf, ADC_BUF_LEN);

  update = TLM_UPDATE_TEMP_STARTED;
}

void EddystoneTLM_GetVoltage(void)
{
    /**Disable the VREFINT channel and VREFINT
    */
  hadc.Instance->CHSELR &= ~(ADC_CHANNEL_TEMPSENSOR);
  ADC->CCR &= ~(ADC_CCR_TSEN);

  HAL_ADC_EnableBufferSensor_Cmd(DISABLE);

  MX_ADC_Init();

  HAL_ADC_EnableBuffer_Cmd(ENABLE);

    /** Calibrate the ADC prior to use
    */
  HAL_ADCEx_Calibration_Start(&hadc, ADC_SINGLE_ENDED);

  uint32_t tickstart = 0;

  /* Get timeout */
  tickstart = HAL_GetTick();

  while (__HAL_SYSCFG_GET_FLAG(SYSCFG_FLAG_VREF_ADC) == RESET)
  {
    /* Check for the Timeout */
    if((HAL_GetTick() - tickstart ) > VREF_ADC_RDYF_TIMEOUT)
    {
      return;
    }
  }

  HAL_ADC_Start_DMA(&hadc, (uint32_t *) adc_buf, ADC_BUF_LEN);

  update = TLM_UPDATE_VOLTAGE_STARTED;
}

void EddystoneTLM_GetAdcount(void)
{
  adcount = uptime * 100 / ADVERTISING_INTERVAL_IN_MS;

  update = TLM_UPDATE_VOLTAGE;
}

void EddystoneTLM_GetUptime(void)
{
  RTC_TimeTypeDef sTime1;
  RTC_TimeTypeDef sTime2;

  memset(&sTime1, 0, sizeof(RTC_TimeTypeDef));
  memset(&sTime2, 0, sizeof(RTC_TimeTypeDef));

  HAL_RTC_GetTime(&hrtc, &sTime1, FORMAT_BIN);
  HAL_RTC_GetTime(&hrtc, &sTime2, FORMAT_BIN);

  if (0 == memcmp(&sTime1, &sTime2, sizeof(RTC_TimeTypeDef)))
  {
    RTC_DateTypeDef sDate;

    HAL_RTC_GetDate(&hrtc, &sDate, FORMAT_BIN);

    uptime = ComputeSeconds(&sDate, &sTime1) * 10 + 10 * sTime1.SubSeconds / 256;

    update = TLM_UPDATE_ADCOUNT;
  }
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
