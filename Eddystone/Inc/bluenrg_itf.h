/**
  ******************************************************************************
  * File Name          : bluenrg_itf.h
  * Description        : This file contains all the functions prototypes for
  *                      the BlueNRG profiles for Eddystone.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2015 STMicroelectronics
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BLUENRG_ITF_H
#define __BLUENRG_ITF_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
   typedef struct
   {
     uint16_t AdvertisingInterval;/*!< Specifies the desired advertising interval. */
     uint8_t CalibratedTxPower;   /*!< Specifies the power at 0m. */
     uint8_t * NamespaceID;       /*!< Specifies the 10-byte namespace to which the beacon belongs. */
     uint8_t * BeaconID;          /*!< Specifies the unique 6-byte beacon ID within the specified namespace. */
   } EddystoneUID_InitTypeDef;

   typedef struct
   {
     uint16_t AdvertisingInterval;/*!< Specifies the desired advertising interval. */
     uint8_t CalibratedTxPower;   /*!< Specifies the power at 0m. */
     uint8_t UrlScheme;           /*!< Specifies the URL Encoded Scheme Prefix. */
     uint8_t * Url;               /*!< Specifies the Encoded URL (max 17 characters). */
     uint8_t UrlLength;           /*!< Specifies the length of the Encoded URL (max 17 characters). */
   } EddystoneURL_InitTypeDef;

   typedef struct
   {
     uint16_t AdvertisingInterval;/*!< Specifies the desired advertising interval. */
     uint8_t  TLM_Version;        /*!< Specifies the version of the TLM frame. */
     uint16_t BatteryVoltage;     /*!< Specifies the battery voltage, in 1 mV/bit. */
     uint16_t BeaconTemperature;  /*!< Specifies the beacon temperature, in Signed 8.8 Fixed Point notation. */
     uint32_t AdvertisingCount;   /*!< Specifies the running count of all advertisement frames. */
     uint32_t Uptime;             /*!< Specifies the time sinbe the beacon was powered-up or rebooted. */
   } EddystoneTLM_InitTypeDef;

/* Exported constants --------------------------------------------------------*/
#define HTTP_WWW          (0x00u)
#define HTTPS_WWW         (0x01u)
#define HTTP              (0x02u)
#define HTTPS             (0x03u)

#define DOT_COM_SLASH     (0x00u)
#define DOT_ORG_SLASH     (0x01u)
#define DOT_EDU_SLASH     (0x02u)
#define DOT_NET_SLASH     (0x03u)
#define DOT_INFO_SLASH    (0x04u)
#define DOT_BIZ_SLASH     (0x05u)
#define DOT_GOV_SLASH     (0x06u)
#define DOT_COM           (0x07u)
#define DOT_ORG           (0x08u)
#define DOT_EDU           (0x09u)
#define DOT_NET           (0x0Au)
#define DOT_INFO          (0x0Bu)
#define DOT_BIZ           (0x0Cu)
#define DOT_GOV           (0x0Du)

/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
   tBleStatus EddystoneUID_Init(EddystoneUID_InitTypeDef *EddystoneUID_Init);
   tBleStatus EddystoneURL_Init(EddystoneURL_InitTypeDef *EddystoneURL_Init);
   tBleStatus EddystoneTLM_Init(EddystoneTLM_InitTypeDef *EddystoneTLM_Init);

   void EddystoneTLM_Update(uint16_t BatteryVoltage, int16_t BeaconTemperature, uint32_t AdvertisingCount, uint32_t Uptime);
#ifdef __cplusplus
}
#endif

#endif /* __BLUENRG_ITF_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
