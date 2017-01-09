/**
  ******************************************************************************
  * File Name          : bluenrg_itf.c
  * Description        : This file provides code for the configuration
  *                      of the BlueNRG profiles for Eddystone.
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

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

#include "hal_types.h"
#include "hci_const.h"
#include "bluenrg_gap.h"
#include "bluenrg_gap_aci.h"
#include "bluenrg_itf.h"

/* Private variables ---------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define ADVERTISING_INTERVAL_INCREMENT (16)
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

tBleStatus EddystoneUID_Init(EddystoneUID_InitTypeDef *EddystoneUID_Init)
{
  tBleStatus ret = BLE_STATUS_SUCCESS;

  /* Disable scan response. */
  hci_le_set_scan_resp_data(0, NULL);

  uint16_t AdvertisingInterval = (EddystoneUID_Init->AdvertisingInterval * ADVERTISING_INTERVAL_INCREMENT / 10);

  /* Put the device in a non-connectable mode. */
  ret = aci_gap_set_discoverable(ADV_NONCONN_IND,                          /*< Advertise as non-connectable, undirected. */
                                 AdvertisingInterval, AdvertisingInterval, /*< Set the advertising interval as 700 ms (0.625 us increment). */
                                 PUBLIC_ADDR, NO_WHITE_LIST_USE,           /*< Use the public address, with no white list. */
                                 0, NULL,                                  /*< Do not use a local name. */
                                 0, NULL,                                  /*< Do not include the service UUID list. */
                                 0, 0);                                    /*< Do not set a slave connection interval. */

  if (ret != BLE_STATUS_SUCCESS)
  {
    return ret;
  }

  /* Remove the TX power level advertisement (this is done to decrease the packet size). */
  ret = aci_gap_delete_ad_type(AD_TYPE_TX_POWER_LEVEL);

  if (ret != BLE_STATUS_SUCCESS)
  {
    return ret;
  }

  uint8_t service_data[] =
  {
    23,                                                                      /*< Length. */
    AD_TYPE_SERVICE_DATA,                                                    /*< Service Data data type value. */
    0xAA, 0xFE,                                                              /*< 16-bit Eddystone UUID. */
    0x00,                                                                    /*< UID frame type. */
    EddystoneUID_Init->CalibratedTxPower,                                    /*< Ranging data. */
    EddystoneUID_Init->NamespaceID[0],                                       /*< 10-byte ID Namespace. */
    EddystoneUID_Init->NamespaceID[1],
    EddystoneUID_Init->NamespaceID[2],
    EddystoneUID_Init->NamespaceID[3],
    EddystoneUID_Init->NamespaceID[4],
    EddystoneUID_Init->NamespaceID[5],
    EddystoneUID_Init->NamespaceID[6],
    EddystoneUID_Init->NamespaceID[7],
    EddystoneUID_Init->NamespaceID[8],
    EddystoneUID_Init->NamespaceID[9],
    EddystoneUID_Init->BeaconID[0],                                         /*< 6-byte ID Instance. */
    EddystoneUID_Init->BeaconID[1],
    EddystoneUID_Init->BeaconID[2],
    EddystoneUID_Init->BeaconID[3],
    EddystoneUID_Init->BeaconID[4],
    EddystoneUID_Init->BeaconID[5],
    0x00,                                                                   /*< Reserved. */
    0x00                                                                    /*< Reserved. */
  };

  uint8_t service_uuid_list[] =
  {
    3,                                                                      /*< Length. */
    AD_TYPE_16_BIT_SERV_UUID_CMPLT_LIST,                                    /*< Complete list of 16-bit Service UUIDs data type value. */
    0xAA, 0xFE                                                              /*< 16-bit Eddystone UUID. */
  };

  uint8_t flags[] =
  {
    2,                                                                      /*< Length. */
    AD_TYPE_FLAGS,                                                          /*< Flags data type value. */
    (FLAG_BIT_LE_GENERAL_DISCOVERABLE_MODE | FLAG_BIT_BR_EDR_NOT_SUPPORTED) /*< BLE general discoverable, without BR/EDR support. */
  };

  /* Update the service data. */
  ret = aci_gap_update_adv_data(sizeof(service_data), service_data);

  if (ret != BLE_STATUS_SUCCESS)
  {
    return ret;
  }

  /* Update the service UUID list. */
  ret = aci_gap_update_adv_data(sizeof(service_uuid_list), service_uuid_list);

  if (ret != BLE_STATUS_SUCCESS)
  {
    return ret;
  }

  /* Update the adverstising flags. */
  ret = aci_gap_update_adv_data(sizeof(flags), flags);

  if (ret != BLE_STATUS_SUCCESS)
  {
    return ret;
  }

  return ret;
}

tBleStatus EddystoneURL_Init(EddystoneURL_InitTypeDef *EddystoneURL_Init)
{
  tBleStatus ret;

  /* Disable scan response. */
  hci_le_set_scan_resp_data(0, NULL);

  uint16_t AdvertisingInterval = (EddystoneURL_Init->AdvertisingInterval * ADVERTISING_INTERVAL_INCREMENT / 10);

  /* Put the device in a non-connectable mode. */
  ret = aci_gap_set_discoverable(ADV_NONCONN_IND,                          /*< Advertise as non-connectable, undirected. */
                                 AdvertisingInterval, AdvertisingInterval, /*< Set the advertising interval as 700 ms (0.625 us increment). */
                                 PUBLIC_ADDR, NO_WHITE_LIST_USE,           /*< Use the public address, with no white list. */
                                 0, NULL,                                  /*< Do not use a local name. */
                                 0, NULL,                                  /*< Do not include the service UUID list. */
                                 0, 0);                                    /*< Do not set a slave connection interval. */

  if (ret != BLE_STATUS_SUCCESS)
  {
    return ret;
  }

  /* Remove the TX power level advertisement (this is done to decrease the packet size). */
  ret = aci_gap_delete_ad_type(AD_TYPE_TX_POWER_LEVEL);

  if (ret != BLE_STATUS_SUCCESS)
  {
    return ret;
  }

  uint8_t service_data[24] =
  {
    6 + EddystoneURL_Init->UrlLength,                                       /*< Length. */
    AD_TYPE_SERVICE_DATA,                                                   /*< Service Data data type value. */
    0xAA, 0xFE,                                                             /*< 16-bit Eddystone UUID. */
    0x10,                                                                   /*< URL frame type. */
    EddystoneURL_Init->CalibratedTxPower,                                   /*< Ranging data. */
    EddystoneURL_Init->UrlScheme,                                           /*< URL Scheme Prefix is http://www. */
    0x00,                                                                   /*< URL */
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00
  };

  for (uint8_t i = 0; i < EddystoneURL_Init->UrlLength; ++i)
  {
    service_data[7 + i] = EddystoneURL_Init->Url[i];
  }

  uint8_t service_uuid_list[] =
  {
    3,                                                                      /*< Length. */
    AD_TYPE_16_BIT_SERV_UUID_CMPLT_LIST,                                    /*< Complete list of 16-bit Service UUIDs data type value. */
    0xAA, 0xFE                                                              /*< 16-bit Eddystone UUID. */
  };

  uint8_t flags[] =
  {
    2,                                                                      /*< Length. */
    AD_TYPE_FLAGS,                                                          /*< Flags data type value. */
    (FLAG_BIT_LE_GENERAL_DISCOVERABLE_MODE | FLAG_BIT_BR_EDR_NOT_SUPPORTED) /*< BLE general discoverable, without BR/EDR support. */
  };

  /* Update the service data. */
  ret = aci_gap_update_adv_data(sizeof(service_data), service_data);

  if (ret != BLE_STATUS_SUCCESS)
  {
    return ret;
  }

  /* Update the service UUID list. */
  ret = aci_gap_update_adv_data(sizeof(service_uuid_list), service_uuid_list);

  if (ret != BLE_STATUS_SUCCESS)
  {
    return ret;
  }

  /* Update the adverstising flags. */
  ret = aci_gap_update_adv_data(sizeof(flags), flags);

  if (ret != BLE_STATUS_SUCCESS)
  {
    return ret;
  }

  return ret;
}

tBleStatus EddystoneTLM_Init(EddystoneTLM_InitTypeDef *EddystoneTLM_Init)
{
  tBleStatus ret = BLE_STATUS_SUCCESS;

  /* Disable scan response. */
  hci_le_set_scan_resp_data(0, NULL);

  uint16_t AdvertisingInterval = (EddystoneTLM_Init->AdvertisingInterval * ADVERTISING_INTERVAL_INCREMENT / 10);

  /* Put the device in a non-connectable mode. */
  ret = aci_gap_set_discoverable(ADV_NONCONN_IND,                          /*< Advertise as non-connectable, undirected. */
                                 AdvertisingInterval, AdvertisingInterval, /*< Set the advertising interval as 700 ms (0.625 us increment). */
                                 PUBLIC_ADDR, NO_WHITE_LIST_USE,           /*< Use the public address, with no white list. */
                                 0, NULL,                                  /*< Do not use a local name. */
                                 0, NULL,                                  /*< Do not include the service UUID list. */
                                 0, 0);                                    /*< Do not set a slave connection interval. */

  if (ret != BLE_STATUS_SUCCESS)
  {
    return ret;
  }

  /* Remove the TX power level advertisement (this is done to decrease the packet size). */
  ret = aci_gap_delete_ad_type(AD_TYPE_TX_POWER_LEVEL);

  if (ret != BLE_STATUS_SUCCESS)
  {
    return ret;
  }

  uint8_t service_data[] =
  {
    17,                                                                      /*< Length. */
    AD_TYPE_SERVICE_DATA,                                                    /*< Service Data data type value. */
    0xAA, 0xFE,                                                              /*< 16-bit Eddystone UUID. */
    0x20,                                                                    /*< TLM frame type. */
    (EddystoneTLM_Init->TLM_Version),                                        /*< TLM version. */
    (EddystoneTLM_Init->BatteryVoltage & 0xFF00) >> 8,                       /*< Battery voltage. */
    (EddystoneTLM_Init->BatteryVoltage & 0x00FF),
    (EddystoneTLM_Init->BeaconTemperature & 0xFF00) >> 8,                    /*< Beacon temperature. */
    (EddystoneTLM_Init->BeaconTemperature & 0x00FF),
    (EddystoneTLM_Init->AdvertisingCount & 0xFF000000) >> 24,                /*< Advertising PDU count. */
    (EddystoneTLM_Init->AdvertisingCount & 0x00FF0000) >> 16,
    (EddystoneTLM_Init->AdvertisingCount & 0x0000FF00) >> 8,
    (EddystoneTLM_Init->AdvertisingCount & 0x000000FF),
    (EddystoneTLM_Init->Uptime & 0xFF000000) >> 24,                          /*< Time since power-on or reboot. */
    (EddystoneTLM_Init->Uptime & 0x00FF0000) >> 16,
    (EddystoneTLM_Init->Uptime & 0x0000FF00) >> 8,
    (EddystoneTLM_Init->Uptime & 0x000000FF)
  };

  uint8_t service_uuid_list[] =
  {
    3,                                                                      /*< Length. */
    AD_TYPE_16_BIT_SERV_UUID_CMPLT_LIST,                                    /*< Complete list of 16-bit Service UUIDs data type value. */
    0xAA, 0xFE                                                              /*< 16-bit Eddystone UUID. */
  };

  uint8_t flags[] =
  {
    2,                                                                      /*< Length. */
    AD_TYPE_FLAGS,                                                          /*< Flags data type value. */
    (FLAG_BIT_LE_GENERAL_DISCOVERABLE_MODE | FLAG_BIT_BR_EDR_NOT_SUPPORTED) /*< BLE general discoverable, without BR/EDR support. */
  };

  /* Update the service data. */
  ret = aci_gap_update_adv_data(sizeof(service_data), service_data);

  if (ret != BLE_STATUS_SUCCESS)
  {
    return ret;
  }

  /* Update the service UUID list. */
  ret = aci_gap_update_adv_data(sizeof(service_uuid_list), service_uuid_list);

  if (ret != BLE_STATUS_SUCCESS)
  {
    return ret;
  }

  /* Update the adverstising flags. */
  ret = aci_gap_update_adv_data(sizeof(flags), flags);

  if (ret != BLE_STATUS_SUCCESS)
  {
    return ret;
  }

  return ret;
}

void EddystoneTLM_Update(uint16_t BatteryVoltage, int16_t BeaconTemperature, uint32_t AdvertisingCount, uint32_t Uptime)
{
  uint8_t service_data[] =
  {
    17,                                                                      /*< Length. */
    AD_TYPE_SERVICE_DATA,                                                    /*< Service Data data type value. */
    0xAA, 0xFE,                                                              /*< 16-bit Eddystone UUID. */
    0x20,                                                                    /*< TLM frame type. */
    0x00,                                                                    /*< TLM version. */
    (BatteryVoltage & 0xFF00) >> 8,                                          /*< Battery voltage. */
    (BatteryVoltage & 0x00FF),
    (BeaconTemperature & 0xFF00) >> 8,                                       /*< Beacon temperature. */
    (BeaconTemperature & 0x00FF),
    (AdvertisingCount & 0xFF000000) >> 24,                                   /*< Advertising PDU count. */
    (AdvertisingCount & 0x00FF0000) >> 16,
    (AdvertisingCount & 0x0000FF00) >> 8,
    (AdvertisingCount & 0x000000FF),
    (Uptime & 0xFF000000) >> 24,                                             /*< Time since power-on or reboot. */
    (Uptime & 0x00FF0000) >> 16,
    (Uptime & 0x0000FF00) >> 8,
    (Uptime & 0x000000FF)
  };

  /* Update the service data. */
  aci_gap_update_adv_data(sizeof(service_data), service_data);
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
