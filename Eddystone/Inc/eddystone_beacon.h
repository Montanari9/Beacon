/**
  ******************************************************************************
  * @file    eddystone_beacon.h
  * @author  MCD Application Team
  * @version
  * @date    8/18/2015
  * @brief
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef EDDYSTONE_BEACON_H
#define EDDYSTONE_BEACON_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#if defined (__CC_ARM)
	#define ASSERT_CONCAT_(a, b) a##b
	#define ASSERT_CONCAT(a, b) ASSERT_CONCAT_(a, b)
	/* These can't be used after statements in c89. */
	#ifdef __COUNTER__
		#define STATIC_ASSERT(e,m) \
			;enum { ASSERT_CONCAT(static_assert_, __COUNTER__) = 1/(!!(e)) }
	#else
		/* This can't be used twice on the same line so ensure if using in headers
		 * that the headers are not included twice (by wrapping in #ifndef...#endif)
		 * Note it doesn't cause an issue when used on same line of separate modules
		 * compiled with gcc -combine -fwhole-program.  */
		#define static_assert(e,m) \
			;enum { ASSERT_CONCAT(assert_line_, __LINE__) = 1/(!!(e)) }
	#endif
#endif

/* Exported types ------------------------------------------------------------*/
enum
{
  NOT_CONNECTED,
  CONNECTED
};

/* Exported constants --------------------------------------------------------*/
#define MAC_ADDRESS                 0x06, 0x05, 0x04, 0x03, 0x02, 0x04

#define EDDYSTONE_UID_BEACON_TYPE   (0x01u)
#define EDDYSTONE_URL_BEACON_TYPE   (0x02u)
#define EDDYSTONE_TLM_BEACON_TYPE   (0x04u)

#define EDDYSTONE_BEACON_TYPE       (EDDYSTONE_URL_BEACON_TYPE)

#define ADVERTISING_INTERVAL_IN_MS  (1000)
#define CALIBRATED_TX_POWER_AT_0_M  ((uint8_t) (-22))
#define NAMESPACE_ID                'w', 'w', 'w', '.', 's', 't', '.', 'c', 'o', 'm'
#define BEACON_ID                   0, 0, 0, 0, 0, 1
#define URL_PREFIX                  HTTP
#define PHYSICAL_WEB_URL            "goo.gl/DnHNNM"
//#define PHYSICAL_WEB_URL            "www.st.com"

#if (0 != (EDDYSTONE_BEACON_TYPE & (EDDYSTONE_BEACON_TYPE - 1)))
  #error "Please select only a single beacon type!"
#endif

#if ((ADVERTISING_INTERVAL_IN_MS <= 0) || (ADVERTISING_INTERVAL_IN_MS > 40959))
  #error "Invalid advertising interval! Please select a value between 0 and 40959 ms."
#endif

#if (0 != (EDDYSTONE_BEACON_TYPE & EDDYSTONE_TLM_BEACON_TYPE))
  #if ((ADVERTISING_INTERVAL_IN_MS < 1000) || ((ADVERTISING_INTERVAL_IN_MS % 1000) != 0))
    #error "EDDYSTONE_TLM only supports intervals in increments of 1 s!"
  #endif
#endif

#if (0 != (EDDYSTONE_BEACON_TYPE & EDDYSTONE_URL_BEACON_TYPE))
#if defined (__CC_ARM)
  STATIC_ASSERT(sizeof(PHYSICAL_WEB_URL) < 17, "The URL must be less than 17 characters.");
#elif defined (__ICCARM__)
  static_assert(sizeof(PHYSICAL_WEB_URL) < 17, "The URL must be less than 17 characters.");
#else
#warning Please check that sizeof(PHYSICAL_WEB_URL) < 17
#endif
#endif

/* Exported Macros -----------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif /* EDDYSTONE_BEACON_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
