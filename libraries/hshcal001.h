/**
 ******************************************************************************
 * @file    hshcal001.h
 * @author  ALCHT IOT Team
 * @brief   HSHCAL001 header driver file
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2018 ALCHT IOT</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of ALCHT IOT nor the names of its contributors
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
#ifndef HSHCAL001_H
#define HSHCAL001_H

#ifdef __cplusplus
extern "C"
{
#endif

/* Includes ------------------------------------------------------------------*/
#include "hshcal001_reg.h"
#include <string.h>

typedef int32_t (*HSHCAL001_Init_Func)(void);
typedef int32_t (*HSHCAL001_DeInit_Func)(void);
typedef int32_t (*HSHCAL001_GetTick_Func)(void);
typedef int32_t (*HSHCAL001_WriteReg_Func)(uint16_t, uint16_t, uint8_t *, uint16_t);
typedef int32_t (*HSHCAL001_ReadReg_Func)(uint16_t, uint16_t, uint8_t *, uint16_t);

typedef struct
{
  HSHCAL001_Init_Func          Init;
  HSHCAL001_DeInit_Func        DeInit;
  uint32_t                  BusType; /*0 means I2C, 1 means SPI-3-Wires */
  uint8_t                   Address;
  HSHCAL001_WriteReg_Func      WriteReg;
  HSHCAL001_ReadReg_Func       ReadReg;
  HSHCAL001_GetTick_Func       GetTick;
} HSHCAL001_IO_t;

typedef struct
{
  HSHCAL001_IO_t        IO;
  hshcal001_ctx_t       Ctx;
  uint8_t            is_initialized;
  uint8_t            hum_is_enabled;
  uint8_t            temp_is_enabled;
} HSHCAL001_Object_t;

typedef struct
{
  uint8_t Temperature;
  uint8_t Humidity;
  uint8_t Power;
  float   Odr;
} HSHCAL001_Capabilities_t;

typedef struct
{
  int32_t (*Init)(HSHCAL001_Object_t *);
  int32_t (*DeInit)(HSHCAL001_Object_t *);
  int32_t (*ReadID)(HSHCAL001_Object_t *, uint8_t *);
  int32_t (*GetCapabilities)(HSHCAL001_Object_t *, HSHCAL001_Capabilities_t *);
} HSHCAL001_CommonDrv_t;

typedef struct
{
  int32_t (*Enable)(HSHCAL001_Object_t *);
  int32_t (*Disable)(HSHCAL001_Object_t *);
  int32_t (*GetOutputDataRate)(HSHCAL001_Object_t *, float *);
  int32_t (*SetOutputDataRate)(HSHCAL001_Object_t *, float);
  int32_t (*GetHumidity)(HSHCAL001_Object_t *, float *);
} HSHCAL001_HUM_Drv_t;

typedef struct
{
  int32_t (*Enable)(HSHCAL001_Object_t *);
  int32_t (*Disable)(HSHCAL001_Object_t *);
  int32_t (*GetOutputDataRate)(HSHCAL001_Object_t *, float *);
  int32_t (*SetOutputDataRate)(HSHCAL001_Object_t *, float);
  int32_t (*GetTemperature)(HSHCAL001_Object_t *, float *);
} HSHCAL001_TEMP_Drv_t;



/** @defgroup HSHCAL001_Exported_Constants HSHCAL001 Exported Constants
 * @{
 */
#define HSHCAL001_I2C_BUS           0U

/** HSHCAL001 error codes  **/
#define HSHCAL001_OK                 0
#define HSHCAL001_ERROR             -1

/**
 * @}
 */
int32_t HSHCAL001_RegisterBusIO(HSHCAL001_Object_t *pObj, HSHCAL001_IO_t *pIO);
int32_t HSHCAL001_Init(HSHCAL001_Object_t *pObj);
int32_t HSHCAL001_DeInit(HSHCAL001_Object_t *pObj);
int32_t HSHCAL001_ReadID(HSHCAL001_Object_t *pObj, uint8_t *Id);
int32_t HSHCAL001_GetCapabilities(HSHCAL001_Object_t *pObj, HSHCAL001_Capabilities_t *Capabilities);
int32_t HSHCAL001_Get_Init_Status(HSHCAL001_Object_t *pObj, uint8_t *Status);

int32_t HSHCAL001_HUM_Enable(HSHCAL001_Object_t *pObj);
int32_t HSHCAL001_HUM_Disable(HSHCAL001_Object_t *pObj);
int32_t HSHCAL001_HUM_GetOutputDataRate(HSHCAL001_Object_t *pObj, float *Odr);
int32_t HSHCAL001_HUM_SetOutputDataRate(HSHCAL001_Object_t *pObj, float Odr);
int32_t HSHCAL001_HUM_GetHumidity(HSHCAL001_Object_t *pObj, float *Value);


int32_t HSHCAL001_TEMP_Enable(HSHCAL001_Object_t *pObj);
int32_t HSHCAL001_TEMP_Disable(HSHCAL001_Object_t *pObj);
int32_t HSHCAL001_TEMP_GetOutputDataRate(HSHCAL001_Object_t *pObj, float *Odr);
int32_t HSHCAL001_TEMP_SetOutputDataRate(HSHCAL001_Object_t *pObj, float Odr);
int32_t HSHCAL001_TEMP_GetTemperature(HSHCAL001_Object_t *pObj, float *Value);


int32_t HSHCAL001_Read_Reg(HSHCAL001_Object_t *pObj, uint8_t Reg, uint8_t *Data);
int32_t HSHCAL001_Write_Reg(HSHCAL001_Object_t *pObj, uint8_t Reg, uint8_t Data);

/**
 * @}
 */

/** @addtogroup HSHCAL001_Exported_Variables HTS221 Exported Variables
 * @{
 */

extern HSHCAL001_CommonDrv_t HSHCAL001_COMMON_Driver;


/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT *****END OF FILE****/
