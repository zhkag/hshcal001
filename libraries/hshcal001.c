/**
 ******************************************************************************
 * @file    hshcal001.c
 * @author  ALCHT IOT Team
 * @brief   HSHCAL001 driver file
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2019 ALCHT IOT</center></h2>
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
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "hshcal001.h"
#include <rtthread.h>

/** @defgroup HSHCAL001_Exported_Variables HSHCAL001 Exported Variables
 * @{
 */

HSHCAL001_CommonDrv_t HSHCAL001_COMMON_Driver =
{
  HSHCAL001_Init,
  HSHCAL001_DeInit,
  HSHCAL001_ReadID,
  HSHCAL001_GetCapabilities,
};

HSHCAL001_HUM_Drv_t HSHCAL001_HUM_Driver =
{
  HSHCAL001_HUM_Enable,
  HSHCAL001_HUM_Disable,
  HSHCAL001_HUM_GetOutputDataRate,
  HSHCAL001_HUM_SetOutputDataRate,
  HSHCAL001_HUM_GetHumidity,

};

HSHCAL001_TEMP_Drv_t HSHCAL001_TEMP_Driver =
{
  HSHCAL001_TEMP_Enable,
  HSHCAL001_TEMP_Disable,
  HSHCAL001_TEMP_GetOutputDataRate,
  HSHCAL001_TEMP_SetOutputDataRate,
  HSHCAL001_TEMP_GetTemperature,
};

/**
 * @}
 */
 
/** @defgroup HSHCAL001_Private_Function_Prototypes HSHCAL001 Private Function Prototypes
 * @{
 */

static int32_t ReadRegWrap(void *Handle, uint8_t Reg, uint8_t *pData, uint16_t Length);
static int32_t WriteRegWrap(void *Handle, uint8_t Reg, uint8_t *pData, uint16_t Length);
static int32_t HSHCAL001_GetOutputDataRate(HSHCAL001_Object_t *pObj, float *Odr);
static int32_t HSHCAL001_SetOutputDataRate(HSHCAL001_Object_t *pObj, float Odr);
static int32_t HSHCAL001_Initialize(HSHCAL001_Object_t *pObj);


/**
 * @}
 */

/** @defgroup HTS221_Exported_Functions HTS221 Exported Functions
 * @{
 */

/**
 * @brief  Register Component Bus IO operations
 * @param  pObj the device pObj
 * @retval 0 in case of success, an error code otherwise
 */
int32_t HSHCAL001_RegisterBusIO(HSHCAL001_Object_t *pObj, HSHCAL001_IO_t *pIO)
{
  int32_t ret;

  if (pObj == NULL)
  {
    ret = HSHCAL001_ERROR;
  }
  else
  {
    pObj->IO.Init      = pIO->Init;
    pObj->IO.DeInit    = pIO->DeInit;
    pObj->IO.BusType   = pIO->BusType;
    pObj->IO.Address   = pIO->Address;
    pObj->IO.WriteReg  = pIO->WriteReg;
    pObj->IO.ReadReg   = pIO->ReadReg;
    pObj->IO.GetTick   = pIO->GetTick;

    pObj->Ctx.read_reg  = ReadRegWrap;
    pObj->Ctx.write_reg = WriteRegWrap;
    pObj->Ctx.handle   = pObj;

    if (pObj->IO.Init != NULL)
    {
      ret = pObj->IO.Init();
    }
    else
    {
      ret = HSHCAL001_ERROR;
    }
  }

  return ret;
}

/**
 * @brief  Initialize the HSHCAL001 sensor
 * @param  pObj the device pObj
 * @retval 0 in case of success, an error code otherwise
 */
int32_t HSHCAL001_Init(HSHCAL001_Object_t *pObj)
{
  if (pObj->is_initialized == 0U)
  {
    if (HSHCAL001_Initialize(pObj) != HSHCAL001_OK)
    {
      return HSHCAL001_ERROR;
    }
  }

  pObj->is_initialized = 1;

  return HSHCAL001_OK;
}

/**
 * @brief  Deinitialize the HSHCAL001 sensor
 * @param  pObj the device pObj
 * @retval 0 in case of success, an error code otherwise
 */
int32_t HSHCAL001_DeInit(HSHCAL001_Object_t *pObj)
{
  if (pObj->is_initialized == 1U)
  {
    if (HSHCAL001_HUM_Disable(pObj) != HSHCAL001_OK)
    {
      return HSHCAL001_ERROR;
    }

    if (HSHCAL001_TEMP_Disable(pObj) != HSHCAL001_OK)
    {
      return HSHCAL001_ERROR;
    }
  }

  pObj->is_initialized = 0;

  return HSHCAL001_OK;
}

/**
 * @brief  Get WHO_AM_I value
 * @param  pObj the device pObj
 * @param  Id the WHO_AM_I value
 * @retval 0 in case of success, an error code otherwise
 */
int32_t HSHCAL001_ReadID(HSHCAL001_Object_t *pObj, uint8_t *Id)
{
  if (hshcal001_device_id_get(&(pObj->Ctx), Id) != HSHCAL001_OK)
  {
    return HSHCAL001_ERROR;
  }

  return HSHCAL001_OK;
}

/**
 * @brief  Get HSHCAL001 sensor capabilities
 * @param  pObj Component object pointer
 * @param  Capabilities pointer to HSHCAL001 sensor capabilities
 * @retval 0 in case of success, an error code otherwise
 */
int32_t HSHCAL001_GetCapabilities(HSHCAL001_Object_t *pObj, HSHCAL001_Capabilities_t *Capabilities)
{
  /* Prevent unused argument(s) compilation warning */
  (void)(pObj);

  Capabilities->Humidity    = 1;
  Capabilities->Temperature = 1;
  Capabilities->Power    = 0;


  return HSHCAL001_OK;
}

/**
 * @brief  Get the HSHCAL001 initialization status
 * @param  pObj the device pObj
 * @param  Status 1 if initialized, 0 otherwise
 * @retval 0 in case of success, an error code otherwise
 */
int32_t HSHCAL001_Get_Init_Status(HSHCAL001_Object_t *pObj, uint8_t *Status)
{
  if (pObj == NULL)
  {
    return HSHCAL001_ERROR;
  }

  *Status = pObj->is_initialized;

  return HSHCAL001_OK;
}

/**
 * @brief  Enable the HSHCAL001 humidity sensor
 * @param  pObj the device pObj
 * @retval 0 in case of success, an error code otherwise
 */
int32_t HSHCAL001_HUM_Enable(HSHCAL001_Object_t *pObj)
{
  /* Check if the component is already enabled */
  if (pObj->hum_is_enabled == 1U)
  {
    return HSHCAL001_OK;
  }

  /* Check if the HSHCAL001 temperature sensor is already enabled. */
  /* If yes, skip the enable function, if not call enable function */
  if (pObj->temp_is_enabled == 0U)
  {
    /* Power on the component. */
    if (hshcal001_power_on_set(&(pObj->Ctx), PROPERTY_ENABLE) != HSHCAL001_OK)
    {
      return HSHCAL001_ERROR;
    }
  }

  pObj->hum_is_enabled = 1;

  return HSHCAL001_OK;
}

/**
 * @brief  Disable the HSHCAL001 humidity sensor
 * @param  pObj the device pObj
 * @retval 0 in case of success, an error code otherwise
 */
int32_t HSHCAL001_HUM_Disable(HSHCAL001_Object_t *pObj)
{
  /* Check if the component is already disabled */
  if (pObj->hum_is_enabled == 0U)
  {
    return HSHCAL001_OK;
  }

  /* Check if the HSHCAL001 temperature sensor is still enable. */
  /* If yes, skip the disable function, if not call disable function */
  if (pObj->temp_is_enabled == 0U)
  {
    /* Power off the component. */
    if (hshcal001_power_on_set(&(pObj->Ctx), PROPERTY_DISABLE) != HSHCAL001_OK)
    {
      return HSHCAL001_ERROR;
    }
  }

  pObj->hum_is_enabled = 0;

  return HSHCAL001_OK;
}

/**
 * @brief  Get the HSHCAL001 humidity sensor output data rate
 * @param  pObj the device pObj
 * @param  Odr pointer where the output data rate is written
 * @retval 0 in case of success, an error code otherwise
 */
int32_t HSHCAL001_HUM_GetOutputDataRate(HSHCAL001_Object_t *pObj, float *Odr)
{
  return HSHCAL001_GetOutputDataRate(pObj, Odr);
}

/**
 * @brief  Set the HSHCAL001 humidity sensor output data rate
 * @param  pObj the device pObj
 * @param  Odr the output data rate value to be set
 * @retval 0 in case of success, an error code otherwise
 */
int32_t HSHCAL001_HUM_SetOutputDataRate(HSHCAL001_Object_t *pObj, float Odr)
{
  return HSHCAL001_SetOutputDataRate(pObj, Odr);
}

/**
 * @brief  Get the HSHCAL001 humidity value
 * @param  pObj the device pObj
 * @param  Value pointer where the humidity value is written
 * @retval 0 in case of success, an error code otherwise
 */
int32_t HSHCAL001_HUM_GetHumidity(HSHCAL001_Object_t *pObj, float *Value)
{
	  
	axis1bit16_t data_raw_humidity;
	uint16_t ht;
	
  if (hshcal001_humidity_raw_get(&(pObj->Ctx), data_raw_humidity.u8bit) != HSHCAL001_OK)
  {
    return HSHCAL001_ERROR;
  }
	
	ht = data_raw_humidity.u8bit[1]*256+data_raw_humidity.u8bit[0];
	
	*Value =ht*0.015625 - 14;
	
  return 0;
}

/**
 * @brief  Enable the HTS221 temperature sensor
 * @param  pObj the device pObj
 * @retval 0 in case of success, an error code otherwise
 */
int32_t HSHCAL001_TEMP_Enable(HSHCAL001_Object_t *pObj)
{
  /* Check if the component is already enabled */
  if (pObj->temp_is_enabled == 1U)
  {
    return HSHCAL001_OK;
  }

  /* Check if the HSHCAL001 humidity sensor is already enabled. */
  /* If yes, skip the enable function, if not call enable function */
  if (pObj->hum_is_enabled == 0U)
  {
    /* Power on the component. */
    if (hshcal001_power_on_set(&(pObj->Ctx), PROPERTY_ENABLE) != HSHCAL001_OK)
    {
      return HSHCAL001_ERROR;
    }
  }

  pObj->temp_is_enabled = 1;

  return HSHCAL001_OK;
}

/**
 * @brief  Disable the HSHCAL001 temperature sensor
 * @param  pObj the device pObj
 * @retval 0 in case of success, an error code otherwise
 */
int32_t HSHCAL001_TEMP_Disable(HSHCAL001_Object_t *pObj)
{
  /* Check if the component is already disabled */
  if (pObj->temp_is_enabled == 0U)
  {
    return HSHCAL001_OK;
  }

  /* Check if the HSHCAL001 humidity sensor is still enable. */
  /* If yes, skip the disable function, if not call disable function */
  if (pObj->hum_is_enabled == 0U)
  {
    /* Power off the component. */
    if (hshcal001_power_on_set(&(pObj->Ctx), PROPERTY_DISABLE) != HSHCAL001_OK)
    {
      return HSHCAL001_ERROR;
    }
  }

  pObj->temp_is_enabled = 0;

  return HSHCAL001_OK;
}

/**
 * @brief  Get the HSHCAL001 temperature sensor output data rate
 * @param  pObj the device pObj
 * @param  Odr pointer where the output data rate is written
 * @retval 0 in case of success, an error code otherwise
 */
int32_t HSHCAL001_TEMP_GetOutputDataRate(HSHCAL001_Object_t *pObj, float *Odr)
{
  return HSHCAL001_GetOutputDataRate(pObj, Odr);
}

/**
 * @brief  Set the HSHCAL001 temperature sensor output data rate
 * @param  pObj the device pObj
 * @param  Odr the output data rate value to be set
 * @retval 0 in case of success, an error code otherwise
 */
int32_t HSHCAL001_TEMP_SetOutputDataRate(HSHCAL001_Object_t *pObj, float Odr)
{
  return HSHCAL001_SetOutputDataRate(pObj, Odr);
}

/**
 * @brief  Get the HSHCAL001 temperature value
 * @param  pObj the device pObj
 * @param  Value pointer where the temperature value is written
 * @retval 0 in case of success, an error code otherwise
 */
int32_t HSHCAL001_TEMP_GetTemperature(HSHCAL001_Object_t *pObj, float *Value)
{
	axis1bit16_t data_raw_temperture;
	uint16_t ht;
	char buf[10];
	
  if (hshcal001_temperature_raw_get(&(pObj->Ctx), data_raw_temperture.u8bit) != HSHCAL001_OK)
  {
    return HSHCAL001_ERROR;
  }
	
	ht = data_raw_temperture.u8bit[1]*256+data_raw_temperture.u8bit[0];
	
	*Value =ht*0.02 -41.92;

  return 0;
}



/**
 * @brief  Get the HSHCAL001 register value
 * @param  pObj the device pObj
 * @param  Reg address to be read
 * @param  Data pointer where the value is written
 * @retval 0 in case of success, an error code otherwise
 */
int32_t HSHCAL001_Read_Reg(HSHCAL001_Object_t *pObj, uint8_t Reg, uint8_t *Data)
{
  if (hshcal001_read_reg(&(pObj->Ctx), Reg, Data, 1) != HSHCAL001_OK)
  {
    return HSHCAL001_ERROR;
  }

  return HSHCAL001_OK;
}

/**
 * @brief  Set the HSHCAL001 register value
 * @param  pObj the device pObj
 * @param  Reg address to be written
 * @param  Data value to be written
 * @retval 0 in case of success, an error code otherwise
 */
int32_t HSHCAL001_Write_Reg(HSHCAL001_Object_t *pObj, uint8_t Reg, uint8_t Data)
{
  if (hshcal001_write_reg(&(pObj->Ctx), Reg, &Data, 1) != HSHCAL001_OK)
  {
    return HSHCAL001_ERROR;
  }

  return HSHCAL001_OK;
}

/**
 * @}
 */

/** @defgroup HSHCAL001_Private_Functions HSHCAL001 Private Functions
 * @{
 */

/**
 * @brief  Get output data rate
 * @param  pObj the device pObj
 * @param  Odr the output data rate value
 * @retval 0 in case of success, an error code otherwise
 */
static int32_t HSHCAL001_GetOutputDataRate(HSHCAL001_Object_t *pObj, float *Odr)
{
  int32_t ret = HSHCAL001_OK;
  hshcal001_odr_t odr_low_level;

  if (hshcal001_data_rate_get(&(pObj->Ctx), &odr_low_level) != HSHCAL001_OK)
  {
    return HSHCAL001_ERROR;
  }

  switch (odr_low_level)
  {
 

    case HSHCAL001_ODR_1Hz:
      *Odr = 1.0f;
      break;

    case HSHCAL001_ODR_5Hz:
      *Odr = 5.0f;
      break;

    case HSHCAL001_ODR_10Hz:
      *Odr = 10.0f;
      break;
	  
	case HSHCAL001_ODR_50Hz:
      *Odr = 50.0f;
      break;

    default:
      ret = HSHCAL001_ERROR;
      break;
  }

  return ret;
}

/**
 * @brief  Set output data rate
 * @param  pObj the device pObj
 * @param  Odr the output data rate value to be set
 * @retval 0 in case of success, an error code otherwise
 */
static int32_t HSHCAL001_SetOutputDataRate(HSHCAL001_Object_t *pObj, float Odr)
{
  hshcal001_odr_t new_odr;

  new_odr = (Odr <= 1.0f) ? HSHCAL001_ODR_1Hz
            : (Odr <= 5.0f) ? HSHCAL001_ODR_5Hz
            : (Odr <= 10.f) ?  HSHCAL001_ODR_10Hz
	          :                  HSHCAL001_ODR_50Hz;

  if (hshcal001_data_rate_set(&(pObj->Ctx), new_odr) != HSHCAL001_OK)
  {
    return HSHCAL001_ERROR;
  }

  return HSHCAL001_OK;
}

/**
 * @brief  Initialize the HSHCAL001 sensor
 * @param  pObj the device pObj
 * @retval 0 in case of success, an error code otherwise
 */
static int32_t HSHCAL001_Initialize(HSHCAL001_Object_t *pObj)
{
  /* Power off the component. */
  if (hshcal001_power_on_set(&(pObj->Ctx), PROPERTY_DISABLE) != HSHCAL001_OK)
  {
    return HSHCAL001_ERROR;
  }


  /* Set default ODR */
  if (HSHCAL001_SetOutputDataRate(pObj, 1.0f) != HSHCAL001_OK)
  {
    return HSHCAL001_ERROR;
  }

  return HSHCAL001_OK;
}

/**
 * @brief  Wrap Read register component function to Bus IO function
 * @param  Handle the device handler
 * @param  Reg the register address
 * @param  pData the stored data pointer
 * @param  Length the length
 * @retval 0 in case of success, an error code otherwise
 */
static int32_t ReadRegWrap(void *Handle, uint8_t Reg, uint8_t *pData, uint16_t Length)
{
  HSHCAL001_Object_t *pObj = (HSHCAL001_Object_t *)Handle;

  if (pObj->IO.BusType == (uint32_t)HSHCAL001_I2C_BUS) /* I2C */
  {
    /* Enable Multi-byte read */
    return pObj->IO.ReadReg(pObj->IO.Address, (Reg | 0x80U), pData, Length);
  }

}

/**
 * @brief  Wrap Write register component function to Bus IO function
 * @param  Handle the device handler
 * @param  Reg the register address
 * @param  pData the stored data pointer
 * @param  Length the length
 * @retval 0 in case of success, an error code otherwise
 */
static int32_t WriteRegWrap(void *Handle, uint8_t Reg, uint8_t *pData, uint16_t Length)
{
  HSHCAL001_Object_t *pObj = (HSHCAL001_Object_t *)Handle;

  if (pObj->IO.BusType == (uint32_t)HSHCAL001_I2C_BUS) /* I2C */
  {
    /* Enable Multi-byte write */
    return pObj->IO.WriteReg(pObj->IO.Address, (Reg | 0x80U), pData, Length);
  }

}

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT ALCHT IOT *****END OF FILE****/
