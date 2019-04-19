/*
 ******************************************************************************
 * @file    hshcal001_reg.c
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
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include "hshcal001_reg.h"
#include <rtthread.h>
/**
  * @addtogroup  hshcal001
  * @brief  This file provides a set of functions needed to drive the
  *         hshcal001 enanced inertial module.
  * @{
  */

/**
  * @addtogroup  interfaces_functions
  * @brief  This section provide a set of functions used to read and write
  *         a generic register of the device.
  * @{
  */
	


/**
  * @brief  Read generic device register
  *
  * @param  hshcal001_ctx_t* ctx: read / write interface definitions
  * @param  uint8_t reg: register to read
  * @param  uint8_t* data: pointer to buffer that store the data read
  * @param  uint16_t len: number of consecutive register to read
  *
  */
int32_t hshcal001_read_reg(hshcal001_ctx_t *ctx, uint8_t reg, uint8_t *data,
                        uint16_t len)
{
  int32_t ret;
  ret = ctx->read_reg(ctx->handle, reg, data, len);
  return ret;
}

/**
  * @brief  Write generic device register
  *
  * @param  hshcal001_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t reg: register to write
  * @param  uint8_t* data: pointer to data to write in register reg
  * @param  uint16_t len: number of consecutive register to write
  *
*/
int32_t hshcal001_write_reg(hshcal001_ctx_t *ctx, uint8_t reg, uint8_t *data,
                         uint16_t len)
{
  int32_t ret;
  ret = ctx->write_reg(ctx->handle, reg, data, len);
  return ret;
}

/**
  * @brief  data_rate: [set]  Output data rate selection.
  *
  * @param  hshcal001_ctx_t *ctx: read / write interface definitions
  * @param  hshcal001_odr_t: change the values of odr in reg CTRL1_REG
  *
  */
int32_t hshcal001_data_rate_set(hshcal001_ctx_t *ctx, hshcal001_odr_t val)
{
  hshcal001_reg_t reg;
  int32_t ret;

  ret = hshcal001_read_reg(ctx, HSHCAL001_CTRL1_REG, &reg.byte, 1);

  if (ret == 0)
  {
    reg.ctrl1_reg.odr = (uint8_t)val;
    ret = hshcal001_write_reg(ctx, HSHCAL001_CTRL1_REG, &reg.byte, 1);
  }

  return ret;
}

/**
  * @brief  data_rate: [get]  Output data rate selection.
  *
  * @param  hshcal001_ctx_t *ctx: read / write interface definitions
  * @param  hshcal001_odr_t: Get the values of odr in reg CTRL1_REG
  *
  */
int32_t hshcal001_data_rate_get(hshcal001_ctx_t *ctx, hshcal001_odr_t *val)
{
  hshcal001_reg_t reg;
  int32_t ret;

  ret = hshcal001_read_reg(ctx, HSHCAL001_CTRL1_REG, &reg.byte, 1);

  switch (reg.ctrl1_reg.odr)
  {
    case HSHCAL001_ODR_1Hz:
      *val = HSHCAL001_ODR_1Hz;
      break;
    case HSHCAL001_ODR_5Hz:
      *val = HSHCAL001_ODR_5Hz;
      break;
    case HSHCAL001_ODR_10Hz:
      *val = HSHCAL001_ODR_10Hz;
      break;
    default:
      *val = HSHCAL001_ODR_50Hz;
      break;
  }

  return ret;
}

/**
  * @brief  humidity_raw: [get]  Humidity output value
  *
  * @param  hshcal001_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t * : buffer that stores data read
  *
  */
int32_t hshcal001_humidity_raw_get(hshcal001_ctx_t *ctx, uint8_t *buff)
{
	 return    hshcal001_read_reg(ctx, HSHCAL001_HUMIDITY_OUT_L, buff, 2);
}

/**
  * @brief  temperature_raw: [get]  Temperature output value
  *
  * @param  hts221_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t * : buffer that stores data read
  *
  */
int32_t hshcal001_temperature_raw_get(hshcal001_ctx_t *ctx, uint8_t *buff)
{
  return hshcal001_read_reg(ctx, HSHCAL001_TEMP_OUT_L, buff, 2);
}

/**
  * @addtogroup  common
  * @brief   This section group common usefull functions
  * @{
  */

/**
  * @brief  device_id: [get] DeviceWhoamI.
  *
  * @param  hshcal001_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t * : buffer that stores data read
  *
  */
int32_t hshcal001_device_id_get(hshcal001_ctx_t *ctx, uint8_t *buff)
{
  return hshcal001_read_reg(ctx, HSHCAL001_WHO_AM_I, buff, 1);
}

/**
  * @brief  power_on: [set]  Switch device on/off
  *
  * @param  hts221_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t val: change the values of pd in reg CTRL_REG1
  *
  */
int32_t hshcal001_power_on_set(hshcal001_ctx_t *ctx, uint8_t val)
{
  hshcal001_reg_t reg;
  int32_t ret;

  ret = hshcal001_read_reg(ctx, HSHCAL001_CTRL1_REG, &reg.byte, 1);

  if (ret == 0)
  {
    reg.ctrl1_reg.mmd = val;
    ret = hshcal001_write_reg(ctx, HSHCAL001_CTRL1_REG, &reg.byte, 1);
  }
  return ret;
}

/**
  * @brief  power_on: [get]  Switch device on/off
  *
  * @param  hshcal001_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t: change the values of pd in reg CTRL1_REG
  *
  */
int32_t hshcal001_power_on_get(hshcal001_ctx_t *ctx, uint8_t *val)
{
  hshcal001_reg_t reg;
  int32_t mm_error;

  mm_error = hshcal001_read_reg(ctx, HSHCAL001_CTRL1_REG, &reg.byte, 1);
  *val = reg.ctrl1_reg.mmd;

  return mm_error;
}

/**
  * @brief  status: [get]  Info about device status
  *
  * @param  hshcal001_ctx_t *ctx: read / write interface definitions
  * @param  hshcal001_status_t: Registers STATUS_REG
  *
  */
int32_t hshcal001_status_get(hshcal001_ctx_t *ctx, hshcal001_status_reg_t *val)
{
  return hshcal001_read_reg(ctx, HSHCAL001_STATUS_REG, (uint8_t *) val, 1);
}

/**
  * @}
  */

/************************ (C) COPYRIGHT ALCHT IOT *****END OF FILE****/
