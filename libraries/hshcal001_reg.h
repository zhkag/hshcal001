/*
 ******************************************************************************
 * @file    hshcal001_reg.h
 * @author  ALCHT IOT Team
 * @brief   This file contains all the functions prototypes for the
 *          hshcal001_reg.c driver.
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
 
 /* Define to prevent recursive inclusion -------------------------------------*/
#ifndef HSHCAL001_REGS_H
#define HSHCAL001_REGS_H

#ifdef __cplusplus
extern "C" {
#endif
	


/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
	
typedef union
{
  int16_t i16bit[3];
  uint8_t u8bit[6];
} axis3bit16_t;

typedef union
{
  int16_t i16bit;
  uint8_t u8bit[2];
} axis1bit16_t;

	typedef struct
{
  uint8_t bit0       : 1;
  uint8_t bit1       : 1;
  uint8_t bit2       : 1;
  uint8_t bit3       : 1;
  uint8_t bit4       : 1;
  uint8_t bit5       : 1;
  uint8_t bit6       : 1;
  uint8_t bit7       : 1;
} bitwise_t;

#define PROPERTY_DISABLE                (0)
#define PROPERTY_ENABLE                 (1)

/** @defgroup hshcal001_interface
  * @{
  */

typedef int32_t (*hshcal001_write_ptr)(void *, uint8_t, uint8_t *, uint16_t);
typedef int32_t (*hshcal001_read_ptr)(void *, uint8_t, uint8_t *, uint16_t);

typedef struct
{
  /** Component mandatory fields **/
  hshcal001_write_ptr  write_reg;
  hshcal001_read_ptr   read_reg;
  /** Customizable optional pointer **/
  void *handle;
} hshcal001_ctx_t;



/** @defgroup hshcal001_Infos
  * @{
  */
/** I2C Device Address 8 bit format  **/
#define HSHCAL001_I2C_ADDRESS   0x18

/** Device Identification (Who am I) **/
#define HSHCAL001_ID            0x1123

/**
  * @}
  */

#define HSHCAL001_WHO_AM_I            0x0f

#define HSHCAL001_CTRL1_REG           0x1B
typedef struct
{
 
  uint8_t reserved1_CTRL1    : 1;
  uint8_t fs                 : 1;
  uint8_t lp                 : 1;
  uint8_t odr                : 2;
  uint8_t mmd                : 2;
  uint8_t reserved2_CTRL1    : 1;
} hshcal001_ctrl1_t;

#define HSHCAL001_STATUS_REG          0x18
typedef struct
{
  uint8_t reserved1_status   : 1;
  uint8_t trdy               : 1;
  uint8_t reserved2_status   : 2;
  uint8_t meas               : 1;
  uint8_t odr                : 1;
  uint8_t drdy               : 1;
  uint8_t srdy               : 1;
} hshcal001_status_reg_t;

#define HSHCAL001_HUMIDITY_OUT_L      0x10
#define HSHCAL001_HUMIDITY_OUT_H      0x11
#define HSHCAL001_TEMP_OUT_L          0x12
#define HSHCAL001_TEMP_OUT_H          0x13

typedef union
{
 
  hshcal001_ctrl1_t          ctrl1_reg;
  hshcal001_status_reg_t     status_reg;
  bitwise_t                  bitwise;
  uint8_t                    byte;
} hshcal001_reg_t;

int32_t hshcal001_read_reg(hshcal001_ctx_t *ctx, uint8_t reg, uint8_t *data,
                        uint16_t len);
int32_t hshcal001_write_reg(hshcal001_ctx_t *ctx, uint8_t reg, uint8_t *data,
                         uint16_t len);
						 
typedef enum
{
  HSHCAL001_ODR_1Hz      = 0,
  HSHCAL001_ODR_5Hz      = 1,
  HSHCAL001_ODR_10Hz     = 2,
  HSHCAL001_ODR_50Hz     = 3,
} hshcal001_odr_t;

int32_t hshcal001_data_rate_set(hshcal001_ctx_t *ctx, hshcal001_odr_t val);
int32_t hshcal001_data_rate_get(hshcal001_ctx_t *ctx, hshcal001_odr_t *val);



int32_t hshcal001_force_set(hshcal001_ctx_t *ctx, uint8_t val);
int32_t hshcal001_force_get(hshcal001_ctx_t *ctx, uint8_t *val);

int32_t hshcal001_humidity_raw_get(hshcal001_ctx_t *ctx, uint8_t *buff);

int32_t hshcal001_temperature_raw_get(hshcal001_ctx_t *ctx, uint8_t *buff);

int32_t hshcal001_device_id_get(hshcal001_ctx_t *ctx, uint8_t *buff);

int32_t hshcal001_power_on_set(hshcal001_ctx_t *ctx, uint8_t val);

int32_t hshcal001_power_on_get(hshcal001_ctx_t *ctx, uint8_t *val);

int32_t hts221_status_get(hshcal001_ctx_t *ctx, hshcal001_status_reg_t *val);


/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /*HSHCAL001_REGS_H */

/************************ (C) COPYRIGHT ALCHT IOT *****END OF FILE****/
