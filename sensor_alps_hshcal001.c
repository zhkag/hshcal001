
#include "sensor_alps_hshcal001.h"
#include "string.h"

#define DBG_ENABLE
#define DBG_LEVEL DBG_LOG
#define DBG_SECTION_NAME  "sensor.alsp.hshcal001"
#define DBG_COLOR
#include <rtdbg.h>

/***********  Common  *****************/

static HSHCAL001_Object_t hshcal001;
static struct rt_i2c_bus_device *i2c_bus_dev;

static int rt_i2c_write_reg(uint16_t addr, uint16_t reg, uint8_t *data, uint16_t len)
{
    rt_uint8_t tmp = reg;
    struct rt_i2c_msg msgs[2];

    msgs[0].addr  = addr;             /* Slave address */
    msgs[0].flags = RT_I2C_WR;        /* Write flag */
    msgs[0].buf   = &tmp;             /* Slave register address */
    msgs[0].len   = 1;                /* Number of bytes sent */

    msgs[1].addr  = addr;             /* Slave address */
    msgs[1].flags = RT_I2C_WR | RT_I2C_NO_START;        /* Read flag */
    msgs[1].buf   = data;             /* Read data pointer */
    msgs[1].len   = len;              /* Number of bytes read */

    if (rt_i2c_transfer(i2c_bus_dev, msgs, 2) != 2)
    {
        return -RT_ERROR;
    }

    return RT_EOK;
}

static int rt_i2c_read_reg(uint16_t addr, uint16_t reg, uint8_t *data, uint16_t len)
{
    rt_uint8_t tmp = reg;
    struct rt_i2c_msg msgs[2];

    msgs[0].addr  = addr;             /* Slave address */
    msgs[0].flags = RT_I2C_WR;        /* Write flag */
    msgs[0].buf   = &tmp;             /* Slave register address */
    msgs[0].len   = 1;                /* Number of bytes sent */

    msgs[1].addr  = addr;             /* Slave address */
    msgs[1].flags = RT_I2C_RD;        /* Read flag */
    msgs[1].buf   = data;             /* Read data pointer */
    msgs[1].len   = len;              /* Number of bytes read */

    if (rt_i2c_transfer(i2c_bus_dev, msgs, 2) != 2)
    {
        return -RT_ERROR;
    }

    return RT_EOK;
}

static rt_err_t _hshcal001_init(struct rt_sensor_intf *intf)
{
    HSHCAL001_IO_t io_ctx;
    rt_uint8_t        id;

    i2c_bus_dev = (struct rt_i2c_bus_device *)rt_device_find(intf->dev_name);
    if (i2c_bus_dev == RT_NULL)
    {
        return -RT_ERROR;
    }

    /* Configure the humilero driver */
    io_ctx.BusType     = HSHCAL001_I2C_BUS; /* I2C */
    io_ctx.Address     = (rt_uint32_t)(intf->user_data) & 0xff;
//		io_ctx.Address     = 0x18;
    io_ctx.Init        = RT_NULL;
    io_ctx.DeInit      = RT_NULL;
    io_ctx.ReadReg     = rt_i2c_read_reg;
    io_ctx.WriteReg    = rt_i2c_write_reg;
    io_ctx.GetTick     = RT_NULL;

    if (HSHCAL001_RegisterBusIO(&hshcal001, &io_ctx) != HSHCAL001_OK)
    {
        return -RT_ERROR;
    }
    else if (HSHCAL001_ReadID(&hshcal001, &id) != HSHCAL001_OK)
    {
        LOG_D("read id failed");
        return -RT_ERROR;
    }
    if (HSHCAL001_Init(&hshcal001) != HSHCAL001_OK)
    {
        LOG_D("hshcal001 init failed");
        return -RT_ERROR;
    }
    return RT_EOK;
}

/***********  Humi  *****************/

static rt_err_t _hshcal001_humi_set_power(rt_sensor_t sensor, rt_uint8_t power)
{
    if (power == RT_SENSOR_POWER_DOWN)
    {
        LOG_D("set power down");
        HSHCAL001_HUM_Disable(&hshcal001);
    }
    else if (power == RT_SENSOR_POWER_NORMAL)
    {
        LOG_D("set power normal");
        HSHCAL001_HUM_Enable(&hshcal001);
    }
    else
    {
        return -RT_ERROR;
    }
    return RT_EOK;
}

/***********  Temp  *****************/

static rt_err_t _hshcal001_temp_set_power(rt_sensor_t sensor, rt_uint8_t power)
{
    if (power == RT_SENSOR_POWER_DOWN)
    {
        HSHCAL001_TEMP_Disable(&hshcal001);
    }
    else if (power == RT_SENSOR_POWER_NORMAL)
    {
        HSHCAL001_TEMP_Enable(&hshcal001);
    }
    else
    {
        return -RT_ERROR;
    }
    return RT_EOK;
}

static rt_size_t hshcal001_fetch_data(struct rt_sensor_device *sensor, void *buf, rt_size_t len)
{
    struct rt_sensor_data *data = (struct rt_sensor_data *)buf;
    
    if (sensor->info.type == RT_SENSOR_CLASS_TEMP)
    {
        float temp_value;

        HSHCAL001_TEMP_GetTemperature(&hshcal001, &temp_value);

        data->type = RT_SENSOR_CLASS_TEMP;
        data->data.temp = temp_value * 10;
        data->timestamp = rt_sensor_get_ts();
    }
    else if (sensor->info.type == RT_SENSOR_CLASS_HUMI)
    {
        float humi_value;

        HSHCAL001_HUM_GetHumidity(&hshcal001, &humi_value);

        data->type = RT_SENSOR_CLASS_HUMI;
        data->data.humi = humi_value * 10;
        data->timestamp = rt_sensor_get_ts();
    }

    return 1;
}

static rt_err_t hshcal001_control(struct rt_sensor_device *sensor, int cmd, void *args)
{
    rt_err_t result = RT_EOK;

    switch (cmd)
    {
    case RT_SENSOR_CTRL_GET_ID:
        HSHCAL001_ReadID(&hshcal001, args);
		    
        break;
    case RT_SENSOR_CTRL_SET_ODR:
        {
            rt_uint16_t odr = (rt_uint32_t)args & 0xffff;
            if (sensor->info.type == RT_SENSOR_CLASS_TEMP)
                HSHCAL001_TEMP_SetOutputDataRate(&hshcal001, odr);
            else if(sensor->info.type == RT_SENSOR_CLASS_HUMI)
                HSHCAL001_HUM_SetOutputDataRate(&hshcal001, odr);
        }
        break;
    case RT_SENSOR_CTRL_SET_POWER:
        if (sensor->info.type == RT_SENSOR_CLASS_TEMP)
            result = _hshcal001_temp_set_power(sensor, (rt_uint32_t)args & 0xff);
        else if(sensor->info.type == RT_SENSOR_CLASS_HUMI)
            result = _hshcal001_humi_set_power(sensor, (rt_uint32_t)args & 0xff);
        break;
    case RT_SENSOR_CTRL_SELF_TEST:
        result =  -RT_EINVAL;
        break;
    default:
        return -RT_ERROR;
    }
    return result;
}

static struct rt_sensor_ops sensor_ops =
{
    hshcal001_fetch_data,
    hshcal001_control
};

int rt_hw_hshcal001_temp_init(const char *name, struct rt_sensor_config *cfg)
{
    rt_int8_t result;
    rt_sensor_t sensor = RT_NULL;

    sensor = rt_calloc(1, sizeof(struct rt_sensor_device));
    if (sensor == RT_NULL)
        return -1;

    sensor->info.type       = RT_SENSOR_CLASS_TEMP;
    sensor->info.vendor     = RT_SENSOR_VENDOR_STM;
    sensor->info.model      = "hshcal001_temp";
    sensor->info.unit       = RT_SENSOR_UNIT_DCELSIUS;
    sensor->info.intf_type  = RT_SENSOR_INTF_I2C;
    sensor->info.range_max  = 100;
    sensor->info.range_min  = -25;
    sensor->info.period_min = 1000;

    rt_memcpy(&sensor->config, cfg, sizeof(struct rt_sensor_config));
    sensor->ops = &sensor_ops;
    
    result = rt_hw_sensor_register(sensor, name, RT_DEVICE_FLAG_RDWR, RT_NULL);
    if (result != RT_EOK)
    {
        LOG_E("device register err code: %d", result);
        rt_free(sensor);
        return -RT_ERROR;
    }

    LOG_I("temp sensor init success");
    return RT_EOK;
}

int rt_hw_hshcal001_humi_init(const char *name, struct rt_sensor_config *cfg)
{
    rt_int8_t result;
    rt_sensor_t sensor = RT_NULL;

    sensor = rt_calloc(1, sizeof(struct rt_sensor_device));
    if (sensor == RT_NULL)
        return -1;

    sensor->info.type       = RT_SENSOR_CLASS_HUMI;
    sensor->info.vendor     = RT_SENSOR_VENDOR_STM;
    sensor->info.model      = "hshcal001_humi";
    sensor->info.unit       = RT_SENSOR_UNIT_PERMILLAGE;
    sensor->info.intf_type  = RT_SENSOR_INTF_I2C;
    sensor->info.range_max  = 100;
    sensor->info.range_min  = 0;
    sensor->info.period_min = 1000;

    rt_memcpy(&sensor->config, cfg, sizeof(struct rt_sensor_config));
    sensor->ops = &sensor_ops;
    
    result = rt_hw_sensor_register(sensor, name, RT_DEVICE_FLAG_RDWR, RT_NULL);
    if (result != RT_EOK)
    {
        LOG_E("device register err code: %d", result);
        rt_free(sensor);
        return -RT_ERROR;
    }

    LOG_I("humi sensor init success");
    return RT_EOK;
}

int rt_hw_hshcal001_init(const char *name, struct rt_sensor_config *cfg)
{
    _hshcal001_init(&cfg->intf);
    
#ifdef PKG_USING_HSHCAL001_TEMP
    rt_hw_hshcal001_temp_init(name, cfg);
#endif
#ifdef PKG_USING_HSHCAL001_HUMI
    rt_hw_hshcal001_humi_init(name, cfg);
#endif

    return 0;
}
