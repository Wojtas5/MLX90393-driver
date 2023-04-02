/**
  * MLX90393.c
  *
  *  Created on: Nov 27, 2020
  *      Author: Wojtas5
 **/

/* ==================================================================== */
/* ============================= Includes ============================= */
/* ==================================================================== */

#include <stddef.h>
#include <string.h>
#include "MLX90393.h"

/* ==================================================================== */
/* =========================== Local macros ============================ */
/* ==================================================================== */

/* This macro will prevent optimization in the delay function */
#define NO_OPERATION(x)           (x)

/* Bytes quantity in status byte */
#define STATUS_BYTE_SIZE          ((uint8_t)1)

/* Bytes quantity per one measured value */
#define BYTES_PER_VALUE           ((uint8_t)2)

/* Macro for converting values count to be measured into bytes */
#define VALUES_COUNT_TO_BYTES(x)  ((x * BYTES_PER_VALUE) + STATUS_BYTE_SIZE)

#define TIME_STANDBY_TO_ACTIVE_US ((uint16_t)580)

#define TIME_CONVERSION_END_US    ((uint16_t)100)

/* ==================================================================== */
/* ========================= Local datatypes ========================== */
/* ==================================================================== */
/* Single Magnetic axis measurement time in microseconds [us], based on [DIG_FILT][OSR] */
static const uint32_t Mlx90393_Tconvm[8][4] =
{
    /* DIG_FILT = 0 */
    {259, 451, 835, 1603},
    /* DIG_FILT = 1 */
    {323, 579, 1091, 2115},
    /* DIG_FILT = 2 */
    {451, 835, 1603, 3139},
    /* DIG_FILT = 3 */
    {707, 1347, 2627, 5187},
    /* DIG_FILT = 4 */
    {1219, 2371, 4675, 9283},
    /* DIG_FILT = 5 */
    {2243, 4419, 8771, 17475},
    /* DIG_FILT = 6 */
    {4291, 8515, 16963, 33859},
    /* DIF_FILT = 7 */
    {8387, 16707, 33347, 66627},
};

/* Lookup table to convert raw values to microteslas [uT] based on [HALLCONF][GAIN_SEL][RES][AXIS] */
#ifdef MLX_USE_CONVERSION
static const float Mlx90393_Sensitivity_Lookup[2][8][4][2] =
{
    /* HALLCONF = 0x0 */
    {
        /* GAIN_SEL = 0, 5x gain */
        {{0.787, 1.267}, {1.573, 2.534}, {3.146, 5.068}, {6.292, 10.137}},
        /* GAIN_SEL = 1, 4x gain */
        {{0.629, 1.014}, {1.258, 2.027}, {2.517, 4.055}, {5.034, 8.109}},
        /* GAIN_SEL = 2, 3x gain */
        {{0.472, 0.760}, {0.944, 1.521}, {1.888, 3.041}, {3.775, 6.082}},
        /* GAIN_SEL = 3, 2.5x gain */
        {{0.393, 0.634}, {0.787, 1.267}, {1.573, 2.534}, {3.146, 5.068}},
        /* GAIN_SEL = 4, 2x gain */
        {{0.315, 0.507}, {0.629, 1.014}, {1.258, 2.027}, {2.517, 4.055}},
        /* GAIN_SEL = 5, 1.667x gain */
        {{0.262, 0.422}, {0.524, 0.845}, {1.049, 1.689}, {2.097, 3.379}},
        /* GAIN_SEL = 6, 1.333x gain */
        {{0.210, 0.338}, {0.419, 0.676}, {0.839, 1.352}, {1.678, 2.703}},
        /* GAIN_SEL = 7, 1x gain */
        {{0.157, 0.253}, {0.315, 0.507}, {0.629, 1.014}, {1.258, 2.027}},
    },

    /* HALLCONF = 0xC (default) */
    {
        /* GAIN_SEL = 0, 5x gain */
        {{0.751, 1.210}, {1.502, 2.420}, {3.004, 4.840}, {6.009, 9.680}},
        /* GAIN_SEL = 1, 4x gain */
        {{0.601, 0.968}, {1.202, 1.936}, {2.403, 3.872}, {4.840, 7.744}},
        /* GAIN_SEL = 2, 3x gain */
        {{0.451, 0.726}, {0.901, 1.452}, {1.803, 2.904}, {3.605, 5.808}},
        /* GAIN_SEL = 3, 2.5x gain */
        {{0.376, 0.605}, {0.751, 1.210}, {1.502, 2.420}, {3.004, 4.840}},
        /* GAIN_SEL = 4, 2x gain */
        {{0.300, 0.484}, {0.601, 0.968}, {1.202, 1.936}, {2.403, 3.872}},
        /* GAIN_SEL = 5, 1.667x gain */
        {{0.250, 0.403}, {0.501, 0.807}, {1.001, 1.613}, {2.003, 3.227}},
        /* GAIN_SEL = 6, 1.333x gain */
        {{0.200, 0.323}, {0.401, 0.645}, {0.801, 1.291}, {1.602, 2.581}},
        /* GAIN_SEL = 7, 1x gain */
        {{0.150, 0.242}, {0.300, 0.484}, {0.601, 0.968}, {1.202, 1.936}},
    }
};
#endif /* MLX_USE_CONVERSION */

/* ==================================================================== */
/* =================== Private function prototypes ==================== */
/* ==================================================================== */

static MLX_StatusType MLX90393_cmdStartBurstMode(MLX_HandleType* mlx);
static MLX_StatusType MLX90393_cmdStartSingleMeasurementMode(MLX_HandleType* mlx);
static MLX_StatusType MLX90393_cmdExitMode(MLX_HandleType* mlx);
static MLX_StatusType MLX90393_cmdReset(MLX_HandleType* mlx);
static MLX_StatusType MLX90393_transceive(MLX_HandleType* mlx, uint8_t* writeData, uint8_t writeLen, uint8_t* readData, uint8_t readLen);
static uint8_t ValuesToRead(uint8_t measuredValues);
static void afterResetDelay(void);

/* ==================================================================== */
/* ========================= Public functions ========================= */
/* ==================================================================== */

MLX_StatusType MLX90393_init(MLX_HandleType* mlx)
{
    MLX_StatusType status = MLX_ERROR;
    
    status = MLX90393_resetDevice(mlx);

    if (MLX_OK == status)
    {
        status = MLX90393_write_registers(mlx);
    }

    mlx->measTimeManager.measurementStarted = MEASUREMENT_NOT_STARTED;

    return status;
}

MLX_StatusType MLX90393_write_registers(MLX_HandleType* mlx)
{
    MLX_StatusType status = MLX_ERROR;

    uint8_t reg_addresses[] = {MLX_REG_ADDRESS_0, MLX_REG_ADDRESS_1, MLX_REG_ADDRESS_2};

    for (int i = 0; i < 3; ++i)
    {
        uint16_t reg_data = 0;

        switch (reg_addresses[i])
        {
            case MLX_REG_ADDRESS_0: reg_data = mlx->settings.reg0.data; break;
            case MLX_REG_ADDRESS_1: reg_data = mlx->settings.reg1.data; break;
            case MLX_REG_ADDRESS_2: reg_data = mlx->settings.reg2.data; break;
            default: break;
        }

        status = MLX90393_cmdWriteRegister(mlx, reg_addresses[i], reg_data);

        if (MLX_OK != status)
        {
            break;
        }
    }

    return status;
}

MLX_StatusType MLX90393_read_registers(MLX_HandleType* mlx)
{
    MLX_StatusType status = MLX_ERROR;

    uint8_t reg_addresses[] = {MLX_REG_ADDRESS_0, MLX_REG_ADDRESS_1, MLX_REG_ADDRESS_2};

    for (int i = 0; i < 3; ++i)
    {
        uint16_t* reg_data = NULL;

        switch (reg_addresses[i])
        {
            case MLX_REG_ADDRESS_0: reg_data = &mlx->settings.reg0.data; break;
            case MLX_REG_ADDRESS_1: reg_data = &mlx->settings.reg1.data; break;
            case MLX_REG_ADDRESS_2: reg_data = &mlx->settings.reg2.data; break;
            default: break;
        }

        status = MLX90393_cmdReadRegister(mlx, reg_addresses[i], reg_data);

        if (MLX_OK != status)
        {
            break;
        }
    }

    return status;
}

void MLX90393_clear_registers(MLX_HandleType* mlx)
{
    memset(&mlx->settings, 0, sizeof(mlx->settings));
}

MLX_StatusType MLX90393_getRawData(MLX_HandleType* mlx, MLX_RawValues* rawData, uint32_t(*timestamp)(void))
{
    MLX_StatusType retValue = MLX_ERROR;
    uint8_t valuesCount = ValuesToRead(mlx->measuredValues);
    uint8_t currentIndex = 0u;
    int16_t readData[3];

    /* Check if any values are to be measured */
    if (0u == valuesCount)
    {
        return MLX_PRE_CONDITION;
    }

    //TODO: To be refactored later. Most likely it will be moved to a separate function
    if (MEASUREMENT_NOT_STARTED == mlx->measTimeManager.measurementStarted)
    {
        retValue = MLX90393_setMode(mlx);

        if (MLX_OK == retValue)
        {
            mlx->measTimeManager.measurementStarted = MEASUREMENT_STARTED;
            mlx->measTimeManager.measStartTimestamp = timestamp() * 1000; //ms converted to us
            mlx->measTimeManager.measurementTime = TIME_STANDBY_TO_ACTIVE_US + 
                (valuesCount * Mlx90393_Tconvm[mlx->settings.reg2.bits.digital_filtering][mlx->settings.reg2.bits.oversampling]) + TIME_CONVERSION_END_US; //+ 5000;

            retValue = MLX_MEAS_NOT_READY;
        }
    }
    else if (((timestamp() * 1000) - mlx->measTimeManager.measStartTimestamp) >= mlx->measTimeManager.measurementTime)
    {
        int measEndTimestamp = timestamp();

        retValue = MLX90393_cmdReadMeasurement(mlx, readData, valuesCount);

        if (MLX_OK == retValue)
        {
            mlx->measTimeManager.measEndTimestamp = measEndTimestamp;

            if ((mlx->measuredValues & MLX_AXIS_X) != 0u)
            {
                /* Substract a value which indicates 0G in a specific resolution */
                if (MLX_RESOLUTION_3 == mlx->settings.reg2.bits.resolution_x)
                {
                    readData[currentIndex] -= 0x4000;
                }
                else if (MLX_RESOLUTION_2 == mlx->settings.reg2.bits.resolution_x)
                {
                    readData[currentIndex] -= 0x8000;
                }

                rawData->x = readData[currentIndex];

                ++currentIndex;
            }

            if ((mlx->measuredValues & MLX_AXIS_Y) != 0u)
            {
                /* Substract a value which indicates 0G in a specific resolution */
                if (MLX_RESOLUTION_3 == mlx->settings.reg2.bits.resolution_y)
                {
                    readData[currentIndex] -= 0x4000;
                }
                else if (MLX_RESOLUTION_2 == mlx->settings.reg2.bits.resolution_y)
                {
                    readData[currentIndex] -= 0x8000;
                }

                rawData->y = readData[currentIndex];

                ++currentIndex;
            }

            if ((mlx->measuredValues & MLX_AXIS_Z) != 0u)
            {
                /* Substract a value which indicates 0G in a specific resolution */
                if (MLX_RESOLUTION_3 == mlx->settings.reg2.bits.resolution_z)
                {
                    readData[currentIndex] -= 0x4000;
                }
                else if (MLX_RESOLUTION_2 == mlx->settings.reg2.bits.resolution_z)
                {
                    readData[currentIndex] -= 0x8000;
                }

                rawData->z = readData[currentIndex];

                ++currentIndex;
            }
        }

        mlx->measTimeManager.measurementStarted = MEASUREMENT_NOT_STARTED;
    }
    else
    {
        retValue = MLX_MEAS_NOT_READY;
    }

    return retValue;
}

#ifdef MLX_USE_CONVERSION
MLX_StatusType MLX90393_getConvertedData(MLX_HandleType* mlx, MLX_ConvertedValues* convertedData)
{
    MLX_StatusType retValue = MLX_ERROR;
    uint16_t hallconf = (mlx->settings.hallconf == MLX_HALLCONF_0x0) ? MLX_LOOKUP_HALLCONF_0x0 : MLX_LOOKUP_HALLCONF_0xC;
    MLX_RawValues rawData;

    retValue = MLX90393_getRawData(mlx, &rawData);

    if (MLX_OK == retValue)
    {
        if ((mlx->measuredValues & MLX_AXIS_X) != 0u)
        {
            /* Convert signed data into float basing on the lookup table */
            convertedData->x = (float)rawData.x *
                Mlx90393_Sensitivity_Lookup[hallconf][mlx->settings.gain][mlx->settings.resolutions.x][MLX_LOOKUP_AXIS_XY];
        }

        if ((mlx->measuredValues & MLX_AXIS_Y) != 0u)
        {
            /* Convert signed data into float basing on the lookup table */
            convertedData->y = (float)rawData.y *
                Mlx90393_Sensitivity_Lookup[hallconf][mlx->settings.gain][mlx->settings.resolutions.y][MLX_LOOKUP_AXIS_XY];
        }

        if ((mlx->measuredValues & MLX_AXIS_Z) != 0u)
        {
            /* Convert signed data into float basing on the lookup table */
            convertedData->z = (float)rawData.z *
                Mlx90393_Sensitivity_Lookup[hallconf][mlx->settings.gain][mlx->settings.resolutions.z][MLX_LOOKUP_AXIS_Z];
        }
    }

    return retValue;
}
#endif /* MLX_USE_CONVERSION */

uint8_t MLX90393_getConversionTimeMsInt(MLX_HandleType* mlx)
{
    return (uint8_t)(Mlx90393_Tconvm[mlx->settings.reg2.bits.digital_filtering][mlx->settings.reg2.bits.oversampling] + 1);
}

MLX_StatusType MLX90393_resetDevice(MLX_HandleType* mlx)
{
    MLX_StatusType retValue = MLX_ERROR;

    retValue = MLX90393_cmdExitMode(mlx);

    if (MLX_OK == retValue)
    {
        retValue = MLX90393_cmdReset(mlx);
    }

    /* Small delay for stability */
    afterResetDelay();

    return retValue;
}

MLX_StatusType MLX90393_setMode(MLX_HandleType* mlx)
{
    MLX_StatusType retValue = MLX_ERROR;

    switch (mlx->mode)
    {
        case MLX_EXIT_MODE:
        {
            MLX90393_cmdExitMode(mlx);
            break;
        }

        case MLX_BURST_MODE:
        {
            retValue = MLX90393_cmdStartBurstMode(mlx);
            break;
        }

        case MLX_SINGLE_MEASUREMENT_MODE:
        {
            retValue = MLX90393_cmdStartSingleMeasurementMode(mlx);
            break;
        }

        case MLX_WAKEUP_ON_CHANGE_MODE:
        default:
        {
            retValue = MLX_PRE_CONDITION;
            break;
        }
    }

    return retValue;
}

MLX_StatusType MLX90393_cmdReadMeasurement(MLX_HandleType* mlx, int16_t* readData, uint8_t valuesCount)
{
    MLX_StatusType retValue = MLX_ERROR;
    uint8_t cmd = MLX_CMD_READ_MEASUREMENT | mlx->measuredValues;
    uint8_t readLen = VALUES_COUNT_TO_BYTES(valuesCount);
    uint8_t readBuffer[9];

    retValue = MLX90393_transceive(mlx, &cmd, sizeof(cmd), readBuffer, readLen);

    if (MLX_OK == retValue)
    {
        uint8_t status = readBuffer[0];

        if ((MLX_STATUS_ERROR & status) != 0u)
        {
            retValue = MLX_ERROR;
        }
        else
        {
            /* TODO: Maybe small refactor of this condition? */
            if (MLX_STATUS_BYTES_TO_READ(((valuesCount - 1) & status)) == (readLen - 1))
            {
                for (int i = 0, j = 1; j < readLen; ++i, j += 2)
                {
                    readData[i] = (int16_t)((uint16_t)(readBuffer[j] << 8) | (uint16_t)(readBuffer[j + 1]));
                }
            }
            else
            {
                retValue = MLX_ERROR;
            }
        }
    }

    return retValue;
}

MLX_StatusType MLX90393_cmdReadRegister(MLX_HandleType* mlx, uint8_t regAddress, uint16_t* regData)
{
    MLX_StatusType retValue = MLX_ERROR;
    uint8_t cmd[2] = {MLX_CMD_READ_REGISTER, regAddress << 2};
    uint8_t readBuffer[3];

    retValue = MLX90393_transceive(mlx, cmd, sizeof(cmd), readBuffer, sizeof(readBuffer));

    if (MLX_OK == retValue)
    {
        uint8_t status = readBuffer[0];

        if ((MLX_STATUS_ERROR & status) != 0u)
        {
            retValue = MLX_ERROR;
        }
        else
        {
            *regData = (uint16_t)((uint16_t)(readBuffer[1] << 8) | (uint16_t)readBuffer[2]);
        }
    }

    return retValue;
}

MLX_StatusType MLX90393_cmdWriteRegister(MLX_HandleType* mlx, uint8_t reg_address, uint16_t reg_data)
{
    MLX_StatusType retValue = MLX_ERROR;
    uint8_t command[4] = {MLX_CMD_WRITE_REGISTER, (uint8_t)(reg_data >> 8), (uint8_t)reg_data, reg_address << 2};
    uint8_t status = 0u;

    retValue = MLX90393_transceive(mlx, command, sizeof(command), &status, sizeof(status));

    if (MLX_OK == retValue)
    {
        if ((MLX_STATUS_ERROR & status) != 0u)
        {
            retValue = MLX_ERROR;
        }
    }

    return retValue;
}

/* ==================================================================== */
/* ======================== Private functions ========================= */
/* ==================================================================== */

static MLX_StatusType MLX90393_cmdStartBurstMode(MLX_HandleType* mlx)
{
    MLX_StatusType retValue = MLX_ERROR;
    uint8_t cmd = MLX_CMD_START_BURST_MODE | mlx->measuredValues;
    uint8_t status = 0u;

    retValue = MLX90393_transceive(mlx, &cmd, sizeof(cmd), &status, sizeof(status));

    if (MLX_OK == retValue)
    {
        if ((MLX_STATUS_ERROR & status) != 0u)
        {
            retValue = MLX_ERROR;
        }
        else
        {
            if ((MLX_STATUS_BURST_MODE & status) == 0u)
            {
                retValue = MLX_CMD_REJECTED;
            }
        }  
    }

    return retValue;
}

static MLX_StatusType MLX90393_cmdStartSingleMeasurementMode(MLX_HandleType* mlx)
{
    MLX_StatusType retValue = MLX_ERROR;
    uint8_t cmd = MLX_CMD_START_SINGLE_MODE | mlx->measuredValues;
    uint8_t status = 0u;

    retValue = MLX90393_transceive(mlx, &cmd, sizeof(cmd), &status, sizeof(status));

    if (MLX_OK == retValue)
    {
        if ((MLX_STATUS_ERROR & status) != 0u)
        {
            retValue = MLX_ERROR;
        }
        else
        {
            if ((MLX_STATUS_SINGLE_MODE & status) == 0u)
            {
                retValue = MLX_CMD_REJECTED;
            }
        }
    }

    return retValue;
}

static MLX_StatusType MLX90393_cmdExitMode(MLX_HandleType* mlx)
{
    MLX_StatusType retValue = MLX_ERROR;
    uint8_t cmd = MLX_CMD_EXIT_MODE;
    uint8_t status = 0u;

    retValue = MLX90393_transceive(mlx, &cmd, sizeof(cmd), &status, sizeof(status));

    if (MLX_OK == retValue)
    {
        if ((MLX_STATUS_ERROR & status) != 0u)
        {
            retValue = MLX_ERROR;
        }
        else
        {
            if ((MLX_STATUS_ALL_MODES & status) != 0u)
            {
                retValue = MLX_CMD_REJECTED;
            }
        }
    }

    return retValue;
}

static MLX_StatusType MLX90393_cmdReset(MLX_HandleType* mlx)
{
    MLX_StatusType retValue = MLX_ERROR;
    uint8_t cmd = MLX_CMD_RESET;
    uint8_t status = 0u;

    retValue = MLX90393_transceive(mlx, &cmd, sizeof(cmd), &status, sizeof(status));

    if (MLX_OK == retValue)
    {
        if ((MLX_STATUS_ERROR & status) != 0u)
        {
            retValue = MLX_ERROR;
        }
        else
        {
            if ((MLX_STATUS_RESET & status) == 0u)
            {
                retValue = MLX_CMD_REJECTED;
            }
        }
    }

    return retValue;
}

static MLX_StatusType MLX90393_transceive(MLX_HandleType* mlx, uint8_t* writeData, uint8_t writeLen,
                                             uint8_t* readData, uint8_t readLen)
{
    MLX_StatusType retValue = MLX_ERROR;

    if (NULL != mlx->write)
    {
        retValue = mlx->write(mlx->deviceAddress, writeData, writeLen);
    }
    
    if (0u == retValue)
    {
        if (NULL != mlx->write)
        {
            retValue = mlx->read(mlx->deviceAddress, readData, readLen);
        }

        if (0u == retValue)
        {
            retValue = MLX_OK;
        }
        else
        {
            // failed to receive the data
            retValue = MLX_READ_ERROR;
        }
    }
    else
    {
        // failed to transmit the data
        retValue = MLX_WRITE_ERROR; 
    }

    return retValue;
}

static uint8_t ValuesToRead(uint8_t measuredValues)
{
    uint8_t valuesToRead = 0;

    for (uint8_t i = 0; i < 4; ++i)
    {
        if (((measuredValues >> i) & 1u) == 1u)
        {
            ++valuesToRead;
        }
    }

    return valuesToRead;
}

static void afterResetDelay(void)
{
    /* Value "5000" is calibrated for 200MHz clock speed, and takes only ~300us on 16MHz clock */
    for (volatile uint16_t i = 0; i < 5000u; ++i)
    {
    	NO_OPERATION(i);
    }
}
