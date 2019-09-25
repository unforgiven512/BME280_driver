/**
 * Copyright (C) 2018 - 2019 Bosch Sensortec GmbH
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 *
 * Neither the name of the copyright holder nor the names of the
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER
 * OR CONTRIBUTORS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
 * OR CONSEQUENTIAL DAMAGES(INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE
 *
 * The information provided is believed to be accurate and reliable.
 * The copyright holder assumes no responsibility
 * for the consequences of use
 * of such information nor for any infringement of patents or
 * other rights of third parties which may result from its use.
 * No license is granted by implication or otherwise under any patent or
 * patent rights of the copyright holder.
 *
 * @file    bme280_defs.h
 * @date    08 Mar 2019
 * @version 3.3.6
 * @brief Definitions required for the BME280 sensor driver
 *
 */

#ifndef BME280_DEFS_H_
#define BME280_DEFS_H_


/* header includes */
#if defined(__KERNEL__)

/* (These includes are used for Linux systems) */
#include <linux/types.h>
#include <linux/kernel.h>

#else

/* (These includes are used for generic/"bare-metal" systems) */
#include <stdint.h>
#include <stddef.h>

#endif


/**
 * \defgroup bme280_defs BME280 Definitions
 *
 * @{
 */


/**
 * \name Common Macros
 *
 * @{
 */

#if !defined(UINT8_C) && !defined(INT8_C)
#define INT8_C(x) S8_C(x)
#define UINT8_C(x) U8_C(x)
#endif

#if !defined(UINT16_C) && !defined(INT16_C)
#define INT16_C(x) S16_C(x)
#define UINT16_C(x) U16_C(x)
#endif

#if !defined(INT32_C) && !defined(UINT32_C)
#define INT32_C(x) S32_C(x)
#define UINT32_C(x) U32_C(x)
#endif

#if !defined(INT64_C) && !defined(UINT64_C)
#define INT64_C(x) S64_C(x)
#define UINT64_C(x) U64_C(x)
#endif

/**
 * @}
 */

/**
 * \name C standard macros
 *
 * @{
 */

#ifndef NULL
#ifdef __cplusplus
#define NULL 0
#else	/* __cplusplus */
#define NULL ((void*)0)
#endif	/* !__cplusplus */
#endif	/* !NULL */

/**
 * @}
 */

/* ------------------------------------------------------------------------------------------------------------------ *
 *   DOCUMENTATION SECTION FOR DOXYGEN -- NOT TO BE COMPILED                                                          *
 * ------------------------------------------------------------------------------------------------------------------ */

#if defined(__DOXYGEN__)

/**
 * \def BME280_FLOAT_ENABLE
 * \brief Enables use of floating-point values for sensor data
 */
#define BME280_FLOAT_ENABLE


#endif	/* defined(__DOXYGEN__) */

/* ------------------------------------------------------------------------------------------------------------------ */



#ifndef BME280_FLOAT_ENABLE
/* #define BME280_FLOAT_ENABLE */
#endif

#ifndef BME280_FLOAT_ENABLE
#ifndef BME280_64BIT_ENABLE
#define BME280_64BIT_ENABLE
#endif	/* !BME280_64BIT_ENABLE */
#endif	/* !BME280_FLOAT_ENABLE */



#ifndef TRUE
#define TRUE UINT8_C(1)
#endif

#ifndef FALSE
#define FALSE UINT8_C(0)
#endif



/**\name I2C addresses */
#define BME280_I2C_ADDR_PRIM              UINT8_C(0x76)
#define BME280_I2C_ADDR_SEC               UINT8_C(0x77)

/**\name BME280 chip identifier */
#define BME280_CHIP_ID                    UINT8_C(0x60)

/**\name Register Address */
#define BME280_CHIP_ID_ADDR               UINT8_C(0xD0)
#define BME280_RESET_ADDR                 UINT8_C(0xE0)
#define BME280_TEMP_PRESS_CALIB_DATA_ADDR UINT8_C(0x88)
#define BME280_HUMIDITY_CALIB_DATA_ADDR   UINT8_C(0xE1)
#define BME280_PWR_CTRL_ADDR              UINT8_C(0xF4)
#define BME280_CTRL_HUM_ADDR              UINT8_C(0xF2)
#define BME280_CTRL_MEAS_ADDR             UINT8_C(0xF4)
#define BME280_CONFIG_ADDR                UINT8_C(0xF5)
#define BME280_DATA_ADDR                  UINT8_C(0xF7)

/**\name API success code */
#define BME280_OK                         INT8_C(0)

/**\name API error codes */
#define BME280_E_NULL_PTR                 INT8_C(-1)
#define BME280_E_DEV_NOT_FOUND            INT8_C(-2)
#define BME280_E_INVALID_LEN              INT8_C(-3)
#define BME280_E_COMM_FAIL                INT8_C(-4)
#define BME280_E_SLEEP_MODE_FAIL          INT8_C(-5)

/**\name API warning codes */
#define BME280_W_INVALID_OSR_MACRO        INT8_C(1)

/**\name Macros related to size */
#define BME280_TEMP_PRESS_CALIB_DATA_LEN  UINT8_C(26)
#define BME280_HUMIDITY_CALIB_DATA_LEN    UINT8_C(7)
#define BME280_P_T_H_DATA_LEN             UINT8_C(8)

/**\name Sensor power modes */
#define BME280_SLEEP_MODE                 UINT8_C(0x00)
#define BME280_FORCED_MODE                UINT8_C(0x01)
#define BME280_NORMAL_MODE                UINT8_C(0x03)


/**
 * \defgroup bme280_defs_bitops Bit and Byte value operations
 *
 * Various macro functions to perform manipulations and operations on bit and byte data.
 *
 * @{
 */

/**
 * \brief Macro to combine two 8 bit data's to form a 16 bit data
 *
 * \param[in]	msb	The most significant byte (ie: the "upper" 8 bits)
 * \param[in]	lsb	The least significant byte (ie: the "lower" 8 bits)
 *
 * \return An unsigned 16 bit value will be the result of this macro.
 */
#define BME280_CONCAT_BYTES(msb, lsb)																				\
		(((uint16_t)msb << 8) | (uint16_t)lsb)

/**
 * \brief Set a bit value in a register, utilizing bit shifting
 */
#define BME280_SET_BITS(reg_data, bitname, data)																	\
		((reg_data & ~(bitname##_MSK)) | ((data << bitname##_POS) & bitname##_MSK))

/**
 * \brief Set a bit value in a register, with the value "in-place"
 */
#define BME280_SET_BITS_POS_0(reg_data, bitname, data)																\
		((reg_data & ~(bitname##_MSK)) | (data & bitname##_MSK))

/**
 * \brief Get a bit value from a register, utilizing bit shifting
 */
#define BME280_GET_BITS(reg_data, bitname)																			\
		((reg_data & (bitname##_MSK)) >> (bitname##_POS))

/**
 * \brief Get a bit value from a register, with the value "in-place"
 */
#define BME280_GET_BITS_POS_0(reg_data, bitname)																	\
		(reg_data & (bitname##_MSK))


/**
 * @}
 */


/**
 * \name Macros for bit masking
 *
 * @{
 */

#define BME280_SENSOR_MODE_MSK      UINT8_C(0x03)
#define BME280_SENSOR_MODE_POS      UINT8_C(0x00)

#define BME280_CTRL_HUM_MSK         UINT8_C(0x07)
#define BME280_CTRL_HUM_POS         UINT8_C(0x00)

#define BME280_CTRL_PRESS_MSK       UINT8_C(0x1C)
#define BME280_CTRL_PRESS_POS       UINT8_C(0x02)

#define BME280_CTRL_TEMP_MSK        UINT8_C(0xE0)
#define BME280_CTRL_TEMP_POS        UINT8_C(0x05)

#define BME280_FILTER_MSK           UINT8_C(0x1C)
#define BME280_FILTER_POS           UINT8_C(0x02)

#define BME280_STANDBY_MSK          UINT8_C(0xE0)
#define BME280_STANDBY_POS          UINT8_C(0x05)

/**
 * @}
 */


/**
 * \name Sensor component selection macros
 *
 * \attention These values are internal for API implementation. Don't relate this to data sheet.
 *
 * @{
 */

#define BME280_PRESS                UINT8_C(1)
#define BME280_TEMP                 UINT8_C(1 << 1)
#define BME280_HUM                  UINT8_C(1 << 2)
#define BME280_ALL                  UINT8_C(0x07)

/**
 * @}
 */


/**
 * \defgroup bme280_defs_setting_values Device Setting Value Definitions
 *
 * These are a collection of defined symbols, containing the values for each option available in each of the device
 * settings:
 * - Oversampling
 *   - Temperature
 *   - Pressure
 *   - Humidity
 * - Standby time
 * - Filter coefficients
 *
 * Additionally, there are bitmasks defined for the selection of a specific setting.
 *
 * @{
 */

/**
 * \name Settings selection macros
 *
 * @{
 */

#define BME280_OSR_PRESS_SEL        UINT8_C(1)
#define BME280_OSR_TEMP_SEL         UINT8_C(1 << 1)
#define BME280_OSR_HUM_SEL          UINT8_C(1 << 2)
#define BME280_FILTER_SEL           UINT8_C(1 << 3)
#define BME280_STANDBY_SEL          UINT8_C(1 << 4)
#define BME280_ALL_SETTINGS_SEL     UINT8_C(0x1F)

/**
 * @}
 */


/**
 * \name Oversampling macros
 *
 * @{
 */

#define BME280_NO_OVERSAMPLING      UINT8_C(0x00)
#define BME280_OVERSAMPLING_1X      UINT8_C(0x01)
#define BME280_OVERSAMPLING_2X      UINT8_C(0x02)
#define BME280_OVERSAMPLING_4X      UINT8_C(0x03)
#define BME280_OVERSAMPLING_8X      UINT8_C(0x04)
#define BME280_OVERSAMPLING_16X     UINT8_C(0x05)

/**
 * @}
 */


/**
 * \name Standby duration selection macros
 *
 * @{
 */

#define BME280_STANDBY_TIME_0_5_MS    (0x00)
#define BME280_STANDBY_TIME_62_5_MS (0x01)
#define BME280_STANDBY_TIME_125_MS  (0x02)
#define BME280_STANDBY_TIME_250_MS  (0x03)
#define BME280_STANDBY_TIME_500_MS  (0x04)
#define BME280_STANDBY_TIME_1000_MS (0x05)
#define BME280_STANDBY_TIME_10_MS   (0x06)
#define BME280_STANDBY_TIME_20_MS   (0x07)

/**
 * @}
 */


/**
 * \name Filter coefficient selection macros
 *
 * @{
 */

#define BME280_FILTER_COEFF_OFF     (0x00)
#define BME280_FILTER_COEFF_2       (0x01)
#define BME280_FILTER_COEFF_4       (0x02)
#define BME280_FILTER_COEFF_8       (0x03)
#define BME280_FILTER_COEFF_16      (0x04)

/**
 * @}
 */


/**
 * @}
 */

/**
 * \brief BME280 Communication Interfaces
 */
enum bme280_intf {
	BME280_SPI_INTF,	/*!< SPI interface */
	BME280_I2C_INTF		/*!< I²C interface */
};

/**
 * \defgroup bme280_defs_typedefs Type definitions
 *
 * @{
 */

typedef int8_t (*bme280_com_fptr_t)(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len);
typedef void (*bme280_delay_fptr_t)(uint32_t period);

/**
 * @}
 */


/**
 * \defgroup bme280_defs_structs Data Structures
 *
 * @{
 */

/**
 * \brief BME280 calibration data
 *
 * The BME280 calibration data consists of "trim" variables.
 */
struct bme280_calib_data {
	uint16_t dig_T1;
	int16_t dig_T2;
	int16_t dig_T3;
	uint16_t dig_P1;
	int16_t dig_P2;
	int16_t dig_P3;
	int16_t dig_P4;
	int16_t dig_P5;
	int16_t dig_P6;
	int16_t dig_P7;
	int16_t dig_P8;
	int16_t dig_P9;
	uint8_t dig_H1;
	int16_t dig_H2;
	uint8_t dig_H3;
	int16_t dig_H4;
	int16_t dig_H5;
	int8_t dig_H6;
	int32_t t_fine;
};

/**
 * \brief BME280 Sensor Data
 *
 * This is the BME280 sensor data structure, which comprises of:
 * - temperature
 * - pressure
 * - humidity
 *
 * \note If the preprocessor symbol \c BME280_FLOAT_ENABLE is defined at the time of compilation, the data will be held
 *       as double-precision floating-point values; the data will be held as 32 bit integer values otherwise.
 */
struct bme280_data {
#ifdef BME280_FLOAT_ENABLE
	double pressure;		/*!< Compensated pressure data */
	double temperature;		/*!< Compensated temperature data */
	double humidity;		/*!< Compensated humidity data */
#else
	uint32_t pressure;		/*!< Compensated pressure data */
	int32_t temperature;	/*!< Compensated temperature data */
	uint32_t humidity;		/*!< Compensated humidity data */
#endif /* BME280_USE_FLOATING_POINT */
};

/**
 * \brief BME280 Sensor Uncompensated Data
 *
 * This is the BME280 \e "raw" (uncompensated) sensor data structure, which comprises of:
 * - temperature
 * - pressure
 * - humidity
 */
struct bme280_uncomp_data {
	uint32_t pressure;		/*!< Uncompensated pressure data */
	int32_t temperature;	/*!< Uncompensated temperature data */
	uint32_t humidity;		/*!< Uncompensated humidity data */
};

/**
 * \brief BME280 sensor settings structure
 *
 * This is the BME280 sensor settings structure, which comprises of:
 * - mode
 * - oversampling settings
 * - filter settings
 * - standby time
 */
struct bme280_settings {
	uint8_t osr_p;			/*!< pressure data oversampling setting */
	uint8_t osr_t;			/*!< temperature data oversampling setting */
	uint8_t osr_h;			/*!< humidity data oversampling setting */
	uint8_t filter;			/*!< filter coefficient */
	uint8_t standby_time;	/*!< standby time */
};

/*! \brief BME280 device structure */
struct bme280_dev {
	uint8_t chip_id;						/*!< Chip ID */
	uint8_t dev_id;							/*!< Device ID */
	enum bme280_intf intf;					/*!< Sensor interface */
	bme280_com_fptr_t read;					/*!< Read function pointer */
	bme280_com_fptr_t write;				/*!< Write function pointer */
	bme280_delay_fptr_t delay_ms;			/*!< Delay function pointer */
	struct bme280_calib_data calib_data;	/*!< Trim/calibration data */
	struct bme280_settings settings;		/*!< Sensor settings */
};

/**
 * @}
 */

/**
 * @}
 */

#endif	/* !BME280_DEFS_H_ */
