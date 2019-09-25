/*
 * Linux userspace test code, simple and mose code directy from the doco.
 * compile like this: gcc linux_userspace.c ../bme280.c -I ../ -o bme280
 * tested: Raspberry Pi.
 * Use like: ./bme280 /dev/i2c-0
 */

#include "bme280.h"

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <fcntl.h>
#include <time.h>



#define SENSOR_I2C_ADDR 0x76


enum {
	ERR_I2C_BUS_FAIL = 17,
	ERR_I2C_NO_SLAVE = 19,
	ERR_BAD_OUTFMT = 23,
	ERR_NOARGS = 29,
	ERR_UNKNOWN = 37
};


typedef enum _output_format {
	OUTPUT_DEFAULT = 1,
	OUTPUT_PLAIN = 1,
	OUTPUT_PRETTY,
	OUTPUT_ANSI,
	OUTPUT_CSV,
	OUTPUT_JSON,
	OUTPUT_XML,
	OUTPUT_UNKNOWN = 99
} output_format_t;


typedef enum _help_reasons {
	HELP_REASON_GENERIC = 1,
	HELP_REASON_DEFAULT = 1,
	HELP_REASON_REQUESTED,
	HELP_REASON_NOARGS,
	HELP_REASON_BADARGS
} help_reason_t;


struct reading_metadata {
	uint32_t seq;
	time_t ts;
};


struct output_cfg {
	output_format_t output_fmt;
	bool use_timestamps;
};


const uint8_t sensor_i2c_addr = SENSOR_I2C_ADDR;

int fd;
//output_format_t outfmt;



int8_t user_i2c_read(uint8_t id, uint8_t reg_addr, uint8_t *data, uint16_t len) {
	write(fd, &reg_addr, 1);
	read(fd, data, len);

	return (0);
}

void user_delay_ms(uint32_t period) {
	usleep(period * 1000);
}

int8_t user_i2c_write(uint8_t id, uint8_t reg_addr, uint8_t *data, uint16_t len) {
	int8_t *buf;

	buf = malloc(len + 1);
	buf[0] = reg_addr;
	memcpy(buf + 1, data, len);
	write(fd, buf, len + 1);
	free(buf);

	return (0);
}

void print_sensor_data_pretty(struct bme280_data *comp_data) {
	printf("======================\r\n");
	printf("  BME280 SENSOR DATA  \r\n");
	printf("======================\r\n\r\n");
#ifdef BME280_FLOAT_ENABLE
	printf("-- The below data is displayed in floating-point format --\r\n\r\n");
	printf("\tTemperature: %0.3f\r\n", comp_data->temperature);
	printf("\tPressure:    %0.3f\r\n", comp_data->pressure);
	printf("\tHumidity:    %0.3f%%\r\n", comp_data->humidity);
#else
	printf("-- The below data is displayed in integer format --\r\n\r\n");
	printf("\tTemperature: %ld\r\n", comp_data->temperature);
	printf("\tPressure:    %ld\r\n", comp_data->pressure);
	printf("\tHumidity:    %ld%%\r\n", comp_data->humidity);
#endif
	printf("\r\n======================\r\n\r\n");
}

void print_csv_header(bool include_timestamp) {
	if (include_timestamp) {
		printf("reading,timestamp,temperature,pressure,humidity\r\n");
	} else {
		printf("reading,temperature,pressure,humidity\r\n");
	}
}

void print_sensor_data_csv(struct bme280_data *comp_data, struct reading_metadata *rdg_md) {
#ifdef BME280_FLOAT_ENABLE
	if (include_timestamp) {
		printf("%lu,%lu,%0.3f,%0.3f,%0.3f\r\n", rdg_md->seq, rdg_md->ts, comp_data->temperature, comp_data->pressure, comp_data->humidity);
	} else {
		printf("%lu,%0.3f,%0.3f,%0.3f\r\n", rdg_md->seq, comp_data->temperature, comp_data->pressure, comp_data->humidity);
	}
#else
	if (include_timestamp) {
		printf("%lu,%lu,%ld,%ld,%ld\r\n", rdg_md->seq, rdg_md->ts, comp_data->temperature, comp_data->pressure, comp_data->humidity);
	} else {
		printf("%lu,%ld,%ld,%ld\r\n", rdg_md->seq, comp_data->temperature, comp_data->pressure, comp_data->humidity);
	}
#endif
}

void print_sensor_data(struct bme280_data *comp_data) {
	printf("======================\r\n");
	printf("  BME280 SENSOR DATA  \r\n");
	printf("======================\r\n\r\n");
#ifdef BME280_FLOAT_ENABLE
	printf("-- The below data is displayed in floating-point format --\r\n\r\n");
	printf("\tTemperature: %0.3f\r\n", comp_data->temperature);
	printf("\tPressure:    %0.3f\r\n", comp_data->pressure);
	printf("\tHumidity:    %0.3f%%\r\n", comp_data->humidity);
#else
	printf("-- The below data is displayed in integer format --\r\n\r\n");
	printf("\tTemperature: %ld\r\n", comp_data->temperature);
	printf("\tPressure:    %ld\r\n", comp_data->pressure);
	printf("\tHumidity:    %ld%%\r\n", comp_data->humidity);
#endif
	printf("\r\n======================\r\n\r\n");
}

void print_usage_info(help_reason_t reason) {
	char help_reason_msg[50];

	switch (reason) {
		case HELP_REASON_NOARGS: {
			sprintf(help_reason_msg, "No arguments were specified.");
			break;
		}

		default: {
			for (int i = 0; i < 50; i++) {
				
			}
		}
	}
}

int8_t stream_sensor_data_forced_mode(struct bme280_dev *dev) {
	int8_t rslt;
	uint8_t settings_sel;
	struct bme280_data comp_data;
	struct reading_metadata metadata;

	metadata.seq = 0;
	metadata.ts = time(NULL);

	/* Recommended mode of operation: Indoor navigation */
	dev->settings.osr_h = BME280_OVERSAMPLING_1X;
	dev->settings.osr_p = BME280_OVERSAMPLING_16X;
	dev->settings.osr_t = BME280_OVERSAMPLING_2X;
	dev->settings.filter = BME280_FILTER_COEFF_16;

	settings_sel = (BME280_OSR_PRESS_SEL | BME280_OSR_TEMP_SEL | BME280_OSR_HUM_SEL | BME280_FILTER_SEL);

	rslt = bme280_set_sensor_settings(settings_sel, dev);

	switch (outcfg->outfmt) {
		case OUTPUT_CSV: {
			print_csv_header(outcfg->use_timestamps);
			break;
		}

		default: {
			printf("[ERROR] Output format %d is not supported at this time.\r\n", outcfg->outfmt);
			exit(ERR_BAD_OUTCFG);
		}
	}

	/* Continuously stream sensor data */
	while (1) {
		rslt = bme280_set_sensor_mode(BME280_FORCED_MODE, dev);

		/* Wait for the measurement to complete and print data @25Hz */
		dev->delay_ms(40);
		rslt = bme280_get_sensor_data(BME280_ALL, &comp_data, dev);

		switch (outcfg->outfmt) {
			case OUTPUT_CSV: {
				print_sensor_data_csv(&comp_data, &outcfg);
				break;
			}

			default: {
				printf("[ERROR] Output format %d is not supported at this time.\r\n", outcfg->outfmt);
				exit(ERR_BAD_OUTCFG);
			}
		}

//		print_sensor_data(&comp_data);
	}

	return (rslt);
}

int main(int argc, char *argv[]) {
	struct bme280_dev dev;
	struct output_cfg outcfg;
	int8_t rslt = BME280_OK;


	/* Perform argument check(s) */
	if (argc < 2) {
		print_usage_info(HELP_REASON_NOARGS);
		exit(ERR_NOARGS);
	}

	if ((fd = open(argv[1], O_RDWR)) < 0) {
		printf("[FATAL] Failed to open the i2c bus: %s\r\n", argv[1]);
		exit(ERR_I2C_BUS_FAIL);
	}

	if (ioctl(fd, I2C_SLAVE, sensor_i2c_addr) < 0) {
		printf("Failed to acquire bus access and/or talk to the sensor, at i2c slave address 0x%02X\r\n", sensor_i2c_addr);
		exit(ERR_I2C_NO_SLAVE);
	}

	dev.dev_id = BME280_I2C_ADDR_PRIM;
	dev.intf = BME280_I2C_INTF;
	dev.read = user_i2c_read;
	dev.write = user_i2c_write;
	dev.delay_ms = user_delay_ms;

	rslt = bme280_init(&dev);

	/*! \bug We are only outputting in CSV format, for now. */
	outcfg.outfmt = OUTPUT_CSV;

	/*! \bug We are not supporting timestamps right now. */
	outcfg.use_timestamps = false;

	stream_sensor_data_forced_mode(&dev);
}
