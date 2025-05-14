#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>

#include "drivers/bus/i2c_bus.h"
#include "drivers/ms5611/ms5611.h"

#define MS5611_ADDR  0x77  // or 0x76, whichever your board uses

static i2c_bus_handle_t i2c;

static ms5611_return_code_t plat_read(uint8_t reg, uint8_t *buf, uint32_t len) {
    // write register, then read len bytes
    return i2c_bus_write_read(i2c, MS5611_ADDR, &reg, 1, buf, len);
}

static ms5611_return_code_t plat_write(uint8_t reg, const uint8_t *buf, uint32_t len) {
    uint8_t tmp[1 + len];
    tmp[0] = reg;
    memcpy(tmp+1, buf, len);
    return i2c_bus_write(i2c, MS5611_ADDR, tmp, len+1);
}

static void plat_delay_us(uint32_t us) { usleep(us); }

int main(void) {
    i2c_config_t cfg = { .bus_num = 1, .speed = I2C_SPEED_400KHZ };
    if (i2c_bus_init(&cfg, &i2c) != STATUS_OK) {
        fprintf(stderr, "FATAL: I2C init failed\n");
        return 1;
    }

    ms5611_dev_t dev;
    memset(&dev, 0, sizeof(dev));
    if (ms5611_init(&dev, plat_read, plat_write, plat_delay_us) != MS5611_RET_OK) {
        fprintf(stderr, "FATAL: ms5611_init failed\n");
        return 2;
    }

    printf("PROM Coeffs: ");
    for (int i = 1; i <= 6; i++)
        printf("%u ", dev.prom_coeffs[i]);
    printf("\n");

    while (1) {
        uint32_t D1, D2;
        if (ms5611_read_adc(&dev, MS5611_CMD_CONV_D1_OSR4096, &D1) != MS5611_RET_OK) break;
        plat_delay_us(MS5611_CONV_TIME_OSR4096_US);
        if (ms5611_read_adc(&dev, MS5611_CMD_CONV_D2_OSR4096, &D2) != MS5611_RET_OK) break;
        plat_delay_us(MS5611_CONV_TIME_OSR4096_US);

        int32_t P, T;
        ms5611_calculate_pressure(&dev, D1, D2, &P, &T);
        printf("P = %ld Pa, T = %.2f°C\n", P, T / 100.0);
        sleep(1);
    }

    i2c_bus_cleanup(i2c);
    return 0;
}