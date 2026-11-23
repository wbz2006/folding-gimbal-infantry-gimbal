#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "BMI088driver.c"

SPI_HandleTypeDef *BMI088_SPI;
uint8_t BMI088_SPI_Read_OK = 1;
static double sim_time, byte_seconds;
static unsigned samples;
static int mode, selected, command_seen, read_op, address, byte_index;
static uint8_t registers[3][128];

void BMI088_ACCEL_NS_L(void) { selected = 1; command_seen = 0; byte_index = 0; }
void BMI088_ACCEL_NS_H(void) { selected = 0; }
void BMI088_GYRO_NS_L(void) { selected = 2; command_seen = 0; byte_index = 0; }
void BMI088_GYRO_NS_H(void) { selected = 0; }
float DWT_GetTimeline_s(void) { return (float)sim_time; }
void DWT_Delay(float seconds) { sim_time += seconds; }

static uint8_t word_byte(int word, int index)
{
    return ((uint16_t)word >> (8 * index)) & 255;
}

uint8_t BMI088_read_write_byte(uint8_t txdata)
{
    sim_time += byte_seconds;
    if (!command_seen)
    {
        command_seen = 1;
        read_op = txdata & 128;
        address = txdata & 127;
        if (selected == 1 && address == BMI088_ACCEL_XOUT_L && read_op)
            ++samples;
        return 0;
    }
    if (!read_op)
    {
        registers[selected][address] = txdata;
        return 0;
    }
    if (selected == 1 && byte_index++ == 0)
        return 0;
    int current = address++;
    if (selected == 1)
    {
        if (mode == 4 && current == BMI088_ACCEL_XOUT_L)
            BMI088_SPI_Read_OK = 0;
        if (current == BMI088_ACC_CHIP_ID)
            return mode == 6 || (mode == 3 && samples) ? 0 : BMI088_ACC_CHIP_ID_VALUE;
        if (mode == 8 && current == BMI088_ACC_RANGE)
            return 0;
        if (current >= BMI088_ACCEL_XOUT_L && current <= BMI088_ACCEL_ZOUT_M)
        {
            int i = current - BMI088_ACCEL_XOUT_L;
            return word_byte(i / 2 == 2 ? 5461 : 0, i % 2);
        }
    }
    else
    {
        if (current == BMI088_GYRO_CHIP_ID)
            return mode == 2 && samples && samples % 2 == 0 ? 0 : BMI088_GYRO_CHIP_ID_VALUE;
        if (current == BMI088_GYRO_BANDWIDTH && mode == 7)
            return 0;
        if (current >= BMI088_GYRO_X_L && current <= BMI088_GYRO_Z_H)
        {
            int i = current - BMI088_GYRO_X_L;
            int word = i / 2 == 2 ? 5 : (mode == 1 && i / 2 == 0 ? 11 : 0);
            if (mode == 9 && i / 2 == 0)
                word = samples % 2 == 0 ? 150 : -150;
            return word_byte(word, i % 2);
        }
    }
    return registers[selected][current];
}

int main(void)
{
    SPI_HandleTypeDef spi = {0};
    for (mode = 0; mode < 10; ++mode)
    {
        sim_time = 0;
        byte_seconds = mode == 5 ? 0.0001 : 0.000004;
        samples = 0;
        caliCount = 0;
        BMI088_SPI_Read_OK = 1;
        memset(registers, 0, sizeof(registers));
        memset(&BMI088, 0, sizeof(BMI088));
        uint8_t status = BMI088Init(&spi, 1);
        if (mode == 0)
        {
            assert(status == BMI088_NO_ERROR);
            assert(samples == 30000 && caliCount == 1);
            assert(fabsf(BMI088.GyroOffset[2] - 5 * BMI088_GYRO_SEN) < 2e-6f);
            assert(sim_time > 15 && sim_time < 30);
        }
        else if (mode == 1 || mode == 5 || mode == 9)
        {
            assert(status == BMI088_CALI_TIMEOUT);
            assert(sim_time >= 30 && sim_time < 30.3);
            assert(BMI088.GyroOffset[2] != GzOFFSET);
        }
        else if (mode == 7 || mode == 8)
        {
            assert(status == (mode == 7 ? BMI088_GYRO_BANDWIDTH_ERROR : BMI088_ACC_RANGE_ERROR));
            assert(samples == 0);
        }
        else
        {
            assert(status == BMI088_NO_SENSOR);
            assert(caliCount == 0);
            if (mode == 6)
                assert(samples == 0);
        }
    }
    puts("PASS IMU calibration: full window, deadline, motion, invalid IDs, SPI failure, register errors; no timeout fallback");
    return 0;
}
