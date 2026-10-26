#include "BMI088Middleware.h"
#include "main.h"

SPI_HandleTypeDef *BMI088_SPI;
uint8_t BMI088_SPI_Read_OK;

void BMI088_ACCEL_NS_L(void)
{
    HAL_GPIO_WritePin(CS2_ACCEL_GPIO_Port, CS2_ACCEL_Pin, GPIO_PIN_RESET);
}
void BMI088_ACCEL_NS_H(void)
{
    HAL_GPIO_WritePin(CS2_ACCEL_GPIO_Port, CS2_ACCEL_Pin, GPIO_PIN_SET);
}

void BMI088_GYRO_NS_L(void)
{
    HAL_GPIO_WritePin(CS2_GYRO_GPIO_Port, CS2_GYRO_Pin, GPIO_PIN_RESET);
}
void BMI088_GYRO_NS_H(void)
{
    HAL_GPIO_WritePin(CS2_GYRO_GPIO_Port, CS2_GYRO_Pin, GPIO_PIN_SET);
}

uint8_t BMI088_read_write_byte(uint8_t txdata)
{
    uint8_t rx_data = 0;
    if (HAL_SPI_TransmitReceive(BMI088_SPI, &txdata, &rx_data, 1, 1) != HAL_OK)
        BMI088_SPI_Read_OK = 0;
    return rx_data;
}
