/*
 * as5600.c
 *
 *  Created on: Aug 10, 2025
 *      Author: MLag
 */

#include "as5600.h"



uint16_t getAngle(void) {
    uint8_t reg = AS5600_ANGLE_REG;
    uint8_t angle_data[2] = {0};
    uint16_t angle = 0;

    // Запрашиваем адрес регистра
    if (HAL_I2C_Master_Transmit(&hi2c1, AS5600_ADDR, &reg, 1, HAL_MAX_DELAY) != HAL_OK) {
        return 0xFFFF; // ошибка передачи
    }

    // Читаем 2 байта
    if (HAL_I2C_Master_Receive(&hi2c1, AS5600_ADDR, angle_data, 2, HAL_MAX_DELAY) != HAL_OK) {
        return 0xFFFF; // ошибка чтения
    }

    // Собираем угол (MSB first!)
    angle = ((uint16_t)angle_data[0] << 8) | angle_data[1];
    return angle;
}
#define CONFIG_REG_HIGH     0x07
#define CONFIG_REG_LOW      0x08

void AS5600_SetSlowFilter(void) {
    uint8_t config_reg_addr = CONFIG_REG_HIGH;
    uint8_t config_data[2];

    // Сначала считываем текущие значения CONFIG
    HAL_I2C_Master_Transmit(&hi2c1, AS5600_ADDR, &config_reg_addr, 1, HAL_MAX_DELAY);
    HAL_I2C_Master_Receive(&hi2c1, AS5600_ADDR, config_data, 2, HAL_MAX_DELAY);

    uint16_t config = (config_data[0] << 8) | config_data[1];

    // Обнуляем биты [3:2] (SLOW_FILTER), потом устанавливаем SLOW_FILTER = 2 (0b10)
    config &= ~(0b11 << 2);
    config |= (0b10 << 2);

    // Обновляем массив байтов
    config_data[0] = (config >> 8) & 0xFF;
    config_data[1] = config & 0xFF;

    // Пишем обратно в CONFIG-регистр
    HAL_I2C_Mem_Write(&hi2c1, AS5600_ADDR, CONFIG_REG_HIGH, I2C_MEMADD_SIZE_8BIT, config_data, 2, HAL_MAX_DELAY);
}
