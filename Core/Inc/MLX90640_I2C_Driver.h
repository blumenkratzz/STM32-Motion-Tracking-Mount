/*
 * Authors: Aleksandr Safronov, Quoc Trung Tran
 * File: MLX90640_I2C_Driver.h
 * Description:
 *   Header file for MLX90640 sensor interface functions.
 *   Modifications:
 *     - Adjusted function signatures and added argument passing to avoid hardcoding
 *       I2C instances for each device.
 *
 * License:
 *   Licensed under Apache 2.0. See LICENSE file for details.
 *   Copyright © 2017 Melexis N.V. (with modifications © 2024 Aleksandr Safronov, Quoc Trung Trun)
 */
/**
 * @copyright (C) 2017 Melexis N.V.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */
#ifndef _MLX90640_I2C_Driver_H_
#define _MLX90640_I2C_Driver_H_
#ifdef __cplusplus
 extern "C" {
#endif
#include <stdint.h>
#include "stm32f4xx_hal.h"  // Include the header that defines I2C_HandleTypeDef

void MLX90640_I2CInit(void);
int  MLX90640_I2CRead(I2C_HandleTypeDef *hi2c, uint8_t slaveAddr,uint16_t startAddress, uint16_t nMemAddressRead, uint16_t *data);
int  MLX90640_I2CWrite(I2C_HandleTypeDef *hi2c, uint8_t slaveAddr,uint16_t writeAddress, uint16_t data);
void MLX90640_I2CFreqSet(int freq);
#ifdef __cplusplus
}
#endif
#endif
    
 
 
