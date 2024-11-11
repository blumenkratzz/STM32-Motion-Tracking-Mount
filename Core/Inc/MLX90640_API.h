/*
 * Authors: Aleksandr Safronov, Quoc Trung Trun
 * File: MLX90640_API.h
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
#ifndef _MLX640_API_H_
#define _MLX640_API_H_
#ifdef __cplusplus
 extern "C" {
#endif
#include "main.h"

#define SCALEALPHA 0.000001f
    
typedef struct
    {
        int16_t kVdd;
        int16_t vdd25;
        float KvPTAT;
        float KtPTAT;
        uint16_t vPTAT25;
        float alphaPTAT;
        int16_t gainEE;
        float tgc;
        float cpKv;
        float cpKta;
        uint8_t resolutionEE;
        uint8_t calibrationModeEE;
        float KsTa;
        float ksTo[5];
        int16_t ct[5];
        uint16_t alpha[768];    
        uint8_t alphaScale;
        int16_t offset[768];    
        int8_t kta[768];
        uint8_t ktaScale;    
        int8_t kv[768];
        uint8_t kvScale;
        float cpAlpha[2];
        int16_t cpOffset[2];
        float ilChessC[3]; 
        uint16_t brokenPixels[5];
        uint16_t outlierPixels[5];  
    } paramsMLX90640;
    
int MLX90640_DumpEE(I2C_HandleTypeDef *hi2c,uint8_t slaveAddr, uint16_t *eeData);
int MLX90640_GetFrameData(I2C_HandleTypeDef *hi2c,uint8_t slaveAddr, uint16_t *frameData);
int MLX90640_ExtractParameters(uint16_t *eeData, paramsMLX90640 *mlx90640);
float MLX90640_GetVdd(uint16_t *frameData, const paramsMLX90640 *params);
float MLX90640_GetTa(uint16_t *frameData, const paramsMLX90640 *params);
void MLX90640_GetImage(uint16_t *frameData, const paramsMLX90640 *params, float *result);
void MLX90640_CalculateTo(uint16_t *frameData, const paramsMLX90640 *params, float emissivity, float tr, float *result);
int MLX90640_SetResolution(I2C_HandleTypeDef *hi2c,uint8_t slaveAddr, uint8_t resolution);
int MLX90640_GetCurResolution(I2C_HandleTypeDef *hi2c,uint8_t slaveAddr);
int MLX90640_SetRefreshRate(I2C_HandleTypeDef *hi2c,uint8_t slaveAddr, uint8_t refreshRate);
int MLX90640_GetRefreshRate(I2C_HandleTypeDef *hi2c,uint8_t slaveAddr);
int MLX90640_GetSubPageNumber(uint16_t *frameData);
int MLX90640_GetCurMode(I2C_HandleTypeDef *hi2c,uint8_t slaveAddr);
int MLX90640_SetInterleavedMode(I2C_HandleTypeDef *hi2c,uint8_t slaveAddr);
int MLX90640_SetChessMode(I2C_HandleTypeDef *hi2c,uint8_t slaveAddr);
void MLX90640_BadPixelsCorrection(uint16_t *pixels, float *to, int mode, paramsMLX90640 *params);
#ifdef __cplusplus
}
#endif

#endif
