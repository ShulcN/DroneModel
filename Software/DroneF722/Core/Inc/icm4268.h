#ifndef ICM42688_H
#define ICM42688_H

#include "stm32f7xx_hal.h"

#define ICM_REG_WHO_AM_I        0x75
#define ICM_REG_ACCEL_XOUT_H    0x1F
#define ICM_REG_GYRO_XOUT_H     0x25
#define ICM_REG_ACCEL_CONFIG0   0x50
//	#define ICM_REG_GYRO_CONFIG0    0x4F
#define ICM_REG_GYRO_CONFIG0   0x54
#define ICM_REG_PWR_MGMT0     0x4E
//#define ICM_REG_WHO_AM_I        0x75
//#define ICM_REG_ACCEL_XOUT_H    0x1F
//#define ICM_REG_GYRO_XOUT_H     0x25
////#define ICM_REG_ACCEL_CONFIG0   0x14
//#define ICM_REG_ACCEL_CONFIG0   0x50
////#define ICM_REG_GYRO_CONFIG0    0x4F
//#define ICM_REG_GYRO_CONFIG0	0x54
////#define ICM_REG_PWR_MGMT0     0x4E
//#define ICM_REG_PWR_MGMT0     0x1B

#define FILTER_ALPHA 0.2f  // Коэффициент для низкочастотного фильтра

// Внешние объекты HAL
extern SPI_HandleTypeDef hspi1;
extern GPIO_TypeDef*    ICM_CS_GPIO_Port;
extern uint16_t         ICM_CS_Pin;

// Прототипы
HAL_StatusTypeDef ICM_Init(void);
HAL_StatusTypeDef ICM_ReadAccel(float accel_data[3]);
HAL_StatusTypeDef ICM_ReadGyro (float gyro_data[3]);
HAL_StatusTypeDef ICM_ReadAccel_RAW(int16_t accel_data[3]);
HAL_StatusTypeDef ICM_ReadGyro_RAW(int16_t gyro_data[3]);
void ICM_CalibrateGyro(float bias[3]);
HAL_StatusTypeDef ICM_ReadRegs(uint8_t reg, uint8_t *pdata, uint16_t len) ;
//HAL_StatusTypeDef ICM_GetAngularVelocity(float gyro_rad_s[3]);
//void ICM_GetEulerAnglesDeg(float accel[3], float gyro[3], float dt, float angles_deg[3]);
//void ICM_GetEulerAnglesRad(float accel[3], float gyro[3], float dt, float angles_rad[3]);
HAL_StatusTypeDef ICM_CalculateEulerAngles(float euler_deg[3], float delta_time, const float gyro_bias[3]);
void ICM_ResetOrientation(void);
void ICM_GetEulerAnglesDeg(float angles[3]);
void ICM_GetEulerAngles(float angles[3]);
HAL_StatusTypeDef ICM_UpdateEulerAngles(void);

#endif // ICM42688_H
