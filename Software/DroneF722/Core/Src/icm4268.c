#include <icm4268.h>
#ifndef MATH
#include <math.h>
#define MATH
#endif

// Глобальная переменная для хранения смещения гироскопа
static float gyro_bias[3] = {0.0f, 0.0f, 0.0f}; // смещение гироскопа в град/с
static int is_calibrated = 0; // флаг, показывающий выполнена ли калибровка

static HAL_StatusTypeDef ICM_WriteReg(uint8_t reg, uint8_t val) {
    uint8_t buf[2] = { reg & 0x7F, val };  // MSB=0 для записи
    HAL_GPIO_WritePin(ICM_CS_GPIO_Port, ICM_CS_Pin, GPIO_PIN_RESET);
    HAL_StatusTypeDef ret = HAL_SPI_Transmit(&hspi1, buf, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(ICM_CS_GPIO_Port, ICM_CS_Pin, GPIO_PIN_SET);
    return ret;
}


HAL_StatusTypeDef ICM_ReadRegs(uint8_t reg, uint8_t *pdata, uint16_t len) {
    uint8_t addr = reg | 0x80; // Устанавливаем бит чтения (MSB=1)
    HAL_GPIO_WritePin(ICM_CS_GPIO_Port, ICM_CS_Pin, GPIO_PIN_RESET);

    HAL_StatusTypeDef ret = HAL_SPI_Transmit(&hspi1, &addr, 1, HAL_MAX_DELAY);
    if (ret != HAL_OK) {
        HAL_GPIO_WritePin(ICM_CS_GPIO_Port, ICM_CS_Pin, GPIO_PIN_SET);
        return ret;
    }

    ret = HAL_SPI_Receive(&hspi1, pdata, len, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(ICM_CS_GPIO_Port, ICM_CS_Pin, GPIO_PIN_SET);
    return ret;
}

HAL_StatusTypeDef ICM_Init(void) {
    uint8_t who = 0;

    if (ICM_WriteReg(ICM_REG_PWR_MGMT0, 0x0F) != HAL_OK) {
        printf("Failed to write PWR_MGMT0\r\n");
        return HAL_ERROR;
    }
    HAL_Delay(50);

    if (ICM_ReadRegs(ICM_REG_WHO_AM_I, &who, 1) != HAL_OK) {
        printf("Failed to read WHO_AM_I\r\n");
        return HAL_ERROR;
    }

    if (who != 0x47) {
        printf("ICM WHO_AM_I = 0x%02X (expected 0x47)\r\n", who);
        return HAL_ERROR;
    }
//
//    if (ICM_WriteReg(ICM_REG_ACCEL_CONFIG0, 0x46) != HAL_OK) return HAL_ERROR;
//    if (ICM_WriteReg(ICM_REG_GYRO_CONFIG0,  0xE6) != HAL_OK) return HAL_ERROR;
	if (ICM_WriteReg(ICM_REG_ACCEL_CONFIG0, 0x05) != HAL_OK) return HAL_ERROR;
	if (ICM_WriteReg(ICM_REG_GYRO_CONFIG0,  0x07) != HAL_OK) return HAL_ERROR;

    HAL_Delay(50);

    uint8_t sts;
    if (ICM_ReadRegs(0x53, &sts, 1) == HAL_OK)
        printf("INT_STATUS=0x%02X\r\n", sts);

    // Сброс флага калибровки при инициализации
    is_calibrated = 0;
    for (int i = 0; i < 3; i++) {
        gyro_bias[i] = 0.0f;
    }

    return HAL_OK;
}



HAL_StatusTypeDef ICM_ReadAccel_RAW(int16_t accel_data[3]) {
    uint8_t buf[6];
    HAL_StatusTypeDef ret = ICM_ReadRegs(ICM_REG_ACCEL_XOUT_H, buf, 6);
    if (ret != HAL_OK) return ret;
    // преобразование в int16
    for (int i = 0; i < 3; i++) {
        accel_data[i] = ((int16_t)buf[2*i] << 8) | buf[2*i + 1];
    }
    return HAL_OK;
}

HAL_StatusTypeDef ICM_ReadGyro_RAW(int16_t gyro_data[3]) {
    uint8_t buf[6];
    HAL_StatusTypeDef ret = ICM_ReadRegs(ICM_REG_GYRO_XOUT_H, buf, 6);
    if (ret != HAL_OK) return ret;
    for (int i = 0; i < 3; i++) {
        gyro_data[i] = ((int16_t)buf[2*i] << 8) | buf[2*i + 1];
    }
    return HAL_OK;
}


// Константы для преобразования сырых данных в физические величины
#define RAD_TO_DEG 57.295779513082320876798154814105f  // 180/PI
#define DEG_TO_RAD 0.01745329251994329576923690768489f // PI/180

// Глобальные переменные для хранения углов Эйлера
float euler_angles[3] = {0.0f, 0.0f, 0.0f}; // крен (roll), тангаж (pitch), рыскание (yaw)
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};      // кватернион ориентации

// Константы для фильтра Madgwick
#define BETA 0.1f         // Параметр фильтра Madgwick
float deltat = 0.0f;      // время интегрирования для фильтра
uint32_t last_update = 0; // время последнего обновления

/**
 * Нормализация кватерниона
 */
void normalizeQuaternion(void) {
    float norm = sqrtf(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    if (norm > 0.0f) {
        norm = 1.0f / norm;
        q[0] *= norm;
        q[1] *= norm;
        q[2] *= norm;
        q[3] *= norm;
    }
}

/**
 * Преобразование кватерниона в углы Эйлера
 * euler_angles[0] = roll (крен) - вращение вокруг X
 * euler_angles[1] = pitch (тангаж) - вращение вокруг Y
 * euler_angles[2] = yaw (рыскание) - вращение вокруг Z
 */
void quaternionToEuler(void) {
    // Преобразование кватерниона в углы Эйлера
    // roll (x-axis rotation)
    float sinr_cosp = 2.0f * (q[0] * q[1] + q[2] * q[3]);
    float cosr_cosp = 1.0f - 2.0f * (q[1] * q[1] + q[2] * q[2]);
    euler_angles[0] = atan2f(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    float sinp = 2.0f * (q[0] * q[2] - q[3] * q[1]);
    if (fabsf(sinp) >= 1.0f)
        euler_angles[1] = copysignf(M_PI / 2.0f, sinp); // использовать 90 градусов, если sinp = ±1
    else
        euler_angles[1] = asinf(sinp);

    // yaw (z-axis rotation)
    float siny_cosp = 2.0f * (q[0] * q[3] + q[1] * q[2]);
    float cosy_cosp = 1.0f - 2.0f * (q[2] * q[2] + q[3] * q[3]);
    euler_angles[2] = atan2f(siny_cosp, cosy_cosp);

    // Преобразование из радиан в градусы
//    euler_angles[0] *= RAD_TO_DEG;
//    euler_angles[1] *= RAD_TO_DEG;
//    euler_angles[2] *= RAD_TO_DEG;
}

/**
 * Алгоритм фильтра Madgwick для объединения данных акселерометра и гироскопа
 * и обновления кватерниона ориентации
 * ax, ay, az - ускорения по осям X, Y, Z в g
 * gx, gy, gz - угловые скорости по осям X, Y, Z в рад/с
 */
void MadgwickAHRSupdate(float ax, float ay, float az, float gx, float gy, float gz) {
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2, _8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

    // Расчет дельты времени для интегрирования
    uint32_t now = HAL_GetTick();
    if (last_update == 0) {
        last_update = now;
        return;
    }
    //deltat = (now - last_update) / 1000.0f; // в секундах
    //deltat = (now - last_update) / 2160000.0f; // в секундах
    deltat = (now - last_update) / 1000.0f;
    last_update = now;

    if ((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)) {
        return; // Предотвращение деления на ноль
    }

    // Скорость изменения кватерниона из угловой скорости
    qDot1 = 0.5f * (-q[1] * gx - q[2] * gy - q[3] * gz);
    qDot2 = 0.5f * (q[0] * gx + q[2] * gz - q[3] * gy);
    qDot3 = 0.5f * (q[0] * gy - q[1] * gz + q[3] * gx);
    qDot4 = 0.5f * (q[0] * gz + q[1] * gy - q[2] * gx);

    // Вычисление градиента
    // Вспомогательные переменные для избежания повторных вычислений
    _2q0 = 2.0f * q[0];
    _2q1 = 2.0f * q[1];
    _2q2 = 2.0f * q[2];
    _2q3 = 2.0f * q[3];
    _4q0 = 4.0f * q[0];
    _4q1 = 4.0f * q[1];
    _4q2 = 4.0f * q[2];
    _8q1 = 8.0f * q[1];
    _8q2 = 8.0f * q[2];
    q0q0 = q[0] * q[0];
    q1q1 = q[1] * q[1];
    q2q2 = q[2] * q[2];
    q3q3 = q[3] * q[3];

    // Нормализация ускорения
    recipNorm = 1.0f / sqrtf(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // Градиент функции ошибки
    s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
    s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q[1] - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
    s2 = 4.0f * q0q0 * q[2] + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
    s3 = 4.0f * q1q1 * q[3] - _2q1 * ax + 4.0f * q2q2 * q[3] - _2q2 * ay;

    // Нормализация градиента
    recipNorm = 1.0f / sqrtf(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    // Применение обратной связи
    qDot1 -= BETA * s0;
    qDot2 -= BETA * s1;
    qDot3 -= BETA * s2;
    qDot4 -= BETA * s3;

    // Интегрирование для получения кватерниона
    q[0] += qDot1 * deltat;
    q[1] += qDot2 * deltat;
    q[2] += qDot3 * deltat;
    q[3] += qDot4 * deltat;

    // Нормализация кватерниона
    normalizeQuaternion();
}

/**
 * Чтение данных акселерометра в физических единицах (g)
 */
HAL_StatusTypeDef ICM_ReadAccel(float accel_data[3]) {
    int16_t raw[3];
    HAL_StatusTypeDef ret = ICM_ReadAccel_RAW(raw);
    if (ret != HAL_OK) return ret;

    // Масштабный коэффициент для ±16g (согласно документации ICM-42688-P)
    // При настройке ACCEL_CONFIG0 = 0x05 (диапазон ±16g)
    const float accel_scale = 1.0f / 2048.0f; // 16g / 32768 LSB

    for (int i = 0; i < 3; i++) {
        accel_data[i] = raw[i] * accel_scale;
    }

    return HAL_OK;
}

/**
 * Чтение данных гироскопа в физических единицах (радиан/с)
 * С учетом калибровки, если она была выполнена
 */
HAL_StatusTypeDef ICM_ReadGyro(float gyro_data[3]) {
    int16_t raw[3];
    HAL_StatusTypeDef ret = ICM_ReadGyro_RAW(raw);
    if (ret != HAL_OK) return ret;

    // Масштабный коэффициент для ±2000dps (градусов в секунду)
    // При настройке GYRO_CONFIG0 = 0x07 (диапазон ±2000dps)
    const float gyro_scale = 1.0f / 16.384f; // в градусах/с

    for (int i = 0; i < 3; i++) {
        // Преобразуем в град/с и вычитаем смещение (если калибровка выполнена)
        float gyro_deg_s = raw[i] * gyro_scale;
        if (is_calibrated) {
            gyro_deg_s -= gyro_bias[i]; // вычитаем смещение, рассчитанное при калибровке
        }
        // Преобразуем в рад/с
        gyro_data[i] = gyro_deg_s * DEG_TO_RAD;
    }

    return HAL_OK;
}

/**
 * Обновление углов Эйлера на основе последних данных датчика
 */
HAL_StatusTypeDef ICM_UpdateEulerAngles(void) {
    float accel[3];
    float gyro[3];

    // Чтение данных с датчика
    if (ICM_ReadAccel(accel) != HAL_OK) return HAL_ERROR;
    if (ICM_ReadGyro(gyro) != HAL_OK) return HAL_ERROR;

    // Обновление кватерниона с использованием фильтра Madgwick
    MadgwickAHRSupdate(accel[0], accel[1], accel[2], gyro[0], gyro[1], gyro[2]);

    // Преобразование кватерниона в углы Эйлера
    quaternionToEuler();

    return HAL_OK;
}

/**
 * Получение текущих углов Эйлера
 * angles[0] = roll (крен) - вращение вокруг X
 * angles[1] = pitch (тангаж) - вращение вокруг Y
 * angles[2] = yaw (рыскание) - вращение вокруг Z
 */
void ICM_GetEulerAngles(float angles[3]) {
    for (int i = 0; i < 3; i++) {
        angles[i] = euler_angles[i];
    }
}

void ICM_GetEulerAnglesDeg(float angles[3]) {
    for (int i = 0; i < 3; i++) {
        angles[i] = euler_angles[i]*RAD_TO_DEG;
    }
}


/**
 * Сброс ориентации до исходного состояния
 */
void ICM_ResetOrientation(void) {
    q[0] = 1.0f;
    q[1] = 0.0f;
    q[2] = 0.0f;
    q[3] = 0.0f;

    euler_angles[0] = 0.0f;
    euler_angles[1] = 0.0f;
    euler_angles[2] = 0.0f;

    last_update = 0;
}

/**
 * Калибровка гироскопа - измерение и сохранение смещения (bias)
 * bias[3] - массив для возврата значений смещения в град/с
 * Также устанавливает глобальный флаг is_calibrated = 1
 */
void ICM_CalibrateGyro(float bias[3]) {
    const int samples = 500;
    int32_t sum[3] = {0};

    // Сброс текущего смещения перед калибровкой
    for (int j = 0; j < 3; j++) {
        gyro_bias[j] = 0.0f;
    }
    is_calibrated = 0; // Временно отключаем учет калибровки

    int16_t raw[3];
    for (int i = 0; i < samples; i++) {
        ICM_ReadGyro_RAW(raw);
        for (int j = 0; j < 3; j++) {
            sum[j] += raw[j];
        }
        HAL_Delay(2);  // небольшая задержка между измерениями
    }

    for (int j = 0; j < 3; j++) {
        bias[j] = sum[j] / (float)samples / 16.384f; // перевод в °/s
        gyro_bias[j] = bias[j]; // Сохраняем значения для использования внутри библиотеки
    }

    is_calibrated = 1; // Устанавливаем флаг, что калибровка проведена

    printf("Gyro calibration completed: bias[0]=%f, bias[1]=%f, bias[2]=%f deg/s\r\n",
           gyro_bias[0], gyro_bias[1], gyro_bias[2]);
}

/**
 * Функция для получения текущих значений смещения гироскопа
 * bias[3] - массив для возврата значений смещения в град/с
 */
void ICM_GetGyroBias(float bias[3]) {
    for (int i = 0; i < 3; i++) {
        bias[i] = gyro_bias[i];
    }
}

/**
 * Функция для ручной установки смещения гироскопа
 * bias[3] - массив значений смещения в град/с
 */
void ICM_SetGyroBias(float bias[3]) {
    for (int i = 0; i < 3; i++) {
        gyro_bias[i] = bias[i];
    }
    is_calibrated = 1; // Устанавливаем флаг, что калибровка настроена
}

/**
 * Включение/отключение компенсации смещения гироскопа
 * enable: 1 - включить, 0 - отключить
 */
void ICM_EnableGyroCalibration(int enable) {
    is_calibrated = enable ? 1 : 0;
}
