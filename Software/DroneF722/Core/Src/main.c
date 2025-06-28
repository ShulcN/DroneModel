/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "usbd_cdc_if.h"
#include <icm4268.h>
#ifndef MATH
#include <math.h>
#define MATH
#endif
//#include "ICM42688.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// Определяем возможные состояния
typedef enum {
  DRONE_STATE_IDLE = 0,       // Бездействие
  DRONE_STATE_TEST_CYCLE,     // Тестовый цикл
  DRONE_STATE_EMERGENCY_STOP, // Аварийная остановка
  DRONE_STATE_LANDING,        // Приземление
  DRONE_STATE_LQR,
  // Будущие состояния автономного движения
  DRONE_STATE_MOVE_FORWARD,
  DRONE_STATE_HOVER,
  DRONE_TEST_DIRS,
  // и т.д.
} DroneState;
volatile DroneState currentState = DRONE_STATE_IDLE;
volatile DroneState requestedState = DRONE_STATE_IDLE;
volatile uint32_t stateStartTime = 0;
volatile uint32_t landingStartAltitude = 0;
volatile uint8_t landingInProgress = 0;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define RX_BUF_SIZE  16
static uint8_t rx_byte;
static char cmd_buf[RX_BUF_SIZE];
static uint8_t cmd_idx = 0;
static uint8_t mode = 0;
uint8_t rx_buf[5];
volatile uint8_t rx_idx = 0;
GPIO_TypeDef* ICM_CS_GPIO_Port = GPIOB;
uint16_t      ICM_CS_Pin       = GPIO_PIN_2;
float angles_rad[3], angles_deg[3], gyro_rad_s[3], angles_speed[3], angles_rad_prev[3];
int16_t accel_raw[3], gyro_raw[3];
float filtered_accel[3] = {0};
float filtered_gyro[3] = {0};
float dt = 0.01f;
float pos[3], prev_pos[3], pos_req[3], speed[3];
//float k1=10,k2=5,k3=10,k4=5,k5=10,k6=5,k7=31623,k8=251,k9=31623,k10=251,k11=0,k12=0;
float k1=10,k2=5,k3=10,k4=5,k5=10,k6=5,k7=100,k8=25,k9=100,k10=25,k11=0,k12=0;
int requested_speed = 1220;
float m = 0.1, Ix=0.0021, Iy=0.0021, Iz=0.0043;
float kf=0.0000158, k_1_2_l=3.33, kf_inv=63463.8;
// Преобразование LQR_DC в PWM и ограничение значений
uint16_t motorPWM[4];
float scale = 0.5f;    // Коэффициент масштабирования (подберите экспериментально)
const float min_pwm = 1140.0f; // Минимальный рабочий PWM
const float max_pwm = 1170.0f; // Максимально допустимый PWM
float bias[3];
uint32_t timer_lqr = 0;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
DMA_HandleTypeDef hdma_tim2_ch1;
DMA_HandleTypeDef hdma_tim2_ch2_ch4;
DMA_HandleTypeDef hdma_tim3_ch1_trig;
DMA_HandleTypeDef hdma_tim4_ch1;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void USB_CDC_RxHandler(uint8_t*, uint32_t);
int _write(int file, char *ptr, int len) {
    CDC_Transmit_FS((uint8_t*) ptr, len); return len;
}
void SetMotorOutputs(uint16_t ch1, uint16_t ch2, uint16_t ch3, uint16_t ch4)
{
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, ch1);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, ch2);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, ch3);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, ch4);
}
void ArmESCs(void)
{
    // Установить минимальный сигнал (обычно 1000 мкс) на все каналы
    SetMotorOutputs(1000, 1000, 1000, 1000);
    // Подождать, чтобы ESC успели проинициализироваться
    HAL_Delay(2000);  // Задержка 2 секунды
}
void testCycle(void){
	for (uint16_t dc = 1000; dc <= 1220; dc += 2) {
	  SetMotorOutputs(dc, dc, dc, dc);
	  HAL_Delay(80);
	}
	for (uint16_t dc = 1220; dc > 1000; dc -= 2) {
	  SetMotorOutputs(dc, dc, dc, dc);
	  HAL_Delay(80);
	}
	// Сбрасываем и ждём
	//SetMotorOutputs(1000, 1000, 1000, 1000);
}
void testMotorsDirs(void){
	SetMotorOutputs(1150, 1000, 1000, 1000);
	HAL_Delay(2500);
	SetMotorOutputs(1000, 1000, 1000, 1000);
	///////////
	SetMotorOutputs(1000, 1150, 1000, 1000);
	HAL_Delay(2500);
	SetMotorOutputs(1000, 1000, 1000, 1000);
	/////////
	SetMotorOutputs(1000, 1000, 1150, 1000);
	HAL_Delay(2500);
	SetMotorOutputs(1000, 1000, 1000, 1000);
	////////
	SetMotorOutputs(1000, 1000, 1000, 1150);
	HAL_Delay(2500);
	SetMotorOutputs(1000, 1000, 1000, 1000);
}
void stopMotors(void){
  //SetMotorOutputs(dc, dc, dc, dc);
	// Сбрасываем и ждём
	SetMotorOutputs(1000, 1000, 1000, 1000);
}
void performLanding(void) {
  static uint32_t landingTimer = 0;
  static uint16_t currentThrottle = 0;

  // При первом входе в режим приземления
  if (!landingInProgress) {
    landingInProgress = 1;
    landingTimer = HAL_GetTick();
    // Предполагаем, что текущая мощность моторов около 1200
    currentThrottle = 1200;
    printf("LANDING STARTED\r\n");
  }

  // Постепенно уменьшаем мощность моторов в течение 5 секунд
  uint32_t elapsedTime = HAL_GetTick() - landingTimer;
  if (elapsedTime < 5000) {
    // Линейное снижение от currentThrottle до 1000 за 5 секунд
    uint16_t newThrottle = 1000 + (uint16_t)((float)(currentThrottle - 1000) * (1.0f - (float)elapsedTime / 5000.0f));
    SetMotorOutputs(newThrottle, newThrottle, newThrottle, newThrottle);
  } else {
    // Приземление завершено
    SetMotorOutputs(1000, 1000, 1000, 1000);
    landingInProgress = 0;
    currentState = DRONE_STATE_IDLE;
    printf("LANDING COMPLETE\r\n");
  }
}

// Функция выполнения тестового цикла с возможностью прерывания
void performTestCycle(void) {
  static uint16_t currentDC = 1000;
  static uint8_t increasing = 1;
  static uint32_t lastUpdate = 0;

  // Проверяем, не поступило ли запроса на смену состояния
  if (requestedState != currentState) {
    // Прерываем текущее выполнение
    return;
  }

  uint32_t now = HAL_GetTick();
  // Обновляем сигнал с частотой примерно 12.5 Гц (80 мс)
  if (now - lastUpdate >= 80) {
    lastUpdate = now;

    if (increasing) {
      currentDC += 2;
      if (currentDC >= requested_speed)
        increasing = 0;
    } else {
      currentDC -= 2;
      if (currentDC <= 1020) {
        increasing = 1;
        // Цикл завершен, можно перейти в режим ожидания
        currentState = DRONE_STATE_IDLE;
        requestedState = DRONE_STATE_IDLE;
        SetMotorOutputs(1000, 1000, 1000, 1000);
        printf("TEST CYCLE COMPLETE\r\n");
        return;
      }
    }

    SetMotorOutputs(currentDC, currentDC, currentDC, currentDC);
  }
}


void performLQRStable(void) {
  float LQR_SPEED[4] = {0, 0, 0, 0};

  // Проверяем, не поступило ли запроса на смену состояния
  if (requestedState != currentState) {
    // Прерываем текущее выполнение
    return;
  }
  uint32_t now = HAL_GetTick();
  //if (timer_lqr<now){
  if (timer_lqr<now && now < timer_lqr + 60000){
  //if (timer_lqr<now){
	  float nu[6];
	  float g = 1.81f;
	  nu[0] = -k1*(pos[0]-pos_req[0])-k2*speed[0];
	  nu[1] = -k3*(pos[1]-pos_req[1])-k4*speed[1];
	  nu[2] = -k5*(pos[2]-pos_req[2])-k6*speed[2];

	  nu[5] = -k11*angles_rad[2]-k12*angles_speed[2];
	  //uint32_t now = HAL_GetTick();
	  float bar_theta = atan2f(nu[0], (nu[2] + g));
	  float bar_phi = atan2f(nu[1], (nu[2] + g));

	  nu[3] = -k7*(angles_rad[0]-bar_phi)-k8*angles_speed[0];
	  nu[4] = -k9*(angles_rad[1]-bar_theta)-k10*angles_speed[1];

	  float U[4];
	  if ((fabsf(cosf(bar_phi)) < 0.00001f) || (fabsf(cosf(bar_theta)) < 0.00001f)) {
	      printf("ERROR: Division by zero in U[0] calculation\r\n");
	      SetMotorOutputs(1100, 1100, 1100, 1100);
	      return;
	  }
	  U[0] = m*(nu[2]+g)/(cosf(bar_phi)*cosf(bar_theta));
	  U[1] = Ix*nu[3]-(Iy-Iz)*angles_speed[1]*angles_speed[2];
	  U[2] = Iy*nu[4]-(Ix-Iz)*angles_speed[0]*angles_speed[2];
	  U[3] = Iz*nu[5]-(Ix-Iy)*angles_speed[0]*angles_speed[1];
	  //printf("T: %f, t_phi: %f, t_theta: %f, t_psi: %f\r\n", U[0], U[1], U[2], U[3]);
	  float temps[4];
	  temps[0] = kf_inv*(U[0]-k_1_2_l*U[2])-U[3];
	  temps[1] = kf_inv*(U[0]-k_1_2_l*U[1])+U[3];
	  temps[2] = kf_inv*(U[0]+k_1_2_l*U[2])-U[3];
	  temps[3] = kf_inv*(U[0]+k_1_2_l*U[1])+U[3];
	  for (int i = 0; i < 4; i++) {
		if (temps[i]<0){
			printf("ERROR ZERO CROSSING IN SQRT %d = %f\r\n", i, temps[i]);
			SetMotorOutputs(1100, 1100, 1100, 1100);
			return;
		}
	  }
	  LQR_SPEED[0] = 0.5*sqrtf(temps[0]);
	  LQR_SPEED[1] = 0.5*sqrtf(temps[1]);
	  LQR_SPEED[2] = 0.5*sqrtf(temps[2]);
	  LQR_SPEED[3] = 0.5*sqrtf(temps[3]);
	  for (int i = 0; i < 4; i++) {
		  // Масштабирование и смещение
		  float pwm = LQR_SPEED[i] * scale + min_pwm;

		  // Ограничение диапазона
		  pwm = fmaxf(pwm, min_pwm);
		  pwm = fminf(pwm, max_pwm);

		  motorPWM[i] = (uint16_t)pwm;
	  }
	  printf("motors: %d, %d, %d, %d\r\n", motorPWM[0], motorPWM[1],motorPWM[2],motorPWM[3]);

	  // Установка PWM для моторов
	  SetMotorOutputs(motorPWM[3],motorPWM[2],motorPWM[0], motorPWM[1]);
  } else {
	  SetMotorOutputs(1100, 1100, 1100, 1100);
  }

}

// Функция обработки состояний - вызывается в основном цикле
void handleDroneState(void) {
  // Если есть запрос на изменение состояния
  if (requestedState != currentState) {
    // Выполняем необходимые действия при смене состояния
    switch (requestedState) {
      case DRONE_STATE_EMERGENCY_STOP:
        // Экстренная остановка всегда имеет приоритет
        SetMotorOutputs(1060, 1060, 1060, 1060);
        printf("EMERGENCY STOP ACTIVATED\r\n");
        break;

      case DRONE_STATE_LANDING:
        // Сброс для начала процесса приземления
        landingInProgress = 0;
        break;

      case DRONE_STATE_TEST_CYCLE:
        // Перезапуск тестового цикла
        printf("TEST CYCLE STARTED\r\n");
        break;

      case DRONE_STATE_LQR:
		  // Перезапуск тестового цикла
    	  timer_lqr = HAL_GetTick()+15000;
		  printf("STARTED LQR\r\n");
		  break;

//      case DRONE_TEST_DIRS:
//    	  printf("TEST DIRS STARTED\r\n");
//		  break;

      default:
        break;
    }

    // Обновляем текущее состояние
    currentState = requestedState;
    stateStartTime = HAL_GetTick();
  }

  // Выполняем действия в соответствии с текущим состоянием
  switch (currentState) {
    case DRONE_STATE_IDLE:
      // В режиме ожидания ничего не делаем с моторами
      break;

    case DRONE_STATE_TEST_CYCLE:
      performTestCycle();
      break;

    case DRONE_STATE_LANDING:
      performLanding();
      break;

    case DRONE_STATE_EMERGENCY_STOP:
      // Просто поддерживаем минимальную мощность
      SetMotorOutputs(1000, 1000, 1000, 1000);
      break;

    case DRONE_STATE_LQR:
		// Просто поддерживаем минимальную мощность
    	performLQRStable();
		break;

    default:
      break;
  }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USB_DEVICE_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_SPI1_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  ArmESCs();
  //TestSPI();
  if (ICM_Init() != HAL_OK) {
	  HAL_Delay(1000);
	  printf("ERROR INIT GYRO\r\n");
  }
  else {
	  ICM_CalibrateGyro(bias);
	  printf("Calibration complete: bias_x=%.2f, bias_y=%.2f, bias_z=%.2f deg/s\r\n",
			 bias[0], bias[1], bias[2]);
	  printf("HIHIHIHA\r\n");
  }
//  uint32_t t0 = HAL_GetTick();
//  ICM_InitOrientation(t0);

//  else {
//	  ICM_EnableDataReadyInt();
//  }

//  uint8_t st;
//  ICM_ReadRegs(0x56, &st, 1);
//  printf("INT_SOURCE0=0x%02X\n", st);
  //testCycle();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  //HAL_UART_Receive_IT(&huart3, &rx_buf, 1);
  //HAL_Delay(12000);
  //testMotorsDirs();
  printf("Begin main loop\r\n");
  HAL_Delay(300);
  HAL_UART_Receive_IT(&huart3, &rx_byte, 1);

//  int16_t raw[3];
//  for (int i = 0; i < 8; i++) {
//      ICM_ReadGyroRaw(raw);
//      printf("%2d: %6d %6d %6d\r\n", i, raw[0], raw[1], raw[2]);
//  }
//  int32_t sum = 0;
//  for (int i = 0; i < 100; i++) {
//	  //printf("%d\r\n", i);
//    ICM_ReadGyroRaw(raw);
//    sum += raw[0];
//    //sum2 += raw[0]*raw[0];
//    HAL_Delay(1);
//  }
//  float mean = sum/100.0f;
  //float var  = sum2/1000.0f - mean*mean;
  //printf("Mean=%f\r\n", mean);
//  int16_t gyro_raw[3];
//  //float accel[3];
//  uint8_t raw[6];
//  uint32_t last_time = HAL_GetTick();
  //ICM_CalibrateAccel(accel_bias);
  //ICM_CalibrateGyro(gyro_bias);
  // Применяем калибровку при чтении:
  //accel_data[i] = (raw_value - accel_bias[i]) / 8192.0f;
  //uint32_t prev_time = HAL_GetTick();
  uint32_t last_orientation_time = HAL_GetTick()-10;
  requestedState = DRONE_STATE_LQR;
  //pos_req[2] = 1;
  //float euler_deg[3];
  while (1)
  {
	  HAL_Delay(10);
	  // Обработка состояния дрона
	  handleDroneState();

	  // Чтение данных ориентации каждые N мс
	  uint32_t now = HAL_GetTick();
//	  if (now - last_orientation_time >= 1) { // 10 мс = 100 Гц


		  if (ICM_UpdateEulerAngles() == HAL_OK) {
			  memcpy(angles_rad, angles_rad_prev, 3);
			  ICM_GetEulerAngles(angles_rad);
//			  for (int i = 0; i<3; i++){
//				  angles_speed[i] = (angles_rad_prev[i] - angles_rad[i])/((now-last_orientation_time)*0.0001f);
//			  }

			  // Вывод ориентации с меньшей частотой, чтобы не перегружать UART
			  if ((now % 100) == 0) {  // каждые 100 мс = 10 Гц
				  printf("ORIENT: %.2f, %.2f, %.2f\r\n", angles_rad[0], angles_rad[1], angles_rad[2]);
			  }
		  } else {
			  printf("ERROR ANGLES\r\n");
		  }
		  last_orientation_time = now;
	//  }
//	  switch(mode){
//	  	  case 1:
//			  testCycle();
//			  mode = 0;
//			  break;
//	  	  case 2:
//	  		  stopMotors();
//	  		  mode = 0;
//	  		  break;
//	  }
//
////	  uint32_t now = HAL_GetTick();
////	  float dt = (now - last_time) * 0.001f;
////	  last_time = now;
////
////	  UpdateOrientation(dt);
//
//	  //uint32_t t = HAL_GetTick();
//	  if (ICM_UpdateEulerAngles() == HAL_OK) {
//		  ICM_GetEulerAngles(angles_rad);
//		  // euler_deg[0]=roll, [1]=pitch, [2]=yaw в градусах
//		  printf("ORIENT: %.2f, %.2f, %.2f\r\n", euler_rad[0], euler_rad[1], euler_rad[2]);
//	  } else {
//		  printf("ERROR\r\n");
//	  }

//	  if (ICM_UpdateOrientationDeg(t, euler_deg) == HAL_OK) {
//	      // euler_deg[0]=roll, [1]=pitch, [2]=yaw в градусах
//		  printf("ORIENT: %.2f, %.2f, %.2f\r\n", euler_deg[0], euler_deg[1], euler_deg[2]);
//	  }
//	  HAL_Delay(1);
//	  HAL_Delay(100);
//
//
//	  if (ICM_ReadGyro_RAW(gyro_raw) == HAL_OK) {
//		  printf("GYRO_RAW: %6d, %6d, %6d\r\n", gyro_raw[0], gyro_raw[1], gyro_raw[2]);
//	  } else {
//		  printf("GYRO READ ERROR\r\n");
//	  }
//
//	  if (ICM_ReadAccel_RAW(accel_raw) == HAL_OK) {
//	      printf("ACCEL: %6d, %6d, %6d\r\n", accel_raw[0], accel_raw[1], accel_raw[2]);
//	  } else {
//	      printf("ACCEL READ ERROR\r\n");
//	  }
//
//	  if (ICM_ReadOrientationDegrees(angles_deg) == HAL_OK) {
//		  printf("Orient: %.2f, %.2f, %.2f\r\n", angles_deg[0], angles_deg[1], angles_deg[2]);
//	  } else {
//		  printf("ACCEL READ ERROR\r\n");
//	  }


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 432;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 107;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 2082;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 2000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 107;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 2082;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 2000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 107;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 2082;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PB2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
uint8_t ParseCommand(const char* buf, uint32_t Len) {
    char cmd[64] = {0};
    if (Len >= sizeof(cmd)) Len = sizeof(cmd)-1;
    memcpy(cmd, buf, Len);

    // Удаляем терминаторы
    for (int i = 0; i < Len; i++) {
        if (cmd[i]=='\r' || cmd[i]=='\n' || cmd[i]==';') {
            cmd[i] = '\0';
            break;
        }
    }
    if (strncmp(cmd, "SP", 2) == 0) {
		float x, y, z;
		if (sscanf(cmd + 2, "%f,%f,%f", &x, &y, &z) == 3) {
			memcpy(prev_pos, pos, 3*sizeof( float ) );
			pos[0] = x;
			pos[1] = y;
			pos[2] = z;

			HAL_UART_Transmit(&huart3, (uint8_t*)"POS_SET\r\n", 9, HAL_MAX_DELAY);
		} else {
			HAL_UART_Transmit(&huart3, (uint8_t*)"POS_ERR\r\n", 10, HAL_MAX_DELAY);
		}
		return 0;
	}// ECHO
    if (strcmp(cmd, "STOP") == 0) {
		requestedState = DRONE_STATE_EMERGENCY_STOP;
		HAL_UART_Transmit(&huart3, (uint8_t*)"EMERGENCY STOP REQUESTED\r\n", 26, HAL_MAX_DELAY);
		return 0;
	}

	if (strcmp(cmd, "LAND") == 0) {
		requestedState = DRONE_STATE_LANDING;
		HAL_UART_Transmit(&huart3, (uint8_t*)"LANDING REQUESTED\r\n", 19, HAL_MAX_DELAY);
		return 0;
	}
    if (strcmp(cmd, "ECHO") == 0) {
        HAL_UART_Transmit(&huart3, (uint8_t*)"ECHO\r\n", 6, HAL_MAX_DELAY);
        return 0;
    }
    // ARM
    if (strcmp(cmd, "ARM") == 0) {
        ArmESCs();
        HAL_UART_Transmit(&huart3, (uint8_t*)"ARMED\r\n", 7, HAL_MAX_DELAY);
        return 0;
    }
    // DISARM
    if (strcmp(cmd, "DISARM") == 0) {
        HAL_UART_Transmit(&huart3, (uint8_t*)"DISARMED\r\n", 10, HAL_MAX_DELAY);
        return 0;
    }
    // TEST-M
    if (strcmp(cmd, "TEST-M") == 0) {
    	requested_speed=1220;
    	requestedState = DRONE_STATE_TEST_CYCLE;
        HAL_UART_Transmit(&huart3, (uint8_t*)"TEST CYCLE\r\n", 12, HAL_MAX_DELAY);
        return 0;
    }
    if (strncmp(cmd, "ST", 2) == 0) {
    	sscanf(cmd + 2, "%d", &requested_speed);
		requestedState = DRONE_STATE_TEST_CYCLE;
		char out[32];
		int n = snprintf(out, sizeof(out),"START in %d\r\n", requested_speed);
		HAL_UART_Transmit(&huart3, (uint8_t*)out, n, HAL_MAX_DELAY);
		return 0;
	}
    // CALB
    if (strcmp(cmd, "CALB") == 0) {
        ICM_CalibrateGyro(bias);
    	ICM_ResetOrientation();
        HAL_UART_Transmit(&huart3, (uint8_t*)"Reset Successfully\r\n", 20, HAL_MAX_DELAY);
        return 0;
    }
    // GET_ACC-POS
    if (strcmp(cmd, "GET_ACC-POS") == 0) {
    	ICM_GetEulerAnglesDeg(angles_deg);
        char out[128];
        int n = snprintf(out, sizeof(out),
            "ANG: %.2f,%.2f,%.2f; POS: %.2f,%.2f,%.2f\r\n",
			angles_deg[0], angles_deg[1], angles_deg[2], pos[0], pos[1], pos[2]);
        HAL_UART_Transmit(&huart3, (uint8_t*)out, n, HAL_MAX_DELAY);
        return 0;
    }
    if (strncmp(cmd, "LQR", 3) == 0){
    	requestedState = DRONE_STATE_LQR;
    	float x, y, z;
		if (sscanf(cmd + 3, "%f,%f,%f", &x, &y, &z) == 3) {
			memcpy(prev_pos, pos, 3*sizeof( float ) );
			pos_req[0] = x;
			pos_req[1] = y;
			pos_req[2] = z;

			//HAL_UART_Transmit(&huart3, (uint8_t*)"POS_SET\r\n", 9, HAL_MAX_DELAY);
		}
    	//sscanf(cmd + 3, "%f,%f,%f", &pos_req[0],&pos_req[1],pos_req[2]);
    	char out[64];
    	int n = snprintf(out, sizeof(out),"POS REQ: %.2f,%.2f,%.2f\r\n", pos_req[0], pos_req[1], pos_req[2]);
    	HAL_UART_Transmit(&huart3, (uint8_t*)out, n, HAL_MAX_DELAY);
		//HAL_Delay(4000);
		return 0;
    }
    // GET_GYRO
//    if (strcmp(cmd, "GET_GYRO") == 0) {
//        char out[64];
//        int n = snprintf(out, sizeof(out),
//            "GYRO: %.2f,%.2f,%.2f\r\n",
//            gyro[0], gyro[1], gyro[2]);
//        HAL_UART_Transmit(&huart3, (uint8_t*)out, n, HAL_MAX_DELAY);
//        return 0;
//    }
    if (strcmp(cmd, "GET_POS") == 0) {
		char out[128];
		int n = snprintf(out, sizeof(out),
			"POS: %.2f,%.2f,%.2f; PREV: %.2f,%.2f,%.2f\r\n",
			pos[0], pos[1], pos[2], prev_pos[0], prev_pos[1], prev_pos[2]);
		HAL_UART_Transmit(&huart3, (uint8_t*)out, n, HAL_MAX_DELAY);
		return 0;
	}
    // SET_COEFFS:k1,k2,...,k12
    if (strncmp(cmd, "SC", 2) == 0) {
    	char out[64];
    	int n = snprintf(out, sizeof(out), "%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f\r\n", k1,k2,k3,k4,k5,k6,k7,k8,k9,k10,k11,k12);
		HAL_UART_Transmit(&huart3, (uint8_t*)out, n, HAL_MAX_DELAY);
        return 0;
    }
    if (strncmp(cmd, "SSC", 3)==0){
    	char out[64];
    	float k=0.0;
    	if (sscanf(cmd+3, "%f", &k) == 1){
    		scale = k;
    	}
    	int l = snprintf(out, sizeof(out),"scale SET TO %.2f\r\n", scale);
		HAL_UART_Transmit(&huart3, (uint8_t*)out, l, HAL_MAX_DELAY);
    }

    if (strncmp(cmd, "SK", 2) == 0) {
    	int n =0;
    	float k=0.0;
		if (sscanf(cmd + 2, "%d,%f", &n, &k) == 2) {
			char out[64];
			switch (n){
			case 1:
				k1=k;
				break;
			case 2:
				k2=k;
				break;
			case 3:
				k3=k;
				break;
			case 4:
				k4=k;
				break;
			case 5:
				k5=k;
				break;
			case 6:
				k6=k;
				break;
			case 7:
				k7=k;
				break;
			case 8:
				k8=k;
				break;
			case 9:
				k9=k;
				break;
			case 10:
				k10=k;
				break;
			case 11:
				k11=k;
				break;
			case 12:
				k12=k;
				break;
			}
			int l = snprintf(out, sizeof(out),"K%d SET TO %.2f\r\n", n,k);
			HAL_UART_Transmit(&huart3, (uint8_t*)out, l, HAL_MAX_DELAY);
		} else {
			HAL_UART_Transmit(&huart3, (uint8_t*)buf, Len, HAL_MAX_DELAY);
		}
		return 0;
        }
    // Неизвестная команда
    HAL_UART_Transmit(&huart3, (uint8_t*)buf, Len, HAL_MAX_DELAY);
    return 1;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
//    if (GPIO_Pin == GPIO_PIN_4) {
//    	printf("LOL\r\n");
//        // Сюда пришло прерывание Data Ready
//        if (ICM_ReadAccel(accel) == HAL_OK && ICM_ReadGyro(gyro) == HAL_OK) {
//            // Обработать данные: сохранить, передать по UART, фильтровать и т.п.
//        	printf("RAW GYRO: %f, %f, %f\r\n", gyro[0], gyro[1], gyro[2]);
//        }
//        // Если INT_CONFIG без latch, очищать статус не нужно.
//        // Если latch=1, статус сбрасывается чтением INT_SOURCE:
//        // uint8_t status;
//        // ICM_ReadRegs(0x56, &status, 1);
//    }
}

uint8_t copy_rx;
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    cmd_buf[cmd_idx++] = rx_byte;
    // конец команды: ';' или '\n' или переполнение буфера
    if (rx_byte == ';' || rx_byte == '\n' || cmd_idx >= RX_BUF_SIZE-1) {
        cmd_buf[cmd_idx] = '\0';
        ParseCommand(cmd_buf, cmd_idx);
        cmd_idx = 0;
    }
    HAL_UART_Receive_IT(&huart3, &rx_byte, 1);
}

void USB_CDC_RxHandler(uint8_t* Buf, uint32_t Len)
{
	HAL_UART_Transmit(&huart3, (uint8_t*)Buf, Len, 10);
//	if (ParseCommand((const char*)Buf, Len) == 1){
//		//CDC_Transmit_FS(Buf, Len);
//		HAL_UART_Transmit(&huart3, (uint8_t*)Buf, Len, 10);
//	}
}
/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
