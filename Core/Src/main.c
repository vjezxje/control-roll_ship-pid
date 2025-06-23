/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bno055.h"
#include "bno_config.h"
#include "math.h"
#include "esc.h"
#include "nrf24l01p.h"
#include "config_data.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TRANSMITTER 					//phát
//#define RECEIVER
#define mainland
//#define Underwater

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void CALCULATOR(float _dt);
float __low_pass_filter(float input, float previous_output, float alpha);
void read_value_BNO055();
void PID_calculator();
void BNO_init();
void calib_euler(bno055_euler_t* eul_caculate,  bno055_euler_t eul_raw, bno055_euler_t eul_offset );
void euler_get_value_offset(bno055_euler_t* eul_offset, bno055_euler_t eul_raw);
void calib_acc(bno055_vec3_t* acc_caculate,  bno055_vec3_t acc_raw, bno055_vec3_t acc_offset );
void acc_get_value_offset(bno055_vec3_t* acc_offset, bno055_vec3_t acc_raw);
void calib_gyr(bno055_vec3_t* gyr_caculate,  bno055_vec3_t gyr_raw, bno055_vec3_t gyr_offset );
void gyr_get_value_offset(bno055_vec3_t* gyr_offset, bno055_vec3_t gyr_raw);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
bno055_t sensor;
error_bno err;

bno055_euler_t eul_raw 			= { 0, 0, 0 };
bno055_euler_t eul_caculate 	= { 0, 0, 0 };
bno055_euler_t eul_data_offset 	= { 0, 0, 0 };

bno055_vec3_t acc_raw 			= { 0, 0, 0 };
bno055_vec3_t acc_caculate 		= { 0, 0, 0 };
bno055_vec3_t acc_data_offset 	= { 0, 0, 0 };

bno055_vec3_t gyr_raw 			= { 0, 0, 0 };
bno055_vec3_t gyr_caculate 		= { 0, 0, 0 };
bno055_vec3_t gyr_data_offset 	= { 0, 0, 0 };

bno055_vec3_t gyr 			= { 0, 0, 0 };

float acc_x_filter  			= 0.0f;
float gyr_y_deg  				= 0.0f,  	gyr_y_filter  		= 0.0f;
float eul_roll_filter 			= 0.0f, 	eul_pitch_filter 	= 0.0f;
float eul_roll_deg 				= 0.0f, 	eul_pitch_deg 		= 0.0f;
float offset_roll 				= 0.0f, 	offset_pitch 		= 0.0f;
float offset_acc 				= 0.0f;

float Wave_BNO = 0.0f, Wave_flash = 0.0f;

float  dt;
double Wave_pulse_omega = 0, 	Wave_pulse_angle 	= 0;
double pulse_omega 		= 0, 	err_pulse_omega 	= 0, 	pulse_angle 	= 0, err_pulse_angle 	= 0;
int16_t encoder_omega 	= 0, 	encoder_pre_omega 	= 0, 	encoder_angle 	= 0, encoder_pre_angle 	= 0;
float omega_raw 		= 0.0f, omega_raw_pre 		= 0.0f, omega_filter	= 0.0f;
float angle 			= 0.0f, omega 				= 0.0f;
float angle_pendulum 	= 0.0f, angle_pe_rad 		= 0.0f;

float F_spinner = 14.121576f;				// F = mg = 1.44 * 9.8066

float Output_PID_omega = 0, Output_PID_angle = 0,  Output_keep= 0, Output_PID = 0;
float  SP_omega 	= 0.0f, SP_angle = 0.0f, SP_angle_save 	= 0.0f;
float  E_angle 		= 0.0f,	E_omega  = 0.0f;

float  Kp_angle 	= 10.0f, 	Kp_omega = 2.0f;

float a_wave 	= 0.0f, 		 a_Rg 	= 0.0f, 	K1 		= 0.0f,	 K2 	 = 0.0f;
float Ls 		= 0.2272125f; 			// Ls = Is * omega_s
float Is 		= 0.0006256619f, dtL	= 0.0f, 	omega_G = 0.0f,  omega_G_Degrees = 0.0f 	;  //[kg/m2]
float Ixx 	 	= 0.0012513238f, W 		= 10.0f, 	Fcut 	= 0.0f;
float GM 		= 0.00514652956f;

#ifdef mainland
float K_omega_G			= 0.24f;
float K_aRg 			= 1.0f;
float CutOffFrequency 	= 0.75f;
float beta 				= 30.0f;
#endif

#ifdef Underwater
float K_omega_G		= 0.335f;
float K_aRg 	= 1.0f;
float CutOffFrequency = 2.5f, 	beta = 15.0f;
#endif

uint8_t	tx_data[32];
data_t data_out;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

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
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM8_Init();
  MX_TIM5_Init();
  MX_SPI1_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */
  	esc_init();												//khởi tạo esc
	HAL_Delay(100);

	calib_esc();											//calib esc
	HAL_Delay(1000);

	BNO_init();												// kết nối cảm biến
	HAL_Delay(100);

	sensor.euler(&sensor, &eul_raw);						// lấy giá trị góc euler
	euler_get_value_offset(&eul_data_offset, eul_raw);		// offset euler

	sensor.acc(&sensor, &acc_raw);							// lấy giá trị gia tốc
	acc_get_value_offset(&acc_data_offset,  acc_raw);		// offset acc

	sensor.gyro(&sensor, &gyr_raw);							// lấy giá trị gia tốc
	gyr_get_value_offset(&gyr_data_offset,  gyr_raw);		// offset acc

	HAL_TIM_Encoder_Start_IT(&htim3, TIM_CHANNEL_1); 		// khởi tạo encoder timer3 kenh 1
	HAL_TIM_Encoder_Start_IT(&htim3, TIM_CHANNEL_2); 		// khoi tao encoder timer3 kenh 2

	HAL_TIM_Encoder_Start_IT(&htim8, TIM_CHANNEL_1); 		// khởi tạo encoder timer8 kenh 1
	HAL_TIM_Encoder_Start_IT(&htim8, TIM_CHANNEL_2); 		// khoi tao encoder timer8 kenh 2

	HAL_TIM_PWM_Start_IT(&htim4, TIM_CHANNEL_4);			// khởi tạo pwm timer4 kênh 4

	HAL_TIM_Base_Start_IT(&htim2);						  	// khởi tạo timer2 cho viec ngat timer

	Fcut = 1 - exp(-0.01*2.0f*M_PI*CutOffFrequency);

#ifdef RECEIVER
	nrf24l01p_rx_init(2500, _1Mbps);
#endif
#ifdef TRANSMITTER
	nrf24l01p_tx_init(2500, _1Mbps);		// khởi tạo RF
#endif
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == 1)		 		//kiểm tra công tắc esc
	  {
		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);		// đèn 15 sáng
		  __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, 1200);		// xuất duty esc 1200
	  }
	  else
	  {
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);		// đèn 15 tắt
		__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, 0);			// xuất duty esc 0
	  }

	#ifdef RECEIVER
	  // Nothing to do
	#endif

	#ifdef TRANSMITTER								// gán dữ liệu gửi
	 data_out.eul_roll_deg		= eul_roll_deg;
	 data_out.eul_pitch_deg 	= eul_pitch_deg;
	 data_out.acc_x_filter 		= acc_x_filter;
	 data_out.omega_filter		= omega_filter;
	 data_out.angle 			= angle;
	 data_out.angle_pendulum 	= angle_pendulum;
	 data_out.SP_angle 			= SP_angle;
	 data_out.gyr_y_deg 		= gyr_y_deg;
	 _pack_data(tx_data, data_out);					// đóng gói dữ liệu gửi
	 nrf24l01p_tx_transmit(tx_data);				// gửi dữ liệu
	 HAL_Delay(50);
	#endif

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
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}
/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)	// hàm ngắt timer 2
{
	if (htim->Instance == TIM2) // 1[ms]		// kiểm tra vào đúng ngắt không
	{
		Wave_BNO++;								// cộng bộ đếm tràn 10ms

		encoder_omega = __HAL_TIM_GET_COUNTER(&htim3);  // lay gia tri encoder o time3
		__HAL_TIM_SET_COUNTER(&htim3, 0);				// reset htim3
		encoder_angle = __HAL_TIM_GET_COUNTER(&htim8);  // lay gia tri encoder o time8
		dt = 0.001f;									// thời gian lấy mẫu dt
		//chong tran xung htim3 (OMEGA)
		err_pulse_omega = encoder_omega - encoder_pre_omega;		// sai số xung omega
		if(err_pulse_omega < -32000)
		{
			Wave_pulse_omega = Wave_pulse_omega +1;					// số lần tràn +1
		}
		if(err_pulse_omega > 32000)
		{
			Wave_pulse_omega = Wave_pulse_omega -1;					// số lần tràn - 1
		}
		pulse_omega = encoder_omega + (Wave_pulse_omega * 65535.0f ); // xung omega
		encoder_pre_omega = encoder_omega;							// xung hiện tại = xung trước đó
		Wave_pulse_omega = 0;										// reset biến đếm tràn

		//chong tran xung htim8 (ANGLE)
		err_pulse_angle = encoder_angle - encoder_pre_angle;		// sai số xung angle
		if (err_pulse_angle < -32000)
		{
			Wave_pulse_angle = Wave_pulse_angle +1;					// số lần tràn +1
		}
		else if(err_pulse_angle > 32000)
		{
			Wave_pulse_angle = Wave_pulse_angle -1;					// số lần tràn -1
		}
		pulse_angle = encoder_angle + (Wave_pulse_angle * 65535.0f); // xung angle
		encoder_pre_angle = encoder_angle;							// xung hiện tại = xung trước đó
		Wave_pulse_angle = 0;										// reset biến đếm tràn

		//Cal ANGlE & OMEGA
		angle     		= pulse_angle * 360.0f / 2400.0f;			    // tính vị trí động cơ
		angle_pendulum  = angle / 4;									// từ góc dco qua góc con lắc

		omega_raw 		= (pulse_omega * 60.0f) / (dt*2400.0f);   		// tính tốc độ động cơ (thô)
		omega_filter 	= __low_pass_filter(omega_raw, omega_filter , 0.4f );  // lọc omega

		// PID controller
		PID_calculator();							//tính toán giá trị đầu vào và đưa ra giá trị pid
		MotorDriver(Output_PID);					// hàm điều khiển động cơ servo

		if (Wave_BNO > 9)    // 10 [ms]
		{
			Wave_BNO = 0;							// reset biến đếm tràn
			sensor.euler(&sensor, &eul_raw);  		// read value eul for sensor
			sensor.acc  (&sensor, &acc_raw);		// read value acc for sensor
			sensor.gyro (&sensor, &gyr_raw);		// read value gyr for sensor

			eul_roll_filter  = eul_roll_filter + (eul_caculate.roll - eul_roll_filter) * Fcut;  	// lọc LPF
			eul_roll_deg     = eul_roll_filter * (180 / M_PI);		// rad -> deg

			eul_pitch_filter = eul_pitch_filter + (eul_caculate.pitch - eul_pitch_filter) * Fcut;	// lọc LPF
			eul_pitch_deg    = eul_pitch_filter * (180 / M_PI);		// rad -> deg

			gyr_y_filter	 = gyr_y_filter + (gyr_caculate.y - gyr_y_filter) * Fcut;				// lọc LPF
			gyr_y_deg  	  	 =  -(gyr_y_filter * (180 / M_PI));		// rad/s -> deg/s

			acc_x_filter	 = acc_x_filter + (acc_caculate.x - acc_x_filter) * Fcut;				// lọc LPF

			// out 			 = out 			+ (		in		  -		out     ) * Fcut

			calib_euler(&eul_caculate,  eul_raw,  eul_data_offset);		// offset giá trị euler
			calib_acc  (&acc_caculate,  acc_raw,  acc_data_offset);		// offset giá trị acc
			calib_gyr  (&gyr_caculate,  gyr_raw,  gyr_data_offset);		// offset giá trị gyr

			if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_12) == 1)
			{
				euler_get_value_offset(&eul_data_offset, eul_raw);  	//lấy giá trị offset
				acc_get_value_offset  (&acc_data_offset, acc_raw);		//lấy giá trị offset
				gyr_get_value_offset  (&gyr_data_offset, gyr_raw);		//lấy giá trị offset
			}

			CALCULATOR(0.01f);										//hàm con tính toán SP omega và angle
		}
	}
}
void CALCULATOR(float _dt)
{
	//Ixx = Is * K_Ixx;												// hệ số Ixx

	a_Rg =(W*GM*sin(eul_roll_filter)*(1+sin(eul_pitch_filter))/Ixx) * K_aRg;	// gia tốc hồi phục[rad/s^2]
	a_wave = acc_x_filter/GM - a_Rg; 								// gia tốc của sóng tác động lên tàu
	dtL = Ixx * a_wave * _dt ; 											// dt = 0.01

	K1 = ((2.0f * Ls * Ls) - (dtL * dtL)) / (2.0f * Ls * Ls);
	if (a_wave >= 0)													// xét dấu a_wave
	{
		omega_G = -((acos(K1) / _dt) * 4.0f) * K_omega_G;				// omega_G [rad/s]
	}
	else
	{
		omega_G =  ((acos(K1) / _dt) * 4.0f) * K_omega_G;				// omega_G [rad/s]
	}
	omega_G_Degrees = omega_G * (180.0f / M_PI);						// omega_G [rad/s] -> [deg/s]

	SP_omega = omega_G_Degrees;											// cài setpoint vận tốc góc
	SP_angle = SP_angle + omega_G_Degrees / 100.0f;						// cài setpoint vị trí
	//hoi tu voi van toc beta
	if(SP_angle >= 0)
	{
		SP_angle = SP_angle - beta / 100.0f;
	}
	else
	{
		SP_angle = SP_angle + beta / 100.0f;
	}
	// giới hạn góc
	if(SP_angle > 60)
	{
		SP_angle = 60;
	}
	if(SP_angle < -60)
	{
		SP_angle = -60;
	}
	// giới hạn omega
	if(SP_omega > 600)
	{
		SP_omega = 600;
	}
	if(SP_omega < -600)
	{
		SP_omega = -600;
	}
}
void PID_calculator()							// tính toán PID
{
	E_angle = SP_angle - angle_pendulum;		//tính sai số angle con lắc
	E_omega = SP_omega - omega_filter;			//tính sai số omega

	Output_PID_angle = Kp_angle * E_angle;		// out = Kp.E
	Output_PID_omega = Kp_omega * E_omega;
	Output_keep = (F_spinner * sin(SP_angle*(M_PI / 180.0f))) / 0.03;		// PID giữ tương ứng với góc SP con lắc

	Output_PID = Output_PID_omega + Output_PID_angle  + Output_keep;		// tổng PID
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)   // ngắt nhận RF
{
	if(GPIO_Pin == NRF24L01P_IRQ_PIN_NUMBER)
	{
		#ifdef RECEIVER
			//nothing to do
		#endif

		#ifdef TRANSMITTER
				nrf24l01p_tx_irq();
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);
		#endif
	}
}
void calib_acc(bno055_vec3_t* acc_caculate,  bno055_vec3_t acc_raw, bno055_vec3_t acc_offset )
{
	acc_caculate->x = acc_raw.x - acc_offset.x;
}
void acc_get_value_offset(bno055_vec3_t* acc_offset, bno055_vec3_t acc_raw)
{
	acc_offset->x = acc_raw.x;
}
void calib_gyr(bno055_vec3_t* gyr_caculate,  bno055_vec3_t gyr_raw, bno055_vec3_t gyr_offset )
{
	gyr_caculate->y = gyr_raw.y - gyr_offset.y;
}
void gyr_get_value_offset(bno055_vec3_t* gyr_offset, bno055_vec3_t gyr_raw)
{
	gyr_offset->y = gyr_raw.y;
}
void calib_euler(bno055_euler_t* eul_caculate,  bno055_euler_t eul_raw, bno055_euler_t eul_offset)
{
	eul_caculate->roll = eul_raw.roll - eul_offset.roll;
	eul_caculate->pitch = eul_raw.pitch - eul_offset.pitch;
}
void euler_get_value_offset(bno055_euler_t* eul_offset, bno055_euler_t eul_raw)
{
	eul_offset->roll = eul_raw.roll;
	eul_offset->pitch = eul_raw.pitch;
}
void BNO_init()
{

	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
	sensor.i2c = &hi2c2;   									 // khai báo kết nối i2c
	sensor.addr = 0x28;	 									 // kết nối i2c ở địa chỉ 0x28
	sensor.mode = BNO_MODE_NDOF_FMC_OFF;					 // chế độ hoạt động của cảm biến
	sensor._temp_unit = BNO_TEMP_UNIT_C;
	HAL_Delay(1000);
	if ((err = bno055_init(&sensor)) == BNO_OK) 				 //kiểm tra kết nối cảm biến
	{
		 HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);	// khi kết nối được cảm biến - sáng đèn 12.
	}
	err = bno055_set_unit(&sensor, BNO_TEMP_UNIT_C, BNO_GYR_UNIT_RPS, BNO_ACC_UNITSEL_M_S2, BNO_EUL_UNIT_RAD);
}
float __low_pass_filter(float input, float previous_output, float alpha)
{
	return alpha * input + (1 - alpha) * previous_output;
}

/* USER CODE END 4 */

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
