/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define LIMIT_MIN_MAX(x,min,max) (x) = (((x)<=(min))?(min):(((x)>=(max))?(max):(x)))

int16_t motor_rpm1;
int16_t motor_rpm2;
int16_t motor_rpm3;
float motor_position;
int16_t motor_rpm5;
int16_t motor_rpm6;
int16_t x;
uint16_t y;
float motorid,current,velocity,position;

#define P_MIN -95.5f    // Radians
#define P_MAX 95.5f
#define V_MIN -45.0f    // Rad/s
#define V_MAX 45.0f
#define KP_MIN 0.0f     // N-m/rad
#define KP_MAX 500.0f
#define KD_MIN 0.0f     // N-m/rad/s
#define KD_MAX 5.0f
#define T_MIN -18.0f
#define T_MAX 18.0f

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);


/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static uint16_t float_to_uint(float x, float x_min, float x_max, uint8_t bits)
{
    float span = x_max - x_min;
    float offset = x_min;

    return (uint16_t) ((x-offset)*((float)((1<<bits)-1))/span);
}

static float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}

void setup_can(){
    CAN_FilterTypeDef can_filter_st = {0};
    can_filter_st.FilterActivation = ENABLE;
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter_st.FilterIdHigh = 0;
    can_filter_st.FilterIdLow = 0;
    can_filter_st.FilterMaskIdHigh = 0;
    can_filter_st.FilterMaskIdLow = 0;
    can_filter_st.FilterBank=0;
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO0_FULL| CAN_IT_RX_FIFO0_OVERRUN);
}



void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan){
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_buffer[8];
    // HAL_CAN_DeactivateNotification(hcan,
    //   CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO0_FULL| CAN_IT_RX_FIFO0_OVERRUN);
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_buffer);

    //M3508 motors
    //if (rx_header.StdId == 0x201)
    //{
    //	motor_rpm1 = (rx_buffer[2] << 8) + rx_buffer[3];
    //}

    //if (rx_header.StdId == 0x202)
    //{
    //	motor_rpm2 = (rx_buffer[2] << 8) + rx_buffer[3];
    //}

    // GM6020 Motors
    //if (rx_header.StdId == 0x205)
    //{
    //	motor_rpm3 = (rx_buffer[2] << 8) + rx_buffer[3];
    //}
    if (rx_buffer[0] == 0x01)
    {

      motorid  = rx_buffer[0];                                                          \
      position = uint_to_float(((rx_buffer[1] << 8 ) | (rx_buffer[2])),P_MIN,P_MAX,16);      \
      velocity = uint_to_float(((rx_buffer[3] << 4 ) | (rx_buffer[4]>>4)),V_MIN,V_MAX,12);   \
      current  = uint_to_float(((rx_buffer[4] << 4 ) | (rx_buffer[5])),T_MIN,T_MAX,12);  
    }
    //if (rx_header.StdId == 0x207)
    //{
    //	motor_rpm5 = (rx_buffer[2] << 8) + rx_buffer[3];
    //}
    //if (rx_header.StdId == 0x208)
    //{
    //	motor_rpm6 = (rx_buffer[2] << 8) + rx_buffer[3];
    //}

    // HAL_CAN_ActivateNotification(hcan,
    //   CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO0_FULL| CAN_IT_RX_FIFO0_OVERRUN);
}




void motor_send_can(float f_p, float f_v, float f_kp, float f_kd, float f_t){
    uint8_t p;
    uint8_t v;
    uint8_t kp;
    uint8_t kd;
    uint8_t t;
    uint8_t buf[8];

    LIMIT_MIN_MAX(f_p,  P_MIN,  P_MAX);
    LIMIT_MIN_MAX(f_v,  V_MIN,  V_MAX);
    LIMIT_MIN_MAX(f_kp, KP_MIN, KP_MAX);
    LIMIT_MIN_MAX(f_kd, KD_MIN, KD_MAX);
    LIMIT_MIN_MAX(f_t,  T_MIN,  T_MAX);

    /* ����Э�飬��float��������ת�� */
    p = float_to_uint(f_p,      P_MIN,  P_MAX,  16);
    v = float_to_uint(f_v,      V_MIN,  V_MAX,  12);
    kp = float_to_uint(f_kp,    KP_MIN, KP_MAX, 12);
    kd = float_to_uint(f_kd,    KD_MIN, KD_MAX, 12);
    t = float_to_uint(f_t,      T_MIN,  T_MAX,  12);

    /* ���ݴ���Э�飬������ת��ΪCAN���������ֶ� */
    buf[0] = p>>8;
    buf[1] = p&0xFF;
    buf[2] = v>>4;
    buf[3] = ((v&0xF)<<4)|(kp>>8);
    buf[4] = kp&0xFF;
    buf[5] = kd>>4;
    buf[6] = ((kd&0xF)<<4)|(t>>8);
    buf[7] = t&0xff;
    CAN_TxHeaderTypeDef CAN_tx_message;
    uint32_t send_mail_box;
    CAN_tx_message.IDE = CAN_ID_STD;
    CAN_tx_message.RTR = CAN_RTR_DATA;
    CAN_tx_message.DLC = 0x08;
    CAN_tx_message.StdId = 0x01;
    if (HAL_CAN_AddTxMessage(&hcan1, &CAN_tx_message, buf, &send_mail_box) ==HAL_OK){

    }
}

void motor_mode(void){
    uint8_t tx_msg[8];
    CAN_TxHeaderTypeDef CAN_tx_message;
    uint32_t send_mail_box;
    CAN_tx_message.IDE = CAN_ID_STD;
    CAN_tx_message.RTR = CAN_RTR_DATA;
    CAN_tx_message.DLC = 0x08;
    CAN_tx_message.StdId = 0x01;
    tx_msg[0] = 0xFF;
    tx_msg[1] = 0xFF;
    tx_msg[2] = 0xFF;
    tx_msg[3] = 0xFF;
    tx_msg[4] = 0xFF;
    tx_msg[5] = 0xFF;
    tx_msg[6] = 0xFF;
    tx_msg[7] = 0xFC;
    if (HAL_CAN_AddTxMessage(&hcan1, &CAN_tx_message, tx_msg, &send_mail_box)==HAL_OK){

    }
}

void zero_positon(void){
    uint8_t tx_msg[8];
    CAN_TxHeaderTypeDef CAN_tx_message;
    uint32_t send_mail_box;
    CAN_tx_message.IDE = CAN_ID_STD;
    CAN_tx_message.RTR = CAN_RTR_DATA;
    CAN_tx_message.DLC = 0x08;
    CAN_tx_message.StdId = 0x01;
    tx_msg[0] = 0xFF;
    tx_msg[1] = 0xFF;
    tx_msg[2] = 0xFF;
    tx_msg[3] = 0xFF;
    tx_msg[4] = 0xFF;
    tx_msg[5] = 0xFF;
    tx_msg[6] = 0xFF;
    tx_msg[7] = 0xFE;
    if (HAL_CAN_AddTxMessage(&hcan1, &CAN_tx_message, tx_msg, &send_mail_box)==HAL_OK){

    }
}


static void ZeroPosition(void)
{
    // motor_mode();
    // HAL_Delay(100);
    // motor_send_can(0,0,0,0,0);
    // HAL_Delay(100);
}


void MotorControl_Start(void)
{
    ZeroPosition();
    zero_positon();

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
  MX_CAN1_Init();
  /* USER CODE BEGIN 2 */
  setup_can();
  MotorControl_Start();

  //uint32_t last_button_time = HAL_GetTick();
  //int16_t spd = 1;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {


//	  ctrl_motor(pid_lol(SPEED,motor_rpm1) , pid_lol_2(SPEED,motor_rpm2));
//	  ctrl_1_GM_motor(pid_lol(SPEED,motor_rpm1));
//	  ctrl_GM_motor(pid_lol(SPEED,motor_rpm3), pid_lol_2(SPEED,motor_rpm4), pid_lol_3(SPEED,motor_rpm5), pid_lol_4(SPEED,motor_rpm6));
	  //ctrl_motor(pid_lol(SPEED,motor_rpm1) , pid_lol(SPEED,motor_rpm2));
	  //ctrl_motor(SPEED);
//	  ctrl_2_new_motor(10,0,0,0,0);
    //  motor_send_can(1,0,0,0,0);
	  HAL_Delay(1);


	  /*
	  if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == 0){
		  if (HAL_GetTick() - last_button_time > 500){
			  spd = (spd == 1) ? 0 : (spd== 0) ? -1 : (spd == -1) ? 1 : 0;
			  last_button_time = HAL_GetTick();
		  }
	  }
	  */


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
  RCC_OscInitStruct.PLL.PLLM = 6;
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

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 3;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_9TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_4TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */

  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */

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
