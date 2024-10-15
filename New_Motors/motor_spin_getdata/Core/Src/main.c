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
int16_t motor_rpm1;
int16_t motor_rpm2;
int16_t motor_rpm3;
int16_t motor_rpm4;
int16_t motor_rpm5;
int16_t motor_rpm6;
uint16_t position;
uint16_t motorid;
uint16_t velocity;
uint16_t current;
uint16_t x;
float CurVelocity;
uint16_t motor_position;
uint16_t id;
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
#define CMD_MOTOR_MODE      0x05
#define CMD_RESET_MODE      0x06
#define CMD_ZERO_POSITION   0x07
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



extern CAN_HandleTypeDef hcan1;

static void _CanFilter(void)
{
    CAN_FilterTypeDef   sCAN_Filter;
    
    sCAN_Filter.FilterBank = 0;                         /* ָ��������ʼ���Ĺ����� */  
    sCAN_Filter.FilterMode = CAN_FILTERMODE_IDMASK;     /* ����ģʽΪ����λģʽ */
    sCAN_Filter.FilterScale = CAN_FILTERSCALE_16BIT;    /* ָ���˲����Ĺ�ģ */
    sCAN_Filter.FilterIdHigh = 00;
    sCAN_Filter.FilterIdLow = 00;             
    sCAN_Filter.FilterMaskIdHigh = 00;
    sCAN_Filter.FilterMaskIdLow = 00;
    sCAN_Filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    sCAN_Filter.FilterActivation = ENABLE;              /* ���û���ù����� */
    sCAN_Filter.SlaveStartFilterBank = 0;               /* ѡ�������ӹ������� */
    
    HAL_CAN_ConfigFilter(&hcan1, &sCAN_Filter);
}

/**
  * @brief  CAN�ӿڳ�ʼ��
  * @param
  * @retval 
  */
void CanComm_Init(void)
{
    _CanFilter();
    HAL_CAN_Start(&hcan1);               /* ����CANͨ�� */  
    HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING);    /* ���������ж����� */
}


#define LIMIT_MIN_MAX(x,min,max) (x) = (((x)<=(min))?(min):(((x)>=(max))?(max):(x)))

/**
  * @brief  Converts a float to an unsigned int, given range and number of bits
  * @param
  * @retval 
  */
static uint16_t float_to_uint(float x, float x_min, float x_max, uint8_t bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    
    return (uint16_t) ((x-offset)*((float)((1<<bits)-1))/span);
}

/**
  * @brief  converts unsigned int to float, given range and number of bits
  * @param
  * @retval 
  */
static float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}

/* get joint motor data */
#define get_joint_motor_measure(data)                                                  \
    {                                                                                       \
				motorid  = data[0];                                                          \
        position = uint_to_float(((data[1] << 8 ) | (data[2])),P_MIN,P_MAX,16);      \
        velocity = uint_to_float(((data[3] << 4 ) | (data[4]>>4)),V_MIN,V_MAX,12);   \
        current  = uint_to_float(((data[4] << 4 ) | (data[5])),T_MIN,T_MAX,12);      \
    }


/*
		joint_motor data,    0:joint motor1 HT-03;  1:joint motor2 HT-03;  2:joint motor3 HT-03;  3:joint motor4 HT-03.
		chassis_motor data,  0:chassis motor1 3508; 1:chassis motor2 3508.


static CAN_TxHeaderTypeDef  chassis_tx_message;
static uint8_t              chassis_can_send_data[8];

/**
  * @brief          hal CAN fifo call back, receive motor data??Unpack the data and store it in the corresponding array
  * @param[in]      hcan, the point to CAN handle
  * @retval         none
  */
/**
  * @brief          hal??CAN???????,??????????,??????????????????????
  * @param[in]      hcan:CAN??????
  * @retval         none
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    uint16_t tmp_value;

    CAN_RxHeaderTypeDef RxHead; 
    uint8_t Rxdata[8];
    HAL_CAN_DeactivateNotification(hcan,CAN_IT_RX_FIFO0_MSG_PENDING|CAN_IT_RX_FIFO0_FULL|CAN_IT_RX_FIFO0_OVERRUN);
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHead, Rxdata);
    x=3;
	
		
	if(RxHead.StdId == 0x205)
	{
		/*?????????????????ID?????????HT03???*/
		tmp_value  = Rxdata[0];
		id = tmp_value;
		tmp_value = (Rxdata[3]<<4)|(Rxdata[4]>>4);
		CurVelocity = uint_to_float(tmp_value, V_MIN, V_MAX, 12);
		get_joint_motor_measure(Rxdata);
		x=5;



     }
	HAL_CAN_ActivateNotification(hcan,CAN_IT_RX_FIFO0_MSG_PENDING|CAN_IT_RX_FIFO0_FULL|CAN_IT_RX_FIFO0_OVERRUN);

}


float CanComm_GetCurVelocity(void)
{
    return CurVelocity;
}


/**
  * @brief          send parameter data from CAN1 to control joint motor
  * @param[in]      buf: 8 bytes data, including motor control parameter information
  * @param[in]      len: size of buf
  * @param[in]      motor_id: id of joint motor,0x01~0x04
  * @retval         none
  */
/**
  * @brief          ???CAN1????????????????????
  * @param[in]      buf: 8????????????????????????????
  * @param[in]      len: buf?????
  * @param[in]      motor_id: ?????ID????0x01??0x04
  * @retval         none
  */
static void CanTransmit(uint8_t *buf, uint8_t len,uint32_t motor_id)
{
    CAN_TxHeaderTypeDef TxHead;             /**!< can??????Э??? */
    uint32_t canTxMailbox;
    //x=4;
    
    if((buf != NULL) && (len != 0))
    {
			TxHead.StdId    = 0x00;         /* ??????????????????0x01-0x04 */
			TxHead.IDE      = CAN_ID_STD;       /* ??????????????????????? */
			TxHead.RTR      = CAN_RTR_DATA;     /* ??????????????? */
			TxHead.DLC      = len;              /* ???????????????? */
		
			if(HAL_CAN_AddTxMessage(&hcan1, &TxHead, buf, (uint32_t *)&canTxMailbox) == HAL_OK )
			{
			}
    }
}

/**
  * @brief          send control parameters of joint motor (0x01, 0x02, 0x03, 0x04)
  * @param[in]      f_p: position command, range [-95.5,95.5] rad
  * @param[in]      f_v: velocity command, range [-45,45] rad/s
  * @param[in]      f_kp: kp parameter, range [0,500] N.m/rad
  * @param[in]      f_kd: kd parameter, range [0,5] N.m/rad/s
  * @param[in]      f_t:  torque command,range [-18,18] N.m
  * @param[in]      motor_id: id of joint motor,0x01~0x04
  * @retval         none
  */
/**
  * @brief          ???????????????(0x01,0x02,0x03,0x04)
  * @param[in]      f_p: ???λ?????Χ [-95.5,95.5] rad
  * @param[in]      f_v: ?????????Χ [-45,45] rad/s
  * @param[in]      f_kp: kp?????? ??Χ [0??500] N.m/rad
  * @param[in]      f_kd: kd????,  ??Χ [0,5] N.m/rad/s
  * @param[in]      f_t: ???????, ??Χ [-18,18] N.m
  * @param[in]      motor_id: ?????ID????0x01??0x04
  * @retval         none
  */
void CanComm_SendControlPara(float f_p, float f_v, float f_kp, float f_kd, float f_t,uint32_t motor_id)
{
    uint16_t p, v, kp, kd, t;
    uint8_t buf[8];
    
    /* ???????????????????Χ?? */
    LIMIT_MIN_MAX(f_p,  P_MIN,  P_MAX);
    LIMIT_MIN_MAX(f_v,  V_MIN,  V_MAX);
    LIMIT_MIN_MAX(f_kp, KP_MIN, KP_MAX);
    LIMIT_MIN_MAX(f_kd, KD_MIN, KD_MAX);
    LIMIT_MIN_MAX(f_t,  T_MIN,  T_MAX);
    
    /* ????Э?飬??float??????????? */
    p = float_to_uint(f_p,   P_MIN,  P_MAX,  16);            
    v = float_to_uint(f_v,   V_MIN,  V_MAX,  12);
    kp = float_to_uint(f_kp, KP_MIN, KP_MAX, 12);
    kd = float_to_uint(f_kd, KD_MIN, KD_MAX, 12);
    t = float_to_uint(f_t,   T_MIN,  T_MAX,  12);
    
    /* ???????Э?飬??????????CAN??????????? */
    buf[0] = p>>8;
    buf[1] = p&0xFF;
    buf[2] = v>>4;
    buf[3] = ((v&0xF)<<4)|(kp>>8);
    buf[4] = kp&0xFF;
    buf[5] = kd>>4;
    buf[6] = ((kd&0xF)<<4)|(t>>8);
    buf[7] = t&0xff;
    
    /* ???CAN????buf?е?????????? */
    CanTransmit(buf, sizeof(buf),motor_id);
}

/**
  * @brief          send mode data to set the control mode of joint motor 
  * @param[in]      cmd: mode command, CMD_MOTOR_MODE(0x05), CMD_RESET_MODE(0x06), CMD_ZERO_POSITION(0x07)
  * @param[in]      motor_id: id of joint motor,0x01~0x04
  * @retval         none
  */
/**
  * @brief          ??????????????ù???????????
  * @param[in]      cmd????????????(0x05)???Ч??(0x06)????λ??????(0x07)
  * @param[in]      motor_id: ?????ID????0x01??0x04
  * @retval         none
  */
void CanComm_ControlCmd(uint8_t cmd,uint32_t motor_id)
{
    uint8_t buf[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00};
    switch(cmd)
    {
			case CMD_MOTOR_MODE:
					buf[7] = 0xFC;
          x=2;
        
					break;
			
			case CMD_RESET_MODE:
					buf[7] = 0xFD;
			break;
			
			case CMD_ZERO_POSITION:
					buf[7] = 0xFE;
			break;
			
			default:
			return; /* ?????????? */
    }
    CanTransmit(buf,sizeof(buf),motor_id);
}

static void ZeroPosition()
{
	CanComm_ControlCmd(CMD_MOTOR_MODE,0x02);
	HAL_Delay(100);
  // CanComm_SendControlPara(0,0,0,0,0,0x02);
    x= 1;
	HAL_Delay(100);
}

/**
  * @brief          call function "ZeroPosition", and calibrate zero position 
  * @param[out]     leg_move_start: "leg_move" valiable point
  * @retval         none
  */
/**
  * @brief          ����ZeroPosition��������У׼�����λ
  * @param[out]     leg_move_start:"leg_move"����ָ��.
  * @retval         none
  */
void MotorControl_Start()
{
  ZeroPosition();
  x=12;
	
	//У׼�����λ


    // CanComm_ControlCmd(CMD_ZERO_POSITION,0x02);
	
	HAL_Delay(100);
}

/**
  * @brief          stop motor rotation
  * @param[out]     leg_move_stop: "leg_move" valiable point
  * @retval         none
  */
/**
  * @brief          ֹͣ����˶�
  * @param[out]     leg_move_stop:"leg_move"����ָ��.
  * @retval         none
  */
void MotorControl_Stop()
{

    CanComm_ControlCmd(CMD_RESET_MODE,0x02);
    motor_position = position;
	
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
  CanComm_Init();
//  MotorControl_Start();

  /* USER CODE BEGIN 2 */

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
    
//    CurVelocity = CanComm_GetCurVelocity();
//    CanComm_SendControlPara(1,0,0,0,0,0x02);
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
  RCC_OscInitStruct.PLL.PLLN = 84;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
