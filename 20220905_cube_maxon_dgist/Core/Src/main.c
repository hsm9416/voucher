/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

RNG_HandleTypeDef hrng;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
USART_HandleTypeDef husart6;

/* USER CODE BEGIN PV */

//Tx(Transmitter)
CAN_TxHeaderTypeDef L_MOTOR_ID;  //left motor id
CAN_TxHeaderTypeDef R_MOTOR_ID;  //right motor id


//Rx(Receiver)
CAN_FilterTypeDef sFilterConfig;   // Filter Settings Structure Variables
CAN_RxHeaderTypeDef RxHeader;      //Can Receive Header
uint8_t RxData[8];                 //Can Receive data save

////status
//uint16_t status[2][4] = { 0 };
//uint32_t status_l = 0;
//uint32_t status_r = 0;

//Encoder position variable
uint16_t encoder[2][4] = { 0 }; //Receive encoder data
uint32_t encoder_10_l = 0; //left motor encoder
uint32_t encoder_10_r = 0; //right motor encoder
double encoder_angle_l = 0; //left motor angle
double encoder_angle_r = 0; //right motor angle
float cnt_to_angle = 0.001758;
float deg_l = 0; //left motor degree
float deg_r = 0; //right motor degree

float vel_l = 0.0;
float vel2_l = 0.0;
float vel_r = 0.0;

float ivel_l = 0.0;
float ivel_r = 0.0;
float pre_vel_l = 0.0;
float pre_vel_r = 0.0;
float ivel_l_max = 0.0;
float ivel_r_max = 0.0;
float ivel_l_min = 0.0;
float ivel_r_min = 0.0;

float iivel_l = 0.0;
float iivel_r = 0.0;
float pre_ivel_l = 0.0;
float pre_ivel_r = 0.0;
float iivel_l_max = 0.0;
float iivel_r_max = 0.0;
float iivel_l_min = 0.0;
float iivel_r_min = 0.0;
float trend_l = 0.0;
float trend_r = 0.0;

float pi = 3.14159265359;
float k = 0.0;
float k1 = 0.0;
float kd = 0.0;
float fr = 0.0;
float fl = 0.0;
float alpha = 0.05; //1;  //0.03;  //0.05;

float PHI_l = 0.0;
float PHI_r = 0.0;

float zl = 0.0;
float al = 0.0;
float bl = 0.0;
float zr = 0.0;
float ar = 0.0;
float br = 0.0;

float a1_l = 0.0;
float a2_l = 0.0;
float a3_l = 0.0;
float v1_l = 0.0;
float v2_l = 0.0;
float v3_l = 0.0;
float a1dot_l = 0.0;
float a2dot_l = 0.0;
float v1dot_l = 0.0;
float v2dot_l =0.0;
uint32_t cnt2 = 0;
uint32_t cnt3 = 0;
uint32_t cnt4 = 0;
uint32_t cnt5 = 0;
int af_min_vel = 0;
int bf_min_vel = 0;
int int_min_vel = 0;
int af_min_deg = 0;
int bf_min_deg = 0;
int int_min_deg = 0;

float a1_r = 0.0;
float a2_r = 0.0;
float a3_r = 0.0;
float v1_r = 0.0;
float v2_r = 0.0;
float v3_r = 0.0;
float a1dot_r = 0.0;
float a2dot_r = 0.0;
float v1dot_r = 0.0;
float v2dot_r =0.0;
int af_min_vel2 = 0;
int bf_min_vel2 = 0;
int int_min_vel2 = 0;
int af_min_deg2 = 0;
int bf_min_deg2 = 0;
int int_min_deg2 = 0;


float deg_l_max = 0.0;
float deg_l_min = 0.0;
float deg_r_max = 0.0;
float deg_r_min = 0.0;

float vel_rpm_l_max = 0.0;
float vel_rpm_l_min = 0.0;
float vel_rpm_r_max = 0.0;
float vel_rpm_r_min = 0.0;
float vel_l_max = 0.0;
float vel_l_min = 0.0;
float vel_r_max = 0.0;
float vel_r_min = 0.0;


float trigger1 = 0.0;
float trigger2 = 0.0;

int topo_l = 0;
int topo_r = 0;
int topo = 0;

//VELOCITY variable
uint16_t vel[2][4] = { 0 }; //Receive velocity data
uint16_t vel_10_l = 0; //left motor_vel
uint16_t vel_10_r = 0; //right motor_vel
double velocity_l = 0; //caculation left motor_vel
double velocity_r = 0; //caculation right motor_vel
float vel_rpm_l = 0; //left motor rpm
float vel_rpm_r = 0; //right motor rpm
float ql = 0.0;
float qr = 0.0;
float ql_raw = 0.0;
float qr_raw = 0.0;
float vel_rpm_l_pre = 0.0;
float vel_rpm_r_pre = 0.0;

//motor current
float comp_tl = 0; //left motor torque
float comp_tr = 0; //right motor torque
int Current_l = 0; //caculation Current_left motor
int Current_r = 0; //caculation Current_right motor
float torque_l = 0.0;
float torque_r = 0.0;

//Switch
int SW[6] = { 0 };
int cnt = 0;
int Level = 0;
int Mode = 0;
float deg_l_init = 0.0;
float deg_r_init = 0.0;

//roll = atof(imu1[0]);
//pitch = atof(imu1[1]);
//yaw = atof(imu1[2]);
//vx = atof(imu1[3]);
//vy = atof(imu1[4]);
//vz = atof(imu1[5]);

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_CAN1_Init(void);
static void MX_CAN2_Init(void);
static void MX_RNG_Init(void);
static void MX_UART4_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART6_Init(void);
static void MX_TIM2_Init(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */

void BEEP_Fliker_ms(uint16_t ms) {
	HAL_GPIO_TogglePin(beep_GPIO_Port, beep_Pin); //beep on
	//HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin); //led on
	HAL_Delay(ms); //delay
	//HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 0); //led off
	HAL_GPIO_WritePin(beep_GPIO_Port, beep_Pin, 0); //beep off
	//TIM_Cmd (TIM5, ENABLE); //TIM control, (Don't use)
}

void MAXON_Init(void) {

	//////CAN TEST
	//Tx Set
	//Need to make DLC 4 type
	L_MOTOR_ID.StdId = 0x601;                 // Standard Identifier, 0 ~ 0x7FF
	//L_MOTOR_ID.ExtId = 0x01;                // Extended Identifier, 0 ~ 0x1FFFFFFF
	L_MOTOR_ID.RTR = CAN_RTR_DATA; // frame type of the message you are sending, DATA or REMOTE
	L_MOTOR_ID.IDE = CAN_ID_STD; // Identifier type of message to send, STD or EXT
	L_MOTOR_ID.DLC = 8;                     // Transmit frame length, 0 ~ 8 byte
	L_MOTOR_ID.TransmitGlobalTime = DISABLE; // Capture the timestamp counter value at the start of frame transmission

	R_MOTOR_ID.StdId = 0x602;                 // Standard Identifier, 0 ~ 0x7FF
	//R_MOTOR_ID.ExtId = 0x01;                // Extended Identifier, 0 ~ 0x1FFFFFFF
	R_MOTOR_ID.RTR = CAN_RTR_DATA; // frame type of the message you are sending, DATA or REMOTE
	R_MOTOR_ID.IDE = CAN_ID_STD; // Identifier type of message to send, STD or EXT
	R_MOTOR_ID.DLC = 8;                     // Transmit frame length, 0 ~ 8 byte
	R_MOTOR_ID.TransmitGlobalTime = DISABLE; // Capture the timestamp counter value at the start of frame transmission

	//RX set
	/* CAN Filter Setting*/
	sFilterConfig.FilterBank = 0;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	sFilterConfig.FilterIdHigh = 0x0000;
	sFilterConfig.FilterIdLow = 0x0000;
	sFilterConfig.FilterMaskIdHigh = 0x0000; // 0x00000000 = accept all the IDs
	sFilterConfig.FilterMaskIdLow = 0x0000;
	sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	sFilterConfig.FilterActivation = ENABLE;
	sFilterConfig.SlaveStartFilterBank = 14;


	if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK) {
		/* Filter configuration Error */
		Error_Handler();
	}

	//HAL_Delay(1000);  //wait motor on
	//Send Message
	//development motor : FR, DV, SD, SO, SM, OE
	//other motor : FR, DV, SD, SO, OE, SM
	//left motor
	HAL_CAN_Start(&hcan1);
	//HAL_Delay(1000);  //wait motor on
	HAL_CAN_AddTxMessage(&hcan1, &L_MOTOR_ID, FR, &TxMailbox);
	HAL_Delay(10);
	HAL_CAN_AddTxMessage(&hcan1, &L_MOTOR_ID, DV, &TxMailbox);
	HAL_Delay(10);
	HAL_CAN_AddTxMessage(&hcan1, &L_MOTOR_ID, SD, &TxMailbox);
	HAL_Delay(10);
	HAL_CAN_AddTxMessage(&hcan1, &L_MOTOR_ID, SO, &TxMailbox);
	HAL_Delay(10);
	HAL_CAN_AddTxMessage(&hcan1, &L_MOTOR_ID, SM, &TxMailbox);
	HAL_Delay(10);
	HAL_CAN_AddTxMessage(&hcan1, &L_MOTOR_ID, OE, &TxMailbox);
	HAL_Delay(10);

	//right motor
	HAL_CAN_AddTxMessage(&hcan1, &R_MOTOR_ID, FR, &TxMailbox);
	HAL_Delay(10);
	HAL_CAN_AddTxMessage(&hcan1, &R_MOTOR_ID, DV, &TxMailbox);
	HAL_Delay(10);
	HAL_CAN_AddTxMessage(&hcan1, &R_MOTOR_ID, SD, &TxMailbox);
	HAL_Delay(10);
	HAL_CAN_AddTxMessage(&hcan1, &R_MOTOR_ID, SO, &TxMailbox);
	HAL_Delay(10);
	HAL_CAN_AddTxMessage(&hcan1, &R_MOTOR_ID, SM, &TxMailbox);
	HAL_Delay(10);
	HAL_CAN_AddTxMessage(&hcan1, &R_MOTOR_ID, OE, &TxMailbox);
	HAL_Delay(10);


	HAL_CAN_AddTxMessage(&hcan1, &L_MOTOR_ID, FR, &TxMailbox);
	HAL_Delay(10);
	HAL_CAN_AddTxMessage(&hcan1, &L_MOTOR_ID, DV, &TxMailbox);
	HAL_Delay(10);
	HAL_CAN_AddTxMessage(&hcan1, &L_MOTOR_ID, SD, &TxMailbox);
	HAL_Delay(10);
	HAL_CAN_AddTxMessage(&hcan1, &L_MOTOR_ID, SO, &TxMailbox);
	HAL_Delay(10);
	HAL_CAN_AddTxMessage(&hcan1, &L_MOTOR_ID, SM, &TxMailbox);
	HAL_Delay(10);
	HAL_CAN_AddTxMessage(&hcan1, &L_MOTOR_ID, OE, &TxMailbox);
	HAL_Delay(10);
}

void CAN_GetRxMessage(void) {
	HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, RxData); //Receive can message

	/*Get_Position*/

	//Get_Position Left  (43, 64, 60)
	if ((RxData[0] == 0x43) & (RxData[1] == 0x64) & (RxData[2] == 0x60)
			& (RxHeader.StdId == 0x581))			//PX
			{
		//PX계산 코드.. CAN1Rx_Arr[4~7] Here..
		encoder[0][0] = RxData[4];
		encoder[0][1] = RxData[5];
		encoder[0][2] = RxData[6];
		encoder[0][3] = RxData[7];

		encoder_10_l = (encoder[0][3] << 24) + (encoder[0][2] << 16)
				+ (encoder[0][1] << 8) + encoder[0][0];

		//left motor minus angle

		if (encoder[0][3] == 0x00FF) {
			encoder_angle_l = 0;
			encoder_10_l = encoder_10_l ^ 0xFFFFFFFF; //Using a Conservative XOR for C-language negative conversion
			encoder_angle_l = -((encoder_10_l) * cnt_to_angle);
			//deg_l = (ceil(encoder_angle_l));
			deg_l = round(encoder_angle_l * 100) / 100;
		}
		//left motor plus angle
		else {
			encoder_angle_l = encoder_10_l * cnt_to_angle;
			//deg_l = (ceil(encoder_angle_l));
			deg_l = round(encoder_angle_l * 100) / 100;
		}
	}

	//Get_Position Right
	if ((RxData[0] == 0x43) & (RxData[1] == 0x64) & (RxData[2] == 0x60)
			& (RxHeader.StdId == 0x582)) //PX
			{
		//PX계산 코드.. CAN2Rx_Arr[4~7] Here..
		encoder[1][0] = RxData[4];
		encoder[1][1] = RxData[5];
		encoder[1][2] = RxData[6];
		encoder[1][3] = RxData[7];

		encoder_10_r = (encoder[1][3] << 24) + (encoder[1][2] << 16)
				+ (encoder[1][1] << 8) + encoder[1][0];

		//right motor minuis angle

		if (encoder[1][3] == 0x00FF) {
			encoder_angle_r = 0;
			encoder_10_r = encoder_10_r ^ 0xFFFFFFFF; //Using a Conservative XOR for C-language negative conversion
			encoder_angle_r = -((encoder_10_r) * cnt_to_angle);
			//deg_r = (ceil(encoder_angle_r));
			deg_r = round(encoder_angle_r * 100) / 100;
		}
		//right motor plus angle
		else {
			encoder_angle_r = encoder_10_r * cnt_to_angle;
			//deg_r = (ceil(encoder_angle_r));
			deg_r = round(encoder_angle_r * 100) / 100;
		}
		deg_r = -1 * deg_r;
	}

	/*Get_Velocity*/

	//Get_Velocity Left
	if ((RxData[0] == 0x43) & (RxData[1] == 0x6C) & (RxData[2] == 0x60)
			& (RxHeader.StdId == 0x581)) //VX
			{
		//VX계산 코드.. CAN2Rx_Arr[4~7] Here..

		vel[0][0] = RxData[4];
		vel[0][1] = RxData[5];
		vel[0][2] = RxData[6];
		vel[0][3] = RxData[7];

		vel_10_l = (vel[0][3] << 24) + (vel[0][2] << 16) + (vel[0][1] << 8)
				+ vel[0][0];

		if (vel[0][3] == 0x00FF) {
			velocity_l = 0;
			vel_10_l = vel_10_l ^ 0xFFFFFFFF;
			velocity_l = -vel_10_l;
			vel_rpm_l = velocity_l / 100;
		} else {
			velocity_l = vel_10_l;
			vel_rpm_l = velocity_l / 100;
		}

	}

	//Get_Velocity Right
	if ((RxData[0] == 0x43) & (RxData[1] == 0x6C) & (RxData[2] == 0x60)
			& (RxHeader.StdId == 0x582)) //VX
			{
		//VX계산 코드.. CAN2Rx_Arr[4~7] Here..

		vel[1][0] = RxData[4];
		vel[1][1] = RxData[5];
		vel[1][2] = RxData[6];
		vel[1][3] = RxData[7];

		vel_10_r = (vel[1][3] << 24) + (vel[1][2] << 16) + (vel[1][1] << 8)
				+ vel[1][0];

		if (vel[1][3] == 0x00FF) {
			velocity_r = 0;
			vel_10_r = vel_10_r ^ 0xFFFFFFFF;
			velocity_r = -vel_10_r;
			vel_rpm_r = velocity_r / 100;
		} else {
			velocity_r = vel_10_r;
			vel_rpm_r = velocity_r / 100;
		}

		vel_rpm_r = -1 * vel_rpm_r;

	}




	HAL_Delay(0.01);

}

// CAN ?��?�� ?��?��?��?�� 콜백, ?��?�� ?��급을 ?��?��?��?�� ?��?��?�� ?��?���?????????? ?��?��?�� ?��?���?????????? ?��?��?��건�? ?�� 모르겠음

//void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *CanHandle) {
//	/* Get RX message */
//	if (HAL_CAN_GetRxMessage(CanHandle, CAN_RX_FIFO0, &RxHeader, RxData)
//			!= HAL_OK) {
//		/* Reception Error */
//		Error_Handler();
//	}
//
//}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//adc 1channel
uint16_t value;

//adc 2channel
float adcVal[2] = { 0 };

//BLUETOOTH
unsigned char RxBuffer[10]; // Rx Buffer
unsigned char B_data[10]; //RxBuffer -> Bluetooth save data
char ble_write_buff[256]; //Bluetooth send data

//AT data
char bluetooth1[] = "+++";
char bluetooth2[] = "\r";
char bluetooth3[] = "AT+BTUART=115200\r";
char bluetooth4[] = "AT+BTNAME=HECTOR H30A\r";
char bluetooth5[] = "ATZ\r";

//BLE flag
int Rx_IRQ = 0; //Need to produce IRQ later
int b_num = 0;  //Bluetooth save data arrange number
int data_send_toggle = 0; //bluetooth send flag

//IMU DATA
char imu[10] = { 0 }; //IMUdata - get it first time and save it one by one
char imu_data[60] = { 0 }; //IMUdata - Alignment
double imu_d[10] = { 0 }; //Data converted to double
char *addr; //"," cut and Temporary data storage

//Final IMU data storage
float roll = 0.00f;
float pitch = 0.00f;
float yaw = 0.00f;
float vx = 0.00f;
float vy = 0.00f;
float vz = 0.00f;
float v = 0.00f;

//IMU flag
int imu_num = 0; //IMU save data arrange number
int stop = 0;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {

   ////BLE Interrupt
   if (huart->Instance == UART4) {

      B_data[b_num] = RxBuffer[0];
      b_num++;
      if (b_num == 9) {
         b_num = 0;
      }
      //1by1 check
      char *B_T = strstr(RxBuffer, "T");
      char *B_O = strstr(RxBuffer, "O");
      char *B_G = strstr(RxBuffer, "G");
      char *B_L = strstr(RxBuffer, "L");
      char *B_E = strstr(RxBuffer, "E");
      char *check_1 = strstr(RxBuffer, "1");
      char *check_2 = strstr(RxBuffer, "2");
      char *check_0 = strstr(RxBuffer, "0");

      if (B_T != NULL && data_send_toggle == 0) {
         data_send_toggle = 1;
      }

      if (B_O != NULL && data_send_toggle == 1) {
         data_send_toggle = 2;
      }
      if (B_G != NULL && data_send_toggle == 2) {
         data_send_toggle = 3;
      }

      if (B_G != NULL && data_send_toggle == 3) {
         data_send_toggle = 4;
      }
      if (B_L != NULL && data_send_toggle == 4) {
         data_send_toggle = 5;
      }
      if (B_E != NULL && data_send_toggle == 5) {
         data_send_toggle = 6;
      }

      if (check_1 != NULL && data_send_toggle == 6) { //check "1"
         data_send_toggle = 7;
      }

      if (check_2 != NULL && data_send_toggle == 6) { //check "2"
         data_send_toggle = 8;
      }

      if (check_0 != NULL && data_send_toggle == 7) { //check "0"
              data_send_toggle = 0;
      }

      if (check_0 != NULL && data_send_toggle == 8) { //check "0"
              data_send_toggle = 0;
      }


      HAL_UART_Receive_IT(&huart4, RxBuffer, 1);
   } //huart->Instance == UART4

   ////IMU Interrupt
   if (huart->Instance == USART3) {

      char *check_start = strstr(imu, "*");
      char *check_start2 = strstr(imu_data, "*");
      char *check_comma = strstr(imu, ",");

      if (check_start != NULL) { //check "*"
         imu_num = 0; //save data number Initialization
      }

      else { //IMU data
         imu_data[imu_num] = imu[0];

         if (imu_data[imu_num] == 0x0A) { //check "/n" / character : LF
            addr = strtok(imu_data, ","); //cut data from the ","

            for (int i = 0; i < 3; i++) {
               imu_d[i] = atof(addr); //Converting characters to double
               addr = strtok(NULL, ","); //Cut it over and over, by result NULL
            }

            //distribute data
            roll = imu_d[0];
            pitch = imu_d[1];
            yaw = imu_d[2];
            //vx = imu_d[3];
            //vy = imu_d[4];
            //vz = imu_d[5];
            //v = sqrt(vx * vx + vy * vy + vz * vz); //Calculating the speed

            //if (fabs(v) <= 0) {
            //   stop = 1;
            //} else {
            //   stop = 0;
            //}
         }

         imu_num++;
      }
      HAL_UART_Receive_IT(&huart3, imu, 1);
   }
}

//void USART4_IRQHandler(void) {
//
//	Rx_IRQ++;
//	HAL_UART_IRQHandler(&huart4);
//	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 0); //led off
//
//}

//TIM Interrupt
float time_sec = 0;
float Counter = 0;
float tenms = 0.0;

//Timer Interrupt
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == htim2.Instance) {
		Counter++;
		time_sec = Counter / 1000;
		tenms = Counter / 100;
	}
}

//Need to analyze the contents.I don't understand exactly.
#ifdef __GNUS__
/* With GCC, small printf (option LD Linker->Libraries->Small printf
 * set to 'Yes') calls__io_putchar(int ch)
 */

#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /*__GNUC__*/

/**
 * @brief Retargets the C library printf function to the USART.
 * @param None
 * @retval None
 */
//Check the defined format. --CAN Receive example //???????
//PUTCHAR_PROTOTYPE {
//	if (ch == '\n') {
//		HAL_UART_Transmit(&husart6, (uint8_t*) "\r", 1, 0xFFFF);
//		HAL_UART_Transmit(&husart6, (uint8_t*) &ch, 1, 0xFFFF);
//	}
//
//	return ch;
//
//}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
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
	MX_ADC1_Init();
	MX_CAN1_Init();
	MX_CAN2_Init();
	MX_RNG_Init();
	MX_UART4_Init();
	MX_USART1_UART_Init();
	MX_USART2_UART_Init();
	MX_USART3_UART_Init();
	MX_USART6_Init();
	MX_TIM2_Init();
	MX_UART4_Init();

	/* Initialize interrupts */
	MX_NVIC_Init();
	/* USER CODE BEGIN 2 */
	//HAL_UART_Receive_IT(&huart4, &RxBuffer, 6);
	HAL_TIM_Base_Start_IT(&htim2);

// Setting Ready
	HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);  //LED ON
	MAXON_Init(); //SET motor CAN
	//HAL_Delay(1000);  //wait motor on

	//BEEP_Fliker_ms(17); //Beep 17ms

	HAL_UART_Receive_IT(&huart4, RxBuffer, 1); //Bluetooth Receive Interrupt
	HAL_UART_Receive_IT(&huart3, imu, 1); //IMU Receive Interrupt
	HAL_Delay(1000);  //wait motor on
	MAXON_Init(); //SET motor CAN
	BEEP_Fliker_ms(17); //Beep 17ms

//		}
//	}

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */

	while (1) {

		//////////////////////////////////////////Switch START
		SW[0] = HAL_GPIO_ReadPin(sw0_GPIO_Port, sw0_Pin); //left
		SW[1] = HAL_GPIO_ReadPin(sw1_GPIO_Port, sw1_Pin); //left
		//SW[2] = HAL_GPIO_ReadPin(sw2_GPIO_Port, sw2_Pin); //left -not use
		SW[3] = HAL_GPIO_ReadPin(sw3_GPIO_Port, sw3_Pin); //right
		SW[4] = HAL_GPIO_ReadPin(sw4_GPIO_Port, sw4_Pin); //right
		//SW[5] = HAL_GPIO_ReadPin(sw5_GPIO_Port, sw5_Pin); //right -not use

		cnt++;
		//if (cnt >= 10000) {
			//cnt = 0;
		//}

		if ((SW[0] == 0) & (cnt % 45 == 0)) {

			Level = Level + 1;

			if (Level == 0) {
				BEEP_Fliker_ms(17);
			}
			if (Level == 1) {
				BEEP_Fliker_ms(17);
				HAL_Delay(300);
				BEEP_Fliker_ms(17);
			}
			if (Level == 2) {
				BEEP_Fliker_ms(17);
				HAL_Delay(300);
				BEEP_Fliker_ms(17);
				HAL_Delay(300);
				BEEP_Fliker_ms(17);
			}

			if (Level > 2) {
				Level = 2;
			}
		}

		if ((SW[1] == 0) & (cnt % 45 == 0)) {

			Level = Level - 1;
			if (Level == 0) {
				BEEP_Fliker_ms(17);
			}
			if (Level == 1) {
				BEEP_Fliker_ms(17);
				HAL_Delay(300);
				BEEP_Fliker_ms(17);
			}
			if (Level == 2) {
				BEEP_Fliker_ms(17);
				HAL_Delay(300);
				BEEP_Fliker_ms(17);
				HAL_Delay(300);
				BEEP_Fliker_ms(17);
			}
			if (Level < 0) {
				Level = 0;
			}
		}

		if ((SW[3] == 0) & (cnt % 45 == 0)) {
			Mode = Mode + 1;
			BEEP_Fliker_ms(17);
			if (Mode > 1) {
				Mode = 1;
			}
		}

		if ((SW[4] == 0) & (cnt % 45 == 0)) {
			//Mode = Mode +2;
			BEEP_Fliker_ms(17);
			Mode = 0;
		}

		//Level = 0;
		//Mode = 0;

		if (Level == 0) {
			k = 0.08;
			k1 = 0.1;
		}

		if (Level == 1) {
			k = 0.1;
			k1 = 0.15;
		}

		if (Level == 2) {
			k = 0.15;
			k1 = 0.2;
		}

		if (Level == -1) {
			k = -0.05;
			kd = 0;
			fr = 0;
			fl = 0;
		}

		if (Level == -2) {
			k = -0.15;
			kd = 0;
			fr = 0;
			fl = 0;
		}

		/////////////////////////////////////////BLE START

		uint16_t ccc = Counter;

		if (data_send_toggle == 7 && (ccc % 2 == 0)) { //send data to PC
			
			sprintf(ble_write_buff,
					"\r\nDATA|{\"0\":%.3f, \"1\":%.2f, \"2\":%.2f, \"3\":%.2f, \"4\":%.2f, \"5\":%.2f, \"6\":%.2f, \"7\":%.2f, \"8\":%d, \"9\":%d}\r\n",
					time_sec, a3_l*180/pi, a3_r*180/pi, vel_rpm_l, vel_rpm_r, roll, pitch, yaw, topo_l, topo_r);
			
			HAL_UART_Transmit_IT(&huart4, &ble_write_buff, strlen(&ble_write_buff)); //Not set

		} else if (data_send_toggle == 8 && (cnt % 10 == 0)) { //send data to mobile

			sprintf(ble_write_buff, "\\r\\nDATA|{\"1\":%.2f, \"2\":%.2f}\\r\\n",
					a3_l*180/pi, a3_r*180/pi);

			HAL_UART_Transmit(&huart4, &ble_write_buff, strlen(&ble_write_buff),
					100);
		}

		//////////////////////////////////////////CAN START
		CAN_GetRxMessage();  //Receive can message

		//Position Command_left (40, 64, 60, 00)
		HAL_CAN_AddTxMessage(&hcan1, &L_MOTOR_ID, GP, &TxMailbox); //Transmit can massage
		CAN_GetRxMessage(); //Receive can message

		//Position Command_right (40, 64, 60, 00)
		HAL_CAN_AddTxMessage(&hcan1, &R_MOTOR_ID, GP, &TxMailbox); //Transmit can massage
		CAN_GetRxMessage(); //Receive can message

		//Velocity Command_left (40, 6C, 60, 00)
		HAL_CAN_AddTxMessage(&hcan1, &L_MOTOR_ID, GV, &TxMailbox); //Transmit can massage
		CAN_GetRxMessage(); //Receive can message

		//Velocity Command_right (40, 6C, 60, 00)
		HAL_CAN_AddTxMessage(&hcan1, &R_MOTOR_ID, GV, &TxMailbox); //Transmit can massage
		CAN_GetRxMessage(); //Receive can message

		//write current



		//-----------------------모드 0 : 현재 각도를 조기 각도로 설정 ----------------------//
		// 버튼을 한번 누름으로써 초기 각도를 설정한다
		if (Mode == 0) {

			deg_l_init = deg_l;
			deg_r_init = deg_r;
			comp_tl = 0;
			comp_tr = 0;

		}


		    if (Mode == 1) { //left Admittance control + AAN 현재 되있지 않음. 

		    	deg_l = deg_l - deg_l_init;
		    	deg_r = deg_r - deg_r_init;

		    	float qvl_raw = 0.0;
		    	float qvr_raw = 0.0;

		    	qvl_raw = vel_rpm_l;
		    	vel_rpm_l = (1 - alpha) * vel_rpm_l_pre + alpha * qvl_raw; //LPF
		    	qvr_raw = vel_rpm_r;
		    	vel_rpm_r = (1 - alpha) * vel_rpm_r_pre + alpha * qvr_raw; //LPF

		    	vel_l = vel_rpm_l * 0.10472; // - trend_l;
		    	vel_r = vel_rpm_r * 0.10472; // - trend_r;

		    	ivel_l = 0.5 * (2*ivel_l + pre_vel_l + vel_l); // - (ivel_l_max + ivel_l_min)/2;
		    	pre_vel_l = vel_l;
		    	ivel_r = 0.5 * (2*ivel_r + pre_vel_r + vel_r); // - (ivel_r_max + ivel_r_min)/2;
		    	pre_vel_r = vel_r;

		    	//ivel_l = (deg_l * pi / 180) - (deg_l_max + deg_l_min)/2;

		    	iivel_l = 0.5 * (2*iivel_l + pre_ivel_l + ivel_l); // - (vel_l_max + vel_l_min)/2;
		    	pre_ivel_l = ivel_l;
		    	iivel_r = 0.5 * (2*iivel_r + pre_ivel_r + ivel_r); // - (iivel_r_max + iivel_r_min)/2;
		    	pre_ivel_r = ivel_r;

		    	//vel2_l = iivel_l - (vel_l_max + vel_l_min)/2;

		    	            a1_l = a2_l;
		    	            a2_l = a3_l;
		    	            v1_l = v2_l;
		    			    v2_l = v3_l;

						   //degree to radian
		    	           a3_l = deg_l*pi/180;

		    	           v3_l = vel_l;
		    	           //v3_l = iivel_l;

		    	           a1dot_l = a2_l-a1_l;
		    	           a2dot_l = a3_l-a2_l;
		    	           v1dot_l = v2_l-v1_l;
		    	           v2dot_l = v3_l-v2_l;

		    			   if ((a1dot_l > 0 && a2dot_l <= 0) && (v2_l > 0.15) && (v2dot_l <= 0)) { // Max angle
		    			     deg_l_max = a2_l;
		    			   }

		    			   if ((a1dot_l < 0 && a2dot_l >= 0) && (v2_l < 0) && (v2dot_l > 0)) {     // % Min angle
		    			      //deg_l_min = a2_l;

		    			      cnt4 = cnt4 + 1;

		    			      if (cnt4 > 1) {
		    			         af_min_deg = cnt-1;   //% after period of min_vel
		    			         bf_min_deg = int_min_deg;

		    			          if (abs(af_min_deg - bf_min_deg) < 10) {

		    			          }

		    			          else {
		    			            deg_l_min = a2_l;
		    			          }

		    			          int_min_deg = cnt-1;

		    			       }

		    			       else {
		    			           //int_min_vel = cnt-1;       //% before period of min_vel
		    			           deg_l_min = a2_l;
		    			       }

		    			     }

		    			     if ((v1dot_l > 0 && v2dot_l <= 0) && (a2_l > 0) && (a2dot_l > 0)) {    // % Max velocity
		    			         vel_l_max = v2_l;
		    			     }
		    			            //if ((v1dot_l < 0 && v2dot_l > 0)) {
		    			            	//vel_l_min = v2_l;
		    			            //}

		    			     if ((v1dot_l < 0 && v2dot_l > 0) && (v2_l < 0)) {   //  % Min velocity
		    			         cnt2 = cnt2 + 1;

		    			         if (cnt2 > 1) {
		    			            af_min_vel = cnt-1;   //% after period of min_vel
		    			            bf_min_vel = int_min_vel;

		    			            if (abs(af_min_vel - bf_min_vel) < 50) {

		    			                            //vel_l_min = v2_l;
		    			            }

		    			            else {
		    			                vel_l_min = v2_l;
		    			            }

		    			            int_min_vel = cnt-1;
		    			          }

		    			          else {
		    			                     //int_min_vel = cnt-1;       //% before period of min_vel
		    			             vel_l_min = v2_l;
		    			          }

		    			      }
 								
								//left Admittance control + AAN 현재 되있지 않음. 
		    			         
								    a1_r = a2_r;
		    			            a2_r = a3_r;
		    			            v1_r = v2_r;
		    			            v2_r = v3_r;


		    			            a3_r = deg_r*pi/180;

		    			            v3_r = vel_r;


		    			            a1dot_r = a2_r-a1_r;
		    			            a2dot_r = a3_r-a2_r;

		    			            v1dot_r = v2_r-v1_r;
		    			            v2dot_r = v3_r-v2_r;


		    			            if ((a1dot_r > 0 && a2dot_r <= 0) && (v2_r > 0.15) && (v2dot_r <= 0)) { // Max angle
		    			            	deg_r_max = a2_r;
		    			            }

		    			            if ((a1dot_r < 0 && a2dot_r >= 0) && (v2_r < 0) && (v2dot_r > 0)) {     // % Min angle
		    			            	//deg_r_min = a2_r;
									
		    			            	cnt5 = cnt5 + 1;

		    			            	if (cnt5 > 1) {
		    			            		af_min_deg2 = cnt-1;   //% after period of min_vel
		    			            		bf_min_deg2 = int_min_deg2;

		    			            		if (abs(af_min_deg2 - bf_min_deg2) < 10) {

		    			            		}

		    			            		else {
		    			            			deg_r_min = a2_r;
		    			            		}

		    			            		int_min_deg2 = cnt-1;

		    			            	}

		    			            	else {
		    			            	//int_min_vel = cnt-1;       //% before period of min_vel
		    			            		deg_r_min = a2_r;
		    			            	}


		    			            }

		    			            if ((v1dot_r > 0 && v2dot_r <= 0) && (a2_r > 0) && (a2dot_r > 0)) {    // % Max velocity
		    			            	vel_r_max = v2_r;
		    			            }


		    			            if ((v1dot_r < 0 && v2dot_r > 0) && (v2_r < 0)) {   //  % Min velocity
		    			            	cnt3 = cnt3 + 1;

		    			            	if (cnt3 > 1) {
		    			            		af_min_vel2 = cnt-1;   //% after period of min_vel
		    			            		bf_min_vel2 = int_min_vel2;

		    			            		if (abs(af_min_vel2 - bf_min_vel2) < 50) {

		    			            		    //vel_l_min = v2_l;

		    			            		}

		    			            		else {
		    			            		    vel_r_min = v2_r;
		    			            		}

		    			            		int_min_vel2 = cnt-1;

		    			            	}

		    			            	else {
		    			            		//int_min_vel = cnt-1;       //% before period of min_vel
		    			            		vel_r_min = v2_r;
		    			            	}

		    			            }



		    			    if (fabs(ivel_l) <= 0.2) {

		    			    }
		    			    if (fabs(ivel_r) <= 0.2) {

		    			    }
		    			    trend_l = (vel_l_max + vel_l_min)/2;
		    			    trend_r = (vel_r_max + vel_r_min)/2;

		    			    if (fabs(vel_l) <= 0.2) {
		    			    	trend_l = 0;
		    			    }
		    			    if (fabs(vel_r) <= 0.2) {
		    			    	trend_r = 0;
		    			    }

		    			    trigger1 = round(PHI_l *100);
		    			    trigger2 = round(PHI_r *100);


		    			    if ((trigger1 == 25) || (trigger1 == 50) || (trigger1 == 75) || (trigger1 == 0)) { //normalizing factor
		    			   
		    			    	if ((vel_l_max ==  vel_l_min) || (deg_l_max == deg_l_min)) {
		    			        zl = 0;
		    			      }

		    			      else {
		    			      zl = fabs(vel_l_max - vel_l_min) / (fabs(deg_l_max - deg_l_min));
		    			      }

		    			      al = (fabs(deg_l_max + deg_l_min))/2;
		    			      bl = fabs(vel_l_max + vel_l_min)/2;

		    			    }

		    			    if ((trigger2 == 25) || (trigger2 == 50) || (trigger2 == 75) || (trigger2 == 0)) { //normalizing factor

		    			      if ((vel_r_max ==  vel_r_min) || (deg_r_max == deg_r_min)) {
		    			        zr = 0;
		    			      }

		    			      else {
		    			      zr = fabs(vel_r_max - vel_r_min) / (fabs(deg_r_max - deg_r_min));
		    			      }

		    			      ar = (fabs(deg_r_max + deg_r_min))/2;
		    			      br = fabs(vel_r_max + vel_r_min)/2;

		    			    }
		    			      PHI_l = atan2(zl * (deg_l * pi / 180 -al), vel_l -bl)/2/pi;

		    			      PHI_r = atan2(zr * (deg_r * pi / 180 -ar), vel_r -br)/2/pi;

		    			    if (PHI_l < 0) {
		    			      PHI_l = PHI_l + 1;
		    			    }

		    			    if (PHI_r < 0) {
		    			      PHI_r = PHI_r + 1;
		    			    }


		    			     if ((PHI_l > 0.95) & (PHI_l <= 1)) {

		    			    }

		    			    if ((PHI_r > 0.95) & (PHI_r <= 1)) {

		    			    }

		      				topo_r = 0;
		      				topo_l = 0;



		        			if ((vel_rpm_l == 0) || (vel_rpm_r ==0)) {
		        			comp_tl = 0;
		        			comp_tr = 0;
		        			}

		         //계단 오르막 결정

		       				 if ((deg_l_min >= 5*pi/180) & ((deg_l_max-deg_l_min) >= 50*pi/180) & (deg_l_max >= 45*pi/180)) {

		          			topo_l = 1;

		        			}

		        			if ((deg_r_min >= 5*pi/180) & ((deg_r_max-deg_r_min) >= 50*pi/180) & (deg_r_max >= 45*pi/180)) {

		          			topo_r = 1;

		        			}


		        //계단 내리막 결정

		        			if ((deg_l_min > 5*pi/180) & ((deg_l_max-deg_l_min) < 50*pi/180) & (deg_l_max >= 40*pi/180)) {

		        			topo_l = 3;

		        			}

		        			if ((deg_r_min > 5*pi/180) & ((deg_r_max-deg_r_min) < 50*pi/180) & (deg_r_max >= 40*pi/180)) {

		            		topo_r = 3;

		        		    }

		        			topo = topo_l + topo_r;
		        
		        if (topo_l == 1) { //계단 오르막 지원

		          if ((PHI_l >= 0.2) & (PHI_l <= 0.45)) {

		          comp_tl = -k * fabs(deg_l*1.4) -4;

		         }
		        }

		        if (topo_r == 1) { //계단 오르막 지원

		          if ((PHI_r >= 0.2) & (PHI_r <= 0.45)) {

		          comp_tr = -k * fabs(deg_r*1.4) -4;

		         }

		        }

		        if (topo_l == 3) { //계단 내리막 지원

		           	if ((PHI_l >= 0.65) & (PHI_l <= 1)) {

		        		comp_tl = fabs(vel_rpm_l * 1.2) * -k;
		        	}

		        	if ((PHI_l >= 0) & (PHI_l <= 0.1)) {

		        		comp_tl = fabs(vel_rpm_l * 1.2) * -k;
		        	}
		        }

		        if (topo_r == 3) { //계단 내리막 지원

		        	if ((PHI_r >= 0.65) & (PHI_r <= 1)) {

		        	    comp_tr = fabs(vel_rpm_r) * -k;

		        	}

		        	if ((PHI_r >= 0) & (PHI_r <= 0.1)) {

		        		comp_tr = fabs(vel_rpm_r) * -k;
		        	}

		        }


		        // 평지
		        if (topo_l == 0) {

		        
		        	if ((PHI_l >= 0.65) & (PHI_l <= 1)) {

		        		comp_tl= fabs(vel_rpm_l) * k1 +2;
		        	}

		        	if ((PHI_l >= 0) & (PHI_l <= 0.15)) {

		        		comp_tl= fabs(vel_rpm_l) * k1 +2;
		        	}

		        }

		        if (topo_r == 0) {

		        
		        	if ((PHI_r >= 0.65) & (PHI_r <= 1)) {

		        		comp_tr= fabs(vel_rpm_r) * k1 +2;
		        	}

		        	if ((PHI_r >= 0) & (PHI_r <= 0.15)) {

		        		comp_tr= fabs(vel_rpm_r) * k1 +2;
		        	}

		        }

		      if((vel_rpm_l < 1) & (vel_rpm_l > -1) & (deg_l <= 20) & (deg_l >= -10))
		      {
		        comp_tl = 0;
		      }



		      if((vel_rpm_r < 1) & (vel_rpm_r > -1) & (deg_r <= 20) & (deg_r >= -10))
			  
			   {

		        comp_tr = 0;

		       }

		      stop = 0;
		      if(stop == 1) {
		        comp_tl = 0;
		        comp_tr = 0;
		       }

		    }

		    if (Mode == 2) {  //서기

		      if (((vel_rpm_l < -5) & (deg_l >= 0)) & ((vel_rpm_r < -5) & (deg_r >= 0))) {

		        comp_tl = fabs(deg_l) * -k * 0.8;
		        comp_tr = fabs(deg_r) * -k * 0.8;

		      }
		    }


		      vel_rpm_l_pre = vel_rpm_l;
		      vel_rpm_r_pre = vel_rpm_r;
		//Computed torque

		      torque_l = comp_tl;
		      torque_r = comp_tr;
		comp_tl = comp_tl / 100;  // Harmonic gear
		comp_tl = comp_tl / 0.0369;
		Current_l = floor(comp_tl / 0.00321 + 0.5); // Compensation torque input
		if (Current_l >= 700) {
			Current_l = 700;
		}
		if (Current_l <= -700) {
			Current_l = -700;
		}

		memcpy(&WC_l[4], &Current_l, sizeof(Current_l));
		HAL_CAN_AddTxMessage(&hcan1, &L_MOTOR_ID, WC_l, &TxMailbox); //Transmit can massage
		HAL_Delay(0.01);

		comp_tr = -comp_tr / 100;
		comp_tr = comp_tr / 0.0369;
		Current_r = floor(comp_tr / 0.00321 + 0.5);
		if (Current_r >= 700) {
			Current_r = 700;
		}
		if (Current_r <= -700) {
			Current_r = -700;
		}

		memcpy(&WC_r[4], &Current_r, sizeof(Current_r));
		HAL_CAN_AddTxMessage(&hcan1, &R_MOTOR_ID, WC_r, &TxMailbox); //Transmit can massage
		HAL_Delay(0.01);

		///ADC_2channel code
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
		adcVal[0] = HAL_ADC_GetValue(&hadc1);

		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
		adcVal[1] = HAL_ADC_GetValue(&hadc1);

	}
	/* USER CODE END WHILE */

	/* USER CODE BEGIN 3 */

	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

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
	RCC_OscInitStruct.PLL.PLLN = 336;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 7;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief NVIC Configuration.
 * @retval None
 */
static void MX_NVIC_Init(void) {
	/* UART4_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(UART4_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(UART4_IRQn);
	/* USART3_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(USART3_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(USART3_IRQn);
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void) {

	/* USER CODE BEGIN ADC1_Init 0 */

	/* USER CODE END ADC1_Init 0 */

	ADC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN ADC1_Init 1 */

	/* USER CODE END ADC1_Init 1 */

	/** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.ScanConvMode = ENABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.DiscontinuousConvMode = ENABLE;
	hadc1.Init.NbrOfDiscConversion = 1;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 2;
	hadc1.Init.DMAContinuousRequests = DISABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	if (HAL_ADC_Init(&hadc1) != HAL_OK) {
		Error_Handler();
	}

	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_1;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}

	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_0;
	sConfig.Rank = 2;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */

	/* USER CODE END ADC1_Init 2 */

}

/**
 * @brief CAN1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_CAN1_Init(void) {

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
	if (HAL_CAN_Init(&hcan1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN CAN1_Init 2 */

	/* USER CODE END CAN1_Init 2 */

}

/**
 * @brief CAN2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_CAN2_Init(void) {

	/* USER CODE BEGIN CAN2_Init 0 */

	/* USER CODE END CAN2_Init 0 */

	/* USER CODE BEGIN CAN2_Init 1 */

	/* USER CODE END CAN2_Init 1 */
	hcan2.Instance = CAN2;
	hcan2.Init.Prescaler = 16;
	hcan2.Init.Mode = CAN_MODE_NORMAL;
	hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
	hcan2.Init.TimeSeg1 = CAN_BS1_1TQ;
	hcan2.Init.TimeSeg2 = CAN_BS2_1TQ;
	hcan2.Init.TimeTriggeredMode = DISABLE;
	hcan2.Init.AutoBusOff = DISABLE;
	hcan2.Init.AutoWakeUp = DISABLE;
	hcan2.Init.AutoRetransmission = DISABLE;
	hcan2.Init.ReceiveFifoLocked = DISABLE;
	hcan2.Init.TransmitFifoPriority = DISABLE;
	if (HAL_CAN_Init(&hcan2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN CAN2_Init 2 */

	/* USER CODE END CAN2_Init 2 */

}

/**
 * @brief RNG Initialization Function
 * @param None
 * @retval None
 */
static void MX_RNG_Init(void) {

	/* USER CODE BEGIN RNG_Init 0 */

	/* USER CODE END RNG_Init 0 */

	/* USER CODE BEGIN RNG_Init 1 */

	/* USER CODE END RNG_Init 1 */
	hrng.Instance = RNG;
	if (HAL_RNG_Init(&hrng) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN RNG_Init 2 */

	/* USER CODE END RNG_Init 2 */

}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void) {

	/* USER CODE BEGIN TIM2_Init 0 */

	/* USER CODE END TIM2_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 1;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 41999;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */

}

/**
 * @brief UART4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_UART4_Init(void) {

	/* USER CODE BEGIN UART4_Init 0 */

	/* USER CODE END UART4_Init 0 */

	/* USER CODE BEGIN UART4_Init 1 */

	/* USER CODE END UART4_Init 1 */
	huart4.Instance = UART4;
	huart4.Init.BaudRate = 9600;
	huart4.Init.WordLength = UART_WORDLENGTH_8B;
	huart4.Init.StopBits = UART_STOPBITS_1;
	huart4.Init.Parity = UART_PARITY_NONE;
	huart4.Init.Mode = UART_MODE_TX_RX;
	huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart4.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart4) != HAL_OK) {
		Error_Handler();
	}
	
	
	char bluetooth1[] = "+++";
	char bluetooth2[] = "\r";
	char bluetooth3[] = "AT+BTUART=115200\r";
	char bluetooth4[] = "AT+BTNAME=HECTOR H30A3\r";
	char bluetooth5[] = "ATZ\r";

	HAL_UART_Transmit(&huart4, (uint8_t*) bluetooth1, strlen(bluetooth1), 100);
	HAL_Delay(100);

	HAL_UART_Transmit(&huart4, (uint8_t*) bluetooth2, strlen(bluetooth2), 100);
	HAL_Delay(100);

	HAL_UART_Transmit(&huart4, (uint8_t*) bluetooth3, strlen(bluetooth3), 100);
	HAL_Delay(100);

	HAL_UART_Transmit(&huart4, (uint8_t*) bluetooth4, strlen(bluetooth4), 100);
	HAL_Delay(100);

	HAL_UART_Transmit(&huart4, (uint8_t*) bluetooth5, strlen(bluetooth5), 100);
	HAL_Delay(300);

	huart4.Instance = UART4;
	huart4.Init.BaudRate = 115200;
	huart4.Init.WordLength = UART_WORDLENGTH_8B;
	huart4.Init.StopBits = UART_STOPBITS_1;
	huart4.Init.Parity = UART_PARITY_NONE;
	huart4.Init.Mode = UART_MODE_TX_RX;
	huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart4.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart4) != HAL_OK) {
		Error_Handler();
	}

	/* USER CODE END UART4_Init 2 */

}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void) {

	/* USER CODE BEGIN USART1_Init 0 */

	/* USER CODE END USART1_Init 0 */

	/* USER CODE BEGIN USART1_Init 1 */

	/* USER CODE END USART1_Init 1 */
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 115200;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_CTS;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART1_Init 2 */

	/* USER CODE END USART1_Init 2 */

}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void) {

	/* USER CODE BEGIN USART2_Init 0 */

	/* USER CODE END USART2_Init 0 */

	/* USER CODE BEGIN USART2_Init 1 */

	/* USER CODE END USART2_Init 1 */
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 115200;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */

}

/**
 * @brief USART3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART3_UART_Init(void) {

	/* USER CODE BEGIN USART3_Init 0 */

	/* USER CODE END USART3_Init 0 */

	/* USER CODE BEGIN USART3_Init 1 */

	/* USER CODE END USART3_Init 1 */
	huart3.Instance = USART3;
	huart3.Init.BaudRate = 115200;
	huart3.Init.WordLength = UART_WORDLENGTH_8B;
	huart3.Init.StopBits = UART_STOPBITS_1;
	huart3.Init.Parity = UART_PARITY_NONE;
	huart3.Init.Mode = UART_MODE_TX_RX;
	huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart3.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart3) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART3_Init 2 */

	/* USER CODE END USART3_Init 2 */

}

/**
 * @brief USART6 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART6_Init(void) {

	/* USER CODE BEGIN USART6_Init 0 */

	/* USER CODE END USART6_Init 0 */

	/* USER CODE BEGIN USART6_Init 1 */

	/* USER CODE END USART6_Init 1 */
	husart6.Instance = USART6;
	husart6.Init.BaudRate = 115200;
	husart6.Init.WordLength = USART_WORDLENGTH_8B;
	husart6.Init.StopBits = USART_STOPBITS_1;
	husart6.Init.Parity = USART_PARITY_NONE;
	husart6.Init.Mode = USART_MODE_TX_RX;
	husart6.Init.CLKPolarity = USART_POLARITY_LOW;
	husart6.Init.CLKPhase = USART_PHASE_1EDGE;
	husart6.Init.CLKLastBit = USART_LASTBIT_DISABLE;
	if (HAL_USART_Init(&husart6) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART6_Init 2 */

	/* USER CODE END USART6_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, LED_Pin | beep_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pins : sw0_Pin sw1_Pin sw2_Pin sw3_Pin
	 sw4_Pin sw5_Pin */
	GPIO_InitStruct.Pin = sw0_Pin | sw1_Pin | sw2_Pin | sw3_Pin | sw4_Pin
			| sw5_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : LED_Pin beep_Pin */
	GPIO_InitStruct.Pin = LED_Pin | beep_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
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
