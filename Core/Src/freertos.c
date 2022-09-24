/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "fdcan.h"

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
/* USER CODE BEGIN Variables */

volatile uint8_t drive_Switch=0x77;         //驱动板开关机：开0x77；关0x88
volatile float brightness=0;                //LED灯输出亮度0.0-100.0%
volatile uint16_t color_Temperature=4600;      //LED灯输出色温2700k-6500k
volatile uint8_t fan_Ratio=0x64;              //风扇输出参数范围0x00-0x64对应DC0-12V
volatile uint8_t drive_TxData[16]={0};      //发送给驱动板的数组
volatile uint8_t drive_RxData[16]={0};      //从驱动板接受的数组

volatile uint16_t first_electric_current=0; //第一路电流值0000-1000（等效为0.0%-100.0%）
volatile uint16_t second_electric_current=0;//第二路电流值0000-1000（等效为0.0%-100.0%）
volatile uint16_t third_electric_current=0; //第三路电流值0000-1000（等效为0.0%-100.0%）
volatile uint16_t fourth_electric_current=0;//第四路电流值0000-1000（等效为0.0%-100.0%）
volatile uint16_t sum_electric_current=0;   //总的电流值0000-4000
volatile uint16_t set_LED_power=0;          //设定的LED功率值

volatile uint8_t drive_State_Update=pdTRUE;      //驱动状态更新

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for fdcanRxTx */
osThreadId_t fdcanRxTxHandle;
const osThreadAttr_t fdcanRxTx_attributes = {
  .name = "fdcanRxTx",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal1,
};
/* Definitions for driveUpdata */
osThreadId_t driveUpdataHandle;
const osThreadAttr_t driveUpdata_attributes = {
  .name = "driveUpdata",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

unsigned char even_parity(uint8_t *data, uint16_t start_i, uint16_t start_j);

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void fdcanRxTxTask01(void *argument);
void driveUpdataTask01(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of fdcanRxTx */
  fdcanRxTxHandle = osThreadNew(fdcanRxTxTask01, NULL, &fdcanRxTx_attributes);

  /* creation of driveUpdata */
  driveUpdataHandle = osThreadNew(driveUpdataTask01, NULL, &driveUpdata_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
		//SPI1_TxData();
		//LCD_ShowString(0,40,"FPS:",WHITE,BLACK,24,0);
		//LCD_Fill(0,0,LCD_W,LCD_H,WHITE);
		//LCD_Fill(0,0,LCD_W,LCD_H,BLACK);
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_fdcanRxTxTask01 */
/**
* @brief Function implementing the fdcanRxTx thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_fdcanRxTxTask01 */
void fdcanRxTxTask01(void *argument)
{
  /* USER CODE BEGIN fdcanRxTxTask01 */
  /* Infinite loop */
  for(;;)
  {
		//先收后发
		Can_receivedata(CanRxData, &can_RxID);
		
		//while (drive_State_Update==pdTRUE) //没有按键输入任务 暂时关闭判断
    //{
      //drive_State_Update=pdFALSE;
      //将drive_TxData数组通过CAN发送给下位机
      for (uint16_t i = 0; i < 2; i++)
      {
        for (uint16_t j = 0; j < 8; j++)
        {
          /* code */
          if(i==0) {CanTxData[j]=drive_TxData[j];}
          else {CanTxData[j]=drive_TxData[j+8];}
        }
        Can_senddata(CanTxData,CANTXDATALONG);
        osDelay(10);//避免帧冲突
      }
    //}
		
    osDelay(1);
  }
  /* USER CODE END fdcanRxTxTask01 */
}

/* USER CODE BEGIN Header_driveUpdataTask01 */
/**
* @brief Function implementing the driveUpdata thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_driveUpdataTask01 */
void driveUpdataTask01(void *argument)
{
  /* USER CODE BEGIN driveUpdataTask01 */
  /* Infinite loop */
  for(;;)
  {
		float cold_Out=0;                   //定义冷色温输出值0.0-100%
    float warm_Out=0;                   //定义暖色温输出值0.0-100%
    uint8_t even_parity_flag = 0; //偶校验标志位
    uint16_t first_electric_current=0;  //第一路电流值0000-1000（等效为0.0%-100.0%）
    uint16_t second_electric_current=0; //第二路电流值0000-1000（等效为0.0%-100.0%）
    uint16_t third_electric_current=0;  //第三路电流值0000-1000（等效为0.0%-100.0%）
    uint16_t fourth_electric_current=0; //第四路电流值0000-1000（等效为0.0%-100.0%）
    uint16_t sum_electric_current=0;    //总的电流值0000-4000
    uint16_t set_LED_power=0;           //设定的LED功率值

    //输入LED灯亮度与色温，计算CCT模式冷暖色温两路输出
    cct(brightness,color_Temperature,&cold_Out,&warm_Out);

    //将冷暖输出进行格式转化并赋四路电流数值（这里假设第一路是冷色温）
    first_electric_current=(cold_Out*1000)+0.5;//+0.5实现float类型转换为uint16_t时四舍五入
    second_electric_current=(cold_Out*1000)+0.5;
    third_electric_current=(warm_Out*1000)+0.5;
    fourth_electric_current=(warm_Out*1000)+0.5;

    //安全检查
    //对四路电流输出值进行限幅
    if(first_electric_current>1000)    {first_electric_current=1000;}
    else if(first_electric_current<0)  {first_electric_current=0;}
    if(second_electric_current>1000)   {second_electric_current=1000;}
    else if(second_electric_current<0) {second_electric_current=0;}
    if(third_electric_current>1000)    {third_electric_current=1000;}
    else if(third_electric_current<0)  {third_electric_current=0;}
    if(fourth_electric_current>1000)   {fourth_electric_current=1000;}
    else if(fourth_electric_current<0) {fourth_electric_current=0;}

    //计算主控板设定的输出功率
    sum_electric_current=first_electric_current+second_electric_current+third_electric_current+fourth_electric_current;
    //判断LED功率是否超出1200W，超出则不输出
    if(sum_electric_current>2000){first_electric_current=second_electric_current=third_electric_current=fourth_electric_current=0;}
    set_LED_power= sum_electric_current *2400/(1000*4);

    drive_TxData[0] =0xaa; //第一帧帧头0xaa
    drive_TxData[1] =(first_electric_current / 256);//第一路恒流输出高八位0x00-0x0a
    drive_TxData[2] =(first_electric_current % 256);//第一路恒流输出低八位0x00-0x63
    drive_TxData[3] =(second_electric_current / 256);//第二路恒流输出高八位0x00-0x0a
    drive_TxData[4] =(second_electric_current % 256);//第二路恒流输出低八位0x00-0x63
    drive_TxData[5] =drive_Switch;//开0x77关0x88机
    even_parity_flag=even_parity(drive_TxData, 0, 5);
    drive_TxData[6] =even_parity_flag;//第一帧前6位偶校验
    drive_TxData[7] =0xfe;//第一帧帧尾

    drive_TxData[8] =0xbb;//第二帧帧头0xbb
    drive_TxData[9] =(third_electric_current / 256);//第三路恒流输出高八位0x00-0x0a
    drive_TxData[10]=(third_electric_current % 256);//第三路恒流输出低八位0x00-0x63
    drive_TxData[11]=(fourth_electric_current / 256);//第四路恒流输出高八位0x00-0x0a
    drive_TxData[12]=(fourth_electric_current % 256);//第四路恒流输出低八位0x00-0x63
    drive_TxData[13]=fan_Ratio;                     //第五路DC0-12V输出0x00-0x64
    even_parity_flag=even_parity(drive_TxData, 8, 13);
    drive_TxData[14]=even_parity_flag;//第二帧前6位偶校验
    drive_TxData[15]=0xff;//第二路帧尾0xff
		
    osDelay(1);
  }
  /* USER CODE END driveUpdataTask01 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/*
* 偶校验2
* uint8_t *data 输入数组 
* uint16_t start_i 开始校验数组序号（包含）
* uint16_t start_j 结束校验数组序号（包含）
*/

unsigned char even_parity(uint8_t *data, uint16_t start_i, uint16_t start_j)
{
    unsigned char parity = 0;
    unsigned char n_bits = 0;
    uint16_t data_long = 0;
    unsigned char value = 0;

    uint16_t i=start_i;
    data_long = start_j - start_i + 1;

    for (i=start_i; i < start_i+data_long; i++)
    {
      /* code */
      value = data[i];
			n_bits = sizeof(data[start_i]) * 8;
      while( n_bits >0)
      {
          parity += value & 1;
          value >>=1;
          n_bits -=1;
      }
    }
    
    /*
    * 当实际数据中“1”的个数为偶数，校验位是“1”，否则校验位是“0”
    */
    return (parity % 2) != 0;
	
}

/* USER CODE END Application */

