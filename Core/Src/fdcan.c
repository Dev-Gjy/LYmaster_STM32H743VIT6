/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    fdcan.c
  * @brief   This file provides code for the configuration
  *          of the FDCAN instances.
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
#include "fdcan.h"

/* USER CODE BEGIN 0 */

const int16_t CANTXDATALONG = 8;  //CAN发送数组长度
const int16_t CANRXDATALONG = 8;  //CAN接受数组长度
volatile uint8_t CanTxData[CANTXDATALONG]={0};  //CAN发送数组
volatile uint8_t CanRxData[CANRXDATALONG]={0};  //CAN接受数组
volatile uint16_t can_RxID=0;	//CAN接收ID
volatile uint8_t drive_State=0;       //驱动板状态码
volatile uint8_t drive_Temperature=0; //驱动板温度
volatile uint8_t drive_Power=0;       //驱动板功率负载

/* USER CODE END 0 */

FDCAN_HandleTypeDef hfdcan1;

/* FDCAN1 init function */
void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */
	
	FDCAN_FilterTypeDef FDCAN1_RXFilter;
	
  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = ENABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 10;
  hfdcan1.Init.NominalSyncJumpWidth = 1;
  hfdcan1.Init.NominalTimeSeg1 = 7;
  hfdcan1.Init.NominalTimeSeg2 = 2;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 1;
  hfdcan1.Init.DataTimeSeg2 = 1;
  hfdcan1.Init.MessageRAMOffset = 0;
  hfdcan1.Init.StdFiltersNbr = 0;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.RxFifo0ElmtsNbr = 32;
  hfdcan1.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.RxFifo1ElmtsNbr = 0;
  hfdcan1.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.RxBuffersNbr = 0;
  hfdcan1.Init.RxBufferSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.TxEventsNbr = 0;
  hfdcan1.Init.TxBuffersNbr = 0;
  hfdcan1.Init.TxFifoQueueElmtsNbr = 16;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  hfdcan1.Init.TxElmtSize = FDCAN_DATA_BYTES_8;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */
	
    //配置RX滤波器   
    FDCAN1_RXFilter.IdType=FDCAN_STANDARD_ID;                       //标准ID
    FDCAN1_RXFilter.FilterIndex=0;                                  //滤波器索引                   
    FDCAN1_RXFilter.FilterType=FDCAN_FILTER_MASK;                   //滤波器类型
    FDCAN1_RXFilter.FilterConfig=FDCAN_FILTER_TO_RXFIFO0;           //过滤器0关联到FIFO0  
    FDCAN1_RXFilter.FilterID1=0x0000;                               //32位ID
    FDCAN1_RXFilter.FilterID2=0x0000;                               //如果FDCAN配置为传统模式的话，这里是32位掩码
    if(HAL_FDCAN_ConfigFilter(&hfdcan1,&FDCAN1_RXFilter)!=HAL_OK) 	//滤波器初始化
		{
			Error_Handler();
		}
    HAL_FDCAN_Start(&hfdcan1);                               //开启FDCAN
    HAL_FDCAN_ActivateNotification(&hfdcan1,FDCAN_IT_RX_FIFO0_NEW_MESSAGE,0);

  /* USER CODE END FDCAN1_Init 2 */

}

void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef* fdcanHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(fdcanHandle->Instance==FDCAN1)
  {
  /* USER CODE BEGIN FDCAN1_MspInit 0 */

  /* USER CODE END FDCAN1_MspInit 0 */
    /* FDCAN1 clock enable */
    __HAL_RCC_FDCAN_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**FDCAN1 GPIO Configuration
    PA11     ------> FDCAN1_RX
    PA12     ------> FDCAN1_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN FDCAN1_MspInit 1 */

  /* USER CODE END FDCAN1_MspInit 1 */
  }
}

void HAL_FDCAN_MspDeInit(FDCAN_HandleTypeDef* fdcanHandle)
{

  if(fdcanHandle->Instance==FDCAN1)
  {
  /* USER CODE BEGIN FDCAN1_MspDeInit 0 */

  /* USER CODE END FDCAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_FDCAN_CLK_DISABLE();

    /**FDCAN1 GPIO Configuration
    PA11     ------> FDCAN1_RX
    PA12     ------> FDCAN1_TX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11|GPIO_PIN_12);

  /* USER CODE BEGIN FDCAN1_MspDeInit 1 */

  /* USER CODE END FDCAN1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

//can发送一组数据(固定格式:ID为0X001,标准帧,数据帧)	
//len:数据长度(最大为8),可设置为FDCAN_DLC_BYTES_2~FDCAN_DLC_BYTES_8				     
//msg:数据指针,最大为8个字节.
//返回值:0,成功;
//其他,失败;
uint8_t FDCAN1_Send_Msg(uint8_t* msg,uint32_t len)
{		
		FDCAN_TxHeaderTypeDef fdcan1_TxHeader;
	
    fdcan1_TxHeader.Identifier=0x001;                          //32位ID
    fdcan1_TxHeader.IdType=FDCAN_STANDARD_ID;                  //标准ID
    fdcan1_TxHeader.TxFrameType=FDCAN_DATA_FRAME;              //数据帧
    fdcan1_TxHeader.DataLength=len;                            //数据长度
    fdcan1_TxHeader.ErrorStateIndicator=FDCAN_ESI_ACTIVE;            
    fdcan1_TxHeader.BitRateSwitch=FDCAN_BRS_OFF;               //关闭速率切换
    fdcan1_TxHeader.FDFormat=FDCAN_CLASSIC_CAN;                //传统的CAN模式
    fdcan1_TxHeader.TxEventFifoControl=FDCAN_NO_TX_EVENTS;     //无发送事件
    fdcan1_TxHeader.MessageMarker=0;                           
    
    if(HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,&fdcan1_TxHeader,msg)!=HAL_OK) return 1;//发送
    
		return 0;	
}

//can口接收数据查询
//buf:数据缓存区;	 
//返回值:0,无数据被收到;
//其他,接收的数据长度;
uint8_t FDCAN1_Receive_Msg(uint8_t *buf, uint16_t *Identifier)
{	
		FDCAN_RxHeaderTypeDef fdcan1_RxHeader;
	
		if(HAL_FDCAN_GetRxMessage(&hfdcan1,FDCAN_RX_FIFO0,&fdcan1_RxHeader,buf)!=HAL_OK)return 0;//接收数据
	
		*Identifier = fdcan1_RxHeader.Identifier;
	
		return fdcan1_RxHeader.DataLength>>16;	
}
/*
*	CAN用户态发送函数
*/

void Can_senddata(char *buf, int len)
{
	
	FDCAN1_Send_Msg(buf,FDCAN_DLC_BYTES_8);//FDCAN_DLC_BYTES_8 代表每次发送8个字节
	
}

/*
*	CAN用户态接收函数
*/

void Can_receivedata(char *buf, uint16_t *rx_Identifier)
{
	
	FDCAN1_Receive_Msg(buf, rx_Identifier);//Identifier 存储收到消息的ID号
	
	if (CanRxData[6]==even_parity(CanRxData, 0, 5))//校验成功读取数据
  {
    drive_State=CanRxData[1];       //驱动板状态码
    drive_Temperature=CanRxData[2]; //驱动板温度
    drive_Power=CanRxData[3];       //驱动板功率负载
  }
	
}
/* USER CODE END 1 */
