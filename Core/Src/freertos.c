/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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
#include "gpio.h"
#include "GUI.h"
#include "tim.h"
#include "DS_18B20.h"
#include "MPU6050.h"
#include "Display_3D.h"
#include "comm.h"
#include "ESP01.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "w25qxx.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
extern GUI_FLASH const GUI_FONT GUI_FontHZ_KaiTi_20;//外部声明
extern GUI_FLASH const GUI_FONT GUI_FontHZ_KaiTi_12;
extern GUI_FLASH const GUI_FONT GUI_FontHZ_KaiTi_16;
extern GUI_FLASH const GUI_FONT GUI_FontHZ_SimSun_12;
extern GUI_CONST_STORAGE GUI_BITMAP bmfcz;
extern GUI_CONST_STORAGE GUI_BITMAP bmlch;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
#define WS_LOGO 0
#define WS_GUI1 1
#define WS_GUI2 2
#define WS_GUI3 3
#define WS_GUI4 4   // 定义五个页面

#define SetADDR1 10 * 4096 
#define SetADDR2 9 * 4096
#define SetADDR3 8 * 4096
#define SetADDR4 7 * 4096

int g_ws = WS_LOGO;
uint32_t intick = 0;

volatile float temp = 0; //温度值
float tempLmt = 32.0f;   //温度上限

uint32_t beeptick = 0;

uint8_t tempwarn = 0;//温度报警
uint8_t mpuwarn = 0;//震动报警
uint32_t warntick = 0;//报警时间戳
uint8_t warnright = 0;//报警状态
uint8_t g_mpustep = 0; //震动监测灵敏度
uint8_t g_warntime = 30;//报警时长
uint8_t g_paridx = 0; //参数索引

uint32_t keytime = 0;
uint8_t pageidx = 0;//界面内页面切换

uint16_t g_upstep = 1000; //上传时间间隔
uint8_t g_bUping = 0;



uint32_t scantime= 50;//扫描时间间隔

#define MAX_DATA_LEN 77
uint8_t g_fax_data[MAX_DATA_LEN];
uint8_t g_fay_data[MAX_DATA_LEN];
uint8_t g_faz_data[MAX_DATA_LEN];
uint8_t g_temp_data[MAX_DATA_LEN];

#define LINE_FAX 0
#define LINE_FAY 1
#define LINE_FAZ 2
#define LINE_TEMP 3
#define LINE_3D 4
uint8_t g_line_idx = LINE_FAX;


float vTemp[80];
int cTemp = 0;
float vPitch[80];
int cPitch = 0;
float vRoll[80];
int cRoll = 0;
float vYaw[80];
int cYaw = 0;

/* USER CODE END Variables */
/* Definitions for MainTask */
osThreadId_t MainTaskHandle;
const osThreadAttr_t MainTask_attributes = {
  .name = "MainTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for KeyTask */
osThreadId_t KeyTaskHandle;
const osThreadAttr_t KeyTask_attributes = {
  .name = "KeyTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for UartTask */
osThreadId_t UartTaskHandle;
const osThreadAttr_t UartTask_attributes = {
  .name = "UartTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for GUITask */
osThreadId_t GUITaskHandle;
const osThreadAttr_t GUITask_attributes = {
  .name = "GUITask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for DataTask */
osThreadId_t DataTaskHandle;
const osThreadAttr_t DataTask_attributes = {
  .name = "DataTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

void WSLogo(void);
void DrawLogo(void);
void DrawGUI1(void);
void DrawGUI2(void);
void DrawGUI3(void);
void DrawGUI4(void);

void Beep(int time,int tune);
void DispSeg(uint8_t num[4],uint8_t dot);
void BeepDone(void);
int KeyWatch(void);
void InitESP8266(void);

void SaveTempLmt(uint32_t addr,float saveval);
void SaveMpuStep(uint32_t addr,uint8_t saveval);
void SaveWarnTime(uint32_t addr,uint8_t saveval);
void SaveUpStep(uint32_t addr,uint16_t saveval);
float LoadTempLmt(uint32_t addr);
uint8_t LoadMpuStep(uint32_t addr);
uint8_t LoadWarnTime(uint32_t addr);
uint16_t LoadUpStep(uint32_t addr);

/* USER CODE END FunctionPrototypes */

void StartMainTask(void *argument);
void StartKeyTask(void *argument);
void StartUartTask(void *argument);
void StartGUITask(void *argument);
void StartDataTask(void *argument);

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
  /* creation of MainTask */
  MainTaskHandle = osThreadNew(StartMainTask, NULL, &MainTask_attributes);

  /* creation of KeyTask */
  KeyTaskHandle = osThreadNew(StartKeyTask, NULL, &KeyTask_attributes);

  /* creation of UartTask */
  UartTaskHandle = osThreadNew(StartUartTask, NULL, &UartTask_attributes);

  /* creation of GUITask */
  GUITaskHandle = osThreadNew(StartGUITask, NULL, &GUITask_attributes);

  /* creation of DataTask */
  DataTaskHandle = osThreadNew(StartDataTask, NULL, &DataTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartMainTask */
/**
  * @brief  Function implementing the MainTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartMainTask */
void StartMainTask(void *argument)
{
  /* USER CODE BEGIN StartMainTask */
	uint8_t num[4] = {0};//数码管的四个数字
	W25QXX_Init();
	float val1 = LoadTempLmt(SetADDR1);
	uint8_t val2 = LoadMpuStep(SetADDR2);
	uint8_t val3 = LoadWarnTime(SetADDR3);
	uint16_t val4 = LoadUpStep(SetADDR4);
	if(val1 >= 0 && val1 <= 99)
		tempLmt = val1;
	if(val2 >= 0 && val2 <= 9)
		g_mpustep = val2;
	if(val3 >= 0 && val3 <= 60)
		g_warntime = val3;
	if(val4 >= 100 && val4 <= 10000)
		g_upstep = val4;
  /* Infinite loop */
  for(;;)
  {
		switch(g_ws)
		{
			case WS_LOGO:
				WSLogo();
			  break;
			default:
				SetLeds(0);
				break;
		}
		
		//报警功能
		if((tempwarn || mpuwarn) && g_warntime > 0)
		{
			warnright = 1;    //报警状态设1
			if(0 == warntick)     //计时开始       
				warntick = osKernelGetTickCount();
			else if(osKernelGetTickCount() >= warntick + 1000 * g_warntime)     //30s后停止清零
			{
				tempwarn = mpuwarn = 0;
				warntick = 0;
				warnright = 0;
			}
			else         //报警动作
			{
				uint32_t tic = warntick + 1000 * g_warntime - osKernelGetTickCount();
				num[0] = (tic/10000)%10;
				num[1] = (tic/1000)%10;
				num[2] = (tic/100)%10;
				num[3] = (tic/10)%10;
				
        if(KeyWatch())   				//监测4个按键同时按下
			  { 
				  if(0 == keytime)
						keytime = osKernelGetTickCount();
				  else if(osKernelGetTickCount() >= keytime + 3000)
				  {
						tempwarn = mpuwarn = 0;
				    warntick = 0;
				    warnright = 0;
			      num[0] = num[1] = num[2] = num[3] = ' ';	
            keytime = 0;
            tempLmt = 40;
            g_mpustep = 0;					
				  }			
			  } 					
				//温度报警时
				if(tempwarn == 1)
				{
					  Beep(100,num[2]);
			  }
				//震动报警时
				else if(mpuwarn == 1)
				{
					Beep(100,num[3]%5);
				}    
			}  
		}
		else
			num[0] = num[1] = num[2] = num[3] = ' ';
		
		//num[0]=num[1]=num[2]=num[3]=1;//11.11
		DispSeg(num,2);//小数位两位
		BeepDone();
		
    osDelay(1);
  }
  /* USER CODE END StartMainTask */
}

/* USER CODE BEGIN Header_StartKeyTask */
/**
* @brief Function implementing the KeyTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartKeyTask */
void StartKeyTask(void *argument)
{
  /* USER CODE BEGIN StartKeyTask */
  /* Infinite loop */
  for(;;)
  {
		uint8_t key = ScanKey();
		if(key > 0)
			printf("%02x\n",key);
		
		switch(g_ws)
		{
			case WS_LOGO:
				if(KEY5 == key)
				{
					g_ws = WS_GUI1;
					intick = 0;
				}
				break;
			case WS_GUI1:
				if(!warnright)
				{
				if(KEY1 == key)
					g_ws = WS_GUI4;
				else if(KEY4 == key)
					g_ws = WS_GUI2;
				else if(KEY2 == key)
				{
					if(pageidx > 0)
						--pageidx;
				}
				else if(KEY3 == key)
				{
					if(pageidx < 2)
						++pageidx;
				}
				else if(KEY6 == key)
					  g_ws = WS_LOGO;
			  }
				break;
			case WS_GUI2:
				if(!warnright)
				{
					if(KEY1 == key)
						g_ws = WS_GUI1;
					else if(KEY4 == key)
					{
						g_ws = WS_GUI3;
						InitESP8266();
					}
					else if(KEY2 == key)
					{
						if(g_line_idx > LINE_FAX)
							--g_line_idx;
					}
					else if(KEY3 == key)
					{
						if(g_line_idx < LINE_3D)
							++g_line_idx;
					}
					else if(KEY5==key)
							{
								if(scantime==1000)
									scantime=50;
								else
									scantime=1000;
							} 
					 else if(KEY6 == key)
						g_ws = WS_LOGO;
					break;
			 }
			case WS_GUI3:
				if(!warnright)
				{
					if(KEY1 == key)
						g_ws = WS_GUI2;
					else if(KEY2 == key)
						g_bUping = 1;
					else if(KEY3 == key)
					{
						g_bUping = 0;
					}
					else if(KEY4 == key)
						g_ws = WS_GUI4;
					else if(KEY5 == key && 0 == pageidx)
					{
						if(0 == pageidx)
						{
							InitESP8266();
						}
					}
					else if(KEY6 == key)
						g_ws = WS_LOGO;
					break;
			  }
			case WS_GUI4:
				if(!warnright)
				{
					if(KEY1 == key)
					{
						g_ws = WS_GUI3;
						InitESP8266();
					}
					else if(KEY4 == key)
						g_ws = WS_GUI1;
					else if(KEY6 == key)
						g_ws = WS_LOGO;
					else if(KEY5 == key)
					{
						++g_paridx;
						g_paridx %= 4;
					}
					else if(KEY2 == key)
					{
						if(0 == g_paridx)
						{
							if(tempLmt > 0)
							{
								tempLmt -= 1;
								SaveTempLmt(SetADDR1,tempLmt);
							}
						}
						if(1 == g_paridx)
						{
							if(g_mpustep > 0)
							{
								--g_mpustep;
								SaveMpuStep(SetADDR2,g_mpustep);
							}
						}
						if(2 == g_paridx)
						{
							if(g_warntime > 0)
							{
								--g_warntime;
								SaveWarnTime(SetADDR3,g_warntime);
							}
						}
						if(3 == g_paridx)
						{
							if(g_upstep > 100)
							{
								g_upstep -= 100;
								SaveUpStep(SetADDR4,g_upstep);
							}
						}
					}
					else if(KEY3 == key)
					{
						if(0 == g_paridx)
						{
							if(tempLmt < 90)
							{
								tempLmt += 1;
								SaveTempLmt(SetADDR1,tempLmt);
							}
						}
						if(1 == g_paridx)
						{
							if(g_mpustep < 9)
							{
								++g_mpustep;
								SaveMpuStep(SetADDR2,g_mpustep);
							}
						}
						if(2 == g_paridx)
						{
							if(g_warntime < 60)
							{
								++g_warntime;
								SaveWarnTime(SetADDR3,g_warntime);
							}
						}
						if(3 == g_paridx)
						{
							if(g_upstep < 10000)
							{
								g_upstep += 100;
								SaveUpStep(SetADDR4,g_upstep);
							}
						}					
					}
					break;
				default:
					if(KEY6 == key)
						g_ws = WS_LOGO;
					break;
			}
		}
    osDelay(1);
  }
  /* USER CODE END StartKeyTask */
}

/* USER CODE BEGIN Header_StartUartTask */
/**
* @brief Function implementing the UartTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartUartTask */
void StartUartTask(void *argument)
{
  /* USER CODE BEGIN StartUartTask */
	StartRecvUart1();
	ESP_Init();
	
  /* Infinite loop */
  for(;;)
  {
		if(recv1_len>0)
		{
			printf("%s",recv1_buff);
			recv1_len = 0;
		}
		
		ESP_Proc();
		if(esp8266.recv_len>0)
		{
			printf("%s",esp8266.recv_data);
			
			char *pb = (char *)(esp8266.recv_data);
			char buf[40];
			if('Q' == pb[0] && 'P' == pb[1] && 'A' == pb[2] && 'R' == pb[3])
			{
				//查询参数，应答
				sprintf(buf,"P1:%.0f,P2:%d,P3:%d,P4:%d\n",tempLmt,g_mpustep,g_warntime,g_upstep);
				USendStr(&huart6,(uint8_t*)buf,strlen(buf));
			}
			else if('P' == pb[0] && '1' == pb[1] && ':' == pb[2])
			{
				//参数设置
				tempLmt = atof(pb + 3);
				pb = strstr(pb,"P2:");
				if(pb)
				{
					g_mpustep = atoi(pb + 3);
					pb = strstr(pb,"P3:");
					if(pb)
					{
						g_warntime = atoi(pb + 3);
						pb = strstr(pb,"P4:");
					  if(pb)
					  {
							g_upstep = atoi(pb + 3);
							SaveTempLmt(SetADDR1,tempLmt);
							SaveMpuStep(SetADDR2,g_mpustep);
							SaveWarnTime(SetADDR3,g_warntime);
							SaveUpStep(SetADDR4,g_upstep);
							USendStr(&huart6,(uint8_t *)("PARA SET OK\n"),12);
						}
					}
				}
			}
			else if('O' == pb[0] && 'P' == pb[1] && 'E' == pb[2] && 'N' == pb[3])
			{
				//打开数据上传
				g_bUping = 1;
				USendStr(&huart6,(uint8_t *)("UP OPEN OK\n"),12);
			}
			else if('C' == pb[0] && 'L' == pb[1] && 'O' == pb[2] && 'S' == pb[3] && 'E' == pb[4])
			{
				//关闭数据上传
				g_bUping = 0;
				USendStr(&huart6,(uint8_t *)("UP CLOSE OK\n"),12);
			}
			esp8266.recv_len = 0;
		}
    osDelay(1);
  }
  /* USER CODE END StartUartTask */
}

/* USER CODE BEGIN Header_StartGUITask */
/**
* @brief Function implementing the GUITask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartGUITask */
void StartGUITask(void *argument)
{
  /* USER CODE BEGIN StartGUITask */
	GUI_Init();//初始化
  /* Infinite loop */
  for(;;)
  {
		switch(g_ws)
		{
			case WS_LOGO:  DrawLogo();  break;
			case WS_GUI1:  DrawGUI1();  break;
			case WS_GUI2:  DrawGUI2();  break;
			case WS_GUI3:  DrawGUI3();  break;
			case WS_GUI4:  DrawGUI4();  break;
			default:
				break;
		}
		osDelay(100);
  }
  /* USER CODE END StartGUITask */
}

/* USER CODE BEGIN Header_StartDataTask */
/**
* @brief Function implementing the DataTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartDataTask */
void StartDataTask(void *argument)
{
  /* USER CODE BEGIN StartDataTask */
	osDelay(1000);
	
	ds18b20_init();
	
	uint8_t mpuok = MPU_init();//初始化
	uint8_t idx = 0;
	uint8_t idy = 0;
	
	uint8_t cnt = 0;
	while(cnt++ <3 && !mpuok)//循环三次,不成功再次编译
	{
		osDelay(500);
		mpuok = MPU_init();
	}
	
	uint32_t dstick = 0;
	uint32_t mputick = 0;
	uint32_t uptick = 0;
	
	int warncnt = 0;//检测震动次数
	int tempcnt = 0;//检测温度超标次数
	float ft = 0;
	uint32_t newdstick=0;
	
  /* Infinite loop */
  for(;;)
  {
		
			if(osKernelGetTickCount()>=dstick+500)
		  {
			  dstick=osKernelGetTickCount();
			  ft = ds18b20_read();
		  }		
			if(ft < 125)
			{
				temp = ft;
				if(osKernelGetTickCount()>=newdstick+scantime)
			  {	

				  newdstick=osKernelGetTickCount();
				  g_temp_data[idy] = (35-temp) * 40 / 10 + 12;
				  ++ idy;
					
				  if(idy >= MAX_DATA_LEN)
				  {
					  memcpy(g_temp_data,g_temp_data + 1,MAX_DATA_LEN - 1);
					  idy = MAX_DATA_LEN - 1;
				  }
	
				  if(temp >= tempLmt)       //温度超限
				  {
						printf("temp:%.1f\r\n",temp);
					  if(++tempcnt >= 10)
					  tempwarn = 1;
				   }
				  else 
					  tempcnt = 0;
				}
			}
		
		
		if(mpuok)
		{
			if(osKernelGetTickCount()>= mputick + 50)
		  {
			  
				MPU_getdata();//数据类型见mpu.c
			}
		  if(osKernelGetTickCount()>=mputick+scantime)
			{
				dstick = osKernelGetTickCount();
				mputick=osKernelGetTickCount();
				g_fax_data[idx] = 32 - fAX * 20 / 90;
				g_fay_data[idx] = 32 - fAY * 20 / 180;
				g_faz_data[idx] = 32 - fAZ * 20 / 180;
				++ idx;
				if(idx >= MAX_DATA_LEN)
				{
					memcpy(g_fax_data,g_fax_data + 1,MAX_DATA_LEN - 1);
					memcpy(g_fay_data,g_fay_data + 1,MAX_DATA_LEN - 1);
					memcpy(g_faz_data,g_faz_data + 1,MAX_DATA_LEN - 1);
					
					idx = MAX_DATA_LEN - 1;
				}
				
				
				
				if((gx*gx + gy*gy + gz*gz > 2000 * (10 - g_mpustep)) && g_mpustep > 0)
				{
					warncnt ++;
					if(warncnt >= 0.25 * (100 - g_mpustep * g_mpustep))
					{
						mpuwarn = 1;
						printf("axyz:%6d %6d %6d,gxyz:%6d %6d %6d\n",ax,ay,az,gx,gy,gz);
					}
				}
				else
					warncnt = 0;
			}
		}
		
		
		
    if(g_bUping && esp8266.bconn)
		{
			if(osKernelGetTickCount() >= uptick + g_upstep)
			{
				uptick = osKernelGetTickCount();
				
				char buf[100];
				sprintf(buf,"T:%4.1f, A:%6d %6d %6d,G:%6d %6d %6d, F:%5.1f %5.1f %5.1f, W:%d\n",
				temp,ax,ay,az,gx,gy,gz,fAX,fAY,fAZ,(tempwarn ? 1 : 0) + (mpuwarn ? 2 : 0));
				USendStr(&huart6,(uint8_t*)buf,strlen(buf));
			}
		}
		osDelay(1);
  }
	
  /* USER CODE END StartDataTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void WSLogo(void)//LOGO界面动作
{
	//intick:声明静态变量,时间戳osKernelGetTickCount为当前时刻
	static uint32_t tick = 0;
	static uint8_t leds_sta = 0x00;
	if(0 == intick)
		intick = osKernelGetTickCount();
	else 
	{
		if(osKernelGetTickCount() >= intick+5000)//延时5s进入gui1
		{
			intick = 0;
			g_ws = WS_GUI1;
			Beep(500,3);
		}
	}
	SetLeds(leds_sta);  //点灯
	if(osKernelGetTickCount() >= tick + 500)
	{
		leds_sta = ~leds_sta; //每500ms取反
		tick = osKernelGetTickCount();
	}
}

void DrawLogo(void)//LOGO界面
{	
	GUI_Clear();
	static uint32_t tick = 0;
	if(0 == intick)
		intick = osKernelGetTickCount();
	else 
	{
		if(osKernelGetTickCount() <= intick+2000)
		{
	    GUI_SetFont(&GUI_FontHZ_KaiTi_20);
	    GUI_DispStringHCenterAt("防火、防盗远\n程监测器设计",64,12);
	    GUI_Update();//刷新
		}
		else if(osKernelGetTickCount() <= intick+4000)
		{
			GUI_Clear();
	    GUI_SetFont(&GUI_FontHZ_KaiTi_16);
	    GUI_DispStringAt("成员1：李承翰\n       18041817\n成员2：冯粲之 \n       18041812",0,0);
      GUI_Update();			
		}
		else if(osKernelGetTickCount() <= intick+5000)
		{
			GUI_Clear();
	    GUI_DrawBitmap(&bmlch,0,0);
			GUI_DrawBitmap(&bmfcz,64,0);
	    GUI_Update();	
		}
		else
			intick = 0;
	}

}

void DrawGUI1(void) //实时监测界面
{
	GUI_Clear();
	GUI_SetFont(&GUI_FontHZ_SimSun_12);
	GUI_SetColor(GUI_COLOR_BLACK);
	GUI_DispStringAt("实时监测",0,0);
	GUI_SetColor(GUI_COLOR_WHITE);
	GUI_DispStringAt("数据曲线",0,13);
	GUI_DispStringAt("无线通信",0,26);
	GUI_DispStringAt("参数设置",0,39);
	GUI_DispStringAt("K1︽  K2《 K3》  K4︾",0,52);
	
	GUI_DrawHLine(52,0,128);
	GUI_DrawVLine(48,0,52);
	
	char buf[20];
	if(0 == pageidx)
	{
	GUI_DispStringAt("当前温度:",50,0);
	GUI_DispStringAt("震动报警:",50,26);
	sprintf(buf,"%.1f℃",temp);
	GUI_DispStringAt(buf,90,13);
	GUI_DispStringAt(mpuwarn ? "是" : "否",90,39);
	}
	else if(1 == pageidx)
	{
		sprintf(buf,"ax:%6d",ax);
		GUI_DispStringAt(buf,50,0);
		if(ax > 0)
		  GUI_FillRect(70,13,70+ax*55/32768,16);
		else if(ax < 0)
			GUI_FillRect(70,13,70-ax*55/32768,16);
		sprintf(buf,"ay:%6d",ay);
		GUI_DispStringAt(buf,50,17);
		if(ay > 0)
		  GUI_FillRect(70,30,70+ay*55/32768,33);
		else if(ay < 0)
			GUI_FillRect(70,30,70-ay*55/32768,33);
		sprintf(buf,"az:%6d",az);
		GUI_DispStringAt(buf,50,34);
				if(az > 0)
		  GUI_FillRect(70,47,70+az*55/32768,50);
		else if(az < 0)
			GUI_FillRect(70,47,70-az*55/32768,50);			
	}
	else if(2 == pageidx)
	{
		sprintf(buf,"gx:%6d",gx);
		GUI_DispStringAt(buf,50,0);
		if(gx > 0)
		  GUI_FillRect(70,13,70+gx*55/32768,16);
		else if(gx < 0)
			GUI_FillRect(70,13,70-gx*55/32768,16);
		sprintf(buf,"gy:%6d",gy);
		GUI_DispStringAt(buf,50,17);
		if(gy > 0)
		  GUI_FillRect(70,30,70+gy*55/32768,33);
		else if(gy < 0)
			GUI_FillRect(70,30,70-gy*55/32768,33);
		sprintf(buf,"gz:%6d",gz);
		GUI_DispStringAt(buf,50,34);
				if(gz > 0)
		  GUI_FillRect(70,47,70+gz*55/32768,50);
		else if(gz < 0)
			GUI_FillRect(70,47,70-gz*55/32768,50);				
	}
	GUI_Update();
}

void DrawGUI2(void)//数据曲线界面
{
	char str[30];
	
	GUI_Clear();
	GUI_SetFont(&GUI_FontHZ_SimSun_12);
	GUI_DispStringAt("实时监测",0,0);	
	GUI_SetColor(GUI_COLOR_BLACK);
	GUI_DispStringAt("数据曲线",0,13);
	GUI_SetColor(GUI_COLOR_WHITE);
	GUI_DispStringAt("无线通信",0,26);
	GUI_DispStringAt("参数设置",0,39);
	GUI_DispStringAt("K1︽  K2《 K3》  K4︾",0,52);
	
	GUI_DrawHLine(52,0,128);
	GUI_DrawVLine(48,0,52);
	
	uint8_t i;
	
	int sw = 128-48;
	int sh = 64-12-12;
	int ox = 48;
	int oy = 12 + sh;
	switch(g_line_idx)
	{
		case LINE_FAY:
			sprintf(str,"俯仰角：%.1f",fAY);
		  for(i = 0; i< MAX_DATA_LEN - 1; ++i)
		  {
				GUI_DrawLine(51 + i, g_fay_data[i],51 + i +1,g_fay_data[i+1]);
		  }
			break;
		case LINE_FAZ:
			sprintf(str,"航向角：%.1f",fAZ);
		  for(i = 0; i< MAX_DATA_LEN - 1; ++i)
		  {
				GUI_DrawLine(51 + i, g_faz_data[i],51 + i +1,g_faz_data[i+1]);
		  }
			break;
		case LINE_TEMP:
			sprintf(str,"温度：%.1f",temp);
		  for(i = 0; i< MAX_DATA_LEN - 1; ++i)
		  {
				GUI_DrawLine(51 + i, g_temp_data[i],51 + i +1,g_temp_data[i+1]);
		  }
      for(i = 0;i<(MAX_DATA_LEN-1)/2;++i)
			{
				GUI_DrawPixel(51+i*2,((35-tempLmt) * 40 / 10 + 12));
			}
			break;
		case LINE_3D:
			ox = (48 + 128) / 2;
		  oy = (12 + 40) / 2 + 2;
		  RateCube(fAY,fAX,fAZ,GUI_COLOR_WHITE,ox,oy);
			break;
		default:
			sprintf(str,"横滚角：%.1f",fAX);
		  for(i = 0; i< MAX_DATA_LEN - 1; ++i)
		  {
				GUI_DrawLine(51 + i, g_fax_data[i],51 + i +1,g_fax_data[i+1]);
		  }
			break;
	}
	GUI_DispStringAt(str,51,0);

	GUI_Update();
}

void DrawGUI3(void)//无线通信界面
{
	char buf[30];
	
	GUI_Clear();
	GUI_SetFont(&GUI_FontHZ_SimSun_12);
	GUI_DispStringAt("实时监测",0,0);
	GUI_DispStringAt("数据曲线",0,13);
	GUI_SetColor(GUI_COLOR_BLACK);	
	GUI_DispStringAt("无线通信",0,26);
	GUI_SetColor(GUI_COLOR_WHITE);	
	GUI_DispStringAt("参数设置",0,39);
	GUI_DispStringAt("K1︽  K2《 K3》  K4︾",0,52);
	
	GUI_DispStringAt((char*)(esp8266.ssid),50,0);
	GUI_DispStringAt(TCP_SERVER,50,12);
	sprintf(buf,"端口：%d",TCP_PORT);
	GUI_DispStringAt(buf,50,24);
	GUI_DispStringAt(esp8266.bconn ? "OK":"ERR",50,36);
	GUI_DispStringAt(g_bUping ? "上传中":"未上传",80,36);
	
	GUI_DrawHLine(52,0,128);
	GUI_DrawVLine(48,0,52);

	GUI_Update();
}

void DrawGUI4(void)//参数设置界面
{
	char buf[20];
	
	GUI_Clear();
	GUI_SetFont(&GUI_FontHZ_SimSun_12);
	GUI_DispStringAt("实时监测",0,0);
	GUI_DispStringAt("数据曲线",0,13);
	GUI_DispStringAt("无线通信",0,26);
	GUI_SetColor(GUI_COLOR_BLACK);	
	GUI_DispStringAt("参数设置",0,39);
	GUI_SetColor(GUI_COLOR_WHITE);	
	GUI_DispStringAt("K1︽  K2《 K3》  K4︾",0,52);
	
	GUI_DrawHLine(52,0,128);
	GUI_DrawVLine(48,0,52);
	
	GUI_DispStringAt("温度上限:",50,0);	
	GUI_DispStringAt("震动灵敏度:",50,13);	
	GUI_DispStringAt("报警时长:",50,26);		
	GUI_DispStringAt("上传间隔:",50,39);	
	
	sprintf(buf,"%.0f℃",tempLmt);
	if(0 == g_paridx)
		GUI_SetColor(GUI_COLOR_BLACK);
	GUI_DispStringAt(buf,104,0);
	GUI_SetColor(GUI_COLOR_WHITE);	
	
	sprintf(buf,"%d",g_mpustep);
	if(1 == g_paridx)
		GUI_SetColor(GUI_COLOR_BLACK);
	GUI_DispStringAt(buf,116,13);
	GUI_SetColor(GUI_COLOR_WHITE);	

	sprintf(buf,"%dS",g_warntime);
	if(2 == g_paridx)
		GUI_SetColor(GUI_COLOR_BLACK);
	GUI_DispStringAt(buf,104,26);
	GUI_SetColor(GUI_COLOR_WHITE);	

	sprintf(buf,"%.1fS",g_upstep / 1000.0f);
	if(3 == g_paridx)
		GUI_SetColor(GUI_COLOR_BLACK);
	GUI_DispStringAt(buf,104,39);
	GUI_SetColor(GUI_COLOR_WHITE);	
	
	GUI_Update();
}
//gui.h中查询led函数

void Beep(int time,int tune)
{
	static uint16_t TAB[] = {494,523,588,660,698,784,880,988};
	HAL_TIM_Base_Start(&htim3);//tim.c
	
	if(tune >= 1 && tune <= 7)//7个音阶
	{
		int pre = 1000000 / TAB[tune];
		__HAL_TIM_SET_AUTORELOAD(&htim3,pre);//预装载
		__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,pre/2);//占空比，超过一半翻转
		
		beeptick = osKernelGetTickCount() + time;
		HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
	}
}


void BeepDone(void)//鸣叫时间
{
	if(beeptick >0 && osKernelGetTickCount() >= beeptick)
	{
		beeptick = 0;
		HAL_TIM_PWM_Stop(&htim3,TIM_CHANNEL_1);//停止
	}
}

void DispSeg(uint8_t num[4],uint8_t dot) //dot 小数点
{
	 for(int i=0;i<4;++i)
	{
		Write595(i,num[i],(dot == (i+1)) ? 1: 0);
		osDelay(1);
	}
}

int KeyWatch(void) //监测4个按键是否都按下
{
	int keyw1,keyw2,keyw3,keyw4,keycnt;
	keyw1 = keyw2 = keyw3 = keyw4 = keycnt = 0;
	if(HAL_GPIO_ReadPin(K1_GPIO_Port, K1_Pin) == GPIO_PIN_RESET)
		keyw1 = 1;
	else
		keyw1 = 0;
	if(HAL_GPIO_ReadPin(K2_GPIO_Port, K2_Pin) == GPIO_PIN_RESET)
		keyw2 = 1;
	else 
		keyw2 = 0;
	if(HAL_GPIO_ReadPin(K3_GPIO_Port, K3_Pin) == GPIO_PIN_RESET)
		keyw3 = 1;
	else 
		keyw3 = 0;
	if(HAL_GPIO_ReadPin(K4_GPIO_Port, K4_Pin) == GPIO_PIN_RESET)
		keyw4 = 1;
	else 
		keyw4 = 0;
	keycnt = keyw1+keyw2+keyw3+keyw4;
	if(keycnt == 4)
	{
		return 1;
	}
	else 
		return 0;
}

void InitESP8266(void)
{
	ESP_SetCIPMode(0);
	if(ESP_IsOK())
	{
		ESP_SetMode(3);
		ESP_GetSSID();
		
		if(ESP_JoinAP(AP_NAME,AP_PSW))
		{
			ESP_GetIPAddr();
			ESP_SetTCPServer(0,0);
			ESP_SetCIPMux(0);
			printf("Station ip :%s\n",esp8266.st_addr);
			
			if(ESP_ClientToServer(TCP_SERVER,TCP_PORT))
			{
				ESP_SetCIPMode(1);
				printf("ESP init OK\n");
			}
		}
	}
}

void SaveTempLmt(uint32_t addr,float saveval)
{
	W25QXX_Write((uint8_t *)(&saveval),addr,sizeof(saveval));
}
void SaveMpuStep(uint32_t addr,uint8_t saveval)
{
	W25QXX_Write((uint8_t *)(&saveval),addr,sizeof(saveval));	
}
void SaveWarnTime(uint32_t addr,uint8_t saveval)
{
	W25QXX_Write((uint8_t *)(&saveval),addr,sizeof(saveval));	
}
void SaveUpStep(uint32_t addr,uint16_t saveval)
{
	W25QXX_Write((uint8_t *)(&saveval),addr,sizeof(saveval));	
}
float LoadTempLmt(uint32_t addr)
{
	float val = 0;
	W25QXX_Read((uint8_t *)(&val),addr,sizeof(val));
	return val;
}
uint8_t LoadMpuStep(uint32_t addr)
{
	uint8_t val = 0;
	W25QXX_Read((uint8_t *)(&val),addr,sizeof(val));
	return val;	
}
uint8_t LoadWarnTime(uint32_t addr)
{
	uint8_t val = 0;
	W25QXX_Read((uint8_t *)(&val),addr,sizeof(val));
	return val;	
}
uint16_t LoadUpStep(uint32_t addr)
{
	uint16_t val = 0;
	W25QXX_Read((uint8_t *)(&val),addr,sizeof(val));
	return val;	
}
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
