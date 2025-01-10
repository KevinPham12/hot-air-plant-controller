/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "usb_device.h"
#include "gpio.h"
#include "stdlib.h"
#include "time.h"

/* user Includes */
#include "includes.h"


/* Private variables ---------------------------------------------------------*/
uint8_t RxData[256];
uint32_t data_received = 0;
uint8_t Str4Display[100];


static  OS_TCB   StartupTaskTCB;
static  CPU_STK  StartupTaskStk[APP_CFG_STARTUP_TASK_STK_SIZE];

static  OS_TCB   CommTaskTCB;
static  CPU_STK  CommTaskStk[APP_CFG_COMM_TASK_STK_SIZE];

static  OS_TCB   BtnTaskTCB;
static  CPU_STK  BtnTaskStk[APP_CFG_BTN_TASK_STK_SIZE];

static  OS_TCB   GuiTaskTCB;
static  CPU_STK  GuiTaskStk[APP_CFG_GUI_TASK_STK_SIZE];

static  OS_TCB	 PlantTaskTCB;
static	CPU_STK	 PlantTaskStk[APP_CFG_PLANT_TASK_STK_SIZE];
OS_Q  	PlantInputQ;
OS_Q	PlantTempQ;
OS_Q	PlantVQ;

OS_Q   CommQ;


/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* user private function prototypes */
static  void  AppTaskCreate         (void);
static  void  StartupTask (void  *p_arg);
static  void  CommTask (void  *p_arg);
static  void  BtnTask (void  *p_arg);
static  void  GuiTask (void  *p_arg);

/* added functions for Lab 3 */
static void PlantTask(void *p_arg);
static float TempToVoltage(float temp);
static float VoltageToTemp(float volt);


/**
  * @brief  The application entry point.
  * @retval None
  */
int main(void)
{

  OS_ERR  os_err;

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  BSP_ClkInit();                                              /* Initialize the main clock                            */
  BSP_IntInit();                                              /* Initialize the interrupt vector table.               */
  BSP_OS_TickInit();                                          /* Initialize kernel tick timer                         */

  Mem_Init();                                                 /* Initialize Memory Managment Module                   */
  CPU_IntDis();                                               /* Disable all Interrupts                               */
  CPU_Init();                                                 /* Initialize the uC/CPU services                       */
  Math_Init();                                                /* Initialize Mathematical Module                       */

  OSInit(&os_err);                                            /* Initialize uC/OS-III                                 */
  if (os_err != OS_ERR_NONE) {
      while (1);
  }

  App_OS_SetAllHooks();                                       /* Set all applications hooks                           */

  OSQCreate(&CommQ,
            "Comm Queue",
             10,
            &os_err);                    					  /* Create COMM Queue         */

  // plant input queue
  OSQCreate(&PlantInputQ,
		  	"Plant Input Queue",
			20,
			&os_err);

  OSQCreate(&PlantVQ,
  		  	"Plant Voltage Setpoint Queue",
  			20,
  			&os_err);

  OSQCreate(&PlantTempQ,
  		  	"Plant Temperature Setpoint Queue",
  			20,
  			&os_err);

  OSTaskCreate(&StartupTaskTCB,                               /* Create the startup task                              */
               "Startup Task",
                StartupTask,
                0u,
                APP_CFG_STARTUP_TASK_PRIO,
               &StartupTaskStk[0u],
                StartupTaskStk[APP_CFG_STARTUP_TASK_STK_SIZE / 10u],
                APP_CFG_STARTUP_TASK_STK_SIZE,
                0u,
                0u,
                0u,
               (OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
               &os_err);
  if (os_err != OS_ERR_NONE) {
      while (1);
  }

  OSStart(&os_err);                                           /* Start multitasking (i.e. give control to uC/OS-III)  */

  while (DEF_ON) {}                                            /* Should Never Get Here.                               */

}

/* Startup Task----------------------------------------------------------*/

static  void  StartupTask (void *p_arg)
{
  OS_ERR  os_err;
  (void)p_arg;

  OS_TRACE_INIT();                                            /* Initialize the uC/OS-III Trace recorder              */

  BSP_OS_TickEnable();                                        /* Enable the tick timer and interrupt                  */
  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USB_DEVICE_Init();

  BSP_LED_Init();

  MX_DMA2D_Init();
  MX_FMC_Init();
  MX_I2C3_Init();
  MX_LTDC_Init();
  MX_SPI5_Init();

  BSP_LCD_Init();
  BSP_LCD_LayerDefaultInit(LCD_BACKGROUND_LAYER, LCD_FRAME_BUFFER);
  BSP_LCD_LayerDefaultInit(LCD_FOREGROUND_LAYER, LCD_FRAME_BUFFER);
  BSP_LCD_SelectLayer(LCD_FOREGROUND_LAYER);
  BSP_LCD_DisplayOn();
  BSP_LCD_Clear(LCD_COLOR_WHITE);

#if OS_CFG_STAT_TASK_EN > 0u
  OSStatTaskCPUUsageInit(&os_err);                            /* Compute CPU capacity with no task running            */
#endif

#ifdef CPU_CFG_INT_DIS_MEAS_EN
  CPU_IntDisMeasMaxCurReset();
#endif

  AppTaskCreate();                                            /* Create Application tasks                             */

  while (DEF_TRUE) {                                          /* Task body, always written as an infinite loop.       */
      BSP_LED_Toggle(0);
      OSTimeDlyHMSM(0u, 0u, 1u, 0u,
                    OS_OPT_TIME_HMSM_STRICT,
                    &os_err);

  }
}

/* AppTaskCreate----------------------------------------------------------*/

static  void  AppTaskCreate (void)
{
    OS_ERR  os_err;

    OSTaskCreate(&CommTaskTCB,                               /* Create the comm task                              */
                 "Comm Task",
                  CommTask,
                  0u,
                  APP_CFG_COMM_TASK_PRIO,
                 &CommTaskStk[0u],
                  CommTaskStk[APP_CFG_COMM_TASK_STK_SIZE / 10u],
                  APP_CFG_COMM_TASK_STK_SIZE,
                  2u,										// maximum messages to send to this task
                  0u,
                  0u,
                 (OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                 &os_err);
    if (os_err != OS_ERR_NONE) {
        while (1);
    }

    OSTaskCreate(&BtnTaskTCB,                               /* Create the comm task                              */
                 "Button Task",
                  BtnTask,
                  0u,
                  APP_CFG_BTN_TASK_PRIO,
                 &BtnTaskStk[0u],
                  BtnTaskStk[APP_CFG_BTN_TASK_STK_SIZE / 10u],
                  APP_CFG_BTN_TASK_STK_SIZE,
                  0u,
                  0u,
                  0u,
                 (OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                 &os_err);
    if (os_err != OS_ERR_NONE) {
        while (1);
    }

    OSTaskCreate(&GuiTaskTCB,                               /* Create the comm task                              */
                 "GUI Task",
                  GuiTask,
                  0u,
                  APP_CFG_GUI_TASK_PRIO,
                 &GuiTaskStk[0u],
                  GuiTaskStk[APP_CFG_GUI_TASK_STK_SIZE / 10u],
                  APP_CFG_GUI_TASK_STK_SIZE,
                  0u,
                  0u,
                  0u,
                 (OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                 &os_err);
    if (os_err != OS_ERR_NONE) {
        while (1);
    }

    OSTaskCreate(&PlantTaskTCB,
				"Plant Task",
				 PlantTask,
				 0u,
				 APP_CFG_PLANT_TASK_PRIO,
				&PlantTaskStk[0u],
				 PlantTaskStk[APP_CFG_PLANT_TASK_STK_SIZE / 10u],
				 APP_CFG_PLANT_TASK_STK_SIZE,
				 0u,
				 0u,
				 0u,
				(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
				&os_err);

    if (os_err != OS_ERR_NONE) {
        while (1);
    }
}


/* COMM Task----------------------------------------------------------*/

static int plant_mode = 0;
static int temp_setpoint = 24;
static double voltage_setpoint = 0;
static int sample_flag = 0;
static float prev_plant_output = 0;
static float plant_input = 0;
static float plant_output = 0;


static  void  CommTask (void *p_arg)
{
  OS_ERR  os_err;
  void        *p_msg;
  OS_MSG_SIZE  msg_size = 0;
  CPU_TS       ts;

  Str4Display[0]='\0';

  (void)p_arg;

#if OS_CFG_STAT_TASK_EN > 0u
  OSStatTaskCPUUsageInit(&os_err);                            /* Compute CPU capacity with no task running            */
#endif

#ifdef CPU_CFG_INT_DIS_MEAS_EN
  CPU_IntDisMeasMaxCurReset();
#endif

  while (DEF_TRUE) {                                          /* Task body, always written as an infinite loop.       */
    p_msg = OSQPend(&CommQ,
                     0,
                     OS_OPT_PEND_BLOCKING,
                    &msg_size,
                    &ts,
                    &os_err);

	if(msg_size >0){
		memcpy(Str4Display,p_msg, msg_size);
		msg_size = 0;



    }

	// post input to PlantInputQ
	OSQPost(&PlantInputQ,
			(char *)p_msg,
			sizeof((char *)p_msg),
			OS_OPT_POST_FIFO,
			&os_err);

	if (Str4Display[0] == 'a')
		plant_mode = 1;
	if (Str4Display[0] == 'm')
		plant_mode = 0;

	if (plant_mode == 0)
	{
		if ((Str4Display[0] == 'u')&&(voltage_setpoint<10))
			voltage_setpoint += 0.05;
		if ((Str4Display[0] == 'j')&&(voltage_setpoint>0))
			voltage_setpoint -= 0.05;
	}
	else if (plant_mode == 1)
	{
		if (Str4Display[0] == 'u'&&(temp_setpoint<120))
			temp_setpoint += 1;
		if ((Str4Display[0] == 'j')&&(temp_setpoint>24))
			temp_setpoint -= 1;
	}



  }
}

/* Button Task----------------------------------------------------------*/

static  void  BtnTask (void *p_arg)
{
  OS_ERR  os_err;
  unsigned int count_press = 0;

  (void)p_arg;

#if OS_CFG_STAT_TASK_EN > 0u
  OSStatTaskCPUUsageInit(&os_err);                            /* Compute CPU capacity with no task running            */
#endif

#ifdef CPU_CFG_INT_DIS_MEAS_EN
  CPU_IntDisMeasMaxCurReset();
#endif

  while (DEF_TRUE) {                                          /* Task body, always written as an infinite loop.       */
	OSTimeDlyHMSM(0u, 0u, 0u, 10u,
					OS_OPT_TIME_HMSM_STRICT,
					&os_err);
    if(HAL_GPIO_ReadPin(KEY_BUTTON_GPIO_PORT, KEY_BUTTON_PIN) == GPIO_PIN_SET){

//		printf("BtnTask running: Button Pressed : %i times \n\r", count_press);
		count_press++;
		while(HAL_GPIO_ReadPin(KEY_BUTTON_GPIO_PORT, KEY_BUTTON_PIN)==GPIO_PIN_SET);
    }
  }
}

/* GUI Task----------------------------------------------------------*/

void GuiTask(void *p_arg)
{
  /* USER CODE BEGIN GuiTask */
  OS_ERR  os_err;
  uint8_t buf[sizeof(Str4Display)];
  uint8_t graphLine[220];

  while(1)
  {
	  //Graph Axis


	  	  /*if (plant_mode == 0)
	  	  {
	  	  	BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
	  	  	BSP_LCD_FillRect(10, 0, 220, 160);
	  	  }*/


	  	  if (plant_mode == 1)
	  	  {
	  		BSP_LCD_SetTextColor(LCD_COLOR_RED);
	  		BSP_LCD_DrawHLine(10, 140 - temp_setpoint, 220);
	  	  }

	  	  if (sample_flag == 0)
	  	  {
	  		  BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
	  		  BSP_LCD_FillRect(10, 0, 230, 160);
	  		  for (int i = 10; i <231; i++)
	  		  {
	  			  //graphLine[i] = i;
	  			  if (i < 230)
	  			  	  {graphLine[i] = graphLine[i+1];}
	  			  else
	  			  	  {graphLine[i] = 140-VoltageToTemp(plant_output);}
	  			  BSP_LCD_DrawPixel(i, graphLine[i], LCD_COLOR_BLUE);
	  		  }
	  	  }
	  	  BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
	  	  BSP_LCD_DrawVLine(10, 20, 120);
	  	  BSP_LCD_DrawHLine(10, 140, 220);

	  	  //Vertical Axis Scale
	  	  BSP_LCD_DrawHLine(7, 20, 3);
	  	  BSP_LCD_DrawHLine(7, 30, 3);
	  	  BSP_LCD_DrawHLine(7, 40, 3);
	  	  BSP_LCD_DrawHLine(7, 50, 3);
	  	  BSP_LCD_DrawHLine(7, 60, 3);
	  	  BSP_LCD_DrawHLine(7, 60, 3);
	  	  BSP_LCD_DrawHLine(7, 70, 3);
	  	  BSP_LCD_DrawHLine(7, 80, 3);
	  	  BSP_LCD_DrawHLine(7, 90, 3);
	  	  BSP_LCD_DrawHLine(7, 100, 3);
	  	  BSP_LCD_DrawHLine(7, 110, 3);
	  	  BSP_LCD_DrawHLine(7, 120, 3);
	  	  BSP_LCD_DrawHLine(7, 130, 3);

	  	  // Student and Course Info
	  	  BSP_LCD_SetTextColor(LCD_COLOR_YELLOW);
	  	  BSP_LCD_FillRect(0, 150, 240, 30);
	  	  BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
	  	  BSP_LCD_SetBackColor(LCD_COLOR_YELLOW);
	  	  BSP_LCD_SetFont(&Font12);
	  	  BSP_LCD_DisplayStringAtLine(13, (uint8_t *)"  Group 4    Course: ENGG*4420");
	  	  BSP_LCD_DrawVLine(80, 150, 30);

	  	  // General Settings
	  	  BSP_LCD_SetTextColor(LCD_COLOR_GRAY);
	  	  BSP_LCD_FillRect(0, 180, 240, 10);
	  	  BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
	  	  BSP_LCD_FillRect(0, 190, 240, 100);
	  	  BSP_LCD_SetTextColor(LCD_COLOR_GRAY);
	  	  BSP_LCD_FillRect(0, 290, 240, 30);
	  	  BSP_LCD_SetTextColor(LCD_COLOR_GRAY);
	  	  BSP_LCD_DrawVLine(120, 210, 95);
	  	  BSP_LCD_DrawHLine(0, 208, 240);
	  	  BSP_LCD_DrawHLine(0, 237, 240);
	  	  BSP_LCD_DrawHLine(0, 262, 240);
	  	  BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
	  	  BSP_LCD_SetBackColor(LCD_COLOR_WHITE);

	  	  char temp_real_buf[10];
	  	  gcvt(VoltageToTemp(plant_output),4, temp_real_buf);

	  	  if (plant_mode == 0)
	  	  {
	  		char vp_sp_buf[10];
	  		gcvt(voltage_setpoint, 4 , vp_sp_buf);

	  		BSP_LCD_DisplayStringAt(0, 195, (uint8_t *)"Manual Mode", CENTER_MODE);
	  	  	BSP_LCD_DisplayStringAt(10, 220, (uint8_t *)"Voltage Setpoint", LEFT_MODE);
	  		BSP_LCD_DisplayStringAt(150, 220, (uint8_t *)vp_sp_buf, LEFT_MODE);
	  	  }
	  	  else
	  	  {
	  		char temp_sp_buf[10];
	  		itoa(temp_setpoint, temp_sp_buf , 10);


	  		BSP_LCD_DisplayStringAt(0, 195, (uint8_t *)"Auto Mode", CENTER_MODE);
	  		BSP_LCD_DisplayStringAt(10, 220, (uint8_t *)"Temp Setpoint", LEFT_MODE);
	  		BSP_LCD_DisplayStringAt(150, 220, (uint8_t *)temp_sp_buf, LEFT_MODE);
	  	  }



	  	  BSP_LCD_DisplayStringAt(10, 245, (uint8_t *)"Output Temp", LEFT_MODE);
	  	  BSP_LCD_DisplayStringAt(150, 245, (uint8_t *)temp_real_buf, LEFT_MODE);
	  	  BSP_LCD_DisplayStringAt(10, 270, (uint8_t *)"Sampling Time", LEFT_MODE);
	  	  BSP_LCD_DisplayStringAt(150, 270, (uint8_t *)"200ms", LEFT_MODE);



/*	  if(Str4Display[0]!='\0')
	  {
	      memcpy(buf,Str4Display, data_received+1);
		  Str4Display[0] = '\0';
		  //BSP_LCD_SetTextColor(LCD_COLOR_RED);
		  //BSP_LCD_ClearStringLine(7);
		  //BSP_LCD_DisplayStringAtLine(7, buf);
	  }*/

	  OSTimeDlyHMSM(0u, 0u, 0u, 10u,
						OS_OPT_TIME_HMSM_STRICT,
						&os_err);
  }
}

/**********************************************************************************************************
*                                            Plant TASK
*
* Description : This task is the discretized transfer function of a Hot Air Plant following:
*
* 				 		  0.119217
* 				H(z) = ---------------
* 						z - 0.904837
*
* 				using settings:
*
* 				 - Sample Time = 200ms
*				 - Blower opening = 50
*
* Arguments   : p_arg   is the argument passed to 'PlantTask()' by 'OSTaskCreate()'.
* Returns     : none
* Notes       : 1) 	You must tune your PID gains in LabVIEW with the above settings,
* 					so that those gains will work when used within your PID Task
*
* 				2)  float plant_intput --> input to descretized transfer function / plant
* 				    float plant_output --> output of descretized transfer function / plant
*********************************************************************************************************
*/



static float error_sum = 0;
static float prev_error = 0;

static void PlantTask(void *p_arg){

	OS_ERR  os_err;
	void        *p_msg;
	OS_MSG_SIZE  msg_size;
	CPU_TS       ts;


	while(1){

		// Get voltage from plant input queue
	    p_msg = OSQPend(&PlantInputQ,
	                     0,
	                     OS_OPT_PEND_NON_BLOCKING, // doesn't wait for queue
	                    &msg_size,
	                    &ts,
	                    &os_err);

	    // if message isn't empty
	    if (msg_size > 0){
	    	// convert message to float
		    plant_input = strtof((char *)p_msg, NULL);
	    }

	    if (plant_mode == 1) {
	    	plant_input = TempToVoltage((float)temp_setpoint);
	    	plant_output = (0.1*(plant_input-plant_output) + 0.01*error_sum + 0.003*((plant_input-plant_output)-prev_error))*(0.119217*plant_input + 0.904837*prev_plant_output);
	    	error_sum += plant_input-plant_output;
	    	prev_error = plant_input-plant_output;
	    }
	    else{
	    	plant_input = (float)voltage_setpoint;
	    	plant_output = 0.119217*plant_input + 0.904837*prev_plant_output;
	    }

	    // calculate plant output based on discrete transfer function


		/*
		 * In order to print out floating point numbers,
		 * go to Project > Properties > C/C++ Build > Settings and
		 * change Runtime Library from 'Newlib-nano' to 'Newlib standard'
		 */

		// convert plant output to temperature before printing to teraterm
		printf("uC/OS - Plant Input : %0.3f Volts ----------- Plant Output : %0.3f Volts \\ %0.3f C\n\r", plant_input, plant_output, VoltageToTemp(plant_output) );

		prev_plant_output = plant_output;


		// 'sample time' (runs every  200 ms)
		sample_flag = 1;
		OSTimeDlyHMSM(0u, 0u, 0u, 200u,
						OS_OPT_TIME_HMSM_STRICT,
						&os_err);
		sample_flag = 0;
	}
}


/**********************************************************************************************************
*
*            						HELPER FUNCTIONS
*
*********************************************************************************************************
*/


/*
 * temp/voltage relationship
 * using polynomial order 2
 */

static float TempToVoltage(float temp){
	return -0.0015*pow(temp,2) + 0.3319*temp - 6.9173;
}

static float VoltageToTemp(float volt){
	return 0.3053*pow(volt,2) + 2.2602*volt + 25.287;
}


/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

#if 0
  /**Configure the Systick interrupt time
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
#endif
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */
#define USE_VCP 1
#define USE_MYMUTEX 0
#if USE_VCP
#ifdef __GNUC__
/* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
int __io_putchar(int ch);
int _write(int file,char *ptr, int len)
{
 int DataIdx;
#if USE_MYMUTEX
 static	OS_MUTEX           MyMutex;
 OS_ERR  os_err;
 CPU_TS  ts;
		// use mutex to protect VCP access
		OSMutexPend((OS_MUTEX *)&MyMutex,
				  (OS_TICK   )0,
				  (OS_OPT    )OS_OPT_PEND_BLOCKING,
				  (CPU_TS   *)&ts,
				  (OS_ERR   *)&os_err);
#endif
 for(DataIdx= 0; DataIdx< len; DataIdx++)
 {
 __io_putchar(*ptr++);
 }
#if USE_MYMUTEX
		OSMutexPost((OS_MUTEX *)&MyMutex,
				  (OS_OPT    )OS_OPT_POST_NONE,
				  (OS_ERR   *)&os_err);
#endif
 return len;
}
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

PUTCHAR_PROTOTYPE
{
	while(CDC_Transmit_HS((uint8_t *)&ch, 1) != USBD_OK);
	return ch;
}

#else
#ifdef __GNUC__
int _write(int32_t file, uint8_t *ptr, int32_t len)
{
	/* Implement your write code here, this is used by puts and printf for example */
	/* return len; */
	int i;
	for(i=0; i<len; i++)
		ITM_SendChar(*ptr++);
	return len;
}
#endif
volatile int32_t ITM_RxBuffer = ITM_RXBUFFER_EMPTY;

int fputc(int ch, FILE *f) {
  return (ITM_SendChar(ch));
}

int fgetc(FILE *f) {                /* blocking */
  while (ITM_CheckChar() != 1);
  return (ITM_ReceiveChar());
}

#endif


/******END OF FILE****/
