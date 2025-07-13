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
#include "adc.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
#include "string.h"
#include "atk_md0240.h"
#include "image.h"
#define ADC_BUFFER_SIZE 625//500us@3.03MHz
#define FFT_LEN 1024
#define WAVEFORMWIDTH 230
#define WAVEFORMHEIGHT 230


float FFT_INPUT[FFT_LEN*2];  
float FFT_OUTPUT[FFT_LEN];
float moni_sin_data[FFT_LEN];
uint8_t INT_FFT_OUTPUT[FFT_LEN];

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define FRAME_AVERAGE_COUNT 1000
volatile uint16_t adc_buffer_a[ADC_BUFFER_SIZE];
volatile uint16_t adc_buffer_b[ADC_BUFFER_SIZE];
volatile uint8_t current_buffer = 0; // 0: BufferA, 1: BufferB

volatile uint8_t adc_data_ready = 0;              // ADC数据准备好标志
volatile uint16_t* last_adc_buffer = NULL;        // 指向最新ADC数据缓冲区的指针

extern UART_HandleTypeDef hcom_uart[]; //  BSP串口句柄

uint16_t tx_buffer1[ADC_BUFFER_SIZE];
uint16_t tx_buffer2[ADC_BUFFER_SIZE];
volatile uint8_t active_tx_buffer = 0;
volatile uint8_t lcd_show_waveform_ready = 0;
volatile uint8_t sendflag = 0;
volatile uint8_t ADCflag = 1;

const PinMap pin_map[7] = {
    {GPIOA, GPIO_PIN_6},  // out0 -> PA6(��ΪPC4)
    {GPIOA, GPIO_PIN_7},  // out1 -> PA7
    {GPIOB, GPIO_PIN_6},  // out2 -> PB6����ΪPC2��
    {GPIOC, GPIO_PIN_7},  // out3 -> PC7����ΪPC8��
    {GPIOA, GPIO_PIN_9},  // out4 -> PA9
    {GPIOA, GPIO_PIN_8},  // out5 -> PA8
    {GPIOA, GPIO_PIN_10}   // out6 -> PA10
};

// frame average variable
volatile uint32_t accum_buffer[ADC_BUFFER_SIZE] = {0};  // 32bit leijia huanchong qv
volatile uint16_t frame_count = 0;
volatile uint8_t average_ready = 0;
volatile uint16_t averaged_buffer[ADC_BUFFER_SIZE];  // pingjvn jieguo huanchong qv
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

COM_InitTypeDef BspCOMInit;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_TIM8_Init();
  MX_TIM20_Init();
  MX_SPI2_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */
  // ���� ADC DMA ����
  HAL_ADC_Start_DMA(&hadc2, (uint32_t*)adc_buffer_a, ADC_BUFFER_SIZE);
// ������ʱ������ ADC
  HAL_TIM_Base_Start(&htim1);
	
  HAL_TIM_OC_Start_IT(&htim20, TIM_CHANNEL_1);
  HAL_TIM_OC_Start_IT(&htim20, TIM_CHANNEL_4);
  HAL_TIM_OC_Start_IT(&htim20, TIM_CHANNEL_5);
//  HAL_TIM_OC_Start_IT(&htim8, TIM_CHANNEL_1);
//  HAL_TIM_OC_Start_IT(&htim8, TIM_CHANNEL_2);

  
  HAL_TIM_PWM_Start(&htim20,TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim20,TIM_CHANNEL_3);
  
  update_gpio_output(0x73);//0b1110011
  atk_md0240_init();

  /* USER CODE END 2 */

  /* Initialize led */
  BSP_LED_Init(LED_GREEN);

  /* Initialize USER push-button, will be used to trigger an interrupt each time it's pressed.*/
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

  /* Initialize COM1 port (115200, 8 bits (7-bit data + 1 stop bit), no parity */
  BspCOMInit.BaudRate   = 115200;
  BspCOMInit.WordLength = COM_WORDLENGTH_8B;
  BspCOMInit.StopBits   = COM_STOPBITS_1;
  BspCOMInit.Parity     = COM_PARITY_NONE;
  BspCOMInit.HwFlowCtl  = COM_HWCONTROL_NONE;
  if (BSP_COM_Init(COM1, &BspCOMInit) != BSP_ERROR_NONE)
  {
    Error_Handler();
  }

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    BspCOMInit.BaudRate   = 256000;
  BspCOMInit.WordLength = COM_WORDLENGTH_8B;
  BspCOMInit.StopBits   = COM_STOPBITS_1;
  BspCOMInit.Parity     = COM_PARITY_NONE;
  BspCOMInit.HwFlowCtl  = COM_HWCONTROL_NONE;
  if (BSP_COM_Init(COM1, &BspCOMInit) != BSP_ERROR_NONE)
  {
    Error_Handler();
  }
  
  atk_md0240_show_pic(20 , 110 ,200 ,100 , image_data   );
  HAL_Delay(1000);
  atk_md0240_clear(ATK_MD0240_WHITE);
  

  draw_axes();//绘制坐标轴
  
  
  
  while (1)
  {
	
    if (adc_data_ready) {
        adc_data_ready = 0;
        // 累加
        for (int i = 0; i < ADC_BUFFER_SIZE; i++) {
            accum_buffer[i] += last_adc_buffer[i];
        }
        frame_count++;

        // 达到平均帧数后处理
        if (frame_count >= FRAME_AVERAGE_COUNT) {
			
            for (int i = 0; i < ADC_BUFFER_SIZE; i++) {
                averaged_buffer[i] = (uint16_t)(accum_buffer[i] / FRAME_AVERAGE_COUNT);
            }
            memset((void*)accum_buffer, 0, sizeof(accum_buffer));
            frame_count = 0;
            average_ready = 1;
			lcd_show_waveform_ready = 1;
        }
    }
	
	if(lcd_show_waveform_ready){
		lcd_show_waveform_ready = 0;
		lcd_show_waveform( averaged_buffer );
	}

//    if (average_ready) {
//		//ADC_Pause(&hadc1);
//		ADCflag = 0;
//        average_ready = 0;
//		sendflag = 1;
//        // 发送数据...
//        for (int i = 0; i < ADC_BUFFER_SIZE; i++) {
//            uint16_t adc_value = averaged_buffer[i];
//            uint8_t data_packet[3] = {
//                0xAA,
//                (uint8_t)(adc_value >> 8),
//                (uint8_t)(adc_value & 0xFF)
//            };
//            HAL_UART_Transmit(&hcom_uart[COM1], data_packet, 3, 100);
//        }
//		//ADC_Resume(&hadc1);
//		sendflag = 0;
//		
//    }
    //HAL_Delay(1);
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV6;
  RCC_OscInitStruct.PLL.PLLN = 50;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */





void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
    last_adc_buffer = (current_buffer == 0) ? adc_buffer_a : adc_buffer_b;
    adc_data_ready = 1;

    // 启动DMA到另一个缓冲区
    if (current_buffer == 0) {
        HAL_ADC_Start_DMA(&hadc2, (uint32_t*)adc_buffer_b, ADC_BUFFER_SIZE);
        current_buffer = 1;
    } else {
        HAL_ADC_Start_DMA(&hadc2, (uint32_t*)adc_buffer_a, ADC_BUFFER_SIZE);
        current_buffer = 0;
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    active_tx_buffer ^= 1;
}


// ��д�Ƚ�ƥ��ص�����
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim) {
  if(htim->Instance == TIM20) {
    switch(htim->Channel) {
      case HAL_TIM_ACTIVE_CHANNEL_1:{
		 //update_gpio_output(0x33);//0b1110010
		 HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_SET);
        //3816
        break;
	  }
	  

	  case HAL_TIM_ACTIVE_CHANNEL_4:{
		  //update_gpio_output(0x73);//0b1110011
		  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_RESET);
		  BSP_LED_Toggle(LED_GREEN);
//		  if((!sendflag)&&(!ADCflag)){
//			ADC_Resume(&hadc1);
//			
//		  }
//		  ADC_Resume(&hadc1);
		  break;
		  //24816
	  }

      default:
		break;
    }
  }

}

void update_gpio_output(uint8_t value) {
    for (int i = 0; i < 7; i++) {
        HAL_GPIO_WritePin(pin_map[i].port, pin_map[i].pin,(value >> i) & 0x01 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }
}


void ADC_Pause(ADC_HandleTypeDef* hadc) {
    HAL_TIM_Base_Stop(&htim1);        // ֹͣ����Դ
    //hadc->State = HAL_ADC_STATE_READY; 
	HAL_ADC_Stop_DMA(hadc);           // 使用HAL库标准函数停止ADC
}

// �ָ�ADCת��
void ADC_Resume(ADC_HandleTypeDef* hadc) {
//    // ���DMA��־
//	DMA_HandleTypeDef* hdma = hadc->DMA_Handle;
////    __HAL_DMA_DISABLE(hadc->DMA_Handle);
//    __HAL_DMA_CLEAR_FLAG(hadc->DMA_Handle, __HAL_DMA_GET_TC_FLAG_INDEX(hdma) | __HAL_DMA_GET_HT_FLAG_INDEX(hdma));
//    __HAL_DMA_ENABLE(hadc->DMA_Handle);
//    
//    // ��������Դ
//    HAL_TIM_Base_Start(&htim1);
//    hadc->State = HAL_ADC_STATE_BUSY;
	   // 重新初始化ADC
    // 根据当前缓冲区启动DMA
    if (current_buffer == 0) {
        HAL_ADC_Start_DMA(hadc, (uint32_t*)adc_buffer_a, ADC_BUFFER_SIZE);
    } else {
        HAL_ADC_Start_DMA(hadc, (uint32_t*)adc_buffer_b, ADC_BUFFER_SIZE);
    }
    
    // 重新启动时钟源
    HAL_TIM_Base_Start(&htim1);
}


uint8_t float_to_uint8(float value) {
    // 1. 将值限制在0.0-255.0范围内
    float clamped = fmaxf(0.0f, fminf(value, 255.0f));
    
    // 2. 四舍五入到最接近的整数
    // 注意：直接加0.5然后截断是实现四舍五入的常用技巧
    return (uint8_t)(clamped + 0.5f);
}


void lcd_show_waveform(volatile uint16_t* p) {
    // 1. 参数检查
    if(p == NULL) return;
    
    // 2. 定义显示区域参数
    const uint16_t DISPLAY_START_X = 5;    // 显示区域起始X坐标
    const uint16_t DISPLAY_START_Y = 5;    // 显示区域起始Y坐标
    const uint16_t DISPLAY_END_Y = 234;    // 显示区域结束Y坐标
    const uint16_t DISPLAY_HEIGHT = DISPLAY_END_Y - DISPLAY_START_Y; // 显示区域高度
    const uint16_t DISPLAY_WIDTH = 230;    // 显示区域宽度 (根据实际情况调整)
    
    // 3. 固定电压范围 (1.4V ~ 1.9V)
    const float VOLT_MIN = 1.4f;           // 最小电压 1.4V
    const float VOLT_MAX = 1.9f;           // 最大电压 1.9V
    
    // 4. 计算对应的ADC值范围
    const uint16_t ADC_MIN = (uint16_t)((VOLT_MIN / 3.3f) * 2048);
    const uint16_t ADC_MAX = (uint16_t)((VOLT_MAX / 3.3f) * 2048);
    const uint16_t ADC_RANGE = ADC_MAX - ADC_MIN;
    
    // 5. 查找最大值位置 (用于确定显示起点)
    uint16_t max = 0;
    uint16_t max_p = 100;
    for(int i = 0; i < ADC_BUFFER_SIZE; i++) {
        if(p[i] > max) {
            max = p[i];
            max_p = i;
        }
    }
    
    // 6. 安全计算起点
    uint16_t start_point;
    if(max_p < 20) {
        start_point = 0;
    } else {
        start_point = max_p - 20;
    }
    
    // 7. 计算实际可绘制的点数
    uint16_t draw_points = DISPLAY_WIDTH / 2;
    if(start_point + draw_points >= ADC_BUFFER_SIZE) {
        draw_points = ADC_BUFFER_SIZE - start_point - 1;
    }
    
    // 8. 绘制第一个点
    // 清除第一列
    atk_md0240_draw_line(DISPLAY_START_X, DISPLAY_START_Y, DISPLAY_START_X, DISPLAY_END_Y, ATK_MD0240_WHITE);
    atk_md0240_draw_line(DISPLAY_START_X+1, DISPLAY_START_Y, DISPLAY_START_X+1, DISPLAY_END_Y, ATK_MD0240_WHITE);
    
    // 约束ADC值在[ADC_MIN, ADC_MAX]范围内
    uint16_t adc_value = p[start_point];
    if(adc_value < ADC_MIN) adc_value = ADC_MIN;
    if(adc_value > ADC_MAX) adc_value = ADC_MAX;
    
    // 计算Y坐标 (注意：高电压对应屏幕顶部)
    uint16_t y_val = ((uint32_t)(adc_value - ADC_MIN) * DISPLAY_HEIGHT) / ADC_RANGE;
    y_val = DISPLAY_END_Y - y_val;  // 反转Y轴
    
    // 绘制第一个点
    atk_md0240_draw_point(DISPLAY_START_X, y_val, ATK_MD0240_BLACK);
    
    // 9. 绘制后续点
    for(int i = 1; i < draw_points; i++) {
        uint16_t col = DISPLAY_START_X + 2*i;
        // 清除当前列
        atk_md0240_draw_line(col, DISPLAY_START_Y, col, DISPLAY_END_Y, ATK_MD0240_WHITE);
        atk_md0240_draw_line(col+1, DISPLAY_START_Y, col+1, DISPLAY_END_Y, ATK_MD0240_WHITE);
        
        // 处理前一个点
        uint16_t prev_adc = p[start_point + i - 1];
        if(prev_adc < ADC_MIN) prev_adc = ADC_MIN;
        if(prev_adc > ADC_MAX) prev_adc = ADC_MAX;
        uint16_t prev_y = ((uint32_t)(prev_adc - ADC_MIN) * DISPLAY_HEIGHT) / ADC_RANGE;
        prev_y = DISPLAY_END_Y - prev_y;
        
        // 处理当前点
        uint16_t curr_adc = p[start_point + i];
        if(curr_adc < ADC_MIN) curr_adc = ADC_MIN;
        if(curr_adc > ADC_MAX) curr_adc = ADC_MAX;
        uint16_t curr_y = ((uint32_t)(curr_adc - ADC_MIN) * DISPLAY_HEIGHT) / ADC_RANGE;
        curr_y = DISPLAY_END_Y - curr_y;
        
        // 绘制连接线
        atk_md0240_draw_line(col - 2, prev_y, col, curr_y, ATK_MD0240_BLACK);
    }
}

void draw_axes(void){
	atk_md0240_draw_line( 4 ,5 , 4, 235 ,ATK_MD0240_BLACK); //纵轴
	atk_md0240_draw_line( 1 ,165 ,4 ,165 ,ATK_MD0240_BLACK); //1V
	atk_md0240_draw_line( 1 ,95 ,4 ,95 ,ATK_MD0240_BLACK); //2V
	atk_md0240_draw_line( 1 ,25 ,4 ,25 ,ATK_MD0240_BLACK);  //3V
	
	atk_md0240_draw_line( 4 ,235 ,235, 235 ,ATK_MD0240_BLACK ); //横轴
	atk_md0240_draw_line( 55 ,235 ,55, 240 ,ATK_MD0240_BLACK ); 
	atk_md0240_draw_line( 105 ,235 ,105 , 240 ,ATK_MD0240_BLACK ); 
	atk_md0240_draw_line( 155 ,235 ,155, 240 ,ATK_MD0240_BLACK );
	atk_md0240_draw_line( 205 ,235 ,205, 240 ,ATK_MD0240_BLACK ); 
	
	//atk_md0240_show_string( 20 ,240, "y:1V/dev",ATK_MD0240_FONT_12 , ATK_MD0240_BROWN  );
	atk_md0240_show_string( 120 ,240, "x:50us/dev",ATK_MD0240_FONT_12 , ATK_MD0240_BROWN  );
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
