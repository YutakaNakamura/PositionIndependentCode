/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
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
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "module.h"
#include "myLib.h"
#include "myPic.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

static int16_t code[] = { /* this could would bet loaded by a loader e.g. into RAM */
// 0x0000:  int MyLib_Calc(int x) {
  0x4b03, //   0: 4a03        ldr r2, [pc, #12] ; (10 <MyLib_Calc+0x10>)
  0xf859, 0x2003, // ldr.w  r2, [r9, r3]
  0x6813, //   4: 6813        ldr r3, [r2, #0]
  0x3301, //   6: 3301        adds  r3, #1
  0x6013, //   8: 6013        str r3, [r2, #0]
  0x0040, //   lsls  r0, r0, #1
  0x4770, //       bx  lr
  0x0000, 0x0000, // .word 0x00000000: index into GOT table for address of variable 'glob'

// 0x0014:  int MyLib_Mul2(int x) {
  0x0040, //   a: 0040        lsls  r0, r0, #1
  0x4770, //   c: 4770        bx  lr

// 0x0018:  void MyLib_Init(void) {
  0x4770, //  18: 4770        bx  lr
  0xbf00  //  1a: bf00        nop
};

/*
 * The entries order below you can get from readelf from the final binary too:
 * Relocation section '.rel.dyn' at offset 0x10364 contains 3 entries:
 * Offset     Info    Type            Sym.Value  Sym. Name
 * 2000000c  00000116 R_ARM_JUMP_SLOT   00000000   MyLib_Init
 * 20000010  00000216 R_ARM_JUMP_SLOT   00000000   MyLib_Calc
 * 20000014  00000316 R_ARM_JUMP_SLOT   00000000   MyLib_Mul2
 */
//ここ、めちゃくちゃ考察する必要ある。　デフォルトの1,3,4,5だと正しくポインタが当たらない。
//1,3,5,7だと正常に動作する。単純に奇数だからOKみたいな話ではないからちゃんとreadelfから見つけること。
typedef enum {
  GOT_PLT_INDEX_MyLib_glob=1,
  GOT_PLT_INDEX_MyLib_Init=3,
  GOT_PLT_INDEX_MyLib_Calc=5,
  GOT_PLT_INDEX_MyLib_Mul2=7,
} Got_Plt_Index_e;

typedef struct {
  const char *name; /*!< name of function */
  size_t offset;    /*!< offset in loaded .code section */
  Got_Plt_Index_e got_plt_idx;  /*!< index in .got_plt table */
} binding_t;

static const binding_t code_bindings[] =
{ /* in this simple use case: the offsets are within the code[] array above */
    {"MyLib_Calc", 0x0000, GOT_PLT_INDEX_MyLib_Calc},
    {"MyLib_Mul2", 0x0014, GOT_PLT_INDEX_MyLib_Mul2},
    {"MyLib_Init", 0x0018, GOT_PLT_INDEX_MyLib_Init},
};

static const binding_t data_bindings[] = {
    {"MyLib glob", 0x0000, GOT_PLT_INDEX_MyLib_glob},
};

/* Force the counter to be placed into memory. */
volatile int i, j = 0 ;
void foobar(void) {}

extern unsigned int _sgot, _sgot_plt; /* symbols provided by the linker */

void BindLibrary(void *codeStart, void *dataStart) {
  /* code bindings */
  for(int i=0; i<sizeof(code_bindings)/sizeof(code_bindings[0]); i++) {
    ((uint32_t*)&_sgot_plt)[code_bindings[i].got_plt_idx] = (uint32_t)(codeStart+code_bindings[i].offset);
  }
  /* data bindings */
  for(int i=0; i<sizeof(data_bindings)/sizeof(data_bindings[0]); i++) {
    ((uint32_t*)&_sgot_plt)[data_bindings[i].got_plt_idx] = (uint32_t)(dataStart+data_bindings[i].offset);
  }
}

int unko1(void) {
	return 1;
}

int unko2(void) {
	return 2;
}

int unko3(void) {
	return 3;
}

int unmatch(void) {
	return -1;
}

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
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
  MX_LPUART1_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */
	  /* load lib into memory */
	  /* bind methods of memory */
	    BindLibrary((void*)code, (void*)&i); /* do the binding to the relocated code */

	    //MyPic_Test();
	    MyLib_Init();
	    j = MyLib_Mul2(55);//本来は<.plt+0x30>
	    i = MyLib_Calc(30);//本来は<.plt+0x20>

	    /* Enter an infinite loop, just incrementing a counter. */
	    while(1) {
	        i++;
	        j++;
	        __asm volatile ("nop");
	    }



//		  HAL_Delay(125);
//		  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
//		  HAL_Delay(125);
//		  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the peripherals clocks
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_LPUART1;
  PeriphClkInit.Lpuart1ClockSelection = RCC_LPUART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
