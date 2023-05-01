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
#include "usb_device.h"
#include "usb_host.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbh_hid.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define KBD_LEFT_CTRL 0x01
#define KBD_LEFT_SHIFT 0x02
#define KBD_LEFT_ALT 0x04
#define KBD_LEFT_GUI 0x08
#define KBD_RIGHT_CTRL 0x10
#define KBD_RIGHT_SHIFT 0x20
#define KBD_RIGHT_ALT 0x40
#define KBD_RIGHT_GUI 0x80
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
extern USBD_HandleTypeDef hUsbDeviceFS;
char uart_buff[100];
uint8_t HID_Buffer[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
void MX_USB_HOST_Process(void);

/* USER CODE BEGIN PFP */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void type_hello_world(void);
void send_key(uint8_t modifier, uint8_t key);
void emoji(char key);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
	int len = sprintf(uart_buff, "ISR callback.\r\n");
	HAL_UART_Transmit(&huart2, (uint8_t *)uart_buff, len, 100);
}

void USBH_HID_EventCallback(USBH_HandleTypeDef *phost)
{
	if (USBH_HID_GetDeviceType(phost) == HID_KEYBOARD) // if the HID is Mouse
	{
		HID_KEYBD_Info_TypeDef *Keyboard_Info = USBH_HID_GetKeybdInfo(phost); // get the info
		uint8_t key = USBH_HID_GetASCIICode(Keyboard_Info);					  // get the key pressed
		int len = sprintf(uart_buff, "Key Pressed = %c\r\n", key);
		HAL_UART_Transmit(&huart2, (uint8_t *)uart_buff, len, 100);

		uint8_t modifier_keys = 0U;
		int lshift, rshift, lctrl, rctrl, lalt, ralt, lgui, rgui;
		if (Keyboard_Info->lshift == 1U)
		{
			modifier_keys |= KBD_LEFT_SHIFT;
			lshift = 1;
		}
		else
			lshift = 0;
		if (Keyboard_Info->rshift == 1U)
		{
			modifier_keys |= KBD_RIGHT_SHIFT;
			rshift = 1;
		}
		else
			rshift = 0;
		if (Keyboard_Info->lctrl == 1U)
		{
			modifier_keys |= KBD_LEFT_CTRL;
			lctrl = 1;
		}
		else
			lctrl = 0;
		if (Keyboard_Info->rctrl == 1U)
		{
			modifier_keys |= KBD_RIGHT_CTRL;
			rctrl = 1;
		}
		else
			rctrl = 0;
		if (Keyboard_Info->lalt == 1U)
		{
			modifier_keys |= KBD_LEFT_ALT;
			lalt = 1;
		}
		else
			lalt = 0;
		if (Keyboard_Info->ralt == 1U)
		{
			modifier_keys |= KBD_RIGHT_ALT;
			ralt = 1;
		}
		else
			ralt = 0;
		if (Keyboard_Info->lgui == 1U)
		{
			modifier_keys |= KBD_LEFT_GUI;
			lgui = 1;
		}
		else
			lgui = 0;
		if (Keyboard_Info->rgui == 1U)
		{
			modifier_keys |= KBD_RIGHT_GUI;
			rgui = 1;
		}
		else
			rgui = 0;
		if ((lctrl == 1 || rctrl == 1) && (lalt == 1 || ralt == 1) && ((key == 'h' || key == 'H') || (key == 'c' || key == 'C') || (key == 'q' || key == 'Q')))
		{
			if (key == 'h' || key == 'H')
				type_hello_world();
			else if ((key == 'c' || key == 'C'))
			{
				// char *command = "echo \"Hello World in command line!\"";
				// type_command(&command, 36);
			}
			else if ((key == 'q' || key == 'Q'))
			{
				emoji(key);
			}
		}
		else
		{
			USBD_HID_SendReport(&hUsbDeviceFS, HID_Buffer, sizeof(HID_Buffer));
			send_key(modifier_keys, Keyboard_Info->keys[0]);
		}
	}
}
void send_key(uint8_t modifier, uint8_t key)
{
	HID_Buffer[0] = modifier;
	HID_Buffer[2] = key;
	USBD_HID_SendReport(&hUsbDeviceFS, HID_Buffer, 8);
	HAL_Delay(100);

	// Release all keys
	HID_Buffer[0] = 0x00;
	HID_Buffer[2] = 0x00;
	USBD_HID_SendReport(&hUsbDeviceFS, HID_Buffer, 8);
	HAL_Delay(100);
}

void emoji(char key)
{
	HID_Buffer[0] = 0x08;
	HID_Buffer[2] = 0x37;
	USBD_HID_SendReport(&hUsbDeviceFS, HID_Buffer, sizeof(HID_Buffer));
	HAL_Delay(100);

	HID_Buffer[0] = 0x00;
	HID_Buffer[2] = 0x00;
	USBD_HID_SendReport(&hUsbDeviceFS, HID_Buffer, sizeof(HID_Buffer));

	if (key == 'q')
	{
		send_key(0x00, 0x1a);
		send_key(0x00, 0x04);
		send_key(0x00, 0x19);
		send_key(0x00, 0x0c);
		send_key(0x00, 0x11);
		send_key(0x00, 0x0a);
		send_key(0x00, 0x2c);
		send_key(0x00, 0x0b);
		send_key(0x00, 0x04);
		send_key(0x00, 0x11);
		send_key(0x00, 0x07);
		send_key(0x00, 0x28);
	}
}

void type_hello_world(void)
{
	HID_Buffer[0] = 0x02;
	HID_Buffer[2] = 0x0B; // H
	USBD_HID_SendReport(&hUsbDeviceFS, HID_Buffer, sizeof(HID_Buffer));
	HAL_Delay(50);

	HID_Buffer[0] = 0x00;
	HID_Buffer[2] = 0x08; // e
	USBD_HID_SendReport(&hUsbDeviceFS, HID_Buffer, sizeof(HID_Buffer));
	HAL_Delay(50);

	HID_Buffer[0] = 0x00;
	HID_Buffer[2] = 0x0F; // l
	USBD_HID_SendReport(&hUsbDeviceFS, HID_Buffer, sizeof(HID_Buffer));
	HAL_Delay(50);

	HID_Buffer[0] = 0x00;
	HID_Buffer[2] = 0x00; // Dummy data between ll in Hello
	USBD_HID_SendReport(&hUsbDeviceFS, HID_Buffer, sizeof(HID_Buffer));
	HAL_Delay(50);

	HID_Buffer[0] = 0x00;
	HID_Buffer[2] = 0x0F; // l
	USBD_HID_SendReport(&hUsbDeviceFS, HID_Buffer, sizeof(HID_Buffer));
	HAL_Delay(50);

	HID_Buffer[0] = 0x00;
	HID_Buffer[2] = 0x12; // o
	USBD_HID_SendReport(&hUsbDeviceFS, HID_Buffer, sizeof(HID_Buffer));
	HAL_Delay(50);

	HID_Buffer[0] = 0x00;
	HID_Buffer[2] = 0x2C; // space
	USBD_HID_SendReport(&hUsbDeviceFS, HID_Buffer, sizeof(HID_Buffer));
	HAL_Delay(50);

	HID_Buffer[0] = 0x02;
	HID_Buffer[2] = 0x1A; // W
	USBD_HID_SendReport(&hUsbDeviceFS, HID_Buffer, sizeof(HID_Buffer));
	HAL_Delay(50);

	HID_Buffer[0] = 0x00;
	HID_Buffer[2] = 0x12; // o
	USBD_HID_SendReport(&hUsbDeviceFS, HID_Buffer, sizeof(HID_Buffer));
	HAL_Delay(50);

	HID_Buffer[0] = 0x00;
	HID_Buffer[2] = 0x15; // r
	USBD_HID_SendReport(&hUsbDeviceFS, HID_Buffer, sizeof(HID_Buffer));
	HAL_Delay(50);

	HID_Buffer[0] = 0x00;
	HID_Buffer[2] = 0x0F; // l
	USBD_HID_SendReport(&hUsbDeviceFS, HID_Buffer, sizeof(HID_Buffer));
	HAL_Delay(50);

	HID_Buffer[0] = 0x00;
	HID_Buffer[2] = 0x07; // d
	USBD_HID_SendReport(&hUsbDeviceFS, HID_Buffer, sizeof(HID_Buffer));
	HAL_Delay(50);

	HID_Buffer[0] = 0x02;
	HID_Buffer[2] = 0x1E; // !
	USBD_HID_SendReport(&hUsbDeviceFS, HID_Buffer, sizeof(HID_Buffer));
	HAL_Delay(50);

	HID_Buffer[0] = 0x00;
	HID_Buffer[2] = 0x28; // /n
	USBD_HID_SendReport(&hUsbDeviceFS, HID_Buffer, sizeof(HID_Buffer));
	HAL_Delay(50);

	HID_Buffer[0] = 0x00;
	HID_Buffer[2] = 0x00; // release all the keys pressed
	USBD_HID_SendReport(&hUsbDeviceFS, HID_Buffer, sizeof(HID_Buffer));
	HAL_Delay(50);

	HAL_Delay(250);
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
	MX_USB_DEVICE_Init();
	MX_USB_HOST_Init();
	MX_USART2_UART_Init();
	/* USER CODE BEGIN 2 */

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		/* USER CODE END WHILE */
		MX_USB_HOST_Process();

		/* USER CODE BEGIN 3 */
		if (HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == GPIO_PIN_RESET)
		{
			type_hello_world();
		}
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
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 4;
	RCC_OscInitStruct.PLL.PLLN = 72;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 3;
	RCC_OscInitStruct.PLL.PLLR = 2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
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
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void)
{

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
	if (HAL_UART_Init(&huart2) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : B1_Pin */
	GPIO_InitStruct.Pin = B1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : LD2_Pin */
	GPIO_InitStruct.Pin = LD2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

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

#ifdef USE_FULL_ASSERT
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
