/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <stdint.h>
#include <stddef.h>
#include "usbd_audio_if.h"

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
I2S_HandleTypeDef hi2s2;
DMA_HandleTypeDef hdma_spi2_tx;

/* USER CODE BEGIN PV */
/* USB Audio -> I2S bridge: 48 kHz, stereo, signed 16-bit PCM. */
#define USB_AUDIO_I2S_FRAMES       1920U
#define USB_AUDIO_I2S_SAMPLES      (USB_AUDIO_I2S_FRAMES * 2U)
#define USB_AUDIO_RING_FRAMES      8192U

/* Circular DMA consumes 40 ms of stereo audio; each half is 20 ms. */
__ALIGN_BEGIN static uint16_t i2s_dma_buffer[USB_AUDIO_I2S_SAMPLES] __ALIGN_END;
/* SPSC ring: USB callback writes, I2S DMA callback reads. */
static uint16_t audio_ring[USB_AUDIO_RING_FRAMES * 2U];
static volatile uint32_t audio_ring_write = 0U;
static volatile uint32_t audio_ring_read = 0U;
static volatile uint32_t audio_ring_overruns = 0U;
static volatile uint32_t audio_ring_underruns = 0U;
static volatile uint32_t audio_i2s_started = 0U;

#define DEBOUNCE_MS 20U
uint8_t button_state = 1;
uint8_t button_event = 0;

uint8_t last_reading = 1;
uint32_t last_change_time = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2S2_Init(void);
/* USER CODE BEGIN PFP */
void PK002_Audio_PushUsbPacket(const uint8_t *data, uint32_t size);
void PK002_Audio_Start(void);
void PK002_Audio_DmaHalfCallback(void);
void PK002_Audio_DmaFullCallback(void);


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/*
 * Called from the USB isochronous OUT callback.
 * USB Audio samples are little-endian interleaved L/R signed 16-bit PCM.
 * Copy complete stereo frames to our own ring before the middleware reuses
 * its USB receive buffer for the next packet.
 */
void PK002_Audio_PushUsbPacket(const uint8_t *data, uint32_t size)
{
    if ((data == NULL) || (size < 4U))
        return;

    size &= ~3UL; /* Only complete L+R frames (4 bytes) are processed. */

    for (uint32_t i = 0U; i < size; i += 4U)
    {
        uint32_t next = (audio_ring_write + 1U) % USB_AUDIO_RING_FRAMES;

        if (next == audio_ring_read)
        {
            audio_ring_overruns++;
            continue; /* Drop newest frame; never overwrite unread samples. */
        }

        audio_ring[2U * audio_ring_write] =
            (uint16_t)((uint16_t)data[i] | ((uint16_t)data[i + 1U] << 8));
        audio_ring[2U * audio_ring_write + 1U] =
            (uint16_t)((uint16_t)data[i + 2U] | ((uint16_t)data[i + 3U] << 8));

        audio_ring_write = next;
    }
}

/*
 * ST's USB middleware calls AUDIO_CMD_START after receiving the initial USB
 * buffer. Thus samples are already queued before the I2S output clock starts.
 */
void PK002_Audio_Start(void)
{
    if (audio_i2s_started != 0U)
        return;

    for (uint32_t i = 0U; i < USB_AUDIO_I2S_SAMPLES; i++)
        i2s_dma_buffer[i] = 0U;

    /* Do not reset the ring here: USB packets were queued before START. */
    if (HAL_I2S_Transmit_DMA(&hi2s2, i2s_dma_buffer,
                             USB_AUDIO_I2S_SAMPLES) == HAL_OK)
    {
        audio_i2s_started = 1U;
    }
    else
    {
        Error_Handler();
    }
}

/*
 * Refill only the half DMA has finished transmitting.
 * I2S PLL is configured for 48 kHz, so input and output frames are copied 1:1.
 * Keep this DMA interrupt callback lightweight to avoid delaying USB service.
 */
static void PK002_Audio_FillHalf(uint32_t first_sample)
{
    const uint32_t frames = USB_AUDIO_I2S_FRAMES / 2U;

    for (uint32_t f = 0U; f < frames; f++)
    {
        const uint32_t dst = first_sample + (2U * f);
        const uint32_t read = audio_ring_read;
        const uint32_t write = audio_ring_write;

        if (read != write)
        {
            i2s_dma_buffer[dst] = audio_ring[2U * read];
            i2s_dma_buffer[dst + 1U] = audio_ring[2U * read + 1U];
            audio_ring_read = (read + 1U) % USB_AUDIO_RING_FRAMES;
        }
        else
        {
            /* Output silence on underrun instead of replaying stale samples. */
            i2s_dma_buffer[dst] = 0U;
            i2s_dma_buffer[dst + 1U] = 0U;
            audio_ring_underruns++;
        }
    }
}

void PK002_Audio_DmaHalfCallback(void)
{
    if (audio_i2s_started != 0U)
        PK002_Audio_FillHalf(0U);
}

void PK002_Audio_DmaFullCallback(void)
{
    if (audio_i2s_started != 0U)
        PK002_Audio_FillHalf(USB_AUDIO_I2S_SAMPLES / 2U);
}


void Button_Update(void)
{
    uint8_t reading =
        HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);

    if (reading != last_reading)
    {
        last_change_time = HAL_GetTick();
        last_reading = reading;
    }

    if ((HAL_GetTick() - last_change_time) >= DEBOUNCE_MS)
    {
        if (button_state != reading)
        {
            button_state = reading;

            // Zdarzenie tylko przy wciśnięciu
            if (button_state == GPIO_PIN_RESET)
            {
                button_event = 1;
            }
        }
    }
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
  MX_DMA_Init();
  MX_I2S2_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 123;
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
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2S2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S2_Init(void)
{

  /* USER CODE BEGIN I2S2_Init 0 */

  /* USER CODE END I2S2_Init 0 */

  /* USER CODE BEGIN I2S2_Init 1 */

  /* USER CODE END I2S2_Init 1 */
  hi2s2.Instance = SPI2;
  hi2s2.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s2.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s2.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s2.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
  hi2s2.Init.AudioFreq = I2S_AUDIOFREQ_48K;
  hi2s2.Init.CPOL = I2S_CPOL_LOW;
  hi2s2.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s2.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S2_Init 2 */

  /* USER CODE END I2S2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);

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
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
