/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Chương trình chính điều khiển LED WS2812B với hiệu ứng ánh sáng và ADC
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <string.h>
#include <stdlib.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// Định nghĩa kiểu liệt kê cho các hiệu ứng ánh sáng
typedef enum {
    COLOR_TRANSITION = 0,  // Chuyển đổi màu mượt mà
    STROBE = 1,            // Nhấp nháy ngẫu nhiên
    PIXEL = 2,             // Thanh pixel di chuyển
    RAINBOW = 3,           // Hiệu ứng cầu vồng
    ADC_GLOW = 4,          // Sáng theo âm thanh (màu magenta)
    ADC_VOLUME_BAR = 5,    // Thanh âm lượng phản ứng âm thanh
    ALL_OFF = 6            // Tắt toàn bộ LED
} EffectType;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// Cấu hình cho LED WS2812B và các tham số hiệu ứng
#define LED_NUMBER         29       // Số lượng LED trong dải
#define BITS_PER_LED       24      // Số bit cho mỗi LED (8 bit mỗi kênh GRB)
#define LED_RESET_SLOTS    48      // Số slot reset để kết thúc frame dữ liệu
#define LED_BUFFER_SIZE    (LED_NUMBER * BITS_PER_LED + LED_RESET_SLOTS) // Kích thước bộ đệm
#define WS_HIGH            70      // Giá trị PWM cho bit 1
#define WS_LOW             35      // Giá trị PWM cho bit 0
#define DMA_TIMEOUT_MS     2000    // Thời gian chờ DMA (ms)
#define FADE_STEPS         50      // Số bước cho hiệu ứng nhạt dần
#define HUE_CHANGE_RATE    5       // Tốc độ thay đổi màu trong COLOR_TRANSITION
#define MAX_BRIGHTNESS     40      // Độ sáng tối đa (giảm để tiết kiệm năng lượng)
#define ADC_GAIN           1.0f    // Độ nhạy tín hiệu âm thanh
#define ADC_THRESHOLD      1900    // Ngưỡng tín hiệu ADC để kích hoạt hiệu ứng
#define RAINBOW_SPEED      3       // Tốc độ chuyển động cầu vồng
#define ADC_UPDATE_RATE    2       // Cập nhật ADC mỗi 2 chu kỳ (~100ms)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
// Cấu trúc cho Timer, DMA và ADC
TIM_HandleTypeDef htim4;
DMA_HandleTypeDef hdma_tim4_ch1;
ADC_HandleTypeDef hadc1;

// Bộ đệm lưu dữ liệu PWM cho LED
uint16_t ws2812Buffer[LED_BUFFER_SIZE];

/* USER CODE BEGIN PV */
// Biến theo dõi trạng thái nút bấm (ít dùng)
volatile uint8_t buttonPressed = 0;
// Biến lưu hiệu ứng hiện tại, mặc định là COLOR_TRANSITION
static EffectType currentEffect = COLOR_TRANSITION;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
// Khai báo các hàm khởi tạo phần cứng
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM4_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */
// Hàm tạo hiệu ứng ánh sáng và chọn chế độ
void WS2812_SetColor(void);
int GetEffectMode(void);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
// Hàm tạo màu sắc và hiệu ứng cho LED
void WS2812_SetColor(void) {
    // Biến static để giữ giá trị giữa các lần gọi
    static uint32_t animationCounter = 0;    // Đếm chu kỳ hiệu ứng
    static uint16_t hueCounter = 0;          // Theo dõi sắc thái màu
    static uint8_t fadeValue = 0;            // Giá trị độ sáng cho fade
    static int8_t fadeDirection = 1;         // Hướng fade (1: tăng, -1: giảm)
    static uint8_t strobeState = 0;          // Trạng thái nhấp nháy
    static uint8_t fadeStep = 0;             // Bước fade hiện tại
    static uint8_t strobeRed = 0;            // Màu đỏ cho STROBE
    static uint8_t strobeGreen = 0;          // Màu xanh lá cho STROBE
    static uint8_t strobeBlue = 0;           // Màu xanh dương cho STROBE
    static uint8_t volumeLevel = 0;          // Số LED sáng trong PIXEL
    static int8_t volumeDirection = 1;       // Hướng di chuyển thanh pixel
    static uint16_t volumeHueCounter = 0;    // Sắc thái cho PIXEL/ADC_VOLUME_BAR
    static float smoothedLedCount = 0;       // Số LED sáng làm mượt cho ADC_VOLUME_BAR
    static uint16_t rainbowHueOffset = 0;    // Độ lệch màu cho RAINBOW

    // Xóa bộ đệm để tránh dữ liệu cũ
    memset(ws2812Buffer, 0, sizeof(ws2812Buffer));
    int idx = 0; // Chỉ số bộ đệm

    // Kiểm tra nếu hiệu ứng thay đổi
    EffectType prevEffect = currentEffect;
    currentEffect = (EffectType)GetEffectMode();
    if (currentEffect != prevEffect) {
        // Đặt lại biến khi chuyển hiệu ứng
        if (currentEffect == COLOR_TRANSITION) {
            hueCounter = 0;
            fadeValue = 0;
            fadeDirection = 1;
            fadeStep = 0;
        } else if (currentEffect == STROBE) {
            strobeState = 0;
            strobeRed = 0;
            strobeGreen = 0;
            strobeBlue = 0;
        } else if (currentEffect == PIXEL) {
            volumeLevel = 0;
            volumeDirection = 1;
            volumeHueCounter = 0;
        } else if (currentEffect == RAINBOW) {
            rainbowHueOffset = 0;
        }
        animationCounter = 0;
    }

    uint8_t green, red, blue; // Giá trị màu GRB
    uint8_t ledCount = 0;     // Số LED sáng trong ADC_VOLUME_BAR

    // Xử lý từng LED trong dải
    for (int i = 0; i < LED_NUMBER; i++) {
        switch (currentEffect) {
            case COLOR_TRANSITION:
                // Cập nhật màu mỗi HUE_CHANGE_RATE chu kỳ
                if (animationCounter % HUE_CHANGE_RATE == 0) {
                    hueCounter++;
                    if (hueCounter >= 256) hueCounter = 0;
                }
                // Cập nhật độ sáng mỗi FADE_STEPS chu kỳ   //  uint8_t hue = (rainbowHueOffset + (i * 255 / LED_NUMBER)) % 256;
                if (animationCounter % FADE_STEPS == 0) {
                    fadeValue += fadeDirection * (MAX_BRIGHTNESS / (FADE_STEPS / 2));
                    fadeStep++;
                    if (fadeStep >= FADE_STEPS) {
                        fadeStep = 0;
                        fadeDirection = -fadeDirection;
                    }
                    if (fadeValue > MAX_BRIGHTNESS) fadeValue = MAX_BRIGHTNESS;
                    if (fadeValue < 0) fadeValue = 0;
                }
                // Tính màu dựa trên hue
                {
                    uint8_t hue = hueCounter;
                    if (hue < 85) {
                        red = hue * 3;
                        green = 255 - hue * 3;
                        blue = 0;
                    } else if (hue < 170) {
                        hue -= 85;
                        red = 255 - hue * 3;
                        green = 0;
                        blue = hue * 3;
                    } else {
                        hue -= 170;
                        red = 0;
                        green = hue * 3;
                        blue = 255 - hue * 3;
                    }
                    // Áp dụng độ sáng
                    red = (red * fadeValue) >> 8;
                    green = (green * fadeValue) >> 8;
                    blue = (blue * fadeValue) >> 8;
                }
                break;

            case STROBE:
                // Bật/tắt LED với màu ngẫu nhiên
                if (strobeState) {
                    red = strobeRed;
                    green = strobeGreen;
                    blue = strobeBlue;
                } else {
                    red = 0;
                    green = 0;
                    blue = 0;
                }
                break;

            case PIXEL:
                // Hiển thị số LED sáng theo volumeLevel
                if (i < volumeLevel) {
                    uint8_t hue = (volumeHueCounter + (i * 8)) % 256;
                    if (hue < 85) {
                        red = hue * 3;
                        green = 255 - hue * 3;
                        blue = 0;
                    } else if (hue < 170) {
                        hue -= 85;
                        red = 255 - hue * 3;
                        green = 0;
                        blue = hue * 3;
                    } else {
                        hue -= 170;
                        red = 0;
                        green = hue * 3;
                        blue = 255 - hue * 3;
                    }
                } else {
                    red = 0;
                    green = 0;
                    blue = 0;
                }
                break;

            case RAINBOW:
                // Tạo hiệu ứng cầu vồng với màu thay đổi theo vị trí
                {
                    uint8_t hue = (rainbowHueOffset + (i * 255 / LED_NUMBER)) % 256;
                    if (hue < 85) {
                        red = hue * 3;
                        green = 255 - hue * 3;
                        blue = 0;
                    } else if (hue < 170) {
                        hue -= 85;
                        red = 255 - hue * 3;
                        green = 0;
                        blue = hue * 3;
                    } else {
                        hue -= 170;
                        red = 0;
                        green = hue * 3;
                        blue = 255 - hue * 3;
                    }
                    // Áp dụng độ sáng tối đa
                    red = (red * MAX_BRIGHTNESS) >> 8;
                    green = (green * MAX_BRIGHTNESS) >> 8;
                    blue = (blue * MAX_BRIGHTNESS) >> 8;
                }
                break;

            case ADC_GLOW:
                {
                    static uint8_t brightness = 0;
                    // Cập nhật ADC mỗi ADC_UPDATE_RATE chu kỳ
                    if (animationCounter % ADC_UPDATE_RATE == 0) {
                        HAL_ADC_Start(&hadc1);
                        if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK) {
                            uint32_t adcValue = HAL_ADC_GetValue(&hadc1);
                            // Tính độ sáng dựa trên ADC
                            brightness = 0;
                            if (adcValue >= ADC_THRESHOLD) {
                                adcValue = adcValue * ADC_GAIN;
                                if (adcValue > 4095) adcValue = 4095;
                                brightness = (uint8_t)((adcValue - ADC_THRESHOLD) * MAX_BRIGHTNESS / (4095 - ADC_THRESHOLD));
                                if (brightness > MAX_BRIGHTNESS) brightness = MAX_BRIGHTNESS;
                            }
                        }
                        HAL_ADC_Stop(&hadc1);
                    }
                    // Sử dụng màu magenta
                    red = 255;
                    green = 0;
                    blue = 255;
                    // Áp dụng độ sáng từ ADC
                    red = (red * brightness) >> 8;
                    green = (green * brightness) >> 8;
                    blue = (blue * brightness) >> 8;
                }
                break;

            case ADC_VOLUME_BAR:
                {
                    static const float SMOOTHING_FACTOR_UP = 8.0f;   // Hệ số làm mượt khi tăng
                    static const float SMOOTHING_FACTOR_DOWN = 8.0f; // Hệ số làm mượt khi giảm

                    // Đọc giá trị ADC
                    HAL_ADC_Start(&hadc1);
                    if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK) {
                        uint32_t adcValue = HAL_ADC_GetValue(&hadc1);
                        float avgAdcValue = adcValue;
                        // Tính số LED sáng dựa trên ADC
                        float targetLedCount = 0;
                        if (avgAdcValue >= ADC_THRESHOLD) {
                            float adjustedAdcValue = avgAdcValue * ADC_GAIN;
                            if (adjustedAdcValue > 4095) adjustedAdcValue = 4095;
                            targetLedCount = (adjustedAdcValue * LED_NUMBER) / 4096;
                            if (targetLedCount > LED_NUMBER) targetLedCount = LED_NUMBER;
                        }
                        // Làm mượt số LED sáng
                        if (smoothedLedCount < targetLedCount) {
                            smoothedLedCount += (targetLedCount - smoothedLedCount) / SMOOTHING_FACTOR_UP;
                        } else {
                            smoothedLedCount -= (smoothedLedCount - targetLedCount) / SMOOTHING_FACTOR_DOWN;
                        }
                        ledCount = (uint8_t)smoothedLedCount;
                    }
                    HAL_ADC_Stop(&hadc1);

                    // Gán màu cho LED sáng
                    if (i < ledCount) {
                        uint8_t hue = (volumeHueCounter + (i * 8)) % 256;
                        if (hue < 85) {
                            red = hue * 3;
                            green = 255 - hue * 3;
                            blue = 0;
                        } else if (hue < 170) {
                            hue -= 85;
                            red = 255 - hue * 3;
                            green = 0;
                            blue = hue * 3;
                        } else {
                            hue -= 170;
                            red = 0;
                            green = hue * 3;
                            blue = 255 - hue * 3;
                        }
                    } else {
                        red = 0;
                        green = 0;
                        blue = 0;
                    }
                }
                break;

            case ALL_OFF:
                // Tắt toàn bộ LED
                red = 0;
                green = 0;
                blue = 0;
                break;
        }

        // Ghi dữ liệu màu vào bộ đệm (theo thứ tự GRB)
        if (idx + 24 <= LED_BUFFER_SIZE) {
            for (int8_t j = 7; j >= 0; j--)
                ws2812Buffer[idx++] = (green & (1 << j)) ? WS_HIGH : WS_LOW;
            for (int8_t j = 7; j >= 0; j--)
                ws2812Buffer[idx++] = (red & (1 << j)) ? WS_HIGH : WS_LOW;
            for (int8_t j = 7; j >= 0; j--)
                ws2812Buffer[idx++] = (blue & (1 << j)) ? WS_HIGH : WS_LOW;
        } else {
            break;
        }
    }

    // Thêm slot reset vào cuối bộ đệm
    for (int i = 0; i < LED_RESET_SLOTS && idx < LED_BUFFER_SIZE; i++)
        ws2812Buffer[idx++] = 0;

    // Cập nhật trạng thái hiệu ứng
    animationCounter++;
    if (currentEffect == STROBE && animationCounter % 40 == 0) {
        strobeState = !strobeState;
        if (strobeState) {
            // Tạo màu ngẫu nhiên cho STROBE
            strobeRed = rand() % 256;
            strobeGreen = rand() % 256;
            strobeBlue = rand() % 256;
        }
    }
    if ((currentEffect == PIXEL || currentEffect == ADC_VOLUME_BAR) && animationCounter % 20 == 0) {
        if (currentEffect == PIXEL) {
            // Cập nhật thanh pixel
            volumeLevel += volumeDirection * 2;
            if (volumeLevel >= LED_NUMBER) {
                volumeLevel = LED_NUMBER;
                volumeDirection = -1;
            } else if (volumeLevel <= 0) {
                volumeLevel = 0;
                volumeDirection = 1;
            }
        }
        volumeHueCounter += 5;
        if (volumeHueCounter >= 256) volumeHueCounter = 0;
    }
    if (currentEffect == RAINBOW && animationCounter % RAINBOW_SPEED == 0) {
        // Cập nhật hiệu ứng cầu vồng
        rainbowHueOffset += 5;
        if (rainbowHueOffset >= 256) rainbowHueOffset = 0;
    }
}

// Hàm chọn chế độ hiệu ứng dựa trên nút bấm
int GetEffectMode(void) {
    static int lastEffect = 0; // Hiệu ứng hiện tại
    static uint32_t lastPress = 0; // Thời gian nhấn nút trước
    uint32_t currentTime = HAL_GetTick();

    // Kiểm tra nút bấm với chống dội phím (300ms)
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET && currentTime - lastPress > 300) {
        lastEffect = (lastEffect + 1) % 7; // Chuyển hiệu ứng (0-6)
        lastPress = currentTime;
    }
    return lastEffect;
}
/* USER CODE END 0 */

/**
  * @brief  Điểm vào của chương trình
  * @retval int
  */
int main(void)
{
    // Khởi tạo HAL
    HAL_Init();
    // Cấu hình xung nhịp hệ thống
    SystemClock_Config();
    // Cấu hình GPIO
    MX_GPIO_Init();
    // Cấu hình DMA
    MX_DMA_Init();
    // Cấu hình Timer4
    MX_TIM4_Init();
    // Cấu hình ADC1
    MX_ADC1_Init();

    // Khởi tạo seed ngẫu nhiên
    srand(HAL_GetTick());A
    // Tạo hiệu ứng ánh sáng lần đầu
    WS2812_SetColor();
    // Bắt đầu truyền DMA cho PWM
    HAL_TIM_PWM_Start_DMA(&htim4, TIM_CHANNEL_1, (uint32_t *)ws2812Buffer, LED_BUFFER_SIZE);

    // Vòng lặp chính
    while (1)
    {
        // Chờ DMA hoàn thành
        uint32_t timeout = DMA_TIMEOUT_MS;
        while (HAL_DMA_GetState(&hdma_tim4_ch1) != HAL_DMA_STATE_READY && timeout--) {
            HAL_Delay(1);
        }
        // Xử lý lỗi DMA
        if (timeout == 0) {
            HAL_TIM_PWM_Stop_DMA(&htim4, TIM_CHANNEL_1);
            HAL_Delay(10);
            HAL_TIM_PWM_Start_DMA(&htim4, TIM_CHANNEL_1, (uint32_t *)ws2812Buffer, LED_BUFFER_SIZE);
            continue;
        }
        // Dừng PWM để cập nhật
        HAL_TIM_PWM_Stop_DMA(&htim4, TIM_CHANNEL_1);
        // Cập nhật hiệu ứng
        WS2812_SetColor();
        // Truyền lại bộ đệm nếu DMA sẵn sàng
        if (HAL_DMA_GetState(&hdma_tim4_ch1) == HAL_DMA_STATE_READY) {
            HAL_TIM_PWM_Start_DMA(&htim4, TIM_CHANNEL_1, (uint32_t *)ws2812Buffer, LED_BUFFER_SIZE);
        }
        // Đợi 20ms trước khi cập nhật tiếp
        HAL_Delay(20);
    }
}

/**
  * @brief Cấu hình xung nhịp hệ thống
  * @retval None
  */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    // Kích hoạt clock cho PWR
    __HAL_RCC_PWR_CLK_ENABLE();
    // Cấu hình điện áp
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    // Cấu hình HSE và PLL
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 4;
    RCC_OscInitStruct.PLL.PLLN = 100;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 4;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
        Error_Handler();

    // Cấu hình clock cho SYSCLK, HCLK, PCLK1, PCLK2
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
        Error_Handler();
}

/**
  * @brief Khởi tạo ADC1
  * @retval None
  */
static void MX_ADC1_Init(void)
{
    ADC_ChannelConfTypeDef sConfig = {0};

    // Cấu hình ADC1
    hadc1.Instance = ADC1;
    hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc1.Init.Resolution = ADC_RESOLUTION_12B;
    hadc1.Init.ScanConvMode = DISABLE;
    hadc1.Init.ContinuousConvMode = DISABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion = 1;
    hadc1.Init.DMAContinuousRequests = DISABLE;
    hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    if (HAL_ADC_Init(&hadc1) != HAL_OK)
        Error_Handler();

    // Cấu hình kênh ADC (PA1)
    sConfig.Channel = ADC_CHANNEL_1;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_56CYCLES;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
        Error_Handler();
}

/**
  * @brief Khởi tạo Timer4
  * @retval None
  */
static void MX_TIM4_Init(void)
{
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};

    // Cấu hình Timer4 cho PWM
    htim4.Instance = TIM4;
    htim4.Init.Prescaler = 0;
    htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim4.Init.Period = 104 - 1; // Chu kỳ 800kHz cho WS2812B
    htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
        Error_Handler();

    // Cấu hình kênh PWM
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
        Error_Handler();

    HAL_TIM_MspPostInit(&htim4);
}

/**
  * @brief Kích hoạt clock cho DMA
  */
static void MX_DMA_Init(void)
{
    // Kích hoạt clock DMA1
    __HAL_RCC_DMA1_CLK_ENABLE();
    // Cấu hình ngắt DMA
    HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
}

/**
  * @brief Khởi tạo GPIO
  * @retval None
  */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // Kích hoạt clock cho GPIO
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    // Cấu hình PA0 (nút bấm)
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // Cấu hình PA1 (ADC)
    GPIO_InitStruct.Pin = GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // Cấu hình PD12 (PWM cho LED)
    GPIO_InitStruct.Pin = GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
}

/**
  * @brief Xử lý lỗi
  * @retval None
  */
void Error_Handler(void)
{
    // Vô hiệu hóa ngắt và lặp vô hạn
    __disable_irq();
    while (1) {}
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief Xử lý lỗi debug
  */
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif /* USE_FULL_ASSERT */