/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Smart Water Tank Monitoring and Pump Control
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdio.h>

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
TIM_HandleTypeDef htim2;
UART_HandleTypeDef huart1;

/* Application variables ----------------------------------------------------*/
volatile uint32_t Distance = 0;
volatile uint8_t PumpState = 0;
volatile uint8_t UltrasonicValid = 0;
volatile uint8_t Percentage = 0;

/* -------------------------------------------------------------------------- */
/*                      PROJECT CALIBRATION VALUES                           */
/* -------------------------------------------------------------------------- */
/*
 * The Wokwi ultrasonic control is treated as a 0-400 cm distance range.
 * For this simulation:
 *   0 cm   -> tank considered FULL  -> 100%
 *   400 cm -> tank considered EMPTY -> 0%
 *
 * For the physical prototype, keep the same formula but change these two
 * values to the measured sensor-to-water distances at 100% and 0% tank level.
 */
#define SENSOR_MAX_DISTANCE_CM       450U
#define TANK_FULL_DISTANCE_CM          0U
#define TANK_EMPTY_DISTANCE_CM       400U

#define PUMP_ON_THRESHOLD_PERCENT     20U
#define PUMP_OFF_THRESHOLD_PERCENT    95U

#define OLED_I2C_ADDRESS              (0x3CU << 1)
#define TIM2_PWM_PERIOD_US            60000U
#define TIM2_PWM_PULSE_US                10U

#define I2C_TIMEOUT_MS                   100U

static uint32_t RegisterTick(void)
{
    return uwTick;
}

static void RegisterDelay(uint32_t milliseconds)
{
    uint32_t start = RegisterTick();

    while ((RegisterTick() - start) < milliseconds)
    {
    }
}

static uint16_t RegisterStringLength(const char *string)
{
    uint16_t length = 0U;

    while (string[length] != '\0')
    {
        length++;
    }

    return length;
}

static uint8_t I2C1_Wait(uint32_t flag, uint32_t state, uint32_t timeout)
{
    uint32_t start = RegisterTick();

    while (((I2C1->SR1 & flag) != 0U) != (state != 0U))
    {
        if ((RegisterTick() - start) >= timeout)
        {
            I2C1->CR1 |= I2C_CR1_STOP;
            return 0U;
        }
    }

    return 1U;
}

static uint8_t I2C1_Write(uint8_t control, const uint8_t *data, uint16_t length)
{
    uint32_t start = RegisterTick();

    I2C1->CR1 |= I2C_CR1_START;
    while ((I2C1->SR1 & I2C_SR1_SB) == 0U)
    {
        if ((RegisterTick() - start) >= I2C_TIMEOUT_MS)
        {
            I2C1->CR1 |= I2C_CR1_STOP;
            return 0U;
        }
    }

    I2C1->DR = OLED_I2C_ADDRESS;
    if (!I2C1_Wait(I2C_SR1_ADDR, 1U, I2C_TIMEOUT_MS))
    {
        return 0U;
    }
    (void)I2C1->SR2;

    I2C1->DR = control;
    if (!I2C1_Wait(I2C_SR1_TXE, 1U, I2C_TIMEOUT_MS))
    {
        return 0U;
    }

    for (uint16_t index = 0U; index < length; index++)
    {
        I2C1->DR = data[index];
        if (!I2C1_Wait(I2C_SR1_TXE, 1U, I2C_TIMEOUT_MS))
        {
            return 0U;
        }
    }

    if (!I2C1_Wait(I2C_SR1_BTF, 1U, I2C_TIMEOUT_MS))
    {
        return 0U;
    }

    I2C1->CR1 |= I2C_CR1_STOP;
    return 1U;
}

static uint8_t I2C1_WriteCommand(uint8_t command)
{
    return I2C1_Write(0x00U, &command, 1U);
}

static void USART1_Write(const char *text)
{
    uint16_t length = RegisterStringLength(text);

    for (uint16_t index = 0U; index < length; index++)
    {
        while ((USART1->SR & USART_SR_TXE) == 0U)
        {
        }
        USART1->DR = (uint8_t)text[index];
    }

    while ((USART1->SR & USART_SR_TC) == 0U)
    {
    }
}

/* -------------------------------------------------------------------------- */
/*                         MINIMAL OLED DRIVER                                */
/* -------------------------------------------------------------------------- */

static uint8_t SSD1306_Buffer[1024];
static uint8_t CurrentX = 0;
static uint8_t CurrentY = 0;

/* 5x7 ASCII Font - Characters 32 to 91 only.
 * The original table does not contain lowercase characters, so all OLED
 * strings below intentionally use uppercase ASCII characters only.
 */
static const uint8_t Font5x7[] =
{
    0x00,0x00,0x00,0x00,0x00, /* space */
    0x00,0x00,0x5F,0x00,0x00, /* ! */
    0x00,0x07,0x00,0x07,0x00, /* " */
    0x14,0x7F,0x14,0x7F,0x14, /* # */
    0x24,0x2A,0x7F,0x2A,0x12, /* $ */
    0x23,0x13,0x08,0x64,0x62, /* % */
    0x36,0x49,0x55,0x22,0x50, /* & */
    0x00,0x05,0x03,0x00,0x00, /* ' */
    0x00,0x1C,0x22,0x41,0x00, /* ( */
    0x00,0x41,0x22,0x1C,0x00, /* ) */
    0x08,0x2A,0x1C,0x2A,0x08, /* * */
    0x08,0x08,0x3E,0x08,0x08, /* + */
    0x00,0x50,0x30,0x00,0x00, /* , */
    0x08,0x08,0x08,0x08,0x08, /* - */
    0x00,0x60,0x60,0x00,0x00, /* . */
    0x20,0x10,0x08,0x04,0x02, /* / */

    0x3E,0x51,0x49,0x45,0x3E, /* 0 */
    0x00,0x42,0x7F,0x40,0x00, /* 1 */
    0x42,0x61,0x51,0x49,0x46, /* 2 */
    0x21,0x41,0x45,0x4B,0x31, /* 3 */
    0x18,0x14,0x12,0x7F,0x10, /* 4 */
    0x27,0x45,0x45,0x45,0x39, /* 5 */
    0x3C,0x4A,0x49,0x49,0x30, /* 6 */
    0x01,0x71,0x09,0x05,0x03, /* 7 */
    0x36,0x49,0x49,0x49,0x36, /* 8 */
    0x06,0x49,0x49,0x29,0x1E, /* 9 */

    0x00,0x36,0x36,0x00,0x00, /* : */
    0x00,0x56,0x36,0x00,0x00, /* ; */
    0x00,0x08,0x14,0x22,0x41, /* < */
    0x14,0x14,0x14,0x14,0x14, /* = */
    0x41,0x22,0x14,0x08,0x00, /* > */
    0x02,0x01,0x51,0x09,0x06, /* ? */

    0x32,0x49,0x79,0x41,0x3E, /* @ */
    0x7E,0x11,0x11,0x11,0x7E, /* A */
    0x7F,0x49,0x49,0x49,0x36, /* B */
    0x3E,0x41,0x41,0x41,0x22, /* C */
    0x7F,0x41,0x41,0x22,0x1C, /* D */
    0x7F,0x49,0x49,0x49,0x41, /* E */
    0x7F,0x09,0x09,0x01,0x01, /* F */
    0x3E,0x41,0x41,0x51,0x32, /* G */
    0x7F,0x08,0x08,0x08,0x7F, /* H */
    0x00,0x41,0x7F,0x41,0x00, /* I */
    0x20,0x40,0x41,0x3F,0x01, /* J */
    0x7F,0x08,0x14,0x22,0x41, /* K */
    0x7F,0x40,0x40,0x40,0x40, /* L */
    0x7F,0x02,0x04,0x02,0x7F, /* M */
    0x7F,0x04,0x08,0x10,0x7F, /* N */
    0x3E,0x41,0x41,0x41,0x3E, /* O */
    0x7F,0x09,0x09,0x09,0x06, /* P */
    0x3E,0x41,0x51,0x21,0x5E, /* Q */
    0x7F,0x09,0x19,0x29,0x46, /* R */
    0x46,0x49,0x49,0x49,0x31, /* S */
    0x01,0x01,0x7F,0x01,0x01, /* T */
    0x3F,0x40,0x40,0x40,0x3F, /* U */
    0x1F,0x20,0x40,0x20,0x1F, /* V */
    0x7F,0x20,0x10,0x20,0x7F, /* W */
    0x63,0x14,0x08,0x14,0x63, /* X */
    0x03,0x04,0x78,0x04,0x03, /* Y */
    0x61,0x51,0x49,0x45,0x43  /* Z */
};

static void ssd1306_WriteCommand(uint8_t command)
{
    (void)I2C1_WriteCommand(command);
}

static void ssd1306_Fill(uint8_t color)
{
    uint8_t fill = (color == 1U) ? 0xFFU : 0x00U;

    for (uint16_t index = 0U; index < sizeof(SSD1306_Buffer); index++)
    {
        SSD1306_Buffer[index] = fill;
    }
}

static void ssd1306_UpdateScreen(void)
{
    for (uint8_t attempt = 0U; attempt < 2U; attempt++)
    {
        uint8_t status = 1U;

        for (uint8_t page = 0U; page < 8U; page++)
        {
            uint8_t page_start = (uint8_t)(0xB0U + page);

            /* Page addressing is supported by all SSD1306 128x64 modules. */
            if (!I2C1_WriteCommand(page_start) ||
                !I2C1_WriteCommand(0x00U) ||
                !I2C1_WriteCommand(0x10U) ||
                !I2C1_Write(0x40U, &SSD1306_Buffer[128U * page], 128U))
            {
                status = 0U;
                break;
            }
        }

        if (status != 0U)
        {
            return;
        }

        /* Recover a transient/stuck I2C state before retrying the frame. */
        I2C1->CR1 &= ~I2C_CR1_PE;
        I2C1->CR1 |= I2C_CR1_PE;
    }
}

static void ssd1306_Init(void)
{
    RegisterDelay(100U);

    /* Standard SSD1306 128x64 initialization sequence. */
    ssd1306_WriteCommand(0xAE); /* Display OFF */
    ssd1306_WriteCommand(0x20); /* Memory addressing mode */
    ssd1306_WriteCommand(0x02); /* Page addressing */
    ssd1306_WriteCommand(0xB0); /* Page start */
    ssd1306_WriteCommand(0xC8); /* COM scan direction */
    ssd1306_WriteCommand(0x00); /* Low column */
    ssd1306_WriteCommand(0x10); /* High column */
    ssd1306_WriteCommand(0x40); /* Display start line */
    ssd1306_WriteCommand(0x81); /* Contrast */
    ssd1306_WriteCommand(0x7F);
    ssd1306_WriteCommand(0xA1); /* Segment remap */
    ssd1306_WriteCommand(0xA6); /* Normal display */
    ssd1306_WriteCommand(0xA8); /* Multiplex */
    ssd1306_WriteCommand(0x3F);
    ssd1306_WriteCommand(0xA4); /* Resume RAM content */
    ssd1306_WriteCommand(0xD3); /* Display offset */
    ssd1306_WriteCommand(0x00);
    ssd1306_WriteCommand(0xD5); /* Display clock */
    ssd1306_WriteCommand(0x80);
    ssd1306_WriteCommand(0xD9); /* Pre-charge */
    ssd1306_WriteCommand(0xF1);
    ssd1306_WriteCommand(0xDA); /* COM pins */
    ssd1306_WriteCommand(0x12);
    ssd1306_WriteCommand(0xDB); /* VCOMH */
    ssd1306_WriteCommand(0x40);
    ssd1306_WriteCommand(0x8D); /* Charge pump */
    ssd1306_WriteCommand(0x14);
    ssd1306_WriteCommand(0xAF); /* Display ON */

    ssd1306_Fill(0);
    ssd1306_UpdateScreen();
}

static void ssd1306_SetCursor(uint8_t x, uint8_t y)
{
    CurrentX = x;
    CurrentY = y;
}

static void ssd1306_WriteStringScaled(const char *str, uint8_t scale)
{
    while (*str != '\0')
    {
        uint8_t ch = (uint8_t)*str;

        if (ch >= 32U && ch <= 90U)
        {
            uint16_t font_index = (uint16_t)(ch - 32U) * 5U;

            for (uint8_t column = 0U; column < 5U; column++)
            {
                uint8_t column_data = Font5x7[font_index + column];

                for (uint8_t pixel_x = 0U; pixel_x < scale; pixel_x++)
                {
                    uint8_t x = (uint8_t)(CurrentX + column * scale + pixel_x);

                    if (x >= 128U)
                    {
                        continue;
                    }

                    for (uint8_t pixel_y = 0U; pixel_y < 7U * scale; pixel_y++)
                    {
                        if ((column_data & (1U << (pixel_y / scale))) != 0U)
                        {
                            uint8_t y = (uint8_t)(CurrentY + pixel_y);

                            if (y < 64U)
                            {
                                SSD1306_Buffer[(y / 8U) * 128U + x] |=
                                    (uint8_t)(1U << (y % 8U));
                            }
                        }
                    }
                }
            }

            CurrentX = (uint8_t)(CurrentX + 6U * scale);
        }

        str++;
    }
}

/* -------------------------------------------------------------------------- */
/*                       FUNCTION PROTOTYPES                                  */
/* -------------------------------------------------------------------------- */

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);

static uint32_t Read_Ultrasonic(void);
static uint8_t Calculate_Level_Percentage(uint32_t distance_cm);

/* -------------------------------------------------------------------------- */
/*                       ULTRASONIC SENSOR                                    */
/* -------------------------------------------------------------------------- */

static uint32_t Read_Ultrasonic(void)
{
    uint32_t timeout;
    uint32_t start_count;
    uint32_t end_count;
    uint32_t pulse_width;

    UltrasonicValid = 0;
    Distance = 0U;

    /* Wait for the next TIM2 PWM rising edge on PA1. */
    timeout = RegisterTick();
    while (TIM2->CNT >= TIM2_PWM_PULSE_US)
    {
        if ((RegisterTick() - timeout) >= 70U)
        {
            return 0U;
        }
    }

    /* Wait for ECHO to become HIGH. */
    timeout = RegisterTick();
    while ((GPIOA->IDR & GPIO_PIN_0) == 0U)
    {
        if ((RegisterTick() - timeout) >= 50U)
        {
            return 0U;
        }
    }

    start_count = TIM2->CNT;

    /* Wait for ECHO to become LOW. Maximum useful echo at 400 cm is < 25 ms. */
    timeout = RegisterTick();
    while ((GPIOA->IDR & GPIO_PIN_0) != 0U)
    {
        if ((RegisterTick() - timeout) >= 50U)
        {
            return 0U;
        }
    }

    end_count = TIM2->CNT;

    /* Handle the timer wrap safely. */
    if (end_count >= start_count)
    {
        pulse_width = end_count - start_count;
    }
    else
    {
        pulse_width = (TIM2_PWM_PERIOD_US - start_count) + end_count;
    }

    /* Convert echo time (us) to distance (cm), approximately 58 us/cm. */
    Distance = (pulse_width + 29U) / 58U;

     /* Accept the full practical sensor range; tank calibration still maps
         distances at or above 400 cm to empty. */
     if (Distance > SENSOR_MAX_DISTANCE_CM)
    {
        return 0U;
    }

    UltrasonicValid = 1U;
    return Distance;
}

/* -------------------------------------------------------------------------- */
/*                       WATER-LEVEL CALCULATION                              */
/* -------------------------------------------------------------------------- */

static uint8_t Calculate_Level_Percentage(uint32_t distance_cm)
{
    uint32_t span;
    uint32_t level;

    if (TANK_EMPTY_DISTANCE_CM <= TANK_FULL_DISTANCE_CM)
    {
        return 0U;
    }

    if (distance_cm <= TANK_FULL_DISTANCE_CM)
    {
        return 100U;
    }

    if (distance_cm >= TANK_EMPTY_DISTANCE_CM)
    {
        return 0U;
    }

    span = TANK_EMPTY_DISTANCE_CM - TANK_FULL_DISTANCE_CM;

    /*
     * Linear mapping:
     *   full distance  -> 100%
     *   empty distance ->   0%
     *
     * This uses the complete 0-400 cm Wokwi control range with the default
     * calibration values, rather than limiting the calculation to 0-100 cm.
     */
    level = ((TANK_EMPTY_DISTANCE_CM - distance_cm) * 100U) / span;

    if (level > 100U)
    {
        level = 100U;
    }

    return (uint8_t)level;
}

/* -------------------------------------------------------------------------- */
/*                               MAIN                                         */
/* -------------------------------------------------------------------------- */

int main(void)
{
    HAL_Init();

    SystemClock_Config();

    MX_GPIO_Init();
    MX_I2C1_Init();
    MX_TIM2_Init();
    MX_USART1_UART_Init();

    /* Start TIM2 as a 1 MHz time base for trigger and echo timing. */
    TIM2->CR1 |= TIM_CR1_CEN;

    PumpState = 0U;
    GPIOA->BSRR = GPIO_PIN_2 << 16U;

    /* Initialize OLED. */
    ssd1306_Init();

    while (1)
    {
        /* Read ultrasonic distance from the JSN-SR04T. */
        Distance = Read_Ultrasonic();

        /* Convert the complete calibrated distance range into level %. */
        if (UltrasonicValid)
        {
            Percentage = Calculate_Level_Percentage(Distance);
        }

        /* -------------------------------------------------------------- */
        /* Emergency float switch: PA3 LOW = emergency condition          */
        /* -------------------------------------------------------------- */
        if ((GPIOA->IDR & GPIO_PIN_3) == 0U)
        {
            PumpState = 0U;
            GPIOA->BSRR = GPIO_PIN_2 << 16U;
        }
        else if (UltrasonicValid)
        {
            /* Low water level: pump ON below 20%. */
            if (Percentage < PUMP_ON_THRESHOLD_PERCENT)
            {
                PumpState = 1U;
                GPIOA->BSRR = GPIO_PIN_2;
            }
            /* Tank almost full: pump OFF at 95% or above. */
            else if (Percentage >= PUMP_OFF_THRESHOLD_PERCENT)
            {
                PumpState = 0U;
                GPIOA->BSRR = GPIO_PIN_2 << 16U;
            }
            /* Between 20% and 95%, retain the previous pump state. */
        }

        /* -------------------------------------------------------------- */
        /* OLED DISPLAY                                                     */
        /* -------------------------------------------------------------- */
        {
            char oled_buf[32];

            ssd1306_Fill(0);

            ssd1306_SetCursor(0, 0);
            ssd1306_WriteStringScaled("LEVEL", 2U);

            ssd1306_SetCursor(0, 16);
            if (UltrasonicValid)
            {
                snprintf(
                    oled_buf,
                    sizeof(oled_buf),
                    "%u %%",
                    Percentage
                );
            }
            else
            {
                snprintf(
                    oled_buf,
                    sizeof(oled_buf),
                    "LEVEL: ERROR"
                );
            }
            ssd1306_WriteStringScaled(oled_buf, 2U);

            ssd1306_SetCursor(0, 32);
            ssd1306_WriteStringScaled("PUMP", 2U);

            ssd1306_SetCursor(0, 48);
            ssd1306_WriteStringScaled(PumpState ? "ON" : "OFF", 2U);

            ssd1306_UpdateScreen();
        }

        /* -------------------------------------------------------------- */
        /* UART TELEMETRY                                                  */
        /* -------------------------------------------------------------- */
        {
            char uart_buf[80];

            if (UltrasonicValid)
            {
                snprintf(
                    uart_buf,
                    sizeof(uart_buf),
                    "Tank Level: %u %% | Distance: %lu cm | Pump: %s\r\n",
                    Percentage,
                    (unsigned long)Distance,
                    PumpState ? "ON" : "OFF"
                );
            }
            else
            {
                snprintf(
                    uart_buf,
                    sizeof(uart_buf),
                    "Tank Level: ERROR | Distance: NO ECHO | Pump: %s\r\n",
                    PumpState ? "ON" : "OFF"
                );
            }

            USART1_Write(uart_buf);
        }

        RegisterDelay(100U);
    }
}

/* -------------------------------------------------------------------------- */
/*                         SYSTEM CLOCK                                      */
/* -------------------------------------------------------------------------- */

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;

    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType =
        RCC_CLOCKTYPE_HCLK |
        RCC_CLOCKTYPE_SYSCLK |
        RCC_CLOCKTYPE_PCLK1 |
        RCC_CLOCKTYPE_PCLK2;

    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
    {
        Error_Handler();
    }
}

/* -------------------------------------------------------------------------- */
/*                              I2C1                                          */
/* -------------------------------------------------------------------------- */

static void MX_I2C1_Init(void)
{
    I2C1->CR1 = I2C_CR1_SWRST;
    I2C1->CR1 = 0U;
    I2C1->CR2 = 36U;
    I2C1->OAR1 = 0x4000U; /* Reserved bit 14 must be kept high. */
    I2C1->CCR = 180U;
    I2C1->TRISE = 37U;
    I2C1->CR1 = I2C_CR1_PE;
}

/* -------------------------------------------------------------------------- */
/*                              TIM2                                          */
/* -------------------------------------------------------------------------- */

static void MX_TIM2_Init(void)
{
    TIM2->CR1 = 0U;
    TIM2->PSC = 71U; /* 72 MHz / 72 = 1 MHz -> 1 us/tick */
    TIM2->ARR = TIM2_PWM_PERIOD_US - 1U;
    TIM2->CCR2 = TIM2_PWM_PULSE_US;
    TIM2->CCMR1 = (6U << TIM_CCMR1_OC2M_Pos) | TIM_CCMR1_OC2PE;
    TIM2->CCER = TIM_CCER_CC2E;
    TIM2->EGR = TIM_EGR_UG;
}

/* -------------------------------------------------------------------------- */
/*                              USART1                                        */
/* -------------------------------------------------------------------------- */

static void MX_USART1_UART_Init(void)
{
    USART1->CR1 = 0U;
    USART1->CR2 = 0U;
    USART1->CR3 = 0U;
    USART1->BRR = 625U; /* 72 MHz / 115200 baud */
    USART1->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;
}

/* -------------------------------------------------------------------------- */
/*                              GPIO                                          */
/* -------------------------------------------------------------------------- */

static void MX_GPIO_Init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_AFIOEN |
                    RCC_APB2ENR_IOPAEN |
                    RCC_APB2ENR_IOPBEN |
                    RCC_APB2ENR_IOPDEN |
                    RCC_APB2ENR_USART1EN;
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN | RCC_APB1ENR_TIM2EN;

    /* PA0 input floating, PA1 AF push-pull 50 MHz, PA2 output push-pull. */
    GPIOA->CRL &= ~((0xFU << 0U) | (0xFU << 4U) | (0xFU << 8U) | (0xFU << 12U));
    GPIOA->CRL |= (0x4U << 0U) | (0xBU << 4U) | (0x2U << 8U) | (0x8U << 12U);
    GPIOA->ODR |= GPIO_PIN_3;

    /* I2C1 PB6/PB7: alternate-function open-drain, 50 MHz. */
    GPIOB->CRL &= ~((0xFU << 24U) | (0xFU << 28U));
    GPIOB->CRL |= (0xFU << 24U) | (0xFU << 28U);

    /* USART1 PA9 alternate-function push-pull, PA10 input floating. */
    GPIOA->CRH &= ~((0xFU << 4U) | (0xFU << 8U));
    GPIOA->CRH |= (0xBU << 4U) | (0x4U << 8U);

    AFIO->EXTICR[0] &= ~AFIO_EXTICR1_EXTI3;
    EXTI->IMR |= EXTI_IMR_MR3;
    EXTI->FTSR |= EXTI_FTSR_TR3;
    NVIC_SetPriority(EXTI3_IRQn, 0U);
    NVIC_EnableIRQ(EXTI3_IRQn);
}

/* -------------------------------------------------------------------------- */
/*                         FLOAT SWITCH                                      */
/* -------------------------------------------------------------------------- */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_3)
    {
        PumpState = 0U;

        /* Immediate emergency pump shutdown as required by the proposal. */
        GPIOA->BSRR = GPIO_PIN_2 << 16U;
    }
}

/* -------------------------------------------------------------------------- */
/*                              ERROR                                         */
/* -------------------------------------------------------------------------- */

void Error_Handler(void)
{
    __disable_irq();

    while (1)
    {
    }
}

#ifdef USE_FULL_ASSERT

void assert_failed(uint8_t *file, uint32_t line)
{
    (void)file;
    (void)line;
}

#endif
