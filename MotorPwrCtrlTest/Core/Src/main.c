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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FreeRTOS.h"   
#include "task.h"    
#include "queue.h"
#include "semphr.h"

#include "INA226.h"
#include "w25q64.h"

#include "soft_i2c.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* ------TASK PD ----------*/
#define START_TASK_PRIO  0
#define START_STK_SIZE   128

#define MOTOR_TASK_PRIO  3
#define MOTOR_STK_SIZE   256

#define TX_TASK_PRIO  2
#define TX_STK_SIZE   256

#define INA226_TASK_PRIO  2
#define INA226_STK_SIZE   128

#define RX_TASK_PRIO  2
#define RX_STK_SIZE   128

#define FLASH_TASK_PRIO  2
#define FLASH_STK_SIZE   128

#define BUTTON_TASK_PRIO  3
#define BUTTON_STK_SIZE   128

#define PPR 11.0
#define COUNTS_PER_REV  PPR*4.0

#define MAX_VOLTAGE 13.0
#define MAX_CURRENT 1.5

#define PWM_MAX 3599.0f
#define PWM_MIN 0.0f

#define I_MAX 100.0f

#define ADDR_PARAMS  0x000000   // params at sector 0
#define ADDR_FAULT   0x001000   // fault at sector 1

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */
/* TASK Variables ------------------------------------------------------------*/
TaskHandle_t StartTask_Handler;
TaskHandle_t MotorTask_Handler;
TaskHandle_t TxTask_Handler;
TaskHandle_t INA226Task_Handler;
TaskHandle_t RxTask_Handler;
TaskHandle_t FlashTask_Handler;
TaskHandle_t ButtonTask_Handler;

SemaphoreHandle_t BinarySemaphore;

typedef enum { STOP=0, RUN=1, FAULT=2 } MotorState_t;

QueueHandle_t g_UserSpeedQueue;
QueueHandle_t g_INA226Queue;
QueueHandle_t g_FlashQueue;
QueueHandle_t g_StateParQueue;
QueueHandle_t g_FaultQueue;

typedef struct { float voltage; float current; } InaData_t;//v i
typedef struct { float speed; float target; float kp; float ki; float kd; float voltage; float current; float mv; float mc; MotorState_t state; float duty; } Telemetry_t; //state2user
typedef struct { float speed; MotorState_t state; float duty; } FlashData_t; 
typedef struct {  MotorState_t state; } FaultState_t; //state2user
typedef struct {
    float Kp;          // proportional gain
    float Ki;          // integral gain
    float Kd;          // derivative gain
    float integral;    // accumulated error (for I term)
    float prev_measured;  // last measured value (for D term)
} PID_t;

typedef union {
    float    f;
    uint8_t  b[4];
} FloatBytes_t;

// serial command: speed target or PID gain
typedef enum { CMD_SPEED, CMD_KP, CMD_KI, CMD_KD, CMD_MV, CMD_MC } CmdType_t;
typedef struct { CmdType_t type; float value; } Cmd_t;

uint8_t  rx_buf[64];            // DMA writes received bytes here
volatile uint16_t rx_len = 0;   // bytes in the latest frame

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
/* Task function prototypes --------------------------------------------------*/
void start_task(void *pvParameters);
void motor_task(void *pvParameters);
void tx_task(void *pvParameters);
void ina226_task(void *pvParameters);
void rx_task(void *pvParameters);
void flash_task(void *pvParameters);
void button_task(void *pvParameters);

// Compute control output (duty) from target and measured speed.
float pid_update(PID_t *pid, float target, float measured, float dt);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
float pid_update(PID_t *pid, float target, float measured, float dt)
{
    float error = target - measured;

    // I term: accumulate error, then anti-windup clamp
    pid->integral += error * dt;
    if (pid->integral > I_MAX)  pid->integral = I_MAX;
    if (pid->integral < -I_MAX) pid->integral = -I_MAX;

    // D term: derivative on measurement (avoids derivative kick on setpoint change)
    float derivative = -(measured - pid->prev_measured) / dt;

    // P + I + D
    float output = pid->Kp * error + pid->Ki * pid->integral + pid->Kd * derivative;
    pid->prev_measured = measured;   // update AFTER using the old value

    // clamp output to [0, 3599]
    if (output > PWM_MAX) output = PWM_MAX;
    if (output < PWM_MIN) output = PWM_MIN;
    return output;
}

void start_task(void *pvParameters)
{
    taskENTER_CRITICAL();

    // create queues
    g_UserSpeedQueue = xQueueCreate(4, sizeof(Cmd_t));
    g_INA226Queue    = xQueueCreate(2, sizeof(InaData_t));
    g_FlashQueue     = xQueueCreate(2, sizeof(FlashData_t));
    g_StateParQueue  = xQueueCreate(2, sizeof(Telemetry_t));
    g_FaultQueue     = xQueueCreate(2, sizeof(FaultState_t));

    BinarySemaphore = xSemaphoreCreateBinary();

    // create tasks
    xTaskCreate(motor_task,  "motor_task",  MOTOR_STK_SIZE,  NULL, MOTOR_TASK_PRIO,  &MotorTask_Handler);
    xTaskCreate(button_task, "button_task", BUTTON_STK_SIZE, NULL, BUTTON_TASK_PRIO, &ButtonTask_Handler);
    xTaskCreate(tx_task,     "tx_task",     TX_STK_SIZE,     NULL, TX_TASK_PRIO,     &TxTask_Handler);
    xTaskCreate(ina226_task, "ina226_task", INA226_STK_SIZE, NULL, INA226_TASK_PRIO, &INA226Task_Handler);
    xTaskCreate(rx_task,     "rx_task",     RX_STK_SIZE,     NULL, RX_TASK_PRIO,     &RxTask_Handler);
    xTaskCreate(flash_task,  "flash_task",  FLASH_STK_SIZE,  NULL, FLASH_TASK_PRIO,  &FlashTask_Handler);

    taskEXIT_CRITICAL();
    vTaskDelete(NULL);
}


static void UART_Printf(const char *format,...)
{
	char tmp[128];
	va_list argptr;
	va_start(argptr,format);
	vsnprintf(tmp, sizeof(tmp), format, argptr);   // safe: never overflow
	va_end(argptr);
	HAL_UART_Transmit(&huart1,(const uint8_t *)&tmp,strlen(tmp),HAL_MAX_DELAY);
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart == &huart1)
    {
        rx_len = Size;                                     // remember how many bytes
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        vTaskNotifyGiveFromISR(RxTask_Handler, &xHigherPriorityTaskWoken);  // notify rx_task
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}




void motor_task(void *pvParameters)
{
    // State machine: STOP -> RUN -> FAULT -> STOP

    TickType_t lastWakeTime = xTaskGetTickCount();  // for precise 100 Hz

    MotorState_t state = STOP;
    float TargetRPM = 0.0f, duty = 0.0f;
    float max_voltage = MAX_VOLTAGE;   // fault threshold, runtime adjustable
    float max_current = MAX_CURRENT;   // fault threshold, runtime adjustable

    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    __HAL_TIM_MOE_ENABLE(&htim1);

    uint16_t pre_count = __HAL_TIM_GET_COUNTER(&htim3);
    uint16_t fault_flag = 0;
    uint16_t telemetry_cnt = 0;   // throttle telemetry to ~10 Hz
    InaData_t lastIna = {0};   // latest INA226 reading, for telemetry
    PID_t pid = {0};
    pid.Kp = 2.0f, pid.Ki = 5.0f, pid.Kd = 0.0f;

    for (;;)
    {
        vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(10));  // 100 Hz control loop
        BaseType_t val = pdFALSE;

        FlashData_t FlashData;
        InaData_t INA226Data;
        Telemetry_t StateParData;
        FaultState_t FaultData;

        uint16_t cur_count = __HAL_TIM_GET_COUNTER(&htim3);
        int32_t delta = (int32_t)(int16_t)(cur_count - pre_count);
        pre_count = cur_count;
        float CurRPM = (float)delta * 60.0f / (COUNTS_PER_REV * 0.01f);

        // receive command from rx_task (non-blocking): speed target or PID gain
        Cmd_t cmd;
        if (xQueueReceive(g_UserSpeedQueue, &cmd, 0) == pdTRUE)
        {
            switch (cmd.type)
            {
            case CMD_SPEED:
                if (cmd.value >= 0.0f && cmd.value <= 2000.0f)   // safety: reject invalid targets
                {
                    TargetRPM = cmd.value;
                }
                break;
            case CMD_KP:  pid.Kp = cmd.value; break;
            case CMD_KI:  pid.Ki = cmd.value; break;
            case CMD_KD:  pid.Kd = cmd.value; break;
            case CMD_MV:  max_voltage = cmd.value; break;
            case CMD_MC:  max_current = cmd.value; break;
            }
        }

        switch (state)
        {
        case STOP:
            // force motor stop
            duty = 0;
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);

            // receive restored default speed from flash_task
            if (xQueueReceive(g_FlashQueue, &FlashData, 0) == pdTRUE)
            {
                TargetRPM = FlashData.speed;
            }
            // receive button semaphore -> if pressed, state = RUN
            if (BinarySemaphore != NULL)
            {
                val = xSemaphoreTake(BinarySemaphore, 0);
                if (val == pdTRUE)
                {
                    state = RUN;
                }
            }
            break;

        case RUN:
            // receive voltage/current from ina226_task -> fault check
            if (xQueueReceive(g_INA226Queue, &INA226Data, 0) == pdTRUE)
            {
                lastIna = INA226Data;   // remember for telemetry
                float v = INA226Data.voltage;
                float i = INA226Data.current;
                if ((v > max_voltage) || (i > max_current))
                {
                    duty = 0;
                    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, duty);
                    state = FAULT;
                }
            }

            // over-speed safety: stop if too fast (no over-current protection yet)
            if (CurRPM > 2000.0f)
            {
                duty = 0;
                __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, duty);
                state = STOP;
            }
            else
            {
                // PID(target, actual) -> update PWM duty
                duty = pid_update(&pid, TargetRPM, CurRPM, 0.01f);
                __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, duty);
            }

            // receive button semaphore -> if pressed, state = STOP
            if (BinarySemaphore != NULL)
            {
                val = xSemaphoreTake(BinarySemaphore, 0);
                if (val == pdTRUE)
                {
                    state = STOP;
                }
            }
            break;

        case FAULT:
            // record the fault, then force motor stop
            FaultData.state = FAULT;
            duty = 0;
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);

            // send fault record to flash_task (only once)
            if (fault_flag == 0)
            {
                xQueueSend(g_FaultQueue, &FaultData, 0);
                fault_flag++;
            }

            // receive button semaphore -> if pressed, state = STOP
            if (BinarySemaphore != NULL)
            {
                val = xSemaphoreTake(BinarySemaphore, 0);
                if (val == pdTRUE)
                {
                    state = STOP;
                }
            }
            break;
        }

        // send telemetry every 10 cycles (10 Hz, readable while tuning)
        telemetry_cnt++;
        if (telemetry_cnt >= 50)
        {
            telemetry_cnt = 0;
            StateParData.speed = CurRPM;
            StateParData.target = TargetRPM;
            StateParData.kp = pid.Kp;
            StateParData.ki = pid.Ki;
            StateParData.kd = pid.Kd;
            StateParData.voltage = lastIna.voltage;
            StateParData.current = lastIna.current;
            StateParData.mv = max_voltage;
            StateParData.mc = max_current;
            StateParData.state = state;
            StateParData.duty = duty;
            xQueueSend(g_StateParQueue, &StateParData, 0);
        }
    }
}


void tx_task(void *pvParameters)
{
    for (;;)
    {
        Telemetry_t StatePar = {0};
        if (xQueueReceive(g_StateParQueue, &StatePar, pdMS_TO_TICKS(20)) == pdTRUE)
        {
            const char *state = "NULL";
            if (StatePar.state == STOP)
            {
                state = "STOP";
            }
            else if (StatePar.state == RUN)
            {
                state = "RUN";
            }
            else if (StatePar.state == FAULT)
            {
                state = "FAULT";
            }

            UART_Printf("duty=%.1f speed=%.1f target=%.1f kp=%.1f ki=%.1f kd=%.3f v=%.2f i=%.3f mv=%.1f mc=%.2f state=%s\r\n",
                        StatePar.duty, StatePar.speed, StatePar.target, StatePar.kp, StatePar.ki, StatePar.kd,
                        StatePar.voltage, StatePar.current, StatePar.mv, StatePar.mc, state);
        }
    }
}

void ina226_task(void *pvParameters)
{
    InaData_t data = {0};

    Soft_I2C_Init();
    INA226_Init();

    // debug: read manufacturer ID to verify the chip responds (expect 0x5449)
    uint16_t id = INA226_ReadReg(INA226_REG_MFG_ID);
    UART_Printf("INA226 ID=0x%04X\r\n", id);

    for (;;)
    {
        data.voltage = INA226_ReadBusVoltage();
        data.current = INA226_ReadCurrent();

        xQueueSend(g_INA226Queue, &data, 0);   // hand to motor_task for fault check

        vTaskDelay(pdMS_TO_TICKS(100));        // sample every 100 ms
    }
}

void rx_task(void *pvParameters)
{
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rx_buf, sizeof(rx_buf));   // start reception

    for (;;)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);   // block until a frame arrives

        if (rx_len < sizeof(rx_buf)) {
            rx_buf[rx_len] = '\0';                 // null-terminate the frame
        }
        Cmd_t cmd;
        if      (rx_buf[0] == 'k' && rx_buf[1] == '=') { cmd.type = CMD_KP;    cmd.value = (float)atof((char *)&rx_buf[2]); }
        else if (rx_buf[0] == 'i' && rx_buf[1] == '=') { cmd.type = CMD_KI;    cmd.value = (float)atof((char *)&rx_buf[2]); }
        else if (rx_buf[0] == 'd' && rx_buf[1] == '=')                     { cmd.type = CMD_KD;    cmd.value = (float)atof((char *)&rx_buf[2]); }
        else if (rx_buf[0] == 'm' && rx_buf[1] == 'v' && rx_buf[2] == '=') { cmd.type = CMD_MV;    cmd.value = (float)atof((char *)&rx_buf[3]); }
        else if (rx_buf[0] == 'm' && rx_buf[1] == 'c' && rx_buf[2] == '=') { cmd.type = CMD_MC;    cmd.value = (float)atof((char *)&rx_buf[3]); }
        else                                                                { cmd.type = CMD_SPEED; cmd.value = (float)atof((char *)rx_buf); }

        xQueueSend(g_UserSpeedQueue, &cmd, 0);

        memset(rx_buf, 0, sizeof(rx_buf));   // clear before re-arming
        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rx_buf, sizeof(rx_buf));  // restart for next frame
    }
}

void flash_task(void *pvParameters)
{
    W25Q64_Init();   // configure CS pin

    // debug: read manufacturer ID to verify the chip responds (expect 0xEF4017)
    uint32_t id = W25Q64_ReadID();
    UART_Printf("W25Q64 ID=0x%08lX\r\n", id);

    // on boot: load saved params, validate before sending
    FlashData_t params = {0};
    W25Q64_ReadData(ADDR_PARAMS, (uint8_t *)&params, sizeof(params));
    if (params.speed >= 0.0f && params.speed < 10000.0f)   // sane range
    {
        xQueueSend(g_FlashQueue, &params, 0);
    }

    for (;;)
    {
        FaultState_t fault;
        if (xQueueReceive(g_FaultQueue, &fault, portMAX_DELAY) == pdTRUE)
        {
            W25Q64_WriteData(ADDR_FAULT, (uint8_t *)&fault, sizeof(fault));
        }
    }
}


void button_task(void *pvParameters)
{
    for (;;)
    {
        // button is pressed (active low)
        if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_RESET)
        {
            // pressed edge: debounce
            vTaskDelay(pdMS_TO_TICKS(20));
            if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_RESET)
            {
                xSemaphoreGive(BinarySemaphore);

                // wait for release, so holding does not retrigger
                while (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_RESET)
                {
                    vTaskDelay(pdMS_TO_TICKS(10));
                }
                // release edge: debounce
                vTaskDelay(pdMS_TO_TICKS(20));
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));   // poll every 10 ms
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
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	// create start task
	xTaskCreate(start_task,"start_task",START_STK_SIZE,NULL,START_TASK_PRIO,&StartTask_Handler);
	vTaskStartScheduler();

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
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

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
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
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 3599;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
