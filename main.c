/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ringBuffer.h"
#include <string.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
//*********    typedef for send at command       ********/
#define GSM_ATC_BUFFER_SIZE             (256)   // AT command buffer size
typedef struct
{
    uint8_t buffer[GSM_ATC_BUFFER_SIZE];
    uint16_t index;
} gsm_atc_buffer_t;

typedef enum
{
    AT_CMD_EVENT_OK = 0,
    AT_CMD_EVENT_TIMEOUT,
    AT_CMD_EVENT_ERROR,
    AT_CMD_EVENT_NOTHING        // dont care to this event
} at_cmd_response_evt_t;
typedef void (*gsm_send_at_cb_t)(at_cmd_response_evt_t evt);
typedef struct
{
    char *cmd;
    char *expected_response;
    char *expected_response_at_the_end_of_response;
    char *expected_error;
    char *expected_error_at_the_end_of_response;
    uint16_t timeout_ms;
    uint16_t current_timeout;
    uint8_t retry_cnt;
    gsm_atc_buffer_t recv_buff;
    gsm_send_at_cb_t send_at_callback;
} atc_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define    GPRS_SET  0
#define    GPRS_SETTING 1
#define    GPRS_DONE 2
#define    PUTTING 3
/************* define for gsm state***********/
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
ringBuffer_t ringBuff;
uint8_t temp[sizeofBuff];
/*************  variable for ringBuffer   *************/
uint8_t GPRS_step = 0;
uint8_t SETTING_step = 0;
uint8_t PUTTING_step = 0;
uint8_t GETTING_step = 0;
/********   variable for handling task *************/

atc_t atc;
/********  AT CMD VAR  ********/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

void gsm_handler (void);


void putFile (at_cmd_response_evt_t evt);    // data putfile handle

void getFile (at_cmd_response_evt_t evt);    // data get file handle

void GPRS_setup (at_cmd_response_evt_t evt); // data setup handle 

void getDone (at_cmd_response_evt_t evt);   // du lieu nhan ve
/**************   main handle program  ******************/
void gsm_hw_polling_task(void);
 /**************   hardware handle  ******************/   

void gsm_hw_send_at_cmd(char *cmd,
                        char *expect_response,
                        char *expected_response_at_the_end_of_response,
                        char *expected_error,
                        char *expected_error_at_the_end_of_response,
                        uint16_t timeout_ms,
                        uint8_t retry_cnt,
                        gsm_send_at_cb_t callback);
/**************   send at command parameter  ******************/




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
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
    ringBufferInit (&ringBuff, temp);        // seting for ring buffer
    USART2 ->CR1 |=   1 << 5;               /// RXEN ENABLE
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      gsm_handler();
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
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
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */
/***********       INTERRUPT HANDLER       **********/
void RX2Callback (void)
{
    WriteToRingBuffer (&ringBuff, USART2);
}
/***********       INTERRUPT HANDLER end       **********/

/*******         HARWARE HANDLER       *************/
void gsm_hw_polling_task(void)
{
    static uint32_t m_last_poll = 0;
    uint32_t now = HAL_GetTick();
    if ((now - m_last_poll) < 5)
    {
        return;
    }
    atc.recv_buff.index = 0;
    while (getByteFromRingBufferAvailableToRead(&ringBuff))
    {
        atc.recv_buff.buffer [atc.recv_buff.index ++] = readFromRingBuffer (&ringBuff);
    }
    
    if (atc.timeout_ms)
    {
        atc.current_timeout ++;
        if (atc.current_timeout > atc.timeout_ms)
        {
            atc.current_timeout = 0;
                            
            if (atc.retry_cnt)
            {
                atc.retry_cnt --;
            }
            if (atc.retry_cnt == 0)
            { 
                atc. timeout_ms = 0;
                if (atc.send_at_callback != NULL)
                {
                    atc.send_at_callback( AT_CMD_EVENT_TIMEOUT); /// viet ham reset he thong
                }
            }
            else
            {
                uart_tx (USART2, (uint8_t *)atc.cmd, strlen(atc.cmd)); 
            }
        }
    }
    if (strstr ((char *) atc.recv_buff.buffer, atc.expected_response))
    {
        bool do_cb = true;
        if( atc.expected_response_at_the_end_of_response)
        {
            uint32_t current_response_length = atc.recv_buff.index;
            uint32_t expect_compare_length = strlen(atc.expected_response_at_the_end_of_response);
            uint8_t *p_compare_end_str = &atc.recv_buff.buffer[current_response_length - expect_compare_length];
            if (expect_compare_length <= current_response_length 
                && (memcmp(p_compare_end_str, atc.expected_response_at_the_end_of_response, expect_compare_length) == 0))
                {
                    do_cb = true;
                }
                else
                {
                    do_cb = false;
                }
        }
        if (do_cb)
        {
            atc.current_timeout = 0;
            if (atc.send_at_callback != NULL)
            {
                atc.send_at_callback (AT_CMD_EVENT_OK);
                
            }
        }
    }
    else if (atc.expected_error 
            && strstr((char *)(atc.recv_buff.buffer), atc.expected_error))
    {
        bool do_cb = true;
        if( atc.expected_error_at_the_end_of_response)
        {
            uint32_t current_response_length = atc.recv_buff.index;
            uint32_t expect_compare_length = strlen(atc.expected_error_at_the_end_of_response);
            uint8_t *p_compare_end_str = &atc.recv_buff.buffer[current_response_length - expect_compare_length];
            if (expect_compare_length <= current_response_length 
                    && (memcmp(p_compare_end_str, atc.expected_error_at_the_end_of_response, expect_compare_length) == 0))
                {
                    do_cb = true;
                }
                else
                {
                    do_cb = false;
                }
        }
        if (do_cb)
        {
            atc. current_timeout = 0;

            if (atc.send_at_callback != NULL)
            {
                atc.send_at_callback(AT_CMD_EVENT_ERROR);
            }
        }
    }
//    memset (atc.recv_buff.buffer, '\0', GSM_ATC_BUFFER_SIZE);
    m_last_poll = HAL_GetTick();
} 
/*******         HARWARE HANDLER end      *************/

/**********    ASSIGN  AT CMD             ***********/
void gsm_hw_send_at_cmd(char *cmd,
                        char *expect_response,
                        char *expected_response_at_the_end_of_response,
                        char *expected_error,
                        char *expected_error_at_the_end_of_response,
                        uint16_t timeout_ms,
                        uint8_t retry_cnt,
                        gsm_send_at_cb_t callback)
{
    if (timeout_ms == 0 || callback == NULL)
    {
        uart_tx(USART2, (uint8_t *)cmd, strlen(cmd));
        return;
    }
#if 0
    DEBUG_PRINTF("%s", cmd);
    static uint32_t max_at_cmd_size = 0;
    if (strlen(cmd) > max_at_cmd_size)
    {
        max_at_cmd_size = strlen(cmd);
        DEBUG_PRINTF("AT cmd max size %d\r\n", max_at_cmd_size);
    }
#endif
  
    atc.cmd = cmd;
    atc.expected_response = expect_response;
    atc.expected_response_at_the_end_of_response = expected_response_at_the_end_of_response;
    atc.expected_error = expected_error;
    atc.expected_error_at_the_end_of_response = expected_error_at_the_end_of_response;
    atc.retry_cnt = retry_cnt;
    atc.send_at_callback = callback;
    atc.timeout_ms = timeout_ms / 5;
    atc.current_timeout = 0;
    memset (atc.recv_buff.buffer, '\0', GSM_ATC_BUFFER_SIZE);
    uart_tx(USART2, (uint8_t *)cmd, strlen(cmd));
}
/**********    ASSIGN  AT CMD end           ***********/

/*****************         SETUP GPRS    ************************/
void GPRS_setup ( at_cmd_response_evt_t evt)
{
    if (evt == AT_CMD_EVENT_TIMEOUT)
    {
        uart_tx (USART2, (uint8_t *)"time out, reset pls\r\n", strlen ("time out, reset pls\r\n"));
        
        return;
    }
    switch (SETTING_step)
        {
            case 0:
                if (evt == AT_CMD_EVENT_OK)
                {
                    SETTING_step++;
                }
                else
                {
                    return;
                }
                gsm_hw_send_at_cmd ("ATE1\r\n", "OK","", "ERROR", "", 1000, 5, GPRS_setup); ///ECHO OFF
            break;
            case 1:
                if (evt == AT_CMD_EVENT_OK)
                {
                    SETTING_step++;
                }
                else
                {
                    return;
                }
                gsm_hw_send_at_cmd ("AT+IPR=115200\r\n", "OK","", "ERROR", "", 1000, 5, GPRS_setup); //BAUDRATE
            break;
            case 2:
                if (evt == AT_CMD_EVENT_OK)
                {
                    SETTING_step++;
                }
                else
                {
                    return;
                }
                gsm_hw_send_at_cmd ("AT+CMEE=2\r\n", "OK","", "ERROR", "", 1000, 5, GPRS_setup); // ENABLE CME ERROR
            break;
            case 3:
                if (evt == AT_CMD_EVENT_OK)
                {
                    SETTING_step++;
                }
                else
                {
                    return;
                }
                gsm_hw_send_at_cmd ("AT+CPIN?\r\n", "+CPIN: READY\r\n","", "ERROR", "", 1000, 5, GPRS_setup); // SIM STATE
            break;
            case 4:
                if (evt == AT_CMD_EVENT_OK)
                {
                    SETTING_step++;
                }
                else
                {
                    return;
                }
                gsm_hw_send_at_cmd ("AT+CIMI\r\n", "OK","", "ERROR", "", 1000, 5, GPRS_setup); // REQUEST SIM IMEI
            break;

            case 5:
                if (evt == AT_CMD_EVENT_OK)
                {
                    SETTING_step++;
                }
                else
                {
                    return;
                }
                gsm_hw_send_at_cmd ("AT+GSN\r\n", "OK","", "ERROR", "", 1000, 5, GPRS_setup); // GET GSN IMEI
            break;
            case 6:
                if (evt == AT_CMD_EVENT_OK)
                {
                    SETTING_step++;
                }
                else
                {
                    return;
                }
                gsm_hw_send_at_cmd ("AT+CFUN=1\r\n", "OK","", "ERROR", "", 1000, 5, GPRS_setup); // Set ME Functionality
            break;
            case 7:
                if (evt == AT_CMD_EVENT_OK)
                {
                    SETTING_step++;
                }
                else
                {
                    return;
                }
                gsm_hw_send_at_cmd ("AT+CGDCONT=1,\"IP\",\"v-internet\"\r\n", "OK","", "+CME ERROR", "", 1000, 5, GPRS_setup); // SET UP APN
            break;
            case 8:
                if (evt == AT_CMD_EVENT_OK)
                {
                    SETTING_step++;
                }
                else
                {
                    return;
                }
                gsm_hw_send_at_cmd ("AT+CSQ\r\n", "OK","", "+CME ERROR", "", 1000, 5, GPRS_setup); // GET SINAL QUALITY
            break;
            case 9:
                //IF NHAN VE 99 GUI LAI
                if (evt == AT_CMD_EVENT_OK)
                {
                    SETTING_step++;
                }
                else
                {
                    return;
                }
                gsm_hw_send_at_cmd ("AT+CGREG=1\r\n", "OK","", "ERROR", "", 1000, 5, GPRS_setup); // ENABLE NETWORK REGISTRATION
            break;
            case 10:
                
                if (evt == AT_CMD_EVENT_OK)
                {
                    SETTING_step++;
                }
                else
                {
                    return;
                }
                gsm_hw_send_at_cmd ("AT+CSTT=\"v-internet\"\r\n", "OK","", "ERROR", "", 1000, 5, GPRS_setup);// START SET APN TASK
            break;
            case 11:
            
                if (evt == AT_CMD_EVENT_OK)
                {
                    SETTING_step++;
                }
                else
                {
                    return;
                }
                gsm_hw_send_at_cmd ("AT+CIICR\r\n", "OK\r\n","", "+CME ERROR", "", 1000, 5, GPRS_setup); // BRINGUP WIRELESS CONNECTION
            break;
            case 12:
                if (evt == AT_CMD_EVENT_OK)
                {
                    SETTING_step++;
                }
                else
                {
                    return;
                }
                gsm_hw_send_at_cmd ("AT+SAPBR=3,1,\"Contype\",\"GPRS\"\r\n", "OK","", "+CME ERROR", "", 1000, 5, GPRS_setup);
            break;
            case 13:
                if (evt == AT_CMD_EVENT_OK)
                {
                    SETTING_step++;
                }
                else
                {
                    return;
                }
                gsm_hw_send_at_cmd ("AT+SAPBR=3,1,\"APN\",\"v-internet\"\r\n", "OK", "", "ERROR", "", 1000, 5, GPRS_setup);
            break;
            case 14:
                if (evt == AT_CMD_EVENT_OK)
                {
                    SETTING_step++;
                }
                else
                {
                    return;
                }
                gsm_hw_send_at_cmd ("AT+SAPBR=1,1\r\n", "OK", "", "+CME ERROR", "", 1000, 5, GPRS_setup);
            break;
            
            default:
                if (evt == AT_CMD_EVENT_ERROR)
                {
                    return;
                }
                GPRS_step = GPRS_DONE;
                return;
        }
      //  SETTING_step++;

}
/*****************         SETUP GPRS end   ************************/
/**********    PUTTING FILE             ***********/
void putFile (at_cmd_response_evt_t evt)
{
    if (evt == AT_CMD_EVENT_TIMEOUT)
    {
        uart_tx (USART2, (uint8_t *)"time out, reset pls\r\n", strlen ("time out, reset pls\r\n"));
        
        return;
    }
    {
        switch (PUTTING_step)
        {
            case 0:
                if (evt == AT_CMD_EVENT_OK)
                {
                    PUTTING_step ++;
                }
                else
                {
                    return;
                }
                gsm_hw_send_at_cmd ("AT+FTPCID=1\r\n", "OK","", "ERROR", "\r\n", 1000, 10, putFile);
            break;
            case 1:
                if (evt == AT_CMD_EVENT_OK)
                {
                    PUTTING_step ++;
                }
                else
                {
                    return;
                }
                gsm_hw_send_at_cmd ("AT+FTPSERV=\"radiotech.vn\"\r\n", "OK", "", "ERROR", "\r\n", 1000, 10,putFile);
            break;
            case 2:
                if (evt == AT_CMD_EVENT_OK)
                {
                    PUTTING_step ++;
                }
                else
                {
                    return;
                }
                gsm_hw_send_at_cmd ("AT+FTPPORT=2603\r\n", "OK", "", "ERROR", "\r\n", 1000, 10,putFile);
            break;
            case 3:
                if (evt == AT_CMD_EVENT_OK)
                {
                    PUTTING_step ++;
                }
                else
                {
                    return;
                }
                gsm_hw_send_at_cmd ("AT+FTPUN=\"bytech\"\r\n", "OK", "", "ERROR", "\r\n", 1000, 10,putFile);
            break;
            case 4:
                if (evt == AT_CMD_EVENT_OK)
                {
                    PUTTING_step ++;
                }
                else
                {
                    return;
                }
                gsm_hw_send_at_cmd ("AT+FTPPW=\"abcd@1234\"\r\n", "OK", "", "ERROR", "\r\n", 1000, 10,putFile);
            break;
            case 5:
                if (evt == AT_CMD_EVENT_OK)
                {
                    PUTTING_step ++;
                }
                else
                {
                    return;
                }
                gsm_hw_send_at_cmd ("AT+FTPPUTNAME=\"TEST.txt\"\r\n", "OK", "", "ERROR", "\r\n", 1000, 10,putFile);
            break;
            case 6:
                if (evt == AT_CMD_EVENT_OK)
                {
                    PUTTING_step ++;
                }
                else
                {
                    return;
                }
                gsm_hw_send_at_cmd ("AT+FTPPUTPATH=\"/\"\r\n", "OK", "", "ERROR", "\r\n", 1000, 10,putFile);
            break;
            case 7:
                if (evt == AT_CMD_EVENT_OK)
                {
                    PUTTING_step ++;
                }
                else
                {
                    return;
                }
                gsm_hw_send_at_cmd ("AT+FTPPUT=1\r\n", "+FTPPUT: 1,1", "", "ERROR", "\r\n", 15000, 1, putFile);
            break;
            case 8:
                if (evt == AT_CMD_EVENT_OK)
                {
                    PUTTING_step ++;
                }
                else
                {
                    return;
                }
                gsm_hw_send_at_cmd ("AT+FTPPUT=2,10\r\n", "+FTPPUT: 2,10", "", "ERROR", "", 6000, 10, putFile);
//                uart_tx (USART2, (uint8_t *)"1234567890\r\n", strlen ("1234567890\r\n"));
            break;
            case 9:
                if (evt == AT_CMD_EVENT_OK)
                {
                    PUTTING_step ++;
//                    uart_tx (USART2, (uint8_t *)"1234567890\r\n", strlen ("1234567890\r\n"));
                }
                else
                {
                    return;
                }
                gsm_hw_send_at_cmd ("12345678900\r\n", "", "", "", "", 5000, 1, putFile);
            break;
            case 10:
                if (evt == AT_CMD_EVENT_OK)
                {
                    PUTTING_step ++;
                }
                else
                {
                    return;
                }
                gsm_hw_send_at_cmd ("AT+FTPPUT=2,0\r\n", "OK", "\r\n", "ERROR", "\r\n", 5000, 1,putFile);
            break;
            default:
                        uart_tx (USART2, (uint8_t *)"putdone", strlen("putdone"));
            break;
        }
    }
}
/**********    PUTTING FILE end            ***********/
/***************       GETTING FILE         ******************/
void getFile (at_cmd_response_evt_t evt)
{
    if (evt == AT_CMD_EVENT_TIMEOUT)
    {
        uart_tx (USART2, (uint8_t *)"time out, reset pls\r\n", strlen ("time out, reset pls\r\n"));
        
        return;
    }
    switch (GETTING_step)
    {
        case 0:
            if (evt == AT_CMD_EVENT_OK)
            {
                GETTING_step ++;
            }
            else
            {
                return;
            }
            gsm_hw_send_at_cmd ("AT+FTPCID=1\r\n", "OK","", "ERROR", "\r\n", 1000, 10, getFile);
        break;
        case 1:
            if (evt == AT_CMD_EVENT_OK)
            {
                GETTING_step ++;
            }
            else
            {
                return;
            }
            gsm_hw_send_at_cmd ("AT+FTPSERV=\"radiotech.vn\"\r\n", "OK","", "ERROR", "\r\n", 1000, 10, getFile);
        break;
        case 2:
            if (evt == AT_CMD_EVENT_OK)
            {
                GETTING_step ++;
            }
            else
            {
                return;
            }
            gsm_hw_send_at_cmd ("AT+FTPPORT=2603\r\n", "OK","", "ERROR", "\r\n", 1000, 10, getFile);
        break;
        case 3:
            if (evt == AT_CMD_EVENT_OK)
            {
                GETTING_step ++;
            }
            else
            {
                return;
            }
            gsm_hw_send_at_cmd ("AT+FTPUN=\"bytech\"\r\n", "OK","", "ERROR", "\r\n", 1000, 10, getFile);
        break;
        case 4:
            if (evt == AT_CMD_EVENT_OK)
            {
                GETTING_step ++;
            }
            else
            {
                return;
            }
            gsm_hw_send_at_cmd ("AT+FTPPW=\"abcd@1234\"\r\n", "OK","", "ERROR", "\r\n", 1000, 10, getFile);
        break;
        case 5:
            if (evt == AT_CMD_EVENT_OK)
            {
                GETTING_step ++;
            }
            else
            {
                return;
            }
            gsm_hw_send_at_cmd ("AT+FTPGETNAME=\"TEST.txt\"\r\n", "OK","", "ERROR", "\r\n", 1000, 10, getFile);
        break;
        case 6:
            if (evt == AT_CMD_EVENT_OK)
            {
                GETTING_step ++;
            }
            else
            {
                return;
            }
            gsm_hw_send_at_cmd ("AT+FTPGETPATH=\"/\"\r\n", "OK","", "ERROR", "\r\n", 1000, 10, getFile);
        break;
        case 7:
            if (evt == AT_CMD_EVENT_OK)
            {
                GETTING_step ++;
            }
            else
            {
                return;
            }
            gsm_hw_send_at_cmd ("AT+FTPGET=1\r\n", "+FTPGET: 1,1","", "ERROR", "\r\n", 20000, 1, getFile);
        break;
        case 8:
            if (evt == AT_CMD_EVENT_OK)
            {
                GETTING_step ++;
            }
            else
            {
                return;
            }
            gsm_hw_send_at_cmd ("AT+FTPGET=2,6\r\n", "+FTPGET: 2,6","", "ERROR", "\r\n", 10000, 2, getFile);
        break;
        default:
            if (evt == AT_CMD_EVENT_OK)
                {
                    GETTING_step ++;
                }
                else
                {
                    return;
                }
            char buff[20];    
            strcpy (buff, (char*)atc.recv_buff.buffer);
            char * dataGet;
            dataGet = strtok (buff, "\r\n");
            dataGet = strtok (NULL, "\r\n");            
            uart_tx (USART3, (uint8_t *)dataGet, strlen (dataGet));
        break;        
    }    
}

/***************       GETTING FILE END            ************/

/*******************      TACH STRING      ********************/            

/*******************      TACH STRING          ***************/

/********************     PROGRAM HANDLER        *****************/
void gsm_handler (void)   /// define handler
{
    gsm_hw_polling_task ();    
    switch (GPRS_step)
    {
        case GPRS_SET:
            GPRS_step = GPRS_SETTING;
            gsm_hw_send_at_cmd ("AT\r\n", "OK","", "", "", 100, 20, GPRS_setup);
            break;
        case GPRS_SETTING:
            break;
        case GPRS_DONE:
            gsm_hw_send_at_cmd ("AT\r\n", "OK","", "", "", 1000, 5, getFile);
            GPRS_step = PUTTING;
            break;
        default:
            break;
    }
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

