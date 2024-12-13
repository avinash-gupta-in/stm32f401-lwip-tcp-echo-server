/* USER CODE BEGIN Header */
/*
 * This is a part of code samples for course available on
 * www.stm32tutorials.com
 *
 * SIMPLE TCP ECHO SERVER USING LWIP
 * *********************************
 * This tcp server listens on TCP port 5000 you can then connect using
 * Hercules TCP client and send data to this server. The server will
 * echo back the same data to the client which will be visible on
 * Hercules.
 * The aim of this sample code is to teach the students how to write a
 * TCP Server in lwIP
 *
 * Hardware
 * ********
 * MCU: STM32F401CC (48 PIN)
 * Board: STM32F401CC Black-pill
 * Crystal: 25MHz
 * CPU Speed: 84MHz
 * RAM: 64KB
 * Core: ARM Cortx-M4
 *
 * Ethernet Controller
 * *******************
 * ENC28J60 (From Microchip)
 * Interface: SPI
 * SPI Instance: SPI1
 * PINS
 *   MCU                | ENC28J60
 *   --------------------------------
 * 	*PA5 (SPI1_SCK) ->  | SCK
 * 	*PA6 (SPI1_MISO)->  | MISO (SO)
 * 	*PA7 (SPI1_MOSI)->  | MOSI (SI)
 * 	*PB2 (EXTI)         | INT
 *  *PA4                | CS
 * 	*PA8                | RESET
 *
 * USART Debug
 * ***********
 * Debug messages are printed on USART2
 * Baud Rate: 115200 (8N1)
 * PA2 (TX)
 * PA3 (RX)
 *
 *
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include <stdio.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <lwipopts.h>
#include <lwip/netif.h>
#include <lwip/tcpip.h>
#include <ethernetif.h>
#include <lwip/api.h>

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
uint16_t listen_port=5000;

/* network interface structure */
struct netif gnetif;

/* Semaphore to signal Ethernet Link state update */
osSemaphoreId Netif_IRQSemaphore = NULL;

const osThreadAttr_t Netif_Thread_attr = {
        .name="NETIF",
        .stack_size = configMINIMAL_STACK_SIZE * 4,
        .priority=osPriorityRealtime
};

/* Ethernet link thread Argument */
struct enc_irq_str irq_arg;

static void Netif_Config(void);


UART_HandleTypeDef huart2;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
void StartDefaultTask(void *argument);
void Error_Handler(void);

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
  MX_USART2_UART_Init();

  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

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
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 168;
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  GPIO_InitTypeDef i;

  i.Mode=GPIO_MODE_OUTPUT_PP;
  i.Pin=GPIO_PIN_1;
  i.Speed=GPIO_SPEED_LOW;
  i.Pull=GPIO_NOPULL;

  HAL_GPIO_Init(GPIOA, &i);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
	printf("Jai Shree Ram !!!\r\n ");
	printf("My Very First lwIP based TCP Server Application !!!\r\n ");
	//Let ENC28J60 startup
	osDelay(500);

	/* Initilaize the LwIP stack */
	tcpip_init(NULL, NULL);//first is done func, 2nd is its args, both null

	Netif_Config();

	//This holds the return values from network functions
	//mainly used to check error or success
	err_t status;

	//two netconns (lwip uses netconn in place of sockets)
	struct netconn *conn;//listening connection
	struct netconn *conn2;//accepted connection
	struct netbuf *buf=NULL;//netbuf struct to receive data from netconn

	char *data_ptr=NULL;//this pointer will be used to extract data from neybuff
	uint16_t len=0;//this will hold the length of data received from client

	//Allocate a new netconn (socket)
	conn=netconn_new(NETCONN_TCP);

	//Bint it to TCP port 5000
	netconn_bind(conn, IP4_ADDR_ANY,listen_port);
	printf("\r\nBinding netconn to port %d",listen_port);

	//Make the netconn (socket) passive and start
	//listening for incoming connections.
	//in lwIP backlogs (queuing of connections) is disabled.
	//so we don't specify the queue length here.
	netconn_listen(conn);
	printf("\r\nListening for incoming connections ...");

	//Enter an infinite loop to continue processing requests
	//one after another
	for(;;)
	{
		//The function below will put the current task (thread)
		//in blocked stated until a client tries to connect.
		//it will return ERR_OK when a new client is connected.
		printf("\r\nWaiting for new client connection ...");
		status=netconn_accept(conn, &conn2);

		if(status!=ERR_OK)
		{
			printf("Some error during accept");
			while(1);//halt
		}

		printf("\r\nA new client just connected!");

		while(1)
		{
			//A new client connected.
			//Wait of input data from client
			//(this input data is called a 'request' from client)
			status=netconn_recv(conn2, &buf);

			if(status!=ERR_OK)
			{
				printf("\r\nSome error during data reception! possible cause connection closed by other side.");
				break;//now jump to start and wait for a new connection from another client.
			}

			printf("\r\nReceived %d bytes from client",netbuf_len(buf));

			netconn_write(conn2,"REPLY:",6,NETCONN_COPY);

			//successfully received data, process it.
			//since a netbuf can be fragmented and multi-part
			//we have to use a loop to process all fragments
			do
			{
				//Get pointer to the payload (data) and its length also.
				netbuf_data(buf, (void*)&data_ptr, &len);

				//Write it back to client
				netconn_write(conn2,(void*)data_ptr,len,NETCONN_COPY);

			}while(netbuf_next(buf)>=0);//try to goto next fragment in chain if available.

			//Free the buffer, since its no longer required,
			//otherwise memory leak will occur and eat up whole available free RAM
			netbuf_delete(buf);

		}//reception loop, continuously receive data as long as client is connected

		//close
		netconn_close(conn2);

		//De-allocate the netconn
		netconn_delete(conn2);
	}

}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM10 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM10) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

int putchar(int ch)
{
    HAL_UART_Transmit(&huart2,(uint8_t *)&ch,1,1);
    return ch;
}

static void Netif_Config(void)
{
  ip4_addr_t ipaddr;//IP address of this device
  ip4_addr_t netmask;
  ip4_addr_t gw;//gateway IP address

  /* IP address setting */
  IP4_ADDR(&ipaddr, IP_ADDR_4, IP_ADDR_3, IP_ADDR_2, IP_ADDR_1);
  IP4_ADDR(&netmask, 255, 255 , 255, 0);
  IP4_ADDR(&gw, IP_ADDR_4, IP_ADDR_3, IP_ADDR_2, 1);

  /* create a binary semaphore used for informing ethernetif of frame reception */
  //osSemaphoreDef(Netif_SEM);
  //Netif_IRQSemaphore = osSemaphoreCreate(osSemaphore(Netif_SEM) , 1);//CMSIS V1 API
  Netif_IRQSemaphore = osSemaphoreNew(1,0,NULL);

  /* - netif_add(struct netif *netif, struct ip_addr *ipaddr,
  struct ip_addr *netmask, struct ip_addr *gw,
  void *state, err_t (* init)(struct netif *netif),
  err_t (* input)(struct pbuf *p, struct netif *netif))

  Adds your network interface to the netif_list. Allocate a struct
  netif and pass a pointer to this structure as the first argument.
  Give pointers to cleared ip_addr structures when using DHCP,
  or fill them with sane numbers otherwise. The state pointer may be NULL.

  The init function pointer must point to a initialization function for
  your ethernet netif interface. The following code illustrates it's use.*/

  printf("Adding Network IF ...\r\n ");

  if(netif_add(&gnetif, &ipaddr, &netmask, &gw, NULL, &ethernetif_init, &tcpip_input)==NULL)
  {
	  //could not initialize the ENC28J60 due to problem between connection of ENC28J60 & MCU
	  printf("Error!\r\n ");

	  while(1);
  }
  else
  {
	  //show network card init success
	  printf("Success!\r\n ");
  }

  /*  Registers the default network interface. */
  netif_set_default(&gnetif);

  /* Set the link callback function, this function is called on change of link status*/
  netif_set_link_callback(&gnetif, ethernetif_update_config);

  irq_arg.netif = &gnetif;
  irq_arg.semaphore = Netif_IRQSemaphore;
  /* Create the Ethernet IRQ handler thread */
  osThreadNew(ethernetif_process_irq, &irq_arg, &Netif_Thread_attr);//changed it to suit CMSIS OS V2

  printf("Waiting for cable ...");

  while(!netif_is_link_up(&gnetif))
  {

  }

  printf("Cable plugged in!\r\n ");

  netif_set_up(&gnetif);
}

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
