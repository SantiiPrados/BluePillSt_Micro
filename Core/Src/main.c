/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum{
    START,
    HEADER_1,
    HEADER_2,
    HEADER_3,
    NBYTES,
    TOKEN,
    PAYLOAD
}_eProtocolo;

_eProtocolo estadoProtocolo;

typedef enum{
       ACK=0x0D,
       ALIVE=0xF0,
//       FIRMWARE=0xF1,
//       SET_LEDS=0xF2,
       IR_SENSOR=0xA0,
//       MOTOR_ACTION=0xA1,
//       SERVO_ACTION=0xA2,
//       ULTRA_SONIC=0xA3,
//       HORQUILLA=0xA4,
//       STARTCONFIG=0xEE,
       OTHERS
   }_eID;

typedef struct{
    uint8_t timeOut;         //!< TiemOut para reiniciar la máquina si se interrumpe la comunicación
    uint8_t indexStart; 	 ///////////////////// AGREGAR ///////////////////////////////////
    uint8_t cheksumRx;       //!< Cheksumm RX
    uint8_t cheksumtx;       //!< Cheksumm Tx
    uint8_t indexWriteRx;    //!< Indice de escritura del buffer circular de recepción
    uint8_t indexReadRx;     //!< Indice de lectura del buffer circular de recepción
    uint8_t indexWriteTx;    //!< Indice de escritura del buffer circular de transmisión
    uint8_t indexReadTx;     //!< Indice de lectura del buffer circular de transmisión
    uint8_t bufferRx[256];   //!< Buffer circular de recepción
    uint8_t bufferTx[256];   //!< Buffer circular de transmisión
    // uint8_t payload[32];     //!< Buffer para el Payload de datos recibidos
}_sDato ;

_sDato datosComProtocol;


typedef union {
    int32_t i32;
    uint32_t ui32;
    uint16_t ui16[2];
    uint8_t ui8[4];
}_udat;

_udat myWord;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
	uint8_t BTNstatus	= 0;
	uint8_t BTNcount	= 0;
	uint8_t T100ms;
	uint8_t rx[256],ir,iw;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);

/**
 * @brief Decodifica las tramas que se reciben
 * La función decodifica el protocolo para saber si lo que llegó es válido.
 * Utiliza una máquina de estado para decodificar el paquete
 */
void decodeProtocol(_sDato *);


/**
 * @brief Procesa el comando (ID) que se recibió
 * Si el protocolo es correcto, se llama a esta función para procesar el comando
 */
void decodeData(_sDato *);


/**
 * @brief Procesa el comando (ID) que se ingresa cómo parámetro
 * En función del ID se arma el payload para enviar
 */
void encodeData(uint8_t id);


/**
 * @brief Envía los datos a la PC
 * La función consulta si el puerto serie está libra para escribir, si es así envía 1 byte y retorna
 */
void sendData(void);


/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if (htim ->Instance == TIM1) {
		T100ms--;
	}if(T100ms==0){
		T100ms=10;
	}
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if(huart ->Instance ==USART1){
		iw++;
		HAL_UART_Receive_IT(&huart1, &rx[iw], 1);
		//HAL_UART_Receive_IT(&huart1, &rx[iw], 1);
//		while (pcCom.readable())
//		{
//			datosComProtocol.bufferRx[datosComProtocol.indexWriteRx++]=pcCom.getc();
//		}
	}
}


//'<' header
//byte1
//byte2
//byte3
//byte4
//checksum = suma de todos los bytes transmitidos
//'>' tail

//  4      1      1    1    N     1
//HEADER NBYTES TOKEN ID PAYLOAD CKS

//HEADER 4 bytes
//'U' 'N' 'E' 'R'

//NBYTES = ID+PAYLOAD+CKS = 2 + nbytes de payload

//TOKEN: ':'

//CKS: xor de todos los bytes enviados menos el CKS

void decodeProtocol(_sDato *datosCom){
	static uint8_t nBytes = 0;
	while (datosCom->indexReadRx != datosCom->indexWriteRx){
		switch (estadoProtocolo){
		case START:
			if (datosCom->bufferRx[datosCom->indexReadRx++]=='U') {
				estadoProtocolo = HEADER_1;
				datosCom->cheksumRx = 0;
			}
			break;
		case HEADER_1:
			if (datosCom->bufferRx[datosCom->indexReadRx++]=='N')
			   {
				   estadoProtocolo=HEADER_2;
			   }
			else{
				datosCom->indexReadRx--;
				estadoProtocolo=START;
			}
			break;
		case HEADER_2:
			if (datosCom->bufferRx[datosCom->indexReadRx++]=='E')
			{
				estadoProtocolo=HEADER_3;
			}
			else{
				datosCom->indexReadRx--;
			   estadoProtocolo=START;
			}
			break;
		case HEADER_3:
			if (datosCom->bufferRx[datosCom->indexReadRx++]=='R')
				{
					estadoProtocolo=NBYTES;
				}
			else{
				datosCom->indexReadRx--;
			   estadoProtocolo=START;
			}
			break;
		case NBYTES:
			datosCom->indexStart=datosCom->indexReadRx;
			nBytes=datosCom->bufferRx[datosCom->indexReadRx++];
			estadoProtocolo=TOKEN;
			break;
		case TOKEN:
			if (datosCom->bufferRx[datosCom->indexReadRx++]==':'){
			   estadoProtocolo=PAYLOAD;
				datosCom->cheksumRx ='U'^'N'^'E'^'R'^ nBytes^':';
				// datosCom->payload[0]=nBytes;
				// indice=1;
			}
			else{
				datosCom->indexReadRx--;
				estadoProtocolo=START;
			}
			break;
		case PAYLOAD:
			if (nBytes>1){
				// datosCom->payload[indice++]=datosCom->bufferRx[datosCom->indexReadRx];
				datosCom->cheksumRx ^= datosCom->bufferRx[datosCom->indexReadRx++];
			}
			nBytes--;
			if(nBytes<=0){
				estadoProtocolo=START;
				if(datosCom->cheksumRx == datosCom->bufferRx[datosCom->indexReadRx]){
					decodeData(datosCom);
				}
			}
			break;
		default:
			estadoProtocolo=START;
			break;
		}
	}

}


void decodeData(_sDato *datosCom){
	#define POSID   2
    #define POSDATA 3

	uint8_t auxBuffTx[50], indiceAux=0, cheksum;
	auxBuffTx[indiceAux++]='U';
	auxBuffTx[indiceAux++]='N';
	auxBuffTx[indiceAux++]='E';
	auxBuffTx[indiceAux++]='R';
	auxBuffTx[indiceAux++]=0;
	auxBuffTx[indiceAux++]=':';

	switch (datosCom->bufferRx[datosCom->indexStart+POSID]) {
		case ALIVE:
			auxBuffTx[indiceAux++]=ALIVE;
			auxBuffTx[indiceAux++]=ACK;
			auxBuffTx[NBYTES]=0x03;
			break;
		default:
			auxBuffTx[indiceAux++]=0xDD;
			auxBuffTx[NBYTES]=0x02;
			break;
	}

	cheksum=0;
	for(uint8_t a=0 ;a < indiceAux ;a++)
	{
		cheksum ^= auxBuffTx[a];
		datosCom->bufferTx[datosComProtocol.indexWriteTx++]=auxBuffTx[a];
	}
		datosCom->bufferTx[datosComProtocol.indexWriteTx++]=cheksum;

}

void encodeData(uint8_t id){
	uint8_t auxBuffTx[50], indiceAux=0, cheksum;
	auxBuffTx[indiceAux++]='U';
	auxBuffTx[indiceAux++]='N';
	auxBuffTx[indiceAux++]='E';
	auxBuffTx[indiceAux++]='R';
	auxBuffTx[indiceAux++]=0;
	auxBuffTx[indiceAux++]=':';

	switch (id) {
	case IR_SENSOR:
//		auxBuffTx[indiceAux++]=IR_SENSOR;
//		auxBuffTx[NBYTES]=0x06;
//
//		myWord.ui32 = sensorIR.valueIRIzq;
//		auxBuffTx[indiceAux++] = myWord.ui8[0];
//		auxBuffTx[indiceAux++] = myWord.ui8[1];
//
//		myWord.ui32 = sensorIR.valueIRDer;
//		auxBuffTx[indiceAux++] = myWord.ui8[0];
//		auxBuffTx[indiceAux++] = myWord.ui8[1];
		break;
		default:
			auxBuffTx[indiceAux++]=0xDD;
			auxBuffTx[NBYTES]=0x02;
			break;
	}
	cheksum=0;
	for(uint8_t a=0 ;a < indiceAux ;a++)
	{
		cheksum ^= auxBuffTx[a];
		datosComProtocol.bufferTx[datosComProtocol.indexWriteTx++]=auxBuffTx[a];
	}
		datosComProtocol.bufferTx[datosComProtocol.indexWriteTx++]=cheksum;
}

void sendData(void){

//	if(pcCom.writable())
//	{
//		pcCom.putc(datosComProtocol.bufferTx[datosComProtocol.indexReadTx++]);
//	}
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
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim1);

  ir=0,iw=0;

  HAL_UART_Receive_IT(&huart1, &rx[iw], 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  HAL_GPIO_TogglePin(LedBuidIn_GPIO_Port, LedBuidIn_Pin);
  HAL_Delay(100);
  HAL_GPIO_TogglePin(LedBuidIn_GPIO_Port, LedBuidIn_Pin);
  HAL_Delay(100);
  HAL_GPIO_TogglePin(LedBuidIn_GPIO_Port, LedBuidIn_Pin);
  HAL_Delay(100);
  HAL_GPIO_TogglePin(LedBuidIn_GPIO_Port, LedBuidIn_Pin);
  HAL_GPIO_TogglePin(LedBuidIn_GPIO_Port, LedBuidIn_Pin);
  HAL_Delay(100);
  HAL_GPIO_TogglePin(LedBuidIn_GPIO_Port, LedBuidIn_Pin);

  while (1)
  {
    /* USER CODE END WHILE */
	  BTNstatus = HAL_GPIO_ReadPin(BTN_GPIO_Port, BTN_Pin);

	  	  if(ir!=iw){
	  		if (__HAL_UART_GET_FLAG(&huart1,UART_FLAG_TXE)){
	  			ir = iw-1;
	  			USART1->DR = rx[ir];
	  			//HAL_UART_Transmit(&huart1, &rx[ir],1,0);
	  			ir=iw;
	  		}
	  	}

	  	  	  	if(!BTNstatus){
	  	  	  		BTNcount++;
	  	  	  		if(BTNcount > 5){
	  	  	  			BTNcount = 0;
	  	  	  		}
	  	  		}

	  	  	  	switch (BTNcount) {
	  	  			case 1:
	  	  					HAL_GPIO_TogglePin(LedBuidIn_GPIO_Port, LedBuidIn_Pin);
	  	  					HAL_Delay(100);
	  	  					HAL_GPIO_TogglePin(LedBuidIn_GPIO_Port, LedBuidIn_Pin);
	  	  					HAL_Delay(900);
	  	  				break;
	  	  			case 2:
	  	  					HAL_GPIO_TogglePin(LedBuidIn_GPIO_Port, LedBuidIn_Pin);
	  	  					HAL_Delay(100);
	  	  					HAL_GPIO_TogglePin(LedBuidIn_GPIO_Port, LedBuidIn_Pin);
	  	  					HAL_Delay(100);
	  	  					HAL_GPIO_TogglePin(LedBuidIn_GPIO_Port, LedBuidIn_Pin);
	  	  					HAL_Delay(100);
	  	  					HAL_GPIO_TogglePin(LedBuidIn_GPIO_Port, LedBuidIn_Pin);
	  	  					HAL_Delay(900);
	  	  				break;
	  	  			case 3:
	  	  					HAL_GPIO_TogglePin(LedBuidIn_GPIO_Port, LedBuidIn_Pin);
	  	  					HAL_Delay(100);
	  	  					HAL_GPIO_TogglePin(LedBuidIn_GPIO_Port, LedBuidIn_Pin);
	  	  					HAL_Delay(100);
	  	  					HAL_GPIO_TogglePin(LedBuidIn_GPIO_Port, LedBuidIn_Pin);
	  	  					HAL_Delay(100);
	  	  					HAL_GPIO_TogglePin(LedBuidIn_GPIO_Port, LedBuidIn_Pin);
	  	  					HAL_Delay(100);
	  	  					HAL_GPIO_TogglePin(LedBuidIn_GPIO_Port, LedBuidIn_Pin);
	  	  					HAL_Delay(100);
	  	  					HAL_GPIO_TogglePin(LedBuidIn_GPIO_Port, LedBuidIn_Pin);
	  	  					HAL_Delay(900);
	  	  				break;
	  	  			case 4:
	  	  				//if(condition){

	  	  				//}
	  	  				break;
	  	  			case 5:
	  	  				break;
	  	  			default:
	  	  				if(BTNcount==5){
	  	  					  		BTNcount=0;
	  	  					  	}
	  	  				break;
	  	  		}
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

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 71;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 10000;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LedBuidIn_GPIO_Port, LedBuidIn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : LedBuidIn_Pin */
  GPIO_InitStruct.Pin = LedBuidIn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LedBuidIn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BTN_Pin */
  GPIO_InitStruct.Pin = BTN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BTN_GPIO_Port, &GPIO_InitStruct);

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
