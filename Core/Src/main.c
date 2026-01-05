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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
#define I2C_ADDR 0x10 // 7 bit addr is 0x08, 8 bit addr is 0x10(write) or 0x11(read). HAL functions take 8 bit addr as arg so 0x10
uint8_t test;
uint8_t cellType; // cellType: 0 if 35E cells, else 50s cells
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

HAL_StatusTypeDef I2C_Read(uint16_t device_addr, uint8_t reg_addr, uint8_t * rData, uint16_t length) {
/*
 * device_addr: 7-bit device addr left shifted by 1
 * reg_addr: 8-bit memory addr to read from
 * *rData: ptr to read data buffer with *length* amount of bytes
 * length: amount of bytes to read
 */
/*
 *	read sequence: device addr + W bit, reg addr to read from, dev addr + R bit, read data from device

	   HAL_I2C_Master_Transmit	(
			I2C_HandleTypeDef * 	hi2c,
			uint16_t 	DevAddress,
			uint8_t * 	pData,
			uint16_t 	Size,
			uint32_t 	Timeout
		HAL_I2C_Master_Receive_DMA	(
			I2C_HandleTypeDef * 	hi2c,
			uint16_t 	DevAddress,
			uint8_t * 	pData,
			uint16_t 	Size
		)
 */
	uint8_t pData[1] = {reg_addr};
	while (HAL_I2C_Master_Transmit(&hi2c1, device_addr, pData, 1, 100) != HAL_OK) { // dev_addr + W, reg_addr
		HAL_Delay(1); // commands take <= 1 ms to complete
	}
	HAL_StatusTypeDef status = HAL_I2C_Master_Receive(&hi2c1, device_addr, rData, length, 100); // dev_addr + R, rData(TX from slave)

	return status;
}

HAL_StatusTypeDef I2C_Write(uint16_t device_addr, uint8_t reg_addr, uint8_t * block, uint16_t length) {
/*
 * device_addr: 7-bit device addr left shifted by 1
 * reg_addr: 8-bit memory addr to write to
 * *block: ptr to write data buffer with *length* amount of bytes
 * length: amount of bytes to write
 */
/*
 * 	write sequence: device addr + W bit, reg addr to write to, write data block
 *
 * 	   HAL_I2C_Master_Transmit	(
			I2C_HandleTypeDef * 	hi2c,
			uint16_t 	DevAddress,
			uint8_t * 	pData,
			uint16_t 	Size,
			uint32_t 	Timeout
 */
	uint8_t pData[length+1]; // total data to transmit is 1 byte reg_addr plus Wdata block
	pData[0] = reg_addr;
	for (int i=0; i<length; i++) {
		pData[i+1] = block[i];
	}
	return HAL_I2C_Master_Transmit(&hi2c1, device_addr, pData, length+1, 100);
}

HAL_StatusTypeDef DataRAM_Read(uint16_t addr, uint8_t * rData, uint8_t length) {
/*
 * addr: 8-bit config reg addr OR subcommand addr
 * *rData: ptr to read data buffer with *length* amount of bytes
 * length: amount of bytes to read
	 * Write address location to 0x3E and read back from 0x40
	 * Used to read configuration registers and for subcommands
 */

/*
 ex: read rdata=0x88 from the RAM address 0x9261
 sequence:
 	 0x10, 0x3e, 0x61, 0x92: write reg_addr 0x9261 to subcmd reg 0x3e
 	 0x10, 0x40, 0x11, x88: write transf. buffer 0x40 then send dev_addr + R, read data 0x88
 */
	uint8_t addressBlock[2] = {addr%256, addr/256}; // turn 2 byte subcmd addr to little endian
	HAL_StatusTypeDef wStatus = I2C_Write(I2C_ADDR, 0x3E, addressBlock, 2);
	HAL_StatusTypeDef rStatus = I2C_Read(I2C_ADDR, 0x40, rData, length);
	return (wStatus == HAL_OK && rStatus == HAL_OK) ? HAL_OK : HAL_ERROR;
}

HAL_StatusTypeDef DataRAM_Write(uint16_t addr, uint8_t * block, uint8_t length) {
/*
 * addr: RAM addr to write to
 * *block: write data
 * length: length of write data
 */

/*
ex: write 0x8C to RAM address 0x9261, write checksum(0x80)+len(0x05) to 0x60
sequence:
	0x10, 0x3e, 0x61, 0x92, 0x8c: write data=0x8c and RAM addr 0x9261 to reg=0x3e
	0x10, 0x60, 0x80, 0x05: write checksum+len to 0x60

* Write RAM addr to 0x3E and Checksum+length to 0x60
* total data length includes:
 *  2 bytes command: 0x3E and 0x3F
 *  2 bytes checksum and length: 0x60 and 0x61
 *  1 byte wData: 0x40(transfer buffer)
 *  max 32 bytes: length of the transfer buffer
 *  if the entire 32-byte transfer buffer is used, the data length will be 0x24 = 32 + 4
*/
	uint8_t addressBlock[2] = {addr%256, addr/256}; // turn 2 byte subcmd addr to little endian
	uint8_t wholeBlock[length+2];
	uint8_t checksum_len[2];

	wholeBlock[0] = addressBlock[0];
	wholeBlock[1] = addressBlock[1];

	for (int i=0; i<length; i++) {
		wholeBlock[i+2] = block[i];
	}

	checksum_len[0] = (~(wholeBlock[0] + wholeBlock[1])) & 0xFF; // checksum
	checksum_len[1] = length+4; // total data length

	HAL_StatusTypeDef wStatusCmd = I2C_Write(I2C_ADDR, 0x3E, wholeBlock, length+2); // write RAM addr and WData to 0x3e
	HAL_StatusTypeDef wStatusChecksum = I2C_Write(I2C_ADDR, 0x60, checksum_len, length+4); // write checksum+len to 0x60

	return (wStatusCmd == HAL_OK && wStatusChecksum == HAL_OK) ? HAL_OK : HAL_ERROR;
}


///////////////	13.3 Settings	///////////////

HAL_StatusTypeDef CfgRegulators(void) {
/*
 * Goal: configure registers REG0 and REG12 to set REG1=3v3 and REG2=disabled
 */
	uint8_t Reg0Data[1] = {0x01};
	uint8_t Reg12Data[1] = {0x0d};

	HAL_StatusTypeDef Reg0Status = DataRAM_Write(0x9237, Reg0Data, 1);
	HAL_StatusTypeDef Reg12Status = DataRAM_Write(0x9236, Reg12Data, 1);


	return (Reg0Status == HAL_OK && Reg12Status == HAL_OK) ? HAL_OK : HAL_ERROR;
}

HAL_StatusTypeDef CfgAlertPin(void) {
/*
 * Configures ALERT Pin function to Alert, and to drive active high using REG18
 */
	uint8_t AlertPinCfgData[1] = {0x0A};
	return DataRAM_Write(0x92FC, AlertPinCfgData, 1);
}

HAL_StatusTypeDef CfgVcellMode(void) {
/*
 * Enable cells 1, 2, 3, 4, 5, 10
 * Sequence:
 	 0x10, 0x3e, 0x04, 0x93, 0x7f, 0x03: writes 0x037F to RAM addr 0x9304
 	 0x10, 0x60, 0xe6, 0x06: writes checksum=0xe6 and length=0x06 to 0x60
 */
	uint8_t NumCellsData[2] = {0x1f, 0x02};
	return DataRAM_Write(0x9304, NumCellsData, 2);
}

HAL_StatusTypeDef CfgUserUnits(void) {
/*
 * Set voltage units to mV: bit [2] = 0
 * Set current units to mA: bits [1:0] = 1
 */
	uint8_t UserUnitsData[1] = {0x01};
	return DataRAM_Write(0x9303, UserUnitsData, 2);
}

HAL_StatusTypeDef CfgPrimaryProtections(uint8_t PrimaryProtectionsData) {
/*
 * PrimaryProtectionsData: 8 bit register Enabled Protections A that enable/disable primary protections
 * Goal: Enable the primary protections(COV, CUV, OCC, SCD, OCD1, OCD2) to control FETs, and enable FET charge pumps
 * COV:[3], CUV:[2], OCC:[4], SCD:[7], OCD1:[5], OCD2:[6]
 */
	HAL_StatusTypeDef EnabledProtectionsStatus = DataRAM_Write(0x9261, PrimaryProtectionsData, 1);
	HAL_StatusTypeDef EnabledCHGFETStatus = DataRAM_Write(0x9265, PrimaryProtectionsData, 1); // COV, OCC, SCD
	HAL_StatusTypeDef EnabledDSGFETStatus = DataRAM_Write(0x9269, PrimaryProtectionsData, 1); // CUV, OCD1, OCD2, SCD

	return (EnabledProtectionsStatus == HAL_OK && EnabledCHGFETStatus == HAL_OK && EnabledDSGFETStatus == HAL_OK) ? HAL_OK : HAL_ERROR;
}

HAL_StatusTypeDef CfgAlarmMask(void) {
	/*
	 * Alarm Mask determines which flags are able to set Alarm Status()(0x62 and 0x64 commands)
	 */
	uint8_t DefaultAlarmMaskData[2] = {0x50, 0x00};
	uint8_t AlertMaskAData[1] = {0xFC};

	HAL_StatusTypeDef DefaultAlarmMaskStatus = DataRAM_Write(0x926D, DefaultAlarmMaskData, 2); // Safety Status A and MSK_SFALERT can trigger alarm

	HAL_StatusTypeDef SFAlertMaskAStatus = DataRAM_Write(0x926F, AlertMaskAData, 1); // SF Alert Mask A: enable COV, CUV, OCC, SCD, OCD1, OCD2 to set MSK_SFALERT
	HAL_StatusTypeDef SFAlertMaskBStatus = DataRAM_Write(0x9270, 0x00, 1); // SF Alert Mask B = 0
	HAL_StatusTypeDef SFAlertMaskCStatus = DataRAM_Write(0x9271, 0x00, 1); // SF Alert Mask C = 0

	return (DefaultAlarmMaskStatus == HAL_OK && SFAlertMaskAStatus == HAL_OK && SFAlertMaskBStatus == HAL_OK && SFAlertMaskCStatus == HAL_OK) ? HAL_OK : HAL_ERROR;
}

HAL_StatusTypeDef CfgFETOptions(int test=0) {
/*
 * Sets FET Options register to enable PDSG FETs, device FET control, and optionally host FET control if test=True
 */
	uint8_t FETOptionsData[1];
	FETOptionsData[0] = (test == 1) ? 0x1C : 0x18; // if testing, bit [2] HOST_FET_EN is set to allow host commands to control FETs

	return DataRAM_Write(0x9308, FETOptionsData, 1);
}

HAL_StatusTypeDef CfgFETChgPump(void) {
	uint8_t FETChgPumpData[1] = {0x01};

	return DataRAM_Write(0x9309, FETChgPumpData, 1);
}

HAL_StatusTypeDef CfgMfgStatusInit(void) {
/*
 * Goal: Enable Permanent Failure(PF) checks and autonomous FET control by setting bits [PF_EN] and [FET_EN]
 */
	uint8_t MfgStatusInitData[2] = {0x50, 0x00};
	return DataRAM_Write(0x9343, MfgStatusInitData, 2);
}


////////////////////////////	13.6 Protections	////////////////////////////
HAL_StatusTypeDef SetPrimaryCUV(uint8_t cellType) {
	uint8_t threshold[1] = { (cellType == 0) ? 0x53 : 0x53 };
	return DataRAM_Write(0x9275, threshold, 1);
}

HAL_StatusTypeDef SetPrimaryCOV(uint8_t cellType) {
	uint8_t threshold[1] = { (cellType == 0) ? 0x34 : 0x31 };
	return DataRAM_Write(0x9278, threshold, 1);
}

HAL_StatusTypeDef SetPrimaryOCC(uint8_t cellType) {
	uint8_t threshold[1] = { (cellType == 0) ? 0x07 : 0x0D };
	return DataRAM_Write(0x9280, threshold, 1);
}

HAL_StatusTypeDef SetPrimaryOCD1(uint8_t cellType) {
	uint8_t threshold[1] = { (cellType == 0) ? 0x07 : 0x0D };
	return DataRAM_Write(0x9282, threshold, 1);
}

HAL_StatusTypeDef SetPrimaryOCD2(uint8_t cellType) {
	uint8_t threshold[1] = { (cellType == 0) ? 0x07 : 0x0D };
	return DataRAM_Write(0x9284, threshold, 1);
}

HAL_StatusTypeDef SetPrimarySCD(uint8_t cellType) {
	uint8_t threshold[1] = { (cellType == 0) ? 0x01 : 0x02 };
	return DataRAM_Write(0x9286, threshold, 1);
}

///////////////////		Commands

HAL_StatusTypeDef EnterConfigUpdateMode(void) {
	uint8_t wholeBlock[2] = {0x90, 0x00};
	HAL_StatusTypeDef status I2C_Write(I2C_ADDR, 0x3E, wholeBlock, 2);
	HAL_Delay(3); // cmd 0x0090 takes approx 2 ms
	return status;
}

HAL_StatusTypeDef ExitConfigUpdateMode(void) {
	uint8_t wholeBlock[2] = {0x92, 0x00};
	HAL_StatusTypeDef status = I2C_Write(I2C_ADDR, 0x3E, wholeBlock, 2);
	HAL_Delay(2); // cmd 0x0092 takes approx 1 ms
	return status;
}

HAL_StatusTypeDef AllFETsON(void) {
/*
 * 0x0096 ALL_FETS_ON(): Allows all FETs to be enabled if nothing else is blocking them
 */
	uint8_t wholeBlock[2] = {0x96, 0x00};
	return I2C_Write(I2C_ADDR, 0x3E, wholeBlock, 2);
}

HAL_StatusTypeDef AllFETsOFF(void) {
/*
 * 0x0095 ALL_FETS_OFF(): disables all FETs
 */
	uint8_t wholeBlock[2] = {0x95, 0x00};
	return I2C_Write(I2C_ADDR, 0x3E, wholeBlock, 2);
}


HAL_StatusTypeDef FETStatusCmd(uint8_t * rData) {
/*
 * FET Status() 0x7f read only command: check if CHG and DSG FETs are on or off
 * bit [0]: 1 if CHG FET on, 0 if off
 * bit [2]: 1 if DSG FET on, 0 if off
 */

	return I2C_Read(I2C_ADDR, 0x7F, rData, 1);
}

HAL_StatusTypeDef CellVoltageCmd(uint8_t cell, uint8_t * CellVoltage) {
/*
 * Goal: use VCell direct commands to read voltage of cell
 * cell: index of the cell to read. available cells are 1, 2, 3, 4, 5, 10.
 * CellVoltage: array of two uint8_t, return the read cell voltage
 *
 * ex: read cell 1 voltage. cmd to read cell 1 V is 0x14. 2 byte read returns rData=0x0E74=3700 mV
 * sequence: 0x10, 0x14, 0x11, 0x74, 0x0e
 */
	uint8_t cmd_addr = 0x12 + (cell * 2);

	return I2C_Read(I2C_ADDR, cmd_addr, CellVoltage, 2); // returns 16 bit(2 byte) mV in little endian
}

HAL_StatusTypeDef AllCellVoltageCmd(uint16_t * AllCellVoltages) {
/*
 * Read and display voltage of all enabled cells: 1, 2, 3, 4, 5, 10
 */
	char buffer[100];

	I2C_Read(I2C_ADDR, 0x14, (uint8_t *)&AllCellVoltages[0], 2);
	I2C_Read(I2C_ADDR, 0x16, (uint8_t *)&AllCellVoltages[1], 2);
	I2C_Read(I2C_ADDR, 0x18, (uint8_t *)&AllCellVoltages[2], 2);
	I2C_Read(I2C_ADDR, 0x1A, (uint8_t *)&AllCellVoltages[3], 2);
	I2C_Read(I2C_ADDR, 0x1C, (uint8_t *)&AllCellVoltages[4], 2);
	I2C_Read(I2C_ADDR, 0x26, (uint8_t *)&AllCellVoltages[5], 2);

	sprintf(buffer, "Cell 1: %u mV, Cell 2: %u mV, Cell 3: %u mV\n", AllCellVoltages[0], AllCellVoltages[1], AllCellVoltages[2]);
	HAL_UART_Transmit(&huart2, buffer, strlen(buffer), 100);

	sprintf(buffer, "Cell 4: %u mV, Cell 5: %u mV, Cell 10: %u mV\n", AllCellVoltages[3], AllCellVoltages[4], AllCellVoltages[5]);
	HAL_UART_Transmit(&huart2, buffer, strlen(buffer), 100);
}

HAL_StatusTypeDef CC2CurrentCmd(uint8_t * CC2Current) {
	uint8_t cmd_addr = 0x3A;
	char buffer[50];

	return I2C_Read(I2C_ADDR, cmd_addr, CC2Current, 2); // returns 16 bit(2 byte) mV in little endian
}

HAL_StatusTypeDef test1(void) {
/*
 * Test 1: configure VCell Mode, regulators, user units.
 * ensure correct voltage at regulator pins.
*/
	uint8_t * rData;
	char buffer[100];

	HAL_StatusTypeDef status = EnterConfigUpdateMode();
	CfgVcellMode(); // enables cells 1-5 and 10
	CfgRegulators(); // configures REG1 to 3.3 V and REG18 to 1.8 V
	CfgUserUnits(); // sets user-volts to mV and user-amps to mA
	ExitConfigUpdateMode();

	DataRAM_Read(0x9304, rData, 2); // vcell mode
	sprintf(buffer, "RAM read data at 0x9304: %u \n", *(uint16_t *)rData);
	HAL_UART_Transmit(&huart2, buffer, strlen(buffer), 100);

	DataRAM_Read(0x9236, rData, 1); // reg12
	sprintf(buffer, "RAM read data at 0x9236: %u \n", *rData);
	HAL_UART_Transmit(&huart2, buffer, strlen(buffer), 100);

	DataRAM_Read(0x9237, rData, 1); // reg0
	sprintf(buffer, "RAM read data at 0x9237: %u \n", *rData);
	HAL_UART_Transmit(&huart2, buffer, strlen(buffer), 100);

	DataRAM_Read(0x9303, rData, 1); // user units
	sprintf(buffer, "RAM read data at 0x9303: %u \n", *rData);
	HAL_UART_Transmit(&huart2, buffer, strlen(buffer), 100);

	return status;
}

HAL_StatusTypeDef test2(void) {
/*
* Test 2: Verify that IC can read voltage of each cell
*/
	uint8_t * CellVoltage;
	char buffer[100];
	int cell = 1; // repeat for cell = 1, 2, 3, 4, 5, 10

	HAL_StatusTypeDef status = EnterConfigUpdateMode();
	CfgVcellMode();
	CfgRegulators();
	CfgUserUnits();
	ExitConfigUpdateMode();

	CellVoltageCmd(cell, CellVoltage);
	sprintf(buffer, "Voltage of cell %u: %u mV\n", cell, *(uint16_t *)CellVoltage);

	return status;
}

HAL_StatusTypeDef test3(void) {
/*
* Test 3: Verify that IC can read pack current
*/
	uint8_t * CC2Current;
	char buffer[100];

	HAL_StatusTypeDef status = EnterConfigUpdateMode();
	CfgVcellMode();
	CfgRegulators();
	CfgUserUnits();
	ExitConfigUpdateMode();

	CC2CurrentCmd(CC2Current);
	sprintf(buffer, "Pack current: %u mA\n", *(uint16_t *)CC2Current);

	return status;
}

HAL_StatusTypeDef test4(void) {
/*
 * Test 4: Configure COV threshold and ensure that OV turns CHG FETs off
 * Poll CHG FET status while increasing voltage applied at cell terminals
 * Ensure that CHG FETs turn off once applied voltage reaches COV threshold
 */
	test = 4;
	//uint8_t * CellVoltage;
	char buffer[100];
	//int cell = 1; // repeat for cell = 1, 2, 3, 4, 5, 10

	HAL_StatusTypeDef status = EnterConfigUpdateMode();
	CfgVcellMode();
	CfgRegulators();
	CfgUserUnits();

	CfgPrimaryProtections(0b00001000); // enable COV = primary protection A[3]

	CfgFETOptions(); // configure FETs
	CfgFETChgPump();
	CfgMfgStatusInit();

	SetPrimaryCOV(cellType); // set COV threshold value

	AllFETsON();

	ExitConfigUpdateMode();

	//CellVoltageCmd(cell, CellVoltage);
	//sprintf(buffer, "Voltage of cell %u: %u mV\n", cell, *(uint16_t *)CellVoltage);
	//FETStatusCmd(uint8_t * rData) // bit[0] is CHG FET status

	HAL_StatusTypeDef CellVoltageCmd

	return status;

}

void test5(void) {

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
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  test1();
  test2();
  test3();
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */


  /* USER CODE END I2C1_Init 2 */

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
