/*  Arduino Library for the Lepton FLiR Thermal Camera Module.
    Copyright (c) 2016 NachtRaveVL      <nachtravevl@gmail.com>

    Permission is hereby granted, free of charge, to any person
    obtaining a copy of this software and associated documentation
    files (the "Software"), to deal in the Software without
    restriction, including without limitation the rights to use,
    copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the
    Software is furnished to do so, subject to the following
    conditions:

    This permission notice shall be included in all copies or
    substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
    EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
    OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
    NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
    HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
    WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
    FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
    OTHER DEALINGS IN THE SOFTWARE.

    Lepton-FLiR-Arduino - Version 0.9.91
*/

/*
NOTE: 	This is not the original library from  ->  NachtRaveVL      <nachtravevl@gmail.com>
		It has been modified in order to be useful with a STM32 F401RE micro controller
		Few things are different from the original library. The methods for the I2C interfacing
		and the SPI routine needed to be completely redefined, also other minor little functions
		needed to be redefined. In addition some functions from the section OEM and RAD of the
		data sheet (Document Number: 110-0144-04 Rev 200) have been added together with some
		registers and enum needed from those.

		Notice also that up to now the camera works with configuration RAW14 (black and white)
		and RGB888 (color image). Where the frame have respectively size 80x60 and 240x60. Other
		sizes were defined previously by the OC and in order to use those, some function for compressing
		the frames should be written.
*/



#include "main.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <algorithm>
#include <math.h>
#include "LeptonFLiR.h"

// STM peripherals
#include "i2c.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"



// buffers for the uart debug message
int msg_len;
char msg[50];

bool agc8Enabled;
LEP_SYS_TELEMETRY_LOCATION telemetryLocation;

uint16_t startUpTime = HAL_GetTick();

unsigned long millis(){

	uint16_t currentTime = HAL_GetTick();
	return currentTime-startUpTime-1;
}

int constrain(int x, int a, int b){
	if(a<b){
		if(x>a && x<b){
			return x;
		} else if (x<=a){
			return a;
		} else if(x>=b){
			return b;
		}
	}else{
		if(x>b && x<a){
			return x;
		} else if (x>=a){
			return a;
		} else if(x<=b){
			return b;
		}
	}
}



#define LEPFLIR_GEN_CMD_TIMEOUT         			 	5000   	// Timeout for commands to be processed
#define LEPFLIR_SPI_FRAME_PACKET_SIZE_RAW14				164 	// 2B ID + 2B CRC + 160B for 80x1 14bpp/8bppAGC thermal image data or telemetry data
#define LEPFLIR_SPI_FRAME_PACKET_SIZE16_RAW14         	82		// same as above but considering size 16 bits
#define LEPFLIR_SPI_FRAME_PACKET_SIZE_RGB888          	244		// size of a line (1 packet)in the RGB configuration
#define LEPFLIR_SPI_FRAME_PACKET_SIZE16_RGB888		  	122		// size of a line (1 packet) for the B&W configuration

int LEPFLIR_SPI_FRAME_PACKET_SIZE = 0;
int LEPFLIR_SPI_FRAME_PACKET_SIZE16 = 0;

#ifndef LEPFLIR_DISABLE_ALIGNED_MALLOC
static inline int roundUpVal16(int val) { return ((val + 15) & -16); }
static inline byte *roundUpPtr16(byte *ptr) { return ptr ? (byte *)(((uintptr_t)ptr + 15) & -16) : NULL; }
static inline byte *roundUpMalloc16(int size) { return (byte *)malloc((size_t)(size + 15)); }
static inline byte *roundUpSpiFrame16(byte *spiFrame) { return spiFrame ? roundUpPtr16(spiFrame) + 16 - 4 : NULL; }
#else
static inline int roundUpVal16(int val) { return val; }
static inline byte *roundUpPtr16(byte *ptr) { return ptr; }
static inline byte *roundUpMalloc16(int size) { return (byte *)malloc((size_t)size); }
static inline byte *roundUpSpiFrame16(byte *spiFrame) { return spiFrame; }
#endif



LeptonFLiR::LeptonFLiR() {
    _storageMode = LeptonFLiR_ImageStorageMode_Count;
    _imageData = _spiFrameData = _telemetryData = NULL;
    _isReadingNextFrame = false;
    _lastI2CError = _lastLepResult = 0;
}

LeptonFLiR::~LeptonFLiR() {
    if (_imageData) free(_imageData);
    if (_spiFrameData) free(_spiFrameData);
    if (_telemetryData) free(_telemetryData);
}



void LeptonFLiR::init(LeptonFLiR_ImageStorageMode storageMode, LeptonFLiR_TemperatureMode tempMode) {
    _storageMode = (LeptonFLiR_ImageStorageMode)constrain((int)storageMode, 0, (int)LeptonFLiR_ImageStorageMode_Count - 1);
    _tempMode    = (LeptonFLiR_TemperatureMode)constrain((int)tempMode, 0, (int)LeptonFLiR_TemperatureMode_Count - 1);

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET); 	// Deafult setting for the CS pin (default high = disabled slave)

#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
	msg_len = sprintf(msg, "\n\r LeptonFLiR::init,   storageMode: %i,   tempMode: %i", _storageMode, _tempMode);
	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
#endif

    _imageData = roundUpMalloc16(getImageTotalBytes());
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    if (!_imageData){
    	msg_len = sprintf(msg, "\n\r  LeptonFLiR::init Failure allocating imageData. ");
    	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
    }
#endif

    _spiFrameData = roundUpMalloc16(getSPIFrameTotalBytes());
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    if (!_spiFrameData){
    	msg_len = sprintf(msg, "\n\r  LeptonFLiR::init Failure allocating spiFrameData.");
    	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
    }
#endif

#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    int mallocOffset = 0;
#ifndef LEPFLIR_DISABLE_ALIGNED_MALLOC
    mallocOffset = 15;
#endif

	msg_len = sprintf(msg, "\n\n\r  LeptonFLiR::init imageData: %i B", (_imageData ? getImageTotalBytes() + mallocOffset : 0));
	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);

	msg_len = sprintf(msg, "    spiFrameData: %i B", (_spiFrameData ? getSPIFrameTotalBytes() + mallocOffset : 0));
	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);

	msg_len = sprintf(msg, "    total: %i B", (_imageData ? getImageTotalBytes() + mallocOffset : 0) + (_spiFrameData ? getSPIFrameTotalBytes() + mallocOffset : 0));
	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);

	msg_len = sprintf(msg, "\n\n\r  LeptonFLiR::init SPIPortSpeed:  ancora da scommentare");
	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);

#endif
}



LeptonFLiR_ImageStorageMode LeptonFLiR::getImageStorageMode() {
    return _storageMode;
}

LeptonFLiR_TemperatureMode LeptonFLiR::getTemperatureMode() {
    return _tempMode;
}


int LeptonFLiR::getImageWidth() {
    switch (_storageMode) {
		case LeptonFLiR_ImageStorageMode_RGB888:
		case LeptonFLiR_ImageStorageMode_RAW14:
        case LeptonFLiR_ImageStorageMode_80x60_16bpp:
        case LeptonFLiR_ImageStorageMode_80x60_8bpp:
            return 80;
        case LeptonFLiR_ImageStorageMode_40x30_16bpp:
        case LeptonFLiR_ImageStorageMode_40x30_8bpp:
            return 40;
        case LeptonFLiR_ImageStorageMode_20x15_16bpp:
        case LeptonFLiR_ImageStorageMode_20x15_8bpp:
            return 20;
        default:
            return 0;
    }
}

int LeptonFLiR::getImageHeight() {
    switch (_storageMode) {
		case LeptonFLiR_ImageStorageMode_RAW14:
		case LeptonFLiR_ImageStorageMode_RGB888:
        case LeptonFLiR_ImageStorageMode_80x60_16bpp:
        case LeptonFLiR_ImageStorageMode_80x60_8bpp:
            return 60;
        case LeptonFLiR_ImageStorageMode_40x30_16bpp:
        case LeptonFLiR_ImageStorageMode_40x30_8bpp:
            return 30;
        case LeptonFLiR_ImageStorageMode_20x15_16bpp:
        case LeptonFLiR_ImageStorageMode_20x15_8bpp:
            return 15;
        default:
            return 0;
    }
}

int LeptonFLiR::getImageBpp() {
    switch (_storageMode) {
		case LeptonFLiR_ImageStorageMode_RGB888:
			return 3;
    	case LeptonFLiR_ImageStorageMode_RAW14:
        case LeptonFLiR_ImageStorageMode_80x60_16bpp:
        case LeptonFLiR_ImageStorageMode_40x30_16bpp:
        case LeptonFLiR_ImageStorageMode_20x15_16bpp:
            return 2;
        case LeptonFLiR_ImageStorageMode_80x60_8bpp:
        case LeptonFLiR_ImageStorageMode_40x30_8bpp:
        case LeptonFLiR_ImageStorageMode_20x15_8bpp:
            return 1;
        default:
            return 0;
    }
}

int LeptonFLiR::getImagePitch() {
    switch (_storageMode) {
		case LeptonFLiR_ImageStorageMode_RGB888:
			return roundUpVal16(80 * 3);
		case LeptonFLiR_ImageStorageMode_RAW14:
			return roundUpVal16(80 * 2);
        case LeptonFLiR_ImageStorageMode_80x60_16bpp:
            return roundUpVal16(80 * 2);
        case LeptonFLiR_ImageStorageMode_80x60_8bpp:
            return roundUpVal16(80 * 1);
        case LeptonFLiR_ImageStorageMode_40x30_16bpp:
            return roundUpVal16(40 * 2);
        case LeptonFLiR_ImageStorageMode_40x30_8bpp:
            return roundUpVal16(40 * 1);
        case LeptonFLiR_ImageStorageMode_20x15_16bpp:
            return roundUpVal16(20 * 2);
        case LeptonFLiR_ImageStorageMode_20x15_8bpp:
            return roundUpVal16(20 * 1);
        default:
            return 0;
    }
}

int LeptonFLiR::getImageTotalBytes() {
    return ((getImageHeight() - 1) * getImagePitch()) + (getImageWidth() * getImageBpp());
}

byte *LeptonFLiR::getImageData() {
    return !_isReadingNextFrame && _imageData ? roundUpPtr16(_imageData) : NULL;
}

byte *LeptonFLiR::getImageDataRow(int row) {
    return !_isReadingNextFrame && _imageData ? (roundUpPtr16(_imageData) + (row * getImagePitch())) : NULL;
}

byte *LeptonFLiR::_getImageDataRow(int row) {
    return _imageData ? roundUpPtr16(_imageData) + (getImagePitch() * row) : NULL;
}

uint16_t LeptonFLiR::getImageDataRowCol(int row, int col) {
    if (_isReadingNextFrame || !_imageData) return 0;
    byte *imageData = roundUpPtr16(_imageData) + (row * getImagePitch()) + (col * getImageBpp());
    return getImageBpp() == 2 ? *((uint16_t *)imageData) : (uint16_t)(*imageData);
}

uint8_t LeptonFLiR::RGB_getImageDataRowCol_AVERAGE(int row, int col) {
    if (_isReadingNextFrame || !_imageData) return 0;

    uint8_t *center, *up, *down, *left, *right, pixel;

	if(row>0 && row<60 && col>0 && col<239){
	    center = roundUpPtr16(_imageData) + (row     * getImagePitch()) + col;
	    up     = roundUpPtr16(_imageData) + ((row-1) * getImagePitch()) + col;
	    down   = roundUpPtr16(_imageData) + ((row+1) * getImagePitch()) + col;
	    left   = roundUpPtr16(_imageData) + (row     * getImagePitch()) + col-1;
	    right  = roundUpPtr16(_imageData) + (row     * getImagePitch()) + col+1;

	    pixel  = (*center) *0.6 + (*up) *0.1 + (*down) *0.1 + *(left) *0.1 + (*right) *0.1;
	}else{
		center = roundUpPtr16(_imageData) + (row * getImagePitch()) + col;
		pixel = *center;
	}

    return  (uint8_t)(pixel);
}

uint8_t LeptonFLiR::RGB_getImageDataRowCol(int row, int col) {
    if (_isReadingNextFrame || !_imageData) return 0;
    uint8_t *imageData = roundUpPtr16(_imageData) + (row * getImagePitch()) + col;
    return  (uint8_t)(*imageData);
}

uint8_t LeptonFLiR::RAW_getImageDataRowCol(int row, int col) {
    if (_isReadingNextFrame || !_imageData) return 0;
    uint8_t *imageData = roundUpPtr16(_imageData) + (row * getImagePitch()) + (col*2) + 1;
    return  (uint8_t)(*imageData);
}


byte *LeptonFLiR::getTelemetryData() {
    return !_isReadingNextFrame && _telemetryData && !(*((uint16_t *)_telemetryData) & 0x0F00 == 0x0F00) ? _telemetryData : NULL;
}

void LeptonFLiR::getTelemetryData(TelemetryData *telemetry) {
    if (_isReadingNextFrame || !_telemetryData || !telemetry) return;
    uint16_t *telemetryData = (uint16_t *)&_telemetryData[4];

    telemetry->revisionMajor = telemetryData[0];       // low byte
    telemetry->revisionMinor = (telemetryData[0])>>8;  // high byte

    telemetry->cameraUptime = ((uint32_t)telemetryData[1] << 16) | (uint32_t)telemetryData[2];

    telemetry->ffcDesired = telemetryData[4] & 0x0004;
    uint_fast8_t ffcState = (telemetryData[4] & 0x0018) >> 3;
    if (telemetry->revisionMajor >= 9 && ffcState >= 1)
        ffcState -= 1;
    telemetry->ffcState = (TelemetryData_FFCState)ffcState;
    telemetry->agcEnabled = telemetryData[4] & 0x0800;
    telemetry->shutdownImminent = telemetryData[3] & 0x0010;

    wordsToHexString(&telemetryData[5], 8, telemetry->serialNumber, 24);
    wordsToHexString(&telemetryData[13], 4, telemetry->softwareRevision, 12);

    telemetry->frameCounter = ((uint32_t)telemetryData[20] << 16) | (uint32_t)telemetryData[21];
    telemetry->frameMean = telemetryData[22];

    telemetry->fpaTemperature = kelvin100ToTemperature(telemetryData[24]);
    telemetry->housingTemperature = kelvin100ToTemperature(telemetryData[26]);

    telemetry->lastFFCTime = ((uint32_t)telemetryData[30] << 16) | (uint32_t)telemetryData[31];
    telemetry->fpaTempAtLastFFC = kelvin100ToTemperature(telemetryData[29]);
    telemetry->housingTempAtLastFFC = kelvin100ToTemperature(telemetryData[32]);

    telemetry->agcRegion.startRow = telemetryData[34];
    telemetry->agcRegion.startCol = telemetryData[35];
    telemetry->agcRegion.endCol = telemetryData[36];
    telemetry->agcRegion.endRow = telemetryData[37];

    telemetry->agcClipHigh = telemetryData[38];
    telemetry->agcClipLow = telemetryData[39];

    telemetry->log2FFCFrames = telemetryData[74];
}

uint32_t LeptonFLiR::getTelemetryFrameCounter() {
    if (_isReadingNextFrame || !_telemetryData) return 0;
    uint16_t *telemetryData = (uint16_t *)&_telemetryData[4];

    return ((uint32_t)telemetryData[20] << 16) | (uint32_t)telemetryData[21];
}

bool LeptonFLiR::getShouldRunFFCNormalization() {
    if (_isReadingNextFrame || !_telemetryData) return false;
    uint16_t *telemetryData = (uint16_t *)&_telemetryData[4];

    uint_fast8_t ffcState = (telemetryData[4] & 0x0018) >> 3;
    if ((telemetryData[0]) >= 9 && ffcState >= 1)
        ffcState -= 1;

    return (telemetryData[4] & 0x0004) && ffcState != (uint_fast8_t)TelemetryData_FFCState_InProgress;
}

int LeptonFLiR::getSPIFrameLines() {
    switch (_storageMode) {
		case LeptonFLiR_ImageStorageMode_RGB888:
    	case LeptonFLiR_ImageStorageMode_RAW14:
        case LeptonFLiR_ImageStorageMode_80x60_16bpp:
        case LeptonFLiR_ImageStorageMode_80x60_8bpp:
            return 1;
        case LeptonFLiR_ImageStorageMode_40x30_16bpp:
        case LeptonFLiR_ImageStorageMode_40x30_8bpp:
            return 2;
        case LeptonFLiR_ImageStorageMode_20x15_16bpp:
        case LeptonFLiR_ImageStorageMode_20x15_8bpp:
            return 4;
        default:
            return 0;
    }
}

int LeptonFLiR::getSPIFrameTotalBytes() {
    return getSPIFrameLines() * roundUpVal16(LEPFLIR_SPI_FRAME_PACKET_SIZE);
}

uint16_t *LeptonFLiR::getSPIFrameDataRow(int row) {
    return (uint16_t *)(roundUpSpiFrame16(_spiFrameData) + (row * roundUpVal16(LEPFLIR_SPI_FRAME_PACKET_SIZE)));
}


static void printSPIFrame(uint16_t *spiFrame) {
	msg_len = sprintf(msg, "\n\r ID: 0x%x    CRC: 0x%x \n\r Data: ", spiFrame[0], spiFrame[1]);
	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
    for (int i = 0; i < 5; ++i) {
    	if(i>0){
    		msg_len = sprintf(msg, "-0x%x", spiFrame[i + 2]);
    	} else {
    		msg_len = sprintf(msg, "0x%x", spiFrame[i + 2]);
    	}
    	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
    }
	msg_len = sprintf(msg, "  ...  ");
	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
    for (int i = 75; i < 80; ++i) {
    	if(i>75){
    		msg_len = sprintf(msg, "-0x%x", spiFrame[i + 2]);
    	}else{
    		msg_len = sprintf(msg, "0x%x", spiFrame[i + 2]);
    	}
    	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
    }
}


// SPI_Transfer uses bare metal command in contrast to HAL function because the
// HAL methods spent too much time for the transmission and risk to lose half of the packet
__attribute__((optimize("-Ofast"))) static void SPI_Transfer(uint16_t *buffer, int count) {

	uint8_t tx_buf[1] = {0};
	uint16_t rx_val;
	uint16_t j = 0;

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET); 	// Enable CS

	// cycle over the entire length of a packet (1 line of the frame)
	while(j < count) {

		// SPI transmit (useless, MOSI can be grounded)
		while (((SPI1->SR)&(SPI_FLAG_TXE)) == 0);
		SPI1->DR = *tx_buf;

		// SPI receive
		while((SPI1->SR & SPI_FLAG_RXNE)==0){ }
		rx_val = SPI1->DR;

		// because of mempcy (used later to save frame in allocated space) I need to flip high and low byte
		// do not change the order of the first 4 bytes (contain ID and RC)
		if(j>=2){
			*buffer++ = (uint8_t)rx_val << 8 | (uint8_t)(rx_val >>8);
		} else {
			*buffer++ = rx_val;
		}

		j+=1;
	}

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);  	// Disable CS

}



bool LeptonFLiR::readInitData() {

#ifdef ELAPSED_TIME_OUTPUT
    uint16_t timStart1 = HAL_GetTick();
#endif

	bool telemetryEnabled, cameraBooted, stateErrors = false;
	uint32_t value = 0;

	// legge AGC state
	receiveCommand(cmdCode(LEP_CID_AGC_ENABLE_STATE, LEP_I2C_COMMAND_TYPE_GET), &value);
	agc8Enabled = value;
	stateErrors = stateErrors || _lastI2CError || _lastLepResult;

	if (agc8Enabled) {
		receiveCommand(cmdCode(LEP_CID_AGC_HEQ_SCALE_FACTOR, LEP_I2C_COMMAND_TYPE_GET), &value);
		agc8Enabled = (value == (uint32_t)LEP_AGC_SCALE_TO_8_BITS);
		stateErrors = stateErrors || _lastI2CError || _lastLepResult;
	}

	// legge Telemetry state
	receiveCommand(cmdCode(LEP_CID_SYS_TELEMETRY_ENABLE_STATE, LEP_I2C_COMMAND_TYPE_GET), &value);
	telemetryEnabled = value;
	stateErrors = stateErrors || _lastI2CError || _lastLepResult;

	if (telemetryEnabled) {
		receiveCommand(cmdCode(LEP_CID_SYS_TELEMETRY_LOCATION, LEP_I2C_COMMAND_TYPE_GET), &value);
		telemetryLocation = (LEP_SYS_TELEMETRY_LOCATION)value;
		stateErrors = stateErrors || _lastI2CError || _lastLepResult;
	}

	uint16_t status; readRegister(LEP_I2C_STATUS_REG, &status);

	#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
	checkForErrors();
	#endif

	// Check for errors
	cameraBooted = (status & LEP_I2C_STATUS_BOOT_MODE_BIT_MASK) && (status & LEP_I2C_STATUS_BOOT_STATUS_BIT_MASK);
	stateErrors = stateErrors || _lastI2CError || _lastLepResult;

	if (stateErrors) {
	#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
		msg_len = sprintf(msg, "\n\r  LeptonFLiR::readNextFrame Errors reading state encountered. Aborting.");
		HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
	#endif
		_isReadingNextFrame = false;
		return false;
	}

	if (!cameraBooted) {
	#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
		msg_len = sprintf(msg, "\n\r  LeptonFLiR::readNextFrame Camera has not yet booted. Aborting.");
		HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
	#endif
		_isReadingNextFrame = false;
		return false;
	}

	// alloca o libera memoria per la telemetry data
	if (telemetryEnabled && !_telemetryData) {
		_telemetryData = (byte *)malloc(LEPFLIR_SPI_FRAME_PACKET_SIZE);

		if (_telemetryData){
			_telemetryData[0] = _telemetryData[1] = 0xFF; // initialize as discard packet

		}

	#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
		if (!_telemetryData){
			msg_len = sprintf(msg, "\n\r  LeptonFLiR::readNextFrame Failure allocating telemetryData.");
			HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
		}

	#endif
	}
	else if (!telemetryEnabled && _telemetryData) {
		free(_telemetryData);
		_telemetryData = NULL;
	}

	#ifdef ELAPSED_TIME_OUTPUT
	    uint32_t timStop1 = HAL_GetTick();
	    msg_len = sprintf(msg, "\n\r  LeptonFLiR::Section :I2C       Elapsed time : %d ms", timStop1 - timStart1 - 1);
		HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
	#endif
	#ifdef ELAPSED_TIME_OUTPUT
	    uint32_t timStart2 = HAL_GetTick();
	#endif

	#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
		if(agc8Enabled){
			msg_len = sprintf(msg, "\n\r  LeptonFLiR::readNextFrame AGC-8bit: enabled \n\r Telemetry:   ");
		}
		else{
			msg_len = sprintf(msg, "\n\r  LeptonFLiR::readNextFrame AGC-8bit: disabled \n\r Telemetry:  ");
		}
		HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);

		if (_telemetryData) {
			if(telemetryLocation == LEP_TELEMETRY_LOCATION_HEADER){
				msg_len = sprintf(msg, "enabled,\n\r Location: header  ");
			}
			else{
				msg_len = sprintf(msg, "enabled,\n\r Location: footer  ");
			}
			HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
		}
		else{
			msg_len = sprintf(msg, "disabled");
			HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
		}
	#endif


		LEP_OEM_VIDEO_OUTPUT_FORMAT video_out_mode = oem_getVideoMode();

		if(video_out_mode == LEP_VIDEO_OUTPUT_FORMAT_RGB888){
			LEPFLIR_SPI_FRAME_PACKET_SIZE = LEPFLIR_SPI_FRAME_PACKET_SIZE_RGB888;
			LEPFLIR_SPI_FRAME_PACKET_SIZE16 = LEPFLIR_SPI_FRAME_PACKET_SIZE16_RGB888;
		}else {
			LEPFLIR_SPI_FRAME_PACKET_SIZE = LEPFLIR_SPI_FRAME_PACKET_SIZE_RAW14;
			LEPFLIR_SPI_FRAME_PACKET_SIZE16 = LEPFLIR_SPI_FRAME_PACKET_SIZE16_RAW14;
		}

		return true;
}



#define LEPFLIR_ENABLE_FRAME_PACKET_DEBUG_OUTPUT    1

bool LeptonFLiR::readNextFrame() {
    if (!_isReadingNextFrame) {
        _isReadingNextFrame = true;


        uint16_t *spiFrame = NULL;
        uint_fast8_t imgRows = getImageHeight();
        uint_fast8_t currImgRow = 0;
        uint_fast8_t spiRows = getSPIFrameLines();
        uint_fast8_t currSpiRow = 0;
        uint_fast8_t teleRows = (_telemetryData ? 3 : 0);
        uint_fast8_t currTeleRow = 0;
        uint_fast8_t currReadRow = 0;
        uint_fast8_t framesSkipped = 0;
        uint_fast8_t currRow = 0;
        bool skipFrame = false;
        bool spiPacketRead = false;

		__HAL_SPI_ENABLE(&hspi1);	// initialize the SPI peripheral (necessary because of how the SPI routine is written)

		// cycle over the 60 lines and the 4 segments (segments only if telemetry is enabled)
		while (currImgRow < imgRows || currTeleRow < teleRows) {

			// if packet has already been saved skip (because it was in discard routine, and a image packet was found)
			if (!spiPacketRead) {
				spiFrame = getSPIFrameDataRow(currSpiRow);

				SPI_Transfer(spiFrame, LEPFLIR_SPI_FRAME_PACKET_SIZE16);  	// read 1 packet/line of the frame

				skipFrame = ((spiFrame[0] & 0x0F00) == 0x0F00);             // read in ID of the packet if it's discard
				currRow = spiFrame[0] & 0x00FF;                             // number of the packet, from 0 to 60
			}
			else{
				spiPacketRead = false;
			}
			// cathegorize the packet depending on what ID says
			// IMAGE PACKET
			if (!skipFrame && currRow == currReadRow && (
				((!teleRows || telemetryLocation == LEP_TELEMETRY_LOCATION_FOOTER) && currRow < 60) ||
				(telemetryLocation == LEP_TELEMETRY_LOCATION_HEADER && currReadRow >= teleRows))) {

				++currReadRow; ++currSpiRow;
			}
			// TELEMETRY PACKET
			else if (!skipFrame && currRow == currReadRow && teleRows &&
				((telemetryLocation == LEP_TELEMETRY_LOCATION_HEADER && currReadRow < teleRows) ||
				 (telemetryLocation == LEP_TELEMETRY_LOCATION_FOOTER && currReadRow >= 60))) {

				// if packet is a telemetry save the encoded configuration
				if (currTeleRow == 0){
					memcpy(_telemetryData, spiFrame, LEPFLIR_SPI_FRAME_PACKET_SIZE);
				}

				++currReadRow; ++currTeleRow;
			}
			// IGNORE PACKET
			else if (!skipFrame && currRow < currReadRow) {
				// do nothing
			}
			// DISCARD PACKET
			else {

				// if the discard packet is not the first packet of the frame retry sync
				if (skipFrame && (currReadRow || framesSkipped)) {
					HAL_Delay(186);			// from data sheet, retry sync == disable CS, wait 185 ms, enable CS
				}

				uint_fast8_t triesLeft =120;
				spiPacketRead = true;

				// read other 120 packets and try find a good one
				while (triesLeft > 0) {

					SPI_Transfer(spiFrame, LEPFLIR_SPI_FRAME_PACKET_SIZE16);  // read packet

					skipFrame = ((spiFrame[0] & 0x0F00) == 0x0F00);  			// save category of packe from ID
					currRow = (spiFrame[0] & 0x00FF);                			// save its number

					// if not discard packet
					if (!skipFrame) {

						if (currRow == currReadRow) { // Reestablished sync at position we're next expecting
							break;
						}
						else if (currRow == 0) { // Reestablished sync at next frame position

							if ((currReadRow || framesSkipped) && ++framesSkipped >= 5) {

								_isReadingNextFrame = false;

								return false;
							}
							else {

								currReadRow = currImgRow = currSpiRow = currTeleRow = 0;

								uint16_t* prevSPIFrame = spiFrame;
								spiFrame = getSPIFrameDataRow(currSpiRow);
								if (spiFrame != prevSPIFrame)
									memcpy(spiFrame, prevSPIFrame, LEPFLIR_SPI_FRAME_PACKET_SIZE);

								break;
							}
						}
					}

					--triesLeft;
				}

				// if I failed to find good packet in 120 read, exit from readNextFrame
				if (triesLeft == 0) {
					_isReadingNextFrame = false;
					return false;
				}
			}

            // Write out to frame
			if (currSpiRow == spiRows) {

				// for now only the versions of RAW14 and RGB888 are working
				if (_storageMode == LeptonFLiR_ImageStorageMode_RAW14 || _storageMode == LeptonFLiR_ImageStorageMode_RGB888 ) {
					memcpy(_getImageDataRow(currImgRow), getSPIFrameDataRow(0) + 2, LEPFLIR_SPI_FRAME_PACKET_SIZE - 4);
				}

				++currImgRow; currSpiRow = 0;
			}
		}
        _isReadingNextFrame = false;
    }
    return true;
}




void LeptonFLiR::Lepton_setup(LEPTON *lepton){

	  agc_setAGCEnabled(lepton->agc_en);
	  agc_setAGCPolicy(lepton->agc_policy);
	  sys_setTelemetryEnabled(lepton->telemetry);

	  if(lepton->format == LeptonFLiR_ImageStorageMode_RGB888){

		  oem_setVideoMode(LEP_VIDEO_OUTPUT_FORMAT_RGB888);
		  vid_setPseudoColorLUT(lepton->color);
		  lepton->W = 3*getImageWidth();
		  lepton->H = getImageHeight();

	  }else if(lepton->format == LeptonFLiR_ImageStorageMode_RAW14){

		  oem_setVideoMode(LEP_VIDEO_OUTPUT_FORMAT_RAW14);
		  lepton->W = getImageWidth();
		  lepton->H = getImageHeight();

	  }

	  sys_runFrameAveraging();						// to reduce noise us the built-in function of frame averaging
	  sys_setNumFramesToAverage(LEP_SYS_FA_DIV_16);	// specify the number of frames used for the averaging

	  LEP_AGC_HISTOGRAM_ROI newROI;
	  newROI.startCol = 10;
	  newROI.endCol   = 70;
	  newROI.startRow = 10;
	  newROI.endRow   = 50;

	  agc_setHistogramRegion(&newROI);				// define a new Region Of Interest for a better histogram

	  readInitData();
}








// ********************************************************************

// 			BEGINNING OF I2C FUNCIONS FOR CAMERA SET UP

// ********************************************************************


void LeptonFLiR::agc_setAGCEnabled(LEP_AGC_ENABLE_TAG state) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    msg_len = sprintf(msg, "\n\n\r LeptonFLiR::agc_setAGCEnabled");
    HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
#endif

    sendCommand(cmdCode(LEP_CID_AGC_ENABLE_STATE, LEP_I2C_COMMAND_TYPE_SET), (uint32_t)state);
}

bool LeptonFLiR::agc_getAGCEnabled() {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    msg_len = sprintf(msg, "\n\n\r LeptonFLiR::agc_getAGCEnabled");
    HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
#endif

    uint32_t state;
    receiveCommand(cmdCode(LEP_CID_AGC_ENABLE_STATE, LEP_I2C_COMMAND_TYPE_GET), &state);
    return (bool)state;
}

void LeptonFLiR::agc_setAGCPolicy(LEP_AGC_POLICY policy) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    msg_len = sprintf(msg, "\n\n\r LeptonFLiR::agc_setAGCPolicy");
	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
#endif

    sendCommand(cmdCode(LEP_CID_AGC_POLICY, LEP_I2C_COMMAND_TYPE_SET), (uint32_t)policy);
}

LEP_AGC_POLICY LeptonFLiR::agc_getAGCPolicy() {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    msg_len = sprintf(msg, "\n\n\r LeptonFLiR::agc_getAGCPolicy");
	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
#endif

    uint32_t policy;
    receiveCommand(cmdCode(LEP_CID_AGC_POLICY, LEP_I2C_COMMAND_TYPE_GET), &policy);
    return (LEP_AGC_POLICY)policy;
}

void LeptonFLiR::agc_setHEQScaleFactor(LEP_AGC_HEQ_SCALE_FACTOR factor) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    msg_len = sprintf(msg, "\n\n\r LeptonFLiR::agc_setHEQScaleFactor");
	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
#endif

    sendCommand(cmdCode(LEP_CID_AGC_HEQ_SCALE_FACTOR, LEP_I2C_COMMAND_TYPE_SET), (uint32_t)factor);
}

LEP_AGC_HEQ_SCALE_FACTOR LeptonFLiR::agc_getHEQScaleFactor() {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    msg_len = sprintf(msg, "\n\n\r LeptonFLiR::agc_getHEQScaleFactor");
	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
#endif

    uint32_t factor;
    receiveCommand(cmdCode(LEP_CID_AGC_HEQ_SCALE_FACTOR, LEP_I2C_COMMAND_TYPE_GET), &factor);
    return (LEP_AGC_HEQ_SCALE_FACTOR)factor;
}

void LeptonFLiR::agc_setAGCCalcEnabled(bool enabled) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    msg_len = sprintf(msg, "\n\n\r LeptonFLiR::agc_setAGCCalcEnabled");
	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
#endif

    sendCommand(cmdCode(LEP_CID_AGC_CALC_ENABLE_STATE, LEP_I2C_COMMAND_TYPE_SET), (uint32_t)enabled);
}

bool LeptonFLiR::agc_getAGCCalcEnabled() {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    msg_len = sprintf(msg, "\n\n\r LeptonFLiR::agc_getAGCCalcEnabled");
	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
#endif

    uint32_t enabled;
    receiveCommand(cmdCode(LEP_CID_AGC_CALC_ENABLE_STATE, LEP_I2C_COMMAND_TYPE_GET), &enabled);
    return enabled;
}

void LeptonFLiR::sys_getCameraStatus(LEP_SYS_CAM_STATUS *status) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    msg_len = sprintf(msg, "\n\n\r LeptonFLiR::sys_getCameraStatus");
	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
#endif

    receiveCommand(cmdCode(LEP_CID_SYS_CAM_STATUS, LEP_I2C_COMMAND_TYPE_GET), (uint16_t *)status, sizeof(LEP_SYS_CAM_STATUS) / 2);
}

LEP_SYS_CAM_STATUS_STATES LeptonFLiR::sys_getCameraStatus() {
    LEP_SYS_CAM_STATUS camStatus;
    sys_getCameraStatus(&camStatus);
    return (LEP_SYS_CAM_STATUS_STATES)camStatus.camStatus;
}

void LeptonFLiR::sys_getFlirSerialNumber(char *buffer, int maxLength) {
    if (!buffer || maxLength < 16) return;

#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    msg_len = sprintf(msg, "\n\n\r LeptonFLiR::sys_getFlirSerialNumber");
	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
#endif

    uint16_t innerBuffer[4];
    receiveCommand(cmdCode(LEP_CID_SYS_FLIR_SERIAL_NUMBER, LEP_I2C_COMMAND_TYPE_GET), innerBuffer, 4);
    wordsToHexString(innerBuffer, 4, buffer, maxLength);
}

void LeptonFLiR::sys_getCustomerSerialNumber(char *buffer, int maxLength) {
    if (!buffer || maxLength < 64) return;

#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    msg_len = sprintf(msg, "\n\n\r LeptonFLiR::sys_getCustomerSerialNumber");
	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
#endif

    uint16_t innerBuffer[16];
    receiveCommand(cmdCode(LEP_CID_SYS_CUST_SERIAL_NUMBER, LEP_I2C_COMMAND_TYPE_GET), innerBuffer, 16);
    wordsToHexString(innerBuffer, 16, buffer, maxLength);
}

uint32_t LeptonFLiR::sys_getCameraUptime() {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    msg_len = sprintf(msg, "\n\n\r LeptonFLiR::sys_getCameraUptime");
	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
#endif

    uint32_t uptime;
    receiveCommand(cmdCode(LEP_CID_SYS_CAM_UPTIME, LEP_I2C_COMMAND_TYPE_GET), &uptime);
    return uptime;
}

float LeptonFLiR::sys_getAuxTemperature() {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    msg_len = sprintf(msg, "\n\n\r LeptonFLiR::sys_getAuxTemperature");
	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
#endif

    uint16_t kelvin100;
    receiveCommand(cmdCode(LEP_CID_SYS_AUX_TEMPERATURE_KELVIN, LEP_I2C_COMMAND_TYPE_GET), &kelvin100);
    return kelvin100ToTemperature(kelvin100);
}

float LeptonFLiR::sys_getFPATemperature() {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    msg_len = sprintf(msg, "\n\n\r LeptonFLiR::sys_getFPATemperature");
	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
#endif

    uint16_t kelvin100;
    receiveCommand(cmdCode(LEP_CID_SYS_FPA_TEMPERATURE_KELVIN, LEP_I2C_COMMAND_TYPE_GET), &kelvin100);
    return kelvin100ToTemperature(kelvin100);
}

void LeptonFLiR::sys_setTelemetryEnabled(bool enabled) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    msg_len = sprintf(msg, "\n\n\rLeptonFLiR::sys_setTelemetryEnabled");
	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
#endif

    sendCommand(cmdCode(LEP_CID_SYS_TELEMETRY_ENABLE_STATE, LEP_I2C_COMMAND_TYPE_SET), (uint32_t)enabled);

    if (!_lastI2CError && !_lastLepResult) {
        if (enabled && !_telemetryData) {
            _telemetryData = (byte *)malloc(LEPFLIR_SPI_FRAME_PACKET_SIZE);

            if (_telemetryData)
                _telemetryData[0] = _telemetryData[1] = 0xFF; // initialize as discard packet
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
            if(!_telemetryData){
                msg_len = sprintf(msg, "\n\r  LeptonFLiR::sys_setTelemetryEnabled Failure allocating telemetryData.");
            	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
            }
#endif
        }
        else if (!enabled && _telemetryData) {
            free(_telemetryData);
            _telemetryData = NULL;
        }
    }
}

bool LeptonFLiR::sys_getTelemetryEnabled() {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    msg_len = sprintf(msg, "\n\n\r LeptonFLiR::sys_getTelemetryEnabled");
    HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
#endif

    uint32_t enabled;
    receiveCommand(cmdCode(LEP_CID_SYS_TELEMETRY_ENABLE_STATE, LEP_I2C_COMMAND_TYPE_GET), &enabled);

    if (!_lastI2CError && !_lastLepResult) {
        if (enabled && !_telemetryData) {
            _telemetryData = (byte *)malloc(LEPFLIR_SPI_FRAME_PACKET_SIZE);

            if (_telemetryData)
                _telemetryData[0] = _telemetryData[1] = 0xFF; // initialize as discard packet
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
            if(!_telemetryData){
                msg_len = sprintf(msg, "\n\r   LeptonFLiR::sys_getTelemetryEnabled Failure allocating telemetryData.");
                HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
            }
#endif
        }
        else if (!enabled && _telemetryData) {
            free(_telemetryData);
            _telemetryData = NULL;
        }
    }

    return enabled;
}


void LeptonFLiR::sys_runFFCNormalization() {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    msg_len = sprintf(msg, "\n\n\r LeptonFLiR::sys_runFFCNormalization");
	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
#endif

    sendCommand(cmdCode(LEP_CID_SYS_RUN_FFC, LEP_I2C_COMMAND_TYPE_RUN));
}

void LeptonFLiR::vid_setPolarity(LEP_VID_POLARITY polarity) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    msg_len = sprintf(msg, "\n\n\r LeptonFLiR::vid_setPolarity");
	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
#endif

    sendCommand(cmdCode(LEP_CID_VID_POLARITY_SELECT, LEP_I2C_COMMAND_TYPE_SET), (uint32_t)polarity);
}

LEP_VID_POLARITY LeptonFLiR::vid_getPolarity() {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    msg_len = sprintf(msg, "\n\n\r LeptonFLiR::vid_getPolarity");
	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
#endif

    uint32_t polarity;
    receiveCommand(cmdCode(LEP_CID_VID_POLARITY_SELECT, LEP_I2C_COMMAND_TYPE_GET), &polarity);
    return (LEP_VID_POLARITY)polarity;
}

void LeptonFLiR::vid_setPseudoColorLUT(LEP_VID_PCOLOR_LUT table) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    msg_len = sprintf(msg, "\n\n\r LeptonFLiR::vid_setPseudoColorLUT");
	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
#endif

    sendCommand(cmdCode(LEP_CID_VID_LUT_SELECT, LEP_I2C_COMMAND_TYPE_SET), (uint32_t)table);
}

LEP_VID_PCOLOR_LUT LeptonFLiR::vid_getPseudoColorLUT() {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    msg_len = sprintf(msg, "\n\n\r LeptonFLiR::vid_getPseudoColorLUT");
	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
#endif

    uint32_t table;
    receiveCommand(cmdCode(LEP_CID_VID_LUT_SELECT, LEP_I2C_COMMAND_TYPE_GET), &table);
    return (LEP_VID_PCOLOR_LUT)table;
}

void LeptonFLiR::vid_setFocusCalcEnabled(bool enabled) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    msg_len = sprintf(msg, "\n\n\r LeptonFLiR::vid_setFocusCalcEnabled");
	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
#endif

    sendCommand(cmdCode(LEP_CID_VID_FOCUS_CALC_ENABLE, LEP_I2C_COMMAND_TYPE_SET), (uint32_t)enabled);
}

bool LeptonFLiR::vid_getFocusCalcEnabled() {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    msg_len = sprintf(msg, "\n\n\r LeptonFLiR::vid_getFocusCalcEnabled");
	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
#endif

    uint32_t enabled;
    receiveCommand(cmdCode(LEP_CID_VID_FOCUS_CALC_ENABLE, LEP_I2C_COMMAND_TYPE_GET), &enabled);
    return enabled;
}

void LeptonFLiR::vid_setFreezeEnabled(bool enabled) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    msg_len = sprintf(msg, "\n\n\r LeptonFLiR::vid_setFreezeEnabled");
	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
#endif

    sendCommand(cmdCode(LEP_CID_VID_FREEZE_ENABLE, LEP_I2C_COMMAND_TYPE_SET), (uint32_t)enabled);
}

bool LeptonFLiR::vid_getFreezeEnabled() {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    msg_len = sprintf(msg, "\n\n\r LeptonFLiR::vid_getFreezeEnabled");
	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
#endif

    uint32_t enabled;
    receiveCommand(cmdCode(LEP_CID_VID_FREEZE_ENABLE, LEP_I2C_COMMAND_TYPE_GET), &enabled);
    return enabled;
}

#ifndef LEPFLIR_EXCLUDE_EXT_I2C_FUNCS

void LeptonFLiR::agc_setHistogramRegion(LEP_AGC_HISTOGRAM_ROI *region) {
    if (!region) return;

#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    msg_len = sprintf(msg, "\n\n\r LeptonFLiR::agc_setHistogramRegion");
	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
#endif

    sendCommand(cmdCode(LEP_CID_AGC_ROI, LEP_I2C_COMMAND_TYPE_SET), (uint16_t *)region, sizeof(LEP_AGC_HISTOGRAM_ROI) / 2);
}

void LeptonFLiR::agc_getHistogramRegion(LEP_AGC_HISTOGRAM_ROI *region) {
    if (!region) return;

#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    msg_len = sprintf(msg, "\n\n\r LeptonFLiR::agc_getHistogramRegion");
	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
#endif

    receiveCommand(cmdCode(LEP_CID_AGC_ROI, LEP_I2C_COMMAND_TYPE_GET), (uint16_t *)region, sizeof(LEP_AGC_HISTOGRAM_ROI) / 2);
}

void LeptonFLiR::agc_getHistogramStatistics(LEP_AGC_HISTOGRAM_STATISTICS *statistics) {
    if (!statistics) return;

#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    msg_len = sprintf(msg, "\n\n\r LeptonFLiR::agc_getHistogramStatistics");
	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
#endif

    receiveCommand(cmdCode(LEP_CID_AGC_STATISTICS, LEP_I2C_COMMAND_TYPE_GET), (uint16_t *)statistics, sizeof(LEP_AGC_HISTOGRAM_STATISTICS) / 2);
}

void LeptonFLiR::agc_setHistogramClipPercent(uint16_t percent) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    msg_len = sprintf(msg, "\n\n\r LeptonFLiR::agc_setHistogramClipPercent");
	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
#endif

    sendCommand(cmdCode(LEP_CID_AGC_HISTOGRAM_CLIP_PERCENT, LEP_I2C_COMMAND_TYPE_SET), percent);
}

uint16_t LeptonFLiR::agc_getHistogramClipPercent() {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    msg_len = sprintf(msg, "\n\n\r LeptonFLiR::agc_getHistogramClipPercent");
	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
#endif

    uint16_t percent;
    receiveCommand(cmdCode(LEP_CID_AGC_HISTOGRAM_CLIP_PERCENT, LEP_I2C_COMMAND_TYPE_GET), &percent);
    return percent;
}

void LeptonFLiR::agc_setHistogramTailSize(uint16_t size) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    msg_len = sprintf(msg, "\n\n\r LeptonFLiR::agc_setHistogramTailSize");
	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
#endif

    sendCommand(cmdCode(LEP_CID_AGC_HISTOGRAM_TAIL_SIZE, LEP_I2C_COMMAND_TYPE_SET), size);
}

uint16_t LeptonFLiR::agc_getHistogramTailSize() {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    msg_len = sprintf(msg, "\n\n\r LeptonFLiR::agc_getHistogramTailSize");
	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
#endif

    uint16_t size;
    receiveCommand(cmdCode(LEP_CID_AGC_HISTOGRAM_TAIL_SIZE, LEP_I2C_COMMAND_TYPE_GET), &size);
    return size;
}

void LeptonFLiR::agc_setLinearMaxGain(uint16_t gain) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    msg_len = sprintf(msg, "\n\n\r LeptonFLiR::agc_setLinearMaxGain");
	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
#endif

    sendCommand(cmdCode(LEP_CID_AGC_LINEAR_MAX_GAIN, LEP_I2C_COMMAND_TYPE_SET), gain);
}

uint16_t LeptonFLiR::agc_getLinearMaxGain() {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    msg_len = sprintf(msg, "\n\n\r LeptonFLiR::agc_getLinearMaxGain");
	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
#endif

    uint16_t gain;
    receiveCommand(cmdCode(LEP_CID_AGC_LINEAR_MAX_GAIN, LEP_I2C_COMMAND_TYPE_GET), &gain);
    return gain;
}

void LeptonFLiR::agc_setLinearMidpoint(uint16_t midpoint) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    msg_len = sprintf(msg, "\n\n\r LeptonFLiR::agc_setLinearMidpoint");
	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
#endif

    sendCommand(cmdCode(LEP_CID_AGC_LINEAR_MIDPOINT, LEP_I2C_COMMAND_TYPE_SET), midpoint);
}

uint16_t LeptonFLiR::agc_getLinearMidpoint() {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    msg_len = sprintf(msg, "\n\n\r LeptonFLiR::agc_getLinearMidpoint");
	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
#endif

    uint16_t midpoint;
    receiveCommand(cmdCode(LEP_CID_AGC_LINEAR_MIDPOINT, LEP_I2C_COMMAND_TYPE_GET), &midpoint);
    return midpoint;
}

void LeptonFLiR::agc_setLinearDampeningFactor(uint16_t factor) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    msg_len = sprintf(msg, "\n\n\r LeptonFLiR::agc_setLinearDampeningFactor");
	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
#endif

    sendCommand(cmdCode(LEP_CID_AGC_LINEAR_DAMPENING_FACTOR, LEP_I2C_COMMAND_TYPE_SET), factor);
}

uint16_t LeptonFLiR::agc_getLinearDampeningFactor() {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    msg_len = sprintf(msg, "\n\n\r LeptonFLiR::agc_getLinearDampeningFactor");
	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
#endif

    uint16_t factor;
    receiveCommand(cmdCode(LEP_CID_AGC_LINEAR_DAMPENING_FACTOR, LEP_I2C_COMMAND_TYPE_GET), &factor);
    return factor;
}

void LeptonFLiR::agc_setHEQDampeningFactor(uint16_t factor) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    msg_len = sprintf(msg, "\n\n\r LeptonFLiR::agc_setHEQDampeningFactor");
	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
#endif

    sendCommand(cmdCode(LEP_CID_AGC_HEQ_DAMPENING_FACTOR, LEP_I2C_COMMAND_TYPE_SET), factor);
}

uint16_t LeptonFLiR::agc_getHEQDampeningFactor() {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    msg_len = sprintf(msg, "\n\n\r LeptonFLiR::agc_getHEQDampeningFactor");
	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
#endif

    uint16_t factor;
    receiveCommand(cmdCode(LEP_CID_AGC_HEQ_DAMPENING_FACTOR, LEP_I2C_COMMAND_TYPE_GET), &factor);
    return factor;
}

void LeptonFLiR::agc_setHEQMaxGain(uint16_t gain) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    msg_len = sprintf(msg, "\n\n\r LeptonFLiR::agc_setHEQMaxGain");
	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
#endif

    sendCommand(cmdCode(LEP_CID_AGC_HEQ_MAX_GAIN, LEP_I2C_COMMAND_TYPE_SET), gain);
}

uint16_t LeptonFLiR::agc_getHEQMaxGain() {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    msg_len = sprintf(msg, "\n\n\r LeptonFLiR::agc_getHEQMaxGain");
	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
#endif

    uint16_t gain;
    receiveCommand(cmdCode(LEP_CID_AGC_HEQ_MAX_GAIN, LEP_I2C_COMMAND_TYPE_GET), &gain);
    return gain;
}

void LeptonFLiR::agc_setHEQClipLimitHigh(uint16_t limit) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    msg_len = sprintf(msg, "\n\n\r LeptonFLiR::agc_setHEQClipLimitHigh");
	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
#endif

    sendCommand(cmdCode(LEP_CID_AGC_HEQ_CLIP_LIMIT_HIGH, LEP_I2C_COMMAND_TYPE_SET), limit);
}

uint16_t LeptonFLiR::agc_getHEQClipLimitHigh() {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    msg_len = sprintf(msg, "\n\n\r LeptonFLiR::agc_getHEQClipLimitHigh");
	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
#endif

    uint16_t limit;
    receiveCommand(cmdCode(LEP_CID_AGC_HEQ_CLIP_LIMIT_HIGH, LEP_I2C_COMMAND_TYPE_GET), &limit);
    return limit;
}

void LeptonFLiR::agc_setHEQClipLimitLow(uint16_t limit) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    msg_len = sprintf(msg, "\n\n\r LeptonFLiR::agc_setHEQClipLimitLow");
	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
#endif

    sendCommand(cmdCode(LEP_CID_AGC_HEQ_CLIP_LIMIT_LOW, LEP_I2C_COMMAND_TYPE_SET), limit);
}

uint16_t LeptonFLiR::agc_getHEQClipLimitLow() {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    msg_len = sprintf(msg, "\n\n\r LeptonFLiR::agc_getHEQClipLimitLow");
	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
#endif

    uint16_t limit;
    receiveCommand(cmdCode(LEP_CID_AGC_HEQ_CLIP_LIMIT_LOW, LEP_I2C_COMMAND_TYPE_GET), &limit);
    return limit;
}

void LeptonFLiR::agc_setHEQBinExtension(uint16_t extension) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    msg_len = sprintf(msg, "\n\n\r LeptonFLiR::agc_setHEQBinExtension");
	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
#endif

    sendCommand(cmdCode(LEP_CID_AGC_HEQ_BIN_EXTENSION, LEP_I2C_COMMAND_TYPE_SET), extension);
}

uint16_t LeptonFLiR::agc_getHEQBinExtension() {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    msg_len = sprintf(msg, "\n\n\r LeptonFLiR::agc_getHEQBinExtension");
	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
#endif

    uint16_t extension;
    receiveCommand(cmdCode(LEP_CID_AGC_HEQ_BIN_EXTENSION, LEP_I2C_COMMAND_TYPE_GET), &extension);
    return extension;
}

void LeptonFLiR::agc_setHEQMidpoint(uint16_t midpoint) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    msg_len = sprintf(msg, "\n\n\r LeptonFLiR::agc_setHEQMidpoint");
	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
#endif

    sendCommand(cmdCode(LEP_CID_AGC_HEQ_MIDPOINT, LEP_I2C_COMMAND_TYPE_SET), midpoint);
}

uint16_t LeptonFLiR::agc_getHEQMidpoint() {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    msg_len = sprintf(msg, "\n\n\r LeptonFLiR::agc_getHEQMidpoint");
	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
#endif

    uint16_t midpoint;
    receiveCommand(cmdCode(LEP_CID_AGC_HEQ_MIDPOINT, LEP_I2C_COMMAND_TYPE_GET), &midpoint);
    return midpoint;
}

void LeptonFLiR::agc_setHEQEmptyCounts(uint16_t counts) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    msg_len = sprintf(msg, "\n\n\r LeptonFLiR::agc_setHEQEmptyCounts");
	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
#endif

    sendCommand(cmdCode(LEP_CID_AGC_HEQ_EMPTY_COUNTS, LEP_I2C_COMMAND_TYPE_SET), counts);
}

uint16_t LeptonFLiR::agc_getHEQEmptyCounts() {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    msg_len = sprintf(msg, "\n\n\r LeptonFLiR::agc_setHEQEmptyCounts");
	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
#endif

    uint16_t counts;
    receiveCommand(cmdCode(LEP_CID_AGC_HEQ_EMPTY_COUNTS, LEP_I2C_COMMAND_TYPE_GET), &counts);
    return counts;
}

void LeptonFLiR::agc_setHEQNormalizationFactor(uint16_t factor) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    msg_len = sprintf(msg, "\n\n\r LeptonFLiR::agc_setHEQNormalizationFactor");
	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
#endif

    sendCommand(cmdCode(LEP_CID_AGC_HEQ_NORMALIZATION_FACTOR, LEP_I2C_COMMAND_TYPE_SET), factor);
}

uint16_t LeptonFLiR::agc_getHEQNormalizationFactor() {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    msg_len = sprintf(msg, "\n\n\r LeptonFLiR::agc_getHEQNormalizationFactor");
	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
#endif

    uint16_t factor;
    receiveCommand(cmdCode(LEP_CID_AGC_HEQ_NORMALIZATION_FACTOR, LEP_I2C_COMMAND_TYPE_GET), &factor);
    return factor;
}

void LeptonFLiR::sys_runPingCamera() {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    msg_len = sprintf(msg, "\n\r LeptonFLiR::sys_runPingCamera");
	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
#endif

    sendCommand(cmdCode(LEP_CID_SYS_PING, LEP_I2C_COMMAND_TYPE_RUN));
}

void LeptonFLiR::sys_setTelemetryLocation(LEP_SYS_TELEMETRY_LOCATION location) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    msg_len = sprintf(msg, "\n\n\r LeptonFLiR::sys_setTelemetryLocation");
	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
#endif

    sendCommand(cmdCode(LEP_CID_SYS_TELEMETRY_LOCATION, LEP_I2C_COMMAND_TYPE_SET), (uint32_t)location);
}

LEP_SYS_TELEMETRY_LOCATION LeptonFLiR::sys_getTelemetryLocation() {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    msg_len = sprintf(msg, "\n\n\r LeptonFLiR::sys_getTelemetryLocation");
	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
#endif

    uint32_t location;
    receiveCommand(cmdCode(LEP_CID_SYS_TELEMETRY_LOCATION, LEP_I2C_COMMAND_TYPE_GET), &location);
    return (LEP_SYS_TELEMETRY_LOCATION)location;
}

void LeptonFLiR::sys_runFrameAveraging() {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    msg_len = sprintf(msg, "\n\r LeptonFLiR::sys_runFrameAveraging");
	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
#endif

    sendCommand(cmdCode(LEP_CID_SYS_EXECTUE_FRAME_AVERAGE, LEP_I2C_COMMAND_TYPE_RUN));
}

void LeptonFLiR::sys_setNumFramesToAverage(LEP_SYS_FRAME_AVERAGE average) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    msg_len = sprintf(msg, "\n\r LeptonFLiR::sys_setNumFramesToAverage");
	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
#endif

    sendCommand(cmdCode(LEP_CID_SYS_NUM_FRAMES_TO_AVERAGE, LEP_I2C_COMMAND_TYPE_SET), (uint32_t)average);
}

LEP_SYS_FRAME_AVERAGE LeptonFLiR::sys_getNumFramesToAverage() {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    msg_len = sprintf(msg, "\n\r LeptonFLiR::sys_getNumFramesToAverage");
	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
#endif

    uint32_t average;
    receiveCommand(cmdCode(LEP_CID_SYS_NUM_FRAMES_TO_AVERAGE, LEP_I2C_COMMAND_TYPE_GET), &average);
    return (LEP_SYS_FRAME_AVERAGE)average;
}

void LeptonFLiR::sys_getSceneStatistics(LEP_SYS_SCENE_STATISTICS *statistics) {
    if (!statistics) return;

#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    msg_len = sprintf(msg, "\n\r LeptonFLiR::sys_getSceneStatistics");
	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
#endif

    receiveCommand(cmdCode(LEP_CID_SYS_SCENE_STATISTICS, LEP_I2C_COMMAND_TYPE_GET), (uint16_t *)statistics, sizeof(LEP_SYS_SCENE_STATISTICS) / 2);
}

void LeptonFLiR::sys_setSceneRegion(LEP_SYS_SCENE_ROI *region) {
    if (!region) return;

#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    msg_len = sprintf(msg, "\n\r LeptonFLiR::sys_setSceneRegion");
	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
#endif

    sendCommand(cmdCode(LEP_CID_SYS_SCENE_ROI, LEP_I2C_COMMAND_TYPE_SET), (uint16_t *)region, sizeof(LEP_SYS_SCENE_ROI) / 2);
}

void LeptonFLiR::sys_getSceneRegion(LEP_SYS_SCENE_ROI *region) {
    if (!region) return;

#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    msg_len = sprintf(msg, "\n\r LeptonFLiR::sys_getSceneRegion");
	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
#endif

    receiveCommand(cmdCode(LEP_CID_SYS_SCENE_ROI, LEP_I2C_COMMAND_TYPE_GET), (uint16_t *)region, sizeof(LEP_SYS_SCENE_ROI) / 2);
}

uint16_t LeptonFLiR::sys_getThermalShutdownCount() {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    msg_len = sprintf(msg, "\n\r LeptonFLiR::sys_getThermalShutdownCount");
	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
#endif

    uint16_t count;
    receiveCommand(cmdCode(LEP_CID_SYS_THERMAL_SHUTDOWN_COUNT, LEP_I2C_COMMAND_TYPE_GET), &count);
    return count;
}

void LeptonFLiR::sys_setShutterPosition(LEP_SYS_SHUTTER_POSITION position) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    msg_len = sprintf(msg, "\n\r LeptonFLiR::sys_setShutterPosition");
	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
#endif

    sendCommand(cmdCode(LEP_CID_SYS_SHUTTER_POSITION, LEP_I2C_COMMAND_TYPE_SET), (uint32_t)position);
}

LEP_SYS_SHUTTER_POSITION LeptonFLiR::sys_getShutterPosition() {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    msg_len = sprintf(msg, "\n\r LeptonFLiR::sys_getShutterPosition");
	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
#endif

    uint32_t position;
    receiveCommand(cmdCode(LEP_CID_SYS_SHUTTER_POSITION, LEP_I2C_COMMAND_TYPE_GET), &position);
    return (LEP_SYS_SHUTTER_POSITION)position;
}

void LeptonFLiR::sys_setFFCShutterMode(LEP_SYS_FFC_SHUTTER_MODE *mode) {
    if (!mode) return;

#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    msg_len = sprintf(msg, "\n\r LeptonFLiR::sys_setFFCShutterMode");
	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
#endif

    sendCommand(cmdCode(LEP_CID_SYS_FFC_SHUTTER_MODE, LEP_I2C_COMMAND_TYPE_SET), (uint16_t *)mode, sizeof(LEP_SYS_FFC_SHUTTER_MODE) / 2);
}

void LeptonFLiR::sys_getFFCShutterMode(LEP_SYS_FFC_SHUTTER_MODE *mode) {
    if (!mode) return;

#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    msg_len = sprintf(msg, "\n\r LeptonFLiR::sys_getFFCShutterMode");
	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
#endif

    receiveCommand(cmdCode(LEP_CID_SYS_FFC_SHUTTER_MODE, LEP_I2C_COMMAND_TYPE_GET), (uint16_t *)mode, sizeof(LEP_SYS_FFC_SHUTTER_MODE) / 2);
}

LEP_SYS_FFC_STATUS LeptonFLiR::sys_getFFCNormalizationStatus() {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    msg_len = sprintf(msg, "\n\r LeptonFLiR::sys_getFFCNormalizationStatus");
	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
#endif

    uint32_t status;
    receiveCommand(cmdCode(LEP_CID_SYS_FFC_STATUS, LEP_I2C_COMMAND_TYPE_GET), &status);
    return (LEP_SYS_FFC_STATUS)status;
}

void LeptonFLiR::vid_setUserColorLUT(LEP_VID_LUT_BUFFER *table) {
    if (!table) return;

#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    msg_len = sprintf(msg, "\n\r LeptonFLiR::vid_setUserColorLUT");
	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
#endif

    sendCommand(cmdCode(LEP_CID_VID_LUT_TRANSFER, LEP_I2C_COMMAND_TYPE_SET), (uint16_t *)table, sizeof(LEP_VID_LUT_BUFFER) / 2);
}

void LeptonFLiR::vid_getUserColorLUT(LEP_VID_LUT_BUFFER *table) {
    if (!table) return;

#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    msg_len = sprintf(msg, "\n\r LeptonFLiR::vid_getUserColorLUT");
	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
#endif

    receiveCommand(cmdCode(LEP_CID_VID_LUT_TRANSFER, LEP_I2C_COMMAND_TYPE_GET), (uint16_t *)table, sizeof(LEP_VID_LUT_BUFFER) / 2);
}

void LeptonFLiR::vid_setFocusRegion(LEP_VID_FOCUS_ROI *region) {
    if (!region) return;

#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    msg_len = sprintf(msg, "\n\r LeptonFLiR::vid_setFocusRegion");
	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
#endif

    sendCommand(cmdCode(LEP_CID_VID_FOCUS_ROI, LEP_I2C_COMMAND_TYPE_SET), (uint16_t *)region, sizeof(LEP_VID_FOCUS_ROI) / 2);
}

void LeptonFLiR::vid_getFocusRegion(LEP_VID_FOCUS_ROI *region) {
    if (!region) return;

#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    msg_len = sprintf(msg, "\n\r LeptonFLiR::vid_getFocusRegion");
	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
#endif

    receiveCommand(cmdCode(LEP_CID_VID_FOCUS_ROI, LEP_I2C_COMMAND_TYPE_GET), (uint16_t *)region, sizeof(LEP_VID_FOCUS_ROI) / 2);
}

void LeptonFLiR::vid_setFocusThreshold(uint32_t threshold) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    msg_len = sprintf(msg, "\n\r LeptonFLiR::vid_setFocusThreshold");
	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
#endif

    sendCommand(cmdCode(LEP_CID_VID_FOCUS_THRESHOLD, LEP_I2C_COMMAND_TYPE_SET), threshold);
}

uint32_t LeptonFLiR::vid_getFocusThreshold() {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    msg_len = sprintf(msg, "\n\r LeptonFLiR::vid_getFocusThreshold");
	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
#endif

    uint32_t threshold;
    receiveCommand(cmdCode(LEP_CID_VID_FOCUS_THRESHOLD, LEP_I2C_COMMAND_TYPE_GET), &threshold);
    return threshold;
}

uint32_t LeptonFLiR::vid_getFocusMetric() {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    msg_len = sprintf(msg, "\n\r LeptonFLiR::vid_getFocusMetric");
	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
#endif

    uint32_t metric;
    receiveCommand(cmdCode(LEP_CID_VID_FOCUS_METRIC, LEP_I2C_COMMAND_TYPE_GET), &metric);
    return metric;
}

void LeptonFLiR::vid_setSceneBasedNUCEnabled(bool enabled) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    msg_len = sprintf(msg, "\n\r LeptonFLiR::vid_setSceneBasedNUCEnabled");
	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
#endif

    sendCommand(cmdCode(LEP_CID_VID_SBNUC_ENABLE, LEP_I2C_COMMAND_TYPE_SET), (uint32_t)enabled);
}

bool LeptonFLiR::vid_getSceneBasedNUCEnabled() {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    msg_len = sprintf(msg, "\n\r LeptonFLiR::vid_getSceneBasedNUCEnabled");
	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
#endif

    uint32_t enabled;
    receiveCommand(cmdCode(LEP_CID_VID_SBNUC_ENABLE, LEP_I2C_COMMAND_TYPE_GET), &enabled);
    return enabled;
}

void LeptonFLiR::vid_setGamma(uint32_t gamma) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    msg_len = sprintf(msg, "\n\r LeptonFLiR::vid_setGamma");
	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
#endif

    sendCommand(cmdCode(LEP_CID_VID_GAMMA_SELECT, LEP_I2C_COMMAND_TYPE_SET), gamma);
}

uint32_t LeptonFLiR::vid_getGamma() {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    msg_len = sprintf(msg, "\n\r LeptonFLiR::vid_getGamma");
	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
#endif

    uint32_t gamma;
    receiveCommand(cmdCode(LEP_CID_VID_GAMMA_SELECT, LEP_I2C_COMMAND_TYPE_GET), &gamma);
    return gamma;
}


void LeptonFLiR::oem_setVideoMode(LEP_OEM_VIDEO_OUTPUT_FORMAT format) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    msg_len = sprintf(msg, "\n\n\r LeptonFLiR::oem_setVideoMode");
	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
#endif

    sendCommand(cmdCodeOEM(LEP_CID_OEM_VIDEO_OUTPUT_FORMAT_SELECT, LEP_I2C_COMMAND_TYPE_SET), (uint32_t)format);

    if (!_lastI2CError && !_lastLepResult) {

    }
}

LEP_OEM_VIDEO_OUTPUT_FORMAT LeptonFLiR::oem_getVideoMode() {

    uint32_t format;
    receiveCommand(cmdCodeOEM(LEP_CID_OEM_VIDEO_OUTPUT_FORMAT_SELECT, LEP_I2C_COMMAND_TYPE_GET), &format);

#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    if(format == 7){
        msg_len = sprintf(msg, "\n\n\r LeptonFLiR::oem_getVideoMode -> Raw14");

    }else if(format == 3){
        msg_len = sprintf(msg, "\n\n\r LeptonFLiR::oem_getVideoMode -> RG888");

    }
	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
#endif

    return (LEP_OEM_VIDEO_OUTPUT_FORMAT)format;
}


void LeptonFLiR::oem_setGPIO_mode(LEP_OEM_GPIO_MODE mode) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    msg_len = sprintf(msg, "\n\n\r LeptonFLiR::oem_setGPIO_mode");
	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
#endif

    sendCommand(cmdCodeOEM(LEP_CID_OEM_GPIO_MODE_SELECT, LEP_I2C_COMMAND_TYPE_SET), (uint32_t)mode);

    if (!_lastI2CError && !_lastLepResult) {

    }
}

LEP_OEM_GPIO_MODE LeptonFLiR::oem_getGPIO_mode() {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    msg_len = sprintf(msg, "\n\n\r LeptonFLiR::oem_getGPIO_mode");
	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
#endif

    uint32_t mode;
    receiveCommand(cmdCodeOEM(LEP_CID_OEM_GPIO_MODE_SELECT, LEP_I2C_COMMAND_TYPE_GET), &mode);

#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT

    switch (mode){
		case 0:
	        msg_len = sprintf(msg, "\n\n\r LeptonFLiR::oem_getGPIO_mode -> MODE_GPIO");
		break;
		case 1:
	        msg_len = sprintf(msg, "\n\n\r LeptonFLiR::oem_getGPIO_mode -> MODE_I2C_MASTER");
		break;
		case 2:
	        msg_len = sprintf(msg, "\n\n\r LeptonFLiR::oem_getGPIO_mode -> SPI_MASTER_VLB_DATA");
		break;
		case 3:
	        msg_len = sprintf(msg, "\n\n\r LeptonFLiR::oem_getGPIO_mode -> MODE_SPIO_MASTER_REG_DATA");
		break;
		case 4:
	        msg_len = sprintf(msg, "\n\n\r LeptonFLiR::oem_getGPIO_mode -> MODE_SPI_SLAVE_VLB_DATA");
		break;
		case 5:
	        msg_len = sprintf(msg, "\n\n\r LeptonFLiR::oem_getGPIO_mode -> MODE_VSYNC");
		break;
		default:
		break;
    }
	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
#endif

    return (LEP_OEM_GPIO_MODE)mode;
}

void LeptonFLiR::oem_setVSYNCPhaseDelay(LEP_OEM_VSYNC_DELAY delay) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    msg_len = sprintf(msg, "\n\n\r LeptonFLiR::oem_setVSYNCPhaseDelay");
	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
#endif

    sendCommand(cmdCodeOEM(LEP_CID_OEM_GPIO_VSYNC_PHASE_DELAY, LEP_I2C_COMMAND_TYPE_SET), (uint32_t)delay);

    if (!_lastI2CError && !_lastLepResult) {

    }
}

LEP_OEM_VSYNC_DELAY LeptonFLiR::oem_getVSYNCPhaseDelay() {

    uint32_t delay;
    receiveCommand(cmdCodeOEM(LEP_CID_OEM_GPIO_VSYNC_PHASE_DELAY, LEP_I2C_COMMAND_TYPE_GET), &delay);

    return (LEP_OEM_VSYNC_DELAY)delay;
}

void LeptonFLiR::oem_ReBootCamera() {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    msg_len = sprintf(msg, "\n\n\r LeptonFLiR::oem_ReBootCamera");
	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
#endif

    sendCommand(cmdCodeOEM(LEP_CID_OEM_RUN_CAMERA_REBOOT, LEP_I2C_COMMAND_TYPE_RUN));

    if (!_lastI2CError && !_lastLepResult) {

    }
}


void LeptonFLiR::oem_setColumnNoiseEstimateControl(LEP_OEM_SATE state) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    msg_len = sprintf(msg, "\n\n\r LeptonFLiR::LEP_SetOemColumnNoiseEstimateControl");
	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
#endif

    sendCommand(cmdCodeOEM(LEP_CID_OEM_COLUMN_NOISE_FILTER, LEP_I2C_COMMAND_TYPE_SET), (uint32_t)state);

    if (!_lastI2CError && !_lastLepResult) {

    }
}


void LeptonFLiR::oem_setPixelNoiseEstimateControl(LEP_OEM_SATE state) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    msg_len = sprintf(msg, "\n\n\r LeptonFLiR::oem_setPixelNoiseEstimateControl");
	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
#endif

    sendCommand(cmdCodeOEM(LEP_CID_OEM_PIXEL_NOISE_FILTER, LEP_I2C_COMMAND_TYPE_SET), (uint32_t)state);

    if (!_lastI2CError && !_lastLepResult) {

    }
}


void LeptonFLiR::rad_setRadiometryControlEnable(LEP_RAD_ENABLE_E_TAG mode) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    msg_len = sprintf(msg, "\n\n\r LeptonFLiR::rad_setRadiometryControlEnable");
	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
#endif

    sendCommand(cmdCodeOEM(LEP_CID_RAD_RADIOMETRY_CONTROL_ENABLE, LEP_I2C_COMMAND_TYPE_SET), (uint32_t)mode);

    if (!_lastI2CError && !_lastLepResult) {

    }
}

LEP_RAD_ENABLE_E_TAG LeptonFLiR::rad_getRadiometryControlEnable() {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    msg_len = sprintf(msg, "\n\n\r LeptonFLiR::rad_getRadiometryControlEnable");
	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
#endif

    uint32_t mode;
    receiveCommand(cmdCodeOEM(LEP_CID_RAD_RADIOMETRY_CONTROL_ENABLE, LEP_I2C_COMMAND_TYPE_GET), &mode);

    return (LEP_RAD_ENABLE_E_TAG)mode;
}

void LeptonFLiR::rad_setTLinearEnableState(LEP_RAD_ENABLE_E_TAG mode) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    msg_len = sprintf(msg, "\n\n\r LeptonFLiR::rad_setTLinearEnableState");
	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
#endif

    sendCommand(cmdCodeOEM(LEP_CID_RAD_T_LINEAR_ENABLE_STATE, LEP_I2C_COMMAND_TYPE_SET), (uint32_t)mode);

    if (!_lastI2CError && !_lastLepResult) {

    }
}

LEP_RAD_ENABLE_E_TAG LeptonFLiR::rad_getTLinearEnableState() {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    msg_len = sprintf(msg, "\n\n\r LeptonFLiR::rad_getTLinearEnableState");
	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
#endif

    uint32_t mode;
    receiveCommand(cmdCodeOEM(LEP_CID_RAD_T_LINEAR_ENABLE_STATE, LEP_I2C_COMMAND_TYPE_GET), &mode);

    return (LEP_RAD_ENABLE_E_TAG)mode;
}

#endif






static inline void byteToHexString(byte value, char *buffer) {
    byte highNibble = value / 16;
    byte lowNibble = value % 16;
    if (highNibble < 10) buffer[0] = '0' + highNibble;
    else buffer[0] = 'A' + (highNibble - 10);
    if (lowNibble < 10) buffer[1] = '0' + lowNibble;
    else buffer[1] = 'A' + (lowNibble - 10);
}

void wordsToHexString(uint16_t *dataWords, int dataLength, char *buffer, int maxLength) {
    bool insertColons = maxLength >= (dataLength * 4) + (dataLength - 1);

    while (dataLength-- > 0 && maxLength > 3) {
        if (maxLength > 3) {
        	byte high_byte = *(dataWords)>>8;
            byteToHexString(high_byte, buffer);
            buffer += 2; maxLength -= 2;
            byte low_byte = *(dataWords);
            byteToHexString(low_byte, buffer);
            buffer += 2; maxLength -= 2;
            ++dataWords;
        }

        if (dataLength > 0 && insertColons && maxLength-- > 0)
            *buffer++ = ':';
    }

    if (maxLength-- > 0)
        *buffer++ = '\0';
}



float kelvin100ToCelsius(uint16_t kelvin100) {
    float kelvin = (kelvin100 / 100) + ((kelvin100 % 100) * 0.01f);
    return kelvin - 273.15f;
}

float kelvin100ToFahrenheit(uint16_t kelvin100) {
    float kelvin = (kelvin100 / 100) + ((kelvin100 % 100) * 0.01f);
    return roundf((((kelvin * 9.0f) / 5.0f) - 459.67f) * 100.0f) / 100.0f;
}

float kelvin100ToKelvin(uint16_t kelvin100) {
    return (kelvin100 / 100) + ((kelvin100 % 100) * 0.01f);
}

uint16_t celsiusToKelvin100(float celsius) {
    float kelvin = celsius + 273.15f;
    return (uint16_t)roundf(kelvin * 100.0f);
}

uint16_t fahrenheitToKelvin100(float fahrenheit) {
    float kelvin = ((fahrenheit + 459.67f) * 5.0f) / 9.0f;
    return (uint16_t)roundf(kelvin * 100.0f);
}

uint16_t kelvinToKelvin100(float kelvin) {
    return (uint16_t)roundf(kelvin * 100.0f);
}

float LeptonFLiR::kelvin100ToTemperature(uint16_t kelvin100) {
    switch (_tempMode) {
        case LeptonFLiR_TemperatureMode_Celsius:
            return kelvin100ToCelsius(kelvin100);
        case LeptonFLiR_TemperatureMode_Fahrenheit:
            return kelvin100ToFahrenheit(kelvin100);
        case LeptonFLiR_TemperatureMode_Kelvin:
            return kelvin100ToKelvin(kelvin100);
        default:
            return 0;
    }
}

uint16_t LeptonFLiR::temperatureToKelvin100(float temperature) {
    switch (_tempMode) {
        case LeptonFLiR_TemperatureMode_Celsius:
            return celsiusToKelvin100(temperature);
        case LeptonFLiR_TemperatureMode_Fahrenheit:
            return fahrenheitToKelvin100(temperature);
        case LeptonFLiR_TemperatureMode_Kelvin:
            return kelvinToKelvin100(temperature);
        default:
            return 0;
    }
}

const char *LeptonFLiR::getTemperatureSymbol() {
    switch (_tempMode) {
        case LeptonFLiR_TemperatureMode_Celsius:
            msg_len = sprintf(msg, "C");
        	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
            return "C";
		break;
        case LeptonFLiR_TemperatureMode_Fahrenheit:
            msg_len = sprintf(msg, "F");
        	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
            return "F";
		break;
        case LeptonFLiR_TemperatureMode_Kelvin:
            msg_len = sprintf(msg, "K");
        	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
            return "K";
		break;
        default:
            msg_len = sprintf(msg, "ERR");
        	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
            return "";
		break;

    }
}

byte LeptonFLiR::getLastI2CError() {
    return _lastI2CError;
}

LEP_RESULT LeptonFLiR::getLastLepResult() {
    return (LEP_RESULT)_lastLepResult;
}

#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT

static const char *textForI2CError(byte errorCode) {
    switch (errorCode) {
        case 0:
            return "Success";
        case 1:
            return "Data too long to fit in transmit buffer";
        case 2:
            return "Received NACK on transmit of address";
        case 3:
            return "Received NACK on transmit of data";
        default:
            return "Other error I2C";
    }
}

static const char *textForLepResult(LEP_RESULT errorCode) {
    switch (errorCode) {
        case LEP_OK:
            return "LEP_OK Camera ok";
        case LEP_ERROR:
            return "LEP_ERROR Camera general error";
        case LEP_NOT_READY:
            return "LEP_NOT_READY Camera not ready error";
        case LEP_RANGE_ERROR:
            return "LEP_RANGE_ERROR Camera range error";
        case LEP_CHECKSUM_ERROR:
            return "LEP_CHECKSUM_ERROR Camera checksum error";
        case LEP_BAD_ARG_POINTER_ERROR:
            return "LEP_BAD_ARG_POINTER_ERROR Camera Bad argument  error";
        case LEP_DATA_SIZE_ERROR:
            return "LEP_DATA_SIZE_ERROR Camera byte count error";
        case LEP_UNDEFINED_FUNCTION_ERROR:
            return "LEP_UNDEFINED_FUNCTION_ERROR Camera undefined function error";
        case LEP_FUNCTION_NOT_SUPPORTED:
            return "LEP_FUNCTION_NOT_SUPPORTED Camera function not yet supported error";
        case LEP_OTP_WRITE_ERROR:
            return "LEP_OTP_WRITE_ERROR Camera OTP write error";
        case LEP_OTP_READ_ERROR:
            return "LEP_OTP_READ_ERROR Double bit error detected (uncorrectible)";
        case LEP_OTP_NOT_PROGRAMMED_ERROR:
            return "LEP_OTP_NOT_PROGRAMMED_ERROR Flag read as non-zero";
        case LEP_ERROR_I2C_BUS_NOT_READY:
            return "LEP_ERROR_I2C_BUS_NOT_READY I2C Bus Error - Bus Not Avaialble";
        case LEP_ERROR_I2C_BUFFER_OVERFLOW:
            return "LEP_ERROR_I2C_BUFFER_OVERFLOW I2C Bus Error - Buffer Overflow";
        case LEP_ERROR_I2C_ARBITRATION_LOST:
            return "LEP_ERROR_I2C_ARBITRATION_LOST I2C Bus Error - Bus Arbitration Lost";
        case LEP_ERROR_I2C_BUS_ERROR:
            return "LEP_ERROR_I2C_BUS_ERROR I2C Bus Error - General Bus Error";
        case LEP_ERROR_I2C_NACK_RECEIVED:
            return "LEP_ERROR_I2C_NACK_RECEIVED I2C Bus Error - NACK Received";
        case LEP_ERROR_I2C_FAIL:
            return "LEP_ERROR_I2C_FAIL I2C Bus Error - General Failure";
        case LEP_DIV_ZERO_ERROR:
            return "LEP_DIV_ZERO_ERROR Attempted div by zero";
        case LEP_COMM_PORT_NOT_OPEN:
            return "LEP_COMM_PORT_NOT_OPEN Comm port not open";
        case LEP_COMM_INVALID_PORT_ERROR:
            return "LEP_COMM_INVALID_PORT_ERROR Comm port no such port error";
        case LEP_COMM_RANGE_ERROR:
            return "LEP_COMM_RANGE_ERROR Comm port range error";
        case LEP_ERROR_CREATING_COMM:
            return "LEP_ERROR_CREATING_COMM Error creating comm";
        case LEP_ERROR_STARTING_COMM:
            return "LEP_ERROR_STARTING_COMM Error starting comm";
        case LEP_ERROR_CLOSING_COMM:
            return "LEP_ERROR_CLOSING_COMM Error closing comm";
        case LEP_COMM_CHECKSUM_ERROR:
            return "LEP_COMM_CHECKSUM_ERROR Comm checksum error";
        case LEP_COMM_NO_DEV:
            return "LEP_COMM_NO_DEV No comm device";
        case LEP_TIMEOUT_ERROR:
            return "LEP_TIMEOUT_ERROR Comm timeout error";
        case LEP_COMM_ERROR_WRITING_COMM:
            return "LEP_COMM_ERROR_WRITING_COMM Error writing comm";
        case LEP_COMM_ERROR_READING_COMM:
            return "LEP_COMM_ERROR_READING_COMM Error reading comm";
        case LEP_COMM_COUNT_ERROR:
            return "LEP_COMM_COUNT_ERROR Comm byte count error";
        case LEP_OPERATION_CANCELED:
            return "LEP_OPERATION_CANCELED Camera operation canceled";
        case LEP_UNDEFINED_ERROR_CODE:
            return "LEP_UNDEFINED_ERROR_CODE Undefined error";
        default:
            return "Other error LEP";
    }
}

void LeptonFLiR::checkForErrors() {
   if (_lastI2CError) {
        msg_len = sprintf(msg, "\n\r  LeptonFLiR::checkErrors lastI2CError: %i  ",_lastI2CError);
        HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
        msg_len = sprintf(msg, ": %s", textForI2CError(getLastI2CError() ));
        HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
    }
    if (_lastLepResult) {
        msg_len = sprintf(msg, "\n\r  LeptonFLiR::checkErrors lastLepResult: %i", _lastLepResult);
        HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
        msg_len = sprintf(msg, ": %s", textForLepResult( getLastLepResult() ) );
		HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
    }
}

#endif


bool LeptonFLiR::waitCommandBegin(int timeout) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    msg_len = sprintf(msg, "\n\r    LeptonFLiR::waitCommandBegin");
	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
#endif

    _lastLepResult = 0;


    uint16_t status;
    if (readRegister(LEP_I2C_STATUS_REG, &status))
        return false;

    if (!(status & LEP_I2C_STATUS_BUSY_BIT_MASK))
        return true;

    unsigned long endTime = millis() + (unsigned long)timeout;

    while ((status & LEP_I2C_STATUS_BUSY_BIT_MASK) && (timeout <= 0 || millis() < endTime)) {

        HAL_Delay(1);


        if (readRegister(LEP_I2C_STATUS_REG, &status))
            return false;
    }

    if (!(status & LEP_I2C_STATUS_BUSY_BIT_MASK))
        return true;
    else {
        _lastLepResult = LEP_TIMEOUT_ERROR;
        return false;
    }
}


bool LeptonFLiR::waitCommandFinish(int timeout) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    msg_len = sprintf(msg, "\n\r    LeptonFLiR::waitCommandFinish");
	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
#endif

    uint16_t status;
    if (readRegister(LEP_I2C_STATUS_REG, &status))
        return false;

    if (!(status & LEP_I2C_STATUS_BUSY_BIT_MASK)) {
        _lastLepResult = (byte)((status & LEP_I2C_STATUS_ERROR_CODE_BIT_MASK) >> LEP_I2C_STATUS_ERROR_CODE_BIT_SHIFT);
        return true;
    }

    unsigned long endTime = millis() + (unsigned long)timeout;

    while ((status & LEP_I2C_STATUS_BUSY_BIT_MASK) && (timeout <= 0 || millis() < endTime)) {

        HAL_Delay(1);

        if (readRegister(LEP_I2C_STATUS_REG, &status))
            return false;
    }

    if (!(status & LEP_I2C_STATUS_BUSY_BIT_MASK)) {
        _lastLepResult = (byte)((status & LEP_I2C_STATUS_ERROR_CODE_BIT_MASK) >> LEP_I2C_STATUS_ERROR_CODE_BIT_SHIFT);
        return true;
    } else {
        _lastLepResult = LEP_TIMEOUT_ERROR;
        return false;
    }
}

uint16_t LeptonFLiR::cmdCode(uint16_t cmdID, uint16_t cmdType) {
    return (cmdID & LEP_I2C_COMMAND_MODULE_ID_BIT_MASK) | (cmdID & LEP_I2C_COMMAND_ID_BIT_MASK) | (cmdType & LEP_I2C_COMMAND_TYPE_BIT_MASK);
}

uint16_t LeptonFLiR::cmdCodeOEM(uint16_t cmdID, uint16_t cmdType) {
    return (LEP_I2C_COMMAND_PROTECTION_BIT_BIT_MASK)|(cmdID & LEP_I2C_COMMAND_MODULE_ID_BIT_MASK) | (cmdID & LEP_I2C_COMMAND_ID_BIT_MASK) | (cmdType & LEP_I2C_COMMAND_TYPE_BIT_MASK);
}

void LeptonFLiR::sendCommand(uint16_t cmdCode) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    msg_len = sprintf(msg, "\n\r  LeptonFLiR::sendCommand cmdCode: 0x%x", cmdCode);
	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
#endif

    if (waitCommandBegin(LEPFLIR_GEN_CMD_TIMEOUT)) {

        if (writeCmdRegister(cmdCode, NULL, 0) == 0) {

            waitCommandFinish(LEPFLIR_GEN_CMD_TIMEOUT);
        }
    }

#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    checkForErrors();
#endif
}

void LeptonFLiR::sendCommand(uint16_t cmdCode, uint16_t value) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    msg_len = sprintf(msg, "\n\r  LeptonFLiR::sendCommand cmdCode: 0x%x", cmdCode);
	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
#endif

    if (waitCommandBegin(LEPFLIR_GEN_CMD_TIMEOUT)) {

        if (writeCmdRegister(cmdCode, &value, 1) == 0) {

            waitCommandFinish(LEPFLIR_GEN_CMD_TIMEOUT);
        }
    }

#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    checkForErrors();
#endif
}

void LeptonFLiR::sendCommand(uint16_t cmdCode, uint32_t value) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    msg_len = sprintf(msg, "\n\r  LeptonFLiR::sendCommand cmdCode: 0x%x", cmdCode);
	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
#endif

    if (waitCommandBegin(LEPFLIR_GEN_CMD_TIMEOUT)) {

        if (writeCmdRegister(cmdCode, (uint16_t *)&value, 2) == 0) {

            waitCommandFinish(LEPFLIR_GEN_CMD_TIMEOUT);
        }
    }

#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    checkForErrors();
#endif
}

void LeptonFLiR::sendCommand(uint16_t cmdCode, uint16_t *dataWords, int dataLength) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    msg_len = sprintf(msg, "\n\r  LeptonFLiR::sendCommand cmdCode: 0x%x", cmdCode);
	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
#endif

    if (waitCommandBegin(LEPFLIR_GEN_CMD_TIMEOUT)) {

        if (writeCmdRegister(cmdCode, dataWords, dataLength) == 0) {

            waitCommandFinish(LEPFLIR_GEN_CMD_TIMEOUT);
        }
    }

#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    checkForErrors();
#endif
}

void LeptonFLiR::receiveCommand(uint16_t cmdCode, uint16_t *value) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
      msg_len = sprintf(msg, "\n\r  LeptonFLiR::receiveCommand cmdCode: 0x%x",cmdCode);
	  HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
#endif

    if (waitCommandBegin(LEPFLIR_GEN_CMD_TIMEOUT)) {

        if (writeRegister(LEP_I2C_COMMAND_REG, cmdCode) == 0) {

            if (waitCommandFinish(LEPFLIR_GEN_CMD_TIMEOUT)) {

                readDataRegister(value, 1);
            }
        }
    }

#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    checkForErrors();
#endif
}

void LeptonFLiR::receiveCommand(uint16_t cmdCode, uint32_t *value) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    msg_len = sprintf(msg, "\n\r  LeptonFLiR::receiveCommand cmdCode: 0x%x",cmdCode);
	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
#endif

    if (waitCommandBegin(LEPFLIR_GEN_CMD_TIMEOUT)) {

        if (writeRegister(LEP_I2C_COMMAND_REG, cmdCode) == 0) {

            if (waitCommandFinish(LEPFLIR_GEN_CMD_TIMEOUT)) {

                readDataRegister((uint16_t *)value, 2);
            }
        }
    }

#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    checkForErrors();
#endif
}

void LeptonFLiR::receiveCommand(uint16_t cmdCode, uint16_t *readWords, int maxLength) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    msg_len = sprintf(msg, "\n\r  LeptonFLiR::receiveCommand cmdCode: 0x%x",cmdCode);
	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
#endif

    if (waitCommandBegin(LEPFLIR_GEN_CMD_TIMEOUT)) {

        if (writeRegister(LEP_I2C_COMMAND_REG, cmdCode) == 0) {

            if (waitCommandFinish(LEPFLIR_GEN_CMD_TIMEOUT)) {

                readDataRegister(readWords, maxLength);
            }
        }
    }

#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    checkForErrors();
#endif
}

int LeptonFLiR::writeCmdRegister(uint16_t cmdCode, uint16_t *dataWords, int dataLength) {

#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    msg_len = sprintf(msg, "\n\r    LeptonFLiR::writeCmdRegister cmdCode: 0x%x",cmdCode);
	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);

    msg_len = sprintf(msg, ", dataWords[%i]:",dataLength);
	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);

    for (int i = 0; i < dataLength; ++i) {
        if (i>0){
            msg_len = sprintf(msg, "-0x%x",dataWords[i]);
	        HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
        } else {
             msg_len = sprintf(msg, "0x%x",dataWords[i]);
	        HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
        }
    }
#endif

    if (dataWords && dataLength) {

    	HAL_StatusTypeDef I2C_Status;
    	uint8_t write_buf[2];
    	write_buf[0] = (uint16_t)dataLength>>8; // high byte
    	write_buf[1] = (uint16_t)dataLength;    // low byte


    	I2C_Status = HAL_I2C_Mem_Write(&hi2c2, DEVICE_ADDRESS_I2C, LEP_I2C_DATA_LENGTH_REG, 2, write_buf, 2, 500);
    	if(I2C_Status != HAL_OK){
    		return _lastI2CError;
    	}

        int maxLength = BUFFER_LENGTH / 2;
        int writeLength = std::min(maxLength, dataLength);
        uint16_t regAddress = dataLength <= 16 ? LEP_I2C_DATA_0_REG : LEP_I2C_DATA_BUFFER;

        while (dataLength > 0) {

        	HAL_StatusTypeDef I2C_Status2;
        	uint8_t write_buf2[2];
        	uint16_t moving_address = regAddress;
        	uint16_t tmp;

        	while(writeLength-- > 0){
        		tmp = *dataWords++;
				write_buf2[0] = tmp>>8;
				write_buf2[1] = tmp;

				I2C_Status2 = HAL_I2C_Mem_Write(&hi2c2, DEVICE_ADDRESS_I2C, moving_address, 2, write_buf2, 2, 500);
				moving_address = moving_address + 0x0002;

				if(I2C_Status2 != HAL_OK){
					return _lastI2CError;
				}
        	}


            regAddress += maxLength * 0x02;
            dataLength -= maxLength;
            writeLength = std::min(maxLength, dataLength);
        }
    }

	uint8_t write_buf3[2];
	write_buf3[0] = cmdCode>>8;
	write_buf3[1] = cmdCode;

	HAL_I2C_Mem_Write(&hi2c2, DEVICE_ADDRESS_I2C, LEP_I2C_COMMAND_REG, 2, write_buf3, 2, 500);
}

int LeptonFLiR::readDataRegister(uint16_t *readWords, int maxLength) {

	HAL_StatusTypeDef status;
	uint8_t read_buf[2];

	status = HAL_I2C_Mem_Read(&hi2c2, DEVICE_ADDRESS_I2C, LEP_I2C_DATA_LENGTH_REG, 2, read_buf, 2, 500);

	int bytesRead;
	int readLength;
	if(status == HAL_OK){
		readLength = ((uint16_t)read_buf[0]<<8|read_buf[1]);
		bytesRead = 2;
	} else {
		return (_lastI2CError = 4);
	}

    if (readLength == 0){
        return (_lastI2CError = 4);
    }

    int min_length = std::min(BUFFER_LENGTH, readLength);
    HAL_StatusTypeDef status2;
    uint8_t read_buf2[2];
    uint16_t moving_address = LEP_I2C_DATA_0_REG;

    status2 = HAL_I2C_Mem_Read(&hi2c2, DEVICE_ADDRESS_I2C, LEP_I2C_DATA_0_REG, 2, read_buf2, 2, 500);

    if(status2 == HAL_OK){
    	bytesRead = min_length;
    }

    while (bytesRead > 0 && readLength > 0) {

#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
        int origWordsRead = bytesRead / 2;
        int origReadLength = readLength / 2;
        int origMaxLength = maxLength;
        uint16_t *origReadWords = readWords;
#endif

        while (bytesRead > 1 && readLength > 1 && maxLength > 0) {

        	*(readWords++) = ((uint16_t)(read_buf2[0] & 0xFF) << 8) | (uint16_t)(read_buf2[1] & 0xFF);

        	moving_address += 0x0002;
            status2 = HAL_I2C_Mem_Read(&hi2c2, DEVICE_ADDRESS_I2C, moving_address, 2, read_buf2, 2, 500);

            bytesRead -= 2; readLength -= 2; --maxLength;
        }

#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
        msg_len = sprintf(msg, "\n\r      LeptonFLiR::readDataRegister readWords[");
		HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);

        if (origWordsRead == origReadLength && origReadLength == origMaxLength) {
            msg_len = sprintf(msg, "%i",origWordsRead);
		    HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
        }
        else if (origWordsRead != origReadLength && origReadLength == origMaxLength) {
            msg_len = sprintf(msg, "r:%d ,lm:%i",origWordsRead,origReadLength);
		    HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
        }
        else {
            msg_len = sprintf(msg, "r:%d ,l:%i, m:",origWordsRead,origReadLength,origMaxLength);
		    HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
        }
        msg_len = sprintf(msg, "] : ");
		HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);

        for (int i = 0; i < origWordsRead; ++i) {
            if (i>0){
                 msg_len = sprintf(msg, "-0x%x",origReadWords[i]);
		         HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
            }
            else {
                 msg_len = sprintf(msg, " 0x%x",origReadWords[i]);
		         HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
            }
        }
#endif


        if (readLength > 0){

        	HAL_StatusTypeDef status4;
        	min_length = std::min(BUFFER_LENGTH, readLength);
        	uint8_t read_buf4[min_length];

        	status4 = HAL_I2C_Mem_Read(&hi2c2, DEVICE_ADDRESS_I2C, moving_address, 2, read_buf4, min_length, 500);

        	if(status4 == HAL_OK){
        		bytesRead += std::min(BUFFER_LENGTH, readLength);
        	}

        }
    }

    while (maxLength-- > 0)
        *readWords++ = 0;

    return (_lastI2CError = readLength ? 4 : 0);
}

int LeptonFLiR::writeRegister(uint16_t regAddress, uint16_t value) {

#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    msg_len = sprintf(msg, "\n\r    LeptonFLiR::writeRegister regAddress: 0x%x, value: 0x%x",regAddress,value);
	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
#endif

    HAL_StatusTypeDef I2C_status;
	uint8_t write_buf[2];
	write_buf[0] = value >> 8; // HIGH BIT
	write_buf[1] = value;      // LOW BIT

	I2C_status = HAL_I2C_Mem_Write(&hi2c2, DEVICE_ADDRESS_I2C, regAddress, 2, write_buf, 2, 500);

    if (I2C_status == HAL_OK){
		return 0;
    } else {
    	return 4;
    }
}


int LeptonFLiR::readRegister(uint16_t regAddress, uint16_t *value) {
#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    msg_len = sprintf(msg, "\n\r    LeptonFLiR::readRegister regAddress: 0x%x",regAddress);
	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
#endif

	int bytesread;
	HAL_StatusTypeDef I2C_status;
	uint8_t read_buf[2];

	I2C_status =  HAL_I2C_Mem_Read(&hi2c2, DEVICE_ADDRESS_I2C, regAddress, 2, read_buf, 2, 500);

	if(I2C_status == HAL_OK) {
		*value = ((uint16_t)read_buf[0]<<8 | read_buf[1]); // HIGH BIT | LOW BIT
		bytesread = 2;
	} else {
		return _lastI2CError;
	}

#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    msg_len = sprintf(msg,"\n\r      LeptonFLiR::readRegister retVal: 0x%x",*value);
	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 500);
#endif

    return _lastI2CError;
}



#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT

void LeptonFLiR::printModuleInfo() {
    char buffer[80];

    msg_len = sprintf(msg, "\n\n\r ~~~ LeptonFLiR Module Info ~~~\n" );
	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);

    msg_len = sprintf(msg, "\n\r Image Storage Mode: \n\r %i  :" , _storageMode );
	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
    switch (_storageMode) {
        case LeptonFLiR_ImageStorageMode_80x60_16bpp:
            msg_len = sprintf(msg, "LeptonFLiR_ImageStorageMode_80x60_8bpp\n" );
            HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
        break;
        case LeptonFLiR_ImageStorageMode_80x60_8bpp:
            msg_len = sprintf(msg, "LeptonFLiR_ImageStorageMode_80x60_8bpp\n" );
            HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
		break;
        case LeptonFLiR_ImageStorageMode_40x30_16bpp:
            msg_len = sprintf(msg, "LeptonFLiR_ImageStorageMode_40x30_16bpp\n" );
            HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
		break;
        case LeptonFLiR_ImageStorageMode_40x30_8bpp:
        	msg_len = sprintf(msg, "LeptonFLiR_ImageStorageMode_40x30_8bpp\n" );
        	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
		break;
        case LeptonFLiR_ImageStorageMode_20x15_16bpp:
            msg_len = sprintf(msg, "LeptonFLiR_ImageStorageMode_20x15_16bpp\n" );
            HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
		break;
        case LeptonFLiR_ImageStorageMode_20x15_8bpp:
            msg_len = sprintf(msg, "LeptonFLiR_ImageStorageMode_20x15_8bpp\n" );
            HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
		break;
        default:    
            msg_len = sprintf(msg, " " );
            HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
		break;
    }

    msg_len = sprintf(msg, "\n\r Temperature Mode : \n\r %i :",_tempMode );
    HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
    switch (_tempMode) {
        case LeptonFLiR_TemperatureMode_Celsius:
            msg_len = sprintf(msg, "LeptonFLiR_TemperatureMode_Celsius");
            HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
            break;
        case LeptonFLiR_TemperatureMode_Fahrenheit:
            msg_len = sprintf(msg, "LeptonFLiR_TemperatureMode_Fahrenheit");
            HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
            break;
        case LeptonFLiR_TemperatureMode_Kelvin:
            msg_len = sprintf(msg, "LeptonFLiR_TemperatureMode_Kelvin" );
            HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
            break;
        default:
        	break;
    }

    msg_len = sprintf(msg, "\n\rMemory Footprint :  \n\r" , _storageMode );
	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
    int mallocOffset = 0;

#ifndef LEPFLIR_DISABLE_ALIGNED_MALLOC
    mallocOffset = 15;
#endif

    msg_len = sprintf(msg, "Image Data: %i" , _imageData ? getImageTotalBytes() + mallocOffset : 0 );
	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);


    msg_len = sprintf(msg, "B, SPI Frame Data: %i" , _spiFrameData ? getSPIFrameTotalBytes() + mallocOffset : 0 );
	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);


    msg_len = sprintf(msg, "B, Telemetry Data: %i" ,_telemetryData ? LEPFLIR_SPI_FRAME_PACKET_SIZE : 0 );
	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);


    msg_len = sprintf(msg, "B, Total: %iB\n" ,(_imageData ? getImageTotalBytes() + mallocOffset : 0) + (_spiFrameData ? getSPIFrameTotalBytes() + mallocOffset : 0) + (_telemetryData ? LEPFLIR_SPI_FRAME_PACKET_SIZE : 0));
	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);


    msg_len = sprintf(msg, "\n\r Power Register :  ");
	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
    uint16_t powerReg; readRegister(LEP_I2C_POWER_REG, &powerReg);

#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    checkForErrors();
#endif

    msg_len = sprintf(msg, "\n\r 0x%x", powerReg);
	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);


    msg_len = sprintf(msg, "\n\n\r Status Register :  ");
	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
    uint16_t statusReg; readRegister(LEP_I2C_STATUS_REG, &statusReg);

#ifdef LEPFLIR_ENABLE_DEBUG_OUTPUT
    checkForErrors();
#endif

    msg_len = sprintf(msg, "\n\r 0x%x", statusReg);
	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);


    msg_len = sprintf(msg, "\n\n\r AGC:");
	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
	if(agc_getAGCEnabled()){
		msg_len = sprintf(msg, "\n\r AGC: Enabled" );
	}else {
		msg_len = sprintf(msg, "\n\r AGC: Disabled");
	}
	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);


    msg_len = sprintf(msg, "\n\n\r AGC Policy:");
	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
    LEP_AGC_POLICY policy = agc_getAGCPolicy();
    msg_len = sprintf(msg, "\n\r %i : ", policy);
	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
    switch (policy) {
        case LEP_AGC_LINEAR:
            msg_len = sprintf(msg, "LEP_AGC_LINEAR");
        	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
        	break;
        case LEP_AGC_HEQ:
            msg_len = sprintf(msg, "LEP_AGC_HEQ");
        	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
		break;
        default:
		break;
    }


    msg_len = sprintf(msg, "\n\n\r AGC HEQ Scale Factor:");
	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
    LEP_AGC_HEQ_SCALE_FACTOR factor = agc_getHEQScaleFactor();
    msg_len = sprintf(msg, "\n\r %i : ", factor);
	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
    switch (factor) {
        case LEP_AGC_SCALE_TO_8_BITS:
            msg_len = sprintf(msg, "LEP_AGC_SCALE_TO_8_BITS");
        	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
		break;
        case LEP_AGC_SCALE_TO_14_BITS:
            msg_len = sprintf(msg, " EP_AGC_SCALE_TO_14_BITS ");
        	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
		break;
        default:
		break;
    }


    msg_len = sprintf(msg, "\n\n\r AGC Calculation Enabled:");
	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
	if(agc_getAGCCalcEnabled()){
	    msg_len = sprintf(msg, "\n\r enabled");
		HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
	}else{
	    msg_len = sprintf(msg, "\n\r disabled");
		HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
	}


    msg_len = sprintf(msg, "\n\n\r SYS Camera Status:");
	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
    LEP_SYS_CAM_STATUS_STATES camStatus = sys_getCameraStatus();
    msg_len = sprintf(msg, "\n\r %i : ", camStatus);
	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
    switch (camStatus) {
        case LEP_SYSTEM_READY:
            msg_len = sprintf(msg, "LEP_SYSTEM_READY ");
        	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
		break;
        case LEP_SYSTEM_INITIALIZING:
            msg_len = sprintf(msg, "LEP_SYSTEM_INITIALIZING");
        	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
		break;
        case LEP_SYSTEM_IN_LOW_POWER_MODE:
            msg_len = sprintf(msg, "LEP_SYSTEM_IN_LOW_POWER_MODE");
        	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
		break;
        case LEP_SYSTEM_GOING_INTO_STANDBY:
            msg_len = sprintf(msg, "LEP_SYSTEM_GOING_INTO_STANDBY");
        	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
		break;
        case LEP_SYSTEM_FLAT_FIELD_IN_PROCESS:
            msg_len = sprintf(msg, "LEP_SYSTEM_FLAT_FIELD_IN_PROCESS ");
        	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
		break;
        default:
		break;
    }


    msg_len = sprintf(msg, "\n\n\r FLiR Serial Number: ");
	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
    sys_getFlirSerialNumber(buffer, 80);
    msg_len = sprintf(msg, "\n\r %s ", buffer);
	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);


    msg_len = sprintf(msg, "\n\n\r Customer Serial Number:");
	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
    sys_getCustomerSerialNumber(buffer, 80);
    msg_len = sprintf(msg, "\n\r %s", buffer);
	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);


    msg_len = sprintf(msg, "\n\n\r Camera Uptime:\n\r ");
	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
	msg_len = sprintf(msg, "\n\r Uptime: %d ms", sys_getCameraUptime());
	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);


	msg_len = sprintf(msg, "\n\n\r Sys Aux Temperature:");
	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
	msg_len = sprintf(msg, "\n\r %i ", sys_getAuxTemperature());
	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
	getTemperatureSymbol();


	msg_len = sprintf(msg, "\n\n\r Sys FPA Temperature:");
	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
	msg_len = sprintf(msg, "\n\r %i ", sys_getFPATemperature());
	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
	getTemperatureSymbol();


    msg_len = sprintf(msg, "\n\n\r Telemetry: ");
	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
	if(sys_getTelemetryEnabled()){
	    msg_len = sprintf(msg, "\n\rTelemetry: enabled");
	}else{
	    msg_len = sprintf(msg, "\n\rTelemetry: disabled");
	}
	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);


	msg_len = sprintf(msg, "\n\n\r Vid Polarity:");
	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
    LEP_VID_POLARITY polarity = vid_getPolarity();
	msg_len = sprintf(msg, "\n\r %i : ", polarity);
	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
    switch (polarity) {
        case LEP_VID_WHITE_HOT:
        	msg_len = sprintf(msg, "LEP_VID_WHITE_HOT");
        	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
		break;
        case LEP_VID_BLACK_HOT:
        	msg_len = sprintf(msg, "LEP_VID_BLACK_HOT");
        	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
		break;
        default:
		break;
    }


	msg_len = sprintf(msg, "\n\n\r Vid Pseudo Color Lookup Table:");
	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
    LEP_VID_PCOLOR_LUT table = vid_getPseudoColorLUT();
	msg_len = sprintf(msg, "\n\r %i : ", table);
	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
    switch (table) {
        case LEP_VID_WHEEL6_LUT:
        	msg_len = sprintf(msg, "LEP_VID_WHEEL6_LUT");
        	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
		break;
        case LEP_VID_FUSION_LUT:
        	msg_len = sprintf(msg, "LEP_VID_FUSION_LUT");
        	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
		break;
        case LEP_VID_RAINBOW_LUT:
        	msg_len = sprintf(msg, "LEP_VID_RAINBOW_LUT");
        	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
		break;
        case LEP_VID_GLOBOW_LUT:
        	msg_len = sprintf(msg, "LEP_VID_GLOBOW_LUT");
        	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
		break;
        case LEP_VID_COLOR_LUT:
        	msg_len = sprintf(msg, "LEP_VID_COLOR_LUT");
        	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
		break;
        case LEP_VID_ICE_FIRE_LUT:
        	msg_len = sprintf(msg, "LEP_VID_ICE_FIRE_LUT");
        	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
		break;
        case LEP_VID_RAIN_LUT:
        	msg_len = sprintf(msg, "LEP_VID_RAIN_LUT");
        	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
		break;
        case LEP_VID_USER_LUT:
        	msg_len = sprintf(msg, "LEP_VID_USER_LUT");
        	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
		break;
        default:
		break;
    }


	msg_len = sprintf(msg, "\n\n\r Vid Focus Calculation Enabled:");
	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
    if(vid_getFocusCalcEnabled()){
    	msg_len = sprintf(msg, "\n\r enabled");
    }else{
    	msg_len = sprintf(msg, "\n\r disabled");
    }


	msg_len = sprintf(msg, "\n\n\r Vid Freeze Enabled:");
	HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
	if(vid_getFreezeEnabled()){
		msg_len = sprintf(msg, "\n\r enabled");
		HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
	}else{
		msg_len = sprintf(msg, "\n\r disabled");
		HAL_UART_Transmit(&huart2, (uint8_t *)msg, msg_len, 5000);
	}
}

#endif
