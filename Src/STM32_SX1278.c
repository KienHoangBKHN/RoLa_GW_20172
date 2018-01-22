/*
 * STM32_SX1278.c
 *
 *  Created on: Jan 22, 2018
 *      Author: kien
 */


/*!_____
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2013 Semtech
 ___    _    _____  ___  _      _    _
/ __|  /_\  |  _  \/ __|| |    /_\  | |_
\__ \ / _ \ | | | |\__ \| |_  / _ \ | _ \
|___//_/ \_\|_| |_||___/|___)/_/ \_\|__ /

	******************************************************************************
  * @file    STM32_SX1278.c
  * @author  Dao Duc Anh
  * @version V1.0.0
  * @date    8/5/2017
  * @brief   SX1278 module driver.
  *          This is the common part of the SX1278 initialization
  *
  @verbatim
  ==============================================================================
                     ##### How to use this driver #####
  ==============================================================================
    This program designed by sanslab engineer. It base on the opensource api
		from cooking hack
		Link: http://www.cooking-hacks.com/media/cooking/images/documentation/tutorial_SX1272/SX1272_library_arduino_v1.4.zip

		This program is distributed in the hope that it will be useful  for anyone want
		to work with the LoRa chip from semtech like SX1278.


  @endverbatim
  ******************************************************************************
 */


#include "STM32_SX1278.h"

// LPD433 (low power device 433 MHz) is a part of ITU region 1 ISM band
/*!
	From 433.050 MHz to 434.790 MHz: BW: 1.74 MHz
	More info: https://en.wikipedia.org/wiki/LPD433
 */

#define  CH_DEFAULT  0x6c8000 // default channel, center frequency = 434.000MHz

// FREQUENCY CHANNELS (BANDWIDTH 500KHz):
#define CH_1_BW_500  0x6c5345 // channel 1, bandwidth 500KHz, center frequency = 433.3MHz ( 433.050MHz - 433.550MHz )
#define CH_2_BW_500  0x6c7af3 // channel 2, bandwidth 500KHz, center frequency = 433.92MHz ( 433.670MHz - 433.920MHz )
#define CH_3_BW_500  0x6ca2a1 // channel 3, bandwidth 500KHz, center frequency = 434.54MHz ( 434.290MHz - 434.790MHz )

// FREQUENCY CHANNELS (BANDWIDTH 250KHz):
#define CH_1_BW_250 0x6c4b45 // channel 1, bandwidth 250KHz, center frequency = 433.175MHz ( 433.050MHz - 433.300MHz )
#define CH_2_BW_250 0x6c5e57 // channel 2, bandwidth 250KHz, center frequency = 433.473MHz ( 433.348MHz - 433.598MHz )
#define CH_3_BW_250 0x6c716a// channel 3, bandwidth 250KHz, center frequency = 433.771MHz ( 433.646MHz - 433.896MHz )
#define CH_4_BW_250 0x6c847c // channel 4, bandwidth 250KHz, center frequency = 434.069MHz ( 433.944MHz - 434.194MHz )
#define	CH_5_BW_250 0x6c978f // channel 5, bandwidth 250KHz, center frequency = 434.367MHz ( 434.242MHz - 434.492MHz )
#define CH_6_BW_250 0x6caaa1 // channel 6, bandwidth 250KHz, center frequency = 434.665MHz ( 434.540MHz - 434.790MHz )

// FREQUENCY CHANNELS (BANDWIDTH 125KHz):
#define CH_1_BW_125  0x6c4745 // channel 1, bandwidth 125KHz, center frequency = 433.1125MHz ( 433.050MHz - 433.175MHz )
#define CH_2_BW_125  0x6c4fe1 // channel 2, bandwidth 125KHz, center frequency = 433.247MHz ( 433.1845MHz - 433.3095MHz )
#define CH_3_BW_125  0x6c587c // channel 3, bandwidth 125KHz, center frequency = 433.3815MHz ( 433.319MHz - 433.444MHz )
#define CH_4_BW_125  0x6c6118 // channel 4, bandwidth 125KHz, center frequency = 433.516MHz ( 433.4535MHz - 433.5785MHz )
#define CH_5_BW_125  0x6c69b3 // channel 5, bandwidth 125KHz, center frequency = 433.6505MHz ( 433.588MHz - 433.713MHz )
#define CH_6_BW_125  0x6c724f // channel 6, bandwidth 125KHz, center frequency = 433.785MHz ( 433.7225MHz - 433.8475MHz )
#define CH_7_BW_125  0x6c7af3// channel 7, bandwidth 125KHz, center frequency = 433.92MHz ( 433.8575MHz - 433.9825MHz )
#define CH_8_BW_125  0x6c8397 // channel 8, bandwidth 125KHz, center frequency = 434.055MHz ( 433.9925MHz - 434.1175MHz )
#define CH_9_BW_125  0x6c8c32 // channel 9, bandwidth 125KHz, center frequency = 434.1895MHz ( 434.127MHz - 434.252MHz )
#define CH_10_BW_125  0x6c94ce // channel 10, bandwidth 125KHz, center frequency = 434.324MHz ( 434.2615MHz - 434.3865MHz )
#define CH_11_BW_125  0x6c9d6a // channel 11, bandwidth 125KHz, center frequency = 434.4585MHz ( 434.396MHz - 434.521MHz )
#define CH_12_BW_125  0x6ca605 // channel 12, bandwidth 125KHz, center frequency = 434.593MHz ( 434.5305MHz - 434.6555MHz )
#define CH_13_BW_125  0x6CaeA1 // channel 13, bandwidth 125KHz, center frequency = 434.7275MHz ( 434.665MHz - 434.790MHz )

// FREQUENCY CHANNELS (BANDWIDTH < 125KHz: separate 72.5KHz):
#define CH_1  0x6c4597 // channel 1, center freq = 433.086MHz
#define CH_2  0x6c4a3b // channel 2, center freq = 433.159MHz
#define CH_3  0x6c4edf // channel 3, center freq = 433.231MHz
#define CH_4  0x6c5383 // channel 4, center freq = 433.304MHz
#define CH_5  0x6c5827 // channel 5, center freq = 433.376MHz
#define CH_6  0x6c5ccb// channel 6, center freq = 433.449MHz
#define CH_7  0x6c616f // channel 7, center freq = 433.521MHz
#define CH_8  0x6c6613// channel 8, center freq = 433.594MHz
#define CH_9  0x6c6ab7 // channel 9, center freq = 433.666MHz
#define CH_10  0x6c6f5b // channel 10, center freq = 433.739MHz
#define CH_11  0x6c73ff // channel 11, center freq = 433.811MHz
#define CH_12  0x6c78a3 // channel 12, center freq = 433.884MHz
#define CH_13  0x6c7d47 // channel 13, center freq = 433.956MHz
#define CH_14  0x6c81eb // channel 14, center freq = 434.029MHz
#define CH_15  0x6c868f // channel 15, center freq = 434.101MHz
#define CH_16  0x6c8b33 // channel 16, center freq = 434.174MHz
#define CH_17  0x6c8fd8 // channel 17, center freq = 434.246MHz
#define CH_18  0x6c947c // channel 18, center freq = 434.319MHz
#define CH_19  0x6c9920 // channel 19, center freq = 434.391MHz
#define CH_20  0x6c9dc4 // channel 20, center freq = 434.464MHz
#define CH_21  0x6ca268 // channel 21, center freq = 434.536MHz
#define CH_22  0x6ca70c // channel 22, center freq = 434.609MHz
#define CH_23  0x6cabb0 // channel 23, center freq = 434.682MHz
#define CH_24  0x6cb054 // channel 24, center freq = 434.754MHz

//LORA BANDWIDTH:
#define  BW_7_8  0x00
#define  BW_10_4  0x01
#define  BW_15_6  0x02
#define  BW_20_8  0x03
#define  BW_31_2  0x04
#define  BW_41_7  0x05
#define  BW_62_5  0x06
#define  BW_125  0x07
#define  BW_250  0x08
#define  BW_500  0x09

/*
 * #define CH_DEFAULT = 0x6c8000; // default channel, center frequency = 434.000MHz

// FREQUENCY CHANNELS (BANDWIDTH 500KHz):
#define CH_1_BW_500 = 0x6c5345; // channel 1, bandwidth 500KHz, center frequency = 433.3MHz ( 433.050MHz - 433.550MHz )
#define CH_2_BW_500 = 0x6c7af3; // channel 2, bandwidth 500KHz, center frequency = 433.92MHz ( 433.670MHz - 433.920MHz )
#define CH_3_BW_500 = 0x6ca2a1; // channel 3, bandwidth 500KHz, center frequency = 434.54MHz ( 434.290MHz - 434.790MHz )

// FREQUENCY CHANNELS (BANDWIDTH 250KHz):
#define CH_1_BW_250 = 0x6c4b45; // channel 1, bandwidth 250KHz, center frequency = 433.175MHz ( 433.050MHz - 433.300MHz )
#define CH_2_BW_250 = 0x6c5e57; // channel 2, bandwidth 250KHz, center frequency = 433.473MHz ( 433.348MHz - 433.598MHz )
#define CH_3_BW_250 = 0x6c716a; // channel 3, bandwidth 250KHz, center frequency = 433.771MHz ( 433.646MHz - 433.896MHz )
#define CH_4_BW_250 = 0x6c847c; // channel 4, bandwidth 250KHz, center frequency = 434.069MHz ( 433.944MHz - 434.194MHz )
#define CH_5_BW_250 = 0x6c978f; // channel 5, bandwidth 250KHz, center frequency = 434.367MHz ( 434.242MHz - 434.492MHz )
#define CH_6_BW_250 = 0x6caaa1; // channel 6, bandwidth 250KHz, center frequency = 434.665MHz ( 434.540MHz - 434.790MHz )

// FREQUENCY CHANNELS (BANDWIDTH 125KHz):
#define CH_1_BW_125 = 0x6c4745; // channel 1, bandwidth 125KHz, center frequency = 433.1125MHz ( 433.050MHz - 433.175MHz )
#define CH_2_BW_125 = 0x6c4fe1; // channel 2, bandwidth 125KHz, center frequency = 433.247MHz ( 433.1845MHz - 433.3095MHz )
#define CH_3_BW_125 = 0x6c587c; // channel 3, bandwidth 125KHz, center frequency = 433.3815MHz ( 433.319MHz - 433.444MHz )
#define CH_4_BW_125 = 0x6c6118; // channel 4, bandwidth 125KHz, center frequency = 433.516MHz ( 433.4535MHz - 433.5785MHz )
#define CH_5_BW_125 = 0x6c69b3; // channel 5, bandwidth 125KHz, center frequency = 433.6505MHz ( 433.588MHz - 433.713MHz )
#define CH_6_BW_125 = 0x6c724f; // channel 6, bandwidth 125KHz, center frequency = 433.785MHz ( 433.7225MHz - 433.8475MHz )
#define CH_7_BW_125 = 0x6c7af3; // channel 7, bandwidth 125KHz, center frequency = 433.92MHz ( 433.8575MHz - 433.9825MHz )
#define CH_8_BW_125 = 0x6c8397; // channel 8, bandwidth 125KHz, center frequency = 434.055MHz ( 433.9925MHz - 434.1175MHz )
#define CH_9_BW_125 = 0x6c8c32; // channel 9, bandwidth 125KHz, center frequency = 434.1895MHz ( 434.127MHz - 434.252MHz )
#define CH_10_BW_125 = 0x6c94ce; // channel 10, bandwidth 125KHz, center frequency = 434.324MHz ( 434.2615MHz - 434.3865MHz )
#define CH_11_BW_125 = 0x6c9d6a; // channel 11, bandwidth 125KHz, center frequency = 434.4585MHz ( 434.396MHz - 434.521MHz )
#define CH_12_BW_125 = 0x6ca605; // channel 12, bandwidth 125KHz, center frequency = 434.593MHz ( 434.5305MHz - 434.6555MHz )
#define CH_13_BW_125 = 0x6CaeA1; // channel 13, bandwidth 125KHz, center frequency = 434.7275MHz ( 434.665MHz - 434.790MHz )

// FREQUENCY CHANNELS (BANDWIDTH < 125KHz: separate 72.5KHz):
#define CH_1 = 0x6c4597; // channel 1, center freq = 433.086MHz
#define CH_2 = 0x6c4a3b; // channel 2, center freq = 433.159MHz
#define CH_3 = 0x6c4edf; // channel 3, center freq = 433.231MHz
#define CH_4 = 0x6c5383; // channel 4, center freq = 433.304MHz
#define CH_5 = 0x6c5827; // channel 5, center freq = 433.376MHz
#define CH_6 = 0x6c5ccb; // channel 6, center freq = 433.449MHz
#define CH_7 = 0x6c616f; // channel 7, center freq = 433.521MHz
#define CH_8 = 0x6c6613; // channel 8, center freq = 433.594MHz
#define CH_9 = 0x6c6ab7; // channel 9, center freq = 433.666MHz
#define CH_10 = 0x6c6f5b; // channel 10, center freq = 433.739MHz
#define CH_11 = 0x6c73ff; // channel 11, center freq = 433.811MHz
#define CH_12 = 0x6c78a3; // channel 12, center freq = 433.884MHz
#define CH_13 = 0x6c7d47; // channel 13, center freq = 433.956MHz
#define CH_14 = 0x6c81eb; // channel 14, center freq = 434.029MHz
#define CH_15 = 0x6c868f; // channel 15, center freq = 434.101MHz
#define CH_16 = 0x6c8b33; // channel 16, center freq = 434.174MHz
#define CH_17 = 0x6c8fd8; // channel 17, center freq = 434.246MHz
#define CH_18 = 0x6c947c; // channel 18, center freq = 434.319MHz
#define CH_19 = 0x6c9920; // channel 19, center freq = 434.391MHz
#define CH_20 = 0x6c9dc4; // channel 20, center freq = 434.464MHz
#define CH_21 = 0x6ca268; // channel 21, center freq = 434.536MHz
#define CH_22 = 0x6ca70c; // channel 22, center freq = 434.609MHz
#define CH_23 = 0x6cabb0; // channel 23, center freq = 434.682MHz
#define CH_24 = 0x6cb054; // channel 24, center freq = 434.754MHz

//LORA BANDWIDTH:
#define  BW_7_8 = 0x00;
#define  BW_10_4 = 0x01;
#define  BW_15_6 = 0x02;
#define  BW_20_8 = 0x03;
#define  BW_31_2 = 0x04;
#define  BW_41_7 = 0x05;
#define  BW_62_5 = 0x06;
#define  BW_125 = 0x07;
#define  BW_250 = 0x08;
#define  BW_500 = 0x09;
 */
const double SignalBwLog[] =
{
    5.0969100130080564143587833158265,
    5.397940008672037609572522210551,
    5.6989700043360188047862611052755
};

//LORA CODING RATE:
#define  CR_5  0x01	// CR = 4/5
#define  CR_6  0x02	// CR = 4/6
#define  CR_7 0x03	// CR = 4/7
#define  CR_8  0x04	// CR = 4/8

//LORA SPREADING FACTOR:
#define  SF_6  0x06
#define  SF_7  0x07
#define  SF_8  0x08
#define  SF_9  0x09
#define  SF_10 0x0A
#define  SF_11  0x0B
#define  SF_12  0x0C

//LORA MODES:
#define  LORA_SLEEP_MODE  0x80
#define  LORA_STANDBY_MODE  0x81
#define  LORA_TX_MODE  0x83
#define  LORA_RX_MODE  0x85
#define  LORA_STANDBY_FSK_REGS_MODE  0xC1

//FSK MODES:
#define  FSK_SLEEP_MODE  0x00
#define  FSK_STANDBY_MODE  0x01
#define  FSK_TX_MODE  0x03
#define  FSK_RX_MODE  0x05

//OTHER CONSTANTS:

#define  st_SD_ON  1
#define  st_SD_OFF 0
#define  HEADER_ON 0
#define  HEADER_OFF 1
#define  CRC_ON  1
#define  CRC_OFF  0
#define  LORA  1
#define  FSK  0
#define  BROADCAST_0  0x00
#define  MAX_LENGTH  255
#define  MAX_PAYLOAD 251
#define  MAX_LENGTH_FSK 64
#define  MAX_PAYLOAD_FSK  60
#define  ACK_LENGTH  5
#define  OFFSET_PAYLOADLENGTH  5
#define  OFFSET_RSSI  137
#define  NOISE_FIGURE 6.0
#define  NOISE_ABSOLUTE_ZERO 174.0
#define MAX_TIMEOUT  10000		//10000 msec = 10.0 sec
#define MAX_WAIT  12000		//12000 msec = 12.0 sec
#define MESH_TIMEOUT  3600000  //3600000 msec = 3600 sec = 1 hour
#define  MAX_RETRIES 5
#define  CORRECT_PACKET  0
#define  INCORRECT_PACKET  1



	// Initialize class variables




_Bool check;

/// Variables /////////////////////////////////////////////////////////////

	//! Variable : SD state.
	//!    st_SD = 00  --> SD_OFF
	//!    st_SD = 01  --> SD_ON
	uint8_t st_SD;

	//! Variable : bandwidth configured in LoRa mode.
	//!    bandwidth = 0000  --> BW = 7.8KHz
	//!    bandwidth = 0001  --> BW = 10.4KHz
	//!    bandwidth = 0010  --> BW = 15.6KHz
	//!    bandwidth = 0011  --> BW = 20.8KHz
	//!    bandwidth = 0100  --> BW = 31.2KHz
	//!    bandwidth = 0101  --> BW = 41.7KHz
	//!    bandwidth = 0110  --> BW = 62.5KHz
	//!    bandwidth = 0111  --> BW = 125KHz
	//!    bandwidth = 1000  --> BW = 250KHz
	//!    bandwidth = 1001  --> BW = 500KHz
	uint8_t _bandwidth;

	//! Variable : coding rate configured in LoRa mode.
	//!    codingRate = 001  --> CR = 4/5
	//!    codingRate = 010  --> CR = 4/6
	//!    codingRate = 011  --> CR = 4/7
	//!    codingRate = 100  --> CR = 4/8
	uint8_t _codingRate;

	//! Variable : spreading factor configured in LoRa mode.
	//!    spreadingFactor = 6   --> SF = 6, 64 chips/symbol
	//!    spreadingFactor = 7   --> SF = 7, 128 chips/symbol
	//!    spreadingFactor = 8   --> SF = 8, 256 chips/symbol
	//!    spreadingFactor = 9   --> SF = 9, 512 chips/symbol
	//!    spreadingFactor = 10  --> SF = 10, 1024 chips/symbol
	//!    spreadingFactor = 11  --> SF = 11, 2048 chips/symbol
	//!    spreadingFactor = 12  --> SF = 12, 4096 chips/symbol
	uint8_t _spreadingFactor;

	//! Variable : frequency channel.
	//!    channel = 0x6b0000  --> CH = 1, 428MHz
	//!    channel = 0x6b2000  --> CH = 2, 428.5MHz
	//!    channel = 0x6b4000  --> CH = 3, 429MHz
	//!    channel = 0x6b6000  --> CH = 4, 429.5MHz
	//!    channel = 0x6b8000  --> CH = 5, 430MHz
	//!    channel = 0x6ba000  --> CH = 6, 430.5MHz
	//!    channel = 0x6bc000  --> CH = 7, 431MHz
	//!    channel = 0x6be000  --> CH = 8, 431.5MHz
	//!    channel = 0x6c0000  --> CH = 9, 432MHz
	//!    channel = 0x6c2000  --> CH = 10, 432.5MHz
	//!    channel = 0x6c4000  --> CH = 11, 433MHz
	//!    channel = 0x6c6000  --> CH = 12, 433.5MHz
	//!    channel = 0x6c8000  --> CH = 13, 434MHz
	//!    channel = 0x6ca000  --> CH = 14, 434.5MHz
	//!    channel = 0x6cc000  --> CH = 15, 435MHz
	//!    channel = 0x6ce000  --> CH = 16, 435.5MHz
	//!    channel = 0x6d0000  --> CH = 17, 436MHz
	//!    channel = 0x6d2000  --> CH = 18, 436.5MHz
	//!    channel = 0x6d4000  --> CH = 19, 437MHz
	//!    channel = 0x6d6000  --> CH = 20, 437.5MHz
	//!    channel = 0x6d8000  --> CH = 21, 438MHz
	//!    channel = 0x6da000  --> CH = 22, 438.5MHz
	//!    channel = 0x6dc000  --> CH = 23, 439MHz
	//!    channel = 0x6de000  --> CH = 24, 439.5MHz
	uint32_t _channel;

	//! Variable : output power.
	//!
	uint8_t _power;

	//! Variable : SNR from the last packet received in LoRa mode.
	//!
	int8_t _SNR;

	//! Variable : RSSI current value.
	//!
	int8_t _RSSI;

	//! Variable : RSSI from the last packet received in LoRa mode.
	//!
	int16_t _RSSIpacket;

	//! Variable : preamble length sent/received.
	//!
	uint16_t _preamblelength;

	//! Variable : payload length sent/received.
	//!
	uint16_t _payloadlength;

	//! Variable : node address.
	//!
	uint8_t _nodeAddress;

	//! Variable : implicit or explicit header in LoRa mode.
	//!
	uint8_t _header = HEADER_ON;

	//! Variable : header received while waiting a packet to arrive.
	//!
	uint8_t _hreceived;

	//! Variable : presence or absence of CRC calculation.
	//!
	uint8_t _CRC;

	//! Variable : packet destination.
	//!
	uint8_t _destination;

	//! Variable : packet number.
	//!
	uint8_t _packetNumber;

	//! Variable : indicates if received packet is correct or incorrect.
  uint8_t _reception;

	//! Variable : number of current retry.
	uint8_t _retries;

  //! Variable : maximum number of retries.
	uint8_t _maxRetries;

  //! Variable : maximum current supply.
	uint8_t _maxCurrent;

	//! Variable : indicates FSK or LoRa modem.
	uint8_t _modem;

	//! Variable : temperature module.
	int _temp;

	//! Variable : current timeout to send a packet.
	uint16_t _sendTime;

	//! Variable : array with all the information about a sent/received/sent(received) ack packet.
	extern pack packet_sent, packet_received, ACK;


//************************************************************
//FUNCTION****************************************************

//BEGIN

void SX1278(void)
{
	_bandwidth = BW_125;
	_codingRate = CR_5;
	_spreadingFactor = SF_7;
	_channel = CH_1;
	_header = HEADER_ON;
	_CRC = CRC_ON;
	_modem = LORA;
	_power = 15;
	_packetNumber = 0;
	_reception = CORRECT_PACKET;
	_retries = 0;
	_maxRetries = 3;
	packet_sent.retry = _retries;
}
/*
 Function: in out data spi.
*/
uint16_t HW_SPI_InOut(SPI_HandleTypeDef *hspi, uint16_t txData )
{
  uint16_t rxData ;

  HAL_SPI_TransmitReceive( hspi, ( uint8_t * ) &txData, ( uint8_t* ) &rxData, 1, HAL_MAX_DELAY);

  return rxData;
}

void SX1272ReadBuffer(SPI_HandleTypeDef *hspi, uint8_t addr, uint8_t *buffer, uint8_t size )
{
    uint8_t i;
		HAL_GPIO_WritePin(SX1278_NSS_PORT, SX1278_NSS_PIN, GPIO_PIN_RESET);
		HW_SPI_InOut(hspi, addr & 0x7F );
		for( i = 0; i < size; i++ )
		{
			buffer[i] = HW_SPI_InOut(hspi, 0 );
		}
  	HAL_GPIO_WritePin(SX1278_NSS_PORT, SX1278_NSS_PIN, GPIO_PIN_SET);
}
/*
 Function: Reads the indicated register.
 Returns: The content of the register
 Parameters:
   address: address register to read from
*/
uint8_t readRegister(SPI_HandleTypeDef *hspi, uint8_t addr )
{
    uint8_t data;
    SX1272ReadBuffer(hspi, addr, &data, 1 );
    return data;
}

void SX1272WriteBuffer(SPI_HandleTypeDef *hspi, uint8_t addr, uint8_t *buffer, uint8_t size )
{
    uint8_t i;
    HAL_GPIO_WritePin(SX1278_NSS_PORT, SX1278_NSS_PIN, GPIO_PIN_RESET);
		HW_SPI_InOut(hspi, addr | 0x80 );
		for( i = 0; i < size; i++ )
			{
				HW_SPI_InOut(hspi, buffer[i] );
			}
    HAL_GPIO_WritePin(SX1278_NSS_PORT, SX1278_NSS_PIN, GPIO_PIN_SET);
}
/*
 Function: Writes on the indicated register.
 Returns: Nothing
 Parameters:
   address: address register to write in
   data : value to write in the register
*/
void writeRegister(SPI_HandleTypeDef *hspi, uint8_t addr, uint8_t data )
{
    SX1272WriteBuffer(hspi, addr, &data, 1 );
}
/*
 Function: It gets the theoretical value of the time-on-air of the packet
 Link: http://www.semtech.com/images/datasheet/sx1276.pdf
 Returns: Float that determines the time-on-air
*/
float timeOnAir( uint16_t payloadlength )
{
	float BW;
	float DE = 0;
	float SF = _spreadingFactor;
	float PL = payloadlength + OFFSET_PAYLOADLENGTH;
	float H = _header;
	float CR = _codingRate;

	// Dara rate optimization enabled if SF is 11 or 12
	if( SF > 10) DE = 1.0;
	else DE = 0.0;

	// payload correction
	if( payloadlength == 0 ) PL = 255;

	// Bandwidth value setting
	if( _bandwidth == BW_125 ) 		BW = 125.0;
	else if( _bandwidth == BW_250 ) BW = 250.0;
	else if( _bandwidth == BW_500 ) BW = 500.0;
	else BW = 125.0;

	// Calculation steps:
	float Tsym = pow(2,SF)/(BW); // ms
	float Tpreamble = (8+4.25)*Tsym;// ms
	float argument1 = ceil( (8.0*PL-4.0*SF+28.0+16.0-20.0*H)/(4.0*(SF-2.0*DE)) )*(CR+4.0);
	float argument2 = 0;
	float payloadSymbNb = 8 + (max( argument1, argument2));
	float Tpayload = payloadSymbNb * Tsym;
	float Tpacket = Tpreamble + Tpayload;

	return Tpacket;
}
/*
 Function: It sets a char array payload packet in a packet struct.
 Returns:  Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t setPayload(SPI_HandleTypeDef *hspi, char *payload)
{
	uint8_t state = 2;
	uint8_t state_f = 2;
	uint16_t length16;

	state = 1;
	length16 = (uint16_t)strlen(payload);
	state = truncPayload(length16);
	if( state == 0 )
	{
		// fill data field until the end of the string
		for(unsigned int i = 0; i < _payloadlength; i++)
		{
			packet_sent.data[i] = payload[i];
		}
	}
	else
	{
		state_f = state;
	}

	// In the case of FSK mode, the max payload is more restrictive
	if( ( _modem == FSK ) && ( _payloadlength > MAX_PAYLOAD_FSK ) )
	{
		_payloadlength = MAX_PAYLOAD_FSK;
		state = 1;
	}

	// Set length with the actual counter value
	// Setting packet length in packet structure
	state_f = setPacketLength_payloadlength(hspi);
	return state_f;
}
/*
 Function: If a packet is received, checks its destination.
 Returns: _Bool that's 'true' if the packet is for the module and
		  it's 'false' if the packet is not for the module.
 Parameters:
   wait: time to wait while there is no a valid header received.
*/
_Bool	availableData(SPI_HandleTypeDef *hspi, uint32_t wait)
{
	uint8_t value;
	uint8_t header = 0;
	_Bool forme = false;
	unsigned long previous;

	// update attribute
	_Bool _hreceived = false;

	previous = HAL_GetTick();
	if( _modem == LORA )
	{
		/// LoRa mode
		// read REG_IRQ_FLAGS
		value = readRegister(hspi,REG_IRQ_FLAGS);

		// Wait to ValidHeader interrupt in REG_IRQ_FLAGS
		while( (bitRead(value, 4) == 0) && (HAL_GetTick()-previous < (unsigned long)wait) )
		{
			// read REG_IRQ_FLAGS
			value = readRegister(hspi,REG_IRQ_FLAGS);

			// Condition to avoid an overflow (DO NOT REMOVE)
			if( HAL_GetTick() < previous )
			{
				previous = HAL_GetTick();
			}
		}

		// Check if ValidHeader was received
		if( bitRead(value, 4) == 1 )
		{
			_hreceived = true;
			while( (header == 0) && (HAL_GetTick()-previous < (unsigned long)wait) )
			{
				// Wait for the increment of the RX buffer pointer
				header = readRegister(hspi,REG_FIFO_RX_BYTE_ADDR);

				// Condition to avoid an overflow (DO NOT REMOVE)
				if( HAL_GetTick() < previous )
				{
					previous = HAL_GetTick();
				}
			}

			// If packet received: Read first uint8_t of the received packet
			if( header != 0 )
			{
				_destination = readRegister(hspi, REG_FIFO);
			}
		}
		else
		{
			forme = false;
			_hreceived = false;
		}
	}
	else
	{
		/// FSK mode
		// read REG_IRQ_FLAGS2
		value = readRegister(hspi, REG_IRQ_FLAGS2);
		// Wait to Payload Ready interrupt
		while( (bitRead(value, 2) == 0) && (HAL_GetTick() - previous < wait) )
		{
			value = readRegister(hspi, REG_IRQ_FLAGS2);
			// Condition to avoid an overflow (DO NOT REMOVE)
			if( HAL_GetTick() < previous )
			{
				previous = HAL_GetTick();
			}
		}// end while
		if( bitRead(value, 2) == 1 )	// something received
		{
			_hreceived = true;
			// Reading first uint8_t of the received packet
			_destination = readRegister(hspi, REG_FIFO);
		}
		else
		{
			forme = false;
			_hreceived = false;
		}
	}


	/* We use '_hreceived' because we need to ensure that '_destination' value
	 * is correctly updated and is not the '_destination' value from the
	 * previously packet
	 */
	if( _hreceived == true )
	{
		// Checking destination
		if( (_destination == _nodeAddress) || (_destination == BROADCAST_0) )
		{ // LoRa or FSK mode
			forme = true;
		}
		else
		{
			forme = false;

			// If it is not a correct destination address, then change to
			// STANDBY to minimize power consumption
			if( _modem == LORA )
			{
				// Setting standby LoRa mode
				writeRegister(hspi, REG_OP_MODE, LORA_STANDBY_MODE);
			}
			else
			{
				// Setting standby FSK mode
				writeRegister(hspi, REG_OP_MODE, FSK_STANDBY_MODE);
			}
		}
	}
	else
	{
		// If timeout has expired, then change to
		// STANDBY to minimize power consumption
		if( _modem == LORA )
		{
			// Setting standby LoRa mode
 			writeRegister(hspi, REG_OP_MODE, LORA_STANDBY_MODE);
		}
		else
		{
			// Setting standby FSK mode
			writeRegister(hspi, REG_OP_MODE, FSK_STANDBY_MODE);
		}
	}
	return forme;
}

/*
 Function: If a packet is received, checks its destination.
 Returns: _Bool that's 'true' if the packet is for the module and
		  it's 'false' if the packet is not for the module.
*/
_Bool	availableData_MAX_TIMEOUT(SPI_HandleTypeDef *hspi)
{
	return availableData(hspi, MAX_TIMEOUT);
}
/*
 * Function: Clears the interruption flags
 *
 * LoRa Configuration registers are accessed through the SPI interface.
 * Registers are readable in all device mode including Sleep. However, they
 * should be written only in Sleep and Stand-by modes.
 *
 * Returns: Nothing
*/
void clearFlags(SPI_HandleTypeDef *hspi)
{
  uint8_t st0;
	// Save the previous status
	st0 = readRegister(hspi, REG_OP_MODE);
	if( _modem == LORA )
	{
		/// LoRa mode
		writeRegister(hspi, REG_OP_MODE, LORA_STANDBY_MODE);// Stdby mode to write in registers
		writeRegister(hspi, REG_IRQ_FLAGS, 0xFF);// LoRa mode flags register
		writeRegister(hspi, REG_OP_MODE, st0);// Getting back to previous status
	}
	else
	{
		/// FSK mode
		writeRegister(hspi, REG_OP_MODE, FSK_STANDBY_MODE);	// Stdby mode to write in registers
		writeRegister(hspi, REG_IRQ_FLAGS1, 0xFF); // FSK mode flags1 register
		writeRegister(hspi, REG_IRQ_FLAGS2, 0xFF); // FSK mode flags2 register
		writeRegister(hspi, REG_OP_MODE, st0); // Getting back to previous status
	}
}
/*
 Function: It gets and stores an ACK if it is received, before ending 'wait' time.
 Returns: Integer that determines if there has been any error
   state = 8  --> The ACK lost
   state = 7  --> The ACK destination incorrectly received
   state = 6  --> The ACK source incorrectly received
   state = 5  --> The ACK number incorrectly received
   state = 4  --> The ACK length incorrectly received
   state = 3  --> N-ACK received
   state = 2  --> The ACK has not been received
   state = 1  --> not used (reserved)
   state = 0  --> The ACK has been received with no errors
 Parameters:
   wait: time to wait while there is no a valid header received.
*/
uint8_t getACK(SPI_HandleTypeDef *hspi, uint32_t wait)
{
	uint8_t state = 2;
	uint8_t value = 0x00;
	unsigned long previous;
	_Bool a_received = false;

  previous = HAL_GetTick();

	if( _modem == LORA )
	{ // LoRa mode
	    value = readRegister(hspi, REG_IRQ_FLAGS);
		// Wait until the ACK is received (RxDone flag) or the timeout expires
		while ((bitRead(value, 6) == 0) && (HAL_GetTick() - previous < wait))
		{
			value = readRegister(hspi, REG_IRQ_FLAGS);
			if( HAL_GetTick() < previous )
			{
				previous = HAL_GetTick();
			}
		}
		if( bitRead(value, 6) == 1 )
		{ // ACK received
			a_received = true;
		}
		// Standby para minimizar el consumo
		writeRegister(hspi, REG_OP_MODE, LORA_STANDBY_MODE);	// Setting standby LoRa mode
	}
	else
	{ // FSK mode
		value = readRegister(hspi, REG_IRQ_FLAGS2);
		// Wait until the packet is received (RxDone flag) or the timeout expires
		while ((bitRead(value, 2) == 0) && (HAL_GetTick() - previous < wait))
		{
			value = readRegister(hspi, REG_IRQ_FLAGS2);
			if( HAL_GetTick() < previous )
			{
				previous = HAL_GetTick();
			}
		}
		if( bitRead(value, 2) == 1 )
		{ // ACK received
			a_received = true;
		}
		// Standby para minimizar el consumo
		writeRegister(hspi, REG_OP_MODE, FSK_STANDBY_MODE);	// Setting standby FSK mode
	}

	if( a_received )
	{
//----	writeRegister(REG_FIFO_ADDR_PTR, 0x00);  // Setting address pointer in FIFO data buffer
		// Storing the received ACK
		ACK.dst = _destination;
		ACK.src = readRegister(hspi, REG_FIFO);
		ACK.packnum = readRegister(hspi, REG_FIFO);
		ACK.length = readRegister(hspi, REG_FIFO);
		ACK.data[0] = readRegister(hspi, REG_FIFO);

		// Checking the received ACK
		if( ACK.dst == packet_sent.src )
		{
			if( ACK.src == packet_sent.dst )
			{
				if( ACK.packnum == packet_sent.packnum )
				{
					if( ACK.length == 0 )
					{
						if( ACK.data[0] == CORRECT_PACKET )
						{
							state = 0;
						}
						else
						{
							state = 3;
						}
					}
					else
					{
						state = 4;
					}
				}
				else
				{
					state = 5;
				}
			}
			else
			{
				state = 6;
			}
		}
		else
		{
			state = 7;
		}
	}
	else
	{
		state = 8;
	}
	clearFlags(hspi);	// Initializing flags
	return state;
}
/*
 Function: It gets and stores an ACK if it is received.
 Returns:
*/
uint8_t getACK_MAX_TIMEOUT(SPI_HandleTypeDef *hspi)
{
	return getACK(hspi, MAX_TIMEOUT);
}
/*
 Function: Checks if BW is a valid value.
 Returns: _Bool that's 'true' if the BW value exists and
		  it's 'false' if the BW value does not exist.
 Parameters:
   band: bandwidth value to check.
*/
_Bool	isBW(uint16_t band)
{
  // Checking available values for _bandwidth
  switch(band)
  {
	  case BW_7_8:
	  case BW_10_4:
	  case BW_15_6:
	  case BW_20_8:
	  case BW_31_2:
	  case BW_41_7:
	  case BW_62_5:
	  case BW_125:
	  case BW_250:
	  case BW_500:	check = true; break;

	  default:		check = false;
  }
	return check;
}
/*
 Function: Gets the BW within the module is configured.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
   state = -1 --> Forbidden command for this protocol
*/
int8_t	getBW(SPI_HandleTypeDef *hspi)
{
  int8_t state = 2;
  uint8_t config1;

  if( _modem == FSK )
  {
	  state = -1;		// BW is not available in FSK mode
  }
  else
  {
	  // take out bits 7-4 from REG_MODEM_CONFIG1 indicates _bandwidth
	  config1 = (readRegister(hspi, REG_MODEM_CONFIG1)) >> 4;
	  _bandwidth = config1;

	  if( (config1 == _bandwidth) && isBW(_bandwidth) )
	  {
		  state = 0;
	  }
	  else
	  {
		  state = 1;
	  }
  }
  return state;
}
/*
 Function: Checks if CR is a valid value.
 Returns: _Bool that's 'true' if the CR value exists and
		  it's 'false' if the CR value does not exist.
 Parameters:
   cod: coding rate value to check.
*/
_Bool	isCR(uint8_t cod)
{
	// Checking available values for _codingRate
  switch(cod)
  {
	  case CR_5:
	  case CR_6:
	  case CR_7:
	  case CR_8:	check = true;
					break;

	  default:		check = false;
  }
	return check;
}
/*
 Function: Indicates the CR within the module is configured.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
   state = -1 --> Forbidden command for this protocol
*/
int8_t	getCR(SPI_HandleTypeDef *hspi)
{
  int8_t state = 2;
  uint8_t config1;

  if( _modem == FSK )
  {
	  state = -1;		// CR is not available in FSK mode
  }
  else
  {
	// take out bits 7-3 from REG_MODEM_CONFIG1 indicates _bandwidth & _codingRate
	config1 = (readRegister(hspi, REG_MODEM_CONFIG1)) >> 1;
	config1 = config1 & 7;	// clears bits 7-4 ---> clears _bandwidth
	_codingRate = config1;
	state = 1;

	if( (config1 == _codingRate) && isCR(_codingRate) )
	{
		state = 0;
	}
  }
  return state;
}
/*
 Function: Indicates if module is configured with or without checking CRC.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t	getCRC(SPI_HandleTypeDef *hspi)
{
	int8_t state = 2;
	uint8_t value;

	if( _modem == LORA )
	{ // LoRa mode

		// take out bit 2 from REG_MODEM_CONFIG2 indicates RxPayloadCrcOn
		value = readRegister(hspi, REG_MODEM_CONFIG2);
		if( bitRead(value, 2) == CRC_OFF )
		{ // CRCoff
			_CRC = CRC_OFF;
			state = 0;
		}
		else
		{ // CRCon
			_CRC = CRC_ON;
			state = 0;
		}
	}
	else
	{ // FSK mode

		// take out bit 2 from REG_PACKET_CONFIG1 indicates CrcOn
		value = readRegister(hspi, REG_PACKET_CONFIG1);
		if( bitRead(value, 4) == CRC_OFF )
		{ // CRCoff
			_CRC = CRC_OFF;
			state = 0;
		}
		else
		{ // CRCon
			_CRC = CRC_ON;
			state = 0;
		}
	}
	if( state != 0 )
	{
		state = 1;
	}
	return state;
}

/*
 Function: Checks if channel is a valid value.
 Returns: _Bool that's 'true' if the CR value exists and
		  it's 'false' if the CR value does not exist.
 Parameters:
   ch: frequency channel value to check.
*/
_Bool	isChannel(uint32_t ch)
{
  // Checking available values for _channel
  switch(ch)
  {
	  case CH_1_BW_500:check = true;break;
	  case CH_2_BW_500:check = true; break;// same value that CH_7_BW_125
	  case CH_3_BW_500:check = true;break;

	  case CH_1_BW_250:check = true;break;
	  case CH_2_BW_250:check = true;break;
	  case CH_3_BW_250:check = true;break;
	  case CH_4_BW_250:check = true;break;
	  case CH_5_BW_250:check = true;break;
	  case CH_6_BW_250:check = true;break;

	  case CH_1_BW_125:check = true;break;
	  case CH_2_BW_125:check = true;break;
	  case CH_3_BW_125:check = true;break;
	  case CH_4_BW_125:check = true;break;
	  case CH_5_BW_125:check = true;break;
	  case CH_6_BW_125:check = true;break;
	  case CH_8_BW_125:check = true;break;
	  case CH_9_BW_125:check = true;break;
	  case CH_10_BW_125:check = true;break;
	  case CH_11_BW_125:check = true;break;
	  case CH_12_BW_125:check = true;break;
	  case CH_13_BW_125:check = true;break;

	  case CH_1:check = true;break;
	  case CH_2:check = true;break;
	  case CH_3:check = true;break;
	  case CH_4:check = true;break;
	  case CH_5:check = true;break;
	  case CH_6:check = true;break;
	  case CH_7:check = true;break;
	  case CH_8:check = true;break;
	  case CH_9:check = true;break;
	  case CH_10:check = true;break;
	  case CH_11:check = true;break;
	  case CH_12:check = true;break;
	  case CH_13:check = true;break;
	  case CH_14:check = true;break;
	  case CH_15:check = true;break;
	  case CH_16:check = true;break;
	  case CH_17:check = true;break;
	  case CH_18:check = true;break;
	  case CH_19:check = true;break;
	  case CH_20:check = true;break;
	  case CH_21:check = true;break;
	  case CH_22:check = true;break;
	  case CH_23:check = true;break;
	  case CH_24:		check = true;
				break;

	  default:		check = false;
  }
	return check;
}
/*
 Function: Indicates the frequency channel within the module is configured.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t getChannel(SPI_HandleTypeDef *hspi)
{
  uint8_t state = 2;
  uint32_t ch;
  uint8_t freq3;
  uint8_t freq2;
  uint8_t freq1;

  freq3 = readRegister(hspi, REG_FRF_MSB);	// frequency channel MSB
  freq2 = readRegister(hspi, REG_FRF_MID);	// frequency channel MID
  freq1 = readRegister(hspi, REG_FRF_LSB);	// frequency channel LSB
  ch = ((uint32_t)freq3 << 16) + ((uint32_t)freq2 << 8) + (uint32_t)freq1;
  _channel = ch;						// frequency channel

  if( (_channel == ch) && isChannel(_channel) )
  {
	  state = 0;
  }
  else
  {
	  state = 1;
  }
  return state;
}
/*
 Function: Indicates if module is configured in implicit or explicit header mode.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t	getHeader(void)
{
	int8_t state = 2;

	// take out bit 2 from REG_MODEM_CONFIG1 indicates ImplicitHeaderModeOn
	if( bitRead(REG_MODEM_CONFIG1, 0) == 0 )
	{ // explicit header mode (ON)
		_header = HEADER_ON;
		state = 1;
	}
	else
	{ // implicit header mode (OFF)
		_header = HEADER_OFF;
		state = 1;
	}
	state = 0;
	return state;
}
/*
 Function: Gets the current supply limit of the power amplifier, protecting battery chemistries.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
 Parameters:
   rate: value to compute the maximum current supply. Maximum current is 45+5*'rate' [mA]
*/
uint8_t getMaxCurrent(SPI_HandleTypeDef *hspi)
{
	int8_t state = 2;
	uint8_t value;

	state = 1;
	_maxCurrent = readRegister(hspi, REG_OCP);

	// extract only the OcpTrim value from the OCP register
	_maxCurrent &= 31;

	if( _maxCurrent <= 15 )
	{
		value = (45 + (5 * _maxCurrent));
	}
	else if( _maxCurrent <= 27 )
	{
		value = (-30 + (10 * _maxCurrent));
	}
	else
	{
		value = 240;
	}

	_maxCurrent = value;
	state = 0;
	return state;
}
/*
 Function: Sets the module in LoRa mode.
 Returns:  Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t setLORA(SPI_HandleTypeDef *hspi)
{
    uint8_t state = 2;
    uint8_t st0;

	writeRegister(hspi, REG_OP_MODE, FSK_SLEEP_MODE);    // Sleep mode (mandatory to set LoRa mode)
	writeRegister(hspi, REG_OP_MODE, LORA_SLEEP_MODE);    // LoRa sleep mode
	writeRegister(hspi, REG_OP_MODE, LORA_STANDBY_MODE);	// LoRa standby mode

	writeRegister(hspi, REG_MAX_PAYLOAD_LENGTH,MAX_LENGTH);

	// Set RegModemConfig1 to Default values
	writeRegister(hspi, REG_MODEM_CONFIG1, 0x72);
	// Set RegModemConfig2 to Default values
	writeRegister(hspi, REG_MODEM_CONFIG2, 0x70);
	// Set RegModemConfig2 to Default values
	writeRegister(hspi, REG_MODEM_CONFIG3, 0x00);

	//delay(100);

	st0 = readRegister(hspi, REG_OP_MODE);	// Reading config mode
	if( st0 == LORA_STANDBY_MODE )
	{ // LoRa mode
		_modem = LORA;
		state = 0;
	}
	else
	{ // FSK mode
		_modem = FSK;
		state = 1;
	}
	return state;
}
/*
 Function: Limits the current supply of the power amplifier, protecting battery chemistries.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
   state = -1 --> Forbidden parameter value for this function
 Parameters:
   rate: value to compute the maximum current supply. Range: 0x00 to 0x1B. The
   Maximum current is:
	Imax = 45+5*OcpTrim [mA] 	if OcpTrim <= 15 (120 mA) /
	Imax = -30+10*OcpTrim [mA] 	if 15 < OcpTrim <= 27 (130 to 240 mA)
	Imax = 240mA 				for higher settings
*/
int8_t setMaxCurrent(SPI_HandleTypeDef *hspi, uint8_t rate)
{
	int8_t state = 2;
	uint8_t st0;

	// Maximum rate value = 0x1B, because maximum current supply = 240 mA
	if (rate > 0x1B)
	{
		state = -1;
	}
	else
	{
		// Enable Over Current Protection
		rate |= 32;

		state = 1;
		st0 = readRegister(hspi, REG_OP_MODE);	// Save the previous status
		if( _modem == LORA )
		{ // LoRa mode
			writeRegister(hspi, REG_OP_MODE, LORA_STANDBY_MODE);	// Set LoRa Standby mode to write in registers
		}
		else
		{ // FSK mode
			writeRegister(hspi, REG_OP_MODE, FSK_STANDBY_MODE);	// Set FSK Standby mode to write in registers
		}
		writeRegister(hspi, REG_OCP, rate);		// Modifying maximum current supply
		writeRegister(hspi, REG_OP_MODE, st0);		// Getting back to previous status
		state = 0;
	}
	return state;
}
/*
 Function: Sets the node address in the module.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
   state = -1 --> Forbidden command for this protocol
 Parameters:
   addr: address value to set as node address.
*/
int8_t setNodeAddress(SPI_HandleTypeDef *hspi, uint8_t addr)
{
	uint8_t st0;
	int8_t value;
	int8_t state = 2;

	// check address value is within valid range
	if( addr > 255 )
	{
		state = -1;
	}
	else
	{
		// Saving node address
		_nodeAddress = addr;
		st0 = readRegister(hspi, REG_OP_MODE);	  // Save the previous status

		// in LoRa mode
		state = 0;

		if( _modem == LORA )
		{
			// in LoRa mode, address is SW controlled
			// set status to success
			state = 0;
		}
		else if( _modem == FSK )
		{
			//Set FSK Standby mode to write in registers
			writeRegister(hspi, REG_OP_MODE, FSK_STANDBY_MODE);

			// Storing node and broadcast address
			writeRegister(hspi, REG_NODE_ADRS, addr);
			writeRegister(hspi, REG_BROADCAST_ADRS, BROADCAST_0);

			value = readRegister(hspi, REG_NODE_ADRS);
			writeRegister(hspi, REG_OP_MODE, st0);		// Getting back to previous status

			if( value == _nodeAddress )
			{
				state = 0;
			}
			else
			{
				state = 1;
			}
		}
	}
	return state;
}
/*
 Function: It sets the packet destination.
 Returns:  Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
 Parameters:
   dest: destination value of the packet sent.
*/
int8_t setDestination(uint8_t dest)
{
	int8_t state = 2;

	state = 1;
	_destination = dest; // Storing destination in a global variable
	packet_sent.dst = dest;	 // Setting destination in packet structure
	packet_sent.src = _nodeAddress; // Setting source in packet structure
	packet_sent.packnum = _packetNumber;	// Setting packet number in packet structure
	_packetNumber++;
	state = 0;
	return state;
}
/*
 Function: It truncs the payload length if it is greater than 0xFF.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t truncPayload(uint16_t length16)
{
	uint8_t state = 2;
	state = 1;

	if( length16 > MAX_PAYLOAD)
	{
		_payloadlength = MAX_PAYLOAD;
	}
	else
	{
		_payloadlength = (length16 & 0xFF);
	}
	state = 0;
	return state;
}
/*
 Function: Sets the packet length in the module.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
   state = -1 --> Forbidden command for this protocol
 Parameters:
   l: length value to set as payload length.
*/
int8_t setPacketLength(SPI_HandleTypeDef *hspi, uint8_t l)
{
	uint8_t st0;
	uint8_t value = 0x00;
	int8_t state = 2;

	st0 = readRegister(hspi, REG_OP_MODE);	// Save the previous status
	//----
	//	truncPayload(l);
	packet_sent.length = l;
	//
	if( _modem == LORA )
  	{ // LORA mode
  		writeRegister(hspi, REG_OP_MODE, LORA_STANDBY_MODE);    // Set LoRa Standby mode to write in registers
		writeRegister(hspi, REG_PAYLOAD_LENGTH_LORA, packet_sent.length);
		// Storing payload length in LoRa mode
		value = readRegister(hspi, REG_PAYLOAD_LENGTH_LORA);
	}
	else
	{ // FSK mode
		writeRegister(hspi, REG_OP_MODE, FSK_STANDBY_MODE);    //  Set FSK Standby mode to write in registers
		writeRegister(hspi, REG_PAYLOAD_LENGTH_FSK, packet_sent.length);
		// Storing payload length in FSK mode
		value = readRegister(hspi, REG_PAYLOAD_LENGTH_FSK);
	}

	if( packet_sent.length == value )
	{
		state = 0;
	}
	else
	{
		state = 1;
	}
	writeRegister(hspi, REG_OP_MODE, st0);	// Getting back to previous status
  	//delay(250);
	return state;
}
/*
 Function: Sets the packet length in the module.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
   state = -1 --> Forbidden command for this protocol
*/
int8_t setPacketLength_payloadlength(SPI_HandleTypeDef *hspi)
{
	uint16_t length;
	length = _payloadlength + OFFSET_PAYLOADLENGTH;
	return setPacketLength(hspi, length);
}
/*
 Function: It sets a packet struct in FIFO in order to send it.
 Returns:  Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t setPacket(SPI_HandleTypeDef *hspi, uint8_t dest, char *payload)
{
	int8_t state = 2;
	uint8_t st0;

	// Save the previous status
	st0 = readRegister(hspi, REG_OP_MODE);
	// Initializing flags
	clearFlags(hspi);

	// Updating incorrect value
	_reception = CORRECT_PACKET;


	if (_retries == 0)
	{
		// Updating these values only if it is the first try
		// Setting destination in packet structure
		state = setDestination(dest);
		if( state == 0 )
		{
			state = setPayload(hspi, payload);
		}
	}
	else
	{
		state = setPacketLength_payloadlength(hspi);
		packet_sent.retry = _retries;
	}

	// Setting address pointer in FIFO data buffer
	writeRegister(hspi, REG_FIFO_TX_BASE_ADDR, 0x00);
	writeRegister(hspi, REG_FIFO_ADDR_PTR, 0x00);
	if( state == 0 )
	{
		state = 1;
		// Writing packet to send in FIFO
		writeRegister(hspi, REG_FIFO, packet_sent.dst); 		// Writing the destination in FIFO
		writeRegister(hspi, REG_FIFO, packet_sent.src);		// Writing the source in FIFO
		writeRegister(hspi, REG_FIFO, packet_sent.packnum);	// Writing the packet number in FIFO
		writeRegister(hspi, REG_FIFO, packet_sent.length); 	// Writing the packet length in FIFO
		for( uint16_t i = 0; i < _payloadlength; i++)
		{
			writeRegister(hspi, REG_FIFO, packet_sent.data[i]);  // Writing the payload in FIFO
		}
		writeRegister(hspi, REG_FIFO, packet_sent.retry);		// Writing the number retry in FIFO
		state = 0;
	}
	writeRegister(hspi, REG_OP_MODE, st0);	// Getting back to previous status
	return state;
}
/*
 Function: Checks if SF is a valid value.
 Returns: _Bool that's 'true' if the SF value exists and
		  it's 'false' if the SF value does not exist.
 Parameters:
   spr: spreading factor value to check.
*/
_Bool	isSF(uint8_t spr)
{
  // Checking available values for _spreadingFactor
  switch(spr)
  {
	  case SF_6:
	  case SF_7:
	  case SF_8:
	  case SF_9:
	  case SF_10:
	  case SF_11:
	  case SF_12:	check = true;
					break;

	  default:		check = false;
  }
	return check;
}
/*
 Function: Sets the module in implicit header mode (header is not sent).
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
   state = -1 --> Forbidden command for this protocol
*/
int8_t	setHeaderOFF(SPI_HandleTypeDef *hspi)
{
	int8_t state = 2;
	uint8_t config1;

	if( _modem == FSK )
	{
		// header is not available in FSK mode
		state = -1;
	}
	else
	{
		// lay du gia tri thanh ghi REG_MODEM_CONFIG1
		config1 = readRegister(hspi, REG_MODEM_CONFIG1);

		// dat bit 2 = 1 REG_MODEM_CONFIG1 <=> headerOFF
		config1 = config1 | 1;
		//cap nhat lai thanh ghi
		writeRegister(hspi, REG_MODEM_CONFIG1,config1);

		// check register
		config1 = readRegister(hspi, REG_MODEM_CONFIG1);
		if( bitRead(config1, 2) == HEADER_OFF )
		{
			// checking headerOFF taking out bit 2 from REG_MODEM_CONFIG1
			state = 0;
			_header = HEADER_OFF;
		}
		else
		{
			state = 1;
		}
	}
	return state;
}
/*
 Function: Sets the indicated SF in the module.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
 Parameters:
   spr: spreading factor value to set in LoRa modem configuration.
*/
uint8_t	setSF(SPI_HandleTypeDef *hspi, uint8_t spr)
{
	uint8_t st0;
	int8_t state = 2;
	uint8_t config2;
	uint8_t config3;

	st0 = readRegister(hspi, REG_OP_MODE);	// Save the previous status

	if( _modem == FSK )
	{
		/// FSK mode
		state = setLORA(hspi);				// Setting LoRa mode
	}
	else
	{
		/// LoRa mode
		// LoRa standby mode
		writeRegister(hspi, REG_OP_MODE, LORA_STANDBY_MODE);

		// Read config2 to modify SF value (bits 7-4)
		config2 = (readRegister(hspi, REG_MODEM_CONFIG2));
		// Read config3 to modify only the LowDataRateOptimize
		config3 = (readRegister(hspi, REG_MODEM_CONFIG3));

		switch(spr)
		{
			case SF_6:
					config2 = config2 & 111;	// clears bits 7 & 4 from REG_MODEM_CONFIG2
					config2 = config2 | 96;	// sets bits 6 & 5 from REG_MODEM_CONFIG2
					break;
			case SF_7:
					config2 = config2 & 127;	// clears bits 7 from REG_MODEM_CONFIG2
					config2 = config2 | 112;	// sets bits 6, 5 & 4
					break;

			case SF_8:
					config2 = config2 & 143;	// clears bits 6, 5 & 4 from REG_MODEM_CONFIG2
					config2 = config2 | 128;	// sets bit 7 from REG_MODEM_CONFIG2
					break;

			case SF_9:
					config2 = config2 & 159;	// clears bits 6, 5 & 4 from REG_MODEM_CONFIG2
					config2 = config2 | 144;	// sets bits 7 & 4 from REG_MODEM_CONFIG2
					break;

			case SF_10:	config2 = config2 & 175;	// clears bits 6 & 4 from REG_MODEM_CONFIG2
					config2 = config2 | 160;	// sets bits 7 & 5 from REG_MODEM_CONFIG2
					break;

			case SF_11:
					config2 = config2 & 191;	// clears bit 6 from REG_MODEM_CONFIG2
					config2 = config2 | 176;	// sets bits 7, 5 & 4 from REG_MODEM_CONFIG2
					getBW(hspi);
					if( _bandwidth == BW_125 )
					{ // LowDataRateOptimize (Mandatory with SF_11 if BW_125)
						config3 = config3 | 8;
					}
					break;

			case SF_12:
					config2 = config2 & 207;	// clears bits 5 & 4 from REG_MODEM_CONFIG2
					config2 = config2 | 192;	// sets bits 7 & 6 from REG_MODEM_CONFIG2

					getBW(hspi);
					if( _bandwidth == BW_125 )
					{ // LowDataRateOptimize (Mandatory with SF_12 if BW_125)
						config3 = config3 | 8;
					}
					break;
	}

	// Check if it is neccesary to set special settings for SF=6
	if( spr == SF_6 )
	{
		// Mandatory headerOFF with SF = 6 (Implicit mode)
		setHeaderOFF(hspi);

		// Set the bit field DetectionOptimize of
		// register RegLoRaDetectOptimize to value "0b101".
		writeRegister(hspi, REG_DETECT_OPTIMIZE, 0x05);

		// Write 0x0C in the register RegDetectionThreshold.
		writeRegister(hspi, REG_DETECTION_THRESHOLD, 0x0C);
	}
	else
	{
		// LoRa detection Optimize: 0x03 --> SF7 to SF12
		writeRegister(hspi, REG_DETECT_OPTIMIZE, 0x03);

		// LoRa detection threshold: 0x0A --> SF7 to SF12
		writeRegister(hspi, REG_DETECTION_THRESHOLD, 0x0A);
	}

	// sets bit 1-0 of REG_MODEM_CONFIG2 (SymbTimout) and bit 2 of REG_MODEM_CONFIG3 (AgcAutoOn) for any SF value
	config2 = config2 | 3;
	config3 = config3 | 4;

	// Update 'config2' and 'config3'
	writeRegister(hspi, REG_MODEM_CONFIG2, config2);
	writeRegister(hspi, REG_MODEM_CONFIG3, config3);

	// Read 'config2' and 'config3' to check update
	config2 = (readRegister(hspi, REG_MODEM_CONFIG2));
	config3 = (readRegister(hspi, REG_MODEM_CONFIG3));

	// (config2 >> 4) ---> take out bits 7-4 from REG_MODEM_CONFIG2 (=_spreadingFactor)
	// bitRead(config3, 3) ---> take out bits 1 from config3 (=LowDataRateOptimize)
	switch(spr)
	{
		case SF_6:	if(		((config2 >> 4) == spr)
						&& 	(bitRead(config3, 2) == 1)
						&& 	(_header == HEADER_OFF))
					{
						state = 0;
					}
					break;
		case SF_7:	if(		((config2 >> 4) == 0x07)
						 && (bitRead(config3, 2) == 1))
					{
						state = 0;
					}
					break;
		case SF_8:	if(		((config2 >> 4) == 0x08)
						 && (bitRead(config3, 2) == 1))
					{
						state = 0;
					}
					break;
		case SF_9:	if(		((config2 >> 4) == 0x09)
						 && (bitRead(config3, 2) == 1))
					{
						state = 0;
					}
					break;
		case SF_10:	if(		((config2 >> 4) == 0x0A)
						 && (bitRead(config3, 2) == 1))
					{
						state = 0;
					}
					break;
		case SF_11:	getBW(hspi);
				if(		((config2 >> 4) == 0x0B)
						 && (bitRead(config3, 2) == 1)
						 && (_bandwidth != BW_125))
					{
						state = 0;
					}
				else if		((_bandwidth == BW_125)
						 && (bitRead(config3, 3) == 1))
					{
						state = 0;
					}
					break;
		case SF_12:	getBW(hspi);
				if(		((config2 >> 4) == 0x0C)
						 && (bitRead(config3, 2) == 1)
						 && (_bandwidth != BW_125))
					{
						state = 0;
					}
				else if		((_bandwidth == BW_125)
						 && (bitRead(config3, 3) == 1))
					{
						state = 0;
					}
					break;
		default:	state = 1;
	}
  }

  writeRegister(hspi, REG_OP_MODE, st0);	// Getting back to previous status

  if( isSF(spr) )
  { // Checking available value for _spreadingFactor
		state = 0;
		_spreadingFactor = spr;
  }
  return state;
}
/*
 Function: It sets an ACK in FIFO in order to send it.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t setACK(SPI_HandleTypeDef *hspi)
{
	uint8_t state = 2;


	clearFlags(hspi);	// Initializing flags

	if( _modem == LORA )
	{ // LoRa mode
		writeRegister(hspi, REG_OP_MODE, LORA_STANDBY_MODE);	// Stdby LoRa mode to write in FIFO
	}
	else
	{ // FSK mode
		writeRegister(hspi, REG_OP_MODE, FSK_STANDBY_MODE);	// Stdby FSK mode to write in FIFO
	}

	// Setting ACK length in order to send it
	state = setPacketLength(hspi, ACK_LENGTH);
	if( state == 0 )
	{
		// Setting ACK
		memset( &ACK, 0x00, sizeof(ACK) );
		ACK.dst = packet_received.src; // ACK destination is packet source
		ACK.src = packet_received.dst; // ACK source is packet destination
		ACK.packnum = packet_received.packnum; // packet number that has been correctly received
		ACK.length = 0;		  // length = 0 to show that's an ACK
		ACK.data[0] = _reception;	// CRC of the received packet

		// Setting address pointer in FIFO data buffer
		writeRegister(hspi, REG_FIFO_ADDR_PTR, 0x00);
		writeRegister(hspi, REG_FIFO_TX_BASE_ADDR, 0x00);

		state = 1;

		// Writing ACK to send in FIFO
		writeRegister(hspi, REG_FIFO, ACK.dst); 		// Writing the destination in FIFO
		writeRegister(hspi, REG_FIFO, ACK.src);		// Writing the source in FIFO
		writeRegister(hspi, REG_FIFO, ACK.packnum);	// Writing the packet number in FIFO
		writeRegister(hspi, REG_FIFO, ACK.length); 	// Writing the packet length in FIFO
		writeRegister(hspi, REG_FIFO, ACK.data[0]);	// Writing the ACK in FIFO

		state = 0;
		_reception = CORRECT_PACKET;		// Updating value to next packet

		HAL_Delay(500);
	}
	return state;
}
/*
 Function: Gets the SF within the module is configured.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
   state = -1 --> Forbidden command for this protocol
*/
int8_t	getSF(SPI_HandleTypeDef *hspi)
{
  int8_t state = 2;
  uint8_t config2;

  if( _modem == FSK )
  {
	  state = -1;		// SF is not available in FSK mode
  }
  else
  {
	// take out bits 7-4 from REG_MODEM_CONFIG2 indicates _spreadingFactor
	config2 = (readRegister(hspi, REG_MODEM_CONFIG2)) >> 4;
	_spreadingFactor = config2;
	state = 1;

	if( (config2 == _spreadingFactor) && isSF(_spreadingFactor) )
	{
		state = 0;
	}
  }
  return state;
}
/*
 Function: Sets the indicated BW in the module.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
 Parameters:
   band: bandwith value to set in LoRa modem configuration.
*/
int8_t	setBW(SPI_HandleTypeDef *hspi, uint16_t band)
{
  uint8_t st0;
  int8_t state = 2;
  uint8_t config1;
  uint8_t config3;

  st0 = readRegister(hspi, REG_OP_MODE);	// Save the previous status

  if( _modem == FSK )
  {
	  state = setLORA(hspi);
  }
  writeRegister(hspi, REG_OP_MODE, LORA_STANDBY_MODE);	// LoRa standby mode
  config1 = (readRegister(hspi, REG_MODEM_CONFIG1));	// Save config1 to modify the BW
  config3 = (readRegister(hspi, REG_MODEM_CONFIG3));	// Save config3 to modify the Low Data Rate Optimization
  switch(band)
  {
	case BW_7_8:  	config1 = config1 & 15;	// clears bits 7-4 from REG_MODEM_CONFIG1
			config1 = config1 | 0;	//sets bit 7-4 from REG_MODEM_CONFIG1
			getSF(hspi);
			if( _spreadingFactor == 11 )
			{ // LowDataRateOptimize (Mandatory with BW_125 if SF_11)
				config3 = config3 | 8;
			}
			if( _spreadingFactor == 12 )
			{ // LowDataRateOptimize (Mandatory with BW_125 if SF_12)
				config3 = config3 | 8;
			}
			break;
	  case BW_10_4: config1 = config1 & 31;	// clears bits 7-4 from REG_MODEM_CONFIG1
			config1 = config1 | 16;	//sets bit 7-4 from REG_MODEM_CONFIG1
			getSF(hspi);
			if( _spreadingFactor == 11 )
			{ // LowDataRateOptimize (Mandatory with BW_125 if SF_11)
				config3 = config3 | 8;
			}
			if( _spreadingFactor == 12 )
			{ // LowDataRateOptimize (Mandatory with BW_125 if SF_12)
				config3 = config3 | 8;
			}
			break;
	  case BW_15_6: config1 = config1 & 47;	// clears bits 7-4 from REG_MODEM_CONFIG1
			config1 = config1 | 32;	//sets bit 7-4 from REG_MODEM_CONFIG1
			getSF(hspi);
			if( _spreadingFactor == 11 )
			{ // LowDataRateOptimize (Mandatory with BW_125 if SF_11)
				config3 = config3 | 8;
			}
			if( _spreadingFactor == 12 )
			{ // LowDataRateOptimize (Mandatory with BW_125 if SF_12)
				config3 = config3 | 8;
			}
			break;
	  case BW_20_8: config1 = config1 & 8;	// clears bits 7-4 from REG_MODEM_CONFIG1
			config1 = config1 | 48;	//sets bit 7-4 from REG_MODEM_CONFIG1
			getSF(hspi);
			if( _spreadingFactor == 11 )
			{ // LowDataRateOptimize (Mandatory with BW_125 if SF_11)
				config3 = config3 | 8;
			}
			if( _spreadingFactor == 12 )
			{ // LowDataRateOptimize (Mandatory with BW_125 if SF_12)
				config3 = config3 | 8;
			}
			break;
	  case BW_31_2: config1 = config1 & 79;	// clears bits 7-4 from REG_MODEM_CONFIG1
			config1 = config1 | 64;	//sets bit 7-4 from REG_MODEM_CONFIG1
			getSF(hspi);
			if( _spreadingFactor == 11 )
			{ // LowDataRateOptimize (Mandatory with BW_125 if SF_11)
				config3 = config3 | 8;
			}
			if( _spreadingFactor == 12 )
			{ // LowDataRateOptimize (Mandatory with BW_125 if SF_12)
				config3 = config3 | 8;
			}
			break;
	  case BW_41_7: config1 = config1 & 95;	// clears bits 7-4 from REG_MODEM_CONFIG1
			config1 = config1 | 80;	//sets bit 7-4 from REG_MODEM_CONFIG1
			getSF(hspi);
			if( _spreadingFactor == 11 )
			{ // LowDataRateOptimize (Mandatory with BW_125 if SF_11)
				config3 = config3 | 8;
			}
			if( _spreadingFactor == 12 )
			{ // LowDataRateOptimize (Mandatory with BW_125 if SF_12)
				config3 = config3 | 8;
			}
			break;
	  case BW_62_5: config1 = config1 & 111;	// clears bits 7-4 from REG_MODEM_CONFIG1
			config1 = config1 | 96;	//sets bit 7-4 from REG_MODEM_CONFIG1
			getSF(hspi);
			if( _spreadingFactor == 11 )
			{ // LowDataRateOptimize (Mandatory with BW_125 if SF_11)
				config3 = config3 | 8;
			}
			if( _spreadingFactor == 12 )
			{ // LowDataRateOptimize (Mandatory with BW_125 if SF_12)
				config3 = config3 | 8;
			}
			break;
	  case BW_125:  config1 = config1 & 127;	// clears bits 7-4 from REG_MODEM_CONFIG1
			config1 = config1 | 112;	//sets bit 7-4 from REG_MODEM_CONFIG1
			getSF(hspi);
			if( _spreadingFactor == 11 )
			{ // LowDataRateOptimize (Mandatory with BW_125 if SF_11)
				config3 = config3 | 8;
			}
			if( _spreadingFactor == 12 )
			{ // LowDataRateOptimize (Mandatory with BW_125 if SF_12)
				config3 = config3 | 8;
			}
			break;
	  case BW_250:  config1 = config1 & 143;	// clears bit 7 from REG_MODEM_CONFIG1
			config1 = config1 | 128;	// sets bit 6 from REG_MODEM_CONFIG1
			break;
	  case BW_500:  config1 = config1 & 159;	//clears bit 6 from REG_MODEM_CONFIG1
			config1 = config1 | 144;	//sets bit 7 from REG_MODEM_CONFIG1
			break;

  }
  writeRegister(hspi, REG_MODEM_CONFIG1,config1);		// Update config1
  writeRegister(hspi, REG_MODEM_CONFIG3,config3);		// Update config3

  config1 = (readRegister(hspi, REG_MODEM_CONFIG1));
  config3 = (readRegister(hspi, REG_MODEM_CONFIG3));
  // (config1 >> 4) ---> take out bits 7-4 from REG_MODEM_CONFIG1 (=_bandwidth)
  switch(band)
  {
	   case BW_7_8: if( (config1 >> 4) == BW_125 )
					{
						state = 0;
						getSF(hspi);
						if( _spreadingFactor == 11 )
						{
							if( bitRead(config3, 3) == 1 )
							{ // LowDataRateOptimize
								state = 0;
							}
							else
							{
								state = 1;
							}
						}
						if( _spreadingFactor == 12 )
						{
							if( bitRead(config3, 3) == 1 )
							{ // LowDataRateOptimize
								state = 0;
							}
							else
							{
								state = 1;
							}
						}
					}
					break;
	   case BW_10_4: if( (config1 >> 4) == BW_125 )
					{
						state = 0;
						getSF(hspi);
						if( _spreadingFactor == 11 )
						{
							if( bitRead(config3, 3) == 1 )
							{ // LowDataRateOptimize
								state = 0;
							}
							else
							{
								state = 1;
							}
						}
						if( _spreadingFactor == 12 )
						{
							if( bitRead(config3, 3) == 1 )
							{ // LowDataRateOptimize
								state = 0;
							}
							else
							{
								state = 1;
							}
						}
					}
					break;
	   case BW_15_6: if( (config1 >> 4) == BW_125 )
					{
						state = 0;
						getSF(hspi);
						if( _spreadingFactor == 11 )
						{
							if( bitRead(config3, 3) == 1 )
							{ // LowDataRateOptimize
								state = 0;
							}
							else
							{
								state = 1;
							}
						}
						if( _spreadingFactor == 12 )
						{
							if( bitRead(config3, 3) == 1 )
							{ // LowDataRateOptimize
								state = 0;
							}
							else
							{
								state = 1;
							}
						}
					}
					break;
	   case BW_20_8: if( (config1 >> 4) == BW_125 )
					{
						state = 0;
						getSF(hspi);
						if( _spreadingFactor == 11 )
						{
							if( bitRead(config3, 3) == 1 )
							{ // LowDataRateOptimize
								state = 0;
							}
							else
							{
								state = 1;
							}
						}
						if( _spreadingFactor == 12 )
						{
							if( bitRead(config3, 3) == 1 )
							{ // LowDataRateOptimize
								state = 0;
							}
							else
							{
								state = 1;
							}
						}
					}
					break;
	   case BW_31_2: if( (config1 >> 4) == BW_125 )
					{
						state = 0;
						getSF(hspi);
						if( _spreadingFactor == 11 )
						{
							if( bitRead(config3, 3) == 1 )
							{ // LowDataRateOptimize
								state = 0;
							}
							else
							{
								state = 1;
							}
						}
						if( _spreadingFactor == 12 )
						{
							if( bitRead(config3, 3) == 1 )
							{ // LowDataRateOptimize
								state = 0;
							}
							else
							{
								state = 1;
							}
						}
					}
					break;
	   case BW_41_7: if( (config1 >> 4) == BW_125 )
					{
						state = 0;
						getSF(hspi);
						if( _spreadingFactor == 11 )
						{
							if( bitRead(config3, 3) == 1 )
							{ // LowDataRateOptimize
								state = 0;
							}
							else
							{
								state = 1;
							}
						}
						if( _spreadingFactor == 12 )
						{
							if( bitRead(config3, 3) == 1 )
							{ // LowDataRateOptimize
								state = 0;
							}
							else
							{
								state = 1;
							}
						}
					}
					break;
	   case BW_62_5: if( (config1 >> 4) == BW_125 )
					{
						state = 0;
						getSF(hspi);
						if( _spreadingFactor == 11 )
						{
							if( bitRead(config3, 3) == 1 )
							{ // LowDataRateOptimize
								state = 0;
							}
							else
							{
								state = 1;
							}
						}
						if( _spreadingFactor == 12 )
						{
							if( bitRead(config3, 3) == 1 )
							{ // LowDataRateOptimize
								state = 0;
							}
							else
							{
								state = 1;
							}
						}
					}
					break;
	   case BW_125: if( (config1 >> 4) == BW_125 )
					{
						state = 0;
						getSF(hspi);
						if( _spreadingFactor == 11 )
						{
							if( bitRead(config3, 3) == 1 )
							{ // LowDataRateOptimize
								state = 0;
							}
							else
							{
								state = 1;
							}
						}
						if( _spreadingFactor == 12 )
						{
							if( bitRead(config3, 3) == 1 )
							{ // LowDataRateOptimize
								state = 0;
							}
							else
							{
								state = 1;
							}
						}
					}
					break;
	   case BW_250: if( (config1 >> 4) == BW_250 )
					{
						state = 0;
					}
					break;
	   case BW_500: if( (config1 >> 4) == BW_500 )
					{
						state = 0;
					}
					break;
  }

  if( ~isBW(band) )
  {
	  state = 1;
  }
  else
  {
	  _bandwidth = band;
  }
  writeRegister(hspi, REG_OP_MODE, st0);	// Getting back to previous status
  return state;
}
/*
 Function: Sets the indicated CR in the module.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
   state = -1 --> Forbidden command for this protocol
 Parameters:
   cod: coding rate value to set in LoRa modem configuration.
*/
int8_t	setCR(SPI_HandleTypeDef *hspi, uint8_t cod)
{
  uint8_t st0;
  int8_t state = 2;
  uint8_t config1;

  st0 = readRegister(hspi, REG_OP_MODE);		// Save the previous status

  if( _modem == FSK )
  {
	  state = setLORA(hspi);
  }
  else
  {
	  writeRegister(hspi, REG_OP_MODE, LORA_STANDBY_MODE);		// Set Standby mode to write in registers

	  config1 = readRegister(hspi, REG_MODEM_CONFIG1);	// Save config1 to modify only the CR
	  switch(cod)
	  {
		 case CR_5: config1 = config1 & 243;	// clears bits 3 & 2 from REG_MODEM_CONFIG1
			    config1 = config1 | 2;	// sets bit 1 from REG_MODEM_CONFIG1
			    break;
		 case CR_6: config1 = config1 & 245;	// clears bits 3 & 1 from REG_MODEM_CONFIG1
			    config1 = config1 | 4;	// sets bit 2 from REG_MODEM_CONFIG1
			    break;
		 case CR_7: config1 = config1 & 247;	// clears bit 3 from REG_MODEM_CONFIG1
			    config1 = config1 | 6;	// sets bits 2 & 1 from REG_MODEM_CONFIG1
			    break;
		 case CR_8: config1 = config1 & 249;	// clears bits 2 & 1 from REG_MODEM_CONFIG1
			    config1 = config1 | 8;	// sets bit 3 from REG_MODEM_CONFIG1
			    break;
	  }
	  writeRegister(hspi, REG_MODEM_CONFIG1, config1);		// Update config1

	  config1 = readRegister(hspi, REG_MODEM_CONFIG1);
	  // ((config1 >> 3) & B0000111) ---> take out bits 5-3 from REG_MODEM_CONFIG1 (=_codingRate)
	  switch(cod)
	  {
		 case CR_5: if( ((config1 >> 1) & 7) == 0x01 )
					{
						state = 0;
					}
					break;
		 case CR_6: if( ((config1 >> 1) & 7) == 0x02 )
					{
						state = 0;
					}
					break;
		 case CR_7: if( ((config1 >> 1) & 7) == 0x03 )
					{
						state = 0;
					}
					break;
		 case CR_8: if( ((config1 >> 1) & 7) == 0x04 )
					{
						state = 0;
					}
					break;
	  }
  }

  if( isCR(cod) )
  {
	  _codingRate = cod;
  }
  else
  {
	  state = 1;
  }
  writeRegister(hspi, REG_OP_MODE,st0);	// Getting back to previous status
  return state;
}
/*
 Function: Sets the module with CRC on.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t	setCRC_ON(SPI_HandleTypeDef *hspi)
{
  uint8_t state = 2;
  uint8_t config;

  if( _modem == LORA )
  { // LORA mode
	config = readRegister(hspi, REG_MODEM_CONFIG2);	// Save config to modify only the CRC bit
	config = config | 4;				// sets bit 2 from REG_MODEM_CONFIG2 = CRC_ON
	writeRegister(hspi, REG_MODEM_CONFIG2,config);

	state = 1;

	config = readRegister(hspi, REG_MODEM_CONFIG2);
	if( bitRead(config, 2) == CRC_ON )
	{ // take out bit 1 from REG_MODEM_CONFIG2 indicates RxPayloadCrcOn
		state = 0;
		_CRC = CRC_ON;
	}
  }
  else
  { // FSK mode
	config = readRegister(hspi, REG_PACKET_CONFIG1);	// Save config to modify only the CRC bit
	config = config | 16;				// set bit 4 from REG_PACKET_CONFIG1 = CRC_ON
	writeRegister(hspi, REG_PACKET_CONFIG1,config);

	state = 1;

	config = readRegister(hspi, REG_PACKET_CONFIG1);
	if( bitRead(config, 4) == CRC_ON )
	{ // take out bit 4 from REG_PACKET_CONFIG1 indicates CrcOn
		state = 0;
		_CRC = CRC_ON;
	}
  }
  if( state != 0 )
  {
	  state = 1;
  }
  return state;
}

/*
 Function: Sets the module with CRC off.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t	setCRC_OFF(SPI_HandleTypeDef *hspi)
{
  int8_t state = 2;
  uint8_t config;

  if( _modem == LORA )
  { // LORA mode
  	config = readRegister(hspi, REG_MODEM_CONFIG2);	// Save config1 to modify only the CRC bit
	config = config & 251;				// clears bit 1 from config1 = CRC_OFF
	writeRegister(hspi, REG_MODEM_CONFIG2,config);

	config = readRegister(hspi, REG_MODEM_CONFIG2);
	if( (bitRead(config, 2)) == CRC_OFF )
	{ // take out bit 1 from REG_MODEM_CONFIG2 indicates RxPayloadCrcOn
	  state = 0;
	  _CRC = CRC_OFF;
	}
  }
  else
  { // FSK mode
	config = readRegister(hspi, REG_PACKET_CONFIG1);	// Save config1 to modify only the CRC bit
	config = config & 239;				// clears bit 4 from config1 = CRC_OFF
	writeRegister(hspi, REG_PACKET_CONFIG1,config);

	config = readRegister(hspi, REG_PACKET_CONFIG1);
	if( bitRead(config, 4) == CRC_OFF )
	{ // take out bit 4 from REG_PACKET_CONFIG1 indicates RxPayloadCrcOn
		state = 0;
		_CRC = CRC_OFF;
	}
  }
  if( state != 0 )
  {
	  state = 1;
  }
  return state;
}
/*
 Function: Sets the indicated channel in the module.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
   state = -1 --> Forbidden command for this protocol
 Parameters:
   ch: frequency channel value to set in configuration.
*/
int8_t setChannel(SPI_HandleTypeDef *hspi, uint32_t ch)
{
  uint8_t st0;
  int8_t state = 2;
  unsigned int freq3;
  unsigned int freq2;
  uint8_t freq1;
  uint32_t freq;

  st0 = readRegister(hspi, REG_OP_MODE);	// Save the previous status
  if( _modem == LORA )
  {
	  // LoRa Stdby mode in order to write in registers
	  writeRegister(hspi, REG_OP_MODE, LORA_STANDBY_MODE);
  }
  else
  {
	  // FSK Stdby mode in order to write in registers
	  writeRegister(hspi, REG_OP_MODE, FSK_STANDBY_MODE);
  }

  freq3 = ((ch >> 16) & 0x0FF);		// frequency channel MSB
  freq2 = ((ch >> 8) & 0x0FF);		// frequency channel MIB
  freq1 = (ch & 0xFF);				// frequency channel LSB

  writeRegister(hspi, REG_FRF_MSB, freq3);
  writeRegister(hspi, REG_FRF_MID, freq2);
  writeRegister(hspi, REG_FRF_LSB, freq1);

  // storing MSB in freq channel value
  freq3 = (readRegister(hspi, REG_FRF_MSB));
  freq = (freq3 << 8) & 0xFFFFFF;

  // storing MID in freq channel value
  freq2 = (readRegister(hspi, REG_FRF_MID));
  freq = (freq << 8) + ((freq2 << 8) & 0xFFFFFF);

  // storing LSB in freq channel value
  freq = freq + ((readRegister(hspi, REG_FRF_LSB)) & 0xFFFFFF);

  if( freq == ch )
  {
    state = 0;
    _channel = ch;
  }
  else
  {
    state = 1;
  }

  if( ~ isChannel(ch) )
  {
	 state = -1;
  }

  writeRegister(hspi, REG_OP_MODE, st0);	// Getting back to previous status
  return state;
}
/*
 Function: Sets the module in FSK mode.
 Returns:   Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t setFSK(SPI_HandleTypeDef *hspi)
{
	uint8_t state = 2;
    uint8_t st0;
    uint8_t config1;

	writeRegister(hspi, REG_OP_MODE, FSK_SLEEP_MODE);	// Sleep mode (mandatory to change mode)
	writeRegister(hspi, REG_OP_MODE, FSK_STANDBY_MODE);	// FSK standby mode
	config1 = readRegister(hspi, REG_PACKET_CONFIG1);
	config1 = config1 & 125;		// clears bits 8 and 1 from REG_PACKET_CONFIG1
	config1 = config1 | 4;		// sets bit 2 from REG_PACKET_CONFIG1
	writeRegister(hspi, REG_PACKET_CONFIG1,config1);	// AddressFiltering = NodeAddress + BroadcastAddress
	writeRegister(hspi, REG_FIFO_THRESH, 0x80);	// condition to start packet tx
	config1 = readRegister(hspi, REG_SYNC_CONFIG);
	config1 = config1 & 63;
	writeRegister(hspi, REG_SYNC_CONFIG,config1);

	//delay(100);

	st0 = readRegister(hspi, REG_OP_MODE);	// Reading config mode
	if( st0 == FSK_STANDBY_MODE )
	{ // FSK mode
		_modem = FSK;
		state = 0;
	}
	else
	{ // LoRa mode
		_modem = LORA;
		state = 1;
	}
	return state;
}
/*
 Function: Sets the module in explicit header mode (header is sent).
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
   state = -1 --> Forbidden command for this protocol
*/
int8_t	setHeaderON(SPI_HandleTypeDef *hspi)
{
  int8_t state = 2;
  uint8_t config1;

  if( _modem == FSK )
  {
	  state = -1;		// header is not available in FSK mode
  }
  else
  {
	config1 = readRegister(hspi, REG_MODEM_CONFIG1);	// Save config1 to modify only the header bit
	if( _spreadingFactor == 6 )
	{
		state = -1;		// Mandatory headerOFF with SF = 6
	}
	else
	{
		config1 = config1 & 254;			// clears bit 2 from config1 = headerON
		writeRegister(hspi, REG_MODEM_CONFIG1,config1);	// Update config1
	}
	if( _spreadingFactor != 6 )
	{ // checking headerON taking out bit 2 from REG_MODEM_CONFIG1
		config1 = readRegister(hspi, REG_MODEM_CONFIG1);
		if( bitRead(config1, 0) == HEADER_ON )
		{
			state = 0;
			_header = HEADER_ON;
		}
		else
		{
			state = 1;
		}
	}
  }
  return state;
}
/*
 Function: Sets the bandwidth, coding rate and spreading factor of the LoRa modulation.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
   state = -1 --> Forbidden command for this protocol
 Parameters:
   mode: mode number to set the required BW, SF and CR of LoRa modem.
*/
int8_t setMode(SPI_HandleTypeDef *hspi, uint8_t mode)
{
	int8_t state = 2;
	uint8_t st0;
	uint8_t config1 = 0x00;
	uint8_t config2 = 0x00;

	st0 = readRegister(hspi, REG_OP_MODE);		// Save the previous status

	// 'setMode' function only can be called in LoRa mode
	if( _modem == FSK )
	{
		setLORA(hspi);
	}

	// LoRa standby mode
	writeRegister(hspi, REG_OP_MODE, LORA_STANDBY_MODE);

	switch (mode)
	{
		// mode 1 (better reach, medium time on air)
		case 1: 	setCR(hspi, CR_5);		// CR = 4/5
					setSF(hspi, SF_12);		// SF = 12
					setBW(hspi, BW_125);		// BW = 125 KHz
					break;

		// mode 2 (medium reach, less time on air)
		case 2: 	setCR(hspi, CR_5);		// CR = 4/5
					setSF(hspi, SF_12);		// SF = 12
					setBW(hspi, BW_250);		// BW = 250 KHz
					break;

		// mode 3 (worst reach, less time on air)
		case 3: 	setCR(hspi, CR_5);		// CR = 4/5
					setSF(hspi, SF_10);		// SF = 10
					setBW(hspi, BW_125);		// BW = 125 KHz
					break;

		// mode 4 (better reach, low time on air)
		case 4: 	setCR(hspi, CR_5);		// CR = 4/5
					setSF(hspi, SF_12);		// SF = 12
					setBW(hspi, BW_500);		// BW = 500 KHz
					break;

		// mode 5 (better reach, medium time on air)
		case 5: 	setCR(hspi, CR_5);		// CR = 4/5
					setSF(hspi, SF_10);		// SF = 10
					setBW(hspi, BW_250);		// BW = 250 KHz
					break;

		// mode 6 (better reach, worst time-on-air)
		case 6: 	setCR(hspi, CR_5);		// CR = 4/5
					setSF(hspi, SF_11);		// SF = 11
					setBW(hspi, BW_500);		// BW = 500 KHz
					break;

		// mode 7 (medium-high reach, medium-low time-on-air)
		case 7: 	setCR(hspi, CR_5);		// CR = 4/5
					setSF(hspi, SF_9);		// SF = 9
					setBW(hspi, BW_250);		// BW = 250 KHz
					break;

		// mode 8 (medium reach, medium time-on-air)
		case 8:		setCR(hspi, CR_5);		// CR = 4/5
					setSF(hspi, SF_9);		// SF = 9
					setBW(hspi, BW_500);		// BW = 500 KHz
					break;

		// mode 9 (medium-low reach, medium-high time-on-air)
		case 9: 	setCR(hspi, CR_5);		// CR = 4/5
					setSF(hspi, SF_8);		// SF = 8
					setBW(hspi, BW_500);		// BW = 500 KHz
					break;

		// mode 10 (worst reach, less time_on_air)
		case 10:	setCR(hspi, CR_5);		// CR = 4/5
					setSF(hspi, SF_7);		// SF = 7
					setBW(hspi, BW_500);		// BW = 500 KHz
					break;

		default:	state = -1; // The indicated mode doesn't exist

	};


	// Check proper register configuration
	if( state == -1 )	// if state = -1, don't change its value
	{
	}
	else
	{
		state = 1;
		config1 = readRegister(hspi, REG_MODEM_CONFIG1);
		switch (mode)
		{
			// (config1 >> 4) ---> take out bits 7-4 from REG_MODEM_CONFIG1 (=_bandwidth)
			// ((config1 >> 1) & 0x07) ---> take out bits 3-1 from REG_MODEM_CONFIG1 (=_codingRate)
			// (config2 >> 4) ---> take out bits 7-4 from REG_MODEM_CONFIG2 (=_spreadingFactor)

			// mode 1: BW = 125 KHz, CR = 4/5, SF = 12.
			case 1:
				if( (config1 >> 4) == BW_125 && ((config1 >> 1) & 0x07) == CR_5) {
					config2 = readRegister(hspi, REG_MODEM_CONFIG2);
					if( (config2 >> 4) == SF_12 ) {
						state = 0;
					}
				}
				break;


			// mode 2: BW = 250 KHz, CR = 4/5, SF = 12.
			case 2:
				if( (config1 >> 4) == BW_250 && ((config1 >> 1) & 0x07) == CR_5) {
					config2 = readRegister(hspi, REG_MODEM_CONFIG2);
					if( (config2 >> 4) == SF_12 ) {
						state = 0;
					}
				}
				break;

			// mode 3: BW = 125 KHz, CR = 4/5, SF = 10.
			case 3:
				if( (config1 >> 4) == BW_125 && ((config1 >> 1) & 0x07) == CR_5) {
					config2 = readRegister(hspi, REG_MODEM_CONFIG2);
					if( (config2 >> 4) == SF_10 ) {
						state = 0;
					}
				}
				break;

			// mode 4: BW = 500 KHz, CR = 4/5, SF = 12.
			case 4:
				if( (config1 >> 4) == BW_500 && ((config1 >> 1) & 0x07) == CR_5) {
					config2 = readRegister(hspi, REG_MODEM_CONFIG2);
					if( (config2 >> 4) == SF_12 ) {
						state = 0;
					}
				}
				break;

			// mode 5: BW = 250 KHz, CR = 4/5, SF = 10.
			case 5:
				if( (config1 >> 4) == BW_250 && ((config1 >> 1) & 0x07) == CR_5) {
					config2 = readRegister(hspi, REG_MODEM_CONFIG2);
					if( (config2 >> 4) == SF_10 ) {
						state = 0;
					}
				}
				break;

			// mode 6: BW = 500 KHz, CR = 4/5, SF = 11.
			case 6:
				if( (config1 >> 4) == BW_500 && ((config1 >> 1) & 0x07) == CR_5) {
					config2 = readRegister(hspi, REG_MODEM_CONFIG2);
					if( (config2 >> 4) == SF_11 ) {
						state = 0;
					}
				}
				break;

			// mode 7: BW = 250 KHz, CR = 4/5, SF = 9.
			case 7:
				if( (config1 >> 4) == BW_250 && ((config1 >> 1) & 0x07) == CR_5) {
					config2 = readRegister(hspi, REG_MODEM_CONFIG2);
					if( (config2 >> 4) == SF_9 ) {
						state = 0;
					}
				}
				break;

			// mode 8: BW = 500 KHz, CR = 4/5, SF = 9.
			case 8:
				if( (config1 >> 4) == BW_500 && ((config1 >> 1) & 0x07) == CR_5) {
					config2 = readRegister(hspi, REG_MODEM_CONFIG2);
					if( (config2 >> 4) == SF_9 ) {
						state = 0;
					}
				}
				break;

			// mode 9: BW = 500 KHz, CR = 4/5, SF = 8.
			case 9:
				if( (config1 >> 4) == BW_500 && ((config1 >> 1) & 0x07) == CR_5) {
					config2 = readRegister(hspi, REG_MODEM_CONFIG2);
					if( (config2 >> 4) == SF_8 ) {
						state = 0;
					}
				}
				break;

			// mode 10: BW = 500 KHz, CR = 4/5, SF = 7.
			case 10:
				if( (config1 >> 4) == BW_500 && ((config1 >> 1) & 0x07) == CR_5) {
					config2 = readRegister(hspi, REG_MODEM_CONFIG2);
					if( (config2 >> 4) == SF_7 ) {
						state = 0;
					}
				}
				break;
		}// end switch
	}
	// Getting back to previous status
	writeRegister(hspi, REG_OP_MODE, st0);
	HAL_Delay(100);
	return state;
}
/*
 Function: Sets the signal power indicated in the module.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
   state = -1 --> Forbidden command for this protocol
 Parameters:
   p: power option to set in configuration.
*/
int8_t setPower(SPI_HandleTypeDef *hspi, char p)
{
  uint8_t st0;
  int8_t state = 2;
  uint8_t value = 0x00;

  st0 = readRegister(hspi, REG_OP_MODE);	  // Save the previous status
  if( _modem == LORA )
  { // LoRa Stdby mode to write in registers
	  writeRegister(hspi, REG_OP_MODE, LORA_STANDBY_MODE);
  }
  else
  { // FSK Stdby mode to write in registers
	  writeRegister(hspi, REG_OP_MODE, FSK_STANDBY_MODE);
  }

  writeRegister(hspi, REG_PA_DAC, 0x84);

  switch (p)
  {

    // M = max
    // H = high
    // I = intermediate
    // L = low

    case 'M':  _power = 0xFF; //20dbm
	       writeRegister(hspi, REG_PA_DAC, 0x87);
               break;

    case 'H':  _power = 0xFC; //14dbm
               break;

    case 'I':  _power = 0xF6; //8dbm
               break;

    case 'L':  _power = 0xF0; //2dbm
               break;
    default:   state = -1;
               break;
  }

  writeRegister(hspi, REG_PA_CONFIG, _power);	// Setting output power value
  value = readRegister(hspi, REG_PA_CONFIG);

  if( value == _power )
  {
	  state = 0;
  }
  else
  {
	  state = 1;
  }

  writeRegister(hspi, REG_OP_MODE, st0);	// Getting back to previous status
  return state;
}
/*
 Function: Sets the signal power indicated as input to the module.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
   state = -1 --> Forbidden command for this protocol
 Parameters:
   pow: power option to set in configuration. The input value range is from
   0 to 14 dBm.
*/
int8_t setPowerNum(SPI_HandleTypeDef *hspi, uint8_t pow)
{
  uint8_t st0;
  int8_t state = 2;
  uint8_t value = 0x00;

  st0 = readRegister(hspi, REG_OP_MODE);	  // Save the previous status
  if( _modem == LORA )
  { // LoRa Stdby mode to write in registers
	  writeRegister(hspi, REG_OP_MODE, LORA_STANDBY_MODE);
  }
  else
  { // FSK Stdby mode to write in registers
	  writeRegister(hspi, REG_OP_MODE, FSK_STANDBY_MODE);
  }

  if ( (pow >= 2) && (pow <= 20) )
  { // Pout= 17-(15-OutputPower) = OutputPower+2
	  if ( pow <= 17 ) {
		writeRegister(hspi, REG_PA_DAC, 0x84);
	  	pow = pow - 2;
	  } else { // Power > 17dbm -> Power = 20dbm
		writeRegister(hspi, REG_PA_DAC, 0x87);
		pow = 15;
	  }
	  _power = pow;
  }
  else
  {
	  state = -1;
  }

  writeRegister(hspi, REG_PA_CONFIG, _power);	// Setting output power value
  value = readRegister(hspi, REG_PA_CONFIG);

  if( value == _power )
  {
	  state = 0;
  }
  else
  {
	  state = 1;
  }

  writeRegister(hspi, REG_OP_MODE, st0);	// Getting back to previous status
  return state;
}
/*
 Function: Gets the preamble length from the module.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t getPreambleLength(SPI_HandleTypeDef *hspi)
{
	int8_t state = 2;
	uint8_t p_length;
	state = 1;
	if( _modem == LORA )
  	{ // LORA mode
  		p_length = readRegister(hspi, REG_PREAMBLE_MSB_LORA);
  		// Saving MSB preamble length in LoRa mode
		_preamblelength = (p_length << 8) & 0xFFFF;
		p_length = readRegister(hspi, REG_PREAMBLE_LSB_LORA);
  		// Saving LSB preamble length in LoRa mode
		_preamblelength = _preamblelength + (p_length & 0xFFFF);
	}
	else
	{ // FSK mode
		p_length = readRegister(hspi, REG_PREAMBLE_MSB_FSK);
		// Saving MSB preamble length in FSK mode
		_preamblelength = (p_length << 8) & 0xFFFF;
		p_length = readRegister(hspi, REG_PREAMBLE_LSB_FSK);
		// Saving LSB preamble length in FSK mode
		_preamblelength = _preamblelength + (p_length & 0xFFFF);
	}
	state = 0;
	return state;
}

/*
 Function: Sets the preamble length in the module
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
 Parameters:
   l: length value to set as preamble length.
*/
uint8_t setPreambleLength(SPI_HandleTypeDef *hspi, uint16_t l)
{
	uint8_t st0;
	uint8_t p_length;
	int8_t state = 2;
	st0 = readRegister(hspi, REG_OP_MODE);	// Save the previous status
	state = 1;
	if( _modem == LORA )
  	{ // LoRa mode
  		writeRegister(hspi, REG_OP_MODE, LORA_STANDBY_MODE);    // Set Standby mode to write in registers
  		p_length = ((l >> 8) & 0x0FF);
  		// Storing MSB preamble length in LoRa mode
		writeRegister(hspi, REG_PREAMBLE_MSB_LORA, p_length);
		p_length = (l & 0x0FF);
		// Storing LSB preamble length in LoRa mode
		writeRegister(hspi, REG_PREAMBLE_LSB_LORA, p_length);
	}
	else
	{ // FSK mode
		writeRegister(hspi, REG_OP_MODE, FSK_STANDBY_MODE);    // Set Standby mode to write in registers
		p_length = ((l >> 8) & 0x0FF);
  		// Storing MSB preamble length in FSK mode
		writeRegister(hspi, REG_PREAMBLE_MSB_FSK, p_length);
		p_length = (l & 0x0FF);
  		// Storing LSB preamble length in FSK mode
		writeRegister(hspi, REG_PREAMBLE_LSB_FSK, p_length);
	}

	state = 0;
	writeRegister(hspi, REG_OP_MODE, st0);	// Getting back to previous status
	return state;
}
/*
 Function: It sets the maximum number of retries.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
   state = -1 -->
*/
uint8_t setRetries(SPI_HandleTypeDef *hspi, uint8_t ret)
{
	int8_t state = 2;
	state = 1;
	if( ret > MAX_RETRIES )
	{
		state = -1;
	}
	else
	{
		_maxRetries = ret;
		state = 0;
	}
	return state;
}
/*
 Function: It sets the timeout according to the configured mode.
 Link: http://www.semtech.com/images/datasheet/sx1278.pdf
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t setTimeout(void)
{
	uint8_t state = 2;
	uint16_t delay;
	state = 1;
	if( _modem == LORA )
	{
		// calculate 'delay'
		delay = ((0.1*_sendTime) + 1);

		float Tpacket = timeOnAir(_payloadlength);

		// calculate final send/receive timeout adding an offset and a random value
		_sendTime = (uint16_t) Tpacket + (rand()%delay) + 1000;
		// update state
		state = 0;
	}
	else
	{
		// update state
		_sendTime = MAX_TIMEOUT;

		// update state
		state = 0;
	}
	return state;
}
/*
 Function: Sets the module ON.
 Returns: Nothing
*/
uint8_t ON(SPI_HandleTypeDef *hspi)
{
	uint8_t state = 2;


	// Powering the module
  HAL_GPIO_WritePin(SX1278_NSS_PORT,SX1278_NSS_PIN,GPIO_PIN_SET);
  HAL_Delay(100);

	// Set Maximum Over Current Protection
	state = setMaxCurrent(hspi, 0x1B);

	if( state == 0 )
	{
		#if (SX1278_debug_mode > 1)
			Serial.println(F("## Setting ON with maximum current supply ##"));
			Serial.println();
		#endif
	}
	else
	{
		return 1;
	}

	// set LoRa mode
	state = setLORA(hspi);

	return state;
}

/*
 Function: Sets the module OFF.
 Returns: Nothing
*/
void OFF(void)
{
	HAL_GPIO_WritePin(SX1278_NSS_PORT,SX1278_NSS_PIN,GPIO_PIN_RESET);
}
/*
 Function: Gets the bandwidth, coding rate and spreading factor of the LoRa modulation.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t getMode(SPI_HandleTypeDef *hspi)
{
	uint8_t st0;
	int8_t state = 2;
	uint8_t value = 0x00;

	// Save the previous status
	st0 = readRegister(hspi, REG_OP_MODE);
	// Setting LoRa mode
	if( _modem == FSK )
	{
		setLORA(hspi);
	}
	value = readRegister(hspi, REG_MODEM_CONFIG1);
	_bandwidth = (value >> 4);   				// Storing 4 MSB from REG_MODEM_CONFIG1 (=_bandwidth)
	_codingRate = (value >> 1) & 0x07;  		// Storing first, second and third bits from
	value = readRegister(hspi, REG_MODEM_CONFIG2);	// REG_MODEM_CONFIG1 (=_codingRate)
	_spreadingFactor = (value >> 4) & 0x0F; 	// Storing 4 MSB from REG_MODEM_CONFIG2 (=_spreadingFactor)
	state = 1;

	if( isBW(_bandwidth) )		// Checking available values for:
	{								//		_bandwidth
		if( isCR(_codingRate) )		//		_codingRate
		{							//		_spreadingFactor
			if( isSF(_spreadingFactor) )
			{
				state = 0;
			}
		}
	}
	writeRegister(hspi, REG_OP_MODE, st0);	// Getting back to previous status
	return state;
}
/*
 Function: Gets the node address in the module.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t getNodeAddress(SPI_HandleTypeDef *hspi)
{
	uint8_t st0 = 0;
	uint8_t state = 2;

	if( _modem == LORA )
	{
		// Nothing to read
		// node address is stored in _nodeAddress attribute
		state = 0;
	}
	else
	{
		// FSK mode
		st0 = readRegister(hspi, REG_OP_MODE);	// Save the previous status

		// Allowing access to FSK registers while in LoRa standby mode
		writeRegister(hspi, REG_OP_MODE, LORA_STANDBY_FSK_REGS_MODE);

		// Read node address
		_nodeAddress = readRegister(hspi, REG_NODE_ADRS);

		// Getting back to previous status
		writeRegister(hspi, REG_OP_MODE, st0);

		// update state
		state = 0;
	}
	return state;
}
/*
 Function: It gets and stores a packet if it is received before ending 'wait' time.
 Returns:  Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
   state = -1 --> Forbidden parameter value for this function
 Parameters:
   wait: time to wait while there is no a valid header received.
*/
int8_t getPacket(SPI_HandleTypeDef *hspi, uint32_t wait)
{
	uint8_t state = 2;
	int8_t state_f = 2;
	uint8_t value = 0x00;
	unsigned long previous;
	_Bool p_received = false;

	previous = HAL_GetTick();

	if( _modem == LORA )
	{
		/// LoRa mode
		// read REG_IRQ_FLAGS
		value = readRegister(hspi, REG_IRQ_FLAGS);

		// Wait until the packet is received (RxDone flag) or the timeout expires
		while( (bitRead(value, 6) == 0) && (HAL_GetTick()-previous < (unsigned long)wait) )
		{
			value = readRegister(hspi, REG_IRQ_FLAGS);

			// Condition to avoid an overflow (DO NOT REMOVE)
			if( HAL_GetTick() < previous )
			{
				previous = HAL_GetTick();
			}
		}

		// Check if 'RxDone' is true and 'PayloadCrcError' is correct
		if( (bitRead(value, 6) == 1) && (bitRead(value, 5) == 0) )
		{
			// packet received & CRC correct
			p_received = true;	// packet correctly received
			_reception = CORRECT_PACKET;
		}
		else
		{
			if( (bitRead(value, 5) == 0) && (_CRC == CRC_ON) )
			{
				// CRC is correct
				_reception = CORRECT_PACKET;
			}
			else
			{
				// CRC incorrect
				_reception = INCORRECT_PACKET;
				state = 3;
			}
		}

	}
	else
	{
		/// FSK mode
		value = readRegister(hspi, REG_IRQ_FLAGS2);
		while( (bitRead(value, 2) == 0) && (HAL_GetTick() - previous < wait) )
		{
			value = readRegister(hspi, REG_IRQ_FLAGS2);
			// Condition to avoid an overflow (DO NOT REMOVE)
			if( HAL_GetTick() < previous )
			{
				previous = HAL_GetTick();
			}
		} // end while (millis)
		if( bitRead(value, 2) == 1 )
		{ // packet received
 			if( (bitRead(value, 1) == 1) && (_CRC == CRC_ON) )
			{ // CRC correct
				_reception = CORRECT_PACKET;
				p_received = true;
			}
			else
			{ // CRC incorrect
				_reception = INCORRECT_PACKET;
				state = 3;
				p_received = false;
			}
		}
		writeRegister(hspi, REG_OP_MODE, FSK_STANDBY_MODE);	// Setting standby FSK mode
	}

	/* If a new packet was received correctly, now the information must be
	 * filled inside the structures of the class
	 */
	if( p_received == true )
	{
		// Store the packet
		if( _modem == LORA )
		{
			/// LoRa
			// Setting address pointer in FIFO data buffer
			writeRegister(hspi, REG_FIFO_ADDR_PTR, 0x00);
			// Storing first uint8_t of the received packet
			packet_received.dst = readRegister(hspi, REG_FIFO);
		}
		else
		{
			/// FSK
			value = readRegister(hspi, REG_PACKET_CONFIG1);
			if( (bitRead(value, 2) == 0) && (bitRead(value, 1) == 0) )
			{
				// Storing first uint8_t of the received packet
				packet_received.dst = readRegister(hspi, REG_FIFO);
			}
			else
			{
				// Storing first uint8_t of the received packet
				packet_received.dst = _destination;
			}
		}

		// Reading second uint8_t of the received packet
		// Reading third uint8_t of the received packet
		// Reading fourth uint8_t of the received packet
		packet_received.src = readRegister(hspi, REG_FIFO);
		packet_received.packnum = readRegister(hspi, REG_FIFO);
		packet_received.length = readRegister(hspi, REG_FIFO);

		// calculate the payload length
		if( _modem == LORA )
		{
			_payloadlength = packet_received.length - OFFSET_PAYLOADLENGTH;
		}

		// check if length is incorrect
		if( packet_received.length > (MAX_LENGTH + 1) )
		{
			#if (SX1278_debug_mode > 0)
				Serial.println(F("Corrupted packet, length must be less than 256"));
			#endif
		}
		else
		{
			// Store payload in 'data'
			for(unsigned int i = 0; i < _payloadlength; i++)
			{
				packet_received.data[i] = readRegister(hspi, REG_FIFO);
			}
			// Store 'retry'
			packet_received.retry = readRegister(hspi, REG_FIFO);

			// Print the packet if debug_mode
			state_f = 0;
		}
	}
	else
	{
		// if packet was NOT received
		state_f = 1;
		if( (_reception == INCORRECT_PACKET) && (_retries < _maxRetries) && (state != 3) )
		{
			_retries++;
		}
	}

	// Setting address pointer in FIFO data buffer to 0x00 again
	if( _modem == LORA )
	{
		writeRegister(hspi, REG_FIFO_ADDR_PTR, 0x00);
	}

	// Initializing flags
	clearFlags(hspi);

	if( wait > MAX_WAIT )
	{
		state_f = -1;
	}

	return state_f;
}
/*
 Function: It gets and stores a packet if it is received before MAX_TIMEOUT expires.
 Returns:  Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t getPacketMAXTimeout(SPI_HandleTypeDef *hspi)
{
	return getPacket(hspi, MAX_TIMEOUT);
}
/*
 Function: Gets the payload length from the module.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t getPayloadLength(SPI_HandleTypeDef *hspi)
{
	uint8_t state = 2;

	if( _modem == LORA )
  	{ // LORA mode
  		// Saving payload length in LoRa mode
		_payloadlength = readRegister(hspi, REG_PAYLOAD_LENGTH_LORA);
		state = 1;
	}
	else
	{ // FSK mode
  		// Saving payload length in FSK mode
		_payloadlength = readRegister(hspi, REG_PAYLOAD_LENGTH_FSK);
		state = 1;
	}
	state = 0;
	return state;
}
/*
 Function: Gets the signal power within the module is configured.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t getPower(SPI_HandleTypeDef *hspi)
{
  uint8_t state = 2;
  uint8_t value = 0x00;

  value = readRegister(hspi, REG_PA_CONFIG);
  state = 1;

  value = value & 15;
  // Pout= 17-(15-OutputPower) = OutputPower+2
  value = value + 2;

  _power = value;
  if( (value >= 2) && (value <= 20) )
  {
	    state = 0;
  }

  return state;
}
/*
 Function: It gets the temperature from the measurement block module.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t getTemp(SPI_HandleTypeDef *hspi)
{
	uint8_t st0;
	uint8_t state = 2;
	st0 = readRegister(hspi, REG_OP_MODE);	// Save the previous status

	if( _modem == LORA )
	{ // Allowing access to FSK registers while in LoRa standby mode
		writeRegister(hspi,REG_OP_MODE, LORA_STANDBY_FSK_REGS_MODE);
	}

	state = 1;
	// Saving temperature value
	_temp = readRegister(hspi,REG_TEMP);
	if( _temp & 0x80 ) // The SNR sign bit is 1
	{
		// Invert and divide by 4
		_temp = ( ( ~_temp + 1 ) & 0xFF );
    }
    else
    {
		// Divide by 4
		_temp = ( _temp & 0xFF );
	}
	if( _modem == LORA )
	{
		writeRegister(hspi,REG_OP_MODE, st0);	// Getting back to previous status
	}

	state = 0;
	return state;
}
/*
 Function: Gets the content of different registers.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t getRegs(SPI_HandleTypeDef *hspi)
{
	int8_t state = 2;
	uint8_t state_f = 2;


	state_f = 1;
	state = getMode(hspi);			// Stores the BW, CR and SF.
	if( state == 0 )
	{
		state = getPower(hspi);		// Stores the power.
	}
	else
	{
		state_f = 1;
	}
 	if( state == 0 )
	{
		state = getChannel(hspi);	// Stores the channel.
	}
	else
	{
		state_f = 1;
	}
	if( state == 0 )
	{
		state = getCRC(hspi);		// Stores the CRC configuration.
	}
	else
	{
		state_f = 1;
	}
	if( state == 0 )
	{
		state = getHeader();	// Stores the header configuration.
	}
	else
	{
		state_f = 1;
	}
	if( state == 0 )
	{
		state = getPreambleLength(hspi);	// Stores the preamble length.
	}
	else
	{
		state_f = 1;
	}
	if( state == 0 )
	{
		state = getPayloadLength(hspi);		// Stores the payload length.
	}
	else
	{
		state_f = 1;
	}
	if( state == 0 )
	{
		state = getNodeAddress(hspi);		// Stores the node address.
	}
	else
	{
		state_f = 1;
	}
	if( state == 0 )
	{
		state = getMaxCurrent(hspi);		// Stores the maximum current supply.
	}
	else
	{
		state_f = 1;
	}
	if( state == 0 )
	{
		state_f = getTemp(hspi);		// Stores the module temperature.
	}
	else
	{
		state_f = 1;
	}
	return state_f;
}
/*
 Function: Gets the current value of RSSI.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t getRSSI(SPI_HandleTypeDef *hspi)
{
	uint8_t state = 2;
	int rssi_mean = 0;
	int total = 5;

	if( _modem == LORA )
	{
		/// LoRa mode
		// get mean value of RSSI
		for(int i = 0; i < total; i++)
		{
			_RSSI = -OFFSET_RSSI + readRegister(hspi, REG_RSSI_VALUE_LORA);
			rssi_mean += _RSSI;
		}
		rssi_mean = rssi_mean / total;
		_RSSI = rssi_mean;

		state = 0;
	}
	else
	{
		/// FSK mode
		// get mean value of RSSI
		for(int i = 0; i < total; i++)
		{
			_RSSI = (readRegister(hspi, REG_RSSI_VALUE_FSK) >> 1);
			rssi_mean += _RSSI;
		}
		rssi_mean = rssi_mean / total;
		_RSSI = rssi_mean;

		state = 0;
	}
	return state;
}
/*
 Function: Gets the SNR value in LoRa mode.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
   state = -1 --> Forbidden command for this protocol
*/
int8_t getSNR(SPI_HandleTypeDef *hspi)
{	// getSNR exists only in LoRa mode
  int8_t state = 2;
  uint8_t value;
  if( _modem == LORA )
  { // LoRa mode
	  state = 1;
	  value = readRegister(hspi, REG_PKT_SNR_VALUE);
	  if( value & 0x80 ) // The SNR sign bit is 1
	  {
		  // Invert and divide by 4
		  value = ( ( ~value + 1 ) & 0xFF ) >> 2;
          _SNR = -value;
      }
      else
      {
		  // Divide by 4
		  _SNR = ( value & 0xFF ) >> 2;
	  }
	  state = 0;
  }
  else
  { // forbidden command if FSK mode
	state = -1;
  }
  return state;
}

/*
 Function: Gets the RSSI of the last packet received in LoRa mode.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
   state = -1 --> Forbidden command for this protocol
*/
int16_t getRSSIpacket(SPI_HandleTypeDef *hspi)
{	// RSSIpacket only exists in LoRa
  int8_t state = 2;
  state = 1;
  if( _modem == LORA )
  { // LoRa mode
	  state = getSNR(hspi);
	  if( state == 0 )
	  {
		  if( _SNR < 0 )
		  {
			  _RSSIpacket = -NOISE_ABSOLUTE_ZERO + 10.0 * SignalBwLog[_bandwidth] + NOISE_FIGURE + ( double )_SNR;
			  state = 0;
		  }
		  else
		  {
			  _RSSIpacket = readRegister(hspi, REG_PKT_RSSI_VALUE);
			  _RSSIpacket = -OFFSET_RSSI + ( double )_RSSIpacket;
			  state = 0;
		  }
	  }
  }
  else
  { // RSSI packet doesn't exist in FSK mode
	state = -1;
  }
  return state;
}
/*
 Function: Configures the module to transmit information.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t sendWithTimeout(SPI_HandleTypeDef *hspi, uint32_t wait)
{
	uint8_t state = 2;
	uint8_t value = 0x00;
	unsigned long previous;

	// wait to TxDone flag
	previous = HAL_GetTick();
	if( _modem == LORA )
	{
		/// LoRa mode
		// Initializing flags
		clearFlags(hspi);
		// LORA mode - Tx
		writeRegister(hspi, REG_OP_MODE, LORA_TX_MODE);

		value = readRegister(hspi, REG_IRQ_FLAGS);

		// Wait until the packet is sent (TX Done flag) or the timeout expires
		while ((bitRead(value, 3) == 0) && (HAL_GetTick() - previous < wait))
		{
			value = readRegister(hspi, REG_IRQ_FLAGS);
			// Condition to avoid an overflow (DO NOT REMOVE)
			if( HAL_GetTick() < previous )
			{
				previous = HAL_GetTick();
			}
		}
		state = 1;
	}
	else
	{
		/// FSK mode
		writeRegister(hspi, REG_OP_MODE, FSK_TX_MODE);  // FSK mode - Tx

		value = readRegister(hspi, REG_IRQ_FLAGS2);
		// Wait until the packet is sent (Packet Sent flag) or the timeout expires
		while ((bitRead(value, 3) == 0) && (HAL_GetTick() - previous < wait))
		{
			value = readRegister(hspi, REG_IRQ_FLAGS2);

			// Condition to avoid an overflow (DO NOT REMOVE)
			if( HAL_GetTick() < previous )
			{
				previous = HAL_GetTick();
			}
		}
		state = 1;
	}
	if( bitRead(value, 3) == 1 )
	{
		state = 0;	// Packet successfully sent
	}

	// Initializing flags
	clearFlags(hspi);
	return state;
}
/*
 Function: Configures the module to transmit information.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t sendWithTimeout_sendtime(SPI_HandleTypeDef *hspi)
{
	setTimeout();
	return sendWithTimeout(hspi, _sendTime);
}
/*
 Function: Configures the module to transmit information.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t sendPacketTimeout_sendtime(SPI_HandleTypeDef *hspi, uint8_t dest, char *payload)
{
	uint8_t state = 2;
	state = setPacket(hspi,dest, payload);	// Setting a packet with 'dest' destination
	if (state == 0)								// and writing it in FIFO.
	{
		state = sendWithTimeout_sendtime(hspi);	// Sending the packet
	}
	return state;
}
/*
 Function: Configures the module to receive information.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t receive(SPI_HandleTypeDef *hspi)
{
	uint8_t state = 1;
	// Initializing packet_received struct
	memset( &packet_received, 0x00, sizeof(packet_received) );

	// Setting Testmode
	writeRegister(hspi, 0x31,0x43);
	// Set LowPnTxPllOff
	writeRegister(hspi, REG_PA_RAMP, 0x09);
	// Set LNA gain: Highest gain. LnaBoost:Improved sensitivity
	writeRegister(hspi, REG_LNA, 0x23);
	// Setting address pointer in FIFO data buffer
	writeRegister(hspi, REG_FIFO_ADDR_PTR, 0x00);
	// change RegSymbTimeoutLsb
	writeRegister(hspi, REG_SYMB_TIMEOUT_LSB, 0xFF);
	// Setting current value of reception buffer pointer
	writeRegister(hspi, REG_FIFO_RX_BYTE_ADDR, 0x00);

	// Proceed depending on the protocol selected
	if( _modem == LORA )
	{
		/// LoRa mode
		// With MAX_LENGTH gets all packets with length < MAX_LENGTH
		state = setPacketLength(hspi, MAX_LENGTH);
		// Set LORA mode - Rx
		writeRegister(hspi, REG_OP_MODE, LORA_RX_MODE);
	}
	else
	{
		/// FSK mode
		state = setPacketLength_payloadlength(hspi);
		// FSK mode - Rx
		writeRegister(hspi, REG_OP_MODE, FSK_RX_MODE);
	}
	return state;
}
/*
 Function: Configures the module to receive all the information on air.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t receiveAll(SPI_HandleTypeDef *hspi, uint32_t wait)
{
	uint8_t state = 2;
	uint8_t config1;
	if( _modem == FSK )
	{
		/// FSK mode
		writeRegister(hspi, REG_OP_MODE, FSK_STANDBY_MODE);		// Setting standby FSK mode
		config1 = readRegister(hspi, REG_PACKET_CONFIG1);
		config1 = config1 & 249;			// clears bits 2-1 from REG_PACKET_CONFIG1
		writeRegister(hspi, REG_PACKET_CONFIG1, config1);		// AddressFiltering = None
	}
	// Setting Rx mode
	state = receive(hspi);

	if( state == 0 )
	{
		// Getting all packets received in wait
		state = getPacket(hspi, wait);
	}
	return state;
}
/*
 Function: Configures the module to receive all the information on air, before MAX_TIMEOUT expires.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t	receiveAll_MAX_TIMEOUT(SPI_HandleTypeDef *hspi)
{
	return receiveAll(hspi, MAX_TIMEOUT);
}
/*
 Function: Configures the module to receive information.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t receivePacketTimeout(SPI_HandleTypeDef *hspi, uint32_t wait)
{
	uint8_t state = 2;
	uint8_t state_f = 2;
	// set RX mode
	state = receive(hspi);

	// if RX mode is set correctly then wait for data
	if( state == 0 )
	{
		// Wait for a new packet for 'wait' time
		if( availableData(hspi, wait) )
		{
			// If packet received, getPacket
			state_f = getPacket(hspi,MAX_TIMEOUT);
		}
		else
		{
			state_f = 1;
		}
	}
	else
	{
		state_f = state;
	}
	return state_f;
}
/*
 Function: Configures the module to receive information and send an ACK.
 Returns: Integer that determines if there has been any error
   state = 4  --> The command has been executed but the packet received is incorrect
   state = 3  --> The command has been executed but there is no packet received
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t receivePacketTimeoutACK(SPI_HandleTypeDef *hspi, uint32_t wait)
{
	uint8_t state = 2;
	uint8_t state_f = 2;

	// set RX mode
	state = receive(hspi);

	// if RX mode is set correctly then wait for data
	if( state == 0 )
	{
		// Wait for a new packet for 'wait' time
		if( availableData(hspi, wait) )
		{
			// If packet received, getPacket
			state = getPacketMAXTimeout(hspi);
		}
		else
		{
			state = 1;
			state_f = 3;  // There is no packet received
		}
	}
	else
	{
		state = 1;
		state_f = 1; // There has been an error with the 'receive' function
	}


	if( (state == 0) || (state == 3) )
	{
		if( _reception == INCORRECT_PACKET )
		{
			state_f = 4;  // The packet has been incorrectly received
		}
		else
		{
			state_f = 1;  // The packet has been correctly received
		}
		state = setACK(hspi);
		if( state == 0 )
		{
			state = sendWithTimeout_sendtime(hspi);
			if( state == 0 )
			{
			state_f = 0;
			}
			else
			{
				state_f = 1; // There has been an error with the 'sendWithTimeout' function
			}
		}
		else
		{
			state_f = 1; // There has been an error with the 'setACK' function
		}
	}
	else
	{
		state_f = 1;
	}
	return state_f;
}
/*
 Function: Configures the module to receive information.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t receivePacketTimeout_sendTime(SPI_HandleTypeDef *hspi)
{
	setTimeout();
	return receivePacketTimeout(hspi, _sendTime);
}
/*
 Function: Configures the module to receive information.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t receivePacketMAXTimeout(SPI_HandleTypeDef *hspi)
{
	return receivePacketTimeout(hspi, MAX_TIMEOUT);
}
/*
 Function: Configures the module to receive information and send an ACK.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t receivePacketMAXTimeoutACK(SPI_HandleTypeDef *hspi)
{
	return receivePacketTimeoutACK(hspi, MAX_TIMEOUT);
}
/*
 Function: Configures the module to receive information and send an ACK.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t receivePacketTimeoutACK_sendTime(SPI_HandleTypeDef *hspi)
{
	setTimeout();
	return receivePacketTimeoutACK(hspi, _sendTime);
}

/*
 Function: Configures the module to transmit information.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t sendPacketTimeout_WaitTime(SPI_HandleTypeDef *hspi, uint8_t dest, char *payload, uint16_t wait)
{
	uint8_t state = 2;

	state = setPacket(hspi, dest, payload);	// Setting a packet with 'dest' destination
	if (state == 0)								// and writing it in FIFO.
	{
		state = sendWithTimeout(hspi, wait);	// Sending the packet
	}
	return state;
}
/*
 Function: Configures the module to transmit information.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t sendPacketTimeout_length(SPI_HandleTypeDef *hspi, uint8_t dest, char *payload, uint16_t length16)
{
	uint8_t state = 2;
	uint8_t state_f = 2;

	state = truncPayload(length16);
	if( state == 0 )
	{
		state_f = setPacket(hspi, dest, payload);	// Setting a packet with 'dest' destination
	}												// and writing it in FIFO.
	else
	{
		state_f = state;
	}
	if( state_f == 0 )
	{
		state_f = sendWithTimeout_sendtime(hspi);	// Sending the packet
	}
	return state_f;
}
/*
 Function: Configures the module to transmit information.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t sendPacketTimeout_wait_length(SPI_HandleTypeDef *hspi, uint8_t dest, char *payload, uint16_t length16, uint16_t wait)
{
	uint8_t state = 2;
	uint8_t state_f = 2;

	state = truncPayload(length16);
	if( state == 0 )
	{
		state_f = setPacket(hspi, dest, payload);	// Setting a packet with 'dest' destination
	}
	else
	{
		state_f = state;
	}
	if( state_f == 0 )								// and writing it in FIFO.
	{
		state_f = sendWithTimeout(hspi, wait);	// Sending the packet
	}
	return state_f;
}
/*
 Function: Configures the module to transmit information and receive an ACK.
 Returns: Integer that determines if there has been any error
   state = 9  --> The ACK lost (no data available)
   state = 8  --> The ACK lost
   state = 7  --> The ACK destination incorrectly received
   state = 6  --> The ACK source incorrectly received
   state = 5  --> The ACK number incorrectly received
   state = 4  --> The ACK length incorrectly received
   state = 3  --> N-ACK received
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t sendPacketTimeoutACK(SPI_HandleTypeDef *hspi, uint8_t dest, char *payload)
{
	uint8_t state = 2;
	uint8_t state_f = 2;

	state = sendPacketTimeout_sendtime(hspi, dest, payload);	// Sending packet to 'dest' destination
	if( state == 0 )
	{
		state = receive(hspi);	// Setting Rx mode to wait an ACK
	}
	else
	{
		state_f = state;
	}
	if( state == 0 )
	{
		if( availableData_MAX_TIMEOUT(hspi) )
		{
			state_f = getACK_MAX_TIMEOUT(hspi);	// Getting ACK
		}
		else
		{
			state_f = 9;
		}
	}
	else
	{
		state_f = state;
	}

	return state_f;
}
/*
 Function: Configures the module to transmit information and receive an ACK.
 Returns: Integer that determines if there has been any error
   state = 9  --> The ACK lost (no data available)
   state = 8  --> The ACK lost
   state = 7  --> The ACK destination incorrectly received
   state = 6  --> The ACK source incorrectly received
   state = 5  --> The ACK number incorrectly received
   state = 4  --> The ACK length incorrectly received
   state = 3  --> N-ACK received
   state = 2  --> The ACK has not been received
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t sendPacketTimeoutACK_length(SPI_HandleTypeDef *hspi,	uint8_t dest,
											char *payload,
											uint16_t length16)
{
	uint8_t state = 2;
	uint8_t state_f = 2;


	// Sending packet to 'dest' destination
	state = sendPacketTimeout_length(hspi, dest, payload, length16);

	// Trying to receive the ACK
	if( state == 0 )
	{
		state = receive(hspi);	// Setting Rx mode to wait an ACK
	}
	else
	{
		state_f = state;
	}
	if( state == 0 )
	{
		if( availableData_MAX_TIMEOUT(hspi) )
		{
			state_f = getACK_MAX_TIMEOUT(hspi);	// Getting ACK
		}
		else
		{
			state_f = 9;
		}
	}
	else
	{
		state_f = state;
	}

	return state_f;
}

/*
 Function: Configures the module to transmit information and receive an ACK.
 Returns: Integer that determines if there has been any error
   state = 9  --> The ACK lost (no data available)
   state = 8  --> The ACK lost
   state = 7  --> The ACK destination incorrectly received
   state = 6  --> The ACK source incorrectly received
   state = 5  --> The ACK number incorrectly received
   state = 4  --> The ACK length incorrectly received
   state = 3  --> N-ACK received
   state = 2  --> The ACK has not been received
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t sendPacketTimeoutACK_wait(SPI_HandleTypeDef *hspi,
											uint8_t dest,
											char *payload,
											uint16_t wait)
{
	uint8_t state = 2;
	uint8_t state_f = 2;

	state = sendPacketTimeout_WaitTime(hspi, dest, payload, wait);	// Sending packet to 'dest' destination
	if( state == 0 )
	{
		state = receive(hspi);	// Setting Rx mode to wait an ACK
	}
	else
	{
		state_f = 1;
	}
	if( state == 0 )
	{
		if( availableData_MAX_TIMEOUT(hspi) )
		{
			state_f = getACK_MAX_TIMEOUT(hspi);	// Getting ACK
		}
		else
		{
			state_f = 9;
		}
	}
	else
	{
		state_f = 1;
	}

	return state_f;
}

/*
 Function: Configures the module to transmit information and receive an ACK.
 Returns: Integer that determines if there has been any error
   state = 9  --> The ACK lost (no data available)
   state = 8  --> The ACK lost
   state = 7  --> The ACK destination incorrectly received
   state = 6  --> The ACK source incorrectly received
   state = 5  --> The ACK number incorrectly received
   state = 4  --> The ACK length incorrectly received
   state = 3  --> N-ACK received
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t sendPacketTimeoutACK_wait_length(SPI_HandleTypeDef *hspi, uint8_t dest,
											char *payload,
											uint16_t length16,
											uint32_t wait)
{
	uint8_t state = 2;
	uint8_t state_f = 2;

	state = sendPacketTimeout_wait_length(hspi, dest, payload, length16, wait);	// Sending packet to 'dest' destination
	if( state == 0 )
	{
		state = receive(hspi);	// Setting Rx mode to wait an ACK
	}
	else
	{
		state_f = 1;
	}
	if( state == 0 )
	{
		if( availableData_MAX_TIMEOUT(hspi) )
		{
			state_f = getACK_MAX_TIMEOUT(hspi);	// Getting ACK
		}
		else
		{
			state_f = 9;
		}
	}
	else
	{
		state_f = 1;
	}

	return state_f;
}
/*
 Function: Configures the module to transmit information with retries in case of error.
 Returns: Integer that determines if there has been any error
   state = 9  --> The ACK lost (no data available)
   state = 8  --> The ACK lost
   state = 7  --> The ACK destination incorrectly received
   state = 6  --> The ACK source incorrectly received
   state = 5  --> The ACK number incorrectly received
   state = 4  --> The ACK length incorrectly received
   state = 3  --> N-ACK received
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t sendPacketTimeoutACKRetries(SPI_HandleTypeDef *hspi, uint8_t dest, char *payload)
{
	uint8_t state = 2;

	// Sending packet to 'dest' destination and waiting an ACK response.
	state = 1;
	while( (state != 0) && (_retries <= _maxRetries) )
	{
		state = sendPacketTimeoutACK(hspi, dest, payload);
		_retries++;
	}
	_retries = 0;

	return state;
}
/*
 Function: Configures the module to transmit information with retries in case of error.
 Returns: Integer that determines if there has been any error
   state = 9  --> The ACK lost (no data available)
   state = 8  --> The ACK lost
   state = 7  --> The ACK destination incorrectly received
   state = 6  --> The ACK source incorrectly received
   state = 5  --> The ACK number incorrectly received
   state = 4  --> The ACK length incorrectly received
   state = 3  --> N-ACK received
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t sendPacketTimeoutACKRetries_length(SPI_HandleTypeDef *hspi, uint8_t dest,
												char *payload,
												uint16_t length16)
{
	uint8_t state = 2;

	// Sending packet to 'dest' destination and waiting an ACK response.
	state = 1;
	while( (state != 0) && (_retries <= _maxRetries) )
	{
		state = sendPacketTimeoutACK_length(hspi, dest, payload, length16);
		_retries++;
	}
	_retries = 0;

	return state;
}

/*
 Function: Configures the module to transmit information with retries in case of error.
 Returns: Integer that determines if there has been any error
   state = 9  --> The ACK lost (no data available)
   state = 8  --> The ACK lost
   state = 7  --> The ACK destination incorrectly received
   state = 6  --> The ACK source incorrectly received
   state = 5  --> The ACK number incorrectly received
   state = 4  --> The ACK length incorrectly received
   state = 3  --> N-ACK received
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t sendPacketTimeoutACKRetries_wait(SPI_HandleTypeDef *hspi, uint8_t dest,
												char *payload,
												uint32_t wait)
{
	uint8_t state = 2;

	// Sending packet to 'dest' destination and waiting an ACK response.
	state = 1;
	while( (state != 0) && (_retries <= _maxRetries) )
	{
		state = sendPacketTimeoutACK_wait(hspi, dest, payload, wait);
		_retries++;
	}
	_retries = 0;

	return state;
}

/*
 Function: Configures the module to transmit information with retries in case of error.
 Returns: Integer that determines if there has been any error
   state = 9  --> The ACK lost (no data available)
   state = 8  --> The ACK lost
   state = 7  --> The ACK destination incorrectly received
   state = 6  --> The ACK source incorrectly received
   state = 5  --> The ACK number incorrectly received
   state = 4  --> The ACK length incorrectly received
   state = 3  --> N-ACK received
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t sendPacketTimeoutACKRetries_length_wait(SPI_HandleTypeDef *hspi,
												uint8_t dest,
												char *payload,
												uint16_t length16,
												uint32_t wait)
{
	uint8_t state = 2;

	// Sending packet to 'dest' destination and waiting an ACK response.
	state = 1;
	while( (state != 0) && (_retries <= _maxRetries) )
	{
		state = sendPacketTimeoutACK_wait_length(hspi, dest, payload, length16, wait);
		_retries++;
	}
	_retries = 0;

	return state;
}
/*
 Function: Configures the module to transmit information.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t sendWithMAXTimeout(SPI_HandleTypeDef *hspi)
{
	return sendWithTimeout(hspi, MAX_TIMEOUT);
}

//END
//************************************************************
