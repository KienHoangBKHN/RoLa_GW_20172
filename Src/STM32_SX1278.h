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
  * @file    STM32_SX1278.h
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


#ifndef STM32_SX1278_H
#define STM32_SX1278_H

/******************************************************************************
 * Includes
 ******************************************************************************/
#include <stdlib.h>
#include <stdint.h>
#include "stm32f4xx_hal.h"
#include <string.h>
#include <math.h>


/******************************************************************************
 * Definitions & Declarations
 *****************************************************************************/

#define SX1278_debug_mode 0

#define SX1278_NSS_PORT GPIOA
#define SX1278_NSS_PIN	GPIO_PIN_4

//! MACROS //
#define bitRead(value, bit) (((value) >> (bit)) & 0x01)  // read a bit
#define bitSet(value, bit) ((value) |= (1UL << (bit)))    // set bit to '1'
#define bitClear(value, bit) ((value) &= ~(1UL << (bit))) // set bit to '0'
#define max(a,b) (a>=b)?a:b

//! REGISTERS //
//							FSK	Commun	LORA
#define        REG_FIFO        					0x00
#define        REG_OP_MODE        				0x01
#define        REG_BITRATE_MSB    			0x02
#define        REG_BITRATE_LSB    			0x03
#define        REG_FDEV_MSB   				0x04
#define        REG_FDEV_LSB    				0x05
#define        REG_FRF_MSB    					0x06
#define        REG_FRF_MID    					0x07
#define        REG_FRF_LSB    					0x08
#define        REG_PA_CONFIG    				0x09
#define        REG_PA_RAMP    					0x0A
#define        REG_OCP    					0x0B
#define        REG_LNA    					0x0C
#define        REG_RX_CONFIG    			0x0D
#define        REG_FIFO_ADDR_PTR  					0x0D
#define        REG_RSSI_CONFIG   			0x0E
#define        REG_FIFO_TX_BASE_ADDR 		    			0x0E
#define        REG_RSSI_COLLISION    			0x0F
#define        REG_FIFO_RX_BASE_ADDR   					0x0F
#define        REG_RSSI_THRESH    			0x10
#define        REG_FIFO_RX_CURRENT_ADDR   				0x10
#define        REG_RSSI_VALUE_FSK	    		0x11
#define        REG_IRQ_FLAGS_MASK    					0x11
#define        REG_RX_BW		    		0x12
#define        REG_IRQ_FLAGS	    					0x12
#define        REG_AFC_BW		    		0x13
#define        REG_RX_NB_BYTES	    					0x13
#define        REG_OOK_PEAK	    			0x14
#define        REG_RX_HEADER_CNT_VALUE_MSB  				0x14
#define        REG_OOK_FIX	    			0x15
#define        REG_RX_HEADER_CNT_VALUE_LSB  				0x15
#define        REG_OOK_AVG	 			0x16
#define        REG_RX_PACKET_CNT_VALUE_MSB  				0x16
#define        REG_RX_PACKET_CNT_VALUE_LSB  				0x17
#define        REG_MODEM_STAT	  					0x18
#define        REG_PKT_SNR_VALUE	  				0x19
#define        REG_AFC_FEI	  			0x1A
#define        REG_PKT_RSSI_VALUE	  				0x1A
#define        REG_AFC_MSB	  			0x1B
#define        REG_RSSI_VALUE_LORA	  				0x1B
#define        REG_AFC_LSB	  			0x1C
#define        REG_HOP_CHANNEL	  					0x1C
#define        REG_FEI_MSB	  			0x1D
#define        REG_MODEM_CONFIG1	 		 		0x1D
#define        REG_FEI_LSB	  			0x1E
#define        REG_MODEM_CONFIG2	  				0x1E
#define        REG_PREAMBLE_DETECT  			0x1F
#define        REG_SYMB_TIMEOUT_LSB  					0x1F
#define        REG_RX_TIMEOUT1	  			0x20
#define        REG_PREAMBLE_MSB_LORA  					0x20
#define        REG_RX_TIMEOUT2	  			0x21
#define        REG_PREAMBLE_LSB_LORA  					0x21
#define        REG_RX_TIMEOUT3	 			0x22
#define        REG_PAYLOAD_LENGTH_LORA			 		0x22
#define        REG_RX_DELAY	 			0x23
#define        REG_MAX_PAYLOAD_LENGTH 					0x23
#define        REG_OSC		 			0x24
#define        REG_HOP_PERIOD	  					0x24
#define        REG_PREAMBLE_MSB_FSK 			0x25
#define        REG_FIFO_RX_BYTE_ADDR 					0x25
#define        REG_PREAMBLE_LSB_FSK 			0x26
#define        REG_MODEM_CONFIG3	 		 		0x26
#define        REG_SYNC_CONFIG	  			0x27
#define        REG_SYNC_VALUE1	 			0x28
//#define	       REG_FEI_MSB						0x28
#define        REG_SYNC_VALUE2	  			0x29
#define	       REG_FEI_MID						0x29
#define        REG_SYNC_VALUE3	  			0x2A
//#define	       REG_FEI_LSB						0x2A
#define        REG_SYNC_VALUE4	  			0x2B
#define        REG_SYNC_VALUE5	  			0x2C
#define	       REG_RSSI_WIDEBAND					0x2C
#define        REG_SYNC_VALUE6	  			0x2D
#define        REG_SYNC_VALUE7	  			0x2E
#define        REG_SYNC_VALUE8	  			0x2F
#define        REG_PACKET_CONFIG1	  		0x30
#define        REG_PACKET_CONFIG2	  		0x31
#define        REG_DETECT_OPTIMIZE	  				0x31
#define        REG_PAYLOAD_LENGTH_FSK			0x32
#define        REG_NODE_ADRS	  			0x33
#define        REG_INVERT_IQ						0x33
#define        REG_BROADCAST_ADRS	 		0x34
#define        REG_FIFO_THRESH	  			0x35
#define        REG_SEQ_CONFIG1	  			0x36
#define        REG_SEQ_CONFIG2	  			0x37
#define        REG_DETECTION_THRESHOLD 					0x37
#define        REG_TIMER_RESOL	  			0x38
#define        REG_TIMER1_COEF	  			0x39
#define        REG_SYNC_WORD						0x39
#define        REG_TIMER2_COEF	  			0x3A
#define        REG_IMAGE_CAL	  			0x3B
#define        REG_TEMP		  			0x3C
#define        REG_LOW_BAT	  			0x3D
#define        REG_IRQ_FLAGS1	  			0x3E
#define        REG_IRQ_FLAGS2	  			0x3F
#define        REG_DIO_MAPPING1	  				0x40
#define        REG_DIO_MAPPING2	  				0x41
#define        REG_VERSION	  				0x42
#define        REG_PLL_HOP	  			0x44
#define        REG_TCXO		  				0x4B
#define        REG_PA_DAC	  				0x4D
#define        REG_FORMER_TEMP	  				0x5B
#define        REG_BIT_RATE_FRAC			0x5D
#define        REG_AGC_REF	  				0x61
#define        REG_AGC_THRESH1		  			0x62
#define        REG_AGC_THRESH2		  			0x63
#define        REG_AGC_THRESH3	  				0x64
#define        REG_PLL			  			0x70

//const uint8_t MAX_PAYLOAD = 251;


//! Structure :
/*!
 */
typedef struct pack
{
	//! Structure Variable : Packet destination
	/*!
 	*/
	uint8_t dst;

	//! Structure Variable : Packet source
	/*!
 	*/
	uint8_t src;

	//! Structure Variable : Packet number
	/*!
 	*/
	uint8_t packnum;

	//! Structure Variable : Packet length
	/*!
 	*/
	uint8_t length;

	//! Structure Variable : Packet payload
	/*!
 	*/
	uint8_t data[251];

	//! Structure Variable : Retry number
	/*!
 	*/
	uint8_t retry;
}pack;

typedef enum
{
    true=1, false=0
}bool;

/******************************************************************************
 * SX1278 FUNCTION*************************************************************
 ******************************************************************************/
		void SX1278(void);
		//REG CONTROLLING FUNCTION
		uint16_t HW_SPI_InOut(SPI_HandleTypeDef *hspi, uint16_t txData );
		void SX1272ReadBuffer(SPI_HandleTypeDef *hspi, uint8_t addr, uint8_t *buffer, uint8_t size );
		void SX1272WriteBuffer(SPI_HandleTypeDef *hspi, uint8_t addr, uint8_t *buffer, uint8_t size );
  	//! It reads an internal module register.
  	/*!
  	\param uint8_t address : address register to read from.
		\return the content of the register.
		*/
		uint8_t readRegister(SPI_HandleTypeDef *hspi, uint8_t address);

		//! It writes in an internal module register.
  	/*!
  	\param uint8_t address : address register to write in.
  	\param uint8_t data : value to write in the register.
		*/
		void writeRegister(SPI_HandleTypeDef *hspi, uint8_t address, uint8_t data);
		//! It puts the module ON
  	/*!
		\param void
		\return void
		*/
		//END REG CONTROLLING FUNCTION
		float timeOnAir( uint16_t payloadlength );

		//! It sets the payload of the packet that is going to be sent.
  	/*!
  	\param char *payload : packet payload.
		\return '0' on success, '1' otherwise
		*/
		uint8_t setPayload(SPI_HandleTypeDef *hspi, char *payload);

		//! It checks if there is an available packet and its destination before a
		//! timeout.
  	/*!
  	\param uint32_t wait : time to wait while there is no a valid header received.
		\return 'true' on success, 'false' otherwise
		*/
		_Bool	availableData(SPI_HandleTypeDef *hspi, uint32_t wait);

		//! It checks if there is an available packet and its destination.
		/*!
		\return 'true' on success, 'false' otherwise
		*/
		_Bool	availableData_MAX_TIMEOUT(SPI_HandleTypeDef *hspi);

		//! It clears the interruption flags.
  	/*!
		\param void
		\return void
		*/
		void clearFlags(SPI_HandleTypeDef *hspi);

		//! It receives and gets an ACK from FIFO, if it arrives before ending
		//! 'wait' time.
		/*!
		 *
		\param uint32_t wait : time to wait while there is no an ACK received.
		\return '8'  --> The ACK lost
				'7'  --> The ACK destination incorrectly received
				'6'  --> The ACK source incorrectly received
				'5'  --> The ACK number incorrectly received
				'4'  --> The ACK length incorrectly received
				'3'  --> N-ACK received
				'2'  --> The ACK has not been received
				'1'  --> not used (reserved)
				'0'  --> The ACK has been received with no errors
		*/
		uint8_t getACK(SPI_HandleTypeDef *hspi, uint32_t wait);
		//! If an ACK is received, it gets it and checks its content.
		/*!
		 *
		\return '8'  --> The ACK lost
				'7'  --> The ACK destination incorrectly received
				'6'  --> The ACK source incorrectly received
				'5'  --> The ACK number incorrectly received
				'4'  --> The ACK length incorrectly received
				'3'  --> N-ACK received
				'2'  --> The ACK has not been received
				'1'  --> not used (reserved)
				'0'  --> The ACK has been received with no errors
		*/
		uint8_t getACK_MAX_TIMEOUT(SPI_HandleTypeDef *hspi);

		//! It is true if the BW selected exists.
  	/*!
		\param uint16_t band : bandwidth value to check.
		\return 'true' on success, 'false' otherwise
		*/
		_Bool	isBW(uint16_t band);

		//! It gets the BW configured.
  	/*!
		It stores in global '_bandwidth' variable the BW selected in the configuration
		\return '0' on success, '1' otherwise
		*/
		int8_t	getBW(SPI_HandleTypeDef *hspi);

		//! It is true if the CR selected exists.
  	/*!
		\param uint8_t cod : the coding rate value to check.
		\return 'true' on success, 'false' otherwise
		*/
		_Bool	isCR(uint8_t cod);

		//! It gets the CR configured.
  	/*!
		It stores in global '_codingRate' variable the CR selected
		in the configuration
		\return '0' on success, '1' otherwise
		*/
		int8_t	getCR(SPI_HandleTypeDef *hspi);

		//! It gets the CRC configured.
  	/*!
  	It stores in global '_CRC' variable '1' enabling CRC generation on
  	payload, or '0' disabling the CRC.
		\return '0' on success, '1' otherwise
		*/
		uint8_t	getCRC(SPI_HandleTypeDef *hspi);

		//! It is true if the channel selected exists.
  	/*!
		\param uint32_t ch : frequency channel value to check.
		\return 'true' on success, 'false' otherwise
		*/
		_Bool isChannel(uint32_t ch);

		//! It gets frequency channel the module is using.
  	/*!
		It stores in global '_channel' variable the frequency channel
		\return '0' on success, '1' otherwise
		*/
		uint8_t getChannel(SPI_HandleTypeDef *hspi);

		//! It gets the header mode configured.
  	/*!
  	It stores in global '_header' variable '0' when header is sent
  	(explicit header mode) or '1' when is not sent (implicit header
  	mode).
		\return '0' on success, '1' otherwise
		*/
		uint8_t	getHeader(void);

		//! It gets the maximum current supply by the module.
		/*!
  	\return '0' on success, '1' otherwise
		*/
		uint8_t getMaxCurrent(SPI_HandleTypeDef *hspi);

		//! It sets the LoRa mode on.
  	/*!
  	It stores in global '_LORA' variable '1' when success
    \return '0' on success, '1' otherwise
		*/
		uint8_t setLORA(SPI_HandleTypeDef *hspi);


		//! It sets the maximum current supply by the module.
		/*!
		It stores in global '_maxCurrent' variable the maximum current supply.
  	\param uint8_t rate : maximum current supply.
		\return '0' on success, '1' otherwise
		*/
		int8_t setMaxCurrent(SPI_HandleTypeDef *hspi, uint8_t rate);

		//! It sets the node address of the mote.
  	/*!
  	It stores in global '_nodeAddress' variable the node address
  	\param uint8_t addr : address value to set as node address.
		\return '0' on success, '1' otherwise
		*/
		int8_t setNodeAddress(SPI_HandleTypeDef *hspi, uint8_t addr);

		//! It sets the destination of a packet.
  	/*!
  	\param uint8_t dest : value to set as destination address.
		\return '0' on success, '1' otherwise
		*/
		int8_t setDestination(uint8_t dest);

		//! It sets the maximum number of bytes from a frame that fit in a packet
		//! structure.
		/*!
		It stores in global '_payloadlength' variable the maximum number of bytes.
  	\param uint16_t length16 : total frame length.
		\return '0' on success, '1' otherwise
		*/
		uint8_t truncPayload(uint16_t length16);

		//! It sets the packet length to send/receive.
  	/*!
  	It stores in global '_payloadlength' variable the payload length of
    the last packet to send/receive.
  	\param uint8_t l : payload length to set in the configuration.
		\return '0' on success, '1' otherwise
		*/
		int8_t setPacketLength(SPI_HandleTypeDef *hspi, uint8_t l);

		//! It sets the packet length to send/receive.
  	/*!
  	It stores in global '_payloadlength' variable the payload length of
    the last packet to send/receive.
		\return '0' on success, '1' otherwise
		*/
		int8_t setPacketLength_payloadlength(SPI_HandleTypeDef *hspi);

		//! It writes a packet in FIFO in order to send it.
		/*!
		\param uint8_t dest : packet destination.
		\param char *payload : packet payload.
		\return '0' on success, '1' otherwise
		*/
		uint8_t setPacket(SPI_HandleTypeDef *hspi, uint8_t dest, char *payload);

		//! It is true if the SF selected exists.
  	/*!
		\param uint8_t spr : spreading factor value to check.
		\return 'true' on success, 'false' otherwise
		*/
		_Bool	isSF(uint8_t spr);

		//! It sets implicit header mode.
  	/*!
  	It stores in global '_header' variable '0' when success
		\return '0' on success, '1' otherwise
		*/
		int8_t	setHeaderOFF(SPI_HandleTypeDef *hspi);

		//! It sets the SF.
  	/*!
		It stores in global '_spreadingFactor' variable the current value of SF
		\param uint8_t spr : spreading factor value to set in the configuration.
		\return '0' on success, '1' otherwise
		*/
		uint8_t	setSF(SPI_HandleTypeDef *hspi, uint8_t spr);

		//! It writes an ACK in FIFO to send it.
		/*!
		*
		\return '0' on success, '1' otherwise
		*/
		uint8_t setACK(SPI_HandleTypeDef *hspi);

		//! It gets the SF configured.
  	/*!
		It stores in global '_spreadingFactor' variable the current value of SF
		\return '0' on success, '1' otherwise
		*/
		int8_t	getSF(SPI_HandleTypeDef *hspi);

		//! It sets the BW.
  	/*!
		It stores in global '_bandwidth' variable the BW selected
		in the configuration
		\param uint16_t band : bandwidth value to set in the configuration.
		\return '0' on success, '1' otherwise
		*/
		int8_t setBW(SPI_HandleTypeDef *hspi, uint16_t band);

		//! It sets the CR.
  	/*!
		It stores in global '_codingRate' variable the CR selected
		in the configuration
		\param uint8_t cod : coding rate value to set in the configuration.
		\return '0' on success, '1' otherwise
		*/
		int8_t	setCR(SPI_HandleTypeDef *hspi, uint8_t cod);

		//! It sets CRC on.
  	/*!
  	It stores in global '_CRC' variable '1' when success
		\return '0' on success, '1' otherwise
		*/
		uint8_t	setCRC_ON(SPI_HandleTypeDef *hspi);

		//! It sets CRC off.
  	/*!
  	It stores in global '_CRC' variable '0' when success
		\return '0' on success, '1' otherwise
		*/
		uint8_t	setCRC_OFF(SPI_HandleTypeDef *hspi);


		//! It sets frequency channel the module is using.
  	/*!
		It stores in global '_channel' variable the frequency channel
		\param uint32_t ch : frequency channel value to set in the configuration.
		\return '0' on success, '1' otherwise
		*/
		int8_t setChannel(SPI_HandleTypeDef *hspi, uint32_t ch);

		//! It sets the FSK mode on.
  	/*!
  	It stores in global '_FSK' variable '1' when success
		\return '0' on success, '1' otherwise
		*/
		uint8_t setFSK(SPI_HandleTypeDef *hspi);

		//! It sets explicit header mode.
  	/*!
  	It stores in global '_header' variable '1' when success
		\return '0' on success, '1' otherwise
		*/
		int8_t	setHeaderON(SPI_HandleTypeDef *hspi);

		//! It sets the BW, SF and CR of the module.
  	/*!
		It stores in global '_bandwidth' variable the BW
		It stores in global '_codingRate' variable the CR
		It stores in global '_spreadingFactor' variable the SF
		\param uint8_t mode : there is a mode number to different values of
		the	configured parameters with this function.
		\return '0' on success, '1' otherwise
		*/
		int8_t setMode(SPI_HandleTypeDef *hspi, uint8_t mode);

		//! It sets the output power of the signal.
  	/*!
		It stores in global '_power' variable the output power of the signal
		\param char p : 'M', 'H' or 'L' if you want Maximum, High or Low
		output power signal.
		\return '0' on success, '1' otherwise
		*/
		int8_t setPower(SPI_HandleTypeDef *hspi, char p);

		//! It sets the output power of the signal.
		/*!
		It stores in global '_power' variable the output power of the signal
		\param uint8_t pow : value to set as output power.
		\return '0' on success, '1' otherwise
		*/
		int8_t setPowerNum(SPI_HandleTypeDef *hspi, uint8_t pow);

		//! It gets the preamble length configured.
  	/*!
		It stores in global '_preamblelength' variable the preamble length
		\return '0' on success, '1' otherwise
		*/
		uint8_t getPreambleLength(SPI_HandleTypeDef *hspi);

		//! It sets the preamble length.
  	/*!
    It stores in global '_preamblelength' variable the preamble length
  	\param uint16_t l : preamble length to set in the configuration.
		\return '0' on success, '1' otherwise
		*/
		uint8_t setPreambleLength(SPI_HandleTypeDef *hspi, uint16_t l);

		//! It sets the total of retries when a packet is not correctly received.
		/*!
		It stores in global '_maxRetries' variable the number of retries.
  	\param uint8_t ret : number of retries.
		\return '0' on success, '1' otherwise
		*/
		uint8_t setRetries(SPI_HandleTypeDef *hspi, uint8_t ret);

		//! It sets the waiting time to send a packet.
  	/*!
   	It stores in global '_sendTime' variable the time for each mode.
		\return '0' on success, '1' otherwise
		*/
		uint8_t setTimeout(void);

		//! It puts the module ON
		uint8_t ON(SPI_HandleTypeDef *hspi);

		//! It puts the module OFF
  	/*!
		\param void
		\return void
		*/
		void OFF(void);

		//! It gets the BW, SF and CR of the module.
  	/*!
		It stores in global '_bandwidth' variable the BW
		It stores in global '_codingRate' variable the CR
		It stores in global '_spreadingFactor' variable the SF
		\return '0' on success, '1' otherwise
		*/
		uint8_t getMode(SPI_HandleTypeDef *hspi);


		//! It gets the node address of the mote.
  	/*!
  	It stores in global '_nodeAddress' variable the node address
		\return '0' on success, '1' otherwise
		*/
		uint8_t getNodeAddress(SPI_HandleTypeDef *hspi);

		//! It receives and gets a packet from FIFO, if it arrives before ending
		//! 'wait' time.
		/*!
		*
		\param uint32_t wait : time to wait while there is not a complete packet received.
		\return '0' on success, '1' otherwise
		*/
		int8_t getPacket(SPI_HandleTypeDef *hspi, uint32_t wait);

		//! It reads a received packet from the FIFO, if it arrives before ending
		//! MAX_TIMEOUT time.
		/*!
		*
		\return '0' on success, '1' otherwise
		*/
		uint8_t getPacketMAXTimeout(SPI_HandleTypeDef *hspi);

		//! It gets the payload length of the last packet to send/receive.
  	/*!
    It stores in global '_payloadlength' variable the payload length of
    the last packet to send/receive.
		\return '0' on success, '1' otherwise
		*/
		uint8_t getPayloadLength(SPI_HandleTypeDef *hspi);

		//! It gets the output power of the signal.
  	/*!
		It stores in global '_power' variable the output power of the signal
		\return '0' on success, '1' otherwise
		*/
		uint8_t getPower(SPI_HandleTypeDef *hspi);

		//! It gets the internal temperature of the module.
		/*!
		It stores in global '_temp' variable the module temperature.
		\return '0' on success, '1' otherwise
		*/
		uint8_t getTemp(SPI_HandleTypeDef *hspi);

		//! It gets the content of the main configuration registers.
  	/*!
		It stores in global '_bandwidth' variable the BW.
		It stores in global '_codingRate' variable the CR.
		It stores in global '_spreadingFactor' variable the SF.
		It stores in global '_power' variable the output power of the signal.
  	It stores in global '_channel' variable the frequency channel.
  	It stores in global '_CRC' variable '1' enabling CRC generation on
  	payload, or '0' disabling the CRC.
  	It stores in global '_header' variable '0' when header is sent (explicit header mode) or '1' when is not sent (implicit header mode).
		It stores in global '_preamblelength' variable the preamble length.
    It stores in global '_payloadlength' variable the payload length of the last packet to send/receive.
  	It stores in global '_nodeAddress' variable the node address.
		It stores in global '_temp' variable the module temperature.
		\return '0' on success, '1' otherwise
		*/
		uint8_t getRegs(SPI_HandleTypeDef *hspi);

		//! It gets the current value of RSSI.
  	/*!
		It stores in global '_RSSI' variable the current value of RSSI
		\return '0' on success, '1' otherwise
		*/
		uint8_t getRSSI(SPI_HandleTypeDef *hspi);

		//! It gets the SNR of the latest received packet.
  	/*!
		It stores in global '_SNR' variable the SNR
		\return '0' on success, '1' otherwise
		*/
		int8_t getSNR(SPI_HandleTypeDef *hspi);

		//! It gets the RSSI of the latest received packet.
  	/*!
		It stores in global '_RSSIpacket' variable the RSSI of the latest
		packet received.
		\return '0' on success, '1' otherwise
		*/
		int16_t getRSSIpacket(SPI_HandleTypeDef *hspi);

		//! It tries to send the packet stored in FIFO before ending 'wait' time.
		/*!
		\param uint32_t wait : time to wait to send the packet.
		\return '0' on success, '1' otherwise
		*/
		uint8_t sendWithTimeout(SPI_HandleTypeDef *hspi, uint32_t wait);

		//! It sends the packet stored in FIFO before ending _sendTime time.
		/*!
		*
		\return '0' on success, '1' otherwise
		*/
		uint8_t sendWithTimeout_sendtime(SPI_HandleTypeDef *hspi);

		//! It sends the packet which payload is a parameter before ending
		//! MAX_TIMEOUT.
		/*!
		\param uint8_t dest : packet destination.
		\param char *payload : packet payload.
		\return '0' on success, '1' otherwise
		*/

		uint8_t sendPacketTimeout_sendtime(SPI_HandleTypeDef *hspi,	uint8_t dest, char *payload);

		//! It puts the module in reception mode.
  	/*!
  	 *
		\return '0' on success, '1' otherwise
		*/
		uint8_t receive(SPI_HandleTypeDef *hspi);

		//! It puts the module in 'promiscuous' reception mode with a timeout.
  	/*!
  	\param uint32_t wait : time to wait to receive something.
		\return '0' on success, '1' otherwise
		*/
		uint8_t receiveAll(SPI_HandleTypeDef *hspi, uint32_t wait);

		//! It puts the module in 'promiscuous' reception mode with a MAX_timeout.
  	/*!
  	\param uint32_t wait : time to wait to receive something.
		\return '0' on success, '1' otherwise
		*/
		uint8_t	receiveAll_MAX_TIMEOUT(SPI_HandleTypeDef *hspi);

		//! It receives a packet before a timeout.
  	/*!
  	\param uint32_t wait : time to wait to receive something.
		\return '0' on success, '1' otherwise
		*/
		uint8_t receivePacketTimeout(SPI_HandleTypeDef *hspi, uint32_t wait);

		//! It receives a packet before a timeout and reply with an ACK.
  	/*!
  	\param uint32_t wait : time to wait to receive something.
		\return '0' on success, '1' otherwise
		*/
		uint8_t receivePacketTimeoutACK(SPI_HandleTypeDef *hspi, uint32_t wait);
		uint8_t receivePacketTimeout_sendTime(SPI_HandleTypeDef *hspi);
		uint8_t receivePacketMAXTimeout(SPI_HandleTypeDef *hspi);
		uint8_t receivePacketMAXTimeoutACK(SPI_HandleTypeDef *hspi);
		uint8_t receivePacketTimeoutACK_sendTime(SPI_HandleTypeDef *hspi);
		uint8_t sendPacketTimeout_WaitTime(SPI_HandleTypeDef *hspi, uint8_t dest, char *payload, uint16_t wait);
		uint8_t sendPacketTimeout_length(SPI_HandleTypeDef *hspi, uint8_t dest, char *payload, uint16_t length16);
		uint8_t sendPacketTimeout_wait_length(SPI_HandleTypeDef *hspi, uint8_t dest, char *payload, uint16_t length16, uint16_t wait);
		uint8_t sendPacketTimeoutACK(SPI_HandleTypeDef *hspi, uint8_t dest, char *payload);
		uint8_t sendPacketTimeoutACK_length(SPI_HandleTypeDef *hspi,	uint8_t dest, char *payload, uint16_t length16);
		uint8_t sendPacketTimeoutACK_wait(SPI_HandleTypeDef *hspi,
											uint8_t dest,
											char *payload,
											uint16_t wait);
		uint8_t sendPacketTimeoutACK_wait_length(SPI_HandleTypeDef *hspi, uint8_t dest,
											char *payload,
											uint16_t length16,
											uint32_t wait);
		uint8_t sendPacketTimeoutACKRetries(SPI_HandleTypeDef *hspi, uint8_t dest, char *payload);
		uint8_t sendPacketTimeoutACKRetries_length(SPI_HandleTypeDef *hspi, uint8_t dest,
												char *payload,
												uint16_t length16);
		uint8_t sendPacketTimeoutACKRetries_wait(SPI_HandleTypeDef *hspi, uint8_t dest,
												char *payload,
												uint32_t wait);
		uint8_t sendPacketTimeoutACKRetries_length_wait(SPI_HandleTypeDef *hspi,
												uint8_t dest,
												char *payload,
												uint16_t length16,
												uint32_t wait);
		uint8_t sendWithMAXTimeout(SPI_HandleTypeDef *hspi);
#endif
