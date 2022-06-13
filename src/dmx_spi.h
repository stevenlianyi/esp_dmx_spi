/*

   This library allows for transmitting ANSI-ESTA E1.11 DMX-512A using an Espressif ESP32. It is based on ESP SPI interface.

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, version 3.

   This program is distributed in the hope that it will be useful, but
   WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
   General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

#include <ESP32DMASPIMaster.h>

extern "C" {
#include "miscCommon.h"
}


#ifndef ESP32_DMX_SPI_h
#define ESP32_DMX_SPI_h

#define SPI_MASTER_DMX_FREQUENCY_MIN 1920000 //
#define SPI_MASTER_DMX_FREQUENCY_MAX 2080000 //
#define SPI_MASTER_DMX_FREQUENCY_STANDARD 2000000 //Default SPI rate of 2Mhz is exactly 8 times the DMX 250K Hz rate

#define SPI_MASTER_DMX_FREQUENCY_DEFAULT SPI_MASTER_DMX_FREQUENCY_STANDARD //default: 2MHz 


//spi bus rate is 2MHz, each bit occupies 0.5 us (500ns)
#define SPI_MASTER_DMX_MTBP_LEN 0 //MTBP number of bytes, min 100 us, length = 0
#define SPI_MASTER_DMX_BREAK_LEN 200 //break number of bytes, , min 88 us, max 100000, default = 88, length = 88*2/8 = 22
#define SPI_MASTER_DMX_MAB_LEN 10 //MAB,mark after break number of bytes min 4us, max 12 us, default = 8, length = 8*2/8 = 2 
#define SPI_MASTER_DMX_DATA_MAX_LEN 513 // Includes start 0, 11 bits per frame, with 1 start bit, 8 data bits and 2 end bits
// Considering that DMX512 operates at 250KHz, SPI is currently 2MHz, which is 8 times higher, so exactly 1 byte corresponds to 1 bit of DMX512. 11 bytes correspond to one frame of DMX
//so the number of bytes needed is 11 * 513 = 5643, taking into account other data at least 0+22+2+5643 = 5665

#define SPI_MASTER_DMX_ONE_FRAME_BYTES 11

#define SPI_MASTER_DMX_DATA_FRAME_LEN  (SPI_MASTER_DMX_DATA_MAX_LEN*SPI_MASTER_DMX_ONE_FRAME_BYTES)
#define SPI_MASTER_DMX_VALID_DATA_LEN (SPI_MASTER_DMX_MTBP_LEN + SPI_MASTER_DMX_BREAK_LEN + SPI_MASTER_DMX_MAB_LEN + SPI_MASTER_DMX_DATA_FRAME_LEN)

#define SPI_MAXSTER_DMX_BUFFER_LENGTH 6144
#define SPI_MAXSTER_DMX_REST_FILL_LENGTH (SPI_MAXSTER_DMX_BUFFER_LENGTH-SPI_MASTER_DMX_VALID_DATA_LEN)

#define SPI_MASTER_DMX_FRAME_START_VAL 0
#define SPI_MASTER_DMX_FRAME_STOP_VAL 0xFF

#define SPI_MASTER_DMX_MTBP_VAL 0xFF
#define SPI_MASTER_DMX_BREAK_VAL 0
#define SPI_MASTER_DMX_MAB_VAL 0xFF

#define SPI_MASTER_DMX_SCK_PIN 14
#define SPI_MASTER_DMX_MISO_PIN 12
#define SPI_MASTER_DMX_MOSU_PIN 17
#define SPI_MASTER_DMX_SS_PIN 2

#define SPI_MASTER_DMX_CHANNEL_DEFAULT 64

class DMX_SPI
{
  public:
    //data
    //func
    DMX_SPI();

    bool begin(uint16_t packageSize = SPI_MASTER_DMX_DATA_MAX_LEN);     // initialize library

    bool begin(uint8_t sckPin, uint8_t misoPin, uint8_t mosiPin, uint8_t ssPin, uint16_t packageSize = SPI_MASTER_DMX_DATA_MAX_LEN);   // initialize library

    uint8_t write(uint16_t channel, uint8_t value); // writes the dmx value to the buffer

    uint16_t write_all(uint8_t * data, uint16_t start, size_t size);  // copies the defined channels into the write buffer
    uint16_t write_all(uint8_t * data, size_t size);  // copies the defined channels into the write buffer
	
    void send(); //send dmx data
	
	uint8_t set_speed(uint8_t speed);
	uint8_t read_speed();

	uint16_t read_package_size();
	
	void print_data(short dataLen, uint8_t *data);

  private:

    //data
    ESP32DMASPI::Master master;
    uint8_t* spi_master_tx_buf;
    uint8_t* spi_master_rx_buf;

    short actualDataFrameLen;

    short dmxChannelDataLen = SPI_MASTER_DMX_CHANNEL_DEFAULT;
    uint8_t dmxChannelData[SPI_MASTER_DMX_DATA_MAX_LEN];

    uint8_t _sckPin;
    uint8_t _misoPin;
    uint8_t _mosiPin;
    uint8_t _ssPin;

    uint8_t _dmxSpeed;
	uint16_t _packageSize;

    // func
    short init_dmx_data(short dataLen, uint8_t *inputData, uint8_t *outputData);
	short calc_channnel_data_len(short dataLen);
};

#endif
