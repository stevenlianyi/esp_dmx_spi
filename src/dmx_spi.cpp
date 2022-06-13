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

#include <dmx_spi.h>


DMX_SPI::DMX_SPI()
{
  spi_master_tx_buf = master.allocDMABuffer(SPI_MAXSTER_DMX_BUFFER_LENGTH);
  spi_master_rx_buf = master.allocDMABuffer(SPI_MAXSTER_DMX_BUFFER_LENGTH);

  memset(spi_master_tx_buf, 0, SPI_MAXSTER_DMX_BUFFER_LENGTH);
  memset(spi_master_rx_buf, 0, SPI_MAXSTER_DMX_BUFFER_LENGTH);

  _sckPin = SPI_MASTER_DMX_SCK_PIN;
  _misoPin = SPI_MASTER_DMX_MISO_PIN;
  _mosiPin = SPI_MASTER_DMX_MOSU_PIN;
  _ssPin = SPI_MASTER_DMX_SS_PIN;

  _packageSize = SPI_MASTER_DMX_CHANNEL_DEFAULT;
}


bool DMX_SPI::begin(uint16_t packageSize)
{
  return begin(_sckPin, _misoPin, _mosiPin, _ssPin, packageSize);
}


bool DMX_SPI::begin(uint8_t sckPin, uint8_t misoPin, uint8_t mosiPin, uint8_t ssPin, uint16_t packageSize)
{
  bool ret = true;

  _sckPin = sckPin;
  _misoPin = misoPin;
  _mosiPin = mosiPin;
  _ssPin = ssPin;

  _packageSize = packageSize;

  actualDataFrameLen = calc_channnel_data_len(_packageSize);
  actualDataFrameLen = (actualDataFrameLen + 8 - 1) / 8 * 8;

  master.setDataMode(SPI_MODE0);           // default: SPI_MODE0
  master.setFrequency(SPI_MASTER_DMX_FREQUENCY_DEFAULT);

  master.setMaxTransferSize(actualDataFrameLen);
  master.begin(HSPI, _sckPin, _misoPin, _mosiPin, _ssPin);

  return ret;
}


uint8_t DMX_SPI::write(uint16_t channel, uint8_t value)
{
  uint8_t data[2];
  data[0] = value;
  data[1] = 0;
  write_all(data, channel, 1);
  return value;
}


uint16_t DMX_SPI::write_all(uint8_t * data, size_t size)
{
  write_all(data, 1, size);

}

uint16_t DMX_SPI::write_all(uint8_t * data, uint16_t start, size_t size)
{
  uint16_t actualSize;
  uint8_t *dataPtr;
  memset(spi_master_tx_buf, 0, SPI_MAXSTER_DMX_BUFFER_LENGTH);

  if ((start + size) > (SPI_MASTER_DMX_DATA_MAX_LEN))
  {
    actualSize = SPI_MASTER_DMX_DATA_MAX_LEN - start;
  }
  else
  {
    actualSize = size;
  }
  dataPtr = spi_master_tx_buf;
  dataPtr += (start - 1);

  init_dmx_data(actualSize, data, dataPtr);

  DBGPRINTF("\n-> dmxChannelData ");
  print_data(dmxChannelDataLen, dmxChannelData);

  return actualSize;
}


short DMX_SPI::init_dmx_data(short dataLen, uint8_t *inputData, uint8_t *outputData)
{
  short ret = 0;
  short dataPos = 0;
  short i;
  short k;

  DBGPRINTF("\n-> init_dmx_data,[%d]", dataLen);
  //fill header
  //fill MTBP
  for (i = 0; i < SPI_MASTER_DMX_MTBP_LEN; i++)
  {
    outputData[dataPos] = SPI_MASTER_DMX_MTBP_VAL;
    dataPos++;
  }

  //fill BREAK
  for (i = 0; i < SPI_MASTER_DMX_BREAK_LEN; i++)
  {
    outputData[dataPos] = SPI_MASTER_DMX_BREAK_VAL;
    dataPos++;
  }

  //fill MAB
  for (i = 0; i < SPI_MASTER_DMX_MAB_LEN; i++)
  {
    outputData[dataPos] = SPI_MASTER_DMX_MAB_VAL;
    dataPos++;
  }

  //file frame data
  //fill start code

  //fill start bit
  outputData[dataPos] = SPI_MASTER_DMX_FRAME_START_VAL;
  dataPos++;
  //fill val
  for (k = 0; k < 8; k++)
  {
    outputData[dataPos] = 0;
    dataPos++;
  }
  //fill stop bits
  outputData[dataPos] = SPI_MASTER_DMX_FRAME_STOP_VAL;
  dataPos++;
  outputData[dataPos] = SPI_MASTER_DMX_FRAME_STOP_VAL;
  dataPos++;

  //fill data
  //低位在前
  DBGPRINTF("\n data:[%]", dataLen);
  for (i = 0; i < dataLen ; i++)
  {
    uint8_t dataVal;
    dataVal = inputData[i];
    DBGPRINTF("\n %02X:", dataVal);
    //fill start bit
    outputData[dataPos] = SPI_MASTER_DMX_FRAME_START_VAL;
    dataPos++;
    //fill val
    uint8_t bitVal;
    for (k = 0; k < 8; k++)
    {
      bitVal = (dataVal >> k) & 0x1;
      if (bitVal > 0)
      {
        outputData[dataPos] = 0xFF;
      }
      else
      {
        outputData[dataPos] = 0;
      }
      DBGPRINTF(" %02X", outputData[dataPos]);
      dataPos++;
    }
    //fill stop bits
    outputData[dataPos] = SPI_MASTER_DMX_FRAME_STOP_VAL;
    dataPos++;
    outputData[dataPos] = SPI_MASTER_DMX_FRAME_STOP_VAL;
    dataPos++;

  }
  ret = dataPos;

  return ret;
}


short DMX_SPI::calc_channnel_data_len(short dataLen)
{
  short ret = 0;
  short dataPos = 0;

  DBGPRINTF("\n-> calc_channnel_data_len,[%d]", dataLen);
  //fill header
  //fill MTBP
  ret += SPI_MASTER_DMX_MTBP_LEN;

  //fill BREAK
  ret += SPI_MASTER_DMX_BREAK_LEN;

  //fill MAB
  ret += SPI_MASTER_DMX_MAB_LEN;

  //file frame data
  //fill start code

  //fill start bit
  ret += SPI_MASTER_DMX_ONE_FRAME_BYTES;

  //fill data
  ret += SPI_MASTER_DMX_ONE_FRAME_BYTES * dataLen;

  return ret;
}


void DMX_SPI::send()
{
  master.transfer(spi_master_tx_buf, spi_master_rx_buf, actualDataFrameLen);
}


uint8_t DMX_SPI::set_speed(uint8_t speed)
{
  _dmxSpeed = speed;
  return _dmxSpeed;
}


uint8_t DMX_SPI::read_speed()
{
  return _dmxSpeed;
}

uint16_t DMX_SPI::read_package_size()
{
  return _packageSize;
}


void DMX_SPI::print_data(short dataLen, uint8_t *data)
{
  for (size_t i = 0; i < dataLen; ++i)
  {
    if (i % 32 == 0)
      printf("\n");
    printf(" %02X", data[i]);
  }
  printf("\n");
}
