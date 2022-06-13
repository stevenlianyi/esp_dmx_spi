#include <dmx_spi.h>

uint8_t sckPin = 14;
uint8_t misoPin = 12;
//uint8_t mosiPin = 13;
uint8_t mosiPin = 17;
//uint8_t ssPin = 15;
uint8_t ssPin = 18;

short dmxChannelDataLen = 64;
uint8_t dmxChannelData[SPI_MASTER_DMX_DATA_MAX_LEN];

DMX_SPI dmxDevice;

void init_dmx_channel_data()
{
  short i;
  uint8_t step = 4;
  uint8_t val = 0xFF;
  printf("\n-> init_dmx_channel_data,[%d]", dmxChannelDataLen);
  for (i = 0; i < dmxChannelDataLen; i += step)
  {
    dmxChannelData[i + 0] = val;
    //dmxChannelData[i + 0] = val;
    //dmxChannelData[i + 1] = 0;
    dmxChannelData[i + 1] = val;
    dmxChannelData[i + 2] = 0;
    //dmxChannelData[i + 2] = val;
    dmxChannelData[i + 3] = 00;
    //dmxChannelData[i + 3] = val;
  }
}


void setup()
{
  Serial.begin(115200);

  printf("\n-> ESP32DMASPI: spi dmx testing ");

  init_dmx_channel_data();

  //dmxDevice.begin();
  dmxDevice.begin(sckPin, misoPin, mosiPin, ssPin, dmxChannelDataLen);
  dmxDevice.write_all(dmxChannelData,1,dmxChannelDataLen);
  
  dmxDevice.print_data(dmxChannelDataLen, dmxChannelData);

  delay(1000);
}

void loop()
{
  dmxDevice.send();

  delay(1000);
}
