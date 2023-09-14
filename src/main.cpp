/*!
 * \author Cody Roberson
 * \details AliCPT Bias Board Control Firmware
 * \version 2.0
 * \date 20230913
 * \copyright
    AliCPT Bias Board Control Firmware
    Copyright (C) 2023 FotonX

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.

  Uses xmodem crc16 algorithm sourced here:
    http://www.nongnu.org/avr-libc/user-manual/group__util__crc.html

 */
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_INA219.h>
#define SW_VERSION_NUMBER 2.0

// *********** I2C DEVICE ADDRESSES *****************
#define __DIGITALPOT_1_I2C_ADDR 0b0101111
#define __DIGITALPOT_2_I2C_ADDR 0b0100011
#define __I2C_REPEATER_I2C_ADDR 0b1100000
#define __IO_EXPANDER_I2C_ADDR 0b0100000
#define __CURR_SENSE_BASE_I2C_ADDR 0b1001000

// *********** I2C DEVICE COMMANDS *****************
#define __AD5144_CMD_WRITE_RDAC 0b00010000
#define __AD5144_CMD_READ_RDAC_8 0b00110000
#define __AD5144_CMD_READ_RDAC_0 0b00000011
#define __LTC4302_CMD_CONNECT_CARD 0b11100000
#define __LTC4302_CMD_DISCONNECT_CARD 0b01100000

// Any interfacing application shall use this structure when communicating with this device
struct Packet
{
  uint32_t command;
  uint32_t arg1;
  uint32_t arg2;
  uint32_t arg3;
  uint32_t checksum;
};

uint16_t crc_xmodem_update(uint16_t crc, uint8_t data)
{
  int i;

  crc = crc ^ ((uint16_t)data << 8);
  for (i = 0; i < 8; i++)
  {
    if (crc & 0x8000)
      crc = (crc << 1) ^ 0x1021; //(polynomial = 0x1021)
    else
      crc <<= 1;
  }
  return crc;
}

uint16_t calc_crc(char *msg, int n)
{

  uint16_t x = 0;

  while (n--)
  {
    x = crc_xmodem_update(x, (uint16_t)*msg++);
  }
  return (x);
}
/***************************************************************************
 * Customized Implementation
 * ************************************************************************/
uint16_t IO_EXPANDER_PINSTATE = 0;
int current_LTC_addr = 0;
Adafruit_INA219 *currsense[8];
int setExpander(int pin, int state)
{
  IO_EXPANDER_PINSTATE ^= (-state ^ IO_EXPANDER_PINSTATE) & (1 << pin);
  Wire.beginTransmission(__IO_EXPANDER_I2C_ADDR);
  Wire.write((uint8_t)(IO_EXPANDER_PINSTATE & 0xFF));
  Wire.write((uint8_t)((IO_EXPANDER_PINSTATE >> 8) & 0xFF));
  return Wire.endTransmission();
}
int setExpander(uint16_t pinsetting)
{
  IO_EXPANDER_PINSTATE = pinsetting;
  Wire.beginTransmission(__IO_EXPANDER_I2C_ADDR);
  Wire.write((uint8_t)(IO_EXPANDER_PINSTATE & 0xFF));
  Wire.write((uint8_t)((IO_EXPANDER_PINSTATE >> 8) & 0xFF));
  return Wire.endTransmission();
}
int setWiper(uint8_t dpot_i2c_addr, uint8_t wiper, uint32_t val)
{
  int status = 0;

  Wire.beginTransmission(dpot_i2c_addr);
  Wire.write(__AD5144_CMD_WRITE_RDAC + wiper); // Write to wiper/RDAC1
  Wire.write((uint8_t)val);                    // write value to chip

  status = Wire.endTransmission();
  return status;
}
int getWiper(uint8_t dpot_i2c_addr, uint8_t wiper, uint32_t *wipervalue)
{
  int status = 0;

  Wire.beginTransmission(dpot_i2c_addr);
  Wire.write(__AD5144_CMD_READ_RDAC_0 + wiper); // Write to wiper/RDAC1
  Wire.write(__AD5144_CMD_READ_RDAC_8);         // write value to chip
  status = Wire.endTransmission();

  if (status == 0)
  {
    Wire.beginTransmission(dpot_i2c_addr);
    *wipervalue = Wire.read() & 0xFF;
    status = Wire.endTransmission();
  }

  return status;
}

int connect_i2c_bus(uint8_t address)
{
  Wire.beginTransmission(__I2C_REPEATER_I2C_ADDR + address);
  Wire.write(__LTC4302_CMD_CONNECT_CARD); // Connect IIC Bus
  return Wire.endTransmission();
}

int disconnect_i2c_bus(uint8_t address)
{
  Wire.beginTransmission(__I2C_REPEATER_I2C_ADDR + address);
  Wire.write(__LTC4302_CMD_DISCONNECT_CARD);
  return Wire.endTransmission();
}

/***************************************************************************
 * End of Customized Implementation
 * ************************************************************************/
void setup()
{
  Serial.begin(115200);
  Wire.begin();
  connect_i2c_bus(__I2C_REPEATER_I2C_ADDR);
  delay(1);
  for (int i = 0; i < 8; i++)
  {
    currsense[i] = new Adafruit_INA219(__CURR_SENSE_BASE_I2C_ADDR + i);
    currsense[i]->begin();
  }
}

/*!
  Process incomming command
  \param pkt Incomming packet
 */
Packet do_command(Packet pkt)
{
  Packet retpacket;
  switch (pkt.command)
  {
  case 1: // Check Connection
    retpacket.command = pkt.command;
    retpacket.arg1 = 1;
    break;
  case 2: // get wiper
    retpacket.command = pkt.command;
    if (pkt.arg1 == 1)
      retpacket.arg2 = getWiper(__DIGITALPOT_1_I2C_ADDR, pkt.arg2, &(retpacket.arg1));
    else
      retpacket.arg2 = getWiper(__DIGITALPOT_2_I2C_ADDR, pkt.arg2, &(retpacket.arg1));
    break;
  case 3: // set wiper
    retpacket.command = pkt.command;
    if (pkt.arg1 == 1)
      retpacket.arg1 = setWiper(__DIGITALPOT_1_I2C_ADDR, pkt.arg2, pkt.arg3);
    else
      retpacket.arg1 = setWiper(__DIGITALPOT_2_I2C_ADDR, pkt.arg2, pkt.arg3);
    break;
  case 4: // get gpio
    retpacket.command = pkt.command;
    retpacket.arg1 = IO_EXPANDER_PINSTATE;
    break;
  case 5: // set all gpio
    retpacket.command = pkt.command;
    retpacket.arg1 = setExpander((uint16_t)pkt.arg1);
    break;
  case 6: // set gpio pin
    retpacket.command = pkt.command;
    retpacket.arg1 = setExpander(pkt.arg1, pkt.arg2);
    break;
  case 7: // get IV from INA219
    retpacket.command = pkt.command;
    retpacket.arg1 = (uint32_t)currsense[pkt.arg1]->getBusVoltage_V();
    retpacket.arg2 = (uint32_t)currsense[pkt.arg1]->getShuntVoltage_mV();
    retpacket.arg3 = (uint32_t)currsense[pkt.arg1]->getCurrent_mA();
    break;

  default:
    retpacket.command = 0xffffffff;
    break;
  }
  retpacket.checksum = calc_crc((char *)&retpacket, sizeof(retpacket) - sizeof(retpacket.checksum));
  return retpacket;
}

void loop()
{
  // Read a packet from Serial
  Packet receivedPacket;
  Packet ret;
  unsigned int savail = (unsigned int)Serial.available();
  if (savail >= sizeof(receivedPacket))
  {
    Serial.readBytes((char *)&receivedPacket, sizeof(receivedPacket));

    // Calculate CRC-16 checksum of the received packet (excluding the checksum field)
    uint16_t calculatedChecksum = calc_crc((char *)&receivedPacket, sizeof(receivedPacket) - sizeof(receivedPacket.checksum));

    while (Serial.available() > 0)
      Serial.read();

    // Check if the received checksum matches the calculated checksum
    if (calculatedChecksum == receivedPacket.checksum)
    {
      // Check the command byte and perform actions accordingly
      ret = do_command(receivedPacket);
      Serial.write((uint8_t *)&ret, sizeof(ret));
    }
    else
    {
      // Checksum mismatch, handle accordingly
      // ...
      ret.command = 0xfffffffe;
      Serial.write((uint8_t *)&ret, sizeof(ret));
    }
  }
  delay(50);
}
