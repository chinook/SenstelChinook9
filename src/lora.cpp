
#include "lora.hpp"

#include "main.hpp"


DigitalOut  chip_select(RFM95_CS);
DigitalOut  chip_reset(RFM95_RST);

InterruptIn chip_interrupt(RFM95_INT);

SPI spi(PA_7, PA_6, PA_5); // MOSI, MISO, SCLK

namespace lora
{

void SetModeIdle()
{
  if(mode != RHModeIdle)
  {
    SpiWrite(RH_RF95_REG_01_OP_MODE, RH_RF95_MODE_STDBY);
    wait_us(100);
    mode = RHModeIdle;
  }
}
bool Sleep()
{
  if(mode != RHModeSleep)
  {
    SpiWrite(RH_RF95_REG_01_OP_MODE, RH_RF95_MODE_SLEEP);
    wait_us(100);
    mode = RHModeSleep;
  }
}
void SetModeRx()
{
  if (mode != RHModeRx){
    SpiWrite(RH_RF95_REG_01_OP_MODE, RH_RF95_MODE_RXCONTINUOUS);
    wait_us(100);
    SpiWrite(RH_RF95_REG_40_DIO_MAPPING1, 0x00); // Interrupt on RxDone
    wait_us(100);
    mode = RHModeRx;
  }
}
void SetModeTx(){
  if (mode != RHModeTx){
    SpiWrite(RH_RF95_REG_01_OP_MODE, RH_RF95_MODE_TX);
    wait_us(100);
    SpiWrite(RH_RF95_REG_40_DIO_MAPPING1, 0x40); // Interrupt on TxDone
    wait_us(100);
    mode = RHModeTx;
  }
}

//
// Functions and routines for LoRa initialization
//

bool SetFrequency(float center)
{
  // Frf = FRF / FSTEP
  uint32_t frf = (center * 1000000.0) / RH_RF95_FSTEP;
  SpiWrite(RH_RF95_REG_06_FRF_MSB, (frf >> 16) & 0xff);
  SpiWrite(RH_RF95_REG_07_FRF_MID, (frf >> 8) & 0xff);
  SpiWrite(RH_RF95_REG_08_FRF_LSB, frf & 0xff);

  return true;
}
void SetTxPower(int8_t power, bool useRFO)
{
  // TODO
}
bool InitRF95()
{
  mode = RHModeInitialising;

  /// The reported device version
  uint8_t deviceVersion;

  // Initialise the slave select pin
  chip_select = 1;

  wait_ms(100);

  // Get the device type and check it
  // This also tests whether we are really connected to a device
  // My test devices return 0x83
  deviceVersion = SpiRead(RH_RF95_REG_42_VERSION);
  if (deviceVersion == 00 ||
      deviceVersion == 0xff){
    pc.printf("LoRa error : Bad device version\r\n");
    while(1);
  }
  else
  {
    //pc.printf("Device version : %d\n\r", deviceVersion);
  }

  // Set sleep mode, so we can also set LORA mode:
  SpiWrite(RH_RF95_REG_01_OP_MODE, RH_RF95_MODE_SLEEP | RH_RF95_LONG_RANGE_MODE);
  wait_ms(10); // Wait for sleep mode to take over from say, CAD

  // Check we are in sleep mode, with LORA set
  uint8_t lora_mode = SpiRead(RH_RF95_REG_01_OP_MODE);
  if (lora_mode != (RH_RF95_MODE_SLEEP | RH_RF95_LONG_RANGE_MODE))
  {
    //pc.printf("No device present\n\r");
    return false; // No device present?
  }

  chip_interrupt.rise(&Isr0);

  // Set up FIFO
  // We configure so that we can use the entire 256 byte FIFO for either receive
  // or transmit, but not both at the same time
  SpiWrite(RH_RF95_REG_0E_FIFO_TX_BASE_ADDR, 0);
  SpiWrite(RH_RF95_REG_0F_FIFO_RX_BASE_ADDR, 0);

  // Packet format is preamble + explicit-header + payload + crc
  // Explicit Header Mode
  // payload is TO + FROM + ID + FLAGS + message data
  // RX mode is implmented with RXCONTINUOUS
  // max message data length is 255 - 4 = 251 octets

  SetModeIdle();
  wait_ms(1);

  ModemConfig cfg;
  memcpy(&cfg, &MODEM_CONFIG_TABLE[0], sizeof(ModemConfig));
  SpiWrite(RH_RF95_REG_1D_MODEM_CONFIG1,       cfg.reg_1d);
  SpiWrite(RH_RF95_REG_1E_MODEM_CONFIG2,       cfg.reg_1e);
  SpiWrite(RH_RF95_REG_26_MODEM_CONFIG3,       cfg.reg_26);

  //setModemConfig(Bw125Cr48Sf4096); // slow and reliable?

  return true;
}

//
// Functions and routines for LoRa control
//

bool WaitForPacketSent(float max_wait_ms)
{
  float timeout = 0;
  while (mode == RHModeTx)
  {
    wait_us(100); // Wait for any previous transmit to finish
    timeout += 0.1;

    // Max timeout check
    if(timeout >= max_wait_ms)
      return false;
  }
  return true;
}
bool IsAvailable()
{
  if (mode == RHModeTx)
    return false;
  SetModeRx();
  return rxBufferValid; // Will be set by the interrupt handler when a good message is received
}
bool WaitAvailableTimeout(uint16_t timeout)
{
  unsigned long starttime = us_ticker_read();
  while ((us_ticker_read() - starttime) < (timeout*1000))
  {
      if (IsAvailable())
      {
        return true;
      }
      wait_us(100);
  }
  return false;
}

bool Recv(uint8_t* buffer, unsigned* length)
{
  if (!IsAvailable()){
    //pc.printf("Device not available\n\r");
    return false;
  }
  if (buffer && length)
  {
    //ATOMIC_BLOCK_START;
    // Skip the 4 headers that are at the beginning of the rxBuf
    if (*length > gBufferLength - RH_RF95_HEADER_LEN)
        *length = gBufferLength - RH_RF95_HEADER_LEN;
    memcpy(buffer, gBuffer + RH_RF95_HEADER_LEN, *length);
    //ATOMIC_BLOCK_END;
  }
  //rf95.clearRxBuf(); // This message accepted and cleared
  //ATOMIC_BLOCK_START;
  gBufferLength = 0;
  rxBufferValid = false;
  //ATOMIC_BLOCK_END;
  return true;
}
bool Send(uint8_t* data, unsigned length)
{
  if (length > RH_RF95_MAX_MESSAGE_LEN)
    return false;

  if(!WaitForPacketSent(1000)) // Make sure we dont interrupt an outgoing message
  {
    pc.printf("Failed to send packet. Packet sending timeout expired ! \n\r");
    wait_ms(200);
    return false;
  }
  SetModeIdle();

  // Position at the beginning of the FIFO
  SpiWrite(RH_RF95_REG_0D_FIFO_ADDR_PTR, 0);
  // The headers
  SpiWrite(RH_RF95_REG_00_FIFO, RH_BROADCAST_ADDRESS);
  SpiWrite(RH_RF95_REG_00_FIFO, RH_BROADCAST_ADDRESS);
  SpiWrite(RH_RF95_REG_00_FIFO, 0);
  SpiWrite(RH_RF95_REG_00_FIFO, 0);
  // The message data
  SpiBurstWrite(RH_RF95_REG_00_FIFO, data, length);
  SpiWrite(RH_RF95_REG_22_PAYLOAD_LENGTH, length + RH_RF95_HEADER_LEN);

  SetModeTx(); // Start the transmitter
  // when Tx is done, interruptHandler will fire and radio mode will return to STANDBY
  wait_ms(1);

  return true;
}

//
// Functions and routines for SPI management
//

uint8_t SpiRead(uint8_t reg)
{
  uint8_t val;

  chip_select = 0;
  //rf95._spi.transfer(reg & ~RH_SPI_WRITE_MASK); // Send the address with the write mask off
  spi.write(reg & ~RH_SPI_WRITE_MASK);
  val = spi.write(0);
  chip_select = 1;

  return val;
}
uint8_t SpiBurstRead(uint8_t reg, uint8_t* dest, uint8_t len)
{
  uint8_t status = 0;

  chip_select = 0;
  status = spi.write(reg & ~RH_SPI_WRITE_MASK);
  while (len--)
    *dest++ = spi.write(0);
  chip_select = 1;

  return status;
}
void SpiWrite(uint8_t reg, uint8_t val)
{
  uint8_t status = 0;

  chip_select = 0;
  status = spi.write(reg | RH_SPI_WRITE_MASK);
  spi.write(val);
  chip_select = 1;
}
void SpiBurstWrite(uint8_t reg, const uint8_t* src, uint8_t len)
{
  uint8_t status = 0;

  chip_select = 0;
  status = spi.write(reg | RH_SPI_WRITE_MASK);
  while (len--)
  {
    spi.write(*src++);
  }
  chip_select = 1;
}

bool Init()
{
  // Init SPI
  spi.format(8,0);
  spi.frequency(1000000);

  chip_reset = 1;

  wait_ms(100);

  // manual reset
  chip_reset = 0;
  wait_ms(10);
  chip_reset = 1;
  wait_ms(10);

  // Init RF95
  //
  if(!InitRF95()){
    pc.printf("LoRa radio init failed\n\r");
    while (1);
  }
  //

  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  //if (!rf95.setFrequency(RF95_FREQ)) {
  if(!SetFrequency(RF95_FREQ)){
    pc.printf("LoRa set frequency failed\n\r");
    while (1);
  }

  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on

  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then
  // you can set transmitter powers from 5 to 23 dBm:

  //
  // Set TX Power
  //SetTxPower(23, false);
  //

  return true;
}

// This function should hopefully be called in its own thread
// as it blocks the current thread by a lot of waiting
bool SendLoraData(const LoraData& data)
{
  SpiWrite(RH_RF95_REG_12_IRQ_FLAGS, 0xFF); // Clear all IRQ flags
  wait_ms(1);

  // Send a message to the other side
  static char message[] = "Hello World Chinook !";
  if(!Send((uint8_t*)message, strlen(message)+1))
  {
    return false;
  }
  wait_ms(1);

  // There has been some problems where LoRa interrupts were not
  // properly cleared, causing weird behaviours
  // This is to ensure that the end of the send function, IRQs are reset.
  wait_ms(1);
  SpiWrite(RH_RF95_REG_12_IRQ_FLAGS, 0xFF); // Clear all IRQ flags
  wait_ms(1);

  return true;
}

bool isr_triggered = false;

void Isr0()
{
  isr_triggered = true;
}

void Isr0_impl()
{
  uint8_t irq_flags = lora::SpiRead(0x12);

  if (lora::mode == lora::RHModeRx && irq_flags & (0x80 | 0x20)){
    //_rxBad++;
  }
  else if (lora::mode == lora::RHModeRx && irq_flags & RH_RF95_RX_DONE)
  {
    //Serial.println("RX DONE");
    // Have received a packet
    uint8_t len = lora::SpiRead(RH_RF95_REG_13_RX_NB_BYTES);

    // Reset the fifo read ptr to the beginning of the packet
    lora::SpiWrite(RH_RF95_REG_0D_FIFO_ADDR_PTR, lora::SpiRead(RH_RF95_REG_10_FIFO_RX_CURRENT_ADDR));
    lora::SpiBurstRead(RH_RF95_REG_00_FIFO, lora::gBuffer, len);
    lora::gBufferLength = len;
    lora::SpiWrite(RH_RF95_REG_12_IRQ_FLAGS, 0xff); // Clear all IRQ flags

    // Remember the RSSI of this packet
    // this is according to the doc, but is it really correct?
    // weakest receiveable signals are reported RSSI at about -66
    //rf95._lastRssi = spiRead(RH_RF95_REG_1A_PKT_RSSI_VALUE) - 137;

    // We have received a message.
    if (lora::gBufferLength < 4)
    {
      // NOP. Message too small !!
    }
    else
    {
      // Extract the 4 headers
      uint8_t _rxHeaderTo    = lora::gBuffer[0];
      uint8_t _rxHeaderFrom  = lora::gBuffer[1];
      uint8_t _rxHeaderId    = lora::gBuffer[2];
      uint8_t _rxHeaderFlags = lora::gBuffer[3];

      if(_rxHeaderTo == RH_BROADCAST_ADDRESS)
      {
        lora::rxBufferValid = true;
      }

      if (lora::rxBufferValid)
          lora::SetModeIdle(); // Got one
    }
  }
  else if (lora::mode == lora::RHModeTx && irq_flags & RH_RF95_TX_DONE)
  {
    lora::SetModeIdle();
  }

  wait_ms(1);
  lora::SpiWrite(RH_RF95_REG_12_IRQ_FLAGS, 0xff); // Clear all IRQ flags
}

void CheckIsr0()
{
  if(isr_triggered)
  {
    isr_triggered = false;
    Isr0_impl();
  }
}

}
