/*
	This is a library for the SL01 Digital Light Sensor
	breakout board.

	The board uses I2C for communication.
*/

#include "xRL0x.h"

SC18IS602 i2cspi = SC18IS602(SC18IS602_ADDRESS_111);

/*---Public Function---*/
/********************************************************
 	Constructor
*********************************************************/
xRL0X::xRL0X(void) 
{

}

// These are indexed by the values of ModemConfigChoice
// Stored in flash (program) memory to save SRAM
PROGMEM static const xRL0X::ModemConfig MODEM_CONFIG_TABLE[] =
{
    //  1d,     1e,      26
    { 0x72,   0x74,    0x00}, // Bw125Cr45Sf128 (the chip default)
    { 0x92,   0x74,    0x00}, // Bw500Cr45Sf128
    { 0x48,   0x94,    0x00}, // Bw31_25Cr48Sf512
    { 0x78,   0xc4,    0x00}, // Bw125Cr48Sf4096
    
};

/********************************************************
 	Configure Module
*********************************************************/
bool xRL0X::begin(void)
{
	//Setup SC18IS602
  //==============================================================================
	setupSC18IS602();
	
  //Setup RFM95W
  //==============================================================================
  
  _mode = RHModeInitialising;
  _thisAddress = RH_BROADCAST_ADDRESS;
  _txHeaderTo = RH_BROADCAST_ADDRESS;
  _txHeaderFrom = RH_BROADCAST_ADDRESS;
  _txHeaderId = 0;
  _txHeaderFlags = 0;
  _rxBad = 0;
  _rxGood = 0;
  _txGood = 0;
  _cad_timeout = 0;
  _ISRedgeFlag = 0;
  
  write(RH_RF95_REG_01_OP_MODE, RH_RF95_MODE_SLEEP | RH_RF95_LONG_RANGE_MODE);
  
  // Set up FIFO
  // We configure so that we can use the entire 256 byte FIFO for either receive
  // or transmit, but not both at the same time
  write(RH_RF95_REG_0E_FIFO_TX_BASE_ADDR, 0);
  write(RH_RF95_REG_0F_FIFO_RX_BASE_ADDR, 0);
  
  setModeIdle();

    // Set up default configuration
    // No Sync Words in LORA mode.
    setModemConfig(Bw125Cr45Sf128); // Radio default
//    setModemConfig(Bw125Cr48Sf4096); // slow and reliable?
    setPreambleLength(8); // Default is 8
    // An innocuous ISM frequency, same as RF22's
    setFrequency(434.0);
    // Lowish power
    setTxPower(13);
    
    return true;
}

void xRL0X::setupSC18IS602(void)
{
  //Setup conversion chip.
  //==============================================================================
  i2cspi.begin();
  i2cspi.setClockDivider(0);

  i2cspi.pinMode(1, INPUT); //Set IRQ pin to input (High-Z)
  i2cspi.pinMode(2, INPUT); //Set Radio Reset pin to input (High-Z) possibly needed for startup.
}

uint8_t xRL0X::read(uint8_t reg)
{
  uint8_t sendArray[5];
  sendArray[0] = (reg & ~SPI_WRITE_MASK);
  sendArray[1] = 0x00;

  i2cspi.transfer(sendArray, 2);

  return sendArray[1];
}

void xRL0X::burstRead(uint8_t reg, uint8_t* dest, uint8_t len)
{
	uint8_t sendArray[200];
	uint8_t ptr = 1;
	
	sendArray[0] = (reg & ~SPI_WRITE_MASK);
	for (int i = 1; i < 200; i++)
	{
		sendArray[i] = 0x00;
	}
	
    i2cspi.transfer(sendArray, len+1);
	
    while (len--)
	*dest++ = sendArray[ptr++];
}

void xRL0X::write(uint8_t reg, uint8_t value)
{
  uint8_t sendArray[5];

  sendArray[0] = (reg | SPI_WRITE_MASK);
  sendArray[1] = value;

  i2cspi.transfer(sendArray, 2);
}

void xRL0X::burstWrite(uint8_t reg, const uint8_t* src, uint8_t len)
{
    uint8_t status = 0;
	uint8_t sendArray[200];
	uint8_t ptr = 0;
	uint8_t numbytes = len + 1;
	
	sendArray[ptr] = reg | SPI_WRITE_MASK;
    
    while (len--)
	{
		ptr++;
		sendArray[ptr] = *src++;
	}
	
	i2cspi.transfer(sendArray, numbytes);
}

bool xRL0X::send(const uint8_t* data, uint8_t len)
{
    if (len > RH_RF95_MAX_MESSAGE_LEN)
	return false;

    waitPacketSent(radioSendTimeout); // Make sure we dont interrupt an outgoing message
    setModeIdle();

    //if (!waitCAD()) 
	//return false;  // Check channel activity

    // Position at the beginning of the FIFO
    write(RH_RF95_REG_0D_FIFO_ADDR_PTR, 0);
    // The headers
    write(RH_RF95_REG_00_FIFO, _txHeaderTo);
    write(RH_RF95_REG_00_FIFO, _txHeaderFrom);
    write(RH_RF95_REG_00_FIFO, _txHeaderId);
    write(RH_RF95_REG_00_FIFO, _txHeaderFlags);
    // The message data
    burstWrite(RH_RF95_REG_00_FIFO, data, len);
    write(RH_RF95_REG_22_PAYLOAD_LENGTH, len + RH_RF95_HEADER_LEN);

    setModeTx(); // Start the transmitter
	waitPacketSent(radioSendTimeout);
    // when Tx is done, interruptHandler will fire and radio mode will return to STANDBY
    return true;
}

bool xRL0X::recv(uint8_t* buf, uint8_t* len)
{	
    if (!available())
	return false;
    if (buf && len)
    {
	// Skip the 4 headers that are at the beginning of the rxBuf
	if (*len > _bufLen-RH_RF95_HEADER_LEN)
	    *len = _bufLen-RH_RF95_HEADER_LEN;
	memcpy(buf, _buf+RH_RF95_HEADER_LEN, *len);
    }
    clearRxBuf(); // This message accepted and cleared
    return true;
}

void xRL0X::poll()
{
	if((i2cspi.digitalRead(1) == 1)) //&& (_ISRedgeFlag == 0))
	{
		//Serial.println("[STATUS] INTERRUPT");
		//_ISRedgeFlag = 1;
		
		// Read the interrupt register
		uint8_t irq_flags = read(RH_RF95_REG_12_IRQ_FLAGS);
		if (_mode == RHModeRx && irq_flags & (RH_RF95_RX_TIMEOUT | RH_RF95_PAYLOAD_CRC_ERROR))
		{
		//Serial.println("[STATUS] RX BAD");
		_rxBad++;
		}
		else if (_mode == RHModeRx && irq_flags & RH_RF95_RX_DONE)
		{
		//Serial.println("[STATUS] PACKET RECEIVED");
		// Have received a packet
		uint8_t len = read(RH_RF95_REG_13_RX_NB_BYTES);

		// Reset the fifo read ptr to the beginning of the packet
		write(RH_RF95_REG_0D_FIFO_ADDR_PTR, read(RH_RF95_REG_10_FIFO_RX_CURRENT_ADDR));
		burstRead(RH_RF95_REG_00_FIFO, _buf, len);
		_bufLen = len;
		write(RH_RF95_REG_12_IRQ_FLAGS, 0xff); // Clear all IRQ flags

		// Remember the last signal to noise ratio, LORA mode
		// Per page 111, SX1276/77/78/79 datasheet
		_lastSNR = (int8_t)read(RH_RF95_REG_19_PKT_SNR_VALUE) / 4;

		// Remember the RSSI of this packet, LORA mode
		// this is according to the doc, but is it really correct?
		// weakest receiveable signals are reported RSSI at about -66
		_lastRssi = read(RH_RF95_REG_1A_PKT_RSSI_VALUE);
		// Adjust the RSSI, datasheet page 87
		if (_lastSNR < 0)
			_lastRssi = _lastRssi + _lastSNR;
		else
			_lastRssi = (int)_lastRssi * 16 / 15;
		if (_usingHFport)
			_lastRssi -= 157;
		else
			_lastRssi -= 164;
			
		// We have received a message.
		validateRxBuf(); 
		if (_rxBufValid)
			setModeIdle(); // Got one 
		}
		else if (_mode == RHModeTx && irq_flags & RH_RF95_TX_DONE)
		{
		//Serial.println("[STATUS] TX DONE");
		_txGood++;
		setModeIdle();
		}
		else if (_mode == RHModeCad && irq_flags & RH_RF95_CAD_DONE)
		{
			//Serial.println("[STATUS] CAD DONE");
			_cad = irq_flags & RH_RF95_CAD_DETECTED;
			setModeIdle();
		}
		// Sigh: on some processors, for some unknown reason, doing this only once does not actually
		// clear the radio's interrupt flag. So we do it twice. Why?
		write(RH_RF95_REG_12_IRQ_FLAGS, 0xff); // Clear all IRQ flags
		write(RH_RF95_REG_12_IRQ_FLAGS, 0xff); // Clear all IRQ flags
		//_ISRedgeFlag = 0;
	}
	//else if ((i2cspi.digitalRead(1) == 0) && (_ISRedgeFlag == 1))
	//{
	//	_ISRedgeFlag = 0;
	//}
}

void xRL0X::validateRxBuf()
{
    if (_bufLen < 4)
	return; // Too short to be a real message
    // Extract the 4 headers
    _rxHeaderTo    = _buf[0];
    _rxHeaderFrom  = _buf[1];
    _rxHeaderId    = _buf[2];
    _rxHeaderFlags = _buf[3];
    if (_promiscuous ||
	_rxHeaderTo == _thisAddress ||
	_rxHeaderTo == RH_BROADCAST_ADDRESS)
    {
	_rxGood++;
	_rxBufValid = true;
    }
}

bool xRL0X::available()
{
	poll();
	
    if (_mode == RHModeTx)
	return false;
    setModeRx();
    return _rxBufValid; // Will be set by the interrupt handler when a good message is received
}

void xRL0X::clearRxBuf()
{
    _rxBufValid = false;
    _bufLen = 0;
}

void xRL0X::setModeTx()
{
    if (_mode != RHModeTx)
    {
	//Serial.println("[STATUS] ModeTX");
	write(RH_RF95_REG_01_OP_MODE, RH_RF95_MODE_TX);
	write(RH_RF95_REG_40_DIO_MAPPING1, 0x40); // Interrupt on TxDone
	_mode = RHModeTx;
    }
}

void xRL0X::setModeRx()
{
    if (_mode != RHModeRx)
    {
	//Serial.println("[STATUS] ModeRX");
	write(RH_RF95_REG_01_OP_MODE, RH_RF95_MODE_RXCONTINUOUS);
	write(RH_RF95_REG_40_DIO_MAPPING1, 0x00); // Interrupt on RxDone
	_mode = RHModeRx;
    }
}

void xRL0X::setModeIdle()
{
    if (_mode != RHModeIdle)
    {
	//Serial.println("[STATUS] ModeIDLE");
	write(RH_RF95_REG_01_OP_MODE, RH_RF95_MODE_STDBY);
	_mode = RHModeIdle;
    }
}

bool xRL0X::sleep()
{
    if (_mode != RHModeSleep)
    {
	write(RH_RF95_REG_01_OP_MODE, RH_RF95_MODE_SLEEP);
	_mode = RHModeSleep;
    }
    return true;
}

// Set one of the canned FSK Modem configs
// Returns true if its a valid choice
bool xRL0X::setModemConfig(ModemConfigChoice index)
{
    if (index > (signed int)(sizeof(MODEM_CONFIG_TABLE) / sizeof(ModemConfig)))
        return false;

    ModemConfig cfg;
    memcpy_P(&cfg, &MODEM_CONFIG_TABLE[index], sizeof(xRL0X::ModemConfig));
    setModemRegisters(&cfg);

    return true;
}

// Sets registers from a canned modem configuration structure
void xRL0X::setModemRegisters(const ModemConfig* config)
{
    write(RH_RF95_REG_1D_MODEM_CONFIG1,       config->reg_1d);
    write(RH_RF95_REG_1E_MODEM_CONFIG2,       config->reg_1e);
    write(RH_RF95_REG_26_MODEM_CONFIG3,       config->reg_26);
}

void xRL0X::setPreambleLength(uint16_t bytes)
{
    write(RH_RF95_REG_20_PREAMBLE_MSB, bytes >> 8);
    write(RH_RF95_REG_21_PREAMBLE_LSB, bytes & 0xff);
}

bool xRL0X::setFrequency(float centre)
{
    // Frf = FRF / FSTEP
    uint32_t frf = (centre * 1000000.0) / RH_RF95_FSTEP;
    write(RH_RF95_REG_06_FRF_MSB, (frf >> 16) & 0xff);
    write(RH_RF95_REG_07_FRF_MID, (frf >> 8) & 0xff);
    write(RH_RF95_REG_08_FRF_LSB, frf & 0xff);
    _usingHFport = (centre >= 779.0);

    return true;
}

void xRL0X::setTxPower(int8_t power, bool useRFO)
{
    // Sigh, different behaviours depending on whther the module use PA_BOOST or the RFO pin
    // for the transmitter output
    if (useRFO)
    {
	if (power > 14)
	    power = 14;
	if (power < -1)
	    power = -1;
	write(RH_RF95_REG_09_PA_CONFIG, RH_RF95_MAX_POWER | (power + 1));
    }
    else
    {
	if (power > 23)
	    power = 23;
	if (power < 5)
	    power = 5;

	// For RH_RF95_PA_DAC_ENABLE, manual says '+20dBm on PA_BOOST when OutputPower=0xf'
	// RH_RF95_PA_DAC_ENABLE actually adds about 3dBm to all power levels. We will us it
	// for 21, 22 and 23dBm
	if (power > 20)
	{
	    write(RH_RF95_REG_4D_PA_DAC, RH_RF95_PA_DAC_ENABLE);
	    power -= 3;
	}
	else
	{
	    write(RH_RF95_REG_4D_PA_DAC, RH_RF95_PA_DAC_DISABLE);
	}

	// RFM95/96/97/98 does not have RFO pins connected to anything. Only PA_BOOST
	// pin is connected, so must use PA_BOOST
	// Pout = 2 + OutputPower.
	// The documentation is pretty confusing on this topic: PaSelect says the max power is 20dBm,
	// but OutputPower claims it would be 17dBm.
	// My measurements show 20dBm is correct
	write(RH_RF95_REG_09_PA_CONFIG, RH_RF95_PA_SELECT | (power-5));
    }
}

bool xRL0X::waitPacketSent(uint16_t timeout)
{
    unsigned long starttime = millis();
    while ((millis() - starttime) < timeout)
    {
		poll();
        if (_mode != RHModeTx) // Any previous transmit finished?
           return true;
	yield();
    }
    return false;
}

// Blocks until a valid message is received or timeout expires
// Return true if there is a message available
// Works correctly even on millis() rollover
bool xRL0X::waitAvailableTimeout(uint16_t timeout)
{
    unsigned long starttime = millis();
    while ((millis() - starttime) < timeout)
    {
        if (available())
		{
           return true;
		}
	yield();
    }
    return false;
}

/***
  Function to set the Radio send/recieve headers
  @param address, the header to be sent in all subsequent messages
  @param to, the header to be sent in all subsequent messages
  @param from, the header to be sent in all subsequent messages
 ***/
void xRL0X::setHeaders(uint8_t address, uint8_t from, uint8_t to)
{
	setThisAddress(address);
	setHeaderFrom(from);
	setHeaderTo(to);
}

void xRL0X::setPromiscuous(bool promiscuous)
{
    _promiscuous = promiscuous;
}

void xRL0X::setThisAddress(uint8_t address)
{
    _thisAddress = address;
}

void xRL0X::setHeaderTo(uint8_t to)
{
    _txHeaderTo = to;
}

void xRL0X::setHeaderFrom(uint8_t from)
{
    _txHeaderFrom = from;
}

void xRL0X::setHeaderId(uint8_t id)
{
    _txHeaderId = id;
}

void xRL0X::setHeaderFlags(uint8_t set, uint8_t clear)
{
    _txHeaderFlags &= ~clear;
    _txHeaderFlags |= set;
}

uint8_t xRL0X::headerTo()
{
    return _rxHeaderTo;
}

uint8_t xRL0X::headerFrom()
{
    return _rxHeaderFrom;
}

uint8_t xRL0X::headerId()
{
    return _rxHeaderId;
}

uint8_t xRL0X::headerFlags()
{
    return _rxHeaderFlags;
}

int16_t xRL0X::lastRssi()
{
    return _lastRssi;
}

xRL0X RL0X = xRL0X();


