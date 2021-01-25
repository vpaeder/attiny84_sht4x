#include <avr/wdt.h>
#include <avr/sleep.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <string.h>
#include <stdio.h>

// constants definition (from SHT4X datasheet)
#define SHT4X_ADDRESS  0x44 // note that some sensors may have the address 0x45 instead

// general commands
#define SHT4X_READ_SERIAL 0x89
#define SHT4X_SOFT_RESET  0x94

// measurement commands
#define SHT4X_CMD_MEAS_HI_PREC       0xfd // measure T & RH with high precision (high repeatability)
#define SHT4X_CMD_MEAS_MED_PREC      0xf6 // measure T & RH with medium precision (medium repeatability)
#define SHT4X_CMD_MEAS_LOW_PREC      0xe0 // measure T & RH with lowest precision (low repeatability)
#define SHT4X_CMD_HI_HEAT_1S_MEAS    0x39 // activate highest heater power & high precis. meas. (typ. 200mW @ 3.3V) for 1s
#define SHT4X_CMD_HI_HEAT_01S_MEAS   0x32 // activate highest heater power & high precis. meas. (typ. 200mW @ 3.3V) for 0.1s
#define SHT4X_CMD_MED_HEAT_1S_MEAS   0x2f // activate medium heater power & high precis. meas. (typ. 110mW @ 3.3V) for 1s
#define SHT4X_CMD_MED_HEAT_01S_MEAS  0x24 // activate medium heater power & high precis. meas. (typ. 110mW @ 3.3V) for 0.1s
#define SHT4X_CMD_LOW_HEAT_1S_MEAS   0x1e // activate lowest heater power & high precis. meas. (typ. 20mW @ 3.3V) for 1s
#define SHT4X_CMD_LOW_HEAT_01S_MEAS  0x15 // activate lowest heater power & high precis. meas. (typ. 20mW @ 3.3V) for 0.1s

// I2C bus pins
#define PIN_I2C_SCL PA0
#define PIN_I2C_SDA PA1



/** \fn inline void wait_clk(uint8_t nclk)
 *  \brief Waits a given number of clock cycles (up to 255).
 *  \param nclk : number of clock cycles to wait.
 */
inline void wait_clk(uint8_t nclk) {
  do {
    asm("nop");
	} while (nclk--);
}

/*******************/
/*** I2C handler ***/
/*******************/

/** \fn inline void pull_scl_high()
 *  \brief Pulls SCL line high (the line is let free to go high through
 *  the appropriate pull-up resistor).
 */
inline void pull_scl_high() {
  // I2C doesn't drive lines to VCC but
  // a 1 is signaled by letting the line
  // go to VCC through a pull-up resistor
  DDRA &= ~_BV(PIN_I2C_SCL);
  // set this if there's no pull-up on that line
  // PORTA |= _BV(PIN_I2C_SCL);
}

/** \fn inline void pull_scl_low()
 *  \brief Pulls SCL line low.
 */
inline void pull_scl_low() {
  DDRA |= _BV(PIN_I2C_SCL);
	PORTA &= ~_BV(PIN_I2C_SCL);
  // set this if there's no pull-up on that line
  // PORTA &= ~_BV(PIN_I2C_SCL);
}

/** \fn inline void pull_sda_high()
 *  \brief Pulls SDA line high (the line is let free to go high through
 *  the appropriate pull-up resistor).
 */
inline void pull_sda_high() {
  DDRA &= ~_BV(PIN_I2C_SDA);
  // set this if there's no pull-up on that line
  // PORTA |= _BV(PIN_I2C_SDA);
}

/** \fn inline void pull_sda_low()
 *  \brief Pulls SDA line low.
 */
inline void pull_sda_low() {
  DDRA |= _BV(PIN_I2C_SDA);
	PORTA &= ~_BV(PIN_I2C_SDA);
  // set this if there's no pull-up on that line
  // PORTA &= ~_BV(PIN_I2C_SDA);
}

/** \fn bool i2c_write_byte(unsigned char data)
 *  \brief Writes a byte on the I2C bus.
 *  \param data: byte to write.
 *  \returns true if the receiver answered with ACK, false otherwise.
 */
bool i2c_write_byte(unsigned char data) {
  uint8_t len = 8;
  do {
    if (data & 0x80) {
      pull_sda_high();
    } else {
      pull_sda_low();
    }
    pull_scl_high();
    pull_scl_low();
    data <<= 1;
  } while (--len);
  // byte transfer ended -> read ack bit
	pull_sda_high();
  pull_scl_high();
	bool ack = (PINA & _BV(PIN_I2C_SDA));
  pull_scl_low();
	pull_sda_low();
  // ack = 0, nack = 1
  return (ack == 0);
}

/** \fn bool i2c_start(unsigned char address, bool read)
 *  \brief Starts a communication on the I2C bus.
 *  \param address: address of the device to communicate with.
 *  \param read: defines the type of communication; true = read, false = write.
 *  \returns true if the receiver answered with ACK, false otherwise.
 */
bool i2c_start(unsigned char address, bool read) {
  // format address
  address <<= 1;
  address += read;
  // start condition
  pull_scl_high();
	wait_clk(1);
  pull_sda_low();
  wait_clk(1);
  pull_scl_low();
  wait_clk(1);
  // send address
  return i2c_write_byte(address);
}

/** \fn void i2c_stop()
 *  \brief Stops a communication on the I2C bus.
 */
void i2c_stop() {
  pull_sda_low();
  wait_clk(1);
  pull_scl_high();
  wait_clk(1);
  pull_sda_high();
  wait_clk(1);
}

/** \fn unsigned char i2c_read_byte(bool ack)
 *  \brief Reads a byte from the I2C bus.
 *  \param ack: if true, sends ACK after reading, otherwise sends NACK.
 *  \returns the byte read.
 */
unsigned char i2c_read_byte(bool ack) {
  uint8_t len = 8;
  unsigned char result = 0;
  pull_sda_high();
  do {
    pull_scl_high();
    result <<= 1;
    result |= ((PINA & _BV(PIN_I2C_SDA)) ? 1 : 0);
    pull_scl_low();
  } while (--len);
	if (ack) {
		pull_sda_low();
	} else {
		pull_sda_high();
	}
	pull_scl_high();
	pull_scl_low();
	pull_sda_low();
	
  return result;
}


/***********************/
/*** SHT4X functions ***/
/***********************/

/** \fn unsigned char crc8(unsigned char vh, unsigned char vl)
 *  \brief Computes the CRC of a 16bit value.
 *  The CRC type is as defined in SHT4X datasheet p.9.
 *  \param vh: high byte.
 *  \param vl: low byte.
 *  \returns computed CRC.
 */
unsigned char crc8(unsigned char vh, unsigned char vl) {
	unsigned char crc = 0xff;
	unsigned char poly = 0x31;
	unsigned char data[2] = {vl, vh};
	unsigned char i = 2;
	while (i--) {
		crc ^= data[i];
		unsigned char j = 8;
		do {
			if (crc & 0x80)
				crc = (crc << 1) ^ poly;
			else
				crc <<= 1;
		} while (--j);
	}
	return crc;
}

/** \fn bool compute_temperature(float & value, unsigned char Th, unsigned char Tl, unsigned char crc)
 *  \brief Computes temperature from SHT4X data, performing a CRC check beforehand.
 *  \param value: returned value.
 *  \param Th: raw data (high byte).
 *  \param Tl: raw data (low byte).
 *  \param crc: CRC returned by sensor.
 *  \returns true if conversion was successful, false otherwise.
 */
bool compute_temperature(float & value, unsigned char Th, unsigned char Tl, unsigned char crc) {
	if (!(crc8(Th, Tl) ^ crc)) {
		unsigned int T = (Th << 8) + Tl;
		value = -45.0f + 175.0f*(float)T/65535.0f;
    // for Fahrenheit, use:
    // value = -49.0f + 315.0f*(float)T/65535.0f;
	} else {
		return false;
	}
	return true;
}

/** \fn bool compute_humidity(float & value, unsigned char Hh, unsigned char Hl, unsigned char crc)
 *  \brief Computes humidity from SHT4X data, performing a CRC check beforehand.
 *  \param value: returned value.
 *  \param Hh: raw data (high byte).
 *  \param Hl: raw data (low byte).
 *  \param crc: CRC returned by sensor.
 *  \returns true if conversion was successful, false otherwise.
 */
bool compute_humidity(float & value, unsigned char Hh, unsigned char Hl, unsigned char crc) {
	if (!(crc8(Hh, Hl) ^ crc)) {
		unsigned int H = (Hh << 8) + Hl;
		value = -6.0f + 125.0f * (float)H/65535.0f;
	} else {
		return false;
	}
	return true;
}

int main() {
  /*** setup ***/
  cli(); // disable interrupts
  // set pin modes and initial states for I2C
  DDRA |= _BV(PIN_I2C_SCL) | _BV(PIN_I2C_SDA);
	PORTA &= ~_BV(PIN_I2C_SCL) & ~_BV(PIN_I2C_SDA);
	
  // watchdog timer for periodic measurements
  ADCSRA &= ~_BV(ADEN);
  MCUSR &= ~_BV(WDRF); // clear watchdog status bit, just in case
  WDTCSR |= _BV(WDCE); // enable watchdog change and watchdog
  WDTCSR |= _BV(WDCE) | _BV(WDE) | _BV(WDIE) | _BV(WDP3) | _BV(WDP0); // set interrupt every 8sec
  
  // timer1 settings (for delays)
  TCCR1B |= _BV(CS10) | _BV(WGM12); // clock prescaler 1/8, CTC
  TIMSK1 &= ~_BV(OCIE1A); // disable the compare match interrupt
  sei(); // enable interrupts
  
	i2c_stop(); // set I2C bus to idle state, just in case
  
	// variables for data measurement
	unsigned char Tl, Th, Tcrc, Hl, Hh, Hcrc;
	
  /*** main loop ***/
  for (;;) {
  	// connect with sensor
  	bool success = i2c_start(SHT4X_ADDRESS, false);
  	if (!success) goto handle_results;
  	// start measurement
		// set measurement type
  	i2c_write_byte(SHT4X_CMD_MEAS_HI_PREC);
  	i2c_stop();
  	// wait for measurement to finish
  	do {} while (!i2c_start(SHT4X_ADDRESS, true));
		// read temperature (high byte - low byte - CRC)
  	Th = i2c_read_byte(true);
  	Tl = i2c_read_byte(true);
  	Tcrc = i2c_read_byte(true);
		// read humidity (high byte - low byte - CRC)
  	Hh = i2c_read_byte(true);
  	Hl = i2c_read_byte(true);
  	Hcrc = i2c_read_byte(true);
  	i2c_stop();
  
    handle_results:
  	if (success) {
  		float temperature, humidity;
  		if ( compute_temperature(temperature, Th, Tl, Tcrc) & compute_humidity(humidity, Hh, Hl, Hcrc) ) {
				// do something with computed values
  		} else {
				// one or both temperature/humidity measurements failed (invalid CRC)
  		}
  	} else {
  		// do something if measurement failed (communication problem)
  	}
    
    // enable power down sleep mode and send attiny to sleep until watchdog timer triggers
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    cli();
    sleep_enable();
    sleep_bod_disable();
    sei();
    sleep_cpu();
    sleep_disable();
  }
}
