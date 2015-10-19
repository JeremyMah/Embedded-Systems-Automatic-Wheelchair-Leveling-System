/*
 * Modified and cleared out some stuff i didnt use.
 * Edited by jmah from  Peter Fleury <pfleury@gmx.ch>  http://jump.to/fleury
 */

#include <inttypes.h>
#include <compat/twi.h>
#include "i2cmaster.h"

/* I2C timer max delay */
#define I2C_TIMER_DELAY 0xFF

void i2c_init(void)
{
  /* initialize TWI clock: 100 kHz clock, TWPS = 0 => prescaler = 1 */
  
  TWSR = 0;                         /* no prescaler */
  TWBR = 12;  /* must be > 10 for stable operation */

}/* i2c_init */

unsigned char i2c_start(unsigned char address)
{
	uint32_t  i2c_timer = 0;
    uint8_t   twst;

	// send START condition
	TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);

	// wait until transmission completed
	i2c_timer = I2C_TIMER_DELAY;
	while(!(TWCR & (1<<TWINT)) && i2c_timer--);
	if(i2c_timer == 0)
		return 1;

	// check value of TWI Status Register. Mask prescaler bits.
	twst = TW_STATUS & 0xF8;
	if ( (twst != TW_START) && (twst != TW_REP_START)) return 1;

	// send device address
	TWDR = address;
	TWCR = (1<<TWINT) | (1<<TWEN);

	// wail until transmission completed and ACK/NACK has been received
	i2c_timer = I2C_TIMER_DELAY;
	while(!(TWCR & (1<<TWINT)) && i2c_timer--);
	if(i2c_timer == 0)
		return 1;

	// check value of TWI Status Register. Mask prescaler bits.
	twst = TW_STATUS & 0xF8;
	if ( (twst != TW_MT_SLA_ACK) && (twst != TW_MR_SLA_ACK) ) return 1;

	return 0;

}/* i2c_start */

void i2c_stop(void)
{
	uint32_t  i2c_timer = 0;

    /* send stop condition */
	TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);
	
	// wait until stop condition is executed and bus released
	i2c_timer = I2C_TIMER_DELAY;
	while((TWCR & (1<<TWSTO)) && i2c_timer--);

}/* i2c_stop */

unsigned char i2c_write( unsigned char data )
{	
	uint32_t  i2c_timer = 0;
    uint8_t   twst;
    
	// send data to the previously addressed device
	TWDR = data;
	TWCR = (1<<TWINT) | (1<<TWEN);

	// wait until transmission completed
	i2c_timer = I2C_TIMER_DELAY;
	while(!(TWCR & (1<<TWINT)) && i2c_timer--);
	if(i2c_timer == 0)
		return 1;

	// check value of TWI Status Register. Mask prescaler bits
	twst = TW_STATUS & 0xF8;
	if( twst != TW_MT_DATA_ACK) return 1;
	return 0;

}/* i2c_write */


unsigned char i2c_readAck(void)
{
	uint32_t  i2c_timer = 0;

	TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA);
	i2c_timer = I2C_TIMER_DELAY;
	while(!(TWCR & (1<<TWINT)) && i2c_timer--);
	if(i2c_timer == 0)
		return 0;

    return TWDR;

}/* i2c_readAck */


unsigned char i2c_readNak(void)
{
	uint32_t  i2c_timer = 0;

	TWCR = (1<<TWINT) | (1<<TWEN);
	i2c_timer = I2C_TIMER_DELAY;
	while(!(TWCR & (1<<TWINT)) && i2c_timer--);
	if(i2c_timer == 0)
		return 0;
	
    return TWDR;

}/* i2c_readNak */
