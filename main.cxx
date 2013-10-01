#include <util/delay.h>

#include <avr/io.h>
#include <util/delay.h>
#include <inttypes.h>

#define BAUD 115200UL
#define UBRR_VAL ((F_CPU+BAUD*8)/(BAUD*16)-1)

void uart_init()
{
  UBRRH = UBRR_VAL >> 8;
  UBRRL = UBRR_VAL & 0xFF;

  UCSRB |= (1<<TXEN);
  UCSRC = (1<<URSEL)|(1<<UCSZ1)|(1<<UCSZ0);
}

void uart_putc(unsigned char c)
{
  while (!(UCSRA & (1<<UDRE)))
  {
  }
  UDR = c;
}

void uart_puts (const char *s)
{
  while (*s)
  {
    uart_putc(*s);
    s++;
  }
}

/*************************************************************************
* Title:    I2C master library using hardware TWI interface
* Author:   Peter Fleury <pfleury@gmx.ch>  http://jump.to/fleury
* File:     $Id: twimaster.c,v 1.3 2005/07/02 11:14:21 Peter Exp $
* Software: AVR-GCC 3.4.3 / avr-libc 1.2.3
* Target:   any AVR device with hardware TWI
* Usage:    API compatible with I2C Software Library i2cmaster.h
**************************************************************************/

#include <inttypes.h>
#include <compat/twi.h>

/* I2C clock in Hz */
#define SCL_CLOCK  100000L
#define I2C_READ    1
#define I2C_WRITE   0

/*************************************************************************
 Initialization of the I2C bus interface. Need to be called only once
*************************************************************************/
void i2c_init(void)
{
  TWSR = 0;                         /* no prescaler */
  TWBR = ((F_CPU/SCL_CLOCK)-16)/2;  /* must be > 10 for stable operation */
}/* i2c_init */

/*************************************************************************
  Issues a start condition and sends address and transfer direction.
  return 0 = device accessible, 1= failed to access device
*************************************************************************/
unsigned char i2c_start(unsigned char address)
{
    uint8_t   twst;

  // send START condition
  TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);

  // wait until transmission completed
  while(!(TWCR & (1<<TWINT)));

  // check value of TWI Status Register. Mask prescaler bits.
  twst = TW_STATUS & 0xF8;
  if ( (twst != TW_START) && (twst != TW_REP_START)) return 1;

  // send device address
  TWDR = address;
  TWCR = (1<<TWINT) | (1<<TWEN);

  // wail until transmission completed and ACK/NACK has been received
  while(!(TWCR & (1<<TWINT)));

  // check value of TWI Status Register. Mask prescaler bits.
  twst = TW_STATUS & 0xF8;
  if ( (twst != TW_MT_SLA_ACK) && (twst != TW_MR_SLA_ACK) ) return 1;

  return 0;

}/* i2c_start */


/*************************************************************************
 Issues a start condition and sends address and transfer direction.
 If device is busy, use ack polling to wait until device is ready

 Input:   address and transfer direction of I2C device
*************************************************************************/
void i2c_start_wait(unsigned char address)
{
  uint8_t   twst;
  while ( 1 )
  {
    // send START condition
    TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);

    // wait until transmission completed
    while(!(TWCR & (1<<TWINT)));

    // check value of TWI Status Register. Mask prescaler bits.
    twst = TW_STATUS & 0xF8;
    if ( (twst != TW_START) && (twst != TW_REP_START)) continue;

    // send device address
    TWDR = address;
    TWCR = (1<<TWINT) | (1<<TWEN);

    // wail until transmission completed
    while(!(TWCR & (1<<TWINT)));

    // check value of TWI Status Register. Mask prescaler bits.
    twst = TW_STATUS & 0xF8;
    if ( (twst == TW_MT_SLA_NACK )||(twst ==TW_MR_DATA_NACK) )
    {
        /* device busy, send stop condition to terminate write operation */
        TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);

        // wait until stop condition is executed and bus released
        while(TWCR & (1<<TWSTO));

        continue;
    }
    //if( twst != TW_MT_SLA_ACK) return 1;
    break;
   }

}/* i2c_start_wait */


/*************************************************************************
 Issues a repeated start condition and sends address and transfer direction

 Input:   address and transfer direction of I2C device

 Return:  0 device accessible
          1 failed to access device
*************************************************************************/
unsigned char i2c_rep_start(unsigned char address)
{
  return i2c_start( address );

}/* i2c_rep_start */


/*************************************************************************
 Terminates the data transfer and releases the I2C bus
*************************************************************************/
void i2c_stop(void)
{
    /* send stop condition */
  TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);

  // wait until stop condition is executed and bus released
  while(TWCR & (1<<TWSTO));

}/* i2c_stop */


/*************************************************************************
  Send one byte to I2C device

  Input:    byte to be transfered
  Return:   0 write successful
            1 write failed
*************************************************************************/
unsigned char i2c_write( unsigned char data )
{
    uint8_t   twst;

  // send data to the previously addressed device
  TWDR = data;
  TWCR = (1<<TWINT) | (1<<TWEN);

  // wait until transmission completed
  while(!(TWCR & (1<<TWINT)));

  // check value of TWI Status Register. Mask prescaler bits
  twst = TW_STATUS & 0xF8;
  if( twst != TW_MT_DATA_ACK) return 1;
  return 0;

}/* i2c_write */


/*************************************************************************
 Read one byte from the I2C device, request more data from device

 Return:  byte read from I2C device
*************************************************************************/
unsigned char i2c_readAck(void)
{
  TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA);
  while(!(TWCR & (1<<TWINT)));

    return TWDR;

}/* i2c_readAck */


/*************************************************************************
 Read one byte from the I2C device, read is followed by a stop condition

 Return:  byte read from I2C device
*************************************************************************/
unsigned char i2c_readNak(void)
{
  TWCR = (1<<TWINT) | (1<<TWEN);
  while(!(TWCR & (1<<TWINT)));

    return TWDR;

}/* i2c_readNak */

enum TMotorCmd
{
  e_Speed = 0,
  e_Turn = 1,
  e_Battery = 10,
  e_Current1 = 11,
  e_Current2 = 12,
  e_Acceleration = 14,
  e_Mode = 15
};

class Motor
{
public:
  Motor()
  {
    uart_puts("Init Motor...\r\n");
    uart_putc('1');
    i2c_init();                             // initialize I2C library

    uart_putc('2');
    Send (e_Speed, 0);
    uart_putc('3');
    Send (e_Turn, 0);
    Send (e_Acceleration, 10);
    Send (e_Mode, 3);
    uart_puts("...ok\r\n");
  }

  void Send (TMotorCmd cmd, unsigned char value)
  {
    i2c_start_wait(0xB0+I2C_WRITE);
    i2c_write(cmd);
    i2c_write(value);
    i2c_stop();
  };

  uint8_t Read (TMotorCmd cmd)
  {
    i2c_start_wait(0xB0+I2C_WRITE);
    i2c_write(cmd);
    i2c_rep_start(0xB0+I2C_READ);
    uint8_t iRet = i2c_readNak();
    i2c_stop();
    return iRet;
  };
};

class Mower
{
public:
  Mower()
    : m_SensorState (e_Normal)
    , m_MovingState (e_Idle)
    , m_AfterBackwardsMovingState (e_Idle)
    , m_iForwardSpeed (0)
    , m_iTurnSpeed (0)
    , m_iTurnCount (0)
  {
    uart_puts("Init Mower...\r\n");
	  PORTB |= 0x1;
	  PORTC |= 0x15;
    uart_puts("...ok\r\n");
  }

  void run()
  {
    for (;;)
    {
      //TODO:  Werte auslesen
      bool bContactLeft = false;//((PINC & 1) == 0);
      bool bContactRight = false;//((PINC & 2) == 0);
      bool bDistanceLeft = false;//((PINC & 4) == 0);
      bool bDistanceRight = false;//((PINC & 8) == 0);
      bool bStartKey = ((PINB & 1) == 0);

      signed char iFeedbackSpeed = m_Motor.Read(e_Speed);

      unsigned char uiBatteryVoltage = m_Motor.Read(e_Battery);
      if (uiBatteryVoltage < 110)
      {
        m_MovingState = e_Idle;
      }

      bool bTransition = false;
      switch (m_SensorState)
      {
      case e_Normal:
        if (bContactLeft)
        {
          m_SensorState = e_ContactLeft;
          bTransition = true;
        }
        else if (bContactRight)
        {
          m_SensorState = e_ContactRight;
          bTransition = true;
        }
        else if (bDistanceRight)
        {
          m_SensorState = e_DistanceRight;
          bTransition = true;
        }
        else if (bDistanceLeft)
        {
          m_SensorState = e_DistanceLeft;
          bTransition = true;
        }
        break;

      case e_DistanceLeft:
      case e_DistanceRight:
        if (bContactLeft)
        {
          m_SensorState = e_ContactLeft;
          bTransition = true;
        }
        else if (bContactRight)
        {
          m_SensorState = e_ContactRight;
          bTransition = true;
        }
        else if (bDistanceRight)
        {
        }
        else if (bDistanceLeft)
        {
        }
        else
        {
          m_SensorState = e_Normal;
          bTransition = true;
        }
        break;

      case e_ContactLeft:
      case e_ContactRight:
          if (bContactLeft)
          {
          }
          else if (bContactRight)
          {
          }
          else
          {
            m_SensorState = e_Normal;
            bTransition = true;
          }
        break;
      }

      if (bTransition)
      {
          switch(m_SensorState)
          {
          case e_ContactLeft:
        	  uart_puts("ContactLeft\r\n");
        	  break;
          case e_ContactRight:
        	  uart_puts("ContactRight\r\n");
        	  break;
          case e_DistanceLeft:
        	  uart_puts("DistanceLeft\r\n");
        	  break;
          case e_DistanceRight:
        	  uart_puts("DistanceRight\r\n");
        	  break;
          case e_Normal:
        	  uart_puts("Normal\r\n");
        	  break;
          }
      }


      if (m_iForwardSpeed > 0)
      {
        int iActualMin = m_iForwardSpeed - 10;
        if (iFeedbackSpeed < iActualMin)
        {
          m_MovingState = e_Idle;
          continue;
        }
      }

      switch (m_MovingState)
      {
      case e_Idle:
        if (bStartKey)
        {
          switch (m_SensorState)
          {
          case e_Normal:
            uart_puts("StartKey -> MovingFast\r\n");
            m_MovingState = e_MovingFast;
            break;

          case e_DistanceLeft:
          case e_DistanceRight:
            uart_puts("StartKey -> MovingSlow\r\n");
            m_MovingState = e_MovingSlow;
            break;

          case e_ContactLeft:
          case e_ContactRight:
            uart_puts("StartKey -> BackForTurn\r\n");
            m_MovingState = e_BackForTurn;
            break;
          }
        }
        else
        {
          m_iForwardSpeed = 0;
          m_iTurnSpeed = 0;
        }
        break;

      case e_MovingFast:
        if (bTransition)
        {
          switch (m_SensorState)
          {
          case e_Normal:
            uart_puts("ERROR: State:MovingFast, Sensor:Normal\r\n");
            break;

          case e_DistanceLeft:
          case e_DistanceRight:
            uart_puts("Distance -> MovingFast > MovingSlow\r\n");
            m_MovingState = e_MovingSlow;
            m_iForwardSpeed = 20;
            m_iTurnSpeed = 0;
            continue;

          case e_ContactLeft:
            uart_puts("ContactLeft -> MovingFast > MovingSlow\r\n");
            m_AfterBackwardsMovingState = e_TurnRight;
            m_MovingState = e_BackForTurn;
            continue;

          case e_ContactRight:
            uart_puts("ContactRight -> MovingFast > MovingSlow\r\n");
            m_AfterBackwardsMovingState = e_TurnLeft;
            m_MovingState = e_BackForTurn;
            continue;
          }
        }
        else
        {
          if (m_iForwardSpeed > 0)
          {
            int iActualMax = m_iForwardSpeed + 10;
            if (iFeedbackSpeed > iActualMax)
            {
              uart_puts("SpeedTooLow -> Idle\r\n");
              m_MovingState = e_Idle;
              continue;
            }
          }

          if (m_iForwardSpeed != 100)
          {
            m_iForwardSpeed += (m_iForwardSpeed < 100) ? 1 : -1;
            //uart_puts("ForwardSpeed -> corrected\r\n");
          }
          m_iTurnSpeed = 0;
        }
        break;

      case e_MovingSlow:
        if (bTransition)
        {
          switch (m_SensorState)
          {
          case e_Normal:
            uart_puts("SensorNormal -> MovingSlow > MovingFast\r\n");
            m_MovingState = e_MovingFast;
            break;

          case e_DistanceLeft:
          case e_DistanceRight:
            break;

          case e_ContactLeft:
          case e_ContactRight:
            uart_puts("Contact -> MovingSlow > BackForTurn\r\n");
            m_MovingState = e_BackForTurn;
            m_iTurnCount = 20;
            m_iForwardSpeed = -20;
            m_iTurnSpeed = 0;
            continue;
          }
        }
        else
        {
          if (m_iForwardSpeed != 20)
          {
            uart_puts("ERROR: MovingSlow & ForwardSpeed!=20\r\n");
          }
        }
        break;

      case e_BackForTurn:
        if (bTransition)
        {
          switch (m_SensorState)
          {
          case e_Normal:
            uart_puts("SensorNormal -> BackForTurn > Turning\r\n");
            m_MovingState = m_AfterBackwardsMovingState;
            m_iForwardSpeed = 0;
            m_iTurnSpeed = 0;
            continue;

          case e_DistanceLeft:
          case e_DistanceRight:
          case e_ContactLeft:
          case e_ContactRight:
            break;
          }
        }
        else
        {
          if (m_iForwardSpeed != -20)
          {
            uart_puts("ERROR: BackForTurn & ForwardSpeed!=-20\r\n");
          }
          m_iTurnSpeed = 0;
        }
        break;

      case e_TurnLeft:
        if (bTransition)
        {
          switch (m_SensorState)
          {
          case e_Normal:
          case e_DistanceLeft:
          case e_DistanceRight:
          case e_ContactRight:
            break;

          case e_ContactLeft:
            m_MovingState = e_TurnRight;
            uart_puts("ContactLeft -> TurnLeft > TurnRight\r\n");
            continue;
          }
        }
        else
        {
          if (m_iForwardSpeed != 0)
          {
            uart_puts("ERROR: TurnLeft & ForwardSpeed!=0\r\n");
          }
          m_iTurnSpeed = -5;

          m_iTurnCount--;
          if (m_iTurnCount <= 0)
          {
            uart_puts("TurnCount -> Turn > MovingFast\r\n");
            m_MovingState = e_MovingFast;
          }
        }
        break;

      case e_TurnRight:
        if (bTransition)
        {
          switch (m_SensorState)
          {
          case e_Normal:
          case e_DistanceLeft:
          case e_DistanceRight:
          case e_ContactLeft:
            break;

          case e_ContactRight:
            uart_puts("ContactRight -> TurnRight > TurnLeft\r\n");
            m_MovingState = e_TurnLeft;
            continue;
          }
        }
        else
        {
          if (m_iForwardSpeed != 0)
          {
            uart_puts("ERROR: TurnRight & ForwardSpeed!=0\r\n");
          }
          m_iTurnSpeed = 5;

          m_iTurnCount--;
          if (m_iTurnCount <= 0)
          {
            uart_puts("TurnCount -> Turn > MovingFast\r\n");
            m_MovingState = e_MovingFast;
          }

        }
        break;
      }

      m_Motor.Send(e_Speed, m_iForwardSpeed);
      m_Motor.Send(e_Turn, m_iTurnSpeed);

      _delay_ms (25);

    }
  }

private:

  enum TSensorState {
    e_Normal = 0,
    e_ContactLeft,
    e_ContactRight,
    e_DistanceLeft,
    e_DistanceRight
  } m_SensorState;

  enum TMovingState {
    e_Idle = 0,
    e_MovingFast,
    e_MovingSlow,
    e_BackForTurn,
    e_TurnLeft,
    e_TurnRight
  } m_MovingState;

  TMovingState m_AfterBackwardsMovingState;

  signed char m_iForwardSpeed;
  signed char m_iTurnSpeed;
  signed char m_iTurnCount;

  Motor m_Motor;
};

int main()
{
  uart_init();

  uart_puts("Boot...\r\n");
  Mower mower;

  uart_puts("Running...\r\n");
  mower.run();
}
