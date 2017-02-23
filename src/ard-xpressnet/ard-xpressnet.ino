/*************************************************************************
Title:    ARD-XPRESSNET Arduino-based decoder for the Lenz XpressNet bus
Authors:  Michael D. Petersen <railfan@drgw.net>
          Nathan D. Holmes <maverick@drgw.net>
File:     $Id: $
License:  GNU General Public License v3

LICENSE:
    Copyright (C) 2017 Michael Petersen
    
    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
*************************************************************************/

#include <util/atomic.h>

#define BAUD 62500
#define QUEUE_DEPTH 64

#define PKT_BUFFER_SIZE  16

uint16_t rxBuffer[QUEUE_DEPTH];
uint8_t headIdx, tailIdx, rxBufferFull;

uint8_t packetBuffer[PKT_BUFFER_SIZE];
uint8_t packetBufferIndex = 0;

void serialInit(void)
{
#include <util/setbaud.h>
  
  UBRR1 = UBRR_VALUE;
  UCSR1A = (USE_2X)?_BV(U2X1):0;
  UCSR1B = _BV(UCSZ12);
  UCSR1C = _BV(UCSZ11) | _BV(UCSZ10);

  /* Enable USART receiver and transmitter and receive complete interrupt */
  UCSR1B |= (_BV(RXCIE1) | _BV(RXEN1) | _BV(TXEN1));
}

ISR(USART1_RX_vect)
{
  uint16_t data = 0;

  if(UCSR1B & _BV(RXB81))
    data |= 0x0100;  // bit 9 set

  data |= UDR1;
  
  rxBufferPush(data);
}

void rxBufferInitialize(void)
{
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
  {
    headIdx = tailIdx = 0;
    rxBufferFull = 0;
  }
}

uint8_t rxBufferDepth(void)
{
  uint8_t result;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
  {
    if(rxBufferFull)
      return(QUEUE_DEPTH);
    result = ((uint8_t)(headIdx - tailIdx) % QUEUE_DEPTH);
  }
  return(result);
}

uint8_t rxBufferPush(uint16_t data)
{
    // If full, bail with a false
    if (rxBufferFull)
      return(0);
  
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
  {
    rxBuffer[headIdx] = data;
  
    if( ++headIdx >= QUEUE_DEPTH )
      headIdx = 0;
    if (headIdx == tailIdx)
      rxBufferFull = 1;
  }
  return(1);
}

uint16_t rxBufferPop(uint8_t snoop)
{
  uint16_t data;
    if (0 == rxBufferDepth())
      return(0);
  
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
  {
    data = rxBuffer[tailIdx];
    if(!snoop)
    {
      if( ++tailIdx >= QUEUE_DEPTH )
        tailIdx = 0;
      rxBufferFull = 0;
    }
  }
  return(data);
}

void setup()
{
	pinMode(2, OUTPUT);
	digitalWrite(2, LOW);

	rxBufferInitialize();

	Serial.begin(57600);
	serialInit();
}

void loop()
{
	uint16_t data;
	uint8_t i, speed;

	if(rxBufferDepth() > 1)
	{
		data = rxBufferPop(0);

		if(data & 0x100)
		{
			// Address received (9th bit set)
			if(packetBufferIndex > 0)
			{
				// Packet received and waiting in buffer
				if(0x40 == (packetBuffer[0] & 0x60))
				{
					// Normal Inquiry
					Serial.print("X");
					Serial.print(packetBuffer[0] && 0x1F);
					Serial.print(" - ");
					// Parse header byte
					switch(packetBuffer[1])
					{
						case 0xE4:
							// Speed, direction, function
							Serial.print("Loco ");
							Serial.print(((uint16_t)(packetBuffer[3] - 0xC0) << 8) + packetBuffer[4]);
							Serial.print(" ");
							// Parse identification byte
							switch(packetBuffer[2])
							{
								case 0x12:
									// 28 Speed Step
									speed = (((packetBuffer[5] & 0x0F) << 1) + ((packetBuffer[5] & 0x10) >> 4));
									if(0x02 == speed)
										Serial.print("ESTOP!");
									else if((0x01 == speed) || (0x03 == speed))
									{
										Serial.print("Unused Speed Step!");
									}
									else
									{
										Serial.print("Speed ");
										if(0 == speed)
											Serial.print(0);
										else
											Serial.print(speed - 3);
										Serial.print("/28 ");
										if(packetBuffer[5] & 0x80)
											Serial.print("FOR");
										else
											Serial.print("REV");
									}
									break;
								case 0x20:
									// Function Group 1
									Serial.print("FN: ");
									if(packetBuffer[5] & 0x10)
										Serial.print("(0) ");
									else
										Serial.print("( ) ");
									if(packetBuffer[5] & 0x01)
										Serial.print("(1) ");
									else
										Serial.print("( ) ");
									if(packetBuffer[5] & 0x02)
										Serial.print("(2) ");
									else
										Serial.print("( ) ");
									if(packetBuffer[5] & 0x04)
										Serial.print("(3) ");
									else
										Serial.print("( ) ");
									if(packetBuffer[5] & 0x08)
										Serial.print("(4) ");
									else
										Serial.print("( ) ");
									break;
								default:
									Serial.print("Unknown!");
									break;
							}
							break;
						default:
							Serial.print("Unknown!");
							break;
					}
				}
				Serial.print(" [");
				for(i=1; i<packetBufferIndex; i++)
				{
					if(i>1)
						Serial.print(" ");
					Serial.print(packetBuffer[i], HEX);
				}
				Serial.print("]");
				Serial.print("\n");
			}
			packetBufferIndex = 0;
			if(!(rxBufferPop(1) & 0x100))  // Snoop, but don't pop it yet (will be done in next loop)
			{
				// Start printing a new line only if there is a 0x1XX packet followed by a response (no 9th bit set)
/*				Serial.print("\n");*/
/*				Serial.print(data, HEX);*/
/*				Serial.print(" ");*/
				packetBuffer[packetBufferIndex] = data & 0xFF;
				if(packetBufferIndex < (PKT_BUFFER_SIZE-1))
					packetBufferIndex++;
			}
		}
		else if(!(data & 0x100))
		{
			// Print non-9th bit data
/*			Serial.print(data, HEX);*/
/*			Serial.print(" ");*/
			packetBuffer[packetBufferIndex] = data & 0xFF;  // FIXME: buffer overflow possible!
			if(packetBufferIndex < (PKT_BUFFER_SIZE-1))
				packetBufferIndex++;
		}
	}
}


