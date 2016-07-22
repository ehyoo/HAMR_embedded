#include "Arduino.h"
#include "decoder.h"

void enable_decoder_clk(){
  pinMode(9, OUTPUT);
  
  TCCR2A = _BV(COM2A0) | _BV(COM2B1) | _BV(WGM20);
  TCCR2B = _BV(WGM22) | _BV(CS20);
  //TCCR2B |= (1 << 0); // prescaler to 1
  
  OCR2A = 20;
  OCR2B = 10;
}

unsigned char read_decoder_byte(){
  return PINF;
}

void init_decoders(){
  // SEL, OE, RST, MUX SELECT
  DDRK |= 0b11111;
  DDRF = 0;          //set all decoder pins to input
  
  //Reset decoder
  PORTK &= (1<<2);
  delayMicroseconds(100);
  PORTK |= (1<<2);
}

void select_decoder(int val){
  switch (val) {
    case 0: {
      // M1
      PORTK |= (1<<3);
      PORTK |= (1<<4); 
      break;
    }
    case 1: {
      // M2
      PORTK &= ~(1<<3);
      PORTK |= (1<<4); 
      break;
    }
    case 2: {
      // MT
      PORTK &= ~(1<<4); 
      break;
    }
  }
}

unsigned int read_decoder(int index){  
  unsigned char lowbyte;
  unsigned int highbyte;

  select_decoder(index); // select decoder to read using mux selects
  
  // set OE low and set SEL low to enable inhibit logic
  PORTK &= ~0b11;
  delayMicroseconds(10); //wait for edge
  
  highbyte = read_decoder_byte();
  
  //set select high
  PORTK |= 1;
  delayMicroseconds(10); // wait for edge
  
  lowbyte = read_decoder_byte();

  //set OE high
  PORTK |= (1<<1);

  // invert number (mux outputs inverted signals)
  return ~((highbyte << 8) | lowbyte);
}

