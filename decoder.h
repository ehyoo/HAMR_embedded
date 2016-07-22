#ifndef DECODER_h
#define DECODER_h 

void init_decoders();
void enable_decoder_clk();
unsigned char read_decoder_byte();
unsigned int read_decoder(int index);
void select_decoder(int val);

 #endif
