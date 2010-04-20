/** Special functions setup ****************************************/

#define COLUMNS 8

#define THROW_TIMEOUT 64
#define ROTARY_TIMEOUT 24

#pragma romdata
// Rotary encoders
// Hight four bit specifies the multiplex input (byte number in the joy struct)
// Low four bit specifies the lowest bit from the two legs of the rotary encoder
// The legs of the rotary encoder should be connected adjacent
ROM BYTE ROTARY_ENCODERS[2] = {0x00, 0x03};

// Throw switches - bitmask for all input column
ROM BYTE THROW_SWITCHES[COLUMNS] = {0xe0, 0xe0, 0xe0, 0xe0, 0xee, 0x00, 0x00, 0xff};


