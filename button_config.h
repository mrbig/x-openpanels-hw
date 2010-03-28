/** Special functions setup ****************************************/

#define COLUMNS 2

#define THROW_TIMEOUT 64

#pragma romdata
// Rotary encoders
// Hight four bit specifies the multiplex input (byte number in the joy struct)
// Low four bit specifies the lowest bit from the two legs of the rotary encoder
// The legs of the rotary encoder should be connected adjacent
ROM BYTE ROTARY_ENCODERS[2] = {0x00, 0x03};

// Throw switches - bitmask for all input column
ROM BYTE THROW_SWITCHES[COLUMNS] = {0x00, 0x63};


