/** Special functions setup ****************************************/

#define COLUMNS 8

#define THROW_TIMEOUT 64
#define ROTARY_TIMEOUT 24

#define LOW_TIMEOUT 255


#pragma romdata
// Rotary encoders
// Hight four bit specifies the multiplex input (byte number in the joy struct)
// Low four bit specifies the lowest bit from the two legs of the rotary encoder
// The legs of the rotary encoder should be connected adjacent
ROM BYTE ROTARY_ENCODERS[9] = {0x00, 0x02, 0x10, 0x12, 0x20, 0x22, 0x30, 0x32, 0x64};

// Throw switches - bitmask for all input column
ROM BYTE THROW_SWITCHES[COLUMNS] = {0xe0, 0xe0, 0xe0, 0xe0, 0xee, 0x00, 0x00, 0xff};

// Inputs, where high-low transition should be delayed.
// Useful for rotary switches, when there's a short low period between inputs
// Can be used together with the THROW_SWITCHES config
ROM BYTE DELAYED_LOW[COLUMNS] = {0x00, 0x00, 0x00, 0x00, 0x22, 0x00, 0x00, 0x00};
