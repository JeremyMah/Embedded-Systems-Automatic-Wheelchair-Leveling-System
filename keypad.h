
// Permission to copy is granted provided that this header remains intact. 
// This software is provided with no warranties.

////////////////////////////////////////////////////////////////////////////////

// Returns '\0' if no key pressed, else returns char '1', '2', ... '9', 'A', ...
// If multiple keys pressed, returns leftmost-topmost one
// Keypad must be connected to port C
// Keypad arrangement
//        Px4 Px5 Px6 Px7
//	  col 1   2   3   4
//  row  ______________
//Px0 1	| 1 | 2 | 3 | A
//Px1 2	| 4 | 5 | 6 | B
//Px2 3	| 7 | 8 | 9 | C
//Px3 4	| * | 0 | # | D

#ifndef KEYPAD_H
#define KEYPAD_H

#include <bit.h>

// Keypad Setup Values
#define KEYPADPORT PORTC
#define KEYPADPIN  PINC
#define ROW1 6
#define ROW2 5
#define ROW3 4
#define ROW4 3

#define COL4 7

////////////////////////////////////////////////////////////////////////////////
//Functionality - Gets input from a keypad via time-multiplexing
//Parameter: None
//Returns: A keypad button press else '\0'
unsigned char GetKeypadKey() {



// Check keys in col 4
KEYPADPORT = SetBit(0xFF,COL4,0); // Set Px7 to 0; others 1
asm("nop"); // add a delay to allow PORTx to stabilize before checking
if (GetBit(~KEYPADPIN,ROW1) ) { return 'A'; }
if (GetBit(~KEYPADPIN,ROW2) ) { return 'B'; }
if (GetBit(~KEYPADPIN,ROW3) ) { return 'C'; }
if (GetBit(~KEYPADPIN,ROW4) ) { return 'D'; }

return '\0';
}

#endif //KEYPAD_H
