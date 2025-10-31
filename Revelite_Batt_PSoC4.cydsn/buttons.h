//*********************************************************************
// FILE: buttons.h for Ultra6
// DESC: This is the interface file for the buttons.c routines...
//
//*********************************************************************
#ifndef _buttons_h_
#define _buttons_h_

    #include "cytypes.h"
        
    // prototypes
    uint8 ReadButtonsOnChange(void);
    
    #define NUMBUTTONS 3
    
    #define BUTTONDOWNTIME 50
    #define BUTTONDOWNHOLDTIME 10
    #define BUTTONDOWNBOUNCETIME 10
    #define BUTTONLONGPRESSTIME 250
    #define BUTTONLONGERPRESSTIME 1000
        
    // states for the button trapping routine
    enum
    {
        e_NOBUTTON,
        e_WAITDOWNBUTTON,
        e_DOWNBUTTON,
        e_LONGPRESSBUTTON,
        e_LONGERPRESSBUTTON,
        e_HOLDPRESS,
        
    };

        // this is how we store the state of each button, this is bit mapped
    typedef struct __attribute__ ((__packed__)) {
        uint8 byButtonDown; // down state
        uint8 byButtonLong; // long down state
        uint8 byButtonHold; // longer down state
        uint8 byButtonDoubleTap; // double tap state
        uint8 byButtonTripleTap; // double tap state
    } structButtonState;
    
    // button defines
    #define BUTTON1 (0x01 << 0)
    #define BUTTON2 (0x01 << 1)
    #define BUTTON3 (0x01 << 2)
    #define ENDBUTTON 0xFF  // for our while loop
    
    enum {
        e_BUTTON1,
        e_BUTTON2,
        e_BUTTON3,
    };
                
    enum {
        e_SWIDLE,
        e_RELEASED,
        e_PRESSED,
        e_LONGPRESS,
        e_LONGERPRESS,
        e_DOUBLETAP,
        e_TRIPLETAP,
    } SWPRESSES;
        
    
#endif // End of _buttons_h_