//
// buttons.c
//
// Handles the 6 button up, down, repeat modes
//
//

#include <project.h>
#include "main.h"
#include "buttons.h"
#include <math.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>


extern volatile structInfo Info;
extern volatile structButtonState ButtonState; // state of the guttons
extern volatile uint16 uiButtonTimer[NUMBUTTONS];

uint8 ReadButtonsOnChange(void) {
	static uint8 byButtonState[NUMBUTTONS] = {e_NOBUTTON, e_NOBUTTON, e_NOBUTTON};
    bool bButton = 0;

    uint8 byBuffer[2];
    LatchRead(LATCH1, PCLA9538_INREG, byBuffer[0]);
    uint8 byAllButtons = byBuffer[0]; // see if buttons are active
    byAllButtons >>= 4;
    byAllButtons = ~byAllButtons;
    
    for(uint8 n=0;n<NUMBUTTONS;n++) {
        bButton = byAllButtons & (0x01 << n); // bit mapped to single bool
    	switch(byButtonState[n]) {
            default:
    		case e_NOBUTTON: { // nothing last was happening to this button
                if(!uiButtonTimer[n]) {  // see if we had a prior down, may be jittery so we wait it out
        			if(bButton) { // anything happening?
        				uiButtonTimer[n] = BUTTONDOWNTIME; // start the down timer
        				byButtonState[n] = e_WAITDOWNBUTTON; // next state
        			} else {
                        // gonna have to check the multiple presses here                            
                        
                        ButtonState.byButtonDown &= ~(0x01 << n);
                        ButtonState.byButtonLong &= ~(0x01 << n);
                        ButtonState.byButtonHold &= ~(0x01 << n);
                    }
                }
    		} break;
    	
    		// we are in the process of trapping a downed key
    		case e_WAITDOWNBUTTON: {
                if(bButton) { // make sure we are still down
                    if(!uiButtonTimer[n]) { // did we elapse the down time?
                        uiButtonTimer[n] = BUTTONLONGPRESSTIME; // time to see if we are long pressing
           				byButtonState[n] = e_DOWNBUTTON; // next state
                        
                        ButtonState.byButtonDown |= (0x01 << n);
                    }
               } else { // something is bouncing
       				byButtonState[n] = e_NOBUTTON; // home state
                    ButtonState.byButtonDown &= ~(0x01 << n);
                    ButtonState.byButtonLong &= ~(0x01 << n);
                    ButtonState.byButtonHold &= ~(0x01 << n);
                }
            } break;
                    
    		case e_DOWNBUTTON: { // now hold here to see if this is a long press
                if(bButton) { // make sure we are still down
                    if(!uiButtonTimer[n]) { // did we elapse the long press down time?
//               				byButtonState[n] = e_LONGPRESSBUTTON; // next state
//                            uiButtonTimer[n] = BUTTONLONGERPRESSTIME; // time to see if we are long pressing
                        
                        byButtonState[n] = e_HOLDPRESS; // next state
                        ButtonState.byButtonLong |= 0x01 << n;
                    }
                } else { // let go?
       				byButtonState[n] = e_NOBUTTON; // home state
                    ButtonState.byButtonDown &= ~(0x01 << n);
                    ButtonState.byButtonLong &= ~(0x01 << n);
                    ButtonState.byButtonHold &= ~(0x01 << n);
                }
            } break;
                    
//                case e_LONGPRESSBUTTON: {
//                    if(bButton) { // make sure we are still down
//                        if(!uiButtonTimer[n]) { // did we elapse the long press down time?
//               				byButtonState[n] = e_HOLDPRESS; // next state
//                            
//                            ButtonState.byButtonHold |= 0x01 << n;
//                        }
//                    } else { // let go?
//           				byButtonState[n] = e_NOBUTTON; // home state
//                        ButtonState.byButtonDown &= ~(0x01 << n);
//                        ButtonState.byButtonLong &= ~(0x01 << n);
//                        ButtonState.byButtonHold &= ~(0x01 << n);
//                    }                           
//                } break;
            
            case e_HOLDPRESS: {
                if(bButton) { // make sure we are still down
                
                } else {
       				byButtonState[n] = e_NOBUTTON; // home state                        
                    ButtonState.byButtonDown &= ~(0x01 << n);
                    ButtonState.byButtonLong &= ~(0x01 << n);
                    ButtonState.byButtonHold &= ~(0x01 << n);
                }
            } break;
        }
	}
        
    return(ButtonState.byButtonDown);
	
} // End of ReadButtonsOnChange( )

