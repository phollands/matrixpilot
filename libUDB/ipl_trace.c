// This file is part of MatrixPilot.

#include "ipl_trace.h"
//#include "options.h"
#include "libUDB.h"

//
//    http://code.google.com/p/gentlenav/
//
// Copyright 2009-2011 MatrixPilot Team
// See the AUTHORS.TXT file for a list of authors of MatrixPilot.
//
// MatrixPilot is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// MatrixPilot is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with MatrixPilot.  If not, see <http://www.gnu.org/licenses/>.

#if (SILSIM != 1 && BOARD_TYPE == AUAV3_BOARD && IPL_MONITOR_EN != 0)
static int ipl_stack[8] = {0, 0, 0, 0, 0, 0, 0, 0};
static int ipl_stack_ptr = 0;
void disp_ipl_stack_level() {
    // assign MSB first
    _LATF13 = (ipl_stack_ptr>>2) & 0x1; // servo out 7
    _LATG1 = (ipl_stack_ptr>>1) & 0x1;  // servo out 6
    _LATG14 = ipl_stack_ptr & 0x1;      // servo out 5
}
// using pre-increment, the bottom stack entry is always zero
// and the ptr points at the current IPL
void push_ipl(void) {
    if (ipl_stack_ptr < 8) {
        ++ipl_stack_ptr;
        disp_ipl_stack_level();
        ipl_stack[ipl_stack_ptr] = SRbits.IPL;
    }
}
// if the stack pointer gets down to zero, no ISR is active
// When an ISR pops its IPL off the stack, this method returns the now active IPL
int pop_ipl(void) {
    if (ipl_stack_ptr > 0) {
        --ipl_stack_ptr;
        disp_ipl_stack_level();
        return ipl_stack[ipl_stack_ptr];
    } else {
        return 0;
    }
}

// Push (pre-increment ipl_stack_ptr) current IPL onto stack: if ipl_stack_ptr is
// already nonzero we are preempting another ISR, if zero, then no ISR was active.
// Indicate current IPL on digtal outputs.
void indicate_entering_isr(void) {
    push_ipl();
    setDigOut(SRbits.IPL);
}

// when current ISR returns, pop the current IPL off the stack, and set dig. out
// to reflect the previous top of the stack
void indicate_exiting_isr(void) {
    setDigOut(pop_ipl()); 
}
#else
void indicate_entering_isr(void) {};
void indicate_exiting_isr(void) {};
#endif
