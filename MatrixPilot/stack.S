; File: stack.S
 // desired stack size in bytes (including the stack guardband)
 // need at least 470 + guard for MatrixPilot_UDB5_WJP project MatrixPilot-auav3
 #define STACK_SIZE 0x2000
 // allocate stack at top of memory from 0x6000 to 0x7FFF (24K to 32K)
 #define STACK_BOT   0X6000


 .section non_eds_stack, stack, address(STACK_BOT)
 .space STACK_SIZE
