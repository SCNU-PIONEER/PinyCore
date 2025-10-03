#include "Hardfault.hpp"
void HardFault_Handler(unsigned int *pStack)
{
    //
    // In case we received a hard fault because of a breakpoint instruction, we
    // return. This may happen when using semihosting for printf outputs and no
    // debugger is connected, i.e. when running a "Debug" configuration in release
    // mode.
    //
    if (NVIC_HFSR & (1uL << 31)) {
        NVIC_HFSR |= (1uL << 31); // Reset Hard Fault status
        *(pStack + 6u) += 2u;     // PC is located on stack at SP + 24 bytes;
        // increment PC by 2 to skip break instruction.
        return; // Return to interrupted application
    }

#if DEBUG
    //
    // Read NVIC registers
    //
    HardFaultRegs.syshndctrl.byte = SYSHND_CTRL; // System Handler Control and
    // State Register
    HardFaultRegs.mfsr.byte = NVIC_MFSR; // Memory Fault Status Register
    HardFaultRegs.bfsr.byte = NVIC_BFSR; // Bus Fault Status Register
    HardFaultRegs.bfar = NVIC_BFAR;      // Bus Fault Manage Address Register
    HardFaultRegs.ufsr.byte = NVIC_UFSR; // Usage Fault Status Register
    HardFaultRegs.hfsr.byte = NVIC_HFSR; // Hard Fault Status Register
    HardFaultRegs.dfsr.byte = NVIC_DFSR; // Debug Fault Status Register
    HardFaultRegs.afsr = NVIC_AFSR;      // Auxiliary Fault Status Register
    //
    // Halt execution
    // If NVIC registers indicate readable memory, change the variable value
    // to != 0 to continue execution.
    //
    _Continue = 0u;
    while (_Continue == 0u)
        ;
    //
    // Read saved registers from the stack
    //
    HardFaultRegs.SavedRegs.r0 = pStack[0];       // Register R0
    HardFaultRegs.SavedRegs.r1 = pStack[1];       // Register R1
    HardFaultRegs.SavedRegs.r2 = pStack[2];       // Register R2
    HardFaultRegs.SavedRegs.r3 = pStack[3];       // Register R3
    HardFaultRegs.SavedRegs.r12 = pStack[4];      // Register R12
    HardFaultRegs.SavedRegs.lr = pStack[5];       // Link register LR
    HardFaultRegs.SavedRegs.pc = pStack[6];       // Program counter PC
    HardFaultRegs.SavedRegs.psr.byte = pStack[7]; // Program status word PSR
    //
    // Halt execution
    // To step out of the HardFaultHandler, change the variable value to != 0.
    //
    _Continue = 0u;
    while (_Continue == 0u) {
    }
#else
    //
    // If this module is included in a release configuration,
    // simply stay in the HardFault handler
    //
    (void)pStack;
    do {
    } while (1);
#endif
}
/*************************** End of file ****************************/
