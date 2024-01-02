// Included Files
#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File

#define RECEIVER_BUFFER 26

// Function Prototype
void initUART();
void scia_fifo_init();
void interrupt_gpio(void);

// Global variables
Uint16 LoopCount = 0;
Uint16 Index = 0;
Uint16 ReceivedChar;
Uint16 ReceiveWatch = 0;
Uint16 IsStartBit = 0;
Uint16 IsEndBit = 0;
Uint16 StartIndex = 0;
Uint16 sciReceiving = 0;
Uint16 Ack = 0;
Uint16 EndIndex = 0;
volatile Uint16 ReceiveEnable = 0;  // Volatile is used for variables which changed inside interrupts

// Test Variables
Uint16 test = 0;

// Interrupts
__interrupt void receive_interrupt(void);


Uint16 Receive_Buff[RECEIVER_BUFFER];
Uint16 SPIArray[RECEIVER_BUFFER - 6];

void main(void){

    // Step 1. Initialize System Control:PLL, WatchDog, enable Peripheral Clocks This example function is found in the F2806x_SysCtrl.c file.
     InitSysCtrl();

    // For this example, only init the pins for the SCI-A port. This function is found in the F2806x_Sci.c file.
    InitSciaGpio();

    // Step 3. Clear all interrupts and initialize PIE vector table:Disable CPU interrupts
    DINT;

    // Initialize PIE control registers to their default state.The default state is all PIE interrupts disabled and flags are cleared.This function is found in the F2806x_PieCtrl.c file.
    InitPieCtrl();

    // Disable CPU interrupts and clear all CPU interrupt flags
    IER = 0x0000; // @suppress("Symbol is not resolved")
    IFR = 0x0000; // @suppress("Symbol is not resolved")

    // Initialize the PIE vector table with pointers to the shell Interrupt Service Routines (ISR). This will populate the entire table, even if the interrupt
    // is not used in this example.  This is useful for debug purposes. The shell ISR routines are found in F2806x_DefaultIsr.c. This function is found in F2806x_PieVect.c.
    InitPieVectTable();

    EALLOW;             // This is needed to write to ELLOW protected registers
    PieVectTable.XINT1 = &receive_interrupt;
    EDIS;               // This is needed to disable write to ELLOW protected registers

    LoopCount = 0;

    // Enable XINT1 in the PIE: group 1 interrupt 4
    PieCtrlRegs.PIECTRL.bit.ENPIE = 1; // Enable the pie block.
    PieCtrlRegs.PIEIER1.bit.INTx4 = 1 ; // Enable pie group 1 INT4
    IER |= M_INT1;   // @suppress("Symbol is not resolved")   Enable CPU INT1
    EINT;


    interrupt_gpio();
    initUART();
    scia_fifo_init();      // Initialize the SCI FIFO


    while(1)
    {
        if(ReceiveEnable){
            memset(Receive_Buff,0,26);
            for(;;){
             // Wait for inc character
             while(SciaRegs.SCIFFRX.bit.RXFFST !=1)
             {
                LoopCount++;
                if(LoopCount > 50000){
                    LoopCount = 0;
                    break;
                }
                //
                // wait for XRDY =1 for empty state
                //
             }

             // Get character
             ReceivedChar = SciaRegs.SCIRXBUF.all;
             Receive_Buff[Index] = ReceivedChar;

             if(Index > 0){
                 if(Receive_Buff[Index] == 0xEA && Receive_Buff[Index - 1] == 0xEA){
                     IsStartBit = 1;
                     StartIndex = Index + 1;
                     sciReceiving = 1;
                 }
                 if(IsStartBit){
                     if(Receive_Buff[Index] == 0xAE && Receive_Buff[Index - 1] == 0xAE){
                         IsEndBit = 1;
                         EndIndex = Index - 2;
                     }
                 }
                 if(Receive_Buff[Index] == 0xFA && Receive_Buff[Index - 1] == 0xFA){
                     Ack = 1;
                 }
            }

            if(Index < 26){
                Index++;
            }
            else{
                ReceiveEnable = 0;
                Index = 0;
                GpioDataRegs.GPASET.bit.GPIO9 = 1;
                break;
             }
           }
           GpioDataRegs.GPBCLEAR.bit.GPIO50 = 1;
           // aux4_f += 0.01;
           int i = 0;
           int j = 0;

           if(IsEndBit){
               for(i = StartIndex; i < EndIndex; i++){
                   if(((Receive_Buff[i] == 0xFA) && (Receive_Buff[i+1] == 0xFA)) || ((Receive_Buff[i] == 0xFA) && (Receive_Buff[i-1] == 0xFA))){

                   }
                   else{
                       SPIArray[j] = Receive_Buff[i];
                       if(j < 20){
                           j++;
                       }
                       else{
                           j=0;
                           break;
                       }
                   }
               }

               StartIndex = 0;
               EndIndex = 0;
               IsStartBit = 0;
               IsEndBit = 0;

           }

        }
    }
}

__interrupt void receive_interrupt(void){
    ReceiveEnable = 1;
    GpioDataRegs.GPACLEAR.bit.GPIO9 = 1;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

void initUART(){

    // 1 stop bit,  No loopback, No parity,8 char bits, async mode,
    // idle-line protocol
    SciaRegs.SCICCR.all =0x0007;

     // enable TX, RX, internal SCICLK, Disable RX ERR, SLEEP, TXWAKE
    SciaRegs.SCICTL1.all =0x0001;

    SciaRegs.SCICTL2.bit.RXBKINTENA = 1;

    // 9600 baud @LSPCLK = 22.5MHz (90 MHz SYSCLK)
    SciaRegs.SCIHBAUD    =0x0001;
    SciaRegs.SCILBAUD    =0x0024;


    SciaRegs.SCICTL1.all =0x0023;  // Relinquish SCI from Reset
}

void interrupt_gpio(void){
    EALLOW;
    GpioCtrlRegs.GPAMUX2.bit.GPIO30 = 0;    // GPIO
    GpioCtrlRegs.GPADIR.bit.GPIO30 = 0;     // Input
    GpioCtrlRegs.GPAQSEL2.bit.GPIO30 = 0;   //  tried to change

    GpioDataRegs.GPACLEAR.bit.GPIO9 = 1;
    GpioCtrlRegs.GPAMUX1.bit.GPIO9 = 0;
    GpioCtrlRegs.GPADIR.bit.GPIO9 = 1;

    GpioCtrlRegs.GPBMUX2.bit.GPIO50 = 0;
    GpioCtrlRegs.GPBPUD.bit.GPIO50 = 1;
    GpioCtrlRegs.GPBMUX2.bit.GPIO50 = 0; // LED
    EDIS;

    EALLOW;
    GpioIntRegs.GPIOXINT1SEL.bit.GPIOSEL = 30; // XINT1 is GPIO30
    EDIS;


    // configure XINT1
    XIntruptRegs.XINT1CR.bit.POLARITY = 0;  // Falling edge interrupt
//  XIntruptRegs.XINT1CR.bit.POLARITY = 1;  // Rising edge interrupt

    // enable XINT1
    XIntruptRegs.XINT1CR.bit.ENABLE = 1;

}

void scia_fifo_init()
{
    SciaRegs.SCIFFTX.all=0xE040;
    SciaRegs.SCIFFRX.all=0x2044;
    SciaRegs.SCIFFCT.all=0x0;
}



