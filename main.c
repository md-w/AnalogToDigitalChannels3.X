
// refer file:///C:/Program%20Files/Microchip/xc8/v2.40/docs/chips/18f4520.html

// CONFIG1H
#pragma config OSC = HS         // Oscillator Selection bits (HS oscillator)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
#pragma config IESO = OFF       // Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)

// CONFIG2L
#pragma config PWRT = ON        // Power-up Timer Enable bit (PWRT enabled)
#pragma config BOREN = ON       // Brown-out Reset Enable bits (Brown-out Reset enabled and controlled by software (SBOREN is enabled))
#pragma config BORV = 2         // Brown Out Reset Voltage bits (Minimum setting)

// CONFIG2H
#pragma config WDT = ON         // Watchdog Timer Enable bit (WDT enabled)
#pragma config WDTPS = 1024     // Watchdog Timer Postscale Select bits (1:1024)

// CONFIG3H
#pragma config CCP2MX = PORTBE  // CCP2 MUX bit (CCP2 input/output is multiplexed with RB3)
#pragma config PBADEN = OFF      // PORTB A/D Enable bit (PORTB<4:0> pins are configured as analog input channels on Reset)
#pragma config LPT1OSC = OFF    // Low-Power Timer1 Oscillator Enable bit (Timer1 configured for higher power operation)
#pragma config MCLRE = ON       // MCLR Pin Enable bit (MCLR pin enabled; RE3 input pin disabled)

// CONFIG4L
#pragma config STVREN = ON      // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config LVP = OFF        // Single-Supply ICSP Enable bit (Single-Supply ICSP disabled)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))

// CONFIG5L
#pragma config CP0 = OFF        // Code Protection bit (Block 0 (000800-001FFFh) not code-protected)
#pragma config CP1 = OFF        // Code Protection bit (Block 1 (002000-003FFFh) not code-protected)
#pragma config CP2 = OFF        // Code Protection bit (Block 2 (004000-005FFFh) not code-protected)
#pragma config CP3 = OFF        // Code Protection bit (Block 3 (006000-007FFFh) not code-protected)

// CONFIG5H
#pragma config CPB = OFF        // Boot Block Code Protection bit (Boot block (000000-0007FFh) not code-protected)
#pragma config CPD = OFF        // Data EEPROM Code Protection bit (Data EEPROM not code-protected)

// CONFIG6L
#pragma config WRT0 = OFF       // Write Protection bit (Block 0 (000800-001FFFh) not write-protected)
#pragma config WRT1 = OFF       // Write Protection bit (Block 1 (002000-003FFFh) not write-protected)
#pragma config WRT2 = OFF       // Write Protection bit (Block 2 (004000-005FFFh) not write-protected)
#pragma config WRT3 = OFF       // Write Protection bit (Block 3 (006000-007FFFh) not write-protected)

// CONFIG6H
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot block (000000-0007FFh) not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM not write-protected)

// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protection bit (Block 0 (000800-001FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF      // Table Read Protection bit (Block 1 (002000-003FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR2 = OFF      // Table Read Protection bit (Block 2 (004000-005FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR3 = OFF      // Table Read Protection bit (Block 3 (006000-007FFFh) not protected from table reads executed in other blocks)

// CONFIG7H
#pragma config EBTRB = OFF      // Boot Block Table Read Protection bit (Boot block (000000-0007FFh) not protected from table reads executed in other blocks)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#include "tick.h"
#include "displayKeyTickCPU.h"
#include "eeprom_wrapper.h"

#define PSWD                        10

#define ANALOG_FULLSCALE   1500

#define M_MAX   ((char)120)
#define M_MIN   ((char)-120)

#define C_MAX   ((char)120)
#define C_MIN   ((char)-120)

#define SET_POINT_HI_MAX  (ANALOG_FULLSCALE/10 - 1)
#define SET_POINT_HI_MIN  (60)

#define SET_POINT_HI_MAX3  (760 - 1)
#define SET_POINT_HI_MIN3  (500)

#define SET_POINT_LO_MAX  (50)
#define SET_POINT_LO_MIN  (30)

#define SET_POINT_LO_MAX3  SET_POINT_HI_MIN3
#define SET_POINT_LO_MIN3  (100)

typedef struct _structControllerSP {
    unsigned int ucLoSetPoint1;
    unsigned int ucHiSetPoint1;
    unsigned int ucLoSetPoint2;
    unsigned int ucHiSetPoint2;
    unsigned int ucLoSetPoint3;
    unsigned int ucHiSetPoint3;
    signed char cMCh1;
    signed char cCCh1;
    signed char cMCh2;
    signed char cCCh2;
    signed char cMCh3;
    signed char cCCh3;
} STRUCT_CONTROLLER_SP;

unsigned char dispMainState = 0;
unsigned char dispSubState = 0;
unsigned char statusByte0 = 0;
unsigned char statusByte1 = 0;
unsigned char statusByte2 = 0;
unsigned char statusByte3 = 0;
unsigned char sec60Counter = 60;
unsigned char ucPassword = PSWD + 5;

bool outputLatch1 = 0;
bool outputLatch1Hist = 0;

bool outputLatch2 = 0;
bool outputLatch2Hist = 0;

bool outputLatch3 = 0;
bool outputLatch3Hist = 0;
//bool alarmLatch = 0;
bool isSound = 0;
STRUCT_CONTROLLER_SP SetPoint;

#define RELAY_LAT LATAbits.LATA5
#define RELAY_TRIS TRISAbits.TRISA5


#define LED1_HI_LAT LATCbits.LATC7
#define LED1_NORMAL_LAT LATCbits.LATC6
#define LED1_LO_LAT LATCbits.LATC5


#define LED1_HI_TRIS TRISCbits.TRISC7
#define LED1_NORMAL_TRIS TRISCbits.TRISC6
#define LED1_LO_TRIS TRISCbits.TRISC5

#define LED2_HI_LAT LATEbits.LATE0
#define LED2_NORMAL_LAT LATEbits.LATE1
#define LED2_LO_LAT LATEbits.LATE2


#define LED2_HI_TRIS TRISEbits.TRISE0
#define LED2_NORMAL_TRIS TRISEbits.TRISE1
#define LED2_LO_TRIS TRISEbits.TRISE2

#define LED3_HI_LAT LATCbits.LATC0
#define LED3_NORMAL_LAT LATAbits.LATA4
#define LED3_LO_LAT LATAbits.LATA3


#define LED3_HI_TRIS TRISCbits.TRISC0
#define LED3_NORMAL_TRIS TRISAbits.TRISA4
#define LED3_LO_TRIS TRISAbits.TRISA3

#define _XTAL_FREQ 20000000

void DELAY_milliseconds(uint16_t milliseconds) {
    while (milliseconds--) {
        __delay_ms(1);
    }
}

void INTERRUPT_Initialize(void) {
    // Disable Interrupt Priority Vectors (16CXXX Compatibility Mode)
    RCONbits.IPEN = 0;
}

void __interrupt() INTERRUPT_InterruptManagerLow(void) {
    // interrupt handler
    if (INTCONbits.TMR0IE && INTCONbits.TMR0IF) {
        // reload TMR0
        TMR0L = timer0ReloadVal;
        displayISR();
        // clear the TMR0 interrupt flag
        INTCONbits.TMR0IF = 0;
    } else if (INTCONbits.INT0IE && INTCONbits.INT0IF) {
        INTCONbits.INT0IF = 0;
    }
}


#define INTERRUPT_GlobalInterruptEnable() (INTCONbits.GIE = 1)
#define INTERRUPT_GlobalInterruptDisable() (INTCONbits.GIE = 0)
#define INTERRUPT_GlobalInterruptHighEnable() (INTCONbits.GIEH = 1)
#define INTERRUPT_GlobalInterruptHighDisable() (INTCONbits.GIEH = 0)
#define INTERRUPT_GlobalInterruptLowEnable() (INTCONbits.GIEL = 1)
#define INTERRUPT_GlobalInterruptLowDisable() (INTCONbits.GIEL = 0)
#define INTERRUPT_PeripheralInterruptEnable() (INTCONbits.PEIE = 1)
#define INTERRUPT_PeripheralInterruptDisable() (INTCONbits.PEIE = 0)

void SYSTEM_Initialize(void) {

    INTERRUPT_Initialize();
    initDisplay();
    initTimer0();
    initExternalInterrupt();
}

void initVariables(void) {
    readByte((unsigned char *) &SetPoint, 0, sizeof (SetPoint));
    if ((SetPoint.ucHiSetPoint1 < SET_POINT_HI_MIN) || (SetPoint.ucHiSetPoint1 > SET_POINT_HI_MAX)) {
        SetPoint.ucHiSetPoint1 = 100;
    }
    if ((SetPoint.ucLoSetPoint1 < SET_POINT_LO_MIN) || (SetPoint.ucLoSetPoint1 > SET_POINT_LO_MAX)) {
        SetPoint.ucLoSetPoint1 = SET_POINT_LO_MIN;
    }
    if ((SetPoint.cMCh1 < -99) || (SetPoint.cMCh1 > 99)) {
        SetPoint.cMCh1 = 0;
    }
    if ((SetPoint.cCCh1 < -99) || (SetPoint.cCCh1 > 99)) {
        SetPoint.cCCh1 = 0;
    }

    if ((SetPoint.ucHiSetPoint2 < SET_POINT_HI_MIN) || (SetPoint.ucHiSetPoint2 > SET_POINT_HI_MAX)) {
        SetPoint.ucHiSetPoint2 = 100;
    }
    if ((SetPoint.ucLoSetPoint2 < SET_POINT_LO_MIN) || (SetPoint.ucLoSetPoint2 > SET_POINT_LO_MAX)) {
        SetPoint.ucLoSetPoint2 = SET_POINT_LO_MIN;
    }
    if ((SetPoint.cMCh2 < -99) || (SetPoint.cMCh2 > 99)) {
        SetPoint.cMCh2 = 0;
    }
    if ((SetPoint.cCCh2 < -99) || (SetPoint.cCCh2 > 99)) {
        SetPoint.cCCh2 = 0;
    }
    if ((SetPoint.ucHiSetPoint3 < SET_POINT_HI_MIN3) || (SetPoint.ucHiSetPoint3 > SET_POINT_HI_MAX3)) {
        SetPoint.ucHiSetPoint3 = SET_POINT_HI_MAX3;
    }
    if ((SetPoint.ucLoSetPoint3 < SET_POINT_LO_MIN3) || (SetPoint.ucLoSetPoint3 > SET_POINT_LO_MAX3)) {
        SetPoint.ucLoSetPoint3 = SET_POINT_LO_MIN3;
    }
    if ((SetPoint.cMCh3 < -99) || (SetPoint.cMCh3 > 99)) {
        SetPoint.cMCh3 = 0;
    }
    if ((SetPoint.cCCh3 < -99) || (SetPoint.cCCh3 > 99)) {
        SetPoint.cCCh3 = 0;
    }
}

void main(void) {
    SYSTEM_Initialize();

    RELAY_TRIS = 0;
    RELAY_LAT = 0;

    LED1_HI_TRIS = 0;
    LED1_NORMAL_TRIS = 0;
    LED1_LO_TRIS = 0;

    LED2_HI_TRIS = 0;
    LED2_NORMAL_TRIS = 0;
    LED2_LO_TRIS = 0;

    LED3_HI_TRIS = 0;
    LED3_NORMAL_TRIS = 0;
    LED3_LO_TRIS = 0;

    INTERRUPT_GlobalInterruptEnable();
    initVariables();
    initADC();

    DISP_MUX_PORT_TRIS_DIGIT3 = 0;
    DISP_MUX_PORT_TRIS_DIGIT2 = 0;
    DISP_MUX_PORT_TRIS_DIGIT1 = 0;
    DISP_MUX_PORT_TRIS_DIGIT0 = 0;

    DISP2_MUX_PORT_TRIS_DIGIT2 = 0;
    DISP2_MUX_PORT_TRIS_DIGIT1 = 0;
    DISP2_MUX_PORT_TRIS_DIGIT0 = 0;

    DISP3_MUX_PORT_TRIS_DIGIT2 = 0;
    DISP3_MUX_PORT_TRIS_DIGIT1 = 0;
    DISP3_MUX_PORT_TRIS_DIGIT0 = 0;

    while (1) {
        statusByte0 = 0x00;
        statusByte1 = 0x00;
        statusByte2 = 0x00;
        statusByte3 = 0x00;
        adcTask();
        if (tick1000mSec) {
            tick1000mSec = 0;
        }
        //        display2Int(counter2, 0);
        //        display3Int(counter3, 0);
        if (tick250mSec) {
            tick250mSec = 0;
        }

        // x :     0     0.5   4.5      5
        // x :     0     102   921      1023
        // y :           760     0      


        int iRoomTemp1 = (((long) iADCValCh1 + (long) SetPoint.cCCh1)*((long) ANALOG_FULLSCALE + (long) SetPoint.cMCh1 * 10)) / (long) 10230;
        int iRoomTemp2 = (((long) iADCValCh2 + (long) SetPoint.cCCh2)*((long) ANALOG_FULLSCALE + (long) SetPoint.cMCh2 * 10)) / (long) 10230;
        //        int iRoomTemp3 = (((long) iADCValCh3 + (long) SetPoint.cCCh3)*((long) ANALOG_FULLSCALE + (long) SetPoint.cMCh3 * 10)) / (long) 10230;

        int iRoomTemp3 = -((760 + (long) SetPoint.cMCh3)*(long) iADCValCh3) / 819 + 855 + (long) SetPoint.cCCh3;



        //        int iRoomTemp = -(760*(long)iADCValCh1)/819 + 855;

        if (iRoomTemp1 < SetPoint.ucLoSetPoint1) {
            statusByte1 |= 0x02;
            outputLatch1 = 1;
        } else if (iRoomTemp1 > SetPoint.ucHiSetPoint1) {
            statusByte1 |= 0x01;
            outputLatch1 = 1;
        } else {
            statusByte1 |= 0x04;
            outputLatch1 = 0;
        }
        if (!outputLatch1Hist && outputLatch1) {
            isSound = 1;
        }
        outputLatch1Hist = outputLatch1;

        if (iRoomTemp2 < SetPoint.ucLoSetPoint2) {
            statusByte2 |= 0x02;
            outputLatch2 = 1;
        } else if (iRoomTemp2 > SetPoint.ucHiSetPoint2) {
            statusByte2 |= 0x01;
            outputLatch2 = 1;
        } else {
            statusByte2 |= 0x04;
            outputLatch2 = 0;
        }
        if (!outputLatch2Hist && outputLatch2) {
            isSound = 1;
        }
        outputLatch2Hist = outputLatch2;

        if (iRoomTemp3 < SetPoint.ucLoSetPoint3) {
            statusByte3 |= 0x02;
            outputLatch3 = 1;
        } else if (iRoomTemp3 > SetPoint.ucHiSetPoint3) {
            statusByte3 |= 0x01;
            outputLatch3 = 1;
        } else {
            statusByte3 |= 0x04;
            outputLatch3 = 0;
        }
        if (!outputLatch3Hist && outputLatch3) {
            isSound = 1;
        }
        outputLatch3Hist = outputLatch3;

        switch (dispMainState) {
            case 0:
                display1SignedInt(iRoomTemp1, 0);
                display2SignedInt(iRoomTemp2, 0);
                display3SignedInt(iRoomTemp3, 0);
                if (keyDown & DNKEY_MASK) {
                    keyDown = 0x00;
                    keyHold = 0x00;
                    isSound = !isSound;
                }
                if (keyDown & PROKEY_MASK) {
                    keyDown = 0x00;
                    keyHold = 0x00;
                    dispMainState++;
                }
                break;
            case 1: //enter PRO
                statusByte0 |= 0x01;
                clear2();
                clear3();
                digit1Assign(SEG_P, 2);
                digit1Assign(SEG_R, 1);
                digit1Assign(SEG_O, 0);

                if (keyDown & PROKEY_MASK) {
                    keyDown = 0x00;
                    keyHold = 0x00;
                    dispMainState += 2;
                }
                if (keyDown & ENTKEY_MASK) {
                    keyDown = 0x00;
                    keyHold = 0x00;
                    dispMainState++;
                    dispSubState = 0;
                }
                break;
            case 2: //PRO
                statusByte0 |= 0x01;
                clear2();
                clear3();
                switch (dispSubState) {
                    case 0:
                        statusByte0 |= 0x04;
                        checkAndInrDcrInt(&SetPoint.ucHiSetPoint1, SET_POINT_HI_MAX, SET_POINT_HI_MIN);
                        display1Int(SetPoint.ucHiSetPoint1, 0);
                        if ((keyDown & ENTKEY_MASK)) {
                            keyDown = 0x00;
                            keyHold = 0x00;
                            dispSubState++;
                        }
                        break;
                    case 1:
                        statusByte0 |= 0x08;
                        checkAndInrDcrInt(&SetPoint.ucLoSetPoint1, SetPoint.ucHiSetPoint1, SET_POINT_LO_MIN);
                        display1Int(SetPoint.ucLoSetPoint1, 0);
                        if ((keyDown & ENTKEY_MASK)) {
                            keyDown = 0x00;
                            keyHold = 0x00;
                            dispSubState++;
                        }
                        break;
                    default:
                        dispSubState = 0;
                        break;
                }
                if ((keyDown & PROKEY_MASK)) {
                    keyDown = 0x00;
                    keyHold = 0x00;
                    dispMainState++;
                    dispSubState = 0;
                    writeByte((unsigned char *) &SetPoint, 0, sizeof (SetPoint));
                }
                break;
            case 3: //enter CAL
                statusByte0 |= 0x02;
                clear2();
                clear3();
                digit1Assign(SEG_C, 2);
                digit1Assign(SEG_A, 1);
                digit1Assign(SEG_L, 0);
                if (keyDown & PROKEY_MASK) {
                    keyDown = 0x00;
                    keyHold = 0x00;
                    dispMainState += 2;
                }
                if (keyDown & ENTKEY_MASK) {
                    keyDown = 0x00;
                    keyHold = 0x00;
                    dispMainState++;
                    dispSubState = 0;
                    selectedIndx = 0;
                }
                break;
            case 4: //CAL
                statusByte0 |= 0x02;
                clear2();
                clear3();
                switch (dispSubState) {
                    case 0:
                        checkAndInrDcrSignedChar(&SetPoint.cMCh1, 99, -99);
                        if (bToggleBitSlow) {
                            statusByte0 |= 0x04;
                            display1SignedInt(SetPoint.cMCh1, 0);
                        } else {
                            display1SignedInt(iRoomTemp1, 0);
                        }
                        if ((keyDown & ENTKEY_MASK)) {
                            keyDown = 0x00;
                            keyHold = 0x00;
                            dispSubState++;
                        }
                        break;
                    case 1:
                        checkAndInrDcrSignedChar(&SetPoint.cCCh1, 99, -99);
                        if (bToggleBitSlow) {
                            statusByte0 |= 0x08;
                            display1SignedInt(SetPoint.cCCh1, 0);
                        } else {
                            display1SignedInt(iRoomTemp1, 0);
                        }

                        if ((keyDown & ENTKEY_MASK)) {
                            keyDown = 0x00;
                            keyHold = 0x00;
                            dispSubState++;
                        }
                        break;
                    default:
                        dispSubState = 0;
                        break;

                }
                if ((keyDown & PROKEY_MASK)) {
                    keyDown = 0x00;
                    keyHold = 0x00;
                    dispMainState++;
                    dispSubState = 0;
                    writeByte((unsigned char *) &SetPoint, 0, sizeof (SetPoint));
                }
                break;
            case 1 + 4: //enter PRO
                statusByte0 |= 0x01;
                clear1();
                clear3();
                digit2Assign(SEG_P, 2);
                digit2Assign(SEG_R, 1);
                digit2Assign(SEG_O, 0);

                if (keyDown & PROKEY_MASK) {
                    keyDown = 0x00;
                    keyHold = 0x00;
                    dispMainState += 2;
                }
                if (keyDown & ENTKEY_MASK) {
                    keyDown = 0x00;
                    keyHold = 0x00;
                    dispMainState++;
                    dispSubState = 0;
                }
                break;
            case 2 + 4: //PRO
                statusByte0 |= 0x01;
                clear1();
                clear3();
                switch (dispSubState) {
                    case 0:
                        statusByte0 |= 0x04;
                        checkAndInrDcrInt(&SetPoint.ucHiSetPoint2, SET_POINT_HI_MAX, SET_POINT_HI_MIN);
                        display2Int(SetPoint.ucHiSetPoint2, 0);
                        if ((keyDown & ENTKEY_MASK)) {
                            keyDown = 0x00;
                            keyHold = 0x00;
                            dispSubState++;
                        }
                        break;
                    case 1:
                        statusByte0 |= 0x08;
                        checkAndInrDcrInt(&SetPoint.ucLoSetPoint2, SetPoint.ucHiSetPoint2, SET_POINT_LO_MIN);
                        display2Int(SetPoint.ucLoSetPoint2, 0);
                        if ((keyDown & ENTKEY_MASK)) {
                            keyDown = 0x00;
                            keyHold = 0x00;
                            dispSubState++;
                        }
                        break;
                    default:
                        dispSubState = 0;
                        break;
                }
                if ((keyDown & PROKEY_MASK)) {
                    keyDown = 0x00;
                    keyHold = 0x00;
                    dispMainState++;
                    dispSubState = 0;
                    writeByte((unsigned char *) &SetPoint, 0, sizeof (SetPoint));
                }
                break;
            case 3 + 4: //enter CAL
                statusByte0 |= 0x02;
                clear1();
                clear3();
                digit2Assign(SEG_C, 2);
                digit2Assign(SEG_A, 1);
                digit2Assign(SEG_L, 0);
                if (keyDown & PROKEY_MASK) {
                    keyDown = 0x00;
                    keyHold = 0x00;
                    dispMainState += 2;
                }
                if (keyDown & ENTKEY_MASK) {
                    keyDown = 0x00;
                    keyHold = 0x00;
                    dispMainState++;
                    dispSubState = 0;
                    selectedIndx = 0;
                }
                break;
            case 4 + 4: //CAL
                statusByte0 |= 0x02;
                clear1();
                clear3();
                switch (dispSubState) {
                    case 0:
                        checkAndInrDcrSignedChar(&SetPoint.cMCh2, 99, -99);
                        if (bToggleBitSlow) {
                            statusByte0 |= 0x04;
                            display2SignedInt(SetPoint.cMCh2, 0);
                        } else {
                            display2SignedInt(iRoomTemp2, 0);
                        }
                        if ((keyDown & ENTKEY_MASK)) {
                            keyDown = 0x00;
                            keyHold = 0x00;
                            dispSubState++;
                        }
                        break;
                    case 1:
                        checkAndInrDcrSignedChar(&SetPoint.cCCh2, 99, -99);
                        if (bToggleBitSlow) {
                            statusByte0 |= 0x08;
                            display2SignedInt(SetPoint.cCCh2, 0);
                        } else {
                            display2SignedInt(iRoomTemp2, 0);
                        }

                        if ((keyDown & ENTKEY_MASK)) {
                            keyDown = 0x00;
                            keyHold = 0x00;
                            dispSubState++;
                        }
                        break;
                    default:
                        dispSubState = 0;
                        break;

                }
                if ((keyDown & PROKEY_MASK)) {
                    keyDown = 0x00;
                    keyHold = 0x00;
                    dispMainState++;
                    dispSubState = 0;
                    writeByte((unsigned char *) &SetPoint, 0, sizeof (SetPoint));
                }
                break;
            case 1 + 8: //enter PRO
                statusByte0 |= 0x01;
                clear1();
                clear2();
                digit3Assign(SEG_P, 2);
                digit3Assign(SEG_R, 1);
                digit3Assign(SEG_O, 0);

                if (keyDown & PROKEY_MASK) {
                    keyDown = 0x00;
                    keyHold = 0x00;
                    dispMainState += 2;
                }
                if (keyDown & ENTKEY_MASK) {
                    keyDown = 0x00;
                    keyHold = 0x00;
                    dispMainState++;
                    dispSubState = 0;
                }
                break;
            case 2 + 8: //PRO
                statusByte0 |= 0x01;
                clear1();
                clear2();
                switch (dispSubState) {
                    case 0:
                        statusByte0 |= 0x04;
                        checkAndInrDcrInt(&SetPoint.ucHiSetPoint3, SET_POINT_HI_MAX3, SET_POINT_HI_MIN3);
                        display3Int(SetPoint.ucHiSetPoint3, 0);
                        if ((keyDown & ENTKEY_MASK)) {
                            keyDown = 0x00;
                            keyHold = 0x00;
                            dispSubState++;
                        }
                        break;
                    case 1:
                        statusByte0 |= 0x08;
                        checkAndInrDcrInt(&SetPoint.ucLoSetPoint3, SetPoint.ucHiSetPoint3, SET_POINT_LO_MIN3);
                        display3Int(SetPoint.ucLoSetPoint3, 0);
                        if ((keyDown & ENTKEY_MASK)) {
                            keyDown = 0x00;
                            keyHold = 0x00;
                            dispSubState++;
                        }
                        break;
                    default:
                        dispSubState = 0;
                        break;
                }
                if ((keyDown & PROKEY_MASK)) {
                    keyDown = 0x00;
                    keyHold = 0x00;
                    dispMainState++;
                    dispSubState = 0;
                    writeByte((unsigned char *) &SetPoint, 0, sizeof (SetPoint));
                }
                break;
            case 3 + 8: //enter CAL
                statusByte0 |= 0x02;
                clear1();
                clear2();
                digit3Assign(SEG_C, 2);
                digit3Assign(SEG_A, 1);
                digit3Assign(SEG_L, 0);
                if (keyDown & PROKEY_MASK) {
                    keyDown = 0x00;
                    keyHold = 0x00;
                    dispMainState += 2;
                }
                if (keyDown & ENTKEY_MASK) {
                    keyDown = 0x00;
                    keyHold = 0x00;
                    dispMainState++;
                    dispSubState = 0;
                    selectedIndx = 0;
                }
                break;
            case 4 + 8: //CAL
                statusByte0 |= 0x02;
                clear1();
                clear2();
                switch (dispSubState) {
                    case 0:
                        checkAndInrDcrSignedChar(&SetPoint.cMCh3, 99, -99);
                        if (bToggleBitSlow) {
                            statusByte0 |= 0x04;
                            display3SignedInt(SetPoint.cMCh3, 0);
                        } else {
                            display3SignedInt(iRoomTemp3, 0);
                        }
                        if ((keyDown & ENTKEY_MASK)) {
                            keyDown = 0x00;
                            keyHold = 0x00;
                            dispSubState++;
                        }
                        break;
                    case 1:
                        checkAndInrDcrSignedChar(&SetPoint.cCCh3, 99, -99);
                        if (bToggleBitSlow) {
                            statusByte0 |= 0x08;
                            display3SignedInt(SetPoint.cCCh3, 0);
                        } else {
                            display3SignedInt(iRoomTemp3, 0);
                        }

                        if ((keyDown & ENTKEY_MASK)) {
                            keyDown = 0x00;
                            keyHold = 0x00;
                            dispSubState++;
                        }
                        break;
                    default:
                        dispSubState = 0;
                        break;

                }
                if ((keyDown & PROKEY_MASK)) {
                    keyDown = 0x00;
                    keyHold = 0x00;
                    dispMainState++;
                    dispSubState = 0;
                    writeByte((unsigned char *) &SetPoint, 0, sizeof (SetPoint));
                }
                break;
            default:
                dispMainState = 0;
                break;
        }

        bool should1On = 0;
        bool should2On = 0;
        bool should3On = 0;

        if (statusByte1 & 0x04) {
            LED1_NORMAL_LAT = 1;
        } else {
            LED1_NORMAL_LAT = 0;
        }

        if (statusByte1 & 0x01) {
            if (bToggleBitSlow) {
                should1On = 1;
            }
        }
        if (should1On) {
            LED1_HI_LAT = 1;
        } else {
            LED1_HI_LAT = 0;
        }
        should1On = 0;
        if (statusByte1 & 0x02) {
            if (bToggleBitSlow) {
                should1On = 1;
            }
        }
        if (should1On) {
            LED1_LO_LAT = 1;
        } else {
            LED1_LO_LAT = 0;
        }
        should1On = 0;
        if (outputLatch1) {
            if (isSound) {
                if (bToggleBitSlow) {
                    should1On = 1;
                }
            }
        }


        if (statusByte2 & 0x04) {
            LED2_NORMAL_LAT = 1;
        } else {
            LED2_NORMAL_LAT = 0;
        }

        if (statusByte2 & 0x01) {
            if (bToggleBitSlow) {
                should2On = 1;
            }
        }
        if (should2On) {
            LED2_HI_LAT = 1;
        } else {
            LED2_HI_LAT = 0;
        }
        should2On = 0;
        if (statusByte2 & 0x02) {
            if (bToggleBitSlow) {
                should2On = 1;
            }
        }
        if (should2On) {
            LED2_LO_LAT = 1;
        } else {
            LED2_LO_LAT = 0;
        }
        should2On = 0;
        if (outputLatch2) {
            if (isSound) {
                if (bToggleBitSlow) {
                    should2On = 1;
                }
            }
        }

        if (statusByte3 & 0x04) {
            LED3_NORMAL_LAT = 1;
        } else {
            LED3_NORMAL_LAT = 0;
        }

        if (statusByte3 & 0x01) {
            if (bToggleBitSlow) {
                should3On = 1;
            }
        }
        if (should3On) {
            LED3_HI_LAT = 1;
        } else {
            LED3_HI_LAT = 0;
        }
        should3On = 0;
        if (statusByte3 & 0x02) {
            if (bToggleBitSlow) {
                should3On = 1;
            }
        }
        if (should3On) {
            LED3_LO_LAT = 1;
        } else {
            LED3_LO_LAT = 0;
        }
        should3On = 0;
        if (outputLatch3) {
            if (isSound) {
                if (bToggleBitSlow) {
                    should3On = 1;
                }
            }
        }


        if (should1On || should2On || should3On) {
            RELAY_LAT = 1;
        } else {
            RELAY_LAT = 0;
        }
        statusByte0 |= statusByte0 << 4;
        directAssign(statusByte0, 3);
        display();
        ClrWdt();
    }
}
