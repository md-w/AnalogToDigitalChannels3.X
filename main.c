
// refer file:///C:/Program%20Files/Microchip/xc8/v2.40/docs/chips/18f4520.html

#define MODE_3CH    0

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

#define ANALOG_FULLSCALE   1500

#define BAR_SCALE_NUM      100
#define BAR_SCALE_DEN      145

#define M_MAX   ((signed char)99)
#define M_MIN   ((signed char)-99)

#define C_MAX   ((signed char)99)
#define C_MIN   ((signed char)-99)

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

typedef enum {
    DISP_MAIN = 0,
    DISP_CH1_ENTER_PRO = 1,
    DISP_CH1_PRO = 2,
    DISP_CH1_ENTER_CAL = 3,
    DISP_CH1_CAL = 4,
    DISP_CH2_ENTER_PRO = 5,
    DISP_CH2_PRO = 6,
    DISP_CH2_ENTER_CAL = 7,
    DISP_CH2_CAL = 8,
    DISP_CH3_ENTER_PRO = 9,
    DISP_CH3_PRO = 10,
    DISP_CH3_ENTER_CAL = 11,
    DISP_CH3_CAL = 12,
} DispMainState;

typedef enum {
    SUB_FIRST = 0,
    SUB_SECOND = 1,
} DispSubState;

DispMainState dispMainState = DISP_MAIN;
DispSubState dispSubState = SUB_FIRST;
unsigned char statusByte0 = 0;
unsigned char statusByte1 = 0;
unsigned char statusByte2 = 0;
unsigned char statusByte3 = 0;

bool outputLatch1 = 0;
bool outputLatch1Hist = 0;

bool outputLatch2 = 0;
bool outputLatch2Hist = 0;

bool outputLatch3 = 0;
bool outputLatch3Hist = 0;

bool isSound = 0;
bool isBarMode = 0;
bool isTestMode = 0;
STRUCT_CONTROLLER_SP SetPoint;

#define RELAY_LAT LATAbits.LATA5
#define RELAY_TRIS TRISAbits.TRISA5


#define LED1_HI_LAT LATCbits.LATC5
#define LED1_NORMAL_LAT LATCbits.LATC6
#define LED1_LO_LAT LATCbits.LATC7


#define LED1_HI_TRIS TRISCbits.TRISC5
#define LED1_NORMAL_TRIS TRISCbits.TRISC6
#define LED1_LO_TRIS TRISCbits.TRISC7

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
    if (INTCONbits.TMR0IE && INTCONbits.TMR0IF) {
        TMR0L = timer0ReloadVal;
        displayISR();
        INTCONbits.TMR0IF = 0;
    } else if (INTCONbits.INT0IE && INTCONbits.INT0IF) {
        INTCONbits.INT0IF = 0;
    }
}

#define INTERRUPT_GlobalInterruptEnable() (INTCONbits.GIE = 1)

void SYSTEM_Initialize(void) {
    INTERRUPT_Initialize();
    initDisplay();
    initTimer0();
    initExternalInterrupt();
}

static void clampUInt(unsigned int *v, unsigned int lo, unsigned int hi, unsigned int dflt) {
    if (*v < lo || *v > hi) *v = dflt;
}

static void clampSChar(signed char *v, signed char lo, signed char hi, signed char dflt) {
    if (*v < lo || *v > hi) *v = dflt;
}

void initVariables(void) {
    readByte((unsigned char *) &SetPoint, 0, sizeof (SetPoint));

    clampUInt(&SetPoint.ucHiSetPoint1, SET_POINT_HI_MIN, SET_POINT_HI_MAX, 100);
    clampUInt(&SetPoint.ucLoSetPoint1, SET_POINT_LO_MIN, SET_POINT_LO_MAX, SET_POINT_LO_MIN);
    clampSChar(&SetPoint.cMCh1, M_MIN, M_MAX, 0);
    clampSChar(&SetPoint.cCCh1, C_MIN, C_MAX, 0);

    clampUInt(&SetPoint.ucHiSetPoint2, SET_POINT_HI_MIN, SET_POINT_HI_MAX, 100);
    clampUInt(&SetPoint.ucLoSetPoint2, SET_POINT_LO_MIN, SET_POINT_LO_MAX, SET_POINT_LO_MIN);
    clampSChar(&SetPoint.cMCh2, M_MIN, M_MAX, 0);
    clampSChar(&SetPoint.cCCh2, C_MIN, C_MAX, 0);

    clampUInt(&SetPoint.ucHiSetPoint3, SET_POINT_HI_MIN3, SET_POINT_HI_MAX3, SET_POINT_HI_MAX3);
    clampUInt(&SetPoint.ucLoSetPoint3, SET_POINT_LO_MIN3, SET_POINT_LO_MAX3, SET_POINT_LO_MIN3);
    clampSChar(&SetPoint.cMCh3, M_MIN, M_MAX, 0);
    clampSChar(&SetPoint.cCCh3, C_MIN, C_MAX, 0);
}

static void evalAlarm(int meas, unsigned int lo, unsigned int hi,
        bool *latch, bool *latchHist, unsigned char *status) {
    if (meas < lo) {
        *status |= 0x02;
        *latch = 1;
    } else if (meas > hi) {
        *status |= 0x01;
        *latch = 1;
    } else {
        *status |= 0x04;
        *latch = 0;
    }
    if (!*latchHist && *latch) isSound = 1;
    *latchHist = *latch;
}

static void consumeKey(void) {
    keyDown = 0x00;
    keyHold = 0x00;
}

static void persistSetpoints(void) {
    writeByte((unsigned char *) &SetPoint, 0, sizeof (SetPoint));
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
        if (tick1000mSec) {
            tick1000mSec = 0;
            isTestMode = 0;
        }
        
        adcTask();

        // x :     0     0.5   4.5      5
        // x :     0     102   921      1023
        // y :           760     0

        int iRoomTemp1 = (((long) iADCValCh1 + (long) SetPoint.cCCh1) * ((long) ANALOG_FULLSCALE + (long) SetPoint.cMCh1 * 10)) / (long) 10230;
        int iRoomTemp2 = (((long) iADCValCh2 + (long) SetPoint.cCCh2) * ((long) ANALOG_FULLSCALE + (long) SetPoint.cMCh2 * 10)) / (long) 10230;
        // channel 3 inverted: higher ADC -> lower reading
        int iRoomTemp3 = -((760 + (long) SetPoint.cMCh3) * (long) iADCValCh3) / 819 + 855 + (long) SetPoint.cCCh3;

        evalAlarm(iRoomTemp1, SetPoint.ucLoSetPoint1, SetPoint.ucHiSetPoint1,
                &outputLatch1, &outputLatch1Hist, &statusByte1);
        evalAlarm(iRoomTemp2, SetPoint.ucLoSetPoint2, SetPoint.ucHiSetPoint2,
                &outputLatch2, &outputLatch2Hist, &statusByte2);
        evalAlarm(iRoomTemp3, SetPoint.ucLoSetPoint3, SetPoint.ucHiSetPoint3,
                &outputLatch3, &outputLatch3Hist, &statusByte3);
        switch (dispMainState) {
            case DISP_MAIN:
                if (isBarMode) {
                    statusByte0 |= 0x80;
                    int iRoomTempInBar1 = ((long)iRoomTemp1 * BAR_SCALE_NUM) / BAR_SCALE_DEN;
                    int iRoomTempInBar2 = ((long)iRoomTemp2 * BAR_SCALE_NUM) / BAR_SCALE_DEN;
                    int iRoomTempInBar3 = ((long)iRoomTemp3 * BAR_SCALE_NUM) / BAR_SCALE_DEN;
                    display1SignedInt(iRoomTempInBar1, 1);
                    display2SignedInt(iRoomTempInBar2, 1);
//                    display3SignedInt(iRoomTempInBar3, 1);
                    display3SignedInt(iRoomTemp3, 0);
                } else {
                    statusByte0 |= 0x40;
                    display1SignedInt(iRoomTemp1, 0);
                    display2SignedInt(iRoomTemp2, 0);
                    display3SignedInt(iRoomTemp3, 0);
                }
                if (keyDown & DNKEY_MASK) {
                    consumeKey();
                    isSound = !isSound;
                }
                if (keyDown & UPKEY_MASK) {                    
                    consumeKey();
                    isTestMode = 1;
                }
                if (keyDown & PROKEY_MASK) {
                    consumeKey();
                    dispMainState = DISP_CH1_ENTER_PRO;
                }
                if (keyDown & ENTKEY_MASK) {
                    consumeKey();
                    isBarMode = !isBarMode;
                }
                break;

            case DISP_CH1_ENTER_PRO:
                statusByte0 |= 0x01;
                clear2();
                clear3();
                digit1Assign(SEG_P, 2);
                digit1Assign(SEG_R, 1);
                digit1Assign(SEG_O, 0);
                if (keyDown & PROKEY_MASK) {
                    consumeKey();
                    dispMainState = DISP_CH1_ENTER_CAL;
                }
                if (keyDown & ENTKEY_MASK) {
                    consumeKey();
                    dispMainState = DISP_CH1_PRO;
                    dispSubState = SUB_FIRST;
                }
                break;

            case DISP_CH1_PRO:
                statusByte0 |= 0x01;
                clear2();
                clear3();
                switch (dispSubState) {
                    case SUB_FIRST:
                        statusByte0 |= 0x04;
                        checkAndInrDcrInt(&SetPoint.ucHiSetPoint1, SET_POINT_HI_MAX, SET_POINT_HI_MIN);
                        display1Int(SetPoint.ucHiSetPoint1, 0);
                        if (keyDown & ENTKEY_MASK) {
                            consumeKey();
                            dispSubState = SUB_SECOND;
                        }
                        break;
                    case SUB_SECOND:
                        statusByte0 |= 0x08;
                        checkAndInrDcrInt(&SetPoint.ucLoSetPoint1, SetPoint.ucHiSetPoint1, SET_POINT_LO_MIN);
                        display1Int(SetPoint.ucLoSetPoint1, 0);
                        if (keyDown & ENTKEY_MASK) {
                            consumeKey();
                            dispSubState = SUB_FIRST;
                        }
                        break;
                    default:
                        dispSubState = SUB_FIRST;
                        break;
                }
                if (keyDown & PROKEY_MASK) {
                    consumeKey();
                    dispMainState = DISP_CH1_ENTER_CAL;
                    dispSubState = SUB_FIRST;
                    persistSetpoints();
                }
                break;

            case DISP_CH1_ENTER_CAL:
                statusByte0 |= 0x02;
                clear2();
                clear3();
                digit1Assign(SEG_C, 2);
                digit1Assign(SEG_A, 1);
                digit1Assign(SEG_L, 0);
                if (keyDown & PROKEY_MASK) {
                    consumeKey();
                    dispMainState = DISP_CH2_ENTER_PRO;
                }
                if (keyDown & ENTKEY_MASK) {
                    consumeKey();
                    dispMainState = DISP_CH1_CAL;
                    dispSubState = SUB_FIRST;
                    selectedIndx = 0;
                }
                break;

            case DISP_CH1_CAL:
                statusByte0 |= 0x02;
                clear2();
                clear3();
                switch (dispSubState) {
                    case SUB_FIRST:
                        checkAndInrDcrSignedChar(&SetPoint.cMCh1, M_MAX, M_MIN);
                        if (bToggleBitSlow) {
                            statusByte0 |= 0x04;
                            display1SignedInt(SetPoint.cMCh1, 0);
                        } else {
                            display1SignedInt(iRoomTemp1, 0);
                        }
                        if (keyDown & ENTKEY_MASK) {
                            consumeKey();
                            dispSubState = SUB_SECOND;
                        }
                        break;
                    case SUB_SECOND:
                        checkAndInrDcrSignedChar(&SetPoint.cCCh1, C_MAX, C_MIN);
                        if (bToggleBitSlow) {
                            statusByte0 |= 0x08;
                            display1SignedInt(SetPoint.cCCh1, 0);
                        } else {
                            display1SignedInt(iRoomTemp1, 0);
                        }
                        if (keyDown & ENTKEY_MASK) {
                            consumeKey();
                            dispSubState = SUB_FIRST;
                        }
                        break;
                    default:
                        dispSubState = SUB_FIRST;
                        break;
                }
                if (keyDown & PROKEY_MASK) {
                    consumeKey();
                    dispMainState = DISP_CH2_ENTER_PRO;
                    dispSubState = SUB_FIRST;
                    persistSetpoints();
                }
                break;

            case DISP_CH2_ENTER_PRO:
                statusByte0 |= 0x01;
                clear1();
                clear3();
                digit2Assign(SEG_P, 2);
                digit2Assign(SEG_R, 1);
                digit2Assign(SEG_O, 0);
                if (keyDown & PROKEY_MASK) {
                    consumeKey();
                    dispMainState = DISP_CH2_ENTER_CAL;
                }
                if (keyDown & ENTKEY_MASK) {
                    consumeKey();
                    dispMainState = DISP_CH2_PRO;
                    dispSubState = SUB_FIRST;
                }
                break;

            case DISP_CH2_PRO:
                statusByte0 |= 0x01;
                clear1();
                clear3();
                switch (dispSubState) {
                    case SUB_FIRST:
                        statusByte0 |= 0x04;
                        checkAndInrDcrInt(&SetPoint.ucHiSetPoint2, SET_POINT_HI_MAX, SET_POINT_HI_MIN);
                        display2Int(SetPoint.ucHiSetPoint2, 0);
                        if (keyDown & ENTKEY_MASK) {
                            consumeKey();
                            dispSubState = SUB_SECOND;
                        }
                        break;
                    case SUB_SECOND:
                        statusByte0 |= 0x08;
                        checkAndInrDcrInt(&SetPoint.ucLoSetPoint2, SetPoint.ucHiSetPoint2, SET_POINT_LO_MIN);
                        display2Int(SetPoint.ucLoSetPoint2, 0);
                        if (keyDown & ENTKEY_MASK) {
                            consumeKey();
                            dispSubState = SUB_FIRST;
                        }
                        break;
                    default:
                        dispSubState = SUB_FIRST;
                        break;
                }
                if (keyDown & PROKEY_MASK) {
                    consumeKey();
                    dispMainState = DISP_CH2_ENTER_CAL;
                    dispSubState = SUB_FIRST;
                    persistSetpoints();
                }
                break;

            case DISP_CH2_ENTER_CAL:
                statusByte0 |= 0x02;
                clear1();
                clear3();
                digit2Assign(SEG_C, 2);
                digit2Assign(SEG_A, 1);
                digit2Assign(SEG_L, 0);
                if (keyDown & PROKEY_MASK) {
                    consumeKey();
                    dispMainState = DISP_CH3_ENTER_PRO;
                }
                if (keyDown & ENTKEY_MASK) {
                    consumeKey();
                    dispMainState = DISP_CH2_CAL;
                    dispSubState = SUB_FIRST;
                    selectedIndx = 0;
                }
                break;

            case DISP_CH2_CAL:
                statusByte0 |= 0x02;
                clear1();
                clear3();
                switch (dispSubState) {
                    case SUB_FIRST:
                        checkAndInrDcrSignedChar(&SetPoint.cMCh2, M_MAX, M_MIN);
                        if (bToggleBitSlow) {
                            statusByte0 |= 0x04;
                            display2SignedInt(SetPoint.cMCh2, 0);
                        } else {
                            display2SignedInt(iRoomTemp2, 0);
                        }
                        if (keyDown & ENTKEY_MASK) {
                            consumeKey();
                            dispSubState = SUB_SECOND;
                        }
                        break;
                    case SUB_SECOND:
                        checkAndInrDcrSignedChar(&SetPoint.cCCh2, C_MAX, C_MIN);
                        if (bToggleBitSlow) {
                            statusByte0 |= 0x08;
                            display2SignedInt(SetPoint.cCCh2, 0);
                        } else {
                            display2SignedInt(iRoomTemp2, 0);
                        }
                        if (keyDown & ENTKEY_MASK) {
                            consumeKey();
                            dispSubState = SUB_FIRST;
                        }
                        break;
                    default:
                        dispSubState = SUB_FIRST;
                        break;
                }
                if (keyDown & PROKEY_MASK) {
                    consumeKey();
                    dispMainState = DISP_CH3_ENTER_PRO;
                    dispSubState = SUB_FIRST;
                    persistSetpoints();
                }
                break;

            case DISP_CH3_ENTER_PRO:
                statusByte0 |= 0x01;
                clear1();
                clear2();
                digit3Assign(SEG_P, 2);
                digit3Assign(SEG_R, 1);
                digit3Assign(SEG_O, 0);
                if (keyDown & PROKEY_MASK) {
                    consumeKey();
                    dispMainState = DISP_CH3_ENTER_CAL;
                }
                if (keyDown & ENTKEY_MASK) {
                    consumeKey();
                    dispMainState = DISP_CH3_PRO;
                    dispSubState = SUB_FIRST;
                }
                break;

            case DISP_CH3_PRO:
                statusByte0 |= 0x01;
                clear1();
                clear2();
                switch (dispSubState) {
                    case SUB_FIRST:
                        statusByte0 |= 0x04;
                        checkAndInrDcrInt(&SetPoint.ucHiSetPoint3, SET_POINT_HI_MAX3, SET_POINT_HI_MIN3);
                        display3Int(SetPoint.ucHiSetPoint3, 0);
                        if (keyDown & ENTKEY_MASK) {
                            consumeKey();
                            dispSubState = SUB_SECOND;
                        }
                        break;
                    case SUB_SECOND:
                        statusByte0 |= 0x08;
                        checkAndInrDcrInt(&SetPoint.ucLoSetPoint3, SetPoint.ucHiSetPoint3, SET_POINT_LO_MIN3);
                        display3Int(SetPoint.ucLoSetPoint3, 0);
                        if (keyDown & ENTKEY_MASK) {
                            consumeKey();
                            dispSubState = SUB_FIRST;
                        }
                        break;
                    default:
                        dispSubState = SUB_FIRST;
                        break;
                }
                if (keyDown & PROKEY_MASK) {
                    consumeKey();
                    dispMainState = DISP_CH3_ENTER_CAL;
                    dispSubState = SUB_FIRST;
                    persistSetpoints();
                }
                break;

            case DISP_CH3_ENTER_CAL:
                statusByte0 |= 0x02;
                clear1();
                clear2();
                digit3Assign(SEG_C, 2);
                digit3Assign(SEG_A, 1);
                digit3Assign(SEG_L, 0);
                if (keyDown & PROKEY_MASK) {
                    consumeKey();
                    dispMainState = DISP_MAIN;
                }
                if (keyDown & ENTKEY_MASK) {
                    consumeKey();
                    dispMainState = DISP_CH3_CAL;
                    dispSubState = SUB_FIRST;
                    selectedIndx = 0;
                }
                break;

            case DISP_CH3_CAL:
                statusByte0 |= 0x02;
                clear1();
                clear2();
                switch (dispSubState) {
                    case SUB_FIRST:
                        checkAndInrDcrSignedChar(&SetPoint.cMCh3, M_MAX, M_MIN);
                        if (bToggleBitSlow) {
                            statusByte0 |= 0x04;
                            display3SignedInt(SetPoint.cMCh3, 0);
                        } else {
                            display3SignedInt(iRoomTemp3, 0);
                        }
                        if (keyDown & ENTKEY_MASK) {
                            consumeKey();
                            dispSubState = SUB_SECOND;
                        }
                        break;
                    case SUB_SECOND:
                        checkAndInrDcrSignedChar(&SetPoint.cCCh3, C_MAX, C_MIN);
                        if (bToggleBitSlow) {
                            statusByte0 |= 0x08;
                            display3SignedInt(SetPoint.cCCh3, 0);
                        } else {
                            display3SignedInt(iRoomTemp3, 0);
                        }
                        if (keyDown & ENTKEY_MASK) {
                            consumeKey();
                            dispSubState = SUB_FIRST;
                        }
                        break;
                    default:
                        dispSubState = SUB_FIRST;
                        break;
                }
                if (keyDown & PROKEY_MASK) {
                    consumeKey();
                    dispMainState = DISP_MAIN;
                    dispSubState = SUB_FIRST;
                    persistSetpoints();
                }
                break;

            default:
                dispMainState = DISP_MAIN;
                break;
        }

        LED1_NORMAL_LAT = (isTestMode || (statusByte1 & 0x04)) ? 1 : 0;
        LED1_HI_LAT = (isTestMode || ((statusByte1 & 0x01) && bToggleBitSlow)) ? 1 : 0;
        LED1_LO_LAT = (isTestMode || ((statusByte1 & 0x02) && bToggleBitSlow)) ? 1 : 0;
        bool relay1 = outputLatch1 && isSound && bToggleBitSlow;

        LED2_NORMAL_LAT = (isTestMode || (statusByte2 & 0x04)) ? 1 : 0;
        LED2_HI_LAT = (isTestMode || ((statusByte2 & 0x01) && bToggleBitSlow)) ? 1 : 0;
        LED2_LO_LAT = (isTestMode || ((statusByte2 & 0x02) && bToggleBitSlow)) ? 1 : 0;
        bool relay2 = outputLatch2 && isSound && bToggleBitSlow;

        LED3_NORMAL_LAT = (isTestMode || (statusByte3 & 0x04)) ? 1 : 0;
        LED3_HI_LAT = (isTestMode || ((statusByte3 & 0x01) && bToggleBitSlow)) ? 1 : 0;
        LED3_LO_LAT = (isTestMode || ((statusByte3 & 0x02) && bToggleBitSlow)) ? 1 : 0;
        bool relay3 = outputLatch3 && isSound && bToggleBitSlow;

        RELAY_LAT = (relay1 || relay2 || relay3) ? 1 : 0;

        statusByte0 |= statusByte0 << 4;
        directAssign(statusByte0, 3);
        display();
        ClrWdt();
    }
}
