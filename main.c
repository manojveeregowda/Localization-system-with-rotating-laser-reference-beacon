#include <stdint.h>
#include "TM4C123GH6PM.h"
#include "globalDefines.h"

#include "xbee.h"       // uses UART1
#include "uart0.h"      // uses UART0
#include "utility.h"    // helper functions
#include "robot.h"


#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <ctype.h>
#include <stdio.h>
#include <math.h>

char* numbers = "0123456789";
char* hex_numbers = "0123456789ABCDEF";

#define DELTA_ANGLE         1.44        // degrees/millisecond changes during rotation
#define TIME_PER_DEGREE     6.944445    // time/degree change during rotation


#define AMULTIPLIER      0.304725       // .36/1.1825//                     // increase 0.30479
#define BMULTIPLIER     0.30294// iNC 0.30295// DEC 0.30293

#define FORWARD             1           // rover wheel forward
#define BACKWARD            2           // rover wheel backward
#define STOP                3           // rover wheel stop

#define AX                  0          // x coordinate of beacon A
#define AY                  0           // y coordinate of beacon A

#define BX                  138        // x coordinate of beacon B
#define BY                  0           // y coordinate of beacon B

#define CX                  138        // x coordinate of beacon C
#define CY                  138        // y coordinate of beacon C

#define A_INIT              0           // initial pointing angle of A
#define B_INIT              90          // initial pointing angle of B
#define C_INIT              180         // initial pointing angle of C

#define F_A_THETA           1           // rotations/sec
#define F_B_THETA           1.2           // rotations/sec
#define F_C_THETA           1.5           // rotations/sec

#define OFFSET_A        -30
#define OFFSTE_C        -42


#define W_A_THETA           1182//1000/F_A_THETA           // frequency of beacon A
#define W_B_THETA           987//1000/F_B_THETA            // frequency of beacon B
#define W_C_THETA           792//1000/F_C_THETA            // frequency of beacon C


#define RATE_SAT            0           // Saturation of rate of change of  angles of beacon A,B,C

#define AEB                 8           // allowed error band
#define BEB                 8           // allowed error band
#define CEB                 8           // allowed error band


float beacon1 =0, beacon2=0,beacon3=0;
// input from pulse recovery (time)
uint64_t storedTable[1000];
// current time and previous time
float t =0, lastT=0;
// last 3 time's data of time and angle for beacon A when it hit rover
float aLastTime2 = 0, aLastTime1 = 0, aTime = 0, aLastEst2 = 0, aLastEst1 = 0, aTheta = A_INIT; ;
// last 3 time's data of time and angle for beacon B when it hit rover
float bLastTime2 = 0, bLastTime1 = 0, bTime = 0, bLastEst2 = 0, bLastEst1 = 0, bTheta = B_INIT;
// last 3 time's data of time and angle for beacon C when it hit rover
float cLastTime2 = 0, cLastTime1 = 0, cTime = 0, cLastEst2 = 0, cLastEst1 = 0, cTheta = C_INIT;
// rate of change of theta for A, B, C (while it hits rover)
float aRateL = 0, aRate = 0, bRateL = 0, bRate = 0, cRateL = 0, cRate = 0;
// last 2 positions of rover
float lastpositionLx = 0, lastpositionx = 0, lastpositionLy = 0, lastpositiony =0;
//estimated angle
float bEst = 0, cEst = 0, aEst = 0;
//current position of rover and for its calculation
float px = 0, py = 0, px1 = 0, py1 = 0, px2 = 0, py2 = 0;
// distance between beacons and helping variables
float h1 = 0,h2 = 0, PA = 0, PC = 0;
//target location, target's angle from rover and rover's rotation time
float tAngle=0, targetx =0 ,targety=0,delaAngle=0, rotaTime =0;
float cabTheta,bcaTheta;
//distance between beacons
float AC, AB,BC;
//for startup
float startTime=0, startupposition1x=0, startupposition1y=0, startupposition2x=0, startupposition2y=0, hAngle=0;
// beacons positions
uint16_t beaconAX, beaconAY,beaconBX,beaconBY,beaconCX,beaconCY;
// counting for beacon identification
uint8_t count=0;
// rotational direction of rover
uint8_t direction=0;
//rover wheel rotation
uint8_t rightWheel = STOP, leftWheel = STOP;
//check for some specific location
bool cond1 = false, cond2 = false, cond = false;
// confirmed signal from beacon A, B, C
bool beaconAhit = false, beaconBhit = false, beaconChit = false;
//for startup
bool startuprover=true, firstStartup=true;
//beacon availability
bool beaconAavailable=false,beaconBavailable=false,beaconCavailable=false;
//beacons identified
bool identifiedA = false, identifiedB = false, identifiedC =false;
// rover modes
bool rovergo=false, rotatting = false;

//testing
float tA[100];
float tB[100];
float tC[100];
uint8_t taCount=0,tbCount=0,tcCount=0;



extern int started;

// code starts here--------------------------------------------------------------------------------------------
float tandeg(float y)
{
    y = tan(y*6.283/360);    // radian = Degree*2*pi/360
    return y;
}
float atandeg(float y)
{
    y = ((atan(y))*360/(2*3.14));    // radian = Degree*2*pi/360
    return y;
}
float sindeg(float y)
{
    y = sin(y*6.283/360);
    return y;
}
float cosdeg(float y)
{
    y = cos(y*6.283/360);
    return y;
}

float mod(float angletheta, float myangle )
{
    int dummy;
    dummy=(int)(angletheta/myangle);
    return (angletheta-myangle*dummy);
}

// find angle between two points
float anglefinder( float tx, float ty, float p_x, float p_y)
{
    float dx,dy;
    float theta1;
    dx = tx - p_x;
    dy = ty - p_y;
    theta1 = abs(atandeg(dy/dx));
    if (dx < 0)
    {
        if (dy < 0)
            theta1 = 180 + theta1;
        else
            theta1 = 180 - theta1;
    }
    else
    {
        if (dy < 0)
            theta1 = 360 - theta1;
    }
    return theta1;
}



void initHW(){
    // 40 MHz
    //SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);

    SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | ( 4 << SYSCTL_RCC_SYSDIV_S ) |
            SYSCTL_RCC_USEPWMDIV | SYSCTL_RCC_PWMDIV_64; // PWM = sysclock / 2

    SYSCTL_GPIOHBCTL_R = 0;
    SYSCTL_RCGC2_R  |= SYSCTL_RCGC2_GPIOB | SYSCTL_RCGC2_GPIOE  | SYSCTL_RCGC2_GPIOF | SYSCTL_RCGC2_GPIOD | SYSCTL_RCGC2_GPIOC | SYSCTL_RCGC2_GPIOA;
    SYSCTL_RCGC1_R  |= SYSCTL_RCGC1_UART1 | SYSCTL_RCGC1_UART0;
    SYSCTL_RCGC0_R |= SYSCTL_RCGC0_PWM0; // Turn-on PWM0 module.
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1;       // turn-on timer

    __asm("    nop; nop; nop;");

    GPIO_PORTF_DIR_R |= 0b11111;
    GPIO_PORTF_DEN_R |= 0b11111;

    SYSCTL_RCGCACMP_R |= 0x01;
    GPIO_PORTC_AFSEL_R |= 0x80;
    GPIO_PORTC_AMSEL_R |= 0x80;
    GPIO_PORTC_DEN_R &= ~0x80;
    COMP_ACREFCTL_R = 0x200;
    COMP_ACCTL0_R = 0x404;
    COMP_ACINTEN_R |= 1;
    COMP_ACRIS_R |= 0x01;
    //NVIC_EN0_R &= ~(1 << (INT_COMP0 - 16));


    GPIO_PORTB_DEN_R    |= 3;
    GPIO_PORTB_DIR_R    |= 0b10;
    GPIO_PORTB_AFSEL_R  |= 3;
    GPIO_PORTB_PCTL_R |= GPIO_PCTL_PB0_U1RX | GPIO_PCTL_PB1_U1TX;

    UART1_CTL_R = 0;                                 // turn-off UART0 to allow safe programming
    UART1_CC_R = UART_CC_CS_SYSCLK;                  // use system clock (40 MHz)
    UART1_IBRD_R = 21;                               // r = 40 MHz / (Nx115.2kHz), set floor(r)=21, where N=16
    UART1_FBRD_R = 45;                               // round(fract(r)*64)=45
    //UART1_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN; // configure for 8N1 w/ 16-level FIFO
    UART1_LCRH_R = UART_LCRH_WLEN_8; // configure for 8N1 w/o FIFO
    UART1_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN; // enable TX, RX, and module

    UART1_IM_R = UART_IM_RXIM;
    NVIC_EN0_R |= 1 << (INT_UART1-16);

    // dont touch pls pls
    GPIO_PORTA_DEN_R    |= 0b11;
    GPIO_PORTA_AFSEL_R  |= 0b11;
    GPIO_PORTA_PCTL_R   |= GPIO_PCTL_PA1_U0TX | GPIO_PCTL_PA0_U0RX;

    UART0_CTL_R = 0;                                                // turn-off UART0 to allow safe programming
    UART0_CC_R = UART_CC_CS_SYSCLK;                                 // use system clock (40 MHz)
    UART0_IBRD_R = 21;                                              // r = 40 MHz / (Nx115.2kHz), set floor(r)=21, where N=16
    UART0_FBRD_R = 45;                                              // round(fract(r)*64)=45
    UART0_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN;                // configure for 8N1 w/ 16-level FIFO
    UART0_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN;    // enable TX, RX, and module


    // Configure Timer 1 as the time base
    TIMER1_CTL_R &= ~TIMER_CTL_TAEN;      // turn-off timer before reconfiguring
    TIMER1_CFG_R = TIMER_CFG_32_BIT_TIMER;    // configure as 32-bit timer (A+B)
    TIMER1_TAMR_R = TIMER_TAMR_TAMR_PERIOD | TIMER_TAMR_TACDIR; // configure for periodic mode (count up)
    TIMER1_TAV_R = 0;



    /*********************************************************************************/
    // Configure the four PWM drive lines.
    GPIO_PORTB_DIR_R   |= 0x60;   // Make bits 5 and 6 outputs
    GPIO_PORTB_DR2R_R  |= 0x60;  // Set drive strength to 2mA
    GPIO_PORTB_DEN_R   |= 0x60;   // Enable bits 5 and 6 for digital
    GPIO_PORTB_AFSEL_R |= 0x60; // Select auxilary function (PWM) for bits 5 and 6

    // Enable PWM on bits 5 and 6.
    GPIO_PORTB_PCTL_R |= GPIO_PCTL_PB5_M0PWM3 | GPIO_PCTL_PB6_M0PWM0;

    GPIO_PORTE_DIR_R   |= 0x30;   // Make bits 4 and 5 outputs
    GPIO_PORTE_DR2R_R  |= 0x30;  // Set drive strength to 2mA
    GPIO_PORTE_DEN_R   |= 0x30;   // Enable bits 4 and 5 for digital
    GPIO_PORTE_AFSEL_R |= 0x30; // Select auxilary function for bits 4 and 5
    // Enable PWM on bits 4 and 5.
    GPIO_PORTE_PCTL_R |= GPIO_PCTL_PE4_M0PWM4 | GPIO_PCTL_PE5_M0PWM5;

    // Configure the PWM generators to drive the motor lines.

    __asm(" NOP"); // Wait 3 clocks+ for safe hardware config (see data sheet).
    __asm(" NOP");
    __asm(" NOP");
    wait_microsecond( 100 );

    SYSCTL_SRPWM_R = SYSCTL_SRPWM_R0;   // Reset PWM modules.
    SYSCTL_SRPWM_R = 0;                 // Leave reset state.
    PWM0_0_CTL_R = 0;                   // Turn off PWM0 generators.
    PWM0_1_CTL_R = 0;
    PWM0_2_CTL_R = 0;
    PWM0_0_GENA_R = PWM_0_GENA_ACTCMPAD_ZERO | PWM_0_GENA_ACTLOAD_ONE; // output 0 on PWM0, gen 1a, cmpa
    PWM0_1_GENB_R = PWM_0_GENB_ACTCMPBD_ZERO | PWM_0_GENB_ACTLOAD_ONE; // output 3 on PWM0, gen 1b, cmpb
    PWM0_2_GENA_R = PWM_0_GENA_ACTCMPAD_ZERO | PWM_0_GENA_ACTLOAD_ONE; // output 4 on PWM0, gen 2a, cmpa
    PWM0_2_GENB_R = PWM_0_GENB_ACTCMPBD_ZERO | PWM_0_GENB_ACTLOAD_ONE;// output 5 on PWM0, gen 2b, cmpb
    PWM0_0_LOAD_R = 1024; // Set period to 40 MHz sys clock / 64 = 625000; 625000 / 1024 = 610.351 kHz.
    PWM0_1_LOAD_R = 1024;
    PWM0_2_LOAD_R = 1024;

    // Invert outputs for duty cycle increases with increasing compare values.
    PWM0_INVERT_R = PWM_INVERT_PWM3INV | PWM_INVERT_PWM4INV | PWM_INVERT_PWM5INV | PWM_INVERT_PWM0INV;
    MOTOR_PIN_1 = 0; // Turn off all generators by default (0=always low, 1023=always high).
    MOTOR_PIN_2 = 0;
    MOTOR_PIN_3 = 0;
    MOTOR_PIN_4 = 0;

    PWM0_0_CTL_R = PWM_0_CTL_ENABLE; // Turn on the PWM generators now configured.
    PWM0_1_CTL_R = PWM_0_CTL_ENABLE;
    PWM0_2_CTL_R = PWM_0_CTL_ENABLE;
    PWM0_ENABLE_R = PWM_ENABLE_PWM3EN | PWM_ENABLE_PWM4EN | PWM_ENABLE_PWM5EN | PWM_ENABLE_PWM0EN;

    NVIC_EN0_R |= 1 << (INT_COMP0 - 16);
}

uint32_t j = 0;
uint32_t index = 0;
void compIsr(){
    if( started ){
        storedTable[j] = TIMER1_TAV_R / 40000;
        j++;
        //if( j >= 100 ){ j = 0; }
    }
    RED_LED ^= 1;
    COMP_ACMIS_R |= 0x01; //turn off interrupt
    NVIC_EN0_R &= ~(1 << (INT_COMP0 - 16)); // disable interrupt

}


void location( float myTable ){
   /* if (count ==2)
    {
        beacon3 = myTable;
        count=3;
    }
*/
    if (count ==1)
    {
        beacon2=myTable;
        count=2;
    }

    if (count ==0)
    {
        beacon1= myTable;
        count=1;
    }

    if (count>1 && (!beaconAavailable ||  !beaconCavailable ))  // !beaconBavailable ||
    {
        if (((myTable <= (beacon1 +W_A_THETA + AEB)) && (myTable >= (beacon1 +W_A_THETA - AEB)))
                || ((myTable <= (beacon2 +W_A_THETA + AEB)) && (myTable >= (beacon2 +W_A_THETA - AEB)))
                || ((myTable <= (beacon3 +W_A_THETA + AEB)) && (myTable >= (beacon3 +W_A_THETA - AEB)))) // assign
        {
            aTime= myTable;

            identifiedA =true;


            tA[taCount]=aTime;
            taCount++;
        }
        if (((myTable <= (beacon1 +W_B_THETA + BEB)) && (myTable >= (beacon1 +W_B_THETA - BEB)))
                || ((myTable <= (beacon2 +W_B_THETA + BEB)) && (myTable >= (beacon2 +W_B_THETA - BEB)))
                || ((myTable <= (beacon3 +W_B_THETA + BEB)) && (myTable >= (beacon3 +W_B_THETA - BEB)))) // assign
        {
            bTime= myTable;
            identifiedB =true;

            tB[tbCount]=bTime;
            tbCount++;
        }
        if (((myTable <= (beacon1 +W_C_THETA + CEB)) && (myTable >= (beacon1 +W_C_THETA - CEB)))
                || ((myTable <= (beacon2 +W_C_THETA + CEB)) && (myTable >= (beacon2 +W_C_THETA - CEB)))
                || ((myTable <= (beacon3 +W_C_THETA + CEB)) && (myTable >= (beacon3 +W_C_THETA - CEB)))) // assign
        {
            cTime= myTable;
            identifiedC =true;

            tC[tcCount]=cTime;
            tcCount++;
        }

        if (identifiedA)
        {

            if ((myTable <= (aTime + W_A_THETA +AEB)) && (myTable >= (aTime + W_A_THETA - AEB)))
            {
                aLastTime1 = aTime;
                aTime= myTable;
                aLastEst1 = mod(OFFSET_A+A_INIT + aLastTime1 * AMULTIPLIER * F_A_THETA,360);
                aTheta = mod(OFFSET_A+A_INIT + aTime * AMULTIPLIER * F_A_THETA,360);
                aRate = (aTheta - aLastEst1) / (aTime - aLastTime1);
                beaconAavailable =true;


                tA[taCount]=aTime;
                            taCount++;

            }
        }

        if (identifiedB)
        {

            if ((myTable <= (bTime + W_B_THETA +BEB)) && (myTable >= (bTime + W_B_THETA - BEB)))
            {
                bLastTime1 = bTime;
                bTime= myTable;
                bLastEst1 = mod(B_INIT + bLastTime1 * 0.36 * F_B_THETA,360);
                bTheta = mod(B_INIT + bTime * 0.36 * F_B_THETA,360);
                bRate = (bTheta - bLastEst1) / (bTime - bLastTime1);
                beaconBavailable =true;


                tB[tbCount]=bTime;
                tbCount++;

            }
        }
        if (identifiedC)
        {
            if ((myTable <= (cTime + W_C_THETA +CEB)) && (myTable >= (cTime + W_C_THETA - CEB)))
            {
                cLastTime1 = cTime;
                cTime= myTable;
                cLastEst1 = mod(OFFSTE_C+C_INIT + cLastTime1 * BMULTIPLIER* F_C_THETA,360);
                cTheta = mod(OFFSTE_C +C_INIT + cTime * BMULTIPLIER * F_C_THETA,360);
                cRate = (cTheta - cLastEst1) / (cTime - cLastTime1);
                beaconCavailable =true;


                tC[tcCount]=cTime;
                tcCount++;

            }
        }
    }







    if (beaconAavailable && beaconCavailable) //beaconBavailable &&
    {
        if ((myTable <= (aTime + W_A_THETA +AEB)) && (myTable >= (aTime + W_A_THETA - AEB)))
        {
            aLastTime2 = aLastTime1;
            aLastTime1 = aTime;
            aTime= myTable;
            aLastEst2 = aLastEst1;
            aLastEst1 = aTheta;
            aTheta = mod(OFFSET_A +A_INIT + aTime*AMULTIPLIER*F_A_THETA,360);

            aRateL = aRate;
            aRate = (aTheta - aLastEst1) / (aTime - aLastTime1);

            if (abs(aRate - aRateL) > RATE_SAT)
                aRate = (aTheta - aLastEst2) / (aTime - aLastTime2);

            tA[taCount]=aTime;
                        taCount++;

            beaconAhit =true;



        }


        if ((myTable <= (bTime + W_B_THETA +BEB)) && (myTable >= (bTime + W_B_THETA - BEB)))
        {
            bLastTime2 = bLastTime1;
            bLastTime1 = bTime;
            bTime = myTable;
            bLastEst2 = bLastEst1;
            bLastEst1 = bTheta;
            bTheta = mod(B_INIT + bTime * 0.36 * F_B_THETA,360);

            bRateL = bRate;
            bRate = (bTheta - bLastEst1) / (bTime - bLastTime1);
            if (abs(bRate - bRateL) > RATE_SAT)
                bRate = (bTheta - bLastEst2) / (bTime - bLastTime2);

            beaconBhit =true;


            tB[tbCount]=bTime;
            tbCount++;
        }

        if ((myTable <= (cTime + W_C_THETA +CEB)) && (myTable >= (cTime + W_C_THETA - CEB)))
        {
            cLastTime2 = cLastTime1;
            cLastTime1 = cTime;
            cTime = myTable;
            cLastEst2 = cLastEst1;
            cLastEst1 = cTheta;
            cTheta =mod(OFFSTE_C +C_INIT + cTime * BMULTIPLIER * F_C_THETA,360);
            cRateL = cRate;
            cRate = (cTheta - cLastEst1) / (cTime - cLastTime1);
            if (abs(cRate - cRateL) > RATE_SAT)
                cRate = (cTheta - cLastEst2) / (cTime - cLastTime2);


            beaconChit =true;



            tC[tcCount]=cTime;
            tcCount++;

        }

    }







    lastT = t;
    if (beaconAhit)
    {
        beaconAhit=false;

        bEst = mod((bTheta + bRate * (aTime - bTime)), 360);
        cEst = mod((cTheta + cRate * (aTime - cTime)),  360);

        if ((!((bEst <= 0.001 && bEst >= -0.001)
                    || (bEst <= 180 + 0.001 && bEst >= 180 - 0.001)
                    || (aTheta <= 0.001 && aTheta >= -0.001)
                    || (aTheta <= 180 + 0.001 && aTheta >= 180 - 0.001))) && beaconBavailable)
        {
            h1 = AB / (1 + (tandeg(aTheta) / tandeg(180 - bEst)));
            px1 = AX + h1;
            py1 = AY + h1 * tandeg(aTheta);
            cond = true;
        }
        if ((!(((aTheta <= cabTheta + 0.001) && (aTheta >= cabTheta - 0.001))
                    || ((aTheta <= bcaTheta + 0.001) && (aTheta >= bcaTheta - 0.001))
                    || ((cEst <= cabTheta + 0.001) && (cEst >= cabTheta - 0.001))
                    || ((cEst <= bcaTheta + 0.001) && (cEst >= bcaTheta - 0.001)))) && beaconCavailable)
        {
            h2 = (AC/ (1 + ((tandeg(360 + cabTheta - aTheta))/ tandeg(cEst - bcaTheta))));
            PA = abs(h2 / cosdeg(cabTheta - aTheta));
            px2 = AX + PA * cosdeg(aTheta);
            py2 = AY + PA * sindeg(aTheta);
            cond2 = true;
        }
        if (cond)
        {
            if (cond2)
            {
                px = (px1 + px2) / 2;
                py = (py1 + py2) / 2;
            }
            else
            {
                px = px1;
                py = py1;
            }
            cond = false;
            cond2 = false;
        }
        else if (cond2)
        {
            px = px2;
            py = py2;
            cond2 = false;
        }

        if (startuprover)
        {
            if (firstStartup)
            {
                startTime = t;
                firstStartup = false;
                startupposition1x = px;
                startupposition1y = py;
            }
            rovergo = true;
            if ((t-startTime)>2000)
            {
                startuprover=false;
                rovergo=false;
                startupposition2x = px;
                startupposition2y = py;
                hAngle= anglefinder(startupposition2x,startupposition2y,
                                     startupposition1x, startupposition1y);
                tAngle= anglefinder(targetx,targety,startupposition2x,startupposition2y);
                delaAngle= hAngle- tAngle;
                if (delaAngle > 180)
                    delaAngle= delaAngle-360;
                if (delaAngle < -180)
                    delaAngle = delaAngle +360;
                if (abs(delaAngle) > 5)
                {
                    rotatting=true;
                    rovergo=false;
                    rotaTime=TIME_PER_DEGREE *abs(delaAngle);
                    if (delaAngle < 0)
                        direction =1;
                    else
                        direction =-1;
                }
            }
        }
    }
    if (beaconBhit)
    {
        beaconBhit = false;


        aEst = mod((aTheta + aRate * (bTime - aTime)),360);
        cEst = mod((cTheta + cRate * (bTime - cTime)),360);

        if ((!((bTheta <= 90 + 0.001 && bTheta >= 90 - 0.001)
                        || (bTheta <= 270 + 0.001 && bTheta >= 270 - 0.001)
                        || (cEst <= 90 + 0.001 && cEst >= 90 - 0.001)
                        || (cEst <= 270 + 0.001 && cEst >= 270 - 0.001))) && beaconCavailable)
        {
            h1 = BC / (1 + (tandeg(bTheta - 90) / tandeg(270 - cEst)));
            py1 = BY + h1;
            px1 = BX - h1 * tandeg(bTheta - 90);
            cond = true;
        }
        if ((!((bTheta <= 0.001 && bTheta >= -0.001)
                        || (bTheta <= 180 + 0.001 && bTheta >= 180 - 0.001)
                        || (aEst <= 0.001 && aEst >= -0.001)
                        || (aEst <= 180 + 0.001 && aEst >= 180 - 0.001))) && beaconAavailable)
        {
            h2 = AB / (1 + (tandeg(180 - bTheta) / tandeg(aEst)));
            px2 = BX - h2;
            py2 = BY + h2 * tandeg(180 - bTheta);
            cond2 = true;
        }

        if (cond)
        {
            if (cond2)
            {
                px = (px1 + px2) / 2;
                py = (py1 + py2) / 2;
            }
            else
            {
                px = px1;
                py = py1;
            }
            cond = false;
            cond2 = false;
        }
        else if (cond2)
        {
            px = px2;
            py = py2;
            cond2 = false;
        }

        if (startuprover)
        {
            if (firstStartup)
            {
                startTime = t;
                firstStartup = false;
                startupposition1x = px;
                startupposition1y = py;
            }
            rovergo = true;
            if ((t-startTime)>2000)
            {
                startuprover=false;
                rovergo=false;
                startupposition2x = px;
                startupposition2y = py;
                hAngle= anglefinder(startupposition2x,startupposition2y,
                                     startupposition1x, startupposition1y);
                tAngle= anglefinder(targetx,targety,startupposition2x,startupposition2y);
                delaAngle= hAngle- tAngle;
                if (delaAngle > 180)
                    delaAngle= delaAngle-360;
                if (delaAngle < -180)
                    delaAngle = delaAngle +360;
                if (abs(delaAngle) > 5)
                {
                    rotatting=true;
                    rovergo=false;
                    rotaTime=TIME_PER_DEGREE *abs(delaAngle);
                    if (delaAngle < 0)
                        direction =1;
                    else
                        direction =-1;
                }
            }
        }
    }

    if (beaconChit)
    {
        beaconChit =false;

        bEst = mod((bTheta + bRate * (cTime - bTime)),360);
        aEst = mod((aTheta + aRate * (cTime - aTime)),360);

        if ((!((bEst <= 90 + 0.001 && bEst >= 90 - 0.001)
                        || (bEst <= 270 + 0.001 && bEst >= 270 - 0.001)
                        || (cTheta <= 90 + 0.001 && cTheta >= 90 - 0.001)
                        || (cTheta <= 270 + 0.001 && cTheta >= 270 - 0.001))) && beaconBavailable)
        {
            h1 = BC / (1 + (tandeg(270 - cTheta) / tandeg(bEst - 90)));
            py1 = CY - h1;
            px1 = CY - h1 * tandeg(270 - cTheta);
            cond = true;
        }
        if ((!(((aEst <= cabTheta + 0.001) && (aEst >= cabTheta - 0.001))
                        || ((aEst <= bcaTheta + 0.001) && (aEst >= bcaTheta - 0.001))
                        || ((cTheta <= cabTheta + 0.001) && (cTheta >= cabTheta - 0.001))
                        || ((cTheta <= bcaTheta + 0.001) && (cTheta >= bcaTheta - 0.001)))) && beaconAavailable)
        {
            h2 = (AC/ (1+ ((tandeg(cTheta - bcaTheta))/ tandeg(360 + cabTheta - aEst))));
            PC = abs(h2 / cosdeg(cTheta - bcaTheta));
            px2 = CX + PC * cosdeg(cTheta);
            py2 = CY + PC * sindeg(cTheta);
            cond2 = true;
        }
        if (cond)
        {
            if (cond2)
            {
                px = (px1 + px2) / 2;
                py = (py1 + py2) / 2;
            }
            else
            {
                px = px1;
                py = py1;
            }
            cond = false;
            cond2 = false;
        }
        else if (cond2)
        {
            px = px2;
            py = py2;
            cond2 = false;
        }
        else
        {
            rovergo=false;
            rotatting =false;
        }

        if (startuprover)
        {
            if (firstStartup)
            {
                startTime = t;
                firstStartup = false;
                startupposition1x = px;
                startupposition1y = py;
            }
            rovergo = true;
            if ((t-startTime)>2000)
            {
                startuprover=false;
                rovergo=false;
                startupposition2x = px;
                startupposition2y = py;
                hAngle= anglefinder(startupposition2x,startupposition2y,
                                     startupposition1x, startupposition1y);
                tAngle= anglefinder(targetx,targety,startupposition2x,startupposition2y);
                delaAngle= hAngle- tAngle;
                if (delaAngle > 180)
                    delaAngle= delaAngle-360;
                if (delaAngle < -180)
                    delaAngle = delaAngle +360;
                if (abs(delaAngle) > 5)
                {
                    rotatting=true;
                    rovergo=false;
                    rotaTime=TIME_PER_DEGREE *abs(delaAngle);
                    if (delaAngle < 0)
                        direction =1;
                    else
                        direction =-1;
                }
            }
        }
    }

    if ((abs(px-targetx)<=5 )&& (abs(py-targety)<=5))
    {
        rotatting=false;
        rovergo=false;
        rightWheel = STOP;
        leftWheel  = STOP;
    }
    if (rotatting)
    {
        hAngle = hAngle + direction * DELTA_ANGLE*(t - lastT);
        rotaTime = rotaTime - (t - lastT);
        if (rotaTime < 0)
        {
            rotaTime = false;
            rotatting = false;
            rovergo = true;
            direction=0;
        }
        if(direction==1)
        {
            rightWheel= FORWARD;
            leftWheel = BACKWARD;
        }
        else
        {
            rightWheel = BACKWARD;
            leftWheel  = FORWARD;
        }

    }

    if (rovergo)
    {
        rightWheel = FORWARD;
        leftWheel  = FORWARD;


    }
}

void reset(){
    __asm("    .global _c_int00\n"
          "    b.w     _c_int00");
}
void put_i_UART1( int in ){
    char out[20];
    int i = 0;

    if(in == 0){
        putcUart1('0');
    }else{
        int rem;
        while(in != 0 && i < 20){
            rem = in % 10;
            out[i] = numbers[rem];
            i++;
            in = in / 10;
        }
    }
    for(i = i-1; i >= 0; i--){
        putcUart1(out[i]);
    }
}

int main(void){
    beaconAX=AX;
    beaconAY=AY;
    beaconBX=BX;
    beaconBY=BY;
    beaconCX=CX;
    beaconCY=CY;

    AC = sqrt(((beaconCX - beaconAX)*(beaconCX - beaconAX)) + ((beaconCY - beaconAY)*(beaconCY - beaconAY)));
    AB = sqrt(((beaconAX - beaconBX)*(beaconAX - beaconBX)) + ((beaconAY - beaconBY)*(beaconAY - beaconBY)));
    BC = sqrt(((beaconCX - beaconBX)*(beaconCX - beaconBX)) + ((beaconCY - beaconBY)*(beaconCY - beaconBY)));

    cabTheta = atandeg((beaconCY - beaconAY) / (beaconCX - beaconAX));        // angle CAB
    bcaTheta = 180 + cabTheta;                      // angle BCA



    initHW();
    initHW_rf();

    motorControl(STOP,STOP);
    char temp[20];
    char c;
    while(1){
        if( started == 1 ){ TIMER1_CTL_R |= TIMER_CTL_TAEN; }
        NVIC_EN0_R |= 1 << (INT_COMP0 - 16);                    // enable interrupt

        if( index < j && started==1){
            location( storedTable[index] );

            if( index == 10 ){
                put_s_UART1( "x: " );
                //py = 100;
                put_i_UART1( (int)(px * 100) );
                put_s_UART1( "  y: " );
                put_i_UART1( (int)(py * 100) );
                put_s_UART1( "\n\r: " );
                //break;
                reset();

            }

            //motorControl( leftWheel, rightWheel );
            index++;
        }
        if( index == 500 ){break;}
        //index = index % 100;
    }

    return 0;
}

