#include "mbed.h"
#include "Encoder.h"
#include "string"
#include <iostream>
//#include "QEI.h"
#include "PwmIn.h"


//STM mbed bug: these macros are MISSING from stm32f3xx_hal_tim.h
#ifdef TARGET_STM32F3
#define __HAL_TIM_GET_COUNTER(__HANDLE__) ((__HANDLE__)->Instance->CNT)
#define __HAL_TIM_IS_TIM_COUNTING_DOWN(__HANDLE__)            (((__HANDLE__)->Instance->CR1 &(TIM_CR1_DIR)) == (TIM_CR1_DIR))
#endif


Serial pc(SERIAL_TX, SERIAL_RX);
//Schrittmotoren: --------------------------------------


DigitalOut  Step_M1(PC_9);
DigitalOut  DIR_M1(PC_8);
DigitalOut  Reset_M1(PA_12);

Ticker t1;

DigitalOut  Step_M2(PC_10);
DigitalOut  DIR_M2(PC_11);
DigitalOut  Reset_M2(PA_5);

Ticker t2;


DigitalOut  Step_M3(PB_1);
DigitalOut  DIR_M3(PB_15);
DigitalOut  Reset_M3(PA_6);

Ticker t3;


PwmOut      Servo1(PB_4);
PwmOut      Servo2(PB_5);
PwmOut      Servo3(PB_14);



/* Variables -----------------------------------------------------------------*/


#define max_Byte_read 12 // Anzahl Bytes für lesen (4= eine floatzahl)
#define max_Byte_write 4 // Anzahl Bytes für senden (4= eine floatzahl)


// PC-Schnittstelle: ++++++++++++++++++++++++++++++++++++++++++++
uint8_t input_key; // für Daten an PC senden gebraucht

union rrr {
    char buf_char[max_Byte_write];   // Buffer für Datenübergabe an serielle Schnittstelle als char
    float buf_fl[(max_Byte_write)/4];   //Buffer als float
} f_send_daten;  // Buffer für Daten

union rrr2 {
    char buf_char[max_Byte_read];   // Buffer für Datenübergabe an serielle Schnittstelle als char
    float buf_fl[max_Byte_read/4]; // Buffer als float
    uint32_t buf_uint32[max_Byte_read/4]; // Buffer als uint32
} f_empfange_daten;

float Eingangsliste[max_Byte_read/4]; // maximal 10 Fliesspunktzahlen können eingelesen werden.
float Ausgangsliste[max_Byte_write/4]; // max. 10 Fliesspunktzahlen absenden (genaue Anzahl in Funktion definiert).

char Transfer_vollst=0; // für Synchronisation der Datenübergabe zum PC
char Rx_index=0; // für Datenlesen vom PC

// Ende PC-Schnittstelle ++++++++++++++++++++++++++++++++++++++++++++


float Zielschritte_M1;
float Zielschritte_M2;
float Zielschritte_M3;


float Istschritte_M1;
float Istschritte_M2;
float Istschritte_M3;



float Dschritte_M1;
float Dschritte_M2;
float Dschritte_M3;


float T1_attach;
float T2_attach;
float T3_attach;


void stepM1()
{

    if (Istschritte_M1<Zielschritte_M1) {
        DIR_M1.write(1);          // vorwärts drehen
        Step_M1 = !Step_M1;
        if (Step_M1==1) {
            Istschritte_M1=Istschritte_M1+1.0f;
        }
    } else if (Istschritte_M1>Zielschritte_M1) {
        DIR_M1.write(0);        // rückwärts drehen
        Step_M1 = !Step_M1;
        if (Step_M1==1) {
            Istschritte_M1=Istschritte_M1-1.0f;
        }
    }
}



void stepM2()
{

    if (Istschritte_M2<Zielschritte_M2) {
        DIR_M2.write(1);          // vorwärts drehen
        Step_M2 = !Step_M2;
        if (Step_M2==1) {
            Istschritte_M2=Istschritte_M2+1.0f;
        }
    } else if (Istschritte_M2>Zielschritte_M2) {
        DIR_M2.write(0);        // rückwärts drehen
        Step_M2 = !Step_M2;
        if (Step_M2==1) {
            Istschritte_M2=Istschritte_M2-1.0f;
        }
    }
}

void stepM3()
{
    if (Istschritte_M3<Zielschritte_M3) {
        DIR_M3.write(1);          // vorwärts drehen
        Step_M3 = !Step_M3;
        if (Step_M3==1) {
            Istschritte_M3=Istschritte_M3+1.0f;
        }

    } else if (Istschritte_M3>Zielschritte_M3) {
        DIR_M3.write(0);        // rückwärts drehen
        Step_M3 = !Step_M3;
        if (Step_M3==1) {
            Istschritte_M3=Istschritte_M3-1.0f;
        }
    }
}






void callback_serPC()
{
    // Note: you need to actually read from the serial to clear the RX interrupt

    f_empfange_daten.buf_char[Rx_index++]=pc.getc(); //Rx_data[0];

    if (Rx_index>=max_Byte_read) { // wenn Anzahl erreicht, dann Index auf Null und Transfer vollst. melden
        Rx_index=0;
        Transfer_vollst=1;
    }

}








int main()
{
    Rx_index=0;
    pc.baud(115200);
    Transfer_vollst=0;

    pc.attach(&callback_serPC);



    DIR_M1.write(1);
    DIR_M2.write(1);
    DIR_M3.write(1);
   


    Zielschritte_M1=0.0f;
    Zielschritte_M2=0.0f;
    Zielschritte_M3=0.0f;
    

    Istschritte_M1=0.0f;
    Istschritte_M2=0.0f;
    Istschritte_M3=0.0f;
    


    Step_M1.write(0);  // definierter Zählanfang
    Step_M2.write(0);
    Step_M3.write(0);
   

    Reset_M1.write(1);
    Reset_M2.write(1);
    Reset_M3.write(1);
    

    T1_attach=1.0f/(6400.0f); // eine Drehung je sec
    t1.attach(&stepM1, T1_attach);
    t2.attach(&stepM2, T1_attach);
    t3.attach(&stepM3, T1_attach);
   


    Servo1.period_us(20000);
    Servo1.write(0.05f);

    Servo2.period_us(20000);
    Servo2.write(0.05f);

    Servo3.period_us(20000);
    Servo3.write(0.05f);



    /* Infinite Loop. */
    while (1) {
        /* Hauptschleife */


        if (Transfer_vollst==1) {//1

             
                                             

                    Servo1.write(   f_empfange_daten.buf_fl[0]);
                    Servo2.write(   f_empfange_daten.buf_fl[1]); 
                    Servo3.write(   f_empfange_daten.buf_fl[2]);

                    Ausgangsliste[0]=3.3333f;
                  

            f_send_daten.buf_fl[0]=Ausgangsliste[0]; // Daten aus Buffer holen
          



            int i;
            for(i=0; i<max_Byte_write; i++) {//3
                input_key=  f_send_daten.buf_char[i];
                pc.putc(input_key);
            }//3

            Transfer_vollst=0; // wird in  HAL_UART_RxCpltCallback(..), siehe main.c oben ausgewertet


        } //1 if Transfer_vollst
    }

    } //main-while



