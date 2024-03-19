/** @file sys_main.c 
*   @brief Application main file
*   @date 11-Dec-2018
*   @version 04.07.01
*
*   This file contains an empty main function,
*   which can be used for the application.
*/

/* 
* Copyright (C) 2009-2018 Texas Instruments Incorporated - www.ti.com 
* 
* 
*  Redistribution and use in source and binary forms, with or without 
*  modification, are permitted provided that the following conditions 
*  are met:
*
*    Redistributions of source code must retain the above copyright 
*    notice, this list of conditions and the following disclaimer.
*
*    Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the 
*    documentation and/or other materials provided with the   
*    distribution.
*
*    Neither the name of Texas Instruments Incorporated nor the names of
*    its contributors may be used to endorse or promote products derived
*    from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
*  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
*  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
*  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
*  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
*  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
*  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*/


/* USER CODE BEGIN (0) */
#include "stdio.h"
#include "stdlib.h"

#include "het.h"
#include "gio.h"
#include "sci.h"
#include "rti.h"
/* USER CODE END */

/* Include Files */

#include "sys_common.h"

/* USER CODE BEGIN (1) */

// |---> [Prototipos de funciones para comunicacion UART]
void inicializacion_puertos(void);
//////////////////////////////////////////////////////////

// |---> [Prototipos de funcion para interrupcion por flancos PWM]
void edgeNotification(hetBASE_t * hetREG,uint32 edge);
/////////////////////////////////////////////////////////////////


// |---> [Prototipos de funciones para comunicacion UART]
bool sciEnviarDatos(uint8 numOfDat, char* charDat, bool sc);
//////////////////////////////////////////////////////////


void Control_Motor1 (void);
void Control_Motor2 (void);
void Control_Motor3 (void);
/* USER CODE END */

/** @fn void main(void)
*   @brief Application main function
*   @note This function is empty by default.
*
*   This function is called after startup.
*   The user can use this function to implement the application.
*/

/* USER CODE BEGIN (2) */

/******** Cable Pololu 131:1 Metal Gearmotor 37Dx73L mm 12V with 64 CPR Encoder *********
Red     ---> Motor power
Black   ---> Motor power
Green   ---> Encoder ground
Blue    ---> Encoder Vcc (3.5 to 20 V)
Yellow  ---> Encoder A output
White   ---> Encoder B output
******************************************************************************************/

enum {
    cond_1=0,
    cond_2,
    cond_3,
    cond_4
};

enum{
    LOW=0,
    HIGH,
};

enum{
    Sin_detectar=0,
    Sen_horario,
    Sen_antihorario
};

float T=0.005;
int Timer_1ms=0, giro=0;
float conv= 0.04285714286;


//          =====Motor 1=====
hetSIGNAL_t PWM0_Het1_10_M1;//RPWM
hetSIGNAL_t PWM1_Het1_28_M1;//LPWM

#define PORT_Canal_A_M1 hetPORT1
#define PORT_Canal_B_M1 hetPORT1
#define BIT_Canal_A_M1 8  //Canal A Het1[8]
#define BIT_Canal_B_M1 23 //Canal B Het1[23]
#define Edge_Canal_A_M1 0
#define Edge_Canal_B_M1 1
uint8_t Cond_Izq_M1[4]= {false, false, false,false};
uint8_t Cond_Der_M1[4]= {false, false,false,false};

//Variables de Control Motor 1
float kp_M1=2.94 ,ki_M1=0.04, kd_M1=0.012;// acc_ki=0.1;
//float kp_M1=2.94 ki_M1=,   kd_M1=    Better configuration

float e_k_M1=0.0, e_k_1_M1=0.0, e_k_2_M1=0.0, aux_e_k_1_M1=0.0;
float acc_ki_M1=0.0;//, Integrator1_M3=0.0, Integrator2_M3=0.0;
float Ref_deg_M1=90.0;
float Control_k_M1=0.0, Control_k_1_M1=0.0;
float C1_M1=0.0, C2_M1=0.0,   C3_M1=0.0;
int Int_Control_M1=0;
float Aux_Control_M1=0.0;
float Pos_Deg_M1=0.00;
int Sentido_M1=0, Cont_Enc_M1=2100;

/*----> ===LIMITES M1===
 * Limite inferior 60 grados.
 * Limites superior 120 grados.
*/

//          ====Motor 2====
hetSIGNAL_t PWM2_Het1_18_M2;//RPWM
hetSIGNAL_t PWM3_Het1_20_M2;//LPWM

#define PORT_Canal_A_M2 hetPORT1
#define PORT_Canal_B_M2 hetPORT1
#define BIT_Canal_A_M2 16 // Canal A Het1[16]
#define BIT_Canal_B_M2 30 // Canal B Het1[30]
#define Edge_Canal_A_M2 2
#define Edge_Canal_B_M2 3
uint8_t Cond_Izq_M2[4]= {false, false, false,false};
uint8_t Cond_Der_M2[4]= {false, false,false,false};

//Variables de Control Motor 2
float kp_M2=2.94 ,ki_M2=0.0, kd_M2=0.0;// acc_ki=0.1;
//float kp_M1=4.25-----> 2.94 (segundo control),ki_M1=0.1, kd_M1=0.009, k_PID_M1=0.09, acc_ki=0.1;  Better configuration
float e_k_M2=0.0, e_k_1_M2=0.0, e_k_2_M2=0.0, aux_e_k_1_M2=0.0;
float acc_ki_M2=0.0;
float Ref_deg_M2=90.0;
float Control_k_M2=0.0, Control_k_1_M2=0.0;
float C1_M2=0.0, C2_M2=0.0,   C3_M2=0.0;
int Int_Control_M2=0;
float Aux_Control_M2=0.0;
float Pos_Deg_M2=0.00;
int Sentido_M2=0, Cont_Enc_M2=2100;


//          ====Motor 3====
hetSIGNAL_t PWM4_Het1_22_M3;//RPWM
hetSIGNAL_t PWM5_Het1_25_M3;//LPWM

#define PORT_Canal_A_M3 hetPORT1
#define PORT_Canal_B_M3 hetPORT1
#define BIT_Canal_A_M3 12 // Canal A Het1[12]
#define BIT_Canal_B_M3 14 // Canal B Het1[14]
#define Edge_Canal_A_M3 4
#define Edge_Canal_B_M3 5
uint8_t Cond_Izq_M3[4]= {false, false, false,false};
uint8_t Cond_Der_M3[4]= {false, false,false,false};

//Variables de Control Motor 3
float kp_M3=2.94 ,ki_M3=0.0, kd_M3=0.0;// acc_ki=0.1;
//float kp_M1=4.25-----> 2.94 (segundo control),ki_M1=0.1, kd_M1=0.009, k_PID_M1=0.09, acc_ki=0.1;  Better configuration
float e_k_M3=0.0, e_k_1_M3=0.0, e_k_2_M3=0.0, aux_e_k_1_M3=0.0;
float acc_ki_M3=0.0;
float Ref_deg_M3=90.0;
float Control_k_M3=0.0, Control_k_1_M3=0.0;
int Int_Control_M3=0;
float Aux_Control_M3=0.0;
float Pos_Deg_M3=0.00;
int Sentido_M3=0, Cont_Enc_M3=2100;


// |---> [Variables para comunicacion UART]
unsigned char receivedData[3];
char command[200];
//////////////////////////////////////////


/*   ==============================================================
 * Delta_robot_1.m
 * linea 77 modificar
 *
     ============================================================== */
/* USER CODE END */

int main(void)
{
/* USER CODE BEGIN (3) */
    inicializacion_puertos();
    //_enable_interrupt_();

    // |---> [Inicializacion del TIMER]
    rtiEnableNotification(rtiNOTIFICATION_COMPARE0);
    rtiStartCounter(rtiCOUNTER_BLOCK0);
    _enable_IRQ();
    _enable_interrupt_();
    // <---| [Inicializacion del TIMER]

    sciReceive(scilinREG, 0x01, receivedData);
   // edgeEnableNotification(hetREG1, BIT_Canal_A_M1);
    //edgeEnableNotification(hetREG1, BIT_Canal_B_M1);

//  PWM0_Het1_20_Motor1.period= 2E3; // 2 Milisegundos  500 Hz
//  PWM0_Het1_20_Motor1.duty= Duty_Motor1;

//              === Motor 1 ====
   PWM0_Het1_10_M1.period=2E3; //RPWM
   PWM1_Het1_28_M1.period=2E3; //LPWM

//              === Motor 2 ====
    PWM2_Het1_18_M2.period=2E3;//RPWM
    PWM3_Het1_20_M2.period=2E3;//LPWM

//              === Motor 3 ====
    PWM4_Het1_22_M3.period=2E3;//RPWM
    PWM5_Het1_25_M3.period=2E3;//LPWM

    while(1){

        if (Timer_1ms>=5){   // Cada 4 ms se inicia el control
            //Control_Motor1();
//          Control_Motor2();
            Control_Motor3();
            Timer_1ms=0;
        }

//      if (Sentido_M3== Sen_horario ){
//          sciEnviarDatos( sprintf(command, "girar a la derecha\n"),command,true);
//          sciEnviarDatos( sprintf(command, "contador M3: %d\n", Cont_Enc_M3),command,true);
//      }
//      else if (Sentido_M3== Sen_antihorario){
//          sciEnviarDatos( sprintf(command, "girar a la izquierda"),command,true);
//          sciEnviarDatos( sprintf(command, "contador M3: %d\n", Cont_Enc_M3),command,true);
//      }
//      else {
//          sciEnviarDatos( sprintf(command, "No girar "),command,true);
//          sciEnviarDatos( sprintf(command, "contador M3: %d\n", Cont_Enc_M3),command,true);
//      }
    }
/* USER CODE END */

    return 0;
}


/* USER CODE BEGIN (4) */
void inicializacion_puertos(void) {
    hetInit();
    gioInit();
    sciInit();
    rtiInit();

}


void Control_Motor1(void){

    kp_M1;ki_M1;kd_M1;

    if (Ref_deg_M1<70.0){
        Ref_deg_M1=70.0;
    }
    else if (Ref_deg_M1 > 150.0){
        Ref_deg_M1=150.0;
    }

    Pos_Deg_M1= Cont_Enc_M1*conv;   // couts *(360/8400)--> 8400 counts por revolucion
    e_k_M1=Ref_deg_M1-Pos_Deg_M1;   //calculo del Error actual e[k]
    acc_ki_M1 += e_k_M1*T;  //Acumulacion del integrador
    Control_k_M1= kp_M1*e_k_M1 + kd_M1* ( e_k_M1 - e_k_1_M1 ) / T + ki_M1*acc_ki_M1;
    Int_Control_M1= (int) Control_k_M1;

    if (Control_k_M1> 0.0){
        giro=1;
        if (Control_k_M1 <= 99.0){
        PWM0_Het1_10_M1.duty= Int_Control_M1;
        PWM1_Het1_28_M1.duty= 0;
        }
        else{                                           //Giro hacia la Derecha (horario)
            Int_Control_M1=99;
            PWM0_Het1_10_M1.duty= Int_Control_M1;
            PWM1_Het1_28_M1.duty= 0;
        }
    }

    else if (Control_k_M1 < 0.0){
        giro=2;
        Aux_Control_M1= Control_k_M1 * -1.0;
        Int_Control_M1= (int) Aux_Control_M1;
        if(Control_k_M1 >-99.0){
            PWM0_Het1_10_M1.duty= 0 ;                   //Giro hacia la Izquierda (antihorario)
            PWM1_Het1_28_M1.duty=  Int_Control_M1;
        }
        else {
            Int_Control_M1=99;
            PWM0_Het1_10_M1.duty= 0 ;
            PWM1_Het1_28_M1.duty= Int_Control_M1;
        }
    }
    else{
        giro=0;
        PWM0_Het1_10_M1.duty= 0;                        //Sin giro
        PWM1_Het1_28_M1.duty= 0;
    }
    pwmSetSignal(hetRAM1, pwm0, PWM0_Het1_10_M1);       //Se ajusta la salida del
    pwmSetSignal(hetRAM1, pwm1, PWM1_Het1_28_M1);       //control
    e_k_1_M1= e_k_M1;  //Se actualiza el error anterior e[k-1]=e[k] despues del calculo del control
}


void Control_Motor2(void){

    kp_M2;ki_M2;kd_M2;

    if (Ref_deg_M2<70.0){
        Ref_deg_M2=70.0;
    }
    else if (Ref_deg_M2 > 150.0){
        Ref_deg_M2=150.0;
    }
    Pos_Deg_M2= Cont_Enc_M2*conv;   // couts *(360/8400)--> 8400 counts por revolucion
    e_k_M2=Ref_deg_M2-Pos_Deg_M2;   //calculo del Error actual e[k]
    acc_ki_M2 += e_k_M2*T;  //Acumulacion del integrador
    Control_k_M2= kp_M2*e_k_M2 + kd_M2* ( e_k_M2 - e_k_1_M2 ) / T + ki_M2*acc_ki_M2;
    Int_Control_M2= (int) Control_k_M2;

    if (Control_k_M2>0.0){
        giro=1;
        if (Control_k_M2 <= 99.0){
        PWM2_Het1_18_M2.duty= Int_Control_M2;
        PWM3_Het1_20_M2.duty= 0;
        }
        else{                                           //Giro hacia la Derecha (horario)
            Int_Control_M2=99;
            PWM2_Het1_18_M2.duty= Int_Control_M2;
            PWM3_Het1_20_M2.duty= 0;
        }
    }

    else if (Control_k_M2 < 0.0){
        giro=2;
        Aux_Control_M2= Control_k_M2 * -1.0;
        Int_Control_M2= (int) Aux_Control_M2;
        if(Control_k_M2 >-99.0 ){
            PWM2_Het1_18_M2.duty= 0 ;                   //Giro hacia la Izquierda (antihorario)
            PWM3_Het1_20_M2.duty=  Int_Control_M2;
        }
        else {
            Int_Control_M2=99;
            PWM2_Het1_18_M2.duty= 0 ;
            PWM3_Het1_20_M2.duty= Int_Control_M2;
        }
    }
    else{
        giro=0;
        PWM2_Het1_18_M2.duty= 0;                        //Sin giro
        PWM3_Het1_20_M2.duty= 0;
    }
    pwmSetSignal(hetRAM1, pwm2, PWM2_Het1_18_M2);       //Se ajusta la salida del
    pwmSetSignal(hetRAM1, pwm3, PWM3_Het1_20_M2);       //control
    e_k_1_M2= e_k_M2;  //Se actualiza el error anterior e[k-1]=e[k] despues del calculo del control

    //}
}

void Control_Motor3(void){

    kp_M3;ki_M3;kd_M3;
    //kp_M3=2.95, ki_M3=0 kd_M3=0.04 good response

    if (Ref_deg_M3<60.0){
        Ref_deg_M3=60.0;
    }
    else if (Ref_deg_M3 > 120.0){
        Ref_deg_M3=120.0;
    }

    Pos_Deg_M3= Cont_Enc_M3*conv;   // couts *(360/8400)--> 8400 counts por revolucion
    e_k_M3=Ref_deg_M3-Pos_Deg_M3;   //calculo del Error actual e[k]
    acc_ki_M3 += e_k_M3*T;  //Acumulacion del integrador
    Control_k_M3= kp_M3*e_k_M3 + kd_M3* ( e_k_M3 - e_k_1_M3 ) / T + ki_M3*acc_ki_M3;
    Int_Control_M3= (int) Control_k_M3;

    if (Control_k_M3>0.0){
        giro=1;
        if (Control_k_M3 <= 99.0){
        PWM4_Het1_22_M3.duty= Int_Control_M3;
        PWM5_Het1_25_M3.duty= 0;
        }
        else{                                           //Giro hacia la Derecha (horario)
            Int_Control_M3=99;
            PWM4_Het1_22_M3.duty= Int_Control_M3;
            PWM5_Het1_25_M3.duty= 0;
        }
    }

    else if (Control_k_M3 < 0.0){
        giro=2;
        Aux_Control_M3= Control_k_M3 * -1.0;
        Int_Control_M3= (int) Aux_Control_M3;
        if(Control_k_M3 >-99.0 ){
            PWM4_Het1_22_M3.duty= 0 ;                   //Giro hacia la Izquierda (antihorario)
            PWM5_Het1_25_M3.duty=  Int_Control_M3;
        }
        else {
            Int_Control_M3=99;
            PWM4_Het1_22_M3.duty = 0;
            PWM5_Het1_25_M3.duty = Int_Control_M3;
        }
    }
    else{
        giro = 0;
        PWM4_Het1_22_M3.duty = 0;
        PWM5_Het1_25_M3.duty = 0;
    }
    pwmSetSignal(hetRAM1, pwm4, PWM4_Het1_22_M3);       //Se ajusta la salida del
    pwmSetSignal(hetRAM1, pwm5, PWM5_Het1_25_M3);       //control
    e_k_1_M3= e_k_M3;  //Se actualiza el error anterior e[k-1]=e[k] despues del calculo del control

    //}
}


// |---> [Funciones para comunicacion UART]
bool sciEnviarDatos(uint8 numOfDat, char* charDat, bool sc) {
    sciSend(scilinREG, numOfDat, (uint8 *) charDat);
    if (sc) {
        sciSend(scilinREG, 0x02, (unsigned char *) "\r\n");
    }
    return true;
}
//////////////////////////////////////////


// |---> [Funcion para interrupcion por flancos PWM]
void edgeNotification(hetBASE_t * hetREG, uint32 edge) {

    uint8_t flanco_Canal_A_M1 = gioGetBit(PORT_Canal_A_M1, BIT_Canal_A_M1);
    uint8_t flanco_Canal_B_M1 = gioGetBit(PORT_Canal_B_M1, BIT_Canal_B_M1);

    uint8_t flanco_Canal_A_M2 = gioGetBit(PORT_Canal_A_M2, BIT_Canal_A_M2);
    uint8_t flanco_Canal_B_M2 = gioGetBit(PORT_Canal_B_M2, BIT_Canal_B_M2);

    uint8_t flanco_Canal_A_M3 = gioGetBit(PORT_Canal_A_M3, BIT_Canal_A_M3);
    uint8_t flanco_Canal_B_M3 = gioGetBit(PORT_Canal_B_M3, BIT_Canal_B_M3);

    //                      ====Sentido de giro Motor 1===
    //edge==0 --> canal A  edge==1 --> canal B
    if (edge==Edge_Canal_A_M1 || edge == Edge_Canal_B_M1){

        //Canal A
        if(edge==Edge_Canal_A_M1 && flanco_Canal_A_M1==1 && flanco_Canal_B_M1==0){   //Condicion derecha 1
            Cond_Der_M1[cond_1]=true;
            Cont_Enc_M1++;
        }

        if(edge==Edge_Canal_A_M1 && flanco_Canal_A_M1==0 && flanco_Canal_B_M1==1){  //condicion derecha 2
                Cond_Der_M1[cond_2]=true;
                Cont_Enc_M1++;
            }

        if(edge==Edge_Canal_A_M1 && flanco_Canal_A_M1==1 && flanco_Canal_B_M1==1){  //condicion izquierda 1
            Cond_Izq_M1[cond_1]=true;
            Cont_Enc_M1--;
        }

        if(edge==Edge_Canal_A_M1 && flanco_Canal_A_M1==0 && flanco_Canal_B_M1==0){  //condicion izquierda 2
            Cond_Izq_M1[cond_2]=true;
            Cont_Enc_M1--;
            }

        //canal B
        if(edge==Edge_Canal_B_M1 && flanco_Canal_A_M1==1 && flanco_Canal_B_M1==1){   //Condicion derecha 3
            Cond_Der_M1[cond_3]=true;
            Cont_Enc_M1++;
        }

        if(edge==Edge_Canal_B_M1 && flanco_Canal_A_M1==0 && flanco_Canal_B_M1==0){  //condicion derecha 4
            Cond_Der_M1[cond_4]=true;
            Cont_Enc_M1++;
            }

        if(edge==Edge_Canal_B_M1 && flanco_Canal_A_M1==0 && flanco_Canal_B_M1==1){  //condicion izquierda 3
            Cond_Izq_M1[cond_3]=true;
            Cont_Enc_M1--;
        }

        if(edge==Edge_Canal_B_M1 && flanco_Canal_A_M1==1 && flanco_Canal_B_M1==0){  //condicion izquierda 4
            Cond_Izq_M1[cond_4]=true;
            Cont_Enc_M1--;
        }
    }

   //                       ====Sentido de giro Motor 2===

        //edge==2 --> canal A  edge==3 --> canal B
        if (edge==Edge_Canal_A_M2|| edge == Edge_Canal_B_M2){

            //Canal A
            if(edge==Edge_Canal_A_M2 && flanco_Canal_A_M2==1 && flanco_Canal_B_M2==0){   //Condicion derecha 1
                Cond_Der_M2[cond_1]=true;
                Cont_Enc_M2++;
            }

            if(edge==Edge_Canal_A_M2 && flanco_Canal_A_M2==0 && flanco_Canal_B_M2==1){  //condicion derecha 2
                    Cond_Der_M2[cond_2]=true;
                    Cont_Enc_M2++;
                }

            if(edge==Edge_Canal_A_M2 && flanco_Canal_A_M2==1 && flanco_Canal_B_M2==1){  //condicion izquierda 1
                Cond_Izq_M2[cond_1]=true;
                Cont_Enc_M2--;
            }

            if(edge==Edge_Canal_A_M2 && flanco_Canal_A_M2==0 && flanco_Canal_B_M2==0){  //condicion izquierda 2
                Cond_Izq_M2[cond_2]=true;
                Cont_Enc_M2--;
                }

            //canal B
            if(edge==Edge_Canal_B_M2 && flanco_Canal_A_M2==1 && flanco_Canal_B_M2==1){   //Condicion derecha 3
                Cond_Der_M2[cond_3]=true;
                Cont_Enc_M2++;
            }

            if(edge==Edge_Canal_B_M2 && flanco_Canal_A_M2==0 && flanco_Canal_B_M2==0){  //condicion derecha 4
                Cond_Der_M2[cond_4]=true;
                Cont_Enc_M2++;
                }

            if(edge==Edge_Canal_B_M2 && flanco_Canal_A_M2==0 && flanco_Canal_B_M2==1){  //condicion izquierda 3
                Cond_Izq_M2[cond_3]=true;
                Cont_Enc_M2--;
            }

            if(edge==Edge_Canal_B_M2 && flanco_Canal_A_M2==1 && flanco_Canal_B_M2==0){  //condicion izquierda 4
                Cond_Izq_M2[cond_4]=true;
                Cont_Enc_M2--;
            }

        }


        //                      ====Sentido de giro Motor 3===

               //edge==4 --> canal A  edge==5 --> canal B
               if (edge==Edge_Canal_A_M3 || edge == Edge_Canal_B_M3){

                //Canal A
                if(edge==Edge_Canal_A_M3 && flanco_Canal_A_M3==1 && flanco_Canal_B_M3==0){   //Condicion derecha 1
                    Cond_Der_M3[cond_1]=true;
                    Cont_Enc_M3++;
                }

                if(edge==Edge_Canal_A_M3 && flanco_Canal_A_M3==0 && flanco_Canal_B_M3==1){  //condicion derecha 2
                        Cond_Der_M3[cond_2]=true;
                        Cont_Enc_M3++;
                    }

                if(edge==Edge_Canal_A_M3 && flanco_Canal_A_M3==1 && flanco_Canal_B_M3==1){  //condicion izquierda 1
                    Cond_Izq_M3[cond_1]=true;
                    Cont_Enc_M3--;
                }

                if(edge==Edge_Canal_A_M3 && flanco_Canal_A_M3==0 && flanco_Canal_B_M3==0){  //condicion izquierda 2
                    Cond_Izq_M3[cond_2]=true;
                    Cont_Enc_M3--;
                    }

                //canal B
                if(edge==Edge_Canal_B_M3 && flanco_Canal_A_M3==1 && flanco_Canal_B_M3==1){   //Condicion derecha 3
                    Cond_Der_M3[cond_3]=true;
                    Cont_Enc_M3++;
                }

                if(edge==Edge_Canal_B_M3 && flanco_Canal_A_M3==0 && flanco_Canal_B_M3==0){  //condicion derecha 4
                    Cond_Der_M3[cond_4]=true;
                    Cont_Enc_M3++;
                    }

                if(edge==Edge_Canal_B_M3 && flanco_Canal_A_M3==0 && flanco_Canal_B_M3==1){  //condicion izquierda 3
                    Cond_Izq_M3[cond_3]=true;
                    Cont_Enc_M3--;
                }

                if(edge==Edge_Canal_B_M3 && flanco_Canal_A_M3==1 && flanco_Canal_B_M3==0){  //condicion izquierda 4
                    Cond_Izq_M3[cond_4]=true;
                    Cont_Enc_M3--;
                }

               }


    //Evaluacion del sentido de giro del Motor 1
   if(Cond_Der_M1[cond_1] && Cond_Der_M1[cond_2] && Cond_Der_M1[cond_3] && Cond_Der_M1[cond_4]){
        Sentido_M1=Sen_horario;
        Cond_Der_M1[cond_1]=false;
        Cond_Der_M1[cond_2]=false;
        Cond_Der_M1[cond_3]=false;
        Cond_Der_M1[cond_4]=false;
    }
    else if(Cond_Izq_M1[cond_1] && Cond_Izq_M1[cond_2] && Cond_Izq_M1[cond_3] && Cond_Izq_M1[cond_4]){
        Sentido_M1=Sen_antihorario;
        Cond_Izq_M1[cond_1]=false;
        Cond_Izq_M1[cond_2]=false;
        Cond_Izq_M1[cond_3]=false;
        Cond_Izq_M1[cond_4]=false;
    }
    else
        Sentido_M1=Sin_detectar;


   //Evaluacion del sentido de giro del Motor 2
  if(Cond_Der_M2[cond_1] && Cond_Der_M2[cond_2] && Cond_Der_M2[cond_3] && Cond_Der_M2[cond_4]){
    Sentido_M2=Sen_horario;
    Cond_Der_M2[cond_1]=false;
    Cond_Der_M2[cond_2]=false;
    Cond_Der_M2[cond_3]=false;
    Cond_Der_M2[cond_4]=false;
   }
   else if(Cond_Izq_M2[cond_1] && Cond_Izq_M2[cond_2] && Cond_Izq_M2[cond_3] && Cond_Izq_M2[cond_4]){
    Sentido_M2=Sen_antihorario;
    Cond_Izq_M2[cond_1]=false;
    Cond_Izq_M2[cond_2]=false;
    Cond_Izq_M2[cond_3]=false;
    Cond_Izq_M2[cond_4]=false;
   }
   else
    Sentido_M2=Sin_detectar;


//Evaluacion del sentido de giro del Motor 3
if(Cond_Der_M3[cond_1] && Cond_Der_M3[cond_2] && Cond_Der_M3[cond_3] && Cond_Der_M3[cond_4]){
    Sentido_M3=Sen_horario;
    Cond_Der_M3[cond_1]=false;
    Cond_Der_M3[cond_2]=false;
    Cond_Der_M3[cond_3]=false;
    Cond_Der_M3[cond_4]=false;
}
else if(Cond_Izq_M3[cond_1] && Cond_Izq_M3[cond_2] && Cond_Izq_M3[cond_3] && Cond_Izq_M3[cond_4]){
    Sentido_M3=Sen_antihorario;
    Cond_Izq_M3[cond_1]=false;
    Cond_Izq_M3[cond_2]=false;
    Cond_Izq_M3[cond_3]=false;
    Cond_Izq_M3[cond_4]=false;
}
else
    Sentido_M3=Sin_detectar;
}


// |---> [Funciones para el TIMER]
void rtiNotification(uint32 notification) {
    Timer_1ms++;
}
//////////////////////////////////


//////CONTROL PID DIGITAL

//  if (c_count==2){    //Contador para el error e[k-2]
//      c_count=2;
//  }
//  else{
//      c_count++;
//  }
//          if (e_k_M1 >= tolerance || (Pos_Deg_M1< Ref_deg_M1 && Pos_Deg_M1<0.0)){  //Tolerancia de 1 grado
//          PWM0_Het1_10_M1.duty= Int_Control_M1;     //Giro hacia la Derecha (horario)
//          PWM1_Het1_28_M1.duty= 0;
//              //gioSetBit(PORT_Motor1, BIT_Motor1_IN1, LOW);     // Horario ---Derecha
//              //gioSetBit(PORT_Motor1, BIT_Motor1_IN2, HIGH);
//              //PWM0_Het1_20_M1.duty=Int_Control_M1;
//
//          giro=1;  //variable para imprimir en pantalla el sentido de giro
//
//          }
//          else if (e_k_M1 < -tolerance){
//
//              PWM0_Het1_10_M1.duty= 0 ;
//              PWM1_Het1_28_M1.duty= Int_Control_M1; //Giro hacia la Izquierda (antihorario)
//              //gioSetBit(PORT_Motor1, BIT_Motor1_IN1, HIGH);   //Antihorario --Izquierda
//              //gioSetBit(PORT_Motor1, BIT_Motor1_IN2, LOW);
//              //PWM0_Het1_20_M1.duty=Int_Control_M1;
//              giro=2;
//          }
//          else{
//
//              PWM0_Het1_10_M1.duty= 0;   //Sin giro
//              PWM1_Het1_28_M1.duty= 0;
//              //gioSetBit(PORT_Motor1, BIT_Motor1_IN1, LOW);
//              //gioSetBit(PORT_Motor1, BIT_Motor1_IN2, LOW);
//              //PWM0_Het1_20_M1.duty=0;
//              giro=0;
//          }
//          if (e_k_M1<=0.0){   //Error positivo  para evitar una salida de control negativo
//              e_k_M1 *= -1.0;
//          }

//acc_ki_M1= (acc_ki_M1+ acc_ki)*k_PID_M1;  //acumulacion de la parte integral del
                                                // control

//Integrator1_M3= acc_ki_M1+ e_k_M1*T;      //Integrador con e[k]

//if (Integrator1_M3>=5.0){
    //Integrator1_M3=5.0;
//}
//else if (Integrator1_M3<=-5.0){        //Anti-windup
//  Integrator1_M3=-5.0;
//}

//Integrator1_M3= ki_M1*Integrator1_M3/2.0;  //formula de la parte integral del control


//Integrator2_M3=acc_ki_M1+ e_k_1_M1*T;  //Integrador con e[k-1]

//if (Integrator2_M3>=5.0){
    //Integrator2_M3=5.0;
//}
//else if (Integrator2_M3<=-5.0){        //Anti-windup
//  Integrator2_M3=-5.0;
//}

//Integrator2_M3= ki_M1*Integrator2_M3/2.0;  //formula de la parte integral del control

//C1_M3= (kp_M1 + kd_M1/T)*k_PID_M1;                //Coeficientes para el control
//C2_M3=(-kp_M1 -(2.0*kd_M1)/T)*k_PID_M1;
//C3_M3= (kd_M1/T)*k_PID_M1;

//Control_k_M1= Control_k_1_M1 +  e_k_M1*C1_M3  + Integrator1_M3 + e_k_1_M1*C2_M3  + Integrator2_M3 + e_k_2_M1*C3_M3; //Salida
                                                                                                            //de control
//Int_Control_M1= (int) Control_k_M1; // Salida de control tipo entero

//if(Control_k_M1 >= 99.0){   //Proteccion de control duty-> [0-99]
    //Control_k_M1 =99.0;
//}
//if(Control_k_M1 <=0.0){  //Proteccion de control duty-> [0-99]
    //Control_k_M1 *=-1.0;
//}

//pwmSetSignal(hetRAM1, pwm0, PWM0_Het1_20_M1);


//aux_e_k_1_M1=e_k_1_M1; //Se guarda e[k-1]

//Control_k_1_M1= Control_k_M1; //Se actualuiza control[k-1]= control[k] despues del calculo de control

//if (c_count==2){ //Si Cont_Enc_M1=2  e[k-2]=e[k-1]
    //e_k_2_M1=aux_e_k_1_M1;
//}

/* USER CODE END */
