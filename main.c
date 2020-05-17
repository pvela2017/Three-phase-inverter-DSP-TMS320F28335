/*
 * Codigo para inversor VSI, conectado a la red usando PLL
 * y a paneles solares. Para inyectar potencia reactiva o activa
 * a la red. Hardware: TMS320F28335.
 * Frecuencia de conmutacion: 3000 Hz
 * salida pwm pines: 00,02,04 y complementos 01,03,05
 *
 * 			Mefisto-LCDA 2017
 * 	     Universidad de Concepcion
 */

#include "DSP2833x_Device.h"
#include "DSP2833x_Examples.h"

#define M 10 //numero de muestras para la media movil

//Variables para debugear
volatile int des_expA=37,des_expB=40,des_expC=40; //Desfase experimental del PLL


//Tabla de seno
/*
volatile unsigned long int sine[60]={1250,1117, 986, 857, 733, 615, 505, 402, 309, 227, 157,
		  98,  53,  22,   4,   0,  11,  36,  74, 126, 190, 267,
		 354, 452, 559, 674, 795, 921,1051,1183,1317,1449,1579,
		1705,1826,1941,2048,2146,2233,2310,2374,2426,2464,2489,
		2500,2496,2478,2447,2402,2343,2273,2191,2098,1995,1885,
		1767,1643,1514,1383,1250};*/

/*Valor maximo= 2500 porque es el valor más grande(en este caso chico) que puede alcanzar el comparador
 * 60 elementos porque 3e3kHz (periodo) cabe 60 veces en 50Hz (periodo); por lo tanto se tienen 60 oprtunidades
 * para cambiar el duty*/

volatile unsigned long int sine[360]={1250,1228,1206,1184,1163,1141,1119,1097,1076,1054,1032,1011,989,968,947,926,905,884,863,842,821,801,781,760,740,720,701,681,662,642,623,605,586,568,549,531,513,496,479,462,445,428,412,396,380,364,349,334,319,305,290,277,263,250,237,224,212,200,188,177,
		166,155,145,135,125,115,106,98,89,82,74,67,60,53,47,41,36,31,26,22,18,15,11,9,6,4,3,1,1,0,0,0,1,2,3,5,7,10,13,16,20,24,29,33,39,44,50,57,63,70,78,85,94,102,111,120,130,139,150,160,
		171,182,194,206,218,230,243,256,270,284,298,312,326,341,357,372,388,404,420,436,453,470,487,505,522,540,558,577,595,614,633,652,671,691,710,730,750,770,791,811,832,852,873,894,915,936,957,979,1000,1022,1043,1065,1086,1108,1130,1152,1173,1195,1217,1239,
		1261,1283,1305,1327,1348,1370,1392,1414,1435,1457,1478,1500,1521,1543,1564,1585,1606,1627,1648,1668,1689,1709,1730,1750,1770,1790,1809,1829,1848,1867,1886,1905,1923,1942,1960,1978,1995,2013,2030,2047,2064,2080,2096,2112,2128,2143,2159,2174,2188,2202,2216,2230,2244,2257,2270,2282,2294,2306,2318,2329,
		2340,2350,2361,2370,2380,2389,2398,2406,2415,2422,2430,2437,2443,2450,2456,2461,2467,2471,2476,2480,2484,2487,2490,2493,2495,2497,2498,2499,2500,2500,2500,2499,2499,2497,2496,2494,2491,2489,2485,2482,2478,2474,2469,2464,2459,2453,2447,2440,2433,2426,2418,2411,2402,2394,2385,2375,2365,2355,2345,2334,
		2323,2312,2300,2288,2276,2263,2250,2237,2223,2210,2195,2181,2166,2151,2136,2120,2104,2088,2072,2055,2038,2021,2004,1987,1969,1951,1932,1914,1895,1877,1858,1838,1819,1799,1780,1760,1740,1719,1699,1679,1658,1637,1616,1595,1574,1553,1532,1511,1489,1468,1446,1424,1403,1381,1359,1337,1316,1294,1272,1250};

/*Valor maximo= 2500 porque es el valor más grande(en este caso chico) que puede alcanzar el comparador
 * 360 elementos porque 3e3kHz (periodo) cabe 60 veces en 50Hz (periodo); por lo tanto se tienen 60 oprtunidades
 * para cambiar el duty Y SE VARIARA CADA 6 PERO SE TIENE UNA MEJOR RESOLUCION DE 1° CUANDO SE NECESITE*/

//Buffer de voltajes de red y corrientes del sistema
volatile int va_buff[360],vb_buff[360],vc_buff[360]; //volatjes de la red
volatile float Vmax=13.0; //voltaje peak definido mediante simulacion
volatile float VA,VB,VC; //Voltajes medidos

//Parametros moduladora
volatile float ma=1; //valores entre 0 y 1 (indice de modulacion)
volatile float mb=1;
volatile float mc=1;
volatile int phi_a=0; //desfase de la moduladora 1° de resolucion valores enteros
volatile int phi_b=0;
volatile int phi_c=0;
volatile int angulo_alpha; //Angulo alpha a pasar a la tansformacion dq0

//Parametros adquisicion
volatile float va[M];
volatile float vb[M];
volatile float vc[M];
volatile float vm_a;
volatile float vm_b;
volatile float vm_c;
volatile float M_mul;
volatile float G_va=1.8207,G_vb=1.874,G_vc=1.895;
volatile float O_va=6.74,O_vb=6.8,O_vc=7.39;

// Funciones de configuracion.
void sys_conf(void);
void DesactivarDog(void);
void PWM_conf(void);
void adc_conf(void);
void IniCpuTimers(void);

//Interrupciones
__interrupt void epwm1_isr(void);
__interrupt void epwm2_isr(void);
__interrupt void epwm3_isr(void);
__interrupt void cpu_timer1_isr(void);


void main(void)
{
// Disable CPU interrupts

	DINT;

// Initialize System Control:
   //InitSysCtrl();
   sys_conf(); //Configuracion el sistema (PLL a 150MHz y adc)
   IniCpuTimers(); //Configuracion timer
   PWM_conf(); //Configuracion ePWM

// Initialize the PIE control registers to their default state.
   InitPieCtrl();

// Disable CPU interrupts and clear all CPU interrupt flags:
   IER = 0x0000;
   IFR = 0x0000;

// Initialize the PIE vector table with pointers to the shell Interrupt
   InitPieVectTable();

   EALLOW;  // This is needed to write to EALLOW protected registers
   //Rellenamos la tabla de interrupciones
   PieVectTable.EPWM1_INT = &epwm1_isr;
   PieVectTable.EPWM2_INT = &epwm2_isr;
   PieVectTable.EPWM3_INT = &epwm3_isr;
   PieVectTable.XINT13= &cpu_timer1_isr;
   EDIS;    // This is needed to disable write to EALLOW protected registers

   ConfigCpuTimer(&CpuTimer1, 150, 55.55555); //Configura el timer1 cada  55 micro (muetreo datos)

   CpuTimer1Regs.TCR.all = 0x4000; // Use write-only instruction to set TSS bit = 0

   //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
   //solo para debugear
   EALLOW;
   GpioCtrlRegs.GPBMUX1.bit.GPIO32 = 0;
   GpioCtrlRegs.GPBDIR.bit.GPIO32 = 1;
   GpioCtrlRegs.GPBMUX1.bit.GPIO34 = 0;
   GpioCtrlRegs.GPBDIR.bit.GPIO34 = 1;
   EDIS;
   GpioDataRegs.GPBCLEAR.bit.GPIO32 = 1;
   GpioDataRegs.GPBCLEAR.bit.GPIO34 = 1;
   //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

   //Activacion de interrupciones
   IER |= M_INT3; //Activa interrupcion CPU INT3 conectada al EPWM
   IER |= M_INT13; //Timer1

   //Activa interrupciones del PIE
   PieCtrlRegs.PIEIER3.bit.INTx1 = 1;
   PieCtrlRegs.PIEIER3.bit.INTx2 = 1;
   PieCtrlRegs.PIEIER3.bit.INTx3 = 1;
   PieCtrlRegs.PIEIER1.bit.INTx6 = 1;
   PieCtrlRegs.PIEIER1.bit.INTx7 = 1;

   //Inicializacion de algunas variables
   M_mul=1.0/M;

   // Enable global Interrupts and higher priority real-time debug events:
   EINT;   // Enable Global interrupt INTM
   ERTM;   // Enable Global realtime interrupt DBGM

//LOOP
   while(1)
   {
   }
}

//Funciones de configuracion
void sys_conf(void)
{
	DesactivarDog();
	EALLOW;

	// HISPCP/LOSPCP prescale register settings, normally it will be set to default values
	   SysCtrlRegs.HISPCP.all = 0x0003;
	   SysCtrlRegs.LOSPCP.all = 0x0002;

	// XCLKOUT to SYSCLKOUT ratio.  By default XCLKOUT = 1/4 SYSCLKOUT
	   // XTIMCLK = SYSCLKOUT/2
	   XintfRegs.XINTCNF2.bit.XTIMCLK = 1;
	   // XCLKOUT = XTIMCLK/2
	   XintfRegs.XINTCNF2.bit.CLKMODE = 1;
	   // Enable XCLKOUT
	   XintfRegs.XINTCNF2.bit.CLKOFF = 0;
	   EDIS;

	   adc_conf(); //Configuracion adc

	   EALLOW;
	   //Se desactivan los perifericos que no se utilizan
	   SysCtrlRegs.PCLKCR0.bit.I2CAENCLK = 0;   // I2C
	   SysCtrlRegs.PCLKCR0.bit.SCIAENCLK = 0;   // SCI-A
	   SysCtrlRegs.PCLKCR0.bit.SCIBENCLK = 0;   // SCI-B
	   SysCtrlRegs.PCLKCR0.bit.SCICENCLK = 0;   // SCI-C
	   SysCtrlRegs.PCLKCR0.bit.SPIAENCLK = 0;   // SPI-A
	   SysCtrlRegs.PCLKCR0.bit.MCBSPAENCLK = 1; // McBSP-A
	   SysCtrlRegs.PCLKCR0.bit.MCBSPBENCLK = 1; // McBSP-B
	   SysCtrlRegs.PCLKCR0.bit.ECANAENCLK=0;    // eCAN-A
	   SysCtrlRegs.PCLKCR0.bit.ECANBENCLK=0;    // eCAN-B

	   SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 0;   // Disable TBCLK within the ePWM
	   SysCtrlRegs.PCLKCR1.bit.EPWM1ENCLK = 1;  // ePWM1
	   SysCtrlRegs.PCLKCR1.bit.EPWM2ENCLK = 1;  // ePWM2
	   SysCtrlRegs.PCLKCR1.bit.EPWM3ENCLK = 1;  // ePWM3
	   SysCtrlRegs.PCLKCR1.bit.EPWM4ENCLK = 0;  // ePWM4
	   SysCtrlRegs.PCLKCR1.bit.EPWM5ENCLK = 0;  // ePWM5
	   SysCtrlRegs.PCLKCR1.bit.EPWM6ENCLK = 0;  // ePWM6
	   SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1;   // Enable TBCLK within the ePWM

	   SysCtrlRegs.PCLKCR1.bit.ECAP3ENCLK = 0;  // eCAP3
	   SysCtrlRegs.PCLKCR1.bit.ECAP4ENCLK = 0;  // eCAP4
	   SysCtrlRegs.PCLKCR1.bit.ECAP5ENCLK = 0;  // eCAP5
	   SysCtrlRegs.PCLKCR1.bit.ECAP6ENCLK = 0;  // eCAP6
	   SysCtrlRegs.PCLKCR1.bit.ECAP1ENCLK = 0;  // eCAP1
	   SysCtrlRegs.PCLKCR1.bit.ECAP2ENCLK = 0;  // eCAP2
	   SysCtrlRegs.PCLKCR1.bit.EQEP1ENCLK = 0;  // eQEP1
	   SysCtrlRegs.PCLKCR1.bit.EQEP2ENCLK = 0;  // eQEP2

	   SysCtrlRegs.PCLKCR3.bit.CPUTIMER0ENCLK = 1; // CPU Timer 0
	   SysCtrlRegs.PCLKCR3.bit.CPUTIMER1ENCLK = 1; // CPU Timer 1
	   SysCtrlRegs.PCLKCR3.bit.CPUTIMER2ENCLK = 1; // CPU Timer 2

	   SysCtrlRegs.PCLKCR3.bit.DMAENCLK = 1;       // DMA Clock
	   SysCtrlRegs.PCLKCR3.bit.XINTFENCLK = 1;     // XTIMCLK
	   SysCtrlRegs.PCLKCR3.bit.GPIOINENCLK = 1;    // GPIO input clock

	   EDIS;

	   InitPll(DSP28_PLLCR,DSP28_DIVSEL); //Inicia PLL 150MHz
}



void PWM_conf(void)
{
	//pines (ocupar 0,2,4)

	EALLOW;
	GpioCtrlRegs.GPAPUD.bit.GPIO0 = 0;    // Enable pull-up on GPIO0 (EPWM1A)
	GpioCtrlRegs.GPAPUD.bit.GPIO1 = 0;    // Enable pull-up on GPIO1 (EPWM1B)
	GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 1;   // Configure GPIO0 as EPWM1A
	GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 1;   // Configure GPIO1 as EPWM1B

	GpioCtrlRegs.GPAPUD.bit.GPIO2 = 0;    // Enable pull-up on GPIO2 (EPWM2A)
	GpioCtrlRegs.GPAPUD.bit.GPIO3 = 0;    // Enable pull-up on GPIO3 (EPWM2B)
	GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 1;   // Configure GPIO2 as EPWM2A
	GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 1;   // Configure GPIO3 as EPWM2B

	GpioCtrlRegs.GPAPUD.bit.GPIO4 = 0;    // Enable pull-up on GPIO4 (EPWM3A)
	GpioCtrlRegs.GPAPUD.bit.GPIO5 = 0;    // Enable pull-up on GPIO5 (EPWM3B)
	GpioCtrlRegs.GPAMUX1.bit.GPIO4 = 1;   // Configure GPIO4 as EPWM3A
	GpioCtrlRegs.GPAMUX1.bit.GPIO5 = 1;   // Configure GPIO5 as EPWM3B
	EDIS;

	EALLOW;
	SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 0;      // Stop all the TB clocks
	EDIS;

	//Parametros
	unsigned int frec=2500; //2500 (frec 3khz)
	unsigned int desfase=0; //833; la tabla desfasa ahora --> 0
	int pre=0b000; // 1
	int pre_rap=0b101; //101->10

	//Limpieza registro TB
	EPwm1Regs.TBCTR=0; //limpia registro TB
	EPwm2Regs.TBCTR=0; //limpia registro TB
	EPwm3Regs.TBCTR=0; //limpia registro TB

	//FASE 1
	//Configuracion
	EPwm1Regs.TBPRD=frec; //Periodo
	EPwm1Regs.TBPHS.half.TBPHS=0;  //Fase
	EPwm1Regs.CMPA.half.CMPA=1250; //setea comparador A
	//EPwm1Regs.CMPB=144; 		  //setea comparador B

	//Registro de control del tiempo
	EPwm1Regs.TBCTL.bit.CTRMODE=0b10;   //Modo Up-count (pagina 23)
	EPwm1Regs.TBCTL.bit.PHSEN=0b0;     // Maestro
	EPwm1Regs.TBCTL.bit.PRDLD=0b0;
	EPwm1Regs.TBCTL.bit.SYNCOSEL=0b01;  // sync flow
	EPwm1Regs.TBCTL.bit.HSPCLKDIV=pre_rap; //presescalador rapido 1
	EPwm1Regs.TBCTL.bit.CLKDIV=pre;    //prescalador 1

	//Registro de control del comparador
	EPwm1Regs.CMPCTL.bit.SHDWAMODE=0; //doble buffer(registro sombra activado)
	EPwm1Regs.CMPCTL.bit.SHDWBMODE=0;
	EPwm1Regs.CMPCTL.bit.LOADAMODE=00; //se carga cuando ctr=zero
	EPwm1Regs.CMPCTL.bit.LOADBMODE=00;

	//Registro de control del calificador de accion
	EPwm1Regs.AQCTLA.bit.CAU=0b10;
	EPwm1Regs.AQCTLA.bit.CAD=0b01;

	//Para 6 pulsos
	EPwm1Regs.AQCTLB.bit.CAU=0b01;
	EPwm1Regs.AQCTLB.bit.CAD=0b10;

	//Activar interrupcion
	EPwm1Regs.ETSEL.bit.INTSEL=0b001; //TBCTR=0
	EPwm1Regs.ETPS.bit.INTCNT=0b01; // 1 evento
	EPwm1Regs.ETPS.bit.INTPRD=0b01; // 1 evento
	EPwm1Regs.ETSEL.bit.INTEN = 1;


	//~~~~FASE 2~~~~
	//Configuracion
	EPwm2Regs.TBPRD=frec; //Periodo
	EPwm2Regs.TBPHS.half.TBPHS=desfase;  //Fase
	EPwm2Regs.CMPA.half.CMPA=1250; //setea comparador A
	//EPwm2Regs.CMPB=144; 		  //setea comparador B

	//Registro de control del tiempo
	EPwm2Regs.TBCTL.bit.CTRMODE=0b10;   //Modo Up-count (pagina 23)
	EPwm2Regs.TBCTL.bit.PHSEN=0b1;      // Esclavo
	EPwm2Regs.TBCTL.bit.PHSDIR=0b0;
	EPwm2Regs.TBCTL.bit.PRDLD=0b0;
	EPwm2Regs.TBCTL.bit.SYNCOSEL=0b00;
	EPwm2Regs.TBCTL.bit.HSPCLKDIV=pre_rap; //presescalador rapido 1
	EPwm2Regs.TBCTL.bit.CLKDIV=pre;    //prescalador 1

	//Registro de control del comparador
	EPwm2Regs.CMPCTL.bit.SHDWAMODE=0b0; //doble buffer(registro sombra activado)
	EPwm2Regs.CMPCTL.bit.SHDWBMODE=0b0;
	EPwm2Regs.CMPCTL.bit.LOADAMODE=0b00; //se carga cuando ctr=zero
	EPwm2Regs.CMPCTL.bit.LOADBMODE=0b00;

	//Registro de control del calificador de accion
	EPwm2Regs.AQCTLA.bit.CAU=0b10;
	EPwm2Regs.AQCTLA.bit.CAD=0b01;

	//Para 6 pulsos
	EPwm2Regs.AQCTLB.bit.CAU=0b01;
	EPwm2Regs.AQCTLB.bit.CAD=0b10;

	//Activar interrupcion
	EPwm2Regs.ETSEL.bit.INTSEL=0b001; //TBCTR=0
	EPwm2Regs.ETPS.bit.INTCNT=0b01; // 1 evento
	EPwm2Regs.ETPS.bit.INTPRD=0b01; // 1 evento
	EPwm2Regs.ETSEL.bit.INTEN = 1;


	//~~~~FASE 3~~~~
	//Configuracion
	EPwm3Regs.TBPRD=frec; //Periodo
	EPwm3Regs.TBPHS.half.TBPHS=desfase;  //Fase
	EPwm3Regs.CMPA.half.CMPA=1250; //setea comparador A
	//EPwm3Regs.CMPB=144; 		  //setea comparador B

	//Registro de control del tiempo
	EPwm3Regs.TBCTL.bit.CTRMODE=0b10;   //Modo Up-count (pagina 23)
	EPwm3Regs.TBCTL.bit.PHSEN=0b1;      // Esclavo
	EPwm3Regs.TBCTL.bit.PHSDIR=0b1;
	EPwm3Regs.TBCTL.bit.PRDLD=0b0;
	EPwm3Regs.TBCTL.bit.SYNCOSEL=0b00;
	EPwm3Regs.TBCTL.bit.HSPCLKDIV=pre_rap; //presescalador rapido 1
	EPwm3Regs.TBCTL.bit.CLKDIV=pre;    //prescalador 1

	//Registro de control del comparador
	EPwm3Regs.CMPCTL.bit.SHDWAMODE=0b0; //doble buffer(registro sombra activado)
	EPwm3Regs.CMPCTL.bit.SHDWBMODE=0b0;
	EPwm3Regs.CMPCTL.bit.LOADAMODE=0b00; //se carga cuando ctr=zero
	EPwm3Regs.CMPCTL.bit.LOADBMODE=0b00;

	//Registro de control del calificador de accion
	EPwm3Regs.AQCTLA.bit.CAU=0b10;
	EPwm3Regs.AQCTLA.bit.CAD=0b01;

	//Para 6 pulsos
	EPwm3Regs.AQCTLB.bit.CAU=0b01;
	EPwm3Regs.AQCTLB.bit.CAD=0b10;

	//Activar interrupcion
	EPwm3Regs.ETSEL.bit.INTSEL=0b001; //TBCTR=0
	EPwm3Regs.ETPS.bit.INTCNT=0b01; // 1 evento
	EPwm3Regs.ETPS.bit.INTPRD=0b01; // 1 evento
	EPwm3Regs.ETSEL.bit.INTEN = 1;


	EALLOW;
	SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1;         // Start all the timers synced
	EDIS;
}

void adc_conf(void)
{
    EALLOW;
	SysCtrlRegs.PCLKCR0.bit.ADCENCLK = 1;

	ADC_cal(); //funcion de calibracion de la dsp automatico
	AdcRegs.ADCTRL3.all = 0x00E0;  // Power up bandgap/reference/ADC circuits
	AdcRegs.ADCTRL3.bit.ADCCLKPS=0b0000; //Core clock divider en 1
	AdcRegs.ADCTRL3.bit.SMODE_SEL=0b1; //modo simultaneo
	DELAY_US(5000L);

	//Configuracion
	AdcRegs.ADCTRL1.bit.SUSMOD=0b00; //no se suspende con breakpoints
	AdcRegs.ADCTRL1.bit.CPS=0b0; //preescalador divide por 1
	AdcRegs.ADCTRL1.bit.CONT_RUN=0b1; //modo continuo
	AdcRegs.ADCTRL1.bit.SEQ_CASC=0b1; //modo cascada

	AdcRegs.ADCTRL2.bit.EPWM_SOCA_SEQ1=0b0;
	AdcRegs.ADCTRL2.bit.EPWM_SOCB_SEQ=0b0;

	AdcRegs.ADCMAXCONV.all=0x0003; //8 muestras (4dobles)

	AdcRegs.ADCCHSELSEQ1.bit.CONV00 = 0x0;    // Setup ADCINA0 y ADCINB0
    AdcRegs.ADCCHSELSEQ1.bit.CONV01 = 0x1;    // Setup ADCINA1 y ADCINB1
    AdcRegs.ADCCHSELSEQ1.bit.CONV02 = 0x2;    // Setup ADCINA2 y ADCINB2
    AdcRegs.ADCCHSELSEQ1.bit.CONV03 = 0x3;    // Setup ADCINA3 y ADCINB3

    AdcRegs.ADCTRL2.bit.SOC_SEQ1=1; //Inicia muestreo

    EDIS;
}

void DesactivarDog(void)
{
    EALLOW;
    SysCtrlRegs.WDCR= 0x0068;
    EDIS;
}

void IniCpuTimers(void)
{
    // CPU Timer 0
    // Initialize address pointers to respective timer registers:
    CpuTimer0.RegsAddr = &CpuTimer0Regs;
    // Initialize timer period to maximum:
    CpuTimer0Regs.PRD.all  = 0xFFFFFFFF;
    // Initialize pre-scale counter to divide by 1 (SYSCLKOUT):
    CpuTimer0Regs.TPR.all  = 0;
    CpuTimer0Regs.TPRH.all = 0;
    // Make sure timer is stopped:
    CpuTimer0Regs.TCR.bit.TSS = 1;
    // Reload all counter register with period value:
    CpuTimer0Regs.TCR.bit.TRB = 1;
    // Reset interrupt counters:
    CpuTimer0.InterruptCount = 0;

    // CPU Timer 1
	// Initialize address pointers to respective timer registers:
	CpuTimer1.RegsAddr = &CpuTimer1Regs;
	// Initialize timer period to maximum:
	CpuTimer1Regs.PRD.all  = 0xFFFFFFFF;
	// Initialize pre-scale counter to divide by 1 (SYSCLKOUT):
	CpuTimer1Regs.TPR.all  = 0;
	CpuTimer1Regs.TPRH.all = 0;
	// Make sure timer is stopped:
	CpuTimer1Regs.TCR.bit.TSS = 1;
	// Reload all counter register with period value:
	CpuTimer1Regs.TCR.bit.TRB = 1;
	// Reset interrupt counters:
	CpuTimer1.InterruptCount = 0;


// CpuTimer2 is reserved for DSP BIOS & other RTOS
// Do not use this timer if you ever plan on integrating
// DSP-BIOS or another realtime OS.
    // Initialize address pointers to respective timer registers:
        CpuTimer2.RegsAddr = &CpuTimer2Regs;
        // Initialize timer period to maximum:
        CpuTimer2Regs.PRD.all  = 0xFFFFFFFF;
        // Make sure timers are stopped:
        CpuTimer2Regs.TCR.bit.TSS = 1;
        // Reload all counter register with period value:
        CpuTimer2Regs.TCR.bit.TRB = 1;
        // Reset interrupt counters:
        CpuTimer2.InterruptCount = 0;
}

//Interrupciones
__interrupt void epwm1_isr(void) //actualiza la tabla
{
	int angulo;
	int comp;
	angulo=angulo_alpha+phi_a+des_expA; //En caso de hacerlo en lazo abierto y sin red usar angulo=i+phi_a; repetir en las otras fases
	if (angulo>359)//en caso de sobrepasar la cantidad maxima de elementos del arreglo
	{
		angulo-=360;
	}
	comp=ma*va_buff[angulo];
	if(comp>2499)
	{
		comp=2499;
	}
	if(comp<1)
	{
		comp=1;
	}
	EPwm1Regs.CMPA.half.CMPA=ma*va_buff[angulo]; //En caso de hacerlo en lazo abierto y sin red usar ma*sine[angulo]; repetir en las otras fases

	EPwm1Regs.ETCLR.bit.INT = 1; //Limpia el flag
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
}

__interrupt void epwm2_isr(void)//actualiza la tabla
{
	int angulo;
	int comp;
	angulo=angulo_alpha+phi_b+des_expB;
	if (angulo>359)//en caso de sobrepasar la cantidad maxima de elementos del arreglo
	{
		angulo-=360;
	}
	comp=mb*vb_buff[angulo];
	if(comp>2499)
	{
		comp=2499;
	}
	if(comp<1)
	{
		comp=1;
	}
	EPwm2Regs.CMPA.half.CMPA=comp;

	EPwm2Regs.ETCLR.bit.INT = 1; //Limpia el flag
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
}

__interrupt void epwm3_isr(void)//actualiza la tabla
{
	int angulo;
	int comp;
	angulo=angulo_alpha+phi_c+des_expC;
	if (angulo>359)//en caso de sobrepasar la cantidad maxima de elementos del arreglo
	{
		angulo-=360;
	}
	comp=mc*vc_buff[angulo];

	if(comp>2499)
	{
		comp=2499;
	}
	if(comp<1)
	{
		comp=1;
	}
	EPwm3Regs.CMPA.half.CMPA=comp;

	EPwm3Regs.ETCLR.bit.INT = 1; //Limpia el flag
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
}

__interrupt void cpu_timer1_isr(void)
{
   CpuTimer1.InterruptCount++;
   GpioDataRegs.GPBSET.bit.GPIO32 = 1;

   float va_b,vb_b,vc_b;
   static int j=0;
   int i=0;

   	vm_a=0;
   	vm_b=0;
   	vm_c=0.0;

   	//Se olvida el ultimo dato
   	for(i=0;i<M-1;i++)
   	{
   		va[i]=va[i+1];
   		vb[i]=vb[i+1];
   		vc[i]=vc[i+1];
   	}
   	//Se carga el nuevo dato
   	//Lado A
   	vc[M-1]=AdcRegs.ADCRESULT0>>4;	//A0
   	vb[M-1]=AdcRegs.ADCRESULT2>>4;	//A1
   	va[M-1]=AdcRegs.ADCRESULT4>>4;	//A2

   	//Se obtiene la media movil
   	for(i=0;i<M;i++)
   	{
   		vm_a+=va[i];
   		vm_b+=vb[i];
   		vm_c+=vc[i];
   	}

   	vm_a*=M_mul;
   	vm_b*=M_mul;
   	vm_c*=M_mul;

   	vm_a=0.00732421875*vm_a-15.0; //v_salida=Limite_superior/(bit_max-bit_zero)+Limite_inferior
   	vm_b=0.00732421875*vm_b-15.0; //=> v_salida=15/(4096-2048)-15
   	vm_c=0.00732421875*vm_c-15.0; //12 bits =4096 (bit_max)

   	//Calibracion de la señal
   	vm_a=G_va*(vm_a+O_va);
   	vm_b=G_vb*(vm_b+O_vb);
   	vm_c=G_vc*(vm_c+O_vc);

   	//Se guardan las variables para graficarlas
   	VA=vm_a;
   	VB=vm_b;
   	VC=vm_c;

   	//Se asegura que no excedan los maximos
   	/*
   	vm_a<0?vm_a=0:vm_a;
   	vm_a>Vmax?vm_a=Vmax:vm_a;
   	vm_b<0?vm_b=0:vm_b;
   	vm_b>Vmax?vm_b=Vmax:vm_b;
   	vm_c>Vmax?vm_c=Vmax:vm_c;
   	vm_c<0?vm_c=0:vm_c;*/

   	//Se guarda 1 periodo en el buffer
   	//Se normalizan los valores
   	va_b=vm_a/Vmax;
   	vb_b=vm_b/Vmax;
   	vc_b=vm_c/Vmax;

   	//Se dejan los valores entre 0 y 2500 para poder usarlos en el comparador
   	//del pwm, en lugar de la tabla sine
  	va_buff[j]=-1250*(va_b-1);  //Se generan las tablas de seno para cada fase
  	vb_buff[j]=-1250*(vb_b-1);
  	vc_buff[j]=-1250*(vc_b-1);

   	angulo_alpha=j;
   	j++;

   	if(j>359)
   	{
   		j=0;
   	}
   	GpioDataRegs.GPBCLEAR.bit.GPIO32 = 1;
   // The CPU acknowledges the interrupt.
   PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}
