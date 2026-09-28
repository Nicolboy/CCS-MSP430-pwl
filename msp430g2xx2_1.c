//******************************************************************************
//  PWL - alimentation fantome 48 V sur batterie, MSP430G2252
//
//  Preampli audio alimente sur batterie. Elevation vers 48 V par DC/DC
//  polyphase, surveillance ADC multivoies, etalonnage sauvegarde en flash,
//  protection de batterie a trois seuils puis veille profonde.
//
//  Copyright (c) 2019-2026 Nicolas Boyer
//  Licence MIT - voir LICENSE
//
//  Journal de version ci-dessous.
//******************************************************************************

        // demarrage, (PWM pulse sur SYNC P2.4 TA1.2 puis) basculer alim5V sur SWITCH P2.7 et PWM pulse sur OSC3 P2.6 TA0.1
        // gestion multi ADC P1.0 48V, P1.1 Batt, P1.2 Vcc mesurer tensions
        // gestion flash, au démarrage tester si flash ecrite
        // gestion polyphase DC/DC decaler les phases a 180°
        // (flash vierge), procedure de calibration: lancer ACTIVE P1.6 puis lancer 48V avec OSC2 P2.2 TA1.1 mesurer 48V sous 10kOhms, tant que 44V>48V>43V diminuer timer PWM CALIB P2.3 TA1.0 et sauver valeur
        //                  emettre signal lumineux (double led)
        // (flash ecrite), restaurer parametre et lancer PWM CALIB puis OSC2
        //          lorsque 48V ok, lancer l'audio preamp avec ACTIVE P1.6
        //  boucle principale
        //      surveiller les tensions, TA0.0 triggered start pour conversion ADC
        //      si 48V>=44V alors clignoter bleu
        //      si Vbatt<1,1V clignotter rouge
        //      si Vbatt <1,05V clignoter vite rouge
        //      si Vbatt <1V sorti boucle principale,
        // arreter audio, arreter 48V et mettre led rouge fixe, passer mode veille profonde
//
// ce programme utilise le timer TA0.0 pour déclencher une conversion toute les 1,5s OK
// ADC conversion  sur 3 canaux pilotée par le timer TA0.0 OK
// sauvegarde et lecture autocalibration en Flash OK
// polyphase Timer TA0.1 et TA1.1 et TA1.2 ok
// autocalibration a tester
// clignotement et controle tension OK
// fin de programme OK
// autocalibration 20190813 OK
// soft start 48V 20190730 OK
// controle de l'audio et des oscillateurs pour PWL mk2.4 2.5 et 2.6
// nouvelle version sans autocalibration mais référence de tension 20200927 OK
// nouvelle version sans oscillateurs polyphase pour revenir vers g2452 14 pins
// 20220804 controle direct par le µc du basculement d'oscillateur
// version g2252 pour PWL 2.13
//******************************************************************************
#include <msp430.h>

// declarations

/* definitions des entrees sorties */

#define ADCBATT                 BIT0        // input ADC P1.0
#define ADC48V                  BIT1        // input ADC P1.1
#define OUTLEDBLUE              BIT2        // input ADC P1.2
#define OUTLEDRED               BIT3        // input Digital P1.3
#define OSC5B_Disable           BIT4        // output P1.4 //will be soft start pwm
#define OSC_Switch              BIT5        // output P1.5 // will be multiplexed to osc5bdisable
#define AUDIO_Enable            BIT6        // output P1.6
#define Pwm48Low                BIT7        // output P1.7 // Pwm48Low, normaly input, force pwm to medium value if set to output 0. (was AUDIO_Power)

//#define   AUDIOPWR                BIT0        // output P2.0 audio power
//#define OUTOSCSWITCH          BIT1        // output P2.1
//#define OUTOSC5V              BIT2        // output P2.2
//#define   AUDIOACTIVE             BIT3        // output P2.3
//#define   OUTPWM5V                BIT4        // output P2.4
//#define   OSC5B_Enable            BIT5        // output P2.5
#define OSC48                   BIT6        // output P2.6
#define OSC48_Disable           BIT7        // output P2.7 // can do soft start?

#define TOFF                    1           // time led on

#define     ADC_CHANNELS        2               //We will sample 2 channels, 0 to 1, sample[0] is +48V, sample[1] is +Vbatt
#define     VPH                 680             //+40V OK was 740
#define     VBATT1              440                 //low batt 1,07v=438 1,1=450
#define     VBATT2              415                 //low batt 1V=410 1,03=420
#define     VBATTSTOP           385                 //stop batt 0,95V=388

#define     MEMORY              0x1040              // Flash memory start
#define     PWM_TIME_START      22                  // 352kHz PWM
#define     PWM_TIME            88                  // 88kHz PWM
#define     TARGET48V           809                 // calculé pour ref 2,5V et rapport resistance 100k 4.7k => 885 is 48V theorical, 1024 is 55,5V
#define     TARGET5V            1020                    // calculé pour ref 2,5V et rapport resistance 10k 10k => 1024 is 5V theorical, 1024 is 5V
#define     PWM_CALIB_DEFAULT   80      //44            // default value for calibration PWM duty cycle = 50% 88 = 100% si valeur augmente tension descend

volatile unsigned int i,j,autocalib,soft_start_ramp;
volatile unsigned int calibration_value = PWM_CALIB_DEFAULT;
int erreur;
unsigned int samples[2];                        // 2 adc channels
unsigned int Min, Max,Middle,Mesure48V;
char *Flash_ptr;                                // Flash pointer
char TEMPO = 1;                                 // temporisation ADC et clignotement LED

void AD_Conversion(void);
void AD_Init(void);
void flash_init (void);
void Save_Calibration_Flash (void);
void Port_Init(void);
void Clock_Init(void);
void Timer_Init(void);
void osc5Vswitch(void);
void osc5Vrelease(void);
void softstart(void);
void main_routine(void);
void shutdown(void);

int main(void)
{
// ********************************* DEBUT
    //+48V is disabled in routine 48vsoft start
//while(1);
        Port_Init();

        Clock_Init();

        Timer_Init();

        osc5Vswitch(); // start 5 Volt

          // powering audio
//        P1OUT |= AUDIO_Power;       // output active Pwm48Low

        AD_Init(); // init the AD

        AD_Conversion(); // discard first conversion //***********************

        softstart(); // +48V osc desactivated
//        while (1);
          // unmute audio output
        P1OUT |= AUDIO_Enable;      // output active debug

        main_routine();

        shutdown();

// ******************************* FIN
}

// ADC10 interrupt service routine
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=ADC10_VECTOR
__interrupt void ADC10_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(ADC10_VECTOR))) ADC10_ISR (void)
#else
#error Compiler not supported!
#endif
{
  ADC10CTL0 &= ~ENC;                        // ADC10 disabled
//  ADC10CTL0 = 0;                            // ADC10, Vref disabled completely *************

    __bic_SR_register_on_exit(LPM1_bits);        // Clear CPUOFF bit from 0(SR) **********
}

// Timer A0 interrupt service routine

#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=TIMER0_A0_VECTOR
__interrupt void Timer_A(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(TIMER0_A0_VECTOR))) Timer_A (void)
#else
#error Compiler not supported!
#endif
{
    return;
}

// ************************* routines
void AD_Conversion (void)
    {

    WDTCTL = WDTPW;
    //************************ note: remember to discard 1st conversion
    ADC10CTL0 &= ~ENC;
    while (ADC10CTL1 & ADC10BUSY);               // Wait if ADC10 core is active
    ADC10SA = (unsigned int) samples;           // Data buffer start, pointer
    __disable_interrupt();                      // Close window in case we get distracted by an interrupt/breakpoint
    ADC10CTL0 |= ENC + ADC10SC + ADC10IE;               // Sampling and conversion start
    ADC10CTL0 |= ENC + ADC10SC + ADC10IE;                       // Sampling and conversion start, I tell you again!
    __bis_SR_register(CPUOFF + GIE);            // LPM0, ADC10_ISR will force exit
    WDTCTL = WDTPW + WDTHOLD;                                       // stop WDT

    };

void AD_Init (void)
    {
      WDTCTL = WDTPW;
      // ********** ADC init

      /* disable ADC10 during initialization */
      ADC10CTL0 &= ~ENC;

      /*
       * Control Register 0
       *
       * ~ADC10SC -- No conversion
       * ~ENC -- Disable ADC
       * ~ADC10IFG -- Clear ADC interrupt flag
       * ADC10IE -- Enable ADC interrupt
       * ADC10ON -- Switch On ADC10
       * REFON -- Enable ADC reference generator
       * REF2_5V -- Set reference voltage generator = 2.5V
       * MSC -- Enable multiple sample and conversion i.e. permet de faire le bloc de conversions d'un coup
       * ~REFBURST -- Reference buffer on continuously
       * ~REFOUT -- Reference output off
       * ~ADC10SR -- Reference buffer supports up to ~200 ksps
       * ADC10SHT_0 -- 4 x ADC10CLKs
       * SREF_1 -- VR+ = VREF+ and VR- = VSS
       *
       * Note: ~<BIT> indicates that <BIT> has value zero
       */

      //  ADC10CTL0 = ADC10ON | SREF_1 | ADC10SHT_0 | ADC10IE | REFON | REF2_5V | MSC;
      ADC10CTL0 = ADC10IE | ADC10ON | REFON | REF2_5V | MSC | ADC10SHT_0 | SREF_1;

      /*
       * Control Register 1
       *
       * ~ADC10BUSY -- No operation is active
       * CONSEQ_1 -- Sequence of channels
       * ADC10SSEL_0 -- ADC10OSC
       * ADC10DIV_0 -- Divide by 1
       * ~ISSH -- Input signal not inverted
       * ~ADC10DF -- ADC10 Data Format as binary
       * SHS_0 -- ADC10SC bit trigerring
       * INCH_2 -- ADC Channel A2 to A0
       *
       * Note: ~<BIT> indicates that <BIT> has value zero
       */

    //  ADC10CTL1 = CONSEQ_1 | ADC10SSEL_2 | ADC10DIV_0 | SHS_1 | INCH_4;
      ADC10CTL1 = CONSEQ_1 | ADC10SSEL_0 | ADC10DIV_0 | SHS_0 | INCH_1;

      /* Analog (Input) Enable Control Register 0 */
      ADC10AE0 = 0x03; // 2 channels P1.0 P1.1

      /* Data Transfer Control Register 1 */
      ADC10DTC1 = 2;    // bloc size = 5 ADC values 16bits

      /* Data Transfer Start Address */
      ADC10SA = (unsigned int) samples;            // Data buffer start, pointer, mandatory

      /* Software delay for REFON to settle */
      __delay_cycles(3000);

      /* enable ADC10 */
//    ADC10CTL0 |= ENC;
     ADC10SA = (unsigned int) samples;              // Data buffer start, pointer, mandatory again, why?

     WDTCTL = WDTPW + WDTHOLD;                                      // stop WDT
    };

void flash_init (void)
{
    // **************************************************** flash memory init
     FCTL2 = FWKEY + FSSEL0 + FN1;             // MCLK/3 for Flash Timing Generator
     WDTCTL = WDTPW + WDTHOLD; // Set hold bit and clear others
     Flash_ptr = (char *) MEMORY;              // Initialize Flash pointer
}

void Save_Calibration_Flash (void)
{
      WDTCTL = WDTPW + WDTHOLD;                 // Stop watchdog timer

      if (CALBC1_1MHZ==0xFF)                    // If calibration constant erased
      {
        while(1);                               // do not load, trap CPU!!
      }
      DCOCTL = 0;                               // Select lowest DCOx and MODx settings
      BCSCTL1 = CALBC1_1MHZ;                    // Set DCO to 1MHz
      DCOCTL = CALDCO_1MHZ;

      FCTL2 = FWKEY + FSSEL0 + FN1;             // MCLK/3 for Flash Timing Generator

      Flash_ptr = (char *) MEMORY;              // Initialize Flash pointer
      FCTL1 = FWKEY + ERASE;                    // Set Erase bit
      FCTL3 = FWKEY;                            // Clear Lock bit
      *Flash_ptr = 0;                           // Dummy write to erase Flash segment

      FCTL1 = FWKEY + WRT;                      // Set WRT bit for write operation
      *Flash_ptr = 0;                           // byte == 0 => data have been saved flag
      Flash_ptr++;
      *Flash_ptr = 0;                           // byte == 0 => data have been saved flag
      Flash_ptr++;
      *Flash_ptr = autocalib;                       // save canal 1 value LSB
      Flash_ptr++;
      *Flash_ptr = autocalib >> 8;                      // save canal 1 value MSB

      FCTL1 = FWKEY;                            // Clear WRT bit
      FCTL3 = FWKEY + LOCK;                     // Set LOCK bit
}

// Watchdog Timer interrupt service routine
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=WDT_VECTOR
__interrupt void watchdog_timer(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(WDT_VECTOR))) watchdog_timer (void)
#else
#error Compiler not supported!
#endif
{
    if (j>TOFF)
    {
    P1OUT &= ~OUTLEDRED;                                        // all LEDs low
    P1OUT &= ~OUTLEDBLUE;                                       // all LEDs low
    }
    if ((j == TEMPO))
    {
    j = 0;
    __bic_SR_register_on_exit(LPM1_bits);        // Clear CPUOFF bit from 0(SR) **********
    }
    else
    {

    }
    j++;
}

void Port_Init (void)
{
      WDTCTL = WDTPW; // activate watchdog
      //************************************** GPIO configuration
      /* Port 1 Output Register */
      P1OUT = 0;
      P1OUT|= OSC5B_Disable;        // osc5B disabled
//    P1OUT |= OSC48V_disable; // disable 48V driver
//    P1OUT &= ~OSC48V_disable; // enable 48V driver
      /* Port 1 Direction Register */

// old      P1DIR |= OUTLEDRED + OUTLEDBLUE + OSC_Switch + OSC5B_Disable + AUDIO_Enable; Pwm48Low
      P1DIR |= OUTLEDRED + OUTLEDBLUE + OSC5B_Disable + AUDIO_Enable;

//    P1OUT &= ~OSC_Switch;         // osc not switched

//    P1DIR &= ~GANG;
      /* Port 1 Interrupt Edge Select Register */
      P1IES = 0;
      /* Port 1 Interrupt Flag Register */
      P1IFG = 0;
//    P1OUT = OSC_Switch + OSC5B_Enable;

      /* Port 2 Port Select Register */
      P2OUT = 0;
      P2OUT|= OSC48_Disable; //
//    P2SEL = OUTOSC5V | OUTPWM5V;                  // disable 48V
      P2SEL = OSC48;    // 48V enabled
      P2SEL &= ~OSC48_Disable;                              // P2.6 option select
      P2SEL2 &= ~(OSC48 + OSC48_Disable);                // P2.6 option select
      /* Port 2 Direction Register */
      // P2DIR = AUDIOPWR | OUTOSCSWITCH | OUTOSC5V | AUDIOACTIVE | OUTPWM5V | OSC5B_enable | OUTOSC48V; // main
      P2DIR|= OSC48; // disable OSC48
      P2DIR|= OSC48_Disable; // disable OSC48
      //P2DIR = AUDIOPWR | OUTOSCSWITCH; // for tests
      /* Port 2 Interrupt Edge Select Register */
      P2IES = 0;
      /* Port 2 Interrupt Flag Register */
      P2IFG = 0;
      WDTCTL = WDTPW + WDTHOLD;                 // Stop WDT
}

void Clock_Init (void)
{
    WDTCTL = WDTPW; // activate watchdog
    BCSCTL2 = SELM_0 | DIVM_0 | DIVS_0;
    BCSCTL3 = XT2S_0 | LFXT1S_2 | XCAP_1;
    if (CALBC1_8MHZ != 0xFF)
    {
      __delay_cycles(10000);
      // Follow recommended flow. First, clear all DCOx and MODx bits. Then
      // apply new RSELx values. Finally, apply new DCOx and MODx bit values.
      DCOCTL = 0x00;
      BCSCTL1 = CALBC1_8MHZ;
      DCOCTL = CALDCO_8MHZ;
    }
    BCSCTL1 |= DIVA_0; // pour le watchdog
    WDTCTL = WDTPW + WDTHOLD;                 // Stop WDT
}

void Timer_Init (void)
{
      WDTCTL = WDTPW; // activate watchdog
      // ********************************* TA0
      TA0CCTL1 = CM_0 | CCIS_0 | OUTMOD_3;
      TA0CTL &= ~(MC1 + MC0); // stop timer 0

      // initialisation des générateurs PWM
      TA0CTL = TASSEL_2 | ID_0 | MC_1; // activate timer 0
      TA0CCR0 = 0;
      TA0CCR0 = PWM_TIME;               // OSC48 active
      TA0CCR1 = 1;                          // activate osc 48
      WDTCTL = WDTPW + WDTHOLD;                 // Stop WDT
}

void osc5Vswitch (void)
{
    //******************************************************* change 5V oscillator
    P1DIR |= OSC_Switch;
    P1OUT &= ~OSC_Switch;                        // SWITCH 5V oscillator, low to switch
    P1OUT &= ~OSC5B_Disable;                                        // activate 5V driver
//  P1DIR |= OUTOSC5V + OUTOSCSWITCH + OSC5B_enable;                                            // output osc5V
    __delay_cycles(1000);
}

void osc5Vrelease (void)
{
    //****************** restart the primary 5V oscillator
//  P2DIR &= ~OUTOSC5V;                             // output osc5V as input
//      P2SEL &= ~OUTOSC5V;                             // set 2.3 PWM output as standard output stop 48V
//  P2OUT &= ~OUTOSCSWITCH;                         // SWITCH ready to activate 5V oscillator A 2.1 and disable 5V driver 2.5
//  P2OUT |= OSC5B_enable;                          // disable 5Vdriver
//  __delay_cycles(1000);
}

void softstart (void)
{

    // start 48V
      P2OUT&= ~OSC48_Disable; //

    IE1 |= WDTIE;                           // Enable WDT interrupt

    // ********************** demarrage led clignotent 10 fois chacune, deux fois
        P1OUT |= OUTLEDRED;
         for ( i = 15; i > 0; i--)
         {
            WDTCTL = WDTPW + WDTTMSEL + WDTSSEL + WDTCNTCL + WDTIS1 + WDTIS0;
            __bis_SR_register(LPM1_bits + GIE);
            P1OUT ^= OUTLEDRED;
            WDTCTL = WDTPW + WDTTMSEL + WDTSSEL + WDTIS1;
            __bis_SR_register(LPM1_bits + GIE);
         }

        P1OUT &= ~OUTLEDRED;
        P1OUT |= OUTLEDBLUE;

         for ( i = 15; i > 0; i--)
         {
            WDTCTL = WDTPW + WDTTMSEL + WDTSSEL + WDTIS1 + WDTIS0;
            __bis_SR_register(LPM1_bits + GIE);
            P1OUT ^= OUTLEDBLUE;
            WDTCTL = WDTPW + WDTTMSEL + WDTSSEL + WDTIS1;
            __bis_SR_register(LPM1_bits + GIE);
         }

        P1OUT &= ~OUTLEDRED;
        P1OUT &= ~OUTLEDBLUE;
        P1OUT |= OUTLEDRED;

         for ( i = 4; i > 0; i--)
         {
            WDTCTL = WDTPW + WDTTMSEL + WDTSSEL + WDTIS1 + WDTIS0;
            __bis_SR_register(LPM1_bits + GIE);
            WDTCTL = WDTPW + WDTTMSEL + WDTSSEL + WDTIS1 + WDTIS0;
            __bis_SR_register(LPM1_bits + GIE);
            WDTCTL = WDTPW + WDTTMSEL + WDTSSEL + WDTIS1 + WDTIS0;
            __bis_SR_register(LPM1_bits + GIE);
            WDTCTL = WDTPW + WDTTMSEL + WDTSSEL + WDTIS1 + WDTIS0;
            __bis_SR_register(LPM1_bits + GIE);
            P1OUT ^= OUTLEDRED;
            WDTCTL = WDTPW + WDTTMSEL + WDTSSEL + WDTIS1;
            __bis_SR_register(LPM1_bits + GIE);
            WDTCTL = WDTPW + WDTTMSEL + WDTSSEL + WDTIS1;
            __bis_SR_register(LPM1_bits + GIE);
            WDTCTL = WDTPW + WDTTMSEL + WDTSSEL + WDTIS1;
            __bis_SR_register(LPM1_bits + GIE);
         }

        P1OUT &= ~OUTLEDRED;
        P1OUT |= OUTLEDBLUE;

         for ( i = 4; i > 0; i--)
         {
            WDTCTL = WDTPW + WDTTMSEL + WDTSSEL + WDTIS1 + WDTIS0;
            __bis_SR_register(LPM1_bits + GIE);
            WDTCTL = WDTPW + WDTTMSEL + WDTSSEL + WDTIS1 + WDTIS0;
            __bis_SR_register(LPM1_bits + GIE);
            WDTCTL = WDTPW + WDTTMSEL + WDTSSEL + WDTIS1 + WDTIS0;
            __bis_SR_register(LPM1_bits + GIE);
            WDTCTL = WDTPW + WDTTMSEL + WDTSSEL + WDTIS1 + WDTIS0;
            __bis_SR_register(LPM1_bits + GIE);
            P1OUT ^= OUTLEDBLUE;
            WDTCTL = WDTPW + WDTTMSEL + WDTSSEL + WDTIS1;
            __bis_SR_register(LPM1_bits + GIE);
            WDTCTL = WDTPW + WDTTMSEL + WDTSSEL + WDTIS1;
            __bis_SR_register(LPM1_bits + GIE);
            WDTCTL = WDTPW + WDTTMSEL + WDTSSEL + WDTIS1;
            __bis_SR_register(LPM1_bits + GIE);
         }

        WDTCTL = WDTPW + WDTHOLD; // Set hold bit and clear others, stop the watchdog
        P1OUT &= ~OUTLEDRED;
        P1OUT &= ~OUTLEDBLUE;
}


void main_routine(void)
{
    // ************************************** main loop
      while (1)
      {

          WDTCTL = WDTPW + WDTHOLD; // Set hold bit and clear others, stop the watchdog

          AD_Conversion ();

//        samples[1] = 500; // dummy for tests
//        samples[0] = 1000; // dummy


    //************************************* calcul des valeurs

            if (samples[1]<VBATTSTOP) break;                        // stop, jump out of the main loop

            if (samples[1]>VBATT1)                                  // compare ADC input P1.0 to VBATT
            {
                P1OUT &= ~OUTLEDRED;
            }
            else
            {
                P1OUT |= OUTLEDRED;
            }

            if (samples[0]>VPH & samples[1]>VBATT1)                 // compare ADC input P1.1 to VBATT
            {
                P1OUT |= OUTLEDBLUE;
            }
            else
            {
                P1OUT &= ~OUTLEDBLUE;
            }

    //************************************** clignotement a 2 vitesses

            if (samples[1]>VBATT2)
                {
                TEMPO = 8; //8
    //              j = 10;
              WDTCTL = WDTPW + WDTTMSEL + WDTSSEL + WDTCNTCL + WDTIS0;              // prepare watchdog
                }
            else
                {
                TEMPO = 3;
    //              j = 10;
              WDTCTL = WDTPW + WDTTMSEL + WDTSSEL + WDTCNTCL + WDTIS0;      // prepare watchdog
                }
              IE1 |= WDTIE;                             // Enable WDT interrupt
                BCSCTL1 |= DIVA_0; // pour le watchdog
            __bis_SR_register(LPM1_bits + GIE);                     // low power mode 1
      }
}



void shutdown(void)
{
      //************************** stop converter if batt too low

        P1OUT &= ~OUTLEDBLUE;
//        P1OUT &= ~OUTLEDRED;
        P1OUT |= OUTLEDRED;                                             // red batt LED on
//      P2SEL &= ~OUTOSC48V;                                            // set 2.2 PWM output as standard output stop 48V
        P2OUT |= OSC48_Disable;                                         // disable 48V PWM driver
      //    TA0CTL &= ~(MC1 + MC0);                                     // Clear MCx bits to stop timer

//      osc5Vrelease ();

      // deactivate audio output and stop the audio power high
            P1OUT &= ~AUDIO_Enable;         // mute audio outputs ??????????

        P1DIR |= Pwm48Low;
        P1OUT &= ~Pwm48Low;         // force PWM


      // shutdown processor
        WDTCTL = WDTPW + WDTHOLD;                                       // stop WDT
        __bis_SR_register(LPM1_bits);                                   //low power mode 1 a desactiver si debug
}


