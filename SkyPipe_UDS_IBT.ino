/************************************************************************************************
  2018.10.16 02:30
  IBT demo radio telescope sketch with UDS connection to Radio-SkyPipe.
  The UDS command processor came form RqadioSky web site.  Added assorted mods.
        Bruce Randall NT4RT  July 10, 2018,  Sept 27, 2018
        Oct 10, 2018  Major I/O schuffle and TIMER1 druven tone process
        Oct 11, 2018 ADC driven by TIMER2 IRQ for accurate timing.
        Oct 12, 2018 added full interpolation to PWR table lookup
        Oct 14, 2018 added "volitile" to all IRQ variables. Removed do nothing code
  In PUSH mode, data is sent every 100 milliSeconds.
  in POLL mode, Data request should not be closser than 100 milliSeconds
  WARNING:  This sketch does NOT fully decode SkyPipe UDS commands.  Any added commands
  to UDS could cause trouble!!!

  Normal Full sacle signal in all processes is 16383
***************************************************************************************************/

#define debug false             // Set true to debug with a terminal in place of SkyPipe.  
// WARNING!  SkyPipe will NOT work if debug is true. IT CRASHES!!!

#include <EEPROM.h>             // Allow EEPROM usage
#include "Exp_Lookup.h"         // Table to undo log detector response to linear temperature

// Define  MCU pins ********** Schuffled for TIMER1 tone output Oct 10, 2018
#define EncoderPinA       3     // Input
#define EncoderPinB       4     // Input
#define Meter_pin         5     // PWM output for analog meter.    
#define Offset_pin        6     // PWM for offset voltage.
#define DetSel_N          7     // Output. Low Selects Audio path for detector 
#define EncoderPush       8     // Input from push switch on encoder  
#define ToneOutAF         9     // Tone Warning: This prevents pins 9 and 10 PWM
#define AudioSelPush      10    // Inputfrom push button switch 
#define LUT_pin           11    // Input.  High for LUT meter output, low for linear.
#define FW_T_pin          12    // Output Pin wiggles at ADC routine.  Test Point ONLY!!
#define LED_Pin           13    // LED on this  pin on UNO PWB
#define DetectorV         0     // Amalog Input A0

// Timing stuff sets ADC and output sample rates 
#define TMR2_Period       124   // OCR value for 2.00mS cycle.  1 / 16E6 * 256 * (124+1) = 2.00mS
#define Ticks             50    // This is the number of adc reads per out sample in PUSH mode
//  2.00mS thru loop * 50 passes = 100mS
int count100mS = 0;             // counter for 50mS and 100mS internal tasks and 100mS output

boolean poll = true;            // if true then data is polled by RSP using a GETD command
int stat = 0;                   // -1 = we were just stopped by a KILL command 0 = startup state
// 1 = INIT rcvd 2 = Ready to Go 3= Running

byte audioSel = 0;              // Select Audio Path.  Possible values below.  Saved in EEPROM
#define No_AF   0                     // 0 = no audio
#define Tone_AF 1                     // 1 = tone audio
#define Det_AF  2                     // 2 = detector audio
byte audioSelZ   = 0;           // previous state of audioSel
#define InitTone 1600           // Zero level tone frequency, Hz * 8
#define Pitch700 30260          // Load to make 700Hz tone with TIMER1
#define K_T1_FP  64062000       // Freq to Period for 16383 data = 2000 Hz

volatile
unsigned int t1Period = Pitch700; // Used in TIMER1 tone generation


int sw_Count = 0;               // counts 100mS ticks for EEPROM write command
#define SW_Detect 12                  // number of ticks to activate EEPROM write 

volatile int adcBox = 0;                 // holds 10 bit ADC data from IRQ routine
volatile boolean adcFlag = false;        // true when adcBox has new data from IRQ
int adcFiltOut14 = 0;           // Output of IIR filter in ADC routine 14 bit wide
#define ADCFiltK_Sel 5          // value 5 gives 2.0mS *(2^5) = about 64mS RC

#define TauFilK_Min 1               // lowest safe value 1 gives 50mS *(2^1) = about 100mS RC 
#define TauFilK_Max 9               // highest svalue 9 gives 50mS *(2^9) = about 26 Sec
#define TauFilK_PWM 25              // calc from floor(255/(Max-Min)), slightly lower is OK (displa filt setting)
byte tauFilK_Sel = TauFilK_Min; // Range limit MUST be checked! save in EEPROM

int dataBus14 = 0;              // main data for output devices

int incomingByte;               // from serial port
byte af_Sw, af_Sw_Z = HIGH;     // Audio Sel & previous state of it

byte encoderA, encoderB, encoderZ = 1;    //"Z" is previous state
byte offset_Set = 128;          // saved in EEPROM

byte eeData = 0;                // used in EEPROM process
#define EEADR_Key 0                 // start of EEPROM
#define EE_Key_Value 0x52           // Value in key to establish EE data is valid
#define EEADR_Ofst 1                // EE Address of offset
#define EEADR_Filk 2                // EE Address of filter K
#define EEADR_ASel 3                // EE Address of Audio Select

// ***********************************************************************************
// ************************** Start Programme ****************************************
void setup() {                  //  UDS stat, poll are inialized in assignment
  InitPort();                   // Setup all I/O ports
  Serial.begin(9600);           // connect to the serial port
  while (!Serial) ;             // wait for serial port to connect. Needed for native USB port only.
  if (debug) {
    Serial.print("^^1002 Arduino UDS ");
    Serial.write(255);
    Serial.println("");
  }
  GetEE();
  TIMER_setup();                // Setup TIMER1 IRQ for tone gen. Setup TIMER2 IRQ driven ADC.
}
//***********************************************************************************
// TIMER1 IRQ for tone generation
ISR(TIMER1_COMPA_vect)          // timer compare interrupt service routine
{
  OCR1A = t1Period;             // tone vaue => counter update
}

//***********************************************************************************
// TIMER2 IRQ for IRQ driven ADC.  IRQ is every 2mS.  This process takes 115uS
ISR(TIMER2_COMPA_vect) // timer compare interrupt service routine
{
  digitalWrite(LED_Pin, HIGH);          // test point to observe timing
  adcBox = analogRead(DetectorV);
  adcFlag = true;
  digitalWrite(LED_Pin, LOW);          // test point to observe timing
}
//***********************************************************************************
//***********************************************************************************
void loop() {
  /**********************************************************************************
    If we are pushing the data to RSP then we need to establish our timing for
    sending new data.  Here we are just doing a delay of about 100ms to get a
    sample rate of about 10 samples per second.
  ************************************************************************************/
  // * * * * * * * * * * * * * * Task done every 2 mSec * * * * * * * * * * * * * *
  // Timing is set ADC_Process which waits for TIMER2 to trigger ADC.
  ADC_Process();       // get ADC data from IRQ and filter. Result in IIR filter
  CmdProcess();        // check for command from Radio SkyPipe
  ENC_Process();       // check for encoder knob changes & process

  // Check for timer task queue up or timeout.
  count100mS++;                           // counter is NEVER 0 after ++
  // * * * * * * * * * * * * * * Task done every 50mSec * * * * * * * * * * * * * *
  if ((count100mS == (Ticks / 2 + 1 )) ||
      (count100mS == 1))     {            // every 50mS updates
    if (digitalRead(LUT_pin) == HIGH) {                  // Lookup table select input pin
      dataBus14 = GetPwrData();           // Change LOG data to PWR data, if switch set.
    }  //x LUT_pin
    else   dataBus14 = adcFiltOut14;

    dataBus14 = TauFilter(dataBus14);     // do "integrator" RC filter
    Set_Tone();                           // 50mStone update needed for smoothness
  }

  // * * * * * * * * * * * * * * Task done every 100mSec * * * * * * * * * * * * * *
  if (count100mS >= Ticks) {            // task done on the 100mS tick
    count100mS = 0;
    ReadSwitches();                     // read push button switches
    Set_PWM();                          // update PWM's outputs
    if (( !poll ) && (stat == 3)) {     // time to PUSH data if PUSH mode
      GETD();                           // send data to PC => SkyPipe
    }
  }
  // * * * * * * * * * * * * * * End of Task done every 100mSec
}    //**** end of main "loop"

// *****************************************************************************************
// ************** functions used by main programe ******************************************

//*****************  command processor *******************
void CmdProcess() {
  while (Serial.available() > 0) { // very long while loop.
    incomingByte = Serial.read();  // read the oldest byte in the serial buffer:

    // **** if it's a K we stop (KILL):
    if (incomingByte == 'K') {
      DumpChar(3);  //GET PAST THE REST OF THE WORD by Reading it.
      stat = -1;
      if (debug) Serial.println("^^1002 DEAD"); // Just for troubleshooting
    }

    // **** if it's an I run the INIT code if any
    if ((incomingByte == 'I') && (stat == 0)) {
      DumpChar(3);  // GET RID OF 'NIT'
      stat = 1 ;
      if (debug) Serial.println("^^1002 INITIALIZED ");
    }

    // **** if it's an L RSP will have to POLL for data
    if (incomingByte == 'L') {
      DumpChar(1); // GET RID OF 2nd 'L'
      poll = true;
      if (debug) Serial.println("^^1002 POLLING ");
    }

    //**** if it's an H sets it to push.  Arduino will PUSH data.
    if (incomingByte == 'H') {
      poll = false;
      if (debug) Serial.println("^^1002 PUSHING ");
    }

    //**** if it's a C then Radio-SkyPipe is requesting number of CHANnels
    if (incomingByte == 'C') {
      DumpChar(3);  // GET RID OF 'HAN'
      // change the last digit to = digit of channels of data (ex. 1)
      delay(10);
      Serial.print("^^20131");
      Serial.write(255);              // print result;
      stat = 2;                       // ready to go
      if (debug) Serial.println("");
    }

    //****  if it's an "A" means STAT was requested so send UDS ready message
    if (incomingByte == 'A' ) {
      DumpChar(1);                    // GET RID of final 'T'
      Serial.print("^^1001");
      Serial.write(255);              // print result
      stat = 3;
      if (debug) Serial.println("");
    }

    //***** if it's an D we should send data to RSP:
    if ((incomingByte == 'D') && poll && (stat == 3) ) {
      GETD() ;
      if (debug) Serial.println(" DATA REQUEST RECEIVED ");
    }

    if (stat == -1) stat = 0;     // clear kill state automatically
  }   //**** End of "while (Serial.available() > 0)"
  // we are finished processing any incoming commands from the PC
}     //**** end of function cmdProcess

/**********************************************************
     Initial setup of all I/O ports
*/
void InitPort() {
  pinMode(ToneOutAF, OUTPUT);
  T1_Tone_Off();                      // turn off TIMER1 output of tone

  // **** Set up rotary encoder
  pinMode(EncoderPinA, INPUT);
  digitalWrite(EncoderPinA, HIGH);    // internal pull-up enabled
  pinMode(EncoderPinB, INPUT);
  digitalWrite(EncoderPinB, HIGH);    // internal pull-up enabled
  pinMode(EncoderPush, INPUT);
  digitalWrite(EncoderPush, HIGH);    // internal pull-up enabled
  pinMode(LUT_pin, INPUT);
  digitalWrite(LUT_pin, HIGH);        // internal pull-up enabled
  pinMode(AudioSelPush, INPUT);
  digitalWrite(AudioSelPush, HIGH);   // internal pull-up enabled

  digitalWrite(DetSel_N, HIGH);       // preset to disable state, stop noise burst at powerup
  pinMode(DetSel_N, OUTPUT);          // Low selects detector audio
  pinMode (FW_T_pin, OUTPUT);
  digitalWrite(FW_T_pin, LOW);        // test point for firmware timing.
  // analogReference (EXTERNAL);        // select 3.3V reference ### probly a bad idea!
  // analogRead (0);
  pinMode(LED_Pin, OUTPUT);
}

/**********************************************************
   Set up TIMER1 for High resolution tone .
*/
void TIMER_setup() {
  // Setup timer IRQ for tone generation.
  // WARNING: This steals Timer1 from PWM's on Pins 9 and 10.  Also breaks servo library.
  noInterrupts();               // disable all interrupts
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;

  OCR1A = Pitch700;             // compare register
  TCCR1B |= (1 << WGM12);       // CTC mode
  TCCR1B |= (1 << CS10);        // 1x prescaler

  TIMSK1 |= (1 << OCIE1A);      // enable timer compare interrupt
  TCCR1A |= (1 << COM1A0);      // pin 9 gets timer toggle output
 
  //   Set up TIMER2 for ADC interupt
  // WARNING: This steals Timer2 from PWM's on Pins 3 and 11.  Also breaks tone() function
  TCCR2A = 0;
  TCCR2B = 0;
  TCNT2  = 0;
  OCR2A  = TMR2_Period;         // compare register
  TCCR2A |= (1 << WGM21);       // CTC mode
  TCCR2B |= ((1 << CS21) | (1 << CS22));        // 256x prescaler & start clock
  TIMSK2 |= (1 << OCIE2A);      // enable timer compare interrupt
  interrupts(); // enable all interrupts
}

/**********************************************************
   This is where data is fetched and sent on to RSP.
*/
void GETD() {
  Serial.print("#0");         // # followed by channel number of data
  Serial.print(dataBus14);    // Scale OK,  Fullscale = 16383
  Serial.write(255);
  Serial.print("^^3001");     // This tells RSP to time stamp it
  Serial.write(255);          // all commands end with this character.
  if (debug)Serial.println("");
  return;
}

/*********************************************************************
   This pulls cahracters from serial port buffer & throws them away.
   It is needed because we decode only one character of commands.
*/
void DumpChar(int charCount) {
  int loops;
  for ( loops = 1; loops <= charCount; loops++ ) {
    delay(10);
    incomingByte = Serial.read();
  }
  incomingByte = 0;
}

/****************************************************************************************
   ADC_Process to global output variable adcFiltOut14.  Output full scale = 16368
   Gets IRQ ADC data from adcBox, and scales by 16x to filtered out in adcFiltOut14 global.
   Filter is 1 pole IIR low pass with shifts making filter K.  About 64mS RC 
   Process takes 23uSec after adcFlag becomes true.
   
   Waits here until flaq from ADC IRQ Process.  NOT BEST BUT WORKS !!!
   .. Changed Internal calcs to 32 bit to avoid round off errors.       July 29, 2018
   .. Changed to TIMER2 IRQ timing of ADC.                              Oct 11, 2018                   
*/
void ADC_Process() {
  long adcFiltIn;                       // IIR filter processes ADC reads to smoothed value
  static long adcFiltMem;               // Memory for IIR filter

// digitalWrite(FW_T_pin, HIGH);         // test point for firmware timing.
  while (!adcFlag);                     // WAIT HERE UNTIL ADC HAS DATA, adcFlag == true
  digitalWrite(FW_T_pin, HIGH);         // test point for firmware timing.
  adcFiltIn = adcBox << 4;              // get ADC *16.  Now 16368 full scale
  adcFlag = false;                      // tell IRQ process we got data
  adcFiltIn = adcFiltIn << 15;          // scale by 2^15 to use long data type
  adcFiltIn -= adcFiltMem;              // get delta step in
  adcFiltIn = adcFiltIn >> ADCFiltK_Sel; // fixed filter K for ADC
  adcFiltMem += adcFiltIn;              // filtered output in analogFilOut
  adcFiltOut14 = (int)(adcFiltMem >> 15);  // scale back to fit in int data type
  digitalWrite(FW_T_pin, LOW);          // test point for firmware timing.
}
/******************************************************************************************
  Takes input from global analogFiltOut.  Full Scale here is 16368 input and output.
  The exponential output from log input is done with a 1024 length look up table.
  uses interpolation for full 14 bit operation.
*/
int GetPwrData() {
  int indx, tOut,tOut2,delta,out, testH;
  indx = (adcFiltOut14 >> 4);                       // 0 ... 1023 here is index
  if (indx < 0)    indx = 0;                        // stay in table!!
  if (indx > 1023) indx = 1023;                     // Table in PROGMEM space
  tOut  = pgm_read_word_near(&lookup[indx]);        //  "= lookup[indx]" DOES NOT WORK
  tOut2 = pgm_read_word_near(&lookup[indx+1]);
  delta = tOut2-tOut;                               // slope info 
  testH = adcFiltOut14 & 0x0F;                      // get  lsb's for interpolate test
  testH *= delta;
  testH = testH >> 4;
  out = tOut+testH;
  return out;
}

/****************************************************************************************
   TauFilter is basic RC time constant for Radio Telescope. NOTE STATIC MEMORY!!!
   Output full scale = 16368
   Main loop will normally run this about every 50mS.
   Internal calcs are 32 bit to avoid round off errors,  Created Sept 27, 2018
*/
int TauFilter(int input) {
  long rcFiltIn;
  static long rcFiltMem;              // Memory for IIR filter
  rcFiltIn = input;                   // put iput in 32 bits
  rcFiltIn = rcFiltIn << 15;          // scale by 2^15 to use long data type
  rcFiltIn -= rcFiltMem;              // get delta step of input
  rcFiltIn = rcFiltIn >> tauFilK_Sel; // Divide it by typical 32 for filter K of 1/32.
  rcFiltMem += rcFiltIn;              // filtered output in analogFilOut
  return (int)(rcFiltMem >> 15);        // scale back to fit in int data type
}

/****************************************************************
  Read shaft encoder and process changes
  Changes offfset value for ADC preamp via a PWM
  If knob is pushed in,  Changes IIR filter time constant
*/
void ENC_Process() {
  encoderA = digitalRead(EncoderPinA);
  encoderB = digitalRead(EncoderPinB);
  if ((encoderA == HIGH) && (encoderZ == LOW)) // detect edge change
  {
    if (encoderB == LOW)                      // EncoderB sorts CW vs CCW
    { // Decrease offset or filter K
      if ( digitalRead(EncoderPush) == HIGH)  // NOT pushed
        offset_Set--;                         // Change offset down
      else tauFilK_Sel--;                    // Change to shorter RC time equiv
    }
    else
    { // Increase offset or filter K
      if ( digitalRead(EncoderPush) == HIGH)  // NOT pushed
        offset_Set++;                         // Change offset up
      else tauFilK_Sel++;                    // Change to longer RC time equiv
    }

    // Check limits here for offset size and filter K size
    // Both are byte size values
    if ( offset_Set < 1 )   offset_Set = 1;
    if ( offset_Set > 254 ) offset_Set = 254;

    if (  tauFilK_Sel < TauFilK_Min) tauFilK_Sel = TauFilK_Min;
    if (  tauFilK_Sel > TauFilK_Max) tauFilK_Sel = TauFilK_Max;
  }
  encoderZ = encoderA;            // to catch state change NEXT time
}

/****************************************************************
  Read push button switches and process changes.  In 100mSec loop.
  Note that a pushed switch is LOW !!!
  AF routing switch.
*/
void ReadSwitches() {
  byte i;
  af_Sw = digitalRead(AudioSelPush);
  if ( (af_Sw == LOW) && (af_Sw_Z == HIGH)) {   // if High to Low edge detected on pin
    audioSelZ = audioSel;                       // to restore after EEPROM write
    audioSel++;
    if (audioSel > Det_AF) audioSel = No_AF;
  }                                             //  x if High to Low edge detected
  af_Sw_Z = af_Sw;                      // setup for NEXT edge detect

  if (af_Sw == HIGH) sw_Count = 0;      // for EEPROM
  if (af_Sw == LOW) {                   // Switch pushed ?
    sw_Count++;
    if (sw_Count > SW_Detect) {         // if a Long Push then do EEPROM write
      audioSel = audioSelZ;             // so EEPROM write dowes not change state
      PutEE();
      for (i = 0; i < 3; i++) {         // Make 3 beeps
        t1Period = Pitch700;            // Need to set pitch for 700 Hz here
        T1_Tone_Off();                  // turn off TIMER1 output of tone
        delay(100);
        T1_Tone_On();                   // turn off TIMER1 output of tone
        delay(300);
        T1_Tone_Off();                  // turn off TIMER1 output of tone
      }                                 // x Make 3 beeps
      while (af_Sw == LOW) {            // wait for pin HIGH to exit (Push released)
        af_Sw = digitalRead(AudioSelPush);
      }                                 // x wait for HIGH to exit
    }                                   // x if a Long Push then do EEPROM write
  }                                     // x Switch pushed ?
}                                       // x function ReadSwitches()

/****************************************************************
  Set Tone Pitch from Global dataBus14 value of 16383 Full Scale
  possible range of about 200 to 2200Hz.
*/
void Set_Tone() {
  long toneFreq8L, tempIL;
  switch (audioSel) {                          // Do audio first
    case No_AF:
      T1_Tone_Off();                      // turn off TIMER1 output of tone
      digitalWrite(DetSel_N, HIGH);
      break;
    case Tone_AF:
      digitalWrite(DetSel_N, HIGH);
      toneFreq8L = dataBus14 + InitTone;     // freq scaled by 8x
      tempIL = K_T1_FP / toneFreq8L;
      t1Period = (unsigned int)tempIL;       // back to 16 bits for timer
      T1_Tone_On();
      break;
    case Det_AF:
      T1_Tone_Off();                      // turn off TIMER1 output of tone
      digitalWrite(DetSel_N, LOW);            //Select detector AF
      break;
  }
}
/****************************************************************
  Enable timer1 tone output.
*/
void T1_Tone_On() {
  pinMode(ToneOutAF, OUTPUT);     // turn on TIMER1 output of tone
}

/****************************************************************
  Disable timer1 tone output.
*/
void T1_Tone_Off() {
  pinMode(ToneOutAF, INPUT);      // turn off TIMER1 output of tone
}

/****************************************************************
  Set PWM from Output data value.
*/
void Set_PWM() {
  int tempI;
  if ( digitalRead(EncoderPush) == HIGH)  {             // NOT pushed, Output = PWM of signal
    analogWrite(Meter_pin, dataBus14 >> 6);            // 8 bit data to PWM
  }
  else  {                                               // Knob pushed, Output filter K displayed
    tempI = ( tauFilK_Sel - TauFilK_Min) * TauFilK_PWM;      // Display & filt OK
    analogWrite ( Meter_pin, tempI );                   // shows log2(filt K)
  }
  analogWrite ( Offset_pin, offset_Set );
}

/****************************************************************
  Gets EEPROM data.  If key is set, puts data in proper variables.
  Must match PutEE.
*/
void GetEE() {
  eeData = EEPROM.read(EEADR_Key);
  if (eeData == EE_Key_Value) {                 // if there is data, get it
    offset_Set = EEPROM.read(EEADR_Ofst);
    tauFilK_Sel  = EEPROM.read(EEADR_Filk);
    if ( tauFilK_Sel < TauFilK_Min) tauFilK_Sel = TauFilK_Min;  // Check.  Wrong value is a mess!
    if ( tauFilK_Sel > TauFilK_Max) tauFilK_Sel = TauFilK_Max;
    audioSel = EEPROM.read(EEADR_ASel);
  }
}

/****************************************************************
  Puts data in EEPROM.  Includes key.
  Must match GetEE.
*/
void PutEE() {
  EEPROM.write(EEADR_Key,  EE_Key_Value);
  EEPROM.write(EEADR_Ofst, offset_Set);
  EEPROM.write(EEADR_Filk, tauFilK_Sel);
  EEPROM.write(EEADR_ASel, audioSel);
}

// ****************** end of programme ***********************************************************
