#include <Event.h>
#include <Timer.h>


#include <LiquidCrystal.h>
#include <LcdBarGraph.h>
#include <stdio.h>



// Debug Modus, zusätzliche Ausgaben in der seriellen Konsole
//#define DEBUG

// Definitionen
// Encoder, PINA muss interuptfähig sein
#define PINA 21
#define PINB 22

// Taster, muss interruptfähig sein
#define PUSHB 20

// analoge Input
#define PIN_A1 A0
#define PIN_A2 A1

//LCD Display

#define LCD_RS 27
#define LCD_EN 28
#define LCD_D4 29
#define LCD_D5 30
#define LCD_D6 31
#define LCD_D7 32

// ENCODER und Interrupt Kram
#define PINA_INPUT INPUT
#define PINB_INPUT INPUT
#define PUSHB_INPUT INPUT

//#define PINA_INPUT INPUT_PULLUP
//#define PINB_INPUT INPUT_PULLUP
//#define PUSHB_INPUT INPUT_PULLUP


#define PINA_INT  FALLING
#define PUSHB_INT FALLING


// Auskoppeldämfpung in dB
#define ATT_KOPPLER1 50
#define ATT_KOPPLER2 30
// welche Ref. für den AD Wandler soll genutzt werden?
#define REFERENCE INTERNAL2V56


// Limits, Index des Struct Arrays
#define LOW_LIMIT 16 // entspricht -70dBm
#define HIGH_LIMIT 2 // entspricht 0dBm

// Werte fuer Frequenzkorrektur
// beziehen sich auf Index des Struct Array
#define QRG_HIGH_CORR 10 // -40dBm
#define QRG_LOW_CORR 3 // -5dBm

#define QRG_FACTOR 0.020469
#define QRG_HIGH_FACTOR 244.8
#define QRG_LOW_FACTOR 86.4

// Kalibrierung
#define CALIBRATE 4 // 4. Eintarg in limM0 --> -10dB

#define QRG 5 // Test Frequenz MHz

// Anzeigeformate
// 2 --> 3 Darsatellungen 0,1,2
#define MAXSCREEN 2 




// Startmeldungen
#define START1 "   PowerMeter"
#define START2 "   1MHz ... 10GHz"
#define START3 "  K08 DL2FW/DM6TT"
#define START4 "  ---------------"


//Linearisierungstabelle für den Messverstärker [db vs. rawU]
//Arrays für die Interpolationn der Messwerte wird in der setup() Routine definiert
//int linM0[17][17];


// Skalierungsfaktor zur Umrechnung der Messwerte, wir später beim kalibrieren ermittelt
//float scalFactor1 = 1.0;
float scalFactor1 = 1.397;
//0.7877 ;
float scalFactor2 = 1.39 ;

int frequenz= QRG; // Frequenz in MHz


const float smooth = 0.004;
//> timeconstant=200ms (app. 1/smooth * TA (10ms)) @smooth=0.01-> timeconstant=1000ms ... - correct for smooth << 1

struct LinStruct {
  int dB;
  float ADnorm;
};

struct LinStruct linM0[17];

// Messbereichswahl
/*
struct MBStruct {
    float minP;         // Bereich Anfang
    float maxP;         // Bereich Ende
    int tLen;           // min. Zeitraum
    byte Nachkomma;     // anzuzeigende Nachkommastellen
    float Divisor;     // Devisor für die Darstellung
    String Unit;        // Einheit
};
*/
struct MBStruct {
    float range;         // Bereich Anfang= *0.05 Ende *0.8
    int tLen;            // min. Zeitraum
    byte Nachkomma;      // anzuzeigende Nachkommastellen
    float Divisor;       // Devisor für die Darstellung
    char Unit[10];        // Einheit
    char Text[10];        // Anzeigetext des Bereiches
};

struct MBStruct MBwahl[10];

int posi = 0;


byte MB[2]; //Messbereich
unsigned long MBtime[2]; // Dauer im Messbereich fuer Auswahl

int MB1 = 5;  //Reserve nach unten hin
int MB2 = 5;  //Reserve nach unten hin
float P1mW = 0.0;
float P2mW = 0.0;
Timer t;

// eingelesene Werte
float rawU1 = 0.0; // takes averaged but unscaled value
float rawU2 = 0.0;


float U1 = 0.0; // scaled values!
float U2 = 0.0;
//char outstr4[4];
//char outstr3[3];

// Auskoppeldämpfung der Koppler
float Koppler1 = ATT_KOPPLER1;  //dB
float Koppler2 = ATT_KOPPLER2;  //dB


volatile boolean turned;   // rotary was turned
volatile boolean fired;    // knob was pushed
volatile boolean up;       // true when turned cw

//#########################################################
// menu handling
int Menu_page = 1;
int item_pos = 1;
boolean edit = false;
boolean change_value = false;
int delta_value = 0;
boolean items_changed = false;
//############################################################



// Anzeige
byte screenNo=0; // Auswahl der Darstellung, max. Wert MAXSCREEN
byte oldScreenNo=0;

// Interrupt Service Routine - für den Button des Encoder
void isrB ()
{
  if (!digitalRead (PUSHB))
    fired = true;
#ifdef DEBUG
  Serial.println("Button Pushed");
#endif
}  // end of isr


// Interrupt Service Routine für die Drehbewegung
void isr ()
{
  if (digitalRead (PINA))
    up = digitalRead (PINB);
  else
    up = !digitalRead (PINB);
#ifdef DEBUG
  Serial.println(" Encoder Turned");
#endif
  turned = true;
}  // end of isr

void check_encoder() {
  if(fired) { // es wurde die Taste gedrueckt
      fired=false;
      if (screenNo==1) { // Kruecke, um den Eingang auszuwaehlen, wandert später in die State Machine
        calibration(screenNo,&scalFactor1,PIN_A1);
        screenNo=0;
      }
      else if(screenNo==2) {
          calibration(screenNo,&scalFactor2,PIN_A2);
          screenNo=0;
      }
  }
  fired=false;
  if (turned)
  {
//    if (!edit && !change_value) { // wir sind nicht im Editiermodus
   /*   if (up) {
        if (Menu_page < 3) Menu_page++;  //dann schalten wir die Menüseite um - wenn wir später mehrere haben - Anzahl noch hardkodiert
      }
      else if (Menu_page > 1) Menu_page--;
 */
      // wir sind im Anzeigemodus, wir waehlen das Anzeigeformat
      if(up) 
        screenNo<MAXSCREEN? screenNo++ :(screenNo=0); 
      else 
        screenNo>0? screenNo-- :(screenNo=MAXSCREEN); 
      //Serial.print("Screen:");
      //Serial.println(screenNo);
 //   }
/*    if (edit && !change_value) { // wir sind im Editiermodus uns wählen das zu ändernde Item aus - Anzahl noch hardkodiert

      if (up) {
        if (item_pos < 3) item_pos++;
      }
      else if (item_pos > 1) item_pos--;

      resetCursor();
    }
    if (edit && change_value) { // wir sind im Editiermodus und haben zusätzlich ein Item zum Ändern ausgewählt

      if (up) delta_value++;
      else delta_value--;

    }
*/  
    turned = false;
  } 
  
/*
  if (fired && change_value) {
    change_value = false;
    fired = false;

  }

 if(fired && edit && !change_value) { // rechts unten : EXIT
    if (item_pos == 4) {  // exit !
      edit = false;
      change_value = false;
      fired = false;
      lcd.noBlink();
      lcd.noCursor();
      items_changed = false;
      }
    
     if (fired && (item_pos == 3) && items_changed){  // links unten : SAVE

       automatik=temp_automatik;// lokale Änderungen speichern!
       Man_state = temp_Man_state;
       items_changed = false;
       fired = false;
      }
    

    if (fired && !change_value && (item_pos == 1)) { // Koppler 1  soll geändert werden
      change_value = true;
      fired = false;
      items_changed = true;
    }
    if (fired && !change_value && (item_pos == 3)) { // Koppler 2 soll geändert werden
      change_value = true;
      fired = false;
      items_changed = true;
    }
    if (fired && !change_value && (item_pos == 2)) { // Messfrequenz soll geändert werden
      change_value = true;
      fired = false;
      items_changed = true;
    }
  }

  if (change_value) {

    if (item_pos == 1) {  // Koppler 1 soll geändert werden

      Koppler1 = Koppler1 + delta_value * 0.1;
      if (Koppler1 < 0.0)  Koppler1 = 0.0;
      delta_value = 0;

    }
    if (item_pos == 3) {  // Koppler 2 soll geändert werden

      Koppler2 = Koppler2 + delta_value * 0.1;
      if (Koppler1 < 0.0)  Koppler1 = 0.0;
      delta_value = 0;
    }

  }
*/

}

// Initialisiere LCD Display
LiquidCrystal lcd(LCD_RS, LCD_EN, LCD_D4, LCD_D5, LCD_D6, LCD_D7); // -- creating LCD instance

LcdBarGraph lbgPWR(&lcd, 13, 0, 3); //


void setup() {
  Serial.begin(115200);
  // -- initializing the LCD
  Serial.println("Start PowerMeter");
#ifdef DEBUG
  
#endif
  Serial.println("---------------------------------------------------------");
  lcd.begin(20, 4);
  lcd.clear();

  //pinMode(PINA,INPUT_PULLUP);
  //pinMode(PINB,INPUT_PULLUP);
  //pinMode(PUSHB,INPUT_PULLUP);
  pinMode(PINA, PINA_INPUT);
  pinMode(PINB, PINB_INPUT);
  pinMode(PUSHB, PUSHB_INPUT);

  // initailisiere die ISR Routinen
  //isr() für Encoder Handling
  //isrB() für Push Button
  // beim Encoder evtl. CHANGE eintragen
  attachInterrupt (digitalPinToInterrupt(PINA), isr, PINA_INT);   // Encoder
  attachInterrupt (digitalPinToInterrupt(PUSHB), isrB, PUSHB_INT); // Push Button

  // setzen der Analogreferenz.
  analogReference(REFERENCE);

  lcd.setCursor(0, 0);
  lcd.print(START1);
  lcd.setCursor(0, 1);
  lcd.print(START2);
  lcd.setCursor(0, 2);
  lcd.print(START3);
  lcd.setCursor(0, 3);
  lcd.print(START4);
  delay(2000);
  lcd.clear();

  linM0[0].dB = 10; linM0[0].ADnorm = 195.2;
  linM0[1].dB = 5; linM0[1].ADnorm = 208.6;
  linM0[2].dB = 0; linM0[2].ADnorm = 231.1;
  linM0[3].dB = -5; linM0[3].ADnorm = 281.5;
  linM0[4].dB = -10; linM0[4].ADnorm = 340.7;
  linM0[5].dB = -15; linM0[5].ADnorm = 406.8;
  linM0[6].dB = -20; linM0[6].ADnorm = 473.2;
  linM0[7].dB = -25; linM0[7].ADnorm = 538.7;
  linM0[8].dB = -30; linM0[8].ADnorm = 604.2;
  linM0[9].dB = -35; linM0[9].ADnorm = 670.0;
  linM0[10].dB = -40; linM0[10].ADnorm = 736.8;
  linM0[11].dB = -45; linM0[11].ADnorm = 800.4;
  linM0[12].dB = -50; linM0[12].ADnorm = 863.8;
  linM0[13].dB = -55; linM0[13].ADnorm = 914.2;
  linM0[14].dB = -60; linM0[14].ADnorm = 958.1;
  linM0[15].dB = -65; linM0[15].ADnorm = 972.1;
  linM0[16].dB = -70; linM0[16].ADnorm = 981.6;

  // Startpunkt fuer Messbereichswahl
  MB[0]=4;
  MB[1]=4;


  // Messbereichswahl

   MBwahl[0].range=1E-3;   MBwahl[0].tLen=1; MBwahl[0].Nachkomma=1;  MBwahl[0].Divisor=1E-6;  strcpy(MBwahl[0].Unit,"nW\0"); strcpy(MBwahl[0].Text,"1nW\0");
   MBwahl[1].range=1E-2;   MBwahl[1].tLen=1; MBwahl[1].Nachkomma=3;  MBwahl[1].Divisor=1E-3;  strcpy(MBwahl[1].Unit,"uW\0"); strcpy(MBwahl[1].Text,"10uW\0");
   MBwahl[2].range=1E-1;   MBwahl[2].tLen=1; MBwahl[2].Nachkomma=2;  MBwahl[2].Divisor=1E-3;  strcpy(MBwahl[2].Unit,"uW\0"); strcpy(MBwahl[2].Text,"100uW\0");
   MBwahl[3].range=1E0;    MBwahl[3].tLen=1; MBwahl[3].Nachkomma=1;  MBwahl[3].Divisor=1E-3;  strcpy(MBwahl[3].Unit,"uW\0"); strcpy(MBwahl[3].Text,"1mW\0");
   MBwahl[4].range=1E1;    MBwahl[4].tLen=1; MBwahl[4].Nachkomma=3;  MBwahl[4].Divisor=1E0;   strcpy(MBwahl[4].Unit,"mW\0"); strcpy(MBwahl[4].Text,"10mW\0");
   MBwahl[5].range=1E2;    MBwahl[5].tLen=1; MBwahl[5].Nachkomma=2;  MBwahl[5].Divisor=1E0;   strcpy(MBwahl[5].Unit,"mW\0"); strcpy(MBwahl[5].Text,"100mW\0");
   MBwahl[6].range=1E3;    MBwahl[6].tLen=1; MBwahl[6].Nachkomma=1;  MBwahl[6].Divisor=1E0;   strcpy(MBwahl[6].Unit,"mW\0"); strcpy(MBwahl[6].Text,"1W\0");
   MBwahl[7].range=1E4;    MBwahl[7].tLen=1; MBwahl[7].Nachkomma=3;  MBwahl[7].Divisor=1E3;   strcpy(MBwahl[7].Unit,"W\0");  strcpy(MBwahl[7].Text,"10W\0");
   MBwahl[8].range=1E5;    MBwahl[8].tLen=1; MBwahl[8].Nachkomma=2;  MBwahl[8].Divisor=1E3;   strcpy(MBwahl[8].Unit,"W\0");  strcpy(MBwahl[8].Text,"100W\0");



  // Hier das eigentliche "Multitasking"
  // alle 1000ms Ausgabe der Rohsignal an den PC
  // alle 10ms die A/D-Wandler einlesen
  // alle 100ms das LCD beschreiben
  // alle 20ms  auf Encoderereignisse reagieren

  t.every(1000, write_raw_seriell, 0);
  t.every(10, take_ads, 0); //lese die analogen Eingänge alle 10ms und glätte diese
  t.every(100, write_lcd, 0); //alle 500ms auf LCD darstellen
  t.every(20, check_encoder, 0); // alle 20ms Änderungen des Encoders detektieren
}

void loop()
{

  t.update(); //timer handling!	mehr passiert in der Hauptschleife nicht, Multitasking läuft im Hintergrund

}

void calibration(byte kanal,float *scalFactor,byte pin){
  // Kalibrierung des Kanals bei limM0[CALIBRATE]
  float dbM=linM0[CALIBRATE].dB;
  float ref=linM0[CALIBRATE].ADnorm;
  char outstr[30];
  byte i,count=0;
  float oldScal=-999;
  float rawU=linM0[HIGH_LIMIT].ADnorm;
  int mp;
  float smooth = 0.1;
  boolean ready=false;
  
  lcd.clear();
  for(i=0;i<sizeof(outstr);i++) outstr[i]='\0';
  LCDout("Kalibrierung Kanal",0,0,18);
  itoa(kanal,outstr,10);
  LCDout(outstr,19,0,1);
  for(i=0;i<sizeof(outstr);i++) outstr[i]='\0';
  LCDout("P:",0,1,6);
  dtostrf(dbM,5,1,outstr);
  LCDout(outstr,2,1,5);
  LCDout("dBm",7,1,3);
  for(i=0;i<sizeof(outstr);i++) outstr[i]='\0';
  LCDout("Ref:",0,2,4);
  dtostrf(ref,6,1,outstr);
  LCDout(outstr,4,2,6);
  for(i=0;i<sizeof(outstr);i++) outstr[i]='\0';
  LCDout("MW:",11,2,3);
  dtostrf(rawU1,6,1,outstr);
  LCDout(outstr,14,2,6);
  for(i=0;i<sizeof(outstr);i++) outstr[i]='\0';
  LCDout("Scal:",0,3,5);
  dtostrf(*scalFactor,5,3,outstr);
  LCDout(outstr,5,3,5);
  //LCDout("0/9",16,1,3);
  // Anfangswert setzen
  rawU=(float)analogRead(pin)* *scalFactor;
  // wir koennen nicht auf die MW  per Task warten, wir müssen die Messungen selbst aufrufen
 //while( int(*scalFactor*10000) != int(oldScal *10000)) {// drei Stellen hinter dem Komma stabil
 while(count<3) {
    oldScal= *scalFactor;
    mp=(float)analogRead(pin);
    rawU = (1.0 - smooth) * rawU + (smooth * mp);
    *scalFactor=ref/rawU;
    for(i=0;i<sizeof(outstr);i++) outstr[i]='\0';
    dtostrf(rawU,6,1,outstr);
    LCDout(outstr,14,2,6);
    for(i=0;i<sizeof(outstr);i++) outstr[i]='\0';
    LCDout("AK:",11,3,3);
    itoa(mp,outstr,10);
    LCDout(outstr,15,3,6);
    for(i=0;i<sizeof(outstr);i++) outstr[i]='\0';
    dtostrf(*scalFactor,5,3,outstr);
    LCDout(outstr,5,3,5);
    
    // Wir erhöhen die Glaettung nach Genauigkeit
    if (!ready && (int(*scalFactor*10) == int(oldScal *10)))
      smooth=0.01;
     else if (!ready && (int(*scalFactor*100) == int(oldScal *100)))
      smooth=0.01;
    else if (!ready && (int(*scalFactor*1000) == int(oldScal *1000))) {
      smooth=0.001;
      ready=true;
    }
    else if (ready && (int(*scalFactor*1000) == int(oldScal *1000))) {
      count++;
      //LCDout("*",0,0,1);
    }
    else
      count=0;
    itoa(count,outstr,10);
    LCDout(outstr,12,1,1);

    for(i=0;i<sizeof(outstr);i++) outstr[i]='\0';
    dtostrf(smooth,5,4,outstr);
    LCDout(outstr,14,1,5);
    delay(40);
    if (fired) {
       fired=false;
       break;
    }
  }
  LCDout("Kalibrierung fertig",0,0,20);
  delay(3000);
  //lcd.clear();
  
  
}

void take_ads()
{
  float mp;
  // erster Kanal
  mp=freq_correction((float)analogRead(PIN_A1)*scalFactor1,frequenz,linM0);
  // falls der MW kleiner als unsere kleinste Spannung ist, glätten wir nicht von 0 an
  if (rawU1 < linM0[HIGH_LIMIT].ADnorm) // noch kein Wert da, dann nehmen wir mal den echten Messwert
    rawU1=mp;
  //Glättung
  rawU1 = (1.0 - smooth) * rawU1 + smooth * mp;
  // zweiter Kanal
  mp=freq_correction((float)analogRead(PIN_A2)*scalFactor2,frequenz,linM0);
  if (rawU2 < linM0[HIGH_LIMIT].ADnorm) // noch kein Wert da, dann nehmen wir mal den echten Messwert
    rawU2=mp;
  rawU2 = (1.0 - smooth) * rawU2 + smooth * mp;
 // rawU2 = (1.0 - smooth) * rawU2 + smooth * (float)analogRead(PIN_A2) * scalFactor2;
}


float freq_correction(float rawU,float qrg, struct LinStruct *linM ) {
  float corrU;
  float offset;
  offset=(qrg * QRG_FACTOR); //Offset(f)= 0,020469 * Messfrequenz in MHz 
  corrU=rawU + offset;     // MW = MW + Offset(f); 

  
  if (corrU > linM[QRG_HIGH_CORR].ADnorm){
    corrU = corrU -((corrU - linM[QRG_HIGH_CORR].ADnorm)/QRG_HIGH_FACTOR) * offset; //  MW = MW - ((MW - 736,8) / 244,8) * Offset(f);

  }
  else if ((corrU > linM[HIGH_LIMIT].ADnorm) && (corrU < linM[QRG_LOW_CORR].ADnorm)) {// nur ausführen, wenn wir einen Wert >0dBm haben, aus Limits ermittelt
    corrU = corrU -((linM[QRG_LOW_CORR].ADnorm-corrU)/QRG_LOW_FACTOR) * offset;  //  W = MW - ((281,5 - MW) / 86,4 ) * Offset(f);

  }
  // Begrenzung der Werte
  //
  //if (MW > 981,6) MW = 981,6; 
  //if (MW < 231,1) MW = 231,1; 
   if (corrU > linM[LOW_LIMIT].ADnorm)         corrU =  linM[LOW_LIMIT].ADnorm;    //#define LOW_LIMIT 16  entspricht -70dBm
   else if (corrU <linM[HIGH_LIMIT].ADnorm)    corrU =  linM[HIGH_LIMIT].ADnorm;
  return corrU;
}

float linearize(float rawU, struct LinStruct *linM, byte linMsize ) {
  // Interpoliert anhand der liM Struct
  // rawU: Messwert, unmgerechnet auf 1024 (siehe take_ads()
  // linM: Zeiger auf Array mit den Lineariserungswerten
  // linMSize: Größe des Arries
  byte pU = 0;
  float U = 0.0;
  linM = linM0;
  //int linMsize=sizeof(linM0)/sizeof(struct LinStruct);
  //byte linMsize=sizeof(linM0)/sizeof(LinStruct);
#ifdef DEBUG
  Serial.print("rawU=");
  Serial.println(rawU);
  Serial.print("sizeof(linM)=");
  Serial.println(linMsize);
#endif
  for (int q = 0; q < linMsize; q++) {
    if (rawU > (float) linM[q].ADnorm) {
      pU = q;
      continue;
    }
  }
#ifdef DEBUG
  Serial.print("PointerIDX=");
  Serial.print (pU);
  Serial.print("  ADNORM=");
  Serial.print(linM[pU].ADnorm);
  Serial.print("  dB=");
  Serial.println(linM[pU].dB);
#endif
  if (pU > (linMsize-1))    U = linM[linMsize].dB;
  else if (pU < 1)          U = linM[0].dB;
  else                      U = ((rawU - linM[pU].ADnorm) / (linM[pU + 1].ADnorm - linM[pU].ADnorm)) * (linM[pU + 1].dB - linM[pU].dB) + linM0[pU].dB;
#ifdef DEBUG
  Serial.print("U lin=");
  Serial.println (U);
#endif
  return U;
}
void LCDout (char *outstring,byte x, byte y, byte len) {

  byte i;
  char empty[21];

  if (len>20) return;
  
  // Leerstring zusammenbauen
  for(i=0;i<len;i++) {
   empty[i]=' ';
  }
  empty[i]='\0';
  lcd.setCursor(x, y);
  lcd.print(empty);
  lcd.setCursor(x, y);
  strncpy(empty,outstring,len);
  lcd.print(empty);
  //lcd.print(outstring);
}

void screen0(float U1,float U2,float P1mW,float P2mW,float VSWR) {

  int lbg_draw_val_limited;
  char outstr[30];
  char outstr1[30];
  float P1mW_out = 0;
  float P2mW_out = 0;
  int i;


  P1mW_out = P1mW / MBwahl[MB[0]].Divisor;
  P2mW_out = P2mW / MBwahl[MB[1]].Divisor;
  outstr[0]='\0';
  dtostrf(U1,4,1,outstr);
  strcat(outstr,"dBm");
  LCDout(outstr,0,0,8);

  outstr[0]='\0';
  dtostrf(U2,4,1,outstr);
  strcat(outstr,"dBm");
  LCDout(outstr,12,0,8);

 
  if (abs(VSWR) > 99.9) VSWR = 99.9;

  //outstr=String(VSWR,2);
  outstr[0]='\0';
  dtostrf(VSWR,4,2,outstr);
  LCDout(outstr,8,2,4);
  outstr[0]='\0';
  strcpy(outstr,"SWR\0");
  LCDout(outstr,9,1,3);

  // Nachkommastellen und Einheit werden aus dem Sturct Array gelesen.
  //[0] Eingang1
  //[1] Eingang2
  outstr[0]='\0';
  for(i=0;i<sizeof(outstr);i++) outstr[i]='\0';
  dtostrf(P1mW_out,5,MBwahl[MB[0]].Nachkomma,outstr); 
  strcat(outstr,MBwahl[MB[0]].Unit);
  LCDout(outstr,1,1,8);

  outstr[0]='\0';
  for(i=0;i<sizeof(outstr);i++) outstr[i]='\0';
  dtostrf(P2mW_out,5,MBwahl[MB[1]].Nachkomma,outstr);
  strcat(outstr,MBwahl[MB[1]].Unit);
  LCDout(outstr,13,1,7);

  outstr[0]='\0';
  for(i=0;i<sizeof(outstr);i++) outstr[i]='\0';
  //strcpy(outstr,"ATT:");
  dtostrf(Koppler1,4,1,outstr1);
  strcat(outstr,outstr1);
  LCDout(outstr,1,2,7);

 
  outstr[0]='\0';
  for(i=0;i<sizeof(outstr);i++) outstr[i]='\0';
  //strcpy(outstr,"ATT:");
  dtostrf(Koppler2,4,1,outstr1);
  strcat(outstr,outstr1);
  LCDout(outstr,13,2,7);
 
  //Den zu zeichnenden Bargraph mit dem jeweiligen Messbereich skalieren und auf 999 begrenzen
  //lbg_draw_val_limited = int(1000 * P1mW / (0.0008 * pow(10, MB1 - 1)));
  lbg_draw_val_limited = int(1000 * P1mW / (MBwahl[MB[0]].range));
  if (lbg_draw_val_limited > 999) lbg_draw_val_limited = 999;
  lbgPWR.drawValue(lbg_draw_val_limited, 1000);
  outstr[0]='\0';
  for(i=0;i<sizeof(outstr);i++) outstr[i]='\0';
  strcpy(outstr,"[");
  strcat(outstr,MBwahl[MB[0]].Text);
  strcat(outstr,"]");

  LCDout(outstr,13,3,7);


  lcd.setCursor(10, 0);
  lcd.print(" ");
  lcd.setCursor(10, 0);

  lcd.print(item_pos);

  edit = true;
  resetCursor();

}

void screen1(byte no,float Ulin,float U,float rawU, float scalFactor) {
  int lbg_draw_val_limited;
  float PmW_out = 0;
  float PmW=0;
  char outstr[30];
  int i;
  byte mb=4;
  float Um=0;
  PmW = pow(10, Ulin / 10.0);
  mb= MBselect(PmW);
  resetCursor();
  outstr[0]='\0';
  for(i=0;i<sizeof(outstr);i++) outstr[i]='\0';
  itoa(no+1,outstr,10);
  strcat(outstr,">");
  LCDout(outstr,0,0,2);

  PmW_out = PmW / MBwahl[mb].Divisor;
    
  for(i=0;i<sizeof(outstr);i++) outstr[i]='\0';
  dtostrf(Ulin,4,1,outstr);
  strcat(outstr,"dBm");
  LCDout(outstr,2,0,8);
  for(i=0;i<sizeof(outstr);i++) outstr[i]='\0';
  dtostrf(PmW_out,5,MBwahl[mb].Nachkomma,outstr); 
  strcat(outstr,MBwahl[mb].Unit);
  LCDout(outstr,11,0,8);


  Um=rawU /  scalFactor;
  for(i=0;i<sizeof(outstr);i++) outstr[i]='\0';
  LCDout("Dm:",0,1,10);
  dtostrf(Um,5,1,outstr);
  //strcat(outstr,"Dr:");
  LCDout(outstr,4,1,10);

  for(i=0;i<sizeof(outstr);i++) outstr[i]='\0';
  LCDout("Dk:",11,1,10);
  dtostrf(rawU,5,1,outstr);
  //strcat(outstr,"Dk:");
  LCDout(outstr,15,1,10);

  // Sannung am AD Wandler
  //Um=rawU / 1024 * UREF / scalFactor ;
  //Um=rawU / scalFactor ;
  
    for(i=0;i<sizeof(outstr);i++) outstr[i]='\0';
    strcpy(outstr,"Faktor:");
    LCDout(outstr,0,2,7);
    for(i=0;i<sizeof(outstr);i++) outstr[i]='\0';
    dtostrf(scalFactor,5,3,outstr);
    LCDout(outstr,9,2,5);
   //Den zu zeichnenden Bargraph mit dem jeweiligen Messbereich skalieren und auf 999 begrenzen
 
  lbg_draw_val_limited = int(1000 * PmW / (MBwahl[mb].range));
  if (lbg_draw_val_limited > 999) lbg_draw_val_limited = 999;
  lbgPWR.drawValue(lbg_draw_val_limited, 1000);
 
  for(i=0;i<sizeof(outstr);i++) outstr[i]='\0';
  strcpy(outstr,"[");
  strcat(outstr,MBwahl[mb].Text);
  strcat(outstr,"]");

  LCDout(outstr,13,3,7);
}

void screen2(float U1,float U2,float P1mW,float P2mW,float VSWR) {
 int lbg_draw_val_limited;
 float P1mW_out = 0;
  float P2mW_out = 0;
  char outstr[30];
  int i;

  resetCursor();
  outstr[0]='\0';
  for(i=0;i<sizeof(outstr);i++) outstr[i]='\0';
  strcpy(outstr,"Screen2");
  LCDout(outstr,0,2,7);
}

void write_lcd() //auffrischen des LCD  - wird alle 100ms angestossen
{
  float VSWR = 1.0;
  float P1mW_out = 0;
  float P2mW_out = 0;
  float U1lin=0;
  float U2lin=0;
  //String outstr;
  //static float old_U1 = 0.0;
  //String P_unit1, P_unit2;
  //int lbg_draw_val_limited;
  
  resetCursor();
  // Frequenzkorrektur
  //rawU1=freq_correction(rawU1,frequenz, linM0);
  //rawU2=freq_correction(rawU2,frequenz, linM0);
    // Lienarisierung aufrufen.
  // Länge der LinM Struct kann nicht in der Funktion bestimmt werden (3.Argument)
  U1lin = linearize(rawU1, linM0, sizeof(linM0) / sizeof(struct LinStruct));
  U2lin = linearize(rawU2, linM0, sizeof(linM0) / sizeof(struct LinStruct));

  U1=U1lin+Koppler1;
  U2=U2lin+Koppler2;

  P1mW = pow(10, U1 / 10.0);
  P2mW = pow(10, U2 / 10.0);

  //MB[0] enthaelt den gewählten Bereich fuer Eingang1, MB[1] für Eingang2
  // die Wwerte ermitteln wir aus dem Array der Struct. In MBwahl[] findet man dann alles notwendig
  MB_Wahl(0,P1mW);
  MB_Wahl(1,P2mW);
  

  P1mW_out = P1mW / MBwahl[MB[0]].Divisor;
  P2mW_out = P2mW / MBwahl[MB[1]].Divisor;
  VSWR = abs((pow(10, ((U2 - U1) / 10.0)) + 1) / (pow(10, ((U2 - U1) / 10.0)) - 0.999999));
// Ausgabe auf der verschiedenen Screens 
  if (screenNo != oldScreenNo){
     lcd.clear();
     oldScreenNo=screenNo;
     LcdBarGraph lbgPWR(&lcd, 13, 0, 3);
  }
  else { // es wurde nicht gedreht
  }
  if (screenNo==0)
    screen0(U1,U2,P1mW,P2mW,VSWR);
  else if (screenNo==1)
    screen1(0,U1lin,U1,rawU1,scalFactor1);
  else if (screenNo==2)
    screen1(1,U2lin,U2,rawU2,scalFactor2);

  resetCursor();

 
}

byte MBselect(float PmW) {
  // Messbereichsumschaltung anhand MBStruct
  // ohne gloable Variablen
  byte i;
  float  minP;
  float maxP;
  // Bereich passt nicht und Wartezeit vorbei, wir suchen den passenden Bereich
  for(i=0;i<(sizeof(MBwahl)/sizeof(MBStruct));i++) {
    minP=MBwahl[i].range *0.05;
    maxP=MBwahl[i].range *0.8;
    if ((PmW<maxP) && (PmW>minP)) {
      // Wir haben den richtigen MB gefunden
       return i;         
    }
  }
  return 0; 
}


byte MB_Wahl(byte kanal,float PmW) {
  // Messbereichsumschaltung anhand MBStruct
  // Arbeitet mit globalenb Variablen MB[] und MBtime[]
  unsigned long switch_MB = millis();
  byte i;
  byte MBakt;
  float  minP;
  float maxP;
  MBakt=MB[kanal];
  // Grenzen des Bereichs min 0.05 * range, max 08*range
  minP=MBwahl[MBakt].range *0.05;
  maxP=MBwahl[MBakt].range *0.8;
  // wir schauen uns den gewählen Bereich mal an, ob er passt
  //if ((PmW<MBwahl[MBakt].maxP) && (PmW>MBwahl[MBakt].minP)) {
    if ((PmW<maxP) && (PmW>minP)) {
      // dann müssen wir nichts tun
      return MB[kanal];
  }
  // Bereich passt nicht, wie lange sind wir denn schon hier?
  if ((switch_MB - MBtime[kanal])< (MBwahl[MBakt].tLen *1000UL)) { // wir sind noch nicht lange genug in den Bereich
    //Serial.println("Warte...");
    return MB[kanal];
  }
  // Bereich passt nicht und Wartezeit vorbei, wir suchen den passenden Bereich
  for(i=0;i<(sizeof(MBwahl)/sizeof(MBStruct));i++) {
    minP=MBwahl[i].range *0.05;
    maxP=MBwahl[i].range *0.8;
    if ((PmW<maxP) && (PmW>minP)) {
  //  if ((PmW < MBwahl[i].maxP) && (PmW > MBwahl[i].minP)) {
      // Wir haben den richtigen MB gefunden
       MBtime[kanal]=switch_MB; // Zeit merken
       MB[kanal]=i; // Bereich merken
       //Serial.print("Switch...");
       //Serial.println(MBwahl[i].range);
       return MB[kanal];         
    }
  }
  return 0; 
}



void resetCursor() { //um den Cursor im Edit-Mode nach Anzeige des Status wieder an die richtige Stelle zu setzen!
  if (item_pos == 1) lcd.setCursor(3, 2);
  if (item_pos == 2) lcd.setCursor(12, 2);
  if (item_pos == 3) lcd.setCursor(18, 2);

}

void write_raw_seriell() {
  Serial.print("U1:");
  Serial.print((int)rawU1);
  Serial.print("(");
  //Serial.print(rawU1 / 1024 * UREF / scalFactor1 / UFACTOR);
  Serial.print("V)  U2:");
  Serial.print((int)rawU2);
  Serial.print("(");
  //Serial.print(rawU2 / 1024 * UREF / scalFactor2 / UFACTOR);
  Serial.println("V)");
}
