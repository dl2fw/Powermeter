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


// Auskoppeldämfpung in dB
#define ATT_KOPPLER1 50
#define ATT_KOPPLER2 30
// welche Ref. für den AD Wandler soll genutzt werden?
#define REFERENCE INTERNAL2V56
// #define REFERENCE INTERNAL1V1
#define UREF 2.56
//Wert bei -70dBm (max. Ausgangsspannung) entspricht 1023
#define U70dBm_1 1.745
#define U70dBm_2 1.715
// Spannungsteiler Ausgang AD8317
#define UFACTOR 1

// Anzeigeformate
// 2 --> 3 Darsatellungen 0,1,2
#define MAXSCREEN 2 




// Startmeldungen
#define START1 "   PowerMeter"
#define START2 "   1MHz ... 10GHz"
#define START3 "  K08 DL2FW/DM6TT"
#define START4 "  ---------------"


//Linearisierungstabelle für den Messverstärker [db vs. rawU 0...1023]
//Arrays für die Interpolationn der Messwerte wird in der setup() Routine definiert
//int linM0[17][17];


// Skalierungsfaktor zur Umrechnung der Messwerte auf -70dBm => 1024
const float scalFactor1 = UREF / U70dBm_1 ;
const float scalFactor2 = UREF / U70dBm_2 ;


const float smooth = 0.004;
//> timeconstant=200ms (app. 1/smooth * TA (10ms)) @smooth=0.01-> timeconstant=1000ms ... - correct for smooth << 1

struct LinStruct {
  int dB;
  unsigned int ADnorm;
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

// eingelesene Werte, werden geglättet 0..1023
float rawU1 = 0.0; // takes averaged but unscaled value
float rawU2 = 0.0;


float U1 = 0.0; // scaled values!
float U2 = 0.0;
char outstr4[4];
char outstr3[3];

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
  Serial.print("VRef:");
  Serial.println(UREF);
  Serial.print("U1 -70dBm (Vor Spannungsteiler):");
  Serial.print(U70dBm_1);
  Serial.print("U2 -70dBm (Vor Spannungsteiler):");
  Serial.print(U70dBm_2);
  Serial.print("V \nFaktor Spannungsteiler:");
  Serial.println(UFACTOR);
  Serial.println("Alle Spannungs-Messwerte beziehen sich auf Ausgang AD8317");
  Serial.println("Alle Raw-Messwerte beziehen sich auf U -70dBm==1023");
#endif
  Serial.println("---------------------------------------------------------");
  lcd.begin(20, 4);
  lcd.clear();

  //pinMode(PINA,INPUT_PULLUP);
  //pinMode(PINB,INPUT_PULLUP);
  //pinMode(PUSHB,INPUT_PULLUP);
  pinMode(PINA, INPUT);
  pinMode(PINB, INPUT);
  pinMode(PUSHB, INPUT);

  // initailisiere die ISR Routinen
  //isr() für Encoder Handling
  //isrB() für Push Button
  // beim Encoder evtl. CHANGE eintragen
  attachInterrupt (digitalPinToInterrupt(PINA), isr, FALLING);   // Encoder
  attachInterrupt (digitalPinToInterrupt(PUSHB), isrB, FALLING); // Push Button

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

  linM0[0].dB = 10; linM0[0].ADnorm = 201;
  linM0[1].dB = 5; linM0[1].ADnorm = 214;
  linM0[2].dB = 0; linM0[2].ADnorm = 242;
  linM0[3].dB = -5; linM0[3].ADnorm = 249;
  linM0[4].dB = -10; linM0[4].ADnorm = 360;
  linM0[5].dB = -15; linM0[5].ADnorm = 431;
  linM0[6].dB = -20; linM0[6].ADnorm = 497;
  linM0[7].dB = -25; linM0[7].ADnorm = 565;
  linM0[8].dB = -30; linM0[8].ADnorm = 632;
  linM0[9].dB = -35; linM0[9].ADnorm = 701;
  linM0[10].dB = -40; linM0[10].ADnorm = 769;
  linM0[11].dB = -45; linM0[11].ADnorm = 834;
  linM0[12].dB = -50; linM0[12].ADnorm = 890;
  linM0[13].dB = -55; linM0[13].ADnorm = 933;
  linM0[14].dB = -60; linM0[14].ADnorm = 962;
  linM0[15].dB = -65; linM0[15].ADnorm = 1000;
  linM0[16].dB = -70; linM0[16].ADnorm = 1023;

  // Startpunkt fuer Messbereichswahl
  MB[0]=4;
  MB[1]=4;


  // Messbereichswahl

   MBwahl[0].range=1E-3;   MBwahl[0].tLen=1; MBwahl[0].Nachkomma=1;  MBwahl[0].Divisor=1E-6;  strcpy(MBwahl[0].Unit,"nW\0"); strcpy(MBwahl[0].Text,"1uW\0");
   MBwahl[1].range=1E-2;   MBwahl[1].tLen=1; MBwahl[1].Nachkomma=3;  MBwahl[1].Divisor=1E-3;  strcpy(MBwahl[1].Unit,"uW\0"); strcpy(MBwahl[1].Text,"10uW\0");
   MBwahl[2].range=1E-1;   MBwahl[2].tLen=1; MBwahl[2].Nachkomma=2;  MBwahl[2].Divisor=1E-3;  strcpy(MBwahl[2].Unit,"uW\0"); strcpy(MBwahl[2].Text,"100uW\0");
   MBwahl[3].range=1E0;    MBwahl[3].tLen=1; MBwahl[3].Nachkomma=1;  MBwahl[3].Divisor=1E-3;  strcpy(MBwahl[3].Unit,"uW\0"); strcpy(MBwahl[3].Text,"1mW\0");
   MBwahl[4].range=1E1;    MBwahl[4].tLen=1; MBwahl[4].Nachkomma=3;  MBwahl[4].Divisor=1E0;   strcpy(MBwahl[4].Unit,"mW\0"); strcpy(MBwahl[4].Text,"10mW\0");
   MBwahl[5].range=1E2;    MBwahl[5].tLen=1; MBwahl[5].Nachkomma=2;  MBwahl[5].Divisor=1E0;   strcpy(MBwahl[5].Unit,"mW\0"); strcpy(MBwahl[5].Text,"100mW\0");
   MBwahl[6].range=1E3;    MBwahl[6].tLen=1; MBwahl[6].Nachkomma=1;  MBwahl[6].Divisor=1E0;   strcpy(MBwahl[6].Unit,"mW\0"); strcpy(MBwahl[6].Text,"1W\0");
   MBwahl[7].range=1E4;    MBwahl[7].tLen=1; MBwahl[7].Nachkomma=3;  MBwahl[7].Divisor=1E3;   strcpy(MBwahl[7].Unit,"W\0");  strcpy(MBwahl[7].Text,"10W\0");
   MBwahl[8].range=1E5;    MBwahl[8].tLen=1; MBwahl[8].Nachkomma=2;  MBwahl[8].Divisor=1E3;   strcpy(MBwahl[8].Unit,"W\0");  strcpy(MBwahl[8].Text,"100W\0");

 /*  MBwahl[0].minP=0.00005;  MBwahl[0].maxP=0.0008;  MBwahl[0].tLen=1; MBwahl[0].Nachkomma=1;  MBwahl[0].Divisor=0.000001; MBwahl[0].Unit="nW";
   MBwahl[1].minP=0.0005;   MBwahl[1].maxP=0.008;   MBwahl[1].tLen=1; MBwahl[1].Nachkomma=3;  MBwahl[1].Divisor=0.001;    MBwahl[1].Unit="uW";
   MBwahl[2].minP=0.005;    MBwahl[2].maxP=0.08;    MBwahl[2].tLen=1; MBwahl[2].Nachkomma=2;  MBwahl[2].Divisor=0.001;    MBwahl[2].Unit="uW";
   MBwahl[3].minP=0.05;     MBwahl[3].maxP=0.8;     MBwahl[3].tLen=1; MBwahl[3].Nachkomma=1;  MBwahl[3].Divisor=0.001;    MBwahl[3].Unit="uW";
   MBwahl[4].minP=0.5;      MBwahl[4].maxP=8.0;     MBwahl[4].tLen=1; MBwahl[4].Nachkomma=3;  MBwahl[4].Divisor=1.0;      MBwahl[4].Unit="mW";
   MBwahl[5].minP=5.0;      MBwahl[5].maxP=80.0;    MBwahl[5].tLen=1; MBwahl[5].Nachkomma=2;  MBwahl[5].Divisor=1.0;      MBwahl[5].Unit="mW";
   MBwahl[6].minP=50.0;     MBwahl[6].maxP=800.0;   MBwahl[6].tLen=1; MBwahl[6].Nachkomma=1;  MBwahl[6].Divisor=1.0;      MBwahl[6].Unit="mW";
   MBwahl[7].minP=500.0;    MBwahl[7].maxP=8000.0;  MBwahl[7].tLen=1; MBwahl[7].Nachkomma=3;  MBwahl[7].Divisor=1000.0;   MBwahl[7].Unit="W";
   MBwahl[8].minP=5000.0;   MBwahl[8].maxP=80000.0; MBwahl[8].tLen=1; MBwahl[8].Nachkomma=2;  MBwahl[8].Divisor=1000.0;   MBwahl[8].Unit="W";
*/
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

void take_ads()
{
  rawU1 = (1.0 - smooth) * rawU1 + smooth * (float)analogRead(PIN_A1) * scalFactor1;
  rawU2 = (1.0 - smooth) * rawU2 + smooth * (float)analogRead(PIN_A2) * scalFactor2;
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
  if (pU > (linMsize - 2))    U = linM[linMsize - 1].dB;
  else if (pU < 1)          U = linM[0].dB;
  else                      U = ((rawU - linM[pU].ADnorm) / (linM[pU + 1].ADnorm - linM[pU].ADnorm)) * (linM[pU + 1].dB - linM[pU].dB) + linM0[pU].dB;
#ifdef DEBUG
  Serial.print("U lin=");
  Serial.println (U);
#endif
  return U;
}

void LCDout (String outstring,byte x, byte y, byte len) {

  byte i;
  char empty[21];

  if (len>21) return;
  
  // Leerstring zusammenbauen
  for(i=0;i<len;i++) {
   empty[i]=' ';
  }
  empty[i]='\0';
  lcd.setCursor(x, y);
  lcd.print(empty);
  lcd.setCursor(x, y);
  lcd.print(outstring);
}

void screen0(float U1,float U2,float P1mW,float P2mW,float VSWR) {

  int lbg_draw_val_limited;
  String outstr;
  float P1mW_out = 0;
  float P2mW_out = 0;


  P1mW_out = P1mW / MBwahl[MB[0]].Divisor;
  P2mW_out = P2mW / MBwahl[MB[1]].Divisor;
  outstr=String(U1,1);
  outstr.concat("dBm");
  LCDout(outstr,0,0,8);

  outstr=String(U2,1);
  outstr.concat("dBm");
  LCDout(outstr,12,0,8);
  dtostrf(U2, 4, 1, outstr4);
 
  if (abs(VSWR) > 99.9) VSWR = 99.9;

  outstr=String(VSWR,2);
  LCDout(outstr,8,2,4);
  LCDout("SWR",9,1,3);

  // Nachkommastellen und Einheit werden aus dem Sturct Array gelesen.
  //[0] Eingang1
  //[1] Eingang2
  outstr=String(P1mW_out,MBwahl[MB[0]].Nachkomma);
  outstr.concat(MBwahl[MB[0]].Unit);
  LCDout(outstr,1,1,8);

  outstr=String(P2mW_out,MBwahl[MB[1]].Nachkomma);
  outstr.concat(MBwahl[MB[1]].Unit);
  LCDout(outstr,13,1,7);

  outstr="AT:";
  outstr.concat(String(Koppler1,1));
  LCDout(outstr,0,2,7);

 
  outstr="AT:";
  outstr.concat(String(Koppler2,1));
  LCDout(outstr,12,2,7);
  
  //Den zu zeichnenden Bargraph mit dem jeweiligen Messbereich skalieren und auf 999 begrenzen
  //lbg_draw_val_limited = int(1000 * P1mW / (0.0008 * pow(10, MB1 - 1)));
  lbg_draw_val_limited = int(1000 * P1mW / (MBwahl[MB[0]].range));
  if (lbg_draw_val_limited > 999) lbg_draw_val_limited = 999;
  lbgPWR.drawValue(lbg_draw_val_limited, 1000);
//  outstr="["+ MBwahl[MB[0]].Text+"]";
  LCDout(outstr,13,3,7);


  lcd.setCursor(10, 0);
  lcd.print(" ");
  lcd.setCursor(10, 0);

  lcd.print(item_pos);

  edit = true;
  resetCursor();

}

void screen1(float U1,float U2,float P1mW,float P2mW,float VSWR) {
  int lbg_draw_val_limited;
  String outstr;
  float P1mW_out = 0;
  float P2mW_out = 0;
  

  resetCursor();
  outstr="SCREEN1  ";
  outstr.concat(String(Koppler1,1));
  LCDout(outstr,0,2,7);
}

void screen2(float U1,float U2,float P1mW,float P2mW,float VSWR) {
  int lbg_draw_val_limited;
  String outstr;
  float P1mW_out = 0;
  float P2mW_out = 0;
  
  resetCursor();
  outstr="SCREEN2  ";
  outstr.concat(String(Koppler2,1));
  LCDout(outstr,0,2,7);
}

void write_lcd() //auffrischen des LCD  - wird alle 100ms angestossen
{
  float VSWR = 1.0;
  float P1mW_out = 0;
  float P2mW_out = 0;
  //String outstr;
  //static float old_U1 = 0.0;
  //String P_unit1, P_unit2;
  //int lbg_draw_val_limited;
  
  resetCursor();
  // Lienarisierung aufrufen.
  // Länge der LinM Struct kann nicht in der Funktion bestimmt werden (3.Argument)
  U1 = linearize(rawU1, linM0, sizeof(linM0) / sizeof(struct LinStruct)) + Koppler1;
  U2 = linearize(rawU2, linM0, sizeof(linM0) / sizeof(struct LinStruct)) + Koppler2;

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
  if (screenNo == oldScreenNo){
    Serial.println("NO CHOICE");
  }
  else { // es wurde gedreht
      //lcd.clear();
      Serial.print("Old CHOICE:");
      Serial.print(oldScreenNo);
      oldScreenNo=screenNo;
      Serial.print("  NEW  CHOICE:");
      Serial.println(screenNo);
      //if (screenNo==0) LcdBarGraph lbgPWR(&lcd, 13, 0, 3); //
  }
  if (screenNo==0)
    screen0(U1,U2,P1mW,P2mW,VSWR);
  else if (screenNo==1)
    screen1(U1,U2,P1mW,P2mW,VSWR);
  else if (screenNo==2)
    screen2(U1,U2,P1mW,P2mW,VSWR);

/*  
  outstr=String(U1,1);
  outstr.concat("dBm");
  LCDout(outstr,0,0,8);

  outstr=String(U2,1);
  outstr.concat("dBm");
  LCDout(outstr,12,0,8);
  dtostrf(U2, 4, 1, outstr4);
 
  if (abs(VSWR) > 99.9) VSWR = 99.9;

  outstr=String(VSWR,2);
  LCDout(outstr,8,2,4);
  LCDout("SWR",9,1,3);

  // Nachkommastellen und Einheit werden aus dem Sturct Array gelesen.
  //[0] Eingang1
  //[1] Eingang2
  outstr=String(P1mW_out,MBwahl[MB[0]].Nachkomma);
  outstr.concat(MBwahl[MB[0]].Unit);
  LCDout(outstr,1,1,8);

  outstr=String(P2mW_out,MBwahl[MB[1]].Nachkomma);
  outstr.concat(MBwahl[MB[1]].Unit);
  LCDout(outstr,13,1,7);

  outstr="AT:";
  outstr.concat(String(Koppler1,1));
  LCDout(outstr,0,2,7);

 
  outstr="AT:";
  outstr.concat(String(Koppler2,1));
  LCDout(outstr,12,2,7);
  
  //Den zu zeichnenden Bargraph mit dem jeweiligen Messbereich skalieren und auf 999 begrenzen
  //lbg_draw_val_limited = int(1000 * P1mW / (0.0008 * pow(10, MB1 - 1)));
  lbg_draw_val_limited = int(1000 * P1mW / (MBwahl[MB[0]].range));
  if (lbg_draw_val_limited > 999) lbg_draw_val_limited = 999;
  lbgPWR.drawValue(lbg_draw_val_limited, 1000);
  outstr="["+ MBwahl[MB[0]].Text+"]";
  LCDout(outstr,13,3,7);


  lcd.setCursor(10, 0);
  lcd.print(" ");
  lcd.setCursor(10, 0);

  lcd.print(item_pos);

  edit = true;
  resetCursor();

 

*/


}


byte MB_Wahl(byte kanal,float PmW) {
  // Messbereichsumschaltung anhand MBStruct
  // Arbeitet mit globalenb Variablen MB[] und MBtime[]
  unsigned long switch_MB = millis();
  byte i;
  byte MBakt;
  MBakt=MB[kanal];
  // Grenzen des Bereichs min 0.05 * range, max 08*range
  float  minP=MBwahl[MBakt].range *0.05;
  float maxP=MBwahl[MBakt].range *0.8;
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
  Serial.print(rawU1 / 1024 * UREF / scalFactor1 / UFACTOR);
  Serial.print("V)  U2:");
  Serial.print((int)rawU2);
  Serial.print("(");
  Serial.print(rawU2 / 1024 * UREF / scalFactor2 / UFACTOR);
  Serial.println("V)");
}