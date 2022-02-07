#include <Event.h>
#include <Timer.h>


#include <LiquidCrystal.h>
#include <LcdBarGraph.h>
#include <stdio.h>
#include "CRC8.h"
#include <EEPROM.h>



// Debug Modus, zusätzliche Ausgaben in der seriellen Konsole
// #define DEBUG

#define DM6TT
//#define DL2FW

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
////////////////////////////////////////////////////////////////////////

#ifdef DM6TT
// ENCODER und Interrupt Kram
#define PINA_INPUT INPUT
#define PINB_INPUT INPUT
#define PUSHB_INPUT INPUT

//#define PINA_INPUT INPUT_PULLUP
//#define PINB_INPUT INPUT_PULLUP
//#define PUSHB_INPUT INPUT_PULLUP


#define PINA_INT  FALLING
#define PUSHB_INT FALLING

#define ATT_KOPPLER1 50
#define ATT_KOPPLER2 30

#define REFERENCE INTERNAL2V56

float scalFactor1 = 1.397;

float scalFactor2 = 1.39 ;

#endif
////////////////////////////////////////////////////////////////////////
#ifdef DL2FW
// ENCODER und Interrupt Kram
#define PINA_INPUT INPUT_PULLUP
#define PINB_INPUT INPUT_PULLUP
#define PUSHB_INPUT INPUT_PULLUP

#define PINA_INT  CHANGE
#define PUSHB_INT FALLING

#define ATT_KOPPLER1 0
#define ATT_KOPPLER2 0


#define REFERENCE INTERNAL1V1

float scalFactor1 = 1.0;

float scalFactor2 = 1.0 ;

#endif
////////////////////////////////////////////////////////////////////////


// Auskoppeldämfpung in dB
//#define ATT_KOPPLER1 50
//#define ATT_KOPPLER2 30
// welche Ref. für den AD Wandler soll genutzt werden?
//#define REFERENCE INTERNAL2V56


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

#define QRG 5 // Test Frequenz Index des QRGarray

// Anzeigeformate
// 2 --> 3 Darsatellungen 0,1,2
#define MAXSCREEN 2

// Anzahl der Elemente im QRGarray[]
#define QRGSIZE 17

// Anzahl der zu verwaltenden Kopppler, durch EEPROM Größe beschränkt
#define KOPPLERSIZE 4


// Anzahl der Menueeintrage
#define MENUSIZE 4

// Anzahl des CFG Werte im DUAL Modus
#define DUALSIZE 4

// Status für check_encoder, wo wir gerade stehen
enum mStates {
  SWR,
  IN1,
  IN2,
  MENU,
  DUAL,
  DUALCFG,
  FRQ,
  CFG1,
  CFG2,
  KOPPLER,
};

enum kStates {
  SELECT,
  CONF,
  EDIT,
  EXIT,
  DEL,
};



// Startmeldungen
#define START1 "   PowerMeter"
#define START2 "   1MHz ... 10GHz"
#define START3 "  K08 DL2FW/DM6TT"
#define START4 "  ---------------"



// Makros, fuer Strings zu leeren
#define EMPTY(a) for(i=0;i<sizeof(a);i++) a[i]='\0'

// Skalierungsfaktor zur Umrechnung der Messwerte, wir später beim kalibrieren ermittelt
//float scalFactor1 = 1.0;
//float scalFactor1 = 1.397;
//0.7877 ;
//float scalFactor2 = 1.39 ;

float frequenz = 0; // Frequenz in MHz, wird aus QRGarray bestimmt
byte frequenzIdx = QRG; // Index in QRGArry


const float smooth = 0.004;
//> timeconstant=200ms (app. 1/smooth * TA (10ms)) @smooth=0.01-> timeconstant=1000ms ... - correct for smooth << 1

struct LinStruct {
  int dB;
  float ADnorm;
};

struct LinStruct linM0[17];

// Messbereichswahl

struct MBStruct {
  float range;         // Bereich Anfang= *0.05 Ende *0.8
  int tLen;            // min. Zeitraum
  byte Nachkomma;      // anzuzeigende Nachkommastellen
  float Divisor;       // Devisor für die Darstellung
  char Unit[10];        // Einheit
  char Text[10];        // Anzeigetext des Bereiches
};

struct EEStruct {
  float scalFactor[2];  // Skalierung der beiden Kanaäle
  float attKanal[2];    //Daempfungskonstatnte pro Kanal
  float attKoppler[2];  // Richtkoppler Auskopplung
  byte qrg[2];          // Messfrewuenzen pro Kanal
  byte SWRqrg;          // Messfrequenz für die SWR Messung
  boolean calibrated[2]; // sind die Kanäle kalibriert?
  byte CRC;            // Pruefsumme
};

struct QRGstruct {
  float frequency;      // Frequenz in KHz
  char Text[10];      // Anzeigetext
};

struct KOPPLERstruct {
  boolean configured;              // wird
  //byte  qrgIdx[QRGSIZE];      // Index de Frequenz, bezieht sich auf QRGstruct/QRGarray
  float sigDB[QRGSIZE];       // Auskoppeldaempfung
  byte minQRGidx;             // tiefste nutzbare Frequenz (Index, siehe oben)
  byte maxQRGidx;             // größte Nutzbare Frequenz
  float attenuation;          // Daempfung
  char name[20];              // Namen des Kopplers
};

struct MBStruct MBwahl[10];
struct EEStruct EEprom;
struct QRGstruct QRGarray[QRGSIZE];
struct KOPPLERstruct KOPPLERarray[KOPPLERSIZE];

mStates menuState = SWR;
mStates oldState = IN1;
mStates CFGstate = FRQ;


int posi = 0;


byte MB[2]; //Messbereich
unsigned long MBtime[2]; // Dauer im Messbereich fuer Auswahl

int MB1 = 5;  //Reserve nach unten hin
int MB2 = 5;  //Reserve nach unten hin
float P1mW = 0.0;
float P2mW = 0.0;
Timer t;

// eingelesene Werte
float smoothU1 = 0.0; // takes averaged but unscaled value
float smoothU2 = 0.0;


float U1 = 0.0; // scaled values!
float U2 = 0.0;


// Auskoppeldämpfung der Koppler
float Koppler1 = ATT_KOPPLER1;  //dB
float Koppler2 = ATT_KOPPLER2;  //dB
float AttKanal1 = 0;
float AttKanal2 = 0;
byte QRGidx1 = 0;
byte QRGidx2 = 0;



volatile boolean turned;   // rotary was turned
volatile boolean fired;    // knob was pushed
volatile boolean up;       // true when turned cw

//#########################################################
// menu handling
int Menu_page = 1;
byte menuPos = 0;
int item_pos = 1;

boolean edit = false;
boolean change_value = false;
int delta_value = 0;
boolean items_changed = false;
//############################################################

//Cursorhandling im DualCFG Modus
byte dualX, dualY;
byte dualPos;

boolean refresh = true; // soll Bildschirm neu aufgebaut werden?


CRC8 crc;


// Anzeige
byte screenNo = 0; // Auswahl der Darstellung, max. Wert MAXSCREEN
byte oldScreenNo = 0;

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
  // wenn keine Taste gedrueckt wurde und nicht gedreht wurde, haben wir nichts zu tun
  if (!fired && !turned)
    return;

  switch (menuState) { // wir schauen, in welchem Status wir uns befiunden. Hier nutzen wir ein enum der besseren Lesbarkeit.
    case SWR:  // Hauptschirm
      //
      /*
        if (fired) { // Taste wurde gedrueckt
        fired = false;
        frequenzIdx = chooseQRG(frequenzIdx);
        stopTasks();
        frequenz = QRGarray[frequenzIdx].frequency;
        writeEEPROM();
        startTasks();
        refresh=true;
        }
      */
      if (fired) { // Taste wurde gedrueckt
        fired = false;
        menuState = MENU;
      }
      if (turned && up) {
        turned = false;
        menuState = DUAL;
        screenNo = 1;
      }
      if (turned && !up) {
        turned = false;
        menuState = IN2;
        screenNo = 2;
      }
      break;
    case DUAL:
      if (turned && up) {
        turned = false;
        menuState = IN1;
        screenNo = 1;
      }
      if (turned && !up) {
        turned = false;
        menuState = SWR;
        screenNo = 2;
      }
      if (fired) { // Taste wurde gedrueckt
        fired = false;
        menuState = DUALCFG;
      }
      break;
    case IN1:   // Eingang1 Anzeige
      if (turned && up) {
        turned = false;
        menuState = IN2;
        screenNo = 2;
      }
      if (turned && !up) {
        turned = false;
        menuState = DUAL;
        screenNo = 0;
      }
      if (fired) {
        fired = false;
        stopTasks();
        calibration(1, &scalFactor1, PIN_A1, AttKanal1);
        startTasks();
        writeEEPROM();
        refresh = true;
      }
      break;
    case   IN2:   // Eingang2 Anzeige
      if (turned && up) {
        turned = false;
        menuState = SWR;
        screenNo = 0;
      }
      if (turned && !up) {
        turned = false;
        menuState = IN1;
        screenNo = 1;
      }
      if (fired) {
        fired = false;
        stopTasks();
        calibration(2, &scalFactor2, PIN_A2, AttKanal2);
        startTasks();
        writeEEPROM();
        refresh = true;
      }
      break;
    case DUALCFG:
      if (turned && up)
        dualPos < DUALSIZE ? dualPos++ : (dualPos = 0);
      else if (turned && !up)
        dualPos > 0 ? dualPos-- : (dualPos = DUALSIZE);
      turned = false;
      if (fired) {
        stopTasks();
        fired = false;
        switch (dualPos) {
          case 0: // Frequenz Kanal1
            QRGidx1 = editQRG(QRGidx1, 4, 2);
            refresh = true;
            menuState = DUAL;
            break;
          case 1: // Attenuation Kanal1
            AttKanal1 = editFloat(AttKanal1, 4, 3, 4, 1, 0.1);
            refresh = true;
            menuState = DUAL;
            break;
          case 2: // Frequenz Kanal2
            QRGidx2 = editQRG(QRGidx2, 14, 2);
            refresh = true;
            menuState = DUAL;
            break;
          case 3: // Attenuation Kanal2
            AttKanal2 = editFloat(AttKanal2, 14, 3, 4, 1, 0.1);
            refresh = true;
            menuState = DUAL;
            break;
        }
        startTasks();
      }
      break;
    case MENU:    // wir sind im Menu
      //
      if (turned && up)
        menuPos < (MENUSIZE - 1) ? menuPos++ : (menuPos = 0);
      else if (turned && !up)
        menuPos > 0 ? menuPos-- : (menuPos = (MENUSIZE - 1));
      turned = false;
      //Serial.print("Conf-Menu menuPos:");
      //Serial.println(menuPos);
      if (fired) {
        // Serial.println("Fired!");;
        stopTasks();
        fired = false;
        switch (menuPos) {
          case 0: // Frequenz
            frequenzIdx = chooseQRG(frequenzIdx);
            frequenz = QRGarray[frequenzIdx].frequency;
            writeEEPROM();
            refresh = true;
            menuState = SWR;
            break;
          case 1: // Konfiguration Kanal 1
            configureInput(1);
            AttKanal1 = EEprom.attKanal[0];
            Koppler1 = EEprom.attKoppler[0];
            writeEEPROM();
            menuState = SWR;
            refresh = true;
            break;
          case 2: // Konfiguration Kanal 2
            configureInput(2);
            AttKanal2 = EEprom.attKanal[1];
            Koppler1 = EEprom.attKoppler[1];
            writeEEPROM();
            menuState = SWR;
            refresh = true;
            break;
          case 3: // Konfiguration Koppler
            Serial.println("Konfiguriere Koppler");
            configureKoppler();
            //writeEEPROM();
            menuState = SWR;
            break;
        }
        startTasks();
        //break;

      }
      break;
  }
  turned = false;
  fired =  false;
  return;
}

// Initialisiere LCD Display
LiquidCrystal lcd(LCD_RS, LCD_EN, LCD_D4, LCD_D5, LCD_D6, LCD_D7); // -- creating LCD instance

LcdBarGraph lbgPWR(&lcd, 13, 0, 3); //


void setup() {
  boolean waitFired = false; // warte auf Tastendruck
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
  MB[0] = 4;
  MB[1] = 4;


  // Messbereichswahl

  MBwahl[0].range = 1E-3;   MBwahl[0].tLen = 1; MBwahl[0].Nachkomma = 1;  MBwahl[0].Divisor = 1E-6;  strcpy(MBwahl[0].Unit, "nW\0"); strcpy(MBwahl[0].Text, "1uW\0");
  MBwahl[1].range = 1E-2;   MBwahl[1].tLen = 1; MBwahl[1].Nachkomma = 3;  MBwahl[1].Divisor = 1E-3;  strcpy(MBwahl[1].Unit, "uW\0"); strcpy(MBwahl[1].Text, "10uW\0");
  MBwahl[2].range = 1E-1;   MBwahl[2].tLen = 1; MBwahl[2].Nachkomma = 2;  MBwahl[2].Divisor = 1E-3;  strcpy(MBwahl[2].Unit, "uW\0"); strcpy(MBwahl[2].Text, "100uW\0");
  MBwahl[3].range = 1E0;    MBwahl[3].tLen = 1; MBwahl[3].Nachkomma = 1;  MBwahl[3].Divisor = 1E-3;  strcpy(MBwahl[3].Unit, "uW\0"); strcpy(MBwahl[3].Text, "1mW\0");
  MBwahl[4].range = 1E1;    MBwahl[4].tLen = 1; MBwahl[4].Nachkomma = 3;  MBwahl[4].Divisor = 1E0;   strcpy(MBwahl[4].Unit, "mW\0"); strcpy(MBwahl[4].Text, "10mW\0");
  MBwahl[5].range = 1E2;    MBwahl[5].tLen = 1; MBwahl[5].Nachkomma = 2;  MBwahl[5].Divisor = 1E0;   strcpy(MBwahl[5].Unit, "mW\0"); strcpy(MBwahl[5].Text, "100mW\0");
  MBwahl[6].range = 1E3;    MBwahl[6].tLen = 1; MBwahl[6].Nachkomma = 1;  MBwahl[6].Divisor = 1E0;   strcpy(MBwahl[6].Unit, "mW\0"); strcpy(MBwahl[6].Text, "1W\0");
  MBwahl[7].range = 1E4;    MBwahl[7].tLen = 1; MBwahl[7].Nachkomma = 3;  MBwahl[7].Divisor = 1E3;   strcpy(MBwahl[7].Unit, "W\0");  strcpy(MBwahl[7].Text, "10W\0");
  MBwahl[8].range = 1E5;    MBwahl[8].tLen = 1; MBwahl[8].Nachkomma = 2;  MBwahl[8].Divisor = 1E3;   strcpy(MBwahl[8].Unit, "W\0");  strcpy(MBwahl[8].Text, "100W\0");

  // Frequnzen
  QRGarray[0].frequency = 1.800;      strcpy(QRGarray[0].Text, "160m\0");
  QRGarray[1].frequency = 3.500;      strcpy(QRGarray[1].Text, "80m\0");
  QRGarray[2].frequency = 7.000;      strcpy(QRGarray[2].Text, "40m\0");
  QRGarray[3].frequency = 10.000;     strcpy(QRGarray[3].Text, "30m\0");
  QRGarray[4].frequency = 14.000;     strcpy(QRGarray[4].Text, "20m\0");
  QRGarray[5].frequency = 18.000;     strcpy(QRGarray[5].Text, "17m\0");
  QRGarray[6].frequency = 21.000;     strcpy(QRGarray[6].Text, "15m\0");
  QRGarray[7].frequency = 24.000;     strcpy(QRGarray[7].Text, "12m\0");
  QRGarray[8].frequency = 28.000;     strcpy(QRGarray[8].Text, "10m\0");
  QRGarray[9].frequency = 50.000;     strcpy(QRGarray[9].Text, "6m\0");
  QRGarray[10].frequency = 72.000;    strcpy(QRGarray[10].Text, "4m\0");
  QRGarray[11].frequency = 144.000;   strcpy(QRGarray[11].Text, "2m\0");
  QRGarray[12].frequency = 433.000;   strcpy(QRGarray[12].Text, "70cm\0");
  QRGarray[13].frequency = 1200.000;  strcpy(QRGarray[13].Text, "23cm\0");
  QRGarray[14].frequency = 2400.000;  strcpy(QRGarray[14].Text, "13cm\0");
  QRGarray[15].frequency = 5000.000;  strcpy(QRGarray[15].Text, "6cm\0");
  QRGarray[16].frequency = 10000.000; strcpy(QRGarray[16].Text, "3cm\0");

  frequenz = QRGarray[QRG].frequency;


  // Hier das eigentliche "Multitasking"
  // alle 1000ms Ausgabe der Rohsignal an den PC
  // alle 10ms die A/D-Wandler einlesen
  // alle 100ms das LCD beschreiben
  // alle 20ms  auf Encoderereignisse reagieren

  startTasks();

  // einladen der im EEProm gspeicherten Werte
  // anhand der Checksumme schauen wir, ob die Daten konsitent sind.

  if (!readEEPROM()) {
    lcd.setCursor(0, 3);
    lcd.print("EEPROM leer, init..");
    EEprom.calibrated[0] = false;
    EEprom.calibrated[1] = false;
    writeEEPROM();

    if (!readEEPROM()) {
      lcd.setCursor(0, 3);
      lcd.print("EEPROM ERROR!!!   ");
      while (1);
    }
  }
  else {
    lcd.setCursor(0, 3);
    lcd.print("Lese EEPROM...");
  }
  delay(1000);
  lcd.clear();
  // Sind die Kanaele kalibiert?

  if (!EEprom.calibrated[0]) {
    lcd.setCursor(0, 0);
    Serial.println("Kanal 1 unkalbibiert");
    lcd.print("Kanal 1 unkalbibiert");
    waitFired = true;
  }
  if (!EEprom.calibrated[1]) {
    lcd.setCursor(0, 1);
    lcd.print("Kanal 2 unkalbibiert");
    Serial.println("Kanal 2 unkalbibiert");
    waitFired = true;
  }
  if (waitFired) {
    lcd.setCursor(0, 3);
    lcd.print("Taste druecken");
    while (!fired) {};
    fired = false;
  }
  lcd.clear();

}

void loop()
{

  t.update(); //timer handling!	mehr passiert in der Hauptschleife nicht, Multitasking läuft im Hintergrund

}

boolean readEEPROM() {
  byte checkCRC = 0;

  /*
    struct EEStruct {
      float scalFactor[2];  // Skalierung der beiden Kanaäle
      float attKanal[2];    //Daempfungskonstatnte pro Kanal
      float attKoppler[2];  // Richtkoppler Auskopplung
      byte qrg[2];          // Messfrewuenzen pro Kanal
      byte SWRqrg;          // Messfrequenz für die SWR Messung
      byte CRC;            // Pruefsumme
    };


    struct MBStruct MBwahl[10];
    struct EEStruct EEprom;
  */
  Serial.print("Lese EEPROM ...");
  int eeAddress = 0;
  EEPROM.get(eeAddress, EEprom);
  crc.reset();
  checkCRC = EEprom.CRC;
  Serial.print("  CRC read:");
  Serial.print(EEprom.CRC);
  EEprom.CRC = 0; // sonst stimmt die CRC Summe nicht, sie darf nicht  mit eingerechnet werden
  crc.add((uint8_t *)&EEprom, sizeof(EEprom));
  Serial.print("  CRC calc:");
  Serial.println(crc.getCRC());
  if ( crc.getCRC()  != checkCRC) {
    Serial.println("EEPROM Checksumme falsch");
    return false;
  }
  else {
    Serial.println("EEPROM Checksumme OK");
    // setzen der globalen Variablen
    scalFactor1 = EEprom.scalFactor[0];
    scalFactor2 = EEprom.scalFactor[1];
    AttKanal1 = EEprom.attKanal[0];
    AttKanal2 = EEprom.attKanal[1];
    Koppler1 = EEprom.attKoppler[0];
    Koppler2 = EEprom.attKoppler[1];
    frequenzIdx = EEprom.SWRqrg;
    frequenz = QRGarray[frequenzIdx].frequency;
    QRGidx1 = EEprom.qrg[0];
    QRGidx2 = EEprom.qrg[1];
    //Serial.print("QRG idx:");
    //Serial.println(frequenzIdx);

    return true;
  }
}

void writeEEPROM() {
  int eeAddress = 0;

  Serial.print("Schreibe EEPROM...");


  EEprom.scalFactor[0] = scalFactor1;
  EEprom.scalFactor[1] = scalFactor2;
  EEprom.attKanal[0] = AttKanal1;
  EEprom.attKanal[1] = AttKanal2;
  EEprom.attKoppler[0] = Koppler1;
  EEprom.attKoppler[1] = Koppler2;
  EEprom.SWRqrg = frequenzIdx;
  // wir speichern nicht die bei den kanaelen gewwahlte Frequenz, die ist nur fuer die temporäre Konfiguration gedacht
  // muss evtl. nochmal überdacht werden, ob das so Sinn macht.
  EEprom.qrg[0] = frequenzIdx;
  EEprom.qrg[1] = frequenzIdx;
  EEprom.CRC = 0;

  crc.reset();
  crc.add((uint8_t *)&EEprom, sizeof(EEprom));
  EEprom.CRC = crc.getCRC();

  Serial.print(" CRC:");
  Serial.println(EEprom.CRC);

  EEPROM.put(eeAddress, EEprom);

}


void startTasks () {
  t.every(1000, write_raw_seriell, 0);
  t.every(10, take_ads, 0); //lese die analogen Eingänge alle 10ms und glätte diese
  t.every(100, write_lcd, 0); //alle 500ms auf LCD darstellen
  t.every(50, check_encoder, 0); // alle 20ms Änderungen des Encoders detektieren
}

void stopTasks() {
  t.stop(write_raw_seriell);
  t.stop(take_ads);
  t.stop(write_lcd);
  t.stop(check_encoder);
}

byte chooseQRG(byte idx) {
  // Frequenzauswahl mit Encoder, Werte aus dem Array QRGarray
  byte qrgIdx = 0;
  float freq = QRGarray[idx].frequency;
  char  *text = QRGarray[idx].Text;
  byte i;
  char outstr[21];
  char outstr1[21];
  byte maxIdx = sizeof(QRGarray) / sizeof(struct QRGstruct) - 1 ; // groesster Index bei den Frequenzen
  //stopTasks();
  lcd.clear();
  EMPTY(outstr);
  strcpy(outstr, "Frequenzbereich");
  LCDout(outstr, 0, 0, 18);
  EMPTY(outstr);
  while (!fired) { //Solange keine Taste gerueckt ist, bleiben wir hier
    if (turned) {
      turned = false;
      if (up)
        idx < maxIdx ? idx++ : (idx = 0);
      else
        idx > 0 ? idx-- : (idx = maxIdx);
    }
    freq = QRGarray[idx].frequency;
    text = QRGarray[idx].Text;
    EMPTY(outstr);
    strcpy(outstr, "Band:");
    LCDout(outstr, 0, 1, 5);
    EMPTY(outstr);
    strcat(outstr, text);
    LCDout("          ", 6, 1, 10);
    LCDout(outstr, 6, 1, 10);
    EMPTY(outstr);
    strcpy(outstr, "Freq:");
    LCDout(outstr, 0, 2, 5);
    EMPTY(outstr);
    dtostrf(freq, 5, 1, outstr);
    LCDout("          ", 6, 2, 10);
    LCDout(outstr, 6, 2, 5);
    LCDout("MHz", 12, 2, 5);
    EMPTY(outstr);
    strcpy(outstr, "Idx:");
    LCDout(outstr, 0, 3, 5);
    EMPTY(outstr);
    itoa(idx, outstr, 10);
    LCDout("     ", 6, 3, 5);
    LCDout(outstr, 6 , 3, 5);

    while (!turned && !fired) {}; // wir warten auf Tastendruck oder Encoder

  }
  // Tasks wieder einschalten
  //startTasks();
  return idx;
  fired = false;
  turned = false;
}

void calibration(byte kanal, float * scalFactor, byte pin, float Att) {
  // Kalibrierung des Kanals bei limM0[CALIBRATE]
  float dbM = linM0[CALIBRATE].dB;
  float ref = linM0[CALIBRATE].ADnorm;
  char outstr[30];
  byte i, count = 0;
  float oldScal = -999;
  float rawU = linM0[HIGH_LIMIT].ADnorm;
  int mp;
  float smooth = 0.1;
  boolean ready = false;
  byte stage = 1;


  // Tasks anhalten, wir machen hier alles selsbt
  //stopTasks();
  lcd.clear();
  EMPTY(outstr);
  strcpy(outstr, "Kalibrierung Kanal");
  LCDout(outstr, 0, 0, 18);
  //LCDout("Kalibrierung Kanal",0,0,18);
  EMPTY(outstr);
  itoa(kanal, outstr, 10);
  LCDout(outstr, 19, 0, 1);
  EMPTY(outstr);
  LCDout("P:", 0, 1, 6);
  dtostrf(dbM + Att, 5, 1, outstr);
  LCDout(outstr, 2, 1, 5);
  LCDout("dBm", 7, 1, 3);
  if (Att > 0) {
    LCDout("Att:", 11, 1, 3);
    EMPTY(outstr);
    dtostrf(Att, 4, 1, outstr);
    LCDout(outstr, 15, 1, 4);
  }
  EMPTY(outstr);
  LCDout("Bitte Signal ...", 0, 2, 19);
  LCDout("Taste druecken", 0, 3, 19);
  while (!fired && !turned) { }
  if (turned) {
    turned = false;
    LCDout("                ", 0, 2, 19);
    LCDout("              ", 0, 3, 19);
    LCDout("Abbruch...", 0, 3, 17);
    delay(1000);
    //startTasks();
    return;
  }
  // wir warten auf ein Signal
  // wir gehen von einen Skalierungsbereich von 0.5-1.5 aus
  rawU = (float)analogRead(pin);
  while (((ref * 0.5) > rawU) ||
         ((ref * 1.5)   < rawU)) {
    delay(20);
    rawU = (float)analogRead(pin);
    LCDout("              ", 0, 3, 19);
    LCDout("Warte...", 0, 3, 17);
    if (turned) {
      turned = false;
      return;
    }
  }

  fired = false;
  LCDout("      ", 13, 1, 7);
  LCDout("                ", 0, 2, 19);
  LCDout("              ", 0, 3, 19);
  LCDout("Ref:", 0, 2, 4);
  dtostrf(ref, 6, 1, outstr);
  LCDout(outstr, 4, 2, 6);
  EMPTY(outstr);
  LCDout("MW:", 11, 2, 3);
  EMPTY(outstr);
  LCDout("Scal:", 0, 3, 5);
  dtostrf(*scalFactor, 5, 3, outstr);
  LCDout(outstr, 5, 3, 5);
  //LCDout("0/9",16,1,3);
  // Anfangswert setzen
  rawU = (float)analogRead(pin) * *scalFactor;
  // wir koennen nicht auf die MW  per Task warten, wir müssen die Messungen selbst aufrufen
  //while( int(*scalFactor*10000) != int(oldScal *10000)) {// drei Stellen hinter dem Komma stabil
  while ((stage < 5) || (count < 50)) { // muss bis Stage 5 laufen und dort 50 Runden drehen
    oldScal = *scalFactor;
    mp = (float)analogRead(pin);
    rawU = (1.0 - smooth) * rawU + (smooth * mp);
    *scalFactor = ref / rawU;
    EMPTY(outstr);
    dtostrf(rawU, 6, 1, outstr);
    LCDout(outstr, 14, 2, 6);
    EMPTY(outstr);
    LCDout("AK:", 11, 3, 3);
    itoa(mp, outstr, 10);
    LCDout(outstr, 15, 3, 4);
    EMPTY(outstr);
    dtostrf(*scalFactor, 5, 3, outstr);
    LCDout(outstr, 5, 3, 5);

    // Wir erhöhen die Glaettung nach Genauigkeit
    // 5 Stufen, count empirisch ermittelt
    // smooth startet mit 0.1
    // stage startet bei 1

    if ((stage == 1) && (count == 60) && (int(*scalFactor * 10) == int(oldScal * 10))) {
      smooth = 0.01;
      stage = 2;
      count = 0;
    }
    else if ((stage == 2) && (count == 100) && (int(*scalFactor * 100) == int(oldScal * 100))) {
      smooth = 0.001;
      stage = 3;
      count = 0;
    }
    else if ( (stage == 3) && (count == 100) && (int(*scalFactor * 1000) == int(oldScal * 1000))) {
      smooth = 0.0001;
      stage = 4;
      count = 0;
    }
    else if ((stage == 4) && (count == 50) && (int(*scalFactor * 1000) == int(oldScal * 1000))) {
      stage = 5;
      LCDout("*", 13, 2, 1);
      count = 0;
    }
    count++;
    EMPTY(outstr);
    itoa(count, outstr, 10);
    //LCDout(outstr,12,1,1);
    EMPTY(outstr);
    itoa(stage, outstr, 10);
    LCDout(outstr, 12, 1, 1);
    EMPTY(outstr);
    dtostrf(smooth, 4, 4, outstr);
    LCDout(outstr, 14, 1, 6);
    delay(40);
    if (fired) {
      fired = false;
      break;
    }
  }
  EEprom.calibrated[kanal - 1] = true;
  LCDout("Kalibrierung fertig ", 0, 0, 20);
  //writeEEPROM();
  delay(3000);
  // Tasks wieder einschalten
  //startTasks();


}
byte editQRG(byte idx, byte x, byte y) {
  char outstr[21];
  byte i;
  EMPTY(outstr);
  LCDout("    ", x, y, 4);
  LCDout(QRGarray[idx].Text, x, y, 4);
  lcd.setCursor(x, y);
  lcd.blink();;
  while (!fired) {
    if (turned && up)
      idx < (QRGSIZE - 1) ? idx++ : (idx = 0);
    else if (turned && !up)
      idx > 0 ? idx-- : (idx = (QRGSIZE - 1));
    if (turned) {
      LCDout("    ", x, y, 4);
      LCDout(QRGarray[idx].Text, x, y, 4);
      turned = false;
    }
    delay(20);
  }
  fired = false;
  turned = false;
  lcd.noBlink();
  lcd.noCursor();
  return idx;
}

float editFloat(float inValue, byte x, byte y, byte len, byte frac, float fStep) {
  char outstr[21];
  byte i;
  EMPTY(outstr);
  dtostrf(inValue, len, frac, outstr);
  LCDout(outstr, x, y, len);
  lcd.setCursor(x, y);
  lcd.blink();;
  while (!fired) {
    if (turned && up) {
      inValue += fStep;
    }
    else if (turned && !up) {
      //if((inValue-fStep) >= -0.0)
      inValue -= fStep;
    }
    if (turned) {
      //Serial.println("Edit turned");
      EMPTY(outstr);
      dtostrf(inValue, len, frac, outstr);
      LCDout(outstr, x, y, len);
      turned = false;
    }
    delay(20);
  }
  fired = false;
  turned = false;
  lcd.noBlink();
  lcd.noCursor();
  return inValue;
}

void configureInput(byte kanal) {
  char outstr[21];
  byte i;
  byte pos = 1;
  byte oldPos = 0;
  const byte mSize = 3;
  lcd.clear();
  EMPTY(outstr);
  strcpy(outstr, "Konf. Kanal");
  LCDout(outstr, 0, 0, 13);
  EMPTY(outstr);
  itoa(kanal, outstr, 10);
  LCDout(outstr, 13, 0, 1);
  LCDout("Eingang Att:", 0, 1, 13);
  EMPTY(outstr);
  dtostrf(EEprom.attKanal[kanal - 1], 4, 1, outstr);
  LCDout(outstr, 13, 1, 5);
  LCDout("dB", 18, 1, 3);
  LCDout("Koppel Att:", 0, 2, 13);
  EMPTY(outstr);
  dtostrf(EEprom.attKoppler[kanal - 1], 4, 1, outstr);
  LCDout(outstr, 13, 2, 5);
  LCDout("dB", 18, 2, 3);
  LCDout("<EXIT>", 0, 3, 10);
  oldPos = 1;
  LCDout("<", 12, pos, 1);
  while (1) {
    if (turned) {
      if  (up)
        pos < mSize ? pos++ : (pos = 1);
      else if (!up)
        pos > 1 ? pos-- : (pos = mSize);
      turned = false;
      LCDout("<", 12, pos, 1);
      LCDout(" ", 12, oldPos, 1);
      oldPos = pos;
      lcd.setCursor(12, pos);
    }
    if (fired) {
      fired = false;
      //Serial.print("Pos");
      //Serial.println(pos);
      if (pos == 1) { // wir aendern die Eingangsdaempfung
        EEprom.attKanal[kanal - 1] = editFloat(EEprom.attKanal[kanal - 1], 13, 1, 4, 1, 0.1);
      }
      else if (pos == 2) {
        EEprom.attKoppler[kanal - 1] = editFloat(EEprom.attKoppler[kanal - 1], 13, 2, 4, 1, 0.1);
      }
      else if (pos == 3) {
        lcd.noCursor();
        lcd.noBlink();
        return;

      }
    }
    delay(20);
  }

}

void refreshKoppler(byte koppler) {
  // Ausgabe der Kopplerdaten
  boolean configured = false;
  char outstr[21];
  byte i;
  byte minIdx, maxIdx;

  LCDout("Koppler:", 0, 0, 8);
  EMPTY(outstr);
  itoa(koppler, outstr, 10);
  LCDout(outstr, 9, 0, 8);
  LCDout("Min:", 0, 1, 4);
  LCDout("Max:", 10, 1, 4);
  LCDout("Daempfung:", 0, 2, 10);
  LCDout("EXIT  EDIT  DEL", 0, 3, 20);
  configured = KOPPLERarray[koppler].configured;

  if (configured) { // den haben wir konfiguriert
    LCDout("[konf.]  ", 11, 0, 9);
    minIdx = KOPPLERarray[koppler].minQRGidx;
    maxIdx = KOPPLERarray[koppler].maxQRGidx;
    LCDout(QRGarray[minIdx].Text, 5, 1, 4);
    LCDout(QRGarray[maxIdx].Text, 14, 1, 4);
    EMPTY(outstr);
    dtostrf(KOPPLERarray[koppler].attenuation, 4, 1, outstr);
    LCDout(outstr, 10, 2, 4);
    LCDout("dB", 14, 2, 2);
  }
  else {
    LCDout("[NO konf].", 11, 0, 9);
    LCDout("    ", 5, 1, 4);
    LCDout("    ", 14, 1, 14);
    LCDout("      ", 10, 2, 6);
  }
}

void refreshPosition(byte edtPos) {
  switch (edtPos) {
    case 0:
      LCDout("<", 9, 1, 1);
      LCDout(" ", 19, 1, 1);
      LCDout(" ", 18, 2, 1);
      LCDout(" ", 5, 3, 1);
      LCDout(" ", 5, 3, 1);
      LCDout(" ", 17, 3, 1);
      break;
    case 1:
      LCDout(" ", 9, 1, 1);
      LCDout("<", 19, 1, 1);
      LCDout(" ", 18, 2, 1);
      LCDout(" ", 5, 3, 1);
      LCDout(" ", 11, 3, 1);
      LCDout(" ", 17, 3, 1);
      break;
    case 2:
      LCDout(" ", 9, 1, 1);
      LCDout(" ", 19, 1, 1);
      LCDout("<", 18, 2, 1);
      LCDout(" ", 5, 3, 1);
      LCDout(" ", 11, 3, 1);
      LCDout(" ", 17, 3, 1);
      break;
    case 3:
      LCDout(" ", 9, 1, 1);
      LCDout(" ", 19, 1, 1);
      LCDout(" ", 18, 2, 1);
      LCDout("<", 5, 3, 1);
      LCDout(" ", 11, 3, 1);
      LCDout(" ", 17, 3, 1);
      break;
    case 4:
      LCDout(" ", 9, 1, 1);
      LCDout(" ", 19, 1, 1);
      LCDout(" ", 18, 2, 1);
      LCDout(" ", 5, 3, 1);
      LCDout("<", 11, 3, 1);
      LCDout(" ", 17, 3, 1);
      break;
    case 5:
      LCDout(" ", 9, 1, 1);
      LCDout(" ", 19, 1, 1);
      LCDout(" ", 18, 2, 1);
      LCDout(" ", 5, 3, 1);
      LCDout(" ", 11, 3, 1);
      LCDout("<", 17, 3, 1);
      break;
  }
}


float *editStuetzstellen(float *sigDB,byte aSize) {
  // Pflege der Stuetzstellen
  
}
void configureKoppler() {
  char outstr[21];
  byte i;
  byte koppler = 0;
  byte minIdx, maxIdx;
  boolean exit = false;
  byte edtPos = 0;
  byte edtMax = 5;
  boolean configured = false;
  boolean exitIT = false;
  float Att = 0.0;
  boolean edit2 = false;
  kStates state = SELECT;

  /*enum kStates {
    SELECT,
    CONF,
    EDIT,
    EXIT,
    DEL,
    };
  */

  lcd.clear();
  fired = false;
  turned = false;
  while (!exitIT) {
    switch (state) {
      case SELECT:    //Auswahl des Kopplers
        while (!fired) {
          refreshKoppler(koppler);
          if (turned && up)
            koppler < (KOPPLERSIZE - 1) ? koppler++ : (koppler = 0);
          else if (turned && !up)
            koppler > 0 ? koppler-- : (koppler = (KOPPLERSIZE - 1));
          turned = false;
        }
        fired = 0;
        state = CONF;
        edtPos = 0;
        break;
      case CONF:    // Konfiguration des Kopplers
        while (!fired) {
          refreshPosition(edtPos); // Pfeil < an die richtige Position setzen
          if (turned && up)
            edtPos < edtMax ? edtPos++ : (edtPos = 0);
          else if (turned && !up)
            edtPos > 0 ? edtPos-- : (edtPos = edtMax);
          turned = false;
        }
        fired = false;
        configured=KOPPLERarray[koppler].configured;
        switch (edtPos) {
          case 0:       // Min QRG
            minIdx = KOPPLERarray[koppler].minQRGidx;
            if (!configured) {
              minIdx = 0; //
              KOPPLERarray[koppler].maxQRGidx = QRGSIZE - 1;
              KOPPLERarray[koppler].attenuation = 0.0;

            }
            KOPPLERarray[koppler].minQRGidx = editQRG(minIdx, 5, 1);
            KOPPLERarray[koppler].configured = true; // nun ist er konfiguriert, Werte auch anzeigen
            // wir springen direkt in den Edit Modus der Max Frequenz
            edtPos = 1;
            fired = true;
            break;
          case 1:     // Max QRG
            maxIdx = KOPPLERarray[koppler].maxQRGidx;
            if (!configured) {
              minIdx = 0; //
              KOPPLERarray[koppler].minQRGidx = 0;
              KOPPLERarray[koppler].attenuation = 0.0;
            }
            KOPPLERarray[koppler].maxQRGidx = editQRG(maxIdx, 14, 1);
            KOPPLERarray[koppler].configured = true; // nun ist er konfiguriert, Werte auch anzeigen
            // wir springen direkt in den Edit Modus der Attenuation
            edtPos = 2;
            fired = true;
            break;
          case 2:     // Att
            Att = KOPPLERarray[koppler].attenuation;
            if (!configured) {
              Att = 0.0; //
              KOPPLERarray[koppler].minQRGidx = 0;
              KOPPLERarray[koppler].maxQRGidx = QRGSIZE - 1;
            }
            KOPPLERarray[koppler].attenuation = editFloat(Att, 10, 2, 4, 1, 0.5);
            if (!configured) {
               // nun fuellen wir das sigDB Array mit dem konfigurierten Wert
               for(i=0;i<QRGSIZE;i++)
                  KOPPLERarray[koppler].sigDB[i]=KOPPLERarray[koppler].attenuation;
            }
            KOPPLERarray[koppler].configured = true; // nun ist er konfiguriert, Werte auch anzeigen
           
            edtPos < edtMax ? edtPos++ : (edtPos = 0); //eine Position weiterspringen, wenn raus aus dem Edit
            break;
          case 3:     // Exit
            state = SELECT; // zureuck zur Kopplerauswahl
            exitIT = true;
            break;
          case 4:     // Edit
            state = EDIT;
            edtPos < edtMax ? edtPos++ : (edtPos = 0); //eine Position weiterspringen, wenn raus aus dem Edit
            break;
          case 5:     // Del
            KOPPLERarray[koppler].configured = false;
            state = SELECT; // zureuck zur Kopplerauswahl
            break;
        }
        refreshKoppler(koppler);
        break;
      case EDIT :  // Edit der Stützstellen
          //sigDB ist Array von float
          editStuetzstellen(KOPPLERarray[koppler].sigDB,QRGSIZE);
          refreshKoppler(koppler);
          state=SELECT;
          break;

    } //switch state
  } //while (außen)

} //configureKoppler




void take_ads()
{
  float mp;
  float mp1;
  float mp2;

  // zuerst schnell die Werte lesen, bevor weiter gerechnet wird
  mp1 = (float)analogRead(PIN_A1);
  mp2 = (float)analogRead(PIN_A2);

  // erster Kanal
  //mp = freq_correction(mp1 * scalFactor1, frequenz, linM0);
  mp = freq_correction(mp1 * scalFactor1, QRGarray[QRGidx1].frequency, linM0);
  // falls der MW kleiner als unsere kleinste Spannung ist, glätten wir nicht von 0 an
  if (smoothU1 < linM0[HIGH_LIMIT].ADnorm) // noch kein Wert da, dann nehmen wir mal den echten Messwert
    smoothU1 = mp;
  //Glättung
  smoothU1 = (1.0 - smooth) * smoothU1 + smooth * mp;
  // zweiter Kanal
  mp = freq_correction(mp2 * scalFactor2, QRGarray[QRGidx2].frequency, linM0);
  if (smoothU2 < linM0[HIGH_LIMIT].ADnorm) // noch kein Wert da, dann nehmen wir mal den echten Messwert
    smoothU2 = mp;
  smoothU2 = (1.0 - smooth) * smoothU2 + smooth * mp;
  // rawU2 = (1.0 - smooth) * rawU2 + smooth * (float)analogRead(PIN_A2) * scalFactor2;
}


float freq_correction(float rawU, float qrg, struct LinStruct * linM ) {
  float corrU;
  float offset;
  // definierte Werte oben im Programm
  // Limits, Index des Struct Arrays
  // #define LOW_LIMIT 16 // entspricht -70dBm
  // #define HIGH_LIMIT 2 // entspricht 0dBm

  // Werte fuer Frequenzkorrektur
  // beziehen sich auf Index des Struct Array
  // #define QRG_HIGH_CORR 10 // -40dBm
  // #define QRG_LOW_CORR 3 // -5dBm

  // #define QRG_FACTOR 0.020469
  // #define QRG_HIGH_FACTOR 244.8
  // #define QRG_LOW_FACTOR 86.4

  //linM0[0].dB = 10; linM0[0].ADnorm = 195.2;
  //linM0[1].dB = 5; linM0[1].ADnorm = 208.6;
  //linM0[2].dB = 0; linM0[2].ADnorm = 231.1;
  //linM0[3].dB = -5; linM0[3].ADnorm = 281.5;
  //linM0[4].dB = -10; linM0[4].ADnorm = 340.7;
  //linM0[5].dB = -15; linM0[5].ADnorm = 406.8;
  //linM0[6].dB = -20; linM0[6].ADnorm = 473.2;
  //linM0[7].dB = -25; linM0[7].ADnorm = 538.7;
  //linM0[8].dB = -30; linM0[8].ADnorm = 604.2;
  //linM0[9].dB = -35; linM0[9].ADnorm = 670.0;
  //linM0[10].dB = -40; linM0[10].ADnorm = 736.8;
  //linM0[11].dB = -45; linM0[11].ADnorm = 800.4;
  //linM0[12].dB = -50; linM0[12].ADnorm = 863.8;
  //linM0[13].dB = -55; linM0[13].ADnorm = 914.2;
  //linM0[14].dB = -60; linM0[14].ADnorm = 958.1;
  //linM0[15].dB = -65; linM0[15].ADnorm = 972.1;
  //linM0[16].dB = -70; linM0[16].ADnorm = 981.6;


  offset = (qrg * QRG_FACTOR); //Offset(f)= 0,020469 * Messfrequenz in MHz
  corrU = rawU + offset;   // MW = MW + Offset(f);

  // corrU ist größer als 736.8 (-40dBm)
  if (corrU > linM[QRG_HIGH_CORR].ADnorm) {
    corrU = corrU - ((corrU - linM[QRG_HIGH_CORR].ADnorm) / QRG_HIGH_FACTOR) * offset; //  MW = MW - ((MW - 736,8) / 244,8) * Offset(f);

  }
  // corrU ist größer als Wert von 0dBm und kleiner als Wert von -5dBm
  else if ((corrU > linM[HIGH_LIMIT].ADnorm) && (corrU < linM[QRG_LOW_CORR].ADnorm)) {// nur ausführen, wenn wir einen Wert >0dBm haben, aus Limits ermittelt
    corrU = corrU - ((linM[QRG_LOW_CORR].ADnorm - corrU) / QRG_LOW_FACTOR) * offset; //  W = MW - ((281,5 - MW) / 86,4 ) * Offset(f);

  }
  // Begrenzung der Werte
  //
  //if (MW > 981,6) MW = 981,6;
  //if (MW < 231,1) MW = 231,1;
  if (corrU > linM[LOW_LIMIT].ADnorm)         corrU =  linM[LOW_LIMIT].ADnorm;    //#define LOW_LIMIT 16  entspricht -70dBm
  else if (corrU < linM[HIGH_LIMIT].ADnorm)    corrU =  linM[HIGH_LIMIT].ADnorm;
  return corrU;
}

float linearize(float rawU, struct LinStruct * linM, byte linMsize ) {
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
  if (pU > (linMsize - 1))    U = linM[linMsize].dB;
  else if (pU < 1)          U = linM[0].dB;
  else                      U = ((rawU - linM[pU].ADnorm) / (linM[pU + 1].ADnorm - linM[pU].ADnorm)) * (linM[pU + 1].dB - linM[pU].dB) + linM0[pU].dB;
#ifdef DEBUG
  Serial.print("U lin=");
  Serial.println (U);
#endif
  return U;
}
void LCDout (char *outstring, byte x, byte y, byte len) {

  byte i;
  char empty[21];

  if (len > 20) return;

  // Leerstring zusammenbauen
  EMPTY(empty);
  //for (i=0;i<sizeof(empty);i++)
  //  empty[i]=" ";
  lcd.setCursor(x, y);
  lcd.print(empty);
  lcd.setCursor(x, y);
  strncpy(empty, outstring, len);
  lcd.print(empty);
  //lcd.print(outstring);
}

void screen0(float U1, float U2, float P1mW, float P2mW, float VSWR, char *QRGtext) {

  int lbg_draw_val_limited;
  char outstr[30];
  char outstr1[30];
  float P1mW_out = 0;
  float P2mW_out = 0;
  int i;


  P1mW_out = P1mW / MBwahl[MB[0]].Divisor;
  P2mW_out = P2mW / MBwahl[MB[1]].Divisor;
  outstr[0] = '\0';
  dtostrf(U1, 4, 1, outstr);
  strcat(outstr, "dBm ");
  LCDout(outstr, 0, 0, 8);

  outstr[0] = '\0';
  dtostrf(U2, 4, 1, outstr);
  strcat(outstr, "dBm ");
  LCDout(outstr, 12, 0, 8);


  if (abs(VSWR) > 99.9) VSWR = 99.9;

  //outstr=String(VSWR,2);
  outstr[0] = '\0';
  dtostrf(VSWR, 4, 2, outstr);
  LCDout(outstr, 8, 2, 4);
  outstr[0] = '\0';
  strcpy(outstr, "SWR\0");
  LCDout(outstr, 9, 1, 3);

  // Nachkommastellen und Einheit werden aus dem Sturct Array gelesen.
  //[0] Eingang1
  //[1] Eingang2

  EMPTY(outstr);
  dtostrf(P1mW_out, 5, MBwahl[MB[0]].Nachkomma, outstr);
  strcat(outstr, MBwahl[MB[0]].Unit);
  LCDout(outstr, 1, 1, 8);


  EMPTY(outstr);
  dtostrf(P2mW_out, 5, MBwahl[MB[1]].Nachkomma, outstr);
  strcat(outstr, MBwahl[MB[1]].Unit);
  LCDout(outstr, 13, 1, 7);


  EMPTY(outstr);
  //strcpy(outstr,"ATT:");
  dtostrf(Koppler1, 4, 1, outstr1);
  strcat(outstr, outstr1);
  LCDout(outstr, 0, 2, 4);
  if (AttKanal1 > 0) // wir haben daemoung im Pfad konfiguriert
    LCDout("+", 4, 2, 1);


  EMPTY(outstr);
  //strcpy(outstr,"ATT:");
  dtostrf(Koppler2, 4, 1, outstr1);
  strcat(outstr, outstr1);
  LCDout(outstr, 13, 2, 7);
  if (AttKanal2 > 0) // wir haben daemoung im Pfad konfiguriert
    LCDout("+", 17, 2, 1);

  //Den zu zeichnenden Bargraph mit dem jeweiligen Messbereich skalieren und auf 999 begrenzen
  //lbg_draw_val_limited = int(1000 * P1mW / (0.0008 * pow(10, MB1 - 1)));
  lbg_draw_val_limited = int(1000 * P1mW / (MBwahl[MB[0]].range));
  if (lbg_draw_val_limited > 999) lbg_draw_val_limited = 999;
  lbgPWR.drawValue(lbg_draw_val_limited, 1000);
  outstr[0] = '\0';
  EMPTY(outstr);
  strcpy(outstr, "[");
  strcat(outstr, MBwahl[MB[0]].Text);
  strcat(outstr, "]");

  LCDout(outstr, 13, 3, 7);

  EMPTY(outstr);
  strcat(outstr, QRGtext);
  LCDout(outstr, 8, 0, 6);


  //edit = true;
  resetCursor();

}

void screen1(byte kanal, float Ulin, float U, float rawU, float scalFactor, char *QRGtext) {
  int lbg_draw_val_limited;
  float PmW_out = 0;
  float PmW = 0;
  char outstr[30];
  int i;
  byte mb = 4;
  float Um = 0;
  PmW = pow(10, Ulin / 10.0);
  mb = MBselect(PmW);
  resetCursor();
  outstr[0] = '\0';
  EMPTY(outstr);
  itoa(kanal, outstr, 10);
  strcat(outstr, ">");
  LCDout(outstr, 0, 0, 2);

  PmW_out = PmW / MBwahl[mb].Divisor;

  EMPTY(outstr);
  // wir zeigen U an, --> Ulin + Eingangsdaempfung
  dtostrf(U, 4, 1, outstr);
  strcat(outstr, "dBm ");
  LCDout(outstr, 2, 0, 8);
  EMPTY(outstr);
  dtostrf(PmW_out, 5, MBwahl[mb].Nachkomma, outstr);
  strcat(outstr, MBwahl[mb].Unit);
  LCDout(outstr, 11, 0, 8);


  Um = rawU /  scalFactor;
  EMPTY(outstr);
  LCDout("Dm:", 0, 1, 10);
  dtostrf(Um, 5, 1, outstr);
  //strcat(outstr,"Dr:");
  LCDout(outstr, 4, 1, 10);

  EMPTY(outstr);
  LCDout("Dk:", 11, 1, 10);
  dtostrf(rawU, 5, 1, outstr);
  //strcat(outstr,"Dk:");
  LCDout(outstr, 15, 1, 10);

  EMPTY(outstr);
  strcpy(outstr, "Scal:");
  LCDout(outstr, 0, 2, 5);
  EMPTY(outstr);
  dtostrf(scalFactor, 5, 3, outstr);
  LCDout(outstr, 6, 2, 5);
  //Den zu zeichnenden Bargraph mit dem jeweiligen Messbereich skalieren und auf 999 begrenzen

  EMPTY(outstr);
  strcat(outstr, QRGtext);
  LCDout(outstr, 14, 2, 6);

  lbg_draw_val_limited = int(1000 * PmW / (MBwahl[mb].range));
  if (lbg_draw_val_limited > 999) lbg_draw_val_limited = 999;
  lbgPWR.drawValue(lbg_draw_val_limited, 1000);

  EMPTY(outstr);
  strcpy(outstr, "[");
  strcat(outstr, MBwahl[mb].Text);
  strcat(outstr, "]");

  LCDout(outstr, 13, 3, 7);
}

void screen2( float U1, float U2, float Att1, float Att2, char *QRGtext1, char *QRGtext2) {
  float PmW1_out = 0;
  float PmW1 = 0;
  float PmW2_out = 0;
  float PmW2 = 0;
  char outstr[30];
  int i;
  byte mb1 = 4;
  byte mb2 = 4;

  PmW1 = pow(10, U1 / 10.0);
  PmW2 = pow(10, U2 / 10.0);
  mb1 = MBselect(PmW1);
  mb1 = MBselect(PmW1);
  resetCursor();
  EMPTY(outstr);

  PmW1_out = PmW1 / MBwahl[mb1].Divisor;
  PmW2_out = PmW2 / MBwahl[mb2].Divisor;

  EMPTY(outstr);
  // wir zeigen U an, --> Ulin + Eingangsdaempfung
  dtostrf(U1, 4, 1, outstr);
  strcat(outstr, "dBm ");
  LCDout(outstr, 0, 0, 8);
  EMPTY(outstr);
  dtostrf(U2, 4, 1, outstr);
  strcat(outstr, "dBm ");
  LCDout(outstr, 10, 0, 8);
  EMPTY(outstr);
  dtostrf(PmW1_out, 5, MBwahl[mb1].Nachkomma, outstr);
  strcat(outstr, MBwahl[mb1].Unit);
  LCDout(outstr, 0, 1, 8);
  EMPTY(outstr);
  dtostrf(PmW2_out, 5, MBwahl[mb2].Nachkomma, outstr);
  strcat(outstr, MBwahl[mb2].Unit);
  LCDout(outstr, 10, 1, 8);
  LCDout("QRG:", 0, 2, 4);
  EMPTY(outstr);
  strcat(outstr, QRGtext1);
  LCDout(outstr, 4, 2, 6);
  LCDout("QRG:", 10, 2, 4);
  EMPTY(outstr);
  strcat(outstr, QRGtext2);
  LCDout(outstr, 14, 2, 6);
  EMPTY(outstr);
  LCDout("Att:", 0, 3, 10);
  dtostrf(Att1, 4, 1, outstr);
  LCDout(outstr, 4, 3, 4);
  EMPTY(outstr);
  LCDout("Att:", 10, 3, 10);
  dtostrf(Att2, 4, 1, outstr);
  LCDout(outstr, 14, 3, 4);

}

void showMenu() {

  //EMPTY(outstr);
  //itoa(kanal, outstr, 10);
  //strcat(outstr, ">");
  // stopTasks(); // wir uebernehmen die Kontrolle selbst
  LCDout("Frequenzbereich    ", 0, 0, 19);
  LCDout("Konfiguration E1   ", 0, 1, 19);
  LCDout("Konfiguration E2   ", 0, 2, 19);
  LCDout("Koppler            ", 0, 3, 19);

  LCDout("<", 18, menuPos, 3);


}

void configureDUAL() {
  // Konfiguration der Dual Anzeige
  switch (dualPos) {
    case 0:
      LCDout("<", 9, 2, 1);
      LCDout(" ", 19, 2, 1);
      LCDout(" ", 9, 3, 1);
      LCDout(" ", 19, 3, 1);
      break;
    case 1:
      LCDout(" ", 9, 2, 1);
      LCDout(" ", 19, 2, 1);
      LCDout("<", 9, 3, 1);
      LCDout(" ", 19, 3, 1);
      break;
    case 2:
      LCDout(" ", 9, 2, 1);
      LCDout("<", 19, 2, 1);
      LCDout(" ", 9, 3, 1);
      LCDout(" ", 19, 3, 1);
      break;
    case 3:
      LCDout(" ", 9, 2, 1);
      LCDout(" ", 19, 2, 1);
      LCDout(" ", 9, 3, 1);
      LCDout("<", 19, 3, 1);
      break;
  }
}

void write_lcd() //auffrischen des LCD  - wird alle 100ms angestossen
{
  float VSWR = 1.0;
  float P1mW_out = 0;
  float P2mW_out = 0;
  float U1lin = 0;
  float U2lin = 0;

  //String outstr;
  //static float old_U1 = 0.0;
  //String P_unit1, P_unit2;
  //int lbg_draw_val_limited;

  resetCursor();

  // Lienarisierung aufrufen.
  // Länge der LinM Struct kann nicht in der Funktion bestimmt werden (3.Argument)
  U1lin = linearize(smoothU1, linM0, sizeof(linM0) / sizeof(struct LinStruct));
  U2lin = linearize(smoothU2, linM0, sizeof(linM0) / sizeof(struct LinStruct));

  U1 = U1lin + Koppler1 + AttKanal1; //Auskoppeldaempfung und evtl. Daempfungsglied vor dem 8317
  U2 = U2lin + Koppler2 + AttKanal2;

  P1mW = pow(10, U1 / 10.0);
  P2mW = pow(10, U2 / 10.0);

  //MB[0] enthaelt den gewählten Bereich fuer Eingang1, MB[1] für Eingang2
  // die Wwerte ermitteln wir aus dem Array der Struct. In MBwahl[] findet man dann alles notwendig
  MB_Wahl(0, P1mW);
  MB_Wahl(1, P2mW);


  P1mW_out = P1mW / MBwahl[MB[0]].Divisor;
  P2mW_out = P2mW / MBwahl[MB[1]].Divisor;
  VSWR = abs((pow(10, ((U2 - U1) / 10.0)) + 1) / (pow(10, ((U2 - U1) / 10.0)) - 0.999999));
  // Ausgabe auf der verschiedenen Screens
  if (refresh || (menuState != oldState)) {
    lcd.clear();
    oldState = menuState;
    LcdBarGraph lbgPWR(&lcd, 13, 0, 3);
    refresh = false;
  }
  else { // es wurde nicht gedreht
  }
  switch (menuState) {
    case SWR:
      screen0(U1, U2, P1mW, P2mW, VSWR, QRGarray[frequenzIdx].Text);
      break;
    case IN1:
      screen1(1, U1lin, U1lin + AttKanal1, smoothU1, scalFactor1, QRGarray[QRGidx1].Text);
      break;
    case IN2:
      screen1(2, U2lin, U2lin + AttKanal2, smoothU2, scalFactor2, QRGarray[QRGidx2].Text);
      break;
    case DUAL:
      screen2(U1lin + AttKanal1, U2lin + AttKanal2, AttKanal1, AttKanal2, QRGarray[QRGidx1].Text, QRGarray[QRGidx2].Text);
      break;
    case MENU:
      showMenu();
      break;
    case DUALCFG:
      configureDUAL();
      screen2(U1lin + AttKanal1, U2lin + AttKanal2, AttKanal1, AttKanal2, QRGarray[QRGidx1].Text, QRGarray[QRGidx2].Text);
      break;
  }
  refresh = false;
  resetCursor();


}

byte MBselect(float PmW) {
  // Messbereichsumschaltung anhand MBStruct
  // ohne gloable Variablen
  byte i;
  float  minP;
  float maxP;
  // Bereich passt nicht und Wartezeit vorbei, wir suchen den passenden Bereich
  for (i = 0; i < (sizeof(MBwahl) / sizeof(MBStruct)); i++) {
    minP = MBwahl[i].range * 0.05;
    maxP = MBwahl[i].range * 0.8;
    if ((PmW < maxP) && (PmW > minP)) {
      // Wir haben den richtigen MB gefunden
      return i;
    }
  }
  return 0;
}


byte MB_Wahl(byte kanal, float PmW) {
  // Messbereichsumschaltung anhand MBStruct
  // Arbeitet mit globalenb Variablen MB[] und MBtime[]
  unsigned long switch_MB = millis();
  byte i;
  byte MBakt;
  float  minP;
  float maxP;
  MBakt = MB[kanal];
  // Grenzen des Bereichs min 0.05 * range, max 08*range
  minP = MBwahl[MBakt].range * 0.05;
  maxP = MBwahl[MBakt].range * 0.8;
  // wir schauen uns den gewählen Bereich mal an, ob er passt
  //if ((PmW<MBwahl[MBakt].maxP) && (PmW>MBwahl[MBakt].minP)) {
  if ((PmW < maxP) && (PmW > minP)) {
    // dann müssen wir nichts tun
    return MB[kanal];
  }
  // Bereich passt nicht, wie lange sind wir denn schon hier?
  if ((switch_MB - MBtime[kanal]) < (MBwahl[MBakt].tLen * 1000UL)) { // wir sind noch nicht lange genug in den Bereich
    //Serial.println("Warte...");
    return MB[kanal];
  }
  // Bereich passt nicht und Wartezeit vorbei, wir suchen den passenden Bereich
  for (i = 0; i < (sizeof(MBwahl) / sizeof(MBStruct)); i++) {
    minP = MBwahl[i].range * 0.05;
    maxP = MBwahl[i].range * 0.8;
    if ((PmW < maxP) && (PmW > minP)) {
      //  if ((PmW < MBwahl[i].maxP) && (PmW > MBwahl[i].minP)) {
      // Wir haben den richtigen MB gefunden
      MBtime[kanal] = switch_MB; // Zeit merken
      MB[kanal] = i; // Bereich merken
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
  Serial.print((int)smoothU1);
  Serial.print("(");
  Serial.print("Dig)  U2:");
  Serial.print((int)smoothU2);
  Serial.print("(");

  Serial.print("Dig)  QRG:");
  Serial.println(frequenz);
}
