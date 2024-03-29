#include <Event.h>
#include <Timer.h>


#include <LiquidCrystal.h>
#include <LcdBarGraph.h>
#include <stdio.h>
#include "CRC8.h"
#include <EEPROM.h>

/*
   Powermeter DARC K08 Project 2020/2022
   basierend auf AD8317 Wandler und einem ATMEGA2560 mit 4x20 LCD Display
   Die Messwerte werden nach Stützstellen interpoliert und frequenzkorrigiert

   Bildchirme lassen sich mit dem Enoder auswählen:
   SWR: SWR Messung mit beiden Kanälen.
   DUAL: Zwei Kanal Messung ohne Koppler unter Bewrücksichtigung eines optionalen Daempfungsgliedes
   DUALKOPPLER: Zwei Kanal Messung unter Berücksichtigung eines Richtkopplers.
   IN1/IN2: Deatils zu den Kanälen, dient der Kalibrierung

   Alle Konfigurationswerte und die Koppler inkl. Stützstellen werden im EEPROM gespeichert.
   Die Anzahl der max. Koppler ist in #define konfigurierbar

   Kalibrierung:
   1. IN1 / IN2 auswählen
   2. Taste drücken
   3. Signal mit -10dB im HF Bereich (KW) anlegen
   4. Kalibrierung ermittelt den Saklierungsfaktor pro Kanal und speichert ihn im EEPROM

   Menüs: Die Menüs erreicht man durch Drücken des Encoders in der entsprechenden Ansicht:
   SWR: Hauptmenü:
          Frequenz: Frequenzmessung für SWR und die anderen Modi, wird im EEPROM gespeichert
          Eingang1/2: Konfiguration des Eingangs: Zusätzliches Dämpfungsglied und den angeschlossenen Koppler
          Koppler: Definition von Kopplerdaten
              EDIT: Hier können die Stützstellen für Min QRG - Max QRG eingegeben werden
   DUAL: Online Änderung des Bandes und der Eingangsdämpfung. Dient zur Messung ohne Koppler. QRG und Daempfung werden nicht im EEPROM gespeichert
   DUALKOPPLER: Online Änderung der Bänder pro Kanal. Der Koppler muss über das SWR-Hauptmenü geändert werden
   IN1/IN2: Kalibrierungsmodus
*/

// Debug Modus, zusätzliche Ausgaben in der seriellen Konsole
// #define DEBUG

/*
   Testmodus.
   Beim Nachbau nach DL2FW, wie im Github beschrieben, muss DL2FW genutzt werden.

*/
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
#define PUSHB_INT CHANGE
//#define PUSHB_INT FALLING

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
#define PUSHB_INT CHANGE // auf Change geaendert wegen fired/realsed Erkennung

#define ATT_KOPPLER1 0
#define ATT_KOPPLER2 0


#define REFERENCE INTERNAL1V1

float scalFactor1 = 1.0;

float scalFactor2 = 1.0 ;

#endif
////////////////////////////////////////////////////////////////////////


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
#define CALIBRATE 4 // 4. Eintrag in limM0 --> -10dB

#define QRG 5 // Test Frequenz Index des QRGarray

// Anzeigeformate
// 2 --> 3 Darsatellungen 0,1,2
#define MAXSCREEN 2

// Anzahl der Elemente im QRGarray[]
#define QRGSIZE 17

// Anzahl der zu verwaltenden Kopppler, durch EEPROM Größe beschränkt (Index 0-9 --> 10 )
#define KOPPLERSIZE 10

// Anzahl der definierten PAS
#define PASIZE 2


// Anzahl der Menueeintrage
#define MENUSIZE 4

// Anzahl des CFG Werte im DUAL Modus
#define DUALSIZE 4

// Anzahl des Anzeigemodi Werte im IN1/IN2 Modus
#define INSIZE 3

// Anzahl Positionen im INCFG Konf.
#define INCFGSIZE 6

// Anzahl des CFG Werte im DUALKOPPLER Modus
#define DUALKOPPLERSIZE 2

//EEPROM Adresse fuer Koppler
// muss evtl. angepasst werden, wenn das andere Struct zu groß wird
#define EEPROM_K_ADDR 100
// dito fuer PA
#define EEPROM_PA_ADDR 1000


// Wie viele Sekunden werden als LAAANG gedrückt erkannt
#define LONGFIRED 2

/*
  // Werte fuer PA Anzeige, wandert spaeter in die Konfiguration
  #define PILIN 12.0 // bis zu welchem Pegel in dBm ist die PA linear
  #define PIMAX 17.0 // Vollaustseuerung der PA bei Input in dBm
  #define GLIN  30.0 // Verstaerkung linear
  #define POMAX 43.0 // max. Ausgangsleistung der PA in dBm
  #define PIMXX 20.0 // absolut max Eingangsleistung (schon in der Sättigung)
  #define ATTKABEL 4.0 // Daempung Kabel von Koppler zur PA
*/

// Werte fuer PA Anzeige, wandert spaeter in die Konfiguration
#define PILIN 2.0 // bis zu welchem Pegel in dBm ist die PA linear
#define PIMAX 7.0 // Vollaustseuerung der PA bei Input in dBm
#define GLIN  30.0 // Verstaerkung linear
#define POMAX 35.0 // max. Ausgangsleistung der PA in dBm
#define PIMXX 10.0 // absolut max Eingangsleistung (schon in der Sättigung)
#define ATTKABEL 4.0 // Daempung Kabel von Koppler zur PA


// Status für check_encoder, wo wir gerade stehen
enum mStates {
  SWR,
  IN1,
  IN2,
  INPA1,
  INPA2,
  MENU,
  DUAL,
  DUALKOPPLER,
  DUALCFG,
  DUALKOPPLERCFG,
  STATUS,
  INCFG1,
  INCFG2,
  INPACFG1,
  INPACFG2,
};

enum kStates {
  SELECT,
  CONF,
  EDIT,
  EXIT,
  DEL,
};

enum calStates {
  ATT,
  LEVEL,
  START,
  BREAK,
};

// Startmeldungen
#define START1 "   PowerMeter"
#define START2 "   1MHz ... 10GHz"
#define START3 "  K08 DL2FW/DM6TT"
#define START4 "  ---------------"



// Makros, fuer Strings zu leeren
#define EMPTY(a) for(i=0;i<sizeof(a);i++) a[i]='\0'

float frequenz = 0; // Frequenz in MHz, wird aus QRGarray bestimmt
byte frequenzIdx = QRG; // Index in QRGArry


const float smooth = 0.004;
//> timeconstant=200ms (app. 1/smooth * TA (10ms)) @smooth=0.01-> timeconstant=1000ms ... - correct for smooth << 1


//Struktur für die Stützstellen / Linearisierung
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

// EEPROm Struktur für die Speicherung der Werte im EEPROM (außer Koppler)
struct EEStruct {
  float scalFactor[2];  // Skalierung der beiden Kanaäle
  float attKanal[2];    //Daempfungskonstatnte pro Kanal
  float intAttKanal[2];    //Daempfungskonstatnte pro Kanal
  byte idxKoppler[2];  // Richtkoppler Auskopplung
  byte qrg[2];          // Messfrewuenzen pro Kanal
  byte SWRqrg;          // Messfrequenz für die SWR Messung
  boolean calibrated[2]; // sind die Kanäle kalibriert?
  byte CRC;            // Pruefsumme
};


// Struktur für die Frequenzwahl
struct QRGstruct {
  float frequency;      // Frequenz in KHz
  char Text[10];      // Anzeigetext
};


// Kopplerstruktur
struct KOPPLERstruct {
  boolean configured;              // wird
  float sigDB[QRGSIZE];       // Auskoppeldaempfung
  byte minQRGidx;             // tiefste nutzbare Frequenz (Index, siehe oben)
  byte maxQRGidx;             // größte Nutzbare Frequenz
  float attenuation;          // Daempfung
  //char name[20];              // Namen des Kopplers
};

// EEPROM Speicherung der Koppler (Kopplerstruktur + CRC)
struct EEKopplerStruct {
  struct KOPPLERstruct Karray[KOPPLERSIZE];    // Gesamtes Kopplerdefinition
  byte CRC;                             // Pruefsumme
};


struct PAstruct {
  float piLin;    // bis zu welchem Pegel in dBm ist die PA linear
  float piMax;   // Vollaustseuerung der PA bei Input in dBm
  float gLin;    // Verstaerkung linear
  float poMax;   // max. Ausgangsleistung der PA in dBm
  float piMxx;   //absolut max Eingangsleistung (schon in der Sättigung)
  float attKabel;// Daempung Kabel von Koppler zur PA
};



struct EEPAstruct {
  struct PAstruct Parray[PASIZE];    // PA Defintion fuer Kanal 1 2
  byte CRC;                      // Pruefsumme
};

struct MBStruct MBwahl[10];
struct EEStruct EEprom;
struct EEKopplerStruct EEpromKoppler;
struct QRGstruct QRGarray[QRGSIZE];
struct KOPPLERstruct KOPPLERarray[KOPPLERSIZE];
struct PAstruct PAarray[PASIZE];
struct EEPAstruct EEpromPA;

mStates menuState = SWR;
mStates oldState = IN1;


int posi = 0;

int longFired = 0; //ms, die die Taste schon gedrueckt ist
unsigned long oldLongFired = 0;

byte MB[2]; //Messbereich
unsigned long MBtime[2]; // Dauer im Messbereich fuer Auswahl

int MB1 = 5;  //Reserve nach unten hin
int MB2 = 5;  //Reserve nach unten hin
float P1mW = 0.0;
float P2mW = 0.0;
Timer t;

// eingelesene Werte
float smoothU1 = 0.0;
float smoothU2 = 0.0;

// Werte in dB , inklusive der Eingangsdämpfung und der Kopplerdämpfung
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
volatile boolean released;    // knob was released
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
byte dualPos = 0;
byte dualKopplerPos = 0;
byte inPos = 0;

boolean refresh = true; // soll Bildschirm neu aufgebaut werden?



CRC8 crc;


// Anzeige
byte screenNo = 0; // Auswahl der Darstellung, max. Wert MAXSCREEN
byte oldScreenNo = 0;


void(* resetFunc) (void) = 0;

// Initialisiere LCD Display
LiquidCrystal lcd(LCD_RS, LCD_EN, LCD_D4, LCD_D5, LCD_D6, LCD_D7); // -- creating LCD instance

LcdBarGraph lbgPWR(&lcd, 13, 0, 3); // 13 Zeichen, Start (0,3)

/*
   Interruptroutine für Encoder und  Taste, Auswertung erfolgt in den einzelnen Funktionen
*/

// Interrupt Service Routine - für den Button des Encoder
void isrB ()
{
  if (!digitalRead (PUSHB)) {
    fired = true;
    released = false;
  }
  else {
    longFired = 0;
    oldLongFired = 0;
    released = true;
  }  // end of isr
}

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

/*
   Long Press Auswertung.
   longFired enthaelt die Zeit in s. die die Taste gedrueckt wurde
   (geht nicht in IRQ wegen millis() )
*/
void checkButton() {
  if (fired) {
    if (!oldLongFired)
      oldLongFired = millis();
    else
      longFired = (millis() - oldLongFired) / 1000;
  }
  else { // da hat schon jemand den fired abgegriffen, also Zeiten loeschen
    longFired = 0;
    oldLongFired = 0;
  }
}

/*
   Ruecksetzen beider EEprom Bereiche
*/


void resetToDefault() {
  byte i;
  EEprom.calibrated[0] = false;
  EEprom.calibrated[1] = false;
  EEprom.intAttKanal[0] = 0.0;
  EEprom.intAttKanal[1] = 0.0;
  EEprom.attKanal[0] = 0.0;
  EEprom.attKanal[1] = 0.0;
  EEprom.idxKoppler[0] = KOPPLERSIZE + 1; // daran erkennen wir, dass keiner zugewiesen wurde
  EEprom.idxKoppler[1] = KOPPLERSIZE + 1; // daran erkennen wir, dass keiner zugewiesen wurde
  writeEEPROM();


  for (i = 0; i < KOPPLERSIZE; i++) {
    KOPPLERarray[i].configured = false;
  }
  writeEEPROMKoppler(EEPROM_K_ADDR);

}

/*
   Hauptasuwertung, ob der Encoder gedreht wurde oder die Taste gedrückt wurde
   Hier werden die menuStates gesetzt, die in der Ausgaberutine verwendet werden
   Erweiterungen von neuen Ausgabe-Screens müssen hier und bei write_lcd() eingebaut werden

*/

void check_encoder() {

  // wenn keine Taste gedrueckt wurde und nicht gedreht wurde, haben wir nichts zu tun
  if (!fired && !turned)
    return;
  switch (menuState) { // wir schauen, in welchem Status wir uns befiunden. Hier nutzen wir ein enum der besseren Lesbarkeit.
    case SWR:  // Hauptschirm
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
        menuState = INPA2;
        screenNo = 2;
      }
      break;
    case DUAL:
      if (turned && up) {
        turned = false;
        menuState = DUALKOPPLER;
        screenNo = 3;
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
    case DUALKOPPLER:
      if (turned && up) {
        turned = false;
        menuState = IN1;
        screenNo = 2;
      }
      if (turned && !up) {
        turned = false;
        menuState = DUAL;
        screenNo = 1;
      }
      if (fired) { // Taste wurde gedrueckt
        fired = false;
        menuState = DUALKOPPLERCFG;
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
      if (fired &&  !released && (longFired >= LONGFIRED)) { //3 sec druecken siehe #define

        fired = false;
        stopTasks();
        calibration(1, &scalFactor1, PIN_A1, AttKanal1);
        startTasks();
        writeEEPROM();
        refresh = true;
      }
      else if (fired && released) { // nir mal kruz gedrueckt, Auswahl des Ausgabewertes
        fired = false;
        menuState = INCFG1;
      }
      break;
    case   IN2:   // Eingang2 Anzeige
      if (turned && up) {
        turned = false;
        menuState = INPA1;
        screenNo = 0;
      }
      if (turned && !up) {
        turned = false;
        menuState = IN1;
        screenNo = 1;
      }
      if (fired &&  !released && (longFired > LONGFIRED)) { //3s druecken
        fired = false;
        stopTasks();
        calibration(2, &scalFactor2, PIN_A2, AttKanal2);
        startTasks();
        writeEEPROM();
        refresh = true;
      }
      else if (fired && released) { // nir mal kruz gedrueckt, Auswahl des Ausgabewertes
        fired = false;
        menuState = INCFG2;
      }
      break;
    case INPA1:  // Eingang 1 mit PA Anzeige
      if (turned && up) {
        turned = false;
        menuState = INPA2;
        screenNo = 0;
      }
      if (turned && !up) {
        turned = false;
        menuState = IN2;
        screenNo = 1;
      }
      if (fired && released) { // nir mal kruz gedrueckt, Auswahl des Ausgabewertes
        fired = false;
        stopTasks();
        configurePA(1);
        writeEEPROMPA(EEPROM_PA_ADDR);
        startTasks();
        //menuState = INPACFG1;
      }
      break;
    case INPA2:// Eingang 1 mit PA Anzeige
      if (turned && up) {
        turned = false;
        menuState = SWR;
        screenNo = 0;
      }
      if (turned && !up) {
        turned = false;
        menuState = INPA1;
        screenNo = 1;
      };
      if (turned && !up) {
        turned = false;
        menuState = IN2;
        screenNo = 1;
      }
      if (fired && released) { // nir mal kruz gedrueckt, Auswahl des Ausgabewertes
        fired = false;
        stopTasks();
        configurePA(2);
        writeEEPROMPA(EEPROM_PA_ADDR);
        startTasks();
        //menuState = INPACFG2;
      }
      break;
    case INCFG1:
      if (turned && up)
        inPos < (INSIZE - 1) ? inPos++ : (inPos = 0);
      else if (turned && !up)
        inPos > 0 ? inPos-- : (inPos = INSIZE - 1);
      if (fired && released) {
        menuState = IN1;
        fired = false;
      }
      break;
    case INCFG2:
      if (turned && up)
        inPos < (INSIZE - 1) ? inPos++ : (inPos = 0);
      else if (turned && !up)
        inPos > 0 ? inPos-- : (inPos = INSIZE - 1);
      if (fired && released) {
        menuState = IN2;
        fired = false;
      }
      break;
    case DUALCFG:
      if (turned && up)
        dualPos < (DUALSIZE - 1) ? dualPos++ : (dualPos = 0);
      else if (turned && !up)
        dualPos > 0 ? dualPos-- : (dualPos = (DUALSIZE - 1));
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
    case DUALKOPPLERCFG:
      if (turned && up)
        dualKopplerPos < (DUALKOPPLERSIZE - 1) ? dualKopplerPos++ : (dualKopplerPos = 0);
      else if (turned && !up)
        dualKopplerPos > 0 ? dualKopplerPos-- : (dualKopplerPos = DUALKOPPLERSIZE - 1);
      turned = false;
      if (fired) {
        stopTasks();
        fired = false;
        switch (dualKopplerPos) {
          case 0: // Frequenz Kanal1
            QRGidx1 = editQRG(QRGidx1, 4, 2);
            refresh = true;
            menuState = DUALKOPPLER;
            break;

          /*           case 1: // Attenuation Kanal1
                      AttKanal1 = editFloat(AttKanal1, 4, 3, 4, 1, 0.1);
                      refresh = true;
                      menuState = DUAL;
                      break;
          */
          case 1: // Frequenz Kanal2
            QRGidx2 = editQRG(QRGidx2, 14, 2);
            refresh = true;
            menuState = DUALKOPPLER;
            break;
            /*
                      case 3: // Attenuation Kanal2
                        AttKanal2 = editFloat(AttKanal2, 14, 3, 4, 1, 0.1);
                        refresh = true;
                        menuState = DUAL;
                        break;
            */
        }
        startTasks();
      }
    case STATUS: // Anzeige des Status durch langes druecken
      // drehen , arus zum Menu
      if (turned)
        menuState = MENU;
      turned = false;
      if (fired) {
        lcd.clear();
        LCDout("Alle Werte werden", 0, 0, 17);
        LCDout("zurueckgesetzt!", 0, 1, 15);
        LCDout("Danach Neustart", 0, 2, 15);
        delay(2000);
        resetToDefault();
        resetFunc();
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
      // Taste muss losgelassen werden. Dient der Auswertung von longFired
      if (fired && released) {
        stopTasks();
        fired = false;
        switch (menuPos) {
          case 0: // Frequenz
            frequenzIdx = chooseQRG(frequenzIdx);
            frequenz = QRGarray[frequenzIdx].frequency;
            Koppler1 = KOPPLERarray[EEprom.idxKoppler[0]].sigDB[frequenzIdx];
            Serial.print("Koppler1:");
            Serial.print(Koppler1);
            Koppler2 = KOPPLERarray[EEprom.idxKoppler[1]].sigDB[frequenzIdx];
            Serial.print(" Koppler2:");
            Serial.println(Koppler2);
            writeEEPROM();
            refresh = true;
            menuState = SWR;
            // wir setzen die temporaren Werte auch auf die gewaehlte Frequenz
            QRGidx1 = frequenzIdx;
            QRGidx2 = frequenzIdx;
            break;
          case 1: // Konfiguration Kanal 1
            configureInput(1);
            AttKanal1 = EEprom.attKanal[0];
            Koppler1 = KOPPLERarray[EEprom.idxKoppler[0]].sigDB[frequenzIdx];
            writeEEPROM();
            menuState = SWR;
            refresh = true;
            break;
          case 2: // Konfiguration Kanal 2
            configureInput(2);
            AttKanal2 = EEprom.attKanal[1];
            Koppler2 = KOPPLERarray[EEprom.idxKoppler[1]].sigDB[frequenzIdx];
            writeEEPROM();
            menuState = SWR;
            refresh = true;
            break;
          case 3: // Konfiguration Koppler
            Serial.println("Konfiguriere Koppler");
            configureKoppler();
            writeEEPROMKoppler(EEPROM_K_ADDR);
            menuState = SWR;
            break;
        }
        startTasks();
        //break;

      }
      else if (fired && !released && (longFired > LONGFIRED)) { // Taste wurde lange gedrueckt
        menuState = STATUS;
        fired = false;
      }
      break;
  }
  turned = false;
  //fired =  false;
  return;
}




/*
   Setup Routine zur Initialisierung der IOs, ISR etc.
*/

void setup() {
  boolean waitFired = false; // warte auf Tastendruck
  byte i;
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


  /*
     Stützstellen für die Linearisierung
     Es werden zurzeit nur eine Version für beide 8317 Kanäle verwendet, da die Daten recht gleich sind
     ADnorm: Der eingelesenen Wert vom AD-Wandler (0...1023). Da eine Saklierung bei Inbetriebnahme vorgenommen wird, muss nichts angepasst werden
     dB: Der referenzierte Pegel
  */

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
  /*
     range: Messbereich, muss mit Text korrospondieren, bezogen auf 1mW
     tLen: Zeitkonstante, die dieser Bereich mindestens 'gehalten' wird.
     Nachkomma: Die Anzahl der Nachkommastellen
     Dvisor: Teilungsfaktor, bezihet sich auf die Unit
     Unit: Einheit der Ausgabe (aus Divisor)
  */

  MBwahl[0].range = 1E-3;   MBwahl[0].tLen = 1; MBwahl[0].Nachkomma = 1;  MBwahl[0].Divisor = 1E-6;  strcpy(MBwahl[0].Unit, "nW\0"); strcpy(MBwahl[0].Text, "1uW\0");
  MBwahl[1].range = 1E-2;   MBwahl[1].tLen = 1; MBwahl[1].Nachkomma = 3;  MBwahl[1].Divisor = 1E-3;  strcpy(MBwahl[1].Unit, "uW\0"); strcpy(MBwahl[1].Text, "10uW\0");
  MBwahl[2].range = 1E-1;   MBwahl[2].tLen = 1; MBwahl[2].Nachkomma = 2;  MBwahl[2].Divisor = 1E-3;  strcpy(MBwahl[2].Unit, "uW\0"); strcpy(MBwahl[2].Text, "100uW\0");
  MBwahl[3].range = 1E0;    MBwahl[3].tLen = 1; MBwahl[3].Nachkomma = 1;  MBwahl[3].Divisor = 1E-3;  strcpy(MBwahl[3].Unit, "uW\0"); strcpy(MBwahl[3].Text, "1mW\0");
  MBwahl[4].range = 1E1;    MBwahl[4].tLen = 1; MBwahl[4].Nachkomma = 3;  MBwahl[4].Divisor = 1E0;   strcpy(MBwahl[4].Unit, "mW\0"); strcpy(MBwahl[4].Text, "10mW\0");
  MBwahl[5].range = 1E2;    MBwahl[5].tLen = 1; MBwahl[5].Nachkomma = 2;  MBwahl[5].Divisor = 1E0;   strcpy(MBwahl[5].Unit, "mW\0"); strcpy(MBwahl[5].Text, "100mW\0");
  MBwahl[6].range = 1E3;    MBwahl[6].tLen = 1; MBwahl[6].Nachkomma = 1;  MBwahl[6].Divisor = 1E0;   strcpy(MBwahl[6].Unit, "mW\0"); strcpy(MBwahl[6].Text, "1W\0");
  MBwahl[7].range = 1E4;    MBwahl[7].tLen = 1; MBwahl[7].Nachkomma = 3;  MBwahl[7].Divisor = 1E3;   strcpy(MBwahl[7].Unit, "W\0");  strcpy(MBwahl[7].Text, "10W\0");
  MBwahl[8].range = 1E5;    MBwahl[8].tLen = 1; MBwahl[8].Nachkomma = 2;  MBwahl[8].Divisor = 1E3;   strcpy(MBwahl[8].Unit, "W\0");  strcpy(MBwahl[8].Text, "100W\0");

  // Frequenzen
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

  // Startwerte der Frequenz bei der Inbetriebnahme
  frequenz = QRGarray[QRG].frequency;
  frequenzIdx = QRG;


  // Initialiiserung des PA Arrays
  for (i = 0; i < PASIZE; i++) {
    PAarray[i].piLin = 0.0;
    PAarray[i].piMax = 0.0;
    PAarray[i].gLin = 0.0;
    PAarray[i].poMax = 0.0;
    PAarray[i].piMxx = 0.0;
    PAarray[i].attKabel = 0.0;
  }



  // Hier das eigentliche "Multitasking"

  // Ausgelagert in eine Subroutine, um die Tasks bei Menüs ein-und auszuschalten
  startTasks();

  // einladen der im EEProm gspeicherten Werte
  // anhand der Checksumme schauen wir, ob die Daten konsitent sind.


  if (!readEEPROM()) {
    lcd.setCursor(0, 3);
    lcd.print("EEPROM leer, init..");
    EEprom.calibrated[0] = false;
    EEprom.calibrated[1] = false;
    EEprom.intAttKanal[0] = 0.0;
    EEprom.intAttKanal[1] = 0.0;
    EEprom.attKanal[0] = 0.0;
    EEprom.attKanal[1] = 0.0;
    EEprom.idxKoppler[0] = KOPPLERSIZE + 1; // daran erkennen wir, dass keiner zugewiesen wurde
    EEprom.idxKoppler[1] = KOPPLERSIZE + 1; // daran erkennen wir, dass keiner zugewiesen wurde

    writeEEPROM();
    // Nach Schreiben und wieder einlesen des EEPROM sollte die Checksumme stimmen, wenn nicht, haben wir ein Problem
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

  // nun die Koppler einlesen,
  if (!readEEPROMKoppler(EEPROM_K_ADDR)) {
    lcd.setCursor(0, 3);
    lcd.print("EEPROM Kopplerinit..");
    writeEEPROMKoppler(EEPROM_K_ADDR);
    // Konsistenzprüfung nach initalem anlegen
    if (!readEEPROMKoppler(EEPROM_K_ADDR)) {
      lcd.setCursor(0, 3);
      lcd.print("EEPROM Koppler ERROR");
      while (1);
    }
  }
  else {
    lcd.setCursor(0, 3);
    lcd.print("Lese EEPROM Koppler");
  }
  // und jetzt noch die PAs
  if (!readEEPROMPA(EEPROM_PA_ADDR)) {
    lcd.setCursor(0, 3);
    lcd.print("EEPROM PAinit..");
    writeEEPROMPA(EEPROM_PA_ADDR);
    // Konsistenzprüfung nach initalem anlegen
    if (!readEEPROMPA(EEPROM_PA_ADDR)) {
      lcd.setCursor(0, 3);
      lcd.print("EEPROM PA ERROR");
      while (1);
    }
  }
  else {
    lcd.setCursor(0, 3);
    lcd.print("Lese EEPROM Koppler");
  }

  delay(500);
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
  if (!countKoppler()) {
    lcd.setCursor(0, 2);
    lcd.print("Keine Koppler def.");
    Serial.println("Keine Koppler def.");
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


/*
   Arduino Hauptschleife lässt allen die Taskverarbeitung machen

*/
void loop()
{

  t.update(); //timer handling!	mehr passiert in der Hauptschleife nicht, Multitasking läuft im Hintergrund

}

/*
   Lese EEPROM für Einstellungen (außer Koppler)
   Es wird ab Adresse 0 gelesen. Die Koppler fangen bei 100 (siehe #define) an, man kann hier die Struct auch nich erweitern
*/

boolean readEEPROM() {
  byte checkCRC = 0;
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
    Koppler1 = KOPPLERarray[EEprom.idxKoppler[0]].attenuation;
    Koppler2 = KOPPLERarray[EEprom.idxKoppler[1]].attenuation;
    frequenzIdx = EEprom.SWRqrg;
    frequenz = QRGarray[frequenzIdx].frequency;
    QRGidx1 = EEprom.qrg[0];
    QRGidx2 = EEprom.qrg[1];
    //Serial.print("QRG idx:");
    //Serial.println(frequenzIdx);

    return true;
  }
}

/*
    Lesen der Koppler
    Adresse wird mit übergeben und im #define definiert
    Koppler werden über memcpy in die Struct kopiert
*/

boolean readEEPROMKoppler(int eeAddress) {
  byte checkCRC = 0;

  Serial.print("Lese EEPROM  Koppler...");
  Serial.print(" Groesse:");
  Serial.println(sizeof(EEpromKoppler));
  EEPROM.get(eeAddress, EEpromKoppler);
  //EEPROM.get(eeAddress, KOPPLERarray);
  //return true;

  crc.reset();
  checkCRC = EEpromKoppler.CRC;
  //Serial.print("  CRC Koppler read:");
  //Serial.print(EEpromKoppler.CRC);
  EEpromKoppler.CRC = 0; // sonst stimmt die CRC Summe nicht, sie darf nicht  mit eingerechnet werden
  crc.add((uint8_t *)&EEpromKoppler, sizeof(EEpromKoppler));
  //Serial.print("  CRC calc:");
  //Serial.println(crc.getCRC());
  if ( crc.getCRC()  != checkCRC) {
    Serial.println("EEPROM Koppler Checksumme falsch");
    return false;
  }
  else {
    //Serial.println("EEPROM Koppler Checksumme OK");
    // setzen der globalen Variablen
    memcpy(KOPPLERarray, &EEpromKoppler.Karray, sizeof(KOPPLERstruct)*KOPPLERSIZE);
    printKopplerCFG();
    return true;
  }

}

/*
 * Lesen der PA definitionen
 */

boolean readEEPROMPA(int eeAddress) {
  byte checkCRC = 0;

  Serial.print("Lese EEPROM  PA...");
  Serial.print(" Groesse:");
  Serial.println(sizeof(EEpromPA));
  EEPROM.get(eeAddress, EEpromPA);
  

  crc.reset();
  checkCRC = EEpromPA.CRC;
  EEpromPA.CRC = 0; // sonst stimmt die CRC Summe nicht, sie darf nicht  mit eingerechnet werden
  crc.add((uint8_t *)&EEpromPA, sizeof(EEpromPA));
  //Serial.print("  CRC calc:");
  //Serial.println(crc.getCRC());
  if ( crc.getCRC()  != checkCRC) {
    Serial.println("EEPROM PA Checksumme falsch");
    return false;
  }
  else {
    //Serial.println("EEPROM PA Checksumme OK");
    // setzen der globalen Variablen
    memcpy(PAarray, &EEpromPA.Parray, sizeof(PAstruct)*PASIZE);
    //printKopplerCFG();
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
  //  EEprom.attKoppler[0] = Koppler1;
  //  EEprom.attKoppler[1] = Koppler2;
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

void writeEEPROMKoppler(int eeAddress ) {
  byte i, j;

  Serial.print("Schreibe EEPROM Koppler...");
  Serial.print(" Groesse:");
  Serial.print(sizeof(KOPPLERstruct)*KOPPLERSIZE);

  //wir kopieren das KopplerArray in EEpromKoppler, weil wir die Checksumme in der Struct brauchen
  memcpy(&EEpromKoppler.Karray, &KOPPLERarray, sizeof(KOPPLERstruct)*KOPPLERSIZE);
  EEpromKoppler.CRC = 0;

  crc.reset();
  crc.add((uint8_t *)&EEpromKoppler, sizeof(EEpromKoppler));
  EEpromKoppler.CRC = crc.getCRC();

  Serial.print(" CRC Koppler:");
  Serial.println(EEpromKoppler.CRC);
  Serial.print(" Groesse pro Koppler:");
  Serial.println(sizeof(KOPPLERstruct));


  EEPROM.put(eeAddress, EEpromKoppler);
  //EEPROM.put(eeAddress, KOPPLERarray);

}


void writeEEPROMPA(int eeAddress ) {
  byte i, j;

  Serial.print("Schreibe EEPROM PA...");
  Serial.print(" Groesse:");
  Serial.print(sizeof(PAstruct)*PASIZE);

  //wir kopieren das PAArray in EEpromKoppler, weil wir die Checksumme in der Struct brauchen
  memcpy(&EEpromPA.Parray, &PAarray, sizeof(PAstruct)*PASIZE);
  EEpromPA.CRC = 0;

  crc.reset();
  crc.add((uint8_t *)&EEpromPA, sizeof(EEpromPA));
  EEpromPA.CRC = crc.getCRC();

  Serial.print(" CRC PA:");
  Serial.println(EEpromPA.CRC);
  Serial.print(" Groesse pro PA:");
  Serial.println(sizeof(PAstruct));


  EEPROM.put(eeAddress, EEpromPA);
  //EEPROM.put(eeAddress, KOPPLERarray);

}


/*
   Ausgabe der Kopplerkofniguration auf der seriellen Konsole
   Dient der Kontrolle, ob alle Stützstellen richtig definiert sind
*/

void printKopplerCFG() {
  // Ausgabe der Kopplerkonfiguration auf der seriellen Konsole
  byte i, j;
  struct KOPPLERstruct  koppler;
  for (i = 0; i < KOPPLERSIZE; i++) {
    Serial.print("Koppler:");
    Serial.print(i);
    koppler = KOPPLERarray[i];
    if (koppler.configured) {
      Serial.println(" konfiguriert");
      Serial.print("   QRG min:");
      Serial.print(QRGarray[koppler.minQRGidx].Text);
      Serial.print(" Max:");
      Serial.print(QRGarray[koppler.maxQRGidx].Text);
      Serial.print(" Daempfung:");
      Serial.println(koppler.attenuation);
      Serial.println("   Stützstellen:");
      for (j = koppler.minQRGidx; j <= koppler.maxQRGidx; j++) {
        Serial.print("     Band:");
        Serial.print(QRGarray[j].Text);
        Serial.print(" Att");
        Serial.print(koppler.sigDB[j]);
        Serial.println("dB");
      }
      Serial.println("--------------------------------------");
    }
    else {
      Serial.println(" NICHT konfiguriert");
    }
  }
}

/*
   Taskhandling Start und Stop
*/

void startTasks () {
  t.every(1000, write_raw_seriell, 0);
  t.every(10, take_ads, 0); //lese die analogen Eingänge alle 10ms und glätte diese
  t.every(100, write_lcd, 0); //alle 500ms auf LCD darstellen
  t.every(50, check_encoder, 0); // alle 20ms Änderungen des Encoders detektieren
  t.every(100, checkButton, 0);
}

void stopTasks() {
  t.stop(write_raw_seriell);
  t.stop(take_ads);
  t.stop(write_lcd);
  t.stop(check_encoder);
  t.stop(checkButton);
}

/*
   Routine zur Auswahl der Frequenz
   Ausgabewerte sind auf den screen angepasst und hard kodiert hier verwendet
*/

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

void refreshCalPosition(byte edtPos) {
  switch (edtPos) {
    case 0:
      LCDout("<", 17 , 1, 1);
      LCDout(" ", 17, 2, 1);
      LCDout(" ", 0, 3, 1);
      LCDout(" ", 8, 3, 1);
      LCDout(" ", 9, 3, 1);
      LCDout(" ", 16, 3, 1);
      break;
    case 1:
      LCDout(" ", 17, 1, 1);
      LCDout("<", 17, 2, 1);
      LCDout(" ", 0, 3, 1);
      LCDout(" ", 8, 3, 1);
      LCDout(" ", 9, 3, 1);
      LCDout(" ", 16, 3, 1);
      break;
    case 2:
      LCDout(" ", 17 , 1, 1);
      LCDout(" ", 17, 2, 1);
      LCDout("<", 0, 3, 1);
      LCDout(">", 8, 3, 1);
      LCDout(" ", 9, 3, 1);
      LCDout(" ", 16, 3, 1);
      break;
    case 3:
      LCDout(" ", 17 , 1, 1);
      LCDout(" ", 17, 2, 1);
      LCDout(" ", 0, 3, 1);
      LCDout(" ", 8, 3, 1);
      LCDout("<", 9, 3, 1);
      LCDout(">", 16, 3, 1);
      break;
  }
}

/* Kalbierungsroutine
    Es wird anhand eines Kalibireungssignal der Saklierungsfaktor berechnet
    Die Engangsdaempfung eines Kanals, die konfiguriert wurde, wird entsprechend mit berücksichtigt
    Es werden 5 Stufen druchlaufen und dabei der Glättungsfaktor immer kleiner gesachaltet
    Die Durchläufe wurden emprisch ermittelt
*/

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
  boolean exitIT = false;
  byte edtPos = 0;
  const byte edtMax = 3;
  float dB = linM0[CALIBRATE].dB;
  float att = EEprom.intAttKanal[kanal - 1];


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
  LCDout("Int. Att:", 0, 1, 10);
  dtostrf(EEprom.intAttKanal[kanal - 1], 5, 1, outstr);
  LCDout(outstr, 9, 1, 5);
  LCDout("dB", 14, 1, 2);
  EMPTY(outstr);

  LCDout("Pegel:", 0, 2, 6);
  dtostrf(linM0[CALIBRATE].dB, 5, 1, outstr);
  LCDout(outstr, 9, 2, 5);
  LCDout("dBm", 14, 2, 3);
  LCDout("<START>  <EXIT>", 1, 3, 16);
  // Kalibrierungsmenü
  while (!exitIT) {
    refreshCalPosition(edtPos);
    switch (edtPos) {
      case 0: // Daempfung
        if (fired)
          att = editFloat(EEprom.intAttKanal[kanal - 1], 9, 1, 4, 1, 5);
        break;
      case 1: //Pegel
        if (fired)
          dB = editFloat(EEprom.intAttKanal[kanal - 1], 9, 2, 4, 1, 5);
        break;
      case 2: //Start
        if (fired) {
          fired = false;
          exitIT = true;
          EEprom.intAttKanal[kanal - 1] = att;
        }
        break;
      case 3:  //Exit
        if (fired) {
          fired = false;
          return;
        }
        break;
    }
    fired = false;
    while (!exitIT && !fired && !turned);
    if (turned && up)
      edtPos < edtMax ? edtPos++ : (edtPos = 0);
    else if (turned && !up)
      edtPos > 0 ? edtPos-- : (edtPos = edtMax);
    turned = false;
  }
  fired = false;

  EMPTY(outstr);
  //lcd.clear();
  LCDout("Bitte Signal ...", 0, 2, 19);
  LCDout("Ext. Pegel:", 0, 1, 12);
  dtostrf(dB, 4, 1, outstr);
  LCDout(outstr, 12, 1, 5);
  LCDout("dBm", 17, 2, 3);
  Serial.print("Kalibrieung: ATT:");
  Serial.print(EEprom.intAttKanal[kanal - 1]);
  Serial.print("  Pegel:");
  Serial.println(dB);
  ref = linearizeDB(dB - EEprom.intAttKanal[kanal - 1], linM0, sizeof(linM0) / sizeof(struct LinStruct));
  Serial.print("Kalibrieung: Ref Stuetzstelle :");
  Serial.println(ref);
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
  LCDout("     Stufe:         ", 0, 1, 20);
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
    LCDout(outstr, 14, 3, 4);
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

/*
    Auswahl der Frequenz.
    Da dies öfters benötigt wird, gibt man x und y an, um die Position auf dem Screen mitzugeben
    Es werden die bandinfos aus der Struct ausgegeben, zurück wird der gewählte Indes geliefert
*/
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

/*
   Editierung von Float werden
   Wurde ausgelagert, da öfters gebraucht
   Position über x,y
   len, frac werden für dtostrf benötigt
   fStep gibt die Schrteittweite an
*/

float editFloat(float inValue, byte x, byte y, byte len, byte frac, float fStep) {
  char outstr[21];
  byte i;
  EMPTY(outstr);
  dtostrf(inValue, len, frac, outstr);
  LCDout(outstr, x, y, len);
  lcd.setCursor(x, y);
  lcd.blink();;
  fired = false;
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


/*
   Auswahl des Koppler
   x1,y1: Ausgabe der Kopplers und der Dämpfung
   x2,y2: AusgabeminQRG/maxQRG als weiteren Hinweis
*/
byte chooseKoppler(byte koppler, byte x1, byte y1, byte x2, byte y2) {
  char outstr[21];
  byte i;
  byte idx;
  byte qrgIdx;
  byte count;
  lcd.setCursor(x1, y1);
  lcd.cursor();
  lcd.blink();
  while (!fired) {

    if (turned && up)
      koppler < (KOPPLERSIZE - 1) ? koppler++ : (koppler = 0);
    else if (turned && !up)
      koppler > 0 ? koppler-- : (koppler = (KOPPLERSIZE - 1));
    // wir koennen nur konfigurierte Koppler wahelen, die konfiguriert sind
    while (!KOPPLERarray[koppler].configured) {
      koppler < (KOPPLERSIZE - 1) ? koppler++ : (koppler = 0);
      if (count > KOPPLERSIZE) { // da gibts keinen Koppler
        return;
      }
    }
    turned = false;
    EMPTY(outstr);
    itoa(koppler, outstr, 10);
    LCDout(outstr, x1, y1, 1);
    EMPTY(outstr);
    dtostrf(KOPPLERarray[koppler].attenuation, 4, 1, outstr);
    strcat(outstr, "dB ");
    LCDout(outstr, x1 + 2, y1, 7);
    LCDout("        ", x2, y2, 8);
    EMPTY(outstr);
    qrgIdx = KOPPLERarray[koppler].minQRGidx;
    strcat(outstr, QRGarray[koppler].Text);
    strcat(outstr, "/");
    qrgIdx = KOPPLERarray[idx].maxQRGidx;
    strcat(outstr, QRGarray[qrgIdx].Text);
    LCDout(outstr, x2, y2, 19);
    lcd.setCursor(x1, y1);
    while (!fired && !turned);
  }
  lcd.noCursor();
  lcd.noBlink();
  fired = false;
  return koppler;

}

byte countKoppler() {
  byte i;
  byte count = 0;
  for (i = 0; i < KOPPLERSIZE; i++) {
    if (KOPPLERarray[i].configured)
      count++;
  }
  return count;
}

/*
   Konfiguration des Eingangskanals
   Kanal wird mit 1 oder 2 angegeben

*/
void configureInput(byte kanal) {
  char outstr[21];
  byte i;
  byte pos = 1;
  byte oldPos = 0;
  const byte mSize = 3;
  byte idx;
  byte qrgIdx;
  byte count;

  count = countKoppler(); //wir zaehlen die Koppler
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
  LCDout("Koppel No:", 0, 2, 10);
  EMPTY(outstr);
  idx = EEprom.idxKoppler[kanal - 1];
  if (idx > KOPPLERSIZE) { // wurde beim initilaisieren gesetzt, also nix definiert
    LCDout("-", 10, 2, 1);
  }
  else {
    itoa(idx, outstr, 10);
    LCDout(outstr, 10, 2, 1);
    EMPTY(outstr);
    dtostrf(KOPPLERarray[idx].attenuation, 4, 1, outstr);
    strcat(outstr, "dB ");
    LCDout(outstr, 13, 2, 8 );
  }
  LCDout("<EXIT>", 13, 3, 6);
  EMPTY(outstr);
  qrgIdx = KOPPLERarray[idx].minQRGidx;
  strcat(outstr, QRGarray[qrgIdx].Text);
  strcat(outstr, "/");
  qrgIdx = KOPPLERarray[idx].maxQRGidx;
  strcat(outstr, QRGarray[qrgIdx].Text);
  LCDout(outstr, 0, 3, 19);
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
      else if (pos == 2 && count) { // Auswahl des Kopplers, nur wenn Koppler definiert sind
        if (idx > KOPPLERSIZE) {
          idx = 0;
        }
        EEprom.idxKoppler[kanal - 1] = chooseKoppler(idx, 10, 2, 0, 3);
        //EEprom.attKoppler[kanal - 1] = editFloat(EEprom.attKoppler[kanal - 1], 13, 2, 4, 1, 0.1);
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

/*
   Konfiguration der PA des Eingangskanals
   Kanal wird mit 1 oder 2 angegeben

*/
void configurePA(byte kanal) {
  char outstr[21];
  byte i;
  byte pos = 1;
  byte oldPos = 0;
  const byte mSize = 6;
  byte idx;
  byte qrgIdx;
  byte count;

  idx = kanal - 1;

  lcd.clear();
  EMPTY(outstr);
  strcpy(outstr, "Konf.PA");
  LCDout(outstr, 0, 0, 7);
  EMPTY(outstr);
  itoa(kanal, outstr, 10);
  LCDout(outstr, 7, 0, 1);
  EMPTY(outstr);
  LCDout("G:", 10, 0, 2);
  EMPTY(outstr);
  dtostrf(PAarray[idx].gLin, 2, 0, outstr);
  LCDout(outstr, 12, 0, 2);
  LCDout("dB", 14, 0, 3);

  LCDout("PIl:", 0, 1, 4);
  EMPTY(outstr);
  dtostrf(PAarray[idx].piLin, 2, 0, outstr);
  LCDout(outstr, 4, 1, 2);
  LCDout("dBm", 6, 1, 3);
  LCDout("PIm:", 10, 1, 4);
  EMPTY(outstr);
  dtostrf(PAarray[idx].piMax, 2, 0, outstr);
  LCDout(outstr, 14, 1, 2);
  LCDout("dBm", 16, 1, 3);

  LCDout("POm:", 0, 2, 4);
  EMPTY(outstr);
  dtostrf(PAarray[idx].poMax, 2, 0, outstr);
  LCDout(outstr, 4, 2, 2);
  LCDout("dBm", 6, 2, 3);
  LCDout("PIx:", 10, 2, 4);
  EMPTY(outstr);
  dtostrf(PAarray[idx].piMxx, 2, 0, outstr);
  LCDout(outstr, 14, 2, 2);
  LCDout("dBm", 16, 2, 3);

  LCDout("Att:", 0, 3, 5);
  EMPTY(outstr);
  dtostrf(PAarray[idx].attKabel, 4, 1, outstr);
  LCDout(outstr, 5, 3, 4);
  LCDout("dB", 9, 3, 3);
  LCDout("EXIT", 15, 3, 5);
  pos = 6; // auf EXIT positionieren
  while (1) {
    refreshPAPosition(pos);
    if (turned) {
      if  (up)
        pos < mSize ? pos++ : (pos = 0);
      else if (!up)
        pos > 0 ? pos-- : (pos = mSize);
      turned = false;
    }
    if (fired) {
      fired = false;
      switch (pos) {
        case 0: // Gain
        PAarray[idx].gLin=editFloat(PAarray[idx].gLin, 12, 0, 2, 0, 1);
        pos=1;
          break;
        case 1: // piLin
        PAarray[idx].piLin=editFloat(PAarray[idx].piLin, 4, 1, 2, 0, 1);
        pos=2;
          break;
        case 2: // piMax
          PAarray[idx].piMax=editFloat(PAarray[idx].piMax, 14, 1, 2, 0, 1);
          pos=3;
          break;
        case 3: // poMax
        PAarray[idx].poMax=editFloat(PAarray[idx].poMax, 4, 2, 2, 0, 1);
          pos=4;
          break;
        case 4: // piMxx
        PAarray[idx].piMxx=editFloat(PAarray[idx].piMxx, 14, 2, 2, 0, 1);
          pos=5;
          break;
          case 5: // attKabel
        PAarray[idx].attKabel=editFloat(PAarray[idx].attKabel, 5, 3, 4, 1, 0.5);
          pos=6;
          break; 
        case 6: //EXIT;
        lcd.clear();
          return;
          break;
      }

    }
    delay(20);
  }

 

}


void refreshPAPosition(byte edtPos) {
  switch (edtPos) {
    case 0:
      LCDout("<", 18, 0, 1);
      LCDout(" ", 9, 1, 1);
      LCDout(" ", 19, 1, 1);
      LCDout(" ", 9, 2, 1);
      LCDout(" ", 19, 2, 1);
      LCDout(" ", 12, 3, 1);
      LCDout(" ", 14, 3, 1);
      break;
    case 1:
      LCDout(" ", 18, 0, 1);
      LCDout("<", 9, 1, 1);
      LCDout(" ", 19, 1, 1);
      LCDout(" ", 9, 2, 1);
      LCDout(" ", 19, 2, 1);
      LCDout(" ", 12, 3, 1);
      LCDout(" ", 14, 3, 1);
      break;
    case 2:
      LCDout(" ", 18, 0, 1);
      LCDout(" ", 9, 1, 1);
      LCDout("<", 19, 1, 1);
      LCDout(" ", 9, 2, 1);
      LCDout(" ", 19, 2, 1);
      LCDout(" ", 12, 3, 1);
      LCDout(" ", 14, 3, 1);
      break;
    case 3:
      LCDout(" ", 18, 0, 1);
      LCDout(" ", 9, 1, 1);
      LCDout(" ", 19, 1, 1);
      LCDout("<", 9, 2, 1);
      LCDout(" ", 19, 2, 1);
      LCDout(" ", 12, 3, 1);
      LCDout(" ", 14, 3, 1);
      break;
    case 4:
      LCDout(" ", 18, 0, 1);
      LCDout(" ", 9, 1, 1);
      LCDout(" ", 19, 1, 1);
      LCDout(" ", 9, 2, 1);
      LCDout("<", 19, 2, 1);
      LCDout(" ", 12, 3, 1);
      LCDout(" ", 14, 3, 1);
      break;
    case 5:
      LCDout(" ", 18, 0, 1);
      LCDout(" ", 9, 1, 1);
      LCDout(" ", 19, 1, 1);
      LCDout(" ", 9, 2, 1);
      LCDout(" ", 19, 2, 1);
      LCDout("<", 12, 3, 1);
      LCDout(" ", 14, 3, 1);
      break;
    case 6:
      LCDout(" ", 18, 0, 1);
      LCDout(" ", 9, 1, 1);
      LCDout(" ", 19, 1, 1);
      LCDout(" ", 9, 2, 1);
      LCDout(" ", 19, 2, 1);
      LCDout(" ", 12, 3, 1);
      LCDout(">", 14, 3, 1);
  }
}
/*
  LCDout("<EXIT>", 13, 3, 6);
  EMPTY(outstr);
  qrgIdx = KOPPLERarray[idx].minQRGidx;
  strcat(outstr, QRGarray[qrgIdx].Text);
  strcat(outstr, "/");
  qrgIdx = KOPPLERarray[idx].maxQRGidx;
  strcat(outstr, QRGarray[qrgIdx].Text);
  LCDout(outstr, 0, 3, 19);
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
      else if (pos == 2 && count) { // Auswahl des Kopplers, nur wenn Koppler definiert sind
        if (idx > KOPPLERSIZE) {
          idx = 0;
        }
        EEprom.idxKoppler[kanal - 1] = chooseKoppler(idx, 10, 2, 0, 3);
        //EEprom.attKoppler[kanal - 1] = editFloat(EEprom.attKoppler[kanal - 1], 13, 2, 4, 1, 0.1);
      }
      else if (pos == 3) {
        lcd.noCursor();
        lcd.noBlink();
        return;

      }
    }
    delay(20);
  }

*/


/*
   Refresh des Bildschirm bei der Kopplerdefinition
   Wurde der Übersichtlichkeit ausgelagert
*/



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

/*
   Position den Cursors im SWR Menü Modus
*/

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


/*
    Editierung der Stützstellen eines Kopplers

*/

void editStuetzstellen(byte koppler) {
  // Pflege der Stuetzstellen im globalen StructArray
  byte i;
  char outstr[21];
  byte minIdx;
  byte maxIdx;
  byte idx;
  boolean exitIT = false;
  float att;

  lcd.clear();
  LCDout("EDIT Koppler:", 0, 0, 14);
  EMPTY(outstr);
  itoa(koppler, outstr, 10);
  LCDout(outstr, 15, 0, 2);
  LCDout("Min:", 0, 1, 4);
  LCDout("Max:", 10, 1, 4);
  LCDout("<SAVE>", 7, 3, 6);
  minIdx = KOPPLERarray[koppler].minQRGidx;
  maxIdx = KOPPLERarray[koppler].maxQRGidx;
  LCDout(QRGarray[minIdx].Text, 5, 1, 4);
  LCDout(QRGarray[maxIdx].Text, 14, 1, 4);
  //KOPPLERarray[koppler].sigDB[i]=KOPPLERarray[koppler].attenuation;
  idx = minIdx;
  fired = false;
  turned = false;
  while (idx <= maxIdx) {
    LCDout("    ", 0, 2, 4);
    LCDout(QRGarray[idx].Text, 0, 2, 4);
    LCDout(":", 4, 2, 1);
    EMPTY(outstr);
    att = KOPPLERarray[koppler].sigDB[idx];
    dtostrf(att, 4, 1, outstr);
    LCDout(outstr, 7, 2, 4);
    LCDout("dB", 12, 2, 2);
    if (!exitIT) {
      LCDout("<<", 15, 2, 2);
      LCDout("        ", 6, 3, 8);
      LCDout("<SAVE>", 7, 3, 6);
    }
    while (!turned && !fired) {} ;
    if (fired && exitIT) {
      fired = false;
      lcd.clear();
      return;
    }
    if (fired) {
      KOPPLERarray[koppler].sigDB[idx] = editFloat(att, 7, 2, 4, 1, 0.1);
      fired = false;
    }
    if (turned && exitIT) {
      //Save wieder verlassen
      exitIT = false;
      idx = maxIdx;
      turned = false;
    }
    if (turned && up && (idx < maxIdx))
      idx++;
    else if (turned && !up && (idx > minIdx ) )
      idx--;
    else if ( turned && up && (idx == maxIdx)) { // Sonderbehandlung, wir springen auf SAVE
      LCDout("  ", 15, 2, 2);
      LCDout("<<SAVE>>", 6, 3, 8);
      exitIT = true;
    }
    turned = false;

  }

}

/*
   Konfiguration der Koppler
   Eigenes Menüsystem
*/
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
        configured = KOPPLERarray[koppler].configured;
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
            //if (!configured) {
            // nun fuellen wir das sigDB Array mit dem konfigurierten Wert
            for (i = 0; i < QRGSIZE; i++)
              KOPPLERarray[koppler].sigDB[i] = KOPPLERarray[koppler].attenuation;
            //}
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
        editStuetzstellen(koppler);
        refreshKoppler(koppler);
        state = SELECT;
        break;

    } //switch state
  } //while (außen)

} //configureKoppler


/*
   Messwerte des AD Kopplers aufnehmen
   Das ist wegen der SWR Messung zeitkritisch
   Deshalb zuerst beide Werte einlesen und dann die Linearisierung druchführen
*/


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
  // Begrenzung der Werte --> wurde nach lienarize verschoben
  //
  //if (MW > 981,6) MW = 981,6;
  //if (MW < 231,1) MW = 231,1;
  if (corrU > linM[LOW_LIMIT].ADnorm)         corrU =  linM[LOW_LIMIT].ADnorm;    //#define LOW_LIMIT 16  entspricht -70dBm
  else if (corrU < linM[HIGH_LIMIT].ADnorm)    corrU =  linM[HIGH_LIMIT].ADnorm;
  return corrU;
}


/*
    Linearisierung anhand der Stützstellen
    Das Array und die Größe werden nicht aus der globalen Definition genutzt, sondern übergeben
    Das dient der Vorbereitung, um mit zwei Linearisierungs Array getrennt für beide Kanäle arbeiten zu können
    Wir aber zurzeit nur ein Array verwendet
*/


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
/*
   Rueckwartslinearisierung
   Eingabe: Pegel, Ausgabe rawU (ADnorm)
   Dient bei der Kalibrierung zur Bestimmung des Refernzswertes
*/

float linearizeDB(float dB, struct LinStruct * linM, byte linMsize ) {
  // Interpoliert anhand der liM Struct und gibt rawU zurueck
  // dB: Pegel
  // linM: Zeiger auf Array mit den Lineariserungswerten
  // linMSize: Größe des Arries
  byte pU = 0;
  float U = 0.0;
  linM = linM0;
  //int linMsize=sizeof(linM0)/sizeof(struct LinStruct);
  //byte linMsize=sizeof(linM0)/sizeof(LinStruct);

  for (int q = 0; q < linMsize; q++) {
    if (dB < (float) linM[q].dB) {
      pU = q;
      continue;
    }
    else if (dB == linM[q].dB) { // Treffer Stuetzstelle
      return linM[q].ADnorm;
    }
  }

  if (pU > (linMsize - 1))    U = linM[linMsize].ADnorm;
  else if (pU < 1)          U = linM[0].ADnorm;
  else                      U = ((dB - linM[pU].dB) / (linM[pU + 1].dB - linM[pU].dB)) * (linM[pU + 1].ADnorm - linM[pU].ADnorm) + linM0[pU].ADnorm;
  return U; // Rueckgabe des interpolierten ADnorm wertes
}

/*
   Zentrale Ausgaberoutine für Bildschirmausgaben
*/

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

/*
    Hier folgen die Definitionen der verschiedenen Ausgabeformate
    Diese werden in write_lcd() entsprechend der in check_encoder() vorgenommenen Auswahl angesprungen
*/

/*
   SWR Screen
*/

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
  LCDout("       ", 13, 3, 7);
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

}

/*
   Detailausgabe Kanal1/2, wird für die Kalibrierung verwendet
*/

void screen1(byte kanal, char *text, float Ulin, float Att, float dbK,  char *QRGtext) {
  int lbg_draw_val_limited;
  float PmW_out = 0;
  float PmW = 0;
  char outstr[30];
  int i;
  byte mb = 4;
  float Um = 0;
  PmW = pow(10, Ulin / 10.0);
  mb = MBselect(PmW);

  outstr[0] = '\0';
  EMPTY(outstr);
  itoa(kanal, outstr, 10);
  strcat(outstr, ">A:");
  LCDout(outstr, 0, 0, 4);
  dtostrf(Att, 4, 1, outstr);
  strcat(outstr, "dB");
  LCDout(outstr, 4, 0, 8);

  LCDout("K:", 12, 0, 2);
  EMPTY(outstr);
  dtostrf(dbK, 4, 1, outstr);
  strcat(outstr, "dB");
  LCDout(outstr, 14, 0, 8);

  PmW_out = PmW / MBwahl[mb].Divisor;

  EMPTY(outstr);
  LCDout(text, 0, 1, 4);
  // wir zeigen Ulin an, --> Ulin
  dtostrf(Ulin, 4, 1, outstr);
  strcat(outstr, "dBm ");
  LCDout(outstr, 4, 1, 8);
  EMPTY(outstr);
  dtostrf(PmW_out, 5, MBwahl[mb].Nachkomma, outstr);
  strcat(outstr, MBwahl[mb].Unit);
  LCDout(outstr, 13, 1, 8);

  LCDout("Band:", 0, 2, 5);
  LCDout(QRGtext, 5, 2, 5);


  //Den zu zeichnenden Bargraph mit dem jeweiligen Messbereich skalieren und auf 999 begrenzen

  // der bezug des Graphen muss wahelbar sein

  lbg_draw_val_limited = int(1000 * PmW / (MBwahl[mb].range));
  if (lbg_draw_val_limited > 999) lbg_draw_val_limited = 999;
  lbgPWR.drawValue(lbg_draw_val_limited, 1000);
  LCDout("       ", 13, 3, 7);
  EMPTY(outstr);
  strcpy(outstr, "[");
  strcat(outstr, MBwahl[mb].Text);
  strcat(outstr, "]");

  LCDout(outstr, 13, 3, 7);
}


/*
   DUAL Screen, ohne Koppler
*/

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

/*
   DUAL Screen mit Koppler
*/

//screen3(U1lin + AttKanal1, U2lin + AttKanal2, EEprom.idxKoppler[0], EEpromidxKoppler[1], QRGarray[QRGidx1].Text, QRGarray[QRGidx2].Text);
void screen3( float U1, float U2, float dbK1, float dbK2, char *QRGtext1, char *QRGtext2) {
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
  LCDout("K:", 0, 3, 2);
  dtostrf(dbK1, 4, 1, outstr);
  LCDout(outstr, 2, 3, 4);
  LCDout("dB", 6, 3, 4);
  EMPTY(outstr);
  LCDout("K:", 10, 3, 2);
  dtostrf(dbK2, 4, 1, outstr);
  LCDout(outstr, 12, 3, 4);
  LCDout("dB", 16, 3, 4);

}

/*
   Ausgabe PA und Kabeldaempfung
*/

void screen4(byte kanal, char *text, float Ulin,  char *QRGtext, float PiLin, float PiMax, float Glin, float PoMax, float PiMxx, float AttKabel) {
  int lbg_draw_val_limited;
  float PmW = 0.0;
  float PmW_out = 0.0;
  float Gdiff = 0.0;
  char outstr[30];
  char state[4];
  int i;
  byte mb = 4;
  float Um = 0;

  // wir berechnen den Status des Signals
  EMPTY(state);
  if ((Ulin - AttKabel) < PiLin)
    strcpy(state, "LIN");
  else if ((Ulin - AttKabel) < PiMax)
    strcpy(state, "SAT");
  else if ((Ulin - AttKabel) < PiMxx)
    strcpy(state, "MAX");
  else
    strcpy(state, "!!!");

  //Ulin hat den Wert hinter dem Koppler, also gemessene Ausgangsleistung
  PmW = pow(10, Ulin / 10.0);
  mb = MBselect(PmW);

  outstr[0] = '\0';
  EMPTY(outstr);
  itoa(kanal, outstr, 10);
  strcat(outstr, "Po:");
  LCDout(outstr, 0, 0, 4);
  dtostrf(Ulin, 4, 1, outstr);
  strcat(outstr, "dBm");
  LCDout(outstr, 4, 0, 8);

  LCDout(state, 14, 0, 3);
  EMPTY(outstr);
  //mW Ausgang Koppler
  PmW_out = PmW / MBwahl[mb].Divisor;


  // Nun Kommt der Eingangspegel der PA
  Ulin -= AttKabel;

  EMPTY(outstr);
  LCDout("PAi:", 0, 1, 4);
  dtostrf(Ulin, 4, 1, outstr);
  strcat(outstr, "dBm ");
  LCDout(outstr, 4, 1, 8);
  EMPTY(outstr);
  PmW = pow(10, Ulin / 10.0);
  mb = MBselect(PmW);
  PmW_out = PmW / MBwahl[mb].Divisor;
  dtostrf(PmW_out, 5, MBwahl[mb].Nachkomma, outstr);
  strcat(outstr, MBwahl[mb].Unit);
  LCDout(outstr, 13, 1, 8);

  // nun Ausgangspegel der PA
  // hier muessen wir rechnen
  // Kabeldaempfung ist bei Ulin schon abgezogen
  if (Ulin < PiLin) { // linearer Bereich
    Ulin += Glin;
  }
  else if (Ulin < PiMax) { // Saettigungsbereich
    Gdiff = (Glin - PoMax + PiMax) / (PiMax - PiLin);
    Ulin +=  Glin - (Gdiff * (Ulin - PiLin));
  }
  else { // mehr geht nicht, als PoMax setzen
    Ulin = PoMax;
  }



  EMPTY(outstr);
  LCDout("PAo:", 0, 2, 4);
  dtostrf(Ulin, 4, 1, outstr);
  strcat(outstr, "dBm ");
  LCDout(outstr, 4, 2, 8);
  EMPTY(outstr);
  PmW = pow(10, Ulin / 10.0);
  mb = MBselect(PmW);
  PmW_out = PmW / MBwahl[mb].Divisor;
  dtostrf(PmW_out, 5, MBwahl[mb].Nachkomma, outstr);
  strcat(outstr, MBwahl[mb].Unit);
  LCDout(" ", 19, 2, 1);
  LCDout(outstr, 13, 2, 8);

  //Den zu zeichnenden Bargraph mit dem jeweiligen Messbereich skalieren und auf 999 begrenzen

  // der bezug des Graphen muss wahelbar sein

  lbg_draw_val_limited = int(1000 * PmW / (MBwahl[mb].range));
  if (lbg_draw_val_limited > 999) lbg_draw_val_limited = 999;
  lbgPWR.drawValue(lbg_draw_val_limited, 1000);


  LCDout("       ", 13, 3, 7);

  EMPTY(outstr);
  strcpy(outstr, "[");
  strcat(outstr, MBwahl[mb].Text);
  strcat(outstr, "]");

  LCDout(outstr, 13, 3, 7);
}

/*
   Ausgabe des Hauptmenüs aus SWR heraus
*/

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

/*
   Cursorsteuerung für DUAL Konfigurationsseite
*/

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

/*
   Cursprsteurung für DUAL Koppler Konfiguration
*/

void configureDUALKoppler() {
  // Konfiguration der Dual Anzeige
  switch (dualKopplerPos) {
    case 0:
      LCDout("<", 9, 2, 1);
      LCDout(" ", 19, 2, 1);
      break;
    case 1:
      LCDout(" ", 9, 2, 1);
      LCDout("<", 19, 2, 1);
      break;
  }
}


void screenStatus() {
  char outstr[30];
  int i;
  EMPTY(outstr);
  LCDout("CFG Uebersicht:", 0, 0, 15);
  LCDout("1:Sc:", 0, 1, 5);
  dtostrf(EEprom.scalFactor[0], 5, 3, outstr);
  LCDout(outstr, 5, 1, 5);
  EMPTY(outstr);
  LCDout("AT:", 11, 1, 3);
  EMPTY(outstr);
  dtostrf(EEprom.intAttKanal[0], 4, 1, outstr);
  LCDout(outstr, 14, 1, 4);
  LCDout("dB", 18, 1, 2);
  LCDout("2:Sc:", 0, 2, 5);
  dtostrf(EEprom.scalFactor[1], 5, 3, outstr);
  LCDout(outstr, 5, 2, 5);
  EMPTY(outstr);
  LCDout("AT:", 11, 2, 3);
  EMPTY(outstr);
  dtostrf(EEprom.intAttKanal[1], 4, 1, outstr);
  LCDout(outstr, 14, 2, 4);
  LCDout("dB", 18, 2, 2);
  LCDout("Kop:", 0, 3, 4);
  EMPTY(outstr);
  itoa(countKoppler(), outstr, 10);
  LCDout(outstr, 4, 3, 1);
  LCDout("<<RESET>>", 7, 3, 10);
}

/*
   Hauptroutine für die Ausgabe auf LCD
   Von hier aus werden die screen[0..]() Funktionen angesprungen
   Steuerung erfolgt über check_encoder()
   Konfigurationen direkt im screen ohne neues Bildschirm werden auch von hier aus angesprungen (die anderen Konfigurationen siehe check_encoder())
*/

void write_lcd() //auffrischen des LCD  - wird alle 100ms angestossen
{
  float VSWR = 1.0;
  float P1mW_out = 0;
  float P2mW_out = 0;
  float dbK1;
  float dbK2;
  float U1lin = 0;
  float U2lin = 0;
  char outstr[21];
  char outstr1[21];
  byte i;
  byte count;

  //String outstr;
  //static float old_U1 = 0.0;
  //String P_unit1, P_unit2;
  //int lbg_draw_val_limited;



  // Lienarisierung aufrufen.
  // Länge der LinM Struct kann nicht in der Funktion bestimmt werden (3.Argument)
  // Beruecksichtigung der internen Daempfung
  U1lin = linearize(smoothU1, linM0, sizeof(linM0) / sizeof(struct LinStruct)) + EEprom.intAttKanal[0];
  U2lin = linearize(smoothU2, linM0, sizeof(linM0) / sizeof(struct LinStruct)) + EEprom.intAttKanal[1];

  // wir setzen die Auskoppeldaempfung anhand der gewaehlten Frequenz und der der Stuetzstellen
  if (EEprom.idxKoppler[0] > KOPPLERSIZE)
    Koppler1 = 0.0;
  else
    Koppler1 = KOPPLERarray[EEprom.idxKoppler[0]].sigDB[QRGidx1];
  if (EEprom.idxKoppler[1] > KOPPLERSIZE)
    Koppler2 = 0.0;
  else
    Koppler2 = KOPPLERarray[EEprom.idxKoppler[1]].sigDB[QRGidx2];


  U1 = U1lin + Koppler1 + AttKanal1; //Auskoppeldaempfung und evtl. zusätzliches externes Dämpfungsglied
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
      screen1(1, "IN:", U1lin, AttKanal1, Koppler1, QRGarray[QRGidx1].Text);
      break;
    case INCFG1:
      switch (inPos) { // Auswahl der Anzeige in IN1/IN2 nach druecken des Buttons
        case 0:
          screen1(1, "IN>", U1lin, AttKanal1, Koppler1, QRGarray[QRGidx1].Text);
          break;
        case 1:
          screen1(1, "AT>", U1lin + AttKanal1, AttKanal1, Koppler1, QRGarray[QRGidx1].Text);
          break;
        case 2:
          screen1(1, "KO>", U1lin + AttKanal1 + Koppler1, AttKanal1, Koppler1, QRGarray[QRGidx1].Text);
          break;
        default:
          Serial.println("INCFG Screen???");

      }
      break;
    case IN2:
      screen1(2, "IN:", U2lin, AttKanal2, Koppler2, QRGarray[QRGidx2].Text);
      break;
    case INCFG2:
      switch (inPos) { // Auswahl der Anzeige in IN1/IN2 nach druecken des Buttons
        case 0:
          screen1(2, "IN>", U2lin, AttKanal2, Koppler2, QRGarray[QRGidx2].Text);
          break;
        case 1:
          screen1(2, "AT>", U2lin + AttKanal2, AttKanal2, Koppler2, QRGarray[QRGidx2].Text);
          break;
        case 2:
          screen1(2, "KO>", U2lin + AttKanal2 + Koppler2, AttKanal2, Koppler2, QRGarray[QRGidx2].Text);
          break;
      }
      break;
    case INPA1:
      screen4(1, "PA", U1lin + AttKanal1 + Koppler1, QRGarray[QRGidx1].Text, PAarray[0].piLin, PAarray[0].piMax, PAarray[0].gLin, PAarray[0].poMax, PAarray[0].piMxx, PAarray[0].attKabel);
      break;
    case INPA2:
      screen4(2, "PA", U2lin + AttKanal2 + Koppler2, QRGarray[QRGidx2].Text, PAarray[1].piLin, PAarray[1].piMax, PAarray[1].gLin, PAarray[1].poMax, PAarray[1].piMxx, PAarray[1].attKabel);
      break;
    case DUAL:
      screen2(U1lin + AttKanal1, U2lin + AttKanal2, AttKanal1, AttKanal2, QRGarray[QRGidx1].Text, QRGarray[QRGidx2].Text);
      break;
    case DUALKOPPLER:

      if (EEprom.idxKoppler[0] > KOPPLERSIZE)
        dbK1 = 0.0;
      else
        dbK1 = KOPPLERarray[EEprom.idxKoppler[0]].sigDB[QRGidx1];
      if (EEprom.idxKoppler[1] > KOPPLERSIZE)
        dbK2 = 0.0;
      else
        dbK2 = KOPPLERarray[EEprom.idxKoppler[1]].sigDB[QRGidx2];
      screen3(U1lin + AttKanal1 + dbK1, U2lin + AttKanal2 + dbK2, dbK1 , dbK2, QRGarray[QRGidx1].Text, QRGarray[QRGidx2].Text);
      break;
    case MENU:
      showMenu();
      break;
    case DUALCFG:
      configureDUAL();
      screen2(U1lin + AttKanal1, U2lin + AttKanal2, AttKanal1, AttKanal2, QRGarray[QRGidx1].Text, QRGarray[QRGidx2].Text);

      break;
    case DUALKOPPLERCFG:
      configureDUALKoppler();
      //Serial.println(EEprom.idxKoppler[1]);
      if (EEprom.idxKoppler[0] > KOPPLERSIZE)
        dbK1 = 0.0;
      else
        dbK1 = KOPPLERarray[EEprom.idxKoppler[0]].sigDB[QRGidx1];
      if (EEprom.idxKoppler[1] > KOPPLERSIZE)
        dbK2 = 0.0;
      else
        dbK2 = KOPPLERarray[EEprom.idxKoppler[1]].sigDB[QRGidx2];
      screen3(U1lin + AttKanal1 + dbK1, U2lin + AttKanal2 + dbK2, dbK1 , dbK2, QRGarray[QRGidx1].Text, QRGarray[QRGidx2].Text);
      break;
    case STATUS: //Ausgabe der konfigurierten Werte
      count = countKoppler();
      screenStatus(); // die Werte werden dort aus den globalen Einstellungen ermittelt

  }
  refresh = false;



}

/*
   Auswahl eines Messbereichs anhand eines Eingangswertes im mW
   Verändert keine globalen Variablen
*/

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

/*
   Auswahl des Messbereiches.
   Setzt globale Varablen MB[kanal]
*/

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


/*
   Ausgabe der Messwerte auf serielle Konsole
*/

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
