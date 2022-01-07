#include <Event.h>
#include <Timer.h>


#include <LiquidCrystal.h>
#include <LcdBarGraph.h>
#include <stdio.h>


const float smooth = 0.004;
//> timeconstant=200ms (app. 1/smooth * TA (10ms)) @smooth=0.01-> timeconstant=1000ms ... - correct for smooth << 1

byte pin_U1 = A0;
byte pin_U2 = A1;

int T1=0;
int T2=1023;
int posi=0;
float linM0[17][17];
float linM1[17][17];

int MB1 = 5;  //Reserve nach unten hin
int MB2 = 5;  //Reserve nach unten hin
float P1mW = 0.0;
float P2mW = 0.0;
Timer t;

float rawU1=0.0; // takes averaged but unscaled value
float rawU2=0.0;
float U1=0.0; // scaled values!
float U2=0.0;
char outstr4[4];
char outstr3[3];
float Koppler1 = 0.0;  //dB
float Koppler2 = 0.0;  //dB

#define PINA 21
#define PINB 22
#define PUSHB 20
#define INTERRUPT 0
#define INTERRUPTB 1

volatile boolean turned;   // rotary was turned
volatile boolean fired;    // knob was pushed
volatile boolean up;  // true when turned cw

//#########################################################
// menu handling
int Menu_page = 1;
int item_pos = 1;
boolean edit = false;
boolean change_value = false;
int delta_value = 0;
boolean items_changed = false;
//############################################################

// Interrupt Service Routine - für den Button des Encoder
void isrB ()
{
  if (!digitalRead (PUSHB))
    fired = true;
}  // end of isr


// Interrupt Service Routine für die Drehbewegung
void isr ()
{
  if (digitalRead (PINA))
    up = digitalRead (PINB);
  else
    up = !digitalRead (PINB);
  turned = true;
}  // end of isr

void check_encoder(){
  if (turned)
    {
      if (!edit && !change_value){  // wir sind nicht im Editiermodus
        if (up){
         if (Menu_page < 3) Menu_page++;  //dann schalten wir die Menüseite um - wenn wir später mehrere haben - Anzahl noch hardkodiert
        }  
        else
        if (Menu_page > 1) Menu_page--;    
      }

      if (edit && !change_value){  // wir sind im Editiermodus uns wählen das zu ändernde Item aus - Anzahl noch hardkodiert

        if (up){
         if (item_pos < 3) item_pos++;
        }  
        else
        if (item_pos > 1) item_pos--;    
      
       resetCursor();      
      }
     if (edit && change_value){ // wir sind im Editiermodus und haben zusätzlich ein Item zum Ändern ausgewählt

      if (up) delta_value++;
      else delta_value--;
      
     }
    turned = false;
    }

if (fired && change_value) {
  change_value = false;
  fired = false;
 
}

if (fired && edit && !change_value) { // rechts unten : EXIT
  /*if (item_pos == 4) {  // exit !
    edit = false;
    change_value = false;
    fired = false;
    lcd.noBlink();
    lcd.noCursor();
  items_changed = false; 
   }
  */
 /* if (fired && (item_pos == 3) && items_changed){  // links unten : SAVE

    automatik=temp_automatik;// lokale Änderungen speichern! 
    Man_state = temp_Man_state;
    items_changed = false;
    fired = false;
  }
*/

  if (fired && !change_value &&(item_pos == 1)) {  // Koppler 1  soll geändert werden
    change_value = true;  
    fired = false;
    items_changed = true;
  }
    if (fired && !change_value &&(item_pos == 3)) {  // Koppler 2 soll geändert werden
    change_value = true;  
    fired = false;
    items_changed = true;
  }
  if (fired && !change_value &&(item_pos == 2)) {  // Messfrequenz soll geändert werden
    change_value = true;  
    fired = false;
    items_changed = true;
  }
}

if(change_value){

  if (item_pos == 1) {  // Koppler 1 soll geändert werden
      
      Koppler1 = Koppler1 + delta_value*0.1;
      if (Koppler1 < 0.0)  Koppler1 =0.0;   
      delta_value = 0;
    
    }
  if (item_pos == 3) {  // Koppler 2 soll geändert werden
      
      Koppler2 = Koppler2 + delta_value*0.1;
      if (Koppler1 < 0.0)  Koppler1 =0.0;   
      delta_value = 0;
      } 
      
    }

    
}


LiquidCrystal lcd(27, 28, 29, 30, 31, 32); // -- creating LCD instance

LcdBarGraph lbgPWR(&lcd, 20,0,3);  // 


void setup(){
    Serial.begin(115200);
  // -- initializing the LCD
  lcd.begin(20,4);
  lcd.clear();

 pinMode(PINA,INPUT_PULLUP);
 pinMode(PINB,INPUT_PULLUP);
 pinMode(PUSHB,INPUT_PULLUP);
 

 attachInterrupt (digitalPinToInterrupt(21), isr, CHANGE);   // interrupt 0 is pin 2
 attachInterrupt (digitalPinToInterrupt(20), isrB, FALLING);   // interrupt 5 is pin 18
  
  analogReference(INTERNAL1V1); // set internal refernce to 1.1V
  
  lcd.setCursor(0,0);
  lcd.print("     SWR-Meter");
  lcd.setCursor(0,1);
  lcd.print(" 10 MHz ... 10 GHz");
  delay(2000);
  lcd.clear();

//LcdBarGraph lbgPWR(&lcd, 0,6,3);  // 

//Linearisierungstabelle für den Messverstärker [db vs. Millivolt]

linM0[0][0]=10.0; linM0[0][1]=188.2;
linM0[1][0]=5.0; linM0[1][1]=191,1;
linM0[2][0]=0.0; linM0[2][1]=197.7;
linM0[3][0]=-5.0; linM0[3][1]=217.4;
linM0[4][0]=-10.0; linM0[4][1]=255.0;
linM0[5][0]=-15.0; linM0[5][1]=312.3;
linM0[6][0]=-20.0; linM0[6][1]=374.9;
linM0[7][0]=-25.0; linM0[7][1]=435.7;
linM0[8][0]=-30.0; linM0[8][1]=497.4;
linM0[9][0]=-35.0; linM0[9][1]=560.0;
linM0[10][0]=-40.0; linM0[10][1]=622.2;
linM0[11][0]=-45.0; linM0[11][1]=688.0;
linM0[12][0]=-50.0; linM0[12][1]=750.1;
linM0[13][0]=-55.0; linM0[13][1]=815.6;
linM0[14][0]=-60.0; linM0[14][1]=877.0;
linM0[15][0]=-65.0; linM0[15][1]=917.6;
linM0[16][0]=-70.0; linM0[16][1]=936.5;

/*   Wird bei größerem Prozessor aktiviert!!!
linM1[0][0]=10.0;   linM1[0][1]=188.2;
linM1[1][0]=5.0;    linM1[1][1]=191,1;
linM1[2][0]=0.0;    linM1[2][1]=200.1;
linM1[3][0]=-5.0;   linM1[3][1]=221.2;
linM1[4][0]=-10.0;  linM1[4][1]=259.0;
linM1[5][0]=-15.0;  linM1[5][1]=312.3;
linM1[6][0]=-20.0;  linM1[6][1]=374.9;
linM1[7][0]=-25.0;  linM1[7][1]=435.7;
linM1[8][0]=-30.0;  linM1[8][1]=497.4;
linM1[9][0]=-35.0;  linM1[9][1]=560.0;
linM1[10][0]=-40.0; linM1[10][1]=622.2;
linM1[11][0]=-45.0; linM1[11][1]=688.0;
linM1[12][0]=-50.0; linM1[12][1]=750.1;
linM1[13][0]=-55.0; linM1[13][1]=815.6;
linM1[14][0]=-60.0; linM1[14][1]=877.0;
linM1[15][0]=-65.0; linM1[15][1]=917.6;
linM1[16][0]=-70.0; linM1[16][1]=936.5;

*/

// Hier das eigentliche "Multitasking" 
// alle 1000ms Ausgabe der Rohsignal an den PC
// alle 10ms die A/D-Wandler einlesen
// alle 100ms das LCD beschreiben
// alle 20ms  auf Encoderereignisse reagieren

  t.every(1000,write_raw_seriell,0);
  t.every(10,take_ads,0); //lese die analogen Eingänge alle 10ms und glätte diese
  t.every(100,write_lcd,0); //alle 100ms auf LCD darstellen
  t.every(20,check_encoder,0);// alle 20ms Änderungen des Encoders detektieren
}

void loop()
{
   
t.update(); //timer handling!	mehr passiert in der Hauptschleife nicht, Multitasking läuft im Hintergrund
  
}

void take_ads()
{
rawU1=(1.0 - smooth) * rawU1 + smooth * (float)analogRead(pin_U1);	
rawU2=(1.0 - smooth) * rawU2 + smooth * (float)analogRead(pin_U2);	

}


void write_lcd() //auffrischen des LCD  - wird alle 100ms angestossen
{
  int pU1=0;
  int pU2=0;
  float VSWR=1.0;
  float P1mW_out;
  float P2mW_out;
  float Divisor1=1.0;
  float Divisor2=1.0;
  int Nachkomma1 = 1;
  int Nachkomma2 = 1;  
  static float old_U1=0.0;
  String P_unit1,P_unit2;
  int lbg_draw_val_limited;
pU1=0;
pU2=0;

//lcd.setCursor(0,2);
//lcd.print(Koppler1);
//lcd.setCursor(16,2);
//lcd.print(Koppler2);

resetCursor();

for(int q = 0; q < 17; q++)
{
  if (rawU1 > linM0[q][1]) pU1 = q;
  if (rawU2 > linM0[q][1]) pU2 = q;
} 

if (pU1 > 15) U1 = linM0[16][0];
  else if (pU1 < 1) U1 = linM0[0][0];
  else U1 = ((rawU1 - linM0[pU1][1]) / (linM0[pU1+1][1] - linM0[pU1][1])) * (linM0[pU1+1][0] - linM0[pU1][0]) + linM0[pU1][0];

if (pU2 > 15) U2 = linM0[16][0];
  else if (pU2 < 1) U2 = linM0[0][0];
  else U2 = ((rawU2 - linM0[pU2][1]) / (linM0[pU2+1][1] - linM0[pU2][1])) * (linM0[pU2+1][0] - linM0[pU2][0]) + linM0[pU2][0];

U1=U1 + Koppler1;
U2=U2 + Koppler2;

//***************TESTMODE

//U1=posi*0.1;

//***********************

P1mW=pow(10,U1/10.0);
P2mW=pow(10,U2/10.0);

MB_Wahl1(Nachkomma1, Divisor1, P_unit1);
MB_Wahl2(Nachkomma2, Divisor2, P_unit2);


P1mW_out=P1mW/Divisor1;
P2mW_out=P2mW/Divisor2;
VSWR=(pow(10,((U2-U1)/10.0))+1)/(pow(10,((U2-U1)/10.0))-0.999999);    
dtostrf(U1,4,1,outstr4);
lcd.setCursor(0,0);
lcd.print("     ");
lcd.setCursor(0,0);
lcd.print(outstr4);
lcd.setCursor(5,0);
lcd.print("dBm");



dtostrf(U2,4,1,outstr4);
lcd.setCursor(12,0);
lcd.print("     ");
lcd.setCursor(12,0);
lcd.print(outstr4);
lcd.setCursor(17,0);
lcd.print("dBm");

if (abs(VSWR)>99.9) VSWR=99.9;
dtostrf(abs(VSWR),4,2,outstr4);
lcd.setCursor(9,1);
lcd.print("SWR");
lcd.setCursor(6,2);
lcd.print("       ");
lcd.setCursor(8,2);
lcd.print(outstr4);


dtostrf(P1mW_out,5,Nachkomma1,outstr4);
lcd.setCursor(0,1);//vorher (1,1)
lcd.print("      ");
lcd.setCursor(1,1);
lcd.print(outstr4);
lcd.setCursor(6,1);
lcd.print(P_unit1);

dtostrf(P2mW_out,5,Nachkomma2,outstr4);
lcd.setCursor(13,1);
lcd.print("      ");
lcd.setCursor(13,1);
lcd.print(outstr4);
lcd.setCursor(18,1);
lcd.print(P_unit2);


dtostrf(Koppler1,4,1,outstr4);
lcd.setCursor(0,2);//vorher (1,1)
lcd.print("      ");
lcd.setCursor(0,2);
lcd.print(outstr4);

dtostrf(Koppler2,4,1,outstr4);
lcd.setCursor(13,2);//vorher (1,1)
lcd.print("      ");
lcd.setCursor(13,2);
lcd.print(outstr4);

//Den zu zeichnenden Bargraph mit dem jeweiligen Messbereich skalieren und auf 999 begrenzen
lbg_draw_val_limited=int(1000*P1mW/(0.0008*pow(10,MB1-1)));
if (lbg_draw_val_limited > 999) lbg_draw_val_limited=999;
lbgPWR.drawValue(lbg_draw_val_limited,1000);
//lbgPWR.drawValue(500,1000);

//lbgU1.drawValue( int(U1), 1024);
//lbgU2.drawValue( int(U2), 1024);


lcd.setCursor(10,0);
lcd.print(" ");
lcd.setCursor(10,0);

lcd.print(item_pos);

edit = true;
resetCursor();

 /*if (fired && !edit && !change_value) {
  edit=true;  // edit mode!!
  fired = false;
  lcd.cursor();
  lcd.blink();

  resetCursor();
}
*/




}


void MB_Wahl1(int &Nachkomma1, float &Divisor1,String &P_unit1){ // Taktik: z.B. P  <0.5mW für 2 Sek. -> Umschalten in nächst kleineren MB,  P > 8mW für 1Sek umschalten in   
  // MB5 = 1.00mW - ; MB6 = 
static boolean was_inside = true;
static unsigned long switch_MB = millis();

switch (MB1)
{

case 1:
  if ((P1mW < 0.0008) && (P1mW > 0.00005))  switch_MB = millis();
  if (P1mW > 0.0008) {
      if ((millis() - switch_MB) > 1 * 1000UL) {
    MB1=2;     
  }
  }
  if (P1mW < 0.00005){
      if ((millis() - switch_MB) > 1 * 1000UL) {
      MB1=1;     
    }
  }
break;


  case 2:
  if ((P1mW < 0.008) && (P1mW > 0.0005))  switch_MB = millis();
  if (P1mW > 0.008) {
      if ((millis() - switch_MB) > 1 * 1000UL) {
    MB1=3;     
  }
  }
  if (P1mW < 0.0005){
      if ((millis() - switch_MB) > 1 * 1000UL) {
      MB1=1;     
    }
  }
break;

  
  
  case 3:
  if ((P1mW < 0.08) && (P1mW > 0.005))  switch_MB = millis();
  if (P1mW > 0.08) {
      if ((millis() - switch_MB) > 1 * 1000UL) {
    MB1=4;     
  }
  }
  if (P1mW < 0.005){
      if ((millis() - switch_MB) > 1 * 1000UL) {
      MB1=2;     
    }
  }
break;
  
  
  
  case 4:
  if ((P1mW < 0.8) && (P1mW > 0.05))  switch_MB = millis();
  if (P1mW > 0.8) {
      if ((millis() - switch_MB) > 1 * 1000UL) {
    MB1=5;     
  }
  }
  if (P1mW < 0.05){
      if ((millis() - switch_MB) > 1 * 1000UL) {
      MB1=3;     
    }
  }
break;



  case 5:
  if ((P1mW < 8.0) && (P1mW > 0.5))  switch_MB = millis();
  if (P1mW > 8.0) {
      if ((millis() - switch_MB) > 1 * 1000UL) {
    MB1=6;     
  }
  }
  if (P1mW < 0.5){
      if ((millis() - switch_MB) > 1 * 1000UL) {
      MB1=4;     
    }
  }
break;
  
  case 6:
  if ((P1mW < 80.0) && (P1mW > 5.0))  switch_MB = millis();
  if (P1mW > 80.0) {
      if ((millis() - switch_MB) > 1 * 1000UL) {
    MB1=7;     
  }
  }
  if (P1mW < 5.0){
      if ((millis() - switch_MB) > 1 * 1000UL) {
      MB1=5;     
    }
  }

  break;

  case 7:
  if ((P1mW < 800.0) && (P1mW > 50.0))  switch_MB = millis();
  if (P1mW > 800.0) {
      if ((millis() - switch_MB) > 1 * 1000UL) {
    MB1=8;     
  }
  }
  if (P1mW < 50.0){
      if ((millis() - switch_MB) > 1 * 1000UL) {
      MB1=6;     
    }
  }
  break;

case 8:
  if ((P1mW < 8000.0) && (P1mW > 500.0))  switch_MB = millis();
  if (P1mW > 8000.0) {
      if ((millis() - switch_MB) > 1 * 1000UL) {
    MB1=9;     
  }
  }
  if (P1mW < 500.0){
      if ((millis() - switch_MB) > 1 * 1000UL) {
      MB1=7;     
    }
  }

  break;


case 9:
 if ((P1mW < 80000.0) && (P1mW > 5000.0))  switch_MB = millis();
  if (P1mW > 80000.0) {
      if ((millis() - switch_MB) > 1 * 1000UL) {
    MB1=9;     
  }
  }
  if (P1mW < 5000.0){
      if ((millis() - switch_MB) > 1 * 1000UL) {
      MB1=8;     
    }
  }

  break;

}

if (MB1==1) {
  Nachkomma1=1;
  Divisor1=0.000001;
  P_unit1="nW";
}
if (MB1==2) {
  Nachkomma1=3;
  Divisor1=0.001;
  P_unit1="uW";
}
if (MB1==3) {
  Nachkomma1=2;
  Divisor1=0.001;
  P_unit1="uW";
}
if (MB1==4) {
  Nachkomma1=1;
  Divisor1=0.001;
  P_unit1="uW";
}
if (MB1==5) {
  Nachkomma1=3;  
  Divisor1=1.0;
  P_unit1="mW";
}
if (MB1==6) {
  Nachkomma1=2;
  Divisor1=1.0;
  P_unit1="mW";  
}
if (MB1==7) {
  Nachkomma1=1;
  Divisor1=1.0;
  P_unit1="mW";  
}
if (MB1==8) {
  Nachkomma1=3;
  Divisor1=1000.0;
  P_unit1="W ";  
}
if (MB1==9) {
  Nachkomma1=2;
  Divisor1=1000.0;
  P_unit1="W ";  
}



}

void MB_Wahl2(int &Nachkomma2, float &Divisor2, String &P_unit2){ // Taktik: <2.0mW für 2 Sek. -> Umschalten in nächst kleineren MB, > 8 für 1Sek umschalten in   
  // MB5 = 1.00mW - ; MB6 = 
static boolean was_inside = true;
static unsigned long switch_MB = millis();

switch (MB2)
{
case 1:
  if ((P2mW < 0.0008) && (P2mW > 0.00005))  switch_MB = millis();
  if (P2mW > 0.0008) {
      if ((millis() - switch_MB) > 1 * 1000UL) {
    MB2=2;     
  }
  }
  if (P2mW < 0.00005){
      if ((millis() - switch_MB) > 1 * 1000UL) {
      MB2=1;     
    }
  }
break;


  case 2:
  if ((P2mW < 0.008) && (P2mW > 0.0005))  switch_MB = millis();
  if (P2mW > 0.008) {
      if ((millis() - switch_MB) > 1 * 1000UL) {
    MB2=3;     
  }
  }
  if (P2mW < 0.0005){
      if ((millis() - switch_MB) > 1 * 1000UL) {
      MB2=1;     
    }
  }
break;

  
  
  case 3:
  if ((P2mW < 0.08) && (P2mW > 0.005))  switch_MB = millis();
  if (P2mW > 0.08) {
      if ((millis() - switch_MB) > 1 * 1000UL) {
    MB2=4;     
  }
  }
  if (P2mW < 0.005){
      if ((millis() - switch_MB) > 1 * 1000UL) {
      MB2=2;     
    }
  }
break;

  
  
  case 4:
  if ((P2mW < 0.8) && (P2mW > 0.05))  switch_MB = millis();
  if (P2mW > 0.8) {
      if ((millis() - switch_MB) > 1 * 1000UL) {
    MB2=5;     
  }
  }
  if (P2mW < 0.05){
      if ((millis() - switch_MB) > 1 * 1000UL) {
      MB2=3;     
    }
  }
break;



  case 5:
  if ((P2mW < 8.0) && (P2mW > 0.5))  switch_MB = millis();
  if (P2mW > 8.0) {
      if ((millis() - switch_MB) > 1 * 1000UL) {
    MB2=6;     
  }
  }
  if (P2mW < 0.5){
      if ((millis() - switch_MB) > 1 * 1000UL) {
      MB2=4;     
    }
  }
break;
  
  case 6:
  if ((P2mW < 80.0) && (P2mW > 5.0))  switch_MB = millis();
  if (P2mW > 80.0) {
      if ((millis() - switch_MB) > 1 * 1000UL) {
    MB2=7;     
  }
  }
  if (P2mW < 5.0){
      if ((millis() - switch_MB) > 1 * 1000UL) {
      MB2=5;     
    }
  }

  break;

  case 7:
  if ((P2mW < 800.0) && (P2mW > 50.0))  switch_MB = millis();
  if (P2mW > 800.0) {
      if ((millis() - switch_MB) > 1 * 1000UL) {
    MB2=8;     
  }
  }
  if (P2mW < 50.0){
      if ((millis() - switch_MB) > 1 * 1000UL) {
      MB2=6;     
    }
  }
  break;

case 8:
  if ((P2mW < 8000.0) && (P2mW > 500.0))  switch_MB = millis();
  if (P2mW > 8000.0) {
      if ((millis() - switch_MB) > 1 * 1000UL) {
    MB2=9;     
  }
  }
  if (P2mW < 500.0){
      if ((millis() - switch_MB) > 1 * 1000UL) {
      MB2=7;     
    }
  }

  break;


case 9:
 if ((P2mW < 80000.0) && (P2mW > 5000.0))  switch_MB = millis();
  if (P2mW > 80000.0) {
      if ((millis() - switch_MB) > 1 * 1000UL) {
    MB2=9;     
  }
  }
  if (P2mW < 5000.0){
      if ((millis() - switch_MB) > 1 * 1000UL) {
      MB2=8;     
    }
  }

  break;

}


if (MB2==1) {
  Nachkomma2=1;
  Divisor2=0.000001;
  P_unit2="nW";
}
if (MB2==2) {
  Nachkomma2=3;
  Divisor2=0.001;
  P_unit2="uW";
}
if (MB2==3) {
  Nachkomma2=2;
  Divisor2=0.001;
  P_unit2="uW";
}

if (MB2==4) {
  Nachkomma2=1;
  Divisor2=0.001;
  P_unit2="uW";
}
if (MB2==5) {
  Nachkomma2=3;  
  Divisor2=1.0;
  P_unit2="mW";
}
if (MB2==6) {
  Nachkomma2=2;
  Divisor2=1.0;
  P_unit2="mW";  
}
if (MB2==7) {
  Nachkomma2=1;
  Divisor2=1.0;
  P_unit2="mW";  
}
if (MB2==8) {
  Nachkomma2=3;
  Divisor2=1000.0;
  P_unit2="W ";  
}
if (MB2==9) {
  Nachkomma2=2;
  Divisor2=1000.0;
  P_unit2="W ";  
}


}

void resetCursor(){  //um den Cursor im Edit-Mode nach Anzeige des Status wieder an die richtige Stelle zu setzen!
  if (item_pos == 1) lcd.setCursor(3,2);
  if (item_pos == 2) lcd.setCursor(12,2);
  if (item_pos == 3) lcd.setCursor(18,2);
        
}

void write_raw_seriell(){
Serial.println(rawU1);
}
