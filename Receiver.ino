//================================              INSA Rennes                          ================================
//================================                                                   ================================
//================================ Département Systèmes et Réseaux de Communication  ================================
//================================                                                   ================================
//================================            Projet électronique                    ================================
//================================                                                   ================================
//================================        Un drone au bout des doigts                ================================
//================================                                                   ================================
//================================            Code récepteur drone                   ================================
//
/**
* An Mirf example which copies back the data it receives.
*
*envoie un tableau de taille 12 = 4*3digits
*à adapter avec Reception_message_triple_sans_reconnaissance
*            et Emission_donnees_gyro_dmp_triplet
*
* Pins:
* Hardware SPI:
* MISO -> 12
* MOSI -> 11
* SCK -> 13
*
* Configurable:
* CE -> 8
* CSN -> 7
*
*/

//////////////////////CONFIGURATION///////////////////////////////
#define chanel_number 8  //set the number of chanels
#define default_servo_value 1500  //set the default servo value
#define PPM_FrLen 20000  //set the PPM frame length in microseconds (1ms = 1000µs)
#define PPM_PulseLen 500  //set the pulse length
#define onState 0  //set polarity of the pulses: 1 is positive, 0 is negative
#define sigPin 3  //set PPM signal output pin on the arduino
//////////////////////////////////////////////////////////////////
int ppm[chanel_number];

#include <SPI.h>
#include <Mirf.h>
#include <nRF24L01.h>
#include <MirfHardwareSpiDriver.h>
#include <string.h>
boolean b = 0;
byte thisChar[12];
char  data[12];
int iyaw ;
int myaw;
int ipitch ;
int mpitch;
int iroll ;
int mroll;
int rroll,rpitch,ryaw;
int ithr,mthr,rthr;

//FAILSAFE 
int tmpthr = 0;
int cptsafe = 0;
//END FAILSAFE

void setup(){

  for(int i=0; i<chanel_number; i++){
    ppm[i]= default_servo_value;
  }

  pinMode(sigPin, OUTPUT);
  digitalWrite(sigPin, !onState);  //set the PPM signal pin to the default state (off)
  
  cli();
  TCCR1A = 0; // set entire TCCR1 register to 0
  TCCR1B = 0;
  
  OCR1A = 100;  // compare match register, change this
  TCCR1B |= (1 << WGM12);  // turn on CTC mode
  TCCR1B |= (1 << CS11);  // 8 prescaler: 0,5 microseconds at 16mhz
  TIMSK1 |= (1 << OCIE1A); // enable timer compare interrupt
  sei();
  
  Serial.begin(9600);
  
 Mirf.spi = &MirfHardwareSpi;
    
  Mirf.init();
  
  /*
   * Configure receiving address.
   */
   
  Mirf.setRADDR((byte *)"serv1");
  
  /*
   * Set the payload length to sizeof(unsigned long) the
   * return type of millis().
   *
   * NB: payload on client and server must be the same.
   */
   
  Mirf.payload = 12;
   char data[Mirf.payload];
  /*
   * Write channel and payload config then power up receiver.
   */
   
  Mirf.config();

  
  Serial.println("Listening..."); 
}

void loop(){
  
   data[Mirf.payload];
   
  if(!Mirf.isSending() && Mirf.dataReady()){
    
    Mirf.getData((byte *) data);
    
String yaw ="000";
yaw.setCharAt(0,data[0]);
yaw.setCharAt(1,data[1]);
yaw.setCharAt(2,data[2]);
iyaw = yaw.toInt();
myaw = map(iyaw,100,460,-180,180);

String pitch ="000";
pitch.setCharAt(0,data[3]);
pitch.setCharAt(1,data[4]);
pitch.setCharAt(2,data[5]);
ipitch = pitch.toInt();
mpitch = map(ipitch,100,280,-90,90);

String roll ="000";
roll.setCharAt(0,data[6]);
roll.setCharAt(1,data[7]);
roll.setCharAt(2,data[8]);
iroll = roll.toInt();
mroll = map(iroll,100,280,-90,90);

String thr ="000";
thr.setCharAt(0,data[9]);
thr.setCharAt(1,data[10]);
thr.setCharAt(2,data[11]);
ithr = thr.toInt();
mthr = map(ithr,100,280,-90,90);


/*
Serial.print("ypr");
   Serial.print("\t");
   Serial.print(myaw);
   Serial.print("\t");
   Serial.print(mpitch); 
   Serial.print("\t");
   Serial.print(mroll);
   Serial.print("\t");
   Serial.print(mthr);
   Serial.println("");
   */
  }

 else{mthr = 90; myaw = 0; mpitch = 0; mroll =0;}
mpitch = -mpitch;
ryaw = map(myaw,-180,180,1000,2000);
rpitch = map(mpitch,-90,90,1000,2000);
rroll = map(mroll,-90,90,1000,2000);
rthr = map(mthr,90,-90,1000,2000);

Serial.print("ypr");
   Serial.print("\t");
   Serial.print(ryaw);
   Serial.print("\t");
   Serial.print(rpitch); 
   Serial.print("\t");
   Serial.print(rroll);
   Serial.print("\t");
   Serial.print(rthr);
   Serial.println("");

ppm[4] = 2000;
ppm[2] = rthr;
ppm[1] = rpitch;
ppm[3] = 1500;
ppm[0] = rroll;


}

ISR(TIMER1_COMPA_vect){  //leave this alone
  static boolean state = true;
  
  TCNT1 = 0;
  
  if(state) {  //start pulse
    digitalWrite(sigPin, onState);
    OCR1A = PPM_PulseLen * 2;
    state = false;
  }
  else{  //end pulse and calculate when to start the next pulse
    static byte cur_chan_numb;
    static unsigned int calc_rest;
  
    digitalWrite(sigPin, !onState);
    state = true;

    if(cur_chan_numb >= chanel_number){
      cur_chan_numb = 0;
      calc_rest = calc_rest + PPM_PulseLen;// 
      OCR1A = (PPM_FrLen - calc_rest) * 2;
      calc_rest = 0;
    }
    else{
      OCR1A = (ppm[cur_chan_numb] - PPM_PulseLen) * 2;
      calc_rest = calc_rest + ppm[cur_chan_numb];
      cur_chan_numb++;
    }     
  }
}

