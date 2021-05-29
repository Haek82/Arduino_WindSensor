#include <TimerOne.h> // Timer Interrupt set to 2 second for read sensors
#include <math.h>
#include <Ethernet.h>
#include <EthernetUdp.h>
#include <OSCMessage.h>
#include <EEPROM.h>

// ###################################### EEPROM ##############################################################
// Memory slots in use:
int memSlotsLocalIp[] = {0,1,2,3};
int memSlotsRemoteIp[] = {4,5,6,7}; 

// ################################### WIND VANE VARIABLES ####################################################
#define WindSensorPin (2) // The pin location of the anemometer sensor
#define WindVanePin (A4) // The pin the wind vane sensor is connected to
#define VaneOffset 0; // define the anemometer offset from magnetic north


#define PowerLed (7)
#define ActLed (6)

int VaneValue; // raw analog value from wind vane
int Direction; // translated 0 - 360 direction
int CalDirection; // converted value with offset applied
int LastValue; // last direction value

volatile bool IsSampleRequired; // this is set true every 2.5s. Get wind speed
volatile unsigned int TimerCount; // used to determine 2.5sec timer count
volatile unsigned long Rotations; // cup rotation counter used in interrupt routine
volatile unsigned long RotationsDebug; // cup rotation counter used for debugging console
volatile unsigned long ContactBounceTime; // Timer to avoid contact bounce in isr

float WindSpeed; // speed miles per hour

bool debug = true;

// ################################### ETHERNET SHIELD VARIABLES ################################################

IPAddress localIp(EEPROM.read(0),EEPROM.read(1),EEPROM.read(2),EEPROM.read(3));
//IPAddress localIp(192, 168, 1 ,210);
byte mac[] = {0x90, 0xA2, 0xDA, 0x10, 0x3D, 0xEE};
unsigned int localPort = 8888;

IPAddress remoteIp(EEPROM.read(4),EEPROM.read(5),EEPROM.read(6),EEPROM.read(7));
//IPAddress remoteIp(192, 168, 1 ,211);
unsigned int remotePort = 8889;

EthernetUDP Udp;

String in_chars = "";

void setup() {

  Serial.begin(9600);

  LastValue = 0;

  IsSampleRequired = false;

  TimerCount = 0;
  Rotations = 0; // Set Rotations to 0 ready for calculations
  
  // Serial.println(EEPROM.read(0));
  // Serial.println(EEPROM.read(1));
  // Serial.println(EEPROM.read(2));
  // Serial.println(EEPROM.read(3));

  // Serial.println(EEPROM.read(4));
  // Serial.println(EEPROM.read(5));
  // Serial.println(EEPROM.read(6));
  // Serial.println(EEPROM.read(7));

  pinMode(WindSensorPin, INPUT);
  pinMode(PowerLed, OUTPUT);
  pinMode(ActLed, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(WindSensorPin), isr_rotation, FALLING);

  Serial.println("Davis Anemometer Test");
  Serial.println("Rotations\tM/S\tDirection\tStrength");

  // Setup the timer interupt
  Timer1.initialize(500000);// Timer interrupt every 2.5 seconds
  Timer1.attachInterrupt(isr_timer);

  Ethernet.begin(mac, localIp);
  Udp.begin(localPort);

  digitalWrite(PowerLed, HIGH);
}



void loop() {

  char in_char = ' ';
  while (Serial.available()){
    in_char = Serial.read();
    if (int(in_char)!=-1){
      in_chars+=in_char;
    }
  }
  if (in_char=='\n'){
    Serial.print(in_chars);

    if (in_chars.indexOf("remotePort ") == 0){
      remotePort = in_chars.substring(11, in_chars.length()).toInt();
      Serial.println(remotePort);
    }
   else if (in_chars.indexOf("localIp ") == 0){
      String ip = "";
      int ip1;
      int ip2;
      int ip3;
      int ip4;

      ip = in_chars.substring(8, in_chars.length()); // split ip to octents and put in own var.
      ip1 = ip.substring(0, ip.indexOf(".")).toInt();
      ip.remove(0, ip.indexOf(".")+1);
      ip2 = ip.substring(0, ip.indexOf(".")).toInt();
      ip.remove(0, ip.indexOf(".")+1);
      ip3 = ip.substring(0, ip.indexOf(".")).toInt();
      ip.remove(0, ip.indexOf(".")+1);
      ip4 = ip.toInt();

      IPAddress localIp(ip1,ip2,ip3,ip4); //init new ip address
      EEPROM.write(memSlotsLocalIp[0], ip1);
      EEPROM.write(memSlotsLocalIp[1], ip2);
      EEPROM.write(memSlotsLocalIp[2], ip3);
      EEPROM.write(memSlotsLocalIp[3], ip4);
      Serial.println(EEPROM.read(0));
      Serial.println(EEPROM.read(1));
      Serial.println(EEPROM.read(2));
      Serial.println(EEPROM.read(3));

      Udp.stop();
      Ethernet.begin(mac, localIp);
      Udp.begin(localPort);
      Serial.println("local ip changed");
    }

    else if (in_chars.indexOf("remoteIp ") == 0){
      String ip = "";
      int ip1;
      int ip2;
      int ip3;
      int ip4;

      ip = in_chars.substring(8, in_chars.length()); // split ip to octents and put in own var.
      ip1 = ip.substring(0, ip.indexOf(".")).toInt();
      ip.remove(0, ip.indexOf(".")+1);
      ip2 = ip.substring(0, ip.indexOf(".")).toInt();
      ip.remove(0, ip.indexOf(".")+1);
      ip3 = ip.substring(0, ip.indexOf(".")).toInt();
      ip.remove(0, ip.indexOf(".")+1);
      ip4 = ip.toInt();

      IPAddress rmiP (ip1,ip2,ip3,ip4); //init new ip address
      EEPROM.write(memSlotsRemoteIp[0], ip1);
      EEPROM.write(memSlotsRemoteIp[1], ip2);
      EEPROM.write(memSlotsRemoteIp[2], ip3);
      EEPROM.write(memSlotsRemoteIp[3], ip4);
      Serial.println(EEPROM.read(4));
      Serial.println(EEPROM.read(5));
      Serial.println(EEPROM.read(6));
      Serial.println(EEPROM.read(7));
      remoteIp = rmiP;
      Udp.stop();
      Ethernet.begin(mac, localIp);
      Udp.begin(localPort);
      Serial.println("remote ip changed");
    }
    

    else if(in_chars.indexOf("help") == 0){
      Serial.println("---------------- HELP ------------------");
      Serial.println("remotePort <port nr> \t remotePort command will change the port OSC commands are sent to.");
      Serial.println("remoteIp <ip address> \t remoteIp changes the Ip address OSC commands are sent to.");
      Serial.println("localIp <ip address> \t localIp will change the Arduino Ip address");
    }

    else if(in_chars.indexOf("debug ") == 0){
      String debugState = "";

      debugState = in_chars.substring(6, in_chars.length());
      if (debugState.indexOf("true") == 0){
        debug = true;
      }
      else if (debugState.indexOf("false") == 0){
        debug = false;
      }

    }

    in_chars = "";
  }


  OSCMessage msg("/wind");

  getWindDirection();

  // Only update the display if change greater than 5 degrees.
  if(abs(CalDirection - LastValue) > 1) {
    LastValue = CalDirection;
  }

  if(IsSampleRequired) {
    // convert to mp/h using the formula V=P(2.25/T)
    // V = P(2.25/2.5) = P * 0.9
    WindSpeed = Rotations * 1.125;
    RotationsDebug = Rotations;
    Rotations = 0; // Reset count for next sample

  IsSampleRequired = false;
  
  if (debug){
    Serial.print(RotationsDebug); Serial.print("\t\t");
    Serial.print(getMS(WindSpeed)); Serial.print("\t");
    Serial.print(CalDirection);
    getHeading(CalDirection); Serial.print("\t\t");
    getWindStrength(WindSpeed);
  }

  msg.add(CalDirection);
  msg.add(getMS(WindSpeed));

  Udp.beginPacket(remoteIp, remotePort);
  msg.send(Udp);
  Udp.endPacket();
  msg.empty();
  }
}

// isr handler for timer interrupt
void isr_timer() {

  TimerCount++;
  digitalWrite(ActLed, LOW);

  if(TimerCount == 4) {
    digitalWrite(ActLed, HIGH);
    IsSampleRequired = true;
    TimerCount = 0;
  }
}

// This is the function that the interrupt calls to increment the rotation count
void isr_rotation() {

  if((millis() - ContactBounceTime) > 15 ) { // debounce the switch contact.
    Rotations++;
    ContactBounceTime = millis();
  }
}

// Convert MPH to Knots
float getKnots(float speed) {
  return speed * 0.868976;
}

float getMS(float speed){
  return speed * 0.47704;
}

// Get Wind Direction
void getWindDirection() {

  VaneValue = analogRead(WindVanePin);
  Direction = map(VaneValue, 0, 1023, 0, 359);
  CalDirection = Direction + VaneOffset;

  if(CalDirection > 360)
    CalDirection = CalDirection - 360;

  if(CalDirection < 0) 
    CalDirection = CalDirection + 360;

}

// Converts compass direction to heading
void getHeading(int direction) {

  if(direction < 22)
    Serial.print(" N");
  else if (direction < 67)
    Serial.print(" NE");
  else if (direction < 112)
    Serial.print(" E");
  else if (direction < 157)
    Serial.print(" SE");
  else if (direction < 212)
    Serial.print(" S");
  else if (direction < 247)
    Serial.print(" SW");
  else if (direction < 292)
    Serial.print(" W");
  else if (direction < 337)
    Serial.print(" NW");
  else
    Serial.print(" N");
}

// converts wind speed to wind strength
void getWindStrength(float speed) {

  if(speed < 2)
    Serial.println("Calm");
  else if(speed >= 2 && speed < 4)
    Serial.println("Light Air");
  else if(speed >= 4 && speed < 8)
    Serial.println("Light Breeze");
  else if(speed >= 8 && speed < 13)
    Serial.println("Gentle Breeze");
  else if(speed >= 13 && speed < 18)
    Serial.println("Moderate Breeze");
  else if(speed >= 18 && speed < 25)
    Serial.println("Fresh Breeze");
  else if(speed >= 25 && speed < 31)
    Serial.println("Strong Breeze");
  else if(speed >= 31 && speed < 39)
    Serial.println("Near Gale");
  else
    Serial.println("RUN");
}
