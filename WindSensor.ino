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
int memSlotsLocalMask[] = {8,9,10,11};
int memSlotsGateway[] = {12,13,14,15};
int memSlotRemotePort[] = {20,21};
int memSlotsMacAddress[] = {101, 102, 103, 104, 105, 106};

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
IPAddress localMask(EEPROM.read(memSlotsLocalMask[0]), EEPROM.read(memSlotsLocalMask[1]), EEPROM.read(memSlotsLocalMask[2]), EEPROM.read(memSlotsLocalMask[3]));
IPAddress localGateway(EEPROM.read(memSlotsGateway[0]), EEPROM.read(memSlotsGateway[1]), EEPROM.read(memSlotsGateway[2]), EEPROM.read(memSlotsGateway[3]));
byte mac[] = {EEPROM.read(memSlotsMacAddress[0]), EEPROM.read(memSlotsMacAddress[1]), EEPROM.read(memSlotsMacAddress[2]), EEPROM.read(memSlotsMacAddress[3]), EEPROM.read(memSlotsMacAddress[4]), EEPROM.read(memSlotsMacAddress[5])};
unsigned int localPort = 8888;

IPAddress remoteIp(EEPROM.read(4),EEPROM.read(5),EEPROM.read(6),EEPROM.read(7));
unsigned int remotePort;

EthernetUDP Udp;

String in_chars = "";

void setup() {

  Serial.begin(9600);

  LastValue = 0;

  IsSampleRequired = false;

  TimerCount = 0;
  Rotations = 0; // Set Rotations to 0 ready for calculations

  pinMode(WindSensorPin, INPUT);
  pinMode(PowerLed, OUTPUT);
  pinMode(ActLed, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(WindSensorPin), isr_rotation, FALLING);

  Serial.println("Absurd Solutions 2021, Windsensor to OSC");
  Serial.println("Rotations\tM/S\tDirection\tStrength");

  // Setup the timer interupt
  Timer1.initialize(500000);// Timer interrupt every 2.5 seconds
  Timer1.attachInterrupt(isr_timer);

  restartEthernet();

  digitalWrite(PowerLed, HIGH);
  remotePort = readUnsignedIntFromEEPROM(memSlotRemotePort[0]);

  //setMacAddress(); // Use once for setting MAC address
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
      writeUnsignedIntIntoEEPROM(memSlotRemotePort[0], remotePort);
      Serial.println("remote port changed");
      restartEthernet();
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

      IPAddress lIp(ip1,ip2,ip3,ip4); //init new ip address
      localIp = lIp;
      EEPROM.write(memSlotsLocalIp[0], ip1);
      EEPROM.write(memSlotsLocalIp[1], ip2);
      EEPROM.write(memSlotsLocalIp[2], ip3);
      EEPROM.write(memSlotsLocalIp[3], ip4);
      Serial.println(EEPROM.read(0));
      Serial.println(EEPROM.read(1));
      Serial.println(EEPROM.read(2));
      Serial.println(EEPROM.read(3));

      restartEthernet();
      Serial.println("local ip changed");
    }

    else if (in_chars.indexOf("localMask ") == 0){
      String mask = "";
      int mask1;
      int mask2;
      int mask3;
      int mask4;

      mask = in_chars.substring(10, in_chars.length()); // split ip to octents and put in own var.
      mask1 = mask.substring(0, mask.indexOf(".")).toInt();
      mask.remove(0, mask.indexOf(".")+1);
      mask2 = mask.substring(0, mask.indexOf(".")).toInt();
      mask.remove(0, mask.indexOf(".")+1);
      mask3 = mask.substring(0, mask.indexOf(".")).toInt();
      mask.remove(0, mask.indexOf(".")+1);
      mask4 = mask.toInt();

      IPAddress localMask(mask1,mask2,mask3,mask4); //init new ip address
      EEPROM.write(memSlotsLocalMask[0], mask1);
      EEPROM.write(memSlotsLocalMask[1], mask2);
      EEPROM.write(memSlotsLocalMask[2], mask3);
      EEPROM.write(memSlotsLocalMask[3], mask4);

      restartEthernet();
      Serial.println("local mask changed");
    }

    else if (in_chars.indexOf("localGateway ") == 0){
      String gw = "";
      int gw1;
      int gw2;
      int gw3;
      int gw4;

      gw = in_chars.substring(13, in_chars.length()); // split ip to octents and put in own var.
      gw1 = gw.substring(0, gw.indexOf(".")).toInt();
      gw.remove(0, gw.indexOf(".")+1);
      gw2 = gw.substring(0, gw.indexOf(".")).toInt();
      gw.remove(0, gw.indexOf(".")+1);
      gw3 = gw.substring(0, gw.indexOf(".")).toInt();
      gw.remove(0, gw.indexOf(".")+1);
      gw4 = gw.toInt();

      IPAddress localMask(gw1,gw2,gw3,gw4); //init new ip address
      EEPROM.write(memSlotsGateway[0], gw1);
      EEPROM.write(memSlotsGateway[1], gw2);
      EEPROM.write(memSlotsGateway[2], gw3);
      EEPROM.write(memSlotsGateway[3], gw4);

      restartEthernet();
      Serial.println("local Gateway changed");
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
      remoteIp = rmiP;
      restartEthernet();
      Serial.println("remote ip changed");
    }

    else if(in_chars.indexOf("help") == 0){
      Serial.println("=======================  HELP =======================");
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

    else if(in_chars.indexOf("show config") == 0){
      Serial.println("======================= CURRENT CONFIG ==========================");
      Serial.print("Local MAC Address: ");
      for (int i=0; i < 6; i++){
        Serial.print(mac[i], HEX);
        if (i < 5) Serial.print(":");
      }
      Serial.println("");

      Serial.print("Local IP Address: ");
      for (int i=0; i<4; i++){
        Serial.print(localIp[i]);
        if (i < 3) Serial.print(".");
      }
      Serial.println("");

      Serial.print("Local Subnet Mask: ");
      for (int i=0; i<4; i++){
        Serial.print(localMask[i]);
        if (i < 3) Serial.print(".");
      }
      Serial.println("");

      Serial.print("Local Gateway: ");
      for (int i=0; i<4; i++){
        Serial.print(localGateway[i]);
        if (i < 3) Serial.print(".");
      }
      Serial.println("");

      Serial.print("Local UDP Port: ");
      Serial.println(localPort);
      Serial.println("");

      Serial.print("Remote IP Address: ");
      for (int i=0; i<4; i++){
        Serial.print(remoteIp[i]);
        if (i < 3) Serial.print(".");
      }
      Serial.println("");

      Serial.print("Remote UDP Port: ");
      Serial.println(remotePort);     

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

void restartEthernet(){
  Udp.stop();
  Ethernet.begin(mac, localIp);
  Ethernet.setSubnetMask(localMask);
  Ethernet.setGatewayIP(localGateway);
  Udp.begin(localPort);
}

void setMacAddress(){
  byte setMacAddressArray[] = {0xA8,0x61,0x0A,0xAE,0x67,0xF1};

  EEPROM.write(memSlotsMacAddress[0], setMacAddressArray[0]);
  EEPROM.write(memSlotsMacAddress[1], setMacAddressArray[1]);
  EEPROM.write(memSlotsMacAddress[2], setMacAddressArray[2]);
  EEPROM.write(memSlotsMacAddress[3], setMacAddressArray[3]);
  EEPROM.write(memSlotsMacAddress[4], setMacAddressArray[4]);
  EEPROM.write(memSlotsMacAddress[5], setMacAddressArray[5]);
}

void writeUnsignedIntIntoEEPROM(int address, unsigned int number)
{ 
  EEPROM.write(address, number >> 8);
  EEPROM.write(address + 1, number & 0xFF);
}
unsigned int readUnsignedIntFromEEPROM(int address)
{
  return (EEPROM.read(address) << 8) + EEPROM.read(address + 1);
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
