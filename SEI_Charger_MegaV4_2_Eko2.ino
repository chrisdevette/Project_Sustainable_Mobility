//datum 3 okt 2014
// update vt.o.v. V4_1: Bij werkelijk laden is dit bij contact "aan" op het scherm te zien.
//Dit programma is tbv Ekoletric 2 die is voorzien van SEI V4:
//Het schakel Up en down signaal is in deze software versie verwerkt
//Hierdoor is fout van drivesignaal tijdens Up actie opgelost
// op basis van charger_uno_final aangepast.
// Veel getest met defaults.h zoals op:
// https://www.sparkfun.com/products/10039#comment-515cad97757b7f4a55000000
// beschreven staat.
// Dit mocht niet baten. Het enige wat hielp is om in deze file
// const int CAN_CHIP_SELECT      = 53; //10 te wijzigen
// aanpassing shield Mega CAN bus shield
//51 11 (SPI MOSI)
//50 12 (SPI MISO)
//52 13 (SPI SCK)
//53 10 (SPI CS)


#include <SPI.h>
#include <MCP2515.h>
#include <Timer.h>
#include <SoftwareSerial.h>

long previousMillis = 0;
long interval = 50;

// programma om een signaal te genereren wanneer
// de Ekolectric in zijn vooruit of achteruit staat en de deur opengaat
//int gearPosition = 0;
int batteryLevel = 100;
int range = 80;
int power = 0;
int regBraking = 0;
boolean ekoMode = false;
int timeTillFull = 0;
boolean doorSafety = false;
boolean emergencyButton = false;
boolean received = false;
boolean disconnected = true;

int VooruitPin = A8;    // Smart RND 1
int AchteruitPin = A9;    // Smart RND 2
int SchakPlusPin = A10;    // Deurcontact d.m.v. lampje in dashboard (wordt dan 12 V)
int SchakMinPin = A11;    // Deurcontact d.m.v. lampje in dashboard (wordt dan 12 V)
int ChargerPin = A12;    // Als laadstekker is ingeplugd zal deze ingang hoger worden.
int DeuropenPin = A13;    // Deurcontact d.m.v. lampje in dashboard (wordt dan 12 V)
int RegPin = 24;      // Digitale uitgangpin t.b.v. Reg-Brake signaal E-carbox
int EcoPin = 26;      // Digitale uitgangpin t.b.v. ECO signaal E-carbox
int DrivePin = 28 ;      // Digitale uitgangpin t.b.v. Drive signaal E-carbox 
int ReversePin = 30 ;      // Digitale uitgangpin t.b.v. Reverse signaal E-carbox 
int ForwardPin = 32 ;      // Digitale uitgangpin t.b.v. Forward signaal E-carbox 
int LaadWeerstandPin = 16 ; // weerstand die aangeeft in laadmode te gaan staan

int AchteruitValue = 0;  // variable to store the value coming from the sensor
int VooruitValue = 0;  // variable to store the value coming from the sensor
int SchakPlusValue = 0;
int SchakMinValue = 0;
int DeuropenValue = 0;  // variable to store the value coming from the sensor
int ChargerValue = 0;  // variable to store the value coming from the sensor
boolean fout = false;
int PlusMin = 0;
boolean Charging = false;
boolean StartStatus_Charching = false;
boolean Deuralarm = false;
int gearpos =0;
int gearpos_old =0;

//Charger parameters - set these accordingly
const int CHARGER_BAUD_RATE = 250;
const unsigned long CHARGER_CAN_ID = 0x1806E5F4;

//MCP2515/shield parameters
const int CAN_CHIP_SELECT      = 53; //10; voor de MEGA moet dit PIN 53 zijn
const int CAN_INTERRUPT_PIN    = 2;
const byte MCP2515_CONFIG =0x80;
const byte MCP2515_LISTEN =0x60;
const byte MCP2515_LOOP   =0x40;
const byte MCP2515_SLEEP  =0x20;
const byte MCP2515_NORMAL =0x00;

/**Object to interact with the MCP2515 directly*/
MCP2515 CAN( CAN_CHIP_SELECT, CAN_INTERRUPT_PIN);
Frame message0;
Frame message1;

byte mask;
const byte INTERRUPT_RESET = B00000000;
boolean setupSuccess = true;

Timer t;
Frame sendFrame;

//Charger state variables
int charger_voltage = 0;    //Reported voltage on battery 
int charger_current = 0;    //Reported current
int set_voltage = 0;        //Desired voltage
int set_current = 0;        //Desired max current
int volteller = 0;         //aantal keer dat max. akkuspanning wordt gemeten
byte output_inhibit = 1;
int charging_state = 0;

/**Initialize the CAN shield*/
boolean initCAN(void)
{
  ////serial.print.print("Initialising CAN bus: ");  

  boolean setupSuccess = false;
  pinMode(CAN_CHIP_SELECT,OUTPUT);        
  // Initialize MCP2515 CAN controller at the specified speed and clock frequency. 
  int baudRate=CAN.Init(CHARGER_BAUD_RATE,16);
  delay(1000);
  
  if(baudRate>0) { 
    //serial.println("OK");
    delay(1000);    
    setCanStatus();        
    setupSuccess = true;
  } else {
    //serial.println("initialisation failed!");
  }   
  return setupSuccess;
}

//Set up the MCP2515
void setCanStatus()
{
  //set configuraton mode
  CAN.Mode(MCP2515_CONFIG);    
  
  //Enable reception of all messages in both buffers 
  byte value= B01100100;
  CAN.Write(RXB0CTRL, value);    
  value = B01100000;
  CAN.Write(RXB1CTRL, value);
    
  //Set the interrupt enable flags and reset interrupts   
  value = B00000011;
  byte mask = B11111111;
  CAN.BitModify(CANINTE,mask, value);
  value = B00000000;
  CAN.BitModify(CANINTF, mask, value); 
    
  //finally set mode to normal
  CAN.Mode(MCP2515_NORMAL);
}
  
//Set up SPI link to MCP2515
void initSPI(void)
{
  SPI.setClockDivider(SPI_CLOCK_DIV2);
  SPI.setDataMode(SPI_MODE0);
  SPI.setBitOrder(MSBFIRST);
  SPI.begin();
}

void setup()
{

  Serial.begin(115200);
  Serial3.begin(115200);
  ChargerValue = analogRead(ChargerPin);
  if (ChargerValue > 200 ) 
      { 
      StartStatus_Charching = true ;
     //als de auto tijdens inpluggen lader in drive staat, drive blokker.
     gearpos = 0;
     gearpos_old = gearpos;
     digitalWrite(DrivePin, LOW);
     digitalWrite(ForwardPin, LOW);
     digitalWrite(ReversePin, LOW);
     // wacht 2 sec en ga dan naar laadstand en schakel extra weerstand voor laadpaal
     // indien dit niet werkt. zal moeten worden gekeken om het pulssignaal uit te lezen.
     delay(2000);
     digitalWrite(LaadWeerstandPin, HIGH);

    }
  else
    {
      StartStatus_Charching = false ;
      digitalWrite(LaadWeerstandPin, LOW);

    }
    
  Set_CAN(); 
        
  pinMode(EcoPin, OUTPUT);  // deze output t.b.v. Eco-Mode signaal E-carbox
  pinMode(DrivePin, OUTPUT);  // deze output t.b.v. Drive signaal E-carbox
  pinMode(ForwardPin, OUTPUT);  // deze output t.b.v. Vooruit signaal E-carbox
  pinMode(ReversePin, OUTPUT);  // deze output t.b.v. achteruit signaal E-carbox
  
  pinMode(LaadWeerstandPin, OUTPUT);  // deze output t.b.v. extra weerstand (via relay) laadmode

 
  //Send a message every 1000ms to keep the CAN bus alive
  int sendEvent = t.every(1000, sendData);
   
}

void loop()
{

  //Main loop
  while(true) {
    ChargerValue = analogRead(ChargerPin);
     // als laadstekker aanwezig, wordt Analoge pin naar massa getrokken
  if (ChargerValue > 200 ) 
      { 
      Charging = true ;
    }
  else
    {
      Charging = false ;
    }
 // als de status rijden c.q laden verandert na opstart. dan wordt de Arduino gereset.  
 if (StartStatus_Charching != Charging) {       asm volatile ("  jmp 0"); }
 
      if (ChargerValue > 200 ) 
            {
//            if (Charging == false) { 
//                   Charging = true;
//               }
             Laden();
            }
      else
            {
             Rijden();
            } 


  }
}

void Rijden(){
  // Lees drivestand en deursignalen in:
  
  VooruitValue = analogRead(VooruitPin);    // RND1
  AchteruitValue = analogRead(AchteruitPin); // RND2
  DeuropenValue = analogRead(DeuropenPin);
  ChargerValue = analogRead(ChargerPin);
  SchakPlusValue = analogRead(SchakPlusPin);
  SchakMinValue = analogRead(SchakMinPin);
  
  PlusMin = 0;
  if ((VooruitValue > 300) && (AchteruitValue < 300) && (SchakPlusValue < 300) && (SchakMinValue > 300))   { gearpos = 1; }
  if ((VooruitValue < 300) && (AchteruitValue < 300) && (SchakPlusValue > 300) && (SchakMinValue > 300))   { gearpos = 0; }
  if ((VooruitValue < 300) && (AchteruitValue > 300) && (SchakPlusValue > 300) && (SchakMinValue < 300))   { gearpos = -1; }
  if ((VooruitValue < 300) && (AchteruitValue > 300) && (SchakPlusValue < 300) && (SchakMinValue > 300))   { PlusMin = 1; }
  if ((VooruitValue > 300) && (AchteruitValue < 300) && (SchakPlusValue > 300) && (SchakMinValue < 300))   { PlusMin = -1; }

  // als deur sensor < dan 1 V is de deur open
  if ((DeuropenValue < 200) && (gearpos != 0)) 
        {fout = true; 
        doorSafety = true;
        Deuralarm = true;}
  else
         {fout = false;
         doorSafety = false;
         Deuralarm = false;} 
 
  // als geschakeld wordt
  if (gearpos_old != gearpos)
  { 
         gearpos_old = gearpos;
         switch (gearpos) {
            case 0:
              digitalWrite(DrivePin, LOW);
              digitalWrite(ForwardPin, LOW);
              digitalWrite(ReversePin, LOW);
            break;
          case 1:
              digitalWrite(DrivePin, HIGH);
              digitalWrite(ForwardPin, HIGH);
              digitalWrite(ReversePin, LOW);
          break;
          case -1:
              digitalWrite(DrivePin, HIGH);
              digitalWrite(ForwardPin, LOW);
              digitalWrite(ReversePin, HIGH);
          break;
         }
  }
        
  unsigned long currentMillis = millis();
 
  if(currentMillis - previousMillis > interval) {
        previousMillis = currentMillis;
        print_com1();
        print_com3();
        delay(500);
       }
    
}


void print_com1()
{
          Serial.print("{");
          Serial.print ("\"gearPosition\""":\"") ;
          Serial.print (gearpos);
          Serial.print ("\",");
          Serial.print ("\"batteryLevel\""":\"") ;
          Serial.print (batteryLevel);
          Serial.print ("\",");
          Serial.print ("\"range\""":\"") ;
          Serial.print (range);
          Serial.print ("\",");
          Serial.print ("\"power\""":\"") ;
          Serial.print (power);
          Serial.print ("\",");
          Serial.print ("\"regBraking\""":\"") ;
          Serial.print (regBraking);
          Serial.print ("\",");
          Serial.print ("\"ekoMode\""":\"") ;
          Serial.print (booleanToString((ekoMode)));
          Serial.print ("\",");
          Serial.print ("\"timeTillFull\""":\"") ;
          Serial.print (timeTillFull);
          Serial.print ("\",");
          Serial.print ("\"charging\""":\"") ;
          Serial.print (booleanToString((Charging)));
          Serial.print ("\",");
          Serial.print ("\"doorSafety\""":\"") ;
          Serial.print (booleanToString((doorSafety )));
          Serial.print ("\",");
          Serial.print ("\"emergencyButton\""":\"") ;
          Serial.print (booleanToString((emergencyButton )));
          Serial.println("\"}");

}
void print_com3()
{
          Serial3.print("{");
          Serial3.print ("\"gearPosition\""":\"") ;
          Serial3.print (gearpos);
          Serial3.print ("\",");
          Serial3.print ("\"batteryLevel\""":\"") ;
          Serial3.print (batteryLevel);
          Serial3.print ("\",");
          Serial3.print ("\"range\""":\"") ;
          Serial3.print (range);
          Serial3.print ("\",");
          Serial3.print ("\"power\""":\"") ;
          Serial3.print (power);
          Serial3.print ("\",");
          Serial3.print ("\"regBraking\""":\"") ;
          Serial3.print (regBraking);
          Serial3.print ("\",");
          Serial3.print ("\"ekoMode\""":\"") ;
          Serial3.print (booleanToString((ekoMode)));
          Serial3.print ("\",");
          Serial3.print ("\"timeTillFull\""":\"") ;
          Serial3.print (timeTillFull);
          Serial3.print ("\",");
          Serial3.print ("\"charging\""":\"") ;
          Serial3.print (booleanToString((Charging)));
          Serial3.print ("\",");
          Serial3.print ("\"doorSafety\""":\"") ;
          Serial3.print (booleanToString((doorSafety )));
          Serial3.print ("\",");
          Serial3.print ("\"emergencyButton\""":\"") ;
          Serial3.print (booleanToString((emergencyButton )));
          Serial3.println("\"}");

}

String booleanToString(boolean bol)
{
  if(bol)
  {
  return "true";
  }
  else
  {
    return "false";
  }
}


void Laden(){
     
          byte interruptFlags = CAN.Read(CANINTF);

    //Wait for next message
    while(!CAN.Interrupt()) {
      t.update();
    }
    
    if(CAN.Interrupt()) {
      //Message received
      if(interruptFlags & RX0IF) {
        interruptFlags = CAN.Read(CANINTF);
        //Retrieve the new message
	message0 = CAN.ReadBuffer(RXB0);
        //Process the message
        processMessage(message0);
        mask = RX0IF;
        CAN.BitModify(CANINTF,mask, INTERRUPT_RESET);       
      }
      
      //Message to be transmitted
      if(interruptFlags & TX0IF) {
        mask = TX0IF;
        CAN.BitModify(CANINTF,mask, INTERRUPT_RESET);
      }
      
      //Error
      if(interruptFlags & ERRIF) {
        //serial.println("Error encountered");
        mask = ERRIF;
        CAN.BitModify(CANINTF,mask, INTERRUPT_RESET);
      }
      if(interruptFlags & MERRF) {
        //serial.println("Error encountered 2");
        mask = MERRF;
        CAN.BitModify(CANINTF,mask, INTERRUPT_RESET);
      }
    } 
}
void Set_CAN(){
  
  //Default values for the message to be send to the charger  
  sendFrame.id = CHARGER_CAN_ID;
  sendFrame.srr = 1;
  sendFrame.rtr = 0;
  sendFrame.ide = 1;
  sendFrame.dlc = 8;
  sendFrame.data[0] = 0;
  sendFrame.data[1] = 0;
  sendFrame.data[2] = 0;
  sendFrame.data[3] = 0;
  sendFrame.data[4] = 1;
  sendFrame.data[5] = 0;
  sendFrame.data[6] = 0;
  sendFrame.data[7] = 0;
  
  //Set up SPI and CAN
  initSPI();
  setupSuccess &= initCAN(); 
}
//Processes the received message
void processMessage(Frame& message) {
  String status_string = "";
  boolean error = false;
  int charger_status = 0;
  
  if(message.id>0) {
 
    charger_voltage = (message.data[0]<<8) + message.data[1];
    charger_current = (message.data[2]<<8) + message.data[3];
    charger_status = message.data[4];

    batteryLevel = charger_voltage/11.75; // 100% als batterij spanning van 117 is behaald;
    range = charger_voltage/14.68;
    
    if (((charger_status) >> 3) && 0x01) {error = true; status_string += "Battery not detected. ";}
    if (((charger_status) >> 4) && 0x01) {error = true; status_string += "Communication timeout. ";}
    if (((charger_status) >> 2) && 0x01) {error = true; status_string += "Input voltage warning. ";}
    if (((charger_status) >> 1) && 0x01) {error = true; status_string += "Temperature warning. ";}
    if (((charger_status) >> 0) && 0x01) {error = true; status_string += "Hardware failure. ";}
    
    if (error) charging_state = 0;
    
    switch (charging_state) {
      case 0:
        //Off state
        output_inhibit = 1;
        
        if (!error) charging_state = 1;
        volteller = 0;
        break;
      case 1:    
        //Charge to 117.5V @ 8A
        output_inhibit = 0;
        set_voltage = 1175;
        set_current = 80;
        status_string += "Charging to 117.5V @ 8A max";
        // als akkuspanning >= 117,5 Volt is met een laadstroom < 1 A, zal een teller oplopen
        if ((charger_voltage > 1175) && (charger_current < 10)) { 
            volteller++;
            //Als meer dan 20 keer bovenstaande voorwaarde wordt gemeten, zal de lader naar een lager spanning en stroom gaan: Case "2"
            if (volteller >= 20){ 
                charging_state = 2;
            }  
        }      
        print_com1();
        break;
      case 2:
        //Charge to 108V
        output_inhibit = 0;
        set_voltage = 1080;
        set_current = 50;
        status_string += "Charging to 108V @ 5A max";
        batteryLevel = 100; // 100% batterij is vol;
        range = 80;

        print_com1();
        break;
    }
    //tijdelijk weer even aangezet op com"3"
    Serial3.print("Vbatt = ");
    Serial3.print((float)charger_voltage/10);
    Serial3.print(" V, I = ");
    Serial3.print((float)charger_current/10);
    Serial3.print(" A, status = ");
    Serial3.println(status_string);
  }
}  

//Sends message to charger. Called every 1000ms
void sendData () {  
  Frame txBuffer;
  
  sendFrame.data[0] = (set_voltage >> 8) & 0xFF;
  sendFrame.data[1] = set_voltage & 0xFF;
  sendFrame.data[2] = 0; //we never charge more than 12A
  sendFrame.data[3] = set_current & 0xFF;
  sendFrame.data[4] = output_inhibit;
  
  CAN.LoadBuffer(TXB0 , sendFrame);
  CAN.SendBuffer(TXB0);
}

