
// Sketch to control toyota windscreen wipers and open garage door
// Copyright Jim West (2016)

#include <RFM69.h>    //get it here: https://www.github.com/lowpowerlab/rfm69
#include <RFM69_ATC.h>//get it here: https://www.github.com/lowpowerlab/rfm69
#include <EEPROM.h>   // used to store parameters, in specific the time interval between reading transmissions
#include <LowPower.h> // used to power down/sleep the unit to save power
#include <radio_struct.h> // library to hold the radio packet structure

//#define INITIAL_SETUP // uncomment this for an initial setup of a moteino

// ********************************************************************************************
// Sketch specific settings
// ********************************************************************************************
#define TITLE "Toyotanode - sketch for controlling toyota wipers"

//*********************************************************************************************
//************ MOTEINO specific settings
//*********************************************************************************************
#define NODEID      8    //must be unique for each node on same network (range up to 254, 255 is used for broadcast)
#define NETWORKID   100   //the same on all nodes that talk to each other (range up to 255)
#define GATEWAYID   1
//Match frequency to the hardware version of the radio on your Moteino (uncomment one):
#define FREQUENCY   RF69_433MHZ
//#define FREQUENCY   RF69_868MHZ
//#define FREQUENCY   RF69_915MHZ

//#define ENCRYPTKEY  "RandomKeyHere123" //exactly the same 16 characters/bytes on all nodes!
#define IS_RFM69HW  //uncomment only for RFM69HW! Leave out if you have RFM69W!
#define LED_DEVICE    3
#define TOYOTA_DEVICE 121
#define GARAGE_DEVICE 32
#define GARAGE_NODE   4
#define MOTEINO_DEVICE 1

#define LED           9 // Moteinos have LEDs on D9

//**********************************************************************************************
// Radio transmission specific settings
// ******************************************************************************************
radioPayload receiveData1, sendData1;
radioPayload2 receiveData2, sendData2;

RFM69 radio;

byte radio_network, radio_node;
byte radio_gateway;
char buff[20];
byte sendSize = 0;
boolean requestACK = false;
char radio_encrypt[16];


// ***********************************************************
// EEPROM Parameter offsets
#define RELAY_DELAY_OFFSET 20
#define UPDATE_INTERVAL_OFFSET 30
#define RADIO_NETWORK 101
#define RADIO_NODE 102
#define RADIO_GATEWAY 103
#define RADIO_ENCRYPT 105

#define EEPROM_SENSOR_SENSITIVITY 130

//*********************************************************************************************
// Serial channel settings
//*********************************************************************************************
#define SERIAL_BAUD   115200

//*******************************************************************************
// General settings
//*******************************************************************************
#define INIT_UPDATE_INTERVAL 5000  // millisecs between general update messages

//************************************************************
// LED is device 3
// ***********************************************************
int ledStatus = 0;    // initially off

//**********************************
// General declarations
//**********************************
unsigned long timepassed = 0L;
unsigned long last_update = 5000L;
unsigned long nCnt = 0L;
String inputString = "";         // a string to hold incoming data
boolean stringComplete = false;  // whether the string is complete
unsigned long requestID;
boolean first_msg = true;

//*************************
//Variables associated with the switch
#define WIPER_PIN   A0
#define WIPER_MIN   2             // min interval for the wiper. Each sweep has been timed at 1.7sec
#define WIPER_MAX   30            // max interval between wipes
#define WIPER_RELAY 8             // pin connected to wiper relay
#define INIT_RELAY_DELAY 500           // millisecs relay should stay triggered

unsigned long sw_last_off = 0L;   // timer when switch was last seen as closed
int sw_status = 0;                // flag, 0 is closed, 1 is open
int sw_state = 0;                 // store state of switch pin
#define SHORT_SWITCH_TIME    10     // switch closure lass than this will be ignored (bounce)
#define LONG_SWITCH_TIME    1000  // switch closure longer than this is a long press, less is a short press
#define SWITCH_PIN          12

int wiper_status = 0;             // used to tell if the wipers are on or off
int relay_status = 0;             // used to control trigger of relay
int wiper_pot = 0;                // records the setting of the potentiometer
float wiper_delay;                // time in sec between wipes
int relay_delay;                  // stores msec that wiper relay should stay on
unsigned long wiper_timer = 0L;   //used
long update_interval = 0;

//*************************************************************
//***  SETUP Section
//*************************************************************
void setup() {
  // enable the serial channel
  Serial.begin(SERIAL_BAUD);

  // set up the radio

#ifdef INITIAL_SETUP
  EEPROM.put(RADIO_NETWORK, NETWORKID);          // temp to set up network
  EEPROM.put(RADIO_NODE, NODEID);                // temp to set up node
  EEPROM.put(RADIO_GATEWAY, GATEWAYID);          // temp to set up gateway
  EEPROM.put(RADIO_ENCRYPT, ENCRYPTKEY);         // temp to set up encryption key
  EEPROM.put(RELAY_DELAY_OFFSET, INIT_RELAY_DELAY);   // set up relay delay
  EEPROM.put(UPDATE_INTERVAL_OFFSET, INIT_UPDATE_INTERVAL)
#endif
  EEPROM.get(RADIO_NETWORK, radio_network);      // get network
  EEPROM.get(RADIO_NODE, radio_node);            // get node
  EEPROM.get(RADIO_GATEWAY, radio_gateway);      // get gateway
  EEPROM.get(RADIO_ENCRYPT, radio_encrypt);      // get encryption key

  Serial.println(TITLE);
  Serial.print("Radio network = ");
  Serial.println(radio_network);
  Serial.print("Radio node = ");
  Serial.println(radio_node);
  Serial.print("Radio gateway = ");
  Serial.println(radio_gateway);
  Serial.print("Radio password = ");
  char y;
  for (int x = 0; x < 16; x++) {
    y = EEPROM.read(RADIO_ENCRYPT + x);
    Serial.print(y);
    radio_encrypt[x] = y;
  }
  Serial.println("");
  //radio_encrypt = radio_encrypt + '"';

  radio.initialize(FREQUENCY, radio_node, radio_network);
#ifdef IS_RFM69HW
  radio.setHighPower(); //uncomment only for RFM69HW!
  Serial.println("Set high power");
#endif
  radio.encrypt(radio_encrypt);

  char buff[50];
  sprintf(buff, "\nTransmitting at %d Mhz...", FREQUENCY == RF69_433MHZ ? 433 : FREQUENCY == RF69_868MHZ ? 868 : 915);
  Serial.println(buff);

  sendData2.nodeID = radio_node;    // This node id should be the same for all devices

  EEPROM.get(RELAY_DELAY_OFFSET, relay_delay);   // set up relay delay
  if ((relay_delay < 0) or (relay_delay > 2000))
  {
    EEPROM.put(RELAY_DELAY_OFFSET, INIT_RELAY_DELAY);   // set up relay delay
    relay_delay = INIT_RELAY_DELAY;
    Serial.println("Correcting relay delay");
  }

  EEPROM.get(UPDATE_INTERVAL_OFFSET, update_interval);   // set up update interval
  if ((update_interval < 0) or (update_interval > 60000))
  {
    EEPROM.put(UPDATE_INTERVAL_OFFSET, INIT_UPDATE_INTERVAL);   // set up update interval
    update_interval = INIT_UPDATE_INTERVAL;
    Serial.println("Correcting interval update");
  }

  //set up of pins
  pinMode(LED, OUTPUT);
  pinMode(SWITCH_PIN, INPUT);
  pinMode(WIPER_RELAY, OUTPUT);
  digitalWrite(WIPER_RELAY, LOW);
}
//************************************************
// End of Setup
//************************************************


//*************************************************************
// main loop
//*************************************************************
void loop() {
  //process any serial input
  serialEvent();            // call the function
  if (stringComplete)       // keep all serial processing in one place
    process_serial();

  sw_state = digitalRead(SWITCH_PIN);

  //check for any received packets
  if (radio.receiveDone())
  {
    Serial.println("Radio packet received");
    process_radio();
  }

  //Serial.println(update_interval);
  if ((last_update + update_interval) < millis())
  {
    last_update = millis();
    send_radio_msg(radio_gateway, radio_node, 'I', MOTEINO_DEVICE, millis() / 1000, 0.0, 0.0, last_update);
    Serial.println("Running status update ");
  }

  // switch detection section

  if (sw_state != sw_status)                    // switch state has changed

  {
    sw_status = sw_state;                       // reset the status
    if (sw_status == 1)                         // record when the botton was pushed so we can see how long it was pushed for
      sw_last_off = millis() - 1;

    if (sw_status == 0)                         // button now released
      if (millis() - sw_last_off > LONG_SWITCH_TIME)    // this is a long press
      {
        Serial.println("Long press detected");
        //send_radio_msg(GARAGE_NODE, radio_node, 'A', GARAGE_DEVICE, 2, 0.0, 0, millis()); // commented out until ready to go live

        sendData2.nodeID = radio_node; // This node's ID
        sendData2.deviceID = GARAGE_DEVICE; // TOYOTA code
        sendData2.instance = 1;
        sendData2.req_ID = millis();
        sendData2.action = 'A';
        sendData2.result = 0;
        sendData2.float1 = 2;
        sendData2.float2 = 0;
        sendData2.float3 = 0;
        sendData2.float4 = 0;

        if (radio.sendWithRetry(GARAGE_NODE, (const void*)(&sendData2), sizeof(sendData2)))
          Serial.println("OK");
        else Serial.println(" nothing");

        //radio.send(gateway, (const void*)(&sendData2), sizeof(sendData2));



        Serial.println("Garage radio message sent");
      }
      else if (millis() - sw_last_off > SHORT_SWITCH_TIME) // this is a short press - wiper control
      {
        Serial.println("Short press detected");
        if (wiper_status == 1)                             // wipers are on, turn them off
        {
          wiper_status = 0;
          Serial.println("Turn wipers off");
          digitalWrite(LED, LOW);
        }
        else                                               // wipers off, turn them on
        {
          wiper_status = 1;
          Serial.println("Turn wipers ON");
          digitalWrite(LED, HIGH);
          wiper_timer = 0L;
        }
      }
      else
        Serial.println("Bounce press detected");    // this just signals a very short press, no action taken
  }

  // This section controls the wipers
  if (wiper_status == 1)                              // Wipers are on
  {

    wiper_delay = (WIPER_MAX - WIPER_MIN) * (analogRead(WIPER_PIN) / 1024.0) + WIPER_MIN;

    if (millis() > (wiper_timer + (wiper_delay * 1000)))
    {
      //Serial.println("Trigger wiper");
      wiper_timer = millis();
      relay_status = 1;
      digitalWrite(WIPER_RELAY, HIGH);
      digitalWrite(LED, LOW);
      //Serial.println("Relay on");
      //Serial.print("Wiper delay = ");
      //Serial.println(wiper_delay);
      Serial.print("Wiper pot input = ");
      Serial.println(analogRead(WIPER_PIN));
    }

    if (relay_status == 1)
    {

      if (millis() > wiper_timer + relay_delay)
      {
        relay_status = 0;
        digitalWrite(WIPER_RELAY, LOW);
        digitalWrite(LED, HIGH);
        //Serial.println("Relay off");
      }
    }

  }

}

// *************************************************************************************************
int sensor_check(int sensor)
{
  int sensor_output;
  int detected = 0;
  sensor_output = analogRead(sensor);

  detected = 1;
  return detected;
}


// *************************************************************************************************
void serialEvent() {
  while (Serial.available()) { // keep on reading while info is available
    // get the new byte:
    char inByte = Serial.read();

    // add it to the inputString:
    if ((inByte >= 65 && inByte <= 90) || (inByte >= 97 && inByte <= 122) || (inByte >= 48 && inByte <= 57) || inByte == 43 || inByte == 44 || inByte == 46 || inByte == 61 || inByte == 63) {
      inputString.concat(inByte);
    }
    if (inByte == 10 || inByte == 13) {
      // user hit enter or such like
      Serial.println(inputString);
      stringComplete = true;
    }
  }
}

// ************************************************************************************************
void process_serial()
{

  if (inputString == "r") //d=dump register values
    radio.readAllRegs();
  if (inputString == "E") //E=enable encryption
    radio.encrypt(radio_encrypt);
  if (inputString == "e") //e=disable encryption
    radio.encrypt(null);

  // clear the string:
  inputString = "";
  stringComplete = false;
}

// **********************************************************************************************
void process_radio()
{
  Serial.print('['); Serial.print(radio.SENDERID, DEC); Serial.print("] ");
  for (byte i = 0; i < radio.DATALEN; i++)
    Serial.print((char)radio.DATA[i]);
  Serial.print("   [RX_RSSI:"); Serial.print(radio.RSSI); Serial.print("]");

  if (radio.ACKRequested())
  {
    radio.sendACK();
    Serial.print(" - ACK sent");
  }

  Serial.println();
  Serial.println("Data received");
  receiveData2 = *(radioPayload2*)radio.DATA;
  printTheData(receiveData2);
  requestID = receiveData2.req_ID;

  sendData2.nodeID = 1;    // always send to the gateway node
  sendData2.instance = receiveData1.instance;

  if (receiveData2.nodeID = radio_node)  // only if the message is for this node
  {
    switch (receiveData2.action) {
      case 'P':   // parameter update
        if (receiveData2.deviceID == TOYOTA_DEVICE)
        {
          Serial.print("Parameter update request (relay delay) for Toyota device ");
          Serial.print(receiveData2.float1);
          Serial.println(" millisecs");
          relay_delay = receiveData2.float1;
          EEPROM.put(RELAY_DELAY_OFFSET, relay_delay);
          send_radio_msg(radio_gateway, radio_node, 'R', TOYOTA_DEVICE, relay_delay, 0.0, 0.0, requestID);
        }
        else if (receiveData2.deviceID == MOTEINO_DEVICE)
        {
          Serial.print("Parameter update request (update interval) for Moteino device ");
          Serial.print(receiveData2.float1);
          Serial.println(" secs");
          update_interval = receiveData2.float1 * 1000;
          EEPROM.put(UPDATE_INTERVAL_OFFSET, update_interval);
          send_radio_msg(radio_gateway, radio_node, 'R', MOTEINO_DEVICE, update_interval / 1000, 0.0, 0.0, requestID);
        }
        break;

      case 'Q':    // Parameter query
        if (receiveData2.deviceID == TOYOTA_DEVICE)  // Toyota
        {
          Serial.println("Parameter request ");
          Serial.print("Relay delay is ");
          Serial.print(relay_delay);
          Serial.println(" milliseconds");
          send_radio_msg(radio_gateway, radio_node, 'R', TOYOTA_DEVICE, relay_delay, 0.0, 0.0, requestID);
        }
        else if (receiveData2.deviceID == MOTEINO_DEVICE)
        {
          Serial.println("Parameter request ");
          Serial.print("Update interval is ");
          Serial.print(relay_delay);
          Serial.println(" milliseconds");
          send_radio_msg(radio_gateway, radio_node, 'R', MOTEINO_DEVICE, update_interval / 1000, 0.0, 0.0, requestID);
        }
        break;

    }   // end swicth case for action
  }   // end if when checking for NODEID
}


// *************************************************************************************************
void printTheData(radioPayload2 & myData)
{
  Serial.print("NodeID=");
  Serial.print(myData.nodeID);
  Serial.print(", deviceID=");
  Serial.print(myData.deviceID);
  Serial.print(", instance=");
  Serial.print(myData.instance);
  Serial.print(", action=");
  Serial.print(myData.action);
  Serial.print(", result=");
  Serial.print(myData.result);
  Serial.print(", req_ID=");
  Serial.print(myData.req_ID);
  Serial.print(", float1=");
  Serial.print(myData.float1);
  Serial.print(", float2=");
  Serial.print(myData.float2);
  Serial.print(", float3=");
  Serial.print(myData.float3);
  Serial.print(", float4=");
  Serial.println(myData.float4);
}

//*******************************************************************************************
// Generic routine for sending a radio message
void send_radio_msg(byte gateway, byte in_node, char action, int device, float param1, float param2, float result, unsigned long requestID)

{
  sendData2.nodeID = in_node; // This node's ID
  sendData2.deviceID = device; // TOYOTA code
  sendData2.instance = 1;
  sendData2.req_ID = requestID;
  sendData2.action = action;
  sendData2.result = result;
  sendData2.float1 = param1;
  sendData2.float2 = param2;
  sendData2.float3 = 0;
  sendData2.float4 = 0;

  if (radio.sendWithRetry(gateway, (const void*)(&sendData2), sizeof(sendData2)))
    Serial.println("OK");
  else Serial.println(" nothing");

  //radio.send(gateway, (const void*)(&sendData2), sizeof(sendData2));
  //printTheData(sendData2);

}

