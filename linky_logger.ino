#include <Arduino.h>
#include <Arduino_JSON.h>
#include <SdFat.h>
#include <SPI.h>
#include "wiring_private.h"

#define FPSTR(pstr_pointer) (reinterpret_cast<const __FlashStringHelper*>(pstr_pointer)) // To retrieve data from program flash memory


//                                            --------------------------------------------
//                                            |Inter-chip communication declarations block|
//                                            --------------------------------------------

// Instantiation of the wifiModuleSerial Uart class as a new hardware serial on pin 3 and 4 on sercom2
Uart wifiModuleSerial(&sercom2, 3, 4, SERCOM_RX_PAD_1, UART_TX_PAD_0);  // Define an hardware serial on pin 3(rx) and 4(tx) for communication with the wifi module (specific to Arduino Zero)

//SoftwareSerial wifiModuleSerial(2, 3);                                // Software serial that use custom digital pins as RX and TX for serial communication with the wifi module
                                                                        // !! Beware that the default internal buffer size of the SoftwareSerial.h has to be changed from 64 char to 256 char, or messages will be truncated


//                                            ------------------------------
//                                            |Global technical consts block|
//                                            ------------------------------

// JSON keys for Init parameters
PROGMEM static const char PARAM_CON_SSID[] = "con_ssid";
PROGMEM static const char PARAM_CON_PASSWORD[] = "con_password";
PROGMEM static const char PARAM_NET_TIMEOUT[] = "net_timeout";
PROGMEM static const char PARAM_NET_ENDPOINT[] = "net_endpoint";
PROGMEM static const char PARAM_NET_FINGERPRINT[] = "net_sha1";
PROGMEM static const char PARAM_NET_MODULE_TIMEOUT[] = "net_module_timeout";
PROGMEM static const char PARAM_AUT_LOGIN[] = "aut_login";
PROGMEM static const char PARAM_AUT_PASSWORD[] = "aut_password";
PROGMEM static const char PARAM_LED_THRESHOLD[] = "led_threshold";
PROGMEM static const char PARAM_MOD_DEBUG[] = "mod_debug";
PROGMEM static const char PARAM_MOD_INITLEDDANCE[] = "mod_initleddance";
PROGMEM static const char PARAM_DEV_SPISDCSPIN[] = "dev_spisdcspin";
PROGMEM static const char PARAM_DEV_PHOTORESPIN[] = "dev_photorespin";

// JSON keys for inter-chip messages
PROGMEM static const char MSG_ENUM_CMD[] = "CMD";
PROGMEM static const char MSG_ENUM_CMD_SEQ[] = "CMD_SEQ";
PROGMEM static const char MSG_ENUM_CODE[] = "CODE";
PROGMEM static const char MSG_ENUM_ENDPOINT[] = "ENDPOINT";
PROGMEM static const char MSG_ENUM_LOGIN[] = "LOGIN";
PROGMEM static const char MSG_ENUM_PASSWORD[] = "PASSWORD";
PROGMEM static const char MSG_ENUM_DATA[] = "DATA";

// Other JSON keys
PROGMEM static const char JSON_LEDMILIS[] = "ledMilis";

// Inter-chip commands enumeration
PROGMEM static const char CMD_HTTPGET[] = "HTTPGET";
PROGMEM static const char CMD_HTTPPOST[] = "HTTPPOST";
PROGMEM static const char CMD_HTTPSGET[] = "HTTPSGET";
PROGMEM static const char CMD_HTTPSPOST[] = "HTTPSPOST";
PROGMEM static const char CMD_NTWKCHANG[] = "NTWKCHANG";

// Other constants
PROGMEM static const char UNDEFINED[] = "undefined";
PROGMEM static const char CONFIG_FILE[] = "linkyLoggerConfig.json";     // Name of the config file on the SD card for init paramaters injection

static const byte DEBUG_LED = 13;                                       // Pin number of the debug LED

const static char START_MESSAGE_BLOCK = '\31';                          // ASCII char to trigger the start of an inter-chip message
const static char STOP_MESSAGE_BLOCK = '\23';                           // ASCII char to trigger the end of an inter-chip message

static const byte MESSAGE_SIZE_LIMIT = 255;                             // Maximum number of char to store / message
static const int CONFIG_FILE_SIZE_LIMIT = 500;                          // Maximum number of char for config file
static const byte RESPONSES_NBR_LIMIT = 5;                              // Maximum number of responses to wait for. If at somme point there is more pending responses, the older ones gets abandonned.


//                                            ----------------------------------
//                                            |Global injectable variables block|
//                                            ----------------------------------

String connectionSSID = "";                                             // wifi network name 
String connectionPSWRD = "";                                            // wifi password
unsigned int responseTimeout = 10000;                                   // Communication timeout milis for pending inter-chip commands, Arduino side. Default is 10000.
unsigned long wifiModuleTimeoutTimeout = 60000;                         // Timeout milis for pending connection on the wifi module side. Also define HTTP request timeout at 1:10 of that value. Default is 60000.
String postBackendEndpoint = "";                                        // Back-end to contact. Complete HTTP/HTTPS entry-point adress for POST method.
String serverCertFingerprint = "";                                      // TLS Cert Fingerprint.
String authentLogin = "";                                               // Login for back-end authentification (if any)
String authentPassword = "";                                            // Password for back-end authentification (if any)
int lightThreshold = 800;                                               // Light detection sensitivity threshold. voltage mesurement on 10 bits (0-1023). The higher the value, the less sensitive the detection is. default is 15.
bool debugMode = false;                                                 // Debug mode. Default is false.
bool showInitLedDance = true;                                           // Led animation to indicate a successful initialization. Default is true.
int pinCS = 8;                                                          // SPI CS pin for SD card reader. Default is Pin 8 (Arduino Zero).
int analogLightResistorPin = A0;                                        // Photoresistor input pin. Default is Analog pin A0.


//                                            --------------------
//                                            |Global struct block|
//                                            --------------------

// Struct to store informations about pending inter-chip commands
struct PendingCommandStruct {
  unsigned long sequence;                                               // Sequence number of the inter-chip command
  unsigned long timeout;                                                // Timeout threshold milis for the inter-chip command
  byte callback;                                                        // callback ID of the inter-chip command
};


//                                            ---------------------------
//                                            |Internal program variables|
//                                            ---------------------------

unsigned long lastMilisCheckPoint = 0;                                  // last check read of milis() function
bool ledHasBlinked = false;                                             // If the Linky led has been detected and the associated procedure has already been performed
bool isReading = false;                                                 // to flag a "currently reading" state from wifiModuleSerial
bool isConnectionOk = false;                                            // to flag if the connection is ok and identify a "network lost" event
char messageBuffer[MESSAGE_SIZE_LIMIT];                                 // Inter-chip message String buffer
byte messageCharIterator = 0;                                           // Inter-chip message iterator
unsigned long cmdSequence = 1;                                          // Internal command messages sequence number
PendingCommandStruct pendingCommands[RESPONSES_NBR_LIMIT];              // Array of PendingCommandStruct struct {sequence numbers, callBack name, timeout}


//                                            ----------------
//                                            |Arduino program|
//                                            ----------------

void setup() {

  delay(3000); // Convenient init delay for debug

  // start debug serial
  Serial.begin(115200);
  Serial.println(F("Initialisation ..."));

  // set build in led as output and turn on the led
  pinMode(DEBUG_LED, OUTPUT);
  digitalWrite(DEBUG_LED, HIGH);

  // start communication with custom hardware serial on pin 3 and 4
  wifiModuleSerial.begin(115200);
  // Assign pins 3 & 4 SERCOM functionality
  pinPeripheral(3, PIO_SERCOM_ALT);
  pinPeripheral(4, PIO_SERCOM_ALT);

  // set pin for the light resistor
  pinMode(analogLightResistorPin,INPUT);
  
  // Check SD card for network auth + parameters
  injectInitParameters();

  digitalWrite(DEBUG_LED, LOW);
  
  // Perform a NTWKCHANG command to init wifi connection on the 8266 wifi module
  initNetworkChange();

  // TODO Check if the backend endpoint is correctly set
  
  Serial.println(F("Initialisation ok."));
}



void loop() {
  if(!debugMode){
    // There is no reason to call the sensors check method that potentially sends commands if connection is lost
    // Memory wise we can not store sensor informations and batch send them later anyway
    if(isConnectionOk){
      // check sensors
      checkForLightResistance();
    } else {
      // Connection lost management
      connectionLostManagement();
    }
    // check for inter-chip messages
    checkForWifiModuleMessages();
    // Command request management timout
    timeoutManagement();
  } else {
    debugLoop();
  }
}


void debugLoop() {
  if((millis() - lastMilisCheckPoint) > 60000){
    JSONVar myData;
    myData[String(FPSTR(JSON_LEDMILIS))] = (millis() - lastMilisCheckPoint);
    Serial.println(F("request start to wifi module"));
    sendToWifiModule(String(FPSTR(CMD_HTTPSGET)).c_str(), postBackendEndpoint.c_str(), "", "", &myData, 3);
    lastMilisCheckPoint = millis();
  }
}


void successfulLEDDance(){
  int i = 0;
  bool ledIsOn = false;
  int ledDelay = 300;
  for(i = 0 ; i < 32 ; i++){
    if(ledIsOn){
      digitalWrite(DEBUG_LED, LOW);
      ledIsOn = false;
    } else {
      digitalWrite(DEBUG_LED, HIGH);
      ledIsOn = true;
    }
    delay(ledDelay);
    ledDelay = ledDelay - 9;
  }
  digitalWrite(DEBUG_LED, HIGH);
  delay(3000);
  digitalWrite(DEBUG_LED, LOW);
}


void failLEDDance(){
  int i = 0;
  bool ledIsOn = false;
  int ledDelay = 500;
  for(i = 0 ; i < 10 ; i++){
    if(ledIsOn){
      digitalWrite(DEBUG_LED, LOW);
      ledIsOn = false;
    } else {
      digitalWrite(DEBUG_LED, HIGH);
      ledIsOn = true;
    }
    delay(ledDelay);
  }
  digitalWrite(DEBUG_LED, LOW);
}


void initNetworkChange(){
  JSONVar ntwkchangJsonVar;
  ntwkchangJsonVar[String(FPSTR(PARAM_NET_FINGERPRINT))] = serverCertFingerprint;  // push TLS cert fingerprint of the server to the wifi module
  ntwkchangJsonVar[String(FPSTR(PARAM_NET_MODULE_TIMEOUT))] = wifiModuleTimeoutTimeout;  // push wifi module timeout value
  sendToWifiModule(String(FPSTR(CMD_NTWKCHANG)).c_str(), "", connectionSSID.c_str(), connectionPSWRD.c_str(), &ntwkchangJsonVar, 0);
}


void injectInitParameters(){
  SdFat sd;
  SdFile configFile;

  if (!sd.begin(pinCS, SPI_HALF_SPEED)){
    Serial.println(F("No SD card found or unable to connect, skipping init params..."));
    sd.initErrorHalt();
  } else {
    Serial.println(F("SD card is ready to use."));
  }

  if(configFile.open(String(FPSTR(CONFIG_FILE)).c_str(), O_READ)){
    Serial.println(F("Config file found, reading params..."));
    // Open config file
    char initParamsBuffer[CONFIG_FILE_SIZE_LIMIT]; // init params String buffer
    
    if(configFile.dataLength() > CONFIG_FILE_SIZE_LIMIT){
      Serial.println(F("Json config file is too large to be parsed"));
      Serial.println(F("Size :"));
      Serial.println(configFile.dataLength());
    } else {
      // Reading the whole file
      int paramsCharIterator = 0;
      int data;
      while ((data = configFile.read()) >= 0) {
        initParamsBuffer[paramsCharIterator] = data; // we append the char to the string buffer
        paramsCharIterator++;
      }
      configFile.close();

      // Analysing params
      JSONVar initParams = JSON.parse(initParamsBuffer);

      String jsonConSSID = String((const char*)initParams[String(FPSTR(PARAM_CON_SSID))]);
      String jsonConPassword = String((const char*)initParams[String(FPSTR(PARAM_CON_PASSWORD))]);
      String jsonNetTimeout = String((const char*)initParams[String(FPSTR(PARAM_NET_TIMEOUT))]);
      String jsonNetEndpoint = String((const char*)initParams[String(FPSTR(PARAM_NET_ENDPOINT))]);
      String jsonNetFingerprint = String((const char*)initParams[String(FPSTR(PARAM_NET_FINGERPRINT))]);
      String jsonNetModuleTimeout = String((const char*)initParams[String(FPSTR(PARAM_NET_MODULE_TIMEOUT))]);
      String jsonAutLogin = String((const char*)initParams[String(FPSTR(PARAM_AUT_LOGIN))]);
      String jsonAutPassword = String((const char*)initParams[String(FPSTR(PARAM_AUT_PASSWORD))]);
      String jsonLedThreshold = String((const char*)initParams[String(FPSTR(PARAM_LED_THRESHOLD))]);
      String jsonModDebug = String((const char*)initParams[String(FPSTR(PARAM_MOD_DEBUG))]);
      String jsonModInitLedDance = String((const char*)initParams[String(FPSTR(PARAM_MOD_INITLEDDANCE))]);
      String jsonDevSPISDCSPin = String((const char*)initParams[String(FPSTR(PARAM_DEV_SPISDCSPIN))]);
      String jsonDevPhotoResPin = String((const char*)initParams[String(FPSTR(PARAM_DEV_PHOTORESPIN))]);

      if(String(FPSTR(UNDEFINED)) != jsonConSSID){
        connectionSSID = jsonConSSID;
      }
      if(String(FPSTR(UNDEFINED)) != jsonConPassword){
        connectionPSWRD = jsonConPassword;
      }
      if(String(FPSTR(UNDEFINED)) != jsonNetTimeout){
        int tmpValue = jsonNetTimeout.toInt();
        if(tmpValue > 0 && tmpValue < 60000){
          responseTimeout = tmpValue;
        }
      }
      if(String(FPSTR(UNDEFINED)) != jsonNetEndpoint){
        postBackendEndpoint = jsonNetEndpoint;
      }
      if(String(FPSTR(UNDEFINED)) != jsonNetFingerprint && 20 == jsonNetFingerprint.length()){
        jsonNetFingerprint.toUpperCase();
        serverCertFingerprint = jsonNetFingerprint;
      }
      if(String(FPSTR(UNDEFINED)) != jsonNetModuleTimeout){
        long tmpValue = jsonNetTimeout.toInt();
        if(tmpValue > 10000 && tmpValue < 120000){
          wifiModuleTimeoutTimeout = tmpValue;
        }
      }
      if(String(FPSTR(UNDEFINED)) != jsonAutLogin){
        authentLogin = jsonAutLogin;
      }
      if(String(FPSTR(UNDEFINED)) != jsonAutPassword){
        authentPassword = jsonAutPassword;
      }
      if(String(FPSTR(UNDEFINED)) != jsonLedThreshold && jsonLedThreshold.toInt() > 0){
        int tmpValue = jsonLedThreshold.toInt();
        if(tmpValue > 0 && tmpValue < 1024){
          lightThreshold = jsonLedThreshold.toInt();
        }
      }
      if(String(FPSTR(UNDEFINED)) != jsonModDebug){
        if("true" == jsonModDebug || "TRUE" == jsonModDebug){
          debugMode = true;
        } else if("false" == jsonModDebug || "FALSE" == jsonModDebug){
          debugMode = false;
        }
      }
      if(String(FPSTR(UNDEFINED)) != jsonModInitLedDance){
        if("true" == jsonModInitLedDance || "TRUE" == jsonModInitLedDance){
          showInitLedDance = true;
        } else if("false" == jsonModInitLedDance || "FALSE" == jsonModInitLedDance){
          showInitLedDance = false;
        }
      }
      // Dev params
      if(String(FPSTR(UNDEFINED)) != jsonDevSPISDCSPin && jsonDevSPISDCSPin.toInt() > 0){
        pinCS = jsonDevSPISDCSPin.toInt();
      }
      if(String(FPSTR(UNDEFINED)) != jsonDevPhotoResPin && jsonDevPhotoResPin.toInt() > 0){
        analogLightResistorPin = jsonDevPhotoResPin.toInt();
      }
      Serial.println(F("Config params found and injected successfuly!"));
    }
  } else {
    Serial.println(F("No config file found or unable to open, skipping init params..."));
  }
  sd.end();  // end SD card communication
}


void checkForLightResistance(){

  int value = analogRead(analogLightResistorPin);   // We read the voltage on the analog input

  if(value > lightThreshold && !ledHasBlinked){
    unsigned long tmpMilis = millis();
    Serial.println(F("led Detected!!"));
    digitalWrite(DEBUG_LED, HIGH);

    if(lastMilisCheckPoint > 0){
      // calculate and send data
      if(isConnectionOk){
        JSONVar myData;
        myData[String(FPSTR(JSON_LEDMILIS))] = tmpMilis - lastMilisCheckPoint;
        Serial.println(F("request start to wifi module"));
        sendToWifiModule(String(FPSTR(CMD_HTTPPOST)).c_str(), postBackendEndpoint.c_str(), "", "", &myData, 1);
      }
    }
    
    lastMilisCheckPoint = tmpMilis;
    ledHasBlinked = true;
    
  } else if(value < lightThreshold && ledHasBlinked){
    ledHasBlinked = false;
    digitalWrite(DEBUG_LED, LOW);
  }
}


/*
* Name: sendToWifiModule
* Description: Function used to send data to the wifi module.
*/
void sendToWifiModule(const char* command, const char* endpoint, const char* login, const char* password, const JSONVar* data, const byte callbackID) {

  //turn on built in LED:
  digitalWrite(DEBUG_LED, HIGH);

  cmdSequence++;  // new request so we increment the sequence

  JSONVar requestMessage;
  requestMessage[String(FPSTR(MSG_ENUM_CMD))] = command;  // Command
  requestMessage[String(FPSTR(MSG_ENUM_CMD_SEQ))] = (unsigned long)cmdSequence;
  requestMessage[String(FPSTR(MSG_ENUM_CODE))] = 0;
  requestMessage[String(FPSTR(MSG_ENUM_ENDPOINT))] = endpoint;
  requestMessage[String(FPSTR(MSG_ENUM_LOGIN))] = login;
  requestMessage[String(FPSTR(MSG_ENUM_PASSWORD))] = password;
  requestMessage[String(FPSTR(MSG_ENUM_DATA))] = *data;
  // Print request to custom serial
  wifiModuleSerial.println(START_MESSAGE_BLOCK + JSON.stringify(requestMessage) + STOP_MESSAGE_BLOCK);
  Serial.println(START_MESSAGE_BLOCK + JSON.stringify(requestMessage) + STOP_MESSAGE_BLOCK);

  // Tag request as pending
  addToPendingCommands(cmdSequence, callbackID);

  //turn off built in LED:
  digitalWrite(DEBUG_LED, LOW);
}


void addToPendingCommands(const unsigned long messageSequenceID, const byte callbackID) {
  // Tag request as pending
  bool added = false;
  for (byte i = 0; i < RESPONSES_NBR_LIMIT; i++) {
    if (0 == pendingCommands[i].sequence) {
      pendingCommands[i].sequence = messageSequenceID;
      pendingCommands[i].callback = callbackID;
      pendingCommands[i].timeout = (millis() + responseTimeout);
      added = true;
      break;
    }
  }
  if (!added) {
    Serial.println(F("Too much commands, the oldest is deleted"));
    pendingCommands[0].sequence = messageSequenceID;
    pendingCommands[0].callback = callbackID;
    pendingCommands[0].timeout = (millis() + responseTimeout);
  }
}


// message management
void timeoutManagement() {
  // We check the pending command messages
  for (byte i = 0; i < RESPONSES_NBR_LIMIT; i++) {
    if (pendingCommands[i].sequence > 0) {
      if (pendingCommands[i].timeout < millis()) {
        // Timeout
        Serial.println(F("Timeout one command lost :"));
        Serial.print(pendingCommands[i].sequence);
        Serial.println();
        pendingCommands[i].sequence = 0;
      }
    }
  }
}


// If a "connection lost" event as been detected, we ask for auto-reconnect, if the demand is not already pending...
void connectionLostManagement() {
  bool isNetworkChangeCommandPending = false;
  for (byte i = 0; i < RESPONSES_NBR_LIMIT; i++) {
    if (0 != pendingCommands[i].sequence && 0 == pendingCommands[i].callback) {
      isNetworkChangeCommandPending = true;
      break;
    }
  }
  if(!isNetworkChangeCommandPending){
    // No connection and no response (not event -12 error callback)
    // Show fail signal then try again
    failLEDDance();
    initNetworkChange();
  }
}


// "Async" message read and build
void checkForWifiModuleMessages() {
  if (wifiModuleSerial.available() > 0) {
    char inChar = (char)wifiModuleSerial.read();
    if (!isReading && START_MESSAGE_BLOCK == inChar) {
      // the start of a message has been detected => we init a string buffer
      isReading = true;
      memset(messageBuffer, 0, MESSAGE_SIZE_LIMIT);
      messageCharIterator = 0;
    } else if (isReading && STOP_MESSAGE_BLOCK == inChar) {
      // the end of a message has been detected => we parse the message, keep the sequence up to date then empty the buffer
      isReading = false;
      JSONVar response = JSON.parse(messageBuffer);

      Serial.print(F("response received : "));
      Serial.print(messageBuffer);
      Serial.println();

      messageCharIterator = 0;

      unsigned long sequenceResp = (unsigned long)response[String(FPSTR(MSG_ENUM_CMD_SEQ))];
      // We store the highest sequence used
      if (sequenceResp > cmdSequence) {
        cmdSequence = sequenceResp;
      }

      for (byte i = 0; i < RESPONSES_NBR_LIMIT; i++) {
        if (sequenceResp == pendingCommands[i].sequence) {
            callbackManagement(pendingCommands[i].callback, response);
            // we delete the pending command
            pendingCommands[i].sequence = 0;
            break;
        }
      }

    } else if (isReading) {
      messageBuffer[messageCharIterator] = inChar; // we append the char to the string buffer
      messageCharIterator++;
    }
  }
}

//                                            ------------------
//                                            |command callbacks|
//                                            ------------------

void callbackManagement(byte callbackID, JSONVar response) {
  
  if (0 == callbackID) {
    networkChangeCallback(response);
  } else if (1 == callbackID) {
    postSensorDataCallback(response);
  } else {
    // Default callBack
    Serial.println(F("Default Response from wifiModule to command :"));
    Serial.println(String((unsigned long)response[String(FPSTR(MSG_ENUM_CMD_SEQ))]));
  }
}


void postSensorDataCallback(JSONVar response) {

  int code = (int)response[String(FPSTR(MSG_ENUM_CODE))];
  // callback for sensor data post to server
  if (200 == code) {
    Serial.println(F("Post sensor Data OK"));
  } else if (-12 == code) {
    // Probably connection lost wait 5 sec and try reconnect
    isConnectionOk = false;
    Serial.println(F("Post sensor Data error code : "));
    Serial.println(code);
    if(showInitLedDance){
     failLEDDance();
    }
  } else {
    Serial.println(F("Post sensor Data error code : "));
    Serial.println(code);
  }
}


void networkChangeCallback(JSONVar response) {
  // callback for sensor data post to server
  int code = (int)response[String(FPSTR(MSG_ENUM_CODE))];
  if (1 == code) {
    isConnectionOk = true;
    Serial.print(F("Network change OK, connected to "));
    Serial.print((const char*)response[String(FPSTR(MSG_ENUM_LOGIN))]);
    Serial.println();
    if(showInitLedDance){
      successfulLEDDance();
    }
  } else {
    isConnectionOk = false;
    Serial.println(F("Network change failed, please check the WIFI password"));
    if(showInitLedDance){
     failLEDDance();
    }
  }
}


// Interrupt handler for SERCOM2
void SERCOM2_Handler() {
  wifiModuleSerial.IrqHandler();
}
