// Include the libraries we need

#include <ArduinoJson.h>
#include <ESP_EEPROM.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>

#include <OneWire.h>
#include <DallasTemperature.h>

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

const char *ssid="***REMOVED***";
const char *password="***REMOVED***";
const char* mqtt_server = "***REMOVED***";

WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
long lastReconnectAttempt = 0;
long lastScreenUpdate = 0;
long lastTempHistoryUpdate = 0;
char msg[500];
int value = 0;

StaticJsonDocument<500> doc;
JsonObject root = doc.to<JsonObject>();

//const char* tempPublishTopic = "keggerators/1/out/temperature";
//const char* setpointPublishTopic = "keggerators/1/out/setpoint";
//const char* fanPublishTopic = "keggerators/1/out/fan";
//const char* doorpublishTopic = "keggerators/1/out/door";
const char* setpointSubscribeTopic = "keggerators/1/in/setpoint";
const char* upHysteriesisSubscribeTopic = "keggerators/1/in/uphysteresis";
const char* downHysteriesisSubscribeTopic = "keggerators/1/in/downhysteresis";
const char* outTopic = "keggerators/1/out";

#define OLED_RESET 4
Adafruit_SSD1306 display(OLED_RESET);

// Data wire is plugged into D3 on ESP8266 NodeMCU
#define ONE_WIRE_BUS 0

//Button 1 is plugged into D5 on NodeMCU
#define UP_BUTTON_PIN 14
//Button 2 is plugged into D6 on NodeMCU
#define DOWN_BUTTON_PIN 12
//door switch is plugged into XX on NodeMCU
#define DOOR_SWITCH_PIN 16
//Fan input is connected to XX on NodeMCU
#define FAN_PULSE_PIN 9
#define DEBOUNCE_TIME 5
volatile bool upButtonState;
volatile bool downButtonState;
volatile long upLastDebounceTime = 0;
volatile long downLastDebounceTime = 0;
volatile unsigned long fanPulseCount = 0;

os_timer_t upDebounceTimer;
os_timer_t downDebounceTimer;

os_timer_t temperatureUpdateTimer;
os_timer_t screenUpdateTimer;
os_timer_t tempGraphUpdateTimer;
os_timer_t fanAndDoorUpdateTimer;

#define FREEZER_RELAY_PIN 13

os_timer_t fridgeControlTimer;


// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);
// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);

//Data for temp graph
#define MAX_TEMP_HISTORY 128
#define INVALID_TEMP 1000.0
double tempHistory[MAX_TEMP_HISTORY];
double maxTemp = INVALID_TEMP;
double minTemp = INVALID_TEMP;

//stores current states
volatile double _setPoint = 40.0;
volatile double _currentTemp = 40.0;
volatile double _upHysteresis = 1.0;
volatile double _downHysteresis = 1.0;
volatile double _lastSetpoint = 0.0;
volatile double _lastUpHysteresis = 0.0;
volatile double _lastDownHysteresis = 0.0;
volatile bool _fanStatus = true;
volatile bool _relayState = false;
volatile bool _doorState = true;

double screenLastSetPoint = -10000.0;
double screenLastCurrentTemp = -10000.0;
bool screenLastFanStatus = false;
bool screenLastRelayState = false;
bool screenLastMQTTState = false;
bool screenGraphUpdate = false;
bool screenLastDoorState = false;

void setup()   {     
  pinMode(UP_BUTTON_PIN, INPUT);
  pinMode(DOWN_BUTTON_PIN, INPUT); 
  pinMode(DOOR_SWITCH_PIN, INPUT);
  pinMode(FAN_PULSE_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(UP_BUTTON_PIN), upInterrupt, CHANGE);   
  attachInterrupt(digitalPinToInterrupt(DOWN_BUTTON_PIN), downInterrupt, CHANGE);   
  attachInterrupt(digitalPinToInterrupt(FAN_PULSE_PIN), fanPulseInterrupt, RISING); 
  pinMode(FREEZER_RELAY_PIN, OUTPUT);
  digitalWrite(FREEZER_RELAY_PIN, HIGH);
  //Serial.begin(9600);
  // Start up the library
  sensors.begin();

  // by default, we'll generate the high voltage from the 3.3v line internally! (neat!)
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3D (for the 128x64)
  // init done
  display.clearDisplay();
  // text display tests
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0,0);
  //reset temp history
  for(int i = 0; i < MAX_TEMP_HISTORY; i++)
  {
    tempHistory[i] = INVALID_TEMP;
  }

  display.println("Connecting to Wifi!");
  display.print("SSID: "); 
  display.println(ssid);
  display.display();

  WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    display.print(".");
    display.display();
  }

  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Wifi Connected!");
  display.display();
  delay(1000);
  display.clearDisplay();
  display.display();

  client.setServer(mqtt_server, 1883);
  client.setCallback(mqtt_callback);

  sensors.requestTemperatures(); // get the first temperature
  _currentTemp = sensors.getTempFByIndex(0);

  EEPROM.begin(24); //4 doubles

  EEPROM.get(0, _setPoint);
  EEPROM.get(8, _upHysteresis);
  EEPROM.get(16, _downHysteresis);

  _lastSetpoint = _setPoint;
  _lastUpHysteresis = _upHysteresis;
  _lastDownHysteresis = _downHysteresis;

  os_timer_setfn(&temperatureUpdateTimer, updateTemperatureCallback, NULL);
  os_timer_arm(&temperatureUpdateTimer, 1000, true);
  os_timer_setfn(&screenUpdateTimer, updateScreenCallback, NULL);
  os_timer_arm(&screenUpdateTimer, 250, true);  
  os_timer_setfn(&tempGraphUpdateTimer, updateTempGraphCallback, NULL); 
  os_timer_arm(&tempGraphUpdateTimer, 168750, true);  
  os_timer_setfn(&fridgeControlTimer, updateFridgeControlStatus, NULL);
  os_timer_arm(&fridgeControlTimer, 1000, true);
  os_timer_setfn(&fanAndDoorUpdateTimer, fanAndDoorUpdateCallback, NULL);
  os_timer_arm(&fanAndDoorUpdateTimer, 250, true);
}


void loop() {
  if (client.loop()) 
  {
    if (millis() >= lastMsg) 
    {
      lastMsg = millis() + 5000;
      //instead of this, send a big JSON
      publishStatus();

      //determine if eeprom needs to be written
      bool writeEEPROM = false;
      if(_setPoint != _lastSetpoint)
      {
        _lastSetpoint = _setPoint;
        EEPROM.put(0, _setPoint);
        writeEEPROM = true;
      }

      if(_upHysteresis != _lastUpHysteresis)
      {
        _lastUpHysteresis = _upHysteresis;
        EEPROM.put(8, _upHysteresis);
        writeEEPROM = true;
      }

      if(_downHysteresis != _lastDownHysteresis)
      {
        _lastDownHysteresis = _downHysteresis;
        EEPROM.put(16, _downHysteresis);
        writeEEPROM = true;
      }

      if(writeEEPROM)
      {
        EEPROM.commit();
      }
    }
  } 
  else
  {
    if (millis() >= lastReconnectAttempt) 
    {
      lastReconnectAttempt = millis() + 5000;
      //Serial.println("Disconnected!");
      // Attempt to reconnect
      if (reconnect()) {
        lastReconnectAttempt = 0;
      }
    }
  }
  //end of main loop
  delay(1);
}

void publishStatus(void)
{
  root["temp"] = _currentTemp;
  root["setpoint"] = _setPoint;
  root["units"] = "F";
  root["UpHysteresis"] = _upHysteresis;
  root["DownHysteresis"] = _downHysteresis;
  root["RelayState"] = _relayState;
  root["FanState"] = _fanStatus;
  root["DoorState"] = _doorState;

  serializeJson(root, msg);
  //Serial.println(msg);
  client.publish(outTopic, msg);
}

void mqtt_callback(char* topic, byte* payload, unsigned int length) {
//  Serial.print("Message arrived [");
//  Serial.print(topic);
//  Serial.print("] ");
//  for (int i = 0; i < length; i++) {
//    Serial.print((char)payload[i]);
//  }
//  Serial.println();

  payload[length] = '\0';

  if(strcmp(topic, setpointSubscribeTopic) == 0)
  {
    char * pch;
    pch = strtok((char *)payload," ,");
    if(pch != NULL)
    {
      _setPoint = atof(pch);
      //Serial.println(pch);
    }
  }
  else if(strcmp(topic, upHysteriesisSubscribeTopic) == 0)
  {
    char * pch;
    pch = strtok((char *)payload," ,");
    if(pch != NULL)
    {
      _upHysteresis = atof(pch);
      //Serial.println(pch);
    }
  }
  else if(strcmp(topic, downHysteriesisSubscribeTopic) == 0)
  {
    char * pch;
    pch = strtok((char *)payload," ,");
    if(pch != NULL)
    {
      _downHysteresis = atof(pch);
      //Serial.println(pch);
    }
  }
}

bool reconnect() {
  // Attempt to connect
  client.disconnect();
  if (client.connect("Keggerator_1")) {
    publishStatus();
    client.subscribe(setpointSubscribeTopic);
    client.subscribe(upHysteriesisSubscribeTopic);
    client.subscribe(downHysteriesisSubscribeTopic);
  }
  return client.connected();
}

void upInterrupt()
{
  bool currentState = digitalRead(UP_BUTTON_PIN);
  if(upButtonState == currentState) return;

  bool debounce = false;

  if((millis() - upLastDebounceTime) <= DEBOUNCE_TIME)
  {
    debounce = true;
  }

  
  upLastDebounceTime = millis();

  if(debounce) return;

  upButtonState = currentState;

  //false on press
  if(upButtonState == false)
  {
    _setPoint = _setPoint + 0.1;
    
  }
}

void downInterrupt()
{
  bool currentState = digitalRead(DOWN_BUTTON_PIN);
  if(downButtonState == currentState) return;

  bool debounce = false;

  if((millis() - downLastDebounceTime) <= DEBOUNCE_TIME)
  {
    debounce = true;
  }

  
  downLastDebounceTime = millis();

  if(debounce) return;

  downButtonState = currentState;

  //false on press
  if(downButtonState == false)
  {
    _setPoint = _setPoint - 0.1;
  }
}

void fanPulseInterrupt()
{
  fanPulseCount++;
}

void updateTemperatureCallback(void *pArg)
{
  sensors.requestTemperatures(); // Send the command to get temperatures
  _currentTemp = sensors.getTempFByIndex(0);
}

void fanAndDoorUpdateCallback(void *pArg)
{
  _doorState = !digitalRead(DOOR_SWITCH_PIN);

  if(fanPulseCount > 0)
  {
    _fanStatus = true;
  }
  else
  {
    _fanStatus = false;
  }

  fanPulseCount = 0;
}

void updateScreenCallback(void *pArg)
{ 
  if((screenLastSetPoint != _setPoint) ||
     (screenLastCurrentTemp != _currentTemp) ||
     (screenLastFanStatus != _fanStatus) ||
     (screenLastRelayState != _relayState) ||
     (screenLastMQTTState != client.connected()) ||
     (screenGraphUpdate == true) || 
     (screenLastDoorState != _doorState))
     {
        screenLastSetPoint = _setPoint;
        screenLastCurrentTemp = _currentTemp;
        screenLastFanStatus = _fanStatus;
        screenLastRelayState = _relayState;
        screenLastDoorState = _doorState;
        screenLastMQTTState = client.connected();
        screenGraphUpdate = false;
        updateDisplay(_currentTemp, _setPoint, _relayState, _fanStatus, _doorState, client.connected());
     }
}

void updateTempGraphCallback(void *pArg)
{
  updateGraph(_currentTemp);
  screenGraphUpdate = true;
}

void updateFridgeControlStatus(void *pArg)
{
  if(_currentTemp > _setPoint + _upHysteresis) //too warm
  {
    //relay on
    _relayState = true;
    digitalWrite(FREEZER_RELAY_PIN, LOW);
  }
  else if(_currentTemp < _setPoint - _downHysteresis) //too cold
  {
    //relay off
    _relayState = false;
    digitalWrite(FREEZER_RELAY_PIN, HIGH);
  }
}

void updateDisplay(double currentTemp, double setPoint, bool freezerState, bool fanState, bool doorState, bool mqttState) {
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(0, 0);
  display.printf("%0.1f", currentTemp);
  display.print((char)247);
  display.print("F");
  display.setTextSize(1);
  display.setCursor(0, 16);
  display.printf("Setpoint: %0.1f", setPoint);
  display.print((char)247);
  display.println("F");
  if(freezerState)
  {
    display.fillCircle(display.width()-6-1, 6, 6, WHITE);
  }
  else
  {
    display.drawCircle(display.width()-6-1, 6, 6, WHITE);
  }
  
  if(!fanState)
  {
    display.setTextSize(1);
    display.setCursor(0, 30);
    display.println("Fan Error!");
  }
  if(!mqttState)
  {
    display.setTextSize(1);
    display.setCursor(0, 45);
    display.println("MQTT Error!");
  }
  if(!doorState)
  {
    display.setTextSize(1);
    display.setCursor(0, 55);
    display.println("Door Open!");
  }

  //if we all good, display the graph
  if(doorState && mqttState && fanState)
  {
    displayGraph(0, 25, display.height()-25);
  }
  display.display();
}

void updateGraph(double inputTemp)
{
  minTemp = 10000.0;
  maxTemp = -10000.0;
  //skooch everything to the left
  double t_Temp;
  for(int i = 0; i < MAX_TEMP_HISTORY-1; i++)
  {
    tempHistory[i] = tempHistory[i+1];
    if(tempHistory[i] < INVALID_TEMP)
    {
      if(tempHistory[i] < minTemp) minTemp = tempHistory[i];
      if(tempHistory[i] > maxTemp) maxTemp = tempHistory[i];
    }
  }
  tempHistory[MAX_TEMP_HISTORY-1] = inputTemp;
  if(inputTemp < minTemp) minTemp = inputTemp;
  if(inputTemp > maxTemp) maxTemp = inputTemp;

//  for(int i = MAX_TEMP_HISTORY-10; i < MAX_TEMP_HISTORY; i ++)
//  {
//    Serial.print(tempHistory[i]);
//    Serial.print(",");
//  }
//  Serial.println("");
}

void displayGraph(int x, int y, int h)
{
  //draw the bounding rectangle
  display.drawRect(x, y, display.width(), h, WHITE);

  //calculate scaling
  double offset = minTemp + ((maxTemp - minTemp)/2.0);
  double scale = (double)(h-3)/(maxTemp - minTemp); //subtract two pixels for border
  if(scale > 5.0) scale = 5.0;
  //draw the graph
  for(int i = 1; i < display.width()-1; i++) //bound left and right border pixels
  {
    if(tempHistory[i] < 1000.0)
    {
      display.drawPixel(i, ((y + h - 2) - ((tempHistory[i] - minTemp)*scale)), WHITE); //subract one to offset upwads for border
    }
  }
}


