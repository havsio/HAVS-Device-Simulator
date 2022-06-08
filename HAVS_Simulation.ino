#include <ArduinoBLE.h>

const char *deviceId = "HAVS-Protect-SIM";

float f = 11; 
const short fourierLoad = 300;

bool haveSentHz = false;
bool iThinkImConnected = false;

#define NUMBER_OF_SENSORS 2
union multi_sensor_data
{
  struct __attribute__((packed))
  {
    float values[NUMBER_OF_SENSORS];
  };
  uint8_t bytes[NUMBER_OF_SENSORS * sizeof(float)];
};
union multi_sensor_data multiSensorData;

unsigned long now = 0;

float acc_x_wog = 0;
float acc_y_wog = 0;
float acc_z_wog = 0;

BLEService havsService("19B10010-E8F2-537E-4F6C-D104768A1214");
BLECharacteristic accAndMsCharacteristic("2713", BLERead | BLENotify, sizeof multiSensorData.bytes);  // accelerations and millisecs 0x2713
BLEFloatCharacteristic btHz("2722", BLERead | BLENotify);                             // hertz 0x2722 sent to app
BLEIntCharacteristic switchCharacteristic("2A57", BLERead | BLEWrite);                // point level, sent from app, 0-4
BLEIntCharacteristic decibelCharacteristic("2B7D", BLERead | BLEWrite);               // decibel value sent to app
BLEIntCharacteristic tempCharacteristic("2A6E", BLERead | BLEWrite);                  // temperature sent to app

bool hasTurnedOffBlink = false;
long loop_timer_now = millis();  // holds the current micros
long previous_millis = millis(); // holds the previous micros
long connected_time = 0; // holds the time we have had a BLE connection
long lastColorOn = 0;
long lastColorOff = 1;
long colorLightTime = 400;
long colorLightTimeOff = 2000;
short pos = 0;
int *dataFreqWeighted = 0;
int *dataOriginalAcceleration = 0;

void blePeripheralConnectHandler(BLEDevice dev)
{
  Serial.println("Connected event, peripheral: ");
  Serial.println(dev.address());
  connected_time = millis();
  iThinkImConnected = true;
  BLE.stopAdvertise();
}

void blePeripheralDisconnectHandler(BLEDevice dev)
{
  Serial.println("Disconnected event, peripheral: ");
  Serial.println(dev.address());

  Serial.println("I'm resetting...");
  delay(2000);
  NVIC_SystemReset();
}


void setup()
{

  digitalWrite(LED_PWR, LOW); // "ON" LED turned off:

  Serial.begin(115200);
  Serial.println("Starting simulation");

  if (!BLE.begin())
  {
    Serial.println("Starting BLE failed!");
    while (1)
      ;
  }

  BLE.setDeviceName(deviceId);
  BLE.setLocalName(deviceId);
  BLE.setAdvertisedService(havsService); // add the service UUID
  havsService.addCharacteristic(accAndMsCharacteristic);
  havsService.addCharacteristic(btHz);
  havsService.addCharacteristic(switchCharacteristic);

  BLE.addService(havsService);

  BLE.setEventHandler(BLEConnected, blePeripheralConnectHandler);
  BLE.setEventHandler(BLEDisconnected, blePeripheralDisconnectHandler);

  BLE.advertise();

  dataFreqWeighted = new int[fourierLoad];
  dataOriginalAcceleration = new int[fourierLoad];


}

void loop()
{

  BLEDevice central = BLE.central();
  
  if (!central.connected())
  {
    central.disconnect();
    if (iThinkImConnected)
    {
      restartBecauseImNotConnected();
    }
    else
    {
      // we are sleeping 3 secs here
      blinkBlue();
    }
  }
  else // central.connected()
  {
    sendHzToApp();
    setColor();
    restartBecauseNoColorForAWhile(); 

    acc_x_wog = random(0, 15);
    acc_y_wog = random(0, 12);
    acc_z_wog = random(0, 2);
    delay(10);
    
    float originalAcceleration = sqrt(pow(acc_x_wog, 2) + pow(acc_y_wog, 2) + pow(acc_z_wog, 2));

    if (abs(originalAcceleration) > 2 || pos > 0)
    {
      float freqWeightedAcceleration = originalAcceleration;

      if(pos == 0) // set start time
        previous_millis = millis();
        
      dataFreqWeighted[pos] = round(freqWeightedAcceleration * 100.00);
      dataOriginalAcceleration[pos] = round(originalAcceleration);

      if (pos < fourierLoad - 1)
      {
        pos++;
      }
      else if (pos == fourierLoad - 1)
      {            
        sendArrayToApp();
      } 
    }
    else
    {
      if(pos > 0){
        // send what we've got so far
        sendArrayToApp();          
      }
      else
        clearArrays(); // so we don't get a long time value if we don't save any value for a period
    }
  }
}
void sendArrayToApp(){
  if(pos > 1)
  {
    // time to send the batch
    loop_timer_now = millis();
    float deltaTime = loop_timer_now - previous_millis;
    float avgValueOriginal = 0.0000;

    for (short i = 0; i < pos; i++)
    {
      avgValueOriginal += (dataOriginalAcceleration[i]);
    }
    avgValueOriginal = (avgValueOriginal / (pos-1));

    float avgValue = 0.0000f;
    for (short i = 0; i < pos; i++)
    {
      avgValue += (dataFreqWeighted[i] / 100.00);
    }

    avgValue = (avgValue / (pos-1));

    Serial.println("Weighted: " + String(avgValue) + " m/s^2\ttime: " + String(deltaTime) + "\traw: " + String(avgValueOriginal) + "m/s^2\tfrequence: " + String(f) + "\tarraySize: " + String(pos-1));

    sendToPhone(avgValue, deltaTime);
  }
  clearArrays();
}

void turnOffBlink(){
  if (!hasTurnedOffBlink)
  {
    hasTurnedOffBlink = true;

    digitalWrite(LEDR, HIGH);
    digitalWrite(LEDG, HIGH);
    digitalWrite(LEDB, HIGH);
  }  
}

void sendHzToApp(){
 
    if (!haveSentHz && (connected_time > 0 && millis() - connected_time > 2000))
    {
      int hzz2 = 450;
      if (hzz2 > 0)
      {
        btHz.writeValue(hzz2);
        haveSentHz = true;
      }
    }
    
}

void restartBecauseImNotConnected(){
  #ifdef DEBUG
    Serial.println("I think I'm connected.. but I'm not... delaying 3 sec");
  #endif
  delay(3000);

  //  if (!(central.connected()))
  //  {
    // #ifdef DEBUG
    //   Serial.println("Still not connected - RESETTING");
    // #endif
    NVIC_SystemReset();
  // }
}

void restartBecauseNoColorForAWhile(){
  // restart if we don't get a color from app
  if (iThinkImConnected && lastColorOn > 0 && millis() - lastColorOn > 60000)
  {
    Serial.println("No color for a while and I think I'm connected.. resetting...");
    delay(2000);
    NVIC_SystemReset();
  }  
}

void sendToPhone(float avgValue, float deltaTime)
{
  multiSensorData.values[0] = avgValue;
  multiSensorData.values[1] = deltaTime;

  accAndMsCharacteristic.writeValue(multiSensorData.bytes, sizeof multiSensorData.bytes);
  clearArrays();
}

void clearArrays()
{
  memset(dataFreqWeighted, 0, sizeof(dataFreqWeighted));
  memset(dataOriginalAcceleration, 0, sizeof(dataOriginalAcceleration));
  pos = 0;
  previous_millis = loop_timer_now;
  loop_timer_now = millis();
}

void blinkBlue()
{

  hasTurnedOffBlink = false;
  digitalWrite(LEDR, HIGH);
  digitalWrite(LEDG, HIGH);
  digitalWrite(LEDB, LOW);
  delay(200);

  digitalWrite(LEDR, HIGH);
  digitalWrite(LEDG, HIGH);
  digitalWrite(LEDB, HIGH);
  delay(2800);

}
void setColor()
{

  turnOffBlink();
  
  if (lastColorOn > lastColorOff)
  {
    // we are on - check how long we have been on
    if (millis() - lastColorOn >= colorLightTime)
    {
      // time to shut off light
      digitalWrite(LEDR, HIGH);
      digitalWrite(LEDG, HIGH);
      digitalWrite(LEDB, HIGH);
      lastColorOff = millis();
    }
  }
  else
  {
    if (lastColorOff == 1 || millis() - lastColorOff >= colorLightTimeOff)
    {
      // time to turn lights on
      // if (switchCharacteristic.written()) {
      switch (switchCharacteristic.value())
      {
      case 0:
      default:
        // Serial.println("Green LED on");
        digitalWrite(LEDR, HIGH);
        digitalWrite(LEDG, LOW);
        digitalWrite(LEDB, HIGH);
        lastColorOn = millis();
        colorLightTimeOff = 2000;
        break;
      case 1:
        digitalWrite(LEDR, HIGH);
        digitalWrite(LEDG, LOW);
        digitalWrite(LEDB, HIGH);
        colorLightTimeOff = 0;
        lastColorOn = millis();
        break;
      case 2:
        // Serial.println("white/YELLOW LED on");
        digitalWrite(LEDR, LOW);
        digitalWrite(LEDG, LOW);
        digitalWrite(LEDB, LOW);
        lastColorOn = millis();
        colorLightTimeOff = 2000;
        break;
      case 3:
        // Serial.println("ORANGE LED on");
        digitalWrite(LEDR, LOW);
        digitalWrite(LEDG, LOW);
        digitalWrite(LEDB, HIGH);
        lastColorOn = millis();
        colorLightTimeOff = 2000;
        break;
      case 4:
        // Serial.println("Red LED on");
        digitalWrite(LEDR, LOW);
        digitalWrite(LEDG, HIGH);
        digitalWrite(LEDB, HIGH);
        lastColorOn = millis();
        colorLightTimeOff = 50;
        break;
      case 99:
        // Serial.println(F("LEDs off"));
        digitalWrite(LEDR, HIGH);
        digitalWrite(LEDG, HIGH);
        digitalWrite(LEDB, HIGH);
        lastColorOn = millis();
        colorLightTimeOff = 2000;
        break;
      }
      //}
    }
  }
}


