// Lifted code direct from these fellows:
// http://www.altelectronics.co.uk/arduino-uno-based-oregon-scientific-v-2-1-sensor-emulator-transmission/
// http://playground.arduino.cc/main/DHT11Lib

/* Include the standard LiquidCrystal library */

/********** MEGA
#include <LiquidCrystal.h>
// Initiliase the LiquidCrystal library with the correct DIO pins 
#define LCD_DATA4 4         // LCD data DIO pin 4 
#define LCD_DATA5 5         // LCD data DIO pin 5 
#define LCD_DATA6 6         // LCD data DIO pin 6 
#define LCD_DATA7 7         // LCD data DIO pin 7 
#define LCD_RESET 8         // LCD Reset DIO pin 
#define LCD_ENABLE 9        // LCD Enable DIO pin 
#define LCD_BACKLIGHT 10    //LCD backlight DIO pin 
LiquidCrystal lcd(LCD_RESET, LCD_ENABLE, LCD_DATA4, LCD_DATA5, LCD_DATA6, LCD_DATA7);

#define RADIO 51
#define POWER 53
#define DHT_DATA 50 // D5
#define DHT_POWER 52 // D4

*/

///********* UNO
#define POWER 2  // D2
#define RADIO 3 // D3
#define DHT_POWER 4 // D4
#define DHT_DATA 5 // D5

byte device_ID = 0xB1;  // B0 = 170 , so B1 = bedroom 1 etc

#define LED 1       // LED PIN on board
//#define THN132N
#undef SERIAL

boolean light = false;
int value = 0;

const unsigned long TIME = 512;
const unsigned long TWOTIME = TIME * 2;

#define SEND_HIGH() digitalWrite(RADIO, HIGH)
#define SEND_LOW() digitalWrite(RADIO, LOW)

// Buffer for Oregon message
#ifdef THN132N
byte buff[8];
#else
byte buff[9];
#endif

/**
* \brief    Send logical “0? over RF
* \details  azero bit be represented by an off-to-on transition
* \         of the RF signal at the middle of a clock period.
* \         Remenber, the Oregon v2.1 protocol add an inverted bit first
*/
inline void sendZero(void)
{
  digitalWrite(LED, HIGH);
  SEND_HIGH();
  delayMicroseconds(TIME);
  SEND_LOW();
  delayMicroseconds(TWOTIME);
  SEND_HIGH();
  delayMicroseconds(TIME);
  digitalWrite(LED, LOW);
}

/**
* \brief    Send logical “1? over RF
* \details  a one bit be represented by an on-to-off transition
* \         of the RF signal at the middle of a clock period.
* \         Remenber, the Oregon v2.1 protocol add an inverted bit first
*/
inline void sendOne(void)
{
  digitalWrite(LED, HIGH);
  SEND_LOW();
  delayMicroseconds(TIME);
  SEND_HIGH();
  delayMicroseconds(TWOTIME);
  SEND_LOW();
  delayMicroseconds(TIME);
  digitalWrite(LED, LOW);
}

/**
* Send a bits quarter (4 bits = MSB from 8 bits value) over RF
*
* @param data Source data to process and sent
*/

/**
* \brief    Send a bits quarter (4 bits = MSB from 8 bits value) over RF
* \param    data   Data to send
*/
inline void sendQuarterMSB(const byte data)
{
  (bitRead(data, 4)) ? sendOne() : sendZero();
  (bitRead(data, 5)) ? sendOne() : sendZero();
  (bitRead(data, 6)) ? sendOne() : sendZero();
  (bitRead(data, 7)) ? sendOne() : sendZero();
}

/**
* \brief    Send a bits quarter (4 bits = LSB from 8 bits value) over RF
* \param    data   Data to send
*/
inline void sendQuarterLSB(const byte data)
{
  (bitRead(data, 0)) ? sendOne() : sendZero();
  (bitRead(data, 1)) ? sendOne() : sendZero();
  (bitRead(data, 2)) ? sendOne() : sendZero();
  (bitRead(data, 3)) ? sendOne() : sendZero();
}

/******************************************************************/
/******************************************************************/
/******************************************************************/

/**
* \brief    Send a buffer over RF
* \param    data   Data to send
* \param    size   size of data to send
*/
void sendData(byte *data, byte size)
{
  for (byte i = 0; i < size; ++i)
  {
    sendQuarterLSB(data[i]);
    sendQuarterMSB(data[i]);
  }
}

/**
* \brief    Send an Oregon message
* \param    data   The Oregon message
*/
void sendOregon(byte *data, byte size)
{
  sendPreamble();
  //sendSync();
  sendData(data, size);
  sendPostamble();
}

/**
* \brief    Send preamble
* \details  The preamble consists of 16 “1? bits
*/
inline void sendPreamble(void)
{
  byte PREAMBLE[] = { 0xFF, 0xFF };
  sendData(PREAMBLE, 2);
}

/**
* \brief    Send postamble
* \details  The postamble consists of 8 “0? bits
*/
inline void sendPostamble(void)
{
#ifdef THN132N
  sendQuarterLSB(0x00);
#else
  byte POSTAMBLE[] = { 0x00 };
  sendData(POSTAMBLE, 1);
#endif
}

/**
* \brief    Send sync nibble
* \details  The sync is 0xA. It is not use in this version since the sync nibble
* \         is include in the Oregon message to send.
*/
inline void sendSync(void)
{
  sendQuarterLSB(0xA);
}

/******************************************************************/
/******************************************************************/
/******************************************************************/

/**
* \brief    Set the sensor type
* \param    data       Oregon message
* \param    type       Sensor type
*/
inline void setType(byte *data, byte* type)
{
  data[0] = type[0];
  data[1] = type[1];
}

/**
* \brief    Set the sensor channel
* \param    data       Oregon message
* \param    channel    Sensor channel (0×10, 0×20, 0×30)
*/
inline void setChannel(byte *data, byte channel)
{
  data[2] = channel;
}

/**
* \brief    Set the sensor ID
* \param    data       Oregon message
* \param    ID         Sensor unique ID
*/
inline void setId(byte *data, byte ID)
{
  data[3] = ID;
}

/**
* \brief    Set the sensor battery level
* \param    data       Oregon message
* \param    level      Battery level (0 = low, 1 = high)
*/
void setBatteryLevel(byte *data, byte level)
{
  if (!level) data[4] = 0x0C;
  else data[4] = 0;
}

/**
* \brief    Set the sensor temperature
* \param    data       Oregon message
* \param    temp       the temperature
*/
void setTemperature(byte *data, float temp)
{
  // Set temperature sign
  if (temp < 0)
  {
    data[6] = 8;
    temp *= -1;
  }
  else
  {
    data[6] = 0;
  }

  // Determine decimal and float part
  int tempInt = (int)temp;
  int td = (int)(tempInt / 10);
  float tff = ((tempInt / 10.0) - (float)td) * 10.0;
  int tf = (int)round(tff);

  float tempFloatf = (temp - (float)tempInt) * 10.0;

  int tempFloat = (int)round(tempFloatf);

  // Set temperature decimal part
  data[5] = (td << 4);
  data[5] |= tf;

  // Set temperature float part
  data[4] |= (tempFloat << 4);
}

/**
* \brief    Set the sensor humidity
* \param    data       Oregon message
* \param    hum        the humidity
*/
void setHumidity(byte* data, byte hum)
{
  char humid[3];

  itoa(hum, humid, 10);


  //data[7] = (hum / 10);
  //data[6] |= (10 * (hum - data[7])) << 4;
  data[7] = humid[0];
  data[6] |= humid[1] << 4;
}

/**
* \brief    Sum data for checksum
* \param    count      number of bit to sum
* \param    data       Oregon message
*/
int Sum(byte count, const byte* data)
{
  int s = 0;

  for (byte i = 0; i < count; i++)
  {
    s += (data[i] & 0xF0) >> 4;
    s += (data[i] & 0xF);
  }

  if (int(count) != count)
    s += (data[count] & 0xF0) >> 4;

  return s;
}

/**
* \brief    Calculate checksum
* \param    data       Oregon message
*/
void calculateAndSetChecksum(byte* data)
{
#ifdef THN132N
  int s = ((Sum(6, data) + (data[6] & 0xF) - 0xA) & 0xFF);
  data[6] |= (s & 0x0F) << 4;     
  data[7] = (s & 0xF0) >> 4;
#else
  data[8] = ((Sum(8, data) - 0xA) & 0xFF);
#endif
}


#define DHTLIB_OK       0
#define DHTLIB_ERROR_CHECKSUM -1
#define DHTLIB_ERROR_TIMEOUT  -2

class dht11
{
public:
  int read(int pin);
  int humidity;
  int temperature;
};



////////////////////////////
/////////////////////
////////////// DHT
//////////////
///////////


int dht11::read(int pin)
{
  // BUFFER TO RECEIVE
  uint8_t bits[5];
  uint8_t cnt = 7;
  uint8_t idx = 0;

  // EMPTY BUFFER
  for (int i = 0; i < 5; i++) bits[i] = 0;

  // REQUEST SAMPLE
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
  delay(18);
  digitalWrite(pin, HIGH);
  delayMicroseconds(40);
  pinMode(pin, INPUT);

  // ACKNOWLEDGE or TIMEOUT
  unsigned int loopCnt = 10000;
  while (digitalRead(pin) == LOW)
  if (loopCnt-- == 0) return DHTLIB_ERROR_TIMEOUT;

  loopCnt = 10000;
  while (digitalRead(pin) == HIGH)
  if (loopCnt-- == 0) return DHTLIB_ERROR_TIMEOUT;

  // READ OUTPUT - 40 BITS => 5 BYTES or TIMEOUT
  for (int i = 0; i < 40; i++)
  {
    loopCnt = 10000;
    while (digitalRead(pin) == LOW)
    if (loopCnt-- == 0) return DHTLIB_ERROR_TIMEOUT;

    unsigned long t = micros();

    loopCnt = 10000;
    while (digitalRead(pin) == HIGH)
    if (loopCnt-- == 0) return DHTLIB_ERROR_TIMEOUT;

    if ((micros() - t) > 40) bits[idx] |= (1 << cnt);
    if (cnt == 0)   // next byte?
    {
      cnt = 7;    // restart at MSB
      idx++;      // next byte!
    }
    else cnt--;
  }

  // WRITE TO RIGHT VARS
  // as bits[1] and bits[3] are allways zero they are omitted in formulas.
  humidity = bits[0];
  temperature = bits[2];

  uint8_t sum = bits[0] + bits[2];

  if (bits[4] != sum) return DHTLIB_ERROR_CHECKSUM;
  return DHTLIB_OK;
}


/******************************************************************/
/******************************************************************/
/******************************************************************/

void setup()
{
  /* add setup code here */

#ifdef MEGA
  lcd.begin(16, 2);   // Set 16x2 LCD Display
  lcd.command(14);  // Turn on Cursor
#endif
  pinMode(LED, OUTPUT);
  pinMode(POWER, OUTPUT);
  pinMode(RADIO, OUTPUT);
  pinMode(DHT_POWER, OUTPUT);

  // Setup DHT & RADIO
  digitalWrite(RADIO, LOW);
  digitalWrite(POWER, LOW);
  digitalWrite(DHT_POWER, LOW);
#ifdef SERIAL
  Serial.begin(57600);
  Serial.println("\n[Oregon V2.1 encoder]");
#endif
  // Create the Oregon message for a temperature/humidity sensor (THGR2228N)
#ifdef THN132N 
  // Create the Oregon message for a temperature only sensor (TNHN132N)
  byte ID[] = { 0xEA, 0x4C };
#else
  // Create the Oregon message for a temperature/humidity sensor (THGR2228N)
  byte ID[] = { 0x1A, 0x2D };
#endif 
  setType(buff, ID);
  setChannel(buff, 0x11);
  setId(buff, device_ID);

  // Wait for DHT to setup
}

float temperature = 0.0;
float humidity = 0.0;
int chk = 0;

dht11 DHT11;

void loop()
{
  // Switch on the DHT and wait 2 seconds
  digitalWrite(DHT_POWER, HIGH);
  delay(2000);

  chk = DHT11.read(DHT_DATA);
#ifdef SERIAL    
  switch (chk) {
    case DHTLIB_OK:
      Serial.println("OK");
      break;
    case DHTLIB_ERROR_CHECKSUM:
      Serial.println("Checksum error");
      break;
    case DHTLIB_ERROR_TIMEOUT:
      Serial.println("Time out error");
      break;
    default:
    Serial.println("Unknown error");
  }
#endif

  if (chk == DHTLIB_OK)
  {
    temperature = DHT11.temperature;
    humidity = DHT11.humidity;

    // turn off the sensor
    digitalWrite(DHT_POWER, LOW);
#ifdef SERIAL
    Serial.print(temperature);
    Serial.print("C      ");
    Serial.print(humidity);
    Serial.print("%     ");
#endif
#ifdef MEGA 
    lcd.home();
    lcd.print("Temp: ");
    lcd.print(temperature);
    lcd.print("C");
    lcd.setCursor(0, 1);
    lcd.print(" Hum: ");
    lcd.print(humidity);
    lcd.print("%");
#endif
    // Get Temperature, humidity and battery level from sensors
    // (ie: 1wire DS18B20 for température, …)
    setBatteryLevel(buff, 1); // 0 : low, 1 : high
    setTemperature(buff, temperature);

#ifndef THN132N
    // Set Humidity
    setHumidity(buff, humidity);
#endif 

    // Calculate the checksum
    calculateAndSetChecksum(buff);

    // Show the Oregon Message
#ifdef SERIAL
    for (byte i = 0; i < sizeof(buff); ++i)   {
      Serial.print(buff[i] >> 4, HEX);
      Serial.print(buff[i] & 0x0F, HEX);
      Serial.println();
    }
#endif

    // Turn on the radio for transmit
    digitalWrite(POWER, HIGH);

    // Send the Message over RF
    sendOregon(buff, sizeof(buff));
    // Send a “pause”
    
    SEND_LOW();
    delayMicroseconds(TWOTIME * 8);
    // Send a copy of the first message. The v2.1 protocol send the
    // message two time
    sendOregon(buff, sizeof(buff));

    // Send low signal for 1s then turn off
    SEND_LOW();
    delay(1000);
    digitalWrite(POWER, LOW);
  }
  // Wait for 300 seconds before we start again
  delay(296 * 1000); 
}
