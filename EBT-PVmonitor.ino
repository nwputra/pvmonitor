#include <ESP8266WiFi.h>
#include <time.h>
#include <SPI.h>
#include <Wire.h>
#include <BlynkSimpleEsp8266.h>
#include <ADS1115_WE.h>
#include "DHT.h"

#define DHTPIN D4		              // Digital pin connected to the DHT sensor
#define DHTTYPE DHT11	            // DHT 11

// Initialize DHT sensor.
//
DHT dht(DHTPIN, DHTTYPE);

// ADS1115
//
ADS1115_WE adc( 0x48 );

// Timer
//
BlynkTimer timer, tmconn;

// Blynk authentication and WiFi access
char auth[] = "your-auth-code";
char ssid[] = "SSID";
char pass[] = "PASSWORD";

unsigned char	ReCnctFlag  = 0;	 // reconnection Flag
unsigned char	ReCnctCount = 0;	 // reconnection Counter
int           v0, v1, v2;
float         iPV, vPV;
float         h, t, x;
unsigned char dhterr;

#define OFF 12                  // ACS offset error


float readChannel(ADS1115_MUX channel)
{
  float voltage = 0.0;

  adc.setCompareChannels(channel);
  adc.startSingleMeasurement();

  while (adc.isBusy()) {}

  voltage = adc.getResult_mV();
  return voltage;
}


void timerCallback()
{
  unsigned char state, sec;
  char buff[6];
  int rssi;
  

  if ( ++sec == 60 )
  {
    sec = 0;
  }

  state = sec % 5;

  switch ( state )
  {
    case 0 : // WiFi
      rssi = WiFi.RSSI();
      Serial.print("Signal      = ");
      Serial.println( rssi );
      Serial.println("");
      break;

    case 1 : // Read DHT Temperature
      //
      x = dht.readTemperature();
      if ( isnan(x) )
      {
        Serial.println(F("Failed to read Temperature from DHT sensor!"));
        dhterr++;
      }
      else
      {
        if ( dhterr )
        {
          // Do not use the humidity reading if
          // temperature reading has failed
          dhterr = 0;
        }
        else
        {
          // No error, value is valid to be used
          t = x;
        }
      }
      Serial.print("Temperature = ");
      Serial.print(t);
      Serial.println(" C");
      Blynk.virtualWrite(V0, t);
      break;

    case 2 : // Read DHT Humidity
      //
      x = dht.readHumidity();
      if ( isnan(x) )
      {
        Serial.println(F("Failed to read Humidity from DHT sensor!"));
        dhterr++;
      }
      else
      {
        if ( dhterr )
        {
          // Do not use the humidity reading if
          // temperature reading has failed
          dhterr = 0;
        }
        else
        {
          // No error, value is valid to be used
          h = x;
        }
      }
      Serial.print("Humidity    = ");
      Serial.print(h);
      Serial.println(" %");
      Blynk.virtualWrite(V1, h);
      break;

    case 3 : // Read ADS1115
      // ACS
      v0 = readChannel(ADS1115_COMP_0_GND);

      // VCC
      v1 = readChannel(ADS1115_COMP_1_GND);

      // PV
      v2 = readChannel(ADS1115_COMP_2_GND);

      // Compute the voltage and current
      vPV = 0.038 * v2;
      iPV = 0.01 * (v0 - (v1 / 2 - OFF));

      Serial.print("Voltage     = ");
      Serial.print(v0);
      Serial.print(" / ");
      Serial.print(v1);
      Serial.print(" / ");
      Serial.println(v2);

      Serial.print("PV          = ");
      Serial.print(vPV);
      Serial.print(" / ");
      Serial.println(iPV);

      Blynk.virtualWrite(V2, vPV);
      Blynk.virtualWrite(V3, iPV);

      break;
  }
}



void setup()
{
  Serial.begin(115200);
  Blynk.begin(auth, ssid, pass, "blynk-cloud.com", 8080);

  dht.begin();
  dhterr = 0;

  if ( !adc.init() )
  {
    Serial.println("ADC not connected!");
    ESP.restart();
  }
  else
  {
    adc.setVoltageRange_mV(ADS1115_RANGE_6144);
    Serial.println("ADC Ready");
  }

  timer.setInterval( 1000, timerCallback );
}


void loop()
{
  if (Blynk.connected())
  {
    // If connected run as normal
    Blynk.run();
  }
  else if (ReCnctFlag == 0)
  {
    //set timer to try to reconnect in 30 seconds
    ReCnctFlag = 1;				// Set reconnection Flag
    Serial.println("Starting reconnection timer in 15 seconds...");
    tmconn.setTimeout(15000L, []() {
      // Lambda Reconnection Timer Function
      ReCnctFlag = 0;			// Reset reconnection Flag
      ReCnctCount++;			// Increment reconnection Counter
      Serial.print("Attempting reconnection #");
      Serial.println(ReCnctCount);
      Blynk.connect();		// Try to reconnect to the server
    });					          // END Timer Function
  }
  tmconn.run();
  timer.run();
}
