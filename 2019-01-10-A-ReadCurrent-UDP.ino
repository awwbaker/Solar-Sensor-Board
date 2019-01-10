
#include <WiFiEsp.h>
#include <WiFiEspClient.h>
#include <WiFiEspUdp.h>
#include <SPI.h>
#include <dht.h>

dht DHT;

int GreenLed = 13; 
int RedLed = 12; 

char msg[200];
String temp_str; 
String hum_str;
char temp[50];
char hum[50];


unsigned int SolarPanel01 = A0;
unsigned int SolarPanel02 = A1;
unsigned int SolarPanel03 = A2;
unsigned int SolarPanel04 = A3;
unsigned int SolarPanel05 = A4;
unsigned int ArrayVoltage = A5;
unsigned int SourceVoltage = A6;
unsigned int TempHumSensor = A7;
unsigned int SolarPanel06 = A8;


IPAddress server(192,168,100,74); //your RPi address, update as required
char ssid[] = "XXXXXX"; // your network SSID (name)
char pass[] = "XXXXXX"; // your network password
int status = WL_IDLE_STATUS; // the Wifi radio's status
unsigned int localPort = 55561;  // local port to listen on

float Vcc = 0;
float amps = 0;
float Vref  = 0; //read your Vcc voltage,typical voltage should be 5000mV(5.0V)
const int mVperAmp = 100; // use 185 for 5A Module, and 66 for 30A Module

WiFiEspUDP Udp;
WiFiEspClient espClient;


void setup() {
   // initialize serial for debugging
   Serial.begin(9600);
  
   
   pinMode(RedLed, OUTPUT); 
   pinMode(GreenLed, OUTPUT);
   
   pinMode(A0, INPUT);
   pinMode(A1, INPUT);
   pinMode(A2, INPUT);
   pinMode(A3, INPUT);
   pinMode(A4, INPUT);
   pinMode(A5, INPUT);   
   pinMode(A6, INPUT);
   pinMode(A7, INPUT);

   digitalWrite(RedLed, HIGH);
   digitalWrite(GreenLed, HIGH);
   delay(500);
   digitalWrite(RedLed, LOW);
   digitalWrite(GreenLed, LOW);
   delay(500);
   digitalWrite(RedLed, HIGH);
   digitalWrite(GreenLed, HIGH);
   delay(500);
   digitalWrite(RedLed, LOW);
   digitalWrite(GreenLed, LOW);
   

   // initialize serial for ESP module
   Serial1.begin(115200);
   // initialize ESP module
   WiFi.init(&Serial1);

   // check for the presence of the shield
   if (WiFi.status() == WL_NO_SHIELD) {
     Serial.println("WiFi shield not present");
    // don't continue
    while (true);
  }
   digitalWrite(RedLed, HIGH);

   // attempt to connect to WiFi network
   while ( status != WL_CONNECTED) {
     Serial.print("Attempting to connect to WPA SSID: ");
     Serial.println(ssid);
     status = WiFi.begin(ssid, pass);
   }
   digitalWrite(GreenLed, HIGH);

   // you're connected now, so print out the data
   Serial.println("You're connected to the network");
   Udp.begin(localPort);

}

long readVcc() {
   // Read 1.1V reference against AVcc
   // set the reference to Vcc and the measurement to the internal 1.1V reference
   #if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
   ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
   #elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
   ADMUX = _BV(MUX5) | _BV(MUX0);
   #elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
   ADMUX = _BV(MUX3) | _BV(MUX2);
   #else
   ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
   #endif

  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA, ADSC)); // measuring

  uint8_t low  = ADCL; // must read ADCL first – it then locks ADCH
  uint8_t high = ADCH; // unlocks both

  long result = (high << 8) | low;

  //result = 1125300L / result; 
  // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  result = 1095720L / result ;
  return result; // Vcc in millivolts
}


float readCurrent(unsigned int port){
 
   float Maxamps=0;
   float Minamps=0;
   float noise=0;
   
   float Voltage=0;
   float Current=0;
   float SensorCount=0;
   float TempSensorCount=0;

   float mVperAmp = 100;          // 66 for 30A Module
   int ACSoffset = 2300; 
   unsigned int ADCValue;
 
   Vcc = readVcc();   
   for (int i=0; i < 250; i++){
      if (port == SolarPanel01){
         SensorCount = analogRead(port)+68;
      } else if (port == SolarPanel02){
         SensorCount = analogRead(port)+56;
      } else if (port == SolarPanel03){
         SensorCount = analogRead(port)+75;
      } else if (port == SolarPanel04){
         SensorCount = analogRead(port)+40;
      } else if (port == SolarPanel05){
         SensorCount = analogRead(port)+59;
      } else if (port == SolarPanel06){
         SensorCount = analogRead(port)+96;
      }  
  
      mVperAmp = ((Vcc / 5000)*100)*.85;
      Voltage =  (((SensorCount / 1024.0) * Vcc) - (Vcc/2));
      amps = amps + (Voltage / mVperAmp);
      delay(10);

      TempSensorCount = TempSensorCount + SensorCount;
   }
   amps = amps / 250;
   
   SensorCount = TempSensorCount / 250;
   Serial.print("Port: " + String(port) + "  SensorCount: " + String(SensorCount));
   Serial.print(" Vcc: ");
   Serial.print(String(Vcc));

   Serial.print(" mVperAmp: ");
   Serial.print(String(mVperAmp));

   Serial.print(" Voltage: ");
   Serial.print(String(Voltage));

   Serial.print(" amps: ");
   Serial.print(String(amps));

   Serial.println("   ");
 
   delay(500);
  if (amps < 1) amps = 0;
   return amps;
}


void loop() {

    DHT.read11(TempHumSensor);
    
    Serial.print("Current humidity = ");
    Serial.print(DHT.humidity);
    Serial.print("%  ");
    Serial.print("temperature = ");
    Serial.print(DHT.temperature+273.15); 
    Serial.println("k  ");
    
   digitalWrite(RedLed, LOW);
   digitalWrite(GreenLed, LOW);
   float temp;
   float total = 0;    
   int valtemp=analogRead(SourceVoltage);
   temp=valtemp/4.092;
   float SensorSourceVoltage =(temp/10);
   Serial.print("valtemp = ");
   Serial.print(valtemp);
   Serial.print("  SensorSourceVoltage = ");
   Serial.println(SensorSourceVoltage);

   float ArrayVolts;
   valtemp=analogRead(ArrayVoltage);
   temp=valtemp/4.092;
   ArrayVolts =(temp/10);
   ArrayVolts = (ArrayVolts*2)*10;
   Serial.print("valtemp = ");
   Serial.print(valtemp);
   Serial.print("  SolarPanelVoltage = ");
   Serial.println(ArrayVolts);


   digitalWrite(RedLed, HIGH);
   digitalWrite(GreenLed, LOW);


    temp_str = "{\"updates\": [ {\"values\":[{\"path\": \"electrical.solar.sensor.voltage\",\"value\": ";
    temp_str = temp_str + String(SensorSourceVoltage) + " }]}],\"context\":\"vessels.self\"}";
    //Serial.println(temp_str);
    Udp.beginPacket(server, localPort);
    Udp.write(temp_str.c_str());
    Udp.endPacket();

    temp_str = "{\"updates\": [ {\"values\":[{\"path\": \"electrical.solar.array.voltage\",\"value\": ";
    temp_str = temp_str + String(ArrayVolts) + " }]}],\"context\":\"vessels.self\"}";
    //Serial.println(temp_str);
    Udp.beginPacket(server, localPort);
    Udp.write(temp_str.c_str());
    Udp.endPacket();
 
    temp = (DHT.temperature+273.15);
    temp_str = "{\"updates\": [ {\"values\":[{\"path\": \"electrical.solar.sensor.temperature\",\"value\": ";
    temp_str = temp_str + String(temp) + " }]}],\"context\":\"vessels.self\"}";
    //Serial.println(temp_str);
    Udp.beginPacket(server, localPort);
    Udp.write(temp_str.c_str());
    Udp.endPacket();

    temp = DHT.humidity/100;
    temp_str = "{\"updates\": [ {\"values\":[{\"path\": \"electrical.solar.sensor.humidity\",\"value\": ";
    temp_str = temp_str + String(temp) + " }]}],\"context\":\"vessels.self\"}";
 //   Serial.println(temp_str);
    Udp.beginPacket(server, localPort);
    Udp.write(temp_str.c_str());
    Udp.endPacket();

    temp = readCurrent(SolarPanel01);
    total = temp;
    temp_str = "{\"updates\": [ {\"values\":[{\"path\": \"electrical.solar.SolarPanel01.current\",\"value\": ";
    temp_str = temp_str + String(temp) + " }]}],\"context\":\"vessels.self\"}";
 //   Serial.println(temp_str);
    Udp.beginPacket(server, localPort);
    Udp.write(temp_str.c_str());
    Udp.endPacket();
    delay(100);

    temp = readCurrent(SolarPanel02);
    total = total + temp;
    temp_str = "{\"updates\": [ {\"values\":[{\"path\": \"electrical.solar.SolarPanel02.current\",\"value\": ";
    temp_str = temp_str + String(temp) + " }]}],\"context\":\"vessels.self\"}";
 //   Serial.println(temp_str);
    Udp.beginPacket(server, localPort);
    Udp.write(temp_str.c_str());
    Udp.endPacket();
    delay(100);

    temp = readCurrent(SolarPanel03);
    total = total + temp;
    temp_str = "{\"updates\": [ {\"values\":[{\"path\": \"electrical.solar.SolarPanel03.current\",\"value\": ";
    temp_str = temp_str + String(temp) + " }]}],\"context\":\"vessels.self\"}";
 //   Serial.println(temp_str);
    Udp.beginPacket(server, localPort);
    Udp.write(temp_str.c_str());
    Udp.endPacket();
    delay(100);

    temp = readCurrent(SolarPanel04);
    total = total + temp;
    temp_str = "{\"updates\": [ {\"values\":[{\"path\": \"electrical.solar.SolarPanel04.current\",\"value\": ";
    temp_str = temp_str + String(temp) + " }]}],\"context\":\"vessels.self\"}";
 //   Serial.println(temp_str);
    Udp.beginPacket(server, localPort);
    Udp.write(temp_str.c_str());
    Udp.endPacket();
    delay(100);

    temp = readCurrent(SolarPanel05);
    total = total + temp;
    temp_str = "{\"updates\": [ {\"values\":[{\"path\": \"electrical.solar.SolarPanel05.current\",\"value\": ";
    temp_str = temp_str + String(temp) + " }]}],\"context\":\"vessels.self\"}";
//    Serial.println(temp_str);
    Udp.beginPacket(server, localPort);
    Udp.write(temp_str.c_str());
    Udp.endPacket();
    delay(100);

    temp = readCurrent(SolarPanel06);
    total = total + temp;
    temp_str = "{\"updates\": [ {\"values\":[{\"path\": \"electrical.solar.SolarPanel06.current\",\"value\": ";
    temp_str = temp_str + String(temp) + " }]}],\"context\":\"vessels.self\"}";
//    Serial.println(temp_str);
    Udp.beginPacket(server, localPort);
    Udp.write(temp_str.c_str());
    Udp.endPacket();
    delay(100);

    temp_str = "{\"updates\": [ {\"values\":[{\"path\": \"electrical.solar.Total.current\",\"value\": ";
    temp_str = temp_str + String(total) + " }]}],\"context\":\"vessels.self\"}";
//    Serial.println(temp_str);
    Udp.beginPacket(server, localPort);
    Udp.write(temp_str.c_str());
    Udp.endPacket();


   Serial.print("Total Current: " + String(total));
   Serial.println("   ");
   digitalWrite(RedLed, HIGH);
   digitalWrite(GreenLed, HIGH);
   delay(2000);
   digitalWrite(RedLed, LOW);
   digitalWrite(GreenLed, LOW);

}
