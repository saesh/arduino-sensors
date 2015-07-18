#include <SPI.h>
#include <Ethernet.h>
#include <EthernetUdp.h>
// https://github.com/adafruit/DHT-sensor-library
#include <DHT.h>

/*
 * Sensors Project Arduino sketch
 * used pins:
 * analog  0 : LDR
 * analog  1 : error LED
 * digital 2 : DHT
 */

// inputs
// analog inputs
#define LDRPIN 0
// digital inputs
#define DHTPIN 2
#define DHTTYPE DHT22

// ethernet
byte mac[] = { 0x90, 0xA2, 0xDA, 0x0D, 0x93, 0x5C };
// time from ntp over udp
unsigned int localPort = 8888; // local port to listen for UDP packets
IPAddress timeServer(78, 47, 148, 174); // pool.ntp.org NTP server
const int NTP_PACKET_SIZE= 48; // NTP time stamp is in the first 48 bytes of the message
byte packetBuffer[ NTP_PACKET_SIZE]; //buffer to hold incoming and outgoing packets
EthernetUDP Udp; // A UDP instance to let us send and receive packets over UDP
EthernetClient client;

// Webservice endpoints for POST requests: temperatur, humidity and light resistance
#define URI_TEMPERATURE     "/api/sensors/kabuff_lt_f678f/measurements"
#define URI_HUMIDITY        "/api/sensors/kabuff_rh_d32ca/measurements"
#define URI_LIGHTRESISTANCE "/api/sensors/kabuff_lr_ga23n/measurements"
String sensors_data_template = "{\"when\":\"$when\", \"value\":\"$value\", \"devicecode\":\"$devicecode\"}";
const String TEMPERATURE_DEVICECODE     = "kabuff_lt_f678f";
const String HUMIDITY_DEVICECODE        = "kabuff_rh_d32ca";
const String LIGHTRESISTANCE_DEVICECODE = "kabuff_lr_ga23n";

// analog outputs
int errorLEDPin = 1;

// setup required objects and variables
DHT dht(DHTPIN, DHTTYPE);
unsigned long lastMeasurement = 0; // when the last measurement was taken in unix time
unsigned long measurementInterval = 300000; // 5 minutes in milliseconds

void setup()
{
  // let the w5100 start up
  delay(50);
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for Leonardo only
  }

  //Serial.println("Initializing DHT22");
  dht.begin();

  //Serial.println("Initializing LDR");
  pinMode(LDRPIN, INPUT);

  //Serial.println("Initializing network");
  // disable sd chip
  pinMode(4,OUTPUT);
  digitalWrite(4,HIGH);
  delay(2000);
  // get ip address from router
  if (Ethernet.begin(mac) == 0) {
    Serial.println("Error: Ethernet could nor get an IP address.");
    // TODO: turn on red LED
  } else {
    Serial.print("Network initialized with IP: ");
    Serial.println(Ethernet.localIP());
  }
  Udp.begin(localPort);

  // take first measurement
  takeMeasurementAndPost();
}

void loop()
{
  unsigned long runTime = millis();

  // check if interval passed
  // millis resets after 50 days to 0
  if (runTime - lastMeasurement >= measurementInterval ||
      runTime < lastMeasurement) {
    // if yes read sensor values and post them
    takeMeasurementAndPost();
  }

  delay (1000);
}

void takeMeasurementAndPost() {
  lastMeasurement = millis();
  unsigned long now = unixTimestampNTP();

  float humidity    = dht.readHumidity();
  float temperature = dht.readTemperature();
  int   lightValue  = 1024 - analogRead(LDRPIN);

  // check if values are valid
  if (!isnan(humidity)) {
    char tbuff[6];
    dtostrf(humidity,3, 2, tbuff);
    String data;
    data += "";
    data += sensors_data_template;
    data.replace("$when", String(now));
    data.replace("$value", String(tbuff));
    data.replace("$devicecode", HUMIDITY_DEVICECODE);
    
    postData(URI_HUMIDITY, data);
  }

  if (!isnan(temperature)) {
    char tbuff[6];
    dtostrf(temperature,3, 2, tbuff);
    String data;
    data += "";
    data += sensors_data_template;
    data.replace("$when", String(now));
    data.replace("$value", String(tbuff));
    data.replace("$devicecode", TEMPERATURE_DEVICECODE);
    
    postData(URI_TEMPERATURE, data);
  }

  if (lightValue >= 0 && lightValue <= 1024) {
    String data;
    data += "";
    data += sensors_data_template;
    data.replace("$when", String(now));
    data.replace("$value", String(lightValue));
    data.replace("$devicecode", LIGHTRESISTANCE_DEVICECODE);
    
    postData(URI_LIGHTRESISTANCE, data);
  }
}

void postData(String uri, String data) {
  if (client.connect("example.com", 80)) {
    client.println("POST " + uri + " HTTP/1.1");
    client.println("User-Agent: Arduino Ethernet Sensors library");
    client.println("Host: example.com");
    client.println("Connnection: close");
    client.print("Content-Length: ");
    client.println(data.length());
    client.println("Content-Type: application/json");
    client.println();
    client.print(data);
    client.println();
  } else {
    Serial.println("connection failed.");
  }
  delay(100);
  if (client.connected()) {
    client.stop();
  }
}

// get the unix timestamp from ntp udp packet
unsigned long unixTimestampNTP()
{
  sendNTPpacket(timeServer);
  delay(1000);
  if ( Udp.parsePacket() ) {
      // We've received a packet, read the data from it
      Udp.read(packetBuffer,NTP_PACKET_SIZE);  // read the packet into the buffer

      //the timestamp starts at byte 40 of the received packet and is four bytes,
      // or two words, long. First, esxtract the two words:
      unsigned long highWord = word(packetBuffer[40], packetBuffer[41]);
      unsigned long lowWord = word(packetBuffer[42], packetBuffer[43]);
      // combine the four bytes (two words) into a long integer
      // this is NTP time (seconds since Jan 1 1900):
      unsigned long secsSince1900 = highWord << 16 | lowWord;

      // now convert NTP time into everyday time:
      // Unix time starts on Jan 1 1970. In seconds, that's 2208988800:
      const unsigned long seventyYears = 2208988800UL;
      // subtract seventy years:
      unsigned long now = secsSince1900 - seventyYears;
      return now;
  }
}

// send an NTP request to the time server at the given address
unsigned long sendNTPpacket(IPAddress& address)
{
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12]  = 49;
  packetBuffer[13]  = 0x4E;
  packetBuffer[14]  = 49;
  packetBuffer[15]  = 52;

  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:
  Udp.beginPacket(address, 123); //NTP requests are to port 123
  Udp.write(packetBuffer,NTP_PACKET_SIZE);
  Udp.endPacket();
}
