/*
  An Arduino sketch that:
    1) Measures the depth of water in a tank, using the DFRobot Model GL-136
       pressure sensor.
    2) Creates a web server to make the data available on the home network.

  The web server code was adapted from example code "WiFi Web Server LED Blink" 
  originally written by Tom Igoe and dated 25 Nov 2012.

 */

// Included Files
#include <WiFiNINA.h>
#include <ArduinoJson.h>
#include "arduino_secrets.h"

// Must #define SECRET_SSID and SECRET_PASS in "arduino_secrets.h"
// Once done, take care to NOT commit that file into a publicly-viewable space (e.g., GitHub)

// Defined Constants
#define DEPTH_SENSOR_INPUT_PIN  (A0)      // (--) - ADC pin to which we connect the sensor output
#define SENSE_RESISTOR          (500)     // Ohms
#define DEPTH_TO_CURRENT_SLOPE  (0.0032)  // A / m
#define DEPTH_TO_CURRENT_OFFSET (0.004)   // A
#define VOLTAGE_TO_ADC_SLOPE    (204.8)   // ADC counts / V

// Use the above constants to compute at compile time, the constants we need to directly convert
// ADC value into depth (m) and to check to for a low-current fault
#define MIN_VALID_ADC_READING   (VOLTAGE_TO_ADC_SLOPE*SENSE_RESISTOR*DEPTH_TO_CURRENT_OFFSET)
#define ADC_TO_DEPTH_SLOPE      (VOLTAGE_TO_ADC_SLOPE*SENSE_RESISTOR*DEPTH_TO_CURRENT_SLOPE)

#define ENABLE_WEB_SERVER

// Global Variables
WiFiServer webServer(80);
StaticJsonDocument<64> data;    // the object that stores our JSON data

// Function Prototypes
void setupWebServer();
void printWiFiStatus();
void processWebRequests();
float readDepthSensor();
float convertAdcToDepth( int adc );

// Setup (runs once after power-on or system reset)
void setup() 
{
  data["tankDepth"] = 0.0;
  data["units"] = "m";
  data["status"] = "OK";
  Serial.begin(9600);       // Initialize serial communication for debug purposes
#ifdef ENABLE_WEB_SERVER
  setupWebServer();         // Start web server and attempt to connect to WiFi
#endif
}

// Loop (runs forever after)
void loop() 
{
  // read the depth sensor voltage from the ADC
  int adcVal;
  float voltage;
  float depth;
  adcVal = analogRead(DEPTH_SENSOR_INPUT_PIN);
  depth = convertAdcToDepth(adcVal);
  
//  Serial.print("Min:0, Max:1024");
//  Serial.print("ADC:");
//  Serial.print(adcVal);
  Serial.print("Min:0.0, Max:5.0");
  Serial.print(", Depth:");
  Serial.println(depth);

#ifdef ENABLE_WEB_SERVER
  if( WiFi.status() != WL_CONNECTED ) {
    setupWebServer();
  }
  // Process any incoming web requests
  processWebRequests();
#endif
}

float convertAdcToDepth( int adc )
{

  float d;

  d = (adc - MIN_VALID_ADC_READING) / ADC_TO_DEPTH_SLOPE;

  if( adc < MIN_VALID_ADC_READING )
  {
    data["status"] = "Fault";
  }
  else
  {
    data["tankDepth"] = d;
    data["status"] = "OK";
  }

  return d;
}

void printWifiStatus() 
{
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your board's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
  // print where to go in a browser:
  Serial.print("To see this page in action, open a browser to http://");
  Serial.println(ip);
}

// Initialize the web server and attempt to connect to the local wifi network
// This function does not return until it succeeds, so if the connection 
// isn't possible, nothing else will happen.
void setupWebServer()
{
  char ssid[] = SECRET_SSID;        // your network SSID (name)
  char pass[] = SECRET_PASS;        // your network password (use for WPA, or use as key for WEP)
  int keyIndex = 0;                 // your network key index number (needed only for WEP)

  int status = WL_IDLE_STATUS;

  // check for the WiFi module:
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
    // don't continue
    while (true);
  }

  String fv = WiFi.firmwareVersion();
  if (fv < WIFI_FIRMWARE_LATEST_VERSION) {
    Serial.println("Please upgrade the firmware");
  }

  // attempt to connect to WiFi network:
  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to Network named: ");
    Serial.println(ssid);                   // print the network name (SSID);
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    status = WiFi.begin(ssid, pass);
    // Blink LED_BUILTIN while waiting 10 seconds for connection:
    for( int i=0; i<10; i++ ) {
        digitalWrite(LED_BUILTIN, HIGH);
        delay(20);
        digitalWrite(LED_BUILTIN, LOW);
        delay(980);
    }
  }
  webServer.begin();                           // start the web server on port 80
  printWifiStatus();                        // you're connected now, so print out the status
}

void processWebRequests()
{
  WiFiClient client = webServer.available();   // listen for incoming clients

  if (client) {                             // if you get a client,
    Serial.println("new client");           // print a message out the serial port
    String currentLine = "";                // make a String to hold incoming data from the client
    while (client.connected()) {            // loop while the client's connected
      if (client.available()) {             // if there's bytes to read from the client,
        char c = client.read();             // read a byte, then
        Serial.write(c);                    // print it out to the serial monitor

        // Check to see if the byte is a newline character
        if (c == '\n') 
        {
          // if the current line is blank, it means we got two newline characters in a row.
          // that signals the end of the client HTTP request, so send a response:
          if (currentLine.length() == 0) {
            // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
            // and a content-type so the client knows what's coming, then a blank line:
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println();

            // the content of the HTTP response follows the header. In this case we are going
            // to send the data in JSON format
            serializeJson(data, client);

            // The HTTP response ends with another blank line:
            client.println();
            // we are all done with this interaction, so 
            // break out of the while loop:
            break;
          } 
          else 
          { 
            // the currentLine was not blank, so the newline character means
            // we should start building a new line
            currentLine = "";
          }
        } 
        else if (c != '\r') 
        {  
          // if we got anyting else but a carriage return character, add it to the currentLine
          currentLine += c;
        }
      }
    }
    // close the connection:
    client.stop();
    Serial.println("client disconnected");
  }
}