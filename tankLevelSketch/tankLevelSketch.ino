/*
  An Arduino sketch that:
    1) Measures the depth of water in a tank, using the DFRobot Model GL-136
       pressure sensor.
    2) Creates a web server to make the data available on the home network.

  The web server code was adapted from example code "WiFi Web Server LED Blink" 
  written by Tom Igoe, 25 Nov 2012.
 */

// Included Files
#include <WiFiNINA.h>
#include "arduino_secrets.h"
// Must #define your SECRET_SSID and SECRET_PASS in "arduino_secrets.h"
// Once done, take care to NOT commit that file into a publicly-viewable space (e.g., GitHub)

// Global Variables
WiFiServer server(80);

// Function Prototypes
void setupWebServer();
void printWiFiStatus();
void processWebRequests();

// Setup (runs once at power-on or system reset)
void setup() 
{
  Serial.begin(9600);       // Initialize serial communication for debug purposes
  setupWebServer();         // Start web server and attempt to connect to WiFi
}

// Loop (runs forever after)
void loop() 
{
  // Read the sensor

  // Convert results to meaningful units (e.g., feet, gallons, % full)

  // Send the results to another computer on the network for logging

  // Process any incoming web requests
  processWebRequests();
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
// This function does not return until it succeeds.
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
  server.begin();                           // start the web server on port 80
  printWifiStatus();                        // you're connected now, so print out the status
}

void processWebRequests()
{
    WiFiClient client = server.available();   // listen for incoming clients

  if (client) {                             // if you get a client,
    Serial.println("new client");           // print a message out the serial port
    String currentLine = "";                // make a String to hold incoming data from the client
    while (client.connected()) {            // loop while the client's connected
      if (client.available()) {             // if there's bytes to read from the client,
        char c = client.read();             // read a byte, then
        Serial.write(c);                    // print it out to the serial monitor
        if (c == '\n') {                    // if the byte is a newline character

          // if the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          if (currentLine.length() == 0) {
            // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
            // and a content-type so the client knows what's coming, then a blank line:
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println();

            // the content of the HTTP response follows the header:
            client.print("Click <a href=\"/H\">here</a> turn the LED on pin 2 on<br>");
            client.print("Click <a href=\"/L\">here</a> turn the LED on pin 2 off<br>");

            // The HTTP response ends with another blank line:
            client.println();
            // break out of the while loop:
            break;
          } else {    // if you got a newline, then clear currentLine:
            currentLine = "";
          }
        } else if (c != '\r') {  // if you got anything else but a carriage return character,
          currentLine += c;      // add it to the end of the currentLine
        }

        // Check to see if the client request was "GET /H" or "GET /L":
        if (currentLine.endsWith("GET /H")) {
          // digitalWrite(ledPin, HIGH);               // GET /H turns the LED on
        }
        if (currentLine.endsWith("GET /L")) {
          // digitalWrite(ledPin, LOW);                // GET /L turns the LED off
        }
      }
    }
    // close the connection:
    client.stop();
    Serial.println("client disconnected");
  }
}