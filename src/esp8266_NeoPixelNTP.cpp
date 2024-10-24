#include <NeoPixelBus.h>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <ESP8266mDNS.h>
#include <NTPClient.h>
#include <sys/time.h>
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <AsyncElegantOTA.h>

// uint16_t pixelCount = 60;
#define colorSaturation 250
#define NUMPIX 60
#define TIME_OFFSET 3600

uint8_t NEOPIXEL_GPIO_PORT = 13;

char ssid[] = "LRSystems"; //  your network SSID (name)
char pass[] = "passordet"; // your network password

// ESP8266WebServer server(80);
AsyncWebServer server(80);
const uint8_t PixelPin = 3; // make sure to set this to the correct pin, ignored for Esp8266

WiFiUDP udp;
NTPClient timeClient(udp, TIME_OFFSET); //+ 6 * 3600 + 30 * 60 + 30
NeoPixelBus<NeoGrbFeature, NeoEsp8266Uart1800KbpsMethod> strip(NUMPIX, PixelPin);
// NeoPixelBus<NeoGrbFeature, Neo400KbpsMethod> strip(PixelCount, 2);

// RgbColor red = RgbColor(colorSaturation, 0, 0);
// RgbColor green = RgbColor(0, colorSaturation, 0);
// RgbColor blue = RgbColor(0, 0, colorSaturation);
// RgbColor white = RgbColor(colorSaturation);
// RgbColor black = RgbColor(0);

int pixels_current_intensity_r[NUMPIX];
int pixels_current_intensity_g[NUMPIX];
int pixels_current_intensity_b[NUMPIX];

// 0 = green backlight, green/white hour, red minute, blue blinkfade second
// 1 = white backlight, red, wide hour, green minute, blue smoothfade second
byte colorSchema = 1;

// 0 = smoothfade
// 1 = blinkfade
// 2 = smoothfade * 4
// 3 = blinkfade with smoothfade tail
// 4 = show only current second
byte secondsSchema = 0;

byte backlightR = 2;
byte backlightG = 2;
byte backlightB = 2;
byte backlightHourR = 25;
byte backlightHourG = 25;
byte backlightHourB = 25;

int loglookup[255];

// Todo: char allerede unsigned?
unsigned char previousSecond;
unsigned char currentSecond;
unsigned char currentSecondPlusOne;
unsigned char currentMinute;
unsigned char currentMinutePlusOne;
unsigned char currentHour;
unsigned char currentHourPlusOne;
unsigned char currentHourMinusOne;

// byte led = 5;

void setupLogLookup()
{
  // coefficients
  double a = 9.7758463166360387E-01;
  double b = 0.021764299;
  for (int i = 0; i < 255; i++)
  {
    loglookup[i] = floor((a * exp(b * i) + .5)) - 1;
  }
}

void setup()
{
  // this resets all the neopixels to an off state
  strip.Begin();
  //  strip.Show();

  Serial.begin(9600);
  Serial.println();

  // We start by connecting to a WiFi network
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, pass);

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");

  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  timeClient.begin();

  // Set up mDNS responder:
  // - first argument is the domain name, in this example
  //   the fully-qualified domain name is "esp8266.local"
  // - second argument is the IP address to advertise
  //   we send our IP address on the WiFi network
  if (!MDNS.begin("clock"))
  {
    Serial.println("Error setting up MDNS responder!");
    //    while (1)
    //    {
    //      delay(1000);
    //    }
  }
  else
  {
    Serial.println("mDNS responder started");
    // Add service to MDNS-SD
    MDNS.addService("http", "tcp", 80);
  }

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(200, "text/plain", "Hi! I am ESP8266."); });

  AsyncElegantOTA.begin(&server); // Start ElegantOTA
  server.begin();
  Serial.println("HTTP server started");

  setupLogLookup();
}

int min(int a, int b)
{
  return a < b ? a : b;
}

void loop()
{
  if (timeClient.update())
  {
    timeval tv = {timeClient.getEpochTime(), 0};
    settimeofday(&tv, nullptr);
  }

  // Serial.println(timeClient.getFormattedTime());
  // unsigned long currentTime = timeClient.getEpochTime();
  // Clear strip, set backlight
  for (int i = 0; i < NUMPIX; i++)
  {
    pixels_current_intensity_r[i] = backlightR;
    pixels_current_intensity_g[i] = backlightG;
    pixels_current_intensity_b[i] = backlightB;
  }
  for (int i = 0; i < NUMPIX; i += 5)
  {
    pixels_current_intensity_r[i] = backlightR + backlightHourR;
    pixels_current_intensity_g[i] = backlightG + backlightHourG;
    pixels_current_intensity_b[i] = backlightB + backlightHourB;
  }

  timeval tim;
  gettimeofday(&tim, nullptr);

  // unsigned long millis1 = tim.tv_usec / 1000;
  unsigned long currentMilliSecond = tim.tv_usec / 1000; // millis1; // % 1000;
  time_t currentTimeSeconds = tim.tv_sec;                //(millis1 / 1000); // + timeClient.getEpochTime(); // secsSince1900;

  unsigned long currentSecond = currentTimeSeconds % 60;
  unsigned long currentSecondPlusOne = (currentTimeSeconds + 1) % 60;

  unsigned int currentMinute = (currentTimeSeconds / 60) % 60;
  unsigned int currentMinutePlusOne = ((currentTimeSeconds / 60) + 1) % 60;
  // unsigned int currentMinuteMinusOne = ((currentTime / 60) - 1) % 60; //(currentTime / 60);

  unsigned int currentHour = (currentTimeSeconds / 720) % 60; // currentMinute / 12;
  unsigned int currentHourPlusOne = ((currentTimeSeconds / 720) + 1) % 60;
  unsigned int currentHourMinusOne = ((currentTimeSeconds / 720) - 1) % 60;

  /*
     Serial.println(hourFormat12(currentTime));
     Serial.println(currentHourMinusOne);
     Serial.println(currentHour);
     Serial.println(currentHourPlusOne);
     Serial.println(currentMinute);
     Serial.println(currentMinutePlusOne);
    Serial.println(currentSecond);
     Serial.println();
  delay(100);
  */
  if (colorSchema == 0)
  {

    // pixels_current_intensity_r[currentHour]+=32;
    // pixels_current_intensity_b[currentHour]+=60;
    pixels_current_intensity_g[currentHour] += 128;
    /*    pixels_current_intensity_r[currentHourPlus]+=30;
        pixels_current_intensity_r[currentHourMinusOne]+=30;
        pixels_current_intensity_g[currentHourPlus]+=30;
        pixels_current_intensity_g[currentHourMinusOne]+=30;
        pixels_current_intensity_b[currentHourPlus]+=30;
        pixels_current_intensity_b[currentHourMinusOne]+=30;*/
    pixels_current_intensity_r[currentMinute % 60] += 128;
  }
  else if (colorSchema == 1)
  {
    pixels_current_intensity_r[currentHourMinusOne] = 2 * (30 - currentMinute / 2);
    pixels_current_intensity_r[currentHour] = 2 * 60;
    pixels_current_intensity_r[currentHourPlusOne] = 2 * (30 + currentMinute / 2);

    //    int value1 = ((60 - currentSecond) * loglookup[(125 - currentMilliSecond / 8)]) / 60;
    //    int value2 = (currentSecond * loglookup[(125 - currentMilliSecond / 8)]) / 60;
    //    int value1 = ((60 - currentSecond) * (1000 - currentMilliSecond)) / 1000;
    //    int value2 = (currentSecond * (1000 - currentMilliSecond)) / 1000;
    pixels_current_intensity_g[currentMinute] += 120 - currentSecond * 2;
    pixels_current_intensity_g[currentMinutePlusOne] += currentSecond * 2;
    /*    pixels_current_intensity_r[currentMinute] += value1;
        pixels_current_intensity_b[currentMinute] += value1;
        pixels_current_intensity_r[currentMinutePlusOne] += value2;
        pixels_current_intensity_b[currentMinutePlusOne] += value2;*/
  }

  if (secondsSchema == 0)
  {
    pixels_current_intensity_b[currentSecond] += 100 - currentMilliSecond / 10;
    pixels_current_intensity_b[currentSecondPlusOne] += currentMilliSecond / 10;
  }
  else if (secondsSchema == 1)
  {
    pixels_current_intensity_b[currentSecond] += loglookup[(250 - currentMilliSecond / 4)];
    pixels_current_intensity_b[currentSecondPlusOne] += loglookup[(currentMilliSecond / 4)];
  }
  else if (secondsSchema == 2)
  {
    pixels_current_intensity_b[currentSecond % 60] += 200 - currentMilliSecond / 5;
    pixels_current_intensity_b[currentSecondPlusOne % 60] += currentMilliSecond / 5;
    pixels_current_intensity_b[(currentSecond + 25) % 60] += 10 - currentMilliSecond / 100;
    pixels_current_intensity_b[(currentSecondPlusOne + 25) % 60] += currentMilliSecond / 100;
    pixels_current_intensity_b[(currentSecond + 35) % 60] += 20 - currentMilliSecond / 50;
    pixels_current_intensity_b[(currentSecondPlusOne + 35) % 60] += currentMilliSecond / 50;
    //  pixels_current_intensity_b[(currentSecond+45) % 60] += 10 - currentMilliSecond/100;
    //  pixels_current_intensity_b[(currentSecondPlus+45) % 60] += currentMilliSecond/100;
  }
  else if (secondsSchema == 3)
  {
    pixels_current_intensity_b[currentSecond % 60] += loglookup[(200 - currentMilliSecond / 5)];
    pixels_current_intensity_b[currentSecondPlusOne % 60] += loglookup[(currentMilliSecond / 5)];
    //  pixels_current_intensity_b[currentSecond % 60] += 200 - currentMilliSecond/5;
    //  pixels_current_intensity_b[currentSecondPlus % 60] += currentMilliSecond/5;
    pixels_current_intensity_b[(currentSecond + 28) % 60] += 10 - currentMilliSecond / 100;
    pixels_current_intensity_b[(currentSecondPlusOne + 28) % 60] += currentMilliSecond / 100;
    pixels_current_intensity_b[(currentSecond + 32) % 60] += 20 - currentMilliSecond / 50;
    pixels_current_intensity_b[(currentSecondPlusOne + 32) % 60] += currentMilliSecond / 50;
    //  pixels_current_intensity_b[(currentSecond+45) % 60] += 10 - currentMilliSecond/100;
    //  pixels_current_intensity_b[(currentSecondPlus+45) % 60] += currentMilliSecond/100;
  }
  /*else {
   pixels_current_intensity_b[currentSecond % 60] += 200 - currentMilliSecond/5;
   pixels_current_intensity_b[currentSecondPlus % 60] += currentMilliSecond/5;
   }*/

  for (int i = 0; i < NUMPIX; i++)
  {
    int i2 = (i + 30) % 60;
    strip.SetPixelColor(i, RgbColor(
                               min(pixels_current_intensity_r[i2], 255),
                               min(pixels_current_intensity_g[i2], 255),
                               min(pixels_current_intensity_b[i2], 255)));
  }

  strip.Show();

  // delay(25);
}
