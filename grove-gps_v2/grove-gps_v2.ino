#include <WioLTEforArduino.h>
#include <TinyGPS++.h>

//状態に対する色設定
#define COLOR_SETUP 0, 10, 0
#define COLOR_MEASURE 0, 0, 10
#define COLOR_MAGNETIC 10, 0, 0
#define COLOR_NONE_MAGNETIC 0, 0, 10
#define COLOR_NEAR 10, 10, 10
#define COLOR_NONE 0, 0, 0

WioLTE Wio;
TinyGPSPlus gps;

void setup()
{
  delay(200);

  SerialUSB.println("");
  SerialUSB.println("____ START ____________________________________");
  
  SerialUSB.println("### I/O Initialize.");
  GpsBegin(&Serial);
  Wio.Init();
  Wio.LedSetRGB(COLOR_SETUP);
  
  SerialUSB.println("### Power supply ON.");
  Wio.PowerSupplyGrove(true);
  delay(500);

  SerialUSB.println("### Setup completed.");
  Wio.LedSetRGB(COLOR_NONE);
}

void loop()
{
  Wio.LedSetRGB(COLOR_MAGNETIC);
  const char* data = GpsRead();
  if (data != NULL && strncmp(data, "$GPGGA,", 7) == 0) {
    SerialUSB.println(data);
  }
  Wio.LedSetRGB(COLOR_NONE);
}

////////////////////////////////////////////////////////////////////////////////////////
//

#define GPS_OVERFLOW_STRING "OVERFLOW"

HardwareSerial* GpsSerial;
char GpsData[100];
char GpsDataLength;

void GpsBegin(HardwareSerial* serial)
{
  GpsSerial = serial;
  GpsSerial->begin(9600);
  GpsDataLength = 0;
}
void displayInfo()
{
    SerialUSB.print(F("Location: ")); 
    if (gps.location.isValid()) {
        SerialUSB.println(gps.location.lat(), 6);
        SerialUSB.println(gps.location.lng(), 6);
    } else {
        SerialUSB.println(F("INVALID"));
    }
}

const char* GpsRead()
{
  while (GpsSerial->available()) {
    char data = GpsSerial->read();

    if (gps.encode(data)) {
      displayInfo();
    }

    
    if (data == '\r') continue;
    if (data == '\n') {
      GpsData[GpsDataLength] = '\0';
      GpsDataLength = 0;
      return GpsData;
    }
    
    if (GpsDataLength > sizeof (GpsData) - 1) { // Overflow
      GpsDataLength = 0;
      return GPS_OVERFLOW_STRING;
    }
    GpsData[GpsDataLength++] = data;
  }

  return NULL;
}

////////////////////////////////////////////////////////////////////////////////////////

