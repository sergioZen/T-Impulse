#include <TinyGPS++.h>
#include <Arduino.h>
#include <HardwareSerial.h>

#define DEBUG(x)                  Serial.print(x);
#define DEBUGLN(x)                Serial.println(x);

#define PwrSwitch1_8V PB0
#define PwrSwitchGPS PA3

#define GPS_TX PC10
#define GPS_EN PC6
#define GPS_1PPS PB5
#define GPS_RST PB2
#define GPS_RX PC11
#define GPS_BAUD_RATE 115200

TinyGPSPlus *gps = nullptr;

HardwareSerial gpsPort(GPS_RX, GPS_TX);

static double Lat = 0, Long = 0;

void GPS_WaitAck(String cmd, String arg = "")
{
  if (arg != "")
  {
    gpsPort.print(cmd);
    gpsPort.print(" ");
    gpsPort.println(arg);
  }
  else
  {
    gpsPort.println(cmd);
  }
  String ack = "";
  uint32_t smap = millis() + 500;
  while (millis() < smap)
  {
    if (gpsPort.available() > 0)
    {
      ack = gpsPort.readStringUntil('\n');
      String acc = "[" + cmd.substring(1) + "] " + "Done";
      if (ack.startsWith(acc))
      {
        Serial.printf("%s %s send success \n", cmd.c_str(), arg.c_str());
        return;
      }
    }
  }
  Serial.printf("%s %s send time out \n", cmd.c_str(), arg.c_str());

}

void gps_init(void)
{
  gps = new TinyGPSPlus();

  pinMode(PwrSwitch1_8V, OUTPUT);
  digitalWrite(PwrSwitch1_8V, HIGH);

  pinMode(PwrSwitchGPS, OUTPUT);
  digitalWrite(PwrSwitchGPS, HIGH);

  Serial.begin(115200);
  delay(5000);

  Serial.println("GPS Each test");
  Serial.println("Never try to send @FER, it clears GPS firmware!!");
  Serial.println("Never try to send @FER, it clears GPS firmware!!");
  Serial.println("Never try to send @FER, it clears GPS firmware!!");
  gpsPort.begin(GPS_BAUD_RATE);
  pinMode(GPS_EN, OUTPUT);
  digitalWrite(GPS_EN, HIGH);
  pinMode(GPS_RST, GPIO_PULLUP);
  // Set  Reset Pin as 0
  digitalWrite(GPS_RST, LOW);
  // Scope shows 1.12s (Low Period)
  delay(200);
  // Set  Reset Pin as 1
  digitalWrite(GPS_RST, HIGH);
  delay(500);

  GPS_WaitAck("@GSTP");
  GPS_WaitAck("@BSSL", "0x2EF");
  GPS_WaitAck("@GSOP", "1 1000 0");
  GPS_WaitAck("@GNS", "0x03");
  //! Start GPS connamd
  GPS_WaitAck("@GSR");
}

void gps_loop(void)
{
    while (gpsPort.available() )
    {
        //Serial.write(gpsPort.read());
        gps->encode(gpsPort.read());
    }

    while (Serial.available())
        gpsPort.write(Serial.read());

    /*
    while (gpsPort.available() > 0)
    {
        gps->encode(gpsPort.read());
    }
    if (gps->charsProcessed() < 10)
    {
        Serial.println(F("WARNING: No GPS data.  Check wiring."));
    }
    */

    /*
    DEBUG(F("LOCATION   Fix Age="));
    DEBUG(gps->location.age());
    DEBUG(F("ms Raw Lat="));
    DEBUG(gps->location.rawLat().negative ? "-" : "+");
    DEBUG(gps->location.rawLat().deg);
    DEBUG("[+");
    DEBUG(gps->location.rawLat().billionths);
    DEBUG(F(" billionths],  Raw Long="));
    DEBUG(gps->location.rawLng().negative ? "-" : "+");
    DEBUG(gps->location.rawLng().deg);
    DEBUG("[+");
    DEBUG(gps->location.rawLng().billionths);
    DEBUG(F(" billionths],  Lat="));
    DEBUG(gps->location.lat());
    Lat = gps->location.lat();
    DEBUG(F(" Long="));
    DEBUGLN(gps->location.lng());
    Long = gps->location.lng();
    */
}
