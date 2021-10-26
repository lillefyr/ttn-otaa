/*******************************************************************************
 * Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman
 * Copyright (c) 2018 Terry Moore, MCCI
 *
 * Permission is hereby granted, free of charge, to anyone
 * obtaining a copy of this document and accompanying files,
 * to do whatever they want with them without any restriction,
 * including, but not limited to, copying, modification and redistribution.
 * NO WARRANTY OF ANY KIND IS PROVIDED.
 *
 * This example sends a valid LoRaWAN packet with payload "Hello,
 * world!", using frequency and encryption settings matching those of
 * the The Things Network.
 *
 * This uses OTAA (Over-the-air activation), where where a DevEUI and
 * application key is configured, which are used in an over-the-air
 * activation procedure where a DevAddr and session keys are
 * assigned/generated for use with all further communication.
 *
 * Note: LoRaWAN per sub-band duty-cycle limitation is enforced (1% in
 * g1, 0.1% in g2), but not the TTN fair usage policy (which is probably
 * violated by this sketch when left running for longer)!

 * To use this sketch, first register your application and device with
 * the things network, to set or generate an AppEUI, DevEUI and AppKey.
 * Multiple devices can use the same AppEUI, but each device has its own
 * DevEUI and AppKey.
 *
 * Do not forget to define the radio type correctly in
 * arduino-lmic/project_config/lmic_project_config.h or from your BOARDS.txt.
 *
 ******************************************************************************/

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <SoftwareSerial.h>
#include <TinyGPS.h>

#include "font.h"
#include <SSD1306.h>


// TTGO T-Beam ESP32
#define SCK     5    // GPIO5  -- SX1278's SCK
#define MISO    19   // GPIO19 -- SX1278's MISnO
#define MOSI    27   // GPIO27 -- SX1278's MOSI
#define SS      18   // GPIO18 -- SX1278's CS
#define RST     14   // GPIO14 -- SX1278's RESET
#define DIO0    26   // GPIO26 -- SX1278's IRQ(Interrupt Request)
#define DIO1    33   // must be hardwired
#define DIO2    32   // can  be hardwired (Is not used, but NC will fail)

#define TXLED   4
#define BUTTON  38
#define GPSLED  6
#define GPSLED1 9

#define GPSRX   15 //12
#define GPSTX   12 //34



// This EUI must be in little-endian format, so least-significant-byte
// first. When copying an EUI from ttnctl output, this means to reverse
// the bytes. For TTN issued EUIs the last bytes should be 0xD5, 0xB3,
// 0x70.
static const u1_t PROGMEM APPEUI[8]={ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}

// This should also be in little endian format, see above.
static const u1_t PROGMEM DEVEUI[8]={ 0x03, 0x6C, 0x04, 0xD0, 0x7E, 0xD5, 0xB3, 0x70 };
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}

// This key should be in big endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// practice, a key taken from ttnctl can be copied as-is.
static const u1_t PROGMEM APPKEY[16] = { 0xD9, 0x53, 0x51, 0xB2, 0xCA, 0x9B, 0x5E, 0xB3, 0xD8, 0x92, 0x31, 0x70, 0x2C, 0x2D, 0xB1, 0x30 };

void os_getDevKey (u1_t* buf) {  memcpy_P(buf, APPKEY, 16);}

char gpsData[20];
unsigned char gpsData2[20];
static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 60;

// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = 18,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 23,
    .dio = {26, 33, 32},
};

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
SSD1306Wire  display(0x3c, SDA, SCL);

String text1 = "";
String text2 = "TTGO";
String text3 = "Ready";

void drawText(int row, int col, String text){
   display.clear();
   display.setFont(Dialog_plain_20); //ArialMT_Plain_24);
   display.setTextAlignment(TEXT_ALIGN_LEFT);
   display.drawString(0, 0, text1);
   display.drawString(0, 16, text2);
   display.drawString(0, 32, text3);
   text1 = text2;
   text2 = text3;
   text3 = text;
   display.display();
}

SoftwareSerial gpsSerial(GPSTX, GPSRX);
TinyGPS gps;

//#define DUMPDATA 1
#ifdef DUMPDATA
void printFloat(double number, int digits)

  if (number < 0.0)
  {
     Serial.print(F('-'));
     number = -number;
  }
  // Handle negative numbers

  // Round correctly so that print(1.999, 2) prints as "2.00"
  double rounding = 0.5;
  for (uint8_t i=0; i<digits; ++i)
    rounding /= 10.0;

  number += rounding;

  // Extract the integer part of the number and print it
  unsigned long int_part = (unsigned long)number;
  double remainder = number - (double)int_part;
  Serial.print(int_part);

  // Print the decimal point, but only if there are digits beyond
  if (digits > 0)
    Serial.print(F("."));

  // Extract digits from the remainder one at a time
  while (digits-- > 0)
  {
    remainder *= 10.0;
    int toPrint = int(remainder);
    Serial.print(toPrint);
    remainder -= toPrint;
  }
}

void gpsdump( float flat, float flon, float falt, float fcourse, float fkmph, unsigned long age, unsigned long datetime, unsigned long hdop )
{
  unsigned long chars;
  int year;
  byte month, day, hour, minute, second, hundredths;
  unsigned short sentences, failed;

//  gps.get_position(&lat, &lon, &age);

  Serial.print(F("Lat/Long(float): ")); printFloat(flat, 5);
  Serial.print(F(", ")); printFloat(flon, 5);
  Serial.print(F(" Fix age: ")); Serial.print(age); Serial.print(F("ms. "));
  Serial.print(F(" (hdop): ")); printFloat(hdop); Serial.println();
  Serial.print(F("Date(ddmmyy): ")); Serial.print(date); 
  Serial.print(F(" Time(hhmmsscc): ")); 
  Serial.println(datetime);

/*
  gps.crack_datetime(&year, &month, &day, &hour, &minute, &second, &hundredths, &age);

  Serial.print(F("Date: ")); Serial.print(static_cast<int>(month)); Serial.print(F("/")); Serial.print(static_cast<int>(day)); Serial.print(F("/")); Serial.print(year);
  Serial.print(F("  Time: ")); Serial.print(static_cast<int>(hour+8));  Serial.print(F(":")); 
    Serial.print(static_cast<int>(minute)); Serial.print(F(":")); Serial.print(static_cast<int>(second));
    Serial.print(F(".")); Serial.print(static_cast<int>(hundredths)); Serial.print(F(" UTC +01:00 Switzerland"));
    */
    /*
  Serial.print(F("  Fix age: "));  Serial.print(age); Serial.println(F("ms."));
  */

 /*
  Serial.print(F("Alt(cm): ")); Serial.print(gps.altitude());
  Serial.print(F(" Course(10^-2 deg): ")); Serial.print(gps.course());
  Serial.print(F(" Speed(10^-2 knots): ")); Serial.println(gps.speed());
  */
  Serial.print(F("Alt(float): ")); printFloat(falt); 
/*  
  Serial.print(F(" Course(float): ")); printFloat(gps.f_course()); Serial.println();
  Serial.print(F("Speed(knots): ")); printFloat(gps.f_speed_knots()); 
  Serial.print(F(" (mph): ")); printFloat(gps.f_speed_mph());
  Serial.print(F(" (mps): ")); printFloat(gps.f_speed_mps()); 
  Serial.print(F(" (kmph): ")); printFloat(gps.f_speed_kmph()); Serial.println();

  gps.stats(&chars, &sentences, &failed);
  Serial.print(F("Stats: characters: ")); Serial.print(chars); 
  Serial.print(F(" sentences: ")); Serial.print(sentences); 
  Serial.print(F(" failed checksum: ")); Serial.println(failed);
  */
}
#endif

char ch;
int i;
void getGPSData() {
  unsigned long start = millis();
  i=0;
  for (int i=0; i<8; i++) { gpsData[i] = 'A'; }
  Serial.println(F("getGPSData["));
  do
  {
    while (gpsSerial.available()) { ch = gpsSerial.read(); gps.encode(ch); if (i<8) { gpsData[i] = ch; i++; gpsData[i] = ' '; } }
  } while (millis() - start < 1000);
  
  Serial.println(F("]"));
  drawText(0,0,gpsData);
}

void getData(){
  unsigned long int age, hdop, cnt;
  int year;
  float alt, lat, lon;
  byte month, day, hour, minute, second, hundredths;
  
  getGPSData();
  gps.f_get_position(&lat, &lon, &age);
  gps.crack_datetime(&year, &month, &day, &hour, &minute, &second, &hundredths, &age);
  hdop = gps.hdop();
  alt = gps.f_altitude();
  Serial.print(F("Lat:")); Serial.println(lat);
  Serial.print(F("Lon:")); Serial.println(lon);
  Serial.print(F("Alt:")); Serial.println(alt);
  Serial.println(year);
  Serial.println(month);
  Serial.println(day);
  Serial.println(hour);
  Serial.println(minute);
  Serial.println(second);

  display.clear();
  display.setTextAlignment(TEXT_ALIGN_RIGHT);
  
  //sprintf(gpsData, "%2d:%2d:%2d    ", hour, minute, second);
  //drawText(0,0, gpsData);
  
  //sprintf(gpsData, "%4.3f   ", lon);
  //drawText(0,0, gpsData);
  
  //sprintf(gpsData, "%4.3f   ", lat);
  //drawText(0,0, gpsData);
  
  ////drawText(31, 13, "Hello");
}

void printHex2(unsigned v) {
    v &= 0xff;
    if (v < 16)
        Serial.print('0');
    Serial.print(v, HEX);
}

void onEvent (ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(": ");
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            Serial.println(F("EV_SCAN_TIMEOUT"));
            //drawText(0,0,"TIMEOUT   ");
            break;
        case EV_BEACON_FOUND:
            Serial.println(F("EV_BEACON_FOUND"));
            //drawText(0,0,"BEACON_FND");
            break;
        case EV_BEACON_MISSED:
            Serial.println(F("EV_BEACON_MISSED"));
            //drawText(0,0,"BEACON_MIS");
            break;
        case EV_BEACON_TRACKED:
            Serial.println(F("EV_BEACON_TRACKED"));
            //drawText(0,0,"BEACON_TRK");
            break;
        case EV_JOINING:
            Serial.println(F("EV_JOINING"));
            //drawText(0,0,"joining   ");
            break;
        case EV_JOINED:
            Serial.println(F("EV_JOINED"));
            {
              u4_t netid = 0;
              devaddr_t devaddr = 0;
              u1_t nwkKey[16];
              u1_t artKey[16];
              LMIC_getSessionKeys(&netid, &devaddr, nwkKey, artKey);
              Serial.print("netid: ");
              Serial.println(netid, DEC);
              Serial.print("devaddr: ");
              Serial.println(devaddr, HEX);
              Serial.print("AppSKey: ");
              for (size_t i=0; i<sizeof(artKey); ++i) {
                if (i != 0)
                  Serial.print("-");
                printHex2(artKey[i]);
              }
              Serial.println("");
              Serial.print("NwkSKey: ");
              for (size_t i=0; i<sizeof(nwkKey); ++i) {
                      if (i != 0)
                              Serial.print("-");
                      printHex2(nwkKey[i]);
              }
              Serial.println();
            }
            // Disable link check validation (automatically enabled
            // during join, but because slow data rates change max TX
      	    // size, we don't use it in this example.
            LMIC_setLinkCheckMode(0);
            //drawText(0,0,"JOINED    ");
            break;
        /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_RFU1:
        ||     Serial.println(F("EV_RFU1"));
        ||     break;
        */
        case EV_JOIN_FAILED:
            Serial.println(F("EV_JOIN_FAILED"));
            //drawText(0,0,"JOIN_FAILED");
            break;
        case EV_REJOIN_FAILED:
            Serial.println(F("EV_REJOIN_FAILED"));
            //drawText(0,0,"REJOIN_FAI");
            break;
        case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if (LMIC.txrxFlags & TXRX_ACK)
              Serial.println(F("Received ack"));
            if (LMIC.dataLen) {
              Serial.print(F("Received "));
              Serial.print(LMIC.dataLen);
              Serial.println(F(" bytes of payload"));
            }
            // Schedule next transmission
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            //drawText(0,0,"TXCOMPLETE");
            break;
        case EV_LOST_TSYNC:
            Serial.println(F("EV_LOST_TSYNC"));
            //drawText(0,0,"LOST_TSYNC");
            break;
        case EV_RESET:
            Serial.println(F("EV_RESET"));
            //drawText(0,0,"RESET     ");
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            Serial.println(F("EV_RXCOMPLETE"));
            //drawText(0,0,"RXCOMPLETE");
            break;
        case EV_LINK_DEAD:
            Serial.println(F("EV_LINK_DEAD"));
            //drawText(0,0,"LINK_DEAD ");
            break;
        case EV_LINK_ALIVE:
            Serial.println(F("EV_LINK_ALIVE"));
            //drawText(0,0,"LINK_ALIVE");
            break;
        /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_SCAN_FOUND:
        ||    Serial.println(F("EV_SCAN_FOUND"));
        ||    break;
        */
        case EV_TXSTART:
            Serial.println(F("EV_TXSTART"));
            //drawText(0,0,"txstart   ");
            break;
        case EV_TXCANCELED:
            Serial.println(F("EV_TXCANCELED"));
            //drawText(0,0,"TXCANCELED");
            break;
        case EV_RXSTART:
            /* do not print anything -- it wrecks timing */
            break;
        case EV_JOIN_TXCOMPLETE:
            Serial.println(F("EV_JOIN_TXCOMPLETE: no JoinAccept"));
            //drawText(0,0,"join_txcom");
            //drawText(0,0,"no JoinAcc");
            break;

        default:
            Serial.print(F("Unknown event: "));
            Serial.println((unsigned) ev);
            //drawText(0,0,"Unknown ev");
            break;
    }
    getData();
    Serial.println(gpsData);
}

void do_send(osjob_t* j){
    getData();
    Serial.println(gpsData);
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
        //drawText(0,0,"TXRXPEND");
        //drawText(0,0,"not sendin");
    } else {
        // Prepare upstream data transmission at the next possible time.
        for (int i=0; i<sizeof(gpsData2); i++) {
          gpsData2[i] = (unsigned char)gpsData[i];
        }
        LMIC_setTxData2(1, gpsData2, sizeof(gpsData2)-1, 0);
        Serial.println(F("Packet queued"));
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

void setup() {
  Serial.begin(115200);
  Serial.println(F("\nStarting ttn-otaa"));
  Serial.println(F("TTGO ESP32 T-BEAM Lora GPS"));
  
  Serial.print("SDA=");Serial.println(SDA);
  Serial.print("SCL=");Serial.println(SCL);
  Wire.begin(SDA,SCL);
  
  display.init();

  display.flipScreenVertically();
  display.setFont(Dialog_plain_20);  //ArialMT_Plain_10);

  for (int i=0; i<8; i++) { gpsData[i] = '?'; }
//  gpsSerial.begin(9600, SERIAL_8N1, GPSTX, GPSRX);
//  gpsSerial.setTimeout(2);
    gpsSerial.begin(9600);  
  // LMIC init
  os_init();
  // Reset the MAC state. Session and pending data transfers will be discarded.
  Serial.println(F("os_init done"));
  LMIC_reset();
  
  getData();
  Serial.println(gpsData);
  // Start job (sending automatically starts OTAA too)
  do_send(&sendjob);
}

void loop() {
  os_runloop_once();
}
