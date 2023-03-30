
#include "loramac.h"

CayenneLPP lpp(100);

//--------------------------------------
//- SBR: lora-wrist1 (sin I Indicador):
//--------------------------------------
/*
// Chose LSB mode on the console and then copy it here.
static const u1_t PROGMEM APPEUI[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
// LSB mode (msb: 70B3D57ED005B4C2)
static const u1_t PROGMEM DEVEUI[8] = {0xC2, 0xB4, 0x05, 0xD0, 0x7E, 0xD5, 0xB3, 0x70};
// MSB mode (msb: 478EC5D0FF8D1B2CF2985AABA694A0B5)
static const u1_t PROGMEM APPKEY[16] = {0x47, 0x8E, 0xC5, 0xD0, 0xFF, 0x8D, 0x1B, 0x2C, 0xF2, 0x98, 0x5A, 0xAB, 0xA6, 0x94, 0xA0, 0xB5};
*/

//--------------------------------------
//- SBR: lora-wrist2 (con I Indicador):
//--------------------------------------
// Chose LSB mode on the console and then copy it here.
static const u1_t PROGMEM APPEUI[8] = {0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
// LSB mode (msb: 70B3D57ED005B745)
static const u1_t PROGMEM DEVEUI[8] = {0x45, 0xB7, 0x05, 0xD0, 0x7E, 0xD5, 0xB3, 0x70};
// MSB mode (msb: 478EC5D0FF8D1B2CF2985AABA694A0B5)
static const u1_t PROGMEM APPKEY[16] = {0xF3, 0x56, 0x9C, 0x9D, 0x86, 0x3F, 0x71, 0x41, 0x2B, 0xCE, 0xCB, 0xB7, 0x13, 0x20, 0x0D, 0xB2};

// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = LORA_NSS,
    .rxtx = RADIO_ANT_SWITCH_RXTX,
    .rst = LORA_RST,
    .dio = {LORA_DIO0, LORA_DIO1_PIN, LORA_DIO2_PIN},
    //.rx_level = HIGH
};

static osjob_t sendjob;
static int spreadFactor = DR_SF7;
static int joinStatus = EV_JOINING;
static const unsigned TX_INTERVAL = 1;
static String lora_msg = "";

void printVariables()
{
    lpp.reset();
    float batt_lvl = float((getVolt() * 3.3 * 2) / 4096);
    lpp.addAnalogInput(3, batt_lvl);

    if (getIMU()->dataReady())
    {
        getIMU()->getAGMT();

        float Acc_x = getIMU()->accX();
        float Acc_y = getIMU()->accY();
        float Acc_z = getIMU()->accZ();
        lpp.addAccelerometer(4, Acc_x, Acc_y, Acc_z);

        float Gyr_x = getIMU()->gyrX();
        float Gyr_y = getIMU()->gyrY();
        float Gyr_z = getIMU()->gyrZ();
        lpp.addGyrometer(5, Gyr_x, Gyr_y, Gyr_z);
    }

    if (getGPS()->location.isUpdated() && getGPS()->altitude.isUpdated() && getGPS()->satellites.isUpdated())
    {
        double gps_lat = getGPS()->location.lat();
        double gps_lng = getGPS()->location.lng();
        double gps_alt = getGPS()->altitude.meters();
        lpp.addGPS(6, (float)gps_lat, (float)gps_lng, (float)gps_alt);
        uint32_t Value = getGPS()->satellites.value();
        lpp.addLuminosity(7, Value);

        char buf[50];
        sprintf(buf, "[%lu]Satellites", millis() / 1000);
        Title_Commit(buf);
    }
}

void os_getArtEui(u1_t *buf)
{
    memcpy_P(buf, APPEUI, 8);
}

void os_getDevEui(u1_t *buf)
{
    memcpy_P(buf, DEVEUI, 8);
}

void os_getDevKey(u1_t *buf)
{
    memcpy_P(buf, APPKEY, 16);
}

void do_send(osjob_t *j)
{
    if (joinStatus == EV_JOINING)
    {
        Serial.println(F("Not joined yet"));
        // Check if there is not a current TX/RX job running
        os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
    }
    else if (LMIC.opmode & OP_TXRXPEND)
    {
        Serial.println(F("OP_TXRXPEND, not sending"));
    }
    else
    {
        Serial.println(F("OP_TXRXPEND,sending ..."));
        static uint8_t mydata[] = "Hello, world!";

        printVariables();
        LMIC_setTxData2(1, lpp.getBuffer(), lpp.getSize(), 0);

        // Prepare upstream data transmission at the next possible time.
        // LMIC_setTxData2(1, mydata, sizeof(mydata) - 1, 0);

        os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);

        char buf[50];
        snprintf(buf, sizeof(buf), "[%lu]data sending!", millis() / 1000);
        Title_Commit(buf);
    }
}

void onEvent(ev_t ev)
{
    char buf[50];
    Serial.print(os_getTime());
    Serial.print(": ");
    switch (ev)
    {
    case EV_TXCOMPLETE:
        Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));

        if (LMIC.txrxFlags & TXRX_ACK)
        {
            Serial.println(F("Received ack"));
            lora_msg = "Received ACK.";
        }

        lora_msg = "rssi:" + String(LMIC.rssi) + " snr: " + String(LMIC.snr);

        if (LMIC.dataLen)
        {
            // data received in rx slot after tx
            Serial.print(F("Data Received: "));

            Serial.write(LMIC.frame + LMIC.dataBeg, LMIC.dataLen);
            Serial.println();

            Title_Commit("New massage!");
            Msg_Commit((char *)LMIC.frame + LMIC.dataBeg, LMIC.dataLen);

            Serial.println(LMIC.dataLen);
            Serial.println(F(" bytes of payload"));
        }
        // Schedule next transmission
        os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
        break;
    case EV_JOINING:
        Serial.println(F("EV_JOINING: -> Joining..."));
        lora_msg = "OTAA 1 joining....";
        joinStatus = EV_JOINING;

        snprintf(buf, sizeof(buf), "[%lu]OTAA 2 joining....", millis() / 1000);
        Title_Commit(buf);

        break;
    case EV_JOIN_FAILED:
        Serial.println(F("EV_JOIN_FAILED: -> Joining failed"));
        lora_msg = "OTAA 3 Joining failed";

        snprintf(buf, sizeof(buf), "[%lu]OTAA 4 joining failed", millis() / 1000);
        Title_Commit(buf);

        break;
    case EV_JOINED:
        Serial.println(F("EV_JOINED"));
        lora_msg = "Joined!";
        joinStatus = EV_JOINED;

        snprintf(buf, sizeof(buf), "[%lu]Joined TTN!", millis() / 1000);
        Title_Commit(buf);

        delay(3);
        // Disable link check validation (automatically enabled
        // during join, but not supported by TTN at this time).
        LMIC_setLinkCheckMode(0);

        break;
    case EV_RXCOMPLETE:
        // data received in ping slot

        snprintf(buf, sizeof(buf), "[%lu]New Message!", millis() / 1000);
        Title_Commit(buf);

        Serial.println(F("EV_RXCOMPLETE"));
        break;
    case EV_LINK_DEAD:
        Serial.println(F("EV_LINK_DEAD"));
        break;
    case EV_LINK_ALIVE:
        Serial.println(F("EV_LINK_ALIVE"));
        break;
    default:
        Serial.println(F("Unknown event"));
        break;
    }
}

void setupLMIC(void)
{
    // LMIC init
    os_init();

    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();

    LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100);
    // Set up the channels used by the Things Network, which corresponds
    // to the defaults of most gateways. Without this, only three base
    // channels from the LoRaWAN specification are used, which certainly
    // works, so it is good for debugging, but can overload those
    // frequencies, so be sure to configure the full frequency range of
    // your network here (unless your network autoconfigures them).
    // Setting up channels should happen after LMIC_setSession, as that
    // configures the minimal channel set.

    LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
    LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI); // g-band
    LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
    LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
    LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
    LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
    LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
    LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
    LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK, DR_FSK), BAND_MILLI);   // g2-band
    // TTN defines an additional channel at 869.525Mhz using SF9 for class B
    // devices' ping slots. LMIC does not have an easy way to define set this
    // frequency and support for class B is spotty and untested, so this
    // frequency is not configured here.

    // Disable link check validation
    LMIC_setLinkCheckMode(0);

    // TTN uses SF9 for its RX2 window.
    LMIC.dn2Dr = DR_SF9;

    // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
    LMIC_setDrTxpow(spreadFactor, 14);

    // Start job
    LMIC_startJoining();

    do_send(&sendjob); // Will fire up also the join
}

void loopLMIC(void)
{
    os_runloop_once();
}
