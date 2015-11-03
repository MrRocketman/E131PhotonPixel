#include "FastLED.h"
FASTLED_USING_NAMESPACE;

SYSTEM_THREAD(ENABLED); // This makes the system cloud connection run on a background thread so as to not delay our timing

/*
* E131.h
*
* Project: E131 - E.131 (sACN) library for Arduino
* Copyright (c) 2015 Shelby Merrick
* http://www.forkineye.com
*
*  This program is provided free for you to use in any way that you wish,
*  subject to the laws and regulations where you are using it.  Due diligence
*  is strongly suggested before using this code.  Please give credit where due.
*
*  The Author makes no warranty of any kind, express or implied, with regard
*  to this program or the documentation contained in this document.  The
*  Author shall not be liable in any event for incidental or consequential
*  damages in connection with, or arising out of, the furnishing, performance
*  or use of these programs.
*
*/

// Helpers
#define htons(x) ( ((x)<<8) | (((x)>>8)&0xFF) )
#define ntohs(x) htons(x)

#define htonl(x) ( ((x)<<24 & 0xFF000000UL) | \
                  ((x)<< 8 & 0x00FF0000UL) | \
                  ((x)>> 8 & 0x0000FF00UL) | \
                  ((x)>>24 & 0x000000FFUL) )
#define ntohl(x) htonl(x)

/* Defaults */
#define E131_DEFAULT_PORT 5568
#define WIFI_CONNECT_TIMEOUT 10000  /* 10 seconds */

/* E1.31 Packet Offsets */
#define E131_ROOT_PREAMBLE_SIZE 0
#define E131_ROOT_POSTAMBLE_SIZE 2
#define E131_ROOT_ID 4
#define E131_ROOT_FLENGTH 16
#define E131_ROOT_VECTOR 18
#define E131_ROOT_CID 22

#define E131_FRAME_FLENGTH 38
#define E131_FRAME_VECTOR 40
#define E131_FRAME_SOURCE 44
#define E131_FRAME_PRIORITY 108
#define E131_FRAME_RESERVED 109
#define E131_FRAME_SEQ 111
#define E131_FRAME_OPT 112
#define E131_FRAME_UNIVERSE 113

#define E131_DMP_FLENGTH 115
#define E131_DMP_VECTOR 117
#define E131_DMP_TYPE 118
#define E131_DMP_ADDR_FIRST 119
#define E131_DMP_ADDR_INC 121
#define E131_DMP_COUNT 123
#define E131_DMP_DATA 125

#define E131_PACKET_SIZE 638

/* E1.31 Packet Structure */
typedef union
{
    struct
    {
        /* Root Layer */
        uint16_t preamble_size;
        uint16_t postamble_size;
        uint8_t	 acn_id[12];
        uint16_t root_flength;
        uint32_t root_vector;
        uint8_t  cid[16];

        /* Frame Layer */
        uint16_t frame_flength;
        uint32_t frame_vector;
        uint8_t  source_name[64];
        uint8_t  priority;
        uint16_t reserved;
        uint8_t  sequence_number;
        uint8_t  options;
        uint16_t universe;

        /* DMP Layer */
        uint16_t dmp_flength;
        uint8_t  dmp_vector;
        uint8_t  type;
        uint16_t first_address;
        uint16_t address_increment;
        uint16_t property_value_count;
        uint8_t  property_values[513];
    } __attribute__((packed));

    uint8_t raw[E131_PACKET_SIZE];
} e131_packet_t;

// Don't change this unless you really know what you're doing!
#define NUMBER_OF_OUTPUTS 16
#define NUMBER_OF_PIXEL_PIN_MAP_ITEMS 6

#define DEFAULT_UNIVERSE_SIZE 512

#define EEPROM_DATA_ADDRESS 0
#define EEPROM_ID 0x7A2E03
#define EEPROM_VERSION 0x0001

// This is used to access pinMaps array elements easier. DO NOT CHANGE!
enum pixelPinMapItems
{
    PIXEL_TYPE = 0,
    NUMBER_OF_PIXELS,
    START_UNIVERSE,
    START_CHANNEL,
    END_UNIVERSE,
    END_CHANNEL
};

////// PixelPinMaps Definition ////////
// - PIXEL_TYPE
// - NUMBER_OF_PIXELS
// - START_UNIVERSE
// - START_CHANNEL
// - END_UNIVERSE
// - END_CHANNEL
typedef struct
{
    uint16_t pixelPinMaps[NUMBER_OF_OUTPUTS][NUMBER_OF_PIXEL_PIN_MAP_ITEMS]; // 2 bytes per element. 6*16 elements * 2 bytes (uint16_t) = 192 bytes. // See above for what each index is
    int universeSize; // Was a uint16_t but that makes it hard to expose as a cloud variable. Making it an int is much less confusing
    uint32_t gammaSettings[NUMBER_OF_OUTPUTS]; // 1 byte per color (eg. FF 0F 4F) = 3 bytes per pin = uint32_t
    uint32_t id;
    uint16_t version;
} eeprom_data_t;

/* Status structure */
typedef struct
{
    uint32_t    num_packets;
    uint32_t    sequence_errors;
    uint32_t    packet_errors;
} e131_stats_t;

/* Error Types */
typedef enum
{
    ERROR_NONE,
    ERROR_ACN_ID,
    ERROR_PACKET_SIZE,
    ERROR_VECTOR_ROOT,
    ERROR_VECTOR_FRAME,
    ERROR_VECTOR_DMP
} e131_error_t;

/* E1.31 Listener Types */
typedef enum
{
    E131_UNICAST,
    E131_MULTICAST
} e131_listen_t;

/* Constants for packet validation */
static const uint8_t ACN_ID[12] = { 0x41, 0x53, 0x43, 0x2d, 0x45, 0x31, 0x2e, 0x31, 0x37, 0x00, 0x00, 0x00 };
static const uint32_t VECTOR_ROOT = 4;
static const uint32_t VECTOR_FRAME = 2;
static const uint8_t VECTOR_DMP = 2;

e131_packet_t pbuff1; // Packet buffer
e131_packet_t pbuff2;   /* Double buffer */
e131_packet_t *pwbuff;  /* Pointer to working packet buffer */
uint8_t sequence; /* Sequence tracker */

uint8_t       *data;                /* Pointer to DMX channel data */
uint16_t      universe;             /* DMX Universe of last valid packet */
e131_packet_t *packet;              /* Pointer to last valid packet */
e131_stats_t  stats;                /* Statistics tracker */

eeprom_data_t eepromData;
// * 6 because each item is a uint16_t which takes up to 5 string characters (65535) + a comma
// + NUMBER_OF_OUTPUTS for a semicolon between each pin
// + 2 for Null termination
char pixelPinMapCharArray[((uint8_t)NUMBER_OF_OUTPUTS * NUMBER_OF_PIXEL_PIN_MAP_ITEMS * 6) + NUMBER_OF_OUTPUTS + 2]; // Char arrays of the EEPROM data for cloud variable
// * 6 because each item is a uint32_t which takes up to 6 string characters as HEX character (FF55FF) + a comma
// + 2 for null termination
char gammaSettingsCharArray[(uint8_t)NUMBER_OF_OUTPUTS * 7 + 2]; // Char array of the EEPROM data for the cloud variable

CRGB leds[576];

// An UDP instance to let us send and receive packets over UDP
UDP udp;
int lastUDPPacketReceiveTime = 0;

bool previousWiFiReadiness = false;
bool wiFiReadiness = false;
IPAddress myIp;
char myIpString[24];
char firmwareVersion[6] = "0.0.2";

/* Diag functions */
void dumpError(e131_error_t error);
uint16_t parsePacket();
e131_error_t validateE131Packet();

// My functions
void readEEPROMData();
void pixelPinMapsToString();
void gammaSettingsToString();

void setup()
{
    memset(pbuff1.raw, 0, sizeof(pbuff1.raw));
    memset(pbuff2.raw, 0, sizeof(pbuff2.raw));
    packet = &pbuff1;
    pwbuff = &pbuff2;

    sequence = 0;
    stats.num_packets = 0;
    stats.sequence_errors = 0;
    stats.packet_errors = 0;
    myIp = WiFi.localIP();
    sprintf(myIpString, "%d.%d.%d.%d\0", myIp[0], myIp[1], myIp[2], myIp[3]);
    Serial.print("ip:");
    Serial.println(myIp);

    WiFi.selectAntenna(ANT_AUTO); // ANT_INTERNAL ANT_EXTERNAL ANT_AUTO

    // Read from EEPROM
    readEEPROMData();

    Serial.begin(115200);

    // Setup cloud variables and functions
    Particle.variable("pixelPinMaps", pixelPinMapCharArray);
    Particle.variable("universeSize", eepromData.universeSize);
    Particle.variable("gammaSetting", gammaSettingsCharArray);
    Particle.variable("localIP", myIpString);
    Particle.variable("e131FVersion", firmwareVersion);

    FastLED.addLeds<WS2811, 0>(leds, 576); // Pin 0, 576 pixels
    FastLED.show();

    // Setup the UDP connection
    if(udp.setBuffer(E131_PACKET_SIZE, pwbuff->raw))
    {
      //udp.begin(E131_DEFAULT_PORT);
    }
}

void loop()
{
    // Check to see if the WiFi connection was lost.
    previousWiFiReadiness = wiFiReadiness;
    wiFiReadiness = WiFi.ready();
    // WiFi stopped. Release udp so we can reinit once it's ready again
    if(wiFiReadiness == false && previousWiFiReadiness == true)
    {
        Serial.println("WiFi not ready");
        udp.stop();
    }
    // WiFi is back. Open up udp port again
    else if(wiFiReadiness == true && previousWiFiReadiness == false)
    {
        Serial.println("WiFi Back online");
        myIp = WiFi.localIP();
        sprintf(myIpString, "%d.%d.%d.%d\0", myIp[0], myIp[1], myIp[2], myIp[3]);
        udp.begin(E131_DEFAULT_PORT);
    }

    /* Parse a packet and update pixels */
    int dataSize = parsePacket();
    if(dataSize > 0)
    {
        // Extract the dmx data and store it in each LED
        int ledIndex = 0;
        int universeShiftedI = 0;
        for(int i = 0; i < dataSize; i ++)
        {
            universeShiftedI = i + (universe - 1) * 512;
            ledIndex = universeShiftedI / 3;
            if(ledIndex < 576) // TODO: Check universe number
            {
              if(universeShiftedI % 3 == 0)
              {
                  leds[ledIndex].r = data[i];
              }
              else if(universeShiftedI % 3 == 1)
              {
                  leds[ledIndex].g = data[i];
              }
              else
              {
                  leds[ledIndex].b = data[i];
              }
            }
        }

        // Write the data to the pixels after we have received all data (This is temporary)
        if(universe == 7)
        {
            FastLED.show();
        }
    }
}

/* Main packet parser */
uint16_t parsePacket()
{
    e131_error_t error;
    uint16_t retval = 0;

    int size = udp.parsePacket();
    if (size > 0)
    {
        udp.read(pwbuff->raw, size);
        error = validateE131Packet();
        if (!error)
        {
            e131_packet_t *swap = packet;
            packet = pwbuff;
            pwbuff = swap;
            //printUDPData(pwbuff->raw, size);
            universe = htons(packet->universe);
            data = packet->property_values + 1;
            retval = htons(packet->property_value_count) - 1;
            if (packet->sequence_number != sequence++)
            {
                stats.sequence_errors++;
                sequence = packet->sequence_number + 1;
            }
            stats.num_packets++;
        }
        else
        {
            dumpError(error);
            stats.packet_errors++;
        }
    }
    return retval;
}

/* Packet validater */
e131_error_t validateE131Packet()
{
    if (memcmp(pwbuff->acn_id, ACN_ID, sizeof(pwbuff->acn_id)))
        return ERROR_ACN_ID;
    if (htonl(pwbuff->root_vector) != VECTOR_ROOT)
        return ERROR_VECTOR_ROOT;
    if (htonl(pwbuff->frame_vector) != VECTOR_FRAME)
        return ERROR_VECTOR_FRAME;
    if (pwbuff->dmp_vector != VECTOR_DMP)
        return ERROR_VECTOR_DMP;
    return ERROR_NONE;
}

void dumpError(e131_error_t error)
{
    switch (error) {
        case ERROR_ACN_ID:
            Serial.print(F("INVALID PACKET ID: "));
            for (uint8_t i = 0; i < sizeof(ACN_ID); i++)
                Serial.print(pwbuff->acn_id[i], HEX);
            Serial.println("");
            break;
        case ERROR_PACKET_SIZE:
            Serial.println(F("INVALID PACKET SIZE: "));
            break;
        case ERROR_VECTOR_ROOT:
            Serial.print(F("INVALID ROOT VECTOR: 0x"));
            Serial.println(htonl(pwbuff->root_vector), HEX);
            break;
        case ERROR_VECTOR_FRAME:
            Serial.print(F("INVALID FRAME VECTOR: 0x"));
            Serial.println(htonl(pwbuff->frame_vector), HEX);
            break;
        case ERROR_VECTOR_DMP:
            Serial.print(F("INVALID DMP VECTOR: 0x"));
            Serial.println(pwbuff->dmp_vector, HEX);
    }
}

void printUDPData(uint8_t *udpData, int size)
{
    Serial.println("Packet");
    for(int i = 0; i < size; i ++)
    {
        Serial.print(udpData[i], HEX);
        Serial.print(" ");
    }
    Serial.println();
}

void readEEPROMData()
{
    EEPROM.get(EEPROM_DATA_ADDRESS, eepromData);

    // See if data needs initialization
    if(eepromData.id != EEPROM_ID)
    {
        // Initialize pixelPinMaps
        for(byte i = 0; i < NUMBER_OF_OUTPUTS; i ++)
        {
            eepromData.pixelPinMaps[i][PIXEL_TYPE] = 0; // Define as WS2812 or whatever
            eepromData.pixelPinMaps[i][NUMBER_OF_PIXELS] = 170;
            eepromData.pixelPinMaps[i][START_UNIVERSE] = (uint16_t)(i + 1);
            eepromData.pixelPinMaps[i][START_CHANNEL] = 0;
            eepromData.pixelPinMaps[i][END_UNIVERSE] = (uint16_t)(i + 1);
            eepromData.pixelPinMaps[i][END_CHANNEL] = DEFAULT_UNIVERSE_SIZE - 3;
        }

        // Init universe size
        eepromData.universeSize = DEFAULT_UNIVERSE_SIZE;

        // Init gamma settings
        for(byte i = 0; i < NUMBER_OF_OUTPUTS; i ++)
        {
            eepromData.gammaSettings[i] = 0xFFB0F0;
        }

        // Init eeprom version and ID
        eepromData.version = EEPROM_VERSION;
        eepromData.id = EEPROM_ID;

        // Save to EEPROM
        EEPROM.put(EEPROM_DATA_ADDRESS, eepromData);
    }

    // Convert pinMaps and gammaSettings to char arrays for cloud variable access
    pixelPinMapsToString();
    gammaSettingsToString();
}

// Convert the pixelPinMaps to a string for cloud access
void pixelPinMapsToString()
{
    memset(pixelPinMapCharArray, 0, ((uint8_t)NUMBER_OF_OUTPUTS * NUMBER_OF_PIXEL_PIN_MAP_ITEMS * 6) + NUMBER_OF_OUTPUTS + 2);
    pixelPinMapCharArray[0] = '\0';

    for(byte i = 0; i < NUMBER_OF_OUTPUTS; i ++)
    {
        sprintf(pixelPinMapCharArray, "%s%u,%u,%u,%u,%u,%u;", pixelPinMapCharArray, eepromData.pixelPinMaps[i][PIXEL_TYPE], eepromData.pixelPinMaps[i][NUMBER_OF_PIXELS], eepromData.pixelPinMaps[i][START_UNIVERSE], eepromData.pixelPinMaps[i][START_CHANNEL], eepromData.pixelPinMaps[i][END_UNIVERSE], eepromData.pixelPinMaps[i][END_CHANNEL]);
    }
}

void gammaSettingsToString()
{
    memset(gammaSettingsCharArray, 0, (uint8_t)NUMBER_OF_OUTPUTS * 7 + 2);
    gammaSettingsCharArray[0] = '\0';

    for(byte i = 0; i < NUMBER_OF_OUTPUTS; i ++)
    {
        sprintf(gammaSettingsCharArray, "%s%X,", gammaSettingsCharArray, (unsigned int)(eepromData.gammaSettings[i]));
    }
}
