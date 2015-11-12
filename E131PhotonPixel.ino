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
        uint16_t unknownValue;
        uint8_t  property_values[513];
    } __attribute__((packed));

    uint8_t raw[E131_PACKET_SIZE];
} e131_packet_t;

// Don't change this unless you really know what you're doing!
#define NUMBER_OF_OUTPUTS 8 // Technically there are 16 available. The extra 8 are just blank headers on the circuit board currently though

#define DEFAULT_UNIVERSE_SIZE 512

#define EEPROM_DATA_ADDRESS 0
#define EEPROM_ID 0x7A2E0a
#define EEPROM_VERSION 0x0001

#define NUMBER_OF_PIXEL_PIN_MAP_ITEMS 6
// This is used to access pinMaps array elements easier. DO NOT CHANGE!
enum outputSettingsItems
{
    PIXEL_TYPE = 0,
    NUMBER_OF_PIXELS,
    START_UNIVERSE,
    START_CHANNEL,
    END_UNIVERSE,
    END_CHANNEL
};

typedef struct
{
    uint16_t outputSettings[NUMBER_OF_OUTPUTS][NUMBER_OF_PIXEL_PIN_MAP_ITEMS]; // 4 bytes per element. 8*16 elements * 4 bytes (uint32_t) = 384 bytes. // See above for what each index is
    char outputNames[NUMBER_OF_OUTPUTS][32]; // 32 Byte max name lengths
    int universeSize; // Was a uint16_t but that makes it hard to expose as a cloud variable. Making it an int is much less confusing
    uint32_t id;
    uint32_t version;
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

enum {TEST_OFF = 0, TEST_RAINBOW, TEST_RGB, TEST_WHITE};
enum {jWS2811 = 0, jWS2811_400, jNEOPIXEL, jWS2812, jWS2812B, jAPA104, jLPD1886};
enum {SYSTEM_RESET = 0, TEST_ALL, SAVE, UNIVERSE_SIZE, CHANNEL_MAP_FOR_OUTPUT, PIXEL_TYPE_FOR_OUTPUT, NUMBER_OF_PIXELS_FOR_OUTPUT, START_UNIVERSE_FOR_OUTPUT, START_CHANNEL_FOR_OUTPUT, END_UNIVERSE_FOR_OUTPUT, END_CHANNEL_FOR_OUTPUT, NAME_FOR_OUTPUT};

eeprom_data_t eepromData;
// * 6 because each item is a uint16_t which takes up to 5 string characters (65535) + a comma
// + NUMBER_OF_OUTPUTS for a semicolon between each pin
// + 2 for Null termination
char outputSettingsCharArray[((uint8_t)NUMBER_OF_OUTPUTS * NUMBER_OF_PIXEL_PIN_MAP_ITEMS * 6) + NUMBER_OF_OUTPUTS + 2]; // Char arrays of the EEPROM data for cloud variable

CRGB *leds = NULL;
int pixelOffsetsInLEDsArray[NUMBER_OF_OUTPUTS] = {0};
int numberOfPixels = 0;

// An UDP instance to let us send and receive packets over UDP
UDP udp;
int lastUDPPacketReceiveTime = 0;

bool previousWiFiReadiness = false;
bool wiFiReadiness = false;
IPAddress myIp;
String myIpString = "";
String firmwareVersion = "0000000009";
String systemVersion = "";

uint8_t testingPixels = 0;
uint8_t rainbowHue = 0;
uint32_t lastTestingChangeTime = 0;

/* Diag functions */
void dumpError(e131_error_t error);
uint16_t parsePacket();
e131_error_t validateE131Packet();

// My functions
void readEEPROMData();
void outputSettingsToString();
int updateParameters(String message);
void messageValues(String theString, int *messageValues);
void messageStrings(String theString, int valueToRetrieve, char *theName);

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

    // Auto Wi-Fi Antenna Selection
    WiFi.selectAntenna(ANT_AUTO); // ANT_INTERNAL ANT_EXTERNAL ANT_AUTO

    // Read from EEPROM
    readEEPROMData();

    // Setup the LED outputs
    setupLEDs();

    // Setup the Serial connection for debugging
    Serial.begin(115200);

    // Setup cloud variables and functions
    systemVersion = System.version();
    myIp = WiFi.localIP();
    myIpString = String(String(myIp[0], DEC) + "." + String(myIp[2], DEC) + "." + String(myIp[2], DEC) + "." + String(myIp[3], DEC));
    Particle.variable("outputConfig", outputSettingsCharArray);
    Particle.variable("universeSize", eepromData.universeSize);
    Particle.variable("localIP", myIpString);
    Particle.variable("e131FVersion", firmwareVersion);
    Particle.variable("sysVersion", systemVersion);
    Particle.function("updateParams", updateParameters);

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
        myIpString = String(String(myIp[0], DEC) + "." + String(myIp[2], DEC) + "." + String(myIp[2], DEC) + "." + String(myIp[3], DEC));
        Serial.print("ip:");
        Serial.println(myIp);
        udp.begin(E131_DEFAULT_PORT);
    }

    if(testingPixels)
    {
        if(testingPixels == TEST_RAINBOW)
        {
            // FastLED's built-in rainbow generator
            fill_rainbow(leds, numberOfPixels, rainbowHue);
            rainbowHue ++;
            FastLED.show();
        }
        else if(testingPixels == TEST_RGB)
        {
            if(millis() > lastTestingChangeTime + 500)
            {
                lastTestingChangeTime = millis();

                if(rainbowHue == 0)
                {
                    fill_solid(leds, numberOfPixels, CRGB(255, 0, 0));
                    rainbowHue ++;
                }
                else if(rainbowHue == 1)
                {
                    fill_solid(leds, numberOfPixels, CRGB(0, 255, 0));
                    rainbowHue ++;
                }
                else
                {
                    fill_solid(leds, numberOfPixels, CRGB(0, 0, 255));
                    rainbowHue = 0;
                }
                FastLED.show();
            }
        }
    }

    /* Parse a packet and update pixels */
    int dataSize = parsePacket();
    if(dataSize > 0)
    {
        // For some reason, the packet for universe 7 is not being received reliably, so instead we are drawing slighly late. As soon as the next round of packets is starting
        /*if(universe == 1)
        {
            FastLED.show();
        }*/

        Serial.print("u:");
        Serial.println(universe);
        
        int outputsUsingThisUniverse[NUMBER_OF_OUTPUTS] = {-1};
        int numberOfOutputsUsingThisUniverse = 0;
        // Determine which, if any outputs are using this universe of data and are controlling pixels
        for(int i = 0; i < NUMBER_OF_OUTPUTS; i ++)
        {
            if(eepromData.outputSettings[i][NUMBER_OF_PIXELS] > 0 && (eepromData.outputSettings[i][START_UNIVERSE] == universe || eepromData.outputSettings[i][END_UNIVERSE] == universe))
            {
                Serial.print("o:");
                Serial.print(i);
                Serial.print("su:");
                Serial.print(eepromData.outputSettings[i][START_UNIVERSE]);
                Serial.print("eu:");
                Serial.println(eepromData.outputSettings[i][END_UNIVERSE]);
         
                outputsUsingThisUniverse[numberOfOutputsUsingThisUniverse] = i;
                numberOfOutputsUsingThisUniverse ++;
            }
        }

        // Extract the dmx data and store it in each LED
        int ledIndex = 0;
        for(int channel = 0; channel < dataSize; channel ++)
        {
            // Check to see if this channel is being using for an output that is using this universe
            for(int i = 0; i < numberOfOutputsUsingThisUniverse; i ++)
            {
                /*Serial.print("o:");
                Serial.print(outputsUsingThisUniverse[i]);
                Serial.print("sc:");
                Serial.print(eepromData.outputSettings[outputsUsingThisUniverse[i]][START_CHANNEL]);
                Serial.print("ec:");
                Serial.println(eepromData.outputSettings[outputsUsingThisUniverse[i]][END_CHANNEL]);*/
                
                // See if we found a channel/output match
                /*if(channel >= eepromData.outputSettings[outputsUsingThisUniverse[i]][START_CHANNEL] && channel <= eepromData.outputSettings[outputsUsingThisUniverse[i]][END_CHANNEL])
                {
                    // Determine which LED in our array we should be writing to
                    ledIndex = pixelOffsetsInLEDsArray[outputsUsingThisUniverse[i]] + channel / 3;
                    
                    /*Serial.print("c:");
                    //Serial.print(channel);
                    //Serial.print("led:");
                    //Serial.println(ledIndex);
                    
                    // Determine which color of data this channel is for
                    if((channel - eepromData.outputSettings[outputsUsingThisUniverse[i]][START_CHANNEL]) % 3 == 0)
                    {
                        //Serial.print("r:");
                        //Serial.println(data[channel]);
                        
                        leds[ledIndex].r = data[channel];
                    }
                    else if((channel - eepromData.outputSettings[outputsUsingThisUniverse[i]][START_CHANNEL]) % 3 == 1)
                    {
                        //Serial.print("g:");
                        //Serial.println(data[channel]);
                        
                        leds[ledIndex].g = data[channel];
                    }
                    else
                    {
                        //Serial.print("b:");
                        //Serial.println(data[channel]);
                        
                        leds[ledIndex].b = data[channel];
                    }
                }*/
                
                
                int universeShiftedChannel = channel + (universe - 1) * eepromData.universeSize;
                int universeShiftedStartChannel = eepromData.outputSettings[outputsUsingThisUniverse[i]][START_CHANNEL] + (eepromData.outputSettings[outputsUsingThisUniverse[i]][START_UNIVERSE] - 1) * eepromData.universeSize;
                int universeShiftedEndChannel = eepromData.outputSettings[outputsUsingThisUniverse[i]][END_CHANNEL] + (eepromData.outputSettings[outputsUsingThisUniverse[i]][END_UNIVERSE] - 1) * eepromData.universeSize;
                
                if(channel < 9)
                {
                    Serial.print("o:");
                    Serial.print(outputsUsingThisUniverse[i]);
                    Serial.print("uc:");
                    Serial.print(universeShiftedChannel);
                    Serial.print("usc:");
                    Serial.print(universeShiftedStartChannel);
                    Serial.print("uec:");
                    Serial.println(universeShiftedEndChannel);
                }
                
                // See if we found a channel/output match
                if(universeShiftedChannel >= universeShiftedStartChannel && universeShiftedChannel <= universeShiftedEndChannel)
                {
                    // Determine which LED in our array we should be writing to
                    ledIndex = pixelOffsetsInLEDsArray[outputsUsingThisUniverse[i]] + universeShiftedChannel / 3;
                    
                    if(channel < 9)
                    {
                        Serial.print("c:");
                        Serial.print(channel);
                        Serial.print("v:");
                        Serial.print(data[channel]);
                        Serial.print("led:");
                        Serial.println(ledIndex);
                    }
                    
                    // Determine which color of data this channel is for
                    if((universeShiftedChannel - universeShiftedStartChannel) % 3 == 0)
                    {
                        if(channel < 9)
                        {
                            Serial.println("r");
                        }
                        
                        leds[ledIndex].r = data[channel];
                    }
                    else if((universeShiftedChannel - universeShiftedStartChannel) % 3 == 1)
                    {
                        if(channel < 9)
                        {
                            Serial.println("g");
                        }
                        
                        leds[ledIndex].g = data[channel];
                    }
                    else
                    {
                        if(channel < 9)
                        {
                            Serial.println("b");
                        }
                        
                        leds[ledIndex].b = data[channel];
                    }
                }
            }
            
            
            
            
            /*ledIndex = (channel + (universe - 1) * eepromData.universeSize) / 3;

            if(ledIndex < NUMBER_OF_PIXELS)
            {
                if(channel % 3 == 0)
                {
                    leds[ledIndex].r = data[channel];
                }
                else if(channel % 3 == 1)
                {
                    leds[ledIndex].g = data[channel];
                }
                else
                {
                    leds[ledIndex].b = data[channel];
                }
            }*/
        }

        FastLED.show();

        // For some reason, the packet for universe 7 is not being received reliably
        /*if(universe == 7)
         {
         FastLED.show();
         }*/
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
            data = packet->property_values;
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
        for(byte i = 0; i < NUMBER_OF_OUTPUTS; i ++)
        {
            // Initialize outputSettings
            eepromData.outputSettings[i][PIXEL_TYPE] = 0; // Define as WS2811
            eepromData.outputSettings[i][NUMBER_OF_PIXELS] = 0;
            eepromData.outputSettings[i][START_UNIVERSE] = 1;
            eepromData.outputSettings[i][START_CHANNEL] = 0;
            eepromData.outputSettings[i][END_UNIVERSE] = 1;
            eepromData.outputSettings[i][END_CHANNEL] = 0;

            String("No Name").toCharArray(eepromData.outputNames[i], 32);
            //eepromData.outputNames[i] = String("No Name");
        }

        // Init universe size
        eepromData.universeSize = DEFAULT_UNIVERSE_SIZE;

        // Init eeprom version and ID
        eepromData.version = EEPROM_VERSION;
        eepromData.id = EEPROM_ID;

        // Save to EEPROM
        EEPROM.put(EEPROM_DATA_ADDRESS, eepromData);
    }

    // data structure has changed, update the model
    /*if(eepromData.version != EEPROM_VERSION)
     {

     }*/

    // Convert pinMaps to char arrays for cloud variable access
    outputSettingsToString();
}

// Convert the outputSettings to a string for cloud access
void outputSettingsToString()
{
    memset(outputSettingsCharArray, 0, ((uint8_t)NUMBER_OF_OUTPUTS * NUMBER_OF_PIXEL_PIN_MAP_ITEMS * 6) + NUMBER_OF_OUTPUTS + 2);
    outputSettingsCharArray[0] = '\0';

    for(byte i = 0; i < NUMBER_OF_OUTPUTS; i ++)
    {
        sprintf(outputSettingsCharArray, "%s%u,%u,%u,%u,%u,%u,%s;", outputSettingsCharArray, eepromData.outputSettings[i][PIXEL_TYPE], eepromData.outputSettings[i][NUMBER_OF_PIXELS], eepromData.outputSettings[i][START_UNIVERSE], eepromData.outputSettings[i][START_CHANNEL], eepromData.outputSettings[i][END_UNIVERSE], eepromData.outputSettings[i][END_CHANNEL], eepromData.outputNames[i]);
    }
}

// This is the cloud function "updateParameters"
// The format of the string should look something like this: "usz,512," or gfo,1,255,2,228,3,255,"
int updateParameters(String message)
{
    int values[10];
    char theName[32];
    messageValues(message, values);

    if(values[0] == NAME_FOR_OUTPUT)
    {
        messageStrings(message, 2, theName);
    }

    switch (values[0])
    {
        case SYSTEM_RESET:
            System.reset();
        case TEST_ALL:
            // do something
            testingPixels = values[1];

            if(testingPixels == TEST_OFF)
            {
                fill_solid(leds, numberOfPixels, CRGB(0, 0, 0));
                FastLED.show();
            }
            else if(testingPixels == TEST_RGB)
            {
                rainbowHue = 0;
            }
            else if(testingPixels == TEST_WHITE)
            {
                fill_solid(leds, numberOfPixels, CRGB(255, 255, 255));
                FastLED.show();
            }
            break;
        case SAVE:
            // Save to EEPROM
            EEPROM.put(EEPROM_DATA_ADDRESS, eepromData);
            // Convert pinMaps and gammaSettings to char arrays for cloud variable access
            outputSettingsToString();
            break;
        case UNIVERSE_SIZE:
            // Update the universeSize
            eepromData.universeSize = values[1];
            break;
        case CHANNEL_MAP_FOR_OUTPUT: // output,pixelType,numberOfPixels,startUniverse,startChannel,endUniverse,endChannel,
            // Update pin map
            eepromData.outputSettings[values[1]][PIXEL_TYPE] = values[2];
            eepromData.outputSettings[values[1]][NUMBER_OF_PIXELS] = values[3];
            eepromData.outputSettings[values[1]][START_UNIVERSE] = values[4];
            eepromData.outputSettings[values[1]][START_CHANNEL] = values[5];
            eepromData.outputSettings[values[1]][END_UNIVERSE] = values[6];
            eepromData.outputSettings[values[1]][END_CHANNEL] = values[7];
            // Convert outputSettings to a string for cloud variable access
            outputSettingsToString();
            // Resetup our leds array
            setupLEDs();
            break;
        case PIXEL_TYPE_FOR_OUTPUT:
            eepromData.outputSettings[values[1]][PIXEL_TYPE] = values[2];
            // Convert outputSettings to a string for cloud variable access
            outputSettingsToString();
            // Resetup our leds array
            setupLEDs();
            break;
        case NUMBER_OF_PIXELS_FOR_OUTPUT:
            eepromData.outputSettings[values[1]][NUMBER_OF_PIXELS] = values[2];
            // Convert outputSettings to a string for cloud variable access
            outputSettingsToString();
            // Resetup our leds array
            setupLEDs();
            break;
        case START_UNIVERSE_FOR_OUTPUT:
            eepromData.outputSettings[values[1]][START_UNIVERSE] = values[2];
            // Convert outputSettings to a string for cloud variable access
            outputSettingsToString();
            break;
        case START_CHANNEL_FOR_OUTPUT:
            eepromData.outputSettings[values[1]][START_CHANNEL] = values[2];
            // Convert outputSettings to a string for cloud variable access
            outputSettingsToString();
            break;
        case END_UNIVERSE_FOR_OUTPUT:
            eepromData.outputSettings[values[1]][END_UNIVERSE] = values[2];
            // Convert outputSettings to a string for cloud variable access
            outputSettingsToString();
            break;
        case END_CHANNEL_FOR_OUTPUT:
            eepromData.outputSettings[values[1]][END_CHANNEL] = values[2];
            // Convert outputSettings to a string for cloud variable access
            outputSettingsToString();
            break;
        case NAME_FOR_OUTPUT:
            strcpy(eepromData.outputNames[values[1]], theName);
            // Convert outputSettings to a string for cloud variable access
            outputSettingsToString();
            break;
        default:
            // Invalid message
            return -1;
            break;
    }

    return 1;
}

// All values should be ints
void messageValues(String theString, int *messageValues)
{
    bool doneReadingString = false;
    int index = 0;
    while(!doneReadingString)
    {
        int commaIndex = theString.indexOf(",");
        if(commaIndex != -1 || (commaIndex == -1 && theString.length() > 0))
        {
            String valueString = theString.substring(0, (commaIndex != -1 ? commaIndex : theString.length()));
            if(valueString.length() > 0)
            {
                messageValues[index] = valueString.toInt();
                ++index;
                theString = theString.substring((commaIndex != -1 ? commaIndex : theString.length() - 1) + 1);
            }
        }
        // This isn't a foolproof method of parsing the string, but it will work. (Meaning if the string is "5,10,,15" the 15 will never get read because the double commans will end the loop)
        else
        {
            doneReadingString = true;
        }
    }
}

void messageStrings(String theString, int valueToRetrieve, char *theName)
{
    bool doneReadingString = false;
    int index = 0;
    while(!doneReadingString)
    {
        int commaIndex = theString.indexOf(",");
        if(commaIndex != -1 || (commaIndex == -1 && theString.length() > 0))
        {
            String valueString = theString.substring(0, (commaIndex != -1 ? commaIndex : theString.length()));
            if(valueString.length() > 0)
            {
                if(index == valueToRetrieve)
                {
                    valueString.toCharArray(theName, 32);
                    return;
                }

                ++index;
                theString = theString.substring((commaIndex != -1 ? commaIndex : theString.length() - 1) + 1);
            }
        }
        // This isn't a foolproof method of parsing the string, but it will work. (Meaning if the string is "5,10,,15" the 15 will never get read because the double commans will end the loop)
        else
        {
            doneReadingString = true;
        }
    }

    String valueString = String("No Name");
    valueString.toCharArray(theName, 32);
}

void setupLEDs()
{
    // Turn off all the old LEDs if there are any
    fill_solid(leds, numberOfPixels, CHSV(0, 0, 0));
    FastLED.show();

    // Determine the total number of pixels
    numberOfPixels = 0; // Reset the numberOfPixels
    int numberOfPixelsForThisOutput = 0;
    for(int i = 0; i < NUMBER_OF_OUTPUTS; i ++)
    {
        if(eepromData.outputSettings[i][NUMBER_OF_PIXELS] > 0)
        {
            numberOfPixelsForThisOutput = eepromData.outputSettings[i][NUMBER_OF_PIXELS];
            // Add these leds to the count
            numberOfPixels += numberOfPixelsForThisOutput;
        }
    }

    // Realloc leds array (not malloc, since this methods can be called multiple times while the application is running)
    leds = (CRGB *)realloc(leds, numberOfPixels * sizeof(CRGB));
    // Initialize leds to 0
    memset(leds, 0, numberOfPixels * sizeof(CRGB));

    // Add the pixels to the led array of the appropriate type and output pin
    int pixelOffset = 0;
    for(int i = 0; i < NUMBER_OF_OUTPUTS; i ++)
    {
        if(eepromData.outputSettings[i][NUMBER_OF_PIXELS] > 0)
        {
            numberOfPixelsForThisOutput = eepromData.outputSettings[i][NUMBER_OF_PIXELS];
            pixelOffsetsInLEDsArray[i] = pixelOffset;

            switch(eepromData.outputSettings[i][PIXEL_TYPE])
            {
                case jWS2811:
                    switch(i)
                {
                    case 0:
                        FastLED.addLeds<WS2811, 0>(leds, pixelOffset, numberOfPixelsForThisOutput);
                        break;
                    case 1:
                        FastLED.addLeds<WS2811, 1>(leds, pixelOffset, numberOfPixelsForThisOutput);
                        break;
                    case 2:
                        FastLED.addLeds<WS2811, 2>(leds, pixelOffset, numberOfPixelsForThisOutput);
                        break;
                    case 3:
                        FastLED.addLeds<WS2811, 3>(leds, pixelOffset, numberOfPixelsForThisOutput);
                        break;
                    case 4:
                        FastLED.addLeds<WS2811, 4>(leds, pixelOffset, numberOfPixelsForThisOutput);
                        break;
                    case 5:
                        FastLED.addLeds<WS2811, 5>(leds, pixelOffset, numberOfPixelsForThisOutput);
                        break;
                    case 6:
                        FastLED.addLeds<WS2811, 6>(leds, pixelOffset, numberOfPixelsForThisOutput);
                        break;
                    case 7:
                        FastLED.addLeds<WS2811, 7>(leds, pixelOffset, numberOfPixelsForThisOutput);
                        break;
                }
                    break;
                case jWS2811_400:
                    switch(i)
                {
                    case 0:
                        FastLED.addLeds<WS2811_400, 0>(leds, pixelOffset, numberOfPixelsForThisOutput);
                        break;
                    case 1:
                        FastLED.addLeds<WS2811_400, 1>(leds, pixelOffset, numberOfPixelsForThisOutput);
                        break;
                    case 2:
                        FastLED.addLeds<WS2811_400, 2>(leds, pixelOffset, numberOfPixelsForThisOutput);
                        break;
                    case 3:
                        FastLED.addLeds<WS2811_400, 3>(leds, pixelOffset, numberOfPixelsForThisOutput);
                        break;
                    case 4:
                        FastLED.addLeds<WS2811_400, 4>(leds, pixelOffset, numberOfPixelsForThisOutput);
                        break;
                    case 5:
                        FastLED.addLeds<WS2811_400, 5>(leds, pixelOffset, numberOfPixelsForThisOutput);
                        break;
                    case 6:
                        FastLED.addLeds<WS2811_400, 6>(leds, pixelOffset, numberOfPixelsForThisOutput);
                        break;
                    case 7:
                        FastLED.addLeds<WS2811_400, 7>(leds, pixelOffset, numberOfPixelsForThisOutput);
                        break;
                }
                    break;
                case jNEOPIXEL:
                    switch(i)
                {
                    case 0:
                        FastLED.addLeds<NEOPIXEL, 0>(leds, pixelOffset, numberOfPixelsForThisOutput);
                        break;
                    case 1:
                        FastLED.addLeds<NEOPIXEL, 1>(leds, pixelOffset, numberOfPixelsForThisOutput);
                        break;
                    case 2:
                        FastLED.addLeds<NEOPIXEL, 2>(leds, pixelOffset, numberOfPixelsForThisOutput);
                        break;
                    case 3:
                        FastLED.addLeds<NEOPIXEL, 3>(leds, pixelOffset, numberOfPixelsForThisOutput);
                        break;
                    case 4:
                        FastLED.addLeds<NEOPIXEL, 4>(leds, pixelOffset, numberOfPixelsForThisOutput);
                        break;
                    case 5:
                        FastLED.addLeds<NEOPIXEL, 5>(leds, pixelOffset, numberOfPixelsForThisOutput);
                        break;
                    case 6:
                        FastLED.addLeds<NEOPIXEL, 6>(leds, pixelOffset, numberOfPixelsForThisOutput);
                        break;
                    case 7:
                        FastLED.addLeds<NEOPIXEL, 7>(leds, pixelOffset, numberOfPixelsForThisOutput);
                        break;
                }
                    break;
                case jWS2812:
                    switch(i)
                {
                    case 0:
                        FastLED.addLeds<WS2812, 0>(leds, pixelOffset, numberOfPixelsForThisOutput);
                        break;
                    case 1:
                        FastLED.addLeds<WS2812, 1>(leds, pixelOffset, numberOfPixelsForThisOutput);
                        break;
                    case 2:
                        FastLED.addLeds<WS2812, 2>(leds, pixelOffset, numberOfPixelsForThisOutput);
                        break;
                    case 3:
                        FastLED.addLeds<WS2812, 3>(leds, pixelOffset, numberOfPixelsForThisOutput);
                        break;
                    case 4:
                        FastLED.addLeds<WS2812, 4>(leds, pixelOffset, numberOfPixelsForThisOutput);
                        break;
                    case 5:
                        FastLED.addLeds<WS2812, 5>(leds, pixelOffset, numberOfPixelsForThisOutput);
                        break;
                    case 6:
                        FastLED.addLeds<WS2812, 6>(leds, pixelOffset, numberOfPixelsForThisOutput);
                        break;
                    case 7:
                        FastLED.addLeds<WS2812, 7>(leds, pixelOffset, numberOfPixelsForThisOutput);
                        break;
                }
                    break;
                case jWS2812B:
                    switch(i)
                {
                    case 0:
                        FastLED.addLeds<WS2812B, 0>(leds, pixelOffset, numberOfPixelsForThisOutput);
                        break;
                    case 1:
                        FastLED.addLeds<WS2812B, 1>(leds, pixelOffset, numberOfPixelsForThisOutput);
                        break;
                    case 2:
                        FastLED.addLeds<WS2812B, 2>(leds, pixelOffset, numberOfPixelsForThisOutput);
                        break;
                    case 3:
                        FastLED.addLeds<WS2812B, 3>(leds, pixelOffset, numberOfPixelsForThisOutput);
                        break;
                    case 4:
                        FastLED.addLeds<WS2812B, 4>(leds, pixelOffset, numberOfPixelsForThisOutput);
                        break;
                    case 5:
                        FastLED.addLeds<WS2812B, 5>(leds, pixelOffset, numberOfPixelsForThisOutput);
                        break;
                    case 6:
                        FastLED.addLeds<WS2812B, 6>(leds, pixelOffset, numberOfPixelsForThisOutput);
                        break;
                    case 7:
                        FastLED.addLeds<WS2812B, 7>(leds, pixelOffset, numberOfPixelsForThisOutput);
                        break;
                }
                    break;
                case jAPA104:
                    switch(i)
                {
                    case 0:
                        FastLED.addLeds<APA104, 0>(leds, pixelOffset, numberOfPixelsForThisOutput);
                        break;
                    case 1:
                        FastLED.addLeds<APA104, 1>(leds, pixelOffset, numberOfPixelsForThisOutput);
                        break;
                    case 2:
                        FastLED.addLeds<APA104, 2>(leds, pixelOffset, numberOfPixelsForThisOutput);
                        break;
                    case 3:
                        FastLED.addLeds<APA104, 3>(leds, pixelOffset, numberOfPixelsForThisOutput);
                        break;
                    case 4:
                        FastLED.addLeds<APA104, 4>(leds, pixelOffset, numberOfPixelsForThisOutput);
                        break;
                    case 5:
                        FastLED.addLeds<APA104, 5>(leds, pixelOffset, numberOfPixelsForThisOutput);
                        break;
                    case 6:
                        FastLED.addLeds<APA104, 6>(leds, pixelOffset, numberOfPixelsForThisOutput);
                        break;
                    case 7:
                        FastLED.addLeds<APA104, 7>(leds, pixelOffset, numberOfPixelsForThisOutput);
                        break;
                }
                    break;
                case jLPD1886:
                    switch(i)
                {
                    case 0:
                        FastLED.addLeds<LPD1886, 0>(leds, pixelOffset, numberOfPixelsForThisOutput);
                        break;
                    case 1:
                        FastLED.addLeds<LPD1886, 1>(leds, pixelOffset, numberOfPixelsForThisOutput);
                        break;
                    case 2:
                        FastLED.addLeds<LPD1886, 2>(leds, pixelOffset, numberOfPixelsForThisOutput);
                        break;
                    case 3:
                        FastLED.addLeds<LPD1886, 3>(leds, pixelOffset, numberOfPixelsForThisOutput);
                        break;
                    case 4:
                        FastLED.addLeds<LPD1886, 4>(leds, pixelOffset, numberOfPixelsForThisOutput);
                        break;
                    case 5:
                        FastLED.addLeds<LPD1886, 5>(leds, pixelOffset, numberOfPixelsForThisOutput);
                        break;
                    case 6:
                        FastLED.addLeds<LPD1886, 6>(leds, pixelOffset, numberOfPixelsForThisOutput);
                        break;
                    case 7:
                        FastLED.addLeds<LPD1886, 7>(leds, pixelOffset, numberOfPixelsForThisOutput);
                        break;
                }
                    break;
                default:
                    break;
            }

            pixelOffset += numberOfPixelsForThisOutput;
        }
    }

    FastLED.show();
}
