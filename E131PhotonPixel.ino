#include "FastLED.h"
FASTLED_USING_NAMESPACE;

SYSTEM_THREAD(ENABLED);

#define PIXEL_PIN 0
#define NUMBER_OF_PIXELS 576

#define PIXEL_PIN_2 7
#define NUMBER_OF_PIXELS_2 576

// UDP Port used for two way communication
unsigned int e131Port = 5568;
// An UDP instance to let us send and receive packets over UDP
UDP udp;
CRGB leds[NUMBER_OF_PIXELS];
CRGB leds2[NUMBER_OF_PIXELS_2];
// Define the E1.31 packet header
const uint8_t e131PacketHeader[] = {0x00, 0x10, 0x00, 0x00, 0x41, 0x53, 0x43, 0x2D, 0x45, 0x31, 0x2E, 0x31, 0x37, 0x00, 0x00, 0x00};
#define E131_PACKET_HEADER_LENGTH 16
#define DMX512_OFFSET 126

bool previousWiFiReadiness = true;
bool wiFiReadiness = true;

int16_t e131PacketUniverseNumber = 0;

int lastSparkProcessTimeMillis = 0;
int ledIndex;
uint8_t r, g, b;
uint32_t pixelColor;

uint8_t udpData[640];

unsigned int packetCount = 0;
unsigned int lastPacketCount = 0;
unsigned int lastPacketReportTime = 0;

// Main setup functions
void setup()
{
    WiFi.selectAntenna(ANT_AUTO); // ANT_INTERNAL ANT_EXTERNAL ANT_AUTO

    // start the UDP listening
    udp.begin(e131Port);
    udp.setBuffer(640, udpData);

    // Print your device IP Address via serial
    Serial.begin(115200);
    Serial.println(WiFi.localIP());

    pinMode(PIXEL_PIN, OUTPUT);
    FastLED.addLeds<WS2811, PIXEL_PIN>(leds, NUMBER_OF_PIXELS);
    FastLED.addLeds<WS2811, PIXEL_PIN_2>(leds2, NUMBER_OF_PIXELS_2);
    FastLED.show();
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
        Serial.println(WiFi.localIP());
        udp.begin(e131Port);

        Serial.print("fm:");
        uint32_t freemem = System.freeMemory();
        Serial.println(freemem);

        displayIPOnMatrix();
    }

    if(millis() - lastPacketReportTime > 1000)
    {
        Serial.print("pkts/s:");
        Serial.println(packetCount - lastPacketCount);

        lastPacketCount = packetCount;
        lastPacketReportTime = millis();
    }

    // Parse any incoming UDP data
    udpParse();
}

void displayIPOnMatrix()
{
    IPAddress myIP = WiFi.localIP();

    displayValue(myIP[3]);

    FastLED.show();
}

void displayValue(uint8_t value)
{
    for(uint8_t i = 0; i < value; i ++)
    {
      leds[i] = CRGB::White;
    }
}

void udpParse()
{
    // Check if data has been received
    int packetSize = udp.parsePacket();
    if (packetSize > 0)
    {
        packetCount ++;

        //Serial.print("PacketSize:");
        //Serial.print(packetSize);
        //Serial.print(" avail:");
        //Serial.println(udp.available());

        // Read in the udp packet
        //char udpData[packetSize];
        udp.read(udpData, udp.available());

        // This is the start of an E1.31 packet
        if(isPacketE131Formatted(udpData, packetSize))
        {
            uint8_t *e131Packet = udpData;
            //uint8_t e131Packet[640];
            //memcpy(e131Packet, udpData, 640);

            // Find the packet universe number
            uint8_t *dmxData = e131Packet + DMX512_OFFSET;
            e131PacketUniverseNumber = e131Packet[114] | (e131Packet[113] << 8);

            //Serial.print("universe:");
            //Serial.println(e131PacketUniverseNumber);

            // Extract the dmx data and store it in each LED
            int i;
            CRGB *ledsToUse;
            for(i = 0; i < packetSize - DMX512_OFFSET; i ++)
            {
                int realI = i + (e131PacketUniverseNumber - 1) * 512;
                ledIndex = realI / 3;
                if(ledIndex >= NUMBER_OF_PIXELS)
                {
                  ledIndex -= NUMBER_OF_PIXELS;
                  ledsToUse = leds2;
                }
                else
                {
                  ledsToUse = leds;
                }
                if(ledIndex < NUMBER_OF_PIXELS)// && e131PacketUniverseNumber)
                {
                    if(realI % 3 == 0)
                    {
                        ledsToUse[ledIndex].r = dmxData[i];
                    }
                    else if(realI % 3 == 1)
                    {
                        ledsToUse[ledIndex].g = dmxData[i];
                    }
                    else
                    {
                        ledsToUse[ledIndex].b = dmxData[i];
                    }
                }
            }
        }

        // Write the data to the pixels
        if(e131PacketUniverseNumber == 7)
        {
            FastLED.show();
        }

        //printUDPData(udpData, packetSize);

        // Store sender ip and port
        //IPAddress ipAddress = udp.remoteIP();
        //int port = udp.remotePort();
    }
}

bool isPacketE131Formatted(uint8_t *udpData, int udpSize)
{
    boolean match = true;
    for (int i = 0; i < E131_PACKET_HEADER_LENGTH; i++)
    {
        // If the header bytes aren't the same, this isn't an E1.31 packet
        if(udpData[i] != e131PacketHeader[i])
        {
            Serial.print("Not an E1.31 Packet at[");
            Serial.print(i);
            Serial.print("] udp:");
            Serial.print(udpData[i], HEX);
            Serial.print(" != e131:");
            Serial.println(e131PacketHeader[i], HEX);
            match = false;
            break;
        }
    }

    return match;
}

void printUDPData(uint8_t *udpData, int size)
{
    for(int i = 0; i < size; i ++)
    {
        Serial.print(udpData[i], HEX);
        Serial.print(" ");
    }
    Serial.println();
}
