//SerialExpander
//A Part Of The MCT Access Control Project
//
//GPIO Expansion Controlled Via Telnet With Integrated Event System

#include <UIPEthernet.h>
#include <DHT.h>
#include "EEPROM.h"
#include "avr/pgmspace.h"

const uint8_t BRINGUP_FLAG = 0x09;

const uint8_t ACKNOWLEDGE = 1;
const uint8_t NOT_ACKNOWLEDGE = 0;

const uint8_t DIGITAL_OUTPUT = 1;
const uint8_t ANALOG_OUTPUT = 2;
const uint8_t DIGITAL_INPUT = 3;
const uint8_t ANALOG_INPUT = 4;
const uint8_t TEMPERATURE_SENSOR = 5;
const uint8_t HUMIDITY_SENSOR = 6;
const uint8_t UNSPECIFIED PROGMEM = 10;

const uint8_t num_supported_features = 3;
const uint8_t supported_features[] = {DIGITAL_OUTPUT, TEMPERATURE_SENSOR, HUMIDITY_SENSOR};
const uint8_t feature_count[] = {8, 1, 1};

const uint16_t LISTENING_PORT = 8001;

const uint8_t DIGITAL_OUTPUT_PINS[] = {9, 8, 7, 6, 5, 4, 3, 2};
const uint8_t NUM_RELAY_PINS = 8;

#define DHTPIN A1
#define DHTTYPE DHT11
const uint16_t DHT_READ_INTERVAL = 2000;

EthernetUDP udp;
EthernetClient client;
DHT dht(DHTPIN, DHTTYPE);

IPAddress server_address;
uint16_t server_port;

uint32_t last_dht_reading = 0;

bool comm_waiting = false;
uint32_t comm_wait_timer = 0;

uint8_t server_wdt_enable = true;
uint32_t server_wdt = 0;
uint16_t server_wdt_timeout = 10000;

float current_temperature = 0;
float current_humidity = 0;

uint8_t mac_address[6] = {0x00,0x01,0x02,0x03,0x04,0x05};

byte device_id[16];

void setup()
{
	  if(EEPROM.read(0) != BRINGUP_FLAG)
    {
      for(int i = 1; i < 17; i++)
        EEPROM.write(i, 0);

      EEPROM.write(0, BRINGUP_FLAG);
    }
    
	    for(int i = 1; i < 17; i++)
        device_id[i-1] = EEPROM.read(i);
 
	initPins();
  resetEthernetModule();

  dht.begin();
  readDHT11();

  //keep trying until we get an ip
  while(Ethernet.begin(mac_address) == 0);

  udp.begin(LISTENING_PORT);
  while(getServerAddress() == 0);
  udp.stop();
}

void loop()
{ 
  if(!client.connected())
    if(client.connect(server_address, server_port) != 1)
      restart();
    else
        server_wdt = millis();

  if(server_wdt_enable)
    if((uint32_t)((long)millis() - server_wdt) > server_wdt_timeout)
      restart();

      if((uint32_t)((long)millis() - last_dht_reading) > DHT_READ_INTERVAL)
   readDHT11();
    
  if(client.available() > 0)
  {
    digitalWrite(A2, HIGH);
    processPacket();
    digitalWrite(A2, LOW);
  }
  
	switch(Ethernet.maintain())
 {
  case 1: //renew failure
    restart();
  case 3: //rebind failure
    restart();
  default:
    break;
 }
}

void processPacket()
{
  while(true)
  {
    uint8_t psize = client.peek();
    if(client.available() == 0 || client.available() < psize)
    {
      if(!comm_waiting)
      {
        comm_wait_timer = millis();
        comm_waiting = true;
      }
      else
        if((uint32_t)((long)millis() - comm_wait_timer) > 10000)
        {
          client.flush();
          comm_waiting = false;
        }
      
      return;
    }
    else
      if(client.available() > 48)
      {
        client.flush();
        return;
      }
      else
        comm_waiting = false;
  
  uint8_t packet_buffer[psize];
  client.read(packet_buffer, psize);

  server_wdt = millis();
  
  if(packet_buffer[1] == 'w') //write to device feature
    {
      switch(packet_buffer[2]) //feature code
      {
        case DIGITAL_OUTPUT:
        if(packet_buffer[3] < 8)
        {
        setRelay(packet_buffer[3], packet_buffer[4]); //packet_bufer[3] is the feature index, packet_buffer[4] is the value
        client.write((uint8_t*)&ACKNOWLEDGE, 1);
        }
        else
          client.write((uint8_t*)&NOT_ACKNOWLEDGE, 1);
        break;
        default:
        client.write((uint8_t*)&NOT_ACKNOWLEDGE, 1);
        break;
      }
    }
    else
      if(packet_buffer[1] == 'r') //read from device feature
      {
        switch(packet_buffer[2]) //feature code
        {
          case DIGITAL_OUTPUT:
          if(packet_buffer[3] < 8)
          {
            client.write((uint8_t*)&ACKNOWLEDGE, 1);
            uint8_t read_value = digitalRead(DIGITAL_OUTPUT_PINS[packet_buffer[3]]);
            client.write((uint8_t*)&read_value, 1); //packet_buffer[3] is the feature index
          }
          else
          client.write((uint8_t*)&NOT_ACKNOWLEDGE, 1);
          break;
          
          case TEMPERATURE_SENSOR:
          if(packet_buffer[3] == 0)
          {
          client.write((uint8_t*)&ACKNOWLEDGE, 1);
          client.write((uint8_t*)&current_temperature, 4);
          }
          else
          client.write((uint8_t*)&NOT_ACKNOWLEDGE, 1);
          break;
          
          case HUMIDITY_SENSOR:
          if(packet_buffer[3] == 0)
          {
            client.write((uint8_t*)&ACKNOWLEDGE, 1);
            client.write((uint8_t*)&current_humidity, 4);
          }
          else
            client.write((uint8_t*)&NOT_ACKNOWLEDGE, 1);
          break;
          
          default:
          client.write((uint8_t*)&NOT_ACKNOWLEDGE, 1);
          break;
        }
      }
      else
        if(packet_buffer[1] == 'f') //general device feature inquiry
        {
          client.write((uint8_t*)&ACKNOWLEDGE, 1);
          client.write((uint8_t*)&num_supported_features, 1); //send the number of feature codes
          client.write(supported_features, num_supported_features); //send the feature codes
          client.write(feature_count, num_supported_features); //send the feature counts
        }
        else
              if(packet_buffer[1] == 'u') //get device uptime
              {
                client.write((uint8_t*)&ACKNOWLEDGE, 1);
                uint32_t uptime = millis();
                client.write((uint8_t*)&uptime, 4);
              }
              else
                if(packet_buffer[1] == 'k') //get device id
                {
                  client.write((uint8_t*)&ACKNOWLEDGE, 1);
                  client.write(device_id, 16);
                }
                else
                  if(packet_buffer[1] == 's') //set device id
                  {
                    memcpy(device_id, packet_buffer+2, 16);
                    for(int i = 1; i < 17; i++)
                      EEPROM.write(i, device_id[i-1]);
                    client.write((uint8_t*)&ACKNOWLEDGE, 1);
                  }
                  else
                    if(packet_buffer[1] == 'z') //restart device
                    {
                      client.write((uint8_t*)&ACKNOWLEDGE, 1);
                      restart();
                    }
                    else
                      if(packet_buffer[1] == 'p') //ping
                      {
                        client.write((uint8_t*)&ACKNOWLEDGE, 1);
                      }
                      else
                      client.write((uint8_t*)&NOT_ACKNOWLEDGE, 1);
  }
}

void readDHT11()
{
  current_temperature = dht.readTemperature(true);
  current_humidity = dht.readHumidity();

  last_dht_reading = millis();
}

void setRelays(uint8_t values[])
{
  for(int i = 0; i < 8; i++)
    setRelay(i, values[i]);
}

void setRelay(uint8_t relay_number, uint8_t value)
{
  digitalWrite(DIGITAL_OUTPUT_PINS[relay_number], !value);
}

void initPins()
{
	//relay pins
	for(int i = 0; i < NUM_RELAY_PINS; i++)
  {
    pinMode(DIGITAL_OUTPUT_PINS[i], OUTPUT);
    digitalWrite(DIGITAL_OUTPUT_PINS[i], HIGH);
  }

  pinMode(A2, OUTPUT);
  digitalWrite(A2, LOW);
		
	//Ethernet module reset pin
	pinMode(A0, OUTPUT);
  digitalWrite(A0, HIGH);
}

void resetEthernetModule()
{
	digitalWrite(A0, LOW);
	delay(500);
	digitalWrite(A0, HIGH);
  delay(500);
}

void restart()
{
  delay(10);
  asm volatile ("  jmp 0");  
}

uint8_t getServerAddress()
{
  int packet_size = udp.parsePacket();
  if(packet_size == 6)
  {
    uint8_t packet_buffer[packet_size];
    udp.read(packet_buffer, packet_size);

    server_address = IPAddress(packet_buffer[0], packet_buffer[1], packet_buffer[2], packet_buffer[3]);
    server_port = ((uint16_t)packet_buffer[4] << 8) | packet_buffer[5];

     return 1;
  }
  else
    return 0;
}

