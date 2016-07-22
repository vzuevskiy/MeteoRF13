#include <SPI.h>         // needed for Arduino versions later than 0018
#include <Ethernet.h>
#include <EthernetUdp.h>         // UDP library from: bjoern@cs.stanford.edu 12/30/2008
#include <OneWire.h>
#include "DHT.h"

/********************************************/
//
//    Настройки датчиков
//    Назначение выводов
//    PIN2 = DHT22
//    PIN3 = Dallas
//
/********************************************/

#define DHTPIN 2
#define DHTTYPE DHT22

DHT dht(DHTPIN, DHTTYPE);
OneWire  ds(3);             // on pin 3 (a 4.7K resistor is necessary)



/********************************************/
//
//    Настройки Ethernet шилда
//
/********************************************/

byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
IPAddress ip(1, 1, 1, 2);
unsigned int localPort = 45454;      // local port to listen on
char packetBuffer[UDP_TX_PACKET_MAX_SIZE];  //buffer to hold incoming packet,
EthernetUDP Udp;
EthernetClient client;
IPAddress server;

/********************************************/
//
//    Функция инициализации устройства
//    При получении пакета INIT, определяет
//    IP адрес сервера для дальнейшей работы
//    INIT получен - Зеленый диод горит
//
/********************************************/

void waitInit()
{
  digitalWrite(5, HIGH);
  digitalWrite(6, HIGH);
  int waitTime = 0;
  while (1) {
    int packetSize = Udp.parsePacket();
    if (packetSize)
    {
      Udp.read(packetBuffer, UDP_TX_PACKET_MAX_SIZE);
      Serial.println(packetBuffer);

      if (!strcmp(packetBuffer, "INIT")) {
        Serial.println(Ethernet.localIP());
        server = Udp.remoteIP();
        digitalWrite(6, LOW);
        break;
      }
    }
    waitTime++;
    if (waitTime == 6000) {      // если не приходит init раз в минуту проверять работу сети
      setup();
    }
    delay(10);
  }

}



/********************************************/
//
//    Setup() - старт скетча
//    1. Держим красный диод включенным пока
//    не получим IP от DHCP
//    2. вызов функции waitInit()
//
/********************************************/

void setup() {
  Serial.begin(9600);
  pinMode(5, OUTPUT);         // желтый
  pinMode(6, OUTPUT);         // красный
  digitalWrite(5, LOW);
  digitalWrite(6, LOW);

  digitalWrite(6, HIGH);

  while (1) {
    if (Ethernet.begin(mac)) {
      Udp.begin(localPort);
      break;
    }
  }

  waitInit();
  dht.begin();
}

int countErrConnection = 0;


void loop() {
  int inc = 0;

  byte i;
  byte present = 0;
  byte type_s;
  byte data[12];
  byte addr[8];
  float celsius = 0;

  while (1) {

    if ( !ds.search(addr)) {
      ds.reset_search();
      delay(250);
      break;
      return;
    }


    if (OneWire::crc8(addr, 7) != addr[7]) {
      return;
    }

    ds.reset();
    ds.select(addr);
    ds.write(0x44, 1);        // start conversion, with parasite power on at the end

    delay(1000);     // maybe 750ms is enough, maybe not
    // we might do a ds.depower() here, but the reset will take care of it.

    present = ds.reset();
    ds.select(addr);
    ds.write(0xBE);         // Read Scratchpad

    for ( i = 0; i < 9; i++) {           // we need 9 bytes
      data[i] = ds.read();
    }

    int16_t raw = (data[1] << 8) | data[0];
    byte cfg = (data[4] & 0x60);
    // at lower res, the low bits are undefined, so let's zero them
    if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
    else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
    else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
    // default is 12 bit resolution, 750 ms conversion time
    celsius += (float)raw / 16.0;
    inc++;
    //Serial.println((float)raw / 16.0);
  }
  //  Serial.println("------------");

  float h = dht.readHumidity();
  float t = dht.readTemperature();

  if (isnan(t) || isnan(h)) {
    // Serial.println("Failed to read from DHT");
    for (int i = 1; i < 6; i++) {
      digitalWrite(6, HIGH);
      delay(150);
      digitalWrite(6, LOW);
      delay(150);
    }
  }
  else {
    inc++;
    if (inc == 1) {
      for (int i = 1; i < 6; i++) {
        digitalWrite(5, LOW);
        delay(150);
        digitalWrite(5, HIGH);
        delay(150);
      }
    }
  }

  float tout = (celsius + t) / inc;

  if (client.connect(server, 7364)) {
    //char out[256] = "ID:001IP:" + Ethernet.localIP();
    client.print("METEO,");
    client.print("S0001,T");
    client.print(tout);
    client.print(",H");
    client.print(h);
    client.print(",INC");
    client.print(inc);
    client.stop();

    countErrConnection = 0;

    for (int i = 1; i < 3; i++) {
      digitalWrite(5, LOW);
      delay(350);
      digitalWrite(5, HIGH);
      delay(350);
    }

  } else {
    // if you didn't get a connection to the server:
    for (int i = 1; i < 3; i++) {
      digitalWrite(6, HIGH);
      delay(350);
      digitalWrite(6, LOW);
      delay(350);
    }

    countErrConnection++;
    if (countErrConnection == 4) {
      setup();                  // если 4 раз данные не удастся передать - начать выполнение скетча с начала
    }
  }

  delay(150000);
}


