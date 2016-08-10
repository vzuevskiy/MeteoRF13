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
//    PIN3 = DS18b20
//
/********************************************/

#define DHTTYPE DHT22
#define DHTPIN 2
#define DS18PIN 3
#define YELLOWLEDPIN 5
#define REDLEDPIN 6


DHT dht(DHTPIN, DHTTYPE);
OneWire  ds(DS18PIN);

/********************************************/
//
//    Настройки сети
//
/********************************************/

byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };

EthernetUDP Udp;
unsigned int localPort = 45454;      // локальный порт для udp
char packetBuffer[UDP_TX_PACKET_MAX_SIZE];  //buffer to hold incoming packet,

EthernetClient client;
IPAddress server;
IPAddress ip;

void(* resetFunc) (void) = 0; // софт ресет

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
  digitalWrite(YELLOWLEDPIN, HIGH);
  digitalWrite(REDLEDPIN, HIGH);
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
        digitalWrite(REDLEDPIN, LOW);
        break;
      }
    }
    waitTime++;
    if (waitTime == 6000) {      // если не приходит init раз в минуту проверять работу сети
      resetFunc(); 
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
  pinMode(YELLOWLEDPIN, OUTPUT);      // желтый
  pinMode(REDLEDPIN, OUTPUT);         // красный
  digitalWrite(YELLOWLEDPIN, LOW);
  digitalWrite(REDLEDPIN, HIGH);

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
  byte data[12];
  byte addr[8];
  float celsius = 0;

  while (1) {

    if ( !ds.search(addr)) {
      ds.reset_search();
      delay(250);
      break;
    }
    
    if (OneWire::crc8(addr, 7) != addr[7]) {
      break;
    }

    ds.reset();
    ds.select(addr);
    ds.write(0x44, 1);        // Говорим датчику провести замер и перевести в цифровой вид, исп-ся паразитное питание
    delay(1000);              // Ждем пока датчик сообразит, по даташиту должно хватить 750мс
    ds.reset();               // Посылаем резет
    ds.select(addr);          // выбираем наш датчик
    ds.write(0xBE);           // Просим выслать замерчик

    for ( i = 0; i < 9; i++) {           // Получаем несчастные 9 байт 
      data[i] = ds.read();
    }

    int16_t raw = (data[1] << 8) | data[0]; // В 0 и 1 байтах наша температурка
    celsius += (float)raw / 16.0;           // 0-3 бит - дробная часть
    inc++;
  }

  float h = dht.readHumidity();
  float t = dht.readTemperature();

  if (isnan(t) || isnan(h)) {
    // Если DHT молчит
    for (int i = 1; i < 6; i++) {
      digitalWrite(REDLEDPIN, HIGH);
      delay(150);
      digitalWrite(REDLEDPIN, LOW);
      delay(150);
    }
  } else {
    inc++;
    if (inc == 1) {
      for (int i = 1; i < 6; i++) {
        digitalWrite(YELLOWLEDPIN, LOW);
        delay(150);
        digitalWrite(YELLOWLEDPIN, HIGH);
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
      digitalWrite(YELLOWLEDPIN, LOW);
      delay(350);
      digitalWrite(YELLOWLEDPIN, HIGH);
      delay(350);
    }

  } else {                          // Если не удается установить связь с сервером    
      for (int i = 1; i < 3; i++) {
       digitalWrite(REDLEDPIN, HIGH);
       delay(350);
       digitalWrite(REDLEDPIN, LOW);
       delay(350);
      }
    countErrConnection++;
    
     if (countErrConnection == 4) {
        resetFunc();                  // если 4 раз данные не удастся передать - софт ресет
     }
  }
  delay(300000);                 // Засыпаем на 300 секунд
}


