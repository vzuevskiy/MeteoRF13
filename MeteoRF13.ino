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
String getMeasurements() {
  String toServer;
  toServer = "METEO,N0001,";
  float h = dht.readHumidity();
  float t = dht.readTemperature();
 
  /********************************************/
  //
  //    Получаем данные от DHT22
  //    Полученные данные записываем в toString
  //
  /********************************************/

  if (isnan(t) || isnan(h)) {
    // Если DHT молчит
    for (int i = 1; i < 6; i++) {
      digitalWrite(REDLEDPIN, HIGH);
      delay(150);
      digitalWrite(REDLEDPIN, LOW);
      delay(150);
    }
  } else {
    toServer = toServer + "S256,"  + "T" + t + ",H" + h;  // 256 - DHT
    for (int i = 1; i < 6; i++) {
      digitalWrite(YELLOWLEDPIN, LOW);
      delay(150);
      digitalWrite(YELLOWLEDPIN, HIGH);
      delay(150);
    }
  }

  /********************************************/
  //
  //    Получаем данные от DS18b20
  //    Полученные данные записываем в toString,
  //    Пока не кончатся датчики
  //
  /********************************************/

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

    for (int i = 0; i < 9; i++) {           // Получаем несчастные 9 байт
      data[i] = ds.read();
    }

    int16_t raw = (data[1] << 8) | data[0]; // В 0 и 1 байтах наша температурка
    celsius = (float)raw / 16.0;           // 0-3 бит - дробная часть

    toServer = toServer + ",S" + addr[2] + ",T" + celsius; // Серийник расположен с 8 по 47 бит, берем кусочек, он наверняка будет уникальным
  }
  return toServer;
}

void loop() {
  /********************************************/
  //
  //    Отправляем на сервер строку toString
  //    сформированную в процессе выполнения
  //    предыдущих операций;
  //    Делаем 5 попыток, если не получается
  //    соединиться с сервером, то делаем софт
  //    резет.
  //
  /********************************************/

  countErrConnection = 0;
  while (1) {
    if (client.connect(server, 7364)) {
      client.print(getMeasurements());
      client.stop();

      for (int i = 1; i < 3; i++) {
        digitalWrite(YELLOWLEDPIN, LOW);
        delay(350);
        digitalWrite(YELLOWLEDPIN, HIGH);
        delay(350);
      }
      break;
    } else {
      // Если не удается установить связь с сервером
      for (int i = 1; i < 3; i++) {
        digitalWrite(REDLEDPIN, HIGH);
        delay(350);
        digitalWrite(REDLEDPIN, LOW);
        delay(350);
      }
      countErrConnection++;
      if (countErrConnection == 5) {
        resetFunc();                  // если 5 раз данные не удастся передать - софт ресет и ждем пока сервер очнется
      }
    }
  }
  delay(300000);                 // Засыпаем на 300 секунд
}
