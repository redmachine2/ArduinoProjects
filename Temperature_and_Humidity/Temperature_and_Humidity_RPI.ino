#include <JeeLib.h> //Low Power functions library
#include <SPI.h>
#include "RF24.h"

// User Config for wireless module
bool radioNumber = 1;
RF24 radio(9,10);
// Radio pipe addresses for the 2 nodes to communicate.
const uint64_t pipes[2] = { 0xF0F0F0F0E1LL, 0xF0F0F0F0D2LL };
byte addresses[][6] = {"1Node","2Node"};


//
// Payload
//

const int min_payload_size = 4;
const int max_payload_size = 32;
const int payload_size_increments_by = 1;
int next_payload_size = min_payload_size;

char receive_payload[max_payload_size+1]; // +1 to allow room for a terminating NULL char

ISR(WDT_vect) { Sleepy::watchdogEvent(); } //Setup the watchdog
//Setup temperature sensor
int DHpin = 8;
byte dat [5];
byte read_data () {
  byte data;
  for (int i = 0; i < 8; i ++) {
    if (digitalRead (DHpin) == LOW) {
      while (digitalRead (DHpin) == LOW); // wait for 50us
      delayMicroseconds (30); // determine the duration of the high level to determine the data is '0 'or '1'
      if (digitalRead (DHpin) == HIGH)
        data |= (1 << (7-i)); // high front and low in the post
      while (digitalRead (DHpin) == HIGH); // data '1 ', wait for the next one receiver
     }
  }
return data;
}
 
void start_test () {
  digitalWrite (DHpin, LOW); // bus down, send start signal
  delay (30); // delay greater than 18ms, so DHT11 start signal can be detected
 
  digitalWrite (DHpin, HIGH);
  delayMicroseconds (40); // Wait for DHT11 response
 
  pinMode (DHpin, INPUT);
  while (digitalRead (DHpin) == HIGH);
  delayMicroseconds (80); // DHT11 response, pulled the bus 80us
  if (digitalRead (DHpin) == LOW);
  delayMicroseconds (80); // DHT11 80us after the bus pulled to start sending data
 
  for (int i = 0; i < 4; i ++) // receive temperature and humidity data, the parity bit is not considered
    dat[i] = read_data ();
 
  pinMode (DHpin, OUTPUT);
  digitalWrite (DHpin, HIGH); // send data once after releasing the bus, wait for the host to open the next Start signal
}

int cToF (int c) {
  int f = 1.8 * c + 32;
  return f;
}
 
void setup () {
  Serial.begin (9600);
  pinMode (DHpin, OUTPUT);

  radio.begin();
  radio.enableDynamicPayloads();

  // Set the PA Level low to prevent power supply related issues since this is a
  // getting_started sketch, and the likelihood of close proximity of the devices. RF24_PA_MAX is default.
  //  radio.setPALevel(RF24_PA_LOW);

//  radio.openWritingPipe(addresses[1]);
//  radio.openReadingPipe(1, addresses[0]);
  radio.openWritingPipe(pipes[0]);
  radio.openReadingPipe(1,pipes[1]);

//  radio.startListening();
}
 
void loop () {
  digitalWrite(13, LOW);
  start_test ();

radio.stopListening();
  Serial.println(F("Now sending"));
  Serial.println(dat[2], DEC);
//  if(!radio.write(&dat, sizeof(byte)*5)){
//    Serial.println(F("failed"));
//  }
//  radio.write(&dat, sizeof(byte)*5);
  radio.write(dat, next_payload_size);

  radio.startListening();
  unsigned long started_waiting_at = millis();
  bool timeout = false;

  while(!radio.available() && !timeout){
    if(millis() - started_waiting_at > 1000){
      timeout = true;
      break;
    }
  }

  if(timeout){
    Serial.println(F("Failed, response time out."));
  }else{
    uint8_t len = radio.getDynamicPayloadSize();
    if(!len){
      return;
    }
    radio.read(receive_payload, len);
    receive_payload[len] = 0;
    Serial.print(F("Got response "));
    Serial.println(receive_payload);
  }

  radio.stopListening();
  delay(500);
  Sleepy::loseSomeTime(1000);
}
