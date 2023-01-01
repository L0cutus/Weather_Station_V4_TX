// ARDUINO ATMel328P TX script to acquire and send data
// via MQTT to an ESP32 server using NRF24L01+
// it acquire:
// TEMPERATURE
// HUMIDITY
// PRESSURE
// via a BME280 sensor
//
// WIND ANEMOMETER
// WIND DIRECTION
// RAIN
// via external sensors
//
// BATTERY voltage


#include <avr/wdt.h>
#include <avr/sleep.h>
#include <SPI.h>
#include <RF24.h>
#include <RF24Network.h>
#include "printf.h"
#include <Adafruit_BME280.h>
#include "definitions.h"

// instantiate an object for the nRF24L01 transceiver
// SPI PINS:
// SCK      = 13
// MISO     = 12
// MOSI     = 11
// SS/CSN   = 10
// CE       = 8
// ---------------
RF24 radio(8, 10);  // using pin 8 for the CE pin, and pin 10 for the CSN pin
RF24Network network(radio);  // Network uses that radio

// Let these addresses be used for the pair
const uint8_t address[6] = "00001";
// It is very helpful to think of an address as a path instead of as
// an identifying device destination

Adafruit_BME280 bme; // I2C

// structure to send to server with statistics
pld_data data_pld; // 20 bytes
pld_wind wind_pld; // 12 bytes

// LED -> pin 15 = D9 (arduino) = PB1 (chip)
volatile boolean ledTimer2Acceso = false;

// ogni byte conterrà il numero di giri medi
// compiuti negli ultimi 3 secondi dall'anemometro
// (vento istantaneo)
volatile byte windInst[3] = {0}; // latest 3 seconds wind speed
volatile byte idxWindInst = 0; // index for windInst array

// Indice dell'array dei secondi
volatile unsigned int idxSeconds = 0;

volatile boolean sendData = false;

void setup () {
  // Switch OFF Watchdog
  WDTCSR = 0;

  Serial.begin(115200);
  delay(200);
  
  // Initialize BME280 sensor
  if (!bme.begin(0x76, &Wire)) 
  {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
//    while (1) {}  // hold in infinite loop
  }

  delay(50);

  bme.setSampling(Adafruit_BME280::MODE_FORCED,
              Adafruit_BME280::SAMPLING_X1, // temperature
              Adafruit_BME280::SAMPLING_X1, // pressure
              Adafruit_BME280::SAMPLING_X1, // humidity
              Adafruit_BME280::FILTER_OFF   );
  bme.setTemperatureCompensation(0.70);


  // initialize the transceiver on the SPI bus
  if (!radio.begin()) {
    Serial.println(F("radio hardware is not responding!!"));
//    while (1) {}  // hold in infinite loop
  }

  radio.setChannel(90);
  network.begin(/*node address*/ this_node);

  // Set the PA Level low to try preventing power supply related problems
  // because these examples are likely run with nodes in close proximity to
  // each other.
  radio.setPALevel(RF24_PA_LOW);  // RF24_PA_MAX is default.

  radio.setDataRate(RF24_250KBPS);
  
  // optionally, increase the delay between retries & # of retries
  radio.setRetries(100, 15);
  //radio.enableDynamicPayloads();

  // save on transmission time by setting the radio to only transmit the
  // number of bytes we need to transmit a float
  //radio.setPayloadSize(sizeof(payload));  // float datatype occupies 4 bytes

  // set the TX address of the RX node into the TX pipe
  //radio.openWritingPipe(address);  // always uses pipe 0

  //radio.stopListening();  // put radio in TX mode

  // For debugging info
  // printf_begin();             // needed only once for printing details
  // radio.printDetails();       // (smaller) function that prints raw register values
  // radio.printPrettyDetails(); // (larger) function that prints human readable data

  portInit();
  rtcInit();

  EIFR |= (1<<INTF0);  // clear flag for interrupt 0
  EIFR |= (1<<INTF1);  // clear flag for interrupt 1

  // Seleziona SLEEP_MODE_PWR_SAVE
  SMCR|=(1<<SM1)|(1<<SM0);

  // INT0 called on FALLING signal
  EICRA = (1<<ISC00); 
  EICRA &= ~(1<<ISC01);
  // INT1 called on FALLING signal
  EICRA = (1<<ISC10); 
  EICRA &= ~(1<<ISC11);

  // Abilita INT0 ed INT1
  EIMSK |= (1<<INT0) | (1<<INT1);
  
}  // end of setup

void loop ()
{
  if(ledTimer2Acceso)
  {
    PORTD |= (1<<PORTD7); // pin D7 arduino HIGH
    delay(4);
    PORTD &= ~(1<<PORTD7); // pin D7 arduino LOW
    delay(4);
    PORTD |= (1<<PORTD7); // pin D7 arduino HIGH
    delay(4);
    PORTD &= ~(1<<PORTD7); // pin D7 arduino LOW
    ledTimer2Acceso = false;
    Serial.print("idxSeconds: ");Serial.println(idxSeconds);  
  }
  
  network.update();  // Check the network regularly
  
  if(sendData)
  {
    bme.takeForcedMeasurement(); // needed for Forced Mode

    data_pld.temperature = bme.readTemperature();
    data_pld.humidity = bme.readHumidity();
    data_pld.pressure = bme.readPressure() / 100.0F;

    // legge la tensione della batteria
    data_pld.batteryV = analogRead(BATTERYV);  // battery voltage 0 - 1023

    // direzione del vento
    readWindDir();

    // median wind , gust wind, latest instantaneous wind
    wind_pld.gustWind = 0.0;
    wind_pld.medianWind = 0.0;
    float gust = 0.0;
    for (int i = 0; i < MAX_BUFFER; i++)
    {
      gust = (float) windData[i] / 4.0 * 2.4;

      // Serial.println(gust);
      // char vBuffer[15];
      // dtostrf(gust,4,2, vBuffer);      
      // sprintf(srlBuf,"windData: %d - Gust %d: %s", windData[i], i, vBuffer);
      // Serial.println(srlBuf);

      if (gust > 18.0 && gust > wind_pld.gustWind)  // se la velocità è maggiore di 18km/h è una raffica
        wind_pld.gustWind = gust;

      wind_pld.medianWind += windData[i];
    }    
    wind_pld.medianWind = wind_pld.medianWind / 4.0 * 2.4 / 200.0;  // km/h
    wind_pld.instWind = windData[MAX_BUFFER-1];

    Serial.print("Size data: ");Serial.println(sizeof(data_pld));
    Serial.print("Size data: ");Serial.println(sizeof(wind_pld));

    radio.powerUp();
    
    RF24NetworkHeader header(/*to node*/ other_node);
    bool report = network.write(header, &data_pld, sizeof(data_pld));

    //delay(100);

    RF24NetworkHeader header1(/*to node*/ other_node);
    bool report1 = network.write(header1, &wind_pld, sizeof(wind_pld));

    radio.powerDown();

    if (report && report1) 
    {
      Serial.println(F("Transmission successful! "));  // payload was delivered
      Serial.print("BatteryV: "); Serial.println((3.3/1024.0)*data_pld.batteryV);
      Serial.print("Temperature: "); Serial.println(data_pld.temperature);
      Serial.print("Humidity: "); Serial.println(data_pld.humidity); 
      Serial.print("Pressure: "); Serial.println(data_pld.pressure); 
      Serial.print("Rain: "); Serial.println(data_pld.rain);
    } else 
    {
      Serial.println(F("Transmission failed or timed out"));  // payload was not delivered
    }

    data_pld.rain = 0.0F;
    sendData = false;
  }

  Serial.flush();
  
  // Do not interrupt before we go to sleep, or the
  // ISR will detach interrupts and we won't wake.
  cli ();

  // disable ADC
  byte oldADCSRA = ADCSRA;
  ADCSRA &= ~(1<<ADEN);  
  
  // Sleep enabled
  SMCR |= (1<<SE);

  // We are guaranteed that the sleep_cpu call will be done
  // as the processor executes the next instruction after
  // interrupts are turned on.
  sei ();  // one cycle
  sleep_cpu ();   // one cycle

  // Sleep disabled
  SMCR &= ~(1<<SE);
  ADCSRA = oldADCSRA;  
} // end of loop  



