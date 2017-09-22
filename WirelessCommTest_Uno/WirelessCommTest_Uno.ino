/*
* First RF sketch using nRF24L01+ radios and Arduino (Uno or Micro)
*/
#include <SPI.h>    // Serial Peripheral Interface bus library
#include "RF24.h"   // Library for nRF24L01s

/****************** User Config ***************************/
/***      Set this radio as radio number 0 or 1         ***/
bool radioNumber = 0;

/* Hardware configuration: Set up nRF24L01 radio on SPI bus pins and pins 7 & 8 */
RF24 radio(7,8);  // For Arduino UNO
//RF24 radio(9,10);  // For Arduino Micro

/* Create a binary address for read/write pipes */
byte addresses[][6] = {"1Node","2Node"};

bool role = 0; // Local state variable that controls whether this node is sending or receiving
void setup() {
  Serial.begin(115200);
  radio.begin();

  // Set the Power Level low
  radio.setPALevel(RF24_PA_LOW);
  
  // Set radio rate and channel
  radio.setDataRate(RF24_250KBPS); 

  // Set the channel (Frequency): 108 = 2.508 GHz (above WiFi)
  radio.setChannel(108);

  // Open a writing and reading pipe on each radio, with opposite addresses
  if(radioNumber){
    radio.openWritingPipe(addresses[1]);
    radio.openReadingPipe(1,addresses[0]);
  }else{
    radio.openWritingPipe(addresses[0]);
    radio.openReadingPipe(1,addresses[1]);
  }
  
  // Pause briefly to let everything "settle"
  delay(1000);
  Serial.println(F("RF24/examples/GettingStarted"));
  Serial.println(F("*** PRESS 'T' to begin transmitting to the other node"));

  // Start the radio listening for data
  radio.startListening();
}
int counter=0;
unsigned long start_time=  0;
void loop() {

  /****************** Transmitting Role ***************************/  
  if (role == 1)
  {
    radio.stopListening();                                    // First, stop listening so we can talk.
    Serial.println(F("Now sending"));
    counter=counter+1;
//    if((counter % 3)==0){
//      start_time = 0; // Take the time, and send it.  This will block until complete
//    }else{
//      start_time = 1;
//    }
    if (!radio.write( &start_time, sizeof(unsigned long) )){
      Serial.println(F("failed"));
    }
    radio.startListening();                                    // Now, continue listening
    unsigned long started_waiting_at = micros();               // Set up a timeout period, get the current microseconds
    boolean timeout = false;                                   // Set up a variable to indicate if a response was received or not
    
    while ( ! radio.available() ) {                            // While nothing is received
      if (micros() - started_waiting_at > 200000 ) {            // If waited longer than 200ms, indicate timeout and exit while loop
          timeout = true;
          break;
      }      
    }
        
    if ( timeout ) {                                             // Describe the results
        Serial.println(F("Failed, response timed out."));
    } else {
        unsigned long got_time;                                 // Grab the response, compare, and send to debugging spew
        radio.read( &got_time, sizeof(unsigned long) );
        unsigned long end_time = micros();
        
        // Send to serial port
        Serial.print(F("Sent "));
        Serial.print(start_time);
        Serial.print(F(", Got response "));
        Serial.print(got_time);
        Serial.print(F(", Round-trip delay "));
        Serial.print(end_time-start_time);
        Serial.println(F(" microseconds"));
    }

    // Try again 100 ms later
    delay(100);
  }
  
  /****************** Receiving Role ***************************/  
  if ( role == 0 )
  {
    unsigned long got_time;    
    if( radio.available()) {                                        // Variable for the received timestamp
      while (radio.available()) {                                   // While there is data ready
        radio.read( &got_time, sizeof(unsigned long) ); 
      }
      radio.stopListening();  
      radio.write( &got_time, sizeof(unsigned long) );              // Send the final one back.      
      radio.startListening();                                       // Now, resume listening so we catch the next packets.     
      Serial.print(F("Sent response "));
      Serial.println(got_time);  
    }
 }
 
/****************** Change Roles via Serial Commands ***************************/
  if ( Serial.available() )
  {
    char c = toupper(Serial.read());
    if ( c == 'T' && role == 0 ) {      
      Serial.println(F("*** CHANGING TO TRANSMIT ROLE -- PRESS 'R' TO SWITCH BACK"));
      role = 1;                  // Become the primary transmitter (ping out)
    } else if ( c == 'R' && role == 1 ) {
      Serial.println(F("*** CHANGING TO RECEIVE ROLE -- PRESS 'T' TO SWITCH BACK"));      
       role = 0;                // Become the primary receiver (pong back)
       radio.startListening();
       } else if ( c == 'G' && start_time == 1 && role == 1 ) {
      Serial.println(F("*** CHANGING TO GREEN LED -- PRESS 'L' TO SWITCH BACK"));      
       start_time = 0;                // Become the primary receiver (pong back)
       radio.startListening();
    } else if ( c == 'L' && start_time == 0 && role == 1 ) {
      Serial.println(F("*** CHANGING TO RED LED -- PRESS 'G' TO SWITCH BACK"));      
       start_time = 1;                // Become the primary receiver (pong back)
       radio.startListening();
     } else if ( c == 'X' &&  role == 1 ) {
      Serial.println(F("*** STOP MOTORS"));      
       start_time = 2525;                // Become the primary receiver (pong back)
       radio.startListening();
    } else if ( c == 'A' && role == 1 ) {
      Serial.println(F("*** TURN LEFT"));      
       start_time = 1010;                // Become the primary receiver (pong back)
       radio.startListening();
    } else if ( c == 'D' &&  role == 1 ) {
      Serial.println(F("*** TURN RIGHT"));      
       start_time = 4040;                // Become the primary receiver (pong back)
       radio.startListening();
    } else if ( c == 'W' &&  role == 1 ) {
      Serial.println(F("*** Forwards"));      
       start_time = 1040;                // Become the primary receiver (pong back)
       radio.startListening();
     } else if ( c == 'S' &&  role == 1 ) {
      Serial.println(F("*** Backwards"));      
       start_time = 4010;                // Become the primary receiver (pong back)
       radio.startListening();
    }
  }

} // End of Loop

// FIN
