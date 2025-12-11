//This code was written to be easy to understand.
									 
//Modify this code as you see fit.
//This code will output data to the Arduino serial monitor.
//Type commands into the Arduino serial monitor to control the EZO-PMP Embedded Dosing Pump.
//This code was written in the Arduino 2.3.2 IDE
//An Arduino UNO was used to test this code.
//This code was last tested 9/24

#include <SoftwareSerial.h>                           //we have to include the SoftwareSerial library, or else we can't use it
#define rx 32                                          //define what pin rx is going to be
#define tx 33                                          //define what pin tx is going to be
#define Enable5VPIN 27 //turns the 3V to 5V converter on if switched to high
#define MOSFET_SENSORS_PIN 14 //pin to control the power of the sensors, the screen and sd card reader


String inputstring = "";                              //a string to hold incoming data from the PC
String devicestring = "";                             //a string to hold the data from the Atlas Scientific product
boolean device_string_complete = false;               //have we received all the data from the Atlas Scientific product
float ml;                                             //used to hold a floating point number that is the volume 



void setup() {                                        //set up the hardware
  Serial.begin(9600);
  pinMode(MOSFET_SENSORS_PIN, OUTPUT);
  digitalWrite(MOSFET_SENSORS_PIN, HIGH);                                //set baud rate for the hardware serial port_0 to 9600
  Serial2.begin(9600, SERIAL_8N1, rx, tx);                                //set baud rate for the software serial port to 9600
  inputstring.reserve(10);                            //set aside some bytes for receiving data from the PC
  devicestring.reserve(30);                           //set aside some bytes for receiving data from the Atlas Scientific product
}


void loop() {                                         //here we go...

  if (Serial.available()) {                //if a string from the PC has been received in its entirety
    inputstring = Serial.readStringUntil(13);           //read the string until we see a <CR>
    Serial2.print(inputstring);                      //send that string to the Atlas Scientific product
    Serial2.print('\r');                             //add a <CR> to the end of the string
    inputstring = "";                                 //clear the string
  }

  if (Serial2.available() > 0) {                     //if we see that the Atlas Scientific product has sent a character
    char inchar = (char)Serial2.read();              //get the char we just received
    devicestring += inchar;                           //add the char to the var called devicestring
    if (inchar == '\r') {                             //if the incoming character is a <CR>
      device_string_complete = true;                  //set the flag
    }
  }

  if (device_string_complete == true) {                           //if a string from the Atlas Scientific product has been received in its entirety
    Serial.println(devicestring);                                 //send that string to the PC's serial monitor
    if (isdigit(devicestring[0]) || devicestring[0]== '-') {      //if the first character in the string is a digit or a "-" sign    
      ml = devicestring.toFloat();                                //convert the string to a floating point number so it can be evaluated by the Arduino   
    }                                                          //in this code we do not use "ml", But if you need to evaluate the ml as a float, this is how it’s done     
    devicestring = "";                                            //clear the string
    device_string_complete = false;                               //reset the flag used to tell if we have received a completed string from the Atlas Scientific product
  }
}
