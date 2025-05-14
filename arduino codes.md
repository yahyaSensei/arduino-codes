
# 1- 7 segment display

```CPP
// Faya-Nugget exercise sketch  (Serial_4digit_7segment_Display.ino)

// Description: (1)Design a 9999 up counter on faya Serial 4-digit 7-segment Display. 
//              (2)When the it counts to maximum value, the counter resets to zero and repeat the counting.
                
// Wiring: Arduino Pin ==> faya Module Pin
//                 D10 ==>  LOAD  (4-digit 7-segment Display)
//                 D11 ==>  CLK 
//                 D12 ==>  DIN 

//--- include library for MAX7219 ---
#include <LedControl.h>

// Create variable with argument, DIN, CLK, LOAD, cascade number
LedControl faya7seg = LedControl(12, 11, 10, 1);

int counter = 0;                 // variable to store counter

void setup() {
  //--- initialize 4-digit 7-segment display ---
  faya7seg.shutdown(0, false);   // set power to normal mode
  faya7seg.setIntensity(0, 8);   // set intensity to 8 ( 0 ~ 15 )
  faya7seg.clearDisplay(0);      // clear display
  show4digits(0);                // out 0000 to display
}

void loop() {

  if (counter == 9999) {        // if reach maximum 9999
    counter = 0;                // reset to zero
  }
  else {
    counter = counter + 1;      // otherwise counter + 1
    show4digits(counter);       // call show4digits subroutine
    delay(200);                 // count 1 every 0.2 second
  }
}

// show for digit subroutine
void show4digits(int number) {
  int num1, num2, num3, num4;

  num1 = number / 1000;                   // get thousands digit
  num2 = (number % 1000) / 100;           // get hundreds digit
  num3 = (number % 100) / 10;             // get tens digit
  num4 = number % 10;                     // get units digit

  faya7seg.setDigit(0, 3, num1, false);   // lit thousands digit
  faya7seg.setDigit(0, 2, num2, false);   // lit hundreds digit
  faya7seg.setDigit(0, 1, num3, false);   // lit tens digit
  faya7seg.setDigit(0, 0, num4, false);   // lit units digit
}

```

# 2 - touch slider

```CPP
// Faya-Nugget exercise sketch  (Touch_Slider.ino)

// Description: Detect the position of the finger on faya Touch Slider and
//              use tone() function to generate corresponding tone, DO-RE-MI-FA-SO-LA-SI-DO’.
                
// Wiring: Arduino Pin ==> faya Module Pin
//                  D5 ==>  P0  (Touch Slider)
//                  D6 ==>  P1 
//                  D7 ==>  P2 
//                  D8 ==>  P3 
//                  D9 ==>  P4
//                 D10 ==>  P5
//                 D11 ==>  P6
//                 D12 ==>  P7
//----------------------------------------------
//                  D3 ==>  BZ  (MTS-300 BUZZER)
//                 GND ==>  GND (MTS-300) 

#include "pitches.h"         // include header file pitches.h
#define bzPin 3              // Buzzer port BZ connects to D3

// variable notes[] array to store frequency of music notes
//Do Re, Mi, Fa, So, La, Si, Do'
int notes[] = {NOTE_C4, NOTE_D4, NOTE_E4, NOTE_F4, NOTE_G4, NOTE_A4, NOTE_B4, NOTE_C5};

void setup() {
  pinMode(bzPin, OUTPUT);    // set BZ pins as OUTPUT

  // set Arduino pins D5 to D12 as input
  for (int MediaPin = 5; MediaPin <= 12; MediaPin++) {
    pinMode(MediaPin, INPUT);
  }
}

void loop() {
  // read the state of D5 to D12 alternatively
  for (int MediaPin = 5; MediaPin <= 12; MediaPin++) {
    int keyReading = digitalRead(MediaPin);
    if (keyReading == HIGH) {                 // when button is pressed
      tone(bzPin, notes[12 - MediaPin], 20);  // play corresponding note
    }
  }
}

```

# 3 - serial 8x8 matrix display

```CPP
// Faya-Nugget exercise sketch (Serial_8x8_Matrix_Diasplay.ino)

// Description: Design a heart pattern and show it on faya Serial 8x8 Matrix Display.
                
// Wiring: Arduino Pin ==> faya Module Pin
//                 D10 ==>  LOAD  (Serial 8x8 Matrix Display)
//                 D11 ==>  CLK 
//                 D12 ==>  DIN

//--- include library for MAX7219 ---
#include <LedControl.h>

// Create variable with argument, DIN, CLK, LOAD, cascade number
LedControl fayaSerial8x8Dot = LedControl(12, 11, 10, 1);

//--- define heart pattern ---
boolean heart[8][8] = 
{
  {0, 0, 0, 1, 1, 1, 0, 0},  // Top curve of the banana shape
  {0, 0, 1, 1, 1, 1, 1, 0},  // Banana outline with more curvature
  {0, 1, 1, 1, 1, 1, 0, 0},  // Banana outline (more curved)
  {1, 1, 1, 1, 1, 0, 0, 0},  // Bottom curve
  {1, 1, 1, 1, 0, 0, 0, 0},  // Bottom curve continues
  {1, 1, 1, 0, 0, 0, 0, 0},  // Slightly flattening
  {1, 1, 0, 0, 0, 0, 0, 0},  // Flattening more
  {0, 1, 0, 0, 0, 0, 0, 0}   // Final curvature of the banana shape
};


void setup() {
  fayaSerial8x8Dot.shutdown(0, false);   // set power mode to normal
  fayaSerial8x8Dot.setIntensity(0, 10);  // set intensity (0 – 15)
  fayaSerial8x8Dot.clearDisplay(0);      // clear display
  show_heart();
}

void loop() { }

//--- subroutine to print a heart pattern ---
void show_heart() {
  for (int i = 0; i <= 7; i++) {                        // looping column
    for (int j = 0; j <= 7; j++) {                      // looping row
      fayaSerial8x8Dot.setLed(0, j, i, heart[i][j]);    // light up each LED
    }
  }
}///*/

```

# 4 - UV sensor
```CPP
// Faya-Nugget exercise sketch (UV_Sensor.ino)

// Description: Convert the output of faya UV Sensor to UV intensity and 
//              show the result at Arduino Serial Monitor.
                
// Wiring: Arduino Pin ==> faya Module Pin
//                  A0 ==>    Vout    (UV Sensor)
//                  A1 ==>  Vref 3.3V

//--- define Arduino pin connected to UV sensor pins ---
#define voutPin A0        // UV Sensor port Vout connects to A0
#define vref3V3Pin A1     // UV Sensor port Vref 3.3V connects to A1

void setup() {
  Serial.begin(9600);     // initialize baud rate = 9600bps
}

void loop() {
  int  val1 = analogRead(voutPin);                          // get analog value of Vout
  int  val2 = analogRead(vref3V3Pin);                       // get analog value of Vref3.3V
  float vout = (3.33 / val2) * val1;                        // calculate voltage Vout
  float uvIntensity = (mapFloat(vout, 0.99, 2.8, 0, 15));   // get UV intensity

  // show result at Arduino Serial Monitor
  Serial.println("===== fayaLab UV Sensor ====="); // print string return
  Serial.print("vout     (analog value) = ");      // print string prompt
  Serial.println(val1);                            // print analog value of Vout
  Serial.print("vref3.3V (analog value) = ");      // print another string prompt
  Serial.println(val2);                            // print analog value of Vref
  Serial.print("uv Voltage   = ");                 // print third string prompt
  Serial.print(vout);                              // print output voltage
  Serial.println("  (V)");                         // print unit for Vout
  Serial.print("uv Intensity = ");                 // pint last string prompt
  Serial.print(uvIntensity);                       // print UV intensity
  Serial.println(" (mW/cm2)\r\n");                 // print unit for UV
  delay(1500);                                     // delay 1.5 second before next routine
}

//--- subroutine for mapFloat() ---
float mapFloat(float value, float fromLow, float fromHigh, float toLow, float toHigh) {
  return (value - fromLow) * (toHigh - toLow) / (fromHigh - fromLow) + toLow;
}

```

# 5 - two axis servo 

```CPP
// Faya-Nugget exercise sketch  (TwoAxis_Servo.ino)

// Description: Use faya Joystick to control the motion of faya 2-axis Servo.

// Wiring: Arduino Pin ==> faya Module Pin
//                  D5 ==>  YM1  (2-axis Servo)
//                  D6 ==>  XM2
//------------------------------------------------
//                  A5 ==>  VRX  (JoyStick Switch)
//                  A4 ==>  VRY

//--- include library and define Arduino pins---
#include <Servo.h>
#define ym1Pin 5      // Servo pin YM1 connects to Arduino D5
#define xm2Pin 6      // Servo pin XM2 connects to Arduino D6
#define vrxPin A5     // Joystick pin VRX connects to A5
#define vryPin A4     // Joystick pin VRY connects to A4

// create servo instance: ServoYM1 (upper servo)
Servo fayaServoym1Pin;
// create servo instance: ServoYM2 (lower servo)
Servo fayaServoxm2Pin;

// variable to store analog value for joystick
int val_vrxPin, val_vryPin;

void setup() {
  fayaServoym1Pin.attach(ym1Pin);    // attach servo YM1 to D5
  fayaServoxm2Pin.attach(xm2Pin);    // attach servo XM2 to D6

  pinMode(vrxPin,INPUT);             // Set A5 as input to read VRX
  pinMode(vryPin,INPUT);             // Set A4 as input to read VRY
}

void loop() {
  val_vrxPin = analogRead(vrxPin);   // read VRX value (0~1023)
  // map VRX value from 0~1023 to 0~180
  val_vrxPin = map(val_vrxPin, 0, 1023, 0, 180);
  fayaServoym1Pin.write(val_vrxPin); // drive the servo to mapped angle

  val_vryPin = analogRead(vryPin);   // read VRX value (0~1023)
  // map VRY value from 0~1023 to 0~180
  val_vryPin = map(val_vryPin, 0, 1023, 0, 180);
  fayaServoxm2Pin.write(val_vryPin); // drive the servo to mapped angle

  delay(100);                        // delay 0.1 second before next control
}

```

# 6 - home security alarm with LDR

```CPP
// Pin Definitions
int LDRPin = A0;    // LDR sensor connected to A0
int buzzerPin = 8;  // Buzzer connected to pin 8
int ledPin = 13;    // LED connected to pin 13

// Threshold for light intensity (adjust as needed)
int threshold = 500;

void setup() {
  pinMode(buzzerPin, OUTPUT);
  pinMode(ledPin, OUTPUT);
  Serial.begin(9600); // Begin serial communication (optional for debugging)
}

void loop() {
  // Read the LDR value
  int LDRValue = analogRead(LDRPin);

  // Print the LDR value to the Serial Monitor (optional)
  Serial.println(LDRValue);

  // Check if the light intensity falls below the threshold
  if (LDRValue < threshold) {
    digitalWrite(buzzerPin, HIGH); // Activate buzzer
    digitalWrite(ledPin, HIGH);    // Turn on LED
  } else {
    digitalWrite(buzzerPin, LOW);  // Deactivate buzzer
    digitalWrite(ledPin, LOW);     // Turn off LED
  }

  delay(100); // Short delay before checking again
}
```

# 7 - motion sensor with PIR

```CPP
// Define the pins
int pirPin = 7;     // Pin connected to the PIR sensor OUT
int buzzerPin = 8;  // Pin connected to the buzzer
int ledPin = 13;    // Pin connected to the LED

void setup() {
  // Initialize pins
  pinMode(pirPin, INPUT);     // PIR sensor as input
  pinMode(buzzerPin, OUTPUT); // Buzzer as output
  pinMode(ledPin, OUTPUT);    // LED as output

  // Start serial communication (optional for debugging)
  Serial.begin(9600);
}

void loop() {
  // Read PIR sensor value
  int sensorValue = digitalRead(pirPin);

  // If motion is detected (sensorValue is HIGH)
  if (sensorValue == HIGH) {
    digitalWrite(buzzerPin, HIGH); // Turn on the buzzer
    digitalWrite(ledPin, HIGH);    // Turn on the LED
    Serial.println("Motion detected!"); // Optional: For debugging purposes
  } else {
    digitalWrite(buzzerPin, LOW);  // Turn off the buzzer
    digitalWrite(ledPin, LOW);     // Turn off the LED
  }
  delay(100); // Delay to avoid rapid triggering
}
```

# 8 - digital DC motor speed control

```CPP
// Faya-Nugget exercise sketch (DC_Motor.ino)
//--- define Arduino pins connected to DC Motor ---
#define sigPin 11 // DC Motor port SIG connects to D11
#define dirPin 12 // DC Motor port DIR connects to D12

void setup() {
  pinMode(dirPin, OUTPUT); // set port DIR as output
  pinMode(sigPin, OUTPUT); // set port SIG as output
}

void loop() {
  digitalWrite(dirPin, HIGH); // set motor direction to CCW
  analogWrite(sigPin, 100);  // start rotates at 100 speed
  delay(10000);              // rotate 10 second
  analogWrite(sigPin, 0);    // stop rotate
  delay(3000);               // stop for 3 second
  digitalWrite(dirPin, LOW);  // set motor direction to CW
  analogWrite(sigPin, 100);  // start rotates at 100 speed
  delay(10000);              // rotates for 10 second
  analogWrite(sigPin, 0);    // stop
  delay(3000);               // delay
}
```