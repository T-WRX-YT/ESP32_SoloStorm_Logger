#include <Arduino.h>

float feedbackKnockFinal;  // holds the calculated value for current feedback knock
float feedbackMax = 0;  // holds the max negative value seen for feedback knock
float fineKnockFinal;  // holds the calculated value for the current fine knock correction
float fineMax = 0;  // holds the max negative value seen for fine knock
uint16_t fineRpmMin = 9999;  // this is the smallest RPM value seen when there is fine knock correction, starts at 9999
uint16_t fineRpmMax = 0;  // this is the higest RPM value seen when there is fine knock, combine both to see what range you've got fine knock (its good enough)
float boostFinal;  // holds the calculated boost value
int16_t coolantFinal;  // holds the calculated coolant temperature
float damFinal; // holds the calculated DAM value, if this isn't 1... youve got problems
int16_t intakeTempFinal;  // holds the calculated value of the intake air temp
uint16_t rpmFinal;  // holds the calculated value of engine speed aka RPM
uint8_t gearFinal; // holds the assumed gear position, not always accurate
uint8_t speedFinal; // holds obd2 vehicle speed
float afrFinal; // holds calulated AFR
uint8_t throttleFinal; // holds throttle plate angle
uint16_t brakeFinal; // holds brake pressure - maybe?
unsigned long timer;  // will hold the seconds count used for the nbp send
unsigned int logger; // holds the value of the button for turning on nbp logging



String incomingData = "";
uint16_t rpm;
uint8_t speed;
uint8_t throttle;

void processSerial();
void sendFrame();
void printHexDump(uint8_t* data, size_t length);


void setup() {
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, 16, 17);  // RX2 -> GPIO16, TX2 -> GPIO17
  Serial.println("begin");
}

void loop() {
  processSerial();
  Serial.println(rpm);
  Serial.println(speed);
  Serial.println(throttle);
  sendFrame();
  delay(100);
}


void sendFrame() {
  uint8_t frame[5];

  frame[0] = 0x60;
  frame[1] = (rpm >> 8) & 0xFF;  // RPM high byte
  frame[2] = rpm & 0xFF;         // RPM low byte
  frame[3] = speed;
  frame[4] = throttle;

  printHexDump(frame, sizeof(frame));  // print before sending
  Serial.write(frame, sizeof(frame));
}

void printHexDump(uint8_t* data, size_t length) {
  Serial.print("Raw: ");
  for (size_t i = 0; i < length; i++) {
    if (data[i] < 16) Serial.print("0");
    Serial.print(data[i], HEX);
    Serial.print(" ");
  }
  Serial.println( );
}


void processSerial() {
  if (Serial2.available ()) {
    incomingData = Serial2.readStringUntil('\n');
    incomingData.trim();

    //Serial.println(incomingData);

    int firstComma = incomingData.indexOf(',');
    int secondComma = incomingData.indexOf(',', firstComma + 1);

    rpm = incomingData.substring(0, firstComma).toInt();
    speed = incomingData.substring(firstComma + 1, secondComma).toInt();
    throttle = incomingData.substring(secondComma + 1).toInt();
  }
}