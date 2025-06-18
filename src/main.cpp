#include <Arduino.h>


struct Payload {
  int16_t ints[7];
  float floats[5];
};

Payload data;

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




void setup() {
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, 16, 17);  // RX2 -> GPIO16, TX2 -> GPIO17
  Serial.println("begin");
}

void loop() {
  const size_t expectedBytes = sizeof(Payload);

  if (Serial2.available() >= expectedBytes) {
    uint8_t buffer[expectedBytes];
    Serial2.readBytes(buffer, expectedBytes);

    memcpy(&data, buffer, expectedBytes);

    // Print received data
    Serial.print("Ints: ");
    for (int i = 0; i < 8; i++) {
      Serial.print(data.ints[i]);
      Serial.print(' ');
    }

    Serial.print("\nFloats: ");
    for (int i = 0; i < 5; i++) {
      Serial.print(data.floats[i], 2);
      Serial.print(' ');
    }
    Serial.println();
  }
}