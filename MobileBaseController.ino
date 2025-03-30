#include <Arduino.h>

void setup() {
    Serial.begin(9600);  // Start serial communication
}

void loop() {
    int accelerator = analogRead(A0);
    int decelerator = analogRead(A4);
    int x = analogRead(A3);
    int y = analogRead(A2);

    bool directionSent = false; // Flag to check if any direction was sent

    if (accelerator > 20) {
        Serial.println("q"); // Send 'q' over serial
        delay(10);
    }
    if (decelerator > 620) {
        Serial.println("z"); // Send 'z' over serial
        delay(10);
    }

    // Check x-axis conditions
    if (x > 401) {
        Serial.println("i"); // Forward
        directionSent = true;
        delay(250);
    }
    if (x < 300) {
        Serial.println(","); // Backward
        directionSent = true;
        delay(250);
    }

    // Check y-axis conditions
    if (y > 390) {
        Serial.println("j"); // Left
        directionSent = true;
        delay(250);
    }
    if (y < 295) {
        Serial.println("l"); // Right
        directionSent = true;
        delay(250);
    }

    // If no direction was sent, send 'k'
    if (!directionSent) {
        Serial.println("k");
        delay(250); // Small delay to prevent spamming
    }
}