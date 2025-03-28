/**
 * ESP32S3 Face Recognition
 * (not detection!)
 *
 * Enroll a couple of faces 2-3 times each,
 * then watch the ESP32 camera recognize between the two!
 *
 * Only works on ESP32 S3 chip.
 */

 // https://github.com/eloquentarduino/EloquentEsp32cam/tree/main
#include <eloquent_esp32cam.h>
#include <eloquent_esp32cam/face/detection.h>
#include <eloquent_esp32cam/face/recognition.h>

#include <I2S.h>
#include "audio.h"


using eloq::camera;
using eloq::face::detection;
using eloq::face::recognition;


String prompt(String message);



/**
 *
 */
void setup() {
    delay(2000);
    Serial.begin(115200);
    Serial.println("Begin");
    
    pinMode(D0, INPUT);

    I2S.setAllPins(-1, 42, 41, -1, -1);
    if (!I2S.begin(PDM_MONO_MODE, SAMPLE_RATE, SAMPLE_BITS)) {
      Serial.println("Failed to initialize I2S!");
      while (1) ;
    }
    if(!SD.begin(21)){
      Serial.println("Failed to mount SD Card!");
      while (1) ;
    }

    // !!!!REPLACE WITH YOUR OWN MODEL!!!!
    camera.pinout.xiao(); // e.g. xiao(), lilygo_tcamera_s3(), ...
    camera.brownout.disable();
    // face recognition only works at 240x240
    camera.resolution.face();
    camera.quality.high();

    // face recognition only works with accurate detection
    detection.accurate();
    detection.confidence(0.6);

    // face recognition confidence
    recognition.confidence(0.90);

    // init camera
    while (!camera.begin().isOk())
        Serial.println(camera.exception.toString());
    // init recognizer
    while (!recognition.begin().isOk())
        Serial.println(recognition.exception.toString());

    Serial.println("Camera OK");
    Serial.println("Face recognizer OK");

    // delete stored data, if user confirms
    if (prompt("Do you want to delete all existing faces? [yes|no]").startsWith("y")) {
        Serial.println("Deleting all existing faces...");
        recognition.deleteAll();
    }

    // dump stored faces, if user confirms
    if (prompt("Do you want to dump existing faces? [yes|no]").startsWith("y")) {
        recognition.dump();
    }

    Serial.println("Awaiting for face...");
}


/**
 *
 */
void loop() {
    // capture picture
    if (!camera.capture().isOk()) {
        Serial.println(camera.exception.toString());
        return;
    }

    // run face detection (not recognition!)
    if (!recognition.detect().isOk())
        return;

    // if face is found, ask user to enroll or recognize
    int reading = digitalRead(D0);
    Serial.println(reading);

    if (reading == 0) {
        record_wav();
        enroll();
    }
    else
        recognize();

    Serial.println("Awaiting for face...");
}


/**
 * Ask user for input
 */
String prompt(String message) {
    String answer;

    do {
        Serial.print(message);

        while (!Serial.available())
            delay(1);

        answer = Serial.readStringUntil('\n');
        answer.trim();
    } while (!answer);

    Serial.print(" ");
    Serial.println(answer);
    return answer;
}


/**
 * Enroll new person
 */
void enroll() {
    String name = prompt("Enter person name:");

    if (recognition.enroll(name).isOk())
        Serial.println("Success!");
    else
        Serial.println(recognition.exception.toString());
}


/**
 * Recognize current face
 */
void recognize() {
    if (!recognition.recognize().isOk()) {
        Serial.println(recognition.exception.toString());
        return;
    }

    Serial.print("Recognized face as ");
    Serial.print(recognition.match.name.c_str());
    Serial.print(" with confidence ");
    Serial.print(recognition.match.similarity);
    Serial.print(" (");
    Serial.print(recognition.benchmark.millis());
    Serial.println("ms)");
}