// https://github.com/eloquentarduino/EloquentEsp32cam/tree/main
#include <eloquent_esp32cam.h>
#include <eloquent_esp32cam/face/detection.h>
#include <eloquent_esp32cam/face/recognition.h>

#include "audio.h"

using eloq::camera;
using eloq::face::detection;
using eloq::face::recognition;

#define BUTTON    D0

#define TXD0      43 // ORANGE CABLE
#define RXD0      44 // BLUE CABLE

#define FLAG_IN   D3 // YELLOW CABLE
#define FLAG_OUT  D4 // BLUE CABLE

uint8_t face_index = 1;
uint8_t reading = 0;

unsigned long lastDetectionTime = 0;
const unsigned long detectionCooldown = 10000; // 10 seconds


HardwareSerial mySerial(0); // XIAO uses UART 0

void sendPromptAudio(uint8_t code)
{
  mySerial.write('S');
  mySerial.write(code);
}

void sendRecognizedFace()
{
  String name = recognition.match.name.c_str();

  if (name == "empty" || name.length() == 0 || !isDigit(name[0])) {
    Serial.println("Skipping UART send — not a valid face ID.");
    return;
  }

  int num = name.toInt();
  sscanf(recognition.match.name.c_str(), "%d", &num);
  mySerial.write('S');
  mySerial.write(num);
}

void printRecognitionInfo() 
{

  Serial.printf("Match name: %s, similarity: %.2f\n", recognition.match.name.c_str(), recognition.match.similarity);

  Serial.print("Recognized face as ");
  Serial.print(recognition.match.name.c_str());
  Serial.print(" with confidence ");
  Serial.print(recognition.match.similarity);
  Serial.print(" (");
  Serial.print(recognition.benchmark.millis());
  Serial.println("ms)");
}


void releaseSharedPins()
{
  SPI.end();
  digitalWrite(SD_CS, LOW);
  delay(10);
  digitalWrite(FLAG_OUT, LOW);
  Serial.println("Released shared SPI pin.");
}

// CODE TO AVOID REPEATED DECECTION OF SAME FACE

struct FaceCooldown{
  String name;
  unsigned long lastSeen;
};

const int MAX_COOLDOWN_FACES = 20;
FaceCooldown faceCooldowns[MAX_COOLDOWN_FACES];
const unsigned long recognitionCooldown = 10000; // 10 seconds

bool isFaceInCooldown(String name)
{
  unsigned long now = millis();

  for(int i = 0; i < MAX_COOLDOWN_FACES; i++)
  {
    if(faceCooldowns[i].name == name)
    {
      return(now - faceCooldowns[i].lastSeen < recognitionCooldown);
    }
  }

  return false;

}

void updateFaceCooldown(String name)
{
  unsigned long now = millis();

  for(int i = 0; i < MAX_COOLDOWN_FACES; i++)
  {
    if(faceCooldowns[i].name == name)
    {
      faceCooldowns[i].lastSeen = now;
      return;
    }
  }

  // Not found — insert new
  for (int i = 0; i < MAX_COOLDOWN_FACES; i++) {
    if (faceCooldowns[i].name == "") {
      faceCooldowns[i].name = name;
      faceCooldowns[i].lastSeen = now;
      return;
    }
  }

  Serial.println("Face cooldown tracker full!");

}


void setup() {
    delay(2000);
    Serial.begin(115200);
    Serial.println("Begin Facial Recognition Program");
    
    // Set for Xiao ESP32S3
    camera.pinout.xiao(); 
    camera.brownout.disable();
    // face recognition only works at 240x240
    camera.resolution.face();
    camera.quality.high();

    // face recognition only works with accurate detection
    detection.accurate();
    detection.confidence(0.7);

    // face recognition confidence
    recognition.confidence(0.92);

    pinMode(BUTTON, INPUT_PULLDOWN);
    pinMode(LED, OUTPUT);
    pinMode(FLAG_IN, INPUT);
    pinMode(FLAG_OUT, OUTPUT);
    
    pinMode(SD_CS, OUTPUT); 
    //SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
    Serial.println("SD Card SPI initialized");
    
    // Causes Camera Capture Failure
    I2S.setAllPins(-1, 42, 41, -1, -1);
    if (!I2S.begin(PDM_MONO_MODE, SAMPLE_RATE, SAMPLE_BITS)) {
      Serial.println("Failed to initialize I2S!");
      while (1);
    }
    Serial.println("I2S (Microphone) initialized");

    mySerial.begin(9600, SERIAL_8N1, RXD0, TXD0);
    Serial.println("UART Communication initialized");

    // init camera
    while (!camera.begin().isOk()) 
        Serial.println(camera.exception.toString());
    Serial.println("Camera initialized");
  
    // init recognizer
    while (!recognition.begin().isOk())
        Serial.println(recognition.exception.toString());
    Serial.println("Recognizer initialized");
    
    // delete stored data, if user confirms
    /*
    if (prompt("Do you want to delete all existing faces? [yes|no]").startsWith("y")) {
        Serial.println("Deleting all existing faces...");
        recognition.deleteAll();
    }
    */
    Serial.println("Deleting all existing faces...");
    recognition.deleteAll();

    // dump stored faces, if user confirms
    /*
    if (prompt("Do you want to dump existing faces? [yes|no]").startsWith("y")) {
        recognition.dump();
    }
    */
    recognition.dump();

    Serial.println("Awaiting for face...");
}

void loop() {
  if (millis() - lastDetectionTime < detectionCooldown) 
  {
    // Still in cooldown period
    return;
  }

  // === CAMERA CAPTURE ===
  if (!camera.capture().isOk()) {
    Serial.print("Camera capture failed: ");
    Serial.println(camera.exception.toString());
    return;
  }

  // === FACE DETECTION ===
  if (!recognition.detect().isOk())
    return;

  reading = digitalRead(D0);
  Serial.println(reading);

  Serial.print("Waiting for button press... current state = ");
  Serial.println(digitalRead(BUTTON));


  // === ENROLL MODE: Button is actively held ===
  if (reading == 1) {
    digitalWrite(FLAG_OUT, HIGH);
    while (digitalRead(FLAG_IN) != 0) {
      Serial.println("Receiver Flag Raised\nWaiting for flag to open");
      delay(100);
    }

    Serial.println("Mounting SD Card");
    SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);

    while (!SD.begin(SD_CS)) {
      Serial.println("Failed to mount SD Card!");
      delay(1000);
    }

    Serial.println("SD Card mounted");
    enroll();
    SPI.end();

    releaseSharedPins();
    reading = 0;
    Serial.println("Awaiting for face...");
    return;
  }

  // === RECOGNITION ===
  if (!recognition.recognize().isOk()) {
    Serial.println(recognition.exception.toString());
    return;
  }

  // === UNKNOWN FACE ===
 String currentName = String(recognition.match.name.c_str());
 if (currentName == "empty" || currentName.length() == 0)
 {
    Serial.println("Unknown face detected.");
    sendPromptAudio(0);   // "Unknown person"
    delay(1000);
    sendPromptAudio(97);  // "Do you want to save this person?"
    delay(5000);

    // Give user 5 seconds to press button
    unsigned long start = millis();
    bool shouldEnroll = false;

    while (millis() - start < 5000) {
      if (digitalRead(D0) == 1) {
        shouldEnroll = true;
        break;
      }
    }

    if (shouldEnroll) {
      releaseSharedPins();         // TEMPORARILY release SPI so slave can use it
      delay(100);                  // give a little buffer
      sendPromptAudio(96);         // Play “Beep” or “Start Speaking”
      delay(1000);                 // give time for playback to finish

      // Reclaim SPI and start recording
      digitalWrite(FLAG_OUT, HIGH);
      while (digitalRead(FLAG_IN) != 0) {
        Serial.println("Receiver Flag Raised\nWaiting for flag to open");
        delay(100);
      }

      Serial.println("Reclaiming SD for enrollment...");
      SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);

      while (!SD.begin(SD_CS)) {
        Serial.println("Failed to mount SD Card!");
        delay(1000);
      }

      Serial.println("SD Card mounted. Beginning enrollment...");
      enroll();
      releaseSharedPins();

    } else {
      Serial.println("User chose not to enroll.");
    }

    Serial.println("Awaiting for face...");
    return;
  }

  // === KNOWN FACE (Recognized) ===
  if (reading == 0 && currentName != "empty" && currentName.length() > 0 && isDigit(currentName[0])) {
    if (isFaceInCooldown(currentName)) {
      Serial.printf("Face %s recently seen. Skipping.\n", currentName.c_str());
      return;
    }

    // New or expired face — process
    sendRecognizedFace();
    printRecognitionInfo();

    updateFaceCooldown(currentName);
    Serial.println("Awaiting for face...");
    return;
  }
}

/**
 * Enroll new person
 */
void enroll() 
{
  digitalWrite(FLAG_OUT, HIGH);
  while(digitalRead(FLAG_IN) != 0)
  {
    delay(100);
  }

  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);

  if(!SD.begin(SD_CS))
  {
    Serial.println("Failed to mount SD Card!");
    releaseSharedPins();
    return;
  }

  if (recognition.enroll((String)face_index).isOk()) {
    Serial.printf("Successfully added user %d!", face_index);
    record_wav(face_index);
    sendPromptAudio(99); // "New Person has been saved!" - Audio Snippit
    face_index++;
    lastDetectionTime = millis();  // Start cooldown after enrollment

  }
  else
  {
    Serial.println(recognition.exception.toString());
  }

  releaseSharedPins();
} 


/**
 * Recognize current face
 */
void recognize() {
    if (!recognition.recognize().isOk()) {
        Serial.println(recognition.exception.toString());
        return;
    }
    if (recognition.match.name.c_str() != "empty") {
        mySerial.write('S');
        int num; 
        sscanf(recognition.match.name.c_str(), "%d", &num);
        mySerial.write(num);
    }

    Serial.print("Recognized face as ");
    Serial.print(recognition.match.name.c_str());
    Serial.print(" with confidence ");
    Serial.print(recognition.match.similarity);
    Serial.print(" (");
    Serial.print(recognition.benchmark.millis());
    Serial.println("ms)");
}


// Function to check for a long button press in order to delete faces
void checkForLongPressToDelete() {
  static unsigned long buttonPressTime = 0;
  static bool longPressActive = false;

  int reading = digitalRead(BUTTON);

  if (reading == HIGH && buttonPressTime == 0) {
    buttonPressTime = millis();  // start timing
  }

  if (reading == LOW && buttonPressTime > 0 && !longPressActive) {
    buttonPressTime = 0;  // reset
  }

  if (reading == HIGH && !longPressActive && (millis() - buttonPressTime >= 3000)) {
    longPressActive = true;
    Serial.println("Long press detected. Prompting delete.");

    sendPromptAudio(96);  // 96.wav = "Do you want to delete all faces?"

    // Start confirmation window
    unsigned long startConfirm = millis();
    bool shouldDelete = false;

    while (millis() - startConfirm < 5000) {
      if (digitalRead(BUTTON) == HIGH) {
        shouldDelete = true;
        break;
      }
    }

    if (shouldDelete) {
      Serial.println("Confirmed. Deleting all faces...");
      recognition.deleteAll();
      sendPromptAudio(95);  // 95.wav = "All faces deleted."
    } else {
      Serial.println("Delete cancelled.");
      sendPromptAudio(94);  // 94.wav = "Cancelled."
    }

    // Reset tracking
    buttonPressTime = 0;
    longPressActive = false;
    delay(1000);  // debounce after delete
  }
}

