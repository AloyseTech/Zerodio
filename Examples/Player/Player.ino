/*
  ###########################
          EXPERIMENTAL
  ###########################
  Audio player, non blocking.
  read 8bit mono .wav file
  Sample rate tested : 22050
  use Audacity to convert your audio file

  !!!! Don't connect speaker directly to the DAC !!!!
  See here for schematics : https://www.arduino.cc/en/Tutorial/SimpleAudioPlayerZero

  The audio file should play right after the Arduino Zero boot,
  and each time the character 'p' is received on serial port
  
  Author : AloyseTech
*/

#include <SD.h>
#include <SPI.h>
#include <zerodio.h>

// which Serial you prefer, for information.
#define SERIAL SerialUSB
//#define SERIAL Serial

// SD chip select pin (with ethernet shield : 4)
#define YOUR_SD_CS 6

//your wav file
const char *filename = "music.wav";

//indicate sample rate here (use audacity to convert your wav)
const unsigned int sampleRate = 22050;

void setup()
{
  pinMode(13, OUTPUT);

  // debug output at 115200 baud
  delay(10);
  SERIAL.begin(115200);
  delay(500);

  // setup SD-card
  SERIAL.print("Initializing SD card...");
  if (!SD.begin(YOUR_SD_CS)) {
    SERIAL.println(" failed!");
    return;
  }
  SERIAL.println(" done.");

  // hi-speed SPI transfers
  // TODO: should be replaced by beginTx and endTx in SD lib...
  SPI.setClockDivider(4);

  AudioPlayer.begin(sampleRate);
  AudioPlayer.play(filename);
  SERIAL.println("Playing file.....");
}

void loop()
{
  digitalWrite(13, HIGH);
  delay(200);
  digitalWrite(13, LOW);
  delay(200);
  if (SERIAL.available()) {
    char c = SERIAL.read();
    if (c = 'p') {
      AudioPlayer.play(filename);
      SERIAL.println("Replaying file...");
    }
  }

}
