#include <EEPROM.h>
#include <Preferences.h>

void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println("Clearing EEPROM ...");
  for (unsigned int i = 0; i < 256; i++)
    EEPROM.write(i,0);
  EEPROM.commit();
  Serial.println("Clearing NVRAM ...");
  Preferences preferences;  
  preferences.clear();
  Serial.println("Done!");
}

void loop() {
}
