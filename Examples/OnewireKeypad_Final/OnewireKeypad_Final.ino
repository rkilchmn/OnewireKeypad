// Creator: Andrew Mascolo

#include <OnewireKeypad.h>
char KEYS[] = {
  '1', '2', '3',
  '4', '5', '6',
  '7', '8', '9',
  '*', '0', '#',
};
const long R_ROW[] = {22, 15, 15};
const long R_COL[] = {68, 68};
OnewireKeypad <Print, 12 > keyPad(Serial, KEYS, 4, 3, PIN_ONEWIREKEYPAD_ADC, 4700, 1000, 1000);


void setup () {
  Serial.begin(115200);

   // keypad
  keyPad.SetResistors( R_ROW, R_COL, 220,  450, 1023); 
  keyPad.SetDebounceTime( 25);

  keyPad.ShowRange();  
}

void loop() {

  if ( char key = keyPad.Getkey() ) {
    Serial << "Key: " << key << " State: ";
    switch (keyPad.Key_State()) {
      case PRESSED:
        Serial.println("PRESSED");
        break;

      case RELEASED:
        Serial.println("RELEASED");
        break;

      case HELD:
        Serial.println("HOLDING");
        break;
    }
  }
}


