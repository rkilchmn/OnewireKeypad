A fork from https://github.com/AndrewMascolo/OnewireKeypad

I found that most AD based libraries did not work for me with a 4x3 matrix as at least 1 key did overlap and was not reliable. So i did some Improvements:
- use multiple resitor values to increase precission 
- support pullup and pulldown schematics 
- take analog digital converters impedance (resitor value) into account which can be used to fine-tune

see examples/OnewireKeypad_Final for more information to how it works

Using calculation tool (see "1 Wire Keyboard CalculatorWIN.zip") for resistor values from this site http://www.rau-deaver.org/1-wire%20keyboard.html