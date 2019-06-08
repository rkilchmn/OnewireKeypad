//OnewireKP.h
/*
The MIT License (MIT)

Copyright (c) 2016 Andrew Mascolo Jr

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

/******************************************************************************************/
/*    To V_ref --- R_pullup - (To A/D input)- R_col0 	- R_col1 	-  ... R_col(cols-1)      */
/*                                        	X         X         X        									*/
/*                 R_row0                 	X         X         X        									*/
/*                 R_row1                   X         X         X        									*/
/*                 R_row2                   X         X         X        									*/
/*             ... R_row(rows-1)                                                          */
/*                                                                                        */
/*   Reccomend to generate Keypad resistor values by                           						*/
/*   1-Wire Keyboard 1.2.0b software from www.rau-deaver.org/Electronics                  */
/******************************************************************************************/

#ifndef OnewireKeypad_h
#define OnewireKeypad_h
#include <Arduino.h>
#include "BitBool.h"

#define NO_KEY '\0'
#define WAITING 0
#define PRESSED 1
#define RELEASED 2
#define HELD 3

#define EXTREMEPREC 100
#define HIGHPREC 50
#define MEDIUMPREC 20
#define LOWPREC 5

struct MissingType {};

#ifdef LiquidCrystal_I2C_h
typedef LiquidCrystal_I2C LCDTYPE;
#else
typedef MissingType LCDTYPE;
#endif

template<class T> inline Print &operator<<(Print &Outport, T str) {
	Outport.print(str);
	return Outport;
}

/*--- Main Class ---*/
template< typename T, unsigned MAX_KEYS >
class OnewireKeypad {
  public:
    OnewireKeypad( T &port, char KP[], uint8_t Rows, uint8_t Cols, uint8_t Pin, long R1, int R2, int R3)
      : port_( port ), latchedKey( BitBool< MAX_KEYS >() ), _Data( KP ), _Rows( Rows ), _Cols( Cols ), _Pin( Pin ), holdTime( 500 ), debounceTime(200), startTime( 0 ), lastState( 0 ), lastRead( 0 ), voltage( 5.0 ), ANALOG_FACTOR(1023 / 5.0), Num( 0 ), R1( R1 ), R2( R2 ), R3( R3 ) { }

    OnewireKeypad( T &port, char KP[], uint8_t Rows, uint8_t Cols, uint8_t Pin, int R1, int R2)
      : port_( port ), latchedKey( BitBool< MAX_KEYS >() ), _Data( KP ), _Rows( Rows ), _Cols( Cols ), _Pin( Pin ), holdTime( 500 ),debounceTime(200), startTime( 0 ), lastState( 0 ), lastRead( 0 ), voltage( 5.0 ), ANALOG_FACTOR(1023 / 5.0), Num( 0 ), R1( R1 ), R2( R2 ) { }

    OnewireKeypad( T &port, char KP[], uint8_t Rows, uint8_t Cols, uint8_t Pin)
      : port_( port ), latchedKey( BitBool< MAX_KEYS >() ), _Data( KP ), _Rows( Rows ), _Cols( Cols ), _Pin( Pin ), holdTime( 500 ),debounceTime(200), startTime( 0 ), lastState( 0 ), lastRead( 0 ), voltage( 5.0 ), ANALOG_FACTOR(1023 / 5.0), Num( 0 ) { }

    char	Getkey();
		char 	Getkey(int Precision);
		char 	DetermineKey(int pinReading);
    void	SetHoldTime(unsigned long setH_Time) { holdTime = setH_Time; }
    void	SetDebounceTime(unsigned long setD_Time) { debounceTime = setD_Time; }
		void	SetKeypadVoltage(float Volts);
		void	SetAnalogPinRange(float range);
		void	SetResistors(const long *R_rows, const long *R_cols, signed long R_pullUpDown, long R_adImpedance, long adMax);
    uint8_t	Key_State();
    bool	readPin();
    void	LatchKey();
    bool	checkLatchedKey(char _key);
    void	addEventKey(void (*userFunc)(void), char KEY);
    void	deleteEventKey(char KEY);
    void	ListenforEventKey();
    void	ShowRange();
	uint8_t _Pin;

  protected:
    T &port_;
    float ANALOG_FACTOR;
		int KP_TOLERANCE = 20; //This number is based on 1/3 the lowest AR value from the ShowRange function.

  private:
    BitBool< MAX_KEYS > latchedKey;
    char 	*_Data;
    uint8_t _Rows, _Cols; //, _Pin;
    enum { SIZE = MAX_KEYS };
    uint8_t Num;
    float 	voltage;
    unsigned long time;
    unsigned long holdTime;
    unsigned long startTime;
    bool 	state, lastState, lastRead;
    long 	R1, R2, R3;
    unsigned long debounceTime, lastDebounceTime;
    unsigned int pinRange;
		float 	lastReading;
    struct {
		void(*intFunc)();
		char keyHolder;
    } Event[MAX_KEYS];

		// multi value resistor matrix
		boolean multiResistor = false;
		long _adLower[MAX_KEYS];
		long _adUpper[MAX_KEYS];

};

template < typename T, typename U > struct IsSameType {
	enum { Value = false };
};

template < typename T > struct IsSameType< T, T > {
	enum { Value = true };
};

template < typename T, unsigned MAX_KEYS >
void OnewireKeypad< T, MAX_KEYS >::SetAnalogPinRange(float range) {
	if (range <= 0 || range > 1023 ) {
		if ( IsSameType< T, LCDTYPE >::Value) {
			port_.print("Error. pinRange must not be less than or equal to 0 or greater than 1023"); // Lcd display
		} else {
			port_.println("Error. pinRange must not be less than or equal to 0 or greater than 1023"); // Serial display
		}
	} else {
		pinRange = range;
	}
}

template < typename T, unsigned MAX_KEYS >
void OnewireKeypad< T, MAX_KEYS >::SetKeypadVoltage(float Volts) {
	if (Volts <= 0 || Volts > 5) {
		if ( IsSameType< T, LCDTYPE >::Value) {
			port_.print("Error. The Voltage must not be less than or equal to 0 or greater than 5"); // Lcd display
		} else {
			port_.println("Error. The Voltage must not be less than or equal to 0 or greater than 5"); // Serial display
		}
	} else {
		voltage = Volts;
		ANALOG_FACTOR = (pinRange / Volts);
	}
}

template < typename T, unsigned MAX_KEYS >
void OnewireKeypad< T, MAX_KEYS >::SetResistors(const long *R_rows, const long *R_cols, signed long R_pullUpDown, long R_adImpedance, long adMax)
{ 
	multiResistor = true;

  if (R_pullUpDown > 0)
  {
    float prev_val_ad = 0;
    float prev_adUpper = 0;
    float prev_overlapp = 0;
		uint8_t prev_idx = 0;

    for (uint8_t i = 0, R = _Rows - 1, C = 0; i < SIZE; i++)
    {
      long R_key = 0;
      // sum up colum resistors
      for (uint8_t j = 0; j < C; j++)
      {
        R_key += R_cols[j];
      }

      // sum up row resistors
      for (uint8_t k = R; k < _Rows - 1; k++)
      {
        R_key += R_rows[k];
      }

      // parallel resistor od R key and R ad (impedance of Analog digital pin to GND)
      float R_par = (R_key * R_adImpedance * 1.0f) / (R_key + R_adImpedance);

      // ad value calculated as voltage drop accros R_par of serial resistors V_ref -> R_pullUpDown -> R_par -> GND
      float val_ad = (adMax * R_par) / (R_pullUpDown + R_par);

      // first pass:  boundary based on distance to previous ad value<
      float adLower = max(0.0f, val_ad - ((val_ad - prev_val_ad) / 2));// max used for edge case i=0
      float adUpper = val_ad + ((val_ad - prev_val_ad) / 2);

      // second pass: adjust for overlapping boundaries
      float overlapp = 0; 
      if (i >= 2) // special case for i=1,2
        overlapp = adLower - prev_adUpper; 

			uint8_t idx = (R*_Cols) + C; // position in _Data

      _adLower[idx] = ceil( adLower - (overlapp / 2));
      _adUpper[idx] = ceil( adUpper);
			KP_TOLERANCE = - (_adUpper[idx]+(_adUpper[idx]-_adLower[idx]/2));
      if (i > 0) 
      {
        if (i == 1)
          _adUpper[prev_idx] = _adLower[idx] - 1; // edge case i = 1
        else 
          _adUpper[prev_idx] = floor( prev_adUpper + (overlapp / 2));
      }

      prev_val_ad = val_ad;
      prev_adUpper = adUpper;
      prev_overlapp = overlapp;
			prev_idx = idx;

      if (R == 0)
        {
          C++;
          R = _Rows - 1;
        }
        else R--;
    }
	}
}

template < typename T, unsigned MAX_KEYS >
char OnewireKeypad< T, MAX_KEYS >::Getkey() {
	// Check R3 and set it if needed
	if ( R3 == 0 ) { R3 = R2; }

	boolean state = readPin();
	
	int pinReading = analogRead(_Pin);
		
	unsigned long currentMillis = millis();
		
	if (state != lastReading ) {
		lastDebounceTime = currentMillis;
		lastReading = state;
	}
		
	if ( ((currentMillis - lastDebounceTime) > debounceTime) ) {
		if (state == false) { return NO_KEY; }
#ifdef DEBUG
		//port_ << "Get Key (): reading=" << pinReading << '\n';
#endif
		return DetermineKey( pinReading);
	}
	
  return NO_KEY;
}

template < typename T, unsigned MAX_KEYS >
char OnewireKeypad< T, MAX_KEYS >::Getkey( int Precision) {
	float Reading = 0;
	char returnValue = NO_KEY;

	if (readPin()) {
		if (Precision > 1){
			for (size_t passes = 0; passes < Precision; passes++) {
				Reading += analogRead(_Pin);
				yield();
			}
			Reading = floor( (Reading / Precision) + 0.5f); // mimic round()
		}
		else {
			Reading = analogRead(_Pin);
		}
		returnValue = DetermineKey( (int) Reading);
	}
	
#ifdef DEBUG
		if (returnValue != NO_KEY)
			port_ << "GK(" << Precision << "):'" << (int) returnValue << "'," << Reading << '\n';
#endif
	
	return returnValue;
}

template < typename T, unsigned MAX_KEYS >
char OnewireKeypad< T, MAX_KEYS >::DetermineKey( int pinReading) {
	for ( uint8_t i = 0, R = _Rows - 1, C = _Cols - 1; i < SIZE; i++ ) {
		if (multiResistor) {
			if ( (pinReading <= _adUpper[i]) &&  (pinReading >= _adLower[i])) 
#ifdef DEBUG
				//port_ << "Determine Key:" << _Data[i] << " " << _adLower[i] << " >= " << pinReading << " <= " << _adUpper[i] << '\n'; // Serial display
#endif				
				return _Data[i];
		} 
		else {
			float V = (voltage * float( R3 )) / (float(R3) + (float(R1) * float(R)) + (float(R2) * float(C)));
			float Vfinal = V * ANALOG_FACTOR;
			
			if ( pinReading <= (int(Vfinal) + 1.9f )) {
				return _Data[(SIZE - 1) - i];
			}
		}

		if ( C == 0 ) {
			R--;
			C = _Cols - 1;
		} else { C--;}
	}
}

template < typename T, unsigned MAX_KEYS >
uint8_t OnewireKeypad< T, MAX_KEYS >::Key_State() {
	uint8_t returnValue;

	if ((state = readPin()) != lastState) {
		returnValue = ( (lastState = state) ? PRESSED : RELEASED); //MOD
#ifdef DEBUG
		port_ << "KS()=" << returnValue << '\n';
#endif
		return returnValue;
	} else if (state) {
		time = millis();

		while (readPin()) {
			if ((millis() - time) > holdTime) {
				returnValue = HELD;
#ifdef DEBUG
				port_ << "KS()=" << returnValue << '\n';
#endif
				return returnValue;
			}
			yield();
		}
		lastState = 0;
		returnValue = RELEASED;
#ifdef DEBUG
		port_ << "KS()=" << returnValue << '\n';
#endif
		return returnValue;
	}
	returnValue = WAITING;
	return returnValue;
}

template < typename T, unsigned MAX_KEYS >
bool OnewireKeypad<T, MAX_KEYS >::readPin() { 
	 int read = analogRead(_Pin);
	 if (KP_TOLERANCE >= 0) {
		 // return 0 if >= tolerance
		 return read >= KP_TOLERANCE; 
	 }
	 else {
		// return 0 if <= tolerance
		 return read <= - KP_TOLERANCE; 
	 }
}


template < typename T, unsigned MAX_KEYS >
void OnewireKeypad<T, MAX_KEYS >::LatchKey() {
	char output[20];
	bool PRINT = false;
	char read = Getkey();
	
	if (read != lastRead) {
		if ( read ) {
			for ( int idx = 0; idx < SIZE; idx++ ) {
				if (read == _Data[idx] ) {
					strcpy_P( output, ( latchedKey[idx] = !latchedKey[idx] ) ? PSTR( "Key X was Latched" ) : PSTR( "Key X was Unlatched" ) ); //MOD
					PRINT = true;
					output[ 4 ] = read; //MOD <- very clever
				}
			}
		}
    
		lastRead = read;
		if ( PRINT ) {
			if ( IsSameType< T, LCDTYPE >::Value) {
				port_.print(output); // Lcd display
			} else {
				port_.println(output); // Serial display
			}
		}
	}
}

template < typename T, unsigned MAX_KEYS >
bool OnewireKeypad< T, MAX_KEYS >::checkLatchedKey(char _key) {
	for ( uint8_t idx = 0; idx < SIZE; idx++ ) {
		if ( _key == _Data[idx] ) { return latchedKey[idx]; }
	}
	return false;
}

template < typename T, unsigned MAX_KEYS >
void OnewireKeypad< T, MAX_KEYS >::addEventKey(void (*userFunc)(void), char KEY) {
	Event[Num].intFunc = userFunc;
	Event[Num].keyHolder = KEY;
	if (Num < MAX_KEYS) {
		Num++;
	} else {
		if ( IsSameType< T, LCDTYPE >::Value) {
			port_.print("Too Many EventKeys"); // Lcd display
		} else {
			port_.println("Too Many EventKeys"); // Serial display
		}
	}
}

template < typename T, unsigned MAX_KEYS >
void OnewireKeypad< T, MAX_KEYS >::deleteEventKey(char KEY) {
	for (uint8_t idx = 0; idx < Num; idx++) {
		if (KEY == Event[ idx ].keyHolder) {
			Event[ idx ].intFunc = NULL;
			Event[ idx ].keyHolder = '~'; // CHANGED from '\0' to '~', because '\0' was causing issues.
			break;
		}
	}
}

template < typename T, unsigned MAX_KEYS >
void OnewireKeypad< T, MAX_KEYS >::ListenforEventKey() {
	for (uint8_t idx = 0; idx < Num; idx++) {
		if (Getkey() == Event[ idx ].keyHolder) {
			if (Key_State() == RELEASED) {
				Event[ idx ].intFunc();
				break;
			}
		}
	}
}

template < typename T, unsigned MAX_KEYS >
void OnewireKeypad< T, MAX_KEYS >::ShowRange() {
	if (R3 == 0) { R3 = R2; }
	port_ << '\n';
	for ( uint8_t R = 0; R < _Rows; R++) {
		for ( uint8_t C = 0; C < _Cols; C++) {
			float V = (voltage * float( R3 )) / (float(R3) + (float(R1) * float(R)) + (float(R2) * float(C)));
      
			if ( !IsSameType< T, LCDTYPE >::Value)
				if (multiResistor) {
					uint8_t i = (R*_Cols) + C; // position in _Data
					port_ << "key:" << _Data[i] << " low:" << _adLower[i] << " high:" << _adUpper[i] << " | "; // 204.6 is from 1023/5.0
				}
				else
					port_ << "V:" << V << ", AR: " << (V * ANALOG_FACTOR) << " | "; // 204.6 is from 1023/5.0
		}
		if ( !IsSameType< T, LCDTYPE >::Value)
			port_.println("\n--------------------------------------------------------------------------------");
	}
}

#endif
