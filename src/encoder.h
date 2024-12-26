#pragma once

#include <Arduino.h>
#include "FunctionalInterrupt.h"

class Encoder {
	public:
		Encoder(const uint8_t& clkPin, const uint8_t& dtPin, const uint8_t& swPin) :
			_clkPin(clkPin),
			_dtPin(dtPin),
			_swPin(swPin)
		{

		}

		void setup() {
			pinMode(_clkPin, INPUT_PULLUP);
			pinMode(_swPin, INPUT_PULLUP);
			pinMode(_dtPin, INPUT_PULLUP);

			attachInterrupt(digitalPinToInterrupt(_clkPin), std::bind(&onClkOrDtInterrupt, this), CHANGE);
			attachInterrupt(digitalPinToInterrupt(_dtPin), std::bind(&onClkOrDtInterrupt, this), CHANGE);
			attachInterrupt(digitalPinToInterrupt(_swPin), std::bind(&onSwInterrupt, this), CHANGE);

			// Updating initial values
			_pressed = readSw();
			_oldClkAndDt = readClkAndDt();
		}

		static void onClkOrDtInterrupt(Encoder* encoder) {
			encoder->readRotation();
			encoder->_interrupted = true;
		}

		static void onSwInterrupt(Encoder* encoder) {
			encoder->readPressed();
			encoder->_interrupted = true;
		}

		bool wasInterrupted() const {
			return _interrupted;
		}

		void acknowledgeInterrupt() {
			_interrupted = false;
		}

		bool isPressed() const {
			return _pressed;
		}

		int16_t getRotation() const {
			return _rotation;
		}

		void setRotation(int16_t value) {
			_rotation = value;
		}

	private:
		uint8_t
			_clkPin,
			_dtPin,
			_swPin;

		int16_t _rotation = 0;
		bool _pressed = false;
		bool _interrupted = false;
		uint8_t _oldClkAndDt = 0;

		uint8_t readClkAndDt() const {
			return (digitalRead(_clkPin) << 1) | digitalRead(_dtPin);
		}

		bool readSw() const {
			return !digitalRead(_swPin);
		}

		void readPressed() {
			_pressed = readSw();
		}

		void readRotation() {
			auto clkAndDt = readClkAndDt();

			if (clkAndDt != _oldClkAndDt) {
				switch (_oldClkAndDt | (clkAndDt << 2)) {
					case 0: case 5: case 10: case 15:
						break;
					case 1: case 7: case 8: case 14:
						_rotation++;
						break;
					case 2: case 4: case 11: case 13:
						_rotation--;
						break;
					case 3: case 12:
						_rotation += 2;
						break;
					default:
						_rotation -= 2;
						break;
				}

				_oldClkAndDt = clkAndDt;
			}
		}
};
