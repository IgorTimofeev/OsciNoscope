#pragma once

#include <EEPROM.h>

enum class Mode : uint8_t {
	Pause,
	ScaleX,
	ScaleY
};

#pragma pack(push, 1)
class Settings {
	public:
		Mode mode = Mode::Pause;
		float scaleVoltage = 3.3f;
		uint32_t scaleTimeMicroseconds = 5000;
		bool samplingEnabled = true;

		void read() {
			Serial.println("Reading settings");

			EEPROM.begin(1 + sizeof(Settings));

			if (EEPROM.readByte(0) != _writtenFlag)
				return;

			Serial.println("Settings was written");
			EEPROM.readBytes(1, this, sizeof(Settings));
		}

		void delayWrite() {
			_writeTime = millis() + 1500;
		}

		void tick() {
			if (_writeTime == 0 || millis() < _writeTime)
				return;

			Serial.println("Writing settings");

			EEPROM.writeByte(0, _writtenFlag);
			EEPROM.writeBytes(1, this, sizeof(Settings));
			EEPROM.commit();

			_writeTime = 0;
		}

	private:
		static const uint8_t _writtenFlag = 0xAB;
		uint32_t _writeTime = 0;

};
#pragma pack(pop)