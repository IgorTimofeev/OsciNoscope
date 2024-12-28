#include <Arduino.h>
#include <Wire.h>
#include "../lib/YOBA/src/hardware/screen/drivers/ST7565Driver.h"
#include "../lib/YOBA/src/hardware/screen/buffers/monochromeBuffer.h"
#include "../lib/YOBA/src/resources/fonts/PIXY10Font.h"
#include "encoder.h"
#include <driver/adc.h>
#include "settings.h"

using namespace yoba;

const float maxADCVoltage = 33.3;

Settings settings = Settings();

ST7565Driver screenDriver = ST7565Driver(
	7,
	6,
	11
);

MonochromeBuffer screenBuffer = MonochromeBuffer(&screenDriver);

Encoder encoder = Encoder(
	18,
	12,
	19
);

MonochromeColor bg = MonochromeColor(false);
MonochromeColor fg = MonochromeColor(true);
PIXY10Font font = PIXY10Font();

bool samplingEnabled = true;
bool encoderPressed = false;
uint8_t calibratedADCReadTime = 24;

uint16_t* samples = nullptr;
uint16_t sampleCount;
uint32_t renderTime = 0;
uint32_t selectedModeBlinkTime = 0;
bool selectedModeBlink = true;

void allocateSamples() {
	sampleCount = settings.scaleTimeMicroseconds / calibratedADCReadTime;
	samples = new uint16_t[sampleCount];
	memset(samples, 0, sampleCount * sizeof(uint16_t));
}

void reallocateSamples() {
	const auto oldSamples = samples;
	const auto oldSampleCount = sampleCount;

	allocateSamples();

	memcpy(samples, oldSamples, std::min(sampleCount, oldSampleCount) * sizeof(uint16_t));
	delete oldSamples;
}

void calibrateADCReading() {
	const uint8_t iterations = 255;

	uint64_t time = micros();

	for (int i = 0; i < iterations; i++)
		adc1_get_raw(ADC1_CHANNEL_0);

	calibratedADCReadTime = (micros() - time) / iterations;
}

void readSamples() {
	if (!samplingEnabled)
		return;

	const auto sampleDelay = (double) settings.scaleTimeMicroseconds / sampleCount;
	double sampleTime = 0;
	double time;

	for (int i = 0; i < sampleCount; i++) {
		do {
			time = (double) micros();
		}
		while (time < sampleTime);

		sampleTime = time + sampleDelay;
		samples[i] = adc1_get_raw(ADC1_CHANNEL_0);
	}
}

void updateSelectedModeBlinkTime() {
	selectedModeBlinkTime = millis() + 500;
}

void readEncoder() {
	if (!encoder.wasInterrupted())
		return;

	encoder.acknowledgeInterrupt();

	// Toggling mode selection
	if (encoder.isPressed() && !encoderPressed) {
		settings.modeSelected = !settings.modeSelected;

		selectedModeBlink = true;
		updateSelectedModeBlinkTime();
	}

	encoderPressed = encoder.isPressed();

	// Rotation
	if (abs(encoder.getRotation()) > 3) {
		if (settings.modeSelected) {
			switch (settings.mode) {
				case Mode::Pause: {
					samplingEnabled = !samplingEnabled;
					break;
				}
				case Mode::ScaleX: {
					if (encoder.getRotation() > 0) {
						settings.scaleTimeMicroseconds = std::min(settings.scaleTimeMicroseconds + 100, 5000000UL);
					}
					else {
						if (settings.scaleTimeMicroseconds > 200) {
							settings.scaleTimeMicroseconds -= 100;
						}
					}

					reallocateSamples();

					break;
				}
				case Mode::ScaleY: {
					if (encoder.getRotation() > 0) {
						settings.scaleVoltage = std::min(settings.scaleVoltage + 0.1f, maxADCVoltage);
					}
					else {
						settings.scaleVoltage -= 0.1f;

						if (settings.scaleVoltage < 0.1)
							settings.scaleVoltage = 0.1;
					}

					break;
				}
			}
		}
		// Cycling between modes
		else {
			auto uintMode = (uint8_t) settings.mode;

//			if (encoder.getRotation() > 0) {
//				uintMode++;
//
//				if (uintMode > (uint8_t) Mode::Last)
//					uintMode = 0;
//			}
//			else {
//				if (uintMode > 0) {
//					uintMode--;
//				}
//				else {
//					uintMode = (uint8_t) Mode::Last;
//				}
//			}

			if (encoder.getRotation() > 0) {
				if (uintMode < (uint8_t) Mode::Last)
					uintMode++;
			}
			else {
				if (uintMode > 0)
					uintMode--;
			}

			settings.mode = (Mode) uintMode;
		}

		encoder.setRotation(0);
	}

	settings.delayWrite();
}

void render() {
	if (millis() < renderTime)
		return;

	const auto& screenSize = screenBuffer.getSize();
	const auto& chartBounds = Bounds(0, 0, 72, screenSize.getHeight());

	const uint8_t sideMargin = 6;

	const auto& sideBounds = Bounds(
		Point(chartBounds.getX() + chartBounds.getWidth() + sideMargin, chartBounds.getY()),
		Size(screenSize.getWidth() - chartBounds.getWidth() - sideMargin, screenSize.getHeight())
	);

	screenBuffer.clear(&bg);

	// Snaps
	const uint8_t snapXCount = 10;
	const uint8_t snapYCount = 10;
	const auto snapXSize = (float) chartBounds.getWidth() / snapXCount;
	const auto snapYSize = (float) chartBounds.getHeight() / snapYCount;

	for (uint8_t snapY = 1; snapY <= snapYCount; snapY++) {
		for (uint8_t snapX = 1; snapX <= snapXCount; snapX++) {
			screenBuffer.renderPixel(
				Point(
					(int32_t) ((float) chartBounds.getX() - 1 + (float) snapX * snapXSize),
					(int32_t) ((float) chartBounds.getY2() - 1 - (float) snapY * snapYSize)
				),
				&fg
			);
		}
	}

	// Chart
	Point previousPoint;
	float previousVoltage;
	Point samplePoint;
	float sampleVoltage;
	float minVoltage = maxADCVoltage;
	float maxVoltage = 0;

	for (uint32_t i = 0; i < sampleCount; i++) {
		sampleVoltage = (float) samples[i] / 4096.f * maxADCVoltage;

		if (sampleVoltage < minVoltage)
			minVoltage = sampleVoltage;

		if (sampleVoltage > maxVoltage)
			maxVoltage = sampleVoltage;

		samplePoint = Point(
			chartBounds.getX() + (int32_t) ((float) i / (float) sampleCount * (float) chartBounds.getWidth()),
			chartBounds.getY() + chartBounds.getHeight() - 1 - (int32_t) (sampleVoltage / settings.scaleVoltage * (float) chartBounds.getHeight())
		);

		if (i > 0) {
			screenBuffer.renderLine(
				previousPoint,
				samplePoint,
				&fg
			);
		}

		previousVoltage = sampleVoltage;
		previousPoint = samplePoint;
	}

	// Detecting signal frequency
	bool raising = false;
	uint32_t raisingCount = 0;

	for (uint32_t i = 0; i < sampleCount; i++) {
		sampleVoltage = (float) samples[i] / 4096.f * maxADCVoltage;

		if (i > 0) {
			if (sampleVoltage > previousVoltage) {
				if (sampleVoltage >= maxVoltage * 0.9f) {
					if (raising) {
						raising = false;
					}
					else {
						raisingCount++;
						raising = true;
					}
				}
				else {
					raising = false;
				}
			}
		}

		previousVoltage = sampleVoltage;
	}

	const float raisingRate = (float) raisingCount * 1000000.f / (float) settings.scaleTimeMicroseconds;

	// Side
	int32_t sideY = sideBounds.getY();

	const uint8_t textBufferLength = 32;
	wchar_t textBuffer[textBufferLength];

	const auto drawSideText = [&](bool alt = false) {
		const uint8_t textMargin = 4;

		screenBuffer.renderText(
			Point(
				sideBounds.getX() + textMargin,
				sideY
			),
			&font,
			alt ? &bg : &fg,
			textBuffer
		);

		sideY += font.getHeight();
	};

	const auto drawSide = [&](Mode mode) {
		if (settings.mode == mode) {
			if (selectedModeBlink) {
				screenBuffer.renderFilledRectangle(Bounds(sideBounds.getX(), sideY, sideBounds.getWidth(), font.getHeight()), 2, &fg);
			}
			else {
				const uint8_t margin = 1;

				screenBuffer.renderFilledRectangle(
					Bounds(
						sideBounds.getX() + margin,
						sideY + margin, sideBounds.getWidth() - margin * 2,
						font.getHeight() - margin * 2
					),
					2,
					&fg
				);
			}

			drawSideText(true);
		}
		else {
			drawSideText(false);
		}
	};

	wcscpy(textBuffer, samplingEnabled ? L"Scan" : L"Pause");
	drawSide(Mode::Pause);

	if (settings.scaleTimeMicroseconds < 1000) {
		swprintf(textBuffer, textBufferLength, L"%d us", settings.scaleTimeMicroseconds);
	}
	else if (settings.scaleTimeMicroseconds < 1000000) {
		swprintf(textBuffer, textBufferLength, L"%.1f ms", (float) settings.scaleTimeMicroseconds / 1000.f);
	}
	else {
		swprintf(textBuffer, textBufferLength, L"%.1f s", (float) settings.scaleTimeMicroseconds / 1000000.f);
	}

	drawSide(Mode::ScaleX);

	swprintf(textBuffer, textBufferLength, L"%.1f V", settings.scaleVoltage);
	drawSide(Mode::ScaleY);

	sideY += 5;

	swprintf(textBuffer, textBufferLength, L"%.1f-%.1f V", minVoltage, maxVoltage);
	drawSideText();

	if (raisingRate < 1000) {
		swprintf(textBuffer, textBufferLength, L"%.1f Hz", raisingRate);
	}
	else if (raisingRate < 1000000) {
		swprintf(textBuffer, textBufferLength, L"%.1f kHz", raisingRate / 1000.f);
	}
	else {
		swprintf(textBuffer, textBufferLength, L"%.1f mHz", raisingRate / 1000000.f);
	}

	drawSideText();

	screenBuffer.flush();

	if (settings.modeSelected && millis() >= selectedModeBlinkTime) {
		selectedModeBlink = !selectedModeBlink;
		updateSelectedModeBlinkTime();
	}

	renderTime = millis() + 1000 / 30;
}

void setup() {
	Serial.begin(115200);

	// Settings
	settings.read();

	// ADC
	adc1_config_width(ADC_WIDTH_BIT_12);
	adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_12);
	calibrateADCReading();

	// Encoder
	encoder.setup();

	// Screen backlight
	pinMode(8, OUTPUT);
	digitalWrite(8, HIGH);

	// Screen buffer
	screenBuffer.setup();

	allocateSamples();
}

void loop() {
	readEncoder();
	readSamples();
	render();
	settings.tick();

	delay(1);
}