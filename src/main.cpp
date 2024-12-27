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

bool encoderPressed = false;
uint8_t calibratedADCReadTime = 24;

uint16_t* samples = nullptr;
uint16_t sampleCount;
double sampleDelay;

void reallocateSamples() {
	delete samples;
	sampleCount = settings.scaleTimeMicroseconds / calibratedADCReadTime;
	sampleDelay = (double) settings.scaleTimeMicroseconds / sampleCount;
	samples = new uint16_t[sampleCount];
}

void calibrateADCReading() {
	const uint8_t iterations = 255;

	uint64_t time = micros();

	for (int i = 0; i < iterations; i++)
		adc1_get_raw(ADC1_CHANNEL_0);

	calibratedADCReadTime = (micros() - time) / iterations;
}

void readEncoder() {
	if (!encoder.wasInterrupted())
		return;

	encoder.acknowledgeInterrupt();

	// Mode cycle
	if (encoder.isPressed() && !encoderPressed) {
		auto uintMode = (uint8_t) settings.mode;
		uintMode++;

		if (uintMode > 2)
			uintMode = 0;

		settings.mode = (Mode) uintMode;
	}

	encoderPressed = encoder.isPressed();

	// Rotation
	if (abs(encoder.getRotation()) > 3) {
		switch (settings.mode) {
			case Mode::Pause: {
				settings.samplingEnabled = !settings.samplingEnabled;
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

		encoder.setRotation(0);
	}

	settings.delayWrite();
}

void readSamples() {
	if (!settings.samplingEnabled)
		return;

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

void render() {
	const auto& screenSize = screenBuffer.getSize();
	const auto& chartBounds = Bounds(0, 0, 71, screenSize.getHeight());

	const uint8_t sideMargin = 4;

	const auto& sideBounds = Bounds(
		Point(chartBounds.getX() + chartBounds.getWidth() + sideMargin, chartBounds.getY()),
		Size(screenSize.getWidth() - chartBounds.getWidth() - sideMargin, screenSize.getHeight())
	);

	screenBuffer.clear(&bg);

	// Snaps
	const uint8_t snapSize = 8;

	for (int32_t snapX = chartBounds.getX() + snapSize - 1; snapX <= chartBounds.getX2(); snapX += snapSize) {
		for (int32_t snapY = chartBounds.getY2() - snapSize; snapY >= chartBounds.getY(); snapY -= snapSize) {
			screenBuffer.renderPixel(Point(snapX, snapY), &fg);
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

	const auto drawSide = [&](bool filled) {
		if (filled)
			screenBuffer.renderFilledRectangle(Bounds(sideBounds.getX(), sideY, sideBounds.getWidth(), font.getHeight()), 2, &fg);

		screenBuffer.renderText(
			Point(
				sideBounds.getX() + 3,
				sideY
			),
			&font,
			filled ? &bg : &fg,
			textBuffer
		);

		sideY += font.getHeight();
	};

	wcscpy(textBuffer, settings.samplingEnabled ? L"Scan" : L"Pause");
	drawSide(settings.mode == Mode::Pause);

	if (settings.scaleTimeMicroseconds < 1000) {
		swprintf(textBuffer, textBufferLength, L"%d us", settings.scaleTimeMicroseconds);
	}
	else if (settings.scaleTimeMicroseconds < 1000000) {
		swprintf(textBuffer, textBufferLength, L"%.1f ms", (float) settings.scaleTimeMicroseconds / 1000.f);
	}
	else {
		swprintf(textBuffer, textBufferLength, L"%.1f s", (float) settings.scaleTimeMicroseconds / 1000000.f);
	}

	drawSide(settings.mode == Mode::ScaleX);

	swprintf(textBuffer, textBufferLength, L"%.1f V", settings.scaleVoltage);
	drawSide(settings.mode == Mode::ScaleY);

	sideY += 3;
	screenBuffer.renderHorizontalLine(Point(sideBounds.getX() + 2, sideY), sideBounds.getWidth() - 2 * 2, &fg);
	sideY += 3;

	swprintf(textBuffer, textBufferLength, L"%.1f-%.1f V", minVoltage, maxVoltage);
	drawSide(false);

	if (raisingRate < 1000) {
		swprintf(textBuffer, textBufferLength, L"%.1f Hz", raisingRate);
	}
	else if (raisingRate < 1000000) {
		swprintf(textBuffer, textBufferLength, L"%.1f kHz", raisingRate / 1000.f);
	}
	else {
		swprintf(textBuffer, textBufferLength, L"%.1f mHz", raisingRate / 1000000.f);
	}

	drawSide(false);

	screenBuffer.flush();
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

	reallocateSamples();
}

void loop() {
	readEncoder();
	readSamples();
	render();
	settings.tick();

	delay(1000 / 24);
}