#include <Arduino.h>
#include <Wire.h>
#include "../lib/YOBA/src/hardware/screen/drivers/ST7565Driver.h"
#include "../lib/YOBA/src/hardware/screen/buffers/monochromeBuffer.h"
#include "../lib/YOBA/src/resources/fonts/PIXY10Font.h"
#include "../lib/YOBA/src/number.h"
#include "encoder.h"
#include <driver/adc.h>
#include "settings.h"

using namespace yoba;

const uint32_t maxADCVoltage = 33300;

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
uint32_t samplingTime = 0;

uint16_t* samples = nullptr;
uint16_t sampleCount;
uint32_t renderTime = 0;

bool modeSelected = false;
bool modeSelectedBlink = true;
uint32_t modeSelectedBlinkTime = 0;

void allocateSamples() {
	sampleCount = settings.scaleTimeMicros / calibratedADCReadTime;
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
	if (!samplingEnabled || millis() < samplingTime)
		return;

	const auto sampleDelay = (double) settings.scaleTimeMicros / sampleCount;
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

	samplingTime = millis() + 1000 / 20;
}

void updateSelectedModeBlinkTime() {
	modeSelectedBlinkTime = millis() + 250;
}

void readEncoder() {
	if (!encoder.wasInterrupted())
		return;

	encoder.acknowledgeInterrupt();

	// Toggling mode selection
	if (encoder.isPressed() && !encoderPressed) {
		modeSelected = !modeSelected;

		modeSelectedBlink = true;
		updateSelectedModeBlinkTime();
	}

	encoderPressed = encoder.isPressed();

	// Rotation
	if (abs(encoder.getRotation()) > 3) {
		if (modeSelected) {
			switch (settings.mode) {
				case Mode::Pause: {
					samplingEnabled = !samplingEnabled;
					break;
				}
				case Mode::ScaleX: {
					if (encoder.getRotation() > 0) {
						settings.scaleTimeMicros = std::min(settings.scaleTimeMicros + 100, 5000000UL);
					}
					else {
						if (settings.scaleTimeMicros > 100) {
							settings.scaleTimeMicros -= 100;
						}
					}

					reallocateSamples();

					break;
				}
				case Mode::ScaleY: {
					if (encoder.getRotation() > 0) {
						if (settings.scaleVoltageMillis < 500) {
							settings.scaleVoltageMillis += 10;
						}
						else if (settings.scaleVoltageMillis < 10000) {
							settings.scaleVoltageMillis += 100;
						}
						else {
							settings.scaleVoltageMillis += 1000;
						}

						if (settings.scaleVoltageMillis > maxADCVoltage)
							settings.scaleVoltageMillis = maxADCVoltage;
					}
					else {
						if (settings.scaleVoltageMillis <= 500) {
							if (settings.scaleVoltageMillis > 10)
								settings.scaleVoltageMillis -= 10;
						}
						else if (settings.scaleVoltageMillis <= 10000) {
							settings.scaleVoltageMillis -= 100;
						}
						else {
							if (settings.scaleVoltageMillis > 1000)
								settings.scaleVoltageMillis -= 1000;
						}
					}

					break;
				}
			}
		}
		// Cycling between modes
		else {
			auto uintMode = (uint8_t) settings.mode;

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
	Point samplePoint;

	uint32_t sampleVoltage;
	uint32_t minVoltage = maxADCVoltage;
	uint32_t maxVoltage = 0;
	uint32_t previousVoltage;

	for (uint32_t i = 0; i < sampleCount; i++) {
		sampleVoltage = samples[i] * maxADCVoltage / 4096;

		if (sampleVoltage < minVoltage)
			minVoltage = sampleVoltage;

		if (sampleVoltage > maxVoltage)
			maxVoltage = sampleVoltage;

		samplePoint = Point(
			chartBounds.getX() + (int32_t) ((float) i / (float) sampleCount * (float) chartBounds.getWidth()),
			chartBounds.getY() + chartBounds.getHeight() - 1 - (int32_t) (sampleVoltage * chartBounds.getHeight() / settings.scaleVoltageMillis)
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
		sampleVoltage = samples[i] * maxADCVoltage / 4096;

		if (i > 0) {
			if (sampleVoltage > previousVoltage) {
				if (sampleVoltage >= (uint32_t) ((float) maxVoltage * 0.9f)) {
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

	const float raisingRate = (float) raisingCount * 1000000.f / (float) settings.scaleTimeMicros;

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
			if (modeSelectedBlink) {
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

	// Scan/pause
	wcscpy(textBuffer, samplingEnabled ? L"Scan" : L"Pause");
	drawSide(Mode::Pause);

	// Scale time
	if (settings.scaleTimeMicros < 1000) {
		swprintf(textBuffer, textBufferLength, L"%d us", settings.scaleTimeMicros);
	}
	else if (settings.scaleTimeMicros < 1000000) {
		swprintf(textBuffer, textBufferLength, L"%.1f ms", (float) settings.scaleTimeMicros / 1000.f);
	}
	else {
		swprintf(textBuffer, textBufferLength, L"%.1f s", (float) settings.scaleTimeMicros / 1000000.f);
	}

	drawSide(Mode::ScaleX);

	// Scale voltage
	if (settings.scaleVoltageMillis < 1000) {
		swprintf(textBuffer, textBufferLength, L"%d mV", settings.scaleVoltageMillis);
	}
	else {
		swprintf(textBuffer, textBufferLength, L"%.1f V", (float) settings.scaleVoltageMillis / 1000.f);
	}

	drawSide(Mode::ScaleY);

	sideY += 5;

	// Min-max
	if (settings.scaleVoltageMillis < 1000) {
		swprintf(textBuffer, textBufferLength, L"%d-%d mV", minVoltage, maxVoltage);
	}
	else {
		swprintf(textBuffer, textBufferLength, L"%.1f-%.1f V", (float) minVoltage / 1000.f, (float) maxVoltage / 1000.f);
	}

	drawSideText();

	// Frequency
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

	if (modeSelected && millis() >= modeSelectedBlinkTime) {
		modeSelectedBlink = !modeSelectedBlink;
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