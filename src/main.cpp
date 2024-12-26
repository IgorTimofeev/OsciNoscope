#include <Arduino.h>
#include <Wire.h>
#include "../lib/YOBA/src/hardware/screen/drivers/ST7565Driver.h"
#include "../lib/YOBA/src/hardware/screen/buffers/monochromeBuffer.h"
#include "../lib/YOBA/src/resources/fonts/PIXY10Font.h"
#include "encoder.h"
#include <driver/adc.h>

using namespace yoba;

ST7565Driver screenDriver = ST7565Driver(
	7,
	6,
	11
);

MonochromeBuffer screenBuffer = MonochromeBuffer(&screenDriver);

Encoder encoder = Encoder(
	12,
	18,
	19
);

MonochromeColor bg = MonochromeColor(false);
MonochromeColor fg = MonochromeColor(true);
PIXY10Font font = PIXY10Font();

float scaleVoltage = 3.3;
uint32_t scaleTimeMicroseconds = 3200;
float ADCVoltage = 3.3;

uint32_t renderTime = 0;

bool scaleXMode = true;
bool wasPressed = false;

uint8_t meanADCReadTime = 24;

void setup() {
	Serial.begin(115200);

	// Backlight
	pinMode(8, OUTPUT);
	digitalWrite(8, HIGH);

	// ADC
	adc1_config_width(ADC_WIDTH_BIT_12);
	adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_12);

	// Calibrating mean ADC read time
	const uint8_t sampleCount = 255;

	uint64_t time = micros();

	for (int i = 0; i < sampleCount; i++)
		adc1_get_raw(ADC1_CHANNEL_0);

	meanADCReadTime = (micros() - time) / sampleCount;

	Serial.printf("Mean ADC read time: %d\n", meanADCReadTime);

	// Encoder
	encoder.setup();

	// Screen
	screenBuffer.setup();
}

void loop() {
	// Encoder
	if (encoder.wasInterrupted()) {
		encoder.acknowledgeInterrupt();

		if (encoder.isPressed() && !wasPressed)
			scaleXMode = !scaleXMode;

		wasPressed = encoder.isPressed();

		if (abs(encoder.getRotation()) > 3) {
			if (encoder.getRotation() > 0) {
				if (scaleXMode) {
					scaleTimeMicroseconds = std::min(scaleTimeMicroseconds + 100, 5000000UL);
				}
				else {
					scaleVoltage = std::min(scaleVoltage + 0.1f, 100.f);
				}
			}
			else {
				if (scaleXMode) {
					if (scaleTimeMicroseconds > 200) {
						scaleTimeMicroseconds -= 100;
					}
				}
				else {
					scaleVoltage -= 0.1f;

					if (scaleVoltage < 1)
						scaleVoltage = 1;
				}
			}

			encoder.setRotation(0);
		}
	}

	const auto& screenSize = screenBuffer.getSize();
	const auto& chartBounds = Bounds(0, 0, 80, screenSize.getHeight());

	const auto& sideBounds = Bounds(
		Point(chartBounds.getX() + chartBounds.getWidth() + 2, chartBounds.getY()),
		Size(screenSize.getWidth() - chartBounds.getWidth() - 2, screenSize.getHeight())
	);

	const uint16_t sampleCount = scaleTimeMicroseconds / meanADCReadTime;
	const double sampleDelay = (double) scaleTimeMicroseconds / sampleCount;
	double sampleTime = 0;
	double time;

	uint16_t samples[sampleCount];

	for (auto& sample : samples) {
		do {
			time = (double) micros();
		}
		while (time < sampleTime);

		sampleTime = time + sampleDelay;
		sample = adc1_get_raw(ADC1_CHANNEL_0);
	}

	screenBuffer.clear(&bg);

	// Axis
//	screenBuffer.renderHorizontalLine(chartBounds.getBottomLeft(), chartBounds.getWidth(), &fg);
//	screenBuffer.renderVerticalLine(chartBounds.getTopLeft(), chartBounds.getHeight(), &fg);

	// Snaps
	const uint8_t snapCount = 10;
	const uint16_t snapSize = chartBounds.getWidth() / snapCount;

	for (int32_t snapX = chartBounds.getX() + snapSize; snapX < chartBounds.getX2(); snapX += snapSize) {
		for (int32_t snapY = chartBounds.getY2() - snapSize; snapY >= chartBounds.getY(); snapY -= snapSize) {
			screenBuffer.renderPixel(Point(snapX, snapY), &fg);
		}
	}

	// Chart
	Point previousPoint;
	float previousVoltage;
	Point samplePoint;
	float sampleVoltage;
	float minVoltage = ADCVoltage;
	float maxVoltage = 0;

	bool raising = false;
	uint32_t raisingCount = 0;

	for (uint32_t i = 0; i < sampleCount; i++) {
		sampleVoltage = (float) samples[i] / 4096.f * ADCVoltage;

		if (sampleVoltage < minVoltage)
			minVoltage = sampleVoltage;

		if (sampleVoltage > maxVoltage)
			maxVoltage = sampleVoltage;

		samplePoint = Point(
			chartBounds.getX() + (int32_t) ((float) i / (float) sampleCount * (float) chartBounds.getWidth()),
			chartBounds.getY() + chartBounds.getHeight() - 1 - (int32_t) (sampleVoltage / scaleVoltage * (float) chartBounds.getHeight())
		);

		if (i > 0) {
			screenBuffer.renderLine(
				previousPoint,
				samplePoint,
				&fg
			);

			// Raising
			if (sampleVoltage > previousVoltage) {
				if (sampleVoltage >= ADCVoltage * 0.9f) {
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
		previousPoint = samplePoint;
	}

	const float raisingRate = (float) raisingCount * 1000000.f / (float) scaleTimeMicroseconds;

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

	swprintf(textBuffer, textBufferLength, L"%.1f ms", (float) scaleTimeMicroseconds / 1000.f);
	drawSide(scaleXMode);

	swprintf(textBuffer, textBufferLength, L"%.1f V", scaleVoltage);
	drawSide(!scaleXMode);

	sideY += 3;
	screenBuffer.renderHorizontalLine(Point(sideBounds.getX() + 2, sideY), sideBounds.getWidth() - 2 * 2, &fg);
	sideY += 3;

	swprintf(textBuffer, textBufferLength, L"%.1f Vmin", minVoltage);
	drawSide(false);

	swprintf(textBuffer, textBufferLength, L"%.1f Vmax", maxVoltage);
	drawSide(false);

	swprintf(textBuffer, textBufferLength, L"%.1f kHz", raisingRate / 1000.f, raisingCount);
	drawSide(false);

	screenBuffer.flush();

	delay(1000 / 24);
}