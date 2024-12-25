#include <Arduino.h>
#include <Wire.h>
#include "../lib/YOBA/src/hardware/screen/drivers/ST7565Driver.h"
#include "../lib/YOBA/src/hardware/screen/buffers/monochromeBuffer.h"
#include "../lib/YOBA/src/resources/fonts/PIXY10Font.h"

using namespace yoba;

ST7565Driver screenDriver = ST7565Driver(
	7,
	6,
	11
);

MonochromeBuffer screenBuffer = MonochromeBuffer(&screenDriver);

const uint8_t analogPin = 0;

MonochromeColor bg = MonochromeColor(false);
MonochromeColor fg = MonochromeColor(true);
PIXY10Font font = PIXY10Font();

uint32_t scaleTimeMicroseconds = 1000;
float scaleVoltage = 3.3;
float ADCVoltage = 3.3;

void setup() {
	Serial.begin(115200);

	// Backlight
	pinMode(8, OUTPUT);
	digitalWrite(8, HIGH);

	screenBuffer.setup();
}

void loop() {
	const auto& screenSize = screenBuffer.getSize();
	const auto& chartBounds = Bounds(0, 0, screenSize.getWidth(), screenSize.getHeight());
	const auto& sideBounds = Bounds(screenSize.getWidth() - 60, 0, 50, screenSize.getHeight());

	const uint32_t sampleCount = 32;
	const uint32_t sampleDelay = scaleTimeMicroseconds / sampleCount;

	uint16_t samples[sampleCount];

	uint32_t sampleTime = 0;

	for (uint32_t i = 0; i < sampleCount; i++) {
		if (micros() < sampleTime) {
			asm("nop");
		}
		else {
			sampleTime = micros() + sampleDelay;
			samples[i] = analogRead(analogPin);
		}
	}

	screenBuffer.clear(&bg);

	// Axis
	screenBuffer.renderHorizontalLine(chartBounds.getBottomLeft(), chartBounds.getWidth(), &fg);
	screenBuffer.renderVerticalLine(chartBounds.getTopLeft(), chartBounds.getHeight(), &fg);

	// Snaps
	const uint8_t snapCount = 10;
	const uint16_t snapSize = chartBounds.getWidth() / snapCount;

	for (int32_t snapX = chartBounds.getX() + snapSize; snapX < chartBounds.getX2(); snapX += snapSize) {
		for (int i = chartBounds.getY(); i < chartBounds.getY2(); i += 2) {
			screenBuffer.renderPixel(Point(snapX, i), &fg);
		}
	}

	// Chart
	Point previousPoint;
	Point samplePoint;
	float sampleVoltage;
	float minVoltage = ADCVoltage;
	float maxVoltage = 0;

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
		}

		previousPoint = samplePoint;
	}

	// Side
	int32_t sideY = sideBounds.getY();

	const uint8_t textBufferLength = 32;
	wchar_t textBuffer[textBufferLength];

	const auto drawSide = [&]() {
		const auto& textBounds = Bounds(
			Point(
				sideBounds.getX(),
				sideY
			),
			font.getSize(textBuffer)
		);

		screenBuffer.renderFilledRectangle(textBounds, &bg);

		screenBuffer.renderText(
			textBounds.getTopLeft(),
			&font,
			&fg,
			textBuffer
		);

		sideY += textBounds.getHeight() - 1;
	};

	if (scaleTimeMicroseconds < 1000) {
		swprintf(textBuffer, textBufferLength, L"Ts: %d us", scaleTimeMicroseconds);
	}
	else if (scaleTimeMicroseconds < 1000000) {
		swprintf(textBuffer, textBufferLength, L"Ts: %.1f ms", (float) scaleTimeMicroseconds / 1000.f);
	}
	else {
		swprintf(textBuffer, textBufferLength, L"Ts: %.1f s", (float) scaleTimeMicroseconds / 1000000.f);
	}

	drawSide();

	swprintf(textBuffer, textBufferLength, L"Vs: %.1f", scaleVoltage);
	drawSide();

	swprintf(textBuffer, textBufferLength, L"Vl: %.1f", minVoltage);
	drawSide();

	swprintf(textBuffer, textBufferLength, L"Vm: %.1f", maxVoltage);
	drawSide();

	screenBuffer.flush();

	delay(500);
}