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

MonochromeColor bg = MonochromeColor(false);
MonochromeColor fg = MonochromeColor(true);
PIXY10Font font = PIXY10Font();

void setup() {
	Serial.begin(115200);

	// Backlight
	pinMode(8, OUTPUT);
	digitalWrite(8, HIGH);

	screenBuffer.setup();
}

int32_t x = 10;
int32_t y = 10;


void loop() {
//	screenBuffer.clear(&bg);
	screenBuffer.renderFilledRectangle(Bounds(0, 0, 128, 64), &bg);

	screenBuffer.renderHorizontalLine(Point(x, y), 30, &fg);
	screenBuffer.renderFilledRectangle(Bounds(x, y + 5, 30, 5), &fg);
	screenBuffer.renderText(Point(x, y + 15), &font, &fg, L"Penis");

	screenBuffer.flush();

//	x++;
//	y++;
//
//	if (x > 128)
//		x = 0;
//
//	if (y > 64)
//		y = 0;

	delay(100);
}