## NeoPixel library strandtest example
# Author: Tony DiCola (tony@tonydicola.com)
#
# Direct port of the Arduino NeoPixel library strandtest example.  Showcases
# various animations on a strip of NeoPixels.
import time

from neopixel import *


# LED strip configuration:
LED_COUNT      = 16      # Number of LED pixels.
LED_PIN1        = 18      # GPIO pin connected to the pixels (18 uses PWM!).
LED_PIN2        = 19
#LED_PIN        = 10      # GPIO pin connected to the pixels (10 uses SPI /dev/spidev0.0).
LED_FREQ_HZ    = 800000  # LED signal frequency in hertz (usually 800khz)
LED_DMA        = 5       # DMA channel to use for generating signal (try 5)
LED_BRIGHTNESS = 255     # Set to 0 for darkest and 255 for brightest
LED_INVERT     = False   # True to invert the signal (when using NPN transistor level shift)
LED_CHANNEL0    = 0       # set to '1' for GPIOs 13, 19, 41, 45 or 53
LED_CHANNEL1    = 1
LED_STRIP      = ws.WS2811_STRIP_GRB   # Strip type and colour ordering


# Define functions which animate LEDs in various ways.

def LED_Signal(strip11, strip12, strip21, strip22, wait_ms=100.0):
	"""Wipe color across display a pixel at a time."""

        """First strip"""
        for i in range(0, LED_COUNT/2):
                strip1.setPixelColor(i, color[strip11])
        for i in range(LED_COUNT/2, LED_COUNT):
                strip1.setPixelColor(i, color[strip12])

	"""Second strip"""
        for i in range(0, LED_COUNT/2):
                strip2.setPixelColor(i, color[strip21])
        for i in range(LED_COUNT/2, LED_COUNT):
                strip2.setPixelColor(i, color[strip22])
	
	
	strip1.show()
	strip2.show() 
	time.sleep(wait_ms/1000.0)


def LED_init():
        strip1 = Adafruit_NeoPixel(LED_COUNT, LED_PIN1, LED_FREQ_HZ, LED_DMA, LED_INVERT, LED_BRIGHTNESS, LED_CHANNEL0, LED_STRIP)
        # Intialize the library (must be called once before other functions).
        strip1.begin()
        strip2 = Adafruit_NeoPixel(LED_COUNT, LED_PIN2, LED_FREQ_HZ, LED_DMA, LED_INVERT, LED_BRIGHTNESS, LED_CHANNEL1, LED_STRIP)
        strip2.begin()


# Main program logic follows:
if __name__ == '__main__':
        try:
                # Create NeoPixel object with appropriate configuration.
                strip1 = Adafruit_NeoPixel(LED_COUNT, LED_PIN1, LED_FREQ_HZ, LED_DMA, LED_INVERT, LED_BRIGHTNESS, LED_CHANNEL0, LED_STRIP)
                # Intialize the library (must be called once before other functions).
                strip1.begin()
                strip2 = Adafruit_NeoPixel(LED_COUNT, LED_PIN2, LED_FREQ_HZ, LED_DMA, LED_INVERT, LED_BRIGHTNESS, LED_CHANNEL1, LED_STRIP)
                strip2.begin()

                Red = Color(255, 0 ,0)
                Green = Color(0, 255, 0)
                White = Color(255, 255, 255)
                No_color = Color(0, 0, 0)
                color = [Red, Green, White, No_color]
                enumerate(color)

                print ('Press Ctrl-C to quit.')
                print ('LED Singalling started...')
                while True:
                        LED_Signal(0, 0, 0, 0)
                        LED_Signal(0, 0, 0, 1)
                        LED_Signal(0, 0, 1, 1)
                        LED_Signal(0, 1, 1, 1)
                        LED_Signal(1, 1, 1, 1)
                        
	except KeyboardInterrupt:
                LED_Signal(2, 2, 2, 2)
                LED_Signal(3, 3, 3, 3)
                LED_Signal(2, 2, 2, 2)
                LED_Signal(3, 3, 3, 3)
                LED_Signal(2, 2, 2, 2)
                LED_Signal(3, 3, 3, 3)
                LED_Signal(2, 2, 2, 2)
                LED_Signal(3, 3, 3, 3)
                print()
                print('Successfully quit')

