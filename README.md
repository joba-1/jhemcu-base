# jhemcu-base
STM32CubeIDE project for JHEMCU flight controller (using STM32F722RETx chip)

trying to program the flight controller to learn STM programming/debugging.
And maybe I find out, why the I2C bus does not work on my FC with betaflight 4.x.

## Decouple ST-Link from Nucleo chip
* remove 2 ST-Link-Nucleo jumpers on the ST-Link
* move jumper JP5 on the Nucleo from U5V to E5V to cut off power from the F401

## Wiring for Debug:
     JHEMCU <-> ST-Link
          K --- CN4-2 (SWD Clk)
        Gnd --- CN4-3 (Gnd) 
          D --- CN4-4 (SWD Data)

## Next steps
* make led blink
* use uarts to print text via usb2serial adapter
* debug via Nucleo-F401RE board ST-Link/V2.1 adapter
