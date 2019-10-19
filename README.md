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

## First steps
* make led blink -> ok
* use uarts to print text via usb2serial adapter -> ok
* debug via Nucleo-F401RE board ST-Link/V2.1 adapter -> ok
* Try I2C1 init and scan -> no signal on logic analyzer
* Try I2C1 pins as normal GPIO pins:
    * SDC (PB6) ok
    * SDA (PB7) always HIGH (external pullup) -> dead pin
    * alternate I2C1 pins not usable (vtx voltage switch)
* Try using I2C3
    * reuse pads for motor 6 (PA8) and 7 (PC9)
    * betaflight I2C error flag gone, but also no signal

## Next steps
* Try I2C3 init and scan
