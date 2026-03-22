# Hardware Bringup
## Sensors
- [X] Reading MS5611 over SPI 
- [X] Reading LSMDSV over SPI
- [ ] Reading ADC for VBATT and 3V3

## USB-C
- [ ] Enabling PD 
- [ ] Flashing

## Pyros
- [ ] Continuity 
- [ ] Blowing

## Storage
- [X] Can R/W to W25Q128

## Buzzer
- [ ] Can drive buzzer

## LEDs
- [ ] Can drive LEDs correctly. PA15 LED turns on when powered without any firmware on them

# Software Testing
## Sensors
- [X] LSM6DSV configured for 16Gs
- [X] LSM6DSV interrupts working
- [X] MS5611 reading pressure and temperature
- [ ] Valid readings from ADC for VBATT and 3V3

## Storage
- [ ] Storing flights works on raw data
- [ ] Can export flights to mass storage
- [ ] Mass storage accessible over USB-C
- [ ] Configuration readable and writable

## Pyros
- [ ] Don't blow things up without continuity
- [ ] Blow things up with continuity

## Buzzer
- [ ] Error tone for continuity failure
- [ ] Error tone for filesystem failure
- [ ] Success tone for armed pyros

## Shell
- [X] Expose capabilities for Dispatch
- [X] Configurable deployments