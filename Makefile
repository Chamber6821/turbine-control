
PORT ?= /dev/ttyUSB0
FQBN ?= arduino:avr:uno

upload: ./turbine-control.ino
	arduino-cli compile --fqbn $(FQBN)
	arduino-cli upload -p $(PORT) --fqbn $(FQBN)

