package main

import (
	"log"

	"periph.io/x/periph/conn/i2c/i2creg"
	"periph.io/x/periph/experimental/devices/pca9685"
	"periph.io/x/periph/host"
)

func main() {
	_, err := host.Init()
	if err != nil {
		log.Fatal(err)
	}
	// According to pinout of Jetson Nano, this is the i2c bus
	bus, err := i2creg.Open("1")
	if err != nil {
		log.Fatal(err)
	}

	pca, err := pca9685.NewI2C(bus, pca9685.I2CAddr)
	if err != nil {
		log.Fatal(err)
	}

	if err := pca.SetPwmFreq(100); err != nil {
		log.Fatal(err)
	}
	if err := pca.SetAllPwm(200, 300); err != nil {
		log.Fatal(err)
	}
}
