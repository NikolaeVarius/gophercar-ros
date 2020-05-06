package main

import (
	"log"
	"periph.io/x/periph/conn/i2c/i2creg"
	"periph.io/x/periph/experimental/devices/pca9685"
	"periph.io/x/periph/host"
	//	"time"
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
	// Channel 1 = Steering
	if err := pca.SetPwm(1, 0, 300); err != nil {
		log.Fatal(err)
	}

	// https://github.com/google/periph/blob/master/conn/gpio/gpio.go read this
	// 400 turns on gogo juice. this is dangerous when on a makeshift test stand
	// Placeholder for testing throttle

	//if err := pca.SetPwm(0, 0, 300); err != nil {
	//        log.Fatal(err)
	//}

}
