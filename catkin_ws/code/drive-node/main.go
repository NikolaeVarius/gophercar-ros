package main

import (
	"log"
	"periph.io/x/periph/conn/i2c/i2creg"
	"periph.io/x/periph/experimental/devices/pca9685"
	"periph.io/x/periph/host"
	//	"time"
)

// Steering Angle Parameters
const MAX_LEFT ANGLE = 0
const MAX_RIGHT_ANGLE = 0
const MAX_LEFT_PULSE = 0
const MAX_RIGHT_PURSE = 0


// Throttle Paramters
const STOP_PULSE = 0
const MIN_THROTTLE = 0 // maybe should just use STOP_PULSE?
const MAX_THROTTLE = 0
const MIN_THROTTLE_PULSE = 0
const MAX_THROTTLE_PULSE = 0
const THROTTLE_CHANNEL = 0
const THROTTLE_STEP = 10


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

// Joystick Events Handler
func joystickROSNode() {
        n, err := goroslib.NewNode(goroslib.NodeConf{
                Name:       "/goroslib",
                MasterHost: "donkeycar",
        })
        if err != nil {
                panic(err)
        }
        fmt.Print("Connected to Master")
        defer n.Close()

        // create a subscriber
        subTopic, err := goroslib.NewSubscriber(goroslib.SubscriberConf{
                Node:     n,
                Topic:    "/joy",
                Callback: onMessage,
        })
        if err != nil {
                panic(err)
        }
        fmt.Print("Connected to subscriber topic")
        defer subTopic.Close()


}




// Translate from input to throttle control pwm values
func setThrottle(throttle float64) {
    throttlePWMVal := getThrottlePWMVal()

}

// Translate from intput to direction pwm values
func setSteering(steering float64) {
    steeringPWMVal := getSteeringPWMVal()
}

func getThrottlePWMVal(val float64) {

}

func getSteeringPWMVal(val float64) {
}

