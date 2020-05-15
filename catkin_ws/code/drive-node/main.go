package main

import (
	"log"
	"fmt"
	"periph.io/x/periph/conn/i2c/i2creg"
	"periph.io/x/periph/experimental/devices/pca9685"
	"periph.io/x/periph/host"

        "github.com/aler9/goroslib"
       // "github.com/aler9/goroslib/msgs"
        "github.com/aler9/goroslib/msgs/geometry_msgs"
       // "github.com/aler9/goroslib/msgs/sensor_msgs"

	//	"time"
)

// Steering Angle Parameters
const MAX_LEFT_ANGLE = 0
const MAX_RIGHT_ANGLE = 0
const MAX_LEFT_PULSE = 0
const MAX_RIGHT_PURSE = 0
const MAX_LEFT_PULSE = 400
const MAX_RIGHT_PULSE = 200
const NEUTRAL_PULSE = 300


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
	
        // Channel 0 = Throttle
	// Initialize PWM throttle at a STOPPED Value
        if err := pca.SetPwm(0, 0, 300); err != nil {
              log.Fatal(err)
        }

	// Channel 1 = Steering
	// Initialize PWM Steering at a Neutral Value
        if err := pca.SetPwm(1, 0, 300); err != nil {
              log.Fatal(err)
        }

	// https://github.com/google/periph/blob/master/conn/gpio/gpio.go read this
	// 400 turns on gogo juice. this is dangerous when on a makeshift test stand
	// Placeholder for testing throttle

	//if err := pca.SetPwm(0, 0, 300); err != nil {
	//        log.Fatal(err)
	//}
        actuatorMessages := make(chan *geometry_msgs.TwistStamped, 100)

        n, err := goroslib.NewNode(goroslib.NodeConf{
                Name:       "/gophercar-actuator",
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
                Topic:    "/actuator",
                Callback: func(msg  *geometry_msgs.TwistStamped) {
                        actuatorMessages <- msg
                },
	})
        if err != nil {
                panic(err)
        }
        fmt.Println("Connected to subscriber topic")
        defer subTopic.Close()

        go func() {
                for x := range actuatorMessages {
		  fmt.Println(x.Header)
		  fmt.Println(x.Twist.Linear.X)
		  fmt.Println(x.Twist.Linear.Y)
		  fmt.Println(x.Twist.Linear.Z)
		  fmt.Println(x.Twist.Angular)
                }

        }()

        infty := make(chan int)
        <-infty
}

func convertStampedTwistedToAngle() {
    return
}


// Translate from input to throttle control pwm values
func setThrottle(throttle float64) error {
    throttlePWMVal := getThrottlePWMVal(throttle)
    fmt.Println(throttlePWMVal)
    return nil
}

// Translate from intput to direction pwm values
func setSteering(steering float64) error {
    steeringPWMVal := getSteeringPWMVal(steering)
    fmt.Println(steeringPWMVal)
    return nil
}

func getThrottlePWMVal(val float64) float64 {
    return 0
}

func getSteeringPWMVal(val float64) float64 {
    return 0
}
