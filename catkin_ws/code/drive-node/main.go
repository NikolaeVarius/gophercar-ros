package main

import (
	"fmt"
	"github.com/aler9/goroslib"
	"github.com/aler9/goroslib/msgs"
	"github.com/aler9/goroslib/msgs/std_msgs"
	//"gopkg.in/alexcesaro/statsd.v2"
	"github.com/cactus/go-statsd-client/statsd"
	"log"
	"periph.io/x/periph/conn/gpio"
	"periph.io/x/periph/conn/i2c/i2creg"
	"periph.io/x/periph/experimental/devices/pca9685"
	"periph.io/x/periph/host"
	"time"
)

// Steering Angle Parameters
const MAX_LEFT_ANGLE = 0
const MAX_RIGHT_ANGLE = 0
const MAX_LEFT_PULSE = 400
const MAX_RIGHT_PULSE = 200
const NEUTRAL_PULSE = 300
const STEERING_CHANNEL = 1

// Throttle Paramters
const STOP_PULSE = 0
const MIN_THROTTLE = 0 // maybe should just use STOP_PULSE?
const MAX_THROTTLE = 0
const CALIBRATION_THROTTLE_PULSE = 330
const MIN_THROTTLE_PULSE = 0
const MAX_THROTTLE_PULSE = 400
const THROTTLE_CHANNEL = 0
const THROTTLE_STEP = 10

// ROS Paramaters
const ROS_MASTER = "donkeycar"
const ROS_NODE_NAME = ""
const ACTUATOR_TOPIC = "/actuator"

var pca *pca9685.Dev
var sc statsd.Statter

// Other
const STATSD_HOST = "161.35.109.219"

func init() {
        // statsd
	var err error
        config := &statsd.ClientConfig{
                Address:       "161.35.109.219:8125",
                Prefix:        "drive-node",
                UseBuffered:   true,
                FlushInterval: 300 * time.Millisecond,
        }
        sc, err = statsd.NewClientWithConfig(config)
        if err != nil {
                log.Fatal(err)
        }

}

func main() {
	_, err := host.Init()
	if err != nil {
		log.Fatal(err)
	}
	// defer closing of statsd client`
        defer sc.Close()
	// According to pinout of Jetson Nano, this is the i2c bus
	bus, err := i2creg.Open("1")
	if err != nil {
		log.Fatal(err)
	}

	pca, err = pca9685.NewI2C(bus, pca9685.I2CAddr)
	if err != nil {
		log.Fatal(err)
	}

	// PWM Freq
	//if err := pca.SetPwmFreq(60); err != nil {
	//       log.Fatal(err)
	//}

	// Channel 0 = Throttle
	// Initialize PWM throttle.
	// When first started, the ESC needs to be calibrated with this pulse
	// This should cause a blinking red light on the ESC to stop blinking, and a single beep to occur
	fmt.Println("Initializing ESC")
	if err := pca.SetPwm(0, 0, CALIBRATION_THROTTLE_PULSE); err != nil {
		log.Fatal(err)
	}

	// Channel 1 = Steering
	// Initialize PWM Steering at a Neutral Value
	fmt.Println("Initializing Steering")
	if err := pca.SetPwm(STEERING_CHANNEL, 0, NEUTRAL_PULSE); err != nil {
		log.Fatal(err)
	}

	actuatorMessages := make(chan *std_msgs.Float64MultiArray, 100)

	n, err := goroslib.NewNode(goroslib.NodeConf{
		Name:       "/gophercar-actuator",
		MasterHost: ROS_MASTER,
	})
	if err != nil {
		panic(err)
	}
	fmt.Println("Connected to Master: " + ROS_MASTER)
	defer n.Close()

	// Subscribe to actuator topic to process joy/keypresses
	subTopic, err := goroslib.NewSubscriber(goroslib.SubscriberConf{
		Node:  n,
		Topic: ACTUATOR_TOPIC,
		Callback: func(msg *std_msgs.Float64MultiArray) {
			actuatorMessages <- msg
		},
	})
	if err != nil {
		panic(err)
	}
	fmt.Println("Subscribed to: " + ACTUATOR_TOPIC)
	defer subTopic.Close()

	// publisher topic to publish current ESC values to a topic
	escThrottleTopic, err := goroslib.NewPublisher(goroslib.PublisherConf{
		Node:  n,
		Topic: "/pwm-throttle",
		Msg:   &std_msgs.Int64{},
		Latch: false,
	})
	if err != nil {
		panic(err)
	}
	fmt.Println("Connected to PWM-Throttle Publisher Topic")
	defer escThrottleTopic.Close()

	escSteeringTopic, err := goroslib.NewPublisher(goroslib.PublisherConf{
		Node:  n,
		Topic: "/pwm-steering",
		Msg:   &std_msgs.Int64{},
		Latch: false,
	})
	if err != nil {
		panic(err)
	}
	fmt.Println("Connected to PWM-Steering Publisher Topic")
	defer escSteeringTopic.Close()

	// Start doing stuff
	ticker := time.NewTicker(100)
	go func() {
		for x := range actuatorMessages {
			select {
			case <-ticker.C:
				fmt.Printf("Recieved: %+v\n", x)
	       		        scErr := sc.Inc("drive_node_recieved", 42, 0.0)
		                if scErr!= nil {
                                        panic(scErr)
                                }

				steeringPwm, steerErr := setSteering(x.Data[0])
				_ = sc.Gauge("steering_pwm", int64(steeringPwm), 0.0)
                                if steerErr != nil {
                                        panic(steerErr)
                                }

				throttlePwm, throttleErr := setThrottle(x.Data[3])
				_ = sc.Gauge("throttle_pwm", int64(throttlePwm), 0.0)
				if throttleErr != nil {
					panic(throttleErr)
				}
			}
		}

	}()

	infty := make(chan int)
	<-infty
}

func convertStampedTwistedToAngle() {
	return
}

// Translate from input to throttle control pwm values
func setThrottle(throttle msgs.Float64) (int, error) {
	throttlePWMVal := getThrottlePWMVal(throttle)
	fmt.Printf("Set Throttle PWM: %v\n", throttlePWMVal)
	if err := pca.SetPwm(0, 0, gpio.Duty(throttlePWMVal)); err != nil {
		return 0, err
	}
	return throttlePWMVal, nil
}

// Translate from intput to direction pwm values
func setSteering(steering msgs.Float64) (int, error) {
	val := getSteeringPWMVal(steering)
	fmt.Printf("Set Steering PWM: %v\n", val)
	if err := pca.SetPwm(1, 0, gpio.Duty(val)); err != nil {
		return 0, err
	}
	return val, nil
}

func getThrottlePWMVal(val msgs.Float64) int {
	var pwmVal int
	if val < 0 {
		pwmVal = MIN_THROTTLE_PULSE
	}

	if val > 0 {
		pwmVal = MAX_THROTTLE_PULSE
	}

	if val == 0 {
		pwmVal = STOP_PULSE
	}

	return pwmVal

}

func getSteeringPWMVal(val msgs.Float64) int {
	var pwmVal int
	if val < 0 {
		//	fmt.Println("go right")
		pwmVal = MAX_RIGHT_PULSE
	}

	if val > 0 {
		//	fmt.Println("go left")
		pwmVal = MAX_LEFT_PULSE
	}

	if val == 0 {
		//	fmt.Println("stay straight")
		pwmVal = NEUTRAL_PULSE
	}

	return pwmVal
}

// copy pasta to try it out some time in the future
func Every(t time.Duration, f func()) *time.Ticker {
	ticker := time.NewTicker(t)

	go func() {
		for {
			select {
			case <-ticker.C:
				f()
			}
		}
	}()

	return ticker
}
