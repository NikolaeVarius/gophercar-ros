package main

import (
	"fmt"
	"log"
	"math"
	"time"

	"github.com/aler9/goroslib"
	"github.com/aler9/goroslib/msgs/std_msgs"
	"github.com/cactus/go-statsd-client/statsd"
	"periph.io/x/periph/conn/gpio"
	"periph.io/x/periph/conn/i2c/i2creg"
	"periph.io/x/periph/experimental/devices/pca9685"
	"periph.io/x/periph/host"
)

// Steering Angle Parameters
const maxLeftAngle = 0
const maxRightAngle = 0
const maxLeftPulse = 400
const maxRightPulse = 200
const neutralPulse = 300
const steeringChannel = 1

// Throttle Paramters
const stopPulse = 0
const minThrottle = 0 // maybe should just use STOP_PULSE?
const maxThrottle = 0
const calibrationThrottlePulse = 330
const minThrottlePulse = 0
const maxThrottlePulse = 400
const throttleChannel = 0
const throttleStep = 10

// Emergency Stop
// If this is ever set to true, car throttle should be set to 0
var emergencyStop = false

// ROS Paramaters
const rosMasterNode = "donkeycar"
const rosNodeName = "/actuator"
const rosActuatorTopic = "/actuator"
const rosSteeringTopic = "/pwm-steering"
const rosThrottleTopic = "/pwm-throttle"

var pca *pca9685.Dev
var sc statsd.Statter

// Other
const statsdHost = "161.35.109.219"
const statsdPort = "8125"

var statsdURL = statsdHost + ":" + statsdPort

func init() {
	// statsd
	config := &statsd.ClientConfig{
		Address:       statsdURL,
		Prefix:        rosNodeName,
		UseBuffered:   true,
		FlushInterval: 300 * time.Millisecond,
	}
	sc, _ = statsd.NewClientWithConfig(config)
	defer sc.Close()
	// if err != nil {
	// 	log.Fatal(err)
	// }

	_, err := host.Init()
	if err != nil {
		log.Fatal(err)
	}

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
	if err := pca.SetPwm(0, 0, calibrationThrottlePulse); err != nil {
		log.Fatal(err)
	}

	// Channel 1 = Steering
	// Initialize PWM Steering at a Neutral Value
	fmt.Println("Initializing Steering")
	if err := pca.SetPwm(steeringChannel, 0, neutralPulse); err != nil {
		log.Fatal(err)
	}

}

func main() {
	n, err := goroslib.NewNode(goroslib.NodeConf{
		Name:       "/gophercar-actuator",
		MasterHost: rosMasterNode,
	})
	if err != nil {
		panic(err)
	}
	fmt.Println("Connected to Master: " + rosMasterNode)
	defer n.Close()

	actuatorMessages := make(chan *std_msgs.Float64MultiArray, 100)

	// Subscribe to actuator topic to process joy/keypresses
	subTopic, err := goroslib.NewSubscriber(goroslib.SubscriberConf{Node: n,
		Topic: rosActuatorTopic,
		Callback: func(msg *std_msgs.Float64MultiArray) {
			actuatorMessages <- msg
		},
	})
	if err != nil {
		fmt.Println("Most likely the topic this subscriber wants to attach to does not exist")
		panic(err)
	}
	fmt.Println("Subscribed to: " + rosActuatorTopic)
	defer subTopic.Close()

	// publisher topic to publish current ESC values to a topic for reporting purposes
	escThrottleTopic, err := goroslib.NewPublisher(goroslib.PublisherConf{
		Node:  n,
		Topic: rosSteeringTopic,
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
		Topic: rosThrottleTopic,
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
	fmt.Println("Main Loop Started")
	go func() {
		for msg := range actuatorMessages {
			select {
			case <-ticker.C:
				carErr := handleActuatorMessage(msg)
				if carErr != nil {
					// This doesn't do anything right now
					// This is a global variable that is checked every actuator cycle
					emergencyStop = true
					fmt.Println("Problem while driving, initiating Emergency Stop: " + carErr.Error())
				}
			}
		}

	}()

	infty := make(chan int)
	<-infty
}

// Handle Joy Twist (Linear/Angular)
func handleJoyTwistMessage() {
	return
}

// Keyboard control
func handleKeyboardMessage() {
	return
}

// Need to refator this to be less dumb
func handleActuatorMessage(msg *std_msgs.Float64MultiArray) error {
	angularValue := msg.Data[0]
	throttleValue := msg.Data[3]

	fmt.Printf("Recieved: %+v\n", msg)
	scErr := sc.Inc("drive_node_recieved", 42, 0.0)
	if scErr != nil {
		return scErr
	}

	steeringPwm, steerErr := setSteering(angularValue)
	_ = sc.Gauge("steering_pwm", int64(steeringPwm), 0.0)
	if steerErr != nil {
		return steerErr
	}

	throttlePwm, throttleErr := setThrottle(throttleValue)
	_ = sc.Gauge("throttle_pwm", int64(throttlePwm), 0.0)
	if throttleErr != nil {
		return throttleErr
	}

	return nil
}

func joyButtonHandler(*std_msgs.Float64MultiArray) {
	return
}

func convertStampedTwistedToAngle() {
	return
}

// Translate from /joy messages to throttle control pwm values
func setThrottle(throttle float64) (int, error) {
	throttlePWMVal := getThrottlePWMVal(throttle)
	fmt.Printf("Set Throttle PWM: %v\n", throttlePWMVal)
	if err := pca.SetPwm(0, 0, gpio.Duty(throttlePWMVal)); err != nil {
		return 0, err
	}
	return throttlePWMVal, nil
}

// Translate from /joy messages to direction pwm values
func setSteering(steering float64) (int, error) {
	val := getSteeringPWMVal(steering)
	fmt.Printf("Set Steering PWM: %v\n", val)
	if err := pca.SetPwm(1, 0, gpio.Duty(val)); err != nil {
		return 0, err
	}
	return val, nil
}

//  Turns a -1/1 Joy value into a scaled PWM value
func getThrottlePWMVal(val float64) int {
	var pwmVal int
	//var scaledPwmVal int

	scaledPwmVal := normalize(float64(val), float64(minThrottlePulse), float64(maxThrottlePulse))
	fmt.Printf("%f converted to %f\n", val, scaledPwmVal)
	// TODO; Fix this up so that going backward is a thing. Not 100% what that would look like
	if val < 0 {
		pwmVal = minThrottlePulse
	}

	if val > 0 {
		pwmVal = int(scaledPwmVal)
	}

	if val == 0 {
		pwmVal = stopPulse
	}

	return pwmVal

}

func getSteeringPWMVal(val float64) int {
	var pwmVal int
	scaledPwmVal := normalize(float64(val), float64(maxRightPulse), float64(maxLeftPulse))
	fmt.Printf("%f convfrted to %f\n", val, scaledPwmVal)
	if val < 0 {
		//	fmt.Println("go right")
		pwmVal = int(scaledPwmVal)
	}

	if val > 0 {
		//	fmt.Println("go left")
		pwmVal = int(scaledPwmVal)
	}

	if val == 0 {
		//	fmt.Println("stay straight")
		pwmVal = neutralPulse
	}

	return pwmVal
}

// converts -1/1 values to equivalent pwm values depending on scale
func normalize(input, min, max float64) float64 {
	i := input*(math.Max(min, max)-math.Min(min, max)) + math.Min(min, max)
	if i < math.Min(min, max) {
		return math.Min(min, max)
	} else if i > math.Max(min, max) {
		return math.Max(min, max)
	} else {
		return i
	}
}

// Handle Button Presses Sent via a Joy Message
func handleJoyBtn(btnIndex float64) {
	return
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
