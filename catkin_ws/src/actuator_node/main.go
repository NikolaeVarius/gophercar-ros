package main

import (
	"fmt"
	"log"
	"math"
	"time"

	"github.com/aler9/goroslib"
	"github.com/aler9/goroslib/msgs/sensor_msgs"
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

// const rosActuatorTopic = "/joy"
// const rosSteeringTopic = "/pwm-steering"
// const rosThrottleTopic = "/pwm-throttle"
const rosActuatorTopic = "/joy"
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

	// Register self as a node
	n, err := goroslib.NewNode(goroslib.NodeConf{
		Name:       "/gophercar-actuator",
		MasterHost: rosMasterNode,
	})
	if err != nil {
		panic(err)
	}
	fmt.Println("Connected to ROS Master: " + rosMasterNode)
	defer n.Close()

	actuatorMessages := make(chan *sensor_msgs.Joy, 100)
	// actuatorMessages := make(chan *std_msgs.Float64MultiArray, 100)

	// Subscribe to actuator topic to process joy/keypresses
	subTopic, err := goroslib.NewSubscriber(goroslib.SubscriberConf{Node: n,
		Topic: rosActuatorTopic,
		// Callback: func(msg *std_msgs.Float64MultiArray) {
		// 	actuatorMessages <- msg
		// },
		Callback: func(msg *sensor_msgs.Joy) {
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
	//
	// escThrottleTopic, err := goroslib.NewPublisher(goroslib.PublisherConf{
	// 	Node:  n,
	// 	Topic: rosSteeringTopic,
	// 	Msg:   &std_msgs.Int64{},
	// 	Latch: false,
	// })
	// if err != nil {
	// 	panic(err)
	// }
	// fmt.Println("Connected to PWM-Throttle Publisher Topic")
	// defer escThrottleTopic.Close()

	// escSteeringTopic, err := goroslib.NewPublisher(goroslib.PublisherConf{
	// 	Node:  n,
	// 	Topic: rosThrottleTopic,
	// 	Msg:   &std_msgs.Int64{},
	// 	Latch: false,
	// })
	// if err != nil {
	// 	panic(err)
	// }
	// fmt.Println("Connected to PWM-Steering Publisher Topic")
	// defer escSteeringTopic.Close()

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
// func handleActuatorMessage(msg *std_msgs.Float64MultiArray) error {
func handleActuatorMessage(msg *sensor_msgs.Joy) error {

	stdMsg := covertJoyToStdMessage(msg)

	// angularValue := msg.Data[0]
	// throttleValue := msg.Data[3]
	angularValue := stdMsg.Data[0]
	throttleValue := stdMsg.Data[3]

	// fmt.Printf("Recieved: %+v\n", msg)
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

	if val == 0 {
		return 0
	}

	scaledPwmVal := normalize(float64(val), float64(minThrottlePulse), float64(maxThrottlePulse))
	fmt.Printf("Throttle PWM value `%f scaled to %f\n", val, scaledPwmVal)
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
	fmt.Printf("Steering PWM value %f scaled to %f\n", val, scaledPwmVal)
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

func covertJoyToStdMessage(msg *sensor_msgs.Joy) std_msgs.Float64MultiArray {
	var newMsg std_msgs.Float64MultiArray
	// 4 probably isn't the right size
	data := make([]float64, 14)

	// fmt.Println("Incoming: %#v\n", msg)

	// Joysticks
	leftJoyX := float64(msg.Axes[0])
	leftJoyY := float64(msg.Axes[1])
	rightJoyX := float64(msg.Axes[2])
	rightJoyY := float64(msg.Axes[5])

	// Direction Buttons
	xDirBtn := float64(msg.Axes[6])
	if xDirBtn == 1 {
		fmt.Println("Left Pressed")
	}

	if xDirBtn == -1 {
		fmt.Println("Right Pressed")
	}

	yDirBtn := float64(msg.Axes[7])
	if yDirBtn == 1 {
		fmt.Println("Up Pressed")
	}

	if yDirBtn == -1 {
		fmt.Println("Down Pressed")
	}

	// Other Buttons
	leftJoyBtn := float64(msg.Buttons[10])
	if leftJoyBtn == 1 {
		fmt.Println("Left Joy Pressed")
	}

	rightJoyBtn := float64(msg.Buttons[11])
	if rightJoyBtn == 1 {
		fmt.Println("Right Joy Pressed")
	}

	// Face buttons - Ps4 Controller based
	squareBtn := float64(msg.Buttons[0])
	if squareBtn == 1 {
		fmt.Println("Square Pressed")
	}

	crossBtn := float64(msg.Buttons[1])
	if crossBtn == 1 {
		fmt.Println("Cross Pressed")
	}

	circleBtn := float64(msg.Buttons[2])
	if circleBtn == 1 {
		fmt.Println("Circle Pressed")
	}

	triangleBtn := float64(msg.Buttons[3])
	if triangleBtn == 1 {
		fmt.Println("Triangle Pressed")
	}

	leftBumperBtn := float64(msg.Buttons[4])
	if leftBumperBtn == 1 {
		fmt.Println("Left Bumper Pressed")
	}

	rightBumperBtn := float64(msg.Buttons[5])
	if rightBumperBtn == 1 {
		fmt.Println("Right Bumper Pressed")
	}

	leftTriggerBtn := float64(msg.Buttons[6])
	if leftTriggerBtn == 1 {
		fmt.Println("Left Trigger Pressed")
	}

	rightTriggerBtn := float64(msg.Buttons[7])
	if rightTriggerBtn == 1 {
		fmt.Println("Right Trigger Pressed")
	}

	optionsBtn := float64(msg.Buttons[8])
	if optionsBtn == 1 {
		fmt.Println("Option Pressed")
	}

	startBtn := float64(msg.Buttons[9])
	if startBtn == 1 {
		fmt.Println("Start Pressed")
	}

	centerBtn := float64(msg.Buttons[12]) // PS Logo
	if centerBtn == 1 {
		fmt.Println("Center Pressed")
	}

	centerBtnAlt := float64(msg.Buttons[13]) // Trackpad
	if centerBtnAlt == 1 {
		fmt.Println("Center Alt. Pressed")
	}

	//==================================
	data[0] = leftJoyX
	data[1] = leftJoyY
	data[2] = rightJoyX
	data[3] = rightJoyY

	data[4] = squareBtn
	data[5] = rightJoyY
	data[6] = leftJoyBtn
	data[7] = rightJoyBtn
	data[8] = crossBtn
	data[9] = circleBtn
	data[10] = triangleBtn
	data[11] = leftBumperBtn

	newMsg.Data = data

	return newMsg
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
