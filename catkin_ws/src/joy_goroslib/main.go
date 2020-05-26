package main

import (
	"fmt"

	"github.com/aler9/goroslib"
	"github.com/aler9/goroslib/msgs"
	"github.com/aler9/goroslib/msgs/std_msgs"

	//	"github.com/aler9/goroslib/msgs/geometry_msgs"
	"log"
	"time"

	"github.com/aler9/goroslib/msgs/sensor_msgs"
	"github.com/cactus/go-statsd-client/statsd"
)

var subTopic *goroslib.Subscriber
var pubTopic *goroslib.Publisher
var sc statsd.Statter

func covertJoyToStdMessage(msg *sensor_msgs.Joy) std_msgs.Float64MultiArray {
	var newMsg std_msgs.Float64MultiArray
	// 4 probably isn't the right size
	data := make([]msgs.Float64, 14)
	// Joy sticks
	fmt.Println("Incoming: %#v\n", msg)
	leftJoyX := msgs.Float64(float64(msg.Axes[0]))
	leftJoyY := msgs.Float64(float64(msg.Axes[1]))
	rightJoyX := msgs.Float64(float64(msg.Axes[2]))
	rightJoyY := msgs.Float64(float64(msg.Buttons[5]))
	leftJoyBtn := msgs.Float64(float64(msg.Axes[10]))
	rightJoyBtn := msgs.Float64(float64(msg.Axes[11]))

	// Face buttons - Ps4 Controller based
	squareBtn := msgs.Float64(float64(msg.Axes[0]))
	crossBtn := msgs.Float64(float64(msg.Axes[1]))
	circleBtn := msgs.Float64(float64(msg.Axes[2]))
	triangleBtn := msgs.Float64(float64(msg.Axes[3]))
	leftBumperBtn := msgs.Float64(float64(msg.Axes[4]))
	leftTriggerBtn := msgs.Float64(float64(msg.Axes[5]))
	rightBumperBtn := msgs.Float64(float64(msg.Axes[6]))
	rightTriggerBtn := msgs.Float64(float64(msg.Axes[7]))
	optionsBtn := msgs.Float64(float64(msg.Axes[8]))
	startBtn := msgs.Float64(float64(msg.Axes[9]))
	centerBtn := msgs.Float64(float64(msg.Axes[12]))    // PS Logo
	centerBtnAlt := msgs.Float64(float64(msg.Axes[13])) // Trackpad

	//	cross_btn := ""
	//	circle_btn := ""
	//	tirangle_btn := ""

	//==================================
	data[0] = leftJoyX
	data[1] = leftJoyY
	data[2] = rightJoyX
	data[3] = rightJoyY
	data[4] = squareBtn
	data[5] = rightJoyY
	// data[6] =
	// data[7] =
	// data[8] =
	// data[9] =
	// data[10] =
	// data[11] =
	// data[12] =
	// data[13] =

	newMsg.Data = data

	return newMsg
}

const STATSD_HOST = "161.35.109.219"

func init() {
	// statsd
	var err error
	config := &statsd.ClientConfig{
		Address:       "161.35.109.219:8125",
		Prefix:        "joy-node",
		UseBuffered:   true,
		FlushInterval: 300 * time.Millisecond,
	}
	sc, err = statsd.NewClientWithConfig(config)
	if err != nil {
		log.Fatal(err)
	}
}

func main() {
	joyMessages := make(chan *sensor_msgs.Joy, 100)
	defer sc.Close()
	n, err := goroslib.NewNode(goroslib.NodeConf{
		Name:       "/goroslib",
		MasterHost: "donkeycar",
	})
	if err != nil {
		panic(err)
	}
	fmt.Println("Connected to Master")
	defer n.Close()

	subTopic, err = goroslib.NewSubscriber(goroslib.SubscriberConf{
		Node:  n,
		Topic: "/joy",
		Callback: func(msg *sensor_msgs.Joy) {
			joyMessages <- msg
		},
	})
	if err != nil {
		panic(err)
	}
	fmt.Println("Connected to Subscriber Topic")

	defer subTopic.Close()

	pubTopic, err = goroslib.NewPublisher(goroslib.PublisherConf{
		Node:  n,
		Topic: "/actuator",
		Msg:   &std_msgs.Float64MultiArray{},
		Latch: false,
	})
	if err != nil {
		panic(err)
	}
	fmt.Println("Connected to Publisher Topic")
	defer pubTopic.Close()

	go func() {
		for x := range joyMessages {
			sc.Inc("joy.publish.counter", 1, 1.0)
			msg := covertJoyToStdMessage(x)
			fmt.Println("Outgoing: %+v\n", msg)
			publishMessage(msg)
		}

	}()

	infty := make(chan int)
	<-infty
}

func publishMessage(msg std_msgs.Float64MultiArray) {
	pubTopic.Write(&msg)
	return
}
