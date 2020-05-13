package main

import (
	"fmt"
	"github.com/aler9/goroslib"
	"github.com/aler9/goroslib/msgs"
	"github.com/aler9/goroslib/msgs/geometry_msgs"
	"github.com/aler9/goroslib/msgs/sensor_msgs"
        "gopkg.in/alexcesaro/statsd.v2"
)

var subTopic *goroslib.Subscriber
var pubTopic *goroslib.Publisher

func covertJoyToTwistStamped(msg *sensor_msgs.Joy) geometry_msgs.TwistStamped {
	fmt.Println("Incoming: %+v\n", msg)
	x_float64 := msgs.Float64(float64(msg.Axes[0]))
	y_float64 := msgs.Float64(float64(msg.Axes[1]))
	linearVector := geometry_msgs.Vector3{X: x_float64, Y: y_float64}
	rawMove := geometry_msgs.Twist{Linear: linearVector, Angular: linearVector}
	stampedMove := geometry_msgs.TwistStamped{Header: msg.Header, Twist: rawMove}
	return stampedMove
}

func main() {
	joyMessages := make(chan *sensor_msgs.Joy, 100)

	c, err := statsd.New() // Connect to the UDP port 8125 by default.
	if err != nil {
	  fmt.Println(err)
	}
	defer c.Close()


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
		Msg:   &geometry_msgs.TwistStamped{},
		Latch: false,
	})
	if err != nil {
		panic(err)
	}
	fmt.Println("Connected to Publisher Topic")
	defer pubTopic.Close()

	go func() {
		for x := range joyMessages {
			publishMessage(covertJoyToTwistStamped(x))
		}
	}()

	infty := make(chan int)
	<-infty
}

func publishMessage(msg geometry_msgs.TwistStamped) {
	//c.Gauge("msg", msg)
	pubTopic.Write(&msg)
	return
}
