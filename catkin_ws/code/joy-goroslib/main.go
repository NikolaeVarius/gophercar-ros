package main

import (
	"fmt"
	"github.com/aler9/goroslib"
	"github.com/aler9/goroslib/msgs"
	"github.com/aler9/goroslib/msgs/geometry_msgs"
	"github.com/aler9/goroslib/msgs/sensor_msgs"
)

//var subTopic *Subscriber
//var pubTopic *Publisher

type Message struct {
	msgs.Package `ros:"my_package"`
	FirstField   msgs.Uint32
	SecondField  msgs.String
}

func onMessage(msg *sensor_msgs.Joy) {
	fmt.Printf("Incoming: %+v\n", msg)
	x_float64 := msgs.Float64(float64(msg.Axes[0]))
	y_float64 := msgs.Float64(float64(msg.Axes[1]))
	linearVector := geometry_msgs.Vector3{X: x_float64, Y: y_float64}
	rawMove := geometry_msgs.Twist{Linear: linearVector, Angular: linearVector}
	stampedMove := geometry_msgs.TwistStamped{Header: msg.Header, Twist: rawMove}
	fmt.Printf("Outgoing: %+v\n", stampedMove)
	//fmt.Printf(msg.Header)
	//fmt.Printf(msg.Axes)
	//fmt.Printf(msg.Buttons)
}

func main() {
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

	msg := &Message{
		FirstField:  3,
		SecondField: "test message",
	}
	pubTopic, err := goroslib.NewPublisher(goroslib.PublisherConf{
		Node:  n,
		Topic: "/joy2",
		Msg:   msg,
		Latch: false,
	})
	if err != nil {
		panic(err)
	}
	fmt.Print("Connected to publisher topic")
	defer pubTopic.Close()

	// freeze main loop
	infty := make(chan int)
	<-infty
}

//func publish(msg *sensor_msgs.Joy) {
//
//}
