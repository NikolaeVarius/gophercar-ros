package main

import (
	"fmt"
	"github.com/aler9/goroslib"
	"github.com/aler9/goroslib/msgs"
	"github.com/aler9/goroslib/msgs/geometry_msgs"
	"github.com/aler9/goroslib/msgs/sensor_msgs"
        "gopkg.in/alexcesaro/statsd.v2"
)

//var subTopic *Subscriber
//var pubTopic *Publisher

type Message struct {
	msgs.Package `ros:"my_package"`
	FirstField   msgs.Uint32
	SecondField  msgs.String
}

func onMessage(msg *sensor_msgs.Joy) {
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

	// freeze main loop
	infty := make(chan int)
	<-infty
}


