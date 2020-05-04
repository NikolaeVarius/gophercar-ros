package main

import (
	"fmt"
	"github.com/aler9/goroslib"
	//"github.com/aler9/goroslib/msgs"
        "github.com/aler9/goroslib/msgs/sensor_msgs"
)

func onMessage(msg *sensor_msgs.Joy) {
	fmt.Printf("Incoming: %+v\n", msg)
}

func main() {
	// create a node with given name and linked to given master.
	// master can be reached with an ip or hostname.
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
	sub, err := goroslib.NewSubscriber(goroslib.SubscriberConf{
		Node:     n,
		Topic:    "/joy",
		Callback: onMessage,
	})
	if err != nil {
		panic(err)
	}
	fmt.Print("Connected to topic")
	defer sub.Close()

	// freeze main loop
	infty := make(chan int)
	<-infty
}
