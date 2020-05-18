package main

import (
	"fmt"
	"github.com/aler9/goroslib"
	"github.com/aler9/goroslib/msgs"
	"github.com/aler9/goroslib/msgs/std_msgs"		
//	"github.com/aler9/goroslib/msgs/geometry_msgs"
	"github.com/aler9/goroslib/msgs/sensor_msgs"
        "github.com/cactus/go-statsd-client/statsd"
	"time"
	"log"
)

var subTopic *goroslib.Subscriber
var pubTopic *goroslib.Publisher
var sc statsd.Statter

func covertJoyToStdMessage(msg *sensor_msgs.Joy) std_msgs.Float64MultiArray {
	var newMsg std_msgs.Float64MultiArray
	// 4 probably isn't the right size
	data := make([]msgs.Float64, 4)
	// Joy sticks
	fmt.Println("Incoming: %#v\n", msg)
	leftJoyX := msgs.Float64(float64(msg.Axes[0]))
	leftJoyY := msgs.Float64(float64(msg.Axes[1]))
	rightJoyX := msgs.Float64(float64(msg.Axes[2]))
	
	rightJoyY := msgs.Float64(float64(msg.Axes[5]))

	// Face buttons
//        square_btn := ""
//	cross_btn := ""
//	circle_btn := ""
//	tirangle_btn := ""



	//==================================
	data[0] = leftJoyX
        data[1] = leftJoyY
	data[2] = rightJoyX
	data[3] = rightJoyY

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
