package main

import (
	"fmt"
//	"time"
	"github.com/aler9/goroslib"
	"github.com/aler9/goroslib/msgs"
	"github.com/aler9/goroslib/msgs/geometry_msgs"
	"github.com/aler9/goroslib/msgs/sensor_msgs"
//        "gopkg.in/alexcesaro/statsd.v2"
)

var subTopic *goroslib.Subscriber
var pubTopic *goroslib.Publisher

//type Message struct {
//	msgs.Package `ros:"my_package"`
//	LinearX   msgs.float64
//	LinearY   msgs.float64
//}

func covertJoyToTwistStamped(msg *sensor_msgs.Joy) geometry_msgs.TwistStamped  {
	fmt.Println("Incoming: %+v\n", msg)
	x_float64 := msgs.Float64(float64(msg.Axes[0]))
	y_float64 := msgs.Float64(float64(msg.Axes[1]))
	linearVector := geometry_msgs.Vector3{X: x_float64, Y: y_float64}
	rawMove := geometry_msgs.Twist{Linear: linearVector, Angular: linearVector}
	stampedMove := geometry_msgs.TwistStamped{Header: msg.Header, Twist: rawMove}
	return stampedMove
}

func main() {

	//stampedTwistMessages := make(chan *geometry_msgs.TwistStamped, 100)
	joyMessages := make(chan *sensor_msgs.Joy, 100)

	n, err := goroslib.NewNode(goroslib.NodeConf{
		Name:       "/goroslib",
		MasterHost: "donkeycar",
	})
	if err != nil {
		panic(err)
	}
	fmt.Println("Connected to Master")
	defer n.Close()
    
	// create a subscriber
	subTopic, err = goroslib.NewSubscriber(goroslib.SubscriberConf{
		Node:     n,
		Topic:    "/joy",
		Callback: func(msg *sensor_msgs.Joy) {
	          //fmt.Println("here")
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
        for x := range joyMessages  {
            publishMessage(covertJoyToTwistStamped(x))
        }
    }()

	infty := make(chan int)
	<-infty
}

func publishMessage(msg geometry_msgs.TwistStamped) {
    fmt.Println("Publishing")
    pubTopic.Write(&msg)
    fmt.Println("Done")
    return
}
