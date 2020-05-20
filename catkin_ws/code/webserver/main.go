// Based off https://github.com/hybridgroup/gocv/blob/master/cmd/mjpeg-streamer/main.go

// How to run:
//
// mjpeg-streamer [camera ID] [host:port]
//
//		go get -u github.com/hybridgroup/mjpeg
// 		go run ./cmd/mjpeg-streamer/main.go 0.0.0.0:8080
package main

import (
	"fmt"
	"os"
	"runtime"

	"net/http"
	_ "net/http/pprof"

	"github.com/aler9/goroslib"
	"github.com/aler9/goroslib/msgs/sensor_msgs"
	"github.com/hybridgroup/mjpeg"
	"gocv.io/x/gocv"
)

var (
	deviceID int
	err      error
	stream   *mjpeg.Stream
)

const ROS_MASTER = "donkeycar"

func main() {
	if len(os.Args) < 2 {
		fmt.Println("How to run:\n\tmjpeg-streamer [camera ID] [host:port]")
		return
	}

	// parse args
	host := os.Args[1]
	fmt.Println(host)

	compressedImageMessages := make(chan *sensor_msgs.CompressedImage, 100)

	n, err := goroslib.NewNode(goroslib.NodeConf{
		Name:       "/webserver",
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
		Topic: "/output/image_raw/compressed",
		Callback: func(msg *sensor_msgs.CompressedImage) {
			compressedImageMessages <- msg
		},
	})
	if err != nil {
		fmt.Println("Most likely the topic this subscriber wants to attach to does not exist")
		panic(err)
	}

	fmt.Println("Subscribed to: " + "/output/image_raw/compressed")
	defer subTopic.Close()
	// create the mjpeg stream
	stream := mjpeg.NewStream()

	// start capturing
	go mjpegCapture(compressedImageMessages)
	// start http server
	http.Handle("/", stream)

	go func() {
		http.ListenAndServe(host, nil)
	}()

	// fmt.Println("Capturing. Point your browser to " + host)
	runtime.Goexit()

}

func mjpegCapture(ch chan *sensor_msgs.CompressedImage) {
	// ticker := time.NewTicker(100)
	for x := range ch {

		buf, _ := gocv.IMDecode(x.Data)
		stream.UpdateJPEG(buf)
	}
}
