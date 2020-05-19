// Based off https://github.com/hybridgroup/gocv/blob/master/cmd/mjpeg-streamer/main.go

// How to run:
//
// mjpeg-streamer [camera ID] [host:port]
//
//		go get -u github.com/hybridgroup/mjpeg
// 		go run ./cmd/mjpeg-streamer/main.go 1 0.0.0.0:8080
//
// +build example

package main

import (
	"fmt"
	"log"
	"net/http"
	"os"
	"time"

	_ "net/http/pprof"

	"github.com/aler9/goroslib"
	"github.com/aler9/goroslib/msgs/std_msgs"
	"github.com/hybridgroup/mjpeg"
	"gocv.io/x/gocv"
)

var (
	deviceID int
	err      error
	webcam   *gocv.VideoCapture
	stream   *mjpeg.Stream
)

const ROS_MASTER = "donkeycar"

func main() {
	if len(os.Args) < 3 {
		fmt.Println("How to run:\n\tmjpeg-streamer [camera ID] [host:port]")
		return
	}

	// parse args
	deviceID := os.Args[1]
	host := os.Args[2]

	// create the mjpeg stream
	stream = mjpeg.NewStream()

	compressedImageMessages := make(chan *std_msgs.CompressedImage, 100)

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
		Callback: func(msg *std_msgs.CompressedImage) {
			compressedImageMessages <- msg
		},
	})
	if err != nil {
		fmt.Println("Most likely the topic this subscriber wants to attach to does not exist")
		panic(err)
	}

	fmt.Println("Subscribed to: " + "/output/image_raw/compressed")
	defer subTopic.Close()

	// start capturing
	go mjpegCapture()

	fmt.Println("Capturing. Point your browser to " + host)

	// Start doing stuff
	ticker := time.NewTicker(100)
	go func() {
		for x := range compressedImageMessages {
			select {
			case <-ticker.C:
				return
			}
		}

	}()

	// start http server
	http.Handle("/", stream)
	log.Fatal(http.ListenAndServe(host, nil))
}

func mjpegCapture() {
	img := gocv.NewMat()
	defer img.Close()

	for {
		if ok := webcam.Read(&img); !ok {
			fmt.Printf("Device closed: %v\n", deviceID)
			return
		}
		if img.Empty() {
			continue
		}

		buf, _ := gocv.IMEncode(".jpg", img)
		stream.UpdateJPEG(buf)
	}
}
