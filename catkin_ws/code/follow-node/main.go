package main

import (
	"fmt"

	"github.com/aler9/goroslib"
	"github.com/aler9/goroslib/msgs/sensor_msgs"
	"gocv.io/x/gocv"
)

var subTopic *goroslib.Subscriber

func main() {
	xmlFile := "./haarcascade_frontalface_default.xml"
	imgStream := make(chan *sensor_msgs.CompressedImage, 100)

	// window := gocv.NewWindow("Face Detect")
	// defer window.Close()

	// img := gocv.NewMat()
	// defer img.Close()

	// color for the rect when faces detected
	// blue := color.RGBA{0, 0, 255, 0}

	// load classifier to recognize faces
	classifier := gocv.NewCascadeClassifier()
	defer classifier.Close()

	if !classifier.Load(xmlFile) {
		fmt.Printf("Error reading cascade file: %v\n", xmlFile)
		return
	}

	n, err := goroslib.NewNode(goroslib.NodeConf{
		Name:       "/gocv",
		MasterHost: "donkeycar",
	})
	if err != nil {
		panic(err)
	}
	fmt.Println("Connected to Master")
	defer n.Close()

	// Apparently can't subscribe directly to compressed image topic
	subTopic, err = goroslib.NewSubscriber(goroslib.SubscriberConf{
		Node:  n,
		Topic: "/output/image_raw",
		Callback: func(msg *sensor_msgs.CompressedImage) {
			imgStream <- msg
		},
	})
	if err != nil {
		panic(err)
	}
	fmt.Println("Connected to Subscriber Topic")
	defer subTopic.Close()

	// go func() {
	// 	for img := range imgStream {
	// 		fmt.Println(len(img.Data))
	// 	}

	// }()
	infty := make(chan int)
	<-infty
		for {

			if img.Empty() {
				continue
			}

			rects := classifier.DetectMultiScale(img)
			fmt.Printf("found %d faces\n", len(rects))

			for _, r := range rects {
				gocv.Rectangle(&img, r, blue, 3)

				size := gocv.GetTextSize("Human", gocv.FontHersheyPlain, 1.2, 2)
				pt := image.Pt(r.Min.X+(r.Min.X/2)-(size.X/2), r.Min.Y-2)
				gocv.PutText(&img, "Human", pt, gocv.FontHersheyPlain, 1.2, blue, 2)
			}

			window.IMShow(img)
			if window.WaitKey(20) >= 0 {
				break
			}
		}
	}
}
