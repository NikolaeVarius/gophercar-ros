package main

import (
	"fmt"

	"github.com/aler9/goroslib"
	"github.com/aler9/goroslib/msgs/sensor_msgs"
	"gocv.io/x/gocv"
)

var subTopic *goroslib.Subscriber
var window = gocv.NewWindow("Face Detect")

func main() {
	xmlFile := "./haarcascade_frontalface_default.xml"
	imgStream := make(chan *sensor_msgs.CompressedImage, 100)
	defer window.Close()

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
		Topic: "/output/image_raw/compressed",
		Callback: func(msg *sensor_msgs.CompressedImage) {
			imgStream <- msg
		},
	})
	if err != nil {
		panic(err)
	}
	fmt.Println("Connected to Subscriber Topic")
	defer subTopic.Close()

	go func() {
		for img := range imgStream {
			fmt.Println("moar")
			fmt.Println(img)
			// showWindow(img)

		}
	}()
	infty := make(chan int)
	<-infty
	// for {

	// 	if img.Empty() {
	// 		continue
	// 	}

	// 	rects := classifier.DetectMultiScale(img)
	// 	fmt.Printf("found %d faces\n", len(rects))

	// 	for _, r := range rects {
	// 		gocv.Rectangle(&img, r, blue, 3)

	// 		size := gocv.GetTextSize("Human", gocv.FontHersheyPlain, 1.2, 2)
	// 		pt := image.Pt(r.Min.X+(r.Min.X/2)-(size.X/2), r.Min.Y-2)
	// 		gocv.PutText(&img, "Human", pt, gocv.FontHersheyPlain, 1.2, blue, 2)
	// 	}

	// }
}

func showWindow(img *sensor_msgs.CompressedImage) {
	imgData := img.Data
	fmt.Println("Processing Frame")

	imgBytes := make([]byte, len(imgData))
	for i := 0; i < len(imgData); i++ {
		imgBytes[i] = uint8(imgData[i])
	}

	imgMat, err := gocv.NewMatFromBytes(480, 640, gocv.MatTypeCV8UC3, imgBytes)
	fmt.Println(imgBytes)
	if err != nil {
		fmt.Println(err)
	}
	if imgMat.Empty() {
		return
	}

	window.IMShow(imgMat)
	window.WaitKey(1)

	// color for the rect when faces detected
	// blue := color.RGBA{0, 0, 255, 0}
	// fmt.Println(img)
	fmt.Println("done")
	return
}
