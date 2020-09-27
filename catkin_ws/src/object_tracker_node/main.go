package main

import (
	"fmt"
	"image"
	"image/color"
	"os"

	"gocv.io/x/gocv"
	"gocv.io/x/gocv/contrib"
)

func getMiddleCoordinates(rect image.Rectangle) (x int, y int) {
	return (rect.Max.X-rect.Min.X)/2 + rect.Min.X, (rect.Max.Y-rect.Min.Y)/2 + rect.Min.Y
}

func getBoundingCoordinates(rect image.Rectangle) (x1 int, y1 int, x2 int, y2 int) {
	return rect.Min.X, rect.Min.Y, rect.Max.Y, rect.Max.Y
}

func main() {
	if len(os.Args) < 2 {
		fmt.Println("How to run:\n\ttracking [camera ID]")
		return
	}

	// parse args
	deviceID := os.Args[1]

	// open webcam
	webcam, err := gocv.OpenVideoCapture(1)
	if err != nil {
		fmt.Printf("Error opening video capture device: %v\n", deviceID)
		return
	}
	defer webcam.Close()

	// open display window
	window := gocv.NewWindow("Tracking")
	defer window.Close()

	// create a tracker instance
	// (one of MIL, KCF, TLD, MedianFlow, Boosting, MOSSE or CSRT)
	tracker := contrib.NewTrackerMOSSE()
	defer tracker.Close()

	// prepare image matrix
	img := gocv.NewMat()
	defer img.Close()

	// read an initial image
	if ok := webcam.Read(&img); !ok {
		fmt.Printf("cannot read device %v\n", deviceID)
		return
	}

	// let the user mark a ROI to track
	rect := gocv.SelectROI("Tracking", img)
	if rect.Max.X == 0 {
		fmt.Printf("user cancelled roi selection\n")
		return
	}

	// initialize the tracker with the image & the selected roi
	init := tracker.Init(img, rect)
	if !init {
		fmt.Printf("Could not initialize the Tracker")
		return
	}

	// color for the rect to draw
	blue := color.RGBA{0, 0, 255, 0}
	fmt.Printf("Start reading device: %v\n", deviceID)
	for {
		if ok := webcam.Read(&img); !ok {
			fmt.Printf("Device closed: %v\n", deviceID)
		}
		if img.Empty() {
			continue
		}

		// update the roi
		rect, _ := tracker.Update(img)

		coordX, _ := getMiddleCoordinates(rect)
		x1, y1, x2, y2 := getBoundingCoordinates(rect)

		fmt.Println("x1: " + string(x1))
		fmt.Println("y1: " + string(y1))
		fmt.Println("x2: " + string(x2))
		fmt.Println("y2: " + string(y2))
		fmt.Println("Middle X: " + string(coordX))

		// draw it.
		gocv.Rectangle(&img, rect, blue, 3)

		// show the image in the window, and wait 10 millisecond
		window.IMShow(img)
		if window.WaitKey(10) >= 0 {
			break
		}
	}
}
