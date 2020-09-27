package main

import (
	"fmt"
	"image"
	"image/color"
	"math"
	"os"
	"strconv"

	"github.com/aler9/goroslib/msgs/geometry_msgs"
	"gocv.io/x/gocv"
	"gocv.io/x/gocv/contrib"
)

func getMiddleCoordinates(rect image.Rectangle) (x int, y int) {
	return (rect.Max.X-rect.Min.X)/2 + rect.Min.X, (rect.Max.Y-rect.Min.Y)/2 + rect.Min.Y
}

func getBoundingCoordinates(rect image.Rectangle) (x1 int, y1 int, x2 int, y2 int) {
	return rect.Min.X, rect.Min.Y, rect.Max.Y, rect.Max.Y
}

func getRectDistances(rect image.Rectangle) (dist float64) {
	return math.Sqrt(math.Pow(float64(rect.Max.X-rect.Min.X), 2) + math.Pow(float64(rect.Max.Y-rect.Min.Y), 2))
}

func main() {
	if len(os.Args) < 2 {
		fmt.Println("How to run:\n\ttracking [camera ID]")
		return
	}

	velocity := geometry_msgs.Twist{
		Linear: geometry_msgs.Vector3{
			X: 0.0,
			Y: 0.0,
			Z: 0.0,
		},
		Angular: geometry_msgs.Vector3{
			X: 0.0,
			Y: 0.0,
			Z: 0.0,
		},
	}
	fmt.Println(velocity)
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
		dist := getRectDistances(rect)

		fmt.Println("x1: " + strconv.Itoa(x1))
		fmt.Println("y1: " + strconv.Itoa(y1))
		fmt.Println("x2: " + strconv.Itoa(x2))
		fmt.Println("y2: " + strconv.Itoa(y2))
		fmt.Println("Middle X: " + strconv.Itoa(coordX))
		fmt.Println("Dist: " + strconv.FormatFloat(dist, 'f', 6, 64))

		// draw it.
		gocv.Rectangle(&img, rect, blue, 3)

		// show the image in the window, and wait 10 millisecond
		window.IMShow(img)
		if window.WaitKey(10) >= 0 {
			break
		}
	}
}
