package main

import (
	"fmt"
	"github.com/notedit/gst"
)

func main() {


	pipelineStr := "nvarguscamerasrc sensor_id=0 ! nvvidconv flip-method=2 ! nvoverlaysink"
	//pipelineStr = fmt.Sprintf(pipelineStr, 100)

	err := gst.CheckPlugins([]string{"x264","rtp","videoparsersbad"})

	if err != nil {
		fmt.Println(err)
	}

	pipeline, err := gst.ParseLaunch(pipelineStr)

	if err != nil {
		panic(err)
	}

	element := pipeline.GetByName("appsink")

	pipeline.SetState(gst.StatePlaying)

	for {

		sample, err := element.PullSample()
		if err != nil {
			if element.IsEOS() == true {
				fmt.Println("eos")
				return
			} else {
				fmt.Println(err)
				continue
			}
		}
		fmt.Println("got sample", sample.Duration)
	}

	pipeline.SetState(gst.StateNull)

	pipeline = nil
	element = nil
}
