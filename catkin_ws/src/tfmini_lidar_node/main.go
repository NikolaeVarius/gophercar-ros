package main

import (
	"fmt"
	"log"
	"time"

	"go.bug.st/serial"
)

var port serial.Port

func init() {
	mode := &serial.Mode{
		BaudRate: 115200,
		DataBits: 8,
		StopBits: serial.OneStopBit,
		Parity:   serial.SpaceParity,
	}

	var err error

	port, err = serial.Open("/dev/ttyTHS1", mode)
	if err != nil {
		log.Fatal(err)
	}
}

func main() {

	buff := make([]byte, 9)
	for {

		time.Sleep(100)

		println("loop")
		n, err := port.Read(buff)
		if err != nil {
			log.Fatal(err)
			break
		}
		if n == 0 {
			fmt.Println("\nEOF")
			break
		}
		fmt.Print(n)
		// fmt.Printf("%v", string(buff[:n]))
		fmt.Printf("%v\n", buff[0])
		fmt.Printf("%v\n", buff[1])
		port.ResetInputBuffer()
	}
}
