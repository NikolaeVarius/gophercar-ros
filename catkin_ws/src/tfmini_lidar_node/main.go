package main

import (
	"fmt"
	"log"

	"go.bug.st/serial"
)

var port serial.Port

func init() {
	mode := &serial.Mode{
		BaudRate: 115200,
	}

	var err error

	port, err = serial.Open("/dev/ttyTHS1", mode)
	if err != nil {
		log.Fatal(err)
	}
}

func main() {

	buff := make([]byte, 100)
	for {
		n, err := port.Read(buff)
		if err != nil {
			log.Fatal(err)
			break
		}
		if n == 0 {
			fmt.Println("\nEOF")
			break
		}
		fmt.Printf("%v", string(buff[:n]))
	}
}
