package main

import (
	"encoding/binary"
	"flag"
	"log"
	"os"

	"github.com/caszuu/PicoPu/pdrv"
	"github.com/google/gousb"
)

var (
	si_mode  = flag.Bool("si", false, "Run pdrv in shader-interface mode.")
	failsafe = flag.Bool("failsafe", false, "Do not return a non-zero code on some failures.")

	flash = flag.Bool("flash", false, "Reboot device into flashing mode.")
)

func main() {
	flag.Parse()

	ctx := gousb.NewContext()
	defer ctx.Close()

	dev, err := pdrv.InitDevice(ctx)
	if err != nil {
		if !*failsafe {
			log.Fatalln("failed initializing device:", err)
		} else {
			os.Exit(0)
		}
	}
	defer dev.Destroy()

	if *flash && *si_mode {
		buf := make([]byte, 1024)
		p := pdrv.SiFlash{Ptype: pdrv.SiTypeFlash}

		binary.Encode(buf, binary.LittleEndian, p)

		err := dev.QueueOutXfer(buf)
		if err != nil && !*failsafe {
			log.Fatalln("failed switching mode:", err)
		}
	}
}
