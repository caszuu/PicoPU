package main

import (
	"flag"
	"log"
	"time"

	"github.com/caszuu/PicoPu/pdrv"
	"github.com/google/gousb"
)

var (
	failsafe = flag.Bool("failsafe", false, "Do not return a non-zero code on some failures.")
	flash    = flag.Bool("flash", false, "Reboot device into flashing mode.")
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
			return
		}
	}
	defer dev.Destroy()

	if *flash {
		var err error

		err = dev.CtlFlash()
		if err != nil {
			if !*failsafe {
				log.Fatalln("failed switching mode:", err)
			} else {
				return
			}
		}

		time.Sleep(time.Millisecond * 750)
	}
}
