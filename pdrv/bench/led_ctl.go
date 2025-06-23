package main

import (
	"encoding/binary"
	"log"
	"math"
	"os/exec"
	"time"

	"github.com/caszuu/PicoPu/pdrv"
)

var ledStrips = [][][2]float32{
	{ // mobo
		{-6.35, 24.765},
		{-3.81, 24.765},
		{-1.27, 24.765},
		{1.27, 24.765},
		{3.81, 24.765},
		{6.35, 24.765},
		{-29.21, 17.145},
		{-27.94, -17.145},
		{-30.48, -17.145},
		{27.94, -17.145},
		{30.48, -17.145},
		{29.21, 17.145},
	},
	{{-30.734, 21.59}, {-27.686, 21.59}, {-24.638, 21.59}, {-21.59, 21.59}, {-18.542, 21.59}},                                                                       // shader 1
	{{-30.734, -12.7}, {-27.686, -12.7}, {-24.638, -12.7}, {-21.59, -12.7}, {-18.542, -12.7}},                                                                       // shader 2
	{{-30.734, -46.989999999999995}, {-27.686, -46.989999999999995}, {-24.638, -46.989999999999995}, {-21.59, -46.989999999999995}, {-18.542, -46.989999999999995}}, // shader 3
	{{30.734, -21.59}, {27.686, -21.59}, {24.638, -21.59}, {21.59, -21.59}, {18.542, -21.59}},                                                                       // shader 4
	{{30.734, 12.7}, {27.686, 12.7}, {24.638, 12.7}, {21.59, 12.7}, {18.542, 12.7}},                                                                                 // shader 5
	{{30.734, 46.989999999999995}, {27.686, 46.989999999999995}, {24.638, 46.989999999999995}, {21.59, 46.989999999999995}, {18.542, 46.989999999999995}},           // shader 6
}

func shadeRainbowLed(pos [2]float32, time float32, brightness float64) (uint8, uint8, uint8) {
	uv := [2]float32{pos[0] / 35 * 2, pos[1] / 25 * 2}
	uv = [2]float32{uv[0]*.5 + .5, uv[1]*.5 + .5}

	col := [3]float64{math.Cos(float64(time + uv[0] + 0)), math.Cos(float64(time + uv[1] + 4)), math.Cos(float64(time + uv[0] + 2))}
	col = [3]float64{col[0]*.5 + .5, col[1]*.5 + .5, col[2]*.5 + .5}

	return uint8(col[0] * 255 * brightness), uint8(col[1] * 255 * brightness), uint8(col[2] * 255 * brightness)
}

func patternRainbow(dev *pdrv.Device, brightness float32) {
	timeVal := float32(0)

	for {
		// calc led states

		ledValues := make([][]byte, 0, 7)

		for _, strip := range ledStrips {
			stripValues := make([]byte, 0, len(strip))

			for _, pos := range strip {
				r, g, b := shadeRainbowLed(pos, timeVal, float64(brightness))
				stripValues, _ = binary.Append(stripValues, binary.LittleEndian, (uint32(r)<<16)|(uint32(g)<<24)|(uint32(b)<<8))
			}

			ledValues = append(ledValues, stripValues)
		}

		// update leds

		for i, vals := range ledValues {
			err := dev.CtlLedUpdate(vals, uint16(i))

			if err != nil {
				log.Println("failed to update leds:", err)
			}
		}

		time.Sleep(time.Millisecond * 16)
		timeVal += 1. / 60
	}
}

func patternReactive(dev *pdrv.Device, brightness float32) {
	c := exec.Command("cava", "-p", "cava_config")

	reader, err := c.StdoutPipe()
	if err != nil {
		panic(err)
	}

	err = c.Start()
	if err != nil {
		panic(err)
	}

	fftBuf := make([]byte, 3)
	timeVal := float32(0)

	for {
		read := 0
		for read < 3 {
			c, err := reader.Read(fftBuf[read:])
			if err != nil {
				panic(err)
			}

			read += c
		}

		// calc led states

		ledValues := make([][]byte, 0, 7)

		for _, strip := range ledStrips {
			stripValues := make([]byte, 0, len(strip))

			for _, pos := range strip {
				r, g, b := shadeRainbowLed(pos, timeVal, float64(brightness))
				r = uint8((float32(r) / 255 * float32(fftBuf[0]) / 255) * 255)
				g = uint8((float32(g) / 255 * float32(fftBuf[1]) / 255) * 255)
				b = uint8((float32(b) / 255 * float32(fftBuf[2]) / 255) * 255)

				stripValues, _ = binary.Append(stripValues, binary.LittleEndian, (uint32(r)<<16)|(uint32(g)<<24)|(uint32(b)<<8))
			}

			ledValues = append(ledValues, stripValues)
		}

		// update leds

		for i, vals := range ledValues {
			err := dev.CtlLedUpdate(vals, uint16(i))

			if err != nil {
				log.Println("failed to update leds:", err)
			}
		}

		timeVal += 1. / 90
	}
}
