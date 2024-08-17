# Broker-sim

A light implementation of a broker "chip" runnable on a linux pc that can perform and manage simple GCS commands over usb serial. It can manage a single shader chip and perform test draws over a usb connection.

It currently used for development of the `shader_core/` firmware and can be used with a stock Raspberry Pico board. It's expected that this implementation will fall into an unmaintained state after the development of the broker chip firmware starts. (which will only happen after the first PicoPU test board will be made)

# Usage

To compile, just pass it directly into your `cc` compiler as it's completely self contained. (apart from system headers)
```sh
gcc -o broker_sim broker_driver.c
```

Then after connecting the Pico and making sure it's accessible from `/dev/ttyACM0`, launch the binary to perform the test draw
```sh
./broker_sim
```

The test draw will generate two images in the current working directory of the state of the framebuffer (color and depth images) after the draw.
