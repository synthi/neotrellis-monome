# neotrellis monome grid with teensy 3.2

## hex files

The hex files here are setup with the following addresses:

```
// 16x8 
Adafruit_NeoTrellis trellis_array[NUM_ROWS / 4][NUM_COLS / 4] = {
  { Adafruit_NeoTrellis(0x32), Adafruit_NeoTrellis(0x30), Adafruit_NeoTrellis(0x2F), Adafruit_NeoTrellis(0x2E)}, // top row
  { Adafruit_NeoTrellis(0x33), Adafruit_NeoTrellis(0x31), Adafruit_NeoTrellis(0x3E), Adafruit_NeoTrellis(0x36) } // bottom row
};
```

```
// 8x8 setup
Adafruit_NeoTrellis trellis_array[NUM_ROWS / 4][NUM_COLS / 4] = {
	{ Adafruit_NeoTrellis(0x2F), Adafruit_NeoTrellis(0x2E) },
	{ Adafruit_NeoTrellis(0x3E), Adafruit_NeoTrellis(0x36) }
};
```

These are in order right to left as you look at the BACK of the grid. So from the front/grid side, these will be flipped.

* see this graphic for a default layout of addresses and jumper positions for 8 neotrellis boards (16x8 layout). For an 8x8 just use the left half.  

![neotrellis_addresses.jpg](../neotrellis_addresses.jpg)

## firmware flashing - Teensy

### Load pre-compiled firmware w/ TyUploader

Get the firmware "hex" file from directory in this repo (download the entire repo ZIP file - not just individual files).

Get TyTools [from GitHub here](https://github.com/Koromix/tytools/releases). More info here (https://koromix.dev/tytools).

Install TyUploader and open it. Be sure your neotrellis-grid is plugged in. The grid/Teensy should show up in the TyUploader application... or select Teensy or Teensyduino from the pull down menu if needed.

<img src="images/tyuploader.png" alt="tyupdater" width="319" height="190" />

Click the Upload button and select the firmware hex file you want to upload. This should upload the firmware and the neotrellis-grid should reboot. That's it.

### If you want to copile the firmware yourself to make changes, etc...

In Arduino/Teensyduino - be sure you have the settings in  `Tools -> USB Type` set to `Serial`

Not critical, but set `Tools -> CPU Speed` to `120 MHz (overclock)`

For reference: [here's a forum post on how to flash Teensy firmware](https://llllllll.co/t/how-to-flash-the-firmware-on-a-teensy-micro-controller/20317)