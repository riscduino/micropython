Note:
  1. Current openocd script assume ch347 usb to jtag driver. If you are using different one the change the riscduino_d32a1.cfg accordingly
  2. To run the openocd
      openocd -s openocd/  -f openocd/riscduino_d32a1.cfg -c "init" -c targets -c riscduino-halt -c targets -c 'riscduino-load build/bes32r20xx.elf' -c 'riscduino-reset' -c resume -c targets -d2
