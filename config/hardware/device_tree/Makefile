# Build I2C7 100kHz device tree overlay
# Requires: device-tree-compiler (sudo apt-get install device-tree-compiler)
# Run from repo root: make -C config/hardware/device_tree

.PHONY: all clean

all: i2c7-100khz.dtbo

i2c7-100khz.dtbo: i2c7-100khz.dts
	dtc -O dtb -o $@ -b 0 -@ $<
	@echo "Built $@"

clean:
	rm -f i2c7-100khz.dtbo
