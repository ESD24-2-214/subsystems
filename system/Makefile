##
# Rotaing Donut
#
# @file
# @version 0.1

all: run monitor

clean:

build:
	pio run

run:
	pio run -t upload -e release

run_debug:
	pio run -t upload -e debug

run_debug_serial:
	pio run -t upload -e serial
	pio device monitor --raw

run_debug_control:
	pio run -t upload -e debug_control
	pio device monitor --raw

run_debug_pwm:
	pio run -t upload -e debug_pwm
	pio device monitor --raw

monitor:
	pio device monitor --raw

emacs:
	pio run -t compiledb

# end
