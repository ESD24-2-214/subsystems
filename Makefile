##
# Rotaing Donut
#
# @file
# @version 0.1

all: build run monitor

clean:

build:
	# When building include all files
	pio run

run:
	pio run -t upload

monitor:
	pio device monitor --raw

# end
