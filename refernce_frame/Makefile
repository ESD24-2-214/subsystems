##
# control
#
# @file
# @version 0.1

CXXFLAGS += -I$(shell nix eval --raw nixpkgs#stdenv.cc.cc)/include

compile_dev:
	clang++ reference_frame.cpp math_stuff.cpp -o control -std=c++20 -g3 \
	-Wall -Wextra -Wconversion -Wdouble-promotion -Wno-unused-parameter \
	-Wno-unused-function -Wno-sign-conversion -fsanitize=undefined -fsanitize-trap
	./control

debug:
	clang++ reference_frame.cpp math_stuff.cpp -o control -std=c++20 -g3 \
	-Wall -Wextra -Wconversion -Wdouble-promotion -Wno-unused-parameter \
	-Wno-unused-function -Wno-sign-conversion -fsanitize=undefined -fsanitize-trap \
	-pedantic -DDEBUG
	./control


test:
	clang++ reference_frame.cpp math_stuff.cpp -o control -std=c++20 -g3 \
	-Wall -Wextra -Wconversion -Wdouble-promotion -Wno-unused-parameter \
	-Wno-unused-function -Wno-sign-conversion -fsanitize=undefined -fsanitize-trap -pedantic \
	-DDEBUG -DTEST
	./control

setup:
	rm compile_commands.json
	bear -- make compile_dev

# end
