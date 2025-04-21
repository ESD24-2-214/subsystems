##
# control
#
# @file
# @version 0.1

CXXFLAGS += -I$(shell nix eval --raw nixpkgs#stdenv.cc.cc)/include


compile_c_r:
	clang++ c_r.cpp math_stuff.cpp -o c_r.bin -std=c++20 -g3 \
	-Wall -Wextra -Wconversion -Wdouble-promotion -Wno-unused-parameter \
	-Wno-unused-function -Wno-sign-conversion -fsanitize=undefined -fsanitize-trap -pedantic
	./c_r.bin

compile_control:
	clang++ control.cpp math_stuff.cpp -o control.bin -std=c++20 -g3 \
	-Wall -Wextra -Wconversion -Wdouble-promotion -Wno-unused-parameter \
	-Wno-unused-function -Wno-sign-conversion -fsanitize=undefined -fsanitize-trap -pedantic
	./control.bin

compile_ref:
	clang++ reference_frame.cpp math_stuff.cpp -o frame.bin -std=c++20 -g3 \
	-Wall -Wextra -Wconversion -Wdouble-promotion -Wno-unused-parameter \
	-Wno-unused-function -Wno-sign-conversion -fsanitize=undefined -fsanitize-trap -pedantic
	./frame.bin

debug:
	clang++ c_r.cpp math_stuff.cpp -o c_r.bin -std=c++20 -g3 \
	-Wall -Wextra -Wconversion -Wdouble-promotion -Wno-unused-parameter \
	-Wno-unused-function -Wno-sign-conversion -fsanitize=undefined -fsanitize-trap \
	-pedantic -DDEBUG
	./c_r.bin


test:
	clang++ c_r.cpp math_stuff.cpp -o c_r.bin -std=c++20 -g3 \
	-Wall -Wextra -Wconversion -Wdouble-promotion -Wno-unused-parameter \
	-Wno-unused-function -Wno-sign-conversion -fsanitize=undefined -fsanitize-trap -pedantic \
	-DDEBUG -DTEST
	./c_r.bin

setup:
	rm compile_commands.json
	bear -- make compile_dev

plot: compile_c_r
	Rscript plot.R

# end
