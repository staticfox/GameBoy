emu: emu.cc
	clang++ emu.cc -Wall -Wextra -O0 -g -std=c++14 -o emu

clean:
	rm -r emu
