emu: emu.cc
	clang++ emu.cc -Wall -Wextra -std=c++14 -o emu

clean:
	rm -r emu
