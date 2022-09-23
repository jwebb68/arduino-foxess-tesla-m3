SUFFIX: .o .cpp .ino

allsrc:=$(shell find . -name '*.cpp' -o -name '*.cc')
allhdr:=$(shell find . -name '*.h' -o -name '*.hh')

.PHONY: all
all:
	echo "whut?"

.PHONY: format
format:
	astyle --project -I -n  $(allsrc) $(allhdr)
	env -u NIX_CFLAGS_COMPILE clang-format -style=file -i $(allsrc) $(allhdr)
