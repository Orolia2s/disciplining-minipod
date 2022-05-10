C= gcc
CPPFLAGS= -Wall -Werror -g -O0 -std=gnu99 -I../src
LDLIBS= -lm -loscillator-disciplining

# First test is just to test utils functions
EXE= test-utils
OBJS= test-utils.o utils.o log.o

# Second test is used to test the library interface
EXE2 = test-minipod-lib
OBJS2 = test-minipod-lib.o

EXE3 = test-checks
OBJS3 = test-checks.o checks.o log.o

EXE4 = test-fine-circular-buffer
OBJS4 = test_fine_circular_buffer.o fine_circular_buffer.o log.o

all: test-minipod test-utils-test test-checks test-checks-test

test-minipod: test-minipod-lib
	LD_LIBRARY_PATH=/usr/lib:/usr/lib64:/usr/local/lib:/usr/local/lib64:/usr/local/lib/x86_64-linux-gnu ./test-minipod-lib

test-utils-test: $(EXE)
	./test-utils

test-checks-test: $(EXE3)
	./test-checks

test-fine-circular-buffer-test: $(EXE4)
	./test-fine-circular-buffer

$(EXE): $(OBJS)
	$(C)  $^ $(LOADLIBES) $(LDLIBS) -o $@

$(EXE2): $(OBJS2)
	$(C)  $^ $(LOADLIBES) $(LDLIBS) -o $@

$(EXE3): $(OBJS3)
	$(C)  $^ $(LOADLIBES) $(LDLIBS) -o $@

$(EXE4): $(OBJS4)
	$(C)  $^ $(LOADLIBES) $(LDLIBS) -o $@

test-utils.o : test-utils.c
	$(C) $(CPPFLAGS) -c $^ -o $@

utils.o : ../src/utils.c
	$(C) $(CPPFLAGS) -c $^ -o $@

log.o : ../src/log.c
	$(C) $(CPPFLAGS) -c $^ -o $@

checks.o: ../src/checks.c
	$(C) $(CPPFLAGS) -c $^ -o $@

fine_circular_buffer.o: ../src/fine_circular_buffer.c
	$(C) $(CPPFLAGS) -c $^ -o $@

clean:
	rm -f *.o
	rm -f test-utils
	rm -f test-minipod-lib
	rm -rf test-checks
	rm -rf test-fine-circular-buffer
