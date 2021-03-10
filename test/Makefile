C= gcc
CPPFLAGS= -Wall -Werror -g -O0 -std=gnu99 -I../src
LDLIBS= -lm

EXE= test
OBJS= main.o utils.o

$(EXE): $(OBJS)
	$(C)  $^ $(LOADLIBES) $(LDLIBS) -o $@

main.o : main.c
	$(C) $(CPPFLAGS) -c $^ -o $@

utils.o : ../src/utils.c
	$(C) $(CPPFLAGS) -c $^ -o $@


clean:
	rm -f *.o
	rm -f test
