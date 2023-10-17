Dependency := oscillator-disciplining.pc

CFLAGS   += $(shell pkg-config --cflags-only-other $(Dependency))
CPPFLAGS += $(shell pkg-config --cflags-only-I $(Dependency))
LDFLAGS  += $(shell pkg-config --libs-only-L $(Dependency))
LDLIBS   += $(shell pkg-config --libs-only-l $(Dependency))

all: test-checks test-minipod-lib test-utils test_fine_circular_buffer
