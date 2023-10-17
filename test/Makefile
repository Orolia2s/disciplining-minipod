SourceFiles != find . -name '*.c'
Binaries    := $(SourceFiles:%.c=%)

Dependency  := oscillator-disciplining.pc

CFLAGS   += $(shell pkg-config --cflags-only-other $(Dependency))
CPPFLAGS += $(shell pkg-config --cflags-only-I $(Dependency))
LDFLAGS  += $(shell pkg-config --libs-only-L $(Dependency))
LDLIBS   += $(shell pkg-config --libs-only-l $(Dependency))

CPPFLAGS += -I "$(shell pwd)/../src"


all: $(Binaries) ## Compile all binaries

run: $(Binaries) ## Run all tests
	for binary in $^; \
	do ./$${binary}; \
	done

clean: ## Remove generated binaries
	$(RM) $(Binaries)
