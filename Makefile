# Builds the shield against the two submodule cores.
# Requires only g++ with C++20 support; run `git submodule update --init` first.
CXX      ?= g++
CXXFLAGS ?= -std=c++20 -O2 -Wall -Wextra -Wpedantic
CPPFLAGS  = -Ideps/planning/core/include -Ideps/verifier/core/include -Ideps/verifier/core/eval

BUILD := build

.PHONY: all eval clean

all: $(BUILD)/shield

$(BUILD):
	mkdir -p $(BUILD)

$(BUILD)/shield: src/shield_main.cpp deps/planning/core/src/astar.cpp deps/verifier/core/src/verifier.cpp | $(BUILD)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $^ -o $@

eval: $(BUILD)/shield
	mkdir -p reports/results
	./$(BUILD)/shield --out reports/results/shield_eval.csv

clean:
	rm -rf $(BUILD)
