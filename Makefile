# Builds the shield against the two submodule cores.
# Requires only g++ with C++20 support; run `git submodule update --init` first.
CXX      ?= g++
CXXFLAGS ?= -std=c++20 -O2 -Wall -Wextra -Wpedantic
CPPFLAGS  = -Ideps/planning/core/include -Ideps/verifier/core/include -Ideps/verifier/core/eval

BUILD := build
CORES := deps/planning/core/src/astar.cpp deps/verifier/core/src/verifier.cpp

.PHONY: all eval test clean

all: $(BUILD)/shield

$(BUILD):
	mkdir -p $(BUILD)

$(BUILD)/shield: src/shield_main.cpp $(CORES) | $(BUILD)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $^ -o $@

# test_shield.cpp includes shield_main.cpp, so the latter is a prerequisite
# but not a source on the command line.
$(BUILD)/test_shield: tests/test_shield.cpp src/shield_main.cpp $(CORES) | $(BUILD)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) tests/test_shield.cpp $(CORES) -o $@

test: $(BUILD)/test_shield
	./$(BUILD)/test_shield

eval: $(BUILD)/shield
	mkdir -p reports/results
	./$(BUILD)/shield --out reports/results/shield_eval.csv

clean:
	rm -rf $(BUILD)
