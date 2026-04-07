CXX ?= g++
CXXFLAGS ?= -std=c++17 -O2 -Wall -Wextra -pedantic
INCLUDES := -Iinclude -I.

TARGET := hexapod-physics-sim
CORE_SRC := src/minphys3d/core/world.cpp
SRC := src/main.cpp $(CORE_SRC)
UNIT_TEST := tests/test_unit_math
REGRESSION_TEST := tests/test_regression_step
BROADPHASE_TEST := tests/test_broadphase_dynamic_tree
SCENE_SUITE := tests/regression_scene_suite
TEST_SRC_COMMON := $(CORE_SRC)

all: $(TARGET)

$(TARGET): $(SRC)
	$(CXX) $(CXXFLAGS) $(INCLUDES) $(SRC) -o $(TARGET)

$(UNIT_TEST): tests/test_unit_math.cpp $(TEST_SRC_COMMON)
	$(CXX) $(CXXFLAGS) $(INCLUDES) tests/test_unit_math.cpp $(TEST_SRC_COMMON) -o $(UNIT_TEST)

$(REGRESSION_TEST): tests/test_regression_step.cpp $(TEST_SRC_COMMON)
	$(CXX) $(CXXFLAGS) $(INCLUDES) tests/test_regression_step.cpp $(TEST_SRC_COMMON) -o $(REGRESSION_TEST)

$(BROADPHASE_TEST): tests/test_broadphase_dynamic_tree.cpp $(TEST_SRC_COMMON)
	$(CXX) $(CXXFLAGS) $(INCLUDES) tests/test_broadphase_dynamic_tree.cpp $(TEST_SRC_COMMON) -o $(BROADPHASE_TEST)

$(SCENE_SUITE): tests/regression_scene_suite.cpp $(TEST_SRC_COMMON)
	$(CXX) $(CXXFLAGS) $(INCLUDES) tests/regression_scene_suite.cpp $(TEST_SRC_COMMON) -o $(SCENE_SUITE)

test: $(UNIT_TEST) $(REGRESSION_TEST) $(BROADPHASE_TEST) $(SCENE_SUITE)
	./$(UNIT_TEST)
	./$(REGRESSION_TEST)
	./$(BROADPHASE_TEST)
	./$(SCENE_SUITE)

run: $(TARGET)
	./$(TARGET)

clean:
	rm -f $(TARGET) $(UNIT_TEST) $(REGRESSION_TEST) $(BROADPHASE_TEST) $(SCENE_SUITE)

.PHONY: all test run clean
