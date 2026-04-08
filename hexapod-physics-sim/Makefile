CXX ?= g++
CXXFLAGS ?= -std=c++17 -O2 -Wall -Wextra -pedantic
INCLUDES := -Iinclude -I.

TARGET := hexapod-physics-sim
CORE_SRC := src/minphys3d/core/world.cpp \
	src/minphys3d/core/world_broadphase.cpp \
	src/minphys3d/core/world_collision.cpp \
	src/minphys3d/core/world_solver.cpp
DEMO_SRC := src/demo/scenes.cpp
SRC := src/main.cpp $(CORE_SRC) $(DEMO_SRC)
UNIT_TEST := tests/test_unit_math
REGRESSION_TEST := tests/test_regression_step
BROADPHASE_TEST := tests/test_broadphase_dynamic_tree
SCENE_SUITE := tests/regression_scene_suite
CONVEX_DISPATCH_TEST := tests/test_convex_dispatch_equivalence
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

$(CONVEX_DISPATCH_TEST): tests/test_convex_dispatch_equivalence.cpp $(TEST_SRC_COMMON)
	$(CXX) $(CXXFLAGS) $(INCLUDES) tests/test_convex_dispatch_equivalence.cpp $(TEST_SRC_COMMON) -o $(CONVEX_DISPATCH_TEST)

test: $(UNIT_TEST) $(REGRESSION_TEST) $(BROADPHASE_TEST) $(SCENE_SUITE) $(CONVEX_DISPATCH_TEST)
	./$(UNIT_TEST)
	./$(REGRESSION_TEST)
	./$(BROADPHASE_TEST)
	./$(SCENE_SUITE)
	./$(CONVEX_DISPATCH_TEST)

run: $(TARGET)
	./$(TARGET)

clean:
	rm -f $(TARGET) $(UNIT_TEST) $(REGRESSION_TEST) $(BROADPHASE_TEST) $(SCENE_SUITE) $(CONVEX_DISPATCH_TEST)

.PHONY: all test run clean
