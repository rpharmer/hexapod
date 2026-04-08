CXX ?= g++
CXXFLAGS ?= -std=c++17 -O2 -Wall -Wextra -pedantic
INCLUDES := -Iinclude -I.

TARGET := hexapod-physics-sim
CORE_SRC := src/minphys3d/core/world.cpp \
	src/minphys3d/core/world_broadphase.cpp \
	src/minphys3d/core/world_collision.cpp \
	src/minphys3d/core/world_solver.cpp \
	src/minphys3d/core/broadphase_system.cpp \
	src/minphys3d/core/contact_pipeline.cpp \
	src/minphys3d/core/constraint_solver.cpp \
	src/minphys3d/core/sleep_system.cpp \
	src/minphys3d/solver/block2_solver.cpp \
	src/minphys3d/solver/block4_solver.cpp \
	src/minphys3d/solver/island_ordering.cpp \
	src/minphys3d/narrowphase/gjk.cpp \
	src/minphys3d/narrowphase/epa.cpp
DEMO_SRC := src/demo/scenes.cpp
SRC := src/main.cpp $(CORE_SRC) $(DEMO_SRC)
UNIT_TEST := tests/test_unit_math
REGRESSION_TEST := tests/test_regression_step
BROADPHASE_TEST := tests/test_broadphase_dynamic_tree
SCENE_SUITE := tests/regression_scene_suite
CONVEX_DISPATCH_TEST := tests/test_convex_dispatch_equivalence
GJK_REGRESSION_TEST := tests/test_gjk_regression
BLOCK2_SOLVER_TEST := tests/test_block2_solver
BLOCK4_SOLVER_TEST := tests/test_block4_solver
ISLAND_ORDERING_TEST := tests/test_island_ordering
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

$(GJK_REGRESSION_TEST): tests/test_gjk_regression.cpp $(TEST_SRC_COMMON)
	$(CXX) $(CXXFLAGS) $(INCLUDES) tests/test_gjk_regression.cpp $(TEST_SRC_COMMON) -o $(GJK_REGRESSION_TEST)

$(BLOCK2_SOLVER_TEST): tests/test_block2_solver.cpp $(TEST_SRC_COMMON)
	$(CXX) $(CXXFLAGS) $(INCLUDES) tests/test_block2_solver.cpp $(TEST_SRC_COMMON) -o $(BLOCK2_SOLVER_TEST)

$(BLOCK4_SOLVER_TEST): tests/test_block4_solver.cpp $(TEST_SRC_COMMON)
	$(CXX) $(CXXFLAGS) $(INCLUDES) tests/test_block4_solver.cpp $(TEST_SRC_COMMON) -o $(BLOCK4_SOLVER_TEST)

$(ISLAND_ORDERING_TEST): tests/test_island_ordering.cpp $(TEST_SRC_COMMON)
	$(CXX) $(CXXFLAGS) $(INCLUDES) tests/test_island_ordering.cpp $(TEST_SRC_COMMON) -o $(ISLAND_ORDERING_TEST)

test: $(UNIT_TEST) $(REGRESSION_TEST) $(BROADPHASE_TEST) $(SCENE_SUITE) $(CONVEX_DISPATCH_TEST) $(GJK_REGRESSION_TEST) $(BLOCK2_SOLVER_TEST) $(BLOCK4_SOLVER_TEST) $(ISLAND_ORDERING_TEST)
	./$(UNIT_TEST)
	./$(REGRESSION_TEST)
	./$(BROADPHASE_TEST)
	./$(SCENE_SUITE)
	./$(CONVEX_DISPATCH_TEST)
	./$(GJK_REGRESSION_TEST)
	./$(BLOCK2_SOLVER_TEST)
	./$(BLOCK4_SOLVER_TEST)
	./$(ISLAND_ORDERING_TEST)

run: $(TARGET)
	./$(TARGET)

clean:
	rm -f $(TARGET) $(UNIT_TEST) $(REGRESSION_TEST) $(BROADPHASE_TEST) $(SCENE_SUITE) $(CONVEX_DISPATCH_TEST) $(GJK_REGRESSION_TEST) $(BLOCK2_SOLVER_TEST) $(BLOCK4_SOLVER_TEST) $(ISLAND_ORDERING_TEST)

.PHONY: all test run clean
