CXX ?= g++
CXXFLAGS ?= -std=c++17 -O2 -Wall -Wextra -pedantic
INCLUDES := -Iinclude -I.

TARGET := hexapod-physics-sim
SRC := src/main.cpp
UNIT_TEST := tests/test_unit_math
REGRESSION_TEST := tests/test_regression_step

all: $(TARGET)

$(TARGET): $(SRC)
	$(CXX) $(CXXFLAGS) $(INCLUDES) $(SRC) -o $(TARGET)

$(UNIT_TEST): tests/test_unit_math.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) tests/test_unit_math.cpp -o $(UNIT_TEST)

$(REGRESSION_TEST): tests/test_regression_step.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) tests/test_regression_step.cpp -o $(REGRESSION_TEST)

test: $(UNIT_TEST) $(REGRESSION_TEST)
	./$(UNIT_TEST)
	./$(REGRESSION_TEST)

run: $(TARGET)
	./$(TARGET)

clean:
	rm -f $(TARGET) $(UNIT_TEST) $(REGRESSION_TEST)

.PHONY: all test run clean
