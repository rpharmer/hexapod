CXX := g++

BUILD_DIR := build
TARGET := $(BUILD_DIR)/hexapod-server

CXXFLAGS := -std=c++20 -pedantic-errors -Wall -Wextra -Werror
CPPFLAGS := -Iinclude -I../hexapod-common/include
LDFLAGS := -lCppLinuxSerial -ltoml11

SERVER_SRC := $(wildcard src/*.cpp)
COMMON_SRC := ../hexapod-common/framing.cpp
SRC := $(SERVER_SRC) $(COMMON_SRC)
OBJ := $(patsubst %.cpp,$(BUILD_DIR)/%.o,$(SRC))
DEP := $(OBJ:.o=.d)

.PHONY: all debug release clean info

all: $(TARGET)

$(TARGET): $(OBJ)
	@mkdir -p $(@D)
	$(CXX) $(OBJ) -o $@ $(LDFLAGS)

$(BUILD_DIR)/%.o: %.cpp
	@mkdir -p $(@D)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -MMD -MP -c $< -o $@

-include $(DEP)

debug: CXXFLAGS += -O0 -g -DDEBUG
debug: all

release: CXXFLAGS += -O2 -DNDEBUG
release: all

clean:
	rm -rf $(BUILD_DIR)

info:
	@echo "Target: $(TARGET)"
	@echo "Sources: $(SRC)"
	@echo "Objects: $(OBJ)"
