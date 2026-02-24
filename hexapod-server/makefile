CXX := g++

BUILD_DIR := build
TARGET := $(BUILD_DIR)/hexapod-server

CXXFLAGS := -std=c++20 -pedantic-errors -Wall -Wextra -Werror
CPPFLAGS := -Iinclude -I../hexapod-common/include
LDFLAGS := -lCppLinuxSerial -ltoml11

SERVER_SRC := $(wildcard src/*.cpp)
COMMON_SRC := ../hexapod-common/framing.cpp

SERVER_OBJ := $(patsubst src/%.cpp,$(BUILD_DIR)/src/%.o,$(SERVER_SRC))
COMMON_OBJ := $(addprefix $(BUILD_DIR)/common/,$(notdir $(COMMON_SRC:.cpp=.o)))
OBJ := $(SERVER_OBJ) $(COMMON_OBJ)
DEP := $(OBJ:.o=.d)

.PHONY: all debug release clean info

all: $(TARGET)

$(TARGET): $(OBJ)
	@mkdir -p $(@D)
	$(CXX) $(OBJ) -o $@ $(LDFLAGS)

$(BUILD_DIR)/src/%.o: src/%.cpp
	@mkdir -p $(@D)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -MMD -MP -c $< -o $@

$(BUILD_DIR)/common/%.o: ../hexapod-common/%.cpp
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
	@echo "Server sources: $(SERVER_SRC)"
	@echo "Common sources: $(COMMON_SRC)"
	@echo "Objects: $(OBJ)"
