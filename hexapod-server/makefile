CXX      := -c++
CXXFLAGS := -pedantic-errors -Wall -Wextra -Werror
LDFLAGS  := -L/usr/lib -lstdc++ -lm -lCppLinuxSerial -ltoml11
# -L../hexapod-common/lib/ -lhexapodcommon
BUILD    := ./build
TARGET   := hexapod-server
INCLUDE  := -Iinclude/ -I../hexapod-common/include/
SRC      :=                      \
   $(wildcard src/*.cpp)         \

SRC_COMMON  := ../hexapod-common

OBJECTS  := $(SRC:%.cpp=$(BUILD)/%.o)
DEPENDENCIES \
         := $(OBJECTS:.o=.d)

copy_dir:
	@echo "Copying $(SRC_COMMON) to ./src ..."
	@cp -a $(SRC_COMMON)/*.cpp ./src/

all: copy_dir main

main: $(BUILD)/$(TARGET)

$(BUILD)/%.o: %.cpp
	@mkdir -p $(@D)
	$(CXX) $(CXXFLAGS) $(INCLUDE) -c $< -MMD -o $@

$(BUILD)/$(TARGET): $(OBJECTS)
	@mkdir -p $(@D)
	$(CXX) $(CXXFLAGS) -o $(BUILD)/$(TARGET) $^ $(LDFLAGS)

-include $(DEPENDENCIES)

.PHONY: copy_dir all main debug release info list


debug: CXXFLAGS += -DDEBUG -g
debug: all

release: CXXFLAGS += -O2
release: all

clean:
	rm -rf $(BUILD)/*

list:
	@echo $(shell ls)

info:
	@echo "[*] Sources:         ${SRC}         "
	@echo "[*] Objects:         ${OBJECTS}     "
	@echo "[*] Dependencies:    ${DEPENDENCIES}"
                       