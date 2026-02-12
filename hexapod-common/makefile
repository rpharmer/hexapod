CXX      := -c++
CXXFLAGS := -pedantic-errors -Wall -Wextra -Werror
DYNAMICLIBCXXFLAGS \
         :=  -fPIC
STATICLIBCXXFLAGS \
         := 
LDFLAGS  := -L/usr/lib -lstdc++ -lm -lCppLinuxSerial -ltoml11
DYNAMICLIBLDFLAGS \
         :=  -shared -Wl,-soname,libctest.so
STATICLIBLDFLAGS \
         := 
BUILD    := ./build
STATICLIBBUILDDIR \
         := $(BUILD)/static
DYNAMICLIBBUILDDIR \
         := $(BUILD)/dynamic
LIBDIR   := ./lib
STATICLIBTARGET \
  := libhexapodcommon.a
DYNAMICLIBTARGET \
  := libhexapodcommon.so
TARGETS  := $(STATICLIBTARGET) $(DYNAMICLIBTARGET)
INCLUDE  := -Iinclude/
SRC      :=                      \
   $(wildcard src/*.cpp)         \

OBJECTS  := $(SRC:%.cpp=$(BUILD)/%.o)
STATICLIBOBJECTS \
         := $(SRC:%.cpp=$(STATICLIBBUILDDIR)/%.o)
DYNAMICLIBOBJECTS \
         := $(SRC:%.cpp=$(DYNAMICLIBBUILDDIR)/%.o)
DEPENDENCIES \
         := $(OBJECTS:.o=.d)

all: $(addprefix $(LIBDIR)/, $(TARGETS))

$(BUILD)/%.o: %.cpp
	@mkdir -p $(@D)
	$(CXX) $(CXXFLAGS) $(INCLUDE) -c $< -MMD -o $@

$(STATICLIBBUILDDIR)/%.o: %.cpp
	@mkdir -p $(@D)
	$(CXX) $(CXXFLAGS) $(STATICLIBCXXFLAGS) $(INCLUDE) -c $< -MMD -o $@

$(DYNAMICLIBBUILDDIR)/%.o: %.cpp
	@mkdir -p $(@D)
	$(CXX) $(CXXFLAGS) $(DYNAMICLIBCXXFLAGS) $(INCLUDE) -c $< -MMD -o $@

$(LIBDIR)/$(TARGET): $(OBJECTS)
	@mkdir -p $(@D)
	$(CXX) $(CXXFLAGS) -o $(LIBDIR)/$(TARGET) $^ $(LDFLAGS)

$(LIBDIR)/$(STATICLIBTARGET): $(STATICLIBOBJECTS)
	@mkdir -p $(@D)
	ar rcs $(LIBDIR)/$(STATICLIBTARGET) $^

$(LIBDIR)/$(DYNAMICLIBTARGET): $(DYNAMICLIBOBJECTS)
	@mkdir -p $(@D)
	$(CXX) $(CXXFLAGS) -o $(LIBDIR)/$(DYNAMICLIBTARGET) $^ $(LDFLAGS) $(DYNAMICLIBLDFLAGS)

-include $(DEPENDENCIES)

.PHONY: all debug release info list clean install


debug: CXXFLAGS += -DDEBUG -g
debug: all

release: CXXFLAGS += -O2
release: all

install: $(LIBDIR)/$(TARGET)
	cp $(LIBDIR)/$(TARGET) /usr/lib/$(TARGET)

clean:
	rm -rf $(BUILD)/*
	rm -rf $(LIBDIR)/*

list:
	@echo $(shell ls)

info:
	@echo "[*] Sources:         ${SRC}         "
	@echo "[*] Objects:         ${OBJECTS}     "
	@echo "[*] Dependencies:    ${DEPENDENCIES}"
                       