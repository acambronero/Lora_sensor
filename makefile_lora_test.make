WORKDIR = `pwd`

# ADD CROSS COMPILE
CXX = /opt/pkg/petalinux/petalinux-v2016.2-final/tools/linux-i386/gcc-arm-linux-gnueabi/bin/arm-linux-gnueabihf-g++
LD = /opt/pkg/petalinux/petalinux-v2016.2-final/tools/linux-i386/gcc-arm-linux-gnueabi/bin/arm-linux-gnueabihf-g++
WINDRES = windres

INC = 
CFLAGS = 
RESINC = 
LIBDIR = 
LIB = 
LDFLAGS = 


INC_RELEASE = $(INC) -ISX126x -ISX126x/radio -ISX126x/radio/sx126x -ISX126x/boards -ISX126x/boards/mcu -ISX126x/boards/mcu/espressif -I.
CFLAGS_RELEASE = $(CFLAGS) -Wall -O0 -g3 -c -fPIC -std=c++11 -DHAVE_PTHREAD -pthread -DZMQ_USE_EPOLL 
RESINC_RELEASE = $(RESINC)
RCFLAGS_RELEASE = $(RCFLAGS)
LIBDIR_RELEASE = $(LIBDIR)
LIB_RELEASE = $(LIB)
LDFLAGS_RELEASE = $(LDFLAGS) -s -pthread -O2 -fPIC
OBJDIR_RELEASE = obj/Release
DEP_RELEASE = 
OUT_RELEASE = bin/Release/Lora_sensor



OBJ_RELEASE = \
$(OBJDIR_RELEASE)/datahelper.o \
$(OBJDIR_RELEASE)/lorahandler.o \
$(OBJDIR_RELEASE)/loratimersloop.o \
$(OBJDIR_RELEASE)/monitoringhandler.o \
$(OBJDIR_RELEASE)/spibase.o \
$(OBJDIR_RELEASE)/spilora.o \
$(OBJDIR_RELEASE)/SX126xHardware.o \
$(OBJDIR_RELEASE)/timershandler.o \
$(OBJDIR_RELEASE)/main.o \
$(OBJDIR_RELEASE)/SX126x/radio/radiohandler.o \
$(OBJDIR_RELEASE)/SX126x/radio/sx126x.o \
$(OBJDIR_RELEASE)/SX126x/boards/timer.o 


all: release

clean: clean_release clean_objets

before_release:
		test -d bin/Release || mkdir -p bin/Release
		test -d $(OBJDIR_RELEASE) || mkdir -p $(OBJDIR_RELEASE)
		test -d $(OBJDIR_RELEASE)/SX126x || mkdir -p $(OBJDIR_RELEASE)/SX126x
		test -d $(OBJDIR_RELEASE)/SX126x/radio || mkdir -p $(OBJDIR_RELEASE)/SX126x/radio
		test -d $(OBJDIR_RELEASE)/SX126x/boards || mkdir -p $(OBJDIR_RELEASE)/SX126x/boards
		test -d $(OBJDIR_RELEASE)/Lora_sensor || mkdir -p $(OBJDIR_RELEASE)/Lora_sensor


after_release:

release: before_release out_release after_release clean_objets

out_release: before_release $(OBJ_RELEASE) $(DEP_RELEASE)
	$(LD) $(LIBDIR_RELEASE) -o $(OUT_RELEASE) $(OBJ_RELEASE)  $(LDFLAGS_RELEASE) $(LIB_RELEASE)

$(OBJDIR_RELEASE)/datahelper.o: datahelper.cpp
	$(CXX) $(CFLAGS_RELEASE) $(INC_RELEASE) -c datahelper.cpp -o $(OBJDIR_RELEASE)/datahelper.o

$(OBJDIR_RELEASE)/SX126x/boards/timer.o: SX126x/boards/mcu/espressif/timer.cpp SX126x/boards/mcu/timer.h
	$(CXX) $(CFLAGS_RELEASE) $(INC_RELEASE) -c SX126x/boards/mcu/espressif/timer.cpp -o $(OBJDIR_RELEASE)/SX126x/boards/timer.o

$(OBJDIR_RELEASE)/lorahandler.o: lorahandler.cpp SX126x/boards/mcu/timer.h
	$(CXX) $(CFLAGS_RELEASE) $(INC_RELEASE) -c lorahandler.cpp -o $(OBJDIR_RELEASE)/lorahandler.o

$(OBJDIR_RELEASE)/loratimersloop.o: loratimersloop.cpp
	$(CXX) $(CFLAGS_RELEASE) $(INC_RELEASE) -c loratimersloop.cpp -o $(OBJDIR_RELEASE)/loratimersloop.o

$(OBJDIR_RELEASE)/monitoringhandler.o: monitoringhandler.cpp
	$(CXX) $(CFLAGS_RELEASE) $(INC_RELEASE) -c monitoringhandler.cpp -o $(OBJDIR_RELEASE)/monitoringhandler.o

$(OBJDIR_RELEASE)/spibase.o: spibase.cpp
	$(CXX) $(CFLAGS_RELEASE) $(INC_RELEASE) -c spibase.cpp -o $(OBJDIR_RELEASE)/spibase.o

$(OBJDIR_RELEASE)/spilora.o: spilora.cpp
	$(CXX) $(CFLAGS_RELEASE) $(INC_RELEASE) -c spilora.cpp -o $(OBJDIR_RELEASE)/spilora.o

$(OBJDIR_RELEASE)/SX126xHardware.o: SX126xHardware.cpp
	$(CXX) $(CFLAGS_RELEASE) $(INC_RELEASE) -c SX126xHardware.cpp -o $(OBJDIR_RELEASE)/SX126xHardware.o

$(OBJDIR_RELEASE)/timershandler.o: timershandler.cpp
	$(CXX) $(CFLAGS_RELEASE) $(INC_RELEASE) -c timershandler.cpp -o $(OBJDIR_RELEASE)/timershandler.o
	
$(OBJDIR_RELEASE)/main.o: main.cpp
	$(CXX) $(CFLAGS_RELEASE) $(INC_RELEASE) -c main.cpp -o $(OBJDIR_RELEASE)/main.o

$(OBJDIR_RELEASE)/SX126x/radio/radiohandler.o: SX126x/radio/sx126x/radiohandler.cpp
	$(CXX) $(CFLAGS_RELEASE) $(INC_RELEASE) -c SX126x/radio/sx126x/radiohandler.cpp -o $(OBJDIR_RELEASE)/SX126x/radio/radiohandler.o

$(OBJDIR_RELEASE)/SX126x/radio/sx126x.o: SX126x/radio/sx126x/sx126x.cpp
	$(CXX) $(CFLAGS_RELEASE) $(INC_RELEASE) -c SX126x/radio/sx126x/sx126x.cpp -o $(OBJDIR_RELEASE)/SX126x/radio/sx126x.o

clean_release:
	rm -f $(OBJ_RELEASE) $(OUT_RELEASE)
	rm -rf bin/Release
	rm -rf $(OBJDIR_RELEASE)

clean_objets:
	rm -rf $(OBJDIR_RELEASE)

.PHONY: after_release clean_release
