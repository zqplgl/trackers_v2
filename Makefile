INCLUDE = -I./include

SRC= ${shell ls src/*.cpp}

all: libTrackDP.so

libTrackDP.so: ${SRC}
	mkdir -p lib
	g++ -shared -fPIC -std=c++11 -o lib/libTrackDP.so ${SRC} ${INCLUDE} `pkg-config --libs opencv`

install: 
	cp ./include/Itracker.h /home/zqp/install_lib/vehicleDll/include
	cp ./lib/libTrackDP.so /home/zqp/install_lib/vehicleDll/lib


clean:
	-rm -rf lib
