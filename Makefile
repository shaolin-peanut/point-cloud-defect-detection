CC = g++
CFLAGS = -Wall -O2 -std=c++14 $(shell pkg-config --cflags pcl_common pcl_io pcl_visualization pcl_filters)
CFLAGS += -I/usr/include/vtk -I/usr/include/geotiff
LDFLAGS = $(shell pkg-config --libs pcl_common pcl_io pcl_visualization pcl_filters)
# LDFLAGS += -lvtk -lgeotiff -llas

SOURCES = pcl_lidar_viz.cpp
OBJECTS = $(SOURCES:.cpp=.o)
EXECUTABLE = lidar_viewer

all: $(EXECUTABLE)

$(EXECUTABLE): $(OBJECTS)
	$(CC) $(OBJECTS) -o $@ $(LDFLAGS)

%.o: %.cpp
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm -f $(OBJECTS) $(EXECUTABLE)

.PHONY: all clean
