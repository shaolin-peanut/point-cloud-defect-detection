CC = g++
CFLAGS = -Wall -O2 -std=c++11
LDFLAGS = -lpcl_io -lpcl_visualization -lpcl_common -lpcl_filters -lpcl_kdtree -lpcl_sample_consensus -lpcl_search -lboost_system -las -lm

SOURCES = main.cpp
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
