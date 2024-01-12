CC = g++
CFLAGS = -std=c++14 -I/usr/include/eigen3
LDFLAGS = -L/usr/local/lib

# -I /usr/include/python3.8/ 
# -L /usr/lib/x86_64-linux-gnu -L /usr/lib/python3.8/config-3.8-x86_64-linux-gnu/ -lopencv_core -lopencv_highgui -lpython3.8


# Get all .cpp files in the current directory
SOURCES := $(wildcard *.cpp)

# Generate corresponding object file names
OBJECTS := $(SOURCES:.cpp=.o)

# The name of the executable
EXECUTABLE = eigen_example

all: $(EXECUTABLE)

$(EXECUTABLE): $(OBJECTS)
	$(CC) $(CFLAGS) $(OBJECTS) -o $@

%.o: %.cpp
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm -f $(OBJECTS) $(EXECUTABLE)
