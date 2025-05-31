CC = gcc
CFLAGS = -Wall -Wextra -g
LIBS = -lrt -lpthread
PARENT = parent
DRONE = drone

PARENT_SRC = src/main.c
DRONE_SRC = src/drone.c
HEADERS = include/simulation.h

all: $(PARENT) $(DRONE)

$(PARENT): $(PARENT_SRC) $(HEADERS)
	$(CC) $(CFLAGS) -o $(PARENT) $(PARENT_SRC) $(LIBS)

$(DRONE): $(DRONE_SRC) $(HEADERS)
	$(CC) $(CFLAGS) -o $(DRONE) $(DRONE_SRC) $(LIBS)

clean:
	rm -f $(PARENT) $(DRONE) simulation_report.txt
	rm -f /dev/shm/drone_sim /dev/shm/step_sync /dev/shm/shared_mutex /dev/shm/drones_ready

.PHONY: all clean
