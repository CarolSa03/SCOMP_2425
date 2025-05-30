CC = gcc
CFLAGS = -Wall -Wextra -Iinclude
PARENT = parent
DRONE = drone
COLLISION = collision

PARENT_SRC = src/main.c
DRONE_SRC = src/drone.c
COLLISION_SRC = src/collision.c
HEADERS = include/simulation.h include/collision.h

all: $(PARENT) $(DRONE)

$(PARENT): $(PARENT_SRC) $(COLLISION_SRC) $(HEADERS)
	$(CC) $(CFLAGS) -o $(PARENT) $(PARENT_SRC) $(COLLISION_SRC)

$(DRONE): $(DRONE_SRC) $(HEADERS)
	$(CC) $(CFLAGS) -o $(DRONE) $(DRONE_SRC)

clean:
	rm -f $(PARENT) $(DRONE)

