CC = gcc
CFLAGS = -Wall -Wextra -g -Iincludes
LIBS = -lrt -lpthread

PARENT_SRC = src/main.c
DRONE_SRC = src/drone.c
THREAD_SRC = src/thread.c
HEADERS = includes/simulation.h

OBJS = main.o thread.o drone.o
TARGET = drone

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CC) $(CFLAGS) -o $@ $^ $(LIBS)

main.o: $(PARENT_SRC) $(HEADERS)
	$(CC) $(CFLAGS) -c $(PARENT_SRC) -o $@

thread.o: $(THREAD_SRC) $(HEADERS)
	$(CC) $(CFLAGS) -c $(THREAD_SRC) -o $@

drone.o: $(DRONE_SRC) $(HEADERS)
	$(CC) $(CFLAGS) -c $(DRONE_SRC) -o $@

clean:
	rm -f $(OBJS) $(TARGET) simulation_report.txt
	rm -f /dev/shm/drone_sim /dev/shm/sem_step /dev/shm/sem_collision /dev/shm/sem.sem_step_ready /dev/shm/sem.sem_step_continue

.PHONY: all clean clean-all 