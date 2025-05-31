// simulation.h
#ifndef SIMULATION_H
#define SIMULATION_H

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <semaphore.h>
#include <pthread.h>
#include <sys/wait.h>
#include <string.h>
#include <signal.h>

#define MAX_DRONES 50
#define MAX_TIME_STEPS 500
#define MAX_COLLISIONS 500

typedef struct {
  float x, y, z;
} Position;

typedef struct {
  float minX, maxX;
  float minY, maxY;
  float minZ, maxZ;
} Drone;

typedef struct {
  int time_step;
  int drone1, drone2;
  Position position1, position2;
} Collision;

typedef struct
{
  Position drone_positions[MAX_DRONES][MAX_TIME_STEPS];
  int current_time_step;
  int simulation_active;
  int collision_detected;
  int total_collisions;
  Collision collision_log[MAX_COLLISIONS];
  int collision_count;
  int num_drones;
  int max_collisions;
  int time_steps;
  int drone_size;
  int drones_finished;
  int drones_at_barrier;
} SharedMemory;

void generate_final_report();
void load_config();

#endif
