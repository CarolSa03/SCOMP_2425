#ifndef collision_h
#define collision_h

#include <unistd.h>
#include <sys/types.h>
#include "simulation.h"

Drone calculate_position(Position pos, int drone_size);

void check_collision(int time_step, int num_drones, Position **positions, int *collision_count,
                     int max_collision, int *total_collisions, Collision *collision_log,
                     DroneProcess *drones, int drone_size);

void generate_report(int num_drones, int total_collisions,
                     int collision_count, Collision *collision_log, int max_collision);

#endif
