#ifndef DRONE_SIMULATION_H
#define DRONE_SIMULATION_H

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/wait.h>
#include <pthread.h>
#include <semaphore.h>
#include <time.h>
#include <math.h>
#include <signal.h>

#define MAX_DRONES 50
#define MAX_TIMESTEPS 100
#define MAX_COLLISIONS 100
#define DEFAULT_DRONE_SIZE 5
#define CONFIG_PATH "data/info.csv"
#define TRAJECTORY_PATH_FORMAT "data/drone%d_movement.csv"

typedef struct
{
    float x;
    float y;
    float z;
} Position;

typedef struct
{
    float minX, maxX;
    float minY, maxY;
    float minZ, maxZ;
} DroneAABB;

typedef struct
{
    Position trajectory[MAX_TIMESTEPS];
    Position current_pos;
    DroneAABB bounding_box;
    int active;
    int drone_id;
} Drone;

typedef struct
{
    int timestep;
    int drone1_id;
    int drone2_id;
    Position pos1;
    Position pos2;
    DroneAABB box1;
    DroneAABB box2;
} CollisionEvent;

typedef struct
{
    Position position;
    DroneAABB bounding_box;
    int is_valid;
} TimeIndexedDroneState;

typedef struct
{
    int detected;
    int timestep_first_detected;
    CollisionEvent event_data;
} CollisionPairState;

typedef struct
{
    Drone drones[MAX_DRONES];
    CollisionEvent collisions[MAX_COLLISIONS];
    TimeIndexedDroneState time_indexed_states[MAX_TIMESTEPS][MAX_DRONES];
    CollisionPairState collision_matrix[MAX_TIMESTEPS][MAX_DRONES][MAX_DRONES];

    int num_drones;
    int drone_size;
    int max_collisions;
    int time_steps;
    int current_timestep;
    int collision_count;
    int simulation_finished;

    int step_ready[MAX_DRONES];
    int collision_detected;
    int report_ready;
    int collision_detected_this_timestep[MAX_DRONES][MAX_DRONES];

    int timestep_ready_for_collision;
    int collision_detection_complete;

    int active_drone_count;
    double simulation_start_time;
    double simulation_end_time;

    int time_indexed_collision_detection_complete;
    int pre_calculation_complete;
} SharedMemory;

extern sem_t *sem_step_ready;
extern sem_t *sem_step_continue;

void load_config(SharedMemory *shm);
void load_drone_trajectory(int drone_id, SharedMemory *shm);
void initialise_simulation(SharedMemory *shm);

DroneAABB drone_bounding(Position pos, int drone_size);
int intersect(DroneAABB box1, DroneAABB box2);
int is_valid_position(Position pos);

void drone_process(int drone_id);
void *collision_detection_thread(void *arg);
void *report_generation_thread(void *arg);

void update_drone_position(int drone_id, int timestep, SharedMemory *shm);
void generate_final_report(SharedMemory *shm);
void cleanup_resources(void);

void print_simulation_status(SharedMemory *shm);
double get_current_time(void);

void pre_calculate_positions(SharedMemory *shm);
void collision_detection(SharedMemory *shm);
void update_position(int drone_id, int timestep, SharedMemory *shm);
int check_collision(int drone1_id, int drone2_id, int timestep, SharedMemory *shm);

#endif