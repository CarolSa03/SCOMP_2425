#ifndef simulation_h
#define simulation_h

#include <unistd.h>
#include <sys/types.h>

#define MAX_DRONE 100

typedef struct
{
  pid_t pid;
  int cmd_pipe[2];
  int pos_pipe[2];
} DroneProcess;

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
} Drone;

typedef struct
{
  int time_step;
  int drone1;
  int drone2;
  Position position1;
  Position position2;
} Collision;

typedef struct
{
  int num_drones;
  int drone_size;
  int max_collisions;
} Config;

#endif
