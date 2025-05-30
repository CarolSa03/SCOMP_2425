#include "../include/simulation.h"
#include "../include/collision.h"
#include <signal.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>
#include <sys/wait.h>
#include <sys/types.h>
#include <fcntl.h>
#include <string.h>

DroneProcess drones[MAX_DRONE];

int collision_count = 0;
int total_collisions = 0;
int num_drones = 2;
int drone_size = 4;
int max_collision = 5;
int time_steps = 20;
Collision *collision_log = NULL;

void handle_sigchld()
{
  char msg[] = "SIGCHLD: drone process has terminated\n";
  write(STDOUT_FILENO, msg, sizeof(msg) - 1);

  pid_t pid;
  int status;
  while ((pid = waitpid(-1, &status, WNOHANG)) > 0)
  {
    char buf[100];
    int len = sprintf(buf, "Child process %d terminated with status %d\n", pid, status);
    write(STDOUT_FILENO, buf, len);
  }
}

void config(const char *filename)
{
  FILE *file = fopen(filename, "r");
  if (file == NULL)
  {
    perror("Failed to open config file");
    return;
  }

  char line[256];
  if (fgets(line, sizeof(line), file) != NULL)
  {
    line[strcspn(line, "\n")] = 0;

    int config_num_drones, config_drone_size, config_max_collisions, config_time_steps;
    if (sscanf(line, "%d, %d, %d, %d", &config_num_drones, &config_drone_size, &config_max_collisions, &config_time_steps) == 4)
    {
      num_drones = config_num_drones;
      drone_size = config_drone_size;
      max_collision = config_max_collisions;
      time_steps = config_time_steps;
    }
    else
    {
      fprintf(stderr, "Failed to parse configuration values\n");
    }
  }

  fclose(file);
}

Position **allocate_positions(int time_steps, int max_drones)
{
  Position **positions = (Position **)malloc(time_steps * sizeof(Position *));
  if (positions == NULL)
  {
    perror("Failed to allocate memory for positions array");
    return NULL;
  }

  for (int i = 0; i < time_steps; i++)
  {
    positions[i] = (Position *)malloc(max_drones * sizeof(Position));
    if (positions[i] == NULL)
    {
      perror("Failed to allocate memory for positions row");

      for (int j = 0; j < i; j++)
      {
        free(positions[j]);
      }
      free(positions);

      return NULL;
    }
  }

  return positions;
}

void free_positions(Position **positions, int time_steps)
{
  if (positions == NULL)
  {
    return;
  }

  for (int i = 0; i < time_steps; i++)
  {
    free(positions[i]);
  }

  free(positions);
}

void create_drone(int index)
{
  if (pipe((drones + index)->cmd_pipe) == -1 || pipe((drones + index)->pos_pipe) == -1)
  {
    perror("pipe");
    exit(EXIT_FAILURE);
  }

  (drones + index)->pid = fork();
  if ((drones + index)->pid == 0)
  {
    close(*((drones + index)->cmd_pipe + 1));
    close(*((drones + index)->pos_pipe + 0));

    dup2(*((drones + index)->cmd_pipe + 0), STDIN_FILENO);
    dup2(*((drones + index)->pos_pipe + 1), STDOUT_FILENO);

    execl("./drone", "./drone", NULL);

    perror("execl");
    exit(EXIT_FAILURE);
  }
  else
  {
    close(*((drones + index)->cmd_pipe + 0));
    close(*((drones + index)->pos_pipe + 1));
  }
}

int main()
{
  struct sigaction sa_chld;

  memset(&sa_chld, 0, sizeof(struct sigaction));

  sa_chld.sa_handler = handle_sigchld;

  if (sigemptyset(&sa_chld.sa_mask) != 0)
    perror("sigemptyset");

  sa_chld.sa_flags = 0;

  if (sigaction(SIGCHLD, &sa_chld, NULL) != 0)
    perror("sigaction");

  config("data/info.csv");

  Position **positions = allocate_positions(time_steps, num_drones);
  if (positions == NULL)
  {
    return EXIT_FAILURE;
  }

  collision_log = (Collision *)malloc(max_collision * 2 * sizeof(Collision));
  if (collision_log == NULL)
  {
    perror("Failed to allocate memory for collision_log");
    return EXIT_FAILURE;
  }

  printf("Configuration loaded from CSV:\n");
  printf("Number of drones: %d\n", num_drones);
  printf("Drone size: %d\n", drone_size);
  printf("Max collisions: %d\n", max_collision);
  printf("Time Steps: %d\n", time_steps);

  for (int i = 0; i < num_drones; i++)
  {
    create_drone(i);
    char buffer[100];
    int len = sprintf(buffer, "INIT data/drone%d_movement.csv\n", i + 1);
    write(*((drones + i)->cmd_pipe + 1), buffer, len);
    close(*((drones + i)->cmd_pipe + 1));
  }

  for (int t = 0; t < time_steps; t++)
  {

    if (total_collisions >= max_collision)
    {
      printf("Simulation terminated early due to excessive collisions.\n");
      break;
    }

    for (int i = 0; i < num_drones; i++)
    {
      char buffer[256];
      ssize_t bytes_read = read(*((drones + i)->pos_pipe + 0), buffer, sizeof(buffer));

      if (bytes_read > 0)
      {
        buffer[bytes_read] = '\0';
        float x, y, z;
        if (sscanf(buffer, "%f,%f,%f", &x, &y, &z) == 3)
        {
          positions[t][i].x = x;
          positions[t][i].y = y;
          positions[t][i].z = z;
        }
        else
        {
          printf("Failed to parse position data: %s\n", buffer);
        }
      }
    }
    check_collision(t, num_drones, positions, &collision_count, max_collision, &total_collisions, collision_log, drones, drone_size);
  }

  generate_report(num_drones, total_collisions, collision_count, collision_log, max_collision);

  for (int i = 0; i < num_drones; i++)
  {
    close(*((drones + i)->pos_pipe + 0));
    int status;
    pid_t result = waitpid(drones[i].pid, &status, WNOHANG);
    if (result > 0)
    {
      printf("Child process %d terminated with status %d\n",
             drones[i].pid, WEXITSTATUS(status));
    }
  }

  free_positions(positions, time_steps);
  free(collision_log);
  return 0;
}