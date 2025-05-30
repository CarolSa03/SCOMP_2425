#include "../include/simulation.h"
#include <signal.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>
#include <sys/wait.h>
#include <sys/types.h>
#include <fcntl.h>
#include <string.h>

void notify_drones_of_collision(pid_t *pids, int count)
{
  sigset_t old_mask, block_mask;
  memset(&block_mask, 0, sizeof(sigset_t));

  if (sigemptyset(&block_mask) != 0)
    perror("sigemptyset");
  if (sigaddset(&block_mask, SIGCHLD) != 0)
    perror("sigaddset");

  if (sigprocmask(SIG_BLOCK, &block_mask, &old_mask) != 0)
    perror("sigprocmask");

  for (int i = 0; i < count; i++)
  {
    kill(pids[i], SIGUSR1);
  }

  if (sigprocmask(SIG_SETMASK, &old_mask, NULL) != 0)
    perror("sigprocmask (restore)");
}

int intersect(Drone a, Drone b)
{
  return (a.minX <= b.maxX &&
          a.maxX >= b.minX &&
          a.minY <= b.maxY &&
          a.maxY >= b.minY &&
          a.minZ <= b.maxZ &&
          a.maxZ >= b.minZ);
}

Drone calculate_position(Position pos, int drone_size)
{
  Drone drone;
  float half = drone_size / 2;
  drone.minX = pos.x - half;
  drone.maxX = pos.x + half;
  drone.minY = pos.y - half;
  drone.maxY = pos.y + half;
  drone.minZ = pos.z - half;
  drone.maxZ = pos.z + half;
  return drone;
}

void check_collision(int time_step, int num_drones, Position **positions, int *collision_count, int max_collision, int *total_collisions, Collision *collision_log, DroneProcess *drones, int drone_size)
{
  int max_possible_collisions = num_drones * (num_drones - 1) / 2;
  pid_t *collided_drones = malloc(sizeof(pid_t) * max_possible_collisions * 2);

  if (collided_drones == NULL)
  {
    perror("Failed to allocate memory for collided_drones.");
    return;
  }

  int collision_count_local = 0;

  for (int i = 0; i < num_drones - 1; i++)
  {
    Drone a = calculate_position(positions[time_step][i], drone_size);

    for (int j = i + 1; j < num_drones; j++)
    {

      Drone b = calculate_position(positions[time_step][j], drone_size);

      if (intersect(a, b))
      {
        char buffer[256];
        int len = sprintf(buffer, "\nCOLLISION DETECTED: Drones %d and %d at time step %d\n",
                          i + 1, j + 1, time_step + 1);
        write(STDOUT_FILENO, buffer, len);

        if (*collision_count < max_collision * 2)
        {
          collision_log[*collision_count].time_step = time_step;
          collision_log[*collision_count].drone1 = i + 1;
          collision_log[*collision_count].drone2 = j + 1;
          collision_log[*collision_count].position1 = positions[time_step][i];
          collision_log[*collision_count].position2 = positions[time_step][j];
          (*collision_count)++;
        }
        collided_drones[collision_count_local++] = drones[i].pid;
        collided_drones[collision_count_local++] = drones[j].pid;

        (*total_collisions)++;

        if (*total_collisions >= max_collision)
        {
          break;
        }
      }
    }
    if (*total_collisions >= max_collision)
    {
      break;
    }
  }

  if (collision_count_local > 0)
  {
    notify_drones_of_collision(collided_drones, collision_count_local);
  }

  if (*total_collisions >= max_collision)
  {
    char msg[] = "\nMAX COLLISIONS EXCEEDED! Terminating simulation.\n";
    write(STDOUT_FILENO, msg, sizeof(msg) - 1);

    for (int i = 0; i < num_drones; i++)
    {
      if (kill(drones[i].pid, 0) == 0)
      {
        if (kill(drones[i].pid, SIGTERM) == -1)
        {
          perror("kill");
        }
      }
    }

   // sleep(1);

    for (int i = 0; i < num_drones; i++)
    {
      if (kill(drones[i].pid, 0) == 0)
      {
        printf("Drone %d (PID %d) not responding, sending SIGKILL\n", i + 1, drones[i].pid);
        kill(drones[i].pid, SIGKILL);
      }
    }
  }
}

void generate_report(int num_drones, int total_collisions, int collision_count, Collision *collision_log, int max_collision)
{
  int fd = open("report.txt", O_WRONLY | O_CREAT | O_TRUNC, 0644);
  if (fd == -1)
  {
    perror("Failed to create report file");
    return;
  }

  char buffer[1024];
  int len;

  len = sprintf(buffer, "DRONE SIMULATION REPORT\n\n");
  write(fd, buffer, len);

  len = sprintf(buffer, "Total drones: %d\n", num_drones);
  write(fd, buffer, len);

  len = sprintf(buffer, "Total collisions: %d\n\n", total_collisions);
  write(fd, buffer, len);

  if (collision_count > 0)
  {
    len = sprintf(buffer, "Collision details:\n");
    write(fd, buffer, len);

    for (int i = 0; i < collision_count; i++)
    {
      len = sprintf(buffer, "Time step %d: Collision between Drone %d and Drone %d at position (%.2f,%.2f,%.2f) and (%.2f,%.2f,%.2f), respectively. \n",
                    collision_log[i].time_step,
                    collision_log[i].drone1,
                    collision_log[i].drone2,
                    collision_log[i].position1.x,
                    collision_log[i].position1.y,
                    collision_log[i].position1.z,
                    collision_log[i].position2.x,
                    collision_log[i].position2.y,
                    collision_log[i].position2.z);
      write(fd, buffer, len);
    }
  }
  else
  {
    len = sprintf(buffer, "COMPLETELY SUCCESSFULLY. NO COLLISIONS.\n");
    write(fd, buffer, len);
  }

  if (collision_count > 0)
  {
    len = sprintf(buffer, (total_collisions < max_collision) ? "INVOLVED IN COLLISIONS." : "TERMINATED DUE TO EXCESSIVE COLLISIONS.");
    write(fd, buffer, len);
  }

  len = sprintf(buffer, (total_collisions < max_collision) ? "\nTHE FIGURE PASSED" : "\nTHE FIGURE FAILED");
  write(fd, buffer, len);

  close(fd);
  printf("\nSimulation report generated: report.txt\n");
}