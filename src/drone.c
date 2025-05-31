#include "../include/simulation.h"

int main(int argc, char *argv[])
{
  char buffer[256];

  if (argc != 2)
  {
    snprintf(buffer, sizeof(buffer), "Usage: %s <drone_id>\n", argv[0]);
    write(STDERR_FILENO, buffer, strlen(buffer));
    exit(1);
  }

  int drone_id = atoi(argv[1]);

  /* opens shared memory area */
  int shm_fd = shm_open("/drone_sim", O_RDWR, 0);
  if (shm_fd == -1)
  {
    perror("drone: shm_open");
    exit(1);
  }

  /* maps shm into address space */
  SharedMemory *shared_mem = mmap(NULL, sizeof(SharedMemory),
                                  PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);
  if (shared_mem == MAP_FAILED)
  {
    perror("drone: mmap");
    exit(2);
  }

  /* opens semaphores */
  sem_t *step_sync_sem = sem_open("/step_sync", 0);
  sem_t *shared_mem_mutex = sem_open("/shared_mutex", 0);
  sem_t *drones_ready_sem = sem_open("/drones_ready", 0);

  if (step_sync_sem == SEM_FAILED || shared_mem_mutex == SEM_FAILED ||
      drones_ready_sem == SEM_FAILED)
  {
    perror("drone: sem_open");
    exit(3);
  }

  char filename[256];
  snprintf(filename, sizeof(filename), "data/drone%d_movement.csv", drone_id + 1);

  FILE *movement_file = fopen(filename, "r");
  Position movements[MAX_TIME_STEPS];
  int loaded_steps = 0;

  char line[512];
  snprintf(buffer, sizeof(buffer), "Drone %d: Reading from %s\n", drone_id + 1, filename);
  write(STDOUT_FILENO, buffer, strlen(buffer));

  while (fgets(line, sizeof(line), movement_file) && loaded_steps < MAX_TIME_STEPS)
  {
    line[strcspn(line, "\n")] = 0;
    if (strlen(line) == 0)
      continue;

    float x, y, z;
    int parsed = sscanf(line, "%f,%f,%f", &x, &y, &z);
    if (parsed == 3)
    {
      movements[loaded_steps].x = x;
      movements[loaded_steps].y = y;
      movements[loaded_steps].z = z;
      loaded_steps++;
    }
  }

  fclose(movement_file);

  if (loaded_steps == 0)
  {
    snprintf(buffer, sizeof(buffer),
             "Drone %d: ERROR - No valid data found in %s\n", drone_id + 1, filename);
    write(STDOUT_FILENO, buffer, strlen(buffer));
    munmap(shared_mem, sizeof(SharedMemory));
    close(shm_fd);
    return 1;
  }

  snprintf(buffer, sizeof(buffer),
           "Drone %d: Successfully loaded %d movement steps\n", drone_id + 1, loaded_steps);
  write(STDOUT_FILENO, buffer, strlen(buffer));

  /* main execution loop */
  for (int t = 0; t < loaded_steps && shared_mem->simulation_active; t++)
  {
    sem_wait(shared_mem_mutex);
    if (t < MAX_TIME_STEPS && drone_id < MAX_DRONES)
    {
      shared_mem->drone_positions[drone_id][t] = movements[t];
    }
    sem_post(shared_mem_mutex);

    /* signals readiness */
    sem_post(drones_ready_sem);

    /* waits for permission to proceed */
    sem_wait(step_sync_sem);

    if (!shared_mem->simulation_active)
    {
      snprintf(buffer, sizeof(buffer),
               "Drone %d: Simulation ended early at step %d\n", drone_id + 1, t);
      write(STDOUT_FILENO, buffer, strlen(buffer));
      break;
    }
  }

  /* signals that this drone has finished */
  sem_wait(shared_mem_mutex);
  shared_mem->drones_finished++;
  snprintf(buffer, sizeof(buffer),
           "Drone %d: Finished simulation - %d/%d drones finished\n",
           drone_id + 1, shared_mem->drones_finished, shared_mem->num_drones);
  write(STDOUT_FILENO, buffer, strlen(buffer));
  sem_post(shared_mem_mutex);

  /* continues signaling readiness until simulation ends */
  while (shared_mem->simulation_active)
  {
    sem_post(drones_ready_sem);
    sem_wait(step_sync_sem);
  }

  /* cleanup */
  munmap(shared_mem, sizeof(SharedMemory));
  close(shm_fd);

  return 0;
}
