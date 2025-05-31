/**
 * Author: Carolina Sá, Nilsa Gil, Diogo
 * Created: May 2025
 **/

#include "../include/simulation.h"

SharedMemory *shared_mem;
sem_t *step_sync_sem;
sem_t *shared_mem_mutex;
sem_t *drones_ready_sem;
pthread_mutex_t report_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t collision_cond = PTHREAD_COND_INITIALIZER;
int simulation_running = 1;

Drone calculate_position(Position pos, int drone_size)
{
  Drone drone;
  float half = (float)drone_size / 2.0f;

  drone.minX = pos.x - half;
  drone.maxX = pos.x + half;
  drone.minY = pos.y - half;
  drone.maxY = pos.y + half;
  drone.minZ = pos.z - half;
  drone.maxZ = pos.z + half;

  return drone;
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

void *collision_detection_thread(void *arg)
{
  char buffer[256];

  while (simulation_running && shared_mem->simulation_active)
  {
    sem_wait(shared_mem_mutex);
    int current_step = shared_mem->current_time_step;
    int num_drones = shared_mem->num_drones;
    int drone_size = shared_mem->drone_size;

    if (current_step >= 0 && current_step < shared_mem->time_steps)
    {
      for (int i = 0; i < num_drones - 1; i++)
      {
        for (int j = i + 1; j < num_drones; j++)
        {
          Position pos1 = shared_mem->drone_positions[i][current_step];
          Position pos2 = shared_mem->drone_positions[j][current_step];

          if ((pos1.x == 0.0f && pos1.y == 0.0f && pos1.z == 0.0f) ||
              (pos2.x == 0.0f && pos2.y == 0.0f && pos2.z == 0.0f))
          {
            continue;
          }

          Drone drone_a = calculate_position(pos1, drone_size);
          Drone drone_b = calculate_position(pos2, drone_size);

          if (intersect(drone_a, drone_b))
          {
            if (shared_mem->collision_count < MAX_COLLISIONS)
            {
              shared_mem->collision_log[shared_mem->collision_count].time_step = current_step;
              shared_mem->collision_log[shared_mem->collision_count].drone1 = i + 1;
              shared_mem->collision_log[shared_mem->collision_count].drone2 = j + 1;
              shared_mem->collision_log[shared_mem->collision_count].position1 = pos1;
              shared_mem->collision_log[shared_mem->collision_count].position2 = pos2;
              shared_mem->collision_count++;
            }

            shared_mem->total_collisions++;
            shared_mem->collision_detected = 1;

            snprintf(buffer, sizeof(buffer),
                     "COLLISION DETECTED: Drones %d and %d at time step %d\n",
                     i + 1, j + 1, current_step + 1);
            write(STDOUT_FILENO, buffer, strlen(buffer));

            pthread_mutex_lock(&report_mutex);
            pthread_cond_signal(&collision_cond);
            pthread_mutex_unlock(&report_mutex);

            if (shared_mem->total_collisions >= shared_mem->max_collisions)
            {
              snprintf(buffer, sizeof(buffer),
                       "COLLISION THRESHOLD EXCEEDED - Stopping simulation\n");
              write(STDOUT_FILENO, buffer, strlen(buffer));
              shared_mem->simulation_active = 0;
              simulation_running = 0;
            }
          }
        }
      }
    }

    sem_post(shared_mem_mutex);
  }

  return NULL;
}

void generate_final_report()
{
  FILE *report_file = fopen("simulation_report.txt", "w");
  if (!report_file)
  {
    perror("fopen");
    return;
  }

  fprintf(report_file, "DRONE SIMULATION REPORT\n");
  fprintf(report_file, "========================\n\n");
  fprintf(report_file, "Total drones: %d\n", shared_mem->num_drones);
  fprintf(report_file, "Total collisions: %d\n", shared_mem->total_collisions);
  fprintf(report_file, "Max allowed collisions: %d\n", shared_mem->max_collisions);
  fprintf(report_file, "Time steps completed: %d\n\n", shared_mem->current_time_step + 1);

  if (shared_mem->collision_count > 0)
  {
    fprintf(report_file, "Collision Details:\n");
    for (int i = 0; i < shared_mem->collision_count; i++)
    {
      Collision c = shared_mem->collision_log[i];
      fprintf(report_file,
              "Time step %d: Collision between Drone %d and Drone %d at position (%.2f,%.2f,%.2f) and (%.2f,%.2f,%.2f), respectively.\n",
              c.time_step + 1, c.drone1, c.drone2,
              c.position1.x, c.position1.y, c.position1.z,
              c.position2.x, c.position2.y, c.position2.z);
    }
  }
  else
  {
    fprintf(report_file, "COMPLETED SUCCESSFULLY. NO COLLISIONS.\n");
  }

  if (shared_mem->collision_count > 0)
  {
    fprintf(report_file,
            (shared_mem->total_collisions < shared_mem->max_collisions) ? "\nINVOLVED IN COLLISIONS." : "\nTERMINATED DUE TO EXCESSIVE COLLISIONS.");
  }

  fprintf(report_file,
          (shared_mem->total_collisions < shared_mem->max_collisions) ? "\nTHE FIGURE PASSED" : "\nTHE FIGURE FAILED");

  fclose(report_file);

  char buffer[100];
  snprintf(buffer, sizeof(buffer), "Simulation report generated: simulation_report.txt\n");
  write(STDOUT_FILENO, buffer, strlen(buffer));
}

void *report_thread_function(void *arg)
{
  int last_reported_collisions = 0;
  char buffer[256];

  while (simulation_running)
  {
    pthread_mutex_lock(&report_mutex);
    pthread_cond_wait(&collision_cond, &report_mutex);
    pthread_mutex_unlock(&report_mutex);

    if (!simulation_running || !shared_mem->simulation_active)
    {
      break;
    }

    sem_wait(shared_mem_mutex);
    int current_collisions = shared_mem->total_collisions;
    int current_step = shared_mem->current_time_step;
    sem_post(shared_mem_mutex);

    if (current_collisions > last_reported_collisions)
    {
      snprintf(buffer, sizeof(buffer),
               "REPORT: %d collisions detected at step %d\n",
               current_collisions, current_step + 1);
      write(STDOUT_FILENO, buffer, strlen(buffer));
      last_reported_collisions = current_collisions;
    }
  }

  generate_final_report();
  return NULL;
}

void load_config()
{
  FILE *config_file = fopen("data/info.csv", "r");
  if (!config_file)
  {
    shared_mem->num_drones = 2;
    shared_mem->drone_size = 4;
    shared_mem->max_collisions = 5;
    shared_mem->time_steps = 20;

    char buffer[50];
    snprintf(buffer, sizeof(buffer), "Using default configuration\n");
    write(STDOUT_FILENO, buffer, strlen(buffer));
    return;
  }

  char line[256];
  if (fgets(line, sizeof(line), config_file))
  {
    if (sscanf(line, "%d,%d,%d,%d",
               &shared_mem->num_drones,
               &shared_mem->drone_size,
               &shared_mem->max_collisions,
               &shared_mem->time_steps) != 4)
    {

      char buffer[60];
      snprintf(buffer, sizeof(buffer), "Error parsing config file, using defaults\n");
      write(STDOUT_FILENO, buffer, strlen(buffer));

      shared_mem->num_drones = 2;
      shared_mem->drone_size = 4;
      shared_mem->max_collisions = 5;
      shared_mem->time_steps = 20;
    }
  }

  fclose(config_file);
}

int main()
{
  char buffer[256];

  /* creates/opens shared memory area */
  int shm_fd = shm_open("/drone_sim", O_CREAT | O_RDWR, S_IRUSR | S_IWUSR);
  if (shm_fd == -1)
  {
    perror("shm_open");
    exit(1);
  }

  /* defines size of shm */
  if (ftruncate(shm_fd, sizeof(SharedMemory)) == -1)
  {
    perror("ftruncate");
    exit(2);
  }

  /* maps shm into address space */
  shared_mem = mmap(NULL, sizeof(SharedMemory),
                    PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);
  if (shared_mem == MAP_FAILED)
  {
    perror("mmap");
    exit(3);
  }

  /* creates semaphores */
  step_sync_sem = sem_open("/step_sync", O_CREAT, 0644, 0);
  shared_mem_mutex = sem_open("/shared_mutex", O_CREAT, 0644, 1);
  drones_ready_sem = sem_open("/drones_ready", O_CREAT, 0644, 0);

  if (step_sync_sem == SEM_FAILED || shared_mem_mutex == SEM_FAILED ||
      drones_ready_sem == SEM_FAILED)
  {
    perror("sem_open");
    exit(4);
  }

  /* initializes shared memory */
  memset(shared_mem, 0, sizeof(SharedMemory));
  shared_mem->simulation_active = 1;
  load_config();

  snprintf(buffer, sizeof(buffer), "Configuration:\n");
  write(STDOUT_FILENO, buffer, strlen(buffer));
  snprintf(buffer, sizeof(buffer), "Number of drones: %d\n", shared_mem->num_drones);
  write(STDOUT_FILENO, buffer, strlen(buffer));
  snprintf(buffer, sizeof(buffer), "Drone size: %d\n", shared_mem->drone_size);
  write(STDOUT_FILENO, buffer, strlen(buffer));
  snprintf(buffer, sizeof(buffer), "Max collisions: %d\n", shared_mem->max_collisions);
  write(STDOUT_FILENO, buffer, strlen(buffer));
  snprintf(buffer, sizeof(buffer), "Time steps: %d\n", shared_mem->time_steps);
  write(STDOUT_FILENO, buffer, strlen(buffer));

  /* creates threads */
  pthread_t collision_thread, report_thread;
  pthread_create(&collision_thread, NULL, collision_detection_thread, NULL);
  pthread_create(&report_thread, NULL, report_thread_function, NULL);

  /* creates drone processes */
  pid_t drone_pids[MAX_DRONES];
  for (int i = 0; i < shared_mem->num_drones; i++)
  {
    drone_pids[i] = fork();
    if (drone_pids[i] == 0)
    {
      char drone_id[10];
      snprintf(drone_id, sizeof(drone_id), "%d", i);
      execl("./drone", "./drone", drone_id, NULL);
      perror("execl");
      exit(1);
    }
  }

  /* main simulation loop */
  for (int t = 0; t < shared_mem->time_steps && shared_mem->simulation_active; t++)
  {
    snprintf(buffer, sizeof(buffer), "Starting time step %d\n", t);
    write(STDOUT_FILENO, buffer, strlen(buffer));

    /* waits for all drones to be ready */
    for (int i = 0; i < shared_mem->num_drones; i++)
    {
      sem_wait(drones_ready_sem);
    }

    /* updates current time step */
    sem_wait(shared_mem_mutex);
    shared_mem->current_time_step = t;
    int finished_drones = shared_mem->drones_finished;
    sem_post(shared_mem_mutex);

    if (finished_drones >= shared_mem->num_drones)
    {
      snprintf(buffer, sizeof(buffer), "All drones finished at time step %d\n", t);
      write(STDOUT_FILENO, buffer, strlen(buffer));
      break;
    }

    /* signals drones to proceed */
    for (int i = 0; i < shared_mem->num_drones; i++)
    {
      sem_post(step_sync_sem);
    }

    /* checks collision threshold */
    sem_wait(shared_mem_mutex);
    int current_collisions = shared_mem->total_collisions;
    int max_allowed = shared_mem->max_collisions;
    sem_post(shared_mem_mutex);

    snprintf(buffer, sizeof(buffer),
             "Time step %d completed - Collisions: %d/%d, Finished drones: %d/%d\n",
             t, current_collisions, max_allowed, finished_drones, shared_mem->num_drones);
    write(STDOUT_FILENO, buffer, strlen(buffer));

    if (current_collisions >= max_allowed)
    {
      snprintf(buffer, sizeof(buffer), "SIMULATION TERMINATED: Maximum collisions reached\n");
      write(STDOUT_FILENO, buffer, strlen(buffer));
      break;
    }
  }

  /* signals end of simulation */
  sem_wait(shared_mem_mutex);
  shared_mem->simulation_active = 0;
  sem_post(shared_mem_mutex);

  /* wakes up report thread */
  pthread_mutex_lock(&report_mutex);
  simulation_running = 0;
  pthread_cond_signal(&collision_cond);
  pthread_mutex_unlock(&report_mutex);

  /* signals remaining drones */
  for (int i = 0; i < shared_mem->num_drones; i++)
  {
    sem_post(step_sync_sem);
  }

  /* waits for all child processes */
  for (int i = 0; i < shared_mem->num_drones; i++)
  {
    waitpid(drone_pids[i], NULL, 0);
  }

  /* waits for threads */
  pthread_join(collision_thread, NULL);
  pthread_join(report_thread, NULL);

  snprintf(buffer, sizeof(buffer), "Simulation completed successfully\n");
  write(STDOUT_FILENO, buffer, strlen(buffer));

  /* cleanup */
  munmap(shared_mem, sizeof(SharedMemory));
  close(shm_fd);
  shm_unlink("/drone_sim");
  sem_unlink("/step_sync");
  sem_unlink("/shared_mutex");
  sem_unlink("/drones_ready");

  return 0;
}
