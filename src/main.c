#include "../includes/simulation.h"
#include <time.h>
#include <math.h>

/* Global variables for resource cleanup - following teacher's pattern */
int fd_shm;
SharedMemory *shm;
sem_t *sem_step_ready;
sem_t *sem_step_continue;
pthread_t collision_thread;
pthread_t report_thread;
pthread_mutex_t step_mutex;
pthread_cond_t step_cond;
pthread_mutex_t collision_mutex;
pthread_cond_t collision_cond;

int main()
{
    pid_t drone_pids[MAX_DRONES];
    int i, data_size;

    printf("=== Enhanced Drone Simulation System ===\n");
    printf("Loading configuration from CSV files...\n");

    /* US361: create shared memory */
    data_size = sizeof(SharedMemory);
    if ((fd_shm = shm_open("/drone_sim", O_CREAT | O_EXCL | O_RDWR,
                           S_IRUSR | S_IWUSR)) == -1)
    {
        perror("shm_open");
        exit(3);
    }

    if (ftruncate(fd_shm, data_size) == -1)
    {
        perror("ftruncate");
        exit(4);
    }

    if ((shm = (SharedMemory *)mmap(NULL, data_size, PROT_READ | PROT_WRITE,
                                    MAP_SHARED, fd_shm, 0)) == MAP_FAILED)
    {
        perror("mmap");
        exit(5);
    }

    initialize_simulation(shm);
    printf("Simulation configured:\n");
    printf("- Drones: %d\n", shm->num_drones);
    printf("- Drone size: %d\n", shm->drone_size);
    printf("- Max collisions: %d\n", shm->max_collisions);
    printf("- Time steps: %d\n", shm->time_steps);

    /* create semaphores for process synchronization */
    if ((sem_step_ready = sem_open("/sem_step_ready", O_CREAT | O_EXCL, 0644, 0)) == SEM_FAILED)
    {
        perror("sem_open step_ready");
        exit(6);
    }

    if ((sem_step_continue = sem_open("/sem_step_continue", O_CREAT | O_EXCL, 0644, 0)) == SEM_FAILED)
    {
        perror("sem_open step_continue");
        exit(7);
    }

    /* mutexes and condition variables */
    if (pthread_mutex_init(&step_mutex, NULL) != 0)
    {
        perror("pthread_mutex_init step");
        exit(8);
    }

    if (pthread_cond_init(&step_cond, NULL) != 0)
    {
        perror("pthread_cond_init step");
        exit(9);
    }

    if (pthread_mutex_init(&collision_mutex, NULL) != 0)
    {
        perror("pthread_mutex_init collision");
        exit(10);
    }

    if (pthread_cond_init(&collision_cond, NULL) != 0)
    {
        perror("pthread_cond_init collision");
        exit(11);
    }

    if (pthread_create(&collision_thread, NULL, collision_detection_thread, shm) != 0)
    {
        perror("pthread_create collision");
        exit(12);
    }

    if (pthread_create(&report_thread, NULL, report_generation_thread, shm) != 0)
    {
        perror("pthread_create report");
        exit(13);
    }

    /* Create drone processes */
    for (i = 0; i < shm->num_drones; i++)
    {
        drone_pids[i] = fork();
        if (drone_pids[i] == 0)
        {
            drone_process(i);
            exit(0);
        }
        else if (drone_pids[i] < 0)
        {
            perror("fork");
            exit(14);
        }
    }

    printf("All %d drones launched. Starting simulation...\n", shm->num_drones);

    /* US364 */
    while (shm->current_timestep < shm->time_steps && !shm->simulation_finished)
    {
        /* wait for all active drones to be ready */
        int active_drones = 0;
        for (i = 0; i < shm->num_drones; i++)
        {
            if (shm->drones[i].active)
            {
                active_drones++;
            }
        }

        /* Block until a drone signals ready */
        for (i = 0; i < active_drones; i++)
        {
            sem_wait(sem_step_ready);
        }

        /* alll drones ready  */
        shm->current_timestep++;

        /* signal collision detection thread */
        pthread_mutex_lock(&step_mutex);
        shm->timestep_ready_for_collision = 1;
        pthread_cond_signal(&step_cond);
        pthread_mutex_unlock(&step_mutex);

        /* wait for collision detection to complete */
        pthread_mutex_lock(&step_mutex);
        while (!shm->collision_detection_complete && !shm->simulation_finished)
        {
            pthread_cond_wait(&step_cond, &step_mutex);
        }
        shm->collision_detection_complete = 0;
        shm->timestep_ready_for_collision = 0;
        pthread_mutex_unlock(&step_mutex);

        /* signal all drones to continue */
        for (i = 0; i < active_drones; i++)
        {
            sem_post(sem_step_continue); /* unblock waiting drones */
        }

        /* clear collision tracking matrix AFTER drones have moved !! */
        for (int x = 0; x < MAX_DRONES; x++)
        {
            for (int y = 0; y < MAX_DRONES; y++)
            {
                shm->collision_detected_this_timestep[x][y] = 0;
            }
        }

        /* update active drone count */
        shm->active_drone_count = 0;
        for (i = 0; i < shm->num_drones; i++)
        {
            if (shm->drones[i].active)
            {
                shm->active_drone_count++;
            }
        }

        printf("Timestep %d/%d (Active drones: %d, Collisions: %d)\n",
               shm->current_timestep, shm->time_steps,
               shm->active_drone_count, shm->collision_count);

        /* check if collision threshold exceeded */
        if (shm->collision_count >= shm->max_collisions)
        {
            printf("SIMULATION TERMINATED: Collision threshold exceeded (%d >= %d)\n",
                   shm->collision_count, shm->max_collisions);
            shm->simulation_finished = 1;
            break;
        }

        /* check if all drones completed their missions */
        if (shm->active_drone_count == 0)
        {
            printf("SIMULATION COMPLETED: All drones completed their missions\n");
            shm->simulation_finished = 1;
            break;
        }
    }

    /* signal simulation end */
    shm->simulation_finished = 1;

    /* US363: Notify all waiting threads */
    pthread_mutex_lock(&collision_mutex);
    pthread_cond_broadcast(&collision_cond);
    pthread_mutex_unlock(&collision_mutex);

    pthread_mutex_lock(&step_mutex);
    pthread_cond_broadcast(&step_cond);
    pthread_mutex_unlock(&step_mutex);

    printf("Simulation finished. Waiting for drones to terminate...\n");

    /* wake up any remaining drone processes */
    for (i = 0; i < shm->num_drones; i++)
    {
        sem_post(sem_step_continue);
    }

    /* wait for all drone processes */
    for (i = 0; i < shm->num_drones; i++)
    {
        wait(NULL);
    }

    /* wait for threads to finish */
    pthread_join(collision_thread, NULL);
    pthread_join(report_thread, NULL);

    print_simulation_status(shm);
    printf("All processes terminated. Cleaning up...\n");
    cleanup_resources();
    printf("Enhanced simulation completed successfully.\n");
    return 0;
}

void load_config(SharedMemory *shm)
{
    FILE *config = fopen(CONFIG_PATH, "r");
    if (config)
    {
        if (fscanf(config, "%d,%d,%d,%d",
                   &shm->num_drones, &shm->drone_size,
                   &shm->max_collisions, &shm->time_steps) != 4)
        {
            printf("Warning: Invalid config format, using defaults\n");
            goto use_defaults;
        }
        fclose(config);

        /* Validate configuration values */
        if (shm->num_drones <= 0 || shm->num_drones > MAX_DRONES)
        {
            printf("Warning: Invalid drone count, using default\n");
            shm->num_drones = 10;
        }
        if (shm->drone_size <= 0)
        {
            shm->drone_size = DEFAULT_DRONE_SIZE;
        }
        if (shm->time_steps <= 0 || shm->time_steps > MAX_TIMESTEPS)
        {
            shm->time_steps = 50;
        }
        printf("Configuration loaded from %s\n", CONFIG_PATH);
    }
    else
    {
    use_defaults:
        shm->num_drones = 10;
        shm->drone_size = DEFAULT_DRONE_SIZE;
        shm->max_collisions = 5;
        shm->time_steps = 50;
        printf("Using default configuration\n");
    }
}

void load_drone_trajectory(int drone_id, SharedMemory *shm)
{
    char filename[100];
    snprintf(filename, sizeof(filename), TRAJECTORY_PATH_FORMAT, drone_id + 1);
    FILE *fp = fopen(filename, "r");

    if (fp)
    {
        for (int step = 0; step < shm->time_steps; step++)
        {
            if (fscanf(fp, "%f,%f,%f",
                       &shm->drones[drone_id].trajectory[step].x,
                       &shm->drones[drone_id].trajectory[step].y,
                       &shm->drones[drone_id].trajectory[step].z) != 3)
            {
                /* If we can't read more data, mark remaining positions as invalid */
                for (int remaining = step; remaining < shm->time_steps; remaining++)
                {
                    shm->drones[drone_id].trajectory[remaining].x = 0.0f;
                    shm->drones[drone_id].trajectory[remaining].y = 0.0f;
                    shm->drones[drone_id].trajectory[remaining].z = 0.0f;
                }
                break;
            }
        }
        fclose(fp);
        printf("Loaded trajectory for drone %d from %s\n", drone_id + 1, filename);
    }
    else
    {
        printf("Warning: Could not load trajectory for drone %d, using default\n", drone_id + 1);
        for (int step = 0; step < shm->time_steps; step++)
        {
            shm->drones[drone_id].trajectory[step].x = drone_id * 10.0f + step;
            shm->drones[drone_id].trajectory[step].y = drone_id * 10.0f;
            shm->drones[drone_id].trajectory[step].z = 100.0f;
        }
    }
}

void initialize_simulation(SharedMemory *shm)
{
    load_config(shm);

    /* initialize simulation state */
    shm->current_timestep = 0;
    shm->collision_count = 0;
    shm->simulation_finished = 0;
    shm->collision_detected = 0;
    shm->report_ready = 0;
    shm->active_drone_count = shm->num_drones;
    shm->timestep_ready_for_collision = 0;
    shm->collision_detection_complete = 0;

    /* tracking matrix */
    for (int i = 0; i < MAX_DRONES; i++)
    {
        for (int j = 0; j < MAX_DRONES; j++)
        {
            shm->collision_detected_this_timestep[i][j] = 0;
        }
    }

    /* starting drones */
    for (int i = 0; i < shm->num_drones; i++)
    {
        shm->drones[i].active = 1;
        shm->drones[i].drone_id = i;
        shm->step_ready[i] = 0;
        load_drone_trajectory(i, shm);

        shm->drones[i].current_pos = shm->drones[i].trajectory[0];

        shm->drones[i].bounding_box = calculate_bounding_box(
            shm->drones[i].current_pos, shm->drone_size);
    }
}

void print_simulation_status(SharedMemory *shm)
{
    printf("\nSIMULATION SUMMARY\n");
    printf("Result: %s\n",
           (shm->collision_count >= shm->max_collisions) ? "FAILED" : "PASSED");
}

/* cleanup function */
void cleanup_resources(void)
{
    /* desstroy mutexes and condition variables */
    pthread_mutex_destroy(&step_mutex);
    pthread_cond_destroy(&step_cond);
    pthread_mutex_destroy(&collision_mutex);
    pthread_cond_destroy(&collision_cond);

    /* unmap shared memory */
    if (munmap(shm, sizeof(SharedMemory)) == -1)
    {
        perror("munmap");
    }

    /* closee file descriptor */
    if (close(fd_shm) == -1)
    {
        perror("close");
    }

    /* removing shared memory and semaphores */
    if (shm_unlink("/drone_sim") == -1)
    {
        perror("shm_unlink");
    }

    if (sem_unlink("/sem_step_ready") == -1)
    {
        perror("sem_unlink step_ready");
    }

    if (sem_unlink("/sem_step_continue") == -1)
    {
        perror("sem_unlink step_continue");
    }
}