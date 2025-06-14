#include "../includes/simulation.h"

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

void signal_handler(int sig)
{
    char msg[100];
    snprintf(msg, sizeof(msg), "Received signal %d, shutting down...\n", sig);
    write(STDOUT_FILENO, msg, strlen(msg));

    if (shm)
    {
        shm->simulation_finished = 1;

        /* wake up all waiting threads */
        pthread_mutex_lock(&collision_mutex);
        pthread_cond_broadcast(&collision_cond);
        pthread_mutex_unlock(&collision_mutex);

        pthread_mutex_lock(&step_mutex);
        pthread_cond_broadcast(&step_cond);
        pthread_mutex_unlock(&step_mutex);

        /* wake up drone processes */
        for (int i = 0; i < shm->num_drones; i++)
        {
            sem_post(sem_step_continue);
        }
    }

    cleanup_resources();
    exit(0);
}

int main()
{
    pid_t drone_pids[MAX_DRONES];
    int i, data_size;

    if (signal(SIGINT, signal_handler) == SIG_ERR)
    {
        perror("signal SIGINT");
        exit(1);
    }
    if (signal(SIGTERM, signal_handler) == SIG_ERR)
    {
        perror("signal SIGTERM");
        exit(2);
    }

    char str[100];
    snprintf(str, sizeof(str), "Loading configuration from CSV files...\n");
    write(STDOUT_FILENO, str, strlen(str));

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

    snprintf(str, sizeof(str), "Simulation configured:\n");
    write(STDOUT_FILENO, str, strlen(str));

    snprintf(str, sizeof(str), "- Drones: %d\n", shm->num_drones);
    write(STDOUT_FILENO, str, strlen(str));

    snprintf(str, sizeof(str), "- Drone size: %d\n", shm->drone_size);
    write(STDOUT_FILENO, str, strlen(str));

    snprintf(str, sizeof(str), "- Max collisions: %d\n", shm->max_collisions);
    write(STDOUT_FILENO, str, strlen(str));

    snprintf(str, sizeof(str), "- Time steps: %d\n", shm->time_steps);
    write(STDOUT_FILENO, str, strlen(str));

    snprintf(str, sizeof(str), "Pre-calculating all drone positions and collision matrix...\n");
    write(STDOUT_FILENO, str, strlen(str));
    pre_calculate_all_positions(shm);
    perform_time_indexed_collision_detection(shm);
    snprintf(str, sizeof(str), "Pre-calculation complete. Collision matrix ready.\n");
    write(STDOUT_FILENO, str, strlen(str));

    /* create semaphores */
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

    snprintf(str, sizeof(str), "All %d drones launched. Starting simulation...\n", shm->num_drones);
    write(STDOUT_FILENO, str, strlen(str));

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

        /* bounds checking */
        if (active_drones < 0 || active_drones > MAX_DRONES)
        {
            snprintf(str, sizeof(str), "Warning: Invalid active drone count %d\n", active_drones);
            write(STDOUT_FILENO, str, strlen(str));
            break;
        }

        /* block until a drone signals ready */
        for (i = 0; i < active_drones; i++)
        {
            sem_wait(sem_step_ready);
        }

        /* alll drones ready  */
        shm->current_timestep++;

        /* bounds checking for timestep */
        if (shm->current_timestep >= MAX_TIMESTEPS)
        {
            snprintf(str, sizeof(str), "Warning: Timestep exceeded maximum\n");
            write(STDOUT_FILENO, str, strlen(str));
            break;
        }

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

        snprintf(str, sizeof(str), "Timestep %d/%d (Active drones: %d, Collisions: %d)\n",
                 shm->current_timestep, shm->time_steps,
                 shm->active_drone_count, shm->collision_count);
        write(STDOUT_FILENO, str, strlen(str));

        /* check if collision threshold exceeded */
        if (shm->collision_count >= shm->max_collisions)
        {
            snprintf(str, sizeof(str), "SIMULATION TERMINATED: Collision threshold exceeded (%d >= %d)\n",
                     shm->collision_count, shm->max_collisions);
            write(STDOUT_FILENO, str, strlen(str));
            shm->simulation_finished = 1;
            break;
        }

        /* check if all drones completed their missions */
        if (shm->active_drone_count == 0)
        {
            snprintf(str, sizeof(str), "SIMULATION COMPLETED: All drones completed their missions\n");
            write(STDOUT_FILENO, str, strlen(str));
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

    snprintf(str, sizeof(str), "Simulation finished. Waiting for drones to terminate...\n");
    write(STDOUT_FILENO, str, strlen(str));

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
    snprintf(str, sizeof(str), "All processes terminated. Cleaning up...\n");
    write(STDOUT_FILENO, str, strlen(str));
    cleanup_resources();
    snprintf(str, sizeof(str), "Simulation completed successfully.\n");
    write(STDOUT_FILENO, str, strlen(str));
    return 0;
}

void pre_calculate_all_positions(SharedMemory *shm)
{
    int drone_id, timestep;
    char str[200];

    /* initialize the time indexed matrix */
    for (timestep = 0; timestep < shm->time_steps; timestep++)
    {
        for (drone_id = 0; drone_id < shm->num_drones; drone_id++)
        {
            if (timestep < shm->time_steps &&
                is_valid_position(shm->drones[drone_id].trajectory[timestep]))
            {

                shm->time_indexed_states[timestep][drone_id].position =
                    shm->drones[drone_id].trajectory[timestep];
                shm->time_indexed_states[timestep][drone_id].bounding_box =
                    calculate_bounding_box(shm->drones[drone_id].trajectory[timestep],
                                           shm->drone_size);
                shm->time_indexed_states[timestep][drone_id].is_valid = 1;
            }
            else
            {
                shm->time_indexed_states[timestep][drone_id].is_valid = 0;
            }
        }
    }

    shm->pre_calculation_complete = 1;
    snprintf(str, sizeof(str), "Pre-calculated positions for %d drones across %d timesteps\n",
             shm->num_drones, shm->time_steps);
    write(STDOUT_FILENO, str, strlen(str));
}

void perform_time_indexed_collision_detection(SharedMemory *shm)
{
    int timestep, i, j;
    char str[500];

    /* bounds checking */
    if (shm->time_steps <= 0 || shm->time_steps > MAX_TIMESTEPS)
    {
        snprintf(str, sizeof(str), "Warning: Invalid time steps for collision detection\n");
        write(STDOUT_FILENO, str, strlen(str));
        return;
    }
    if (shm->num_drones <= 0 || shm->num_drones > MAX_DRONES)
    {
        snprintf(str, sizeof(str), "Warning: Invalid drone count for collision detection\n");
        write(STDOUT_FILENO, str, strlen(str));
        return;
    }

    /* starting the coollision matrix */
    for (timestep = 0; timestep < shm->time_steps; timestep++)
    {
        for (i = 0; i < shm->num_drones; i++)
        {
            for (j = 0; j < shm->num_drones; j++)
            {
                shm->collision_matrix[timestep][i][j].detected = 0;
                shm->collision_matrix[timestep][i][j].timestep_first_detected = -1;
            }
        }
    }

    /* perform collision detection for each timestep */
    for (timestep = 0; timestep < shm->time_steps; timestep++)
    {
        for (i = 0; i < shm->num_drones - 1; i++)
        {
            if (!shm->time_indexed_states[timestep][i].is_valid)
                continue;

            for (j = i + 1; j < shm->num_drones; j++)
            {
                if (!shm->time_indexed_states[timestep][j].is_valid)
                    continue;

                /* check if collision already detected for this pair */
                if (shm->collision_matrix[timestep][i][j].detected)
                    continue;

                /* perform AABB collision check */
                if (check_aabb_collision(
                        shm->time_indexed_states[timestep][i].bounding_box,
                        shm->time_indexed_states[timestep][j].bounding_box))
                {

                    /* mark collision in matrix */
                    shm->collision_matrix[timestep][i][j].detected = 1;
                    shm->collision_matrix[timestep][j][i].detected = 1;
                    shm->collision_matrix[timestep][i][j].timestep_first_detected = timestep;
                    shm->collision_matrix[timestep][j][i].timestep_first_detected = timestep;

                    /* store collision event data */
                    CollisionEvent *collision = &shm->collision_matrix[timestep][i][j].event_data;
                    collision->timestep = timestep;
                    collision->drone1_id = i;
                    collision->drone2_id = j;
                    collision->pos1 = shm->time_indexed_states[timestep][i].position;
                    collision->pos2 = shm->time_indexed_states[timestep][j].position;
                    collision->box1 = shm->time_indexed_states[timestep][i].bounding_box;
                    collision->box2 = shm->time_indexed_states[timestep][j].bounding_box;

                    snprintf(str, sizeof(str),
                             "TDrones %d and %d at timestep %d\n"
                             " Drone %d: pos(%.1f,%.1f,%.1f) box[%.1f-%.1f,%.1f-%.1f,%.1f-%.1f]\n"
                             " Drone %d: pos(%.1f,%.1f,%.1f) box[%.1f-%.1f,%.1f-%.1f,%.1f-%.1f]\n",
                             i, j, timestep,
                             i, collision->pos1.x, collision->pos1.y, collision->pos1.z,
                             collision->box1.minX, collision->box1.maxX,
                             collision->box1.minY, collision->box1.maxY,
                             collision->box1.minZ, collision->box1.maxZ,
                             j, collision->pos2.x, collision->pos2.y, collision->pos2.z,
                             collision->box2.minX, collision->box2.maxX,
                             collision->box2.minY, collision->box2.maxY,
                             collision->box2.minZ, collision->box2.maxZ);
                    write(STDOUT_FILENO, str, strlen(str));
                }
            }
        }
    }

    shm->time_indexed_collision_detection_complete = 1;
    snprintf(str, sizeof(str), "Collision detection complete\n");
    write(STDOUT_FILENO, str, strlen(str));
}

void load_config(SharedMemory *shm)
{
    char str[256];
    FILE *config = fopen(CONFIG_PATH, "r");
    if (config)
    {
        if (fscanf(config, "%d,%d,%d,%d",
                   &shm->num_drones, &shm->drone_size,
                   &shm->max_collisions, &shm->time_steps) != 4)
        {
            snprintf(str, sizeof(str), "Warning: Invalid config format, using defaults\n");
            write(STDOUT_FILENO, str, strlen(str));
            goto use_defaults;
        }
        fclose(config);

        /* validate configuration values */
        if (shm->num_drones <= 0 || shm->num_drones > MAX_DRONES)
        {
            snprintf(str, sizeof(str), "Warning: Invalid drone count, using default\n");
            write(STDOUT_FILENO, str, strlen(str));
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
        if (shm->max_collisions < 0 || shm->max_collisions > MAX_COLLISIONS)
        {
            shm->max_collisions = 5;
        }
        snprintf(str, sizeof(str), "Configuration loaded from %s\n", CONFIG_PATH);
        write(STDOUT_FILENO, str, strlen(str));
    }
    else
    {
    use_defaults:
        shm->num_drones = 10;
        shm->drone_size = DEFAULT_DRONE_SIZE;
        shm->max_collisions = 5;
        shm->time_steps = 50;
        snprintf(str, sizeof(str), "Using default configuration\n");
        write(STDOUT_FILENO, str, strlen(str));
    }
}

void load_drone_trajectory(int drone_id, SharedMemory *shm)
{
    char filename[100], str[350];

    /* bounds checking */
    if (drone_id < 0 || drone_id >= MAX_DRONES)
    {
        snprintf(str, sizeof(str), "Warning: Invalid drone ID %d\n", drone_id);
        write(STDOUT_FILENO, str, strlen(str));
        return;
    }

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
                /*if we cannot read any more data, it will mark remaining positions as invalid */
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
        snprintf(str, sizeof(str), "Loaded trajectory for drone %d from %s\n", drone_id + 1, filename);
        write(STDOUT_FILENO, str, strlen(str));
    }
    else
    {
        snprintf(str, sizeof(str), "Warning: Could not load trajectory for drone %d, using default\n", drone_id + 1);
        write(STDOUT_FILENO, str, strlen(str));
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
    shm->time_indexed_collision_detection_complete = 0;
    shm->pre_calculation_complete = 0;

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
    char str[300];
    snprintf(str, sizeof(str), "\nSIMULATION SUMMARY\n");
    write(STDOUT_FILENO, str, strlen(str));
    snprintf(str, sizeof(str), "Result: %s\n",
             (shm->collision_count >= shm->max_collisions) ? "FAILED" : "PASSED");
    write(STDOUT_FILENO, str, strlen(str));
}

/* cleanup function */
void cleanup_resources(void)
{
    /* destroy mutexes and condition variables */
    pthread_mutex_destroy(&step_mutex);
    pthread_cond_destroy(&step_cond);
    pthread_mutex_destroy(&collision_mutex);
    pthread_cond_destroy(&collision_cond);

    /* unmap shared memory */
    if (munmap(shm, sizeof(SharedMemory)) == -1)
    {
        perror("munmap");
    }

    /* close file descriptor */
    if (close(fd_shm) == -1)
    {
        perror("close");
    }

    /* removes shared memory and semaphores */
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