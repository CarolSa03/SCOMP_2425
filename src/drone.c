#include "../includes/simulation.h"

extern pthread_mutex_t collision_mutex;
extern pthread_cond_t collision_cond;

/* drone process function */
void drone_process(int drone_id)
{
    int fd;
    SharedMemory *shm;
    char str[200];

    snprintf(str, sizeof(str), "Drone %d process started (PID: %d)\n",
             drone_id, getpid());
    write(STDOUT_FILENO, str, strlen(str));

    /* open existing shared memory */
    if ((fd = shm_open("/drone_sim", O_RDWR, 0)) == -1)
    {
        perror("drone shm_open");
        exit(1);
    }

    if ((shm = (SharedMemory *)mmap(NULL, sizeof(SharedMemory),
                                    PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0)) == MAP_FAILED)
    {
        perror("drone mmap");
        exit(2);
    }

    /* oen semaphores in child process */
    if ((sem_step_ready = sem_open("/sem_step_ready", 0)) == SEM_FAILED)
    {
        perror("drone sem_open step_ready");
        exit(3);
    }

    if ((sem_step_continue = sem_open("/sem_step_continue", 0)) == SEM_FAILED)
    {
        perror("drone sem_open step_continue");
        exit(4);
    }

    /* US364: main drone loop */
    while (!shm->simulation_finished && shm->drones[drone_id].active)
    {
        if (shm->current_timestep >= shm->time_steps)
        {
            snprintf(str, sizeof(str),
                     "Drone %d: trajectory completed at timestep %d\n",
                     drone_id, shm->current_timestep);
            write(STDOUT_FILENO, str, strlen(str));
            break;
        }

        update_drone_position(drone_id, shm->current_timestep, shm);
        Position *current_pos = &shm->drones[drone_id].current_pos;

        /* validate position */
        if (!is_valid_position(*current_pos))
        {
            shm->drones[drone_id].active = 0;
            snprintf(str, sizeof(str),
                     "Drone %d: mission completed (invalid position reached)\n",
                     drone_id);
            write(STDOUT_FILENO, str, strlen(str));
            break;
        }

        snprintf(str, sizeof(str),
                 "Drone %d: timestep %d, position (%.1f, %.1f, %.1f)\n",
                 drone_id, shm->current_timestep,
                 current_pos->x, current_pos->y, current_pos->z);
        write(STDOUT_FILENO, str, strlen(str));

        /* signal ready for next timestep */
        shm->step_ready[drone_id] = 1;
        /* signal coordinator that this drone is ready */
        sem_post(sem_step_ready);

        /* block until coordinator signals continue */
        sem_wait(sem_step_continue);

        /* aknowledge by clearing ready flag */
        shm->step_ready[drone_id] = 0;
    }

    shm->drones[drone_id].active = 0;

    /* cleaning up !!*/
    if (munmap(shm, sizeof(SharedMemory)) == -1)
    {
        perror("drone munmap");
    }

    if (close(fd) == -1)
    {
        perror("drone close");
    }

    snprintf(str, sizeof(str), "Drone %d process ending\n", drone_id);
    write(STDOUT_FILENO, str, strlen(str));
    exit(0);
}

void update_drone_position(int drone_id, int timestep, SharedMemory *shm)
{
    if (timestep < shm->time_steps)
    {
        shm->drones[drone_id].current_pos = shm->drones[drone_id].trajectory[timestep];
        shm->drones[drone_id].bounding_box = calculate_bounding_box(
            shm->drones[drone_id].current_pos,
            shm->drone_size);
    }
    else
    {
        shm->drones[drone_id].active = 0;
    }
}

DroneAABB calculate_bounding_box(Position pos, int drone_size)
{
    DroneAABB box;
    float half_size = (float)drone_size / 2.0f;

    box.minX = pos.x - half_size;
    box.maxX = pos.x + half_size;
    box.minY = pos.y - half_size;
    box.maxY = pos.y + half_size;
    box.minZ = pos.z - half_size;
    box.maxZ = pos.z + half_size;

    return box;
}

int check_aabb_collision(DroneAABB box1, DroneAABB box2)
{
    return (box1.minX <= box2.maxX && box1.maxX >= box2.minX &&
            box1.minY <= box2.maxY && box1.maxY >= box2.minY &&
            box1.minZ <= box2.maxZ && box1.maxZ >= box2.minZ);
}

int is_valid_position(Position pos)
{
    return !(pos.x == 0.0f && pos.y == 0.0f && pos.z == 0.0f);
}