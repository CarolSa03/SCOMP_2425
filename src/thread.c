#include "../includes/simulation.h"

extern pthread_mutex_t collision_mutex;
extern pthread_cond_t collision_cond;
extern pthread_mutex_t step_mutex;
extern pthread_cond_t step_cond;

/* US362 & US363: collision detection  */
void *collision_detection_thread(void *arg)
{
    SharedMemory *shm = (SharedMemory *)arg;
    int i, j;
    char str[200];

    snprintf(str, sizeof(str), "collision detection thread started (ID: %u)\n",
             (unsigned int)pthread_self());
    write(STDOUT_FILENO, str, strlen(str));

    while (!shm->simulation_finished)
    {
        pthread_mutex_lock(&step_mutex);
        while (!shm->timestep_ready_for_collision && !shm->simulation_finished)
        {
            pthread_cond_wait(&step_cond, &step_mutex);
        }

        if (shm->simulation_finished)
        {
            pthread_mutex_unlock(&step_mutex);
            break;
        }

        pthread_mutex_unlock(&step_mutex);

        for (i = 0; i < shm->num_drones - 1; i++)
        {
            if (!shm->drones[i].active)
                continue;

            for (j = i + 1; j < shm->num_drones; j++)
            {
                if (!shm->drones[j].active)
                    continue;

                if (shm->collision_detected_this_timestep[i][j] == 1)
                {
                    continue;
                }

                if (check_aabb_collision(shm->drones[i].bounding_box,
                                         shm->drones[j].bounding_box))
                {
                    shm->collision_detected_this_timestep[i][j] = 1;
                    shm->collision_detected_this_timestep[j][i] = 1;

                    if (shm->collision_count < MAX_COLLISIONS)
                    {
                        CollisionEvent *collision = &shm->collisions[shm->collision_count];
                        collision->timestep = shm->current_timestep;
                        collision->drone1_id = i;
                        collision->drone2_id = j;
                        collision->pos1 = shm->drones[i].current_pos;
                        collision->pos2 = shm->drones[j].current_pos;
                        collision->box1 = shm->drones[i].bounding_box;
                        collision->box2 = shm->drones[j].bounding_box;
                        shm->collision_count++;

                        snprintf(str, sizeof(str),
                                 "COLLISION DETECTED! Drones %d and %d at timestep %d\n"
                                 " Drone %d: pos(%.1f,%.1f,%.1f) box[%.1f-%.1f,%.1f-%.1f,%.1f-%.1f]\n"
                                 " Drone %d: pos(%.1f,%.1f,%.1f) box[%.1f-%.1f,%.1f-%.1f,%.1f-%.1f]\n",
                                 i, j, shm->current_timestep,
                                 i, collision->pos1.x, collision->pos1.y, collision->pos1.z,
                                 collision->box1.minX, collision->box1.maxX,
                                 collision->box1.minY, collision->box1.maxY,
                                 collision->box1.minZ, collision->box1.maxZ,
                                 j, collision->pos2.x, collision->pos2.y, collision->pos2.z,
                                 collision->box2.minX, collision->box2.maxX,
                                 collision->box2.minY, collision->box2.maxY,
                                 collision->box2.minZ, collision->box2.maxZ);
                        write(STDOUT_FILENO, str, strlen(str));

                        pthread_mutex_lock(&collision_mutex);
                        shm->collision_detected = 1;
                        pthread_cond_signal(&collision_cond);
                        pthread_mutex_unlock(&collision_mutex);
                    }
                }
            }
        }

        pthread_mutex_lock(&step_mutex);
        shm->collision_detection_complete = 1;
        pthread_cond_signal(&step_cond);
        pthread_mutex_unlock(&step_mutex);
    }

    snprintf(str, sizeof(str), "collision detection thread ending\n");
    write(STDOUT_FILENO, str, strlen(str));
    pthread_exit(NULL);
}

/* US363 & US365: report generation thread */
void *report_generation_thread(void *arg)
{
    SharedMemory *shm = (SharedMemory *)arg;
    char str[200];

    snprintf(str, sizeof(str), "report generation thread started (ID: %u)\n",
             (unsigned int)pthread_self());
    write(STDOUT_FILENO, str, strlen(str));

    while (!shm->simulation_finished)
    {
        /* US363: Wait for collision notification*/
        pthread_mutex_lock(&collision_mutex);
        while (!shm->collision_detected && !shm->simulation_finished)
        {
            pthread_cond_wait(&collision_cond, &collision_mutex);
        }

        if (shm->simulation_finished)
        {
            pthread_mutex_unlock(&collision_mutex);
            break;
        }

        if (shm->collision_detected)
        {
            snprintf(str, sizeof(str),
                     "Report thread: Processing collision event (total: %d/%d)\n",
                     shm->collision_count, shm->max_collisions);
            write(STDOUT_FILENO, str, strlen(str));

            if (shm->collision_count >= shm->max_collisions)
            {
                snprintf(str, sizeof(str),
                         "CRITICAL: Collision threshold reached!\n");
                write(STDOUT_FILENO, str, strlen(str));
            }

            shm->collision_detected = 0;
        }

        pthread_mutex_unlock(&collision_mutex);
    }

    /* US365: Generate final report when simulation ends */
    generate_final_report(shm);
    shm->report_ready = 1;

    snprintf(str, sizeof(str), "report generation thread ending\n");
    write(STDOUT_FILENO, str, strlen(str));
    pthread_exit(NULL);
}

/* US365: final simulation report generation */
void generate_final_report(SharedMemory *shm)
{
    FILE *report_file;
    char str[200];
    int i, active_drones;

    report_file = fopen("simulation_report.txt", "w");
    if (report_file == NULL)
    {
        perror("fopen report");
        return;
    }

    fprintf(report_file, "DRONE SIMULATION REPORT\n");

    fprintf(report_file, "SIMULATION CONFIGURATION:\n");
    fprintf(report_file, "- Total drones: %d\n", shm->num_drones);
    fprintf(report_file, "- Drone size (collision box): %d units\n", shm->drone_size);
    fprintf(report_file, "- Collision threshold: %d\n", shm->max_collisions);
    fprintf(report_file, "\n");

    active_drones = 0;
    for (i = 0; i < shm->num_drones; i++)
    {
        if (shm->drones[i].active)
        {
            active_drones++;
        }
    }

    fprintf(report_file, "INDIVIDUAL DRONE STATUS:\n");
    for (i = 0; i < shm->num_drones; i++)
    {
        Position *pos = &shm->drones[i].current_pos;
        // DroneAABB *box = &shm->drones[i].bounding_box;
        fprintf(report_file, "Drone %d: \n", i + 1);
        fprintf(report_file, " Position: (%.1f, %.1f, %.1f)\n",
                pos->x, pos->y, pos->z);
        // fprintf(report_file, " Bounding Box: X[%.1f-%.1f] Y[%.1f-%.1f] Z[%.1f-%.1f]\n", box->minX, box->maxX, box->minY, box->maxY, box->minZ, box->maxZ);
    }
    fprintf(report_file, "\n");
    fprintf(report_file, "COLLISION ANALYSIS:\n");
    fprintf(report_file, "- Total collisions detected: %d\n", shm->collision_count);
    fprintf(report_file, "- Collision rate: %.2f per timestep\n",
            shm->current_timestep > 0 ? (float)shm->collision_count / shm->current_timestep : 0.0f);

    if (shm->collision_count == 0)
    {
        fprintf(report_file, "- No collisions detected during simulation.\n");
    }
    else
    {
        fprintf(report_file, "\nDETAILED COLLISION EVENTS:\n");
        for (i = 0; i < shm->collision_count; i++)
        {
            CollisionEvent *c = &shm->collisions[i];
            fprintf(report_file, "Collision %d:\n", i + 1);
            fprintf(report_file, " - Timestep: %d\n", c->timestep);
            fprintf(report_file, " - Drones involved: %d and %d\n",
                    c->drone1_id, c->drone2_id);
            fprintf(report_file, " - Drone %d position: (%.1f, %.1f, %.1f)\n",
                    c->drone1_id, c->pos1.x, c->pos1.y, c->pos1.z);
            fprintf(report_file, " - Drone %d position: (%.1f, %.1f, %.1f)\n",
                    c->drone2_id, c->pos2.x, c->pos2.y, c->pos2.z);
            fprintf(report_file, "\n");
        }
    }

    fprintf(report_file, "\nSIMULATION VALIDATION RESULT: ");
    if (shm->collision_count >= shm->max_collisions)
    {
        fprintf(report_file, "FAILED\n");
        fprintf(report_file, "Reason: Collision threshold exceeded (%d >= %d)\n",
                shm->collision_count, shm->max_collisions);
        fprintf(report_file, "Recommendation: Revise flight paths to reduce collision risk\n");
    }
    else
    {
        fprintf(report_file, "PASSED\n");
        fprintf(report_file, "Reason: Safe flight pattern maintained\n");
        fprintf(report_file, "Quality: %d collisions detected (within threshold of %d)\n",
                shm->collision_count, shm->max_collisions);
    }

    fclose(report_file);
    snprintf(str, sizeof(str), "final report generated: simulation_report.txt\n");
    write(STDOUT_FILENO, str, strlen(str));
}