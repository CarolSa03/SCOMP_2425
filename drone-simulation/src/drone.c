#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/wait.h>

volatile sig_atomic_t handling_collision = 0;
volatile sig_atomic_t time_to_go = 0;

void collision_handler()
{
  if (handling_collision)
    return;

  handling_collision = 1;

  char msg[100];
  int len = sprintf(msg, "Drone PID %d: Collision detected! Handling SIGUSR1...\n", getpid());
  write(STDERR_FILENO, msg, len);

  // sleep(1);
  // handling_collision = 0;
}

void termination_handler(int signum)
{
  if (signum == SIGINT || signum == SIGTERM)
  {
    // char msg[] = "Drone: Received termination signal, cleaning up...\n";
    // write(STDOUT_FILENO, msg, sizeof(msg) - 1);
    time_to_go = 1;
    // exit(0);
  }
}

int main()
{
  struct sigaction sa_collision;
  memset(&sa_collision, 0, sizeof(struct sigaction));
  sa_collision.sa_handler = collision_handler;

  if (sigfillset(&sa_collision.sa_mask) != 0)
    perror("sigfillset");
  sa_collision.sa_flags = 0;

  sigaction(SIGUSR1, &sa_collision, NULL);

  struct sigaction sa_term;
  memset(&sa_term, 0, sizeof(struct sigaction));
  sa_term.sa_handler = termination_handler;
  sigemptyset(&sa_term.sa_mask);
  sa_term.sa_flags = 0;

  sigaction(SIGINT, &sa_term, NULL);
  sigaction(SIGTERM, &sa_term, NULL);

  char script_path[100];
  FILE *script;
  fgets(script_path, sizeof(script_path), stdin);
  sscanf(script_path, "INIT %s", script_path);

  if ((script = fopen(script_path, "r")) == NULL)
  {
    perror("fopen");
    exit(EXIT_FAILURE);
  }

  char line[256];
  while (fgets(line, sizeof(line), script) != NULL)
  {
    if (handling_collision)
    {
     // sleep(1);
      handling_collision = 0;
    }

    if (time_to_go)
    {
      break;
    }

    line[strcspn(line, "\n")] = 0;
    printf("%s", line);
    fflush(stdout);
    // sleep(1);
  }
  if (fclose(script) != 0)
  {
    perror("Error closing script.");
    exit(EXIT_FAILURE);
  }
  if (time_to_go)
  {
    char msg[] = "Drone: Received termination signal, cleaning up...\n";
    write(STDOUT_FILENO, msg, sizeof(msg) - 1);
  }
  return 0;
}