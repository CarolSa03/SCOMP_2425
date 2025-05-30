# Drone Figure Simulation System

---
# Component Diagram


## 1.Component Description

- **Main Process ('main.c')**: creates a process for each drone, manages pipes for communication, and stores all drones positions over time.
- **Drone Process ('drone.c')**: each drone reads its movement script and sends its current position to the main process at each time step.
- **Movement Scripts**: '.txt' files with sequential positions for each drone.

## 2.Example Drone Movement Script

Example for drone1_movement.txt:

0,0,0
1,2,3
2,4,6
3,6,9
4,8,12

Each line represents the drone's position (x, y, z) at a given time step.


## 3.Approach for Each User Story

---

### US261 - Initiate simulation for a figure

- **Process and Pipes: How communication works**
  To enable a two-way communication between the main process and each drone process, *two unidirectional pipe are created for every drone*. Each pipe serves a distinct purpose and direction:

  **1. Command Pipe (cmd_pipe):**
    - *Purpose:* allows the main process to send initialization commands or configuration (such as the path to the movement script) to the drone process.
    - *Direction:* Main Process -> Drone Process
    - *Usage in code:*
      - the main process **writes** to the command pipe (cmd_pipe[1])
      - the drone process **reads** from the command pipe (cmd_pipe)

  **2. Position Pipe (pos_pipe)**
  - *Purpose:* enables the drone process to send its current position back to the main process at each time step.
  - *Direction:* Drone process -> Main process
  - *Usage in code:*
      - the drone process **writes** to the position pipe (pos_pipe[1])
      - the main process **reads** from the position pipe (pos_pipe)
  
  **Why Two Pipes?**
  A Unix pipe is a strictly **one-way**: data flows from the write end to the read end. To get a full two ways communication (main <-> drone), we need two pipes, one for each direction.

  **Pipe End Management**
  To avoid any accidental or unintended communication, each process closes the end it does not use:
  - *The main process closes:*
    - the *read end* of the command pipe (it only writes commands)
    - the *write end* of the position pipe (it only reads positions)
  - *The drone process closes:*
    - the *write end* of the command pipe (it only reads commands)
    - the *read end* of the position pipe (it only write positions)
  It ensures that each process only has access to the pipe ends it needs, making communication clear and preventing **deadlocks** or **resource leaks**.

----

### US262 - Capture and Process Drone Movement

- **Objective:**  
  Accurately track and analyze each drone's movement throughout the simulation by enabling effective communication between drone processes and the main process.


- **How Drone Movement is Captured:**  
  Each drone process sequentially reads its movement script file (e.g., `droneX_movement.csv`), which contains the drone’s positions (x, y, z) at every discrete time step.

  - The main process sends the path to the movement script to each drone via the command pipe during initialization.
  - At each time step, the drone process reads the next position from its script and sends this current position to the main process through the position pipe (usually via `stdout` redirection).


- **How the Main Process Processes Positions:**  
  The main process listens to all drones by reading from their respective position pipes at every simulation time step:

  - It reads the position updates sent by each drone process for the current time step.
  - These positions are stored in a 3d time index matrix in a two-dimensional array `positions[TIME_STEPS][MAX_DRONES]`, where:
    - `positions[t][i]` corresponds to the position of drone `i` at time step `t`.
  - This matrix allows the main process to maintain a complete temporal record of every drone’s position throughout the simulation.


- **Why Store All Positions Temporally?**  
  Maintaining a full history of drone positions over time enables:

  - **Collision detection:** By analyzing the positions at each time step, the system can identify collisions or close encounters.
  - **Post-simulation analysis:** Supports movement history tracking, reporting, and further data analysis after the simulation completes.
  

- **Summary:**  
  Through pipes, drones continuously communicate their real-time position updates to the main process, which centrally records and manages this data for synchronization, safety checks, and analytical purposes.

----

### US263 - Detect and Handle Drone Collisions in Real Time

- **What is a Collision?**  
  A collision happens when two or more drones occupy overlapping space at the same time step. Since drones are not points but have a physical size, each drone is represented as a 3D bounding box.


- **How Collision Detection Works**  
  At every simulation time step, the main process:
  - Reads the current positions of all drones.
  - Calculates each drone’s 3D bounding box based on its coordinates and size.
  - Checks for intersections between every pair of drones’ bounding boxes to identify collisions.


- **How Collision Handling is Implemented**  
  When a collision is detected:
  - The system logs detailed information: time step, drone IDs involved, and their positions.
  - It sends a `SIGUSR1` signal to each involved drone process, triggering their asynchronous collision handlers.
  - The main process keeps track of the total number of collisions.
  - If the total collisions surpass a predefined threshold, the system:
    - Initiates a graceful shutdown by sending `SIGTERM` to all drone processes.
    - If needed, follows up with `SIGKILL` to forcibly stop any remaining processes.
  - This ensures a safe and controlled stop to the simulation when too many collisions occur.

----
### US264 - Synchronize drone execution with a time step

- **Objective:**  
  Ensure that drone movements in the simulation are synchronized by discrete time steps, so the simulation accurately represents real-world concurrent execution.


- **Synchronization using pipes:**  
  At each time step, the main process waits to receive a position update from every drone process through its dedicated pipe (`pos_pipe`). The main process blocks on each pipe until it receives the position from each drone, which ensures that no drone advances to the next step until all have sent their positions.

  By using the blocking nature of pipes, the simulation achieves precise synchronization of all drone processes at every time step.


- **How synchronization works:**  
  The main simulation loop iterates over a fixed number of time steps (`TIME_STEPS`), ensuring that all drone position updates for each step are fully received and processed before moving to the next step.


- **At each iteration (`t`):**

  1. **Position updates from drones:**  
     Each drone process sends its current position through a dedicated pipe. The main process reads from these pipes for *all* drones before advancing.

  2. **Blocking reads ensure synchronization:**  
     The main process blocks and waits to receive the position update of *each* drone. This guarantees that positions correspond to the *same* time step.

  3. **Data storage:**  
     The main process stores the received positions in a 2D array indexed by `[time_step][drone_id]`. This creates a complete temporal history of all drone positions.

  4. **Collision detection:**  
     After receiving and storing all positions for the current time step, the main process performs collision checks based on the synchronized data.

  5. **Step advancement:**  
     Only after processing the current time step’s data does the simulation loop advance to the next time step (`t + 1`).


----
  ### US265 - Generate a simulation report

  In order to check the simulation results and understand if the figure is a valid one or if it cannot be used in the show, there is a need to generate a report. 
  
  - For this purpose, the report.c module is responsible for generating and displaying the simulation report. The report provides a clear overview of the simulation results, including the following information:

  *Drones:* The number of drones used in the simulation and their execution status.

  *Time Step:* The total time steps used in the simulation.
  
  *Collisions:* The number of collisions that occurred in the simulation with the drones that collided aswell as the time step it happened.
  
  *Valid or Invalid:* The figure is considered *valid* if the number of collisions does not suprass the collision threshold. If the threshold is exceeded, the figure is marked as *invalid* and cannot be used in the show.

Diagram:

class MainProcess
class Drone
class CommandPipe
class PositionPipe
class ConfigFile
class MovementScript
class ReportFile

MainProcess --|> Drone : forks
MainProcess --> CommandPipe : writes commands
MainProcess --> PositionPipe : reads positions
MainProcess --> ConfigFile : reads configuration
MainProcess --> ReportFile : generates report

Drone --> CommandPipe : reads commands
Drone --> PositionPipe : writes positions
Drone --> MovementScript : executes script

MainProcess -- Drone : SIGUSRs1 / SIGTERM
Drone -- MainProcess : SIGCHLD

Auto-avaliação:

Carolina Sá - 40%
Diogo Correia - 30%
Nilsa Gil - 30%