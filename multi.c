//multi.c file
#include "multi.h"
#include <sys/ipc.h>
#include <sys/shm.h>

// Commands for robots
#define CMD_NONE    0
#define CMD_EXPLORE 1
#define CMD_EXIT    2
// round-robin index for fair robot scheduling
static int rr_index = 0;
#define SEM_START(i) (i)
#define SEM_DONE(i)  ((i) + MAX_ROBOTS)

typedef struct {
    int  cmd[MAX_ROBOTS];
    int  chromosome_index[MAX_ROBOTS];
    int  path_length[MAX_ROBOTS];

    int start_x[MAX_ROBOTS];
    int start_y[MAX_ROBOTS];
    int start_z[MAX_ROBOTS];

    Move moves[MAX_ROBOTS][MAX_PATH_LIMIT];
    double fitness[MAX_ROBOTS];

} SharedState;

static SharedState *shared = NULL;
static int shmid = -1;
static int semid  = -1;

int child_count = 0;
int IS_CHILD = 0;
ChildProcess *child_pool = NULL;

// best result per robot
static Chromosome best_per_robot[MAX_ROBOTS];
static int best_initialized[MAX_ROBOTS] = {0};

union semun {
    int val;
    struct semid_ds *buf;
    unsigned short *array;
};

static struct sembuf sem_acquire = {0, -1, SEM_UNDO};
static struct sembuf sem_release = {0,  1, SEM_UNDO};

static void robot_worker_loop(int robot_id)
{
    IS_CHILD = 1;

    while (1) {
        sem_acquire.sem_num = SEM_START(robot_id);
        semop(semid, &sem_acquire, 1);

        int cmd = shared->cmd[robot_id];
        if (cmd == CMD_EXIT) _exit(0);

        if (cmd == CMD_EXPLORE) {
            int length = shared->path_length[robot_id];
            Move *moves = shared->moves[robot_id];

            Point pos = {
                shared->start_x[robot_id],
                shared->start_y[robot_id],
                shared->start_z[robot_id]
            };

            double survivors = 0, coverage = 0, risk = 0, length_penalty = 0;

            int total = size_x * size_y * size_z;
            char *visited = calloc(total, 1);

            for (int i = 0; i < length; i++) {
                pos = apply_move(pos, moves[i]);

                if (!is_free_cell(pos.x, pos.y, pos.z)) {
                    shared->fitness[robot_id] = -10000.0;
                    goto done;
                }

                int idx = pos.z * size_y * size_x + pos.y * size_x + pos.x;
                int cell = grid[pos.z][pos.y][pos.x];

                ExplorationMap[pos.z][pos.y][pos.x] = cell;

                if (!visited[idx]) {
                    visited[idx] = 1;
                    coverage++;
                }

                if (cell == 2) {
                    survivors++;
                    break;
                }
                if (cell == 3) risk++;

                length_penalty++;
            }

	    shared->fitness[robot_id] = 
                  W_SURVIVORS * survivors +
                  W_COVERAGE * coverage -
                  W_LENGTH * length_penalty -
                  W_RISK * risk;

        done:
            free(visited);
        }

        sem_release.sem_num = SEM_DONE(robot_id);
        semop(semid, &sem_release, 1);
    }
}

double robot_evaluate_fitness(Move *moves, int length, Point start)
{
    int id = get_free_child();
    if (id < 0) return -10000.0;

    shared->cmd[id] = CMD_EXPLORE;
    shared->path_length[id] = length;

    shared->start_x[id] = start.x;
    shared->start_y[id] = start.y;
    shared->start_z[id] = start.z;

    for (int i = 0; i < length; i++)
        shared->moves[id][i] = moves[i];

    sem_release.sem_num = SEM_START(id);
    semop(semid, &sem_release, 1);

    sem_acquire.sem_num = SEM_DONE(id);
    semop(semid, &sem_acquire, 1);

    double f = shared->fitness[id];

    // store best per robot
    if (!best_initialized[id] || f > best_per_robot[id].fitness) {
        best_per_robot[id].moves = malloc(sizeof(Move) * length);
        for (int i = 0; i < length; i++)
            best_per_robot[id].moves[i] = moves[i];

        best_per_robot[id].length = length;
        best_per_robot[id].fitness = f;
        best_per_robot[id].start = start;
        best_initialized[id] = 1;
    }

    release_child(child_pool[id].pid);
    return f;
}

Chromosome get_best_for_robot(int robot_id)
{
    return best_per_robot[robot_id];
}

/* ---- pool management (unchanged logic) ---- */

pid_t create_child()
{
    child_pool = realloc(child_pool, (child_count + 1) * sizeof(ChildProcess));
    int id = child_count;

    pid_t pid = fork();
    if (pid == 0) robot_worker_loop(id);

    child_pool[id].pid = pid;
    child_pool[id].busy = 0;
    child_pool[id].robot_id = id;
    child_pool[id].last_used = time(NULL);
    child_count++;

    return pid;
}

void init_robot_pool(int num)
{
    shmid = shmget(IPC_PRIVATE, sizeof(SharedState), IPC_CREAT | 0666);
    shared = shmat(shmid, NULL, 0);
    memset(shared, 0, sizeof(SharedState));

    semid = semget(IPC_PRIVATE, 2 * MAX_ROBOTS, IPC_CREAT | 0660);
    unsigned short vals[2 * MAX_ROBOTS] = {0};
    union semun arg; arg.array = vals;
    semctl(semid, 0, SETALL, arg);

    for (int i = 0; i < num; i++) create_child();
}

void shutdown_robot_pool(void)
{
    for (int i = 0; i < child_count; i++) {
        shared->cmd[i] = CMD_EXIT;
        sem_release.sem_num = SEM_START(i);
        semop(semid, &sem_release, 1);
    }

    for (int i = 0; i < child_count; i++)
        waitpid(child_pool[i].pid, NULL, 0);

    shmdt(shared);
    shmctl(shmid, IPC_RMID, NULL);
    semctl(semid, 0, IPC_RMID);
    
    // Free best paths stored for each robot
    for (int i = 0; i < MAX_ROBOTS; i++) {
        if (best_initialized[i]) {
            free(best_per_robot[i].moves);
            best_per_robot[i].moves = NULL;
            best_initialized[i] = 0;
        }
    }

    free(child_pool);
}

int get_free_child()
{
    int checked = child_count;

    while (checked--) {
        int i = rr_index;
        rr_index = (rr_index + 1) % child_count;

        if (!child_pool[i].busy) {
            child_pool[i].busy = 1;
            child_pool[i].last_used = time(NULL);
            return i;
        }
    }

    // no free robot
    return -1;
}



void release_child(pid_t pid)
{
    for (int i = 0; i < child_count; i++)
        if (child_pool[i].pid == pid)
            child_pool[i].busy = 0;
}