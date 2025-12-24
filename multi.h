//multi.h file
#ifndef MULTI_H
#define MULTI_H


#include <unistd.h>     // fork, _exit, getpid
#include <signal.h>     // SIGTERM, kill
#include <sys/types.h>  // pid_t
#include <time.h>       // time_t
#include <string.h>     // memset
#include <stdio.h>      // perror, printf
#include <stdlib.h>     // malloc, realloc, free, exit

#include <sys/ipc.h>    
#include <sys/shm.h>    // shmget, shmat, shmctl
#include <sys/sem.h>    // semget, semop, semctl   
#include "genetic.h"    // for Move, MAX_PATH_LENGTH, POPULATION_SIZE
#include "graph.h"      // for size_x, size_y, size_z, grid
#include <sys/wait.h>

//if we need less then the unused are removed using aging
//if we need more then all robot will do more than 1 task 
//if we need exactly 8 then all are used
#define MAX_ROBOTS 8



typedef struct
{
    pid_t pid;        // PID of the child process
    time_t last_used; // last time the process was used
    int busy;         // 0 = free, 1 = working
    int robot_id;     // 0..MAX_ROBOTS-1: which slot in shared state
} ChildProcess;

pid_t create_child();
int get_free_child();
void release_child(pid_t);
void aging(int);

// Initialize / shutdown the robot pool + shared memory
void init_robot_pool(int num_robots);
void shutdown_robot_pool(void);

//  get best result per robot
Chromosome get_best_for_robot(int robot_id);

extern double W_SURVIVORS;
extern double W_COVERAGE;
extern double W_LENGTH;
extern double W_RISK;
extern int child_count;
extern int IS_CHILD;
extern ChildProcess *child_pool;
double robot_evaluate_fitness(Move *moves, int length, Point start);


#endif