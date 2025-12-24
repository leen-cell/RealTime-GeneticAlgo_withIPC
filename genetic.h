//genetic.h
#ifndef GENETIC_H
#define GENETIC_H
#include "graph.h"

#define MAX_PATH_LIMIT 2000   // upper bound for paths stored in shared memory

// External configuration variables (defined in main.c, loaded from config.txt)
extern int POPULATION_SIZE;
extern int MAX_GENERATIONS;
extern double ELITE_PERCENT;
extern double MUTATION_RATE;
extern double INJECT_PERCENT;

// Fitness weights (external from main.c)
extern double W_SURVIVORS;
extern double W_COVERAGE;
extern double W_LENGTH;
extern double W_RISK;
typedef enum {
    MOVE_POS_X, MOVE_NEG_X,
    MOVE_POS_Y, MOVE_NEG_Y,
    MOVE_POS_Z, MOVE_NEG_Z
} Move;

typedef struct {
    Move* moves;    
    int length;
    double fitness;
        Point start;
} Chromosome;

typedef struct {
    int total_spatial_collisions;
    int total_temporal_collisions;
    int conflicted_cells_count;
} CollisionReport;

CollisionReport detect_collisions(Chromosome team[8]);
double evaluate_team_fitness(Chromosome team[8]);
Chromosome* genetic_algorithm();
Chromosome* create_new_population();
Chromosome create_valid_individual();
Chromosome create_path_with_astar();
double evaluate_fitness(Chromosome* c);
Chromosome* select_parents(Chromosome* population);
Chromosome crossover(Chromosome p1, Chromosome p2);
void mutate(Chromosome* c);
int is_free_cell(int x, int y, int z);
int paths_are_identical(Chromosome a, Chromosome b);
void sort_population(Chromosome* population);
Point apply_move(Point p, Move m);
static Move opposite_move(Move m);
//static Chromosome clone_chromosome(const Chromosome *src);
////static void free_chromosome(Chromosome *c);


#endif