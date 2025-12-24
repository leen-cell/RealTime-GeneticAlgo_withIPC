//genetic.c
//note i did not use the weights file i did not see the need for it 
//we can discuss it later if needed

#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include "genetic.h"
#include "multi.h" 
#include <limits.h>    
#include <string.h>    

// External variables from main.c (loaded from config.txt)
extern int POPULATION_SIZE;
extern int MAX_GENERATIONS;
extern double ELITE_PERCENT;
extern double MUTATION_RATE;   // 10% chance
extern double INJECT_PERCENT;  // percentage of new random paths per generation

Chromosome* genetic_algorithm() {
    
    //start by creating the intial population
    Chromosome* population = create_new_population();

    //evaluate fitness of each individual in the population (via robots)
    for(int i = 0; i < POPULATION_SIZE; i++){
        population[i].fitness = evaluate_fitness(&population[i]);
    }

    int elite_count = (int)(POPULATION_SIZE * ELITE_PERCENT);
    if (elite_count < 1) elite_count = 1;

    Chromosome* new_population = malloc(sizeof(Chromosome) * POPULATION_SIZE);
    if (!new_population) {
        perror("malloc");
        exit(1);
    }

    for (int gen = 0; gen < MAX_GENERATIONS; gen++) {

        // Sort population by fitness (descending)
        sort_population(population);

        if (gen == 0 || gen == MAX_GENERATIONS - 1 || (gen + 1) % 50 == 0) {
        	printf("Generation %d | Best fitness = %.2f\n", gen + 1, population[0].fitness);
        }

        // Copy elites directly to new population
        for (int i = 0; i < elite_count; i++){
            new_population[i] = population[i];
        }

        // Fill the rest of the population using crossover + mutation
        for (int i = elite_count; i < POPULATION_SIZE; i++) {

            double r = (double)rand() / RAND_MAX;

            if (r < INJECT_PERCENT) {
                // inject new exploratory path
                Chromosome fresh = create_valid_individual();
                fresh.fitness = evaluate_fitness(&fresh);
                new_population[i] = fresh;
                continue;
            }

            // Select parents
            Chromosome* parents = select_parents(population);

            // Crossover
            Chromosome child = crossover(parents[0], parents[1]);

            // Mutation
            mutate(&child);

            // Evaluate fitness (via IPC)
            child.fitness = evaluate_fitness(&child);

            new_population[i] = child;

            free(parents);
        }

        // Replace old population with new one
        Chromosome* temp = population;      
        population = new_population;
        new_population = temp;
    }

        return population;
}

Chromosome* create_new_population(){
    Chromosome* population = malloc(sizeof(Chromosome) * POPULATION_SIZE);
    if (!population) {
        perror("malloc");
        exit(1);
    }

    for (int i = 0; i < POPULATION_SIZE; i++) {
        population[i] = create_valid_individual();
    }

    return population;
}

Chromosome create_valid_individual() {
    Chromosome c;
    c.length = MAX_PATH_LENGTH;
    c.fitness = 0.0;

    c.moves = malloc(sizeof(Move) * c.length);
    if (!c.moves) { perror("malloc"); exit(1); }

    // random start position on highest floor
    Point pos;
    pos.z = size_z - 1;

    do {
        pos.x = rand() % size_x;
        pos.y = rand() % size_y;
    } while (grid[pos.z][pos.y][pos.x] == 1);

    c.start = pos;

    for (int i = 0; i < c.length; i++) {
        int valid = 0;
        Move chosen = MOVE_POS_X;

        for (int attempt = 0; attempt < 20 && !valid; attempt++) {
            Move m = rand() % 6;
            Point next = apply_move(pos, m);

            if (next.x >= 0 && next.x < size_x &&
                next.y >= 0 && next.y < size_y &&
                next.z >= 0 && next.z < size_z &&
                grid[next.z][next.y][next.x] != 1) {

                valid = 1;
                chosen = m;
                pos = next;
            }
        }

        if (!valid) {
            c.length = i;
            break;
        }

        c.moves[i] = chosen;
       	
	if (i + 1 >= size_x * size_y * size_z) {
    		c.length = i + 1;
    		break;
	}

        // stop if robot finds survivor (only if discovered)
        if (grid[pos.z][pos.y][pos.x] == 2) {   
        	c.length = i + 1;
        	break;
        }
    }

    return c;
}

// fitness computed by the robot via IPC
double evaluate_fitness(Chromosome *c) {
    return robot_evaluate_fitness(c->moves, c->length, c->start);
}

void sort_population(Chromosome* population) {
    for (int i = 0; i < POPULATION_SIZE - 1; i++) {
        for (int j = i + 1; j < POPULATION_SIZE; j++) {
            if (population[j].fitness > population[i].fitness) {
                Chromosome temp = population[i];
                population[i] = population[j];
                population[j] = temp;
            }
        }
    }
}

int paths_are_identical(Chromosome a, Chromosome b) {
    if (a.length != b.length) return 0;

    for (int i = 0; i < a.length; i++) {
        if (a.moves[i] != b.moves[i]) {
            return 0;
        }
    }

    return 1;
}

Chromosome* select_parents(Chromosome* population) {
    Chromosome* parents = malloc(sizeof(Chromosome) * 2);
    if (!parents) {
        perror("malloc");
        exit(1);
    }

    
    sort_population(population);

    // Tournament size 
    int K = POPULATION_SIZE / 5;   // top 20%
    if (K < 2) K = 2;

    // --- Select parent 1 ---
    int p1_idx = rand() % K;
    parents[0] = population[p1_idx];

    // --- Select parent 2 (different & tolerant) ---
    int p2_idx;
    int attempts = 0;

    do {
        p2_idx = rand() % K;
        attempts++;
    } while (paths_are_identical(population[p2_idx], parents[0]) &&
             attempts < 10);

    // Fallback safety
    if (paths_are_identical(population[p2_idx], parents[0])) {
        p2_idx = (p1_idx + 1) % K;
    }

    parents[1] = population[p2_idx];

    return parents;
}


Chromosome crossover(Chromosome p1, Chromosome p2) {

    Chromosome child;

    int min_len = (p1.length < p2.length) ? p1.length : p2.length;
    if (min_len < 2) {
        child = p1;
        child.moves = malloc(sizeof(Move) * child.length);
        if (!child.moves) { perror("malloc"); exit(1); }
        for (int i = 0; i < child.length; i++)
            child.moves[i] = p1.moves[i];
        child.fitness = 0.0;
        return child;
    }

    child.length = min_len;
    child.fitness = 0.0;

    child.moves = malloc(sizeof(Move) * min_len);
    if (!child.moves) { perror("malloc"); exit(1); }

    int cut = rand() % (min_len - 1);

    for (int i = 0; i <= cut; i++)
        child.moves[i] = p1.moves[i];

    for (int i = cut + 1; i < min_len; i++)
        child.moves[i] = p2.moves[i];

    return child;
}

void mutate(Chromosome* c) {
    if (c->length <= 2) return;

    double r = (double)rand() / RAND_MAX;
    if (r > MUTATION_RATE)
        return;

    int idx = rand() % c->length;
    c->moves[idx] = rand() % 6;
}

// Helper: opposite move to trace back
static Move opposite_move(Move m) {
    switch (m) {
        case MOVE_POS_X: return MOVE_NEG_X;
        case MOVE_NEG_X: return MOVE_POS_X;
        case MOVE_POS_Y: return MOVE_NEG_Y;
        case MOVE_NEG_Y: return MOVE_POS_Y;
        case MOVE_POS_Z: return MOVE_NEG_Z;
        case MOVE_NEG_Z: return MOVE_POS_Z;
        default: return m;
    }
}

// A* pathfinding to nearest survivor 
Chromosome create_path_with_astar(Point forced_start) {
    Chromosome c;
    c.length = 0;
    c.fitness = 0.0;
    c.start = forced_start;
    c.moves = malloc(sizeof(Move) * MAX_PATH_LENGTH);
    if (!c.moves) { perror("malloc"); exit(1); }

    // Find survivor with highest estimated fitness potential
    Point target = {-1, -1, -1};
    int max_potential = -1;

    for (int z = 0; z < size_z; z++) {
        for (int y = 0; y < size_y; y++) {
            for (int x = 0; x < size_x; x++) {
                if (grid[z][y][x] == 2) {  // survivor
                    int dist = abs(x - forced_start.x) + abs(y - forced_start.y) + abs(z - forced_start.z);
                    // Estimate fitness: survivor bonus + coverage gain - length penalty
                    int potential = 6 * 1 + 2 * dist - 1 * dist;  // simplified: 6 + dist
                    if (potential > max_potential) {
                        max_potential = potential;
                        target = (Point){x, y, z};
                    }
                }
            }
        }
    }

    // If no survivor found, choose farthest free cell for maximum exploration
    if (target.x == -1) {
        int max_dist = -1;
        for (int z = 0; z < size_z; z++) {
            for (int y = 0; y < size_y; y++) {
                for (int x = 0; x < size_x; x++) {
                    if (grid[z][y][x] != 1) {  // free cell
                        int dist = abs(x - forced_start.x) + abs(y - forced_start.y) + abs(z - forced_start.z);
                        if (dist > max_dist) {
                            max_dist = dist;
                            target = (Point){x, y, z};
                        }
                    }
                }
            }
        }
    }

    // A* implementation (6 directions, Manhattan heuristic)
    typedef struct {
        Point pos;
        int g, h, f;
        Move came_from_move;
    } Node;

    Node open[MAX_PATH_LENGTH];
    int open_count = 0;

    int closed[size_z][size_y][size_x];
    memset(closed, 0, sizeof(closed));

    Move parent_move[size_z][size_y][size_x];
    memset(parent_move, 0, sizeof(parent_move));

    // Start node
    open[0].pos = forced_start;
    open[0].g = 0;
    open[0].h = abs(target.x - forced_start.x) + abs(target.y - forced_start.y) + abs(target.z - forced_start.z);
    open[0].f = open[0].g + open[0].h;
    open[0].came_from_move = 0;
    open_count = 1;

    Move directions[6] = {MOVE_POS_X, MOVE_NEG_X, MOVE_POS_Y, MOVE_NEG_Y, MOVE_POS_Z, MOVE_NEG_Z};
    Point deltas[6] = {{1,0,0}, {-1,0,0}, {0,1,0}, {0,-1,0}, {0,0,1}, {0,0,-1}};

    int found = 0;

    while (open_count > 0 && c.length < MAX_PATH_LENGTH - 1) {
        // Find lowest f node
        int current_idx = 0;
        for (int i = 1; i < open_count; i++) {
            if (open[i].f < open[current_idx].f) current_idx = i;
        }

        Point curr = open[current_idx].pos;

        // Remove from open
        open[current_idx] = open[--open_count];

        if (curr.x == target.x && curr.y == target.y && curr.z == target.z) {
            found = 1;
            break;
        }

        closed[curr.z][curr.y][curr.x] = 1;

        for (int d = 0; d < 6; d++) {
            Point next = {curr.x + deltas[d].x, curr.y + deltas[d].y, curr.z + deltas[d].z};

            if (next.x < 0 || next.x >= size_x || next.y < 0 || next.y >= size_y ||
                next.z < 0 || next.z >= size_z || grid[next.z][next.y][next.x] == 1 ||
                closed[next.z][next.y][next.x]) {
                continue;
            }

            int tentative_g = open[current_idx].g + 1;

            int in_open = -1;
            for (int i = 0; i < open_count; i++) {
                if (open[i].pos.x == next.x && open[i].pos.y == next.y && open[i].pos.z == next.z) {
                    in_open = i;
                    break;
                }
            }

            if (in_open == -1 || tentative_g < open[in_open].g) {
                Node new_node;
                new_node.pos = next;
                new_node.g = tentative_g;
                new_node.h = abs(next.x - target.x) + abs(next.y - target.y) + abs(next.z - target.z);
                new_node.f = new_node.g + new_node.h;
                new_node.came_from_move = directions[d];

                if (in_open == -1) {
                    open[open_count++] = new_node;
                } else {
                    open[in_open] = new_node;
                }

                parent_move[next.z][next.y][next.x] = directions[d];
            }
        }
    }

    // Reconstruct path if goal reached or open exhausted
    if (found || open_count > 0) {
        Point current = found ? target : open[0].pos;  // go as far as possible if no path to goal
        while (!(current.x == forced_start.x && current.y == forced_start.y && current.z == forced_start.z)) {
            Move m = parent_move[current.z][current.y][current.x];
            if (m == 0 && c.length > 0) break;  // safety
            c.moves[c.length++] = m;
            current = apply_move(current, opposite_move(m));
        }

        // Reverse moves to go from start to goal
        for (int i = 0; i < c.length / 2; i++) {
            Move temp = c.moves[i];
            c.moves[i] = c.moves[c.length - 1 - i];
            c.moves[c.length - 1 - i] = temp;
        }
    }

    return c;
}

int is_free_cell(int x, int y, int z) {
    return (x >= 0 && x < size_x &&
            y >= 0 && y < size_y &&
            z >= 0 && z < size_z &&
            grid[z][y][x] != 1);
}

Point apply_move(Point p, Move m) {
    Point r = p;

    switch (m) {
        case MOVE_POS_X: r.x++; break;
        case MOVE_NEG_X: r.x--; break;
        case MOVE_POS_Y: r.y++; break;
        case MOVE_NEG_Y: r.y--; break;
        case MOVE_POS_Z: r.z++; break;
        case MOVE_NEG_Z: r.z--; break;
    }
    return r;
}

// Collision detection between the 8 best paths
CollisionReport detect_collisions(Chromosome team[8]) {
    CollisionReport report = {0, 0, 0};
    
    int max_length = 0;
    for (int i = 0; i < 8; i++) {
        if (team[i].length > max_length) max_length = team[i].length;
    }

    // Temporal collisions: same time step, same cell
    for (int t = 0; t < max_length; t++) {
        Point positions[8];
        int valid[8] = {0};
        
        for (int r = 0; r < 8; r++) {
            if (t < team[r].length) {
                Point pos = team[r].start;
                for (int m = 0; m < t; m++) {
                    pos = apply_move(pos, team[r].moves[m]);
                }
                positions[r] = pos;
                valid[r] = 1;
            }
        }
        
        for (int i = 0; i < 8; i++) {
            if (!valid[i]) continue;
            for (int j = i+1; j < 8; j++) {
                if (!valid[j]) continue;
                if (positions[i].x == positions[j].x &&
                    positions[i].y == positions[j].y &&
                    positions[i].z == positions[j].z) {
                    report.total_temporal_collisions++;
                }
            }
        }
    }

    // Spatial collisions: any cell visited by more than one robot
    int visited[8][size_z][size_y][size_x];
    memset(visited, 0, sizeof(visited));
    
    for (int r = 0; r < 8; r++) {
        Point pos = team[r].start;
        visited[r][pos.z][pos.y][pos.x] = 1;
        
        for (int i = 0; i < team[r].length; i++) {
            pos = apply_move(pos, team[r].moves[i]);
            visited[r][pos.z][pos.y][pos.x] = 1;
        }
    }
    
    for (int z = 0; z < size_z; z++) {
        for (int y = 0; y < size_y; y++) {
            for (int x = 0; x < size_x; x++) {
                int count = 0;
                for (int r = 0; r < 8; r++) {
                    if (visited[r][z][y][x]) count++;
                }
                if (count > 1) {
                    report.total_spatial_collisions += (count - 1);
                    report.conflicted_cells_count++;
                }
            }
        }
    }
    
    return report;
}

// Team fitness with collision penalty
double evaluate_team_fitness(Chromosome team[8]) {
    double total_survivors = 0, total_coverage = 0, total_length = 0, total_risk = 0;
    
    for (int i = 0; i < 8; i++) {
        Point pos = team[i].start;
        char visited[size_z][size_y][size_x];
        memset(visited, 0, sizeof(visited));
        
        total_coverage++;
        visited[pos.z][pos.y][pos.x] = 1;
        
        if (grid[pos.z][pos.y][pos.x] == 2) total_survivors++;
        
        for (int m = 0; m < team[i].length; m++) {
            pos = apply_move(pos, team[i].moves[m]);
            total_length++;
            
            if (grid[pos.z][pos.y][pos.x] == 2) total_survivors++;
            if (grid[pos.z][pos.y][pos.x] == 3) total_risk++;
            
            if (!visited[pos.z][pos.y][pos.x]) {
                total_coverage++;
                visited[pos.z][pos.y][pos.x] = 1;
            }
        }
    }
    
    CollisionReport collisions = detect_collisions(team);
    double collision_penalty = 50.0 * (collisions.total_temporal_collisions * 10 + 
                                      collisions.total_spatial_collisions);
    
    return 8 * total_survivors + 2 * total_coverage - total_length - 5 * total_risk - collision_penalty;
}