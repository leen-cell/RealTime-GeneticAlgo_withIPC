// main.c
#include <stdio.h>
#include <stdlib.h>
#include <string.h>    
#include <ctype.h>    
#include <time.h>
#include "graph.h"
#include "genetic.h"
#include "multi.h"
#include "visualize.h"

// Global configuration variables
int POPULATION_SIZE = 50;
int MAX_GENERATIONS = 200;
double ELITE_PERCENT = 0.10;
double MUTATION_RATE = 0.10;
double INJECT_PERCENT = 0.30;

double W_SURVIVORS = 6.0;
double W_COVERAGE = 2.0;
double W_LENGTH = 1.0;
double W_RISK = 5.0;

int NUM_ROBOTS = 8;
char GRID_FILE[256] = "map3d.txt";  

// Function to read config file
void read_config(const char* filename) {
    FILE* file = fopen(filename, "r");
    if (!file) {
        printf("Warning: Config file '%s' not found. Using default values.\n", filename);
        return;
    }

    char line[256];
    while (fgets(line, sizeof(line), file)) {
        // Skip comments and empty lines
        if (line[0] == '#' || line[0] == '\n' || line[0] == '\r') continue;

        char key[64];
        char value_str[192];
        if (sscanf(line, "%63[^=]=%191[^\n]", key, value_str) != 2) continue;

        // Remove leading/trailing whitespace from value
        char* val_start = value_str;
        while (isspace(*val_start)) val_start++;
        char* val_end = val_start + strlen(val_start) - 1;
        while (val_end > val_start && isspace(*val_end)) *val_end-- = '\0';

        if (strcmp(key, "POPULATION_SIZE") == 0) POPULATION_SIZE = atoi(val_start);
        else if (strcmp(key, "MAX_GENERATIONS") == 0) MAX_GENERATIONS = atoi(val_start);
        else if (strcmp(key, "ELITE_PERCENT") == 0) ELITE_PERCENT = atof(val_start);
        else if (strcmp(key, "MUTATION_RATE") == 0) MUTATION_RATE = atof(val_start);
        else if (strcmp(key, "INJECT_PERCENT") == 0) INJECT_PERCENT = atof(val_start);
        else if (strcmp(key, "W_SURVIVORS") == 0) W_SURVIVORS = atof(val_start);
        else if (strcmp(key, "W_COVERAGE") == 0) W_COVERAGE = atof(val_start);
        else if (strcmp(key, "W_LENGTH") == 0) W_LENGTH = atof(val_start);
        else if (strcmp(key, "W_RISK") == 0) W_RISK = atof(val_start);
        else if (strcmp(key, "NUM_ROBOTS") == 0) NUM_ROBOTS = atoi(val_start);
        else if (strcmp(key, "GRID_FILE") == 0) strncpy(GRID_FILE, val_start, sizeof(GRID_FILE) - 1);
    }

    fclose(file);
    printf("Configuration loaded successfully from '%s'\n", filename);
}

void print_path_from_moves(Chromosome c);

int main(int argc, char* argv[])
{
    srand(time(NULL));
    read_config("config.txt");

    const char* filename = (argc < 2) ? "map3d.txt" : argv[1];
    load_3d_map(filename);

    init_robot_pool(8);

    genetic_algorithm();   // robots evaluate & store best internally
    printf("\nGenetic Algorithm completed %d generations.\n", MAX_GENERATIONS);

    printf("     FINAL RESCUE TEAM REPORT\n\n");
 

    Chromosome team[8];
    for (int i = 0; i < 8; i++) {
        team[i] = get_best_for_robot(i);
        printf("Robot %d | Fitness: %.2f | Length: %d | Start: (%d,%d,%d)\n",
               i, team[i].fitness, team[i].length,
               team[i].start.x, team[i].start.y, team[i].start.z);
        printf("  Path: ");
        print_path_from_moves(team[i]);
    }
   
    CollisionReport collisions = detect_collisions(team);
    double team_fitness = evaluate_team_fitness(team);

    printf("\n--- Collision Report ---\n");
    printf("Temporal collisions (same time, same cell): %d\n", collisions.total_temporal_collisions);
    printf("Spatial conflicts (shared cells): %d (in %d distinct cells)\n",
           collisions.total_spatial_collisions, collisions.conflicted_cells_count);

    printf("\nTeam Overall Fitness (with collision penalty): %.2f\n", team_fitness);

    if (collisions.total_temporal_collisions == 0 && collisions.total_spatial_collisions == 0) {
        printf("No collisions detected! Safe multi-robot deployment.\n");
    } else {
        printf("Collisions detected. Consider re-running with different parameters.\n");
    }

    printf("\n--- A* vs Genetic Algorithm Comparison (per robot) ---\n");

    for (int i = 0; i < 8; i++) {
        Chromosome ga_best = get_best_for_robot(i);

        // A* from same start
        Chromosome astar_path = create_path_with_astar(ga_best.start);

        // Manual fitness calculation for A* (same as GA but without shared ExplorationMap effects)
        double astar_survivors = 0, astar_coverage = 0, astar_length = 0, astar_risk = 0;
        Point a_pos = astar_path.start;
        char a_visited[size_z][size_y][size_x];
        memset(a_visited, 0, sizeof(a_visited));

        a_visited[a_pos.z][a_pos.y][a_pos.x] = 1;
        astar_coverage++;

        if (grid[a_pos.z][a_pos.y][a_pos.x] == 2) astar_survivors++;

        int a_valid = 1;
        for (int m = 0; m < astar_path.length; m++) {
            a_pos = apply_move(a_pos, astar_path.moves[m]);

            if (!is_free_cell(a_pos.x, a_pos.y, a_pos.z)) {
                a_valid = 0;
                break;
            }

            if (!a_visited[a_pos.z][a_pos.y][a_pos.x]) {
                a_visited[a_pos.z][a_pos.y][a_pos.x] = 1;
                astar_coverage++;
            }

            if (grid[a_pos.z][a_pos.y][a_pos.x] == 2) astar_survivors++;
            if (grid[a_pos.z][a_pos.y][a_pos.x] == 3) astar_risk++;

            astar_length++;
        }

        double astar_fitness = a_valid ? 
            W_SURVIVORS * astar_survivors + W_COVERAGE * astar_coverage - W_LENGTH * astar_length - W_RISK * astar_risk :
            -10000.0;

        printf("Robot %d [Start: (%d,%d,%d)]\n", i, ga_best.start.x, ga_best.start.y, ga_best.start.z);
        printf("  Genetic : Length = %3d | Fitness = %.2f\n", ga_best.length, ga_best.fitness);
        printf("  A*      : Length = %3d | Fitness = %.2f\n", astar_path.length, astar_fitness);

        double ratio;
        if (astar_fitness <= 0) {
            printf("  → A* failed to find valid path (invalid or blocked)\n");
            ratio = 0.0;
        } else {
            ratio = (ga_best.fitness / astar_fitness) * 100.0;
            printf("  → GA achieved %.1f%% of A* optimal fitness\n", ratio);
        }

        free(astar_path.moves);
    }
     printf("do you want to visualize the map? [y,n]");
    char v;
    scanf("%c",&v);
    if(v == 'y')
    {
      visualize_paths_3d(team, 8);
    }


    shutdown_robot_pool();

    // Free the 3D grid and ExplorationMap
    for (int z = 0; z < size_z; z++) {
        for (int y = 0; y < size_y; y++) {
            free(grid[z][y]);
            free(ExplorationMap[z][y]);
        }
        free(grid[z]);
        free(ExplorationMap[z]);
    }
    free(grid);
    free(ExplorationMap);
    grid = NULL;
    ExplorationMap = NULL;

    return 0;
}

void print_path_from_moves(Chromosome c)
{
    Point pos = c.start;
    printf("(%d,%d,%d)", pos.x, pos.y, pos.z);

    for (int i = 0; i < c.length; i++) {
        pos = apply_move(pos, c.moves[i]);
        printf(" -> (%d,%d,%d)", pos.x, pos.y, pos.z);
    }
    printf("\n");
}