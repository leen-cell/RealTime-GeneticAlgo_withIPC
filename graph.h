#ifndef GRAPH_H
#define GRAPH_H

typedef struct {
    int x, y, z;
} Point;

extern int size_x, size_y, size_z;
extern int ***grid;//0 free 1 obstacle 2 survivor 3 risk
extern int ***ExplorationMap; // and -1 for unknown
extern int MAX_PATH_LENGTH;

void load_3d_map(const char* filename);
void print_grid();

#endif