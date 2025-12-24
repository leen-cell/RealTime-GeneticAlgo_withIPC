#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "graph.h"
int MAX_PATH_LENGTH;    //it will be calculated based on the map size
int ***grid = NULL;
int size_x = 0, size_y = 0, size_z = 0;
int ***ExplorationMap = NULL; //the robot will explore and add to it this is the one will be used 


// -----------------  3D Grid (calculating the map floors)-----------------
void load_3d_map(const char* filename){
    FILE* fp = fopen(filename,"r");
    if(!fp){ perror("Cannot open map"); exit(1); }

//calculating map size
    size_z = 0; size_y = 0; size_x = 0;
    char line[1024];
    int current_level_rows = 0;

    while(fgets(line,sizeof(line),fp)){
        if(strcmp(line,"\n")==0){
            size_z++;
            if(current_level_rows>size_y) size_y = current_level_rows;
            current_level_rows = 0;
        } else current_level_rows++;
        if(size_x==0 && line[0]!='\n'){
            char* token = strtok(line," ");
            while(token){ size_x++; token=strtok(NULL," "); }
        }
    }
    if(current_level_rows>0){ size_z++; if(current_level_rows>size_y) size_y = current_level_rows; }
    rewind(fp);

//daynamic 3D
    grid = malloc(size_z * sizeof(int**));
    for(int z=0; z<size_z; z++){
        grid[z] = malloc(size_y * sizeof(int*));
        for(int y=0; y<size_y; y++)
            grid[z][y] = malloc(size_x * sizeof(int));
    }

//reading the values
    int z=0, y=0;
    while(fgets(line,sizeof(line),fp)){
        if(strcmp(line,"\n")==0){ z++; y=0; continue; }
        int x=0;
        char* token = strtok(line," ");
        while(token){
            grid[z][y][x] = atoi(token);
            token = strtok(NULL," ");
            x++;
        }
        y++;
    }

    fclose(fp);
    ////////////////////should be removed 
    // calculate path length
    MAX_PATH_LENGTH = size_x * size_y * size_z;

    // Allocate ExplorationMap with -1 = unknown
    ExplorationMap = malloc(size_z * sizeof(int**));

    for (int z = 0; z < size_z; z++) {
    ExplorationMap[z] = malloc(size_y * sizeof(int*));
    for (int y = 0; y < size_y; y++) {
        ExplorationMap[z][y] = malloc(size_x * sizeof(int));
        for (int x = 0; x < size_x; x++)
            ExplorationMap[z][y][x] = -1;  // unknown
    }
}

}


// --------------print the map -----------------
void print_grid(){
    for(int z=0; z<size_z; z++){
        printf("Level Z=%d\n",z);
        for(int y=0;y<size_y;y++){
            for(int x=0;x<size_x;x++) printf("%d ",grid[z][y][x]);
            printf("\n");
        }
        printf("\n");
    }
}