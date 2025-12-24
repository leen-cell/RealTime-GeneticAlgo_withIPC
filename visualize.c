#include <GL/freeglut.h>
#include <math.h>
#include <stdio.h>
#include "visualize.h"
#include "graph.h"

// ---------- Settings ----------
#define CELL_SIZE   1.0f
#define FLOOR_GAP   1.4f   // vertical spacing between z-levels
#define ROBOT_R     0.18f  // sphere radius
#define STEP_MS     30     // timer tick (ms)

static Chromosome *team = NULL;
static int nrobots = 0;

// animation state
static int step_index[8];        // current move index per robot
static float step_t[8];          // 0..1 interpolation between step and step+1
static int paused = 0;
static float speed = 0.06f;      // how fast step_t increases per tick

// camera
static float camYaw = 45.0f;
static float camPitch = 35.0f;
static float camDist = 14.0f;
static int lastX, lastY;
static int dragging = 0;

static float colors[8][3] = {
    {1,0.2f,0.2f}, {0.2f,1,0.2f}, {0.2f,0.4f,1},
    {1,1,0.2f}, {1,0.2f,1}, {0.2f,1,1},
    {1,0.6f,0.2f}, {0.7f,0.7f,0.7f}
};

static Point pos_after_k_moves(const Chromosome *c, int k) {
    Point p = c->start;
    if (k > c->length) k = c->length;
    for (int i = 0; i < k; i++) p = apply_move(p, c->moves[i]);
    return p;
}

static void cell_to_world(int x, int y, int z, float *wx, float *wy, float *wz) {
    // Center map around origin
    float ox = - (size_x * CELL_SIZE) * 0.5f;
    float oz = - (size_y * CELL_SIZE) * 0.5f;

    *wx = ox + (x + 0.5f) * CELL_SIZE;
    *wy = (float)z * FLOOR_GAP;
    *wz = oz + (y + 0.5f) * CELL_SIZE;
}

static void draw_cube(float x, float y, float z, float sx, float sy, float sz) {
    glPushMatrix();
    glTranslatef(x, y, z);
    glScalef(sx, sy, sz);
    glutSolidCube(1.0);
    glPopMatrix();
}

static void draw_grid_floor(int z) {
    // faint floor plane per level
    glColor4f(0.95f, 0.95f, 0.95f, 1.0f);
    glBegin(GL_QUADS);
        float x0 = -(size_x * CELL_SIZE) * 0.5f;
        float z0 = -(size_y * CELL_SIZE) * 0.5f;
        float x1 =  (size_x * CELL_SIZE) * 0.5f;
        float z1 =  (size_y * CELL_SIZE) * 0.5f;
        glVertex3f(x0, z * FLOOR_GAP - 0.02f, z0);
        glVertex3f(x1, z * FLOOR_GAP - 0.02f, z0);
        glVertex3f(x1, z * FLOOR_GAP - 0.02f, z1);
        glVertex3f(x0, z * FLOOR_GAP - 0.02f, z1);
    glEnd();

    // grid lines
    glColor3f(0.82f, 0.82f, 0.82f);
    glBegin(GL_LINES);
    for (int x = 0; x <= size_x; x++) {
        float wx0 = -(size_x * CELL_SIZE) * 0.5f + x * CELL_SIZE;
        float wz0 = -(size_y * CELL_SIZE) * 0.5f;
        float wz1 =  (size_y * CELL_SIZE) * 0.5f;
        glVertex3f(wx0, z * FLOOR_GAP - 0.01f, wz0);
        glVertex3f(wx0, z * FLOOR_GAP - 0.01f, wz1);
    }
    for (int y = 0; y <= size_y; y++) {
        float wz0 = -(size_y * CELL_SIZE) * 0.5f + y * CELL_SIZE;
        float wx0 = -(size_x * CELL_SIZE) * 0.5f;
        float wx1 =  (size_x * CELL_SIZE) * 0.5f;
        glVertex3f(wx0, z * FLOOR_GAP - 0.01f, wz0);
        glVertex3f(wx1, z * FLOOR_GAP - 0.01f, wz0);
    }
    glEnd();
}

static void draw_map_cells() {
    for (int z = 0; z < size_z; z++) {
        draw_grid_floor(z);

        for (int y = 0; y < size_y; y++) {
            for (int x = 0; x < size_x; x++) {
                int cell = grid[z][y][x];
                if (cell == 0) continue;

                float wx, wy, wz;
                cell_to_world(x, y, z, &wx, &wy, &wz);

                if (cell == 1) {        // obstacle
                    glColor3f(0.15f, 0.15f, 0.15f);
                    draw_cube(wx, wy + 0.25f, wz, 0.95f, 0.5f, 0.95f);
                } else if (cell == 2) { // survivor
                    glColor3f(0.1f, 0.9f, 0.2f);
                    draw_cube(wx, wy + 0.20f, wz, 0.70f, 0.40f, 0.70f);
                } else if (cell == 3) { // risk
                    glColor3f(1.0f, 0.45f, 0.05f);
                    draw_cube(wx, wy + 0.20f, wz, 0.70f, 0.40f, 0.70f);
                }
            }
        }
    }
}

static void draw_robot_path_lines() {
    // draw full GA path as a 3D polyline per robot (thin)
    for (int r = 0; r < nrobots; r++) {
        glColor3fv(colors[r % 8]);
        glLineWidth(2.0f);

        glBegin(GL_LINE_STRIP);
        Point p = team[r].start;

        float wx, wy, wz;
        cell_to_world(p.x, p.y, p.z, &wx, &wy, &wz);
        glVertex3f(wx, wy + 0.12f, wz);

        for (int i = 0; i < team[r].length; i++) {
            p = apply_move(p, team[r].moves[i]);
            cell_to_world(p.x, p.y, p.z, &wx, &wy, &wz);
            glVertex3f(wx, wy + 0.12f, wz);
        }
        glEnd();
    }
}

static void draw_robots_moving() {
    for (int r = 0; r < nrobots; r++) {
        int k = step_index[r];
        if (k < 0) k = 0;
        if (k > team[r].length) k = team[r].length;

        Point a = pos_after_k_moves(&team[r], k);
        Point b = pos_after_k_moves(&team[r], k + 1);

        float ax, ay, az, bx, by, bz;
        cell_to_world(a.x, a.y, a.z, &ax, &ay, &az);
        cell_to_world(b.x, b.y, b.z, &bx, &by, &bz);

        float t = step_t[r];
        float rx = ax + (bx - ax) * t;
        float ry = ay + (by - ay) * t + 0.18f;
        float rz = az + (bz - az) * t;

        glColor3fv(colors[r % 8]);
        glPushMatrix();
        glTranslatef(rx, ry, rz);
        glutSolidSphere(ROBOT_R, 18, 18);
        glPopMatrix();
    }
}

static void setup_lighting() {
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_COLOR_MATERIAL);
    glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);

    float pos[] = { 6.0f, 10.0f, 6.0f, 1.0f };
    glLightfv(GL_LIGHT0, GL_POSITION, pos);

    float amb[] = { 0.35f, 0.35f, 0.35f, 1.0f };
    glLightModelfv(GL_LIGHT_MODEL_AMBIENT, amb);

    glEnable(GL_DEPTH_TEST);
}

static void display() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    // camera (orbit)
    float radYaw = camYaw * (float)M_PI / 180.0f;
    float radPitch = camPitch * (float)M_PI / 180.0f;

    float cx = camDist * cosf(radPitch) * cosf(radYaw);
    float cy = camDist * sinf(radPitch);
    float cz = camDist * cosf(radPitch) * sinf(radYaw);

    gluLookAt(cx, cy, cz, 0, 0.5f*(size_z-1)*FLOOR_GAP, 0, 0, 1, 0);

    draw_map_cells();
    draw_robot_path_lines();
    draw_robots_moving();

    glutSwapBuffers();
}

static void timer(int v) {
    if (!paused) {
        for (int r = 0; r < nrobots; r++) {
            if (step_index[r] >= team[r].length) continue;

            step_t[r] += speed;
            if (step_t[r] >= 1.0f) {
                step_t[r] = 0.0f;
                step_index[r]++;

                if (step_index[r] > team[r].length)
                    step_index[r] = team[r].length;
            }
        }
    }

    glutPostRedisplay();
    glutTimerFunc(STEP_MS, timer, 0);
}

static void reshape(int w, int h) {
    if (h == 0) h = 1;
    glViewport(0, 0, w, h);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(60.0, (double)w / (double)h, 0.1, 200.0);
}

static void keyboard(unsigned char key, int x, int y) {
    (void)x; (void)y;

    switch (key) {
        case 27: // ESC
            exit(0);
        case ' ':
            paused = !paused;
            break;
        case '+':
        case '=':
            speed *= 1.15f;
            if (speed > 0.5f) speed = 0.5f;
            break;
        case '-':
        case '_':
            speed /= 1.15f;
            if (speed < 0.005f) speed = 0.005f;
            break;
        case 'r':
        case 'R':
            for (int i = 0; i < nrobots; i++) { step_index[i] = 0; step_t[i] = 0; }
            break;
        case 'w':
            camPitch += 3.0f; if (camPitch > 85.0f) camPitch = 85.0f;
            break;
        case 's':
            camPitch -= 3.0f; if (camPitch < -10.0f) camPitch = -10.0f;
            break;
        case 'a':
            camYaw -= 4.0f;
            break;
        case 'd':
            camYaw += 4.0f;
            break;
        case 'q':
            camDist -= 0.7f; if (camDist < 4.0f) camDist = 4.0f;
            break;
        case 'e':
            camDist += 0.7f; if (camDist > 60.0f) camDist = 60.0f;
            break;
    }
}

static void mouse(int btn, int state, int x, int y) {
    if (btn == GLUT_LEFT_BUTTON) {
        dragging = (state == GLUT_DOWN);
        lastX = x; lastY = y;
    }
}

static void motion(int x, int y) {
    if (!dragging) return;
    int dx = x - lastX;
    int dy = y - lastY;
    lastX = x; lastY = y;

    camYaw += dx * 0.4f;
    camPitch += dy * 0.3f;
    if (camPitch > 85.0f) camPitch = 85.0f;
    if (camPitch < -10.0f) camPitch = -10.0f;
}

void visualize_paths_3d(Chromosome robots[], int num_robots) {
    team = robots;
    nrobots = num_robots;
    if (nrobots > 8) nrobots = 8;

    for (int i = 0; i < nrobots; i++) { step_index[i] = 0; step_t[i] = 0.0f; }

    int argc = 1;
    char *argv[] = { (char*)"rescue_viz3d", NULL };
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(1100, 750);
    glutCreateWindow("Rescue Robots - 3D GA Paths (Animated)");

    glClearColor(0.98f, 0.98f, 1.0f, 1.0f);
    setup_lighting();

    glutDisplayFunc(display);
    glutReshapeFunc(reshape);
    glutKeyboardFunc(keyboard);
    glutMouseFunc(mouse);
    glutMotionFunc(motion);
    glutTimerFunc(STEP_MS, timer, 0);

    printf("\n3D Controls:\n");
    printf("  Mouse drag: rotate camera\n");
    printf("  W/S: pitch  A/D: yaw  Q/E: zoom\n");
    printf("  Space: pause/resume   +/-: speed   R: restart   ESC: quit\n\n");

    glutMainLoop();
}
