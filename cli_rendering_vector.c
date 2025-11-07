#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <time.h>
#include<unistd.h>

/* =========================================== utilities ===================================== */
void* xmalloc(size_t size)
{
    void* res = malloc(size);
    if (res == NULL)
    { 
        perror("not enouth memory");
        exit(1);
    }
    return (void*)res;
} 

double get_time_millis() {
    struct timespec ts;
    // CLOCK_MONOTONIC is used for measuring elapsed time.
    if (clock_gettime(CLOCK_MONOTONIC, &ts) != 0) {
        perror("clock_gettime failed");
        return -1.0; // Indicate error
    }
    return ((double)ts.tv_sec * 1000.0) + ((double)ts.tv_nsec / 1000000.0);
}

/* ======================================= geometric struct ================================== */

typedef struct {
    unsigned char r,g,b;
} Color;

typedef struct {
    float x, y, z;
} Vec3;

#define type_calar 0
#define type_point 1
#define type_vector 2
#define type_vector_color 3

typedef struct {
    int type;
    union
    {
        int scalar;
        Vec3 point;
        struct {
            Vec3 a;
            Vec3 b;
        } vector;
        struct {
            Vec3 a;
            Vec3 b;
            Color color;
        } vector_color;
    };
} GeometricValue;


/* ============================================ hash Map ===================================== */
#define TABLE_SIZE 359 // quite small prime 

typedef struct {
    char *string;
    int is_occupied;
} Hash_Entry;


// hash function (djb2)
unsigned int hash(const char *str) {
    unsigned long hash = 5381;
    int c;
    while ((c = *str++)) {
        hash = ((hash << 5) + hash) + c;
    }
    return hash % TABLE_SIZE;
}

// init of hash table entries
void init_table(Hash_Entry *table) {
    for (int i = 0; i < TABLE_SIZE; i++) {
        table[i].string = NULL;
        table[i].is_occupied = 0;
    }
}

typedef struct {
    Hash_Entry entries[TABLE_SIZE];
    GeometricValue values[TABLE_SIZE];
} GeometricMap;

void init_map(GeometricMap *map) {
    for (int i = 0; i < TABLE_SIZE; i++) {
        map->entries[i].string = NULL;
        map->entries[i].is_occupied = 0;
        // Also good practice to zero-out the corresponding value
        memset(&map->values[i], 0, sizeof(GeometricValue));
    }
}

void insert_entity(GeometricMap *map, const char *key, GeometricValue entity) {
    unsigned int index = hash(key);
    unsigned int original_index = index;

    while (map->entries[index].is_occupied) {
        // If the key already exists, just update the value
        if (strcmp(map->entries[index].string, key) == 0) {
            map->values[index] = entity;
            return;
        }
        index = (index + 1) % TABLE_SIZE;
        if (index == original_index) {
            printf("Hash map is full\n");
            return;
        }
    }

    // Found an empty slot, insert new entry
    map->entries[index].string = strdup(key);
    map->entries[index].is_occupied = 1;
    map->values[index] = entity;
}

GeometricValue* search_entity(GeometricMap *map, const char *key) {
    unsigned int index = hash(key);
    unsigned int original_index = index;

    while (map->entries[index].is_occupied) {
        if (strcmp(map->entries[index].string, key) == 0) {
            // Key found, return a pointer to the corresponding value
            return &map->values[index];
        }
        index = (index + 1) % TABLE_SIZE;
        if (index == original_index) {
            return NULL; // Table searched completely
        }
    }
    return NULL; // Found an empty slot, key cannot be in the table
}

/* ======================================== vector operation ================================= */
Vec3 loadVec(float x,float y,float z)
{
    Vec3 res ;
    res.x = x;
    res.y = y;
    res.z = z;
    return res;
}
Vec3 addVec3 (Vec3 * v1, Vec3* v2) {return loadVec(v1->x + v2->x,v1->y + v2->y,v1->z + v2->z);}

Vec3 scalarVec3 (float scalar , Vec3* v) {return loadVec(scalar*v->x,scalar*v->y, scalar*v->z);}

float dotProduct (Vec3 * v1, Vec3* v2) {return v1->x*v2->x+ v1->y*v2->y + v1->z*v2->z;}

Vec3 crossProduct(Vec3* v1, Vec3* v2)  
{ 
    return loadVec(
        v1->y * v2->z - v1->z * v2->y,
        v1->z * v2->x - v1->x * v2->z,
        v1->x * v2->y - v1->y * v2->x
    );
}

float magnitude(Vec3* v)
{
    return sqrtf(v->x * v->x + v->y * v->y + v->z * v->z);
}

Vec3 normalize(Vec3* v)
{
    float mag = magnitude(v);
    if (mag == 0) return loadVec(0, 0, 0);
    return scalarVec3(1.0f / mag, v);
}

Vec3 subVec3(Vec3* v1, Vec3* v2)
{
    return loadVec(v1->x - v2->x, v1->y - v2->y, v1->z - v2->z);
}

/* ======================================= Rendering Function ================================ */
// rendering function
typedef struct  {
    Vec3 position;
    Vec3 direction; // Where the camera is looking
    float fov;     // Field of view (e.g., 90.0f degrees)
} Camera;



typedef struct {
    int x, y;
} Vec2i;

Vec2i Zeros2() { Vec2i v ; v.x =0; v.y= 0 ; return v;}
Vec3 Zeros3() { Vec3 v ; v.x =0; v.y= 0; v.z= 0 ; return v;}

typedef struct {
    Vec3 v;
    Color color;
} RenderVector;

const int SCREEN_WIDTH = 40;
const int SCREEN_HEIGHT = 40;

Vec2i project(Vec3 point, Camera* cam, int width, int height) 
{
    // 1. Translate point to camera space
    Vec3 p_camera_space = subVec3(&point, &(cam->position));
    
    // 2. Build camera basis vectors
    // Forward (Z) is the camera direction
    Vec3 forward = cam->direction;
    
    // Right (X) = forward × world_up
    Vec3 world_up = loadVec(0, 1, 0);
    Vec3 right = crossProduct(&forward, &world_up);
    right = normalize(&right);
    
    // Up (Y) = right × forward
    Vec3 up = crossProduct(&right, &forward);
    up = normalize(&up);
    
    // 3. Rotate point into camera space using basis vectors
    // This transforms from world space to camera view space
    Vec3 rotated;
    rotated.x = dotProduct(&p_camera_space, &right);   // Project onto right
    rotated.y = dotProduct(&p_camera_space, &up);      // Project onto up
    rotated.z = dotProduct(&p_camera_space, &forward); // Project onto forward
    
    // 4. Perspective projection
    if (rotated.z <= 0.0001f) {
        // Point is behind or at camera, return off-screen
        Vec2i offscreen = {-9999, -9999};
        return offscreen;
    }
    
    float focal_length = (float)width / (2.0f * tanf(cam->fov * M_PI / 360.0f));
    
    float projected_x = (rotated.x * focal_length) / rotated.z;
    float projected_y = (rotated.y * focal_length) / rotated.z;
    
    // 5. Convert to screen coordinates
    Vec2i screen_coords;
    screen_coords.x = (int)((projected_x + (float)width / 2.0f));
    screen_coords.y = (int)((projected_y + (float)height / 2.0f));
    
    return screen_coords;
}


// Draws a line in the framebuffer between two 2D points
void drawLine(char fb[SCREEN_HEIGHT][SCREEN_WIDTH], Vec2i p1, Vec2i p2, char character) 
{
    int x1 = p1.x, y1 = p1.y;
    int x2 = p2.x, y2 = p2.y;

    int dx = abs(x2 - x1);
    int sx = x1 < x2 ? 1 : -1;
    int dy = -abs(y2 - y1);
    int sy = y1 < y2 ? 1 : -1;
    int err = dx + dy;
    int e2;

    while (1) {
        // Check bounds before drawing
        if (x1 >= 0 && x1 < SCREEN_WIDTH && y1 >= 0 && y1 < SCREEN_HEIGHT) {
            fb[y1][x1] = character;
        }

        if (x1 == x2 && y1 == y2) break;
        
        e2 = 2 * err;
        if (e2 >= dy) { err += dy; x1 += sx; }
        if (e2 <= dx) { err += dx; y1 += sy; }
    }
}

void init_buffer_terminal(char fb[SCREEN_HEIGHT][SCREEN_WIDTH])
{
    for (int x = 0; x < SCREEN_HEIGHT; x++ )
        {
            for (int y = 0 ; y < SCREEN_WIDTH; y++ )
            {
                fb[x][y] = '.';
            }
        }
}
void renderTerminal(char fb[SCREEN_HEIGHT][SCREEN_WIDTH])
{
    for (int x = 0; x < SCREEN_HEIGHT; x++ )
    {
        for (int y = 0 ; y < SCREEN_WIDTH; y++ )
        {
            printf(" %c" ,fb[x][y]);
        }
        printf("\n");
    }
}

void updateCamera(Camera* camera, double deltaTime, Vec3 center, Vec3 position, Vec3 rotationDirection) 
{
    // Calculate the camera position relative to center
    Vec3 cameraPos = addVec3(&center, &position);
    camera->position = cameraPos;
    
    // Calculate direction vector from camera to center (where we're looking)
    Vec3 toCenter = subVec3(&center, &cameraPos);
    camera->direction = normalize(&toCenter);
    
    // Calculate the radius vector (from center to camera)
    Vec3 radiusVec = subVec3(&cameraPos, &center);
    float radius = magnitude(&radiusVec);
    
    if (radius < 0.0001f) {
        // Camera is at the center, can't rotate
        return;
    }
    
    // Normalize the radius vector to get the normal to the rotation plane
    Vec3 normal = normalize(&radiusVec);
    
    // Project rotationDirection onto the plane perpendicular to the normal
    // projection = rotationDirection - (rotationDirection · normal) * normal
    float dotProd = dotProduct(&rotationDirection, &normal);
    Vec3 normalComponent = scalarVec3(dotProd, &normal);
    Vec3 projectedRotation = subVec3(&rotationDirection, &normalComponent);
    
    // Check if the projected vector has any magnitude
    float projMag = magnitude(&projectedRotation);
    if (projMag < 0.0001f) {
        // rotationDirection is parallel to normal, no rotation possible
        return;
    }
    
    // Normalize the projected rotation direction
    Vec3 rotAxis = normalize(&projectedRotation);
    
    // Calculate rotation angle based on deltaTime
    // Adjust the speed factor (0.001) to control rotation speed
    float angularSpeed = 0.5f; // radians per millisecond
    float angle = angularSpeed * (float)deltaTime;
    
    // Rotate the radius vector around the rotation axis using Rodrigues' rotation formula
    // v_rot = v*cos(θ) + (k × v)*sin(θ) + k*(k · v)*(1 - cos(θ))
    // where k is the rotation axis and v is the vector to rotate
    
    float cosAngle = cosf(angle);
    float sinAngle = sinf(angle);
    
    Vec3 crossProd = crossProduct(&rotAxis, &radiusVec);
    float dotAxisRadius = dotProduct(&rotAxis, &radiusVec);
    
    // v*cos(θ)
    Vec3 term1 = scalarVec3(cosAngle, &radiusVec);
    
    // (k × v)*sin(θ)
    Vec3 term2 = scalarVec3(sinAngle, &crossProd);
    
    // k*(k · v)*(1 - cos(θ))
    Vec3 term3 = scalarVec3(dotAxisRadius * (1.0f - cosAngle), &rotAxis);
    
    // Combine all terms
    Vec3 temp = addVec3(&term1, &term2);
    Vec3 newRadiusVec = addVec3(&temp, &term3);
    
    // Update camera position: center + newRadiusVec
    camera->position = addVec3(&center, &newRadiusVec);
    
    // Update camera direction to look at center
    Vec3 newToCenter = subVec3(&center, &camera->position);
    camera->direction = normalize(&newToCenter);
}


int main (void)
{
    Camera camera = {
    .position = {0, 0, -5}, // Position the camera in front of the origin
    .direction = {0, 0, 1},  // Looking down the positive Z-axis
    .fov = 90.0f
    };
    char terminalBuffer[SCREEN_HEIGHT][SCREEN_WIDTH];
    init_buffer_terminal(terminalBuffer);
    Vec2i origin = project(Zeros3(), &camera,SCREEN_HEIGHT, SCREEN_WIDTH);
    Vec3 pos = loadVec(2.0,2.0,0.0);
    Vec2i v1 = project(pos, &camera,SCREEN_HEIGHT, SCREEN_WIDTH);
    
    pos = loadVec(0.0,2.0,0.0);
    

    double prevTime = get_time_millis();
    while (1)
    {
    double currentTime = get_time_millis();
    double deltaTime = currentTime - prevTime;
    prevTime = currentTime;
        Vec3 dist = {0,0,-4};
        Vec3 rotdir = {0,1,1};
        updateCamera(&camera, deltaTime, Zeros3(),dist, rotdir);

        Vec2i origin = project(Zeros3(), &camera,SCREEN_HEIGHT, SCREEN_WIDTH);
        Vec2i v1 = project(loadVec(2.0,2.0,0.0), &camera, SCREEN_HEIGHT, SCREEN_WIDTH);
        Vec2i v2 = project(loadVec(0.0,2.0,0.0), &camera, SCREEN_HEIGHT, SCREEN_WIDTH);
        drawLine(terminalBuffer,origin,v1,'#');
        drawLine(terminalBuffer,origin,v2,'%');
        drawLine(terminalBuffer,v1,v2,'x');
        system("clear");
        renderTerminal(terminalBuffer);
        init_buffer_terminal(terminalBuffer);

        usleep(500000); 
    }
}
// to do 
// camera rotation 
// color of the vector 
// normal axis
// use the hasmap right now 
