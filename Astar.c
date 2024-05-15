#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>
#include <float.h>

// --------------- Min Binary Heap implementation ---------------
typedef struct Node
{
    double f;
    double g;
    double h;

    int x;
    int y;

    struct Heap *heap;
    struct Node *parent;
} Node;

typedef struct
{
    size_t max;
    size_t len;
    Node **ref;
} Heap;

Heap *heap_init()
{
    Heap *h = malloc(sizeof *h);
    h->max = 0;
    h->len = 0;
    h->ref = NULL;
    return h;
}

Node *node_init(int x, int y, double g, double h, Node *parent)
{
    Node *node = malloc(sizeof *node);
    node->x = x;
    node->y = y;
    node->g = g;
    node->h = h;
    node->f = g + h;

    node->parent = parent;
    return node;
}

Node *update_node(Node *old, double g, double h, Node *parent, Heap *heap)
{
    Node *node = old;
    // Distance from start
    if (g >= 0 && h >= 0)
    {
        node->g = g;
        node->h = h;
        node->f = g + h;
    }

    if (parent)
        node->parent = parent;
    if (heap)
        node->heap = heap;
    return node;
}

void swap_nodes(Heap *h, int i1, int i2)
{
    Node *tmp;
    tmp = h->ref[i1];
    h->ref[i1] = h->ref[i2];
    h->ref[i2] = tmp;
}

void print_heap(Heap *h)
{
    int layer_length = 1;
    int depth = log2(h->len - 1) + 1;
    int total = 0;
    // i is index of h->ref[i]ent layer
    for (int i = 0; i < depth; i++)
    {
        for (int k = pow(2, i); k < pow(2, i) + layer_length; k++)
        {
            if (k - 1 >= h->len || total >= h->len)
            {
                printf("-");
                printf("\n");
                return;
            }
            else
            {
                printf("%lf", h->ref[k - 1]->f);
                printf("(");
                printf("%d", h->ref[k - 1]->x);
                printf(" ");
                printf("%d", h->ref[k - 1]->y);
                printf(")");
            }

            printf(" ");
            total++;
        }
        printf("\n");
        layer_length *= 2;
    }
    printf("\n");
}

int heap_add(Heap *h, Node *n)
{
    size_t i;
    size_t size = sizeof n;

    if (!h)
        return -1;

    if (h->len >= h->max)
    {
        size_t max;

        if (h->len < 15)
            max = 15;
        else
            max = 2 * h->len;

        h->max = max;
        h->ref = realloc(h->ref, max * size);
    }

    i = h->len++;
    h->ref[i] = n;

    // Tree reorder
    if (!i)
    {
        // i is root
        return 0;
    }

    do
    {
        if (h->ref[(i - 1) / 2] && h->ref[(i - 1) / 2]->f > n->f)
        {

            swap_nodes(h, i, (i - 1) / 2);
            i = (i - 1) / 2;
        }
        else
        {
            break;
        }
    } while (i != 0);
}

void heap_delete(Heap *h, int index)
{
    h->len--;

    swap_nodes(h, h->len, index);
    Node *deleted = h->ref[h->len];

    h->ref[h->len] = NULL;

    int i = 0;
    int right_index;
    int left_index;
    do
    {
        if (i == 0)
        {
            right_index = 2;
            left_index = 1;
        }
        else
        {
            left_index = i * 2;
            right_index = i * 2 + 1;
        }

        // Element is root
        if (!h->ref[0])
        {
            return;
        }

        // Element has no children
        if (left_index >= h->len && right_index >= h->len)
        {
            return;
        }

        // Element has no left child, but has right
        if (left_index >= h->len && right_index < h->len)
        {
            // Element not breaking tree
            if (h->ref[i]->f < h->ref[right_index]->f)
            {
                return;
            }
            swap_nodes(h, i, right_index);
            i = right_index;
            continue;
        }

        // Element has no right child, but has left
        if (right_index >= h->len && left_index < h->len)
        {
            // Element not breaking tree
            if (h->ref[i]->f < h->ref[left_index]->f)
            {
                return;
            }
            swap_nodes(h, i, left_index);
            i = left_index;
            continue;
        }

        //  Element has both children and not breaking tree
        if (h->ref[i]->f < h->ref[left_index]->f && h->ref[i]->f < h->ref[right_index]->f)
        {
            return;
        }

        // Element has both children and breaks tree
        if (h->ref[left_index]->f < h->ref[right_index]->f)
        {
            swap_nodes(h, i, left_index);
            i = left_index;
        }
        else
        {
            swap_nodes(h, i, right_index);
            i = right_index;
        }
    } while (true);
}

// --------------- Pathfinding implementation ---------------
typedef struct Path
{
    Node *start;
    Heap *closed;
    Heap *open;
} Path;

#define KNRM "\x1B[0m"
#define KGRA "\x1B[1;30m"
#define KRED "\x1B[31m"
#define KGRN "\x1B[32m"
#define KYEL "\x1B[33m"
#define KBLU "\x1B[34m"
#define KMAG "\x1B[35m"
#define KCYN "\x1B[36m"
#define KWHT "\x1B[37m"
void display_path(Path *p, int start_x, int start_y, int end_x, int end_y, int grid_size, int obstacles_len, int obstacles_list[obstacles_len][2])
{
    // Field
    char *grid[grid_size][grid_size];
    for (int i = 0; i < grid_size; i++)
    {
        for (int j = 0; j < grid_size; j++)
        {
            grid[i][j] = KGRA "0";
        }
    }

    // Closed list
    if (p->closed)
        for (int i = 0; i < p->closed->len; i++)
        {
            grid[p->closed->ref[i]->x][p->closed->ref[i]->y] = KGRN "x";
        }

    // Obstacles
    for (int i = 0; i < obstacles_len; i++)
    {
        grid[obstacles_list[i][0]][obstacles_list[i][1]] = KRED "-";
    }

    // Path
    Node *tile = p->start;
    while (true)
    {
        if (!tile->parent)
        {
            break;
        }
        grid[tile->x][tile->y] = KBLU "1";
        tile = tile->parent;
    }

    // Start and end
    grid[start_x][start_y] = KMAG "s";
    grid[end_x][end_y] = KMAG "e";

    // -- Display --
    for (int i = 0; i < grid_size; i++)
    {
        for (int j = 0; j < grid_size; j++)
        {
            printf("%s", grid[i][j]);
            printf(" ");
        }
        printf("\n");
    }
}

float calc_distance(int x1, int y1, int x2, int y2)
{
    return sqrt(pow((float)x2 - (float)x1, 2) + pow((float)y2 - (float)y1, 2));
}

Path *find_path(int start_x, int start_y, int end_x, int end_y, int grid_len, Node *grid[grid_len][grid_len], int obstacles_len, int obstacles_list[obstacles_len][2])
{
    Heap *open = heap_init();
    Heap *closed = heap_init();
    Heap *obstacles = heap_init();

    for (int i = 0; i < obstacles_len; i++)
    {
        heap_add(obstacles, update_node(grid[obstacles_list[i][0]][obstacles_list[i][1]], DBL_MAX / 2, DBL_MAX / 2, NULL, obstacles));
    }

    // -- Init starting node --
    heap_add(open, update_node(grid[start_x][start_y], 0, calc_distance(start_x, start_y, end_x, end_y), NULL, open));

    // -- Work cycle --
    while (true)
    {
        Node *prev = &*(open->ref[0]);
        heap_delete(open, 0);

        /*
        (x-1,y-1) (x,y-1) (x+1,y-1)
        (x-1,y)   (x,y)   (x+1,y)
        (x-1,y+1) (x,y+1) (x+1,y+1)
        */
        int successors[8][2] = {{prev->x - 1, prev->y - 1},
                                {prev->x - 1, prev->y},
                                {prev->x - 1, prev->y + 1},
                                {prev->x, prev->y - 1},
                                {prev->x, prev->y + 1},
                                {prev->x + 1, prev->y - 1},
                                {prev->x + 1, prev->y},
                                {prev->x + 1, prev->y + 1}};

        int open_len = open->len;

        for (int i = 0; i < 8; i++)
        {
            if (successors[i][0] < 0 || successors[i][1] < 0 || successors[i][0] > grid_len || successors[i][1] > grid_len)
            {
                continue;
            }

            Node *successor = grid[successors[i][0]][successors[i][1]];

            if (successor->heap == obstacles)
            {
                continue;
            }

            double g = prev->g + calc_distance(prev->x, prev->y, successor->x, successor->y);
            double h = calc_distance(successor->x, successor->y, end_x, end_y);
            double f = g + h;

            // -- Check for final condition
            if (successor->x == end_x && successor->y == end_y)
            {
                prev->heap = closed;
                heap_add(closed, prev);
                heap_add(closed, update_node(successor, g, h, prev, closed));

                Path *r = malloc(sizeof *r);
                r->start = successor;
                r->open = open;
                r->closed = closed;

                return r;
            }

            // -- Check if node is in open or closed list with lower f
            if ((successor->heap == open || successor->heap == closed))
            {
                if (successor->f > f)
                {
                    successor->g = g;
                    successor->h = h;
                    successor->f = f;
                }

                continue;
            }
            else
            {
                heap_add(open, update_node(successor, g, h, prev, open));
            }
        }

        prev->heap = closed;
        heap_add(closed, prev);
    };
}
// --------------------------------------------------------------

int main()
{
    int grid_size = 30;
    int obstacles[25][2] = {{5, 1}, {5, 2}, {5, 3}, {5, 4}, {5, 5}, {5, 6}, {5, 7}, {5, 8}, {5, 9}, {5, 10}, {10, 1}, {10, 2}, {10, 3}, {10, 4}, {10, 5}, {10, 6}, {10, 7}, {10, 8}, {10, 9}, {10, 10}, {10, 11}, {10, 12}, {10, 13}, {10, 14}, {10, 15}};
    int start_x = 0;
    int start_y = 0;
    int end_x = 20;
    int end_y = 20;

    Node *grid[grid_size][grid_size];
    for (int i = 0; i < grid_size; i++)
    {
        for (int j = 0; j < grid_size; j++)
        {
            grid[i][j] = node_init(i, j, DBL_MAX / 2, DBL_MAX / 2, NULL);
        }
    }

    Path *p = find_path(start_x, start_y, end_x, end_y, grid_size, grid, sizeof obstacles / sizeof obstacles[0], obstacles);

    display_path(p, start_x, start_y, end_x, end_y, grid_size, sizeof obstacles / sizeof obstacles[0], obstacles);
}