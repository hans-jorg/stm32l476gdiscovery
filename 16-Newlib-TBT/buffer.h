#ifndef BUFFER_H
#define BUFFER_H
/**
 *  @file   buffer.h
 */


/**
 * @brief  Data structure to store info about fifo, including its data
 *
 * @note   Uses x[1] hack. This structure is a header for the area
 */

struct buffer_s {
    char    *front;             // pointer to first char in fifo
    char    *rear;              // pointer to last char in fifo
    int     size;               // number of char stored in fifo
    int     capacity;           // number of chars in data
    char    data[1];            // flexible array
};

typedef struct buffer_s *buffer_t;

#define BUFFER_SIZE_UNSIGNED(SIZE) \
    ((sizeof(struct buffer_s)+(SIZE)+sizeof(unsigned)-1)/sizeof(unsigned))
#define BUFFER_SIZE_BYTES(SIZE) \
    (BUFFER_SIZE_UNSIGNED(SIZE)*sizeof(unsigned))
#define BUFFER_DEFINE_AREA(AREANAME,SIZE) unsigned AREANAME[BUFFER_SIZE_UNSIGNED(SIZE)]

buffer_t  buffer_init(void *area,int size);
void      buffer_deinit(buffer_t f);
int       buffer_insert(buffer_t f, char x);
int       buffer_remove(buffer_t f);
void      buffer_clear(buffer_t f);

#define buffer_capacity(F)  ((F)->capacity)
#define buffer_size(F)      ((F)->size)
#define buffer_empty(F)     ((F)->size==0)
#define buffer_full(F)      ((F)->size==buffer_capacity(F))

#endif
