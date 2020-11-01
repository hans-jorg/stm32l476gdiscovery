/**
 * @file    buffer.c
 *
 * @note    FIFO buffer_t  for chars
 * @note    Uses a static data defined by DECLARE_BUFFER_AREA macro
 * @note    It does not use malloc
 * @note    Size must be defined in DECLARE_BUFFER_AREA and in buffer_init (Ugly)
 * @note    Uses as less dependencies as possible
 */

#include "buffer.h"


/**
 * @brief   initializes a fifo area
 *
 * @note    it actually returns a pointer to the same address as
 *          the one passed as parameter
 */

buffer_t
buffer_init(void *b, int n) {
buffer_t  f = (buffer_t) b;

    f->front = f->rear = f->data;
    f->size = 0;
    f->capacity = n;
    return f;
}

/**
 * @brief   Clears fifo
 *
 * @note    Does not free any area, because the area is static
 *          This can change in the future
 */

void
buffer_deinit(buffer_t  f) {

    f->size = 0;
    f->front = f->rear = f->data;

}

/**
 * @brief   Clears fifo
 *
 * @note    Does not free area. For now identical to deinit
 */
 void
 buffer_clear(buffer_t  f) {

    f->size = 0;
    f->front = f->rear = f->data;

}

/**
 * @brief   Insert an element in fifo
 *
 * @note    return -1 when full
 */

int
buffer_insert(buffer_t  f, char x) {

    if( buffer_full(f) )
        return -1;

    *(f->rear++) = x;
    f->size++;
    if( (f->rear - f->data) > f->capacity )
        f->rear = f->data;
    return 0;
}

/**
 * @brief   Removes an element from fifo
 *
 * @note    return -1 when empty
 */

int
buffer_remove(buffer_t  f) {
char ch;

    if( buffer_empty(f) )
        return -1;

    ch = *(f->front++);
    f->size--;
    if( (f->front - f->data) > f->capacity )
        f->front = f->data;
    return ch;
}
