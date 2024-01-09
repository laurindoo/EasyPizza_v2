/*
 * queue.c
 *
 *  Created on: Dec 22, 2023
 *      Author: lucas
 */


#include <my_queue.h>
#include <stdio.h>
#include <stdlib.h>

void Queue_init(Queue* volatile const me,
                 int  (*pFn_is_full)  (Queue* volatile const me),
                 int  (*pFn_is_empty) (Queue* volatile const me),
                 int  (*pFn_get_size) (Queue* volatile const me),
                 void (*pFn_insert)   (Queue* volatile const me, int k),
                 int  (*pFn_remove)   (Queue* volatile const me)
               ) {
    me->head  = 0;
    me->tail  = 0;
    me->size  = 0;

    me->is_full    = pFn_is_full;
    me->is_empty   = pFn_is_empty;
    me->get_size   = pFn_get_size;
    me->insert     = pFn_insert;
    me->remove     = pFn_remove;
}

void Queue_cleanup(Queue* volatile const me) {
}

Queue* Queue_create() {
    Queue* me = (Queue*) malloc(sizeof(Queue));
    if (me == NULL)
        return NULL;
    Queue_init(me,
               Queue_is_full,
               Queue_is_empty,
               Queue_get_size,
               Queue_insert,
               Queue_remove
              );
    return me;
}

void Queue_destroy(Queue* volatile const me) {
    if (me != NULL) {
        Queue_cleanup(me);
        free(me);
    }
}

int  Queue_is_full  (Queue* volatile const me) {
    return (me->head + 1) % QUEUE_SIZE == me->tail;
}

int  Queue_is_empty (Queue* volatile const me) {
    return (me->head == me->tail);
}

int  Queue_get_size (Queue* volatile const me) {
    return me->size;
}

void Queue_insert   (Queue* volatile const me, int k) {
    if (!me->is_full(me)) {
        me->buffer[me->head] = k;
        me->head = (me->head + 1) % QUEUE_SIZE;
        ++me->size;
    }
}

int  Queue_remove   (Queue* volatile const me) {
    int value = -9999; // sentinel value
    if (!me->is_empty(me)) {
        value = me->buffer[me->tail];
        me->tail = (me->tail + 1) % QUEUE_SIZE;
        --me->size;
    }
    return value;
}
