/*
 * queue.h
 *
 *  Created on: Dec 22, 2023
 *      Author: lucas
 */

#ifndef __QUEUE_H
#define __QUEUE_H

#define QUEUE_SIZE 5

typedef struct Queue Queue;
struct Queue {
	volatile int buffer[QUEUE_SIZE];
    volatile int head;
    volatile int tail;
    volatile int size;

    int  (*is_full)  (Queue* volatile const me);
    int  (*is_empty) (Queue* volatile const me);
    int  (*get_size) (Queue* volatile const me);
    void (*insert)   (Queue* volatile const me, int k);
    int  (*remove)   (Queue* volatile const me);
};

/* Constructor and destructor */
void Queue_init( Queue* volatile const me,
                 int  (*is_full)  (Queue* volatile const me),
                 int  (*is_empty) (Queue* volatile const me),
                 int  (*get_size) (Queue* volatile const me),
                 void (*insert)   (Queue* volatile const me, int k),
                 int  (*remove)   (Queue* volatile const me)
               );
void Queue_cleanup(Queue* volatile const me);

/* Operations */
int  Queue_is_full  (Queue* volatile const me);
int  Queue_is_empty (Queue* volatile const me);
int  Queue_get_size (Queue* volatile const me);
void Queue_insert   (Queue* volatile const me, int k);
int  Queue_remove   (Queue* volatile const me);

/**/

Queue* Queue_create();
void   Queue_destroy(Queue* const me);

#endif
