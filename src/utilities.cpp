#include "../include/utilities.h"

void physics_engine::freeArray(double *&data) {
    // delete[] is operator to deallocate memory that was previously allocated on the heap; deallocates reserved memory and helps prevent memory leaks
    // note that delete is for single objects while delete[] is for arrays
    delete[] data;
    data = nullptr; // pointer now becomes a dangling pointer so this is good practice
}

void physics_engine::freeArray(int *&data) {
    delete[] data;
    data = nullptr;
}