#ifndef UTILITIES_H
#define UTILITIES_H

#if defined(__APPLE__)
#define scs_force_inline inline
#else
#define scs_force_inline __forceinline
#endif

namespace physics_engine {
    void freeArray(double *&data);
    void freeArray(int *&data);
}

#endif