#ifndef SYSTEM_STATE_H
#define SYSTEM_STATE_H

// tracks various object properties like position, velocity, and orientation
// using pointers since managing multiple constraints and bodies in system
// note how we can think of this SystemState object as a large datastructure and we can use a pointer to p oint to it
namespace physics_engine {
    class SystemState {
        public:
            SystemState();
            ~SystemState();

            void copy(const SystemState *state);
            void resize(int bodyCount, int constraintCount);
            void destroy();

            void localToWorld(double x, double y, double *x_t, double *y_t, int body);
            void velocityAtPoint(double x, double y, double *v_x, double *v_y, int body);
            void applyForce(double x_1, double y_1, double f_x, double f_y, int body);

            int *indexMap;

            double *a_theta;
            double *v_theta;
            double *theta;

            double *a_x;
            double *a_y;
            double *v_x;
            double *v_y;
            double *p_x;
            double *p_y;

            double *f_x;
            double *f_y;
            double *t;

            double *r_x;
            double *r_y;
            double *r_t;

            double *m;

            int n; // bodies in system
            int n_c; // # of constrainsts
            double dt;
    };
}

#endif /* SYSTEM_STATE_H */