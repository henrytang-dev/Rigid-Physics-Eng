#include "../include/system_state.h"

#include "../include/utilities.h"

#include <assert.h>
#include <cmath>
#include <cstring>

physics_engine::SystemState::SystemState() {
    indexMap = nullptr;

    a_theta = nullptr;
    v_theta = nullptr;
    theta = nullptr;

    a_x = nullptr;
    a_y = nullptr;
    v_x = nullptr;
    v_y = nullptr;
    p_x = nullptr;
    p_y = nullptr;

    f_x = nullptr;
    f_y = nullptr;
    t = nullptr;

    m = nullptr;

    r_x = 0;
    r_y = 0;
    r_t = 0;

    n = 0;
    n_c = 0;
    dt = 0.0;
}

physics_engine::SystemState::~SystemState() {
    assert(n == 0);
    assert(n_c == 0);
}

// copying a passed SystemState to this SystemState
void physics_engine::SystemState::copy(const SystemState *state) {
    resize(state -> n, state -> n_c);

    if (state -> n == 0) {
        return;
    }
    // void std::memcpy(void *dest, void *src, std::size_t count);
    std::memcpy((void *)indexMap, (void *)state->indexMap, sizeof(int) * n_c);

    std::memcpy((void *)v_theta, (void *)state->v_theta, sizeof(double) * n);
    std::memcpy((void *)a_theta, (void *)state -> a_theta, sizeof(double) * n); // sizeof(double) for each a_theta and n bodies with this data
    std::memcpy((void *)theta, (void *)state -> theta, sizeof(double) * n);

    std::memcpy((void *)a_x, (void *)state -> a_x, sizeof(double) * n);
    std::memcpy((void *)a_y, (void *)state->a_y, sizeof(double) * n);
    std::memcpy((void *)v_x, (void *)state->v_x, sizeof(double) * n);
    std::memcpy((void *)v_y, (void *)state->v_y, sizeof(double) * n);
    std::memcpy((void *)p_x, (void *)state->p_x, sizeof(double) * n);
    std::memcpy((void *)p_y, (void *)state->p_y, sizeof(double) * n);

    std::memcpy((void *)f_x, (void *)state->f_x, sizeof(double) * n);
    std::memcpy((void *)f_y, (void *)state->f_y, sizeof(double) * n);
    std::memcpy((void *)t, (void *)state->t, sizeof(double) * n);

    std::memcpy((void *)m, (void *)state->m, sizeof(double) * n);

    std::memcpy((void *)r_x, (void *)state->r_x, sizeof(double) * n_c * 2);
    std::memcpy((void *)r_y, (void *)state->r_y, sizeof(double) * n_c * 2);
    std::memcpy((void *)r_t, (void *)state->r_t, sizeof(double) * n_c * 2);
}

void physics_engine::SystemState::resize(int bodyCount, int constraintCount) {
    if (n >= bodyCount && n_c >= constraintCount) {
        return;
    }

    destroy();

    n = bodyCount;
    n_c = constraintCount;

    indexMap = new int[n_c];

    a_theta = new double[n];
    v_theta = new double[n];
    theta = new double[n];

    a_x = new double[n];
    a_y = new double[n];
    v_x = new double[n];
    v_y = new double[n];
    p_x = new double[n];
    p_y = new double[n];

    f_x = new double[n];
    f_y = new double[n];
    t = new double[n];

    m = new double[n];
    
    r_x = new double[(size_t)n_c * 2];
    r_y = new double[(size_t)n_c * 2];
    r_t = new double[(size_t)n_c * 2];
}

void physics_engine::SystemState::destroy() {
    if (n > 0) {
        freeArray(a_theta);
        freeArray(v_theta);
        freeArray(theta);

        freeArray(a_x);
        freeArray(a_y);
        freeArray(v_x);
        freeArray(v_y);
        freeArray(p_x);
        freeArray(p_y);

        freeArray(f_x);
        freeArray(f_y);
        freeArray(t);
        
        freeArray(m);
    }
    if (n_c > 0) {
        freeArray(indexMap);

        freeArray(r_x);
        freeArray(r_y);
        freeArray(r_t);
    }
    n_c = 0;
    n = 0;
}

// x_t is a pointer to a memory allocation storing transformed coordinates
void physics_engine::SystemState::localToWorld(double x, double y, double *x_t, double *y_t, int body) {
    // world coords
    const double x0 = p_x[body];
    const double y0 = p_y[body];

    const double theta = this -> theta[body]; // orientation relative to world coord system
    const double cos_theta = std::cos(theta);
    const double sin_theta = std::sin(theta);

    *x_t = cos_theta * x - sin_theta * y + x0;
    *y_t = sin_theta * x + cos_theta * y + y0;
}

void physics_engine::SystemState::velocityAtPoint(double x, double y, double *v_x, double *v_y, int body) {
    double w_x, w_y; // world coordinates
    localToWorld(x, y, &w_x, &w_y, body); // conversion of local to world

    const double v_theta = this -> v_theta[body]; // angular velocity
    const double angularToLinear_x = -v_theta * (w_y - this -> p_y[body]);
    const double angularToLinear_y = v_theta * (w_x - this->p_x[body]);

    *v_x = this->v_x[body] + angularToLinear_x;
    *v_y = this->v_y[body] + angularToLinear_y;
}

void physics_engine::SystemState::applyForce(double x_1, double y_1, double f_x, double f_y, int body) {
    double w_x, w_y;
    localToWorld(x_1, y_1, &w_x, &w_y, body); // so we can apply force to global reference
    
    this->f_x[body] += f_x;
    this->f_y[body] += f_y;

    this->t[body] += (w_y - this->p_y[body]) * -f_x + (w_x - this->p_x[body]) * f_y; // updating torque
}