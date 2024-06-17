#ifndef RIGID_BODY_H
#define RIGID_BODY_H

namespace physics_engine {
    struct RigidBody {
        public:
            RigidBody();
            ~RigidBody();
            
            void localToWorld(double x, double y, double *w_x, double *w_y);
            void worldToLocal(double x, double y, double *w_x, double *w_y);

            double p_x; // x-pos
            double p_y; // y-pos

            double v_x; // x-velocity
            double v_y; // y-velocity

            double theta; // orientation of rigid body (rads)
            double v_theta; // angular velocity

            double m; // Mass
            double I; // Inertia

            int index; // identification in relation to rigid system

            void reset();
            double energy() const;


    };
}

#endif //RIGID_BODY_H