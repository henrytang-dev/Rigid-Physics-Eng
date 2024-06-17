#ifndef RIGID_BODY_SYSTEM_H
#define RIGID_BODY_SYSTEM_H

#include "rigid_body.h"
#include "constraint.h"
#include "force_generator.h"
#include "matrix.h"
#include "sparse_matrix.h"
#include "system_state.h"


#include <vector>

namespace physics_engine {
    class RigidBodySystem {
        public:
            static const int ProfilingSamples = 60 * 10; // for performance tracking for optimization and debugging; 600 frames is the idea

            RigidBodySystem();
            virtual ~RigidBodySystem();

            virtual void reset();
            virtual void process(double dt, int steps = 1);

            void addRigidBody(RigidBody *body);
            void removeRigidBody(RigidBody *body);
            RigidBody *getRigidBody(int i);

            void addConstraint(Constraint *constraint);
            void removeConstraint(Constraint *constraint);

            void addForceGenerator(ForceGenerator *generator);
            void removeForceGenerator(ForceGenerator *generator);

            int getRigidBodyCount() const { return (int) m_rigidBodies.size(); }
            int getConstraintCount() const { return (int)m_constraints.size(); }
            int getForceGeneratorCount() const { return (int)m_forceGenerators.size(); }

            int getFullConstraintCount() const;

            float getOdeSolveMicroseconds() const;
            float getConstraintSolveMicroseconds() const;
            float getForceEvalMicroseconds() const;
            float getConstraintEvalMicroseconds() const;

            inline const SystemState *state() const { return &m_state; } // inline suggests to compiler to expand function code
            // &m_state returns the address which is effectively done as a pointer that is returned (since it returned the address)
        
        protected:
            static float findAverage(long long *samples);

            void populateSystemState();
            void populateMassMatrices(Matrix *M, Matrix *M_inv);
            void processForces();

        protected:
            std::vector<RigidBody *> m_rigidBodies;
            std::vector<Constraint *> m_constraints;
            std::vector<ForceGenerator *> m_forceGenerators;

            SystemState m_state;

            long long *m_odeSolveMicroseconds;
            long long *m_constraintSolveMicroseconds;
            long long *m_forceEvalMicroseconds;
            long long *m_constraintEvalMicroseconds;
            long long m_frameIndex;
    };
}

#endif