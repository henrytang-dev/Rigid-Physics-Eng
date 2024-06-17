#ifndef FORCE_GENERATOR_H
#define FORCE_GENERATOR_H

#include "system_state.h"

namespace physics_engine {
    class ForceGenerator {
        public:
            ForceGenerator();
            virtual ~ForceGenerator();

            virtual void apply(SystemState *system) = 0;

            int m_index;
    };
}

#endif 