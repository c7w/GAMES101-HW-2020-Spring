#include <iostream>
#include <vector>

#include "CGL/vector2D.h"

#include "mass.h"
#include "rope.h"
#include "spring.h"

namespace CGL {

    Rope::Rope(Vector2D start, Vector2D end, int num_nodes, float node_mass, float k, vector<int> pinned_nodes)
    {
        // TODO (Part 1): Create a rope starting at `start`, ending at `end`, and containing `num_nodes` nodes.

        auto step = (end - start) / (num_nodes - 1);
        for (int i = 0; i < num_nodes; ++i) {
            auto pos = start + i * step;
            masses.push_back(new Mass(pos, node_mass, false));
        }

        for(int i = 0; i < num_nodes - 1; ++i) {
            springs.push_back(new Spring(masses[i], masses[i+1], k));
        }

//        Comment-in this part when you implement the constructor
        for (auto &i : pinned_nodes) {
            masses[i]->pinned = true;
        }
    }

    void Rope::simulateEuler(float delta_t, Vector2D gravity)
    {
        for (auto &s : springs)
        {
            // TODO (Part 2): Use Hooke's law to calculate the force on a node
            auto v1 = s->m1;
            auto v2 = s->m2;

            v1->forces +=  (v2->position - v1->position) / (v2->position - v1->position).norm() * s->k * ((v2->position - v1->position).norm() - s->rest_length);
            v2->forces +=  (v1->position - v2->position) / (v1->position - v2->position).norm() * s->k * ((v2->position - v1->position).norm() - s->rest_length);
        }

        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                // TODO (Part 2): Add the force due to gravity, then compute the new velocity and position
                
                m->forces += -0.05 * m->velocity; // -kv
                Vector2D a = m->forces / m->mass + gravity;
                
                // Explicit Method, 
                // m->position += m->velocity * delta_t;
                // m->velocity += a * delta_t;
                
                m->last_position = m->position;
                
                m->velocity += a * delta_t;
                m->position += m->velocity * delta_t;
                
                

                // TODO (Part 2): Add global damping

            }

            // Reset all forces on each mass
            m->forces = Vector2D(0, 0);
        }
    }

    void Rope::simulateVerlet(float delta_t, Vector2D gravity)
    {
        for (auto &s : springs)
        {
            // TODO (Part 3): Simulate one timestep of the rope using explicit Verlet （solving constraints)
            auto v1 = s->m1;
            auto v2 = s->m2;

            v1->forces +=  (v2->position - v1->position) / (v2->position - v1->position).norm() * s->k * ((v2->position - v1->position).norm() - s->rest_length);
            v2->forces +=  (v1->position - v2->position) / (v1->position - v2->position).norm() * s->k * ((v2->position - v1->position).norm() - s->rest_length);
        
            
            // auto v1 = s->m1;
            // auto v2 = s->m2;

            // v1->last_position = v1->position;
            // v2->last_position = v2->position;

            // v1->force = (v2->position - v1->position) / (v2->position - v1->position).norm() * ((v2->position - v1->position).norm() - s->rest_length) * 0.5;
            // v2->force = (v1->position - v2->position) / (v1->position - v2->position).norm() * ((v2->position - v1->position).norm() - s->rest_length) * 0.5;

        }

        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                Vector2D temp_position = m->position;
                // TODO (Part 3.1): Set the new position of the rope mass


                Vector2D a = m->forces / m->mass + gravity;

                m->position = temp_position + (1-5e-5) * (m->position - m->last_position) >+ a * delta_t * delta_t;
                m->last_position = temp_position;

                // TODO (Part 4): Add global Verlet damping
            }

            // Reset all forces on each mass
            m->forces = Vector2D(0, 0);
        }
    }
}
