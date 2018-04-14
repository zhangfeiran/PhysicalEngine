#include "physics/physics.hpp"

namespace _462 {

	Physics::Physics() {
		reset();
	}

	Physics::~Physics() {
		reset();
	}

	void Physics::step(real_t dt) {
		// TODO step the world forward by dt. Need to detect collisions, apply
		// forces, and integrate positions and orientations.
		//
		// Note: put RK4 here, not in any of the physics bodies
		//
		// Must use the functions that you implemented
		//
		// Note, when you change the position/orientation of a physics object,
		// change the position/orientation of the graphical object that represents
		// it

		//collision
		for (SphereBody *sb : spheres)
			for (PlaneBody *pb : planes)
				collides(*sb, *pb, real_t(1.0f));

		for (SphereBody *sb : spheres)
			for (TriangleBody *tb : triangles)
				collides(*sb, *tb, real_t(spheres.size() == 1 ? 0.85f : 1.0f));

		for (real_t i = 0; i < spheres.size(); i++)
			for (real_t j = i + 1; j < spheres.size(); j++)
				collides(*spheres[i], *spheres[j], real_t(1.0f));

		for (SphereBody *sb : spheres) {
			sb->apply_force(gravity, Vector3::Zero);
		}

		//newtons_cradle
		if (springs.size() > 1) {
			real_t i(4.0);
			for (Spring *sp : springs) {
				sp->body2_offset = Vector3(i, 4.0, 0.0);
				i -= 2.0;
				sp->step(dt);
			}
		}

		//spring_rotation
		if (springs.size() == 1) {
			for (Spring *sp : springs) {
				sp->step(dt);
			}
		}

		//update
		for (SphereBody *sb : spheres) {
			sb->velocity += sb->force / sb->mass * dt;

			if (spheres.size() == 1) {
				sb->angular_velocity = sb->torque / (2.0 / 5.0*sb->mass*sb->radius*sb->radius);
				sb->angular_velocity.x = 0.01;
			}

			sb->position += sb->velocity*dt;
			sb->orientation =sb->orientation*Quaternion(sb->angular_velocity, dt/4.0) ;

			sb->sphere->position = sb->position;
			sb->sphere->orientation = sb->orientation;
		}
	}

	void Physics::add_sphere(SphereBody* b) {
		spheres.push_back(b);
	}

	size_t Physics::num_spheres() const {
		return spheres.size();
	}

	void Physics::add_plane(PlaneBody* p) {
		planes.push_back(p);
	}

	size_t Physics::num_planes() const {
		return planes.size();
	}

	void Physics::add_triangle(TriangleBody* t) {
		triangles.push_back(t);
	}

	size_t Physics::num_triangles() const {
		return triangles.size();
	}

	void Physics::add_spring(Spring* s) {
		springs.push_back(s);
	}

	size_t Physics::num_springs() const {
		return springs.size();
	}

	void Physics::reset() {
		for (SphereList::iterator i = spheres.begin(); i != spheres.end(); i++) {
			delete *i;
		}
		for (PlaneList::iterator i = planes.begin(); i != planes.end(); i++) {
			delete *i;
		}
		for (TriangleList::iterator i = triangles.begin(); i != triangles.end(); i++) {
			delete *i;
		}
		for (SpringList::iterator i = springs.begin(); i != springs.end(); i++) {
			delete *i;
		}

		spheres.clear();
		planes.clear();
		triangles.clear();
		springs.clear();

		gravity = Vector3::Zero;
		collision_damping = 0.0;
	}

}
