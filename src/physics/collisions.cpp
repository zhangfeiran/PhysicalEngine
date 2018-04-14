#include "physics/collisions.hpp"

namespace _462 {

	bool collides(SphereBody& body1, SphereBody& body2, real_t collision_damping)
	{
		// TODO detect collision. If there is one, update velocity
		Vector3 p1 = body1.position, p2 = body2.position;
		if (distance(p1, p2) >= (body1.radius + body2.radius))
			return false;
		Vector3 v1 = body1.velocity, v2 = body2.velocity;
		real_t m1 = body1.mass, m2 = body2.mass;
		Vector3 d = (p1 - p2) / distance(p1, p2);
		Vector3 v22 = 2 * m1 / (m1 + m2)*dot(v1 - v2, d)*d;
		body2.velocity = v2 + v22;
		body1.velocity = (m1*v1 + m2*v2 - m2*body2.velocity) / m1;
		return true;
	}

	bool collides(SphereBody& body1, TriangleBody& body2, real_t collision_damping)
	{
		// TODO detect collision. If there is one, update velocity
		Vector3 p = body1.position, p1 = body2.vertices[0], p2 = body2.vertices[1], p3 = body2.vertices[2];

		real_t a = ((p2.y - p1.y)*(p3.z - p1.z) - (p2.z - p1.z)*(p3.y - p1.y));
		real_t b = ((p2.z - p1.z)*(p3.x - p1.x) - (p2.x - p1.x)*(p3.z - p1.z));
		real_t c = ((p2.x - p1.x)*(p3.y - p1.y) - (p2.y - p1.y)*(p3.x - p1.x));

		Vector3 normal=normalize(Vector3(a,b,c));
		real_t d = dot(p - p2, normal);
		if (abs(d) > body1.radius)
			return false;
		p1 -= p; p2 -= p; p3 -= p;
		Vector3 u = cross(p2, p3), v = cross(p3, p1);
		if (dot(u, v) < 0.0f)
			return false;
		Vector3 w = cross(p1, p2);
		if (dot(u, w) < 0.0f)
			return false;

		body1.velocity = body1.velocity - 2.0*dot(body1.velocity, normal)*normal;
		body1.velocity=body1.velocity*collision_damping;
		return true;
	}

	bool collides(SphereBody& body1, PlaneBody& body2, real_t collision_damping)
	{
		// TODO detect collision. If there is one, update velocity
		real_t d = dot(body1.position - body2.position, body2.normal);
		if (abs(d) > body1.radius)
			return false;
		body1.velocity = body1.velocity - 2.0*dot(body1.velocity, body2.normal)*body2.normal;
		return true;
	}

}
