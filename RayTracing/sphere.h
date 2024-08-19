#ifndef SPHERE_H
#define SPHERE_H

#include "hittable.h"

class sphere : public hittable {
public:
	sphere(const point3& center, double radius) : center(center), radius(radius) {}

	bool hit(const ray& r, interval ray_t, hit_record& rec) const override {
		vec3 oc = r.origin() - center;
		double a = r.direction().length_squared();
		double h = dot(oc, r.direction());
		double c = oc.length_squared() - radius * radius;
		double discriminant = h * h - a * c;

		if (discriminant < 0) {
			return false;
		}

		double root = std::sqrt(discriminant);

		double temp = (-h - root) / a;
		if (!ray_t.surrounds(temp)) {
			temp = (-h + root) / a;
			if (!ray_t.surrounds(temp)) {
				return false;
			}
		}

		rec.t = temp;
		rec.p = r.at(rec.t);
		vec3 outward_normal = (rec.p - center) / radius;
		rec.set_face_normal(r, outward_normal);
		return true;
	}

private:
	point3 center;
	double radius;
};

#endif