#ifndef _TRIANGLE_MESH_H_
#define _TRIANGLE_MESH_H_

#include <map>

#include "..\vec3.h"
#include "..\constant.h"
#include "bbox.h"
#include "intersection.h"
#include "qbvh.h"

#include "..\memfile.h"
#include "..\textutil.h"

namespace hstd {

namespace rt {

struct Material {
	Float3 diffuse;
	Float3 specular;
	float specular_coefficient;
	float metalic;

	Material(Float3& diffuse, Float3& specular, float specular_coefficient, float metalic) :
	diffuse(diffuse), specular(specular), specular_coefficient(specular_coefficient), metalic(metalic) {}
};

typedef std::map<std::string, int32_t> MaterialMap;
typedef std::vector<Material> MaterialVector;

struct Triangle {
	Int3 v_index;
	Int3 vt_index;
	Int3 vn_index;
    int32_t material;
};

struct MeshBody {
	std::vector<Float3> v;
	std::vector<Float3> vt;
	std::vector<Float3> vn;
	std::vector<Triangle> triangle;
	MaterialMap matmap;
    MaterialVector matvec;
};

struct TriangleElement {
	const Float3* v[3];
	const Float3* vt[3];
	const Float3* vn[3];
	const Material* material;

	TriangleElement() {
		v[0] = v[1] = v[2] = NULL;
		vn[0] = vn[1] = vn[2] = NULL;
		vt[0] = vt[1] = vt[2] = NULL;
		material = NULL;
	}
};

class TriangleMesh {
private:
	MeshBody body_;
	QBVH qbvh_;
public:
	void set(const MeshBody& body) {
		body_ = body;
	}
	MeshBody& get_body() { return body_; }
	void build(std::vector<RefTriangle>& ref_triangle) {
		qbvh_.build(ref_triangle);
	}

	TriangleElement getTriangle(const int triangle_index) const {
		TriangleElement t;
		if (triangle_index >= body_.triangle.size())
			return t;

        Triangle now_t = body_.triangle[triangle_index];
		//const Triangle& now_t = body_.triangle[triangle_index];
		for (int i = 0; i < 3; ++i) {
			t.v[i] = &body_.v[now_t.v_index[i]];
			if (0 <= now_t.vn_index[i] && now_t.vn_index[i] < body_.vn.size())
				t.vn[i] = &body_.vn[now_t.vn_index[i]];
			if (0 <= now_t.vt_index[i] && now_t.vt_index[i] < body_.vt.size())
				t.vt[i] = &body_.vt[now_t.vt_index[i]];
		}
        if (now_t.material != -1)
    		t.material = &body_.matvec[now_t.material];

		return t;
	}

	virtual bool intersect(const Ray &ray, Hitpoint* hitpoint) const {
		*hitpoint = Hitpoint();
		return qbvh_.intersect(ray, hitpoint, -1);

#if 0
        hitpoint->distance = 1e+8f;
        bool hit = false;

        int counter = -1;
        for (auto& triangle: body_.triangle) {
            ++counter;
            const Float3 &p1 = body_.v[triangle.v_index[0]];
            const Float3 &p2 = body_.v[triangle.v_index[1]];
            const Float3 &p3 = body_.v[triangle.v_index[2]];
            const Float3 e1 = p2 - p1;
            const Float3 e2 = p3 - p1;
            const Float3 s1 = cross(ray.dir, e2);
            const float divisor = dot(s1, e1);

            if (divisor == 0.0f)
                continue;
            float invDivisor = 1.0f / divisor;

            // Compute first barycentric coordinate
            const Float3 d = ray.org - p1;
            float b1 = dot(d, s1) * invDivisor;
            if (b1 < 0. || b1 > 1.)
                continue;

            // Compute second barycentric coordinate
            Float3 s2 = cross(d, e1);
            float b2 = dot(ray.dir, s2) * invDivisor;
            if (b2 < 0. || b1 + b2 > 1.)
                continue;

            // Compute _t_ to intersection point
            float t = dot(e2, s2) * invDivisor;
            if (t < kEPS || t > kINF)
                continue;

            if (hitpoint->distance > t) {
                hit = true;
                hitpoint->distance = t;
                hitpoint->b1 = b1;
                hitpoint->b2 = b2;
                hitpoint->triangle_index = counter;
                
                auto v = b1 * e1 + b2 * e2 + p1;
                auto hp2 = ray.org + hitpoint->distance * ray.dir;
                std::cout << v << " " << hp2 << std::endl;
            }
        }

        return hit;
#endif
	}
};


} // namespace rt

} // namespace hstd

#endif // _TRIANGLE_MESH_H_