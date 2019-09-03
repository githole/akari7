#define STB_IMAGE_WRITE_IMPLEMENTATION
#define STBI_MSC_SECURE_CRT
#include "external/stb/stb_image_write.h"
#include "external/fmath/fmath.hpp"
#include "external/tinyobjloader/tiny_obj_loader.h"

#include "vec3.h"
#include "util.h"
#include "etc.h"
#include "hdr.h"
#include "test.h"
#include "XYZ.h"
#include "bbox.h"

#include "hlib/rt/objMesh.h"

#include <iostream>
#include <thread>
#include <mutex>
#include <vector>
#include <atomic>
#include <random>
// #include <omp.h>

#define _USE_MATH_DEFINES
#include <math.h>

// #define TEST

constexpr int NUM_THREAD = 72;

class FloatImage
{
private:
    size_t width_, height_;
    std::vector<hmath::Double3> data_;
//    std::vector<hmath::Double3> data2_;
    std::vector<uint32_t> sample_map_;

    std::vector<hmath::Double3> mean_var_;
public:
    FloatImage(size_t w = 0, size_t h = 0) : width_(w), height_(h), data_(width_ * height_),
//        data2_(width_ * height_),
        sample_map_(width_ * height_),
        mean_var_(width_ * height_)
    {
    }

    size_t width() const { return width_; }
    size_t height() const { return height_; }

    hmath::Double3& operator()(int x, int y)
    {
        return data_[x + y * width_];
    }
#if 0
    hmath::Double3& data2(int x, int y)
    {
        return data2_[x + y * width_];
    }
#endif
    hmath::Double3& mean_var(int x, int y)
    {
        return mean_var_[x + y * width_];
    }

    uint32_t& samples(int x, int y)
    {
        return sample_map_[x + y * width_];
    }
};

namespace mesh
{
    struct Material
    {
        hmath::Float3 diffuse;
        hmath::Float3 emission;
        float ior = 0;
    };
    std::vector<Material> material_table;

    struct Mesh
    {
        std::vector<float> vertex;
        std::vector<uint32_t> index;
        std::vector<uint32_t> material_map;
    };
    Mesh loadMesh(const std::string& filename)
    {
        // TODO: 外部からなんとかできるようにしたいところではある
        material_table.clear();
        material_table.push_back({ hmath::Float3(0.3f, 0.3f, 0.3f), hmath::Float3(0, 0, 0) });
//        material_table.push_back({ hmath::Float3(0.0f, 0.0f, 0.0f), hmath::Float3(160, 80, 20) / 64.0f * 64.0f });
        material_table.push_back({ hmath::Float3(0.0f, 0.0f, 0.0f), hmath::Float3(50, 80, 120) / 64.0f * 64.0f * 2.0f });
        material_table.push_back({ hmath::Float3(1.0f, 1.0f, 1.0f), hmath::Float3(0, 0, 0), 1.5f });

        std::vector<uint32_t> material_map;

        tinyobj::attrib_t attrib;
        std::vector<tinyobj::shape_t> shapes;
        std::vector<tinyobj::material_t> materials;
        std::string warn;
        std::string err;
        std::vector<uint32_t> indices;
        tinyobj::LoadObj(&attrib, &shapes, &materials, &warn, &err, filename.c_str(), nullptr, true);

        for (auto& shape : shapes)
        {
            // printf("%s\n", shape.name.c_str());

            int current_material = 0;

            if (shape.name == "light")
            {
                current_material = 1;
            }
            else if (shape.name == "normal")
            {
                current_material = 0;
            }
            else if (shape.name == "glass")
            {
                current_material = 2;
            }

            for (auto& index : shape.mesh.indices)
            {
                indices.push_back(index.vertex_index);
                material_map.push_back(current_material);
                // indices.push_back(index.normal_index);
            }
        }
        return { attrib.vertices, indices, material_map };
    }

    namespace oldlib
    {
        struct Triangle
        {
            hstd::Float3 v[3];
            hstd::Float3 n[3];
            Material* mat;
        };
        std::vector<Triangle> triangles;
        std::vector<hstd::rt::RefTriangle> ref_triangles;
        hstd::rt::QBVH qbvh;
    }
}

namespace
{
    const hmath::Float3 rgb2y(0.2126, 0.7152, 0.0722);
    const hmath::Double3 double_rgb2y(0.2126, 0.7152, 0.0722);
}

void save_image(const char* filename, FloatImage& image, bool enable_filter = false)
{
    const auto Width = image.width();
    const auto Height = image.height();
    uint64_t total = 0;
    std::vector<float> tonemapped_image(Width * Height * 3);
    std::vector<uint8_t> uint8_tonemapped_image(Width * Height * 3);

    std::vector<float> fdata(Width * Height * 3);
    std::vector<float> fdata2(Width * Height * 3);

    fmath::PowGenerator degamma(1.0f / 2.2f);

    for (int iy = 0; iy < Height; ++iy) {
        for (int ix = 0; ix < Width; ++ix) {
            auto index = ix + iy * Width;
            const double N = image.samples(ix, iy);
            auto p = image(ix, iy) / N;
            total += image.samples(ix, iy);

            //const double lumi = dot(p, rgb2y);
            //p[0] = p[1] = p[2] = image.data2(ix, iy)[0] / (N - 1) - (N / (N - 1)) * lumi * lumi;

            fdata[index * 3 + 0] = (float)p[0];
            fdata[index * 3 + 1] = (float)p[1];
            fdata[index * 3 + 2] = (float)p[2];
        }
    }


    for (int iy = 0; iy < Height; ++iy) {
        for (int ix = 0; ix < Width; ++ix) {
            auto index = ix + iy * Width;
            fdata2[index * 3 + 0] = (degamma.get(hmath::clamp((float)fdata[index * 3 + 0], 0.0f, 1.0f)));
            fdata2[index * 3 + 1] = (degamma.get(hmath::clamp((float)fdata[index * 3 + 1], 0.0f, 1.0f)));
            fdata2[index * 3 + 2] = (degamma.get(hmath::clamp((float)fdata[index * 3 + 2], 0.0f, 1.0f)));
        }
    }

    if (enable_filter) {
        // filter::median(fdata2, tonemapped_image, Width, Height);
        for (int iy = 0; iy < Height; ++iy) {
            for (int ix = 0; ix < Width; ++ix) {
                auto index = ix + iy * Width;
                uint8_tonemapped_image[index * 3 + 0] = (uint8_t)(fdata2[index * 3 + 0] * 255);
                uint8_tonemapped_image[index * 3 + 1] = (uint8_t)(fdata2[index * 3 + 1] * 255);
                uint8_tonemapped_image[index * 3 + 2] = (uint8_t)(fdata2[index * 3 + 2] * 255);
            }
        }
    }
    else {
        for (int iy = 0; iy < Height; ++iy) {
            for (int ix = 0; ix < Width; ++ix) {
                auto index = ix + iy * Width;
                uint8_tonemapped_image[index * 3 + 0] = (uint8_t)(fdata2[index * 3 + 0] * 255);
                uint8_tonemapped_image[index * 3 + 1] = (uint8_t)(fdata2[index * 3 + 1] * 255);
                uint8_tonemapped_image[index * 3 + 2] = (uint8_t)(fdata2[index * 3 + 2] * 255);
            }
        }
    }


    double average = (double)total / (Width * Height);
//    std::cout << "Average: " << average << " samples/pixel" << std::endl;
    stbi_write_png(filename, (int)Width, (int)Height, 3, uint8_tonemapped_image.data(), (int)(Width * sizeof(uint8_t) * 3));
#if 0
    std::string hdrfilename = filename;
    hhdr::save((hdrfilename + +".hdr").c_str(), fdata.data(), (int)Width, (int)Height, false);
#endif
}



namespace
{

    template<typename V, typename T>
    inline void directionToPolarCoordinate(const V& dir, T *theta, T *phi) {
        *theta = acos(dir[1]);
        *phi = atan2(dir[2], dir[0]);
        if (*phi < 0)
            *phi += 2.0f * M_PI;
    }

    template <typename Vec3, typename T>
    inline Vec3 polarCoordinateToDirection(T theta, T phi, const Vec3 &normal,
        const Vec3 &tangent,
        const Vec3 &binormal) {
        return
            sin(theta) * cos(phi) * tangent + 
            cos(theta) * normal +
            sin(theta) * sin(phi) * binormal;
    }

    // Y-up
    template <typename Vec3, typename T>
    inline Vec3 polarCoordinateToDirectionWorldSpace(T theta, T phi)
    {
        return Vec3(sin(theta) * cos(phi), cos(theta), sin(theta) * sin(phi));
    }

    template <typename real, typename Vec3, typename rng>
    Vec3 sample_uniform_sphere_surface(rng &rng) {
        const real tz = rng.next01() * 2 - 1;
        const real phi = rng.next01() * M_PI * 2;
        const real k = sqrt(1.0 - tz * tz);
        const real tx = k * cos(phi);
        const real ty = k * sin(phi);

        return Vec3(tx, ty, tz);
    }

    template <typename real, typename Vec3, typename rng>
    Vec3 sample_uniform_inside_sphere(rng& rng)
    {
        const auto u = 2 * rng.next01() - 1;
        const auto phi = 2 * M_PI * rng.next01();
        const auto r = pow(rng.next01(), 1 / 3.0f);
        const auto z = r * u;
        const auto x = r * cos(phi) * sqrt(1 - z * z);
        const auto y = r * sin(phi) * sqrt(1 - z * z);
        return Vec3(x, y, z);
    }
}

static std::uniform_real_distribution<> dist01(0.0, 1.0);

namespace integrator
{
    using namespace hmath;

    struct Ray
    {
        Float3 org;
        Float3 dir;
        float near = 0;
        float far = std::numeric_limits<float>::infinity();
        bool anyhit = false;

        int id = -1;
    };

    // from smallpt
    struct Sphere
    {
        double rad;
        Float3 p;
        Sphere(double rad_, Float3 p_) :
            rad(rad_), p(p_) {}
        double intersect(const Ray &r) const
        {
            Float3 op = p - r.org;
            double t, eps = 1e-4, b = dot(op, r.dir), det = b * b - dot(op, op) + rad * rad;
            if (det < 0) return 0; else det = sqrt(det);
            return (t = b - det) > eps ? t : ((t = b + det) > eps ? t : 0);
        }
    };

    bool intersect_plane(const Float3&n, const Float3& o, const Float3& org, const Float3& dir, float &t)
    {
        auto d = dot(n, dir);
        if (d < -1e-6f) {
            t = dot(o - org, n) / d;
            return (t >= 0);
        }

        return false;
    }

    struct IntersectionInfo
    {
        bool hit = false;
        Float3 pos;
        Float3 normal;
        int id = -1;
        float t = std::numeric_limits<float>::infinity();
    };

    IntersectionInfo intersect(const Ray &r)
    {
        IntersectionInfo info;

        hstd::rt::Ray ray;
        ray.org = r.org;
        ray.dir = r.dir;
        hstd::rt::Hitpoint hp;

        const auto ret = mesh::oldlib::qbvh.intersect(ray, &hp, r.id, r.anyhit, r.near, r.far);

        if (ret)
        {
            info.t = hp.distance;
            info.id = hp.triangle_index;
            info.hit = ret;

            const auto& triangle = mesh::oldlib::triangles[hp.triangle_index];

            auto e1 = triangle.v[1] - triangle.v[0];
            auto e2 = triangle.v[2] - triangle.v[0];
            auto n = hstd::cross(e1, e2);
            n = hstd::normalize(n);
            info.normal = n;

            const float u = hp.b1;
            const float v = hp.b2;

            info.pos =
                (1 - u - v) * triangle.v[0] +
                u * triangle.v[1] +
                v * triangle.v[2];
        }
        else
        {
            info.hit = false;
        }

        return info;
    }


    Float3 cosine_weighted(Rng &random, const Float3& normal, const Float3& tangent, const Float3& binormal) {
        const float phi = random.next01() * 2.0f * pi<float>();
        const float r2 = random.next01(), r2s = sqrt(r2);

        const float tx = r2s * cos(phi);
        const float ty = r2s * sin(phi);
        const float tz = sqrt(1.0f - r2);

        return tz * normal + tx * tangent + ty * binormal;
    }


    namespace guiding
    {
        struct Param
        {
            float c;
            float k;
            float rho;
            float num_sample;

            float space_jitter_radius;
            float dir_jitter_radius;
        };

        struct Vertex
        {
            Float3 pos;
            Float3 dir;
            float radiance = 0;
        };

        // QuadTree
        // [0, 1) x [0, 1)
        struct QuadTree
        {
            // 全部nullならleaf
            // std::unique_ptr<QuadTree> child[4] = {};
            //QuadTree* child[4] = {};
            
            QuadTree* child = nullptr;

            bool isLeaf() const
            {
//                return !child[0] && !child[1] && !child[2] && !child[3];
                return !child;
            }

            float flux_tmp[NUM_THREAD] = {};
            float flux_total = 0;
            float prob = 0;

            // float flux = 0;
            float pmin[2] = {0, 0};
            float pmax[2] = {1, 1};

            bool inside(float u, float v) const
            {
                return
                    pmin[0] <= u && u <= pmax[0] &&
                    pmin[1] <= v && v <= pmax[1];
            }

            void deallocate()
            {
                if (child)
                {
                    for (int c = 0; c < 4; ++c)
                    {
                        child[c].deallocate();
                    }
                }
                delete[] child;
                child = nullptr;
            }

            QuadTree() = default;

            // copy禁止
            QuadTree(const QuadTree& tree) = delete;
            QuadTree& operator=(const QuadTree& tree) = delete;

            QuadTree& operator=(QuadTree&& tree)
            {
                this->child = tree.child;
                for (int c = 0; c < 4; ++c)
                {
                    this->flux_total = tree.flux_total;
                    this->prob = tree.prob;
                    std::copy(tree.flux_tmp, tree.flux_tmp + NUM_THREAD, this->flux_tmp);
                    this->pmin[0] = tree.pmin[0];
                    this->pmax[0] = tree.pmax[0];
                    this->pmin[1] = tree.pmin[1];
                    this->pmax[1] = tree.pmax[1];
                }
                tree.child = nullptr;
                return *this;
            }

            QuadTree(QuadTree&& tree)
            {
                *this = std::move(tree);
            }

            ~QuadTree()
            {
                deallocate();
            }
        };

        float update_prob(QuadTree* tree, float prob = 0.25f)
        {
            if (!tree->isLeaf())
            {
                float sum = 0;
                for (int c = 0; c < 4; ++c)
                {
                    sum += update_prob(&tree->child[c], prob);
                }
                tree->prob = sum;
                return sum;
            }

            tree->prob = prob;
            return prob;
        }

        void reset(QuadTree* tree)
        {
            if (tree == nullptr)
                return;

            tree->prob = 0;
            tree->flux_total = 0;
            std::fill(tree->flux_tmp, tree->flux_tmp + NUM_THREAD, 0);

            if (!tree->isLeaf())
            {
                for (int c = 0; c < 4; ++c)
                {
                    reset(&tree->child[c]);
                }
            }
        }

        int calc_depth(float u, float v, QuadTree* tree, int depth = 0)
        {
            if (!tree->isLeaf())
            {
                for (int c = 0; c < 4; ++c)
                {
                    if (tree->child[c].inside(u, v))
                    {
                        return calc_depth(u, v, &tree->child[c], depth + 1);
                    }
                }

            }
            return depth;
        }

        void direct_populate(float value, float u, float v, QuadTree* tree)
        {
            if (tree->inside(u, v))
            {
                tree->flux_total += value;
                if (!tree->isLeaf())
                {
                    for (int c = 0; c < 4; ++c)
                    {
                        direct_populate(value, u, v, &tree->child[c]);
                    }
                }
            }
        }

        void populate(int thread_id, const Float3& pos, const Float3& dir, float radiance, QuadTree* tree)
        {
            float theta, phi;
            directionToPolarCoordinate(dir, &theta, &phi);
            const float v = (cos(theta) + 1) / 2; //theta / M_PI;
            const float u = phi / (2 * M_PI);

            if (tree->inside(u, v))
            {
//                tree->flux += radiance; // TODO: なんかおかしくないか？
                tree->flux_tmp[thread_id] += radiance;
                if (!tree->isLeaf())
                {
                    for (int c = 0; c < 4; ++c)
                    {
                        populate(thread_id, pos, dir, radiance, &tree->child[c]);
                    }
                }
            }
        }

        QuadTree copy(const QuadTree* tree)
        {
            QuadTree copy_tree;

            copy_tree.prob = tree->prob;
            copy_tree.flux_total = tree->flux_total;
            std::copy(tree->flux_tmp, tree->flux_tmp + NUM_THREAD, copy_tree.flux_tmp);

            copy_tree.pmin[0] = tree->pmin[0];
            copy_tree.pmin[1] = tree->pmin[1];
            copy_tree.pmax[0] = tree->pmax[0];
            copy_tree.pmax[1] = tree->pmax[1];

            if (!tree->isLeaf())
            {
                copy_tree.child = new QuadTree[4];
                for (int c = 0; c < 4; ++c)
                {
                    copy_tree.child[c] = copy(&tree->child[c]);
                }
            }

            return copy_tree;
        }

        void refine_quad_tree_sub(const Param& p, const float total_flux, QuadTree* tree)
        {
            if (total_flux == 0)
                return;

            const auto current_flux = tree->flux_total;

            if (tree->isLeaf())
            {
                if (tree->flux_total / total_flux > p.rho)
                {
                    // subdivide
                    const float mid[2] = {
                        (tree->pmin[0] + tree->pmax[0]) / 2,
                        (tree->pmin[1] + tree->pmax[1]) / 2,
                    };

                    tree->child = new QuadTree[4];
                    for (int c = 0; c < 4; ++c)
                    {
                        // tree->child[c] = std::unique_ptr<QuadTree>(new QuadTree());
                        tree->child[c].flux_total = tree->flux_total / 4;
                    }

                    tree->child[0].pmin[0] = tree->pmin[0];
                    tree->child[0].pmin[1] = tree->pmin[1];
                    tree->child[0].pmax[0] = mid[0];
                    tree->child[0].pmax[1] = mid[1];
                                  
                    tree->child[1].pmin[0] = mid[0];
                    tree->child[1].pmin[1] = tree->pmin[1];
                    tree->child[1].pmax[0] = tree->pmax[0];
                    tree->child[1].pmax[1] = mid[1];
                                  
                    tree->child[2].pmin[0] = tree->pmin[0];
                    tree->child[2].pmin[1] = mid[1];
                    tree->child[2].pmax[0] = mid[0];
                    tree->child[2].pmax[1] = tree->pmax[1];
                                  
                    tree->child[3].pmin[0] = mid[0];
                    tree->child[3].pmin[1] = mid[1];
                    tree->child[3].pmax[0] = tree->pmax[0];
                    tree->child[3].pmax[1] = tree->pmax[1];

                    for (int c = 0; c < 4; ++c)
                    {
                        refine_quad_tree_sub(p, total_flux, &tree->child[c]);
                    }
                }
                return;
            }

            // node
#if 1
            if (tree->flux_total / total_flux < p.rho)
            {
                // まとめる
                for (int c = 0; c < 4; ++c)
                {
                    tree->child[c].deallocate();
                }
                tree->child = nullptr;
                return;
            }
#endif

            for (int c = 0; c < 4; ++c)
            {
                refine_quad_tree_sub(p, total_flux, &tree->child[c]);
            }
        }

        // 空間構造
        struct BinaryTree
        {
            // 両方nullならleaf
            // 中間nodeなら、両方not null
            std::unique_ptr<BinaryTree> child[2] = {};

            // info
            int split_axis = 0; // x, y, z
            hrt::BBox bbox;
            // uint64_t num_sample = {};
            uint32_t num_sample_table[NUM_THREAD] = {};
            uint32_t num_sample_total = 0;

            bool isLeaf() const
            {
                return !child[0] && !child[1];
            }

            // sample
            // std::vector<Vertex> samples;

            // quad tree
            QuadTree quad_tree;

            // debug
            Float3 debug_color;
        };

        void merge_binary_tree_sub(QuadTree* tree)
        {
            if (tree == nullptr)
                return;

            tree->flux_total = 0;
            for (int i = 0; i < NUM_THREAD; ++i)
            {
                tree->flux_total += tree->flux_tmp[i];
            }

            if (!tree->isLeaf())
            {
                for (int c = 0; c < 4; ++c)
                {
                    merge_binary_tree_sub(&tree->child[c]);
                }
            }
        }

        void merge_binary_tree(BinaryTree* tree)
        {
            if (tree->isLeaf())
            {
                merge_binary_tree_sub(&tree->quad_tree);

                tree->num_sample_total = 0;
                for (int i = 0; i < NUM_THREAD; ++i)
                {
                    tree->num_sample_total +=
                        tree->num_sample_table[i];
                }
            }

            if (!tree->isLeaf())
            {
                for (int c = 0; c < 2; ++c)
                {
                    merge_binary_tree(tree->child[c].get());
                }
            }
        }

        void update_binarytree_prob(BinaryTree* tree)
        {
            if (tree->isLeaf())
            {
                update_prob(&tree->quad_tree);
            }

            if (!tree->isLeaf())
            {
                for (int c = 0; c < 2; ++c)
                {
                    update_binarytree_prob(tree->child[c].get());
                }
            }
        }

        void refine_quad_tree(const Param& p, BinaryTree* tree)
        {
            if (!tree->isLeaf())
            {
                for (int c = 0; c < 2; ++c)
                {
                    refine_quad_tree(p, tree->child[c].get());
                }
            }
            else
            {
                refine_quad_tree_sub(p, tree->quad_tree.flux_total, &tree->quad_tree);
            }
        }

        void reset(BinaryTree* tree)
        {
            // tree->num_sample = 0;
            // tree->samples.clear();
            tree->num_sample_total = 0;
            std::fill(tree->num_sample_table, tree->num_sample_table + NUM_THREAD, 0);
            reset(&tree->quad_tree);

            if (!tree->isLeaf())
            {
                for (int c = 0; c < 2; ++c)
                {
                    reset(tree->child[c].get());
                }
            }
        }

        // Rng global_rng;

        void refine_binary_tree(const Param& p, BinaryTree* tree, int depth = 0)
        {
            auto next = [&]() {
#if 1
                if (depth < 8)
                {
                    std::thread th0([&]() {
                        refine_binary_tree(p, tree->child[0].get(), depth + 1);
                    });
                    std::thread th1([&]() {
                        refine_binary_tree(p, tree->child[1].get(), depth + 1);
                    });

                    th0.join();
                    th1.join();
                }
                else
                {
                    refine_binary_tree(p, tree->child[0].get(), depth + 1);
                    refine_binary_tree(p, tree->child[1].get(), depth + 1);
                }
#else
                refine_binary_tree(p, tree->child[0].get(), depth + 1);
                refine_binary_tree(p, tree->child[1].get(), depth + 1);
#endif
            };

            if (!tree->isLeaf())
            {
                // 中間node
                next();
            }
            else
            {
                // leaf
//                if (tree->num_sample_total > p.c * sqrt(pow(2.0f, p.k)))
                if (tree->num_sample_total > p.c * sqrt(p.num_sample))
                {
                    // subdivide
                    const auto split_axis = tree->split_axis;

                    hrt::BBox child_bbox[2];

                    auto mid = (tree->bbox.pmax[split_axis] + tree->bbox.pmin[split_axis]) / 2;

                    child_bbox[0] = tree->bbox;
                    child_bbox[1] = tree->bbox;
                    child_bbox[0].pmax[split_axis] = mid;
                    child_bbox[1].pmin[split_axis] = mid;

                    for (int c = 0; c < 2; ++c)
                    {
                        tree->child[c] = std::unique_ptr<BinaryTree>(new BinaryTree());
                        tree->child[c]->split_axis = (split_axis + 1) % 3;
                        tree->child[c]->bbox = child_bbox[c];
                        tree->child[c]->num_sample_total = tree->num_sample_total / 2;
#if 0
                        tree->child[c]->debug_color = Float3(
                            global_rng.next01(), 
                            global_rng.next01(), 
                            global_rng.next01());
#endif

                        tree->child[c]->quad_tree = guiding::copy(&tree->quad_tree);
                    }
                    tree->quad_tree = {};
                    next();
                }
            }
        }

        BinaryTree* traverse(BinaryTree* tree, const Float3& x)
        {
            if (!tree->bbox.inside(x))
            {
                return nullptr;
            }

            if (!tree->isLeaf())
            {
                const int axis = tree->split_axis;
                auto med = (tree->bbox.pmin[axis] + tree->bbox.pmax[axis]) / 2;
                if (x[axis] < med)
                {
                    return  traverse(tree->child[0].get(), x);
                }
                else
                {
                    return  traverse(tree->child[1].get(), x);
                }
            }
            
            // leaf
            return tree;
        }


        struct Pt
        {
            float p[2];
            float pdf;
        };

        float pdf_tree(const Float3& dir, const integrator::guiding::QuadTree* tree)
        {
            float pdf = 1;
            float theta, phi;
            directionToPolarCoordinate(dir, &theta, &phi);
            const float v = (cos(theta) + 1) / 2; //theta / M_PI;
            const float u = phi / (2 * M_PI);

            const auto* current = tree;

            for (int depth = 0; ;++depth)
            {
                if (!current->isLeaf())
                {
                    float p00 = current->child[0].prob;
                    float p01 = current->child[1].prob;
                    float p10 = current->child[2].prob;
                    float p11 = current->child[3].prob;
                    const auto sum = p00 + p01 + p10 + p11;

                    for (int c = 0; c < 4; ++c)
                    {
                        if (current->child[c].inside(u, v))
                        {
                            pdf = pdf * 4.0f * current->child[c].prob / sum;
                            current = &current->child[c];
                            break;
                        }
                    }
                }
                else
                {
                    break;
                }
            }

            return pdf;
        }

        Pt sample2(Rng& rng, const integrator::guiding::QuadTree* tree)
        {
            const auto* current = tree;
            float pdf = 1;

            for (;;)
            {
                if (!current->isLeaf())
                {
                    float p00 = current->child[0].prob;
                    float p01 = current->child[1].prob;
                    float p10 = current->child[2].prob;
                    float p11 = current->child[3].prob;
                    const auto sum = p00 + p01 + p10 + p11;

                    float P00 = p00 / sum;
                    float P01 = p01 / sum;
                    float P10 = p10 / sum;
                    float P11 = p11 / sum;

                    const auto u = rng.next01();
                    int index = 0;
                    if (u < P00)
                    {
                        index = 0;
                    }
                    else if (u < P00 + P01)
                    {
                        index = 1;
                    }
                    else if (u < P00 + P01 + P10)
                    {
                        index = 2;
                    }
                    else
                    {
                        index = 3;
                    }

                    pdf = pdf * 4.0f * current->child[index].prob / sum;
                    current = &current->child[index];
                }
                else
                {
                    break;
                }
            }

            Pt pt;
            pt.p[0] = rng.next01() * (current->pmax[0] - current->pmin[0]) + current->pmin[0];
            pt.p[1] = rng.next01() * (current->pmax[1] - current->pmin[1]) + current->pmin[1];
            pt.pdf = pdf;
            
            return pt;
        }

        void sample(const integrator::guiding::QuadTree* tree, Pt* pt, size_t NUM)
        {
            for (int i = 0; i < NUM; ++i)
            {
                auto& cpt = pt[i].p;
                const auto* current = tree;
                pt[i].pdf = 1;

                for (;;)
                {
                    float pmin[2] = { current->pmin[0], current->pmin[1] };
                    float pmax[2] = { current->pmax[0], current->pmax[1] };
                    if (!current->isLeaf())
                    {
                        float p00 = current->child[0].prob;
                        float p01 = current->child[1].prob;
                        float p10 = current->child[2].prob;
                        float p11 = current->child[3].prob;

                        // 上下warp
                        const auto sum = p00 + p01 + p10 + p11;
                        const auto upper = (p00 + p01) / sum;
                        const auto lower = (p10 + p11) / sum;

                        auto ylen = pmax[1] - pmin[1];
                        auto upper_len = ylen * upper;
                        auto lower_len = ylen * lower;
                        auto threshold = pmin[1] + upper_len;

                        int inside_upper = 0;
                        int inside_left = 0;

                        float left, right;

                        if (pmin[1] <= cpt[1] && cpt[1] <= threshold)
                        {
                            inside_upper = 1;

                            // 上側、min側ともいう（左上原点）
                            cpt[1] = (cpt[1] - pmin[1]) / upper_len * (ylen / 2) + pmin[1];

                            // 左右warp
                            const auto sum2 = p00 + p01;
                            left = p00 / sum2;
                            right = p01 / sum2;
                        }
                        else
                        {
                            cpt[1] = (cpt[1] - threshold) / lower_len * (ylen / 2) + (ylen / 2) + pmin[1];

                            // 左右warp
                            const auto sum2 = p10 + p11;
                            left = p10 / sum2;
                            right = p11 / sum2;
                        }

                        // 左右warp
                        auto xlen = pmax[0] - pmin[0];
                        auto left_len = xlen * left;
                        auto right_len = xlen * right;
                        auto threshold2 = pmin[0] + left_len;

                        if (pmin[0] <= cpt[0] && cpt[0] <= threshold2)
                        {
                            inside_left = 1;
                            cpt[0] = (cpt[0] - pmin[0]) / left_len * (xlen / 2) + pmin[0];
                        }
                        else
                        {
                            cpt[0] = (cpt[0] - threshold2) / right_len * (xlen / 2) + (xlen / 2) + pmin[0];
                        }

                        // 次へ
                        bool outside = false; // 数値誤差とかでaabbの外側にでたら処理打ち切る

                        for (int k = 0; k < 2; ++k)
                        {
                            if (pt[i].p[k] < current->pmin[k])
                            {
                                outside = true;
                                pt[i].p[k] = current->pmin[k];
                            }
                            if (pt[i].p[k] > current->pmax[k])
                            {
                                outside = true;
                                pt[i].p[k] = current->pmax[k];
                            }
                        }

                        for (int c = 0; c < 4; ++c)
                        {
                            if (current->child[c].inside(cpt[0], cpt[1]))
                            {
                                constexpr int idx_table[2][2] =
                                {
                                    { 3, 2 },
                                    { 1, 0 },
                                };
                                pt[i].pdf = pt[i].pdf * 4.0f * current->child[idx_table[inside_upper][inside_left]].prob / sum;

                                current = &current->child[c];
                                break;
                            }
                        }

                        if (outside)
                            return;
                    }
                    else
                    {
                        break;
                    }
                }
            }
        }
    }

    guiding::BinaryTree global_binary_tree;
    guiding::BinaryTree* debug_tree;


    Float3 g_camera_position;
    Float3 g_camera_dir;
    Float3 g_screen_side;
    Float3 g_screen_up;
    const float g_length = 0.1f;

    void initialzie_camera(int w, int h)
    {
        // カメラデータを直接突っ込む
#if 0
        Float3 camera_position(15, 9, 10);
        Float3 camera_dir = normalize(Float3(0, 0, 0) - camera_position);
        const float fovy = (90.0f) / 180.0f * hmath::pi<float>();
#endif
        //        hmath::Float3 camera_position(34, 15, 32);

        /*
        hmath::Float3 camera_position(24, 15, 17);
        Float3 camera_dir = normalize(Float3(0, 0, 0) - camera_position);
        const float fovy = (45.0f) / 180.0f * hmath::pi<float>();
        */

#if 1
        // 本番
        hmath::Float3 camera_position(34, 4, 27);
        Float3 camera_dir = normalize(Float3(0, 5, 0) - camera_position);
        const float fovy = (29.0f) / 180.0f * hmath::pi<float>();
#endif

#if 0
        hmath::Float3 camera_position(34, 20, 27);
        Float3 camera_dir = normalize(Float3(0, 5, 0) - camera_position);
        const float fovy = (45.0f) / 180.0f * hmath::pi<float>();
#endif
        //const float ang = 0.15f;
        //Float3 camera_up(sin(ang), cos(ang), 0);
        Float3 camera_up(0, 1, 0);

        const auto aspect = (float)w / h;
        const float screen_height = g_length * tan(fovy / 2);
        const float screen_width = screen_height * aspect;
        const auto screen_side = normalize(cross(camera_dir, camera_up)) * screen_width;
        const auto screen_up = normalize(cross(camera_dir, screen_side)) * screen_height;

        g_camera_position = camera_position;
        g_camera_dir = camera_dir;
        g_screen_side = screen_side;
        g_screen_up = screen_up;
    }

    void get_initial_ray(Rng& rng, int w, int h, int x, int y, Float3& initial_pos, Float3& initial_dir)
    {

        const float U = ((float)(x + rng.next01()) / w) * 2 - 1;
        const float V = ((float)(y + rng.next01()) / h) * 2 - 1;
        const float r = (U * U + V * V);
#if 1
        auto new_U = U;
        auto new_V = V;
#else
#if 0
        int ch = rng.next() % 3;
        float table[3] = { -0.05f, 0, 0.05f };
        const float k1 = table[ch];
        const float new_U = U * (1.0f + k1 * r);
        const float new_V = V * (1.0f + k1 * r);
#else
        const float cjitter = rng.next01() * 0.01 - 0.005;
        const float ch = 0.1f + cjitter;

        const float k1 = ch;
        const float new_U = U * (1.0f + k1 * r);
        const float new_V = V * (1.0f + k1 * r);

        const float U2 = U * aspect;
        const float r2 = (U2 * U2 + V * V);
        const float a = 2.0f;
        const float final_weight = a / (a + r2);
#endif
#endif

        initial_pos = g_camera_position + g_camera_dir * g_length + new_U * g_screen_side + new_V * g_screen_up;
        initial_dir = normalize(initial_pos - g_camera_position);
    }

    std::mutex mutex;

// #define PATHTRACING
    // ここに素晴らしいintegratorを書く
    Float3 get_radiance(const integrator::guiding::Param& p, int thread_id, Rng& rng, Float3& initial_pos, Float3& initial_dir, int loop, int sample, int total_sample, int w, int h, int x, int y)
    {
#if 0
        // guiding debug
        if (x == 0 && y == 0 && sample == 0)
        {
            integrator::Ray ray;
            ray.org = initial_pos;
            ray.dir = initial_dir;
            auto info = intersect(ray);
            debug_tree = guiding::traverse(&global_binary_tree, info.pos);
        }
#endif

        // naive pathtracing
#if 1
        integrator::Ray ray;
        ray.org = initial_pos;
        ray.dir = initial_dir;
        Float3 throughput(1, 1, 1);
        Float3 L(0, 0, 0);

        bool into = true;

        constexpr int MAX_DEPTH = 5;

        // guiding
        guiding::Vertex vertex_list[MAX_DEPTH];
        Float3 weight[MAX_DEPTH + 1];
        Float3 final_emission;
        guiding::BinaryTree* item_list[MAX_DEPTH + 1] = {};

        int depth;
        for (depth = 0; depth < MAX_DEPTH; ++depth)
        {
            auto org_ray = ray;
            auto info = intersect(ray);
            if (!info.hit)
            {
                // 背景
//                L += product(throughput, Float3(0.01, 0.01, 0.01));
//                L += product(throughput, Float3(0.1, 0.1, 0.1));
                break;
            }
            ray.org = info.pos;
            ray.id = info.id;

            // guiding
            {
                vertex_list[depth].pos = org_ray.org;
                vertex_list[depth].dir = org_ray.dir;
            }

            const auto& triangle = mesh::oldlib::triangles[info.id];
            const auto& material = *triangle.mat;

            guiding::BinaryTree* item = nullptr;

            item = guiding::traverse(&global_binary_tree, info.pos);

            item_list[depth + 1] = item;

            if (material.ior == 0)
            {
                float Prr = 0;

                if (loop == 1 || !item)
                    Prr = 1;
                else
                    Prr = 0.33; // TODO: 改良の余地あり

#ifdef PATHTRACING
                Prr = 1;
#endif

                if (rng.next01() <= Prr)
                {
                    const auto ts = hmath::tangentSpace(info.normal);
                    const Float3 tangent = std::get<0>(ts);
                    const Float3 binormal = std::get<1>(ts);

                    // 次の方向サンプリング
                    auto next_dir = cosine_weighted(rng, info.normal, tangent, binormal);
                    const float pdf_omega = dot(info.normal, next_dir) / M_PI;
                    float MISWeight = 1;
#ifndef PATHTRACING
                    if (Prr != 1)
                    {
                        float pdf_guiding = guiding::pdf_tree(next_dir, &item->quad_tree);
                        float pdf_bsdf = pdf_omega;
                        MISWeight = (1 / Prr) * (pdf_bsdf / (pdf_bsdf + pdf_guiding));
                    }
#endif

                    ray.dir = next_dir;
                    L += product(throughput, material.emission);
                    const float coeff = (float)(dot(info.normal, next_dir) / M_PI / pdf_omega) * MISWeight;
                    throughput = product(throughput, material.diffuse * coeff);
                }
                else
                {
                    // 次の方向サンプリング
                    //guiding::Pt pt;
                    //pt.p[0] = rng.next01();
                    //pt.p[1] = rng.next01();
                    //guiding::sample(&item->quad_tree, &pt, 1);
#if 1
                    auto pt = guiding::sample2(rng, &item->quad_tree);
                    const float pdf_omega = pt.pdf / (4.0f * M_PI);
                    const float phi = pt.p[0] * 2 * M_PI;
                    const float theta = acos(pt.p[1] * 2 - 1);
                    Float3 next_dir = polarCoordinateToDirectionWorldSpace<Float3>(theta, phi);
#else
                    const auto ts = hmath::tangentSpace(info.normal);
                    const Float3 tangent = std::get<0>(ts);
                    const Float3 binormal = std::get<1>(ts);
                    Float3 next_dir = cosine_weighted(rng, info.normal, tangent, binormal);
                    const float pdf_omega = 1 / (4.0f * M_PI);
#endif

                    float MISWeight = 1;
                    {
                        float pdf_guiding = pdf_omega;
                        float pdf_bsdf = dot(info.normal, next_dir) / M_PI;
                        MISWeight = (1 / (1 - Prr)) * (pdf_guiding / (pdf_bsdf + pdf_guiding));
                    }

                    if (MISWeight < 0)
                        MISWeight = 0;

                    ray.dir = next_dir;
                    L += product(throughput, material.emission);
                    const float coeff = (float)(std::max(0.0f, dot(info.normal, next_dir)) / M_PI / pdf_omega) * MISWeight;
                    throughput = product(throughput, material.diffuse * coeff);
                }

                // guiding
                if (depth + 1 < MAX_DEPTH)
                {
                    weight[depth + 1] = material.diffuse;
                }

                auto lumi = dot(rgb2y, material.emission);

                // 光源にhitしたら終了する
                if (lumi > 0)
                {
                    final_emission = material.emission;
                    break;
                }
            }
            else
            {
                integrator::Ray reflection_ray;
                reflection_ray.org = info.pos;
                reflection_ray.dir = ray.dir - info.normal * 2.0f * dot(info.normal, ray.dir);

                // Snellの法則
                const float nc = 1.0; // 真空の屈折率
                const float nt = material.ior; // オブジェクトの屈折率
                const float nnt = into ? nc / nt : nt / nc;
                into = !into;

                const auto orienting_normal = dot(info.normal, ray.dir) < 0.0 ? info.normal : (-1.0f * info.normal);
                const float ddn = dot(ray.dir, orienting_normal);
                const float cos2t = 1.0 - nnt * nnt * (1.0 - ddn * ddn);

                if (cos2t < 0.0) { // 全反射
                    ray = reflection_ray;
                    continue;
                }

                // 屈折の方向
                integrator::Ray refraction_ray;
                refraction_ray.org = info.pos;
                refraction_ray.dir = normalize(ray.dir * nnt - info.normal * (into ? 1.0f : -1.0f) * (ddn * nnt + sqrt(cos2t)));

                // SchlickによるFresnelの反射係数の近似を使う
                const double a = nt - nc, b = nt + nc;
                const double R0 = (a * a) / (b * b);

                const float c = 1.0 - (into ? -ddn : dot(refraction_ray.dir, -1.0f * orienting_normal));
                const float Re = R0 + (1.0 - R0) * pow(c, 5.0); // 反射方向の光が反射してray.dirの方向に運ぶ割合。同時に屈折方向の光が反射する方向に運ぶ割合。
                const float nnt2 = pow(into ? nc / nt : nt / nc, 2.0); // レイの運ぶ放射輝度は屈折率の異なる物体間を移動するとき、屈折率の比の二乗の分だけ変化する。
                const float Tr = (1.0 - Re) * nnt2; // 屈折方向の光が屈折してray.dirの方向に運ぶ割合


                const float probability = 0.25f + 0.5f * Re;
                if (rng.next01() < probability) { // 反射
                    ray = reflection_ray;
                    throughput = product(throughput, Re * material.diffuse / probability);
                }
                else
                {
                    // 屈折
                    ray = refraction_ray;
                    throughput = product(throughput, Tr * material.diffuse / (1.0f - probability));
                }
            }

        }

        // guiding処理
        if (depth <  MAX_DEPTH)
        {
            const auto num = depth;
            auto lumi = dot(rgb2y, final_emission);
            if (lumi > 0)
            {
                vertex_list[depth].radiance = dot(rgb2y, final_emission);

                for (depth = depth - 1; depth >= 0; --depth)
                {
                    vertex_list[depth].radiance =
                        vertex_list[depth + 1].radiance * dot(rgb2y, weight[depth + 1]);
                }

                // i = 0はカメラ原点なので無視
                for (int i = 1; i < num; ++i)
                {
                    // jitter
                    vertex_list[i].pos += p.space_jitter_radius * sample_uniform_inside_sphere<float, hmath::Float3>(rng);
                    vertex_list[i].dir += p.dir_jitter_radius * sample_uniform_inside_sphere<float, hmath::Float3>(rng);
                    vertex_list[i].dir = normalize(vertex_list[i].dir);

                    //auto* item = guiding::traverse(&global_binary_tree, vertex_list[i].pos);
                    auto* item = item_list[i];
                    if (item)
                    {
                        ++item->num_sample_table[thread_id];
                        guiding::populate(thread_id, vertex_list[i].pos, vertex_list[i].dir, vertex_list[i].radiance / total_sample, &item->quad_tree);
                    }
                }
            }
        }

        return L;
#endif
#if 0
        {
            integrator::Ray ray;
            ray.org = initial_pos;
            ray.dir = initial_dir;

            auto info = intersect(ray);
            float occluded = 0;
            if (info.hit)
            {
                auto hitpos = info.pos;

                const auto ts = hmath::tangentSpace(info.normal);
                const Float3 tangent = std::get<0>(ts);
                const Float3 binormal = std::get<1>(ts);

                // ao
                for (int i = 0; i < 1; ++i)
                {
                    auto next_dir = cosine_weighted(rng, info.normal, tangent, binormal);
                    integrator::Ray aoray;

                    aoray.org = hitpos;
                    aoray.dir = next_dir;
                    aoray.id = info.id;
                    aoray.near = 0;
                    aoray.anyhit = true;
                    auto ao_info = intersect(aoray);

                    if (ao_info.hit)
                    {
                        occluded += 1.0;
                    }
                }
            }
            else
            {
                return {};
            }

            return (1 - occluded) * Float3(1, 1, 1);
        }
#endif
    }
}

void test()
{
    integrator::guiding::QuadTree tree;
    std::mt19937 gen{ 0 };
    std::normal_distribution<> x_dist(0.2, 0.3);
    std::normal_distribution<> y_dist(0.5, 0.2);

    hmath::Rng rng;

    for (int loop = 0; loop < 64; ++loop)
    {
        for (int i = 0; i < 8192; ++i)
        {
            // integrator::guiding::direct_populate(1.0f, x_dist(gen), y_dist(gen), &tree);
            hmath::Float3 dir = sample_uniform_sphere_surface<float, hmath::Float3>(rng);
            float theta, phi;
            directionToPolarCoordinate(dir, &theta, &phi);
            // float v = theta / M_PI;
            float v = (cos(theta) + 1) / 2;
            float u = phi / (2 * M_PI);
            integrator::guiding::direct_populate(1.0f, u, v, &tree);
        }
        integrator::guiding::Param p;
        p.c = 12000; // TODO: ハイパラ
        p.rho = 0.01;
        integrator::guiding::refine_quad_tree_sub(p, tree.flux_total, &tree);
        integrator::guiding::reset(&tree);
    }
    integrator::guiding::update_prob(&tree);

    // warping
        
    constexpr int NUM = 8192;

    integrator::guiding::Pt pt[NUM];
    for (int i = 0; i < NUM; ++i)
    {
        pt[i].p[0] = dist01(gen);
        pt[i].p[1] = dist01(gen);
    }

    integrator::guiding::sample(&tree, pt, NUM);

    constexpr int W = 256;
    std::vector<float> fdata(W * W * 3);
#if 1
    for (int i = 0; i < NUM; ++i)
    {
        const float u = pt[i].p[0];
        const float v = pt[i].p[1];

        const int ix = u * W;
        const int iy = v * W;
        fdata[(ix + iy * W) * 3 + 0] = pt[i].pdf;
    }
#endif
#if 0
    for (int iy = 0; iy < W; ++iy)
    {
        for (int ix = 0; ix < W; ++ix)
        {
            const float u = (float)(ix+0.5f) / W;
            const float v = (float)(iy+0.5f) / W;
            auto depth = integrator::guiding::calc_depth(u, v, &tree);
            fdata[(ix + iy * W) * 3 + 0] = depth / 10.0f;
        }
    }
#endif
    hhdr::save("tree.hdr", fdata.data(), (int)W, (int)W, false);
}

void test2()
{
    integrator::guiding::QuadTree tree;
    std::mt19937 gen{ 0 };
    hmath::Rng rng;

    for (int loop = 0; loop < 64; ++loop)
    {
        for (int i = 0; i < 8192; ++i)
        {
            // integrator::guiding::direct_populate(1.0f, x_dist(gen), y_dist(gen), &tree);
            while (1)
            {
                hmath::Float3 dir = sample_uniform_sphere_surface<float, hmath::Float3>(rng);
                float theta, phi;
                directionToPolarCoordinate(dir, &theta, &phi);

                if (-0.9 <= dir[2] && dir[2] < -0.7 &&
                    0 < dir[1] && dir[1] < 1)
                {
                    // float v = theta / M_PI;
                    float v = (cos(theta) + 1) / 2;
                    float u = phi / (2 * M_PI);
                    integrator::guiding::direct_populate(1.0f, u, v, &tree);
                    break;
                }
            }
        }
        integrator::guiding::Param p;
        p.c = 12000; // TODO: ハイパラ
        p.rho = 0.001;
        integrator::guiding::refine_quad_tree_sub(p, tree.flux_total, &tree);
        integrator::guiding::reset(&tree);
    }
    integrator::guiding::update_prob(&tree);

    constexpr int W = 256;
    std::vector<float> fdata(W * W * 3);
    for (int iy = 0; iy < W; ++iy)
    {
        for (int ix = 0; ix < W; ++ix)
        {
            const float u = (float)(ix + 0.5f) / W;
            const float v = (float)(iy + 0.5f) / W;
            auto depth = integrator::guiding::calc_depth(u, v, &tree);
            fdata[(ix + iy * W) * 3 + 0] = depth / 10.0f;
        }
    }
    hhdr::save("tree.hdr", fdata.data(), (int)W, (int)W, false);

    constexpr int NUM = 64;

    integrator::guiding::Pt pt[NUM];
    for (int i = 0; i < NUM; ++i)
    {
        pt[i].p[0] = dist01(gen);
        pt[i].p[1] = dist01(gen);
    }
    integrator::guiding::sample(&tree, pt, NUM);

    for (int i = 0; i < NUM; ++i)
    {
        const float phi = pt[i].p[0] * 2 * M_PI;
        const float theta = acos(pt[i].p[1] * 2 - 1);
        auto next_dir = polarCoordinateToDirection<float>(theta, phi, hmath::Float3(0, 1, 0), hmath::Float3(1, 0, 0), hmath::Float3(0, 0, 1));

        printf("(%f, %f), <%f, %f, %f>\n", theta, phi, next_dir[0], next_dir[1], next_dir[2]);
    }
}

int main(int argc, char** argv)
{
    //test2();
    //return 0;
#if 0
    integrator::guiding::QuadTree tree;
    tree.flux = 100;
    integrator::guiding::Param p;
    p.c = 12000; // TODO: ハイパラ
    p.rho = 0.1;
    integrator::guiding::refine_quad_tree(p, 100, &tree);


    auto copy_tree = integrator::guiding::copy(&tree);

    return 0;
#endif
    /*
    const int Width = 1920 / 2;
    const int Height = 1080 / 2;
    */

    const int Width = 1280;
    const int Height = 720;

    integrator::initialzie_camera(Width, Height);

//    bool end_flag = false;

    // メッシュ読み込み
    printf("Load and Build.\n");
//    auto mesh = mesh::loadMesh("head.obj");
//    auto mesh = mesh::loadMesh("C:/Code/VSProjects/pathological_mesher/pathological_mesher/rtcamp.obj");
    auto mesh = mesh::loadMesh("rtcamp.obj");
    {
        const auto numFace = mesh.index.size() / 3;
        mesh::oldlib::triangles.resize(numFace);
        mesh::oldlib::ref_triangles.resize(numFace);
        for (size_t i = 0; i < numFace; ++i)
        {
            for (int x = 0; x < 3; ++x)
            {
                /*
                mesh::oldlib::triangles[i].v[0][a] = impl::vertices[indices[i * 6 + 0] * 3 + a];
                mesh::oldlib::triangles[i].v[1][a] = impl::vertices[indices[i * 6 + 2] * 3 + a];
                mesh::oldlib::triangles[i].v[2][a] = impl::vertices[indices[i * 6 + 4] * 3 + a];
                mesh::oldlib::triangles[i].n[0][a] = impl::normals[indices[i * 6 + 1] * 3 + a];
                mesh::oldlib::triangles[i].n[1][a] = impl::normals[indices[i * 6 + 3] * 3 + a];
                mesh::oldlib::triangles[i].n[2][a] = impl::normals[indices[i * 6 + 5] * 3 + a];
                */

                mesh::oldlib::triangles[i].v[0][x] = mesh.vertex[mesh.index[i * 3 + 0] * 3 + x];
                mesh::oldlib::triangles[i].v[1][x] = mesh.vertex[mesh.index[i * 3 + 1] * 3 + x];
                mesh::oldlib::triangles[i].v[2][x] = mesh.vertex[mesh.index[i * 3 + 2] * 3 + x];
            }

            mesh::oldlib::triangles[i].mat = &mesh::material_table[
                mesh.material_map[i * 3 + 0]];

            for (int a = 0; a < 3; ++a)
            {
                mesh::oldlib::ref_triangles[i].p[a] = &mesh::oldlib::triangles[i].v[a];
            }
            mesh::oldlib::ref_triangles[i].original_triangle_index = i;
        }
        mesh::oldlib::qbvh.build(mesh::oldlib::ref_triangles);
    }
    printf("Build BVH End.\n");

    // setup
    {
        integrator::global_binary_tree.bbox = hrt::BBox(
            hmath::Float3(-100, -50, -100),
            hmath::Float3(100,  50, 100));
    
    }

    integrator::guiding::Param p;
    p.c = 120; // TODO: ハイパラ
    p.rho = 0.01f;
    p.space_jitter_radius = 0.1f;
    p.dir_jitter_radius = 0.5f;

    constexpr int LOOP = 64;

    FloatImage result_image(Width, Height);
    FloatImage image[LOOP];

    for (int i = 0; i < LOOP; ++i)
        image[i] = FloatImage(Width, Height);

    FloatImage current_image(Width, Height);
    double image_weight[LOOP];

    int loop = 1;

#if 1

    std::atomic<bool> end_flag;
    end_flag.store(false);

    // 時間監視スレッドを立てる
    double tmp_image_weight[LOOP];
    std::thread watcher([&end_flag, &tmp_image_weight, Width, Height, &image, &loop, &image_weight]() {
        FloatImage result_image(Width, Height);
        int image_index = 0;
        char buf[256];

        // 開始時間を取得しておく
        auto start = std::chrono::system_clock::now();

        auto tick_start = start;
        for (;;) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 0.1秒眠る

            // 15秒経過を計る
            auto current = std::chrono::system_clock::now();

            const auto total_ms = std::chrono::duration_cast<std::chrono::milliseconds>(current - start).count();
            const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(current - tick_start).count();

//            const auto last = total_ms >= 57 * 1000;

            const auto last = total_ms >= 57 * 1000;
            if (ms >= 5 * 1000 || last)
            {
                const auto current_loop = loop;
                for (int i = 1; i < current_loop; ++i)
                {
                    tmp_image_weight[i] = image_weight[i];
                }

#if 0
                // image varを計算 (tmp)
                double tmp_num_sample = 0;
                for (int iy = 0; iy < Height; ++iy)
                {
                    for (int ix = 0; ix < Width; ++ix)
                    {
                        tmp_num_sample += image[current_loop].samples(ix, iy);
                    }
                }
                tmp_image_weight[current_loop] = tmp_num_sample / (Width * Height);

                double tmp_image_weight_sum = 0;
                for (int i = 1; i <= current_loop; ++i)
                {
                    tmp_image_weight_sum += tmp_image_weight[i];
                }
#endif


                double average_sample_per_pixel = 0;
                for (int iy = 0; iy < Height; ++iy)
                {
                    for (int ix = 0; ix < Width; ++ix)
                    {
                        tmp_image_weight[current_loop] = image[current_loop].samples(ix, iy);

                        double tmp_image_weight_sum = 0;
                        for (int i = 1; i <= current_loop; ++i)
                        {
                            tmp_image_weight_sum += tmp_image_weight[i];
                            average_sample_per_pixel += image[i].samples(ix, iy);
                        }

                        hmath::Double3 col;
                        for (int i = 1; i <= current_loop; ++i)
                        {
                            if (image[i].samples(ix, iy) > 0)
                            {
                                col = col +
                                    (tmp_image_weight[i] / tmp_image_weight_sum) * image[i](ix, iy) / (double)image[i].samples(ix, iy);
                            }
                        }
                        result_image(ix, iy) = col;
                        result_image.samples(ix, iy) = 1;
                    }
                }

                printf("Average Sample per Pixel: %.2f\n", average_sample_per_pixel / (Width * Height));

                tick_start = current;

                if (!last)
                {
                    sprintf(buf, "%03d.png", image_index);
                }
                else
                {
                    sprintf(buf, "final_image.png", image_index);
                }
                save_image(buf, result_image);
                std::cout << "Saved: " << buf << std::endl;
                ++image_index;

                if (last)
                {
                    end_flag.store(true);
                    return;
                }

#if 0
                // 画像出力
                tick_start = current;

                sprintf(buf, "%03d.png", image_index);
                save_image(buf, image);
                std::cout << "Saved: " << buf << std::endl;
                ++image_index;
#endif
            }


#if 0

            // 123秒経過を計る
            if (std::chrono::duration_cast<std::chrono::milliseconds>(current - start).count() >= 122 * 1000 /* 安全をみてちょっと早めにする */) {
                // 画像出力して全部終了じゃい
                end_flag = true;
                save_image("final_image.png", image, true);
                std::cout << "Saved: final_image.png" << std::endl;
                return;
            }
#endif
        }
    });
#endif
    // omp_set_num_threads(NUM_THREAD);

    char buf[256];
    int image_index = 0;
    FloatImage raw_image(Width, Height);
    for (loop = 1; loop < LOOP; ++loop)
    {
        int num_sample = 1 << (loop - 1);

        p.k = loop;
        p.num_sample = num_sample;

        printf("loop: %d\n", loop);

        if (end_flag.load())
            break;


#if 0
#pragma omp parallel for schedule(dynamic)
        for (int iy = 0; iy < Height; ++iy)
        {
            if (end_flag.load())
                break;

            for (int ix = 0; ix < Width; ++ix)
            {
                if (end_flag.load())
                    break;

                const int tid = omp_get_thread_num();
                const int current_seed = (ix + iy * 8192);

                hmath::Rng rng;
                rng.set_seed(current_seed);

                for (int s = 0; s < num_sample; ++s)
                {
                    if (end_flag.load())
                        break;

                    hmath::Float3 pos, dir;
                    integrator::get_initial_ray(rng, Width, Height, ix, iy, pos, dir);

                    auto ret = integrator::get_radiance(p, tid, rng, pos, dir, loop, s, num_sample, Width, Height, ix, iy);

                    if (is_valid(ret)) {
                        const auto dret = hmath::Double3(ret[0], ret[1], ret[2]);

                        // 分散計算用
                        const auto lumi = dot(dret, double_rgb2y);
                        image[loop].mean_var(ix, iy)[0] += lumi;
                        image[loop].mean_var(ix, iy)[1] += lumi * lumi;
                        image[loop](ix, iy) += dret;
                        image[loop].samples(ix, iy) += 1;
                    }
                }
            }
        }
#endif
        struct Work
        {
            int begin_x = 0;
            int begin_y = 0;
            int end_x = 0;
            int end_y = 0;
            int num_sample = 0;
            int loop = 0;

            FloatImage* image = nullptr;
            std::atomic<bool> working = false;

            Work() {}

            Work(const Work& w) noexcept
            {
                begin_x = w.begin_x;
                begin_y = w.begin_y;
                end_x = w.end_x;
                end_y = w.end_y;
                num_sample = w.num_sample;
                loop = w.loop;
                image = w.image;
                working.store(w.working.load());
            }
        };

        std::vector<Work> work_list;
        int work_list_index = 0;
        int num_loop = 0;

        static constexpr int BlockWidth = 16;
        static constexpr int BlockHeight = 16;
        for (int begin_y = 0; begin_y < Height; begin_y += BlockHeight) {
            for (int begin_x = 0; begin_x < Width; begin_x += BlockWidth) {
                Work w;

                w.begin_x = begin_x;
                w.begin_y = begin_y;
                w.end_x = hmath::clamp(begin_x + BlockWidth, 0, Width);
                w.end_y = hmath::clamp(begin_y + BlockHeight, 0, Height);
                w.image = &image[loop];
                w.num_sample = num_sample;
                w.loop = loop;

                work_list.push_back(w);
            }
        }

        // スレッドに処理を振り分ける
        std::vector<std::thread> thread_list;
        std::mutex work_list_mutex;

        for (int thread_id = 0; thread_id < NUM_THREAD; ++thread_id) {
            std::thread thread([&loop, &p, Width, Height, &end_flag, thread_id, &work_list, &work_list_index, &num_loop, &work_list_mutex]() {
                hetc::set_thread_group(thread_id);

                hmath::Rng rng;
                rng.set_seed(thread_id + loop * NUM_THREAD);

                for (;;) {
                    Work* current_work = nullptr;
                    {
                        std::lock_guard<std::mutex> lock(work_list_mutex);

                        if (work_list_index < work_list.size())
                        {
                            if (!work_list[work_list_index].working.load()) {
                                current_work = &work_list[work_list_index];
                                work_list[work_list_index].working.store(true);
                            }
                            work_list_index++;
                        }
                    }

                    if (current_work == nullptr) {
                        break;
                    }

                    // タスク処理
                    {
                        for (int iy = current_work->begin_y; iy < current_work->end_y; ++iy)
                        {
                            if (end_flag.load())
                                break;

                            for (int ix = current_work->begin_x; ix < current_work->end_x; ++ix)
                            {
                                if (end_flag.load())
                                    break;

                                for (int s = 0; s < current_work->num_sample; ++s)
                                {
                                    if (end_flag.load())
                                        break;

                                    hmath::Float3 pos, dir;
                                    integrator::get_initial_ray(rng, Width, Height, ix, iy, pos, dir);

                                    auto ret = integrator::get_radiance(p, thread_id, rng, pos, dir, current_work->loop, s, current_work->num_sample, Width, Height, ix, iy);

                                    if (is_valid(ret)) {
                                        const auto dret = hmath::Double3(ret[0], ret[1], ret[2]);

                                        // 分散計算用
                                        const auto lumi = dot(dret, double_rgb2y);
                                        (*current_work->image).mean_var(ix, iy)[0] += lumi;
                                        (*current_work->image).mean_var(ix, iy)[1] += lumi * lumi;
                                        (*current_work->image)(ix, iy) += dret;
                                        (*current_work->image).samples(ix, iy) += 1;
                                    }
                                }
                            }
                        }
                    }
                }
            });

            thread_list.push_back(std::move(thread));
        }

        for (auto& t : thread_list) {
            t.join();
        }

#if 0
        // image varを計算
        double weight = 0;
        for (int iy = 0; iy < Height; ++iy)
        {
            for (int ix = 0; ix < Width; ++ix)
            {
                const double samples = image[loop].samples(ix, iy);
                const double lumi2 = image[loop].mean_var(ix, iy)[1];
                const double lumi = image[loop].mean_var(ix, iy)[0];
                const double X = lumi2 / samples;
                const double Y = pow(lumi / samples, 2);
                const double var = X - Y;
                if (var > 0)
                {
                    weight += (1 / var) / (Width * Height);
                    raw_image(ix, iy) = hmath::Float3(var, 0, 0);
                    raw_image.samples(ix, iy) = 1;
                }
            }
        }
#endif
        // weightはサンプル数にします
        double weight = num_sample;
#if 0
        printf("\n========================\n");
        printf("%e\n", weight);
        printf("========================\n");
#endif
        image_weight[loop] = weight;

        double image_weight_sum = 0;
        for (int i = 1; i <= loop; ++i)
        {
            image_weight_sum += image_weight[i];
        }

#if 0
        // 重み付き和を計算
#pragma omp parallel for schedule(dynamic)
        for (int iy = 0; iy < Height; ++iy)
        {
            for (int ix = 0; ix < Width; ++ix)
            {

                hmath::Double3 col;
                for (int i = 1; i <= loop; ++i)
                {
                    col = col + (image_weight[i] / image_weight_sum) * image[i](ix, iy) / (double)image[i].samples(ix, iy);
                }

                result_image(ix, iy) = col;
                result_image.samples(ix, iy) = 1;

                raw_image(ix, iy) = image[loop](ix, iy) / (double)image[loop].samples(ix, iy);
                raw_image.samples(ix, iy) = 1;
            }
        }
#endif

#if 0
        printf("\n");
        for (int i = 1; i <= loop; ++i)
        {
            printf("[%f]", (image_weight[i] / image_weight_sum));
        }
        printf("\n");
#endif
        // debug
        if (0)
        {

#if 0
            hmath::Rng rng;
            rng.set_seed(0);
            hmath::Float3 pos, dir;
            integrator::get_initial_ray(rng, Width, Height, Width/2, Height/2, pos, dir);
            integrator::Ray ray;
            ray.org = pos;
            ray.dir = dir;
            auto info = intersect(ray);
            auto* tree = integrator::guiding::traverse(&integrator::global_binary_tree, info.pos);

            for (int i = 0; i < 16; ++i)
            {
                integrator::guiding::Pt pt;
                pt.p[0] = rng.next01();
                pt.p[1] = rng.next01();
                sample(&tree->quad_tree, &pt, 1);

                const float phi = pt.p[0] * 2 * M_PI;
                const float theta = acos(pt.p[1] * 2 - 1);
                auto next_dir = polarCoordinateToDirectionWorldSpace<hmath::Float3>(theta, phi);
            }

            for (int iy = 0; iy < Height; ++iy)
            {
                for (int ix = 0; ix < Width; ++ix)
                {
                    if (tree)
                    {
                        image[loop].samples(ix, iy) = 1;

                        //image(ix, iy) = tree->debug_color;
                        //image(ix, iy) = hmath::Float3(tree->quad_tree.flux / 10 / num_sample, 0, 0);

#if 1
                        auto& aabb = tree->bbox;

                        const float u = (ix + 0.5f) / Width;
                        const float v = (iy + 0.5f) / Height;

                        auto depth = integrator::guiding::calc_depth(u, v, &tree->quad_tree);

                        hmath::Float3 tbl[] = {
                            hmath::Float3(1, 0, 0),
                            hmath::Float3(0, 1, 0),
                            hmath::Float3(0, 0, 1),
                            hmath::Float3(1, 1, 0),
                            hmath::Float3(0, 1, 1),
                            hmath::Float3(1, 1, 1),
                        };

                        if (depth >= 6)
                            image[loop](ix, iy) = hmath::Float3(0.1, 0.1, 0.1);
                        else
                            image[loop](ix, iy) = tbl[depth];

#endif
                    }
                }
            }
#endif
#if 1
            for (int iy = 0; iy < Height; ++iy)
            {
                for (int ix = 0; ix < Width; ++ix)
                {
                    {
                        hmath::Rng rng;
                        rng.set_seed(0);

                        hmath::Float3 pos, dir;
                        integrator::get_initial_ray(rng, Width, Height, ix, iy, pos, dir);

                        // debug
                        {
                            integrator::Ray ray;
                            ray.org = pos;
                            ray.dir = dir;
                            auto info = intersect(ray);
                            auto* tree = integrator::guiding::traverse(&integrator::global_binary_tree, info.pos);

                            if (tree)
                            {
                                //image(ix, iy) = tree->debug_color;
                                //image(ix, iy) = hmath::Float3(tree->quad_tree.flux / 10 / num_sample, 0, 0);
                                
#if 1
                                auto& aabb = tree->bbox;

                                const auto d = (info.pos - aabb.pmin) / (aabb.pmax - aabb.pmin);
                                const float u = d[0];
                                const float v = d[1];

                                auto depth = integrator::guiding::calc_depth(u, v, &tree->quad_tree);

                                hmath::Float3 tbl[] = {
                                    hmath::Float3(1, 0, 0),
                                    hmath::Float3(0, 1, 0),
                                    hmath::Float3(0, 0, 1),
                                    hmath::Float3(1, 1, 0),
                                    hmath::Float3(0, 1, 1),
                                    hmath::Float3(1, 1, 1),
                                };

                                if (depth >= 6)
                                    result_image(ix, iy) = hmath::Float3(0.1, 0.1, 0.1);
                                else
                                    result_image(ix, iy) = tbl[depth];

#endif
                            }
                        }

                    }
                }
            }
#endif
        }

        // refine guiding
        {
            printf("BEGIN refine\n");
            integrator::guiding::merge_binary_tree(&integrator::global_binary_tree);
            integrator::guiding::refine_quad_tree(p, &integrator::global_binary_tree);
            integrator::guiding::refine_binary_tree(p, &integrator::global_binary_tree);
            integrator::guiding::reset(&integrator::global_binary_tree);
            integrator::guiding::update_binarytree_prob(&integrator::global_binary_tree);
            printf("END refine\n");
        }

        printf("END loop\n\n");
#if 0
        // save
        {
            sprintf(buf, "%03d.png", image_index);
//            save_image(buf, image[loop]);
            save_image(buf, result_image);

            sprintf(buf, "raw_%03d.png", image_index);
            save_image(buf, raw_image);

            std::cout << "Saved: " << buf << std::endl << std::endl;
            ++image_index;
        }
#endif
    }



#if 0
    // 時間監視スレッドを立てる
    std::thread watcher([&end_flag, &image]() {
        int image_index = 0;
        char buf[256];

        // 開始時間を取得しておく
        auto start = std::chrono::system_clock::now();

        auto tick_start = start;
        for (;;) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 0.1秒眠る

            // 15秒経過を計る
            auto current = std::chrono::system_clock::now();
            if (std::chrono::duration_cast<std::chrono::milliseconds>(current - tick_start).count() >= 3 * 1000) {
                // 画像出力
                tick_start = current;

                sprintf(buf, "%03d.png", image_index);
                save_image(buf, image);
                std::cout << "Saved: " << buf << std::endl;
                ++image_index;
            }

            // 123秒経過を計る
            if (std::chrono::duration_cast<std::chrono::milliseconds>(current - start).count() >= 122 * 1000 /* 安全をみてちょっと早めにする */) {
                // 画像出力して全部終了じゃい
                end_flag = true;
                save_image("final_image.png", image, true);
                std::cout << "Saved: final_image.png" << std::endl;
                return;
            }
        }
    });

    // workリストを適当にこしらえる
    struct Work
    {
        int begin_x = 0;
        int begin_y = 0;
        int end_x = 0;
        int end_y = 0;

        FloatImage* image = nullptr;
        std::atomic<bool> working = false;

        Work() {}

        Work(const Work& w) noexcept
        {
            begin_x = w.begin_x;
            begin_y = w.begin_y;
            end_x = w.end_x;
            end_y = w.end_y;
            image = w.image;
            working.store(w.working.load());
        }
    };

    // 時間の許す限り処理を続ける
    std::vector<Work> work_list;
    int work_list_index = 0;
    int num_loop = 0;

    static constexpr int BlockWidth = 32;
    static constexpr int BlockHeight = 32;

    for (int begin_y = 0; begin_y < Height; begin_y += BlockHeight) {
        for (int begin_x = 0; begin_x < Width; begin_x += BlockWidth) {
            Work w;

            w.begin_x = begin_x;
            w.begin_y = begin_y;
            w.end_x = hmath::clamp(begin_x + BlockWidth, 0, Width);
            w.end_y = hmath::clamp(begin_y + BlockHeight, 0, Height);
            w.image = &image;

            work_list.push_back(w);
        }
    }

    // スレッドに処理を振り分ける
    std::vector<std::thread> thread_list;
    std::mutex work_list_mutex;

    for (int thread_id = 0; thread_id < NUM_THREAD; ++thread_id) {
        std::thread thread([Width, Height, &end_flag, thread_id, &work_list, &work_list_index, &num_loop, &work_list_mutex]() {
            hetc::set_thread_group(thread_id);

            hmath::Rng rng;
            rng.set_seed(thread_id);

            for (;;) {
                Work* current_work = nullptr;
                {
                    std::lock_guard<std::mutex> lock(work_list_mutex);

                    if (!work_list[work_list_index].working.load()) {
                        current_work = &work_list[work_list_index];
                        work_list[work_list_index].working.store(true);
                    }
                    work_list_index++;
                    if (work_list_index == work_list.size()) {
                        work_list_index = 0;
                        ++num_loop;
                    }
                }

                if (current_work == nullptr) {
                    continue;
                }

                // タスク処理
                for (int iy = current_work->begin_y; iy < current_work->end_y; ++iy) {
                    for (int ix = current_work->begin_x; ix < current_work->end_x; ++ix) {

                        auto& p = (*current_work->image)(ix, iy);
                        auto& vp = (*current_work->image).data2(ix, iy);

                        for (int batch = 0; batch < 1; ++batch) {
                            if (end_flag)
                                return;

                            const int current_seed = (ix + iy * 8192) * 8192 + num_loop;
                            const auto ret = integrator::get_radiance(Width, Height, ix, iy, current_seed);

                            if (is_valid(ret)) {
                                const auto dret = hmath::Double3(ret[0], ret[1], ret[2]);
                                p += dret;

                                // 分散計算用
                                const auto lumi = dot(dret, rgb2y);
                                vp[0] += lumi * lumi;

                                current_work->image->samples(ix, iy) += 1;
                            }
                        }
                    }
                }

                // 完了
                current_work->working.store(false);
            }
        });

        thread_list.push_back(std::move(thread));
    }

    for (auto& t : thread_list) {
        t.join();
    }

#endif
    watcher.join();

    return 0;
}
