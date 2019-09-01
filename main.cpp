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
#include <omp.h>

#define _USE_MATH_DEFINES
#include <math.h>

// #define TEST

constexpr int NUM_THREAD = 72;

class FloatImage
{
private:
    size_t width_, height_;
    std::vector<hmath::Double3> data_;
    std::vector<hmath::Double3> data2_;
    std::vector<uint32_t> sample_map_;
public:
    FloatImage(size_t w, size_t h) : width_(w), height_(h), data_(width_ * height_), data2_(width_ * height_), sample_map_(width_ * height_)
    {
    }

    size_t width() const { return width_; }
    size_t height() const { return height_; }

    hmath::Double3& operator()(int x, int y)
    {
        return data_[x + y * width_];
    }

    hmath::Double3& data2(int x, int y)
    {
        return data2_[x + y * width_];
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
        // TODO: �O������Ȃ�Ƃ��ł���悤�ɂ������Ƃ���ł͂���
        material_table.clear();
        material_table.push_back({ hmath::Float3(0.18f, 0.18f, 0.18f), hmath::Float3(0, 0, 0) });
        material_table.push_back({ hmath::Float3(0.0f, 0.0f, 0.0f), hmath::Float3(160, 80, 20) / 64.0f });
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

    fmath::PowGenerator degamma(1.2f /* ���傢�R���g���X�g���߂� */ / 2.2f);

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
    std::cout << "Average: " << average << " samples/pixel" << std::endl;
    stbi_write_png(filename, (int)Width, (int)Height, 3, uint8_tonemapped_image.data(), (int)(Width * sizeof(uint8_t) * 3));

    std::string hdrfilename = filename;
    hhdr::save((hdrfilename + +".hdr").c_str(), fdata.data(), (int)Width, (int)Height, false);
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
}

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

            auto& triangle = mesh::oldlib::triangles[hp.triangle_index];

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

    static std::uniform_real_distribution<> dist01(0.0, 1.0);

    namespace guiding
    {
        struct Param
        {
            float c;
            float k;
            float rho;
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
            // �S��null�Ȃ�leaf
            // std::unique_ptr<QuadTree> child[4] = {};
            QuadTree* child[4] = {};
            bool isLeaf() const
            {
                return !child[0] && !child[1] && !child[2] && !child[3];
            }

            float flux_tmp[NUM_THREAD] = {};
            float flux_total = 0;

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
                for (int c = 0; c < 4; ++c)
                {
                    if (child[c])
                    {
                        child[c]->deallocate();
                        delete child[c];
                        child[c] = nullptr;
                    }
                }
            }

            QuadTree() = default;

            // copy�֎~
            QuadTree(const QuadTree& tree) = delete;
            QuadTree& operator=(const QuadTree& tree) = delete;

            QuadTree& operator=(QuadTree&& tree)
            {
                for (int c = 0; c < 4; ++c)
                {
                    this->child[c] = tree.child[c];
                    this->flux_total = tree.flux_total;
                    std::copy(tree.flux_tmp, tree.flux_tmp + NUM_THREAD, this->flux_tmp);
                    this->pmin[0] = tree.pmin[0];
                    this->pmax[0] = tree.pmax[0];
                    this->pmin[1] = tree.pmin[1];
                    this->pmax[1] = tree.pmax[1];
                    tree.child[c] = nullptr;
                }
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

        void reset(QuadTree* tree)
        {
            if (tree == nullptr)
                return;

            tree->flux_total = 0;
            std::fill(tree->flux_tmp, tree->flux_tmp + NUM_THREAD, 0);
            for (int c = 0; c < 4; ++c)
            {
                reset(tree->child[c]);
            }
        }

        int calc_depth(float u, float v, QuadTree* tree, int depth = 0)
        {
            if (!tree->isLeaf())
            {
                for (int c = 0; c < 4; ++c)
                {
                    if (tree->child[c]->inside(u, v))
                    {
                        return calc_depth(u, v, tree->child[c], depth + 1);
                    }
                }

            }
            return depth;
        }

        void populate(int thread_id, const Float3& pos, const Float3& dir, float radiance, QuadTree* tree)
        {
            float theta, phi;
            directionToPolarCoordinate(dir, &theta, &phi);
            const float v = theta / M_PI;
            const float u = phi / (2 * M_PI);

            if (tree->inside(u, v))
            {
//                tree->flux += radiance; // TODO: �Ȃ񂩂��������Ȃ����H
                tree->flux_tmp[thread_id] += radiance;
                if (!tree->isLeaf())
                {
                    for (int c = 0; c < 4; ++c)
                    {
                        populate(thread_id, pos, dir, radiance, tree->child[c]);
                    }
                }
            }
        }

        QuadTree copy(const QuadTree* tree)
        {
            QuadTree copy_tree;

            copy_tree.flux_total = tree->flux_total;
            std::copy(tree->flux_tmp, tree->flux_tmp + NUM_THREAD, copy_tree.flux_tmp);

            copy_tree.pmin[0] = tree->pmin[0];
            copy_tree.pmin[1] = tree->pmin[1];
            copy_tree.pmax[0] = tree->pmax[0];
            copy_tree.pmax[1] = tree->pmax[1];

            if (!tree->isLeaf())
            {
                for (int c = 0; c < 4; ++c)
                {
                    copy_tree.child[c] = new QuadTree();
                    *copy_tree.child[c] = copy(tree->child[c]);
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

                    for (int c = 0; c < 4; ++c)
                    {
                        // tree->child[c] = std::unique_ptr<QuadTree>(new QuadTree());
                        tree->child[c] = new QuadTree();
                        tree->child[c]->flux_total = tree->flux_total / 4;
                    }

                    tree->child[0]->pmin[0] = tree->pmin[0];
                    tree->child[0]->pmin[1] = tree->pmin[1];
                    tree->child[0]->pmax[0] = mid[0];
                    tree->child[0]->pmax[1] = mid[1];

                    tree->child[1]->pmin[0] = mid[0];
                    tree->child[1]->pmin[1] = tree->pmin[1];
                    tree->child[1]->pmax[0] = tree->pmax[0];
                    tree->child[1]->pmax[1] = mid[1];

                    tree->child[2]->pmin[0] = tree->pmin[0];
                    tree->child[2]->pmin[1] = mid[1];
                    tree->child[2]->pmax[0] = mid[0];
                    tree->child[2]->pmax[1] = tree->pmax[1];

                    tree->child[3]->pmin[0] = mid[0];
                    tree->child[3]->pmin[1] = mid[1];
                    tree->child[3]->pmax[0] = tree->pmax[0];
                    tree->child[3]->pmax[1] = tree->pmax[1];

                    for (int c = 0; c < 4; ++c)
                    {
                        refine_quad_tree_sub(p, total_flux, tree->child[c]);
                    }
                }
                return;
            }

            // node
#if 1
            if (tree->flux_total / total_flux < p.rho)
            {
                printf("*");
                // �܂Ƃ߂�
                for (int c = 0; c < 4; ++c)
                {
                    tree->child[c] = nullptr;
                }
                return;
            }
#endif

            for (int c = 0; c < 4; ++c)
            {
                refine_quad_tree_sub(p, total_flux, tree->child[c]);
            }
        }

        // ��ԍ\��
        struct BinaryTree
        {
            // ����null�Ȃ�leaf
            // ����node�Ȃ�A����not null
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

            for (int c = 0; c < 4; ++c)
            {
                merge_binary_tree_sub(tree->child[c]);
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

        Rng global_rng;

        void refine_binary_tree(const Param& p, BinaryTree* tree)
        {
            if (!tree->isLeaf())
            {
                // ����node
                refine_binary_tree(p, tree->child[0].get());
                refine_binary_tree(p, tree->child[1].get());
            }
            else
            {
                // leaf
                if (tree->num_sample_total > p.c * sqrt(pow(2.0f, p.k)))
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
                        tree->child[c]->debug_color = Float3(
                            global_rng.next01(), 
                            global_rng.next01(), 
                            global_rng.next01());

                        tree->child[c]->quad_tree = guiding::copy(&tree->quad_tree);
                    }
                    tree->quad_tree = {};

                    refine_binary_tree(p, tree->child[0].get());
                    refine_binary_tree(p, tree->child[1].get());
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
                auto left_result = traverse(tree->child[0].get(), x);
                if (left_result == nullptr)
                {
                    auto right_result = traverse(tree->child[1].get(), x);
                    if (right_result == nullptr)
                    {
                        assert(false);
                    }
                    else
                    {
                        return right_result;
                    }
                }
                else
                {
                    return left_result;
                }
            }
            
            // leaf
            return tree;
        }
    }

    guiding::BinaryTree global_binary_tree;
    guiding::BinaryTree* debug_tree;

    void get_initial_ray(Rng& rng, int w, int h, int x, int y, Float3& initial_pos, Float3& initial_dir)
    {
        // �J�����f�[�^�𒼐ړ˂�����
#if 0
        Float3 camera_position(15, 9, 10);
        Float3 camera_dir = normalize(Float3(0, 0, 0) - camera_position);
        const float fovy = (90.0f) / 180.0f * hmath::pi<float>();
#endif
        hmath::Float3 camera_position(34, 15, 32);
        Float3 camera_dir = normalize(Float3(0, 0, 0) - camera_position);
        const float fovy = (45.0f) / 180.0f * hmath::pi<float>();

        //const float ang = 0.15f;
        //Float3 camera_up(sin(ang), cos(ang), 0);
        Float3 camera_up(0, 1, 0);

        const auto aspect = (float)w / h;
        const float length = 0.1f;
        const float screen_height = length * tan(fovy / 2);
        const float screen_width = screen_height * aspect;
        const auto screen_side = normalize(cross(camera_dir, camera_up)) * screen_width;
        const auto screen_up = normalize(cross(camera_dir, screen_side)) * screen_height;

        const float U = ((float)(x + rng.next01()) / w) * 2 - 1;
        const float V = ((float)(y + rng.next01()) / h) * 2 - 1;
        const float r = (U * U + V * V);

        auto new_U = U;
        auto new_V = V;
#if 0
#if 1
        int ch = rng.next() % 3;
        float table[3] = { -0.05f, 0, 0.05f };
        const float k1 = table[ch];
        const float new_U = U * (1.0f + k1 * r);
        const float new_V = V * (1.0f + k1 * r);
#else
        const float cjitter = rng.next01() * 0.1 - 0.05;
        const float ch = -0.2f + cjitter;

        const float k1 = ch;
        const float new_U = U * (1.0f + k1 * r);
        const float new_V = V * (1.0f + k1 * r);

        const float U2 = U * aspect;
        const float r2 = (U2 * U2 + V * V);
        const float a = 2.0f;
        const float final_weight = a / (a + r2);
#endif
#endif

        initial_pos = camera_position + camera_dir * length + new_U * screen_side + new_V * screen_up;
        initial_dir = normalize(initial_pos - camera_position);
    }

    // �����ɑf���炵��integrator������
    Float3 get_radiance(int thread_id, Rng& rng, Float3& initial_pos, Float3& initial_dir, int sample, int total_sample, int w, int h, int x, int y)
    {

        const float current_time = rng.next01();

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
        Float3 weight[MAX_DEPTH];
        Float3 final_emission;

        int depth;
        for (depth = 0; depth < MAX_DEPTH; ++depth)
        {
            auto org_ray = ray;
            auto info = intersect(ray);
            if (!info.hit)
            {
                // �w�i
                L += product(throughput, Float3(0.01, 0.01, 0.01));
                break;
            }
            ray.org = info.pos;
            ray.id = info.id;

            // guiding
            {
                vertex_list[depth].pos = org_ray.org;
                vertex_list[depth].dir = org_ray.dir;
            }

            auto& triangle = mesh::oldlib::triangles[info.id];
            auto& material = *triangle.mat;

            if (material.ior == 0)
            {
                // ���̕����T���v�����O
                const auto ts = hmath::tangentSpace(info.normal);
                const Float3 tangent = std::get<0>(ts);
                const Float3 binormal = std::get<1>(ts);
                auto next_dir = cosine_weighted(rng, info.normal, tangent, binormal);

                ray.dir = next_dir;
                L += product(throughput, material.emission);
                throughput = product(throughput, material.diffuse);

                // guiding
                if (depth + 1 < MAX_DEPTH)
                {
                    weight[depth + 1] = material.diffuse;
                }

                auto lumi = dot(rgb2y, material.emission);

                // ������hit������I������
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

                // Snell�̖@��
                const float nc = 1.0; // �^��̋��ܗ�
                const float nt = material.ior; // �I�u�W�F�N�g�̋��ܗ�
                const float nnt = into ? nc / nt : nt / nc;
                into = !into;

                const auto orienting_normal = dot(info.normal, ray.dir) < 0.0 ? info.normal : (-1.0f * info.normal);
                const float ddn = dot(ray.dir, orienting_normal);
                const float cos2t = 1.0 - nnt * nnt * (1.0 - ddn * ddn);

                if (cos2t < 0.0) { // �S����
                    ray = reflection_ray;
                    continue;
                }

                // ���܂̕���
                integrator::Ray refraction_ray;
                refraction_ray.org = info.pos;
                refraction_ray.dir = normalize(ray.dir * nnt - info.normal * (into ? 1.0f : -1.0f) * (ddn * nnt + sqrt(cos2t)));

                // Schlick�ɂ��Fresnel�̔��ˌW���̋ߎ����g��
                const double a = nt - nc, b = nt + nc;
                const double R0 = (a * a) / (b * b);

                const float c = 1.0 - (into ? -ddn : dot(refraction_ray.dir, -1.0f * orienting_normal));
                const float Re = R0 + (1.0 - R0) * pow(c, 5.0); // ���˕����̌������˂���ray.dir�̕����ɉ^�Ԋ����B�����ɋ��ܕ����̌������˂�������ɉ^�Ԋ����B
                const float nnt2 = pow(into ? nc / nt : nt / nc, 2.0); // ���C�̉^�ԕ��ˋP�x�͋��ܗ��̈قȂ镨�̊Ԃ��ړ�����Ƃ��A���ܗ��̔�̓��̕������ω�����B
                const float Tr = (1.0 - Re) * nnt2; // ���ܕ����̌������܂���ray.dir�̕����ɉ^�Ԋ���


                const float probability = 0.25f + 0.5f * Re;
                if (rng.next01() < probability) { // ����
                    ray = reflection_ray;
                    throughput = product(throughput, Re * material.diffuse / probability);
                }
                else
                {
                    // ����
                    ray = refraction_ray;
                    throughput = product(throughput, Tr * material.diffuse / (1.0f - probability));
                }
            }

        }

        // guiding����
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

                // i = 0�̓J�������_�Ȃ̂Ŗ���
                for (int i = 1; i < num; ++i)
                {
                    auto* item = guiding::traverse(&global_binary_tree, vertex_list[i].pos);
                    if (item)
                    {
                        // ++item->num_sample;
                        // item->samples.push_back(vertex_list[i]);
                        ++item->num_sample_table[thread_id];
                        guiding::populate(thread_id, vertex_list[i].pos, vertex_list[i].dir, vertex_list[i].radiance / total_sample, &item->quad_tree);

                        //item->quad_tree.flux += vertex_list[i].radiance / total_sample;
                    }
                }
            }
        }

        return L;
#endif
#if 0
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
            for (int i = 0; i < 16; ++i)
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
                    occluded += 1.0 / 16.0f;
                }
            }
        }
        else
        {
            return {};
        }

        return (1 - occluded) * Float3(1, 1, 1);
#endif
    }
}

int main(int argc, char** argv)
{
    integrator::guiding::QuadTree tree;
#if 0
    tree.flux = 100;
    integrator::guiding::Param p;
    p.c = 12000; // TODO: �n�C�p��
    p.rho = 0.1;
    integrator::guiding::refine_quad_tree(p, 100, &tree);


    auto copy_tree = integrator::guiding::copy(&tree);

    return 0;
#endif

    const int Width = 1920 / 4;
    const int Height = 1080 / 4;

    FloatImage image(Width, Height);
    bool end_flag = false;

    // ���b�V���ǂݍ���
    printf("Load and Build.\n");
//    auto mesh = mesh::loadMesh("head.obj");
    auto mesh = mesh::loadMesh("C:/Code/VSProjects/pathological_mesher/pathological_mesher/rtcamp.obj");
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
            hmath::Float3(-50, -10, -50),
            hmath::Float3(50,  50, 50));
    }

    char buf[256];
    int image_index = 0;
    for (int loop = 1; loop <= 16; ++loop)
    {
        const int num_sample = 1 << loop;

        printf("loop: %d\n", loop);
        
#pragma omp parallel for schedule(dynamic)
        for (int iy = 0; iy < Height; ++iy)
        {
            const int tid = omp_get_thread_num();

            for (int ix = 0; ix < Width; ++ix)
            {
                const int current_seed = (ix + iy * 8192) * 8192 + loop;

                for (int s = 0; s < num_sample; ++s)
                {
                    hmath::Rng rng;
                    rng.set_seed(current_seed);

                    hmath::Float3 pos, dir;
                    integrator::get_initial_ray(rng, Width, Height, ix, iy, pos, dir);
                    auto ret = integrator::get_radiance(tid, rng, pos, dir, num_sample, Width, Height, ix, iy, current_seed);

                    if (is_valid(ret)) {
                        const auto dret = hmath::Double3(ret[0], ret[1], ret[2]);
                        //p += dret;

                        // ���U�v�Z�p
                        //const auto lumi = dot(dret, rgb2y);
                        //vp[0] += lumi * lumi;

                        //current_work->image->samples(ix, iy) += 1;

                        image(ix, iy) += dret;
                        image.samples(ix, iy) += 1;
                    }
                }
            }
        }

        // debug
        {
            
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
                                image.samples(ix, iy) = 1;

                                //image(ix, iy) = tree->debug_color;
                                //image(ix, iy) = hmath::Float3(tree->quad_tree.flux / 10 / num_sample, 0, 0);
                                
#if 1
                                auto& aabb = tree->bbox;

                                const auto d = (info.pos - aabb.pmin) / (aabb.pmax - aabb.pmin);
                                const float u = d[0];
                                const float v = d[2];

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
                                    image(ix, iy) = hmath::Float3(0.1, 0.1, 0.1);
                                else
                                    image(ix, iy) = tbl[depth];

#endif
                            }
                        }

                    }
                }
            }
        }

        // refine guiding
        {
            printf("BEGIN refine\n");
            integrator::guiding::Param p;
            p.c = 1200; // TODO: �n�C�p��
            p.k = loop;
            p.rho = 0.1f;

            integrator::guiding::merge_binary_tree(&integrator::global_binary_tree);
            integrator::guiding::refine_quad_tree(p, &integrator::global_binary_tree);
            integrator::guiding::refine_binary_tree(p, &integrator::global_binary_tree);
            integrator::guiding::reset(&integrator::global_binary_tree);
            printf("END refine\n");
        }

        // save
        {
            sprintf(buf, "%03d.png", image_index);
            save_image(buf, image);
            std::cout << "Saved: " << buf << std::endl;
            ++image_index;
        }
    }



#if 0
    // ���ԊĎ��X���b�h�𗧂Ă�
    std::thread watcher([&end_flag, &image]() {
        int image_index = 0;
        char buf[256];

        // �J�n���Ԃ��擾���Ă���
        auto start = std::chrono::system_clock::now();

        auto tick_start = start;
        for (;;) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 0.1�b����

            // 15�b�o�߂��v��
            auto current = std::chrono::system_clock::now();
            if (std::chrono::duration_cast<std::chrono::milliseconds>(current - tick_start).count() >= 3 * 1000) {
                // �摜�o��
                tick_start = current;

                sprintf(buf, "%03d.png", image_index);
                save_image(buf, image);
                std::cout << "Saved: " << buf << std::endl;
                ++image_index;
            }

            // 123�b�o�߂��v��
            if (std::chrono::duration_cast<std::chrono::milliseconds>(current - start).count() >= 122 * 1000 /* ���S���݂Ă�����Ƒ��߂ɂ��� */) {
                // �摜�o�͂��đS���I�����Ⴂ
                end_flag = true;
                save_image("final_image.png", image, true);
                std::cout << "Saved: final_image.png" << std::endl;
                return;
            }
        }
    });

    // work���X�g��K���ɂ����炦��
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

    // ���Ԃ̋������菈���𑱂���
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

    // �X���b�h�ɏ�����U�蕪����
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

                // �^�X�N����
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

                                // ���U�v�Z�p
                                const auto lumi = dot(dret, rgb2y);
                                vp[0] += lumi * lumi;

                                current_work->image->samples(ix, iy) += 1;
                            }
                        }
                    }
                }

                // ����
                current_work->working.store(false);
            }
        });

        thread_list.push_back(std::move(thread));
    }

    for (auto& t : thread_list) {
        t.join();
    }

    watcher.join();
#endif

    return 0;
}
