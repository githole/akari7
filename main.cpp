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

// #define TEST

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
        // TODO: 外部からなんとかできるようにしたいところではある
        material_table.clear();
        material_table.push_back({ hmath::Float3(0.18f, 0.18f, 0.18f), hmath::Float3(0, 0, 0) });
        material_table.push_back({ hmath::Float3(0.0f, 0.0f, 0.0f), hmath::Float3(160, 80, 20) });
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
    const hmath::Double3 rgb2y(0.2126, 0.7152, 0.0722);
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

    fmath::PowGenerator degamma(1.2f /* ちょいコントラスト強める */ / 2.2f);

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

    // hhdr::save("hoge.hdr", fdata.data(), (int)Width, (int)Height, false);
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

    // ここに素晴らしいintegratorを書く
    Float3 get_radiance(int w, int h, int x, int y, uint64_t seed)
    {
        Rng rng;
        rng.set_seed(seed);

        const float current_time = rng.next01();

        // カメラデータを直接突っ込む
#if 0
        Float3 camera_position(15, 9, 10);
        Float3 camera_dir = normalize(Float3(0, 0, 0) - camera_position);
        const float fovy = (90.0f) / 180.0f * hmath::pi<float>();
#endif
        Float3 camera_position(34, 15, 32);
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

        const auto initial_pos = camera_position + camera_dir * length + new_U * screen_side + new_V * screen_up;
        const auto initial_dir = normalize(initial_pos - camera_position);


        // naive pathtracing
#if 1
        integrator::Ray ray;
        ray.org = initial_pos;
        ray.dir = initial_dir;
        Float3 throughput(1, 1, 1);
        Float3 L(0, 0, 0);

        bool into = true;

        for (int depth = 0; depth < 5; ++depth)
        {
            auto info = intersect(ray);
            if (!info.hit)
            {
                // 背景
                L += product(throughput, Float3(0.01, 0.01, 0.01));
                break;
            }
            ray.org = info.pos;
            ray.id = info.id;

            auto& triangle = mesh::oldlib::triangles[info.id];
            auto& material = *triangle.mat;

            if (material.ior == 0)
            {
                // 次の方向サンプリング
                const auto ts = hmath::tangentSpace(info.normal);
                const Float3 tangent = std::get<0>(ts);
                const Float3 binormal = std::get<1>(ts);
                auto next_dir = cosine_weighted(rng, info.normal, tangent, binormal);

                ray.dir = next_dir;
                L += product(throughput, material.emission);
                throughput = product(throughput, material.diffuse);
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
    const int NUM_THREAD = 72;

    const int Width = 1920;
    const int Height = 1080;

    FloatImage image(Width, Height);
    bool end_flag = false;

    // メッシュ読み込み
    printf("Load nad Build.\n");
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

    watcher.join();

    return 0;
}
