#define STB_IMAGE_WRITE_IMPLEMENTATION
#define STBI_MSC_SECURE_CRT
#include "external/stb/stb_image_write.h"
#include "external/fmath/fmath.hpp"

#include "vec3.h"
#include "util.h"
#include "etc.h"
#include "hdr.h"
#include "test.h"
#include "XYZ.h"
#include "bbox.h"

#include <iostream>
#include <thread>
#include <mutex>
#include <vector>
#include <atomic>

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
        Float3 normal;
        int id = -1;
        float t = std::numeric_limits<float>::infinity();
    };

    IntersectionInfo intersect(const Ray &r)
    {
        IntersectionInfo info;

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
        Float3 camera_position(15, 9, 10);
        Float3 camera_dir = normalize(Float3(3, 0, 1) - camera_position);
        const float fovy = (120.0f) / 180.0f * hmath::pi<float>();

        const float ang = 0.15f;
        Float3 camera_up(sin(ang), cos(ang), 0);

        const auto aspect = (float)w / h;
        const float length = 0.1f;
        const float screen_height = length * tan(fovy / 2);
        const float screen_width = screen_height * aspect;
        const auto screen_side = normalize(cross(camera_dir, camera_up)) * screen_width;
        const auto screen_up = normalize(cross(camera_dir, screen_side)) * screen_height;

        const float U = ((float)(x + rng.next01()) / w) * 2 - 1;
        const float V = ((float)(y + rng.next01()) / h) * 2 - 1;
        const float r = (U * U + V * V);
#if 0
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

        const auto initial_pos = camera_position + camera_dir * length + new_U * screen_side + new_V * screen_up;
        const auto initial_dir = normalize(initial_pos - camera_position);

        return {};
    }
}

int main(int argc, char** argv)
{
    const int NUM_THREAD = 72;

    const int Width = 1920;
    const int Height = 1080;

    FloatImage image(Width, Height);
    bool end_flag = false;

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
            if (std::chrono::duration_cast<std::chrono::milliseconds>(current - tick_start).count() >= 1 * 1000) {
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

                            const int current_seed = ix + iy * 8192 + num_loop;
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
