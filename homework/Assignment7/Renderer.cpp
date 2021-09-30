//
// Created by goksu on 2/25/20.
//

#include <fstream>
#include "Scene.hpp"
#include "Renderer.hpp"

#include <thread>
#include <mutex>


inline float deg2rad(const float& deg) { return deg * M_PI / 180.0; }

const float EPSILON = 0.0001;

// 多线程需要的全局变量
std::vector<Vector3f> framebuffer;
std::mutex mut;

void task(const Scene& scene, const Vector3f& eye_pos, const Vector3f& dir, const int& m, const int& spp)
{
    Vector3f color = scene.castRay(Ray(eye_pos, dir), 0);
    mut.lock();
    framebuffer[m] += color / spp;
    mut.unlock();
}

// The main render function. This where we iterate over all pixels in the image,
// generate primary rays and cast these rays into the scene. The content of the
// framebuffer is saved to a file.
void Renderer::Render(const Scene& scene)
{
    //std::vector<Vector3f> framebuffer(scene.width * scene.height);
    framebuffer.resize(scene.width * scene.height);

    float scale = tan(deg2rad(scene.fov * 0.5));
    float imageAspectRatio = scene.width / (float)scene.height;
    Vector3f eye_pos(278, 273, -800);
    int m = 0;

    // change the spp value to change sample ammount
    //int spp = 16;
    int spp = 8;
    std::cout << "SPP: " << spp << "\n";
    for (uint32_t j = 0; j < scene.height; ++j) {
        for (uint32_t i = 0; i < scene.width; ++i) {
            // generate primary ray direction
            float x = (2 * (i + 0.5) / (float)scene.width - 1) *
                      imageAspectRatio * scale;
            float y = (1 - 2 * (j + 0.5) / (float)scene.height) * scale;

            Vector3f dir = normalize(Vector3f(-x, y, 1));
            std::vector<std::thread> thread_vec;    // 创建一个用于存线程的vector，然后在一个循环中join掉
            for (int k = 0; k < spp; k++){
                //framebuffer[m] += scene.castRay(Ray(eye_pos, dir), 0) / spp;  
                thread_vec.emplace_back(std::thread(task, std::ref(scene), eye_pos, dir, m, spp));  // 传递scene的引用使用标准库的ref
            }
            for (std::thread& t : thread_vec)
                t.join();
            // for (int k = 0; k < spp; k++){
            //     framebuffer[m] += scene.castRay(Ray(eye_pos, dir), 0) / spp;  
            // }
            m++;
        }
        UpdateProgress(j / (float)scene.height);
    }
    UpdateProgress(1.f);

    // save framebuffer to file
    FILE* fp = fopen("binary.ppm", "wb");
    (void)fprintf(fp, "P6\n%d %d\n255\n", scene.width, scene.height);
    for (auto i = 0; i < scene.height * scene.width; ++i) {
        static unsigned char color[3];
        color[0] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].x), 0.6f));
        color[1] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].y), 0.6f));
        color[2] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].z), 0.6f));
        fwrite(color, 1, 3, fp);
    }
    fclose(fp);    
}
