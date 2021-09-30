//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"


void Scene::buildBVH() {
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray &ray) const
{
    return this->bvh->Intersect(ray);
}

void Scene::sampleLight(Intersection &pos, float &pdf) const
{
    float emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
        }
    }
    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum){
                objects[k]->Sample(pos, pdf);
                break;
            }
        }
    }
}

bool Scene::trace(
        const Ray &ray,
        const std::vector<Object*> &objects,
        float &tNear, uint32_t &index, Object **hitObject)
{
    *hitObject = nullptr;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        float tNearK = kInfinity;
        uint32_t indexK;
        Vector2f uvK;
        if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear) {
            *hitObject = objects[k];
            tNear = tNearK;
            index = indexK;
        }
    }


    return (*hitObject != nullptr);
}

// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    // TO DO Implement Path Tracing Algorithm here
    Intersection inter = intersect(ray);
    Vector3f hitColor = Vector3f(0);

    if (!inter.happened) 
        return hitColor;
    if (inter.m->hasEmission())
        return Vector3f(1);

    float pdf_light = 0.0;
    Intersection inter_light;
    sampleLight(inter_light, pdf_light);

    Vector3f p = inter.coords;
    Vector3f wo = normalize(-ray.direction);  
    Vector3f N = normalize(inter.normal);
    
    Vector3f x = inter_light.coords;
    Vector3f ws = normalize(x - p);
    Vector3f NN = normalize(inter_light.normal);

    Vector3f L_dir = Vector3f(0);   // 直射光
    if ((intersect(Ray(p, ws)).coords - x).norm() < 0.001 && pdf_light > 0.001)
        L_dir = inter_light.emit * inter.m->eval(wo, ws, N) * dotProduct(ws, N) * dotProduct(-ws, NN) / ((x - p).norm() * (x - p).norm()) / pdf_light;
   
    Vector3f L_indir = Vector3f(0); // 非直射光
    float RR = get_random_float();
    if (RR < RussianRoulette)
    {
        auto wi = inter.m->sample(wo, N);
        auto pdf = inter.m->pdf(wi, wo, N);
        auto f_r = inter.m->eval(wi, wo, N);
        L_indir = castRay(Ray(p, wi), depth) * f_r * dotProduct(wi, N) / pdf / RussianRoulette;
    }
    hitColor = L_dir + L_indir;

    return hitColor;
}