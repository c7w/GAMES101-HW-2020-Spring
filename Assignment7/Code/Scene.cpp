//  
//   Created by Göksu Güvendiren on 2019-05-14.
//  

#include "Scene.hpp"

Vector3f shade(const Scene *scene, Intersection p, Vector3f dir);

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

//   Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    // TO DO Implement Path Tracing Algorithm here

    auto first_intersect = intersect(ray);
    if(first_intersect.happened) {
        return shade(this, first_intersect, -ray.direction);
    } else { return Vector3f(0.0); }

}

Vector3f shade(const Scene* scene, Intersection p, Vector3f dir) 
{

    if (p.m->hasEmission()) {

        return p.m->getEmission();
    
    } else {
    
        // Direct light contributions
        Vector3f L_dir(0.0);
        Intersection inter;
        float dir_pdf;
        scene->sampleLight(inter, dir_pdf);

        auto orig = p.coords;
        Ray r(orig, normalize(inter.coords - orig));
        Intersection test_inter = scene->intersect(r);

        if (fabs((test_inter.coords - inter.coords).norm()) < 0.001 ) {
            // L_dir = emit * eval(wo, ws, N) * dot(ws, N) * dot(ws,NN) / |x-p|^2 / pdf_light
            L_dir = inter.emit * p.m->eval(r.direction, dir, p.normal) *
                    (dotProduct(r.direction, p.normal)) *
                    (dotProduct(-r.direction, inter.normal)) /
                    (inter.coords - p.coords).norm() /
                    (inter.coords - p.coords).norm() / dir_pdf;
        }

        // Indirect light contributions
        Vector3f L_indir(0.0);

        if (get_random_float() < scene->RussianRoulette) {
            /*
                10 wi = sample(wo, N)
                11 Trace a ray r(p, wi)
                12 If ray r hit a non -emitting object at q
                13 L_indir = shade(q, wi) * eval(wo, wi, N) * dot(wi, N) / pdf(wo, wi, N) / RussianRoulette
            */
            Vector3f wi = normalize(p.m->sample(dir, p.normal));

            Ray r(p.coords, wi);
            Intersection test_inter = scene->intersect(r);
            if(test_inter.happened && test_inter.obj->hasEmit() == false) {
                auto q = test_inter.coords;
                L_indir = shade(scene, test_inter, -wi) *
                        p.m->eval(wi, dir, p.normal) * 
                        (dotProduct(wi, p.normal)) /
                        p.m->pdf(wi, dir, p.normal) / scene->RussianRoulette;
            }
        }
        return L_dir + L_indir;
    }

    


}