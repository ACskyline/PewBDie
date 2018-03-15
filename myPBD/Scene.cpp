//
// Created by sean on 3/2/18.
//

#include "Scene.h"
#include <fstream>
#include <sstream>
#include <iostream>
#include <Partio.h>

//----------Scene Class----------//
Scene::Scene()
{

}

Scene::~Scene()
{
    for(std::vector<Primitive*>::iterator iter = primitives.begin(); iter != primitives.end(); ++iter)
    {
        delete (*iter);
    }
    primitives.clear();
}

bool Scene::intersect(const glm::vec3& p1, const glm::vec3& p2, float threshold, glm::vec3& q, glm::vec3& normal) const
{
    for(std::vector<Primitive*>::const_iterator iter = primitives.begin(); iter != primitives.end(); ++iter)
    {
        if((*iter)->intersect(p1, p2, threshold, q, normal))
            return true;
    }
    return false;
}

Plane::Plane(glm::vec3 normal, float distance) : n(normal), dis(distance)
{

}

bool Plane::intersect(const glm::vec3& p1, const glm::vec3& p2, float threshold, glm::vec3& intersect, glm::vec3& normal) const
{
    float v1, v2;
    v1 = glm::dot(p1, n) - dis;
    v2 = glm::dot(p2, n) - dis;
    if(v2 < threshold)
    {
        normal = n;
        if(v1 >= threshold)
        {
            intersect = ((v1 - threshold) * p2 - (v2 - threshold) * p1) / (v1 - v2);
        }
        else
        {
            intersect = p2 - (v2 - threshold) * normal;
        }
        return true;
    }
    else
        return false;
}

void Plane::exportFile(std::string name, int frame) {
    //TO DO
}

Sphere::Sphere(const glm::vec3 _center, float _radius) : center(_center), radius(_radius)
{
    centerFake = glm::vec3(0.f);
    radiusScale = 1.f;
}

bool Sphere::intersect(const glm::vec3& p1, const glm::vec3& p2, float threshold, glm::vec3& intersect, glm::vec3& normal) const
{
    float v1 = glm::length(p1 - center) - radius;
    float v2 = glm::length(p2 - center) - radius;

    if(v2 < threshold)
    {
        if(v1 >= threshold)
        {
            glm::vec3 newV0 = p2 - p1;
            glm::vec3 newP0 = glm::vec3(p1[0], p1[1], p1[2]);

            newV0 = glm::normalize(newV0);

            float temp = glm::dot(newV0, center - newP0);
            float t = temp - sqrt(temp * temp - glm::dot(center - newP0, center - newP0) + (radius + threshold) * (radius + threshold));

            intersect = newP0 + t * newV0;
            normal = glm::normalize(intersect - center);
        }
        else
        {
            glm::vec3 newV0 = p2 - center;
            float length = glm::length(newV0);
            float t = (radius + threshold) / length;

            intersect = center + newV0 * t;
            normal = glm::normalize(intersect - center);
        }
        return true;
    }
    else
        return false;
}

int Sphere::importFile(std::string name) {
    float diameter = -1;
    glm::vec3 firstV(0.f);
    glm::vec3 secondV(0.f);
    bool gotFirstV = false;

    std::fstream file;
    file.open(name, std::ios::in);
    if (file.is_open())
    {
        std::string str;
        while (std::getline(file, str))
        {
            if (str.substr(0, 2) == "v ")
            {
                std::stringstream ss;
                ss << str.substr(2);
                glm::vec3 v;
                ss >> v.x;
                ss >> v.y;
                ss >> v.z;
                //extra!!
                if(!gotFirstV) {
                    firstV = v;
                    secondV = v;//for clarity
                    gotFirstV = true;
                }
                else {
                    float distance = glm::length(firstV - v);
                    if(distance > diameter)
                    {
                        diameter = distance;
                        secondV = v;
                    }
                }
                //extra!!
                vertices.push_back(v);
            }
            else if (str.substr(0, 3) == "vt ")
            {
                //nothing
            }
            else if (str.substr(0, 3) == "vn ")
            {
                //nothing
            }
            else if (str.substr(0, 2) == "f ")
            {
                //nothing
            }
            else if (str[0] == '#')
            {
                //comment
            }
            else
            {
                //others
            }
        }
    }
    else
    {
        std::cout << "can not open" << name << std::endl;
        return 1;
    }

    radiusScale = radius / (diameter / 2.f);
    centerFake = (firstV + secondV)/2.f;

    file.close();
    return 0;
}

void Sphere::exportFile(std::string name, int frame) {
    //TO DO
    glm::vec3 centerOffset =  glm::vec3(0.f) - centerFake;
    int n = vertices.size();

    Partio::ParticlesDataMutable *parts = Partio::create();
    Partio::ParticleAttribute posH, vH, mH;
    mH = parts->addAttribute("m", Partio::VECTOR, 1);
    posH = parts->addAttribute("position", Partio::VECTOR, 3);
    vH = parts->addAttribute("v", Partio::VECTOR, 3);

    for (int i = 0; i < n; i++) {
        int idx = parts->addParticle();
        float *m = parts->dataWrite<float>(mH, idx);
        float *p = parts->dataWrite<float>(posH, idx);
        float *v = parts->dataWrite<float>(vH, idx);

        m[0] = 1.0;
        p[0] = (vertices[i].x + centerOffset.x) * radiusScale + center.x;
        p[1] = (vertices[i].y + centerOffset.y) * radiusScale + center.y;
        p[2] = (vertices[i].z + centerOffset.z) * radiusScale + center.z;
        v[0] = 0;
        v[1] = 0;
        v[2] = 0;
    }

    std::stringstream ss;
    ss << name;
    ss << frame;
    ss << ".bgeo";
    Partio::write(ss.str().c_str(), *parts);
    parts->release();
}