//
// Created by sean on 3/2/18.
//

#ifndef MYPBD_SCENE_H
#define MYPBD_SCENE_H

#include "glm/glm.hpp"
#include <vector>
#include <string>

class Primitive
{
public:
    virtual bool intersect(const glm::vec3& p1, const glm::vec3& p2, float threshold, glm::vec3& q, glm::vec3& normal) const = 0;
    virtual void exportFile(std::string name, int frame) = 0;
};

class Plane : public Primitive
{
public:
    Plane(glm::vec3 normal, float distance);
    virtual bool intersect(const glm::vec3& p1, const glm::vec3& p2, float threshold, glm::vec3& q, glm::vec3& normal) const;
    virtual void exportFile(std::string name, int frame);
    glm::vec3 n;
    float dis;//distance between origin and the plane
};

class Sphere : public Primitive
{
public:
    Sphere(const glm::vec3 _center, float _radius);
    std::vector<glm::vec3> vertices;
    glm::vec3 centerFake;
    float radiusScale;
    virtual bool intersect(const glm::vec3& p1, const glm::vec3& p2, float threshold, glm::vec3& q, glm::vec3& normal) const;
    int importFile(std::string name);
    virtual void exportFile(std::string name, int frame);
    glm::vec3 center;
    float radius;
};

class Scene
{
public:
    Scene();
    virtual ~Scene();
    bool intersect(const glm::vec3& p1, const glm::vec3& p2, float threshold, glm::vec3& q, glm::vec3& normal) const;
    std::vector<Primitive*> primitives;
};


#endif //MYPBD_SCENE_H
