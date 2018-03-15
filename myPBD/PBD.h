//
// Created by sean on 3/2/18.
//

#ifndef MYPBD_PBD_H
#define MYPBD_PBD_H

#include "Points.h"
#include "Constraint.h"
#include "Scene.h"
#include <sstream>

class PBD {
public:
    struct Edge{
        int v1, v2;
        int tri1, tri2;
    };

    int iteration;
    float thickness;

    float stretchStiffness;
    float bendStiffness;

    Points points;
    std::vector<Constraint*> constraints;
    std::vector<CollisionConstraint> constraintsCollision;
    std::vector<int> triangles;
    std::vector<Edge> edges;

    PBD(int iteration, float thickness, float stretchStiffness = 0.5f, float bendStiffness = 0.1f);
    ~PBD();
    void initialize(int _dimX, int _dimZ, glm::vec3 clothMin, glm::vec3 clothMax);
    int initializeFromObj(std::string name, glm::vec3 T, glm::vec3 R, glm::vec3 S);
    void update(Scene* s, float dt, int frame);//frame is for debug
    void exportFile(std::string name, int frame);
    void printAll();
    void printAllPredict();
    void printDebug();
private:
    void initializeEdges();
    void initializeConstraints();
    void gravity(glm::vec3 force, float dt);
    void dampVelocity(float kDamp);
    void computePredictedPostion(float dt);
    void collisionDetection(Scene* s);
    void resolveConstraints(int frame);//frame for debug
    void integrate(float dt);
    void updateVelocity(float friction, float restitution);
    void clearCollisionConstraints();
    int loadObj(std::string name, glm::vec3 T, glm::vec3 R, glm::vec3 S);
    void parseObjFace(std::stringstream& ss, std::vector<int>& index);
};


#endif //MYPBD_PBD_H
