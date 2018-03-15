//
// Created by sean on 3/2/18.
//

#ifndef MYPBD_CONSTRAINT_H
#define MYPBD_CONSTRAINT_H

#include "Points.h"

#ifndef MY_NEWBEND
#define MY_NEWBEND
#endif

class Constraint
{
public:
    Constraint(Points *_points, float _stiffness);
    virtual ~Constraint();
    virtual bool project();
    Points *points;
    float stiffness;
};

class FixedPointConstraint : public Constraint
{
public:
    FixedPointConstraint(Points *_points, unsigned int _p, const glm::vec3& q);
    FixedPointConstraint(const FixedPointConstraint& constraint);
    virtual bool project();
    unsigned int p;
    glm::vec3 q;
};

class StretchConstraint : public Constraint
{
public:
    StretchConstraint(Points *_points, float _stiffness, unsigned int _p1, unsigned int _p2, float _l);
    virtual bool project();
    unsigned int p1;
    unsigned int p2;
    float l;
};

class BendConstraint : public Constraint
{
public:
    BendConstraint(Points *_points, float stiffness, unsigned int _p1, unsigned int _p2, unsigned int _p3, unsigned int _p4, float _phi);
    virtual bool project();
    bool projectReal();
    unsigned int p1;
    unsigned int p2;
    unsigned int p3;
    unsigned int p4;
    float phi;
};

class CollisionConstraint : public Constraint
{
public:
    CollisionConstraint(Points *_points, unsigned int _p, const glm::vec3& _q, const glm::vec3& _n);
    virtual bool project();
    unsigned int p;
    glm::vec3 q;
    glm::vec3 n;
};

#endif //MYPBD_CONSTRAINT_H
