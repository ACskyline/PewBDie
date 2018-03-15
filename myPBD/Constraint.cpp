//
// Created by sean on 3/2/18.
//

#include "Constraint.h"

#ifndef EPSILON
#define EPSILON 0.000000001f
#endif

Constraint::Constraint(Points *_points, float _stiffness) :
        points(_points),
        stiffness(_stiffness)
{

}

Constraint::~Constraint()
{
    points = NULL;
}

bool Constraint::project()
{
    return true;
}

FixedPointConstraint::FixedPointConstraint(Points *_points, unsigned int _p, const glm::vec3& _q) :
        Constraint(_points, 1.0f),
        p(_p),
        q(_q)
{

}

FixedPointConstraint::FixedPointConstraint(const FixedPointConstraint& constraint) :
        Constraint(constraint),
        p(constraint.p),
        q(constraint.q)
{

}

bool FixedPointConstraint::project()
{
    points->posLock[p];
    glm::vec3 predictPos = points->posPredict[p];

    if(glm::length(predictPos - q) < EPSILON)
        return true;

    glm::vec3 dp = q - predictPos;
    points->posPredict[p] += dp * stiffness;

    //if(std::isnan(points->posPredict[p0].x) || std::isnan(points->posPredict[p0].y) || std::isnan(points->posPredict[p0].z))
        //printf("fixed:nan>>%d\n",p0);

    return false;
}

StretchConstraint::StretchConstraint(Points *_points, float _stiffness, unsigned int _p1, unsigned int _p2, float _l) :
        Constraint(_points, _stiffness),
        p1(_p1),
        p2(_p2),
        l(_l)
{

}

bool StretchConstraint::project()
{
    glm::vec3 p1PosP = points->posPredict[p1];
    glm::vec3 p2PosP = points->posPredict[p2];

    float length = glm::length(p1PosP - p2PosP);
    if(glm::abs(length - l) < EPSILON)
        return true;

    float w1 = points->massInv[p1];
    float w2 = points->massInv[p2];
    if(length==0) length = 1;
    glm::vec3 dp1 = -1 * w1 / (w1 + w2) * (length - l) * (p1PosP - p2PosP) / length;
    glm::vec3 dp2 = w2 / (w1 + w2) * (length - l) * (p1PosP - p2PosP) / length;
    points->posPredict[p1] += dp1 * stiffness;
    points->posPredict[p2] += dp2 * stiffness;

    //if(std::isnan(points->posPredict[p1].x) || std::isnan(points->posPredict[p1].y) || std::isnan(points->posPredict[p1].z) ||
       //std::isnan(points->posPredict[p2].x) || std::isnan(points->posPredict[p2].y) || std::isnan(points->posPredict[p2].z))
        //printf("stretch:nan>>%d,%d\n",p1,p2);


    return false;
}

BendConstraint::BendConstraint(Points *verts, float stiff, unsigned int p1, unsigned int p2, unsigned int p3, unsigned int p4, float phi) :
        Constraint(verts, stiff), p1(p1), p2(p2), p3(p3), p4(p4), phi(phi)
{

}

bool BendConstraint::project()
{
#ifdef MY_NEWBEND
    return projectReal();
#endif

    //return true if current position is OK. return false if the position is being projected.
    glm::vec3 p1PosP = points->posPredict[p1],
            p2PosP = points->posPredict[p2],
            p3PosP = points->posPredict[p3],
            p4PosP = points->posPredict[p4];

    float w1 = points->massInv[p1];
    float w2 = points->massInv[p2];
    float w3 = points->massInv[p3];
    float w4 = points->massInv[p4];

    glm::vec3 newP1(0, 0, 0);
    glm::vec3 newP2 = p2PosP - p1PosP;
    glm::vec3 newP3 = p3PosP - p1PosP;
    glm::vec3 newP4 = p4PosP - p1PosP;

    float lx23 = glm::length(glm::cross(newP2, newP3));
    if(lx23==0) lx23 = 1;

    float lx24 = glm::length(glm::cross(newP2, newP4));
    if(lx24==0) lx24 = 1;

    glm::vec3 n1 = glm::cross(newP2, newP3) / lx23;
    glm::vec3 n2 = glm::cross(newP2, newP4) / lx24;
    float d = glm::clamp(glm::dot(n1, n2), -1.f, 1.f);

    if(glm::abs(acos(d) - phi) <EPSILON)
        return true;

    glm::vec3 q3 = (glm::cross(newP2, n2) + glm::cross(n1, newP2) * d) / lx23;
    glm::vec3 q4 = (glm::cross(newP2, n1) + glm::cross(n2, newP2) * d) / lx24;
    glm::vec3 q2 = -(glm::cross(newP3, n2) + glm::cross(n1, newP3) * d) / lx23
              -(glm::cross(newP4, n1) + glm::cross(n2, newP4) * d) / lx24;
    glm::vec3 q1 = -q2 - q3 - q4;

    float denominator = w1 * glm::dot(q1,q1) + w2 * glm::dot(q2,q2) + w3 * glm::dot(q3,q3) + w4 * glm::dot(q4,q4);
    if(denominator==0) denominator = 1;

    glm::vec3 dp1 = -1.f * w1 * glm::sqrt(1-d*d)*(glm::acos(d) - phi) * q1 / denominator;
    glm::vec3 dp2 = -1.f * w2 * glm::sqrt(1-d*d)*(glm::acos(d) - phi) * q2 / denominator;
    glm::vec3 dp3 = -1.f * w3 * glm::sqrt(1-d*d)*(glm::acos(d) - phi) * q3 / denominator;
    glm::vec3 dp4 = -1.f * w4 * glm::sqrt(1-d*d)*(glm::acos(d) - phi) * q4 / denominator;

    points->posPredict[p1] += dp1 * stiffness;
    points->posPredict[p2] += dp2 * stiffness;
    points->posPredict[p3] += dp3 * stiffness;
    points->posPredict[p4] += dp4 * stiffness;

    //if(std::isnan(points->posPredict[p1].x) || std::isnan(points->posPredict[p1].y) || std::isnan(points->posPredict[p1].z) ||
       //std::isnan(points->posPredict[p2].x) || std::isnan(points->posPredict[p2].y) || std::isnan(points->posPredict[p2].z) ||
       //std::isnan(points->posPredict[p3].x) || std::isnan(points->posPredict[p3].y) || std::isnan(points->posPredict[p3].z) ||
       //std::isnan(points->posPredict[p4].x) || std::isnan(points->posPredict[p4].y) || std::isnan(points->posPredict[p4].z))
        //printf("bend:nan>>%d,%d,%d,%d,%f\n",p1,p2,p3,p4,denominator);

    return false;
}

bool BendConstraint::projectReal() {
    glm::vec3 p1PosP = points->posPredict[p1],
            p2PosP = points->posPredict[p2],
            p3PosP = points->posPredict[p3],
            p4PosP = points->posPredict[p4];

    float w1 = points->massInv[p1];
    float w2 = points->massInv[p2];
    float w3 = points->massInv[p3];
    float w4 = points->massInv[p4];

    glm::vec3 n1 = glm::cross(p1PosP - p3PosP, p2PosP - p3PosP);
    glm::vec3 n2 = glm::cross(p2PosP - p4PosP, p1PosP - p4PosP);
    float d = glm::clamp(glm::dot(glm::normalize(n1), glm::normalize(n2)), -1.f, 1.f);

    if(glm::abs(acos(d) - phi) <EPSILON)
        return true;

    float n1Length = glm::length(n1);
    float n2Length = glm::length(n2);
    if(n1Length==0) n1Length = 1;
    if(n2Length==0) n2Length = 1;
    n1 = n1 / (n1Length * n1Length);
    n2 = n2 / (n2Length * n2Length);
    glm::vec3 e = p2PosP - p1PosP;
    float eLength = glm::length(e);
    e = glm::normalize(e);

    glm::vec3 q1 = glm::dot(p3PosP - p2PosP, e) * n1 + glm::dot(p4PosP - p2PosP, e) * n2;
    glm::vec3 q2 = glm::dot(p1PosP - p3PosP, e) * n1 + glm::dot(p1PosP - p4PosP, e) * n2;
    glm::vec3 q3 = eLength * n1;
    glm::vec3 q4 = eLength * n2;

    if(glm::dot(glm::cross(n1,n2),e) > 0)
    {
        q1 = -q1;
        q2 = -q2;
        q3 = -q3;
        q4 = -q4;
    }

    float denominator = w1 * glm::dot(q1,q1) + w2 * glm::dot(q2,q2) + w3 * glm::dot(q3,q3) + w4 * glm::dot(q4,q4);
    if(denominator==0) denominator = 1;

    glm::vec3 dp1 = -1.f * w1 * q1 * (glm::acos(d) - phi) / denominator;
    glm::vec3 dp2 = -1.f * w2 * q2 * (glm::acos(d) - phi) / denominator;
    glm::vec3 dp3 = -1.f * w3 * q3 * (glm::acos(d) - phi) / denominator;
    glm::vec3 dp4 = -1.f * w4 * q4 * (glm::acos(d) - phi) / denominator;

    points->posPredict[p1] += dp1 * stiffness;
    points->posPredict[p2] += dp2 * stiffness;
    points->posPredict[p3] += dp3 * stiffness;
    points->posPredict[p4] += dp4 * stiffness;

    return false;
}

CollisionConstraint::CollisionConstraint(Points *_points, unsigned int _p, const glm::vec3& _q, const glm::vec3& _n) :
        Constraint(_points, 1.0f), p(_p), q(_q), n(_n)
{

}

bool CollisionConstraint::project()
{
    glm::vec3 pPosP = points->posPredict[p];
    if(glm::dot(pPosP - q, n) > 0)
        return true;

    glm::vec3 dp = q - pPosP;
    points->posPredict[p] += dp * stiffness;

    return false;
}