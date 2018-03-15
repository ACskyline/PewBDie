//
// Created by sean on 3/2/18.
//

#include "PBD.h"
#include <Partio.h>
#include "glm/gtx/transform.hpp"
#include <fstream>

//#define MYDEBUG_UPDATE //print pos after each step in update
//#define MYDEBUG_CONSTRAINT //print pos in resolveConstraints after projecting each type of constraint
//#define MYDEBUG_PREDICT //print posPredict instead of pos
//#define MYDEBUG_LOGTXT //out put massInv, vel, position to several txt files
//#define MYDEBUG_FRAME 67

PBD::PBD(int _iteration, float _thickness, float _stretchStiffness, float _bendStiffness) {
    iteration = _iteration;
    thickness = _thickness;
    stretchStiffness = _stretchStiffness;
    bendStiffness = _bendStiffness;
}

PBD::~PBD(){
    for(int i = 0;i<constraints.size();i++) {
        delete constraints[i];
    }
}

void PBD::initialize(int dimX, int dimZ, glm::vec3 clothMin, glm::vec3 clothMax){
    glm::vec3 delta = clothMax - clothMin;
    delta.x /= (dimX - 1.f);
    //delta.y /= (dimY - 1.f);
    delta.z /= (dimZ - 1.f);
    points.resize(dimX * dimZ);
    for(int i = 0;i<dimX;i++){
        for(int k = 0;k<dimZ;k++){
            int index = dimZ * i + k;
            points.pos[index] = glm::vec3(delta.x * i + clothMin.x, (clothMin.y + clothMax.y)/2.f,delta.z*k+clothMin.z);
            points.vel[index] = glm::vec3(0.f);
            points.massInv[index] = 1.f;
        }
    }

    triangles.resize((dimX-1) * (dimZ-1) * 2 * 3);
    bool rowFlip = false, columnFlip = false;
    for(int i = 0;i<dimX - 1;i++){
        for(int k = 0;k<dimZ - 1;k++)
        {
            int index = (dimZ-1) * i + k;
            triangles[6*index+0] = dimZ*i+k;
            triangles[6*index+1] = dimZ*i+k+1;
            triangles[6*index+2] = dimZ*(i+1)+(rowFlip^columnFlip ? k+1 : k);

            triangles[6*index+3] = dimZ*(i+1)+k+1;
            triangles[6*index+4] = dimZ*(i+1)+k;
            triangles[6*index+5] = dimZ*i+(rowFlip^columnFlip ? k : k+1);

            rowFlip = !rowFlip;
        }
        columnFlip = !columnFlip;
        rowFlip = false;
    }
    initializeEdges();
    initializeConstraints();
}

int PBD::initializeFromObj(std::string name, glm::vec3 T, glm::vec3 R, glm::vec3 S) {
    points.clear();
    triangles.clear();
    edges.clear();
    if(loadObj(name, T, R, S))
        return 1;
    initializeEdges();
    initializeConstraints();
    return 0;
    //std::cout << "edge:" << edges.size() << " triangle:" << triangles.size() << std::endl;
}

void PBD::update(Scene *s, float dt, int frame) {
    gravity(glm::vec3(0.0f, -9.8f, 0.0f), dt);//!
#ifdef MYDEBUG_UPDATE
    if(frame==-1||frame==MYDEBUG_FRAME)
    {
        std::cout << ">>>1.applyExternalForce:<<<" << std::endl;
        printDebug();
    }
#endif
    dampVelocity(0.01f);

    computePredictedPostion(dt);//!
#ifdef MYDEBUG_UPDATE
    if(frame==-1||frame==MYDEBUG_FRAME)
    {
        std::cout << ">>>2.computePredictedPostion:<<<" << std::endl;
        printDebug();
    }
#endif
    collisionDetection(s);//?
#ifdef MYDEBUG_UPDATE
    if(frame==-1||frame==MYDEBUG_FRAME)
    {
        std::cout << ">>>2.5 collisionDetection:<<<" << std::endl;
        printDebug();
    }
#endif
    resolveConstraints(frame);//?
#ifdef MYDEBUG_UPDATE
    if(frame==-1||frame==MYDEBUG_FRAME)
    {
        std::cout << ">>>2.8 resolveConstraints:<<<" << std::endl;
        printDebug();
    }
#endif
    integrate(dt);//!
#ifdef MYDEBUG_UPDATE
    if(frame==-1||frame==MYDEBUG_FRAME)
    {
        std::cout << ">>>3.integrate:<<<" << std::endl;
        printDebug();
    }
#endif
    updateVelocity(0.98f, 0.4f);
    clearCollisionConstraints();
}

void PBD::initializeEdges()
{
    int vert_num = points.size;
    int tri_num = triangles.size() / 3;

    int *first_edge = new int[vert_num + 3 * tri_num];
    int *next_edge = first_edge + vert_num;

    for(int i = 0; i < vert_num; ++i)
        first_edge[i] = -1;

    int edge_count = 0;
    const int* triangle = &triangles[0];
    int i1, i2;
    for(int t = 0; t < tri_num; ++t)
    {
        i1 = triangle[2];
        for(int n = 0; n < 3; ++n)
        {
            i2 = triangle[n];
            if(i1 < i2)
            {
                Edge new_edge;
                new_edge.v1 = i1;
                new_edge.v2 = i2;
                new_edge.tri1 = t;
                new_edge.tri2 = t;
                edges.push_back(new_edge);

                int edge_idx = first_edge[i1];
                if(edge_idx == -1)
                {
                    first_edge[i1] = edge_count;
                }
                else
                {
                    while(true)
                    {
                        int idx = next_edge[edge_idx];
                        if(idx == -1)
                        {
                            next_edge[edge_idx] = edge_count;
                            break;
                        }
                        edge_idx = idx;
                    }
                }

                next_edge[edge_count] = -1;
                edge_count++;
            }
            i1 = i2;
        }
        triangle += 3;
    }

    triangle = &triangles[0];
    for(int t = 0; t < tri_num; ++t)
    {
        i1 = triangle[2];
        for(int n = 0; n < 3; ++n)
        {
            i2 = triangle[n];
            if(i1 > i2)
            {
                bool is_new_edge = true;
                for(int edge_idx = first_edge[i2]; edge_idx != -1; edge_idx = next_edge[edge_idx])
                {
                    Edge *edge = &edges[edge_idx];
                    if((edge->v2 == i1) && (edge->tri1 == edge->tri2))
                    {
                        edge->tri2 = t;
                        is_new_edge = false;
                        break;
                    }
                }
                if(is_new_edge)
                {
                    Edge new_edge;
                    new_edge.v1 = i1;
                    new_edge.v2 = i2;
                    new_edge.tri1 = t;
                    new_edge.tri2 = t;
                    edges.push_back(new_edge);

                    int edge_idx = first_edge[i1];
                    if(edge_idx == -1)
                    {
                        first_edge[i1] = edge_count;
                    }
                    else
                    {
                        while(true)
                        {
                            int idx = next_edge[edge_idx];
                            if(idx == -1)
                            {
                                next_edge[edge_idx] = edge_count;
                                break;
                            }
                            edge_idx = idx;
                        }
                    }

                    next_edge[edge_count] = -1;
                    edge_count++;
                }
            }
            i1 = i2;
        }
        triangle += 3;
    }

    delete[] first_edge;
}

void PBD::initializeConstraints() {
    for(int i = 0; i < points.pos.size(); ++i)
    {
        if(i == 0)//the fixed point
        {
            FixedPointConstraint* fixedConstraint = new FixedPointConstraint(&points, i, points.pos[i]);
            //uncomment the code below
            //constraint.push_back(fixedConstraint);
        }
    }


    // generate stretch constraints. assign a stretch constraint for each edge.
    glm::vec3 p1, p2;
    float s_stiff = 1.0f - std::pow((1 - stretchStiffness), 1.0f / iteration);
    for(std::vector<Edge>::iterator e = edges.begin(); e != edges.end(); ++e)
    {
        int start = e->v1;
        int end = e->v2;

        float restLength = glm::length(points.pos[start] - points.pos[end]);
        StretchConstraint* stretchConstraint = new StretchConstraint(&points, s_stiff, start, end, restLength);
        constraints.push_back(stretchConstraint);

    }

    glm::vec3 bendP1, bendP2, bendP3, bendP4;
    int id1, id2, id3, id4;
    int *tri;

    float b_stiff = 1.0f - std::pow((1 - bendStiffness), 1.0f / iteration);
    for(std::vector<Edge>::iterator e = edges.begin(); e != edges.end(); ++e)
    {
        if(e->tri1 == e->tri2)
            continue;
        id1 = e->v1;
        id2 = e->v2;

        tri = &triangles[3 * e->tri1];
        while((*tri == id1)||(*tri == id2))
            tri++;
        id3 = *tri;

        tri = &triangles[3 * e->tri2];
        while((*tri == id1)||(*tri == id2))
            tri++;
        id4 = *tri;

        bendP1 = points.pos[id1];
        bendP2 = points.pos[id2];
        bendP3 = points.pos[id3];
        bendP4 = points.pos[id4];

        glm::vec3 newP1(0, 0, 0);
        glm::vec3 newP2 = bendP2 - bendP1;
        glm::vec3 newP3 = bendP3 - bendP1;
        glm::vec3 newP4 = bendP4 - bendP1;

        glm::vec3 n1 = glm::normalize(glm::cross(newP2, newP3));
        glm::vec3 n2 = glm::normalize(glm::cross(newP2, newP4));

#ifdef MY_NEWBEND
        float cosTheta = glm::dot(n1, -n2);//float cosTheta = glm::dot(n1, n2);//!!!
#else
        float cosTheta = glm::dot(n1, n2);
#endif
        cosTheta = glm::clamp(cosTheta, -1.f, 1.f);
        float theta = glm::acos(cosTheta);

        BendConstraint* bendConstraint = new BendConstraint(&points, b_stiff, id1, id2, id3, id4, theta);
        constraints.push_back(bendConstraint);
    }
}

void PBD::gravity(glm::vec3 force, float dt) {
    int size = points.size;
    for(unsigned int i = 0; i < size; ++i)
    {
        points.vel[i] += dt * force * points.massInv[i];
    }
}

void PBD::dampVelocity(float k_damp)
{
    float totalMass = 0.0f;
    glm::vec3 massPos;
    glm::vec3 massVel;
    for(int i = 0; i < points.size; ++i)
    {
        glm::vec3 pos = points.pos[i];
        glm::vec3 vel = points.vel[i];
        float mass = 1 / points.massInv[i];
        massPos += mass * pos;
        massVel += mass * vel;
        totalMass += mass;
    }

    glm::vec3 centerMassPos = massPos / totalMass;
    glm::vec3 centerMassVel = massVel / totalMass;

    glm::vec3 L;
    glm::mat3x3 I;
    for(int i = 0; i < points.size; ++i)
    {
        float mass = 1 / points.massInv[i];
        glm::vec3 vel = points.vel[i];
        glm::vec3 r = points.pos[i] - centerMassPos;
        L += glm::cross(r, mass * vel);

        glm::mat3x3 skewMatrixR(glm::vec3(0, r[2], -r[1]), glm::vec3(-r[2], 0, r[0]), glm::vec3(r[1], -r[0], 0));
        I += skewMatrixR * glm::transpose(skewMatrixR) * mass;
    }

    glm::vec3 omega = I._inverse() * L;

    for(int i = 0; i < points.size; ++i)
    {
        glm::vec3 r = points.pos[i] - centerMassPos;
        glm::vec3 vel = points.vel[i];
        glm::vec3 deltaV = centerMassVel + glm::cross(omega, r) - vel;
        points.vel[i] += k_damp * deltaV;
    }

}

void PBD::computePredictedPostion(float dt)
{
    int size = points.size;
    for(int i = 0; i < size; ++i)
    {
        points.posPredict[i] = points.pos[i] + dt * points.vel[i];
    }
}

void PBD::collisionDetection(Scene *s)
{
    int size = points.size;
    glm::vec3 x, p, q, n;
    for(int i = 0; i < size; ++i)
    {
        x = points.pos[i];
        p = points.posPredict[i];
        if(s->intersect(x, p, thickness, q, n))
        {
            CollisionConstraint c(&points, i, q, n);
            constraintsCollision.push_back(c);
        }
    }
}

void PBD::resolveConstraints(int frame)
{
    bool finish = true;
    bool reverse = false;
    int i, size;
    for(int n = 0; n < iteration; ++n)
    {
        size = constraints.size();
        for(i = reverse ? (size - 1) : 0; (i < size) && (i >= 0); reverse ? --i : ++i)
        {
            finish &= constraints[i]->project();
        }
#ifdef MYDEBUG_CONSTRAINT
        if(frame==-1||frame==MYDEBUG_FRAME) {
            std::cout << ">>>A.internal constraints:<<<" << std::endl;
            printDebug();
        }
#endif
        // solve all the external constraints.
        size = constraintsCollision.size();
        for(i = reverse ? (size - 1) : 0; (i < size) && (i >= 0); reverse ? --i : ++i)
        {
            finish &= constraintsCollision[i].project();
        }
#ifdef MYDEBUG_CONSTRAINT
        if(frame==-1||frame==MYDEBUG_FRAME) {
            std::cout << ">>>B.external collision constraints:<<<" << std::endl;
            printDebug();
        }
#endif

        if(finish)
            break;
        reverse = !reverse;
    }
    points.unlockPosAll();
}

void PBD::integrate(float dt)
{
    int size = points.size;
    float inv_dt = 1.0f / dt;
    for(unsigned int i = 0; i < size; ++i)
    {
        points.vel[i] = (points.posPredict[i] - points.pos[i]) * inv_dt;
        points.pos[i] = points.posPredict[i];
    }
}

void PBD::updateVelocity(float friction, float restitution)
{
    glm::vec3 normal, vn, vt;
    float norm_fraction;
    for(std::vector<CollisionConstraint>::iterator s = constraintsCollision.begin(); s != constraintsCollision.end(); ++s)
    {
        int pointIndex = s->p;
        glm::vec3 currentVel = points.vel[pointIndex];
        normal = s->n;
        if(glm::dot(normal, currentVel) < 0)
        {
            vn = glm::dot(normal, currentVel) * normal;
            vt = currentVel - vn;

            glm::vec3 newVn = -1 * restitution * vn;
            glm::vec3 newVt = friction * vt;

            points.vel[pointIndex] = newVn + newVt;
        }
        int a = 0;
    }

}

void PBD::clearCollisionConstraints()
{
    constraintsCollision.clear();
}

void PBD::exportFile(std::string name, int frame) {
    int n = points.size;

#ifdef MYDEBUG_LOGTXT
    std::ofstream ofs;
    std::stringstream oss;
    oss << name;
    oss << frame;
    oss << ".txt";
    ofs.open(oss.str().c_str(), std::ios_base::out | std::ios_base::trunc);
#endif

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

        m[0] = 1.0 / points.massInv[i];
        p[0] = points.pos[i].x;
        p[1] = points.pos[i].y;
        p[2] = points.pos[i].z;
        v[0] = points.vel[i].x;
        v[1] = points.vel[i].y;
        v[2] = points.vel[i].z;

#ifdef MYDEBUG_LOGTXT
        ofs << "massInv:" << m[0]
            << ", posX:" << p[0]
            << ", posY:" << p[1]
            << ", posZ:" << p[2]
            << ", velX:" << v[0]
            << ", velY:" << v[1]
            << ", velZ:" << v[2] << std::endl;
#endif
    }

#ifdef MYDEBUG_LOGTXT
    ofs.close();
#endif

    std::stringstream ss;
    ss << name;
    ss << frame;
    ss << ".bgeo";
    Partio::write(ss.str().c_str(), *parts);
    parts->release();
}

void PBD::printAll() {
    std::cout << "pos::::::::::::" << std::endl;
    for(int i = 0;i<points.size;i++) {
        std::cout << i << ":" << points.pos[i].x << "," << points.pos[i].y << "," << points.pos[i].z << std::endl;
    }
}

void PBD::printAllPredict() {
    std::cout << "posPredict::::::::::::" << std::endl;
    for(int i = 0;i<points.size;i++) {
        std::cout << i << ":" << points.posPredict[i].x << "," << points.posPredict[i].y << "," << points.posPredict[i].z << std::endl;
    }
}

void PBD::printDebug() {
#ifdef MYDEBUG_PREDICT
    printAllPredict();
#else
    printAll();
#endif
}

int PBD::loadObj(std::string name, glm::vec3 T, glm::vec3 R, glm::vec3 S) {
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
                v = glm::vec3((glm::translate(T)
                               * glm::rotate(R.z, glm::vec3(0,0,1))
                               * glm::rotate(R.y, glm::vec3(0,1,0))
                               * glm::rotate(R.x, glm::vec3(1,0,0))
                               * glm::scale(S) * glm::vec4(v,1.f)));
                //extra!!
                points.pos.push_back(v);
                points.posPredict.push_back(v);
                points.vel.push_back(glm::vec3(0.f));
                points.massInv.push_back(1.f);
                points.size++;
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
                std::stringstream ss;
                ss << str.substr(2);
                std::vector<int> aIndices;

                //Parsing
                parseObjFace(ss, aIndices);

                //Collecting(Reassembling)
                //if there are more than 3 vertices for one face then split it in to several triangles
                for (int i = 0; i < aIndices.size(); i++)
                {
                    if (i >= 3)
                    {
                        triangles.push_back(aIndices.at(0));
                        triangles.push_back(aIndices.at(i - 1));
                    }
                    triangles.push_back(aIndices.at(i));
                }
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
        std::cout << "can not open " << name << std::endl;
        return 1;
    }
    file.close();
    return 0;
}

void PBD::parseObjFace(std::stringstream& ss, std::vector<int>& index) {
    char discard;
    char peek;
    //int count;
    int data;

    //One vertex in one loop
    do
    {
        int VI, NI, TI = -1;

        ss >> peek;
        if (peek >= '0' && peek <= '9')
        {
            ss.putback(peek);
            ss >> data;
            VI = data - 1;//index start at 1 in an .obj file but at 0 in an array
            ss >> discard;
        }
        else
        {
            //push default value
            VI = -1;
        }

        ss >> peek;
        if (peek >= '0' && peek <= '9')
        {
            ss.putback(peek);
            ss >> data;
            TI = data - 1;//index start at 1 in an .obj file but at 0 in an array
            ss >> discard;
        }
        else
        {
            //push default value
            TI = -1;
            //hasTexcoord = false;
        }

        ss >> peek;
        if (peek >= '0' && peek <= '9')
        {
            ss.putback(peek);
            ss >> data;
            NI = data - 1;//index start at 1 in an .obj file but at 0 in an array
            //no discard here because it is the end for this vertex
        }
        else
        {
            //push default value
            NI = -1;
            //hasNormal = false;
        }

        index.push_back(VI);

    } while (!ss.eof());
}