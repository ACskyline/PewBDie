//
// Created by sean on 3/2/18.
//

#ifndef MYPBD_POINTLIST_H
#define MYPBD_POINTLIST_H

#include <vector>
#include "glm/glm.hpp"

class Points {
public:
    std::vector<glm::vec3> pos;
    std::vector<glm::vec3> posPredict;
    std::vector<glm::vec3> vel;
    std::vector<bool> posLock;
    std::vector<float> massInv;
    int size;

    Points();
    ~Points();
    void resize(int _size);
    void clear();
    void unlockPosAll();
};


#endif //MYPBD_POINTLIST_H
