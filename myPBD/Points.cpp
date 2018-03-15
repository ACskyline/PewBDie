//
// Created by sean on 3/2/18.
//

#include "Points.h"

Points::Points() {

}

Points::~Points() {

}

void Points::resize(int _size) {
    pos.clear();
    posPredict.clear();
    vel.clear();
    posLock.clear();
    massInv.clear();

    size = _size;
    pos.resize(size);
    posPredict.resize(size);
    vel.resize(size);
    posLock.resize(size);
    massInv.resize(size);
}

void Points::clear() {
    resize(0);
}

void Points::unlockPosAll() {
    for(std::vector<bool>::iterator i = posLock.begin(); i != posLock.end(); ++i)
    {
        *i = false;
    }
}