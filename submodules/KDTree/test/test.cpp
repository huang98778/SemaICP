/*
 * @Author: your name
 * @Date: 2020-10-31 17:20:56
 * @LastEditTime: 2020-10-31 17:47:31
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /semICP/submodules/KDTree/test/test.cpp
 */
#include "2DTree.h"
#include <iostream>

int main(){
    
    
    coordinate exm_set[6];
    exm_set[0].x = 2;    exm_set[0].y = 3;
    exm_set[1].x = 5;    exm_set[1].y = 4;
    exm_set[2].x = 9;    exm_set[2].y = 6;
    exm_set[3].x = 4;    exm_set[3].y = 7;
    exm_set[4].x = 8;    exm_set[4].y = 1;
    exm_set[5].x = 7;    exm_set[5].y = 2;

    struct TreeNode * root = nullptr;
    DoubleDimTree *tree(new DoubleDimTree());
    root = tree->buildKdtree(exm_set, 6, root);

    coordinate nearestpoint;
    double distance;
    coordinate target;
    target.x = 2.1;
    target.y = 3.2;

    tree->searchNearest(root, target, nearestpoint, distance);
    std::cout<<"The nearest distance is "<<distance<<",and the nearest point is "<<nearestpoint.x<<","<<nearestpoint.y<<std::endl;

    return 0;
}