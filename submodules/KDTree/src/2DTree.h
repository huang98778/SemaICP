/*
 * @Author: your name
 * @Date: 2020-10-31 17:22:09
 * @LastEditTime: 2020-10-31 17:32:24
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /semICP/submodules/KDDoubleDimTree/DoubleDimTree.h
 */

#define KDtreeSize 1000

#define UL unsigned long

using namespace std;


struct coordinate
{
    double x = 0;
    double y = 0;
    UL index = 0;
};

struct TreeNode
{
    struct coordinate dom_elt;
    UL split = 0;
    struct TreeNode* left = nullptr;
    struct TreeNode* right = nullptr;
};

class DoubleDimTree
{
private:
    /* data */
public:
    DoubleDimTree(/* args */);
    ~DoubleDimTree();

    TreeNode *buildKdtree(coordinate exm_set[], UL size, TreeNode *T);
    void chooseSplit(coordinate exm_set[], UL size, UL &split, coordinate &SplitChoice);
    void searchNearest(TreeNode *Kd, coordinate target, coordinate &nearestpoint, double &distance);
    double getDistance(coordinate a, coordinate b);
};
