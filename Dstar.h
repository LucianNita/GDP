#ifndef DSTAR_H
#define DSTAR_H

#include <math.h>
#include <stack>
#include <queue>
#include <list>
#include <stdio.h>
#include <ext/hash_map>

using namespace std;
using namespace __gnu_cxx;

class Mesh {
 public:
  int x;
  int h;
  pair<double,double> key;
  
  bool operator > (const Mesh &node) const {
    if (key.first-0.0000001 > node.key.first) return true;
    else if (key.first < node.key.first-0.0000001) return false;
    return key.second > node.key.second;
  }
  
  bool operator <= (const Mesh &node) const {
    if (key.first < node.key.first) return true;
    else if (key.first > node.key.first) return false;
    return key.second < node.key.second + 0.0000001;
  }
  
  bool operator < (const Mesh &node) const {
    if (key.first + 0.000001 < node.key.first) return true;
    else if (key.first - 0.000001 > node.key.first) return false;
    return key.second < node.key.second;
  }

  bool operator != (const Mesh &node) const {
    return ((x != node.x) || (h != node.h));
  }
  
  bool operator == (const Mesh &node) const {
    return ((x == node.x) && (h == node.h));
  }  
  
};

struct Meshpoint {
  int x,h;
};

struct nodeData {

  double g;
  double rhs;
  double c;

};

class Mesh_hash {
 public:
  size_t operator()(const Mesh &node) const {
    return node.x + /*34245*/node.h;
  }
};


typedef priority_queue<Mesh, vector<Mesh>, greater<Mesh> > ds_pq;
typedef hash_map<Mesh, float, Mesh_hash, equal_to<Mesh> > ds_oh;
typedef hash_map<Mesh,nodeData, Mesh_hash, equal_to<Mesh> > ds_ch;

class Dstar {

 public:

  Dstar();
  void   initialize(int sX, int sH, int gX, int gH);
  void   updateCost(int x, int h, double cost);
  void   updateStart(int x, int h);
  void   updateGoal(int x, int h);
  bool   replan();

  list<Mesh> getPath();
  double getG(Mesh u);
 
 private:

  list<Mesh> path;
  double km;
  int ExpLim;
  double C1;
  Mesh nodestart, nodegoal, nodelast;
  

  ds_pq openList;
  ds_ch cellHash;
  ds_oh openHash;

  bool   close(double x, double h);
  void   makeNewCell(Mesh p);
  double getG(Mesh p);
  double getRHS(Mesh p);
  void   setG(Mesh p, double g);
  double setRHS(Mesh p, double rhs);
  double eightCondist(Mesh a, Mesh b);
  int    computeShortestPath();
  void   updateVertex(Mesh u);
  void   insert(Mesh u);
  void   remove(Mesh u);
  double trueDist(Mesh a, Mesh b);
  double heuristic(Mesh a, Mesh b);
  Mesh  calculateKey(Mesh u);
  void   getSucc(Mesh u, list<Mesh> &p);
  void   getPred(Mesh u, list<Mesh> &p);
  double cost(Mesh a, Mesh b);
  bool   occupied(Mesh u);
  bool   isValid(Mesh u);
  float  Hash_key(Mesh u);
};

#endif