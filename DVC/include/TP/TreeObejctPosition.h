#pragma once
#include <math.h>
#include <vector>


class Point{
public:
    Point(double p1, double p2) : pos1(p1),pos2(p2){};
    double getPos1()const{return pos1;}; //zwraca pozycje 1
    double getPos2()const{return pos2;}; //zwraca pozycje 2
    void setPos(double p1, double p2) { pos1 = p1;pos2 = p2;}; //przypisuje pozycje punktu do zmiennych prywatnych
    double countDistance(const Point& p)const {return pow(p.pos1-pos1,2) + pow(p.pos2-pos2,2); };
    //zwraca kwadrat roznicy polozenia od punktu do punktu
private:
    double pos1, pos2;
};


class TreeObejctPosition{
public:
    TreeObejctPosition(unsigned short id, Point sum, double radius, double num);

    bool addIfInclude (const Point& p,unsigned short idp, double weigth);
    bool ifInclude (const Point& p);
    Point getPoint()const;
    unsigned short getId()const{return id;};
    double getRadius()const {return radius;};
    bool isVisited()const {return visited;};
    void setVisited(){visited = true;};
    unsigned short getUpdateCounter()const {return updateCounter;};

private:
    unsigned short id;
    std::vector<unsigned short> id_list;
    Point sum;
    double num;
    const double radius;
    bool visited;
    unsigned short updateCounter;
};

