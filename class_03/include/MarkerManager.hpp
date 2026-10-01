#pragma once
#include<vector>

struct Marker
{
    int id;
    int x_mm;
    int y_mm;
    int priority;
};

int for_total(const std::vector<Marker> &markers);

bool comesfirst(const Marker& a,const Marker& b);

class MarkerManager{
public:
    bool add(const Marker& marker);
    bool contains(int id) const;
    void showcase();
    int number(int min);
    void modify(int id,int priority); 
    void sort_MarkerManager();
private:
    std::vector<Marker> markers;
};