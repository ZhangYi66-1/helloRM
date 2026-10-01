#include"MarkerManager.hpp"
#include<vector>
#include<iostream>
#include<algorithm>

int for_total(const std::vector<Marker> &markers){
    int total_=0;
    for(const Marker& marker : markers){
        if(marker.priority>=80){
            total_++;
        }
    }
    return total_;
}

bool MarkerManager::add(const Marker& marker){
    if(marker.id>=0 && marker.priority<=100 && marker.priority>=0 ){
        for(const Marker& marker_ : markers){
            if(marker_.id==marker.id){
                return false;
            }
        }
        markers.push_back(marker);
        return true;
    }
    else return false;
}
bool MarkerManager::contains(int id) const{
    for(const Marker& m : markers){
        if(m.id==id){
            std::cout<<"yes\n";
            return true;
        }
    }
    std::cout<<"no\n";
    return false;
}
void MarkerManager::showcase(){
    int i=markers.size();
    if(i==0){
        return;
    }
    else{
        for(const Marker& m : markers){
            std::cout<<m.id<<" ("<<m.x_mm<<","<<m.y_mm<<") "<<m.priority<<std::endl;
        }
        return;
    }
}
int MarkerManager::number(int min){
    int total=0;
    for(const Marker& m : markers){
        if(m.priority>min) total++;
    }
    return total;
}

void MarkerManager::modify(int id,int priority){
	for(Marker m : markers){
		if(m.id==id){
			if(priority<=100 && priority>=0){
				m.priority=priority;
				std::cout<<"success\n";
				return;
			}
			std::cout<<"false\n";
			return;
		}
	}
	std::cout<<"false\n";
	return;
}
bool comesfirst(const Marker& a,const Marker& b){
	if(a.priority==b.priority) return a.id<b.id;
	else return a.priority>b.priority;
}
void MarkerManager::sort_MarkerManager(){
	std::sort(markers.begin(),markers.end(),comesfirst);
	return;
}