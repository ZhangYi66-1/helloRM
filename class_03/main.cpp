#include"MarkerManager.hpp"
#include<vector>
#include<iostream>

int main(){
    std::vector<Marker> markerss={
        {1,100,200,80},
        {2,200,-50,95},
        {3,0,0,60},
    };
    for(const Marker& marker : markerss){
        std::cout<<marker.id<<" (";
        std::cout<<marker.x_mm<<",";
        std::cout<<marker.y_mm<<") ";
        std::cout<<marker.priority<<std::endl;
    }
    Marker *p=&markerss[0];
    Marker &pp=markerss[0];
    std::cout << p->id <<" "<< pp.id<<std::endl;
    std::cout << for_total(markerss) <<std::endl;
    class MarkerManager mm;
    for(int i=0;i<3;i++){
    	mm.add(markerss[i]);
	}
    std::cout<<mm.contains(2)<<std::endl;
    std::cout<<mm.number(80)<<std::endl;
    mm.showcase();
    mm.modify(1,95);
    mm.sort_MarkerManager();
    mm.showcase();
    int id,priority,n;
    struct Marker m;
    while(1){
        //1 查看全部标记、2 添加标记、3 按编号查询、4 按阈值统计、0 退出
    	std::cout<<"1.查看全部标记"<<std::endl;
    	std::cout<<"2.添加标记"<<std::endl;
    	std::cout<<"3.按编号查询"<<std::endl;
    	std::cout<<"4.按阈值统计"<<std::endl;
    	std::cout<<"0.退出"<<std::endl;
    	std::cin>>n;
    	switch(n){
    		case 1:
    			mm.showcase();
    			break;
    		case 2:
				std::cout<<"请输入编号:\n";
				std::cin>>m.id;
				std::cout<<"请输入x坐标\n";
				std::cin>>m.x_mm;
				std::cout<<"请输入y坐标\n";
				std::cin>>m.y_mm;
				std::cout<<"请输入优先级\n";
				std::cin>>m.priority;
    			mm.add(m);
    			mm.sort_MarkerManager();
    			break;
    		case 3:
    			std::cin>>id;
    			mm.contains(id);
    			break;
    		case 4:
    			std::cin>>priority;
    			std::cout<<mm.number(priority)<<std::endl;
    			break;
    		case 0:
    			return 0;
		}
    }
    return 0;
}