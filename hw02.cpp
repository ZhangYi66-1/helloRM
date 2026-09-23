#include<iostream>
int main(){
    int year=0;
    std::cin>>year;
    if(year%4==0){
        if(year%100==0){
            if(year%400==0)
            {
                std::cout<<"yes";
            }  
            else{
                std::cout<<"no";
            }
        }
        else{
            std::cout<<"yes";
        }
    }
    else{
        std::cout<<"no";
    }
    return 0;
}