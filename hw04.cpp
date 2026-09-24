#include<iostream>
int main(){
    int sum=0,min,max;
    int n;
    std::cin>>n;
    min=n;
    max=n;
    sum=n;
    for(int i=1;i<=4;i++)
    {
        std::cin>>n;
        sum+=n;
        if(max<n){
            max=n;
        }
        if(min>n){
            min=n;
        }
    }
    double average;
    average=(sum-min-max)/3.0;
    printf("%.2f",average);
    return 0;
}