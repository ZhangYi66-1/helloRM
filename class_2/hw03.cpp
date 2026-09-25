#include<iostream>
int main(){
    int sum=0;
    int n;
    std::cin>>n;
    for(int i=1;i<=n;i++){
        sum+=i;
        std::cout<<sum<<" ";
    }
    return 0;
}
