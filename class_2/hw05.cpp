#include<iostream>
//using namespace std;
int main(){
	int a[1000],b;
	for(int i=0;i<=1000;i++){
		std::cin>>a[i];
		if(a[i]==0){
			b=i;
			break;
		}
	}
	for(int i=b;i>=0;i--){
		std::cout<<a[i]<<" ";
	}
	return 0;
}
