#include<iostream> 
#include<string>
//using namespace std;
void move_(char a,int &x,int &y){
	if(a=='U'){
		y++;
	}
	if(a=='D'){
		y--;
	}
	if(a=='L'){
		x--;
	}
	if(a=='R'){
		x++;
	}
	return;
}
int main(){
	std::string a;
	std::cin>>a;
	int len=a.size();
	int x=0,y=0;
	for(int i=0;i<=len;i++){ 
		move_(a[i],x,y);
	}
	if(x==0 && y==0){
        std::cout<<"true";
    }
    else{
        std::cout<<"false";
    }
	return 0;
}
