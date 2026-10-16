#include<iostream>
struct stu{
	int Chinese;
	int English;
	int Math;
	int sum;
	int num;
};
int main(){
	int n;
	std::cin>>n;
	struct stu student[301];
	for(int i=1;i<=n;i++){
		std::cin>>student[i].Chinese;
		std::cin>>student[i].English;
		std::cin>>student[i].Math;
		student[i].sum=student[i].Chinese+student[i].English+student[i].Math;
		student[i].num=i;
	}
	stu a;
	for(int i=1;i<n;i++){
		for(int j=1;j<n;j++){
			if(student[j].sum>student[j+1].sum){
				a=student[j];
				student[j]=student[j+1];
				student[j+1]=a;
			}
			else if(student[j].sum==student[j+1].sum){
				if(student[j].Chinese>student[j+1].Chinese){
					a=student[j];
					student[j]=student[j+1];
					student[j+1]=a;
				}
				else if(student[j].num<student[j+1].num){
					a=student[j];
					student[j]=student[j+1];
					student[j+1]=a;
				}
			}
		}
	}
	for(int i=0;i<5;i++){
		std::cout<<student[n-i].num<<" "<<student[n-i].sum<<std::endl;
	}
	return 0;
}
/*5 80 90 70 90 80 70 85 85 85 100 60 70 70 70 70*/