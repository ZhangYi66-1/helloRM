#include<iostream>
#include<cstring>
#include<deque>
using namespace std;
//BFS

int BFS(){
	int dadx[8][8];
	int dady[8][8];
	deque<pair<int,int>>q1;
	int row,col;
	cin>>row>>col;
	int a[row][col];
	int b[row][col];
	int c[row][col];
	int total=0;
	int num,da;
	while(total<row*col){
		cin>>num>>da;
		for(int i=0;i<num;i++){
			a[total/col][total%col]=da;
			b[total/col][total%col]=0;
			total++;
		}
	}
	int dx[4]={0,0,1,-1};
	int dy[4]={1,-1,0,0};
	int z_x=3,z_y=3;
	int qi_x=0,qi_y=0;
	int x1=0,y1=0;
	b[x1][y1]=1;
	pair<int,int> cr;
	//q1.push_back({x1, y1});
	while(b[z_x][z_y]!=1){
		for(int i=0;i<4;i++){
			if(x1+dx[i] >= 0 && y1+dy[i] >= 0 && x1+dx[i] < row && y1+dy[i] < col ){
				if(b[x1+dx[i]][y1+dy[i]]==0 && a[x1+dx[i]][y1+dy[i]]!=0){
					q1.push_back({x1+dx[i],y1+dy[i]});
					dad[x1+dx[i]][y1+dy[i]]={x1,y1};
					b[x1+dx[i]][y1+dy[i]]=1;
				}
			}
		}
		cr=q1.front();
		q1.pop_front();
		x1=cr.first;
		y1=cr.second;
	}
	int x,y;
	x=z_x;
	y=z_y;
	cout<<x<<","<<y<<" ";
	while(x!=qi_x || y!=qi_y){
		cout<<dad[x][y].first<<","<<dad[x][y].second<<" ";
		x=dad[x][y].first;
		y=dad[x][y].second;
	}
	//cout<<x<<","<<y<<" "<<endl;
	
	for (int i = 0; i < row; i++) {
        for (int j = 0; j < col; j++) {
            cout << a[i][j] << " ";
        }
        cout << endl;
    }
    
}
int main(){
	BFS();
	
	return 0;
} 
 //4 4 3 1 3 0 4 1 1 0 5 1
//8 8 1 4 0 1 3 1 1 1 0 1 1 1 3 1 0 1 3 1 1 1 3 2 1 1 1 1 3 1
//1 1 8 1 1 2 3 1 1 1 8 1 1 5 3 2 1 1 3 1 8 1 0 1 1 2 0 1 1 1 1 1 8 1 3 1
//0 1 1 1 3 1 1 1 8 1 1 7 8 1 1 3 0 1 1 1 0 1 3 1 1 1