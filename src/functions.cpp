#include "functions.h"


// rdm class, for gentaring random flot numbers
rdm::rdm() {i=time(0);}
float rdm::randomize() { i=i+1;  srand (i);  return float(rand())/float(RAND_MAX);}



//Norm function 
float Norm(std::vector<float> x1,std::vector<float> x2)
{
return pow(	(pow((x2[0]-x1[0]),2)+pow((x2[1]-x1[1]),2))	,0.5);
}


//sign function
float sign(float n)
{
if (n<0.0){return -1.0;}
else{return 1.0;}
}


//Nearest function
std::vector<float> Nearest(  std::vector< std::vector<float>  > V, std::vector<float>  x){

float min=Norm(V[0],x);
int min_index;
float temp;

for (int j=0;j<V.size();j++)
{
temp=Norm(V[j],x);
if (temp<=min){
min=temp;
min_index=j;}

}

return V[min_index];
}



//Steer function
std::vector<float> Steer(  std::vector<float> x_nearest , std::vector<float> x_rand, float eta){
std::vector<float> x_new;

if (Norm(x_nearest,x_rand)<=eta){
x_new=x_rand;
}
else{


float m=(x_rand[1]-x_nearest[1])/(x_rand[0]-x_nearest[0]);

x_new.push_back(  (sign(x_rand[0]-x_nearest[0]))* (   sqrt( (pow(eta,2)) / ((pow(m,2))+1) )   )+x_nearest[0] );
x_new.push_back(  m*(x_new[0]-x_nearest[0])+x_nearest[1] );

if(x_rand[0]==x_nearest[0]){
x_new[0]=x_nearest[0];
x_new[1]=x_nearest[1]+eta;
}



}
return x_new;
}


//gridValue function
int gridValue(nav_msgs::OccupancyGrid &mapData,std::vector<float> Xp){

float resolution=mapData.info.resolution;
float Xstartx=mapData.info.origin.position.x;
float Xstarty=mapData.info.origin.position.y;

float width=mapData.info.width;
std::vector<signed char> Data=mapData.data;

//returns grid value at "Xp" location
//map data:  100 occupied      -1 unknown       0 free
float indx=(  floor((Xp[1]-Xstarty)/resolution)*width)+( floor((Xp[0]-Xstartx)/resolution) );
int out;
out=Data[int(indx)];
return out;
}

inline float getCellIndex(float cell_x_new,float cell_y_new, float width) {return ( cell_y_new*width)+cell_x_new;}
//search for outliers by checking neighbours of free space cells
void checkNeighbours(nav_msgs::OccupancyGrid &mapData,std::vector<float> Xp, char& obs){


    //std::vector<float> neigbours;
    float resolution=mapData.info.resolution;//m/cell
    float Xstartx=mapData.info.origin.position.x;
    float Xstarty=mapData.info.origin.position.y;

    float width=mapData.info.width;
    float height=mapData.info.height;
    std::vector<signed char> Data=mapData.data;

    float cell_y=floor((Xp[1]-Xstarty)/resolution);
    float cell_x=floor((Xp[0]-Xstartx)/resolution);

    for (int i=-1;i<=1;i++){
        for(int j=-1;j<=1;j++){
            //neigbours.push_back(getIndex(cell_x+i,cell_y+j));
            if (Data[int(getCellIndex(cell_x+i,cell_y+j,width))]>40) obs = 1;
        }
    }
}

// ObstacleFree function-------------------------------------

char ObstacleFree(std::vector<float> xnear, std::vector<float> &xnew, nav_msgs::OccupancyGrid mapsub){
float rez=float(mapsub.info.resolution)*.2;
float stepz=int(ceil(Norm(xnew,xnear))/rez); 
std::vector<float> xi=xnear;
char  obs=0; char unk=0; char free=0;
 
geometry_msgs::Point p;
for (int c=0;c<stepz;c++){
  xi=Steer(xi,xnew,rez);
  		

   //if (gridValue(mapsub,xi) ==100){     obs=1; }
    if (gridValue(mapsub,xi) >=40) {     obs=1; }

    if (gridValue(mapsub,xi) ==-1){      unk=1;	break;}

    if (gridValue(mapsub,xi)<40) {free=1;

    }
  }
checkNeighbours(mapsub,xi,obs);
char out=0;
 xnew=xi;
 if (unk==1){  out=-1;}
 	
 if (obs==1){  out=0;}
 		
 if (unk!=1 && free==1 && obs!=1 ){   out=1;}

 return out;
 }

rrt_exploration::FrontierTF Point2Tf(cartographer_ros_msgs::SubmapList SubmapList_,std::vector<float> point) {
    bool init=true;
    float min_dist;
    float dist;
    cartographer_ros_msgs::SubmapEntry best_submap;
    rrt_exploration::FrontierTF newTFPoint;
    for (auto& submap_msg : SubmapList_.submap) {
        std::vector<float> submap_point;
        submap_point.push_back(submap_msg.pose.position.x);
        submap_point.push_back(submap_msg.pose.position.y);
        dist=Norm(submap_point,point);
        if(init==true){
            min_dist=dist;
            best_submap=submap_msg;
            init=false;
        }
        else {
            if(min_dist>dist){
            min_dist=dist;
            best_submap=submap_msg;
            }
        }
    }
    std::cout<<"najbliza submapa "<<best_submap.submap_index<<std::endl;
    geometry_msgs::Transform tr;
    geometry_msgs::Vector3 vec;
//    transform.setOrigin( tf::Vector3((point[0]-best_submap.pose.position.x), (point[1]-best_submap.pose.position.y),0) );
//    transform.setRotation( tf::Quaternion(0, 0, 0, 1) );
    vec.x=point[0]-best_submap.pose.position.x;
    vec.y=point[1]-best_submap.pose.position.y;
    vec.z=0;
    newTFPoint.submapIndex=best_submap.submap_index;
    newTFPoint.transform=vec;

    return newTFPoint;

}

 


     
   


  
 
 
 
 





























