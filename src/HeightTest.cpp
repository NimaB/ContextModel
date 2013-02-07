/*
 * HeightTest.cpp
 *
 *  Created on: Feb 4, 2013
 *      Author: sm
 */
//Fist input is the path to the point cloud
//Second input is the point index
#include "functions.cpp"
int main(int argc, char **argv)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZRGB>);

	if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (argv[1], *cloud) == -1) //* load the file
	   {
	     PCL_ERROR ("Couldn't read file \n");
	     return (-1);
	   }
	//Quaternion<float> q;
	//q = Quaternion<float> (-0.500398163355, 0.499999841466, -0.499601836645, 0.499999841466);
	//pcl::transformPointCloud (*cloud, *cloud2, Eigen::Vector3f (0, 0, 0), q);
	pcl::transformPointCloud(*cloud,*cloud2,pcl::getTransformation(0,0,1.23,0,PI/6,0));
	//float *a;float *b;float *c;
	//pcl::getEulerAngles(q,a,b,c);
	int index = 0;
	for(int i = 1; i < (argc-1); i++)
  {
		index = atoi(argv[i+1]);
	std::cout<<"The point #"<<i<<"coordinates are:"<<"("<<cloud->points[index].x<<","<<
			cloud->points[index].y<<","<<
			cloud->points[index].z<<")"<<std::endl;

	std::cout<<"The transfered point #"<<i<<"coordinates are:"<<"("<<cloud2->points[index].x<<","<<
				cloud2->points[index].y<<","<<
				cloud2->points[index].z<<")"<<std::endl;

  }
}




