#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <stdio.h>
#include <time.h>
#include "functions.cpp"


void Vis (pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals,int VP)
{
  // --------------------------------------------------------
  // -----Open 3D viewer and add point cloud and normals-----
  // --------------------------------------------------------
  pcl::visualization::PCLVisualizer viewer ;//(new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer.setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  viewer.addPointCloud<pcl::PointXYZRGB> (cloud, rgb , "sample cloud",VP);
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
  viewer.addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (cloud, normals, 10, 0.05, "normals");
  viewer.addCoordinateSystem (1.0);
  viewer.initCameraParameters ();
  while(!viewer.wasStopped())
  {
	  viewer.spin();
  }
}
//overload:
void Vis (pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud)
{
  pcl::visualization::PCLVisualizer viewer ;
  viewer.setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBL> rgb(cloud);
  viewer.addPointCloud<pcl::PointXYZRGBL> (cloud, rgb , "sample cloud");
  //viewer.addPointCloud<pcl::PointXYZRGBL> (cloud, "sample cloud");
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "sample cloud");
  while(!viewer.wasStopped())
  {
	  viewer.spin();
  }

}
//
void Vis (pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
  pcl::visualization::PCLVisualizer viewer ;
  viewer.setBackgroundColor (0, 0, 0);
  //pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  viewer.addPointCloud (cloud, "sample cloud");
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
  //viewer.addCoordinateSystem (1.0);
  //viewer.initCameraParameters ();

  while(!viewer.wasStopped())
  {
	  viewer.spin();
  }

}


//First argument will choose 1:cloud+normals or 2:result visualization
int main(int argc, char **argv) {
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZRGBL>);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normal (new pcl::PointCloud<pcl::Normal>);
//    std::vector<float> labels;
    std::vector<float> ScoreList;//Here the computed score for each point is saved
//    std::vector<float> IntPoints;
if(atoi(argv[1]) == 1)
{
	if ((pcl::io::loadPCDFile(argv[2], *cloud) || (pcl::io::loadPCDFile(argv[3], *cloud_normal))) == -1)
		    {
		      PCL_ERROR("Couldn't read the file \n");
		      return (-1);
		    }
	Vis(cloud,cloud_normal,1);
	Eigen::Affine3f transformation = pcl::getTransformation(1.21,0,0,ALFA,BETHA,GAMA);
	pcl::transformPointCloud(*cloud,*cloud,transformation);
	Vis(cloud,cloud_normal,2);
}
else
	if(atoi(argv[1]) == 2)
	{

		std::cout<<"loading the pointcloud ..."<<std::endl;
		if ((pcl::io::loadPCDFile(argv[2], *cloud2)) == -1)
				    {
				      PCL_ERROR("Couldn't read the pointcloud file \n");
				      return (-1);
				    }

//---------------------------------------------------------------
		std::cout<<"Down sampling the pointcloud ..."<<std::endl;
		pcl::VoxelGrid<pcl::PointXYZRGBL> sor;
		  sor.setInputCloud (cloud2);
		  sor.setLeafSize (0.03f, 0.03f, 0.03f);
		  sor.filter (*cloud2);
//---------------------------------------------------------------
//		labels = ReadFileToVector(argv[3]);
//		IntPoints = ReadFileToVector(argv[4]);

		  std::cout<<"loading the Score List ..."<<std::endl;
		  ScoreList = ReadFileToVector(argv[3]);

		int NofPositive = 0;
		int NofNegative = 0;

		for(size_t i = 0;i < cloud2->points.size(); i++)
		{
			std::cout<<"test i = "<<i<<std::endl;
			cloud2->points[i].label = 255;
			std::cout<<"test label "<<i<<std::endl;
			if(ScoreList[i] > 0)
			{
				cloud2->points[i].r = 255 - ScoreList[i];
				NofPositive++;
			}
			else
				NofNegative++;

			std::cout<<"score list"<<i<<":"<<ScoreList[i]<<"label"<<cloud2->points[i].label<<std::endl;
			}
		cout<<NofPositive <<" Points out of "<< cloud2->points.size()<<" were positive."<<endl;

		std::cout<<"Writing the resulting pointcloud ..."<<std::endl;
		if(pcl::io::savePCDFileASCII("Focused3.pcd",*cloud2) == -1)
		  				 {
		  					 PCL_ERROR("Focused save fail!");
		  				 }

		std::cout<<"Visualizing the pointcloud ..."<<std::endl;
		Vis(cloud2);

		}
	else
		cerr<<"First argument should be either 1:cloud+normal or 2:result visulization!";
}//end of main

