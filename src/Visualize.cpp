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
    std::vector<float> labels;
    std::vector<float> IntPoints;
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

		if ((pcl::io::loadPCDFile(argv[2], *cloud2)) == -1)
				    {
				      PCL_ERROR("Couldn't read the file \n");
				      return (-1);
				    }

//---------------------------------------------------------------
		pcl::VoxelGrid<pcl::PointXYZRGBL> sor;
		  sor.setInputCloud (cloud2);
		  sor.setLeafSize (0.03f, 0.03f, 0.03f);
		  sor.filter (*cloud2);
//---------------------------------------------------------------
		labels = ReadFileToVector(argv[3]);
		IntPoints = ReadFileToVector(argv[4]);

		//here we initialize all labels to 255
		/*for (size_t i =0; i < cloud2->size(); i++)
			//cloud2->points[i].label = 255;
			cloud2->points[i].rgba = 255;
*/
		int NofPositive = 0;
		int NofNegative = 0;
		for(int i = 0;i < (int)labels.size(); i++)
			{
				//std::cout<<labels.size()<<std::endl;
				std::cout<<labels[i]<<"   "<<IntPoints[i]<<endl;
				//if(labels[i] == 1)
				if(labels[i] > 0)
				{
					cloud2->points[IntPoints[i]].r = 255;
					cloud2->points[IntPoints[i]].g = 255;
					cloud2->points[IntPoints[i]].b = 255;
					cout<<"point "<<IntPoints[i]<<" is set to white"<<endl;
					NofPositive++;
				}
				else
				{
					cloud2->points[IntPoints[i]].r = 0;
					cloud2->points[IntPoints[i]].g = 255;
					cloud2->points[IntPoints[i]].b = 255;
					NofNegative++;

				}
			}
		cout<<NofPositive <<"Points out of "<< cloud2->points.size()<<" were positive."<<endl;

		if(pcl::io::savePCDFileASCII("Focused3.pcd",*cloud2) == -1)
		  				 {
		  					 PCL_ERROR("Focused save fail!");
		  				 }
		//clock_t start = clock();
		Vis(cloud2);
		//printf("Time elapsed: %f\n", (((double)clock() - start) *100)/ CLOCKS_PER_SEC);
		//pcl::visualization::PCLVisualizer vie("test");
		//vie.addPointCloud( cloud2);
		// while(!vie.wasStopped())
		  //{
			//  vie.spinOnce();
		  //}
		}
	else
		cerr<<"First argument should be either 1:cloud+normal or 2:result visulization!";
}//end of main

