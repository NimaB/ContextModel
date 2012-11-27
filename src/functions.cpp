#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/extract_indices.h>
//#include </usr/include/pcl-1.5/pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/vfh.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/common.h>//minmax3D
#include <pcl/registration/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <fstream>

using namespace std;
using namespace Eigen;

#define Lparam 0 // parameter for loading normals from a file
// Euler angels
#define ALFA  0
#define BETHA ((PI/2) + (PI/6))
#define GAMA  0

#define PI 3.14159265
//#define OverWrite 0
#define BPnTresh 5 // A threshold for cloud blob number of points
#define DaFiFormat 1 //the format in which the feature vector is saved in,
                     //1: For the one includes class of sample feature indices and ignore '0' values.
                     //2: For the one includes only the feature vector with all values.

//-----------------------------------------------------------------

/*
  void Vis (pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud)
  {
  pcl::visualization::PCLVisualizer viewer ;
  viewer.setBackgroundColor (0, 0, 0);
  viewer.addPointCloud<pcl::PointXYZRGBL> (cloud, "sample cloud");
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  while(!viewer.wasStopped())
  {
  viewer.spin();
  }
  
  }
*/

//-----------------------------------------------------------------------------------------
//This function generates a pointcloud of dense points with given dimentions
void CloudGenerate (float MinX,float MaxX,float MinY,float MaxY,float MinZ,float MaxZ,float step,pcl::PointCloud<pcl::PointXYZRGBL>::Ptr CloudOut)
{
  CloudOut->height = abs(ceil((MaxX - MinX)/step));
  CloudOut->width  = abs(ceil((MaxY - MinY)/step) * ceil((MaxZ - MinZ)/step));
  cout<<CloudOut->height<<" "<<CloudOut->width<<endl;
  
  CloudOut->points.resize(CloudOut->height * CloudOut->width);
  int ind = 0;
  for(float i = MinX; i <= MaxX;i+= step)
    {
      for(float j = MinY; j <= MaxY;j+= step)
	{
	  for(float k = MinZ; k <= MaxZ;k+= step)
	    {
	      CloudOut->points[ind].x = i;
	      CloudOut->points[ind].y = j;
	      CloudOut->points[ind].z = k;
	      ind++;
	    }
	}
    }
  
}
//This function generates a small cloud around the given center with the given radius. NumofLayers shows layers number around the center.
void CloudGenerate (pcl::PointXYZRGBL CloudCenter,float CloudRadius,int NumofLayers,pcl::PointCloud<pcl::PointXYZRGBL>::Ptr CloudOut)
{
	CloudOut->height = 1;//(NumofLayers * 2) + 1;//includes the center point.
	CloudOut->width = ((NumofLayers * 2) + 1) * ((NumofLayers * 2) + 1);
	cout<<CloudOut->height<<" "<<CloudOut->width<<endl;

	CloudOut->points.resize(CloudOut->height * CloudOut->width);

	float step = CloudRadius / NumofLayers;
	int ind = 0;
	for (float i = -CloudRadius; i <= CloudRadius; i+= step)
			{
			CloudOut->points[ind].x = CloudCenter.x + i;
			CloudOut->points[ind].y = CloudCenter.y;
			CloudOut->points[ind].z = CloudCenter.z;
			ind++;
			if(i != 0)
			{
				CloudOut->points[ind].x = CloudCenter.x + i;
				CloudOut->points[ind].y = CloudCenter.y + i;
				CloudOut->points[ind].z = CloudCenter.z;
				ind++;
			}

			}
		for (float j = -CloudRadius; j <= CloudRadius; j+= step)
					{
					if(j != 0)
						{
						CloudOut->points[ind].x = CloudCenter.x;
						CloudOut->points[ind].y = CloudCenter.y + j;
						CloudOut->points[ind].z = CloudCenter.z;
						ind++;
						CloudOut->points[ind].x = CloudCenter.x;
						CloudOut->points[ind].y = CloudCenter.y + j;
						CloudOut->points[ind].z = CloudCenter.z + j;
						ind++;
						}

					}
		for (float k = -CloudRadius; k <= CloudRadius; k+= step)
							{
							if(k != 0)
								{
								CloudOut->points[ind].x = CloudCenter.x;
								CloudOut->points[ind].y = CloudCenter.y;
								CloudOut->points[ind].z = CloudCenter.z + k;
								ind++;
								CloudOut->points[ind].x = CloudCenter.x + k;
								CloudOut->points[ind].y = CloudCenter.y;
								CloudOut->points[ind].z = CloudCenter.z + k;
								ind++;
								}


							}

}
//-----------------------------------------------------------------------------------------
void DownSample(pcl::PointCloud<pcl::PointXYZRGB>::Ptr in_cloud,pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_cloud)
{
  
  pcl::VoxelGrid<pcl::PointXYZRGB> sor;
  sor.setInputCloud (in_cloud);
  sor.setLeafSize (0.03f, 0.03f, 0.03f);
  sor.filter (*out_cloud);
  
  
}
//-----------------------------------------------------------------------------------------

std::vector<float> ReadFileToVector(string Fname)
{
  std::vector<float> result;
  std::ifstream myFile (Fname.c_str());
  std::string lineData;
  
  
  if(myFile.is_open())
    {
      while(myFile.good())
	{
	  getline(myFile,lineData);
	  //std::cout<<"line data is:"<<lineData<<endl;
	  float f;
	  std::stringstream linestream(lineData);
	  
	  while(linestream >> f)
	    {
	      //cout<<"f is :"<<f<<endl;
	      //result.push_back(f);
	    }
	  result.push_back(f);
	}
      myFile.close();
      
    }
  else std::cerr<<"Unable to open the file"<<Fname<<"!"<<endl;
  return result;
}
/*std::vector<std::vector<float> > ReadFileToVector(string Fname)
  {
  std::vector<std::vector<float> > result;
  std::ifstream myFile (Fname.c_str());
  std::string lineData;
  
  
  if(myFile.is_open())
  {
  while(myFile.good())
  {
  getline(myFile,lineData);
  //std::cout<<"line data is:"<<lineData<<endl;
  float f;
  std::vector<float> row;
  std::stringstream linestream(lineData);
  
  while(linestream >> f)
  {
  //cout<<"f is :"<<f<<endl;
  row.push_back(f);
  }
  
  
  result.push_back(row);
  
  
  }
  myFile.close();
  
  }
  else std::cerr<<"Unable to open the file!"<<endl;
  return result;
  
  
  }*/

//-----------------------------------------------------------------------------------------
//function for estimating the height of the sensor to use as an input for translation vector
//in case we dont have the information about sensor's height with a weak assumption that the lowest point in the pointcloud is a point of the floor
//so the floor is a part of the pointcloud.
float SHEstimate(pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_in)
{
  
  //Get MinMax3D of the pointcloud
  
  Eigen::Vector4f Min_pt;
  Eigen::Vector4f Max_pt;
  
  pcl::getMinMax3D(*cloud_in,Min_pt,Max_pt);
  
  //	   std::cerr<<"X's Min and Max:" << Min_pt(0)<<","<<Max_pt(0) << endl;
  //	   std::cerr<<"Y's Min and Max:" << Min_pt(1)<<","<<Max_pt(1) << endl;
  //	   std::cerr<<"Z's Min and Max:" << Min_pt(2)<<","<<Max_pt(2) << endl<<endl;
  
  return(fabs((float)Min_pt[1]));
}

//overload:
float SHEstimate(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in)
{
  
  //Get MinMax3D of the pointcloud
  
  Eigen::Vector4f Min_pt;
  Eigen::Vector4f Max_pt;
  
  pcl::getMinMax3D(*cloud_in,Min_pt,Max_pt);
  
  //	   std::cerr<<"X's Min and Max:" << Min_pt(0)<<","<<Max_pt(0) << endl;
  //	   std::cerr<<"Y's Min and Max:" << Min_pt(1)<<","<<Max_pt(1) << endl;
  //	   std::cerr<<"Z's Min and Max:" << Min_pt(2)<<","<<Max_pt(2) << endl<<endl;
  
  return(fabs((float)Min_pt[1]));
}
//-----------------------------------------------------------------------------------------





//Function to write features and their class to a file as one line
//cofsample shows the current samples class which is a positive or negative
//overwrite is a boolean to choose if the file should be overwritten or not.
void WtoFile(int cofsample,VectorXf features,bool overwrite,string data_filename,string label_filename,int format)
{
  ofstream out;
  ofstream out2;
  if(overwrite)
    {
      cerr<<"overwritting"<<endl;
      //out.open("test1.txt");
      out.open(data_filename.c_str());
      out2.open(label_filename.c_str());
    }
  else
    {
      //cerr<<"Adding to existing file"<<endl;
      //out.open("test1.txt",fstream::in | fstream::out | fstream::app);
      out.open(data_filename.c_str(),fstream::in | fstream::out | fstream::app);
      out2.open(label_filename.c_str(),fstream::in | fstream::out | fstream::app);
    }
  if( format == 1 )
    out<<cofsample<<"    ";
  
  out2<<cofsample<<"\n";
  
  for(int i = 0;i < features.rows(); i++)
    {
      if( format == 1 )
	{
	  if(features[i] != 0)
	    out<<i+1<<":"<<features[i]<<" ";
	}
      else
	if( format == 2 )
	  out<<features[i]<<" ";
    }
  out<<"\n";
  //	ostream_iterator<std::string> output_iterator("test.txt", "\n");
  //    copy(output_iterator);
}

//OverLoaded:
void WtoFile(std::vector<int> interestpoints,bool overwrite,string data_filename)
{
  ofstream out;
  if(overwrite)
    {
      cerr<<"overwritting"<<endl;
      //out.open("test1.txt");
      out.open(data_filename.c_str());
    }
  else
    {
      out.open(data_filename.c_str(),fstream::in | fstream::out | fstream::app);
    }
  
  out<<interestpoints[0]<<" "<<interestpoints[1]<<"\n";
  
}

//--------------------------------------------------------------------------------------

//-----------------------------------------------------------------------------------





//Function for Normal estimation
void NormEst(pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_in,pcl::PointCloud<pcl::Normal>::Ptr normals,bool load,string FilePath)
{
  if(load)
    {
      if(pcl::io::loadPCDFile<pcl::Normal> ("normals.pcd",*normals) == -1)
	{
	  PCL_ERROR("No file to read!");
	  return ;
	}
    }
  else
    if((cloud_in->isOrganized()) && (cloud_in->width > 1))
      {
	// Using IntegralImages.
	cout<<"Estimating Normals using Integral images"<<endl;
	pcl::IntegralImageNormalEstimation<pcl::PointXYZRGBL, pcl::Normal> Ine;
	Ine.setNormalEstimationMethod(Ine.AVERAGE_3D_GRADIENT);
	Ine.setMaxDepthChangeFactor(0.02f);
	Ine.setNormalSmoothingSize(10.0f);
	Ine.setInputCloud(cloud_in);
	Ine.compute(*normals);
	
      }
    else
      {
	//using normal method
	//cout<<"Estimating Normals using normal method"<<endl;
	pcl::NormalEstimation<pcl::PointXYZRGBL,pcl::Normal> ne;
	ne.setInputCloud(cloud_in);
	pcl::search::KdTree<pcl::PointXYZRGBL>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBL> ());
	ne.setSearchMethod(tree);
	ne.setRadiusSearch(0.05);
	ne.compute(*normals);
      }
  //write normals to a file
  if(pcl::io::savePCDFileASCII(FilePath, *normals) == -1)
    {
      PCL_ERROR("save fail!");
    }
  
}

//-----------------------------------------------------------------------------------
//overloaded:
void NormEst(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in,pcl::PointCloud<pcl::Normal>::Ptr normals,bool load,string FilePath)
{
  if(load)
    {
      if(pcl::io::loadPCDFile<pcl::Normal> (FilePath,*normals) == -1)
	{
	  PCL_ERROR("No file to read!");
	  return ;
	}
    }
  else
    {
      if(cloud_in->isOrganized() && cloud_in->width > 1)
	{
	  // Using IntegralImages.
	  cout<<"Estimating Normals using Integral images"<<endl;
	  pcl::IntegralImageNormalEstimation<pcl::PointXYZRGB, pcl::Normal> Ine;
	  Ine.setNormalEstimationMethod(Ine.AVERAGE_3D_GRADIENT);
	  Ine.setMaxDepthChangeFactor(0.02f);
	  Ine.setNormalSmoothingSize(10.0f);
	  Ine.setInputCloud(cloud_in);
	  Ine.compute(*normals);
	  
	}
      else
	{
	  //using normal method
	  //cout<<"Estimating Normals using normal method"<<endl;
	  pcl::NormalEstimation<pcl::PointXYZRGB,pcl::Normal> ne;
	  ne.setInputCloud(cloud_in);
	  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
	  ne.setSearchMethod(tree);
	  ne.setRadiusSearch(0.05);
	  ne.compute(*normals);
	}
      //write normals to a file
      if(pcl::io::savePCDFileASCII(FilePath, *normals) == -1)
	{
	  PCL_ERROR("save fail!");
	}
    }
}

//-----------------------------------------------------------------------------------





//In this function the query point and its neighborhood is given as a pointcloud with
//the world's coordinate system and the detected point as a positive or negative example of
//the object's point( witch would be used as the viewpoint)
//inputs are: BlobIn, which is the blob of query point and its neighbors. VPoint, which is the
// described viewpoint
//output: FVector which is the resulting vector of features. [AngelOfView,AvHeight,VFH]
//------------------------------------------------------------------------------------
void FeatureExtract(pcl::PointCloud<pcl::PointXYZRGBL>::Ptr blob_in,pcl::PointCloud<pcl::Normal>::Ptr blob_norm_in, pcl::PointXYZRGBL opoint, VectorXf * fvector)
{
  //pcl::PointCloud<pcl::Normal>::Ptr Blob_Norm (new pcl::PointCloud<pcl::Normal>);
  pcl::PointCloud<pcl::VFHSignature308>::Ptr vfhs (new pcl::PointCloud<pcl::VFHSignature308> ());
  
  //NormEst(blob_in,Blob_Norm,Lparam);//the boolean shows if it should load from a file or should estimate.
  //---------------------------------------------------------
  //estimating the angel between gravity vector and the vector from veiwpoint to query point (vp_p)
  //The above explanation is for the commented line, now I added code to get centeral norm and
  //compute the angel between this normal and the gravity vector.(9-11-12)
  
  //Vector4f GVector (0,1,0,0);
  Vector3f GVector (0,1,0);
  Vector3f BlobCenter_norm (blob_norm_in->points[1].normal);
  //Vector4f BlobCenter,VP_P;
  //Vector4f vpoint;
  
  //vpoint = opoint.getVector4fMap();
  
  //VP_P = vpoint - BlobCenter;
  //VP_P.normalize();
  
  //double AOfView = (GVector.dot(VP_P) + 1.0) * 0.5;
  float AOfView;
  AOfView = acos(GVector.dot(BlobCenter_norm));
  
  //----------------------------------------------------------------
  //Estimationg the average height of points in the input blob.
  //As the pointcloud is transfered into world's coordinate we can assume points z as their height.
  float AvHeight = 0;	int n_points = (int) blob_in->points.size();
  for(int i = 0; i < n_points; ++i)
    AvHeight = AvHeight + blob_in->points[i].x;
  AvHeight = AvHeight/n_points;
  
  
  // Create the VFH estimation class, and pass the input dataset+normals to it

  pcl::VFHEstimation<pcl::PointXYZRGBL, pcl::Normal, pcl::VFHSignature308> vfh;
  vfh.setInputCloud (blob_in);
  vfh.setInputNormals (blob_norm_in);
  // Create an empty kdtree representation, and pass it to the FPFH estimation object.
  // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
  //pcl::KdTreeFLANN<pcl::PointXYZRGBL>::Ptr tree (new pcl::KdTreeFLANN<pcl::PointXYZRGBL> ());
  pcl::search::KdTree<pcl::PointXYZRGBL>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBL> ());
  vfh.setSearchMethod (tree);
  vfh.compute(*vfhs);
  
  //-----------------------------------------------------------------
  
  //---------------------------------------------------------------------
  
  
  VectorXf temp,temp2(310);
  temp = vfhs->getMatrixXfMap();
  for(int i = 2; i < 310;i++)
    temp2[i] = temp[i-2];
  temp2[0] = AOfView;
  temp2[1] = AvHeight;
  //temp.rows() = 309;
  //cerr<<"temp="<<temp2.rows()<<" "<<temp2;
  *fvector = temp2;
}

//This function gets a point and a pointcloud with labels, look if there is a positive label around
//that point in the pointcloud,
bool LabelLookup(pcl::PointXYZRGBL point,pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_in,float radius)

{
  pcl::search::KdTree<pcl::PointXYZRGBL> tree;
  vector<int>   IdRadiusSearch;
  vector<float> Sdistance;
  bool label = 0;

  tree.setInputCloud(cloud_in);
  if(tree.radiusSearch(point,radius,IdRadiusSearch,Sdistance) > 0)

    {
	  for (size_t i = 0; i < IdRadiusSearch.size(); i++)
		  {
		  //cout<<"i: "<<i<<"index: "<<IdRadiusSearch[i]<<endl;
		  if(cloud_in->points[IdRadiusSearch[i]].label == 1)
		  {
			  label = 1;
			  return(label);
		  }

		  }
      //cout<<"Points count= "<<IdRadiusSearch.size()<<endl;
    }
  else
    cerr<<"Not enough points in the neighborhood!"<<endl;


  return(label);

}

