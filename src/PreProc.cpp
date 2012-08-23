#include "functions.cpp"

//In this function input params are:
//1:input pointcloud type.(1:PointXYZRGB and 2:PointXYZRGBL)
//2:path of the input pointcloud pcd file
//3:Path of transformed pointcloud pcd file
//4:path of cloud normals file pcd file
//5:Path of Test Full Points pointcloud (TFP_cloud.pcd)
//6:path to the copy of pointcloud in XYZRGBL type
int main(int argc, char **argv)

{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZRGBL>);

	float a;
	a= 1.21;// for test(will be removed later)

if (atoi(argv[1]) == 1)
{
	cout<<"input cloud is RGB"<<endl;
	 if (pcl::io::loadPCDFile(argv[2], *cloud) == -1)
	    {
	      PCL_ERROR("Couldn't read the file \n");
	      return (-1);
	    }
	 //--------------------------------------------------------------------------
	 	 //Transfering the pointcloud to the world's coordinate system (zero would be on the floor)

	 	  //a = SHEstimate(cloud);

	 	  cout<<"sensor height estimated to be: "<< a<<endl;

	 	  pcl::transformPointCloud(*cloud,*cloud,pcl::getTransformation(a,0,0,ALFA,BETHA,GAMA));

	 	  cerr<<"Number of points in the input cloud after transfer: "<< cloud->size()<<endl;

	 	 cout<<"number of points are: "<< cloud->height<<"*"<<cloud->width<<
	 			 "cloud is organized:" <<cloud->isOrganized()<<endl;
	 			 //"sensor orientation :" <<cloud->sensor_orientation_<<endl;

	 	 		 if(pcl::io::savePCDFileASCII(argv[3],*cloud) == -1)
	 	 		 {
	 	 			 PCL_ERROR("Cloud2 save fail!");
	 	 		 }
	 //--------------------------------------------------

	 	 NormEst(cloud,cloud_normals,Lparam,argv[4]);
	 	 //cout<<"Normals: "<<cloud_normals->height<<endl;

	}
else
if (atoi(argv[1]) == 2)//This is when the input is of type RGBL
 	{
	cout<<"input cloud is RGBL"<<endl;
		 pcl::io::loadPCDFile(argv[2], *cloud2);
	 		//--------------------------------------------------------------------------
	 			 //Transfering the pointcloud to the world's coordinate system (zero would be on the floor

		 	 //a = SHEstimate(cloud2);

	 			  cout<<"sensor height estimated to be: "<< a<<endl;

	 			  pcl::transformPointCloud(*cloud2,*cloud2,pcl::getTransformation(a,0,0,ALFA,BETHA,GAMA));

	 			  cerr<<"Number of points in the input cloud after transfer: "<< cloud2->size()<<endl;

	 			 cout<<"number of points are: "<< cloud2->height<<"*"<<cloud2->width<<
	 					 "cloud is organized:" <<cloud2->isOrganized()<<endl;
	 					 //"sensor orientation :" <<cloud->sensor_orientation_<<endl;

	 			 		 if(pcl::io::savePCDFileASCII(argv[3],*cloud2) == -1)
	 			 		 {
	 			 			 PCL_ERROR("Cloud2 save fail!");
	 			 		 }
	 		//--------------------------------------------------

	 			 NormEst(cloud2,cloud_normals,Lparam,argv[4]);
	 			 //cout<<"Normals: "<<cloud_normals->height<<endl;


}


//--------------------------------------------------
	 //This part is for generating TestCloud

		 Eigen::Vector4f Min_pt;
		 Eigen::Vector4f Max_pt;

		 if(atoi(argv[1]) == 1)
		 pcl::getMinMax3D(*cloud,Min_pt,Max_pt);
		 else
			 if(atoi(argv[1]) == 2)
				 pcl::getMinMax3D(*cloud2,Min_pt,Max_pt);


		 pcl::PointCloud<pcl::PointXYZRGBL>::Ptr TFP_cloud(new pcl::PointCloud<pcl::PointXYZRGBL>);
		 float minx = ((float)Min_pt(0));
		 float maxx = ((float)Max_pt(0));
		 float miny = ((float)Min_pt(1));
		 float maxy = ((float)Max_pt(1));
		 float minz = ((float)Min_pt(2));
		 float maxz = ((float)Max_pt(2));
		 cout<<"minx: "<<minx<<" "<<
				 "maxx: "<<maxx<<" "<<
				 "miny: "<<miny<<" "<<
				 "maxy: "<<maxy<<" "<<
				 "minz: "<<minz<<" "<<
				 "maxz: "<<maxz<<" "<<endl;
		 CloudGenerate(minx,maxx,miny,maxy,minz,maxz,0.05,TFP_cloud);

		 cout<<"cloud Size: "<<TFP_cloud->points.size();
		 if(pcl::io::savePCDFileASCII(argv[5],*TFP_cloud) == -1)
		 	 			 {
		 	 				 PCL_ERROR("Cloud2 save fail!");
		 	 			 }
	//-------------------------------------------------



	 if ((argc == 7) && (atoi(argv[1]) == 1))//This condition makes sure that the input pointcloud is RGB type
		                                     //Andd a path for RGBL version is provided.
	 {
		 pcl::copyPointCloud(*cloud,*cloud2);
		 if(pcl::io::savePCDFileASCII(argv[6],*cloud2) == -1)
		 {
			 PCL_ERROR("Cloud2 save fail!");
		 }
	 }


	 /*DownSample(cloud,cloud);
	 if(pcl::io::savePCDFileASCII("test.pcd",*cloud) == -1)
	 			 {
	 				 PCL_ERROR("Cloud2 save fail!");
	 			 }*/

//----------------------------------------------------

}
