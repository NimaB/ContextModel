//In this file feature extract is implemented in a way that calss of samples is decided directly based on
//annotaded context.
#include "functions.cpp"
#include "BlobExtract.cpp"

int main(int argc, char **argv)
{//First argument is the choice between train or test
	  //Second input argument is the path of the input pointcloud
	  //Third is the path for the input cloud normals
	  //Fourth is the path of the file to keep interest point list
	  //Fifth is path of the data file to save feature vector in
	  //Sixth  is the path of the data file to save labels of samples.
	  //Seventh  is the path of the TFPcloud

	  //cloud is the input cloud with the point type PointXYZ
	  //pcl::PointCloud<pcl::PointXYZ>::Ptr Cloud (new pcl::PointCloud<pcl::PointXYZ>);
	  //cloud_labeled is the input cloud with the type PointXYZRGBL which also contains labels from annotation
	  pcl::PointCloud<pcl::PointXYZRGBL>::Ptr Cloud_labeled (new pcl::PointCloud<pcl::PointXYZRGBL>);
	  pcl::PointCloud<pcl::PointXYZRGBL>::Ptr Cloud_Filtered (new pcl::PointCloud<pcl::PointXYZRGBL>);
	  pcl::PointCloud<pcl::Normal>::Ptr Cloud_Norm (new pcl::PointCloud<pcl::Normal>);
	  pcl::PointCloud<pcl::PointXYZRGBL>::Ptr Cloud_Blob (new pcl::PointCloud<pcl::PointXYZRGBL>);
	  pcl::PointCloud<pcl::Normal>::Ptr Cloud_Blob_Norm (new pcl::PointCloud<pcl::Normal>);
	  pcl::PointCloud<pcl::PointXYZRGBL>::Ptr Search_Cloud (new pcl::PointCloud<pcl::PointXYZRGBL>);

	  //cloud for Test with Full Points according the min max of the original cloud
	  pcl::PointCloud<pcl::PointXYZRGBL>::Ptr TFPcloud (new pcl::PointCloud<pcl::PointXYZRGBL>);


	  float B_Radius = 0.15;//Radius of the querypoint's blob
	  float Voxel_Radius = (4/3) * B_Radius;//Radius used for voxel downsample.
	  float S_Radius = Voxel_Radius + 0.02;//Radius of the sphere to search object point in. It would depend on the object class
	  //The added Offset is just to make sure that the voxeled point is inside the sphere

	  const char* TofOperation;//to choose train or test.
	  pcl::PCDWriter writer;
	  VectorXf FVector;
	  pcl::PointXYZRGBL QPoint;//query point
	  pcl::PointXYZRGBL OPoint;//object point
	  bool OverWrite;
	  int CofSample = 0;
	  int PosSample = 0;
	  int NegSample = 0;
	  int Noclass = 0;
	  vector<int> InterestPoints;//This is a vector of length 2, which in each iteration keeps the index of the QPoint
	  //and OPoint. InterestPoints = [QPoint,OPoint]

	  //Loading the input pointcloud
	  cout<<"Loading Pointcloud..."<<endl;
	  if ((pcl::io::loadPCDFile(argv[2], *Cloud_labeled)) == -1)
	    {
	      PCL_ERROR("Couldn't read the file \n");
	      return (-1);
	    }
	  cout<<"Number of points in the input cloud_labeled: "<< Cloud_labeled->size()<<endl;

	  //Loading input pointcloud's Normals
	  cout<<"Loading cloud Normals..."<<endl;
	  if (pcl::io::loadPCDFile(argv[3], *Cloud_Norm) == -1)
	    {
	      PCL_ERROR("Couldn't read the file \n");
	      return (-1);
	    }
	  cerr<<"Numer of points in the input cloud_Norm: "<< Cloud_Norm->size()<<endl;


	  InterestPoints.resize(2);
	//----------------------------------------
	  //Set operation type flag

	  if(atoi(argv[1]) == 1)
	    TofOperation = "train";
	  //cout<<"train"<<endl;
	  else
	    if(atoi(argv[1]) == 2)
	      TofOperation = "test";
	  //cout<<"test"<<endl;
	  cout<<"type:"<<TofOperation<<endl;
	 //----------------------------------------
	  /*
	   * This part can be used to kind of crop the point cloud to get a pointcloud focused around
	   * the annotated Object with a given radius ( here we used 0.4)
	  if(TofOperation == "train")
	    {
	      for (size_t i =0; i < Cloud_labeled->points.size() ; i++)
		{
	  	  if(Cloud_labeled->points[i].label == 1)
		    {
		      cerr<<"The center selected index is : "<< i <<endl;
		      BlobExtract(Cloud_labeled->points[i],Cloud_labeled,Cloud_Norm,0.40,Cloud_labeled,Cloud_Norm);
		      if(pcl::io::savePCDFileASCII("Focused.pcd",*Cloud_labeled) == -1)
			{
			  PCL_ERROR("Focused save fail!");
			}
		      break;
		    }
		}
	    }*/
	  //--------------------------------------------------------------
// Create the filtering object: downsample the dataset using a leaf size of 3cm
	  pcl::VoxelGrid<pcl::PointXYZRGBL> sor;
	  sor.setInputCloud (Cloud_labeled);
	  sor.setLeafSize (Voxel_Radius, Voxel_Radius, Voxel_Radius);
	  sor.filter (*Cloud_Filtered);

	  if(pcl::io::savePCDFileASCII("Downsampled.pcd",*Cloud_Filtered) == -1)
	  	 			 {
	  	 				 PCL_ERROR("Cloud2 save fail!");
	  	 			 }

	  cout<<"Numer of points in the input cloud_filtered: "<< Cloud_Filtered->size()<<endl;
	  //---------------------------------------------------------------

	  vector<int> SBlobInd;// this vector will keep the indices of extract blob for search.

	  if(TofOperation == "test")
	  {

		  if (pcl::io::loadPCDFile(argv[7], *TFPcloud) == -1)
		     {
		       PCL_ERROR("Couldn't read the file \n");
		       return (-1);
		     }

	  }

	  //choose query point from the downsampled pointcloud but for the rest of the process we would use the original pointcloud
	  for (size_t i =0; i < Cloud_Filtered->points.size() ; i++)
	    {
	      cout<<"sample number: "<<i<<"out of "<<Cloud_Filtered->points.size()<<endl;
	      QPoint = Cloud_Filtered->points[i];
	      //if((QPoint.label != 1) || (TofOperation == "test")) // here we make sure that query point it self is not a part of an object.
		{
	    	  InterestPoints[0] = (int)i;
		  BlobExtract(QPoint,Cloud_labeled,Cloud_Norm,B_Radius,Cloud_Blob,Cloud_Blob_Norm);
		  if ( Cloud_Blob->size() >= BPnTresh)
		    {
			  if(TofOperation == "train")
			  {
		      SBlobInd = BlobExtract(QPoint,Cloud_Filtered,S_Radius,Search_Cloud);
			  }
			  else
				  if(TofOperation == "test")
				  {
					  SBlobInd = BlobExtract(QPoint,TFPcloud,S_Radius,Search_Cloud);
				  }
		      //cout<<"blob size: "<<Cloud_Blob->points.size()<<endl;
		      //cout<<"search size: "<<Search_Cloud->points.size()<<endl;

		      for(size_t j = 0; j < Search_Cloud->points.size(); ++j)
			{
			  OPoint = Search_Cloud->points[j];
			  //cout<<"object point :"<<j<<OPoint<<endl;
			  InterestPoints[1] = SBlobInd[(int)j];

			  if (TofOperation == "train")
			    {
			      if(QPoint.label == 1)
				{
				  CofSample = 1;
				  PosSample ++;
				}
			      else if(QPoint.label == 255)// the default value for the label is 255
				{
				  CofSample = -1;
				  NegSample ++;
				}
			      else
				{
				  //cerr<<"There is more than two class!!!"<<endl;
				  //cout<<"object point label = "<<OPoint.label<<endl;
				  CofSample = -1;
				  Noclass ++;
				  //return(-1);
				}
			      //cout<<"class of sample is: "<<CofSample<<endl;
			    }

			  FeatureExtract(Cloud_Blob,Cloud_Blob_Norm,OPoint,  &FVector);
			  //cout<<"first element of feature vector is: "<< FVector[0]<<endl;


			  //writing the vector to the file:
			  if ((i ==0) && (j == 0))//just for the first iteration
			    OverWrite = 1;
			  else
			    OverWrite = 0;

			  WtoFile(CofSample,FVector,OverWrite,argv[5],argv[6],DaFiFormat);
			  WtoFile(InterestPoints,OverWrite,argv[4]);

			}
		    }

		  else
		    {
		      cerr<< "This query point is ignored as there is not enough points in its vicinity." <<endl;
		    }
		}


	    }

	  if(TofOperation == "train")
	    cout <<"Positive Samples :"<<PosSample<<endl<<"Negative Samles: " << NegSample << endl<<
	      "No Class : "<< Noclass<<endl;

	  return 0;
	}//end of main
