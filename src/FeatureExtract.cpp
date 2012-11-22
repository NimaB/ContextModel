#include "functions.cpp"
#include "BlobExtract.cpp"



int main(int argc, char **argv)
{
  //First argument is the choice between train or test
  //Second input argument is the path of the input pointcloud
  //Third is the path for the input cloud normals
  //Fourth is the path of the file to keep interest point list
  //Fifth is path of the data file to save feature vector in
  //Sixth  is the path of the data file to save labels of samples.
  //Seventh is the path of FullTrainDataset
  //eighth is the path of FullTrainDataset labels
  //REMOVED:ninth   is the path of the TFPcloud
	//Ninth would be the path for cloud filtered 2
  
  //cloud is the input cloud with the point type PointXYZ
  //pcl::PointCloud<pcl::PointXYZ>::Ptr Cloud (new pcl::PointCloud<pcl::PointXYZ>);
  //cloud_labeled is the input cloud with the type PointXYZRGBL which also contains labels from annotation
  pcl::PointCloud<pcl::PointXYZRGBL>::Ptr Cloud_labeled (new pcl::PointCloud<pcl::PointXYZRGBL>);
  pcl::PointCloud<pcl::PointXYZRGBL>::Ptr Cloud_Filtered (new pcl::PointCloud<pcl::PointXYZRGBL>);//this one is for qpoint
  pcl::PointCloud<pcl::PointXYZRGBL>::Ptr Cloud_Filtered2 (new pcl::PointCloud<pcl::PointXYZRGBL>);//This one is for opoints
  pcl::PointCloud<pcl::Normal>::Ptr Cloud_Norm (new pcl::PointCloud<pcl::Normal>);
  pcl::PointCloud<pcl::PointXYZRGBL>::Ptr Cloud_Blob (new pcl::PointCloud<pcl::PointXYZRGBL>);
  pcl::PointCloud<pcl::Normal>::Ptr Cloud_Blob_Norm (new pcl::PointCloud<pcl::Normal>);
  pcl::PointCloud<pcl::PointXYZRGBL>::Ptr Search_Cloud (new pcl::PointCloud<pcl::PointXYZRGBL>);
  
  //cloud for Test with Full Points according the min max of the original cloud
  //pcl::PointCloud<pcl::PointXYZRGBL>::Ptr TFPcloud (new pcl::PointCloud<pcl::PointXYZRGBL>);
  
  
  float B_Radius = 0.15;//Radius of the querypoint's blob
  float Voxel_Radius = (4/3) * B_Radius;//Radius used for voxel downsample.
  float Voxel_Radius2 = B_Radius;
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
  if ((pcl::io::loadPCDFile(argv[2], *Cloud_labeled)) == -1)
    {
      PCL_ERROR("Couldn't read the file \n");
      return (-1);
    }
  cerr<<"Number of points in the input cloud_labeled: "<< Cloud_labeled->size()<<endl;
  
  //Loading input pointcloud's Normals
  if (pcl::io::loadPCDFile(argv[3], *Cloud_Norm) == -1)
    {
      PCL_ERROR("Couldn't read the file \n");
      return (-1);
    }
  cerr<<"Number of points in the input cloud_Norm: "<< Cloud_Norm->size()<<endl;
  
  
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
  /*Focuses the pointcloud to surrounding the object
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
  //sor.setLeafSize (0.03f, 0.03f, 0.03f);
  sor.setLeafSize (Voxel_Radius, Voxel_Radius, Voxel_Radius);
  sor.filter (*Cloud_Filtered);
  
  sor.setLeafSize (Voxel_Radius2, Voxel_Radius2, Voxel_Radius2);
  sor.filter(*Cloud_Filtered2);

  //In case the point of interest for us is OPoint we save Cloud_Filtered2, if it is QPoint then
    // Cloud_Filtered should be saved for later result visualization.
  if(pcl::io::savePCDFileASCII(argv[9],*Cloud_Filtered2) == -1)
    {
      PCL_ERROR("Cloud2 save fail!");
    }
  
  cerr<<"Number of points in the input cloud_filtered: "<< Cloud_Filtered2->size()<<endl;
  //---------------------------------------------------------------
  
  vector<int> SBlobInd;// this vector will keep the indices of extract blob for search.
  
//  if(TofOperation == "test")
//    {
//
//      if (pcl::io::loadPCDFile(argv[9], *TFPcloud) == -1)
//	{
//	  PCL_ERROR("Couldn't read the file \n");
//	  return (-1);
//	}
//
//    }
  
  //choose query point from the downsampled pointcloud but for the rest of the process we would use the original pointcloud
  for (size_t i =0; i < Cloud_Filtered->points.size() ; i++)
    {
      cerr<<"sample number: "<<i<<"out of "<<Cloud_Filtered->points.size()<<endl;
      QPoint = Cloud_Filtered->points[i];
      if((QPoint.label != 1) || (TofOperation == "test")) // here we make sure that query point it self is not a part of an object.
	{
    	  InterestPoints[0] = (int)i;
	  BlobExtract(QPoint,Cloud_labeled,Cloud_Norm,B_Radius,Cloud_Blob,Cloud_Blob_Norm);
	  if ( Cloud_Blob->size() >= BPnTresh)
	    {
	      //if(TofOperation == "train")
		//{
		  SBlobInd = BlobExtract(QPoint,Cloud_Filtered2,S_Radius,Search_Cloud);
		//}
	      //else
		//if(TofOperation == "test")
		  //{
		    //SBlobInd = BlobExtract(QPoint,TFPcloud,S_Radius,Search_Cloud);
		  //}
	      //cout<<"blob size: "<<Cloud_Blob->points.size()<<endl;
	      //cout<<"search size: "<<Search_Cloud->points.size()<<endl;
	      
	      for(size_t j = 0; j < Search_Cloud->points.size(); ++j)
		{
		  OPoint = Search_Cloud->points[j];
		  //cout<<"object point :"<<j<<OPoint<<endl;
		  InterestPoints[1] = SBlobInd[(int)j];
		  
		  if (TofOperation == "train")
		    {
		      if(OPoint.label == 1)
			{
			  CofSample = 1;
			  PosSample ++;
			}
		      else if(OPoint.label == 255)// the default value for the label is 255
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
		  
		  //Here we are saving a train dataset with samples from all clouds for one object class.
		  if (argc>8)
		  WtoFile(CofSample,FVector,0,argv[7],argv[8],DaFiFormat);
		  else
			  cerr<<"Full train dataset is not being saved! No path for it is provided.*argc="<<argc<<endl;
		}
	    }
	  
	  else
	    {
	      cerr<< "This query point is ignored as there is not enough points in its vicinity." <<endl;
	    }
	}
      
      
    }
  
  if(TofOperation == "train")
    cout <<"Positive Samples :"<<PosSample<<endl<<"Negative Samples: " << NegSample << endl<<
      "No Class : "<< Noclass<<endl;
  
  return 0;
}//end of main

