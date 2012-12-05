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

//Cloud Declarations:-----------------------------------------
  //cloud is the input cloud with the point type PointXYZ
  //cloud_labeled is the input cloud with the type PointXYZRGBL which also contains labels from annotation
  pcl::PointCloud<pcl::PointXYZRGBL>::Ptr Cloud_labeled (new pcl::PointCloud<pcl::PointXYZRGBL>);
  pcl::PointCloud<pcl::PointXYZRGBL>::Ptr Cloud_Filtered (new pcl::PointCloud<pcl::PointXYZRGBL>);//this one is for qpoint
  pcl::PointCloud<pcl::Normal>::Ptr Cloud_Norm (new pcl::PointCloud<pcl::Normal>);
  pcl::PointCloud<pcl::PointXYZRGBL>::Ptr Cloud_Blob (new pcl::PointCloud<pcl::PointXYZRGBL>);
  pcl::PointCloud<pcl::Normal>::Ptr Cloud_Blob_Norm (new pcl::PointCloud<pcl::Normal>);
  pcl::PointCloud<pcl::PointXYZRGBL>::Ptr Search_Cloud (new pcl::PointCloud<pcl::PointXYZRGBL>);
//-------------------------------------------------------------
  

#define B_Radius  0.15         //Radius of the querypoint's blob
#define OPoint_Radius 0.05     //Radius for a sphere around each generated OPoints to look for
                               //Actual annotated object points.
float   ObjRadius    = 0.15;        // Approximate radius for object, here for Trash bin.
float   Voxel_Radius = (4/3) * ObjRadius;//Radius used for voxel downsample.
float   S_Radius     = 2 * ObjRadius;// Sphere around each quary point for generating candidate OPoints.
int     NumofLayers  = 2;           // Number of layers of points for generated sphere.
float   ObjNeighbourRadius = S_Radius+ObjRadius;

  const char* TofOperation;//to choose train or test.
  pcl::PCDWriter writer;
  VectorXf FVector;
  pcl::PointXYZRGBL QPoint;//query point
  pcl::PointXYZRGBL OPoint;//object point
  pcl::PointXYZRGBL ObjCenter;
  bool OverWrite;
  bool InNeighborhood;//Aflag to show if Qpoint is in neighborhood of the object
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
  
  //--------------------------------------------------------------
  // Create the filtering object: downsample the dataset using a leaf size of 3cm
  cout<<"Downsampling pointcloud with radius= "<<Voxel_Radius<<"..."<<endl;

  pcl::VoxelGrid<pcl::PointXYZRGBL> sor;
  sor.setInputCloud (Cloud_labeled);
  //sor.setLeafSize (0.03f, 0.03f, 0.03f);
  sor.setLeafSize (Voxel_Radius, Voxel_Radius, Voxel_Radius);
  sor.filter (*Cloud_Filtered);
  
  //In case the point of interest for us is OPoint we save Cloud_Filtered2, if it is QPoint then
  // Cloud_Filtered should be saved for later result visualization.
  cout<<"Saving Downsampled pointcloud... "<<endl;
  if(pcl::io::savePCDFileASCII(argv[9],*Cloud_Filtered) == -1)
    {
      PCL_ERROR("Cloud2 save fail!");
    }
  
  cerr<<"Number of points in the input cloud_filtered: "<< Cloud_Filtered->size()<<endl;
  //---------------------------------------------------------------
  
  vector<int> SBlobInd;// this vector will keep the indices of extract blob for search.
  
  //Generating a template for Search cloud,which would be translated based on
     //the given center(QPoints) in the loop.
  CloudGenerate(S_Radius,NumofLayers,Search_Cloud);
 
  if (TofOperation == "train")
  {
	  cout<<"Looking for annotated Object center..."<<endl;
	  ObjCenter = GetObjCenter(Cloud_labeled,1);
  }
  //choose query point from the downsampled pointcloud but for the rest of the process we would use the original pointcloud
  for (size_t i =0; i < Cloud_Filtered->points.size() ; i++)
    {
      cerr<<"Query point number: "<<i<<"out of "<<Cloud_Filtered->points.size()<<endl;
      QPoint = Cloud_Filtered->points[i];
      InNeighborhood = 0;
      
      if((QPoint.label != 1) || (TofOperation == "test")) // here we make sure that query point it self is not a part of an object.
	{
    	  InterestPoints[0] = (int)i;
	  BlobExtract(QPoint,Cloud_labeled,Cloud_Norm,B_Radius,Cloud_Blob,Cloud_Blob_Norm);

	  if ( Cloud_Blob->size() >= BPnTresh)
	    {
		  if (TofOperation == "train")
		  {
			  if (pcl::euclideanDistance(ObjCenter,QPoint) <= ObjNeighbourRadius)
				  InNeighborhood = 1;
			  	  cout<<"QPoint is in neighborhood of object!"<<endl;
		  }
		  
	      for(size_t j = 0; j < Search_Cloud->points.size(); ++j)
		{
	    	  //Does the translation
		  OPoint.x = Search_Cloud->points[j].x + QPoint.x;
		  OPoint.y = Search_Cloud->points[j].y + QPoint.y;
		  OPoint.z = Search_Cloud->points[j].z + QPoint.z;

		  InterestPoints[1] = j;
		  cout<<"Generated OPoint number: "<<j<<endl;
		  
		  if (TofOperation == "train")
		    {

			  if (InNeighborhood)
			  {
				  if(LabelLookup(OPoint,Cloud_labeled,OPoint_Radius) == 1)
				  {
					  CofSample = 1;
					  PosSample ++;
				  }
				  else
				  {
					  CofSample = -1;
					  NegSample ++;
				  }
			  }
			  else
			  	{
			  	  CofSample = -1;
			  	  NegSample ++;
			  	}
			  
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

