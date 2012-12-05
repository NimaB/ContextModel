//function BlobExtract
//Inputs: querypoint, input pointcloud, search radius
//Outputs: output pointcloud
using namespace std;

vector<int> BlobExtract(pcl::PointXYZRGBL qpoint,pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_in,float radius,pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_out)

{
  pcl::search::KdTree<pcl::PointXYZRGBL> tree;
  vector<int>   IdRadiusSearch;
  vector<float> Sdistance;

  tree.setInputCloud(cloud_in);
  if(tree.radiusSearch(qpoint,radius,IdRadiusSearch,Sdistance) > 0)
    {
      //cout<<"Points count= "<<IdRadiusSearch.size()<<endl;
    }
  else
    cerr<<"Not enough points in the neighborhood!"<<endl;

  pcl::copyPointCloud(*cloud_in,IdRadiusSearch,*cloud_out);
  return(IdRadiusSearch);

}

//-------------------
//Overloading for extract with point type cloud normals
void BlobExtract(pcl::PointXYZRGBL qpoint,pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_in,pcl::PointCloud<pcl::Normal>::Ptr cloud_norm_in,float radius,pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_out,pcl::PointCloud<pcl::Normal>::Ptr cloud_norm_out)

{
  pcl::search::KdTree<pcl::PointXYZRGBL> tree;
  vector<int>   IdRadiusSearch;
  vector<float> Sdistance;

  tree.setInputCloud(cloud_in);
  if(tree.radiusSearch(qpoint,radius,IdRadiusSearch,Sdistance) > 0)
    {
      //cout<<"Points count= "<<IdRadiusSearch.size()<<endl;
    }
  else
    cerr<<"Not enough points in the neighborhood!"<<endl;

  pcl::copyPointCloud(*cloud_in,IdRadiusSearch,*cloud_out);
  pcl::copyPointCloud(*cloud_norm_in,IdRadiusSearch,*cloud_norm_out);

}


//-----------------------------------------------------------------------------------
void BlobExtract(pcl::PointXYZRGBL objpoint,pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_in,float radius,pcl::PointIndices::Ptr inliers)

{
  pcl::search::KdTree<pcl::PointXYZRGBL> tree;
  vector<float> Sdistance;

  tree.setInputCloud(cloud_in);
  if(tree.radiusSearch(objpoint,radius,inliers->indices,Sdistance) > 0)
    {

    }
  else
    cerr<<"Not enough points in the neighborhood!"<<endl;

}
