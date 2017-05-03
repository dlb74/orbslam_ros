
#ifndef TYPEDEFS_H
#define TYPEDEFS_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/ch_graham_andrew.h>
#include <CGAL/convex_hull_2.h>
#include <CGAL/Min_circle_2.h>
#include <CGAL/Min_circle_2_traits_2.h>

typedef pcl::PointXYZ PointT;
typedef pcl::Normal NormalT;
typedef pcl::FPFHSignature33 FeatureT;
typedef Eigen::Matrix4f Matrix;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointCloud<PointT>::Ptr PointCloudPtr;
typedef pcl::PointCloud<PointT>::ConstPtr PointCloudConstPtr;
typedef pcl::PointCloud<NormalT> NormalCloud;
typedef pcl::PointCloud<NormalT>::Ptr NormalCloudPtr;
typedef pcl::PointCloud<NormalT>::ConstPtr NormalCloudConstPtr;
typedef pcl::PointCloud<FeatureT> FeatureCloud;
typedef pcl::PointCloud<FeatureT>::Ptr FeatureCloudPtr;
typedef pcl::PointCloud<FeatureT>::ConstPtr FeatureCloudConstPtr;

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef K::Point_2 Point_2;
typedef std::vector<Point_2> Points;
typedef CGAL::Min_circle_2_traits_2<K>  Traits;
typedef CGAL::Min_circle_2<Traits>      Min_circle;

float scene_ss_ (0.01f);
float max_height (5.00f);
float min_radius = 0.02;
float max_radius = 3.0;

float slice_step = 0.1;
int step = 3;

struct CircleData
{
    cv::Point2f center;
    double radius;
    double height;
};

//struct Tree
//{
//    cv::Point2f center;
//    double radius;
//    double height;
//    int divPart; //this case is divide by 6 parts
//};




#endif // end TYPEDEFS_H
