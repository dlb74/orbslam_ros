
#include <string>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/passthrough.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/ModelCoefficients.h>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/ch_graham_andrew.h>
#include <CGAL/convex_hull_2.h>
#include <CGAL/Min_circle_2.h>
#include <CGAL/Min_circle_2_traits_2.h>

#include "PCLTypeDefine.h"

using namespace std;

class PCLFilter
{
public:


    void downsample(PointCloud::Ptr cloud_input, PointCloud::Ptr& cloud_output);

    void outlierFilte(PointCloud::Ptr cloud_input, PointCloud::Ptr& cloud_output);

    double computeCloudResolution (const PointCloud::ConstPtr &cloud, int gap);

    void planeDetect(pcl::PointCloud<PointT>::Ptr cloud_input,
                     pcl::ModelCoefficients::Ptr& plane_coefficient);

    void pointCloudSlice(vector< vector < PointCloud::Ptr > >& sliced_cloud,
                         vector<double>& sliceHeights,
                         PointCloud::Ptr cloud,
                         pcl::ModelCoefficients::Ptr plane_coefficient,
                         double resolution,
                         int sliceConst);

    void translateHorizentalX(PointCloud::Ptr cloud_input,
                             PointCloud::Ptr& cloud_output,
                             pcl::ModelCoefficients::Ptr& plane_coefficient);

    void translateHorizentalY(PointCloud::Ptr cloud_input,
                             PointCloud::Ptr& cloud_output,
                             pcl::ModelCoefficients::Ptr& plane_coefficient);


    void form2DSlices(vector< vector < PointCloud::Ptr > > sliced_cloud,
                      vector < PointCloud::Ptr >& clouds, vector<double> sliceHeights, double resolution);

    void clustering2DPoints(PointCloud::Ptr cloud_input, vector<PointCloud::Ptr>& clusters);

    void convexHull(PointCloud::Ptr cluster, PointCloud::Ptr& cluster_hull);

    CircleData findCircleCenterBounding(PointCloud::Ptr linePCL);

    void findCylinder(vector<CircleData> circles, vector< pcl::ModelCoefficients::Ptr>& cylinders);

private:


};


void PCLFilter::downsample(PointCloud::Ptr cloud_input, PointCloud::Ptr& cloud_output)
{

    /**
    *  Downsample Clouds to Extract keypoints
    */
    pcl::UniformSampling<PointT> uniform_sampling;
    uniform_sampling.setInputCloud (cloud_input);
    uniform_sampling.setRadiusSearch (scene_ss_);
    pcl::PointCloud<int> keypointIndices2;
    uniform_sampling.compute(keypointIndices2);
    pcl::copyPointCloud(*cloud_input, keypointIndices2.points, *cloud_output);
    //std::cout << "Scene total points: " << cloud_input->size () << "; Selected Keypoints: " << cloud_output->size () << std::endl;

}


void PCLFilter::outlierFilte(PointCloud::Ptr cloud_input, PointCloud::Ptr& cloud_output)
{
    /**
     * remove noisy measurements, e.g. outliers,
     * from a point cloud dataset using statistical
     * analysis techniques.
     */

    // Create the filtering object
    pcl::StatisticalOutlierRemoval< PointT > sor_outlier;
    sor_outlier.setInputCloud (cloud_input);
    sor_outlier.setMeanK (50);
    sor_outlier.setStddevMulThresh (1.0);
    sor_outlier.filter (*cloud_output);

//    std::cerr << "PointCloud after outlier filtering: " << cloud_output->width * cloud_output->height
//       << " data points"<<endl ;

}

double PCLFilter::computeCloudResolution (const PointCloud::ConstPtr &cloud, int gap = 100)
{
    double res = 0.0;
    int n_points = 0;
    int nres;
    vector<int> indices (2);
    vector<float> sqr_distances (2);
    pcl::KdTreeFLANN<PointT> tree;
    tree.setInputCloud (cloud);

    for (size_t i = 0; i < cloud->size (); i += cloud->size()/gap)
    {
        if (! pcl_isfinite ((*cloud)[i].x))
        {
            continue;
        }
        //Considering the second neighbor since the first is the point itself.
        nres = tree.nearestKSearch (i, 2, indices, sqr_distances);
        if (nres == 2)
        {
            res += sqrt (sqr_distances[1]);
            ++n_points;
        }
    }

    if (n_points != 0)
    {
        res /= n_points;
    }
    return res;
}



void PCLFilter::planeDetect(pcl::PointCloud<PointT>::Ptr cloud_input,
                 pcl::ModelCoefficients::Ptr& plane_coefficient)
{
    // Create the segmentation object for the planar model and set all the parameters
    pcl::SACSegmentation<PointT> seg;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::PointCloud<PointT>::Ptr cloud_plane (new pcl::PointCloud<PointT> ());

    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (1000);
    seg.setDistanceThreshold (0.1);


    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (cloud_input);
    seg.segment (*inliers, *plane_coefficient);



}


bool cloud_passthrough (PointCloud::Ptr cloud, PointCloud::Ptr cloud_out, double slice_height, double thickness)
{

    string axis = "z";
    // Create the filtering object
    pcl::PassThrough<PointT> pass;

    pass.setInputCloud (cloud);
    pass.setFilterFieldName (axis);
    pass.setFilterLimits (slice_height, (slice_height + thickness));
    pass.setFilterLimitsNegative (false);
    pass.filter (*cloud_out);

    //cout<<"cloud_out: "<<cloud_out->points.size()<<endl;

    if (!cloud_out->points.empty())
        return true;
    else
        return false;

}


void PCLFilter::pointCloudSlice(vector< vector < PointCloud::Ptr > >& sliced_cloud,
                     vector<double>& sliceHeights,
                     PointCloud::Ptr cloud,
                     pcl::ModelCoefficients::Ptr plane_coefficient,
                     double resolution,
                     int sliceConst = 3)
{

    double thickness = sliceConst * resolution;
    double currentHeight = -plane_coefficient->values[3] + 3;


    bool sliceScanned = false;

    while(1)
    {

        PointCloud::Ptr cloudUpperSlice(new PointCloud);
        PointCloud::Ptr cloudLowerSlice(new PointCloud);

        //currentHeight += step*thickness;
        //cout<<"eeeeeeeee: "<<currentHeight<<endl;
        if(cloud_passthrough (cloud, cloudUpperSlice, currentHeight, thickness) &&
            cloud_passthrough (cloud, cloudLowerSlice, currentHeight-(thickness), thickness))
        {
            sliceScanned = true;
            vector< PointCloud::Ptr> oneSlice;
            oneSlice.push_back(cloudLowerSlice);
            oneSlice.push_back(cloudUpperSlice);
            sliced_cloud.push_back(oneSlice);
            sliceHeights.push_back(currentHeight);
            oneSlice.clear();
            currentHeight -= sliceConst*thickness;

        }else{

            //cout<<"aaaaaaa: "<<fabs(currentHeight) <<"  "<<sliceScanned<<endl;
            if (sliceScanned || fabs(currentHeight) > max_height)
                break;
            else
                currentHeight -= sliceConst*thickness;
                continue;
        }
    }
}

void PCLFilter::translateHorizentalX(pcl::PointCloud<PointT>::Ptr cloud_input,
                         pcl::PointCloud<PointT>::Ptr& cloud_output,
                         pcl::ModelCoefficients::Ptr& plane_coefficient)
{

    //std::cerr << "Plane coefficients: " << *plane_coefficient << std::endl;

    /**  Using a Affine3f
      This method is easier and less error prone
    */

    float thetax = -atan((plane_coefficient->values[1])/plane_coefficient->values[2]);
    Eigen::Affine3f transform_x = Eigen::Affine3f::Identity();
    // The same rotation matrix as before; theta radians arround X axis
    transform_x.rotate (Eigen::AngleAxisf (thetax, Eigen::Vector3f::UnitX()));
    // You can either apply transform_1 or transform_2; they are the same
    pcl::transformPointCloud (*cloud_input, *cloud_output, transform_x);

//    float thetay = atan(-(plane_coefficient->values[0])/plane_coefficient->values[2]);
//    Eigen::Affine3f transform_y = Eigen::Affine3f::Identity();
//    // The same rotation matrix as before; theta radians arround Y axis
//    transform_y.rotate (Eigen::AngleAxisf (thetay, Eigen::Vector3f::UnitY()));
//    // You can either apply transform_1 or transform_2; they are the same
//    pcl::transformPointCloud (*cloud_output, *cloud_output, transform_y);

//    float thetaz = atan(-(plane_coefficient->values[0])/plane_coefficient->values[1]);
//    Eigen::Affine3f transform_z = Eigen::Affine3f::Identity();
//    // The same rotation matrix as before; theta radians arround Y axis
//    transform_z.rotate (Eigen::AngleAxisf (thetaz, Eigen::Vector3f::UnitZ()));
//    // You can either apply transform_1 or transform_2; they are the same
//    pcl::transformPointCloud (*cloud_output, *cloud_output, transform_z);

}

void PCLFilter::translateHorizentalY(pcl::PointCloud<PointT>::Ptr cloud_input,
                         pcl::PointCloud<PointT>::Ptr& cloud_output,
                         pcl::ModelCoefficients::Ptr& plane_coefficient)
{


    //std::cerr << "Plane coefficients: " << *plane_coefficient << std::endl;

    /**  Using a Affine3f
      This method is easier and less error prone
    */

    float thetay = atan(-(plane_coefficient->values[0])/plane_coefficient->values[2]);
    Eigen::Affine3f transform_y = Eigen::Affine3f::Identity();
    // The same rotation matrix as before; theta radians arround Y axis
    transform_y.rotate (Eigen::AngleAxisf (thetay, Eigen::Vector3f::UnitY()));
    // You can either apply transform_1 or transform_2; they are the same
    pcl::transformPointCloud (*cloud_input, *cloud_output, transform_y);


}


void PCLFilter::form2DSlices(vector< vector < PointCloud::Ptr > > sliced_cloud,
                  vector < PointCloud::Ptr >& clouds, vector<double> sliceHeights, double resolution)
{

        for (size_t i = 0; i < sliced_cloud.size(); i++)
        {


                pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
//        for (int j = 0; j < sliced_cloud[i][0]->points.size(); j++)
//		{
//            pcl::PointXYZ intersectionPoint;
//            pcl::PointXYZ nearestPoint;
//            //find intersection of two points with slice plane
//            if (kdtreeSearchNearest3D (sliced_cloud[i][0]->points[j], sliced_cloud[i][1], nearestPoint, resolution)){
//                findIntersection(sliced_cloud[i][0]->points[j], nearestPoint, sliceHeights[i], intersectionPoint);
//                cloud->points.push_back(intersectionPoint);

////                cout<<"heheda111111: "<<sliced_cloud[i][0]->points[j]<<endl;
////                cout<<"heheda222222: "<<nearestPoint<<endl;
////                cout<<"intersectionPoint: "<<intersectionPoint<<endl;
//            }
//		}
        //cv::Mat image(rows, cols, CV_8U, 255);

        //bubbleSort(cloud, cloud->points.size());

        for (size_t j = 0; j < sliced_cloud[i][0]->points.size(); j++)
        {
            PointT intersectionPoint;
            intersectionPoint.x = sliced_cloud[i][0]->points[j].x;
            intersectionPoint.y = sliced_cloud[i][0]->points[j].y;
            intersectionPoint.z = sliceHeights[i];
//            intersectionPoint.r = 0;
//            intersectionPoint.g = 255;
//            intersectionPoint.b = 0;
            cloud->points.push_back(intersectionPoint);
        }
        for (size_t j = 0; j < sliced_cloud[i][1]->points.size(); j++)
        {
            PointT intersectionPoint;
            intersectionPoint.x = sliced_cloud[i][1]->points[j].x;
            intersectionPoint.y = sliced_cloud[i][1]->points[j].y;
            intersectionPoint.z = sliceHeights[i];
//            intersectionPoint.r = 0;
//            intersectionPoint.g = 255;
//            intersectionPoint.b = 0;
            cloud->points.push_back(intersectionPoint);
        }


                clouds.push_back(cloud);
        }
}



void PCLFilter::clustering2DPoints(pcl::PointCloud<PointT>::Ptr cloud_input, vector<pcl::PointCloud<PointT>::Ptr>& clusters)
{
    /**
    * clustering
    */
    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    tree->setInputCloud (cloud_input);

    vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance (0.04); // 4cm
    ec.setMinClusterSize (50);
    ec.setMaxClusterSize (5000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud_input);
    ec.extract (cluster_indices);


    int j = 0;
    for (vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);
        for (vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
        {

          cloud_cluster->points.push_back (cloud_input->points[*pit]); //*
        }
        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        //std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;

        //std::stringstream ss;
        //ss << "cloud_cluster_" << j << ".pcd";
        //writer.write<PointT> (ss.str (), *cloud_cluster, false);

        if (!cloud_cluster->points.empty())
            clusters.push_back(cloud_cluster);
        j++;
    }
}


void PCLFilter::convexHull(PointCloud::Ptr cluster, PointCloud::Ptr& cluster_hull)
{
    //2D Convex Hulls and Extreme Points
    //cout<<"slice_clusters: "<<cluster->points.size()<<endl;

    Points points, result;
    for(int i = 0; i < cluster->points.size(); i++)
    {
        points.push_back(Point_2(cluster->points[i].x, cluster->points[i].y));
    }

    CGAL::convex_hull_2( points.begin(), points.end(), back_inserter(result) );
    //cout << result.size() << " points on the convex hull" << std::endl;

    for(size_t i = 0; i < result.size(); i++)
    {

        PointT cloudPoint;
        cloudPoint.x = result[i][0];
        cloudPoint.y = result[i][1];
        cloudPoint.z = cluster->points[0].z;

        cluster_hull->points.push_back(cloudPoint);
//        cout<<result[i][0]<<endl;
    }
}

CircleData PCLFilter::findCircleCenterBounding(PointCloud::Ptr linePCL)
{
    Points points;
    for(size_t i = 0; i < linePCL->points.size(); i++)
    {
        points.push_back(Point_2(linePCL->points[i].x, linePCL->points[i].y));
    }

//    points.push_back(Point_2(0,0));
//    points.push_back(Point_2(1,1));
//    points.push_back(Point_2(0,1));
//    points.push_back(Point_2(1,0));

//    Min_circle mc1( &points[0], &points[linePCL->points.size() - 1], false);
    Min_circle mc2( &points[0], &points[linePCL->points.size() - 1], true);

    Traits::Circle c = mc2.circle();
    //CGAL::set_pretty_mode( std::cout);
    //std::cout << mc2 <<endl;

    CircleData CD;

    CD.center.x = c.center()[0];
    CD.center.y = c.center()[1];
    CD.radius = sqrt(c.squared_radius());
    CD.height = linePCL->points[0].z;

    return CD;

}

bool isClosePoint2D(double pt1x, double pt1y, double pt2x, double pt2y, double threshold = 0.4)
{

    if(sqrtf(fabs(pt1x-pt2x)*fabs(pt1x-pt2x)+fabs(pt1y-pt2y)*fabs(pt1y-pt2y)) < threshold)
    {
        //cout<<"heheda: "<<sqrtf(fabs(pt1x-pt2x)*fabs(pt1x-pt2x)+fabs(pt1y-pt2y)*fabs(pt1y-pt2y))<<endl;
        return true;
    }
    return false;
}

bool isCircleGood(CircleData circle, CircleData centers_cylinder)
{
        if (isClosePoint2D(circle.center.x, circle.center.y,
                           centers_cylinder.center.x, centers_cylinder.center.y, 0.4) &&
                circle.radius > min_radius &&
                circle.radius < max_radius &&
                fabs(circle.radius - centers_cylinder.radius) < 0.5)
        {
                return true;
        }else
        {
                return false;
        }
}

bool isTooClosedCircles(CircleData circle_1, CircleData circle_2)
{
    float distance = sqrt((fabs(circle_1.center.x - circle_2.center.x)*fabs(circle_1.center.x - circle_2.center.x))
                          + (fabs(circle_1.center.y - circle_2.center.y)*fabs(circle_1.center.y - circle_2.center.y)));
    if ( distance < circle_1.radius )
    {
        return true;
    }
    else
    {
        return false;
    }
}



bool optimizeCylinder(vector<CircleData> cylinder_input, pcl::ModelCoefficients::Ptr& cylinder_output)
{

//    double x = cylinder_input[0].center.x;
//    double y = cylinder_input[0].center.y;
    double radius = cylinder_input[0].radius;
    double errors = fabs(radius - cylinder_input[1].radius);
    int numSlice = 1;
    double xSum = cylinder_input[0].center.x;
    double ySum = cylinder_input[0].center.y;
    double radiusSum;

    int gap = 0;
    cout<<"-----------------------------------------------"<<endl;

    for (int i = 1; i < cylinder_input.size(); i++)
    {
        cout<<"Height: " <<i<<" "<<cylinder_input[i].center.x<<" "<<cylinder_input[i].center.y<<" "<<cylinder_input[i].height<<endl;

        if (cylinder_input[i].height != cylinder_input[i-1].height)
        {
            if (fabs(radius - cylinder_input[i].radius) < 1.5*errors &&
                    fabs(cylinder_input[i].radius - cylinder_input[i+1].radius) < 1.5*errors)
            {
                xSum=xSum+(cylinder_input[i].center.x);
                ySum=ySum+(cylinder_input[i].center.y);

                radiusSum=radiusSum+(cylinder_input[i].radius);
                //errors = (errors + fabs(radius - cylinder_input[i].radius))/2;
                numSlice++;
            }

            if (fabs(cylinder_input[i].height - cylinder_input[i-1].height) > 0.5)
            {
                gap=3;
            }


        }
    }

    double treeRadius = radiusSum/numSlice;

    if (gap < 3 && numSlice > 5 &&
            treeRadius > min_radius &&
            treeRadius < max_radius )
    {
        cylinder_output->values.push_back( (xSum/numSlice)+0.1 );
        cylinder_output->values.push_back( (ySum/numSlice)+0.2 );
        cylinder_output->values.push_back( cylinder_input[0].height);
        cylinder_output->values.push_back( (xSum/numSlice)+0.1 );
        cylinder_output->values.push_back( (ySum/numSlice)+0.2 );
        cylinder_output->values.push_back( cylinder_input.back().height);
        cylinder_output->values.push_back( treeRadius );
        return true;
    }

    return false;
}

/**
 * @brief findCylinder
 * @param centers
 * @param cylinders
 *
 * find cylinder from slice circles
 */
void PCLFilter::findCylinder(vector<CircleData> circles, vector< pcl::ModelCoefficients::Ptr>& cylinders)
{

    //cout<< "circles: "<<circles.size()<<endl;

    vector< vector<CircleData> > centers_cylinder;

    pcl::PointCloud<PointT>::Ptr temp_centers (new pcl::PointCloud<PointT>());

    for (size_t i = 0; i < circles.size(); i++)
    {

        //temp_centers->points.push_back(center2D);

        bool haveClosePoint = false;

        if (i > 0)
        {
            for (size_t j = 0; j < centers_cylinder.size(); j++)
            {
                for (size_t k = 0; k < centers_cylinder[j].size(); k++)
                {
                    if(isCircleGood(circles[i], centers_cylinder[j][k]))
                    {
                        haveClosePoint = true;
                        centers_cylinder[j].push_back(circles[i]);
                        break;
                    }
                }
            }
        }

//        cout<<"iiiiiii: "<< i<<" "<<haveClosePoint<<endl;

        if (!haveClosePoint)
        {
            vector<CircleData> cylinder_hypo;
            cylinder_hypo.push_back(circles[i]);
            centers_cylinder.push_back(cylinder_hypo);
        }

    }



    for (size_t i = 0; i < centers_cylinder.size(); i++)
    {
        bool hasCircleInside = false;

        if (centers_cylinder[i].size() > 4)
        {
            for (size_t j = 0; j < centers_cylinder.size(); j++)
            {
                if (i > j)
                {
                    if (isTooClosedCircles(centers_cylinder[i][int(centers_cylinder[i].size()/2)],
                                           centers_cylinder[j][int(centers_cylinder[i].size()/2)]))
                    {
                        hasCircleInside = true;
                        break;
                    }
                }
            }


            pcl::ModelCoefficients::Ptr cylinder (new pcl::ModelCoefficients());

            if (!hasCircleInside)
            {
                if (optimizeCylinder(centers_cylinder[i], cylinder))
                {
                    //            cylinder->values[0] = centers_cylinder[i][0].x;
                    //cout<<"hehe: "<<i<<endl;
                    cylinders.push_back(cylinder);
                }
            }

        }
    }

//    cout<< "cylinders: "<<cylinders.size()<<endl;
}



