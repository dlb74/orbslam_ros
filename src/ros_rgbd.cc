/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <opencv2/core/core.hpp>
#include <boost/thread/thread.hpp>
//#include "pcl_visualizer.h"
#include <pcl/visualization/cloud_viewer.h>
#include <thread>


#include "include/System.h"
#include "include/pointcloudmapping.h"
#include "PCLTypeDefine.h"
#include "PCLFilter.h"
#include "include/Tree.h"


bool treeDetect = false;
bool stopPrint = false;


using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM)
    {
        rotationState = 0;
        one = false;
        two = false;
        three = false;
        four = false;
        five = false;
        six = false;

    }

    void GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD);

    void TrackKeyFrame();

    int PoseGauss(vector<KeyFrame*> keyFrames);

    void multipleTest(PointCloud::Ptr cloud);

    ORB_SLAM2::System* mpSLAM;
//    bool stopPrint;

private:

    PCLFilter filter;

    cv::Mat initPose;

    int preSize;

    int rotationState;
    
    bool one;
    bool two;
    bool three;
    bool four;
    bool five;
    bool six;

//    bool treeDetect;

    std::vector<ORB_SLAM2::Tree> Trees;

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

    void resetView();

//    void detectTrees(PointCloud::Ptr cloud_in, PointCloud::Ptr& cloud_out);
    void detectTrees(PointCloud::Ptr cloud_in);



};

boost::shared_ptr<pcl::visualization::PCLVisualizer> simpleVis (PointCloud::ConstPtr cloud)
{
    // --------------------------------------------
    // -----Open 3D viewer and add point cloud-----
    // --------------------------------------------
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("FinalViewer"));
    viewer->setBackgroundColor (0, 0, 0);
//    pcl::visualization::PointCloudColorHandlerCustom<PointT> single_color(cloud, 0, 255, 0);
    viewer->addPointCloud<PointT> (cloud, "sample cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");

//    for (int i = 0; i < slicedCloud3d.size(); i++)
//    {
//        //std::cerr << "pointSize: "<< slicedCloud3d[i]->points.size()<< std::endl;
//        stringstream ss;
//        ss << i;
//        viewer->addPointCloud<PointT> (slicedCloud3d[i], ss.str());
//        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, ss.str());
//    }
    viewer->addSphere (cloud->points[0], 0.2, 0.5, 0.5, 0.0, "sphere");

    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();



  return (viewer);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "RGBD");
    ros::start();



    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 RGBD path_to_vocabulary path_to_settings" << endl;        
        ros::shutdown();
        return 1;
    }    

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::RGBD,true);

    ImageGrabber igb(&SLAM);

    ros::NodeHandle nh;

    //message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/camera/rgb/image_raw", 1);
    //message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "/camera/depth_registered/sw_registered/image_rect_raw", 1);
    
    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/camera/rgb/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "/camera/depth_registered/sw_registered/image_rect_raw", 1);
    
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub,depth_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD,&igb,_1,_2));

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
    cout<<"SaveKeyFrameTrajectoryTUM"<<endl;

    ros::shutdown();


    return 0;
}

void ImageGrabber::GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrD;
    try
    {
        cv_ptrD = cv_bridge::toCvShare(msgD);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    if (!treeDetect)
        TrackKeyFrame();

//    cout<<"treeDetect: "<<treeDetect<<endl;

    if (!stopPrint)
    {
//        cout<<"stopPrint: false "<<stopPrint<<endl;
        mpSLAM->TrackRGBDTrees(cv_ptrRGB->image,cv_ptrD->image,cv_ptrRGB->header.stamp.toSec(), Trees);
    }else
    {
//        cout<<"stopPrint: true "<<stopPrint<<endl;
        mpSLAM->TrackRGBDTrees(cv_ptrRGB->image,cv_ptrD->image,cv_ptrRGB->header.stamp.toSec(), Trees);
    }



    //cout<<"Trees.size():  "<< Trees.size() << endl;

//    cv::Mat imdepth;
//    cv_ptrRGB->image.copyTo(imdepth);

//    cv::rectangle(imdepth, cv::Point(100, 100), cv::Point(200, 200), cv::Scalar(0, 0, 255), 3, 4, 0);

//    cv::imshow("Current Depth Frame",imdepth);
    //cv::waitKey(0);


}


void ImageGrabber::TrackKeyFrame()
{

    vector<KeyFrame*> keyFrames = mpSLAM->GetPointCloudMapping()->GetKeyFrames();
    int frameSize = keyFrames.size();

    if (frameSize > 0)
    {
        if (frameSize == 1)
        {
            initPose = keyFrames[0]->GetPose();
        }

//        if (frameSize != preSize)
//        //if (frameSize != preSize && frameSize%10 == 0)
//        {
            PoseGauss(keyFrames);

//        }

        preSize = frameSize;
    }

}

void ImageGrabber::resetView()
{
    one = false;
    two = false;
    three = false;
    four = false;
    five = false;
    six = false;
}

int cameraRotationAngle(double theta, cv::Mat currentPose)
{

    if (theta > -10 && theta < 10 && currentPose.at<float>(2,2) > 0)
    {
//        one = true;
        return 1;
    }
    if (theta > -70 && theta < -50 && currentPose.at<float>(2,2) > 0)
    {
        //two = true;
        return 2;
    }
    if (theta > -70 && theta < -50 && currentPose.at<float>(2,2) < 0)
    {
        //three = true;
        return 3;
    }
    if (theta > -10 && theta < 10 && currentPose.at<float>(2,2) < 0)
    {
        //four = true;
        return 4;
    }
    if (theta > 50 && theta < 70 && currentPose.at<float>(2,2) < 0)
    {
        //five = true;
        return 5;
    }
    if (theta > 50 && theta < 70 && currentPose.at<float>(2,2) > 0)
    {
        //six = true;
        return 6;
    }
}

void ImageGrabber::multipleTest(PointCloud::Ptr cloud)
{
    cout<<"ssssss: "<<endl;
}

int ImageGrabber::PoseGauss(vector<KeyFrame*> keyFrames)
{

    cv::Mat jac;

    cv::Mat currentPose = keyFrames[keyFrames.size()-1]->GetRotation();

    const double PI = 3.1415926;

    double thetaz = atan2(currentPose.at<float>(1,0), currentPose.at<float>(0,0)) / PI * 180;

    double thetay = atan2(-1 * currentPose.at<float>(2,0), sqrt(currentPose.at<float>(2,1)*currentPose.at<float>(2,1)
                                                                + currentPose.at<float>(2,2)*currentPose.at<float>(2,2))) / PI * 180;

//    cout<<"thetay: "<<thetay<<endl;
    double thetax = atan2(currentPose.at<float>(2,1), currentPose.at<float>(2,2)) / PI * 180;

    //jac = keyFrames[keyFrames.size()-1]->GetPose() - initPose;




    if (!stopPrint)
    {
//        cout<<"key frame: "<<keyFrames.size()<<endl;

//        cout//<<"~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n KeyFrame: \n"
//            //<<keyFrames[keyFrames.size()-1]->GetPose()
//            //<<"\n Rotation: \n"
//            //<<currentPose
//            //<<"\n thetaz: \n"
//            //<<thetaz
//            <<"\n thetay: \n"
//            <<thetay
//            <<"\n states: \n"
//            <<one<<" "<<two<<" "<<three<<" "<<four<<" "<<five<<" "<<six<<" "

//            <<"\n currentPose.at<float>(2,2): \n"
//            <<currentPose.at<float>(2,2)

//            //<<"\n thetax: \n"
//            //<<thetax
//            <<"\n~~~~~~~~~~~~~~~~~~~~~~~~~~~~ " <<endl;
    }


    
    
    switch (cameraRotationAngle( thetay, currentPose))
    {
        case 2:
        {
            two = true;
            break;
        }
        case 3:
        {
            three = true;
            break;
        }
        case 4:
        {
            four = true;
            break;
        }
        case 5:
        {
            five = true;
            break;
        }
        case 6:
        {
            six = true;
            break;
        }
    }



\
        

    if (two == true && three == true && four == true && five == true && six == true)
    //if (two == true && six == true)
    {
        if (cameraRotationAngle( thetay, currentPose) == 1)
        {
            one = true;
            
            cout << "Got 360 degree cloud map." << endl;
        
            treeDetect = true;

        }

        //PointCloud::Ptr cloud = mpSLAM->GetPointCloudMapping()->GetGlobalMap();
        //cout<<"size: "<<cloud.size()<<endl;
        //treeDetect = true;
        
    }
    


    if (treeDetect && !stopPrint)
    {

//        cout << "Tree detection algorithm begining..... " << cloud->points.size() <<"\n"
//            << mpSLAM->GetLoopCloser()->isFinishedGBA()
//            << "\nisFinished\n"
//            << mpSLAM->GetLoopCloser()->isRunningGBA()
//            <<endl;

        if (mpSLAM->GetLoopCloser()->isFinishedGBA() == true
            && mpSLAM->GetLoopCloser()->isRunningGBA() == false)
        {
            PointCloud::Ptr cloud = mpSLAM->GetPointCloudMapping()->GetGlobalMap();

            PointCloud::Ptr cloud_output (new PointCloud());

            thread myTask(&ImageGrabber::detectTrees, this, cloud);
            if (myTask.joinable())
                myTask.join();

            //detectTrees(cloud, cloud_output);


            if (!cloud_output->points.empty())
            {
                cout<<"cloud_output->points.empty() aaaaaaaaaa"<<endl;
                treeDetect = false;
                stopPrint = true;
                resetView();

            }
        }
    }

    return 0;
}

int findPart(cv::Point2f treeCenter)
{
//    cout<<"atan(treeCenter.y/treeCenter.x): "<<atan(treeCenter.y/treeCenter.x)<<endl;
//    if (atan(treeCenter.y/treeCenter.x))
    //part 1
    if (treeCenter.x >= 0 && treeCenter.y >= 0 && atan(treeCenter.y/treeCenter.x) > 1.047)
    {
        return 1;
    }
    if (treeCenter.x < 0 && treeCenter.y > 0 && atan(treeCenter.y/treeCenter.x) < -1.047)
    {
        return 1;
    }
    //part 2
    if (treeCenter.x > 0 && treeCenter.y > 0 && atan(treeCenter.y/treeCenter.x) < 1.047 && atan(treeCenter.y/treeCenter.x) > 0)
    {
        return 2;
    }
    //part 3
    if (treeCenter.x > 0 && treeCenter.y < 0 && atan(treeCenter.y/treeCenter.x) > -1.047 && atan(treeCenter.y/treeCenter.x) < 0)
    {
        return 3;
    }
    //part 4
    if (treeCenter.x > 0 && treeCenter.y < 0 && atan(treeCenter.y/treeCenter.x) < -1.047)
    {
        return 4;
    }
    if (treeCenter.x < 0 && treeCenter.y < 0 && atan(treeCenter.y/treeCenter.x) > 1.047)
    {
        return 4;
    }
    //part 5
    if (treeCenter.x < 0 && treeCenter.y < 0 && atan(treeCenter.y/treeCenter.x) < 1.047 && atan(treeCenter.y/treeCenter.x) > 0)
    {
        return 5;
    }
    //part 6
    if (treeCenter.x < 0 && treeCenter.y > 0 && atan(treeCenter.y/treeCenter.x) > -1.047 && atan(treeCenter.y/treeCenter.x) < 0)
    {
        return 6;
    }


}

//void ImageGrabber::detectTrees(PointCloud::Ptr cloud_in, PointCloud::Ptr& cloud_out)
void ImageGrabber::detectTrees(PointCloud::Ptr cloud_in)
{

    pcl::PCDWriter writer;
    
    PointCloud::Ptr cloud_downsampled (new PointCloud());
    PointCloud::Ptr cloud_outfilter (new PointCloud());
    PointCloud::Ptr cloud_translated_x (new PointCloud());
    PointCloud::Ptr cloud_translated_y (new PointCloud());

    filter.downsample(cloud_in, cloud_downsampled);
    filter.outlierFilte(cloud_downsampled, cloud_outfilter);


    pcl::ModelCoefficients::Ptr plane_coefficient (new pcl::ModelCoefficients);

    filter.planeDetect(cloud_outfilter, plane_coefficient);
    filter.translateHorizentalX(cloud_outfilter, cloud_translated_x, plane_coefficient);
//    filter.planeDetect(cloud_outfilter, plane_coefficient);
//    filter.translateHorizentalY(cloud_outfilter, cloud_translated_y, plane_coefficient);

    double resolution = filter.computeCloudResolution(cloud_translated_x);

    cout<<"resolution: "<< resolution << endl;

    vector< vector < PointCloud::Ptr > > sliced_cloud;
    vector < PointCloud::Ptr > slicedCloud2d;
    vector<double> sliceHeights;

    //cout<<"plane_coefficient: "<< plane_coefficient->values[0] << endl;
    
    filter.pointCloudSlice(sliced_cloud, sliceHeights, cloud_translated_x, plane_coefficient, resolution, step);

    //from 3D to 2D
    filter.form2DSlices(sliced_cloud, slicedCloud2d, sliceHeights, resolution);
    vector<CircleData> circles;

    for (size_t i = 0; i < slicedCloud2d.size(); i++)
    {
        vector<PointCloud::Ptr> slice_clusters;

        filter.clustering2DPoints(slicedCloud2d[i], slice_clusters);
        for (size_t j = 0; j < slice_clusters.size(); j++)
        {
            PointCloud::Ptr cluster_hull (new (pcl::PointCloud<PointT>));
            filter.convexHull(slice_clusters[j], cluster_hull);

            CircleData circleData;
            circleData = filter.findCircleCenterBounding(cluster_hull);
            circles.push_back(circleData);
        }
    }

    vector< pcl::ModelCoefficients::Ptr> cylinders;

    filter.findCylinder(circles, cylinders);
    //cout<< "cylinders_size: "<<cylinders.size()<<endl;

    for (size_t i = 0; i < cylinders.size(); i++)
    {
//        cout<< "tree_ "<< i <<" _: \nx: "<< cylinders[i]->values[0] <<
//           " \ny: " << cylinders[i]->values[1] <<
//           "\nTree Height (visiable area): " << fabs(cylinders[i]->values[2] - cylinders[i]->values[5])<<
//           "\nTree radius: "<< cylinders[i]->values[6] <<endl;
        ORB_SLAM2::Tree tree;
        tree.center.x = cylinders[i]->values[0];
        tree.center.y = cylinders[i]->values[1];
        tree.height = fabs(cylinders[i]->values[2] - cylinders[i]->values[5]);
        tree.radius = cylinders[i]->values[6];

        tree.divPart = findPart(tree.center);

        Trees.push_back(tree);
        //cout<<"Trees.divPart:  "<< tree.divPart << endl;
    }

//    cout<<"Trees.divPart:  "<< Trees[Trees.size()-1].divPart << endl;

    for (size_t i = 0; i < slicedCloud2d.size(); i++)
    {

        //*cloud_out += *slicedCloud2d[i];
//        cout<<"cloud__"<<i<<"__: "<< slicedCloud2d[i]->points.size() << endl;

//        for (int j = 0; j < slicedCloud2d[i]->points.size(); j++)
//        {
//            cout<<"cloud__"<<i<<"__"<<j<<"__: "<< slicedCloud2d[i]->points[j]<<endl;
//        }

    }

    //pcl::visualization::CloudViewer viewer("FinalViewer");
    //viewer.showCloud( cloud_out );
//    cout<<"cloud_out333333333 size: "<< cloud_out->points.size() << endl;

    writer.write ("tree1.pcd", *cloud_outfilter, false);
    cout<<"saved cloud"<<endl;

}





