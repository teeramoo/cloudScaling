/* \author Geoffrey Biggs */


#include <iostream>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/common/distances.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <string.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

// --------------
// -----Help-----
// --------------
void
printUsage (const char* progName)
{
  std::cout << "\n\nUsage: "<<progName<<" [options]\n\n"
            << "Options:\n"
            << "-------------------------------------------\n"
            << "-h           this help\n"
            << "-s           Simple visualisation example\n"
            << "-r           RGB colour visualisation example\n"
            << "-c           Custom colour visualisation example\n"
            << "-n           Normals visualisation example\n"
            << "-a           Shapes visualisation example\n"
            << "-v           Viewports example\n"
            << "-i           Interaction Customization example\n"
            << "\n\n";
}


boost::shared_ptr<pcl::visualization::PCLVisualizer> simpleVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}


boost::shared_ptr<pcl::visualization::PCLVisualizer> rgbVis (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "sample cloud");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}


boost::shared_ptr<pcl::visualization::PCLVisualizer> customColourVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, 0, 255, 0);
  viewer->addPointCloud<pcl::PointXYZ> (cloud, single_color, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}


boost::shared_ptr<pcl::visualization::PCLVisualizer> normalsVis (
    pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud, pcl::PointCloud<pcl::Normal>::ConstPtr normals)
{
  // --------------------------------------------------------
  // -----Open 3D viewer and add point cloud and normals-----
  // --------------------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "sample cloud");
  viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (cloud, normals, 10, 0.05, "normals");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  
  pcl::Normal randNormal = normals->points[70];
  pcl::PointXYZRGB randPoint = cloud->points[70];
  cout << "Point : " << randPoint.x << ", " << randPoint.y << ", " << randPoint.z << endl;
  cout << "Normal point: " << randNormal.normal_x << ", " << randNormal.normal_y << ", " << randNormal.normal_z << endl;
  
  cout << "eq: " << (randPoint.x * randNormal.normal_x) + (randPoint.y * randNormal.normal_y) + (randPoint.z * randNormal.normal_z) << endl;
  
  pcl::ModelCoefficients coeffs;
  coeffs.values.push_back (randPoint.x);
  coeffs.values.push_back (randPoint.y);
  coeffs.values.push_back (randPoint.z);
  coeffs.values.push_back (randNormal.normal_x);
  coeffs.values.push_back (randNormal.normal_y);
  coeffs.values.push_back (randNormal.normal_z);
  coeffs.values.push_back (0.01);
  viewer->addCylinder (coeffs);
  
  return (viewer);
}


boost::shared_ptr<pcl::visualization::PCLVisualizer> shapesVis (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();

  //------------------------------------
  //-----Add shapes at cloud points-----
  //------------------------------------
  viewer->addLine<pcl::PointXYZRGB> (cloud->points[0],
                                     cloud->points[cloud->size() - 1], "line");
  viewer->addSphere (cloud->points[0], 0.2, 0.5, 0.5, 0.0, "sphere");

  //---------------------------------------
  //-----Add shapes at other locations-----
  //---------------------------------------
  pcl::ModelCoefficients coeffs;
  coeffs.values.push_back (0.0);
  coeffs.values.push_back (0.0);
  coeffs.values.push_back (1.0);
  coeffs.values.push_back (0.0);
  viewer->addPlane (coeffs, "plane");
  coeffs.values.clear ();
  coeffs.values.push_back (0.3);
  coeffs.values.push_back (0.3);
  coeffs.values.push_back (0.0);
  coeffs.values.push_back (0.0);
  coeffs.values.push_back (1.0);
  coeffs.values.push_back (0.0);
  coeffs.values.push_back (5.0);
  viewer->addCone (coeffs, "cone");

  return (viewer);
}


boost::shared_ptr<pcl::visualization::PCLVisualizer> viewportsVis (
    pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud, pcl::PointCloud<pcl::Normal>::ConstPtr normals1, pcl::PointCloud<pcl::Normal>::ConstPtr normals2)
{
  // --------------------------------------------------------
  // -----Open 3D viewer and add point cloud and normals-----
  // --------------------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->initCameraParameters ();

  int v1(0);
  viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
  viewer->setBackgroundColor (0, 0, 0, v1);
  viewer->addText("Radius: 0.01", 10, 10, "v1 text", v1);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud1", v1);

  int v2(0);
  viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
  viewer->setBackgroundColor (0.3, 0.3, 0.3, v2);
  viewer->addText("Radius: 0.1", 10, 10, "v2 text", v2);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> single_color(cloud, 0, 255, 0);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, single_color, "sample cloud2", v2);

  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud1");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud2");
  viewer->addCoordinateSystem (1.0);

  viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (cloud, normals1, 10, 0.05, "normals1", v1);
  viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (cloud, normals2, 10, 0.05, "normals2", v2);

  return (viewer);
}


unsigned int text_id = 0;
void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event,
                            void* viewer_void)
{
  pcl::visualization::PCLVisualizer *viewer = static_cast<pcl::visualization::PCLVisualizer *> (viewer_void);
  if (event.getKeySym () == "r" && event.keyDown ())
  {
    std::cout << "r was pressed => removing all text" << std::endl;

    char str[512];
    for (unsigned int i = 0; i < text_id; ++i)
    {
      sprintf (str, "text#%03d", i);
      viewer->removeShape (str);
    }
    text_id = 0;
  }
}

void mouseEventOccurred (const pcl::visualization::MouseEvent &event,
                         void* viewer_void)
{
  pcl::visualization::PCLVisualizer *viewer = static_cast<pcl::visualization::PCLVisualizer *> (viewer_void);
  if (event.getButton () == pcl::visualization::MouseEvent::LeftButton &&
      event.getType () == pcl::visualization::MouseEvent::MouseButtonRelease)
  {
    std::cout << "Left mouse button released at position (" << event.getX () << ", " << event.getY () << ")" << std::endl;

    char str[512];
    sprintf (str, "text#%03d", text_id ++);
    viewer->addText ("clicked here", event.getX (), event.getY (), str);
  }
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> interactionCustomizationVis ()
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addCoordinateSystem (1.0);

  viewer->registerKeyboardCallback (keyboardEventOccurred, (void*)viewer.get ());
  viewer->registerMouseCallback (mouseEventOccurred, (void*)viewer.get ());

  return (viewer);
}


// --------------
// -----Main-----
// --------------
int
main (int argc, char** argv) {
    // --------------------------------------
    // -----Parse Command Line Arguments-----
    // --------------------------------------
    std::string sInput = "";
    std::string sInputKeyframe = "";
    if (pcl::console::find_argument(argc, argv, "-h") >= 0) {
        printUsage(argv[0]);
        return 0;
    }
    bool simple(false), rgb(false), custom_c(false), normals(false),
            shapes(false), viewports(false), interaction_customization(false);
    if (pcl::console::find_argument(argc, argv, "-s") >= 0) {
        simple = true;
        std::cout << "Simple visualisation example\n";
    } else if (pcl::console::find_argument(argc, argv, "-c") >= 0) {
        custom_c = true;
        std::cout << "Custom colour visualisation example\n";
    } else if (pcl::console::find_argument(argc, argv, "-r") >= 0) {
        rgb = true;
        std::cout << "RGB colour visualisation example\n";
    } else if (pcl::console::find_argument(argc, argv, "-n") >= 0) {
        normals = true;
        std::cout << "Normals visualisation example\n";
    } else if (pcl::console::find_argument(argc, argv, "-a") >= 0) {
        shapes = true;
        std::cout << "Shapes visualisation example\n";
    } else if (pcl::console::find_argument(argc, argv, "-v") >= 0) {
        viewports = true;
        std::cout << "Viewports example\n";
    } else if (pcl::console::find_argument(argc, argv, "-i") >= 0) {
        interaction_customization = true;
        std::cout << "Interaction Customization example\n";
    } else {
        printUsage(argv[0]);
        return 0;
    }

    // ------------------------------------
    // -----Create example point cloud-----
    // ------------------------------------
    pcl::PointCloud<pcl::PointXYZ>::Ptr basic_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoint_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    std::vector<int> vkpCounter;
    double tkpx = 0, tkpy = 0, tkpz = 0;

    double heightPlatform = 0.00;
    double searchRadius = 0.00;
    double realDistance = 0.0;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_p5(new pcl::PointCloud<pcl::PointXYZRGB>);


    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

    if(argc != 8)
    {
        cout << " usage :"<<  argv[0]   << " -r <path_to_pointclouds_with_keyframes> <path_to_point_cloud_upward> <path_to_keyframes> <height_of_the_platform> <search_radius> <real distance travelled>" << endl;
        return  -1;
    }

        sInput = std::string(argv[2]);
        sInputKeyframe = std::string(argv[4]);
        heightPlatform = std::stod(std::string(argv[5]));
        searchRadius = std::stod(std::string(argv[6]));
        realDistance = std::stod(std::string(argv[7]));


        std::string sInputUpward = std::string(argv[3]);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_upward_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);


        pcl::io::loadPCDFile(sInput, *cloud);
        pcl::io::loadPCDFile(sInputKeyframe, *keypoint_ptr);
        pcl::io::loadPCDFile(sInputUpward, *cloud_upward_ptr);


        int keypointSize = keypoint_ptr->points.size();
        cout << "keypointSize : " << keypointSize << endl;

        int halfKPsize = (int) floor((double)keypointSize/2);
        cout << "halfKPsize : " << halfKPsize << endl;

          tkpx = keypoint_ptr->points[halfKPsize].x;
          tkpy = keypoint_ptr->points[halfKPsize].y;
          tkpz = keypoint_ptr->points[halfKPsize].z;


        // Create the segmentation object
        pcl::SACSegmentation<pcl::PointXYZRGB> seg;
        // Optional
        seg.setOptimizeCoefficients (true);
        // Mandatory
        seg.setModelType (pcl::SACMODEL_PLANE);
        seg.setMethodType (pcl::SAC_RANSAC);
        seg.setDistanceThreshold (0.1);

        seg.setInputCloud (cloud);
        seg.segment (*inliers, *coefficients);

        if (inliers->indices.size () == 0)
        {
            PCL_ERROR ("Could not estimate a planar model for the given dataset.");
            return (-1);
        }
        else if(inliers->indices.size() < cloud->points.size() * 0.2)
        {
            cout << "Warning! Less than 20 per cent of total number of point cloud"
                  <<  " represents the detected planar model" << endl << endl;
        }


        std::cerr << "Model coefficients: " << coefficients->values[0] << " "
                  << coefficients->values[1] << " "
                  << coefficients->values[2] << " "
                  << coefficients->values[3] << std::endl;

        std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;
/*
        int kpcounter = 0;

        int calculated_point =0;
        for (size_t j = 0; j < cloud->points.size(); j++) {
            pcl::PointXYZRGB point = cloud->points[j];

            double tmpx = point.x;
            double tmpy = point.y;
            double tmpz = point.z;
            double dist = sqrt(pow(tkpx - tmpx, 2.0) + pow(tkpy - tmpy, 2.0) + pow(tkpz - tmpz, 2.0));



            if (dist < 0.8dist < 0.8/) {
                calculated_point++;
                uint32_t temprgb = (uint32_t) point.rgb;
                if (temprgb == 16777215)
                    vkpCounter.push_back(kpcounter);
                float tfrgb = *reinterpret_cast<float *>(&temprgb);
                point.rgb = tfrgb;
                point_cloud_ptr->points.push_back(point);
                kpcounter++;
            }
        }

        cout << "Total pointcloud" << cloud->points.size() << endl;
        cout << "total points calculated" << calculated_point << endl;
        cout << "conversion done.. " << endl;
        cout << "KF total :" << vkpCounter.size() << endl;

        basic_cloud_ptr->width = (int) basic_cloud_ptr->points.size();
        basic_cloud_ptr->height = 1;
        point_cloud_ptr->width = (int) point_cloud_ptr->points.size();
        point_cloud_ptr->height = 1;
        keypoint_ptr->width = (int) keypoint_ptr->points.size();
        keypoint_ptr->height = 1;

        writer.write("filtered_under_kf.pcd", *point_cloud_ptr, false);
*/
    pcl::PCDWriter writer;

        // ----------------------------------------------------------------
        // -----Calculate surface normals with a search radius of 0.05-----
        // ----------------------------------------------------------------
        pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
        ne.setInputCloud(point_cloud_ptr);
        pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
        ne.setSearchMethod(tree);
        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals1(new pcl::PointCloud<pcl::Normal>);
//        ne.setRadiusSearch(0.05);
//        ne.compute(*cloud_normals1);

        // ---------------------------------------------------------------
        // -----Calculate surface normals with a search radius of 0.1-----
        // ---------------------------------------------------------------
        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2(new pcl::PointCloud<pcl::Normal>);
        ne.setRadiusSearch(searchRadius);
        //ne.setViewPoint((float)tkpx,(float)tkpy,(float)tkpz);
        ne.compute(*cloud_normals2);

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr filter_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::Normal>::Ptr filter_cloud_normals(new pcl::PointCloud<pcl::Normal>);

        std::vector<double> vDistance;
        double dTotalDist = 0;

    double cumDistance = 0;
    double A = coefficients->values[0];
    double B = coefficients->values[1];
    double C = coefficients->values[2];
    double D = coefficients->values[3];

    int numSample = 0;
    for(int i =0; i < keypoint_ptr->points.size(); i++)
    {
        cumDistance += fabs(A*keypoint_ptr->points[i].x + B*keypoint_ptr->points[i].y + C*keypoint_ptr->points[i].z +D) / sqrt(pow(A,2) + pow(B,2) + pow(C,2));
        numSample +=1;
    }

//    double dScale = heightPlatform/ (cumDistance/keypoint_ptr->points.size());
    double dScale = heightPlatform/ (cumDistance/numSample);
    cout << "Total cumDistance : " << cumDistance << endl;
//    cout << "Average cumDistance : " << cumDistance/keypoint_ptr->points.size() << endl;
    cout << "Average cumDistance : " << cumDistance/numSample << endl;
    cout << "Scale : " << dScale << endl;
    cout << "Scaling map..." << endl;

    double avgX = (keypoint_ptr->points[1].x + keypoint_ptr->points[keypoint_ptr->points.size()-1].x) /2.0;
    double avgY = (keypoint_ptr->points[1].y + keypoint_ptr->points[keypoint_ptr->points.size()-1].y)/2.0;
    double avgZ = (keypoint_ptr->points[1].z + keypoint_ptr->points[keypoint_ptr->points.size()-1].z)/2.0;

    double alternativeScale = fabs(A*avgX + B*avgY + C*avgZ +D) / sqrt(pow(A,2) + pow(B,2) + pow(C,2));
    cout << "Alternative scale is : " << heightPlatform/alternativeScale << endl;

    double distanceFirst = fabs(A*keypoint_ptr->points[1].x + B*keypoint_ptr->points[1].y + C*keypoint_ptr->points[1].z +D)
            / sqrt(pow(A,2) + pow(B,2) + pow(C,2));
    double distanceLast = fabs(A*keypoint_ptr->points[keypoint_ptr->points.size()-1].x + B*keypoint_ptr->points[keypoint_ptr->points.size()-1].y
            + C*keypoint_ptr->points[keypoint_ptr->points.size()-1].z +D) / sqrt(pow(A,2) + pow(B,2) + pow(C,2));

    cout << "distFirst : " << distanceFirst << endl;
    cout << "distLast : " << distanceLast << endl;


        // Scale the whole map based on height of cam
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr scaled_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr scaled_keypoint_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr scaled_cloud_upward_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);


        for (size_t j = 0; j < cloud->points.size(); j++)
        {
            pcl::PointXYZRGB tpoint = cloud->points[j];
            tpoint.x *= dScale;
            tpoint.y *= dScale;
            tpoint.z *= dScale;

            //Convert color to float
            /*
            uint32_t temprgb = (uint32_t) tpoint.rgb;
            float tfrgb = *reinterpret_cast<float *>(&temprgb);
            tpoint.rgb = tfrgb;
            scaled_cloud_ptr->points.push_back(tpoint);
             */
            scaled_cloud_ptr->points.push_back(tpoint);
        }


        for (size_t j = 0; j < cloud_upward_ptr->points.size(); j++)
        {
            pcl::PointXYZRGB tpoint = cloud_upward_ptr->points[j];
            tpoint.x *= dScale;
            tpoint.y *= dScale;
            tpoint.z *= dScale;

            scaled_cloud_upward_ptr->points.push_back(tpoint);
        }

        // Calculate distance travelled in the map
        double dTravelledDistScaled = 0;
        for (size_t j = 0; j < keypoint_ptr->points.size(); j++)
        {
            pcl::PointXYZRGB tpoint = keypoint_ptr->points[j];
            tpoint.x *= dScale;
            tpoint.y *= dScale;
            tpoint.z *= dScale;

            //Convert color to float
            /*
            uint32_t temprgb = (uint32_t) tpoint.rgb;
            float tfrgb = *reinterpret_cast<float *>(&temprgb);
            tpoint.rgb = tfrgb;
            scaled_keypoint_ptr->points.push_back(tpoint);
            */
            scaled_keypoint_ptr->points.push_back(tpoint);
            if (j > 0)
            {
                double dTempDist = sqrt(
                        pow(scaled_keypoint_ptr->points[j].x - scaled_keypoint_ptr->points[j - 1].x, 2.0) +
                        pow(scaled_keypoint_ptr->points[j].y - scaled_keypoint_ptr->points[j - 1].y, 2.0) +
                        pow(scaled_keypoint_ptr->points[j].z - scaled_keypoint_ptr->points[j - 1].z, 2.0));
                dTravelledDistScaled += dTempDist;
            }
        }

        double disTravel2 = sqrt(
                pow(scaled_keypoint_ptr->points.back().x - scaled_keypoint_ptr->points.front().x, 2.0) +
                pow(scaled_keypoint_ptr->points.back().y - scaled_keypoint_ptr->points.front().y, 2.0) +
                pow(scaled_keypoint_ptr->points.back().z - scaled_keypoint_ptr->points.front().z, 2.0));

        double originalDisTravel = sqrt(pow(keypoint_ptr->points.back().x - keypoint_ptr->points.front().x, 2.0) +
                                        pow(keypoint_ptr->points.back().y - keypoint_ptr->points.front().y, 2.0) +
                                        pow(keypoint_ptr->points.back().z - keypoint_ptr->points.front().z, 2.0));

        cout << "original distance tralvel : " << originalDisTravel << endl;
        cout << "Actual scale : " << 43.0 / originalDisTravel << endl;

        cout << "Distance travelled = " << dTravelledDistScaled << endl;
        cout << "Distance travelled 2 : " << disTravel2 << endl;

        cout << "Actual distance travelled : " << disTravel2/dScale * 43.0 / originalDisTravel << endl;
        cout << "Scaling done" << endl;

        scaled_cloud_ptr->width = (int) scaled_cloud_ptr->points.size();
        scaled_cloud_ptr->height = 1;
        scaled_keypoint_ptr->width = (int) scaled_keypoint_ptr->points.size();
        scaled_keypoint_ptr->height = 1;
        scaled_cloud_upward_ptr->width = (int) scaled_cloud_upward_ptr->points.size();
        scaled_cloud_upward_ptr->height = 1;

        cout << "Saving scaled point cloud files..." << endl;
        writer.write("Downward_scaled_point_cloud.pcd", *scaled_cloud_ptr, false);
        writer.write("Scaled_keyframe.pcd", *scaled_keypoint_ptr, false);
        writer.write("Upward_scaled_poincloud.pcd", *scaled_cloud_upward_ptr, false);



        //scale with real distance
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr real_scaled_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr real_scaled_keypoint_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr real_scaled_cloud_upward_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

    double realScale = realDistance / sqrt( pow(keypoint_ptr->points.back().x - keypoint_ptr->points.front().x,2) +
                                            pow(keypoint_ptr->points.back().y - keypoint_ptr->points.front().y,2) +
                                            pow(keypoint_ptr->points.back().z - keypoint_ptr->points.front().z,2) ) ;

    cout << "real scale : " << realScale << endl;
     //Scaling downward points
    for (size_t j = 0; j < cloud->points.size(); j++)
    {
        pcl::PointXYZRGB tpoint = cloud->points[j];
        tpoint.x *= realScale;
        tpoint.y *= realScale;
        tpoint.z *= realScale;

        //Convert color to float
        /*
        uint32_t temprgb = (uint32_t) tpoint.rgb;
        float tfrgb = *reinterpret_cast<float *>(&temprgb);
        tpoint.rgb = tfrgb;
        scaled_cloud_ptr->points.push_back(tpoint);
         */
        real_scaled_cloud_ptr->points.push_back(tpoint);
    }

    //Scaling upward points
    for (size_t j = 0; j < cloud_upward_ptr->points.size(); j++)
    {
        pcl::PointXYZRGB tpoint = cloud_upward_ptr->points[j];
        tpoint.x *= realScale;
        tpoint.y *= realScale;
        tpoint.z *= realScale;

        real_scaled_cloud_upward_ptr->points.push_back(tpoint);
    }

    // Calculate distance travelled in the map

    //Scaling keyframe
    for (size_t j = 0; j < keypoint_ptr->points.size(); j++)
    {
        pcl::PointXYZRGB tpoint = keypoint_ptr->points[j];
        tpoint.x *= realScale;
        tpoint.y *= realScale;
        tpoint.z *= realScale;

        //Convert color to float
        /*
        uint32_t temprgb = (uint32_t) tpoint.rgb;
        float tfrgb = *reinterpret_cast<float *>(&temprgb);
        tpoint.rgb = tfrgb;
        scaled_keypoint_ptr->points.push_back(tpoint);
        */
        real_scaled_keypoint_ptr->points.push_back(tpoint);

    }





    real_scaled_cloud_ptr->width = (int) real_scaled_cloud_ptr->points.size();
    real_scaled_cloud_ptr->height = 1;
    real_scaled_keypoint_ptr->width = (int) real_scaled_keypoint_ptr->points.size();
    real_scaled_keypoint_ptr->height = 1;
    real_scaled_cloud_upward_ptr->width = (int) real_scaled_cloud_upward_ptr->points.size();
    real_scaled_cloud_upward_ptr->height = 1;


    writer.write("real_distance_Downward_scaled_point_cloud.pcd", *scaled_cloud_ptr, false);
    writer.write("real_distance_Scaled_keyframe.pcd", *scaled_keypoint_ptr, false);
    writer.write("real_distance_Upward_scaled_poincloud.pcd", *scaled_cloud_upward_ptr, false);

        cout << "Saved scaled point clouds" << endl;

        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
        cout << "Creating view port." << endl;
        if (simple) {
            viewer = simpleVis(basic_cloud_ptr);
        } else if (rgb) {
            viewer = rgbVis( cloud/* scaled_cloud_ptr */);
        } else if (custom_c) {
            viewer = customColourVis(basic_cloud_ptr);
        } else if (normals) {
            viewer = normalsVis(filter_cloud_ptr, filter_cloud_normals);
            //viewer = normalsVis(point_cloud_ptr, cloud_normals2);
        } else if (shapes) {
            viewer = shapesVis(scaled_cloud_ptr);
        } else if (viewports) {
            viewer = viewportsVis(point_cloud_ptr, cloud_normals1, cloud_normals2);
        } else if (interaction_customization) {
            viewer = interactionCustomizationVis();
        }

    viewer->addPlane(*coefficients,0,0,0,"plane");

        //--------------------
        // -----Main loop-----
        //--------------------
        while (!viewer->wasStopped()) {
            viewer->spinOnce(100);
            boost::this_thread::sleep(boost::posix_time::microseconds(100000));
        }
    }
