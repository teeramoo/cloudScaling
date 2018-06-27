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

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_p5(new pcl::PointCloud<pcl::PointXYZRGB>);


    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

    if(argc != 6)
    {
        cout << " usage : ./cloud_visualizer -r <path_to_pointclouds_with_keyframes> <path_to_keyframes> <height_of_the_platform> <search_radius>" << endl;
    }

    if (argc == 6) {
        sInput = std::string(argv[2]);
        sInputKeyframe = std::string(argv[3]);

        heightPlatform = std::stod(std::string(argv[4]));
        searchRadius = std::stod(std::string(argv[5]));

        pcl::io::loadPCDFile(sInput, *cloud);
        pcl::io::loadPCDFile(sInputKeyframe, *keypoint_ptr);

        int keypointSize = keypoint_ptr->points.size();
        cout << "keypointSize : " << keypointSize << endl;

        int halfKPsize = (int) floor((double)keypointSize/2);
        cout << "halfKPsize : " << halfKPsize << endl;

          tkpx = keypoint_ptr->points[halfKPsize].x;
          tkpy = keypoint_ptr->points[halfKPsize].y;
          tkpz = keypoint_ptr->points[halfKPsize].z;

//        tkpx = keypoint_ptr->points[83].x;
//        tkpy = keypoint_ptr->points[83].y;
//        tkpz = keypoint_ptr->points[83].z;


        // Create the segmentation object
        pcl::SACSegmentation<pcl::PointXYZRGB> seg;
        // Optional
        seg.setOptimizeCoefficients (true);
        // Mandatory
        seg.setModelType (pcl::SACMODEL_PLANE);
        seg.setMethodType (pcl::SAC_RANSAC);
        seg.setDistanceThreshold (0.01);

        seg.setInputCloud (cloud);
        seg.segment (*inliers, *coefficients);

        if (inliers->indices.size () == 0)
        {
            PCL_ERROR ("Could not estimate a planar model for the given dataset.");
            return (-1);
        }

        std::cerr << "Model coefficients: " << coefficients->values[0] << " "
                  << coefficients->values[1] << " "
                  << coefficients->values[2] << " "
                  << coefficients->values[3] << std::endl;

        std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;
/*


        // Create the filtering object
        pcl::ExtractIndices<pcl::PointXYZRGB> extract (true);

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_p (new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_p2(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_p3(new pcl::PointCloud<pcl::PointXYZRGB>);

        extract.setInputCloud (cloud);
        extract.setIndices (inliers);
        extract.setNegative (true);
        extract.filter(*cloud_p);

        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_p1 (new pcl::visualization::PCLVisualizer ("True"));
        viewer_p1->setBackgroundColor (0, 0, 0);
        viewer_p1->addPointCloud<pcl::PointXYZRGB> (cloud_p, "true");
        viewer_p1->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "true");
        viewer_p1->addCoordinateSystem (1.0);
        viewer_p1->initCameraParameters ();
        while (!viewer_p1->wasStopped()) {
            viewer_p1->spinOnce(100);
            boost::this_thread::sleep(boost::posix_time::microseconds(100000));
        }


        extract.setNegative (false);
        extract.filter(*cloud_p2);

        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_p2 (new pcl::visualization::PCLVisualizer ("False"));
        viewer_p2->setBackgroundColor (0, 0, 0);
        viewer_p2->addPointCloud<pcl::PointXYZRGB> (cloud_p2, "false");
        viewer_p2->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "false");
        viewer_p2->addCoordinateSystem (1.0);
        viewer_p2->initCameraParameters ();
        while (!viewer_p2->wasStopped()) {
            viewer_p2->spinOnce(100);
            boost::this_thread::sleep(boost::posix_time::microseconds(100000));
        }




        pcl::PointIndices::Ptr outliers (new pcl::PointIndices);
        extract.getRemovedIndices(*outliers);
        std::cerr << "Model outliers: " << outliers->indices.size () << std::endl;

        extract.setIndices(outliers);
        extract.filter(*cloud_p3);

        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_p3 (new pcl::visualization::PCLVisualizer ("Outlier"));
        viewer_p3->setBackgroundColor (0, 0, 0);
        viewer_p3->addPointCloud<pcl::PointXYZRGB> (cloud_p3, "outlier");
        viewer_p3->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "outlier");
        viewer_p3->addCoordinateSystem (1.0);
        viewer_p3->initCameraParameters ();
        while (!viewer_p3->wasStopped()) {
            viewer_p3->spinOnce(100);
            boost::this_thread::sleep(boost::posix_time::microseconds(100000));
        }



        //2nd segmentation
        seg.setOptimizeCoefficients (true);
        // Mandatory
        seg.setModelType (pcl::SACMODEL_PLANE);
        seg.setMethodType (pcl::SAC_RANSAC);
        seg.setDistanceThreshold (0.01);

        pcl::ModelCoefficients::Ptr coefficients2 (new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers2 (new pcl::PointIndices);
        seg.setInputCloud (cloud_p3);
        seg.segment (*inliers2, *coefficients2);

        if (inliers2->indices.size () == 0)
        {
            PCL_ERROR ("Could not estimate a planar model for the given dataset.");
            return (-1);
        }

        std::cerr << "Model coefficients2: " << coefficients2->values[0] << " "
                  << coefficients2->values[1] << " "
                  << coefficients2->values[2] << " "
                  << coefficients2->values[3] << std::endl;

        std::cerr << "Model inliers2: " << inliers2->indices.size () << std::endl;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_p4(new pcl::PointCloud<pcl::PointXYZRGB>);



//    pcl::ExtractIndices<pcl::PointXYZRGB> extract2 (true);
        extract.setInputCloud (cloud_p3);
        extract.setIndices (inliers2);
        extract.setNegative (false);
        extract.filter(*cloud_p4);


        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_p4 (new pcl::visualization::PCLVisualizer ("Inlier2"));
        viewer_p4->setBackgroundColor (0, 0, 0);
        viewer_p4->addPointCloud<pcl::PointXYZRGB> (cloud_p4, "inlier2");
        viewer_p4->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "inlier2");
        viewer_p4->addCoordinateSystem (1.0);
        viewer_p4->initCameraParameters ();
        while (!viewer_p4->wasStopped()) {
            viewer_p4->spinOnce(100);
            boost::this_thread::sleep(boost::posix_time::microseconds(100000));
        }


        pcl::PointIndices::Ptr outliers2 (new pcl::PointIndices);
        extract.getRemovedIndices(*outliers2);
        std::cerr << "Model outliers: " << outliers2->indices.size () << std::endl;

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_p5(new pcl::PointCloud<pcl::PointXYZRGB>);
        extract.setIndices(outliers2);
        extract.filter(*cloud_p5);

        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_p5 (new pcl::visualization::PCLVisualizer ("Outlier2"));
        viewer_p5->setBackgroundColor (0, 0, 0);
        viewer_p5->addPointCloud<pcl::PointXYZRGB> (cloud_p5, "outlier2");
        viewer_p5->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "outlier2");
        viewer_p5->addCoordinateSystem (1.0);
        viewer_p5->initCameraParameters ();
        while (!viewer_p5->wasStopped()) {
            viewer_p5->spinOnce(100);
            boost::this_thread::sleep(boost::posix_time::microseconds(100000));
        }
*/

        int kpcounter = 0;

        int calculated_point =0;
        for (size_t j = 0; j < cloud->points.size(); j++) {
            pcl::PointXYZRGB point = cloud->points[j];

            double tmpx = point.x;
            double tmpy = point.y;
            double tmpz = point.z;
            double dist = sqrt(pow(tkpx - tmpx, 2.0) + pow(tkpy - tmpy, 2.0) + pow(tkpz - tmpz, 2.0));



            if (dist < 0.8/*dist < 0.8 */) {
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
        /*
        cout << "KF at: " << endl;
        for(size_t k = 0; k < vkpCounter.size(); k++)
        {
            int location = vkpCounter[k];
            cout << point_cloud_ptr->points[location].x << ", " << point_cloud_ptr->points[location].y << ", " << point_cloud_ptr->points[location].z << ", Color: " << point_cloud_ptr->points[location].rgb << endl;
        }
        cout << "-------------------------" << endl;
        */
    }
    else
    {
        /*
        std::cout << "Genarating example point clouds.\n\n";
        // We're going to make an ellipse extruded along the z-axis. The colour for
        // the XYZRGB cloud will gradually go from red to green to blue.

        for(int j = 0; j < 256; j++)
        {
            uint32_t color = j;
            float val = *reinterpret_cast<float*>(&color);
             cout << val << ", " << j << endl;
        }

            uint32_t color = 16777215;
          float val = *reinterpret_cast<float*>(&color);
          cout << val << ", " << color << endl;
        uint8_t r(255), g(15), b(15);
        for (float z(-1.0); z <= 1.0; z += 0.05)
        {
          for (float angle(0.0); angle <= 360.0; angle += 5.0)
          {
            pcl::PointXYZ basic_point;
            basic_point.x = 0.5 * cosf (pcl::deg2rad(angle));
            basic_point.y = sinf (pcl::deg2rad(angle));
            basic_point.z = z;
            basic_cloud_ptr->points.push_back(basic_point);

            pcl::PointXYZRGB point;
            point.x = basic_point.x;
            point.y = basic_point.y;
            point.z = basic_point.z;
            uint32_t rgb = (static_cast<uint32_t>(r) << 16 | static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
            point.rgb = *reinterpret_cast<float*>(&rgb);
            point_cloud_ptr->points.push_back (point);

          }

          if (z < 0.0)
          {
            r -= 12;
            g += 12;
          }
          else
          {
            g -= 12;
            b += 12;
          }
        }
       */
    }
/*
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.01);

    seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients);

    if (inliers->indices.size () == 0)
    {
        PCL_ERROR ("Could not estimate a planar model for the given dataset.");
        return (-1);
    }

    std::cerr << "Model coefficients: " << coefficients->values[0] << " "
              << coefficients->values[1] << " "
              << coefficients->values[2] << " "
              << coefficients->values[3] << std::endl;

    std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;



    // Create the filtering object
    pcl::ExtractIndices<pcl::PointXYZRGB> extract (true);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_p (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_p2(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_p3(new pcl::PointCloud<pcl::PointXYZRGB>);

    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.setNegative (true);
    extract.filter(*cloud_p);

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_p1 (new pcl::visualization::PCLVisualizer ("True"));
    viewer_p1->setBackgroundColor (0, 0, 0);
    viewer_p1->addPointCloud<pcl::PointXYZRGB> (cloud_p, "true");
    viewer_p1->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "true");
    viewer_p1->addCoordinateSystem (1.0);
    viewer_p1->initCameraParameters ();
    while (!viewer_p1->wasStopped()) {
        viewer_p1->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }


    extract.setNegative (false);
    extract.filter(*cloud_p2);

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_p2 (new pcl::visualization::PCLVisualizer ("False"));
    viewer_p2->setBackgroundColor (0, 0, 0);
    viewer_p2->addPointCloud<pcl::PointXYZRGB> (cloud_p2, "false");
    viewer_p2->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "false");
    viewer_p2->addCoordinateSystem (1.0);
    viewer_p2->initCameraParameters ();
    while (!viewer_p2->wasStopped()) {
        viewer_p2->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }




    pcl::PointIndices::Ptr outliers (new pcl::PointIndices);
    extract.getRemovedIndices(*outliers);
    std::cerr << "Model outliers: " << outliers->indices.size () << std::endl;

    extract.setIndices(outliers);
    extract.filter(*cloud_p3);

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_p3 (new pcl::visualization::PCLVisualizer ("Outlier"));
    viewer_p3->setBackgroundColor (0, 0, 0);
    viewer_p3->addPointCloud<pcl::PointXYZRGB> (cloud_p3, "outlier");
    viewer_p3->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "outlier");
    viewer_p3->addCoordinateSystem (1.0);
    viewer_p3->initCameraParameters ();
    while (!viewer_p3->wasStopped()) {
        viewer_p3->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }



    //2nd segmentation
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.01);

    pcl::ModelCoefficients::Ptr coefficients2 (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers2 (new pcl::PointIndices);
    seg.setInputCloud (cloud_p3);
    seg.segment (*inliers2, *coefficients2);

    if (inliers2->indices.size () == 0)
    {
        PCL_ERROR ("Could not estimate a planar model for the given dataset.");
        return (-1);
    }

    std::cerr << "Model coefficients2: " << coefficients2->values[0] << " "
              << coefficients2->values[1] << " "
              << coefficients2->values[2] << " "
              << coefficients2->values[3] << std::endl;

    std::cerr << "Model inliers2: " << inliers2->indices.size () << std::endl;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_p4(new pcl::PointCloud<pcl::PointXYZRGB>);



//    pcl::ExtractIndices<pcl::PointXYZRGB> extract2 (true);
    extract.setInputCloud (cloud_p3);
    extract.setIndices (inliers2);
    extract.setNegative (false);
    extract.filter(*cloud_p4);


    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_p4 (new pcl::visualization::PCLVisualizer ("Inlier2"));
    viewer_p4->setBackgroundColor (0, 0, 0);
    viewer_p4->addPointCloud<pcl::PointXYZRGB> (cloud_p4, "inlier2");
    viewer_p4->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "inlier2");
    viewer_p4->addCoordinateSystem (1.0);
    viewer_p4->initCameraParameters ();
    while (!viewer_p4->wasStopped()) {
        viewer_p4->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }


    pcl::PointIndices::Ptr outliers2 (new pcl::PointIndices);
    extract.getRemovedIndices(*outliers2);
    std::cerr << "Model outliers: " << outliers2->indices.size () << std::endl;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_p5(new pcl::PointCloud<pcl::PointXYZRGB>);
    extract.setIndices(outliers2);
    extract.filter(*cloud_p5);

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_p5 (new pcl::visualization::PCLVisualizer ("Outlier2"));
    viewer_p5->setBackgroundColor (0, 0, 0);
    viewer_p5->addPointCloud<pcl::PointXYZRGB> (cloud_p5, "outlier2");
    viewer_p5->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "outlier2");
    viewer_p5->addCoordinateSystem (1.0);
    viewer_p5->initCameraParameters ();
    while (!viewer_p5->wasStopped()) {
        viewer_p5->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }

*/



        basic_cloud_ptr->width = (int) basic_cloud_ptr->points.size();
        basic_cloud_ptr->height = 1;
        point_cloud_ptr->width = (int) point_cloud_ptr->points.size();
        point_cloud_ptr->height = 1;
        keypoint_ptr->width = (int) keypoint_ptr->points.size();
        keypoint_ptr->height = 1;

        pcl::PCDWriter writer;
        writer.write("filtered_under_kf.pcd", *point_cloud_ptr, false);

        // ----------------------------------------------------------------
        // -----Calculate surface normals with a search radius of 0.05-----
        // ----------------------------------------------------------------
        pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
        ne.setInputCloud(point_cloud_ptr);
        pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
        ne.setSearchMethod(tree);
        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals1(new pcl::PointCloud<pcl::Normal>);
        ne.setRadiusSearch(0.05);
        ne.compute(*cloud_normals1);

        // ---------------------------------------------------------------
        // -----Calculate surface normals with a search radius of 0.1-----
        // ---------------------------------------------------------------
        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2(new pcl::PointCloud<pcl::Normal>);
        ne.setRadiusSearch(searchRadius/*0.8*/);
        //ne.setViewPoint((float)tkpx,(float)tkpy,(float)tkpz);
        ne.compute(*cloud_normals2);

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr filter_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::Normal>::Ptr filter_cloud_normals(new pcl::PointCloud<pcl::Normal>);

        std::vector<double> vDistance;
        double dTotalDist = 0;
        cout << "Size of cloud_normals 2 is : " << cloud_normals2->points.size() << endl;
        cout << "Size of vkpCounter is : " << vkpCounter.size() << endl;

    // There is something wrong with vDistance.size();
        int loopCounter = 0;
        for (size_t j = 0; j < cloud_normals2->points.size(); j++) {
            pcl::PointXYZRGB tpoint = point_cloud_ptr->points[j];
            pcl::Normal tkpNorm = cloud_normals2->points[j];

            float rgbVal = tpoint.rgb;
            uint32_t trgbVal = *reinterpret_cast<uint32_t *>(&rgbVal);
            if (trgbVal == 16777215)    //White point (is keyframe)
            {
                continue;
            }

            for (size_t k = 0; k < vkpCounter.size(); k++) {
                int location = vkpCounter[k];
                pcl::PointXYZRGB tkp = point_cloud_ptr->points[location];
                //Eigen::Vector4f evPoint = tpoint.getVector4fMap();

                const Eigen::Vector4f evPoint(tpoint.x, tpoint.y, tpoint.z, 0.0);
                const Eigen::Vector4f evNorm(tkpNorm.normal_x, tkpNorm.normal_y, tkpNorm.normal_z, 0.0);
                const Eigen::Vector4f evkp(tkp.x, tkp.y, tkp.z, 0.0);
                //const Eigen::Vector4f test(1.2,2.2,3.1,0.0);
                //const double distt = 3.2;
                //Eigen::Vector4f evkp = tkp.getVector4fMap();

                //sqrPointToLineDistance(point_cloud_ptr->points[location].getVector4fMap(), point_cloud_ptr->points[location].getVector4fMap(),point_cloud_ptr->points[location].getVector4fMap());
                //sqrPointToLineDistance(evkp, evPoint, evNorm, 1.0);
                //sqrPointToLineDistance(test,test,test,distt);

                // There is some problem here --> change square matrix please
                double sqrDist = (evNorm.cross3(evPoint - evkp)).squaredNorm() / evNorm.squaredNorm();
           //     if((evNorm.cross3(evPoint - evkp)).squaredNorm() > 1 && evNorm.squaredNorm() != 1.0)
           //     {
           //         if( (evNorm[0] >1.0 or evNorm[0] < -1.0) and (evNorm[1] >1.0 or evNorm[1] < -1.0) and (evNorm[2] >1.0 or evNorm[2] < -1.0))
           //         {
                        cout << "evNorm[0], [1], [2] are : " << evNorm[0] << " , " << evNorm[1] << " , " << evNorm[2] << endl;

                        cout << "Norm is greater than 1 " << endl;
                        cout << "evNorm : " << evNorm << endl;
                        cout << "evPoint : " << evPoint << endl;
                        cout << "evkp : " << evkp << endl;
                        cout << "evPoint - evkp : " << evPoint - evkp << endl;
                        Eigen::Vector4f evPoint2(tpoint.x, tpoint.y, tpoint.z, 1.0);
                        Eigen::Vector4f evkp2(tkp.x, tkp.y, tkp.z, 1.0);
                        cout << "evPoint x evkp : " << evPoint2.cross3(evkp2) << endl;
                        cout << "evNorm.cross3 : " << (evNorm.cross3(evPoint - evkp)) << endl;
                        cout << "evNorm.cross3.squaredNorm : " << (evNorm.cross3(evPoint - evkp)).squaredNorm() <<endl;
                        cout << "evNorm.squaredNorm : " << evNorm.squaredNorm() << endl;
                        cout << "sqrDist : " << sqrDist << endl;

            //        }

           //     }
                //Change dist here to adjust range from keypoints
                if (sqrDist < 1e-4) {
                    loopCounter ++;
                    filter_cloud_ptr->points.push_back(tpoint);
                    filter_cloud_normals->points.push_back(tkpNorm);

                    double dist = sqrt(
                            pow(tpoint.x - tkp.x, 2.0) + pow(tpoint.y - tkp.y, 2.0) + pow(tpoint.z - tkp.z, 2.0));
                    //cout << "KP " << k << " " << tkp.x << " " << tkp.y << " " << tkp.z << " point " << tpoint.x << " " << tpoint.y << " " << tpoint.z << " dist " << dist << endl;
                    vDistance.push_back(dist);
                    dTotalDist += dist;
                }
            }
        }

        filter_cloud_ptr->width = (int) filter_cloud_ptr->points.size();
        filter_cloud_ptr->height = 1;

        cout << "loopCounter / totalPossibility is : " << loopCounter << " / " << vkpCounter.size() * (cloud_normals2->points.size() - vkpCounter.size()) << endl;
        cout << "dTotalDist is : " << dTotalDist << endl;
        cout << "vDistance.size() is : " << vDistance.size() << endl;

        // Scale the whole map based on height of cam
        double dMeanDist = dTotalDist / vDistance.size();
        double dScale = heightPlatform / dMeanDist;

        cout << "Mean dist: " << dMeanDist << endl;
        cout << "Scale: " << dScale << endl;

        cout << "Scaling map..." << endl;


    double cumDistance = 0;
    double A = coefficients->values[0];
    double B = coefficients->values[1];
    double C = coefficients->values[2];
    double D = coefficients->values[3];

    for(int i =0; i < keypoint_ptr->points.size(); i++)
    {
        cumDistance += fabs(A*keypoint_ptr->points[i].x + B*keypoint_ptr->points[i].y + C*keypoint_ptr->points[i].z +D) / sqrt(pow(A,2) + pow(B,2) + pow(C,2));
    }

    cout << "Total cumDistance : " << cumDistance << endl;
    cout << "Average cumDistance : " << cumDistance/keypoint_ptr->points.size() << endl;

    dScale = heightPlatform/ (cumDistance/keypoint_ptr->points.size());

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr scaled_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr scaled_keypoint_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
        for (size_t j = 0; j < cloud->points.size(); j++)
        {
            pcl::PointXYZRGB tpoint = cloud->points[j];
            tpoint.x *= dScale;
            tpoint.y *= dScale;
            tpoint.z *= dScale;

            //Convert color to float
            uint32_t temprgb = (uint32_t) tpoint.rgb;
            float tfrgb = *reinterpret_cast<float *>(&temprgb);
            tpoint.rgb = tfrgb;
            scaled_cloud_ptr->points.push_back(tpoint);
        }

        double dTravelledDistScaled = 0;
        for (size_t j = 0; j < keypoint_ptr->points.size(); j++)
        {
            pcl::PointXYZRGB tpoint = keypoint_ptr->points[j];
            tpoint.x *= dScale;
            tpoint.y *= dScale;
            tpoint.z *= dScale;

            //Convert color to float
            uint32_t temprgb = (uint32_t) tpoint.rgb;
            float tfrgb = *reinterpret_cast<float *>(&temprgb);
            tpoint.rgb = tfrgb;
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

        cout << "Distance travelled = " << dTravelledDistScaled << endl;

        cout << "dScale : " << dScale << endl;
        cout << "Distance travelled 2 : " << disTravel2 << endl;
        cout << "Scaling done" << endl;
        scaled_cloud_ptr->width = (int) scaled_cloud_ptr->points.size();
        scaled_cloud_ptr->height = 1;
        scaled_keypoint_ptr->width = (int) scaled_keypoint_ptr->points.size();
        scaled_keypoint_ptr->height = 1;

        cout << "Saving scaled point cloud files..." << endl;
        writer.write("scaled_point_cloud.pcd", *scaled_cloud_ptr, false);
        writer.write("scaled_keyframe.pcd", *scaled_keypoint_ptr, false);
        cout << "Saved scaled point clouds" << endl;

        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
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
