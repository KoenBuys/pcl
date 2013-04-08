/**
 *  Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id: $
 * @brief This file is the execution node of the Human Tracking 
 * @copyright Copyright (2011) Willow Garage
 * @authors Koen Buys, Anatoly Baksheev
 **/

#include <boost/filesystem.hpp>

#include <Eigen/Core>

#include <fstream>

#include <iostream>

#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/time.h>
#include <pcl/console/parse.h>
#include <pcl/console/print.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/png_io.h>
#include <pcl/gpu/people/people_detector.h>
#include <pcl/gpu/people/colormap.h>
#include <pcl/visualization/image_viewer.h>
#include <pcl/search/pcl_search.h>

using namespace pcl::visualization;
using namespace pcl::console;
using namespace pcl::gpu;
using namespace std;

typedef pcl::PointXYZRGBA PointT;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

struct ProjMatrix : public pcl::search::OrganizedNeighbor<PointT>
{  
  using pcl::search::OrganizedNeighbor<PointT>::projection_matrix_;
};

float estimateFocalLength(const pcl::PointCloud<PointT>::ConstPtr &cloud)
{
  ProjMatrix proj_matrix;
  proj_matrix.setInputCloud(cloud);  
  Eigen::Matrix3f KR = proj_matrix.projection_matrix_.topLeftCorner <3, 3> ();    
  return (KR(0,0) + KR(1,1))/KR(2,2)/2;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

vector<string> getPcdFilesInDir(const string& directory)
    {
  namespace fs = boost::filesystem;
  fs::path dir(directory);

  if (!fs::exists(dir) || !fs::is_directory(dir))
    PCL_THROW_EXCEPTION(pcl::IOException, "[getPcdFilesInDir] : Wrong PCD directory");

  vector<string> result;
  fs::directory_iterator pos(dir);
  fs::directory_iterator end;

  for(; pos != end ; ++pos)
    if (fs::is_regular_file(pos->status()) )
      if (fs::extension(*pos) == ".pcd")
        result.push_back(pos->path().string());

  return result;
    }

string 
make_png_name(int counter, const char* suffix)
{
  char buf[4096];
  sprintf (buf, "./people_%04d_%s.png", counter, suffix);
  return buf;
}

string
make_ext_png_name(int counter1, int counter2, const char* suffix)
{
  char buf[4096];
  sprintf (buf, "./people_%04d_%04d_%s.png", counter1, counter2, suffix);
  return buf;
}

string
make_ext_pcd_name(int counter1, int counter2, const char* suffix)
{
  char buf[4096];
  sprintf (buf, "./people_%04d_%04d_%s.pcd", counter1, counter2, suffix);
  return buf;
}

template<typename T> void 
savePNGFile(const std::string& filename, const pcl::gpu::DeviceArray2D<T>& arr)
{
  int c;
  pcl::PointCloud<T> cloud(arr.cols(), arr.rows());
  arr.download(cloud.points, c);
  pcl::io::savePNGFile(filename, cloud);
}

template <typename T> void
savePNGFile (const std::string& filename, const pcl::PointCloud<T>& cloud)
{
  //PCL_DEBUG ("[savePNGFile] : (D) : writing %s with W:%d H:%d S:%d\n", filename.c_str(), cloud.width, cloud.height, cloud.points.size());
  pcl::io::savePNGFile(filename, cloud);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
class PeoplePCDApp
{
  public:
    typedef pcl::gpu::people::PeopleDetector PeopleDetector;

    enum { COLS = 640, ROWS = 480 };

    PeoplePCDApp () : counter_(0), cmap_device_(ROWS, COLS)//, final_view_("Final labeling")//, image_view_("Input image")
    {
      //final_view_.setSize (COLS, ROWS);
      //image_view_.setSize (COLS, ROWS);

      //final_view_.setPosition (0, 0);
      //image_view_.setPosition (650, 0);

      people::uploadColorMap(color_map_);
    }

    void cloud_cb (const pcl::PointCloud<PointT>::ConstPtr &cloud)
    {
      PCL_DEBUG("[PeoplePCDApp::cloud_cb] : (D) : Cloud Callback\n");
      processReturn = people_detector_.processProb(cloud);
      ++counter_;
      PCL_DEBUG("[PeoplePCDApp::cloud_cb] : (D) : Done\n");
    }

    void
    writeXMLFile(std::string& filename)
    {
      filebuf fb;
      fb.open (filename.c_str(), ios::out);
      ostream os(&fb);
      people_detector_.person_attribs_->writePersonXMLConfig(os);
      fb.close();
    }

    void
    readXMLFile(std::string& filename)
    {
      filebuf fb;
      fb.open (filename.c_str(), ios::in);
      istream is(&fb);
      people_detector_.person_attribs_->readPersonXMLConfig(is);
      fb.close();
    }

    void
    visualizeAndWrite(const pcl::PointCloud<PointT>::ConstPtr &cloud)
    {
      PCL_DEBUG("[PeoplePCDApp::visualizeAndWrite] : (D) : called\n");


      // First the 2nd it smooth labels
      const PeopleDetector::Labels& labelsmooth = people_detector_.rdf_detector_->getSmoothLabels();
      people::colorizeLabels(color_map_, labelsmooth, cmap_device_);

      int c;
      pcl::PointCloud<pcl::RGB> cmap_smooth(cmap_device_.cols(), cmap_device_.rows());
      cmap_device_.download(cmap_smooth.points, c);

      savePNGFile(make_png_name(counter_, "c2s"), cmap_smooth);

      // The 2nd it labels
      const PeopleDetector::Labels& labels2 = people_detector_.rdf_detector_->getLabels();
      people::colorizeLabels(color_map_, labels2, cmap_device_);

      pcl::PointCloud<pcl::RGB> cmap2(cmap_device_.cols(), cmap_device_.rows());
      cmap_device_.download(cmap2.points, c);

      savePNGFile(make_png_name(counter_, "c2"), cmap2);

      // The 1st it labels
      const PeopleDetector::Labels& labels1 = people_detector_.rdf_detector_->getLabelsFirst();
      people::colorizeLabels(color_map_, labels1, cmap_device_);

      pcl::PointCloud<pcl::RGB> cmap1(cmap_device_.cols(), cmap_device_.rows());
      cmap_device_.download(cmap1.points, c);

      savePNGFile(make_png_name(counter_, "c1"), cmap1);

      // The 1st it labels
      const PeopleDetector::Labels& labels1s = people_detector_.rdf_detector_->getSmoothLabelsFirst();
      people::colorizeLabels(color_map_, labels1s, cmap_device_);

      pcl::PointCloud<pcl::RGB> cmap1s(cmap_device_.cols(), cmap_device_.rows());
      cmap_device_.download(cmap1s.points, c);

      savePNGFile(make_png_name(counter_, "c1s"), cmap1s);

      // Visualize
      final_view_.showRGBImage<pcl::RGB>(cmap_smooth);
      //final_view_.showRGBImage<pcl::RGB>(people_detector_.haar_cascade_detector_);
      final_view_.spinOnce(1, true);

      image_view_.showRGBImage<PointT>(cloud);
      image_view_.spinOnce(1, true);

      // Save other PNG formats
      savePNGFile(make_png_name(counter_, "ii"), *cloud);
      //savePNGFile(make_png_name(counter_, "haar"), people_detector_.haar_cascade_detector_->cloud_gray_);
      savePNGFile(make_png_name(counter_, "s2"), labelsmooth);
      savePNGFile(make_png_name(counter_, "d1"), people_detector_.depth_device1_);
      savePNGFile(make_png_name(counter_, "d2"), people_detector_.depth_device2_);

      PCL_DEBUG("[PeoplePCDApp::visualizeAndWrite] : (D) : Done writing files\n");
    }

    void
    convertProbToRGB (pcl::PointCloud<pcl::device::prob_histogram>& histograms, int label, pcl::PointCloud<pcl::RGB>& rgb)
    {
      //PCL_DEBUG("[PeoplePCDApp::convertProbToRGB] : (D) : size: %d\n", histograms.points.size());
      for(size_t t = 0; t < histograms.points.size(); t++)
      {
        float value = histograms.points[t].probs[label];
        float value8 = value * 255;
        char val = static_cast<char> (value8);
        pcl::RGB p;
        p.r = val; p.b = val; p.g = val;
        rgb.points.push_back(p);
      }
      rgb.width = histograms.width;
      rgb.height = histograms.height;

      //PCL_DEBUG("[PeoplePCDApp::convertProbToRGB] : (D) : W:%d H:%d S:%d\n",rgb.width, rgb.height, rgb.points.size());
    }

    void
    writeProb(const pcl::PointCloud<PointT>::ConstPtr &cloud)
    {
      PCL_DEBUG("[PeoplePCDApp::writeProb] : (D) : Called\n");
      //const pcl::device::LabelProbability& prob = people_detector_.rdf_detector_->getProbability1();
      int c;

      // first write the first iteration
      pcl::PointCloud<pcl::device::prob_histogram> prob_host(people_detector_.rdf_detector_->P_l_1_.cols(), people_detector_.rdf_detector_->P_l_1_.rows());
      people_detector_.rdf_detector_->P_l_1_.download(prob_host.points, c);
      prob_host.width = people_detector_.rdf_detector_->P_l_1_.cols();
      prob_host.height = people_detector_.rdf_detector_->P_l_1_.rows();

      if(prob_host.width == 0)
        PCL_ERROR ("[PeoplePCDApp::writeProb] : (E) : width == 0\n");

      if(prob_host.height == 0)
        PCL_ERROR ("[PeoplePCDApp::writeProb] : (E) : height == 0\n");

      PCL_DEBUG("[PeoplePCDApp::writeProb] : (D) : savePNGFile histogram1\n");
      for(int i = 0; i < pcl::gpu::people::NUM_LABELS; i++)
      {
        pcl::PointCloud<pcl::RGB> rgb;
        convertProbToRGB(prob_host, i, rgb);
        savePNGFile(make_ext_png_name(counter_,i, "hist1"), rgb);

        //pcl::io::savePCDFileASCII (make_ext_pcd_name(counter_,i, "hist1"), rgb);
      }

      // and now again for the second iteration
      pcl::PointCloud<pcl::device::prob_histogram> prob_host2(people_detector_.rdf_detector_->P_l_2_.cols(), people_detector_.rdf_detector_->P_l_2_.rows());
      people_detector_.rdf_detector_->P_l_2_.download(prob_host2.points, c);
      prob_host.width = people_detector_.rdf_detector_->P_l_2_.cols();
      prob_host.height = people_detector_.rdf_detector_->P_l_2_.rows();

      PCL_DEBUG("[PeoplePCDApp::writeProb] : (D) : savePNGFile histogram2\n");
      for(int i = 0; i < pcl::gpu::people::NUM_LABELS; i++)
      {
        pcl::PointCloud<pcl::RGB> rgb;
        convertProbToRGB(prob_host2, i, rgb);
        savePNGFile(make_ext_png_name(counter_, i, "hist2"), rgb);
      }

      // and now again for the Gaus test
      pcl::PointCloud<pcl::device::prob_histogram> prob_host_gaus(people_detector_.rdf_detector_->P_l_Gaus_.cols(), people_detector_.rdf_detector_->P_l_Gaus_.rows());
      people_detector_.rdf_detector_->P_l_Gaus_.download(prob_host_gaus.points, c);
      prob_host_gaus.width = people_detector_.rdf_detector_->P_l_Gaus_.cols();
      prob_host_gaus.height = people_detector_.rdf_detector_->P_l_Gaus_.rows();

      PCL_DEBUG("[PeoplePCDApp::writeProb] : (D) : savePNGFile gaus\n");
      for(int i = 0; i < pcl::gpu::people::NUM_LABELS; i++)
      {
        pcl::PointCloud<pcl::RGB> rgb;
        convertProbToRGB(prob_host_gaus, i, rgb);
        savePNGFile(make_ext_png_name(counter_, i, "gaus"), rgb);
      }

      // and now again for the Gaus Temp
      pcl::PointCloud<pcl::device::prob_histogram> prob_host_gaus_temp(people_detector_.rdf_detector_->P_l_Gaus_Temp_.cols(), people_detector_.rdf_detector_->P_l_Gaus_Temp_.rows());
      people_detector_.rdf_detector_->P_l_Gaus_Temp_.download(prob_host_gaus_temp.points, c);
      prob_host_gaus_temp.width = people_detector_.rdf_detector_->P_l_Gaus_Temp_.cols();
      prob_host_gaus_temp.height = people_detector_.rdf_detector_->P_l_Gaus_Temp_.rows();

      PCL_DEBUG("[PeoplePCDApp::writeProb] : (D) : savePNGFile gaus temp\n");
      for(int i = 0; i < pcl::gpu::people::NUM_LABELS; i++)
      {
        pcl::PointCloud<pcl::RGB> rgb;
        convertProbToRGB(prob_host_gaus_temp, i, rgb);
        savePNGFile(make_ext_png_name(counter_, i, "gaus_temp"), rgb);
      }

      // and now again for the Org Planes
      pcl::PointCloud<pcl::device::prob_histogram> prob_host_plane(people_detector_.org_plane_detector_->P_l_dev_.cols(), people_detector_.org_plane_detector_->P_l_dev_.rows());
      people_detector_.org_plane_detector_->P_l_dev_.download(prob_host_plane.points, c);
      prob_host_plane.width = people_detector_.rdf_detector_->P_l_Gaus_Temp_.cols();
      prob_host_plane.height = people_detector_.rdf_detector_->P_l_Gaus_Temp_.rows();

      PCL_DEBUG("[PeoplePCDApp::writeProb] : (D) : savePNGFile planes\n");
      for(int i = 0; i < pcl::gpu::people::NUM_LABELS; i++)
      {
        pcl::PointCloud<pcl::RGB> rgb;
        convertProbToRGB(prob_host_plane, i, rgb);
        savePNGFile(make_ext_png_name(counter_, i, "plane"), rgb);
      }

    }

    int counter_;
    int processReturn;
    PeopleDetector people_detector_;
    PeopleDetector::Image cmap_device_;

    ImageViewer final_view_;
    ImageViewer image_view_;

    pcl::gpu::DeviceArray<pcl::RGB> color_map_;
};

void print_help()
{
  PCL_INFO("\nPeople tracking app options (help):\n");
  PCL_INFO("\t -numTrees \t<int> \tnumber of trees to load\n");
  PCL_INFO("\t -tree0 \t<path_to_tree_file>\n");
  PCL_INFO("\t -tree1 \t<path_to_tree_file>\n");
  PCL_INFO("\t -tree2 \t<path_to_tree_file>\n");
  PCL_INFO("\t -tree3 \t<path_to_tree_file>\n");
  PCL_INFO("\t -prob  \t<bool> \tsave prob files or not, default 0\n");
  PCL_INFO("\t -debug \t<bool> \tset debug output\n");
  PCL_INFO("\t -pcd   \t<path_to_pcd_file>\n");
  PCL_INFO("\t -XML   \t<path_to_XML_file> \tcontains person specifics, defaults to generic.xml\n");
}

int main(int argc, char** argv)
{
  PCL_INFO("[Main] : (I) : People tracking on PCD files version 0.3\n");
  if(find_switch (argc, argv, "--help") || find_switch (argc, argv, "-h"))
    return print_help(), 0;

  bool saveProb = 1;
  parse_argument (argc, argv, "-prob", saveProb);

  bool debugOutput = 1;
  parse_argument (argc, argv, "-debug", debugOutput);
  if(debugOutput)
    setVerbosityLevel(L_DEBUG);

  std::string treeFilenames[4] = 
  {
      "d:/TreeData/results/forest1/tree_20.txt",
      "d:/TreeData/results/forest2/tree_20.txt",
      "d:/TreeData/results/forest3/tree_20.txt",
      "d:/TreeData/results/forest3/tree_20.txt"
  };
  int numTrees = 4;
  parse_argument (argc, argv, "-numTrees", numTrees);
  parse_argument (argc, argv, "-tree0", treeFilenames[0]);
  parse_argument (argc, argv, "-tree1", treeFilenames[1]);
  parse_argument (argc, argv, "-tree2", treeFilenames[2]);
  parse_argument (argc, argv, "-tree3", treeFilenames[3]);

  std::string XMLfilename("/data/git/pcl/gpu/people/data/generic.xml");
  parse_argument (argc, argv, "-XML", XMLfilename);
  PCL_DEBUG("[Main] : (D) : Will read XML %s\n", XMLfilename.c_str());

  if (numTrees == 0 || numTrees > 4)
  {
    PCL_ERROR("[Main] : (E) : Main : Invalid number of trees\n");
    return -1;
  }
  PCL_DEBUG("[Main] : (D) : Read %d Trees\n", numTrees);

  // loading trees
  using pcl::gpu::people::RDFBodyPartsDetector;

  vector<string> names_vector(treeFilenames, treeFilenames + numTrees);
  PCL_DEBUG("[Main] : (D) : Trees collected\n");
  RDFBodyPartsDetector::Ptr rdf(new RDFBodyPartsDetector(names_vector));
  PCL_DEBUG("[Main] : (D) : Loaded files into rdf\n");

  // Create the app
  PeoplePCDApp app;
  PCL_DEBUG("[Main] : (D) : App created\n");
  app.people_detector_.rdf_detector_ = rdf;

  // Read in person specific configuration
  app.readXMLFile(XMLfilename);
  PCL_DEBUG("[Main] : (D) : Person configuration filename %s\n", XMLfilename.c_str());

  string pcd_file, pcd_folder;
  vector<string> pcd_files;

  // See if we can find a folder or a PCD file to load
  try
  {
    if (pcl::console::parse_argument (argc, argv, "-pcd", pcd_file) > 0)
    {
      if (boost::filesystem::is_regular_file(pcd_file) )
      {
        if (boost::filesystem::extension(boost::filesystem::path(pcd_file)) == ".pcd")
        {
          pcd_files.push_back(pcd_file);
          PCL_DEBUG("[Main] : (D) : Will read %s\n", pcd_file.c_str());
        }
        else
        {
          PCL_ERROR("[Main] : (E) : Not correct .pcd extension\n");
          return -1;
        }
      }
      else
      {
        PCL_ERROR("[Main] : (E) : Not correct file type given\n");
        return -1;
      }
    }
    else if (pcl::console::parse_argument (argc, argv, "-pcd_folder", pcd_folder) > 0)
    {
      pcd_files = getPcdFilesInDir(pcd_folder);
      PCL_DEBUG("[Main] : (D) : Indexed %d files\n", pcd_files.size());
    }
  }
  catch (const pcl::PCLException& /*e*/)
  {
    return PCL_ERROR ("[Main] : (E) : Can't open depth source\n"), -1;
  }

  std::sort(pcd_files.begin(), pcd_files.end());

  // Now iterate over the pcd files
  for(unsigned int i = 0; i < pcd_files.size(); i++)
  {
    // loading cloud file
    pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    int res = pcl::io::loadPCDFile<PointT> (pcd_files[i], *cloud);

    if (res == -1) //* load the file
    {
      PCL_ERROR("[Main] : (E) : Couldn't read cloud file\n");
      return res;
    }

    /// Run the app
    {
      pcl::ScopeTime frame_time("[Main] : (I) : frame_time");
      app.cloud_cb(cloud);
    }
    if(app.processReturn == 1)
    {
      PCL_DEBUG("[Main] : (D) : Calling visualisation and write return 1\n");
      app.visualizeAndWrite(cloud);
      if(saveProb)
      {
        PCL_DEBUG("[Main] : (D) : Saving probabilities\n");
        app.writeProb(cloud);
      }
    }
    else if(app.processReturn == 2)
    {
      PCL_DEBUG("[Main] : (D) : Calling visualisation and write return 2\n");
      app.visualizeAndWrite(cloud);
      if(saveProb)
      {
        PCL_DEBUG("[Main] : (D) : Saving probabilities\n");
        app.writeProb(cloud);
      }
    }
    else
    {
      PCL_DEBUG("[Main] : (D) : No good person found\n");
    }
  }
  return 0;
}
