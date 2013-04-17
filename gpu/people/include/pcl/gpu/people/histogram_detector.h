/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Open Perception
 *  Copyright (c) 2013, KU Leuven
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
 * @author: Koen Buys
 */


#ifndef PCL_GPU_PEOPLE_HISTOGRAM_DETECTOR_H
#define PCL_GPU_PEOPLE_HISTOGRAM_DETECTOR_H

#include <pcl/pcl_exports.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <pcl/gpu/people/label_detector.h>
#include <pcl/common/statistics/statistics.h>

namespace pcl
{
  namespace gpu
  {
    namespace people
    {
      class HistogramDetector : public LabelDetector
      {
        public:
          typedef boost::shared_ptr<HistogramDetector> Ptr;
          typedef pcl::PointXYZRGBA               PointTC;
          typedef DeviceArray2D<unsigned char>    Labels;

          HistogramStatistics<PointTC>::Ptr histogram_host;

          /** \brief Default Histogram Detector Constructor */
          HistogramDetector();

          void
          process(const PointCloud<PointTC>::ConstPtr &cloud, Labels labels);
      };
    }
  }
}

#endif /* PCL_GPU_PEOPLE_HISTOGRAM_DETECTOR_H */
