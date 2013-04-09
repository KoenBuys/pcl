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


#ifndef PCL_GPU_PEOPLE_LABEL_DETECTOR_H
#define PCL_GPU_PEOPLE_LABEL_DETECTOR_H

#include <pcl/pcl_exports.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/gpu/containers/device_array.h>
#include <pcl/gpu/people/colormap.h>
#include <pcl/gpu/people/label_blob2.h>
#include <pcl/gpu/people/label_common.h>
#include "pcl/gpu/people/person_attribs.h"
#include <boost/shared_ptr.hpp>
#include <string>
#include <vector>

namespace pcl
{
  namespace gpu
  {
    namespace people
    {
      /**
       * \brief This implements an Abstract Interface Class of which the other detectors will inherit
       */

      class LabelDetector
      {
        public:
          typedef pcl::PointCloud<pcl::device::prob_histogram>  HostLabelProbability;

          LabelDetector()
          {
            //allocate_buffers();
          }

          LabelDetector(int cols, int rows) : rows_(rows), cols_(cols)
          {
            //allocate_buffers(rows, cols);
          }

          virtual void process() =0;
          //void allocate_buffers(int rows = 480, int cols = 640);

          HostLabelProbability                    P_l_host_;        // This is a HOST histogram!
          HostLabelProbability                    P_l_host_prev_;

          pcl::device::LabelProbability           P_l_dev_;         // This is a DEVICE histogram!
          pcl::device::LabelProbability           P_l_dev_prev_;

          int                                     rows_;                // should default to 480
          int                                     cols_;                // should default to 640
      };
    }
  }
}


#endif /* PCL_GPU_PEOPLE_LABEL_DETECTOR_H */
