#ifndef ZCM_DEPLOY_H
#define ZCM_DEPLOY_H

#include "vtimepoint.h"

#include "zcm_types/camera_basler/cpp_types/ZcmCameraBaslerJpegFrame.hpp"
#include "zcm_types/lidar_cluster/cpp_types/ZcmLidarClustersList.hpp"
#include "zcm_types/lidar_cluster/cpp_types/ZcmLidarCluster.hpp"

using ZCM_BaslerJpeg = ZcmCameraBaslerJpegFrame;
using ZCM_LidarClustersList = ZcmLidarClustersList;


struct Tracks_Pack
{
    ZCM_BaslerJpeg jpegFrame;
    ZCM_LidarClustersList clustersList;

    VTimePoint processing_begin;
};



#endif // ZCM_DEPLOY_H
