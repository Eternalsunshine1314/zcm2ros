#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/NavSatFix.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/Quaternion.h>
#include <image_transport/image_transport.h>

#include <zcm2ros/RadarData.h>
#include <zcm2ros/RadarInfo.h>
#include <zcm2ros/RadarDetection.h>
#include <zcm2ros/RadarDetectionArray.h>
#include <zcm2ros/RadarTrack.h>
#include <zcm2ros/RadarTrackArray.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <zcm/zcm-cpp.hpp>
#include "zcm_types/camera_basler/cpp_types/ZcmCameraBaslerJpegFrame.hpp"
#include "zcm_types/lidar_scala/cpp_types/ZcmLidarScalaDataScan.hpp"
#include "zcm_types/lidar_scala/cpp_types/ZcmLidarScalaObjectScan.hpp"
#include "zcm_types/radar_conti/cpp_types/ZcmRadarContiObjectsScan.hpp"
#include "zcm_types/nav_gw/cpp_types/ZcmNavNV08CNavGw.hpp"
#include "zcm_types/stereo_pair/cpp_types/ZcmStereoPairReconstruction.hpp"
#include "zcm_types/track_detector/cpp_types/ZcmTrackDetectTracksScan.hpp"
#include "zcm_types/track_detector/cpp_types/ZcmTrackDetectTrack.hpp"

#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/Geodesic.hpp>

#include <ibeo_core/ibeo_core.h>
#include <cmath>

#include <zcm/zcm-cpp.hpp>
#include "vapplication.h"
#include "config.h"
#include "vlog_pretty.h"
#include "vsyssignal.h"
#include "vthread.h"
#include "tracks_pack_collector.h"
#include "tracks_pack_processor.h"

ros::Publisher rosLidarDataPublisher, rosStereoPairPublisher, rosRadarMarkerPublisher, rosTrackPublisher, rosRadarDataPublisher, rosNavigationPublisher, rosLidarMarkerPublisher;
image_transport::Publisher rosJpegPublisher, rosBirdViewPublisher, rosLidarClusterPublisher, rosStereoClusterPublisher;

sensor_msgs::CompressedImage rosJpegFrame;

static constexpr auto PID_PATH = "/tmp/niias";
static constexpr double scaleY =  atan(0.25 * M_PI / 180);
static constexpr double scaleZ = atan(0.8 * M_PI / 180);

static cv::Mat copy_from_cam_mtx( const float cam_mtx[3][3] ) {
    cv::Mat res( cv::Size(3,3), CV_32FC1 );

    for ( int i = 0; i < 3; ++i ){
        for ( int j = 0; j < 3; ++j ){
            res.at<float>(i, j) = cam_mtx[i][j];
        }
    }
    return res;
}

static cv::Mat inv_persp(const cv::Mat& src, const ZcmCameraBaslerJpegFrame *msg) {
    constexpr int SCALE = 20;

    cv::Mat K = copy_from_cam_mtx(msg->info.calibrating_params.cam_mtx);

    cv::Mat distCoeff = (cv::Mat_<double>(5, 1) <<
                                        0.0, 0.0, 0.0, 0.0, 0.0);

    cv::Mat tvec = (cv::Mat_<double>(3, 1) <<   msg->info.calibrating_params.tvec[0],
                                                msg->info.calibrating_params.tvec[1],
                                                msg->info.calibrating_params.tvec[2] );

    cv::Mat rvec = (cv::Mat_<double>(3, 1) <<   msg->info.calibrating_params.rvec[0],
                                                msg->info.calibrating_params.rvec[1],
                                                msg->info.calibrating_params.rvec[2] );


    const int roi_left = -10;
    const int roi_right = 10;
    const float roi_near = 14.0;
    const int length = 100;

    // массив точек объекта objectPoints 3D
    std::vector<cv::Point3f> objectPoints;
    objectPoints.push_back(cv::Point3f(roi_left, 0, roi_near));
    objectPoints.push_back(cv::Point3f(roi_right, 0, roi_near));
    objectPoints.push_back(cv::Point3f(roi_left, 0, roi_near + length));
    objectPoints.push_back(cv::Point3f(roi_right, 0, roi_near + length));

    // cv::projectPoints -> массив точек на изображении imgPoints 2D
    std::vector<cv::Point2f> imgPoints;

    projectPoints(objectPoints, rvec, tvec, K, distCoeff, imgPoints);

    // The 4 points where the mapping is to be done
    const int out_width = SCALE * fabs(roi_left) + SCALE * roi_right;
    const int out_height = SCALE * length;

    std::vector<cv::Point2f> pts2;
    pts2.push_back(cv::Point2f(0, out_height));
    pts2.push_back(cv::Point2f(out_width, out_height));
    pts2.push_back(cv::Point2f(0, 0));
    pts2.push_back(cv::Point2f(out_width,0));

    // Get the Perspective Transform Matrix
    cv::Mat M(cv::Size(2, 4), CV_32FC1);
    M = getPerspectiveTransform(imgPoints, pts2);

    // Apply the Perspective Transform just found to the src image
    cv::Mat out(cv::Size(out_width, out_height), CV_8UC1);
    cv::warpPerspective(src, out, M, out.size());

    // *tm = M;             // матрица преобразования исходного изображения в обратную перспективу
    return out;          // изображение в обратной перспективе
}

static cv::Mat mergeCluster2Img(const ZcmCameraBaslerJpegFrame *msg, const ZcmLidarClustersList *clusterMsg ) {
    // Inverse Perspective Mapping

    cv::Mat frontViewMat = cv::imdecode(msg->jpeg, CV_LOAD_IMAGE_COLOR);

    if (clusterMsg->clustersCount < 1) return frontViewMat;

    // Decompose the projection matrix into:
    // intrinsic parameter matrix
    cv::Mat K = copy_from_cam_mtx(msg->info.calibrating_params.cam_mtx);

    cv::Mat distCoeff = (cv::Mat_<double>(5, 1) <<
                                                0.0, 0.0, 0.0, 0.0, 0.0);

    cv::Mat tvec = (cv::Mat_<double>(3, 1) <<   msg->info.calibrating_params.tvec[0],
            msg->info.calibrating_params.tvec[1],
            msg->info.calibrating_params.tvec[2] );

    cv::Mat rvec = (cv::Mat_<double>(3, 1) <<   msg->info.calibrating_params.rvec[0],
            msg->info.calibrating_params.rvec[1],
            msg->info.calibrating_params.rvec[2] );



    std::vector<cv::Point3f> objectPoints;


    for (auto cluster : clusterMsg->scanClusters) {

        objectPoints.push_back(cv::Point3f(-1 * cluster.bounding_rect.bottom_left_y, 0.0, cluster.bounding_rect.bottom_left_x));
        objectPoints.push_back(cv::Point3f(-1 * cluster.bounding_rect.bottom_right_y, 0.0, cluster.bounding_rect.bottom_right_x));
        objectPoints.push_back(cv::Point3f(-1 * cluster.bounding_rect.top_left_y, 0.0, cluster.bounding_rect.top_left_x));
        objectPoints.push_back(cv::Point3f(-1 * cluster.bounding_rect.top_right_y, 0.0, cluster.bounding_rect.top_right_x));

        objectPoints.push_back(cv::Point3f(-1 * cluster.bounding_rect.bottom_left_y, -2.0, cluster.bounding_rect.bottom_left_x));
        objectPoints.push_back(cv::Point3f(-1 * cluster.bounding_rect.bottom_right_y, -2.0,  cluster.bounding_rect.bottom_right_x));
        objectPoints.push_back(cv::Point3f(-1 * cluster.bounding_rect.top_left_y, -2.0, cluster.bounding_rect.top_left_x));
        objectPoints.push_back(cv::Point3f(-1 * cluster.bounding_rect.top_right_y, -2.0, cluster.bounding_rect.top_right_x));
    }

    // cv::projectPoints -> массив точек на изображении imgPoints 2D
    std::vector<cv::Point2f> clustersListPoints;

    projectPoints(objectPoints, rvec, tvec, K, distCoeff, clustersListPoints);

    cv::Mat clustersMat = cv::Mat(clustersListPoints);


    cv::Mat greyMat, rangeMat, mergedMat, newMat, resultMat;

//    resultMat = cv::Mat::zeros(frontViewMat.size(), frontViewMat.type());
    frontViewMat.copyTo(resultMat);
    for (int i = 0; i < clustersListPoints.size(); i += 8) {

        bool inFrame = true;
        for (int j = 0; j < 8; j ++) {
            if( clustersListPoints[i+j].y < 0    || clustersListPoints[i+j].x < 0   ||
                clustersListPoints[i+j].y > 2000 || clustersListPoints[i+j].x > 2000 ) {
                inFrame = false;
                break;
            }

            cv::circle(resultMat, clustersListPoints[i+j], 2, cv::Scalar(0, 0, 255), cv::FILLED);
        }
        if (!inFrame) continue;

        for (int j = 0; j < 2; j ++) {
            cv::line(resultMat, clustersListPoints[i + j*4], clustersListPoints[i+1 + j*4], cv::Scalar(0, 0, 255), 2);
            cv::line(resultMat, clustersListPoints[i+1 + j*4], clustersListPoints[i+2 + j*4], cv::Scalar(0, 0, 255), 2);
            cv::line(resultMat, clustersListPoints[i+2 + j*4], clustersListPoints[i+3 + j*4], cv::Scalar(0, 0, 255), 2);
            cv::line(resultMat, clustersListPoints[i+3 + j*4], clustersListPoints[i + j*4], cv::Scalar(0, 0, 255), 2);
        }

        for (int j = 0; j < 4; j ++) {
            cv::line(resultMat, clustersListPoints[i + j], clustersListPoints[i + j + 4], cv::Scalar(0, 0, 255), 2);
        }

    }

    return resultMat;
}

Config readConfig(VApplication& app) {

    //  Print conf & exit
    if ( app.args().has_flag("--printconf") )
    {
        std::cout << Config::get() << std::endl;
    }

    //  Reading config
    if ( !app.args().has_flag("-c") )
        vwarning << "Cannot find param '-c' configname, using default (%).";
    auto confname = app.args().take_std_value_or("-c", "%");
    if (confname == "%")
        confname = app.full_app_name() + ".conf";
    auto config = Config::load( confname );

    //  Storing PID into filename
    if ( !app.args().has_flag("-p") )
        vwarning << "Cannot find param '-p' pidname, using default (%).";
    auto pidname = app.args().take_std_value_or("-p", "%");
    if (pidname == "%")
        pidname = app.app_name() + ".pid";
    app.pid().store( PID_PATH, pidname );

    return config;
}

class HandlerTest {

public:
    ~HandlerTest() {}

    void stereoPairHandler(const zcm::ReceiveBuffer *, const std::string &chan, const ZcmStereoPairReconstruction *msg) {
        sensor_msgs::PointCloud pointCloud;
        geometry_msgs::Point32 pointCloudPoint;

        pointCloud.header.frame_id = "stereo_pair_pointcloud_frame";

        for (auto point : msg->points) {
            pointCloudPoint.x = point.x;
            pointCloudPoint.y = point.y;
            pointCloudPoint.z = point.z;
            pointCloud.points.push_back(pointCloudPoint);
        }
        std::cout << "StereoPair cloud sent." << std::endl;
        rosStereoPairPublisher.publish(pointCloud);
    }

    void baslerJpegHandler(const zcm::ReceiveBuffer *, const std::string &chan, const ZcmCameraBaslerJpegFrame *msg) {
        rosJpegFrame.format = "jpeg compressed";
        rosJpegFrame.data = msg->jpeg;

        uint32_t secs = msg->timestamp_ns / 1000000000ULL;
        uint32_t nanos = msg->timestamp_ns - (secs * 1000000000ULL);
        rosJpegFrame.header.stamp.nsec = nanos;
        rosJpegFrame.header.stamp.sec = secs;

        cv::Mat frontViewImage = cv::imdecode(msg->jpeg, CV_LOAD_IMAGE_COLOR);
        cv::Mat birdViewImage = inv_persp(frontViewImage, msg);

        cv::transpose(birdViewImage, birdViewImage);
        cv::rotate(birdViewImage, birdViewImage, 1);

        sensor_msgs::ImagePtr frontViewMsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frontViewImage).toImageMsg();
        sensor_msgs::ImagePtr birdViewMsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", birdViewImage).toImageMsg();

        rosJpegPublisher.publish(frontViewMsg);
        rosBirdViewPublisher.publish(birdViewMsg);
    }

    void lidarDataHandler(const zcm::ReceiveBuffer *, const std::string &chan, const ZcmLidarScalaDataScan *msg) {

        unsigned short id = 0;
        visualization_msgs::MarkerArray markerArray, clearArray;
        visualization_msgs::Marker marker;

        marker.header.frame_id = "lidar_marker_frame";
        marker.ns = "lidar_namespace";
        marker.color.r = 0.5;
        marker.color.g = 1.0;
        marker.color.b = 0.5;
        marker.color.a = 1.0;

        marker.action = visualization_msgs::Marker::DELETEALL;
        clearArray.markers.push_back(marker);
        rosLidarMarkerPublisher.publish(clearArray);

        sensor_msgs::PointCloud pointCloud;
        geometry_msgs::Point32 pointCloudPoint;

        pointCloud.header.frame_id = "lidar_pointcloud_frame";

        uint32_t nanos = msg->service.u_timestamp * 1000;
        uint32_t secs = nanos / 1000000000ULL;
        nanos -= (secs * 1000000000ULL);

        for (auto point : msg->scanPoints) {
            pointCloudPoint.x = point.x;
            pointCloudPoint.y = point.y;
            pointCloudPoint.z = point.z;
            pointCloud.points.push_back(pointCloudPoint);

            marker.id = id++;
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.action = visualization_msgs::Marker::ADD;

            marker.pose.position.x = point.x;
            marker.pose.position.y = point.y;
            marker.pose.position.z = point.z;

            marker.scale.x = point.echoPulseWidth;
            marker.scale.y = scaleY * point.radialDistance;
            marker.scale.z = scaleZ * point.radialDistance;

            markerArray.markers.push_back(marker);

        }
        rosLidarDataPublisher.publish(pointCloud);
        rosLidarMarkerPublisher.publish(markerArray);
    }

    void radarDataHandler(const zcm::ReceiveBuffer *, const std::string &chan, const ZcmRadarContiObjectsScan *msg) {

        zcm2ros::RadarDetectionArray detectionArray;
        zcm2ros::RadarDetection detection;
        zcm2ros::RadarTrackArray trackArray;
        zcm2ros::RadarTrack track;
        zcm2ros::RadarData radarData;
        zcm2ros::RadarInfo radarInfo;
        std::vector<zcm2ros::RadarInfo> infoArray;

        visualization_msgs::MarkerArray markerArray, clearArray;
        visualization_msgs::Marker marker;

        marker.header.frame_id = "radar_marker_frame";
        marker.ns = "radar_namespace";

        marker.action = visualization_msgs::Marker::DELETEALL;
        clearArray.markers.push_back(marker);
        rosRadarMarkerPublisher.publish(clearArray);

        unsigned short id = 0;
        for (auto point : msg->objects) {

            visualization_msgs::Marker  arrow;
            marker.id = id;
            marker.type = visualization_msgs::Marker::CUBE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = point.general.dist_long - point.extended.obj_len/2;
            marker.pose.position.y = point.general.dist_lat;
            marker.pose.position.z = 0.5;
            marker.scale.x = point.extended.obj_len;
            marker.scale.y = point.extended.obj_width;
            marker.scale.z = 1;
            marker.color.a = 0.2;
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 1.0;

            markerArray.markers.push_back(marker);

            arrow.header.frame_id = "radar_marker_frame";
            arrow.ns = "radar_namespace";
            arrow.action = visualization_msgs::Marker::ADD;
            arrow.color.a = 1.0;
            arrow.color.r = 0.0;
            arrow.color.g = 1.0;
            arrow.color.b = 1.0;
            arrow.type = visualization_msgs::Marker::ARROW;
            arrow.id = id+1;
            arrow.scale.x = 0.1;
            arrow.scale.y = 0.3;
            //arrow.scale.z = 1.0;
            arrow.color.r = 1.0;
            arrow.color.g = 0.3;
            arrow.color.b = 0.0;

            geometry_msgs::Point start, end;

            start.z = 0.5;
            start.y = marker.pose.position.y;
            start.x = marker.pose.position.x;

            end.z = 0.5;
            end.x += start.x + point.general.vrel_long ;
            end.y += start.y + point.general.vrel_lat ;

            arrow.points.push_back(start);
            arrow.points.push_back(end);

            markerArray.markers.push_back(arrow);

            // -------------------------------------------------------------

            detection.position.x = point.general.dist_long;
            detection.position.y = point.general.dist_lat;
            detection.position.z = 0;
            detection.velocity.x = point.general.vrel_long;
            detection.velocity.y = point.general.vrel_lat;
            detection.velocity.z = 0;
            detection.detection_id = id;
            detection.amplitude = point.general.rcs;

            detectionArray.detections.push_back(detection);

            track.track_id = id;
            track.linear_acceleration.x = point.extended.arel_long;
            track.linear_acceleration.y = point.extended.arel_lat;
            track.linear_acceleration.z = 0;
            track.linear_velocity.x = point.general.vrel_long;
            track.linear_velocity.y = point.general.vrel_lat;
            track.linear_velocity.z = 0;

            trackArray.tracks.push_back(track);

            radarInfo.id = id;
            radarInfo.rcs = point.general.rcs;
            radarInfo.dist_lat = point.general.dist_lat;
            radarInfo.dist_long = point.general.dist_long;
            radarInfo.arel_lat = point.extended.arel_lat;
            radarInfo.arel_long = point.extended.arel_long;
            radarInfo.vrel_lat = point.general.vrel_lat;
            radarInfo.vrel_long = point.general.dist_long;
            radarInfo.obj_width = point.extended.obj_width;
            radarInfo.obj_len = point.extended.obj_len;
            radarInfo.obj_class = point.extended.obj_class;
            radarInfo.areal_long_rms = point.quality.areal_long_rms;
            radarInfo.arel_lat_rms = point.quality.arel_lat_rms;
            radarInfo.dist_long_rms = point.quality.dist_long_rms;
            radarInfo.dist_lat_rms = point.quality.dist_lat_rms;
            radarInfo.orientation_rms = point.quality.orientation_rms;
            radarInfo.prob_of_exist = point.quality.prob_of_exist;
            radarInfo.vrel_lat_rms = point.quality.vrel_lat_rms;
            radarInfo.vrel_long_rms = point.quality.vrel_long_rms;
            radarInfo.dyn_prop = point.general.dyn_prop;
            radarInfo.meas_state = point.quality.meas_state;
            radarInfo.orientation_angel = point.extended.orientation_angel;

            infoArray.push_back(radarInfo);

            id += 2;
        }

        detectionArray.header.frame_id = "radar_detection_array_frame";
        trackArray.header.frame_id = "radar_track_array_frame";

        radarData.detections = detectionArray;
        radarData.tracks = trackArray;
        radarData.info = infoArray;
        radarData.header.frame_id = "radar_data";
        uint32_t nanos = msg->service.u_timestamp * 1000;
        uint32_t secs = nanos / 1000000000ULL;
        nanos -= (secs * 1000000000ULL);
        radarData.header.stamp.sec = secs;
        radarData.header.stamp.nsec = nanos;

        rosRadarDataPublisher.publish(radarData);
        rosRadarMarkerPublisher.publish(markerArray);
    }

    void navigationDataHandler(const zcm::ReceiveBuffer *, const std::string &chan, const ZcmNavNV08CNavGw *msg) {
        sensor_msgs::NavSatFix rosNavigationMsg;

        GeographicLib::Math::real  x = msg->x;
        GeographicLib::Math::real  y = msg->y;
        GeographicLib::Math::real  z = msg->z;
        GeographicLib::Math::real lat, lon, h;

        GeographicLib::Geocentric geocentric(GeographicLib::Constants::WGS84_a(), GeographicLib::Constants::WGS84_f());

        geocentric.Reverse( x , y, z, lat, lon, h );

        rosNavigationMsg.longitude = lon;
        rosNavigationMsg.latitude = lat;
        rosNavigationMsg.altitude = h;
        rosNavigationMsg.header.frame_id = "navigation_frame";

        uint32_t nanos = msg->service.u_timestamp * 1000;
        uint32_t secs = nanos / 1000000000ULL;
        nanos -= (secs * 1000000000ULL);
        rosNavigationMsg.header.stamp.sec = secs;
        rosNavigationMsg.header.stamp.nsec = nanos;

        rosNavigationPublisher.publish(rosNavigationMsg);
    }

    void trackHandler(const zcm::ReceiveBuffer *, const std::string &chan, const ZcmTrackDetectTracksScan *msg) {
        std::vector<ZcmTrackDetectTrack> tracks = msg->track;

        visualization_msgs::MarkerArray markerArray, clearArray;
        visualization_msgs::Marker marker;

        unsigned short id = 0;

        marker.header.frame_id = "track_marker_frame";
        marker.ns = "track_namespace";

        marker.action = visualization_msgs::Marker::DELETEALL;
        clearArray.markers.push_back(marker);
        rosTrackPublisher.publish(clearArray);

        for (auto track : tracks) {
            marker.id = id++;
            marker.type = visualization_msgs::Marker::LINE_LIST;
            marker.action = visualization_msgs::Marker::ADD;

            marker.scale.x = 0.1;

            marker.color.r = 0.5;
            marker.color.g = 1.0;
            marker.color.b = 0.5;
            marker.color.a = 1.0;

            geometry_msgs::Point gPoint;
            gPoint.z = 0.0;

            for (auto i = 1; i < track.point.size(); i++) {
                gPoint.x = track.point[i-1].x;
                gPoint.y = track.point[i-1].y;
                marker.points.push_back(gPoint);

                gPoint.x = track.point[i].x;
                gPoint.y = track.point[i].y;
                marker.points.push_back(gPoint);

                markerArray.markers.push_back(marker);
            }

        }

        rosTrackPublisher.publish(markerArray);
    }
};

void collectJpegAndClusters(Tracks_Pack_Collector &collector, Tracks_Pack_Processor &processor, image_transport::Publisher &rosPublisher) {
    //  Новый пакет отправляем в поток на обработку, заобдно проверяем забитость очереди.
    collector.received.connect( [&processor, &rosPublisher](const Tracks_Pack& pack)
                                {
                                    auto clusterMat = mergeCluster2Img(&pack.jpegFrame, &pack.clustersList);

                                    sensor_msgs::ImagePtr jpegClusterMsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8",clusterMat).toImageMsg();
                                    rosPublisher.publish(jpegClusterMsg);

                                    processor.on_received(pack);
                                });
}

int main( int argc, char ** argv )
{
    VApplication app( argc, argv );
    auto config = readConfig(app);

    ros::init(argc, argv, "zcm2ros_publisher");
    ros::NodeHandle nodeHandler;
    image_transport::ImageTransport imageTransportHandler(nodeHandler);

    zcm::ZCM Zcm( config.zcm_receive.target );
    if ( !Zcm.good() )
        std::cout << "Can't init ZCM with target"  << std::endl;

    HandlerTest hand;

    Zcm.subscribe( "FLZcmCameraBaslerJpegFrame", &HandlerTest::baslerJpegHandler, &hand );
    rosBirdViewPublisher = imageTransportHandler.advertise("bird_view", 1 );
    rosJpegPublisher = imageTransportHandler.advertise("front_view", 1 );

    Zcm.subscribe( "FrontCenterLidarScalaScanData", &HandlerTest::lidarDataHandler, &hand );
    rosLidarDataPublisher = nodeHandler.advertise<sensor_msgs::PointCloud>("lidar_point_cloud", 1 );
    rosLidarMarkerPublisher = nodeHandler.advertise<visualization_msgs::MarkerArray>("lidar_marker_array", 1 );

    Zcm.subscribe( "ZcmStReconstruction", &HandlerTest::stereoPairHandler, &hand );
    rosStereoPairPublisher = nodeHandler.advertise<sensor_msgs::PointCloud>("stereo_pair_cloud", 1 );

    Zcm.subscribe( "FrontLeftContiObjectScan", &HandlerTest::radarDataHandler, &hand );
    rosRadarDataPublisher = nodeHandler.advertise<zcm2ros::RadarData>("radar_data_array", 1 );
    rosRadarMarkerPublisher = nodeHandler.advertise<visualization_msgs::MarkerArray>("radar_marker_array", 1 );

    Zcm.subscribe( "ZcmNavNV08CNavGw", &HandlerTest::navigationDataHandler, &hand );
    rosNavigationPublisher = nodeHandler.advertise<sensor_msgs::NavSatFix>("navigation_data", 1 );

    Zcm.subscribe( "FZcmTrackFusion", &HandlerTest::trackHandler, &hand );
    rosTrackPublisher = nodeHandler.advertise<visualization_msgs::MarkerArray>("track_marker_array", 1 );

    rosLidarClusterPublisher = imageTransportHandler.advertise("jpeg_cluster_view", 1 );
    rosStereoClusterPublisher = imageTransportHandler.advertise("jpeg_stereo_cluster_view", 1 );

    Tracks_Pack_Collector jpegLidarCollector( &Zcm, config.zcm_receive );

    config.zcm_receive.lidar_cluster_channel = config.zcm_receive.lidar_cluster_channel2;
    Tracks_Pack_Collector jpegStereoCollector( &Zcm, config.zcm_receive );

    Tracks_Pack_Processor processor;

    collectJpegAndClusters(jpegLidarCollector, processor, rosLidarClusterPublisher);
    collectJpegAndClusters(jpegStereoCollector, processor, rosStereoClusterPublisher);

    Zcm.run();

}