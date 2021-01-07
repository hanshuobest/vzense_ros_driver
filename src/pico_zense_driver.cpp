#include <sys/stat.h>
#include "Vzense_enums.h"
#include "pico_zense_driver.h"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include <sensor_msgs/CameraInfo.h>
#include <chrono>
#include <ctime>

class Timer
{
    using Clock = std::chrono::high_resolution_clock;

public:
    /*! \brief start or restart timer */
    inline void Tic()
    {
        start_ = Clock::now();
    }
    /*! \brief stop timer */
    inline void Toc()
    {
        end_ = Clock::now();
    }
    /*! \brief return time in ms */
    inline double Elasped()
    {
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_ - start_);
        return duration.count();
    }

private:
    Clock::time_point start_, end_;
};

namespace autolabor_driver
{
    PicoZenseDriver::PicoZenseDriver() : _session_index(0)
    {
    }

    PicoZenseDriver::~PicoZenseDriver()
    {
        if (_device_handle != nullptr)
        {
            Ps2_StopStream(_device_handle, _session_index);
            Ps2_CloseDevice(_device_handle);
            Ps2_Shutdown();
        }
    }

    void PicoZenseDriver::initParams()
    {
        ros::NodeHandle private_nh("~");
        private_nh.param<std::string>("namespace", _namespace, "pico_camera");
        private_nh.param<std::string>("frame_name", _frame_name, "pico_camera");
        private_nh.param<int>("device_index", _device_index, 0);
        private_nh.param<int>("read_frame_interval", _read_frame_interval, 100);
        private_nh.param<int>("depth_range", _depth_range, 0);
        private_nh.param<int>("background_filter_threshold", _background_filter_threshold, 20);
        private_nh.param<int>("skip_row", _skip_row, 0);
        private_nh.param<int>("skip_column", _skip_column, 0);

        private_nh.param<bool>("output_depth_image", _output_depth_image, false);
        ros::NodeHandle nh(_namespace);
        if (_output_depth_image)
        {
            _depth_image_pub = nh.advertise<sensor_msgs::Image>("depth_image", 5);
        }
        private_nh.param<bool>("output_color_image", _output_color_image, false);
        if (_output_color_image)
        {
            _color_image_pub = nh.advertise<sensor_msgs::Image>("color_image", 8);
        }
        private_nh.param<bool>("output_color_info", _output_color_info, false);
        if (_output_color_info)
        {
            _color_info_pub = nh.advertise<sensor_msgs::CameraInfo>("color_info", 5);
        }

        private_nh.param<bool>("output_point_cloud", _output_point_cloud, true);
        if (_output_point_cloud)
        {
            _point_cloud_pub = nh.advertise<sensor_msgs::PointCloud>("point_cloud", 5);
        }
        private_nh.param<bool>("depth_spatial_filter", _depth_spatial_filter, true);
        private_nh.param<bool>("depth_time_filter", _depth_time_filter, true);
        private_nh.param<bool>("depth_distortion_correction", _depth_distortion_correction, true);
        private_nh.param<bool>("depth_straighten_correction", _depth_straighten_correction, true);
    }

    bool PicoZenseDriver::initCamera()
    {
        // Init SDK
        if ((_status = Ps2_Initialize()) != PsRetOK)
        {
            ROS_ERROR("Ps2_Initialize error, error code : %d", _status);
            return false;
        }
        // Get Count
        uint32_t device_count = 0;
        if ((_status = Ps2_GetDeviceCount(&device_count)) != PsRetOK)
        {
            ROS_ERROR("Ps2_GetDeviceCount error, error code : %d", _status);
            return false;
        }
        else if (device_count == 0)
        {
            ROS_ERROR("No Device connected!");
            return false;
        }
        // Open Device
        if (_device_index < device_count)
        {
            PsDeviceInfo device_info;
            Ps2_GetDeviceInfo(&device_info, _device_index);
            if ((_status = Ps2_OpenDevice(device_info.uri, &_device_handle)) != PsRetOK)
            {
                ROS_ERROR("Ps2_OpenDevice error, error code : %d", _status);
                return false;
            }

            if ((_status = Ps2_StartStream(_device_handle, _session_index)) != PsRetOK)
            {
                ROS_ERROR("Ps2_StartStream error, error code : %d", _status);
                return false;
            }

            if (configCamera())
            {
                return true;
            }
        }
        ROS_ERROR("Device number %d exceeds maximum", _device_index);
        return false;
    }

    bool PicoZenseDriver::configCamera()
    {
        // Set Data Mode
        if (_output_color_image)
        {
            Ps2_SetDataMode(_device_handle, _session_index, PsDepthAndRGB_30);
            Ps2_SetColorPixelFormat(_device_handle, _session_index, PsPixelFormatBGR888);
        }
        else
        {
            Ps2_SetDataMode(_device_handle, _session_index, PsDepthAndIR_30);
            Ps2_SetIrFrameEnabled(_device_handle, _session_index, false);
        }

        PsDepthRange depthRange;
        switch (std::max(0, std::min(8, _depth_range)))
        {
        case 0:
            depthRange = PsNearRange;
            break;
        case 1:
            depthRange = PsMidRange;
            break;
        case 2:
            depthRange = PsFarRange;
            break;
        case 3:
            depthRange = PsXNearRange;
            break;
        case 4:
            depthRange = PsXMidRange;
            break;
        case 5:
            depthRange = PsXFarRange;
            break;
        case 6:
            depthRange = PsXXNearRange;
            break;
        case 7:
            depthRange = PsXXMidRange;
            break;
        case 8:
            depthRange = PsXXFarRange;
            break;
        default:
            ROS_ERROR("something wrong");
            return false;
        }
        Ps2_SetDSPEnabled(_device_handle, _session_index, true);
        Ps2_SetDepthRange(_device_handle, _session_index, depthRange);
        Ps2_SetSpatialFilterEnabled(_device_handle, _session_index, _depth_spatial_filter);
        Ps2_SetTimeFilterEnabled(_device_handle, _session_index, _depth_time_filter);
        Ps2_SetComputeRealDepthCorrectionEnabled(_device_handle, _session_index, _depth_straighten_correction);
        Ps2_SetDepthDistortionCorrectionEnabled(_device_handle, _session_index, _depth_distortion_correction);
        Ps2_SetThreshold(_device_handle, _session_index, _background_filter_threshold);

        PsReturnStatus status;
        status = Ps2_SetRGBResolution(_device_handle, _session_index, PsRGB_Resolution_640_480);
        if (status != PsRetOK)
        {
            std::cerr << "Ps2_SetRGBResolution error: " << status << "\n";
            return false;
        }

        return true;
    }

    void PicoZenseDriver::publishColorInfo()
    {
        std::cout << "step into publishColorInfo" << std::endl;
        sensor_msgs::CameraInfo cam_info;

        PsReturnStatus status;
        PsCameraParameters camera_params;
        status = Ps2_GetCameraParameters(_device_handle, _session_index, PsRgbSensor, &camera_params);
        if (status != PsRetOK)
        {
            std::cerr << "Ps2_GetCameraParameters error: " << status << "\n";
            return;
        }

        u_int16_t resolution;
        status = Ps2_GetRGBResolution(_device_handle, _session_index, &resolution);
        if (status != PsRetOK)
        {
            std::cerr << "Ps2_GetRGBResolution error: " << status << std::endl;
            return;
        }

        switch (resolution)
        {
        case 0:
            cam_info.width = 1920;
            cam_info.height = 1080;
            break;
        case 1:
            cam_info.width = 1280;
            cam_info.height = 720;
            break;
        case 2:
            cam_info.width = 640;
            cam_info.height = 480;
            break;
        case 3:
            cam_info.width = 640;
            cam_info.height = 360;
            break;
        }

        std::cout << "step 1 is ok-------------------\n";

        // cam_info.width = 640;
        // cam_info.height = 480;
        
        cam_info.header.frame_id = "camera_color_optical_frame";
        cam_info.K.at(0) = camera_params.fx;
        cam_info.K.at(1) = 0.0;
        cam_info.K.at(2) = camera_params.cx;

        cam_info.K.at(3) = 0.0;
        cam_info.K.at(4) = camera_params.fy;
        cam_info.K.at(5) = camera_params.cy;

        cam_info.K.at(6) = 0.0;
        cam_info.K.at(7) = 0.0;
        cam_info.K.at(8) = 1.0;
        std::cout << "step 2 is ok-------------------\n";

        //------------------
        cam_info.P.at(0) = camera_params.fx;
        cam_info.P.at(1) = 0.0;
        cam_info.P.at(2) = camera_params.cx;
        cam_info.P.at(3) = 0.0;

        cam_info.P.at(4) = 0.0;
        cam_info.P.at(5) = camera_params.fy;
        cam_info.P.at(6) = camera_params.cy;
        cam_info.P.at(7) = 0.0;

        cam_info.P.at(8) = 0.0;
        cam_info.P.at(9) = 0.0;
        cam_info.P.at(10) = 1.0;
        cam_info.P.at(11) = 0.0;
        std::cout << "step 3 is ok-------------------\n";
        //-----------------------

        cam_info.distortion_model = "plumb_bob";
        cam_info.R.at(0) = 1.0;
        cam_info.R.at(1) = 0.0;
        cam_info.R.at(2) = 0.0;

        cam_info.R.at(3) = 0.0;
        cam_info.R.at(4) = 1.0;
        cam_info.R.at(5) = 0.0;

        cam_info.R.at(6) = 0.0;
        cam_info.R.at(7) = 0.0;
        cam_info.R.at(8) = 1.0;
        std::cout << "step 4 is ok-------------------\n";

        cam_info.D.resize(5);
        cam_info.D.at(0) = camera_params.k1;
        cam_info.D.at(1) = camera_params.k2;
        cam_info.D.at(2) = camera_params.p1;
        cam_info.D.at(3) = camera_params.p2;
        cam_info.D.at(4) = camera_params.k3;

        std::cout << "step 5 is ok-------------------\n";

        _color_info_pub.publish(cam_info);
    }

    void PicoZenseDriver::publishColorImage()
    {
        if (_output_color_image && ready_.rgb &&
            Ps2_GetFrame(_device_handle, _session_index, PsRGBFrame, &_color_frame) == PsRetOK)
        {
            sensor_msgs::Image img_msg;
            img_msg.header.frame_id = _frame_name + "_color_frame";
            img_msg.header.stamp = ros::Time::now();
            img_msg.width = _color_frame.width;
            img_msg.height = _color_frame.height;
            img_msg.is_bigendian = false;
            img_msg.encoding = sensor_msgs::image_encodings::BGR8;
            img_msg.step = img_msg.width * 3;
            int len = img_msg.width * img_msg.height * 3;
            img_msg.data.resize(len);
            for (int i = 0; i < len; i++)
            {
                img_msg.data[i] = _color_frame.pFrameData[i];
            }
            _color_image_pub.publish(img_msg);
        }
    }

    void PicoZenseDriver::publishDepthImage()
    {
        if (_output_depth_image && ready_.depth &&
            Ps2_GetFrame(_device_handle, _session_index, PsDepthFrame, &_depth_frame) == PsRetOK)
        {
            sensor_msgs::Image img_msg;
            img_msg.header.frame_id = _frame_name + "_depth_frame";
            img_msg.header.stamp = ros::Time::now();
            img_msg.width = _depth_frame.width;
            img_msg.height = _depth_frame.height;
            img_msg.is_bigendian = false;
            img_msg.encoding = sensor_msgs::image_encodings::MONO16;
            img_msg.step = img_msg.width * 2;
            int len = img_msg.width * img_msg.height * 2;
            img_msg.data.resize(len);
            for (int i = 0; i < len; i++)
            {
                img_msg.data[i] = _depth_frame.pFrameData[i];
            }
            _depth_image_pub.publish(img_msg);
        }
    }

    void PicoZenseDriver::publishPointCloud()
    {
        if (_output_point_cloud)
        {
            if (ready_.depth && !_output_depth_image)
            {
                Ps2_GetFrame(_device_handle, _session_index, PsDepthFrame, &_depth_frame);
            }
            PsVector3f point[_depth_frame.width * _depth_frame.height];
            _status = Ps2_ConvertDepthFrameToWorldVector(_device_handle, _session_index, _depth_frame, point);
            if (_status == PsRetOK)
            {
                sensor_msgs::PointCloud point_msg;
                point_msg.header.frame_id = _frame_name + "_depth_frame";
                point_msg.header.stamp = ros::Time::now();
                for (int i = 0; i < _depth_frame.height; i = i + 1 + _skip_row)
                {
                    for (int j = 0; j < _depth_frame.width; j = j + 1 + _skip_column)
                    {
                        size_t index = i * _depth_frame.width + j;
                        if (point[index].x != 0 || point[index].y != 0 || point[index].z != 0)
                        {
                            geometry_msgs::Point32 p;
                            p.x = point[index].x / 1000.0;
                            p.y = point[index].y / 1000.0;
                            p.z = point[index].z / 1000.0;
                            point_msg.points.push_back(p);
                        }
                    }
                }
                _point_cloud_pub.publish(point_msg);
            }
        }
    }

    void PicoZenseDriver::run()
    {
        initParams();
        //ros::Duration duration(_read_frame_interval / 1000.0);
        //ros::Duration duration(0.01);
        ros::Rate loop_rate(100);
        if (initCamera())
        {
            publishTf();

            while (ros::ok())
            {

                Timer timer_obj;
                timer_obj.Tic();
                PsReturnStatus status = Ps2_ReadNextFrame(_device_handle, _session_index, &ready_);
                timer_obj.Toc();

                std::cout << "read frame cost time: " << timer_obj.Elasped() << std::endl;
                if (status == PsRetOK)
                {
                    timer_obj.Tic();
                    publishColorImage();
                    timer_obj.Toc();
                    std::cout << "publish color image cost time: " << timer_obj.Elasped() << std::endl;

                    publishDepthImage();
                    publishPointCloud();

                    timer_obj.Tic();
                    timer_obj.Toc();
                    publishColorInfo();
                    std::cout << "publish color info cost time: " << timer_obj.Elasped() << std::endl;
                }

                //duration.sleep();
                //ros::spinOnce();
                loop_rate.sleep();
            }
        }
    }

    void PicoZenseDriver::publishTf()
    {
        Ps2_GetCameraExtrinsicParameters(_device_handle, _session_index, &_camera_ep);
        tf::Matrix3x3 rotation_matrix(_camera_ep.rotation[0], _camera_ep.rotation[1], _camera_ep.rotation[2],
                                      _camera_ep.rotation[3], _camera_ep.rotation[4], _camera_ep.rotation[5],
                                      _camera_ep.rotation[6], _camera_ep.rotation[7], _camera_ep.rotation[8]);
        double roll, pitch, yaw;
        rotation_matrix.getRPY(roll, pitch, yaw);

        geometry_msgs::TransformStamped msg;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = _frame_name + "_color_frame";
        msg.child_frame_id = _frame_name + "_depth_frame";
        msg.transform.rotation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
        msg.transform.translation.x = _camera_ep.translation[0] / 1000.0;
        msg.transform.translation.y = _camera_ep.translation[1] / 1000.0;
        msg.transform.translation.z = _camera_ep.translation[2] / 1000.0;
        _static_tf_broadcaster.sendTransform(msg);

        msg.header.frame_id = _frame_name;
        msg.child_frame_id = _frame_name + "_color_frame";
        msg.transform.rotation = tf::createQuaternionMsgFromRollPitchYaw(-1.57, 0.0, -1.57);
        msg.transform.translation.x = 0.031;
        msg.transform.translation.y = 0.0;
        msg.transform.translation.z = 0.0;
        _static_tf_broadcaster.sendTransform(msg);
    }
} // namespace autolabor_driver

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "PicoZenseDriver");
    autolabor_driver::PicoZenseDriver picoZenseDriver;
    picoZenseDriver.run();
    return 0;
}
