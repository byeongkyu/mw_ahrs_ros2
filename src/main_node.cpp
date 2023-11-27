#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include <cstdio>
#include <csignal>
#include <fcntl.h>
#include <libserial/SerialPort.h>
#include <boost/algorithm/string.hpp>
#include <tf2/LinearMath/Quaternion.h>


class MWAHRSDriverNode : public rclcpp::Node
{
    public:
        MWAHRSDriverNode(): Node("mw_ahrs_driver")
        {
            this->declare_parameter<std::string>("port_name", "/dev/ttyIMU");
            this->declare_parameter<int>("baudrate", 921600);
            this->declare_parameter<std::string>("frame_id", "imu_link");
            this->declare_parameter<double>("rate", 100.0);

            auto port_name = this->get_parameter("port_name").get_parameter_value().get<std::string>();
            auto baudrate = this->get_parameter("baudrate").get_parameter_value().get<uint32_t>();

            RCLCPP_INFO(this->get_logger(), "Port: [%s] and baudrate %ld", port_name.c_str(), baudrate);

            is_started_ = false;

            try
            {
                ser_.Open(port_name);
            }
            catch(LibSerial::OpenFailed &e)
            {
                RCLCPP_ERROR(this->get_logger(), "Exceptions: \033[0;91m%s\033[0m", e.what());
                RCLCPP_ERROR(this->get_logger(), "\033[0;91mFailed to open port\033[0m [\033[0;92m%s\033[0m]...", port_name.c_str());
                exit(-1);
            }

            auto ser_baudrate = LibSerial::BaudRate::BAUD_9600;
            switch(baudrate)
            {
                case 9600:   ser_baudrate = LibSerial::BaudRate::BAUD_9600;   break;
                case 19200:  ser_baudrate = LibSerial::BaudRate::BAUD_19200;  break;
                case 38400:  ser_baudrate = LibSerial::BaudRate::BAUD_38400;  break;
                case 57600:  ser_baudrate = LibSerial::BaudRate::BAUD_57600;  break;
                case 115200: ser_baudrate = LibSerial::BaudRate::BAUD_115200; break;
                case 921600: ser_baudrate = LibSerial::BaudRate::BAUD_921600; break;
            }
            ser_.SetBaudRate(ser_baudrate);
            ser_.FlushIOBuffers();
            rclcpp::sleep_for(std::chrono::milliseconds(100));

            pub_imu_msg_ = this->create_publisher<sensor_msgs::msg::Imu>("imu_raw", rclcpp::SystemDefaultsQoS());
            imu_msg_.header.frame_id = this->get_parameter("frame_id").get_parameter_value().get<std::string>();
            imu_msg_.orientation_covariance = {0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01};
            imu_msg_.angular_velocity_covariance = {0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01};
            imu_msg_.linear_acceleration_covariance = {0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01};


            // Reset Sensor
            ser_.Write("rst\n");
            rclcpp::sleep_for(std::chrono::milliseconds(2000));
            ser_.FlushIOBuffers();

            while(rclcpp::ok())
            {
                if(ser_.GetNumberOfBytesAvailable() == 0)
                {
                    break;
                }
                ser_.FlushIOBuffers();
            }

            RCLCPP_INFO(this->get_logger(), "Ready for streaming...");


            char rate_str[100];
            sprintf(rate_str, "sp=%d\n", int(1000 / this->get_parameter("rate").as_double()));

            ser_.Write(rate_str);
            rclcpp::sleep_for(std::chrono::milliseconds(100));
            ser_.Write("ss=7\n");

            auto period = std::chrono::duration<double>(1.0 / this->get_parameter("rate").as_double());
            timer_ = this->create_wall_timer(period, std::bind(&MWAHRSDriverNode::timer_callback, this));
        }
        ~MWAHRSDriverNode()
        {
            if(is_started_)
            {
                RCLCPP_INFO(this->get_logger(), "Stop streaming...");
                ser_.Write("ss=0\n");
            }
        }


    private:
        void timer_callback()
        {
            std::string recv_data;

            try
            {
                ser_.ReadLine(recv_data, '\n', 10);

                if(!is_started_)
                {
                    RCLCPP_INFO(this->get_logger(), "Start streaming...");
                    is_started_ = true;
                }

                std::vector<std::string> split_str;
                boost::split(split_str, recv_data, boost::is_any_of(" "));

                std::vector<double> imu_recv_data;
                for(size_t i = 0; i < split_str.size(); i++)
                {
                    if(split_str[i] == "" or split_str[i] == " ")
                        continue;

                    imu_recv_data.push_back(atof(split_str[i].c_str()));
                }

                imu_msg_.linear_acceleration.x = imu_recv_data[0];
                imu_msg_.linear_acceleration.y = imu_recv_data[1];
                imu_msg_.linear_acceleration.z = imu_recv_data[2];

                imu_msg_.angular_velocity.x = imu_recv_data[3] * M_PI / 180.0;
                imu_msg_.angular_velocity.y = imu_recv_data[4] * M_PI / 180.0;
                imu_msg_.angular_velocity.z = imu_recv_data[5] * M_PI / 180.0;

                tf2::Quaternion q;
                q.setRPY(imu_recv_data[6] * M_PI / 180.0, imu_recv_data[7] * M_PI / 180.0, imu_recv_data[8] * M_PI / 180.0);

                imu_msg_.orientation.x = q.getX();
                imu_msg_.orientation.y = q.getY();
                imu_msg_.orientation.z = q.getZ();
                imu_msg_.orientation.w = q.getW();

                imu_msg_.header.stamp = this->now();
                pub_imu_msg_->publish(imu_msg_);
            }
            catch(LibSerial::ReadTimeout &e)
            {
                return;
            }
        }

    private:
        rclcpp::TimerBase::SharedPtr timer_;
        LibSerial::SerialPort ser_;
        std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Imu>> pub_imu_msg_;
        sensor_msgs::msg::Imu imu_msg_;
        bool is_started_;
};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MWAHRSDriverNode>());
    rclcpp::shutdown();
    return 0;
}
