#include <geometry_msgs/Quaternion.h>
#include <ros/ros.h>
#include <serial/serial.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>
#include <string>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>


bool zero_orientation_set = false;

bool set_zero_orientation(std_srvs::Empty::Request&,
                          std_srvs::Empty::Response&)
{
    ROS_INFO("Zero Orientation Set.");
    zero_orientation_set = false;
    return true;
}

bool DecodeIMUData(double result[], unsigned char chrTemp[], unsigned char mode )
{
    int scale;
    if( chrTemp[1] = mode)
    {
        switch(mode)
        {
        case 0x51: scale = 16; //a
            break;
        case 0x52: scale = 16; //w
            break;
        case 0x53: scale = 2000;//angle
            break;
        }
        result[0] = (short(chrTemp[3]<<8|chrTemp[2]))/32768.0*scale;
        result[1] = (short(chrTemp[5]<<8|chrTemp[4]))/32768.0*scale;
        result[2] = (short(chrTemp[7]<<8|chrTemp[6]))/32768.0*scale;

//      double T = (short(chrTemp[9]<<8|chrTemp[8]))/340.0+36.25;
        printf("aresult = %4.3f\t%4.3f\t%4.3f\t\r\n",result[0],result[1],result[2]);
        return true;
    }
    return false;
}

int main(int argc, char** argv)
{
    serial::Serial ser;
    std::string port;
    std::string tf_parent_frame_id;
    std::string tf_frame_id;
    std::string imu_frame_id;
    double time_offset_in_seconds;

    tf::Quaternion zero_orientation;

    ros::init(argc, argv, "imu_serial_node");

    ros::NodeHandle private_node_handle("~");
    private_node_handle.param<std::string>("port", port, "/dev/ttyUSB0");
    private_node_handle.param<std::string>("tf_parent_frame_id", tf_parent_frame_id, "imu_base");
    private_node_handle.param<std::string>("tf_frame_id", tf_frame_id, "imu");
    private_node_handle.param<std::string>("imu_frame_id", imu_frame_id, "imu_base");
    private_node_handle.param<double>("time_offset_in_seconds", time_offset_in_seconds, 0.0);

    ros::NodeHandle nh;
    ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("imu", 100);
    ros::ServiceServer service = nh.advertiseService("set_zero_orientation", set_zero_orientation);

    ros::Rate r(100); // 1000 hz

    unsigned char chrBuffer[1000];
    unsigned char chrTemp[1000];
    unsigned short usLength=0,usRxLength=0;

    while(ros::ok())
    {
        try
        {
            if (ser.isOpen())
            {
                // read string from serial device
                if(ser.available())
                {
                    usLength = ser.read(chrBuffer,33);
                    if (usLength>0)
                    {
                        usRxLength += usLength;
                        while (usRxLength >= 11)
                        {
                            memcpy(chrTemp,chrBuffer,usRxLength);
                            if (!((chrTemp[0] == 0x55) & ((chrTemp[1] == 0x51) | (chrTemp[1] == 0x52) | (chrTemp[1] == 0x53))))
                            {
                                for (int i = 1; i < usRxLength; i++) chrBuffer[i - 1] = chrBuffer[i];
                                usRxLength--;
                                continue;
                            }
                            double*  Angle;
                            if(!DecodeIMUData(Angle, chrTemp, 0X53))
                                continue;

                            tf::Quaternion orientation(Angle[2],Angle[1],Angle[0]);

                            if (!zero_orientation_set)
                            {
                                zero_orientation = orientation;
                                zero_orientation_set = true;
                            }

                            tf::Quaternion differential_rotation;
                            differential_rotation = zero_orientation.inverse() * orientation;

                            // calculate measurement time
                            ros::Time measurement_time = ros::Time::now() + ros::Duration(time_offset_in_seconds);

                            // publish imu message
                            sensor_msgs::Imu imu;
                            imu.header.stamp = measurement_time;
                            imu.header.frame_id = imu_frame_id;

                            quaternionTFToMsg(differential_rotation, imu.orientation);

                            // i do not know the orientation covariance
                            imu.orientation_covariance[0] = 1000000;
                            imu.orientation_covariance[1] = 0;
                            imu.orientation_covariance[2] = 0;
                            imu.orientation_covariance[3] = 0;
                            imu.orientation_covariance[4] = 1000000;
                            imu.orientation_covariance[5] = 0;
                            imu.orientation_covariance[6] = 0;
                            imu.orientation_covariance[7] = 0;
                            imu.orientation_covariance[8] = 0.000001;

                            // angular velocity is not provided
                            imu.angular_velocity_covariance[0] = -1;

                            // linear acceleration is not provided
                            imu.linear_acceleration_covariance[0] = -1;

                            imu_pub.publish(imu);

                            // publish tf transform
                            static tf::TransformBroadcaster br;
                            tf::Transform transform;
                            transform.setRotation(differential_rotation);
                            br.sendTransform(tf::StampedTransform(transform, measurement_time, tf_parent_frame_id, tf_frame_id));

                            for (int i = 11; i < usRxLength; i++)
                                chrBuffer[i - 11] = chrBuffer[i];
                            usRxLength -= 11;
                        }
                    }
                }
                else  // ser not available
                {

                }
            }
            else
            {
                // try and open the serial port
                try
                {
                    ser.setPort(port);
                    ser.setBaudrate(115200);
                    serial::Timeout to = serial::Timeout::simpleTimeout(1000);
                    ser.setTimeout(to);
                    ser.open();
                }
                catch (serial::IOException& e)
                {
                    ROS_ERROR_STREAM("Unable to open serial port " << ser.getPort() << ". Trying again in 5 seconds.");
                    ros::Duration(5).sleep();
                }

                if(ser.isOpen())
                {
                    ROS_DEBUG_STREAM("Serial port " << ser.getPort() << " initialized.");
                }
                else
                {
                    //ROS_INFO_STREAM("Could not initialize serial port.");
                }

            }
        }
        catch (serial::IOException& e)
        {
            ROS_ERROR_STREAM("Error reading from the serial port " << ser.getPort() << ". Closing connection.");
            ser.close();
        }
        ros::spinOnce();
        r.sleep();
    }

}

