/**
 * This file is written by WANG Guangfu from China.
 * Email: thuwgf@gmail.com
 * Date: 2022-07-14.
 * License: Apache License 2.0.
 */
#include <cstdio>
#include <cstdlib>
#include <fcntl.h>
#include <unistd.h>
#include <cassert>
#include <termios.h>
#include <cstring>
#include <sys/time.h>
#include <ctime>
#include <array>
#include <memory>
#include <sys/types.h>

#include "sensor_msgs/msg/imu.hpp"
#include "rclcpp/rclcpp.hpp"


using std::cos;
using std::sin;

class MPU6050HW {
public:
    MPU6050HW() {
        bzero(mBuff, 1024);
        mRunning = false;
    };

    ~MPU6050HW() {
        if (mRunning) {
            close();
        }
    };

    void open(int baud, const std::string &serial_port = "/dev/ttyUSB0") {
        if (mRunning) {
            fprintf(stderr, "Uart has been open: Handler: %d\n", mMPUHandle);
            return;
        }
        mMPUHandle = uart_open(serial_port.c_str());
        if (mMPUHandle == -1) {
            fprintf(stderr, "Uart open error\n");
            exit(EXIT_FAILURE);
        }

        if (uart_set(baud, 8, 'N', 1) == -1) {
            fprintf(stderr, "Uart set failed!\n");
            exit(EXIT_FAILURE);
        }
        mRunning = true;
    };

    void close() {
        mRunning = false;
        if (uart_close() == -1) {
            fprintf(stderr, "uart_close error\n");
            exit(EXIT_FAILURE);
        }
    };

    void getData(std::array<double, 9> &data, bool withAngle = true) {
        int ret = recv_data(mBuff, 44);
        if (ret == -1) {
            fprintf(stderr, "uart read failed!\n");
            exit(EXIT_FAILURE);
        }
        for (int i = 0; i < ret; i++) {
            parse_data(mBuff[i]);
        }
        data[0] = mAcceleration[0];
        data[1] = mAcceleration[1];
        data[2] = mAcceleration[2];
        data[3] = mGyroscope[0];
        data[4] = mGyroscope[1];
        data[5] = mGyroscope[2];
        if (!withAngle)
            return;
        else {
            data[6] = mAngle[0];
            data[7] = mAngle[1];
            data[8] = mAngle[2];
        }
    };

private:
    int uart_open(const char *pathname) {
        mMPUHandle = ::open(pathname, O_RDWR | O_NOCTTY);
        if (-1 == mMPUHandle) {
            perror("Can't Open Serial Port");
            return (-1);
        } else
            printf("Open %s success!\n", pathname);
        if (isatty(STDIN_FILENO) == 0)
            printf("Standard input is not a terminal device\n");
        else
            printf("Isatty success!\n");
        return mMPUHandle;
    }

    int uart_set(int nSpeed, int nBits, char nEvent, int nStop) {
        struct termios newtio, oldtio;
        if (tcgetattr(mMPUHandle, &oldtio) != 0) {
            perror("SetupSerial 1");
            printf("Tcgetattr( fd,&oldtio) -> %d\n", tcgetattr(mMPUHandle, &oldtio));
            return -1;
        }
        bzero(&newtio, sizeof(newtio));
        newtio.c_cflag |= CLOCAL | CREAD;
        newtio.c_cflag &= ~CSIZE;
        switch (nBits) {
            case 7:
                newtio.c_cflag |= CS7;
                break;
            case 8:
                newtio.c_cflag |= CS8;
                break;
        }
        switch (nEvent) {
            case 'o':
            case 'O':
                newtio.c_cflag |= PARENB;
                newtio.c_cflag |= PARODD;
                newtio.c_iflag |= (INPCK | ISTRIP);
                break;
            case 'e':
            case 'E':
                newtio.c_iflag |= (INPCK | ISTRIP);
                newtio.c_cflag |= PARENB;
                newtio.c_cflag &= ~PARODD;
                break;
            case 'n':
            case 'N':
                newtio.c_cflag &= ~PARENB;
                break;
            default:
                break;
        }

        /*set baud*/

        switch (nSpeed) {
            case 2400:
                cfsetispeed(&newtio, B2400);
                cfsetospeed(&newtio, B2400);
                break;
            case 4800:
                cfsetispeed(&newtio, B4800);
                cfsetospeed(&newtio, B4800);
                break;
            case 9600:
                cfsetispeed(&newtio, B9600);
                cfsetospeed(&newtio, B9600);
                break;
            case 115200:
                cfsetispeed(&newtio, B115200);
                cfsetospeed(&newtio, B115200);
                break;
            case 460800:
                cfsetispeed(&newtio, B460800);
                cfsetospeed(&newtio, B460800);
                break;
            default:
                cfsetispeed(&newtio, B9600);
                cfsetospeed(&newtio, B9600);
                break;
        }
        if (nStop == 1)
            newtio.c_cflag &= ~CSTOPB;
        else if (nStop == 2)
            newtio.c_cflag |= CSTOPB;
        newtio.c_cc[VTIME] = 0;
        newtio.c_cc[VMIN] = 0;
        tcflush(mMPUHandle, TCIFLUSH);

        if ((tcsetattr(mMPUHandle, TCSANOW, &newtio)) != 0) {
            perror("Com set error");
            return -1;
        }
        printf("Set done!\n");
        return 0;
    }

    int uart_close() {
        assert(mMPUHandle);
        ::close(mMPUHandle);
        return 0;
    }

    int send_data(char *send_buffer, int length) {
        length = write(mMPUHandle, send_buffer, length * sizeof(unsigned char));
        return length;
    }

    int recv_data(char *recv_buffer, int length) {
        length = read(mMPUHandle, recv_buffer, length);
        return length;
    }

    void parse_data(char chr) {
        static char chrBuf[100];
        static unsigned char chrCnt = 0;
        signed short sData[4];
        unsigned char i;

        time_t now;
        chrBuf[chrCnt++] = chr;
        if (chrCnt < 11)
            return;

        if ((chrBuf[0] != 0x55) || ((chrBuf[1] & 0x50) != 0x50)) {
            //printf("Error:%x %x\r\n", chrBuf[0], chrBuf[1]);
            memcpy(&chrBuf[0], &chrBuf[1], 10);
            chrCnt--;
            return;
        }

        memcpy(&sData[0], &chrBuf[2], 8);
        switch (chrBuf[1]) {
            case 0x51:
                for (i = 0; i < 3; i++)
                    mAcceleration[i] = (double) sData[i] / 32768.0 * 16.0;
                time(&now);
            //printf("\r\nT:%s a:%6.3f %6.3f %6.3f ",
            //  asctime(localtime(&now)), mAcceleration[0], mAcceleration[1], mAcceleration[2]);
                break;
            case 0x52:
                for (i = 0; i < 3; i++)
                    mGyroscope[i] = (double) sData[i] / 32768.0 * 2000.0;
            // printf("w:%7.3f %7.3f %7.3f ", mGyroscope[0], mGyroscope[1], mGyroscope[2]);
                break;
            case 0x53:
                for (i = 0; i < 3; i++)
                    mAngle[i] = (double) sData[i] / 32768.0 * 180.0;
            // printf("A:%7.3f %7.3f %7.3f ", mAngle[0], mAngle[1], mAngle[2]);
                break;
        }
        chrCnt = 0;
    }

private:
    double mAcceleration[3]{};
    double mGyroscope[3]{};
    double mAngle[3]{};
    char mBuff[1024]{};
    int mMPUHandle{};
    bool mRunning = false;
};

class MPU6050Node : public rclcpp::Node {
public:
    MPU6050Node() : Node("mpu6050_node") {
        mIMUDevice = std::make_shared<MPU6050HW>();
        // 115200 for JY61 ,9600 for others
        // 115200 meaning 100Hz, 9600 meaning 20Hz.
        this->declare_parameter<int>("baudrate", 115200);
        this->declare_parameter("serial_port", "/dev/ttyUSB0");
        this->declare_parameter("imu_rate", 50);
        int baudrate = this->get_parameter("baudrate").as_int();
        std::string serial_port = this->get_parameter("serial_port").as_string();
        int imu_rate = this->get_parameter("imu_rate").as_int();

        mIMUDevice->open(baudrate, serial_port);
        // create publisher and setup timer callback. here 10 meaning queue size.
        mPublisher = this->create_publisher<sensor_msgs::msg::Imu>("/imu/data", 10);
        mSeq = 0;
        std::chrono::duration<double, std::milli> dur(1000 / imu_rate);
        mTimer = this->create_wall_timer(dur, std::bind(&MPU6050Node::timer_callback, this));
    };

private:
    void timer_callback() {
        mIMUDevice->getData(mRawImuData);
        auto msg = sensor_msgs::msg::Imu();
        // fill in data.
        msg.header.frame_id = "imu_link";
        msg.header.stamp = this->get_clock()->now();
        msg.linear_acceleration.x = mRawImuData[0] * mGravity;
        msg.linear_acceleration.y = mRawImuData[1] * mGravity;
        msg.linear_acceleration.z = mRawImuData[2] * mGravity;
        msg.angular_velocity.x = mRawImuData[3] * M_PI / 180;
        msg.angular_velocity.y = mRawImuData[4] * DegreeToRad;
        msg.angular_velocity.z = mRawImuData[5] * DegreeToRad;
        double f_cos = cos(mRawImuData[6] * DegreeToRadHalf);
        double f_sin = sin(mRawImuData[6] * DegreeToRadHalf);
        double s_cos = cos(mRawImuData[7] * DegreeToRadHalf);
        double s_sin = sin(mRawImuData[7] * DegreeToRadHalf);
        double t_cos = cos(mRawImuData[8] * DegreeToRadHalf);
        double t_sin = sin(mRawImuData[8] * DegreeToRadHalf);
        msg.orientation.w = f_cos * s_cos * t_cos + f_sin * s_sin * t_sin;
        msg.orientation.x = f_sin * s_cos * t_cos - f_cos * s_sin * t_sin;
        msg.orientation.y = f_cos * s_sin * t_cos + f_sin * s_cos * t_sin;
        msg.orientation.z = f_cos * s_cos * t_sin - f_sin * s_sin * t_cos;

        // RCLCPP_INFO(this->get_logger(), "MPU6050: \n\tSequence: %llu"
        //             "\n\tAcceleration(m/s^2): %6.3f %6.3f %6.3f; "
        //             "\n\tGyroscope(rad/s): %6.3f %6.3f %6.3f; "
        //             "\n\tEuler Angle(degree): %6.3f %6.3f %6.3f",
        //             mSeq,
        //             full_data.linear_acceleration.x,
        //             full_data.linear_acceleration.y,
        //             full_data.linear_acceleration.z,
        //             full_data.angular_velocity.x,
        //             full_data.angular_velocity.y,
        //             full_data.angular_velocity.z,
        //             mRawImuData[6], mRawImuData[7], mRawImuData[8]);
        mPublisher->publish(msg);
        // usleep(1000);
        mSeq++;
    }

private:
    std::shared_ptr<MPU6050HW> mIMUDevice;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr mPublisher;
    rclcpp::TimerBase::SharedPtr mTimer;
    std::array<double, 9> mRawImuData{};

    const double mGravity = 9.801;
    const double DegreeToRad = M_PI / 180;
    const double DegreeToRadHalf = M_PI / 360;
    unsigned long long mSeq = 0;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MPU6050Node>());
    rclcpp::shutdown();
    return 0;
}
