#pragma once

#include <boost/asio.hpp>
#include <string>

using boost::asio::ip::udp;

namespace LY_UTILS {
    /**
     * @brief tail是VOFA+调试助手规定的帧尾
     * 并且选用了justFloat模式： 发送的数据都是浮点数
     * 这部分需要补充，但是必须使用相同的tail
     * 参考这部分补充内容
     */

    /**
     *  serial_write_data_pub.push({
            '!', 
            1, 
            shoot_flag_locker.getValue(), 
            predict_pack.pitch_setpoint + pitch_offset, 
            predict_pack.yaw_setpoint + yaw_offset
            });
     */
    struct SendFrame {
        float vaild;
        float shootflag;
        float pitch;
        float yaw;
        unsigned char tail[4]{0x00, 0x00, 0x80, 0x7f};
    };

    struct CenterStateFrame {
        float x_c;
        float v_x;
        float y_c;
        float v_y;
        float z_1;
        float z_2;
        float v_z;
        float k;
        unsigned char tail[4]{0x00, 0x00, 0x80, 0x7f};
    };

    class UDPSender {
    public:
        /**
         * @brief Construct a new UDPSender object
         *
         * @param ip_address 目标ip地址，在这里指定为本机的ip地址
         * @param port 目标端口，同样为windows的端口，在VOFA+中使用的部分，可以方便的
         */
        UDPSender(const std::string &ip_address, int port)
            : remote_endpoint_(boost::asio::ip::address::from_string(ip_address), port),
              socket_(io_service_, udp::endpoint(udp::v4(), 0)) {
        }

        /**
         * @brief 发送数据，这里建议
         *
         * @tparam Tw
         * @param data
         */
        template <typename T>
        void send(const T &data) {
            std::string message(sizeof(data), '\0');
            std::memcpy(message.data(), &data, sizeof(data));

            socket_.send_to(boost::asio::buffer(message), remote_endpoint_);
        }

    private:
        boost::asio::io_service io_service_;
        udp::endpoint remote_endpoint_;
        udp::socket socket_;
    };
}
