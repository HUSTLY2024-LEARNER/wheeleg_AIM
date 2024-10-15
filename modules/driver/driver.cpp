#include <thread>
#include <chrono>
#include <boost/asio.hpp>
#include <umt.hpp>
#include <driver.hpp>
#include <DaHengCamera.h>
#include <GxIAPI.h>
#include <DxImageProc.h>
#include <utils.h>

#include "CRC16.h"

using namespace LY_UTILS;

namespace DRIVER
{
    void real_serial_write_loop(boost::asio::serial_port *boost_serial_port, const bool& required_stop)
    {
        umt::Subscriber<SerialWriteData> serial_write_data_sub("serial_write", 0);
        while(!required_stop)
        {
            try{
                auto serial_write_data = serial_write_data_sub.pop();
                serial_write_data.start_flag = '!';
                unsigned char msg[sizeof(SerialWriteData)];
                memcpy(msg, &serial_write_data, sizeof(SerialWriteData));
                // COUT("SEND DATA", GREEN);
                addCRC16(msg);

                boost::asio::write(*boost_serial_port, boost::asio::buffer(msg, sizeof(SerialWriteData)));
            }catch(umt::MessageError &e){
                std::this_thread::sleep_for(std::chrono::milliseconds(200));
                COUT("serial write loop pop error",RED);
            }
        }
    }

    void timer_handler(const boost::system::error_code& ec, std::function<void(const boost::system::error_code&)> func,  boost::asio::io_context& io)
    {
        static bool data_received = false;
        std::atomic<bool> should_stop(false);

        if(!ec && ! data_received){
            should_stop = true;
        }else if(!ec){
            COUT("serial work fine", GREEN);
            boost::asio::deadline_timer timer(io);
            timer.expires_from_now(boost::posix_time::seconds(1));
            timer.async_wait([&, func](const boost::system::error_code& ec){
                timer_handler(ec, func, io);
            });
        }
    }
    bool is_serial_port_available(const std::string& port_name)
    {
        if(access(port_name.c_str(), F_OK) != -1)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    bool real_serial_io(const std::string &device_path, const int baud_rate)
    {
        auto serial_work_status = umt::ObjManager<SerialOK>::find_or_create("serial_work_ok");

        auto *this_serial_work_ok = serial_work_status.get();
        this_serial_work_ok->serial_ok = false;

        std::atomic<bool> should_stop(false);

        boost::asio::io_context io_context;

        boost::system::error_code result;

        if(!is_serial_port_available(device_path))
        {
            COUT("error serial path", RED);
            return false;
        }

        const auto boost_serial_port = std::make_shared<boost::asio::serial_port>(io_context);
        if (boost_serial_port->open(device_path, result).failed()) {
            COUT("cannot open serial port", RED);
            return false;
        }

        COUT("open serial port", GREEN);

        boost_serial_port->set_option(boost::asio::serial_port_base::baud_rate(baud_rate));
        boost_serial_port->set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
        boost_serial_port->set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
        boost_serial_port->set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
        boost_serial_port->set_option(boost::asio::serial_port_base::character_size(8));
                
        uint8_t read_data_length = sizeof(SerialReadData::IMU_Flag);
        std::unique_ptr<uint8_t[]> data_tmp(new uint8_t[read_data_length]);

        std::unique_ptr<uint8_t[]> data_trash_can(new uint8_t[1]);

        auto serial_imu_flag_data = umt::ObjManager<DRIVER::SerialReadData::IMU_Flag>::find_or_create("imu_flag_data");

        std::thread real_serial_write_thread(std::bind(&real_serial_write_loop, boost_serial_port.get(), std::ref(should_stop)));

        boost::asio::deadline_timer timer(io_context);
        bool data_received = false;

        // start serial watcher
        std::function<void(const boost::system::error_code&)> handler_func;

        handler_func = [&](const boost::system::error_code& ec){
            timer_handler(ec, handler_func, io_context);
        };

        timer.expires_from_now(boost::posix_time::seconds(1));
        timer.async_wait(handler_func);

        // 异步读取数据
        using AsyncReadCallback = std::function<void(const boost::system::error_code&, std::size_t)>;

        AsyncReadCallback async_read_callback;

        async_read_callback = [&](const boost::system::error_code& ec, std::size_t bytes_transfered)
        {
            if (!ec) {
                data_received = true;

                SerialReadData::IMU_Flag imu_flag;
                if(data_tmp[0] == '!') // ?
                {
                    // CICECOO
                    if (verifyCRC16(reinterpret_cast<SerialReadData::IMU_Flag *>(data_tmp.get()))) {
                        this_serial_work_ok->serial_ok = true;
                    
                        // COUT("right bite", GREEN);
                        memcpy(serial_imu_flag_data.get(), data_tmp.get(), sizeof(SerialReadData::IMU_Flag));

                        // 每次读取成功后再次发起异步读取操作
                        boost::asio::async_read(*boost_serial_port, boost::asio::buffer(data_tmp.get(), read_data_length),
                            [&](const boost::system::error_code& ec, std::size_t bytes_transferred)
                            {
                                async_read_callback(ec, bytes_transferred);
                            });
                    }
                }
                else
                {
                    COUT("error bite", RED);
                    int error_length;
                    for(int i = 0; i < read_data_length; i ++)
                    {
                        if(data_tmp[i] == '!')
                        {
                            error_length =i;
                        }
                    }
                    std::cout<<"error length: "<<error_length<<std::endl;
                    for(int j = 0; j < error_length; j++)
                    {
                        boost::asio::async_read(*boost_serial_port, boost::asio::buffer(data_trash_can.get(), uint8_t(1)),
                        [&](const boost::system::error_code& ec, std::size_t bytes_transferred)
                        {
                            std::cout<<"throw 1 bite"<<std::endl;
                        });
                    }
                    boost::asio::async_read(*boost_serial_port, boost::asio::buffer(data_tmp.get(), read_data_length),
                        [&](const boost::system::error_code& ec, std::size_t bytes_transferred)
                        {
                            async_read_callback(ec, bytes_transferred);
                        });
                }
                
            } else {
                data_received = false;
                std::cerr << "Error reading data: " << ec.message() << std::endl;
                should_stop = true;
            }
        };

        // 在发起异步读取操作时回调函数
        boost::asio::async_read(*boost_serial_port, boost::asio::buffer(data_tmp.get(), read_data_length), 
            [&](const boost::system::error_code& ec, std::size_t bytes_transferred)
            {
                async_read_callback(ec, bytes_transferred);
            });

        // 启动事件循环
        std::thread([&io_context](){
            io_context.run();
        }).join();

        real_serial_write_thread.join();

        return false;
    }

    bool real_video_io(const std::string& camera_sn, const int& exposure_time)
    {
        int empty_mat_count = 0;

        umt::Publisher<DRIVER::PicStamp> pic_pub("image_raw");

        std::unique_ptr<DaHengCamera> daheng_camera = std::make_unique<DaHengCamera>();

        constexpr auto Width = 1280;
        constexpr auto Height = 1024;

        daheng_camera->initLib();
        daheng_camera->openDevice(camera_sn.c_str());
        daheng_camera->setRoiParam(Width,Height,0,0);
        daheng_camera->setExposureGainParam(false,false,exposure_time,500,8000,12,5,13,28,28,true);
        daheng_camera->setWhiteBalanceParam(false,1.71,2.17,1,GX_AWB_LAMP_HOUSE_ADAPTIVE);
        daheng_camera->setAAROIParam(Width / 2, Height / 2, 320, 256);
        daheng_camera->acquisitionStart();

        auto time_begin = std::chrono::steady_clock::now();
        auto begin_milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(time_begin.time_since_epoch()).count();
        
        while(true)
        {
            PicStamp pic_stamp;
            std::chrono::steady_clock::time_point time_point;
            daheng_camera->ProcGetImage(&pic_stamp.pic,&time_point);
            auto now_milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(time_point.time_since_epoch()).count();
            long time_stamp = now_milliseconds - begin_milliseconds;
            pic_stamp.time_stamp = time_stamp;

            if(pic_stamp.pic.empty()){
                empty_mat_count++;
            } else {
                empty_mat_count = 0;
            }  
            if(empty_mat_count >= 10)
            {
                return false;
            }
            pic_pub.push(pic_stamp);
        }

        return false;
    }

    void bkg_driver_auto_restart(const std::string &device_path, const int baud_rate, const std::string &camera_sn, const int& exposure_time)
    {
        using namespace std::chrono_literals;

        // serial_io
        std::thread([device_path, baud_rate]() {
            COUT("device_path: "<<device_path, GREEN);
            while (!real_serial_io(device_path, baud_rate)) {
                auto serial_work_status = umt::ObjManager<SerialOK>::find_or_create("serial_work_ok");
                serial_work_status->serial_ok = false;
                std::this_thread::sleep_for(500ms);
            }
        }).detach();

        // video_io
        std::thread([camera_sn, exposure_time](){
            while(!real_video_io(camera_sn, exposure_time)){
                std::this_thread::sleep_for(500ms);
            }
        }).detach();
    }

PYBIND11_EMBEDDED_MODULE(DRIVER_, m) {
    namespace py = pybind11;
    m.def("bkg_driver_auto_restart", bkg_driver_auto_restart, py::arg("device_path"), py::arg("baud_rate"), py::arg("camera_sn"), py::arg("exposure_time"));
}

} // namespace DRIVER