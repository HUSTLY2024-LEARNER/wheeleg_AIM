#include <chrono>
#include <functional>
#include <thread>
#include <string>

#include <json/json.h>
#include <opencv2/opencv.hpp>
#include <boost/archive/iterators/binary_from_base64.hpp>
#include <boost/archive/iterators/transform_width.hpp>
#include <boost/algorithm/string.hpp>
#include <client_ws/client_ws.hpp>
#include <driver.hpp>
#include <SmartLog.hpp>
#include <umt.hpp>


using namespace LY_UTILS;

namespace SIM
{   
    using WsClient = SimpleWeb::SocketClient<SimpleWeb::WS>;
    using InMessage =  std::function<void(std::shared_ptr<WsClient::Connection>, std::shared_ptr<WsClient::InMessage>)>;

    std::string ros_host = "localhost:9090";

    std::unique_ptr<WsClient> serial_write_client = std::make_unique<WsClient>(ros_host);

    bool serial_success_start = false;

    DRIVER::SerialReadData::IMU_Flag extractShortSerialData(const std::string& message)
    {
        Json::CharReaderBuilder readerBuilder;
        Json::Value root;
        std::istringstream dataStream(message);
        if (!Json::parseFromStream(readerBuilder, dataStream, &root, nullptr)) {
            std::cerr << "Failed to parse JSON." << std::endl;
            return {};
        }
        const Json::Value& msgXField = root["msg"]["x"];
        const Json::Value& msgYField = root["msg"]["y"];
        const Json::Value& msgZField = root["msg"]["z"];
        DRIVER::SerialReadData::IMU_Flag serial_data_short_temp;
        //std::cout << "Content of 'msg' field: " << msgField << std::endl;
        if (msgXField.isDouble()&&msgYField.isDouble()&&msgZField.isDouble()) {
            serial_data_short_temp.yaw_now=msgXField.asFloat();
            serial_data_short_temp.pitch_now=msgYField.asFloat();
            int mode=msgZField.asFloat()/100;
            int request=(msgZField.asFloat()-mode*100)/10;
            int number=msgZField.asFloat()-mode*100-10*request;
            serial_data_short_temp.mode_want=mode;
            serial_data_short_temp.aim_request=request;
            serial_data_short_temp.number_want=number;
            return serial_data_short_temp;
        } else {
            std::cerr << "'msg''x' or 'msg''y' or 'msg''z' field is not double." << std::endl;
            return {};
        }
        return {};
    }

    std::string extractImageData(const std::string& message) {
        Json::CharReaderBuilder readerBuilder;
        Json::Value root;
        std::istringstream dataStream(message);
        if (!Json::parseFromStream(readerBuilder, dataStream, &root, nullptr)) {
            std::cerr << "Failed to parse JSON." << std::endl;
            return {};
        }
        const Json::Value& msgField = root["msg"]["data"];
        //std::cout << "Content of 'msg' field: " << msgField << std::endl;
        if (msgField.isString()) {
            return msgField.asString();
        } else {
            std::cerr << "'msg''data' field is not string." << std::endl;
            return {};
        }
            return {};
    }

    std::vector<unsigned char> base64_decode(const std::string &base64_string) {
        // Remove newline characters and other non-Base64 characters
        std::string cleaned_string = boost::algorithm::replace_all_copy(base64_string, "\n", "");
        cleaned_string = boost::algorithm::replace_all_copy(cleaned_string, "\r", "");
        cleaned_string = boost::algorithm::erase_all_copy(cleaned_string, "\t");
        cleaned_string = boost::algorithm::erase_all_copy(cleaned_string, " ");

        // Base64 decoding
        using namespace boost::archive::iterators;
        typedef transform_width<binary_from_base64<std::string::const_iterator>, 8, 6> ItBinaryT;
        std::string result(ItBinaryT(cleaned_string.begin()), ItBinaryT(cleaned_string.end()));

        // Convert the result to a vector of unsigned char
        return std::vector<unsigned char>(result.begin(), result.end());
    }

    // Function to convert Base64 string to cv::Mat
    cv::Mat base64_to_mat(const std::string &base64_string) {
        // Decode Base64 string to binary data
        std::vector<unsigned char> binary_data = base64_decode(base64_string);
        // Convert binary data to cv::Mat
        cv::Mat image = cv::imdecode(binary_data, cv::IMREAD_COLOR);

        return image;
    }

    void sim_serial_write_loop(const bool &required_stop)
    {
        umt::Subscriber<DRIVER::SerialWriteData> serial_write_data_sub("serial_write",0);
        std::string serial_write_topic="/sim_serial_write";
        while(!required_stop)
        {
            try{
                auto serial_write_data = serial_write_data_sub.pop();

                COUT("SERIAL WRITE POP ONCE", BLUE);

                //TODO RAPID_JSON
                //将从umt中sub到的serial_write发送到仿真中：使用rosbridge-websocket
                //rapidjson::Document msg;
                //msg.SetObject();
                //msg.AddMember("x", serial_write_data.yaw_setpoint, msg.GetAllocator());
                //msg.AddMember("y", serial_write_data.pitch_setpoint, msg.GetAllocator());
                //msg.AddMember("z", serial_write_data.detect_number*100+serial_write_data.shoot_flag, msg.GetAllocator());
                //rapidjson::StringBuffer strbuf;
                //rapidjson::Writer<rapidjson::StringBuffer> writer(strbuf);
                //msg.Accept(writer);

                std::string message = "";//"\"op\":\"publish\", \"topic\":\"" + serial_write_topic + "\", \"msg\":" + strbuf.GetString();
                //message = "{" + message + "}";

                // 设置 WebSocket 连接的回调函数
                serial_write_client->on_open = [message](std::shared_ptr<WsClient::Connection> connection) {
                    connection->send(message);
                    // 创建一个新线程，在其中关闭连接
                    std::thread([connection]() {
                        std::this_thread::sleep_for(std::chrono::milliseconds(50));
                        connection->close();
                        serial_write_client->stop();
                    }).detach();
                };

                // 启动 WebSocket 客户端放在一个新线程中
                std::thread([=]() {
                    serial_write_client->start();
                    
                }).detach();
            }catch(umt::MessageError &e){
                std::this_thread::sleep_for(std::chrono::milliseconds(200));
                COUT("serial write loop pop error",RED);
            }
        }
    }

    bool sim_serial_io()
    {
        std::unique_ptr<WsClient> serial_judge_client=std::make_unique<WsClient>(ros_host);

        //////////判断ros connect//////////
        std::string judge_connection_topic="/chatter";
        std::string judge_message = "\"op\":\"subscribe\", \"topic\":\"" + judge_connection_topic + "\"";
        judge_message = "{" + judge_message + "}";

        std::promise<bool> ros_connect_promise;
        std::future<bool> ros_connect_future = ros_connect_promise.get_future();
        int count=0;
        serial_judge_client->on_message=[&ros_connect_promise,&ros_connect_future,&count](std::shared_ptr<WsClient::Connection> /*connection*/, std::shared_ptr<WsClient::InMessage> in_message) {
            //只在回调函数的第一次调用set_value_at_thread_exit
            if (ros_connect_future.valid() && ros_connect_future.wait_for(std::chrono::seconds(0)) == std::future_status::timeout&&count==0) {
                if(in_message->string().empty()){
                    ros_connect_promise.set_value_at_thread_exit(false);
                    count++;
                }
                else{
                    COUT("success to catch /chatter :"<<in_message->string(),MAGENTA);
                    //TODO: judge the details
                    ros_connect_promise.set_value_at_thread_exit(true);
                    count++;
                }
            }
        };
        
        std::promise<bool> thread_stop_promise;
        std::future<bool> thread_stop_future = thread_stop_promise.get_future();

        serial_judge_client->on_open = [judge_message,&serial_judge_client,&thread_stop_promise](std::shared_ptr<WsClient::Connection> connection) {
            connection->send(judge_message);
            // 创建一个新线程，在其中关闭连接
            std::thread([connection,&serial_judge_client,&thread_stop_promise]() {
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
                connection->close();
                serial_judge_client->stop();
                thread_stop_promise.set_value_at_thread_exit(true);
            }).detach();
        };
        
        std::thread([&serial_judge_client]() {
            serial_judge_client->start();
        }).detach();

        COUT("wait ros to connect",YELLOW);

        std::future_status stop_status = thread_stop_future.wait_for(std::chrono::seconds(1));

        if(stop_status == std::future_status::ready)
        {
            bool thread_auto_stop_flag = thread_stop_future.get();
            std::future_status connect_status = ros_connect_future.wait_for(std::chrono::seconds(1));
            if(connect_status == std::future_status::ready){
                bool ros_connect_flag = ros_connect_future.get();
                if(!ros_connect_flag){
                    COUT("failed to connect ros host",RED);
                    return false;
                }
                else{
                    COUT("connected",GREEN);
                }
            }
            else{
                COUT("connection time out",RED);
                return false;
            }
        }
        else
        {
            COUT("connection time out",RED);
            return false;
        }
        //////////判断完毕，可启动sim driver//////////

        serial_success_start = true;

        ////o
        bool required_stop_flag=false;
        std::thread serial_write_thread(sim_serial_write_loop,std::ref(required_stop_flag));

        ////i
        auto serial_robot_status = umt::ObjManager<DRIVER::SerialReadData::RobotStatus>::find_or_create("robot_status");
        auto serial_imu_flag_data = umt::ObjManager<DRIVER::SerialReadData::IMU_Flag>::find_or_create("imu_flag_data");

        std::unique_ptr<WsClient> serial_read_client=std::make_unique<WsClient>(ros_host);

        std::string serial_data_topic="/serial_message";
        std::string message = "\"op\":\"subscribe\", \"topic\":\"" + serial_data_topic + "\"";
        message = "{" + message + "}";

        serial_read_client->on_message = [&serial_robot_status,&serial_imu_flag_data](std::shared_ptr<WsClient::Connection> /*connection*/, std::shared_ptr<WsClient::InMessage> in_message) {
            DRIVER::SerialReadData::IMU_Flag short_data = extractShortSerialData(in_message->string());
            memcpy((uint8_t *)serial_imu_flag_data.get(),(uint8_t *)&short_data,sizeof(short_data));
            COUT("succeed to decode serial message",BLUE);
        };

        serial_read_client->on_open = [message](std::shared_ptr<WsClient::Connection> connection) {
            connection->send(message);
        };

        std::thread([&serial_read_client](){
            serial_read_client->start();
        }).join();

        serial_write_thread.join();
        return false;
    }

    void sim_video_io()
    {   
        auto time_begin = std::chrono::system_clock::now();
        auto begin_milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(time_begin.time_since_epoch()).count();

        std::cout<<"start to capture"<<std::endl;
        std::unique_ptr<WsClient> video_cap_client = std::make_unique<WsClient>(ros_host);
        std::string image_topic="/image_raw";
        std::string message = "\"op\":\"subscribe\", \"topic\":\"" + image_topic + "\"";
        message = "{" + message + "}";

        umt::Publisher<DRIVER::PicStamp> pic_pub("image_raw");

        video_cap_client->on_message = [&pic_pub, begin_milliseconds](std::shared_ptr<WsClient::Connection> /*connection*/, std::shared_ptr<WsClient::InMessage> in_message) {
            std::string data = extractImageData(in_message->string());
            cv::Mat image = base64_to_mat(data);
            cv::flip(image, image, 1);

            COUT("succeed to decode jpeg", GREEN);

            auto time_now = std::chrono::system_clock::now();
            auto now_milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(time_now.time_since_epoch()).count();
            long time_stamp = now_milliseconds - begin_milliseconds;
            COUT("TIME STAMP : "<<time_stamp,CYAN);
            pic_pub.push({image,time_stamp});
        };

        video_cap_client->on_open = [message](std::shared_ptr<WsClient::Connection> connection) {
            connection->send(message);
        };

        std::thread([&video_cap_client](){
            video_cap_client->start();
        }).join();
    }

    void bkg_sim_auto_restart()
    {
        using namespace std::chrono_literals;

        std::thread([=]() {
            while (!sim_serial_io()) {
                std::this_thread::sleep_for(500ms);
            }
        }).detach();

        std::thread([=](){
            while(!serial_success_start)
            {
                std::this_thread::sleep_for(500ms);
            }
            sim_video_io();
        }).detach();
    }



PYBIND11_EMBEDDED_MODULE(SIMULATOR_, m) {
    namespace py = pybind11;
    m.def("bkg_sim_auto_restart", bkg_sim_auto_restart);
}



// UMT_EXPORT_OBJMANAGER_ALIA::IMU_Flag,DRIVER::SerialReadData::IMU_Flag,c){
//     c.def_readwrite("pitch_now",&DRIVER::SerialReadData::IMU_Flag::pitch_now);
//     c.def_readwrite("yaw_now",&DRIVER::SerialReadData::IMU_Flag::yaw_now);
//     c.def_readwrite("aim_request",&DRIVER::SerialReadData::IMU_Flag::aim_request);
//     c.def_readwrite("mode_want",&DRIVER::SerialReadData::IMU_Flag::mode_want);
//     c.def_readwrite("number_want",&DRIVER::SerialReadData::IMU_Flag::number_want);
// }

}  // namespace SIM


