#pragma once

namespace PREDICTOR
{
    struct PredictionPackage
    {
        uint8_t detect_number;
        uint8_t shoot_flag;
        float pitch_setpoint;
        float yaw_setpoint;
    };    

    template<typename T> class VariableLocker
    {
    private:
        std::atomic<bool> locked;
        std::atomic<T> value;

    public:
        VariableLocker(): locked(false), value(T()) {}

        void setValue(T val)
        {
            if(locked == false)
            {
                value = val;
            }

            if(val == T(1))
            {
                locked = true;
                std::thread t([this](){
                    std::this_thread::sleep_for(std::chrono::milliseconds(300));
                    locked = false;
                });
                t.detach();
            }
        }

        T getValue()
        {
            return value;
        }

        bool isLocked()
        {
            return locked;
        }
    };
    
} //namespace PREDICTOR