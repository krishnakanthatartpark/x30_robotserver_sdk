#pragma once

#include "message_interface.hpp"
#include <vector>
#include <string>
#include <chrono>
#include <nlohmann/json.hpp>
#include <rapidxml/rapidxml.hpp>
#include <sstream>
#include <iomanip>
#include <ctime>

namespace protocol {

/**
 * @brief 导航点信息
 */
struct NavigationPoint {
    int mapId = 0;
    int value = 0;
    double posX = 0.0;
    double posY = 0.0;
    double posZ = 0.0;
    double angleYaw = 0.0;
    int pointInfo = 0;
    int gait = 0;
    int speed = 0;
    int manner = 0;
    int obsMode = 0;
    int navMode = 0;
    int terrain = 0;
    int posture = 0;
};

/**
 * @brief 获取当前时间戳字符串
 * @return 格式化的时间戳字符串
 */
inline std::string getCurrentTimestamp() {
    auto now = std::chrono::system_clock::now();
    auto time_t_now = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&time_t_now), "%Y-%m-%d %H:%M:%S");
    return ss.str();
}

class MessageBase : public IMessage {
public:
    uint16_t sequenceNumber = 0;

    uint16_t getSequenceNumber() const override {
        return sequenceNumber;
    }

    void setSequenceNumber(uint16_t sequenceNumber) override {
        this->sequenceNumber = sequenceNumber;
    }
};

/**
 * @brief 获取实时状态请求
 */
class GetRealTimeStatusRequest : public MessageBase {
public:
    std::string timestamp;

    GetRealTimeStatusRequest() : timestamp(getCurrentTimestamp()) {}

    MessageType getType() const override {
        return MessageType::GET_REAL_TIME_STATUS_REQ;
    }

    std::string serialize() const override {
        // 使用XML格式
        std::stringstream ss;
        ss << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
        ss << "<PatrolDevice>\n";
        ss << "<Type>1002</Type>\n";
        ss << "<Command>1</Command>\n";
        ss << "<Time>" << timestamp << "</Time>\n";
        ss << "<Items/>\n";
        ss << "</PatrolDevice>";
        return ss.str();
    }

    bool deserialize(const std::string&) override {
        return false;
    }
};

/**
 * @brief 获取实时状态响应
 */
class GetRealTimeStatusResponse : public MessageBase {
public:
    int motionState = 0;
    double posX = 0.0;
    double posY = 0.0;
    double posZ = 0.0;
    double angleYaw = 0.0;
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    double speed = 0.0;
    double curOdom = 0.0;
    double sumOdom = 0.0;
    uint64_t curRuntime = 0;
    uint64_t sumRuntime = 0;
    double res = 0.0;
    double x0 = 0.0;
    double y0 = 0.0;
    int h = 0;
    int electricity = 0;
    int location = 0;
    int RTKState = 0;
    int onDockState = 0;
    int gaitState = 0;
    int motorState = 0;
    int chargeState = 0;
    int controlMode = 0;
    int mapUpdateState = 0;

    GetRealTimeStatusResponse() {}

    MessageType getType() const override {
        return MessageType::GET_REAL_TIME_STATUS_RESP;
    }

    std::string serialize() const override {
        // sdk不负责响应的序列化
        return "";
    }

    bool deserialize(const std::string& data) override {
        try {
            rapidxml::xml_document<> doc;
            std::vector<char> buffer(data.begin(), data.end());
            buffer.push_back('\0');
            doc.parse<rapidxml::parse_non_destructive>(&buffer[0]);

            rapidxml::xml_node<>* root = doc.first_node("PatrolDevice");
            if (!root) return false;

            rapidxml::xml_node<>* items_node = root->first_node("Items");
            if (!items_node) return false;

            // 解析各个字段
            auto get_node_value = [&](const char* name, auto& value) {
                rapidxml::xml_node<>* node = items_node->first_node(name);
                if (node) {
                    std::stringstream ss(node->value());
                    ss >> value;
                }
            };

            get_node_value("MotionState", motionState);
            get_node_value("PosX", posX);
            get_node_value("PosY", posY);
            get_node_value("PosZ", posZ);
            get_node_value("AngleYaw", angleYaw);
            get_node_value("Roll", roll);
            get_node_value("Pitch", pitch);
            get_node_value("Yaw", yaw);
            get_node_value("Speed", speed);
            get_node_value("CurOdom", curOdom);
            get_node_value("SumOdom", sumOdom);
            get_node_value("CurRuntime", curRuntime);
            get_node_value("SumRuntime", sumRuntime);
            get_node_value("Res", res);
            get_node_value("X0", x0);
            get_node_value("Y0", y0);
            get_node_value("H", h);
            get_node_value("Electricity", electricity);
            get_node_value("Location", location);
            get_node_value("RTKState", RTKState);
            get_node_value("OnDockState", onDockState);
            get_node_value("GaitState", gaitState);
            get_node_value("MotorState", motorState);
            get_node_value("ChargeState", chargeState);
            get_node_value("ControlMode", controlMode);
            get_node_value("MapUpdateState", mapUpdateState);

            return true;
        } catch (const std::exception& e) {
            return false;
        }
    }
};

/**
 * @brief Motion control request message (ASDU Type 2)
 *
 * This class constructs a motion control request XML message for controlling
 * robot motion commands such as forward, backward, turn, stop, or gait switch.
 */
class MotionControlRequest : public MessageBase {
public:
    int command = 1;      ///< Motion command ID (1=forward, 2=backward, 3=turn left, 4=turn right, 6=stop, 11=left, 12=right, 20=gait switch, etc.)
    float value = -1.0f;  ///< Value associated with the command (velocity in m/s or rad/s, or gait type)

    std::string timestamp;

    MotionControlRequest() : timestamp(getCurrentTimestamp()) {}

    /**
     * @brief Returns the message type
     */
    MessageType getType() const override {
        return MessageType::MOTION_CONTROL_REQ;
    }

    /**
     * @brief Serialize the motion control request into XML
     */
    std::string serialize() const override {
        std::ostringstream ss;

        ss << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
        ss << "<PatrolDevice>\n";
        ss << "  <Type>2</Type>\n";       // ASDU Type 2 = Motion Control
        ss << "  <Command>" << command << "</Command>\n";
        ss << "  <Time>" << timestamp << "</Time>\n";
        ss << "  <Items>\n";
        ss << "    <Value>" << value << "</Value>\n";
        ss << "  </Items>\n";
        ss << "</PatrolDevice>\n"; 

        return ss.str();
    }

    bool deserialize(const std::string&) override {
        // Request does not require deserialization
        return false;
    }

};

/**
 * @brief Motion control response message
 *
 * This class parses the response XML sent by the robot after executing
 * a motion control command. Contains velocity value and error code.
 */
class MotionControlResponse : public MessageBase {
public:
    float value = 0.0f;              ///< Velocity value reported by the robot
    int errorCode = 0;                ///< Execution result: 0=success, 1=failure

    MotionControlResponse() = default;

    MessageType getType() const override {
        return MessageType::MOTION_CONTROL_RESP;
    }

    std::string serialize() const override {
        // SDK does not serialize response
        return "";
    }

    bool deserialize(const std::string& data) override {
        try {
            rapidxml::xml_document<> doc;
            std::vector<char> buffer(data.begin(), data.end());
            buffer.push_back('\0');
            doc.parse<rapidxml::parse_non_destructive>(&buffer[0]);

            rapidxml::xml_node<>* root = doc.first_node("PatrolDevice");
            if (!root) return false;

            rapidxml::xml_node<>* items_node = root->first_node("Items");
            if (!items_node) return false;

            // Helper lambda to extract node values
            auto get_node_value = [&](const char* name, auto& out_value) {
                rapidxml::xml_node<>* node = items_node->first_node(name);
                if (node) {
                    std::stringstream ss(node->value());
                    ss >> out_value;
                }
            };

            get_node_value("Value", value);
            get_node_value("ErrorCode", errorCode);

            return true;
        } catch (const std::exception&) {
            return false;
        }
    }
};

/**
 * @brief 导航任务请求
 */
class NavigationTaskRequest : public MessageBase {
public:
    std::vector<NavigationPoint> points;
    std::string timestamp;

    NavigationTaskRequest() : timestamp(getCurrentTimestamp()) {}

    MessageType getType() const override {
        return MessageType::NAVIGATION_TASK_REQ;
    }

    std::string serialize() const override {
        // 使用XML格式
        std::stringstream ss;
        ss << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
        ss << "<PatrolDevice>\n";
        ss << "<Type>1003</Type>\n";
        ss << "<Command>1</Command>\n";
        ss << "<Time>" << timestamp << "</Time>\n";

        // 添加导航点
        for (const auto& point : points) {
            ss << "<Items>\n";
            ss << "  <MapId>" << point.mapId << "</MapId>\n";
            ss << "  <Value>" << point.value << "</Value>\n";
            ss << "  <PosX>" << point.posX << "</PosX>\n";
            ss << "  <PosY>" << point.posY << "</PosY>\n";
            ss << "  <PosZ>" << point.posZ << "</PosZ>\n";
            ss << "  <AngleYaw>" << point.angleYaw << "</AngleYaw>\n";
            ss << "  <PointInfo>" << point.pointInfo << "</PointInfo>\n";
            ss << "  <Gait>" << point.gait << "</Gait>\n";
            ss << "  <Speed>" << point.speed << "</Speed>\n";
            ss << "  <Manner>" << point.manner << "</Manner>\n";
            ss << "  <ObsMode>" << point.obsMode << "</ObsMode>\n";
            ss << "  <NavMode>" << point.navMode << "</NavMode>\n";
            ss << "  <Terrain>" << point.terrain << "</Terrain>\n";
            ss << "  <Posture>" << point.posture << "</Posture>\n";
            ss << "</Items>\n";
        }

        ss << "</PatrolDevice>";
        return ss.str();
    }

    bool deserialize(const std::string&) override {
        return false;
    }
};

/**
 * @brief 导航任务响应
 */
class NavigationTaskResponse : public MessageBase {
public:
    int value = 0;
    ErrorCode_Navigation errorCode = ErrorCode_Navigation::SUCCESS;
    int errorStatus = 0;

    NavigationTaskResponse() {}

    MessageType getType() const override {
        return MessageType::NAVIGATION_TASK_RESP;
    }

    std::string serialize() const override {
        // sdk不负责响应的序列化
        return "";
    }

    bool deserialize(const std::string& data) override {
        try {
            rapidxml::xml_document<> doc;
            std::vector<char> buffer(data.begin(), data.end());
            buffer.push_back('\0');
            doc.parse<rapidxml::parse_non_destructive>(&buffer[0]);

            rapidxml::xml_node<>* root = doc.first_node("PatrolDevice");
            if (!root) return false;

            rapidxml::xml_node<>* items_node = root->first_node("Items");
            if (!items_node) return false;

            // 解析各个字段
            auto get_node_value = [&](const char* name, auto& value) {
                rapidxml::xml_node<>* node = items_node->first_node(name);
                if (node) {
                    std::stringstream ss(node->value());
                    ss >> value;
                }
            };

            get_node_value("Value", value);

            // 解析错误码
            int error_code_value = 0;
            get_node_value("ErrorCode", error_code_value);
            errorCode = static_cast<ErrorCode_Navigation>(error_code_value);

            get_node_value("ErrorStatus", errorStatus);

            return true;
        } catch (const std::exception& e) {
            return false;
        }
    }
};

/**
 * @brief 查询任务状态请求
 */
class QueryStatusRequest : public MessageBase {
public:
    std::string timestamp;

    QueryStatusRequest() : timestamp(getCurrentTimestamp()) {}

    MessageType getType() const override {
        return MessageType::QUERY_STATUS_REQ;
    }

    std::string serialize() const override {
        // 使用XML格式
        std::stringstream ss;
        ss << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
        ss << "<PatrolDevice>\n";
        ss << "<Type>1007</Type>\n";
        ss << "<Command>1</Command>\n";
        ss << "<Time>" << timestamp << "</Time>\n";
        ss << "<Items/>\n";
        ss << "</PatrolDevice>";
        return ss.str();
    }

    bool deserialize(const std::string&) override {
        return false;
    }
};

/**
 * @brief 查询任务状态响应
 */
class QueryStatusResponse : public MessageBase {
public:
    int value = 0;
    int status = 0;
    ErrorCode_QueryStatus errorCode = ErrorCode_QueryStatus::COMPLETED;

    QueryStatusResponse() {}

    MessageType getType() const override {
        return MessageType::QUERY_STATUS_RESP;
    }

    std::string serialize() const override {
        // sdk不负责响应的序列化
        return "";
    }

    bool deserialize(const std::string& data) override {
        try {
            rapidxml::xml_document<> doc;
            std::vector<char> buffer(data.begin(), data.end());
            buffer.push_back('\0');
            doc.parse<rapidxml::parse_non_destructive>(&buffer[0]);

            rapidxml::xml_node<>* root = doc.first_node("PatrolDevice");
            if (!root) return false;

            rapidxml::xml_node<>* items_node = root->first_node("Items");
            if (!items_node) return false;

            // 解析各个字段
            auto get_node_value = [&](const char* name, auto& value) {
                rapidxml::xml_node<>* node = items_node->first_node(name);
                if (node) {
                    std::stringstream ss(node->value());
                    ss >> value;
                }
            };

            get_node_value("Value", value);

            // 解析状态
            get_node_value("Status", status);

            // 解析错误码
            int error_code_value = 0;
            get_node_value("ErrorCode", error_code_value);
            errorCode = static_cast<ErrorCode_QueryStatus>(error_code_value);

            return true;
        } catch (const std::exception& e) {
            return false;
        }
    }
};

/**
 * @brief 取消任务请求
 */
class CancelTaskRequest : public MessageBase {
public:
    std::string timestamp;

    CancelTaskRequest() : timestamp(getCurrentTimestamp()) {}

    MessageType getType() const override {
        return MessageType::CANCEL_TASK_REQ;
    }

    std::string serialize() const override {
        // 使用XML格式
        std::stringstream ss;
        ss << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
        ss << "<PatrolDevice>\n";
        ss << "<Type>1004</Type>\n";
        ss << "<Command>1</Command>\n";
        ss << "<Time>" << timestamp << "</Time>\n";
        ss << "<Items/>\n";
        ss << "</PatrolDevice>";
        return ss.str();
    }

    bool deserialize(const std::string&) override {
        return false;
    }
};

/**
 * @brief 取消任务响应
 */
class CancelTaskResponse : public MessageBase {
public:
    ErrorCode_CancelTask errorCode = ErrorCode_CancelTask::SUCCESS;

    CancelTaskResponse() {}

    MessageType getType() const override {
        return MessageType::CANCEL_TASK_RESP;
    }

    std::string serialize() const override {
        // sdk不负责响应的序列化
        return "";
    }

    bool deserialize(const std::string& data) override {
        try {
            rapidxml::xml_document<> doc;
            std::vector<char> buffer(data.begin(), data.end());
            buffer.push_back('\0');
            doc.parse<rapidxml::parse_non_destructive>(&buffer[0]);

            rapidxml::xml_node<>* root = doc.first_node("PatrolDevice");
            if (!root) return false;

            rapidxml::xml_node<>* items_node = root->first_node("Items");
            if (!items_node) return false;

            // 解析错误码
            int error_code_value = 0;
            rapidxml::xml_node<>* error_code_node = items_node->first_node("ErrorCode");
            if (error_code_node) {
                std::stringstream ss(error_code_node->value());
                ss >> error_code_value;
                errorCode = static_cast<ErrorCode_CancelTask>(error_code_value);
            }

            return true;
        } catch (const std::exception& e) {
            return false;
        }
    }
};

} // namespace protocol
