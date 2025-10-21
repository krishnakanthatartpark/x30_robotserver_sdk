#pragma once

#include "types.h"
#include <memory>
#include <string>
#include <future>

namespace robotserver_sdk {

// Forward declaration to hide implementation details
class RobotServerSdkImpl;

/**
 * @brief Main class for the RobotServer SDK
 *
 * This class provides the primary interface for communicating with a quadruped robot,
 * including connection management, navigation task control, and status queries.
 */
class RobotServerSdk {
public:
    /**
     * @brief Constructor
     * @param options SDK configuration options
     */
    explicit RobotServerSdk(const SdkOptions& options = SdkOptions());

    /**
     * @brief Destructor
     */
    ~RobotServerSdk();

    /**
     * @brief Disable copy constructor
     */
    RobotServerSdk(const RobotServerSdk&) = delete;

    /**
     * @brief Disable assignment operator
     */
    RobotServerSdk& operator=(const RobotServerSdk&) = delete;

    /**
     * @brief Connect to the quadruped robot control system
     * @param host Host address
     * @param port Port number
     * @return True if the connection is successful
     */
    bool connect(const std::string& host, uint16_t port);

    /**
     * @brief Disconnect from the quadruped robot control system
     */
    void disconnect();

    /**
     * @brief Check if currently connected
     * @return True if connected
     */
    bool isConnected() const;

    /**
     * @brief request1002 Retrieve the robot's real-time status
     * @return Real-time status information
     */
    RealTimeStatus request1002_RunTimeState();

    /**
     * @brief request2 Motion control for the robot
     */
    void request2_Motion_Control(int command, float value, MotionControlResultCallback callback);

    /**
     * @brief request1003 Asynchronously start a navigation task using a callback
     * @param points List of navigation points
     * @param callback Callback function for navigation result
     */
    void request1003_StartNavTask(const std::vector<NavigationPoint>& points, NavigationResultCallback callback);

    /**
     * @brief request1004 Cancel the current navigation task
     * @return True if the operation was successful
     */
    bool request1004_CancelNavTask();

    /**
     * @brief request1007 Query the current navigation task status
     * @return Result of the navigation task status query
     */
    TaskStatusResult request1007_NavTaskState();

    /**
     * @brief Get the SDK version
     * @return SDK version string
     */
    static std::string getVersion();

private:
    std::unique_ptr<RobotServerSdkImpl> impl_; ///< PIMPL implementation
};

} // namespace robotserver_sdk
