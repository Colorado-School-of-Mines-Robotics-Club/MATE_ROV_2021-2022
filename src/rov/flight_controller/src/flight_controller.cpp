#include <array>
#include <chrono>
#include <cmath>
#include <memory>

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"

#include <rclcpp/rclcpp.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "rov_interfaces/msg/bno055_data.hpp"
#include "rov_interfaces/msg/thruster_setpoints.hpp"
#include "rov_interfaces/msg/pwm.hpp"
#include "rov_interfaces/srv/create_continuous_servo.hpp"

#include "Thruster.hpp"

using namespace std::chrono_literals;

#define NUM_THRUSTERS 8
#define MASS 100
#define IXX 100
#define IXY 100
#define IXZ 100
#define IYX 100
#define IYY 100
#define IYZ 100
#define IZX 100
#define IZY 100
#define IZZ 100

class FlightController : public rclcpp::Node {
public:
    FlightController() : Node(std::string("flight_controller")) {
        this->inertia_tensor << IXX,IXY,IXZ,
                                IYX,IYY,IYZ,
                                IZX,IZY,IZZ;
        // define thrusters TODO: replace with a config file? (temp values atm)
        thrusters[0] = Thruster(Eigen::Vector3d(0,0,0), Eigen::Vector3d(0,0,0));
        thrusters[1] = Thruster(Eigen::Vector3d(0,0,0), Eigen::Vector3d(0,0,0));
        thrusters[2] = Thruster(Eigen::Vector3d(0,0,0), Eigen::Vector3d(0,0,0));
        thrusters[3] = Thruster(Eigen::Vector3d(0,0,0), Eigen::Vector3d(0,0,0));
        thrusters[4] = Thruster(Eigen::Vector3d(0,0,0), Eigen::Vector3d(0,0,0));
        thrusters[5] = Thruster(Eigen::Vector3d(0,0,0), Eigen::Vector3d(0,0,0));
        thrusters[6] = Thruster(Eigen::Vector3d(0,0,0), Eigen::Vector3d(0,0,0));
        thrusters[7] = Thruster(Eigen::Vector3d(0,0,0), Eigen::Vector3d(0,0,0));
        // this lambda is horrible and probably should be refactored, but.... it only runs once on construction time so does it even matter??
        // alternatively, we could create config file that is updated when thruster config files are changed.
        // cmake could create task to do what this lambda is doing, but at compile time!
        auto concatThrusterGeometry = [](std::array<Thruster, NUM_THRUSTERS> thrusters)->Eigen::Matrix<double, 6, NUM_THRUSTERS> {
            Eigen::MatrixXd geometry;
            Eigen::MatrixXd temp_geometry;
            Eigen::Vector3d i_hat(1,0,0);
            Eigen::Vector3d j_hat(0,1,0);
            Eigen::Vector3d k_hat(0,0,1);
            for(int i = 0; i < NUM_THRUSTERS; i++) {
                Thruster t = thrusters[i];
                // calculate linear and rotation contribution
                Eigen::Vector3d linear_contribution(t.maximum_thrust);
                Eigen::Vector3d rotation_contribution(t.position.cross(t.maximum_thrust));

                // concatenate them
                Eigen::MatrixXd toConcat(linear_contribution.rows()+rotation_contribution.rows(), linear_contribution.cols());
                toConcat << linear_contribution, 
                             rotation_contribution;
                temp_geometry = geometry;
                Eigen::MatrixXd geometry(6, geometry.cols()+1);
                geometry << temp_geometry, toConcat;
            }
            return geometry;
        };

        this->thruster_geometry = concatThrusterGeometry(this->thrusters);
        // this->thruster_geometry_inverse = this->thruster_geometry.inverse();

        // use PWM service to register thrusters on PCA9685
        this->registerThrusters();
        
        thruster_setpoint_subscription.subscribe(this, "thruster_setpoints");
        bno_data_subscription.subscribe(this, "bno055_data");
        
        _publisher = this->create_publisher<rov_interfaces::msg::PWM>("PWM", 10);

        // sync = http://wiki.ros.org/message_filters
        typedef message_filters::sync_policies::ApproximateTime<rov_interfaces::msg::ThrusterSetpoints, rov_interfaces::msg::BNO055Data> approximate_policy;
        message_filters::Synchronizer<approximate_policy> sync(approximate_policy(10), thruster_setpoint_subscription, bno_data_subscription);
        sync.setMaxIntervalDuration(rclcpp::Duration(0.15,0));
        sync.registerCallback(std::bind(&FlightController::update, this, std::placeholders::_1, std::placeholders::_2));

        stall_detector = this->create_wall_timer(250ms, std::bind(&FlightController::StallDetector, this));
    }
private:
    void registerThrusters() {
        // create service client
        auto pca9685 = this->create_client<rov_interfaces::srv::CreateContinuousServo>("create_continuous_servo");
        for(int i=0; i < NUM_THRUSTERS; i++) {
            // create continuous servo creation requests on channels 0 -> NUM_THRUSTERS
            auto req = std::make_shared<rov_interfaces::srv::CreateContinuousServo_Request>();
            req->channel = i;
            // ensure service is not busy
            while(!pca9685->wait_for_service(1s)) {
                if (!rclcpp::ok()) {
                    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
                    exit(0);
                }
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
            }

            // create lambda function to handle asynchronous callbacks
            using ServiceResponseFuture = rclcpp::Client<rov_interfaces::srv::CreateContinuousServo>::SharedFuture;
            auto registerThrusterCallback = [this](ServiceResponseFuture future) {
                auto res = future.get();
                if(res->result)
                    RCLCPP_INFO(this->get_logger(), "Successfully registered continuous servo on channel %i", res->channel);
                else
                    RCLCPP_ERROR(this->get_logger(), "Unsuccessfully registered continuous servo on channel %i", res->channel);
            };
            // asynchronously send these servo creation requests
            auto future_result = pca9685->async_send_request(req, registerThrusterCallback);
        }
    }

    void update(const rov_interfaces::msg::ThrusterSetpoints::ConstSharedPtr& setpoints, const rov_interfaces::msg::BNO055Data::ConstSharedPtr& bno_data) {
        Eigen::Vector3d desired_force;
        Eigen::Vector3d desired_torque;
        // fill the matrixes
        
        auto now = std::chrono::high_resolution_clock::now();
        int dt_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - this->last_updated).count();
        this->last_updated = now;
        translation_setpoints(0,0) = setpoints->vx;
        translation_setpoints(1,0) = setpoints->vy;
        translation_setpoints(2,0) = setpoints->vz;
        attitude_setpoints(0,0) = setpoints->omegax;
        attitude_setpoints(1,0) = setpoints->omegay;
        attitude_setpoints(2,0) = setpoints->omegaz;

        // update desired force
        // Linear, discrete-time PID controller
        Eigen::Vector3d linear_accel(bno_data->linearaccel.i, bno_data->linearaccel.j,bno_data->linearaccel.k); // m/s^2
        Eigen::Vector3d linear_velocity = linear_velocity + linear_accel * dt_ms / 1000; // get an approximation of linear velocity
        Eigen::Vector3d linear_velocity_err;
        linear_velocity_err[0] = translation_setpoints(0,0) - linear_velocity[0];
        linear_velocity_err[1] = translation_setpoints(1,0) - linear_velocity[1];
        linear_velocity_err[2] = translation_setpoints(2,0) - linear_velocity[2];

        // lambdas for linear PID loop
        auto P = [](double& kp, Eigen::Vector3d& linear_velocity_err)->Eigen::Vector3d{
            return kp * linear_velocity_err;
        };
        auto I = [](double& ki, Eigen::Vector3d& linear_velocity_err, int& dt_ms)->Eigen::Vector3d{return ki * linear_velocity_err * dt_ms / 1000;};
        auto D = [](double& kd, Eigen::Vector3d& linear_velocity_err, Eigen::Vector3d& linear_velocity_err_last, int& dt_ms)->Eigen::Vector3d{
            return kd* (linear_velocity_err - linear_velocity_err_last) / (static_cast<double>(dt_ms) / 1000);
        };

        // P + I + D
        desired_torque = P(kp,linear_velocity_err) 
                        + (linear_integral += I(ki, linear_velocity_err, dt_ms)) 
                        + D(kd, linear_velocity_err, linear_velocity_err_last, dt_ms);

        // this is direct mapping from velocity to output force
        // warning: no proportional controller might be unweildy to use
        // desired_force(0,0) = (translation_setpoints(0,0) - translation_setpoints_last(0,0));
        // desired_force(1,0) = (translation_setpoints(1,0) - translation_setpoints_last(1,0));
        // desired_force(2,0) = (translation_setpoints(2,0) - translation_setpoints_last(2,0));
        // desired_force = MASS / dt_ms * desired_force;

        // update desired torque
        // TODO: look at this if performance needs to be improved (loses accuracy tho)
        // see https://stackoverflow.com/questions/24197182/efficient-quaternion-angular-velocity/24201879#24201879
        Eigen::Vector3d ha = dt_ms * 0.5 * attitude_setpoints; // vector of half angle
        double l = ha.norm(); // magnitude
        if (l > 0) {
            ha *= sin(l) / l;
            quaternion_reference = Eigen::Quaterniond(cos(l), ha.x(), ha.y(), ha.z());
        } else {
            quaternion_reference = Eigen::Quaterniond(1.0, ha.x(), ha.y(), ha.z());
        }

        // Non Linear P^2 Quaternion based control scheme
        // see: http://www.diva-portal.org/smash/get/diva2:1010947/FULLTEXT01.pdf
        auto quaternion_measured = Eigen::Quaterniond(bno_data->orientation.w, bno_data->orientation.i, bno_data->orientation.j, bno_data->orientation.k);
        auto q_err = quaternion_reference * quaternion_measured.conjugate(); // hamilton product (hopefully)
        Eigen::Vector3d axis_err;

        if(q_err.w() < 0) { // this could be simplified to a negation but eh
            axis_err = q_err.conjugate().vec();
        } else {
            axis_err = q_err.vec();
        }

        Eigen::Vector3d omega = Eigen::Vector3d(bno_data->gyroscope.i, bno_data->gyroscope.j, bno_data->gyroscope.k);
        desired_torque = (-Pq * axis_err) - (Pw * omega);

        // control allocation
        Eigen::Matrix<double, 6, 1> forcesAndTorques;
        forcesAndTorques(0,0) = desired_force.x();
        forcesAndTorques(1,0) = desired_force.y();
        forcesAndTorques(2,0) = desired_force.z();
        forcesAndTorques(3,0) = desired_torque.x();
        forcesAndTorques(4,0) = desired_torque.y();
        forcesAndTorques(5,0) = desired_torque.z();
        
        Eigen::Matrix<double, 6, 1> throttles = this->thruster_geometry_inverse * forcesAndTorques;

        // publish PWM values
        for(int i = 0; i < NUM_THRUSTERS; i++) {
            rov_interfaces::msg::PWM msg;
            msg.angle_or_throttle = static_cast<float>(throttles(i,0)); // this is a source of noise in output signals, may cause system instability??
            msg.is_continuous_servo = true;
            msg.channel = i;
            _publisher->publish(msg);
        }

        // update last
        linear_accel_last = linear_accel;
        linear_velocity_err_last = linear_velocity_err;
        translation_setpoints_last = this->translation_setpoints;
        attitude_setpoints_last = this->attitude_setpoints;
        quaternion_reference_last = this->quaternion_reference;
    }

    void StallDetector() {
        auto now = std::chrono::high_resolution_clock::now();
        auto duration_since_last = this->last_updated - now;
        int milliseconds_since_last = std::chrono::duration_cast<std::chrono::milliseconds>(duration_since_last).count();
        if(milliseconds_since_last > 250) {
            // WE HAVE LOST CONNECTION, STOP THRUSTING
            // TODO:implement
        }
    }

    message_filters::Subscriber<rov_interfaces::msg::ThrusterSetpoints> thruster_setpoint_subscription;
    message_filters::Subscriber<rov_interfaces::msg::BNO055Data> bno_data_subscription;
    rclcpp::Publisher<rov_interfaces::msg::PWM>::SharedPtr _publisher;

    rclcpp::TimerBase::SharedPtr stall_detector;

    std::chrono::time_point<std::chrono::high_resolution_clock> last_updated;

    Eigen::Vector3d translation_setpoints = Eigen::Vector3d(3,1);
    Eigen::Vector3d attitude_setpoints = Eigen::Vector3d(3,1);
    Eigen::Quaterniond quaternion_reference;
    Eigen::Vector3d translation_setpoints_last = Eigen::Vector3d(3,1);
    Eigen::Vector3d attitude_setpoints_last = Eigen::Vector3d(3,1);
    Eigen::Quaterniond quaternion_reference_last;
    Eigen::Vector3d linear_accel_last;
    Eigen::Vector3d linear_integral;
    Eigen::Vector3d linear_velocity_err_last;
    std::array<Thruster, NUM_THRUSTERS> thrusters;
    Eigen::MatrixXd thruster_geometry = Eigen::MatrixXd(6, NUM_THRUSTERS);
    Eigen::MatrixXd thruster_geometry_inverse = Eigen::MatrixXd(6, NUM_THRUSTERS);
    Eigen::MatrixXd inertia_tensor = Eigen::MatrixXd(3,3);

    double Pq = 1.0, Pw = 1.0; // TODO: tune these gain constants
    double kp = 1.0, ki = 1.0, kd = 1.0;
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FlightController>());
    rclcpp::shutdown();
    return 0;
}
