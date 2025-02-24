# drone_velctrl
Ardupilot based LLC
# Custol fligth mode
## Create a new mode using input_quaternion():
```
// in your custom mode header
class ModeQuaternion : public Mode {
public:
    ModeQuaternion(void);
    bool init(bool ignore_checks) override;
    void run() override;
    
private:
    Quaternion target_attitude;
    Vector3f target_ang_vel;
};
```
```
// in your custom mode implementation 
void ModeQuaternion::run()
{
    // Apply simple mode transform to pilot inputs if needed
    update_simple_mode();

    // Get pilot inputs and convert to quaternion
    float target_roll, target_pitch;
    get_pilot_desired_lean_angles(target_roll, target_pitch, copter.aparm.angle_max, copter.aparm.angle_max);
    float target_yaw_rate = get_pilot_desired_yaw_rate();
    
    // Create quaternion from euler angles
    target_attitude.from_euler(target_roll, target_pitch, ahrs.yaw);
    
    // Create angular velocity vector
    target_ang_vel.x = target_roll;  // You'll need to convert appropriately
    target_ang_vel.y = target_pitch; // from pilot inputs to desired
    target_ang_vel.z = target_yaw_rate; // angular velocities
    
    // Handle motor spool states similar to stabilize mode
    // See lines in mode_stabilize.cpp:
    startLine: 19
    endLine: 63

    // Call quaternion attitude controller
    attitude_control->input_quaternion(target_attitude, target_ang_vel);

    // Set throttle
    float pilot_desired_throttle = get_pilot_desired_throttle();
    attitude_control->set_throttle_out(pilot_desired_throttle, true, g.throttle_filt);
}
```
### For even lower level control, you could directly set the attitude targets and run the quaternion controller:
```
void ModeQuaternion::run()
{
    // Set attitude targets directly
    attitude_control->_attitude_target = your_desired_quaternion;
    attitude_control->_ang_vel_target = your_desired_angular_velocity;
    
    // Run the quaternion attitude controller
    attitude_control->attitude_controller_run_quat();
    
    // Handle throttle
    attitude_control->set_throttle_out(pilot_desired_throttle, true, g.throttle_filt);
}
```
### The key points to understand:
The input_quaternion() function (see lines in AC_AttitudeControl.cpp):
```
void AC_AttitudeControl::input_quaternion(Quaternion& attitude_desired_quat, Vector3f ang_vel_body)
{
    // update attitude target
    update_attitude_target();

    // Limit the angular velocity
    ang_vel_limit(ang_vel_body, radians(_ang_vel_roll_max), radians(_ang_vel_pitch_max), radians(_ang_vel_yaw_max));
    Vector3f ang_vel_target = attitude_desired_quat * ang_vel_body;

    if (_rate_bf_ff_enabled) {
        Quaternion attitude_error_quat = _attitude_target.inverse() * attitude_desired_quat;
        Vector3f attitude_error_angle;
        attitude_error_quat.to_axis_angle(attitude_error_angle);

        // When acceleration limiting and feedforward are enabled, the sqrt controller is used to compute an euler
        // angular velocity that will cause the euler angle to smoothly stop at the input angle with limited deceleration
        // and an exponential decay specified by _input_tc at the end.
        _ang_vel_target.x = input_shaping_angle(wrap_PI(attitude_error_angle.x), _input_tc, get_accel_roll_max_radss(), _ang_vel_target.x, ang_vel_target.x, radians(_ang_vel_roll_max), _dt);
        _ang_vel_target.y = input_shaping_angle(wrap_PI(attitude_error_angle.y), _input_tc, get_accel_pitch_max_radss(), _ang_vel_target.y, ang_vel_target.y, radians(_ang_vel_pitch_max), _dt);
        _ang_vel_target.z = input_shaping_angle(wrap_PI(attitude_error_angle.z), _input_tc, get_accel_yaw_max_radss(), _ang_vel_target.z, ang_vel_target.z, radians(_ang_vel_yaw_max), _dt);
    } else {
        _attitude_target = attitude_desired_quat;
        _ang_vel_target = ang_vel_target;
    }

    // calculate the attitude target euler angles
    _attitude_target.to_euler(_euler_angle_target);

    // Convert body-frame angular velocity into euler angle derivative of desired attitude
    ang_vel_to_euler_rate(_attitude_target, _ang_vel_target, _euler_rate_target);

    // rotate target and normalize
    Quaternion attitude_desired_update;
    attitude_desired_update.from_axis_angle(ang_vel_target * _dt);
    attitude_desired_quat = attitude_desired_quat * attitude_desired_update;
    attitude_desired_quat.normalize();

    // Call quaternion attitude controller
    attitude_controller_run_quat();
}

```
This provides a high-level interface for quaternion control while still handling things like angular velocity limits and smoothing.
### The attitude controller explanation (see comment block in AC_AttitudeControl.cpp):
The attitude controller works around the concept of the desired attitude, target attitude
and measured attitude. The desired attitude is the attitude input into the attitude controller
that expresses where the higher level code would like the aircraft to move to. The target attitude is moved
to the desired attitude with jerk, acceleration, and velocity limits. The target angular velocities are fed
directly into the rate controllers. The angular error between the measured attitude and the target attitude is
fed into the angle controller and the output of the angle controller summed at the input of the rate controllers.
By feeding the target angular velocity directly into the rate controllers the measured and target attitudes
remain very close together.

All input functions below follow the same procedure
1. define the desired attitude or attitude change based on the input variables
2. update the target attitude based on the angular velocity target and the time since the last loop
3. using the desired attitude and input variables, define the target angular velocity so that it should
   move the target attitude towards the desired attitude
4. if _rate_bf_ff_enabled is not being used then make the target attitude
   and target angular velocities equal to the desired attitude and desired angular velocities.
5. ensure _attitude_target, _euler_angle_target, _euler_rate_target and
   _ang_vel_target have been defined. This ensures input modes can be changed without discontinuity.
6. attitude_controller_run_quat is then run to pass the target angular velocities to the rate controllers and
   integrate them into the target attitude. Any errors between the target attitude and the measured attitude are
   corrected by first correcting the thrust vector until the angle between the target thrust vector measured
   trust vector drops below 2*AC_ATTITUDE_THRUST_ERROR_ANGLE. At this point the heading is also corrected.


## For testing
```
#include "Copter.h"

class ModeQuaternionTest : public Mode {
public:
    ModeQuaternionTest(void);
    bool init(bool ignore_checks) override;
    void run() override;

private:
    const float HOVER_THROTTLE = 0.5f; // Adjust this based on your vehicle
};

bool ModeQuaternionTest::init(bool ignore_checks)
{
    // Initialize any mode-specific variables here
    return true;
}

void ModeQuaternionTest::run()
{
    // Set desired neutral attitude (null quaternion)
    Quaternion target_attitude;
    target_attitude.initialise(); // This creates identity quaternion (no rotation)

    // Set zero angular velocity
    Vector3f target_ang_vel(0.0f, 0.0f, 0.0f);

    // Handle motor spool states
    if (!motors->armed()) {
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
    } else {
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
    }

    switch (motors->get_spool_state()) {
    case AP_Motors::SpoolState::SHUT_DOWN:
        // Motors Stopped
        attitude_control->reset_yaw_target_and_rate();
        attitude_control->reset_rate_controller_I_terms();
        break;

    case AP_Motors::SpoolState::GROUND_IDLE:
        // Landed
        attitude_control->reset_yaw_target_and_rate();
        attitude_control->reset_rate_controller_I_terms_smoothly();
        break;

    case AP_Motors::SpoolState::THROTTLE_UNLIMITED:
        // Flying - run quaternion controller
        attitude_control->input_quaternion(target_attitude, target_ang_vel);
        break;

    case AP_Motors::SpoolState::SPOOLING_UP:
    case AP_Motors::SpoolState::SPOOLING_DOWN:
        // Do nothing
        break;
    }

    // Set constant throttle for hover
    attitude_control->set_throttle_out(HOVER_THROTTLE, true, g.throttle_filt);
}
```
