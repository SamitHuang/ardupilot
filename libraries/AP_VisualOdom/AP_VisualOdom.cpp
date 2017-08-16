/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "AP_VisualOdom.h"
#include "AP_VisualOdom_Backend.h"
#include "AP_VisualOdom_MAV.h"

extern const AP_HAL::HAL &hal;


// table of user settable parameters
const AP_Param::GroupInfo AP_VisualOdom::var_info[] = {

    // @Param: _TYPE
    // @DisplayName: Visual odometry camera connection type
    // @Description: Visual odometry camera connection type
    // @Values: 0:None,1:MAV
    // @User: Advanced
    AP_GROUPINFO("_TYPE", 0, AP_VisualOdom, _type, 0),

    // @Param: _POS_X
    // @DisplayName: Visual odometry camera X position offset
    // @Description: X position of the camera in body frame. Positive X is forward of the origin.
    // @Units: m
    // @User: Advanced

    // @Param: _POS_Y
    // @DisplayName: Visual odometry camera Y position offset
    // @Description: Y position of the camera in body frame. Positive Y is to the right of the origin.
    // @Units: m
    // @User: Advanced

    // @Param: _POS_Z
    // @DisplayName: Visual odometry camera Z position offset
    // @Description: Z position of the camera in body frame. Positive Z is down from the origin.
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("_POS", 1, AP_VisualOdom, _pos_offset, 0.0f),

    // @Param: _ORIENT
    // @DisplayName: Visual odometery camera orientation
    // @Description: Visual odometery camera orientation
    // @Values: 0:Forward, 2:Right, 4:Back, 6:Left, 24:Up, 25:Down
    // @User: Advanced
    AP_GROUPINFO("_ORIENT", 2, AP_VisualOdom, _orientation, ROTATION_NONE),

    AP_GROUPEND
};

AP_VisualOdom::AP_VisualOdom()
{
    AP_Param::setup_object_defaults(this, var_info);
}

// detect and initialise any sensors
void AP_VisualOdom::init()
{
    // create backend
    if (_type == AP_VisualOdom_Type_MAV) {
        _driver = new AP_VisualOdom_MAV(*this);
    }
}

// return true if sensor is enabled
bool AP_VisualOdom::enabled() const
{
    return ((_type != AP_VisualOdom_Type_None) && (_driver != nullptr));
}

// return true if sensor is basically healthy (we are receiving data)
bool AP_VisualOdom::healthy() const
{
    if (!enabled()) {
        return false;
    }

    // healthy if we have received sensor messages within the past 300ms
    return ((AP_HAL::millis() - _state.last_update_ms) < AP_VISUALODOM_TIMEOUT_MS);
}

// consume VISION_POSITION_DELTA MAVLink message
void AP_VisualOdom::handle_msg(mavlink_message_t *msg)
{
    // exit immediately if not enabled
    if (!enabled()) {
        return;
    }

    // call backend
    if (_driver != nullptr) {
        _driver->handle_msg(msg);
    }
}

/*
bool AP_VisualOdom::is_lost() {
    if(AP_HAL::millis() - _raw_state.last_update_ms < 500)
        return true;
    else
        return false;
    //todo: add counter
}
*/

//save new vio message to _raw_state
void AP_VisualOdom::save_vio_state(mavlink_message_t *msg){
    mavlink_vision_position_delta_t packet;

    mavlink_msg_vision_position_delta_decode(msg, &packet);
    //check if confidence 
    
    const Vector3f angle_delta(packet.angle_delta[0], packet.angle_delta[1], packet.angle_delta[2]);
    const Vector3f position_delta(packet.position_delta[0], packet.position_delta[1], packet.position_delta[2]);
    _raw_state.angle_delta = angle_delta;
    //_state.angle_delta.rotate((enum Rotation)_frontend._orientation.get());
    // rotate and store position_delta
    _raw_state.position_delta = position_delta;
    //_frontend._state.position_delta.rotate((enum Rotation)_frontend._orientation.get());
    _raw_state.time_delta_usec = packet.time_delta_usec;
    _raw_state.confidence = packet.confidence;
    _raw_state.last_update_ms = AP_HAL::millis();
}

VisualOdomState_t AP_VisualOdom::get_vio_state(){
    return _raw_state;
}

//get velocity in the head-forward-right frame from the vio state  
bool AP_VisualOdom::calc_vel_xy(float &vel_fw, float &vel_rg){
    /*
    Vector3f vel;
    Matrix3f dcm;

    if(_raw_state.confidence < 50 )
        return false;

    //rotate to forward-horizon frame
    Vector3f vel_fw_rg;
    vel = _raw_state.position_delta * (1000000.0f/_raw_state.time_delta_usec);
    dcm.from_euler(ahrs.roll, ahrs.pitch, 0);
    vel_fw_rg = dcm * vel;

    vel_fw = vel_fw_rg.x;
    vel_rg = vel_fw_rg.y;
    */
    return true;
}



