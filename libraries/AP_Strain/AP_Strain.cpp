#include "AP_Strain.h"

#include <utility>
#include <stdio.h>

#include "AP_Strain_Backend.h"
#include <GCS_MAVLink/GCS.h>
#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_CANManager/AP_CANManager.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>
#include <AP_HAL/I2CDevice.h>

#include <AP_Arming/AP_Arming.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_Vehicle/AP_Vehicle.h>


extern const AP_HAL::HAL& hal;

// singleton instance
AP_Strain *AP_Strain::_singleton;

/*
  AP_Strain constructor
 */
AP_Strain::AP_Strain()
{
    _singleton = this;
}

// initialise the strain object, loading backend drivers
void AP_Strain::init(void)
{   
    if (_num_sensors != 0) {
        // don't re-init if we've found some sensors already
        return;
    }
    // ?? add checks to ensure the sensors are initalized correctly 
    for (uint8_t i=0; i<STRAIN_MAX_INSTANCES; i++) {
        sensors[i].status = Status::NotConnected;
        sensors[i].healthy = false;
        sensors[i].calibrated = false;
        sensors[i].I2C_id = 0x9 + i;
        detect_instance(i);
        drivers[i]->init();
        _num_sensors++;
    }

    init_done = true;
    // AP_HAL::panic("AP_Strain::init() not implemented");
}

void AP_Strain::update(void)
{
    for (uint8_t i=0; i<_num_sensors; i++) {
        if (drivers[i] != nullptr) {
            drivers[i]->update();
        }
    }

    // add logging // TODO
    // Log_Strain();
}

AP_Strain_Backend *AP_Strain::get_backend(uint8_t id) const {
    if (id >= STRAIN_MAX_INSTANCES) {
        return nullptr;
    }
    if (drivers[id] != nullptr) {
        
        return nullptr;
        
    }
    return drivers[id];
};

// zero the strain sensors 
// void AP_Strain::calibrate(void)
// {
//     for (uint8_t i=0; i<num_instances; i++) {
//         if (drivers[i] != nullptr) {
//             drivers[i]->calibrate();
//         }
//     }
// }

// adds backend driver to the front end object
bool AP_Strain::_add_backend(AP_Strain_Backend *backend , uint8_t instance)
{
    if (!backend) {
        return false;
    }
    if (instance >= STRAIN_MAX_INSTANCES) {
        AP_HAL::panic("Too many Strain drivers");
    }
    if (drivers[instance] != nullptr) 
    {
        // we've allocated the same instance twice
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
    }
    drivers[instance] = backend;
    _num_sensors = MAX(_num_sensors, instance+1);
    return true;
}

void AP_Strain::detect_instance(uint8_t instance)
{
    
    #ifndef HAL_BUILD_AP_PERIPH
    if (!hal.util->was_watchdog_armed()) 
    {
        hal.scheduler->delay(100);
    }
    #endif

    FOREACH_I2C(i) 
    {
        if (_add_backend(AP_Strain_Backend::detect(sensors[instance],hal.i2c_mgr->get_device(i, sensors[instance].I2C_id)), instance)) 
        {
            break;
        }
    }
}

namespace AP {

    AP_Strain &strain()
    {
        return *AP_Strain::get_singleton();
    }
    
    };

