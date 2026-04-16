#pragma once

#include "types.hpp"

/** Test / bench IMU: writes a fixed sample into `RobotState`. */
struct DummyImu {
    ImuSample sample{};
    bool enabled{true};

    void inject(RobotState& st, TimePointUs timestamp_us) const {
        if (!enabled) {
            st.has_imu = false;
            st.imu = ImuSample{};
            return;
        }
        st.imu = sample;
        st.imu.timestamp_us = timestamp_us;
        st.imu.valid = true;
        st.has_imu = true;
    }
};
