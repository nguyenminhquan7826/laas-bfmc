#pragma once

namespace laas {

enum class RuntimeState {
    INIT = 0,
    READY,
    RUNNING,
    STOPPED,
    ERROR,
    EMERGENCY_STOP
};

}  // namespace laas
