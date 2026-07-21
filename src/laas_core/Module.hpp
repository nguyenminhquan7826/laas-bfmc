#pragma once

namespace laas {

class IModule {
public:
    virtual ~IModule() = default;
    virtual bool init() = 0;
    virtual void tick() = 0;
    virtual void stop() = 0;
};

}  // namespace laas
