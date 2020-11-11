#include "common/phonebook.hpp"
#include "common/plugin.hpp"
#include "common/threadloop.hpp"

using namespace ILLIXR;

// Inherit from `plugin` if you don't need the threadloop
class scheduler_plugin : public plugin {
public:
    scheduler_plugin(std::string name_, phonebook* pb_)
        : threadloop{name_, pb_}
        { }
    virtual void start() override { }
    virtual ~scheduler_plugin() override { }
};

// This line makes the plugin importable by Spindle
PLUGIN_MAIN(scheduler_plugin);
