#include "common/phonebook.hpp"
#include "common/plugin.hpp"
#include "common/threadloop.hpp"
#include "../src/beginner_tutorials/dag_controller_be.h"

using namespace ILLIXR;

// Inherit from `plugin` if you don't need the threadloop
class scheduler_plugin : public plugin {
public:
    scheduler_plugin(std::string name_, phonebook* pb_)
        : plugin{name_, pb_}
		, controller{"../nav2d", 2, 2, 2, 2}
		, sb{pb->lookup_impl<switchboard>()}
        {
			for (const auto& pair : controller.node_dag.name_id_map) {
				std::string name = pair.first;
				sb->schedule<thread_info>(id, name + "_thread_id", [this](switchboard::ptr<const thread_info>, size_t) {
					controller.recv_node_info();
				});
				triggers.insert(std::make_pair<std::string, switchboard::writer<switchboard::event_wrapper<bool>>>(std::move(name), std::move(sb->get_writer<switchboard::event_wrapper<bool>>(name + "_trigger"))));
			}
		}

private:
	DAGControllerBE controller;
	const std::shared_ptr<switchboard> sb;
	std::unordered_map<std::string, switchboard::writer<switchboard::event_wrapper<bool>>> triggers;
	
};

// This line makes the plugin importable by Spindle
PLUGIN_MAIN(scheduler_plugin);