#include <mutex>
#include "common/phonebook.hpp"
#include "common/plugin.hpp"
#include "common/threadloop.hpp"
#include "common/debug.hpp"
#include "../src/beginner_tutorials/dag_controller_be.h"

using namespace ILLIXR;

// Inherit from `plugin` if you don't need the threadloop
class scheduler_plugin : public plugin, public DAGControllerFE {
public:
    scheduler_plugin(std::string name_, phonebook* pb_)
        : plugin{name_, pb_}
		, controller{std::make_unique<DAGControllerBE>("../robotics_project/illixr_dag.txt", this, false, "no", "no", 1, 3, 3, 4, 5, 6, 7, 1)}
		, sb{pb->lookup_impl<switchboard>()}
	{
		std::cout << "Hello world " << controller->node_dag_mc.name_id_map.size() << "\n";
		for (const auto& pair : controller->node_dag_mc.name_id_map) {
			std::string name = pair.first;
			std::cerr << "Creating entry for " << pair.first << ": " << pair.second << "\n";
			sb->schedule<thread_info>(
				id,
				name + "_thread_id",
				[this, name](switchboard::ptr<const thread_info> event, size_t) {
					std::lock_guard<std::mutex> lock{mutex};
					std::cerr << "thread_id for " << event->name << " is " << event->pid << std::endl;
					controller->recv_node_info(event->name, event->pid, ::getpid());
				}
			);
			triggers.insert(std::make_pair<std::string, switchboard::writer<switchboard::event_wrapper<bool>>>(
				std::move(name),
				std::move(sb->get_writer<switchboard::event_wrapper<bool>>(name + "_trigger"))
			));
		}

		sb->schedule<switchboard::event_wrapper<bool>>(
			id,
			controller->get_last_node_cc_name() + "_completion",
			[this](switchboard::ptr<const switchboard::event_wrapper<bool>>, size_t event) {
				controller->recv_critical_exec_end();
			}
		);
	}

	virtual void start2() override {
		controller->start();
	}

	virtual void trigger_node(std::string name, bool reset) {
		// CPU_TIMER_TIME_FUNCTION_INFO(cpu_timer::make_type_eraser<FrameInfo>(std::to_string(id), "", 0));
		triggers.at(name).put(new (triggers.at(name).allocate()) switchboard::event_wrapper<bool> {true});
	}

	virtual void stop() override {
		controller.reset();
		plugin::stop();
		// calls controller's destructor
	}

private:
	const std::shared_ptr<switchboard> sb;
	std::mutex mutex;
	std::unordered_map<std::string, switchboard::writer<switchboard::event_wrapper<bool>>> triggers;
	std::unique_ptr<DAGControllerBE> controller;
};

// This line makes the plugin importable by Spindle
PLUGIN_MAIN(scheduler_plugin);
