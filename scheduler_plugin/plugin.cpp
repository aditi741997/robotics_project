#include <mutex>
#include "common/phonebook.hpp"
#include "common/plugin.hpp"
#include "common/threadloop.hpp"
#include "common/debug.hpp"
#include "../src/beginner_tutorials/dag_controller_be.h"

#define DEBUG2(x1, x2) std::cerr << __FILE__ << ':' << __LINE__ << ": " << #x1 << "=" << x1 << ", " << #x2 << "=" << x2 << std::endl;
#define DEBUG(x) std::cerr << __FILE__ << ':' << __LINE__ << ": " << #x << "=" << x << std::endl;

using namespace ILLIXR;

static size_t read_int_file(std::string path) {
	auto file = std::ifstream{path};
	size_t result;
	file >> result;
	return result;
}

// Inherit from `plugin` if you don't need the threadloop
class scheduler_plugin : public plugin, public DAGControllerFE {
public:
    scheduler_plugin(std::string name_, phonebook* pb_)
        : plugin{name_, pb_}
		, cpu_freq{read_int_file("/sys/devices/system/cpu/cpu0/cpufreq/scaling_max_freq") / 1000}
		, root{std::string{"../robotics_project/"}}
		, fc{read_int_file(root + "illixr_fc_" + std::to_string(cpu_freq) + ".txt")}
		, controller{std::make_unique<DAGControllerBE>(root + "illixr_dag_" + std::to_string(cpu_freq) + ".txt", this, false, "no", "no", 1, fc, 3, 4, 5, 6, 7, 1)}
		, sb{pb->lookup_impl<switchboard>()}
	{
		if (!is_scheduler()) { abort(); }
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
			sb->schedule<switchboard::event_wrapper<bool>>(
				id,
				name + "_completion",
				[this, name](switchboard::ptr<const switchboard::event_wrapper<bool>>, size_t event) {
					// controller->update_ci(name, ci);
					if (name == controller->get_last_node_cc_name()) {
						controller->recv_critical_exec_end();
					} else {
					controller->notify_node_exec_end(name);
					}
				}
			);
			triggers.insert(std::make_pair<std::string, switchboard::writer<switchboard::event_wrapper<bool>>>(
				std::move(name),
				std::move(sb->get_writer<switchboard::event_wrapper<bool>>(name + "_trigger"))
			));
		}
	}

	virtual void start2() override {
		controller->start();
	}

	virtual void trigger_node(std::string name, bool reset) {
		// CPU_TIMER_TIME_FUNCTION_INFO(cpu_timer::make_type_eraser<FrameInfo>(std::to_string(id), "", 0));
		// DEBUG2(name, ::gettid())
		triggers.at(name).put(new (triggers.at(name).allocate()) switchboard::event_wrapper<bool> {true});
	}

	virtual void stop() override {
		controller.reset();
		plugin::stop();
		// calls controller's destructor
	}

private:
	const std::shared_ptr<switchboard> sb;
	size_t cpu_freq;
	std::string root;
	size_t fc;
	std::mutex mutex;
	std::unordered_map<std::string, switchboard::writer<switchboard::event_wrapper<bool>>> triggers;
	std::unique_ptr<DAGControllerBE> controller;
};

// This line makes the plugin importable by Spindle
PLUGIN_MAIN(scheduler_plugin);
