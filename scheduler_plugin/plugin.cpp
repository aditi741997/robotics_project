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
	if (!file.good()) {
		std::cerr << "Could not open: " << path << std::endl;
		abort();
	}
	size_t result;
	file >> result;
	return result;
}

static void copy(std::string inp, std::string out) {
	auto inp_file = std::ifstream {inp, std::ios::binary};
	auto out_file = std::ofstream {out, std::ios::binary};
	if (inp_file.fail()) { std::cerr << "Could not open: " << inp << std::endl; }
	if (out_file.fail()) { std::cerr << "Could not open: " << out << std::endl; }
	out_file << inp_file.rdbuf();
	if (inp_file.fail()) { std::cerr << "Could not read: " << inp << std::endl; }
	if (out_file.fail()) { std::cerr << "Could not write: " << out << std::endl; }
}

// Inherit from `plugin` if you don't need the threadloop
class scheduler_plugin : public plugin, public DAGControllerFE {
public:
    scheduler_plugin(std::string name_, phonebook* pb_)
        : plugin{name_, pb_}
		, sb{pb->lookup_impl<switchboard>()}
	{
		if (is_scheduler()) {
			size_t fc = std::stoi(std::getenv("ILLIXR_SCHEDULER_FC"));
			auto dag_file = std::string{std::getenv("ILLIXR_SCHEDULER_CONFIG")};
			controller = std::make_unique<DAGControllerBE>(dag_file, this, false, "no", "no", 1, fc, 3, 4, 5, 6, 7, 1);
			for (const auto& pair : controller->node_dag_mc.name_id_map) {
				std::string name = pair.first;
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
	}

	virtual void start2() override {
		if (controller) {
			controller->start();
		}
	}

	virtual void trigger_node(std::string name, bool reset) {
		// CPU_TIMER_TIME_FUNCTION_INFO(cpu_timer::make_type_eraser<FrameInfo>(std::to_string(id), "", 0));
		// DEBUG2(name, ::gettid())
		triggers.at(name).put(new (triggers.at(name).allocate()) switchboard::event_wrapper<bool> {true});
	}

	virtual void stop() override {
		if (controller) {
			controller.reset();
		}
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
