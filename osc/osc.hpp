#include <bopt/program.h>

namespace osc {

class OSC {
    void init();
    void loop() {
        for (auto &task : tasks_) {
            // Update program parameters
            // task.error ...
            // auto out = task.pid_gains.compute_output(task.error);
            // program.set_parameter(task.parameters.desired_task_acceleration)
            // for(int i =0; i < 100; ++i) {
            // program.set_parametr(task.parameters.desired_task_acceleration[i],
            // out[i]);
            // }
        }
    }

    void set_reference();

    void get_tasks();

    std::vector<Task<double>> tasks_;
};

}  // namespace osc
