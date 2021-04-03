
#include <iostream>
#include <memory>

#include "robot_vars.h"
#include "gnc/control/ramsete.h"

#include "rapidjson/document.h"
#include "rapidjson/writer.h"
#include "rapidjson/stringbuffer.h"

int main(int argc, char **argv) {

    // Initialize the robot variables shared memory
    std::shared_ptr<RobotVariables> robot_vars = std::make_shared<RobotVariables>();

    std::shared_ptr<RamseteController> controller = 
        std::make_shared<RamseteController>(
            &robot_vars->ref_traj, 
            &robot_vars->robot_state, 
            &robot_vars->ctrl_out, 
            0.4, 
            18);

    controller->output();

    const char* json = "{\"project\":\"rapidjson\",\"stars\":10}";
    rapidjson::Document d;
    d.Parse(json);

    // 2. Modify it by DOM.
    rapidjson::Value& s = d["stars"];
    s.SetInt(s.GetInt() + 1);

    // 3. Stringify the DOM
    rapidjson::StringBuffer buffer;
    rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
    d.Accept(writer);

    // Output {"project":"rapidjson","stars":11}
    std::cout << buffer.GetString() << std::endl;
    return 0;
}