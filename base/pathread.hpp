#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>

#include <nlohmann/json.hpp>
#include <glm/glm.hpp>
#define GLM_ENABLE_EXPERIMENTAL true
#include <glm/ext.hpp>
#include <glm/gtx/transform.hpp>

using json = nlohmann::json;

struct CameraCheckpoint {
    glm::vec3 point;
    glm::vec3 dir;
    int t;
};

CameraCheckpoint cpFromObj(const json& j) {
    CameraCheckpoint cc;
    cc.point = glm::vec3(j["x"], j["y"], j["z"]);
    cc.dir = glm::vec3(j["dirx"], j["diry"], j["dirz"]);
    cc.t = int(j["t"]);
    return cc;
}

glm::mat4 getInterpolatedView(const CameraCheckpoint& cc1,
			      const CameraCheckpoint& cc2,
			      int t) {
    float coeff = float(t - cc1.t) / float(cc2.t - cc1.t);
    CameraCheckpoint cc;
    cc.point = coeff * cc2.point + (1 - coeff) * cc1.point;
    cc.dir = glm::normalize(coeff * cc2.dir + (1 - coeff) * cc1.dir);

    glm::vec3 up(0.0f, 1.0f, 0.0f);
    glm::vec3 x_axis = glm::normalize(glm::cross(cc.dir, up));
    glm::vec3 y_axis = glm::normalize(glm::cross(x_axis, cc.dir));

    glm::mat3 rotation = glm::transpose(glm::mat3(x_axis, y_axis, -cc.dir));
    glm::mat4 transform = glm::mat4(rotation) * glm::translate(-cc.point);
    return transform;
}

std::vector<glm::mat4> getPath(const std::string& str) {
    std::ifstream fs(str);

    if(!fs) {
	std::cerr << "Could not open file " << str << std::endl;
	exit(0);
    }
    
    std::stringstream buffer;
    buffer << fs.rdbuf();
    std::string ss = buffer.str();

    std::cout << "String sent to parser: " << ss << std::endl;
    
    json list = json::parse(ss);

    if(!list.is_array()) {
	std::cerr << "JSON object was not a list!" << std::endl;
	std::exit(0);
    }
    
    std::vector<CameraCheckpoint> cps;
    for (json::iterator it = list.begin(); it != list.end(); ++it) {
	cps.push_back(cpFromObj(*it));
    }

    std::vector<glm::mat4> views;
    int t = cps[0].t;
    int current_cp = 0;
    while(current_cp < cps.size() - 1) {

	views.push_back(getInterpolatedView(cps[current_cp], cps[current_cp + 1], t));
	t++;
	if (t >= cps[current_cp + 1].t) {
	    current_cp++;
	}
    }

    views.push_back(getInterpolatedView(cps[current_cp - 1], cps[current_cp], t));

    return views;
}
