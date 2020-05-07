#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>

#include <glm/glm.hpp>
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/ext.hpp>
#include <glm/gtx/transform.hpp>

#include <json-c/json.h>

struct CameraCheckpoint {
    glm::vec3 point;
    glm::vec3 dir;
    int t;
};


CameraCheckpoint cpFromObj(json_object* obj) {
    CameraCheckpoint cc;

    json_object* tobj;
    
    json_object_object_get_ex(obj, "x", &tobj);
    cc.point.x = float(json_object_get_double(tobj));
    json_object_object_get_ex(obj, "y", &tobj);
    cc.point.y = float(json_object_get_double(tobj));
    json_object_object_get_ex(obj, "z", &tobj);
    cc.point.z = float(json_object_get_double(tobj));

    json_object_object_get_ex(obj, "dirx", &tobj);
    cc.dir.x = float(json_object_get_double(tobj));
    json_object_object_get_ex(obj, "diry", &tobj);
    cc.dir.y = float(json_object_get_double(tobj));
    json_object_object_get_ex(obj, "dirz", &tobj);
    cc.dir.z = float(json_object_get_double(tobj));

    json_object_object_get_ex(obj, "t", &tobj);

    cc.t = json_object_get_int(tobj);

    return cc;
}

CameraCheckpoint getInterpolatedCheckpoint(const CameraCheckpoint& cc1,
					    const CameraCheckpoint& cc2,
					    int t) {
    
    float coeff = float(t - cc1.t) / float(cc2.t - cc1.t);
    CameraCheckpoint cc;
    cc.point = coeff * cc2.point + (1 - coeff) * cc1.point;
    cc.dir = glm::normalize(coeff * cc2.dir + (1 - coeff) * cc1.dir);

    return cc;
}

glm::mat4 getInterpolatedView(const CameraCheckpoint& cc1,
			      const CameraCheckpoint& cc2,
			      int t) {
    CameraCheckpoint cc = getInterpolatedCheckpoint(cc1, cc2, t);
    
    glm::vec3 up(0.0f, 1.0f, 0.0f);
    glm::vec3 x_axis = glm::normalize(glm::cross(cc.dir, up));
    glm::vec3 y_axis = glm::normalize(glm::cross(x_axis, cc.dir));

    glm::mat3 rotation = glm::transpose(glm::mat3(x_axis, y_axis, -cc.dir));
    glm::mat4 transform = glm::mat4(rotation) * glm::translate(-cc.point);
    return transform;
}

std::vector<CameraCheckpoint> getCameraCheckpointsFromObjects(json_object* obj) {
    std::vector<CameraCheckpoint> cps;
    
    array_list* arr = json_object_get_array(obj);

    size_t ss = array_list_length(arr);

    for(size_t i = 0; i < ss; i++) {
	json_object* tobj;
	tobj = (json_object*)array_list_get_idx(arr, i);
	CameraCheckpoint cc = cpFromObj(tobj);
	cps.push_back(cc);
    }

    return cps;
}

std::vector<CameraCheckpoint> parse_to_cc(const std::string& file_name) {
    std::ifstream fs(file_name);

    if(!fs) {
	std::cerr << "Could not open file " << file_name << std::endl;
	exit(0);
    }
    
    std::stringstream buffer;
    buffer << fs.rdbuf();
    std::string ss = buffer.str();

    
    // json list = json::parse(ss); NB
    json_object* jo = json_tokener_parse(ss.c_str());

    if(!json_object_is_type(jo, json_type_array)) {
	std::cerr << "JSON object was not a list!" << std::endl;
	std::exit(0);
    }

    std::vector<CameraCheckpoint> cps = getCameraCheckpointsFromObjects(jo);
    
    return cps;
}

std::vector<glm::mat4> getPath(const std::string& file_name) {
    
    std::vector<CameraCheckpoint> cps = parse_to_cc(file_name);

    std::vector<glm::mat4> views;
    int t = cps[0].t;
    size_t current_cp = 0;
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

std::pair<glm::vec3, glm::vec3> getInterpolatedComp(const CameraCheckpoint& cp1,
						    const CameraCheckpoint& cp2,
						    int t) {
    CameraCheckpoint cc = getInterpolatedCheckpoint(cp1, cp2, t);

    float yaw = std::atan2(cc.dir.x, cc.dir.z);
    float pitch = std::atan2(cc.dir.y, sqrt(cc.dir.x * cc.dir.x + cc.dir.z * cc.dir.z));

    return std::pair<glm::vec3, glm::vec3>(glm::vec3(pitch, yaw, 0.0f) * 180.0 / M_PI, cc.point);
}

// Decomposed as in returning path as  (rotation, position) pairs, where rotation is on Euler form
std::vector<std::pair<glm::vec3, glm::vec3> > getPathDecomposed(const std::string& str) {
    std::vector<CameraCheckpoint> cps = parse_to_cc(str);

    std::vector<std::pair<glm::vec3, glm::vec3> > comps;

    int t = cps[0].t;
    size_t current_cp = 0;
    while(current_cp < cps.size() - 1) {
      
        comps.push_back(getInterpolatedComp(cps[current_cp], cps[current_cp + 1], t));
	// std::cout << "Rotation " << t << ": " << glm::to_string(comps[comps.size() - 1].first) << std::endl;
	// std::cout << "Position " << t << ": " << glm::to_string(comps[comps.size() - 1].second) << std::endl;
	t++;
	if (t >= cps[current_cp + 1].t) {
	    current_cp++;
	}
    }

    comps.push_back(getInterpolatedComp(cps[current_cp - 1], cps[current_cp], t));

    return comps;
}
