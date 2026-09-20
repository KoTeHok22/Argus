#pragma once

#include <cstdint>
#include <string>

namespace argus {

enum class ObjectClass : uint8_t {
    unknown = 0,
    person = 1,
    debris = 2,
    equipment = 3,
};

struct ClassificationParams {
    float min_height_evidence_m = 0.05f;
    float person_min_height_m = 1.10f;
    float person_max_height_m = 2.30f;
    float person_max_footprint_m = 1.00f;
    uint32_t person_min_points = 4;
    float debris_max_height_m = 0.60f;
    float equipment_min_footprint_m = 1.60f;
};

struct Classification {
    ObjectClass object_class = ObjectClass::unknown;
    std::string reason;
};

Classification classify_object(float extent_x_m, float extent_y_m, float extent_z_m,
                               uint32_t point_count, const ClassificationParams& p);

const char* object_class_name(ObjectClass c);

} // namespace argus