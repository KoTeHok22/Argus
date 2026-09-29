#include "argus_core/classification.hpp"

#include <algorithm>

namespace argus {

Classification classify_object(float extent_x_m, float extent_y_m, float extent_z_m,
                               uint32_t point_count, const ClassificationParams& p) {
    Classification out;
    const float footprint = std::max(extent_x_m, extent_y_m);
    const float height = extent_z_m;

    if (height < p.min_height_evidence_m) {
        out.object_class = ObjectClass::unknown;
        out.reason = "no_height_evidence";
        return out;
    }
    if (height >= p.person_min_height_m && height <= p.person_max_height_m &&
        footprint <= p.person_max_footprint_m && point_count >= p.person_min_points) {
        out.object_class = ObjectClass::person;
        out.reason = "tall_narrow";
        return out;
    }
    if (height > 0.0f && height < p.debris_max_height_m) {
        out.object_class = ObjectClass::debris;
        out.reason = "low_object";
        return out;
    }
    if (footprint >= p.equipment_min_footprint_m) {
        out.object_class = ObjectClass::equipment;
        out.reason = "large_footprint";
        return out;
    }
    out.object_class = ObjectClass::unknown;
    out.reason = "unclassified";
    return out;
}

const char* object_class_name(ObjectClass c) {
    switch (c) {
        case ObjectClass::person:
            return "person";
        case ObjectClass::debris:
            return "debris";
        case ObjectClass::equipment:
            return "equipment";
        default:
            return "unknown";
    }
}

} // namespace argus