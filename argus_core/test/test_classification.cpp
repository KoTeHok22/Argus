#include <gtest/gtest.h>

#include "argus_core/classification.hpp"

namespace {

argus::ClassificationParams params() {
    return argus::ClassificationParams{};
}

TEST(Classification, PersonTallNarrow) {
    const auto c = argus::classify_object(0.45f, 0.30f, 1.75f, 20, params());
    EXPECT_EQ(c.object_class, argus::ObjectClass::person);
    EXPECT_EQ(c.reason, "tall_narrow");
}

TEST(Classification, LowObjectIsDebris) {
    const auto c = argus::classify_object(0.8f, 0.5f, 0.25f, 30, params());
    EXPECT_EQ(c.object_class, argus::ObjectClass::debris);
}

TEST(Classification, WideFootprintIsEquipment) {
    const auto c = argus::classify_object(3.0f, 2.1f, 1.5f, 400, params());
    EXPECT_EQ(c.object_class, argus::ObjectClass::equipment);
}

TEST(Classification, SparseTallObjectIsUnknown) {
    const auto c = argus::classify_object(0.4f, 0.3f, 1.6f, 2, params());
    EXPECT_EQ(c.object_class, argus::ObjectClass::unknown);
}

TEST(Classification, MidSizedObjectIsUnknown) {
    const auto c = argus::classify_object(0.9f, 0.7f, 0.9f, 50, params());
    EXPECT_EQ(c.object_class, argus::ObjectClass::unknown);
}

TEST(Classification, DebrisTakesPrecedenceOverEquipmentWidth) {
    const auto c = argus::classify_object(2.5f, 2.0f, 0.3f, 200, params());
    EXPECT_EQ(c.object_class, argus::ObjectClass::debris);
}

TEST(Classification, DegenerateFlatTargetIsUnknown) {
    const auto c = argus::classify_object(0.44f, 0.035f, 0.0005f, 31, params());
    EXPECT_EQ(c.object_class, argus::ObjectClass::unknown);
    EXPECT_EQ(c.reason, "no_height_evidence");
}

TEST(Classification, NamesAreStable) {
    EXPECT_STREQ(argus::object_class_name(argus::ObjectClass::person), "person");
    EXPECT_STREQ(argus::object_class_name(argus::ObjectClass::debris), "debris");
    EXPECT_STREQ(argus::object_class_name(argus::ObjectClass::equipment), "equipment");
    EXPECT_STREQ(argus::object_class_name(argus::ObjectClass::unknown), "unknown");
}

} // namespace