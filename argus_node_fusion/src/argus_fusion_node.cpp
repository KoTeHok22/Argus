// Copyright 2026 Argus Team
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Нода слияния (Фаза 4, PLAN.md 9): голосование детекторов, габарит,
// кластеризация, трекинг, тревоги. Выходы: /argus/obstacles, /argus/gauge,
// /argus/markers, /argus/explain. Реализация - Ф4.x (LifecycleNode).

#include <rclcpp/rclcpp.hpp>

namespace argus {

class FusionNode : public rclcpp::Node {
public:
    FusionNode() : Node("argus_fusion") {
        RCLCPP_INFO(get_logger(),
                    "argus_fusion: каркас. Реализация - Ф4.x (голосование, "
                    "габарит, кластеризация, трекинг). Перевод на LifecycleNode "
                    "- Ф5.6.6.");
    }
};

} // namespace argus

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<argus::FusionNode>());
    rclcpp::shutdown();
    return 0;
}
