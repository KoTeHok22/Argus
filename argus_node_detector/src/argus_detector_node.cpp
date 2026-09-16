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
// Нода детекторов (Фаза 3, PLAN.md 8): no_return, geometry, free_space.
// Выходы: /argus/anom_nr, /argus/anom_g, /argus/anom_fs.
// Реализация - Ф3.5.1-Ф3.5.3.

#include <rclcpp/rclcpp.hpp>

namespace argus {

class DetectorNode : public rclcpp::Node {
public:
    DetectorNode() : Node("argus_detector") {
        RCLCPP_INFO(get_logger(),
                    "argus_detector: каркас. Реализация - Ф3.5.x (три детектора "
                    "аномалий)");
    }
};

} // namespace argus

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<argus::DetectorNode>());
    rclcpp::shutdown();
    return 0;
}
