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
// Нода модели тоннеля (Фаза 2, PLAN.md 7.1.2): карта свободного пространства
// (DUFOMap/Bonxai) + профиль сечения (7.1.4). Выход: /argus/model.
// Правило обновления: только чистыми кадрами (7.1.3). Реализация - Ф2.3.2.

#include <rclcpp/rclcpp.hpp>

namespace argus {

class TunnelModelNode : public rclcpp::Node {
public:
    TunnelModelNode() : Node("argus_tunnel_model") {
        RCLCPP_INFO(get_logger(),
                    "argus_tunnel_model: каркас. Реализация - Ф2.3.2 (модель "
                    "свободного пространства + профиль тоннеля)");
    }
};

} // namespace argus

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<argus::TunnelModelNode>());
    rclcpp::shutdown();
    return 0;
}
