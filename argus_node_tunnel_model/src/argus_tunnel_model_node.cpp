
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
