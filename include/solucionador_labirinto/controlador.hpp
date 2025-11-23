#ifndef SOLUCIONADOR_LABIRINTO_CONTROLADOR_HPP
#define SOLUCIONADOR_LABIRINTO_CONTROLADOR_HPP

#include <rclcpp/rclcpp.hpp>
#include <cg_interfaces/srv/move_cmd.hpp>
#include <cg_interfaces/msg/robot_sensors.hpp>
#include "solucionador_labirinto/algoritmos.hpp"
#include <memory>

namespace solucionador {

class ControladorRobo : public rclcpp::Node {
public:
    ControladorRobo();
    
    void executar_exploracao();

private:
    void callback_sensores(const cg_interfaces::msg::RobotSensors::SharedPtr msg);
    void mover_robo(const std::string& direcao);
    void validar_mapa();
    
    rclcpp::Client<cg_interfaces::srv::MoveCmd>::SharedPtr cliente_movimento_;
    rclcpp::Subscription<cg_interfaces::msg::RobotSensors>::SharedPtr subscription_;
    
    cg_interfaces::msg::RobotSensors::SharedPtr ultimo_sensor_;
    std::unique_ptr<Explorador> explorador_;
};

} // namespace solucionador

#endif // SOLUCIONADOR_LABIRINTO_CONTROLADOR_HPP
