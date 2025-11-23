#include <rclcpp/rclcpp.hpp>
#include "solucionador_labirinto/controlador.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    auto robo = std::make_shared<solucionador::ControladorRobo>();
    
    // atenção: agora chamamos a exploração, não a navegação direta
    try {
        robo->executar_exploracao();
    } catch (const std::exception& e) {
        RCLCPP_ERROR(robo->get_logger(), "Erro durante exploração: %s", e.what());
    }
    
    rclcpp::shutdown();
    return 0;
}
