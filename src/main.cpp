#include <rclcpp/rclcpp.hpp>
#include "solucionador_labirinto/controlador.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    auto robo = std::make_shared<solucionador::ControladorRobo>();
    
    // Verifica argumentos para decidir o modo
    std::string modo = "navegar"; // padrão
    if (argc > 1) {
        for (int i = 1; i < argc; ++i) {
            std::string arg = argv[i];
            if (arg == "--explorar" || arg == "explorar") {
                modo = "explorar";
            }
        }
    }

    try {
        if (modo == "explorar") {
            RCLCPP_INFO(robo->get_logger(), "Modo selecionado: EXPLORAÇÃO (Parte 2)");
            robo->executar_exploracao();
        } else {
            RCLCPP_INFO(robo->get_logger(), "Modo selecionado: NAVEGAÇÃO DIRETA (Parte 1)");
            RCLCPP_INFO(robo->get_logger(), "Dica: Para rodar a exploração, use o argumento '--explorar'");
            robo->executar_navegacao_direta();
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(robo->get_logger(), "Erro durante execução: %s", e.what());
    }
    
    rclcpp::shutdown();
    return 0;
}
