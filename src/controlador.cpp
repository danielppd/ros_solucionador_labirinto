#include "solucionador_labirinto/controlador.hpp"
#include <chrono>
#include <thread>

using namespace std::chrono_literals;

namespace solucionador {

ControladorRobo::ControladorRobo() : Node("controlador_robo") {
    cliente_movimento_ = this->create_client<cg_interfaces::srv::MoveCmd>("/move_command");
    cliente_mapa_ = this->create_client<cg_interfaces::srv::GetMap>("/get_map");
    
    // assinatura (Subscribe) nos sensores para a Parte 2
    subscription_ = this->create_subscription<cg_interfaces::msg::RobotSensors>(
        "/culling_games/robot_sensors",
        10,
        std::bind(&ControladorRobo::callback_sensores, this, std::placeholders::_1)
    );
    
    ultimo_sensor_ = nullptr; // guarda a última leitura
    explorador_ = std::make_unique<Explorador>(); // instancia nossa IA
}

void ControladorRobo::callback_sensores(
    const cg_interfaces::msg::RobotSensors::SharedPtr msg) {
    // chamado automaticamente sempre que o robô manda dados dos sensores
    ultimo_sensor_ = msg;
}

void ControladorRobo::mover_robo(const std::string& direcao) {
    auto request = std::make_shared<cg_interfaces::srv::MoveCmd::Request>();
    request->direction = direcao;
    
    auto result = cliente_movimento_->async_send_request(request);
    
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) ==
        rclcpp::FutureReturnCode::SUCCESS) {
        // movimento executado com sucesso
    }
}

void ControladorRobo::executar_exploracao() {
    RCLCPP_INFO(this->get_logger(), "Iniciando Exploração Sincronizada (Parte 2)...");
    
    // espera inicial para estabilizar
    std::this_thread::sleep_for(1s);
    
    while (rclcpp::ok()) {
        // 1. garante leitura nova:
        // limpa a leitura anterior para forçar o recebimento de uma nova
        ultimo_sensor_ = nullptr;
        
        // fica preso aqui até chegar uma mensagem nova do sensor
        while (ultimo_sensor_ == nullptr) {
            rclcpp::spin_some(this->get_node_base_interface());
            std::this_thread::sleep_for(100ms);
        }
        
        // 2. atualiza mapa (agora temos certeza que o sensor é atual)
        explorador_->atualizar_mapa(
            ultimo_sensor_->up,
            ultimo_sensor_->down,
            ultimo_sensor_->left,
            ultimo_sensor_->right
        );
        
        // checagem de vitória
        std::vector<std::string> sensores = {
            ultimo_sensor_->up,
            ultimo_sensor_->down,
            ultimo_sensor_->left,
            ultimo_sensor_->right
        };
        
        bool alvo_proximo = false;
        for (const auto& sensor : sensores) {
            if (sensor == "t") {
                alvo_proximo = true;
                break;
            }
        }
        
        if (alvo_proximo) {
            RCLCPP_INFO(this->get_logger(), "!!! ALVO AVISTADO (VIZINHO) !!!");
            // opcional: mover para o alvo para finalizar bonito
            if (ultimo_sensor_->up == "t") mover_robo("up");
            else if (ultimo_sensor_->down == "t") mover_robo("down");
            else if (ultimo_sensor_->left == "t") mover_robo("left");
            else if (ultimo_sensor_->right == "t") mover_robo("right");
            break;
        }
        
        // 3. decide
        auto proxima_direcao = explorador_->proximo_passo();
        
        // 4. age e espera
        if (proxima_direcao.has_value()) {
            RCLCPP_INFO(this->get_logger(), "Movendo: %s", proxima_direcao.value().c_str());
            mover_robo(proxima_direcao.value());
            
            // pausa longa para garantir que o simulador mova o robô fisicamente
            // antes de tentarmos ler o sensor de novo na próxima volta
            std::this_thread::sleep_for(800ms);
        } else {
            RCLCPP_INFO(this->get_logger(), "Exploração finalizada!");
            break;
        }
    }
    
    validar_mapa();
}

void ControladorRobo::validar_mapa() {
    // parte final: prova que o mapa gerado serve para navegar
    RCLCPP_INFO(this->get_logger(), "--- VALIDANDO O MAPA GERADO ---");
    
    // pega o mapa que a gente acabou de descobrir
    const auto& mapa_final = explorador_->get_mapa_conhecido();
    
    // encontra onde está o alvo no mapa descoberto
    std::optional<Posicao> alvo;
    for (int r = 0; r < static_cast<int>(mapa_final.size()); r++) {
        for (int c = 0; c < static_cast<int>(mapa_final[0].size()); c++) {
            if (mapa_final[r][c] == 't') {
                alvo = Posicao{r, c};
                break;
            }
        }
        if (alvo.has_value()) break;
    }
    
    if (alvo.has_value()) {
        RCLCPP_INFO(this->get_logger(), "Alvo encontrado no mapa mental em: (%d, %d)",
                    alvo.value().linha, alvo.value().coluna);
        
        // roda o BFS da Parte 1 usando APENAS o mapa que descobrimos
        auto rota = bfs_encontrar_caminho(mapa_final, {1, 1}, alvo.value());
        RCLCPP_INFO(this->get_logger(), 
                    "Prova de conceito: Rota otimizada calculada com sucesso: %zu passos.",
                    rota.size());
    } else {
        RCLCPP_ERROR(this->get_logger(), 
                     "Exploramos tudo e não achamos o alvo! (Ou ele não foi marcado)");
    }
}

void ControladorRobo::executar_navegacao_direta() {
    RCLCPP_INFO(this->get_logger(), "Iniciando Parte 1: Navegação com Mapa...");

    // 1. Aguardar o serviço de mapa ficar disponível
    while (!cliente_mapa_->wait_for_service(1s)) {
        if (!rclcpp::ok()) return;
        RCLCPP_INFO(this->get_logger(), "Aguardando serviço /get_map...");
    }

    // 2. Chamar o serviço para pegar o mapa
    auto request = std::make_shared<cg_interfaces::srv::GetMap::Request>();
    auto result_future = cliente_mapa_->async_send_request(request);

    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) !=
        rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(this->get_logger(), "Falha ao chamar serviço de mapa");
        return;
    }

    auto response = result_future.get();
    
    // Converter std::vector<std::string> para std::vector<char>
    std::vector<char> mapa_chars;
    mapa_chars.reserve(response->occupancy_grid_flattened.size());
    for (const auto& s : response->occupancy_grid_flattened) {
        if (!s.empty()) {
            mapa_chars.push_back(s[0]);
        } else {
            mapa_chars.push_back(' '); // Espaço vazio como fallback
        }
    }

    // 3. Processar o mapa recebido (converter lista achatada para matriz)
    auto dados_mapa = processar_mapa(
        mapa_chars, 
        response->occupancy_grid_shape[1], // largura
        response->occupancy_grid_shape[0]  // altura
    );

    if (!dados_mapa.inicio_encontrado || !dados_mapa.alvo_encontrado) {
        RCLCPP_ERROR(this->get_logger(), "Mapa inválido: inicio ou alvo não encontrados.");
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Mapa recebido! Calculando rota de (%d,%d) ate (%d,%d)...",
        dados_mapa.inicio.linha, dados_mapa.inicio.coluna,
        dados_mapa.alvo.linha, dados_mapa.alvo.coluna);

    // 4. Calcular a rota otimizada (BFS)
    auto caminho = bfs_encontrar_caminho(dados_mapa.mapa, dados_mapa.inicio, dados_mapa.alvo);

    if (caminho.empty()) {
        RCLCPP_ERROR(this->get_logger(), "Não foi possível encontrar um caminho!");
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Caminho encontrado com %zu passos. Executando...", caminho.size());

    // 5. Executar os movimentos
    for (const auto& direcao : caminho) {
        if (!rclcpp::ok()) break;
        RCLCPP_INFO(this->get_logger(), "Movendo: %s", direcao.c_str());
        mover_robo(direcao);
        // Pequeno delay para visualização
        std::this_thread::sleep_for(500ms); 
    }

    RCLCPP_INFO(this->get_logger(), "Parte 1 concluída com sucesso!");
}

} // namespace solucionador
