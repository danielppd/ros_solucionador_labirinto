#include "solucionador_labirinto/algoritmos.hpp"
#include <queue>
#include <map>

namespace solucionador {

MapaProcessado processar_mapa(const std::vector<char>& lista_achatada,
                              int largura, int altura) {
    MapaProcessado resultado;
    resultado.inicio_encontrado = false;
    resultado.alvo_encontrado = false;
    resultado.mapa.resize(altura);
    
    for (int i = 0; i < altura; i++) {
        // pega o pedaço da lista que corresponde a uma linha
        int inicio_linha = i * largura;
        int fim_linha = (i + 1) * largura;
        
        std::vector<char> linha;
        for (int j = inicio_linha; j < fim_linha; j++) {
            linha.push_back(lista_achatada[j]);
        }
        resultado.mapa[i] = linha;
        
        // procura robô e alvo nesta linha
        for (int j = 0; j < static_cast<int>(linha.size()); j++) {
            if (linha[j] == 'r') {
                resultado.inicio = {i, j};
                resultado.inicio_encontrado = true;
            }
            if (linha[j] == 't') {
                resultado.alvo = {i, j};
                resultado.alvo_encontrado = true;
            }
        }
    }
    
    return resultado;
}

std::vector<std::string> bfs_encontrar_caminho(
    const std::vector<std::vector<char>>& mapa,
    const Posicao& inicio,
    const Posicao& alvo) {
    
    int linhas = mapa.size();
    int colunas = mapa[0].size();
    
    // fila armazena: (posicao_atual, caminho_percorrido)
    std::queue<std::pair<Posicao, std::vector<std::string>>> fila;
    fila.push({inicio, {}});
    
    std::set<Posicao> visitados;
    visitados.insert(inicio);
    
    // dicionário: (mudança_linha, mudança_coluna) -> comando_texto
    std::map<std::pair<int, int>, std::string> direcoes = {
        {{-1, 0}, "up"},    // cima diminui a linha
        {{1, 0}, "down"},   // baixo aumenta a linha
        {{0, -1}, "left"},  // esquerda diminui a coluna
        {{0, 1}, "right"}   // direita aumenta a coluna
    };
    
    while (!fila.empty()) {
        auto [pos_atual, caminho] = fila.front();
        fila.pop();
        
        // chegou no alvo?
        if (pos_atual == alvo) {
            return caminho;
        }
        
        // tenta andar para os vizinhos
        for (const auto& [delta, comando] : direcoes) {
            int nova_lin = pos_atual.linha + delta.first;
            int nova_col = pos_atual.coluna + delta.second;
            Posicao nova_pos = {nova_lin, nova_col};
            
            // verifica se não saiu do mapa
            if (nova_lin >= 0 && nova_lin < linhas && 
                nova_col >= 0 && nova_col < colunas) {
                
                // verifica se não é parede ('b') e se não visitou ainda
                if (mapa[nova_lin][nova_col] != 'b' && 
                    visitados.find(nova_pos) == visitados.end()) {
                    
                    visitados.insert(nova_pos);
                    auto novo_caminho = caminho;
                    novo_caminho.push_back(comando);
                    fila.push({nova_pos, novo_caminho});
                }
            }
        }
    }
    
    return {}; // retorna lista vazia se não achar nada
}

Explorador::Explorador(int largura, int altura)
    : largura_(largura), altura_(altura) {
    
    // inicializa tudo como '?'
    mapa_conhecido_.resize(altura_, std::vector<char>(largura_, '?'));
    
    // posição inicial
    pos_atual_ = {1, 1};
    mapa_conhecido_[1][1] = 'f';
    
    visitados_.insert(pos_atual_);
}

void Explorador::atualizar_mapa(const std::string& up, const std::string& down,
                                const std::string& left, const std::string& right) {
    int lin = pos_atual_.linha;
    int col = pos_atual_.coluna;
    
    struct Vizinho {
        int linha;
        int coluna;
        std::string valor;
    };
    
    std::vector<Vizinho> vizinhos = {
        {lin - 1, col, up},
        {lin + 1, col, down},
        {lin, col - 1, left},
        {lin, col + 1, right}
    };
    
    for (const auto& viz : vizinhos) {
        // verifica limites antes de gravar no mapa
        if (viz.linha >= 0 && viz.linha < altura_ &&
            viz.coluna >= 0 && viz.coluna < largura_) {
            mapa_conhecido_[viz.linha][viz.coluna] = viz.valor[0];
        }
    }
}

std::optional<std::string> Explorador::proximo_passo() {
    int lin = pos_atual_.linha;
    int col = pos_atual_.coluna;
    
    struct Movimento {
        int delta_lin;
        int delta_col;
        std::string direcao;
    };
    
    std::vector<Movimento> movimentos = {
        {-1, 0, "up"},
        {0, 1, "right"},
        {1, 0, "down"},
        {0, -1, "left"}
    };
    
    // 1. tenta avançar
    for (const auto& mov : movimentos) {
        int viz_l = lin + mov.delta_lin;
        int viz_c = col + mov.delta_col;
        
        // verifica se o vizinho existe antes de acessar a lista
        bool dentro_do_mapa = (viz_l >= 0 && viz_l < altura_) &&
                             (viz_c >= 0 && viz_c < largura_);
        
        if (dentro_do_mapa) {
            // só entra aqui se a coordenada for válida
            bool eh_parede = (mapa_conhecido_[viz_l][viz_c] == 'b');
            Posicao viz_pos = {viz_l, viz_c};
            bool foi_visitado = (visitados_.find(viz_pos) != visitados_.end());
            
            if (!eh_parede && !foi_visitado) {
                // acha um caminho novo
                pilha_.push(pos_atual_);
                visitados_.insert(viz_pos);
                pos_atual_ = viz_pos;
                return mov.direcao;
            }
        }
    }
    
    // 2. backtracking (voltar)
    if (!pilha_.empty()) {
        Posicao anterior = pilha_.top();
        pilha_.pop();
        
        int delta_l = anterior.linha - lin;
        int delta_c = anterior.coluna - col;
        
        pos_atual_ = anterior;
        
        if (delta_l == -1) return "up";
        if (delta_l == 1) return "down";
        if (delta_c == -1) return "left";
        if (delta_c == 1) return "right";
    }
    
    return std::nullopt;
}

const std::vector<std::vector<char>>& Explorador::get_mapa_conhecido() const {
    return mapa_conhecido_;
}

const Posicao& Explorador::get_posicao_atual() const {
    return pos_atual_;
}

} // namespace solucionador
