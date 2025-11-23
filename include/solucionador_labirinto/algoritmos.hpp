#ifndef SOLUCIONADOR_LABIRINTO_ALGORITMOS_HPP
#define SOLUCIONADOR_LABIRINTO_ALGORITMOS_HPP

#include <vector>
#include <string>
#include <tuple>
#include <optional>
#include <set>
#include <stack>

namespace solucionador {

// estrutura para representar uma posição no mapa
struct Posicao {
    int linha;
    int coluna;
    
    bool operator==(const Posicao& other) const {
        return linha == other.linha && coluna == other.coluna;
    }
    
    bool operator<(const Posicao& other) const {
        if (linha != other.linha) return linha < other.linha;
        return coluna < other.coluna;
    }
};

// estrutura para resultado do processamento do mapa
struct MapaProcessado {
    std::vector<std::vector<char>> mapa;
    Posicao inicio;
    Posicao alvo;
    bool inicio_encontrado;
    bool alvo_encontrado;
};

// processa mapa achatado e encontra robô e alvo
MapaProcessado processar_mapa(const std::vector<char>& lista_achatada, 
                              int largura, int altura);

// encontra caminho otimizado usando BFS
std::vector<std::string> bfs_encontrar_caminho(
    const std::vector<std::vector<char>>& mapa,
    const Posicao& inicio,
    const Posicao& alvo
);

// classe para exploração do labirinto
class Explorador {
public:
    Explorador(int largura = 29, int altura = 29);
    
    // atualiza mapa com leitura dos sensores
    void atualizar_mapa(const std::string& up, const std::string& down,
                       const std::string& left, const std::string& right);
    
    // decide próximo passo na exploração
    std::optional<std::string> proximo_passo();
    
    // retorna o mapa descoberto
    const std::vector<std::vector<char>>& get_mapa_conhecido() const;
    
    // retorna posição atual
    const Posicao& get_posicao_atual() const;

private:
    int largura_;
    int altura_;
    std::vector<std::vector<char>> mapa_conhecido_;
    Posicao pos_atual_;
    std::set<Posicao> visitados_;
    std::stack<Posicao> pilha_;
};

} // namespace solucionador

#endif // SOLUCIONADOR_LABIRINTO_ALGORITMOS_HPP
