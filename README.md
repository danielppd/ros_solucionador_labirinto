# Solucionador de Labirinto - Culling Games (ROS 2) - C++

## Descrição
Este projeto é uma solução em C++ para o desafio de navegação autônoma utilizando ROS 2. O objetivo é controlar um robô em um ambiente simulado (grid), fazendo-o encontrar o caminho otimizado entre um ponto de partida e um alvo.

O algoritmo implementado utiliza **BFS (Busca em Largura)** para garantir o menor caminho possível em um grafo não ponderado, e **DFS com Backtracking** para exploração completa do labirinto.

## Tecnologias Utilizadas
- **ROS 2 (Humble)**
- **C++17**
- **Algoritmos:** 
  - Breadth-First Search (BFS) para caminho ótimo
  - Depth-First Search (DFS) com Backtracking para exploração
- **Simulador:** Culling Games (Mapas 2D)

## Estrutura do Projeto

```
solucionador_labirinto/
├── CMakeLists.txt
├── package.xml
├── include/
│   └── solucionador_labirinto/
│       ├── algoritmos.hpp
│       └── controlador.hpp
└── src/
    ├── algoritmos.cpp
    ├── controlador.cpp
    └── main.cpp
```

## Instalação e Build

Certifique-se de ter o ROS 2 instalado. Clone este repositório no seu workspace.

1. Na raiz do workspace, compile o pacote:

```bash
colcon build --packages-select solucionador_labirinto
source install/setup.bash
```

## Como Executar

São necessários dois terminais. Certifique-se de dar `source install/setup.bash` em ambos.

**Terminal 1 - Simulação:** Inicia o ambiente do jogo.

```bash
ros2 run cg maze
```

**Terminal 2 - Solucionador:**

O solucionador possui dois modos de operação, conforme os requisitos do desafio:

1. **Parte 1 - Navegação Direta (Padrão):**
   O robô recebe o mapa completo do servidor e calcula a rota otimizada imediatamente.
   ```bash
   ros2 run solucionador_labirinto rodar_solucao
   ```

2. **Parte 2 - Exploração (Fog of War):**
   O robô explora o labirinto desconhecido usando sensores e valida o mapa ao final.
   ```bash
   ros2 run solucionador_labirinto rodar_solucao --explorar
   ```


## Arquitetura do Código

### algoritmos.hpp/cpp
Contém as funções e classes principais:

- **processar_mapa()**: Converte o mapa achatado em matriz 2D e identifica posições do robô e alvo
- **bfs_encontrar_caminho()**: Implementa BFS para encontrar o caminho ótimo
- **Classe Explorador**: Gerencia a exploração do labirinto usando DFS com backtracking
  - `atualizar_mapa()`: Atualiza o mapa interno com leituras dos sensores
  - `proximo_passo()`: Decide a próxima direção a seguir na exploração

### controlador.hpp/cpp
Implementa a classe `ControladorRobo` que integra com ROS 2:

- Cliente do serviço `/move_command` para mover o robô
- Subscriber no tópico `/robot_sensors` para ler sensores
- `executar_exploracao()`: Loop principal que coordena exploração
- `validar_mapa()`: Valida que o mapa gerado permite calcular rotas ótimas

### main.cpp
Ponto de entrada da aplicação que inicializa o nó ROS 2.

## Lógica dos Algoritmos

### Parte 1 - BFS (Caminho Ótimo)
1. Recebe o mapa como entrada
2. Usa uma fila para explorar vizinhos nível por nível
3. Mantém registro de células visitadas
4. Retorna o menor caminho ao encontrar o alvo

### Parte 2 - Exploração com DFS
1. Inicializa mapa interno com células desconhecidas ('?')
2. A cada passo, atualiza o mapa com leituras dos sensores
3. Tenta avançar para células não visitadas (DFS)
4. Quando não há mais opções, volta para posição anterior (Backtracking)
5. Ao finalizar, valida o mapa rodando BFS sobre ele

## Sincronização ROS 2
O código implementa sincronização cuidadosa:
- Limpa leitura anterior de sensores antes de cada movimento
- Aguarda nova mensagem antes de tomar decisões
- Usa delays para garantir que o simulador processe os movimentos

## Compilação
O projeto usa `ament_cmake` como sistema de build, com dependências:
- `rclcpp`: Cliente C++ do ROS 2
- `cg_interfaces`: Mensagens e serviços customizados do simulador


## Vídeo de Demonstração

- [link para o vídeo](https://drive.google.com/drive/u/0/folders/1fYxEx8BB9Ypm_MF_szbvw_4IY0ICb0qW)