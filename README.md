# Solucionador de Labirinto - Culling Games (ROS 2)

## Descrição
Este projeto é uma solução para o desafio de navegação autônoma utilizando ROS 2. O objetivo é controlar um robô em um ambiente simulado (grid), fazendo-o encontrar o caminho otimizado entre um ponto de partida e um alvo.

O algoritmo implementado utiliza **BFS (Busca em Largura)** para garantir o menor caminho possível em um grafo não ponderado.

## Tecnologias Utilizadas
* **ROS 2 (Humble)**
* **Python 3**
* **Algoritmo:** Breadth-First Search (BFS)
* **Simulador:** Culling Games (Mapas 2D)

## Instalação e Build

Certifique-se de ter o ROS 2 instalado. Clone este repositório no seu workspace (ex: `ponderada_ros/src`).

1. Na raiz do workspace, instale as dependências e compile:

```bash
colcon build
source install/setup.bash
```

### Como Executar
São necessários dois terminais. Certifique-se de dar source install/setup.bash em ambos.

Terminal 1 - Simulação: Inicia o ambiente do jogo.

```bash
ros2 run cg maze
```
Terminal 2 - Controlador (Solução): Inicia o nó que calcula a rota e move o robô.

```bash
ros2 run solucionador_labirinto rodar_solucao
```

## Lógica do Algoritmo
1. Obtenção do Mapa: O nó consome o serviço /get_map para receber o grid achatado.

2. Processamento: Transforma o vetor em uma matriz 2D e identifica as coordenadas (x, y) do Robô e do Alvo.

3. Pathfinding (BFS):

    - Utiliza uma fila (Queue) para explorar os vizinhos (Cima, Baixo, Esquerda, Direita).

    - Mantém um registro de células visitadas para evitar loops.

    - Ao encontrar o alvo, reconstrói o caminho de volta.

4. Atuação: Envia comandos sequenciais para o serviço /move_command.