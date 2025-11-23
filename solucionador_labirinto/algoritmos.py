from collections import deque

def processar_mapa(lista_achatada, largura, altura):
    """
    Transforma a lista de letras em uma matriz e acha onde está o robô ('r') e o alvo ('t').
    """
    mapa = []
    inicio = None
    alvo = None
    
    for i in range(altura):
        # pega o pedaço da lista que corresponde a uma linha
        inicio_linha = i * largura
        fim_linha = (i + 1) * largura
        linha = lista_achatada[inicio_linha:fim_linha]
        mapa.append(linha)
        
        # procura robô e alvo nesta linha
        if 'r' in linha:
            inicio = (i, linha.index('r')) # (linha, coluna)
        if 't' in linha:
            alvo = (i, linha.index('t'))
            
    return mapa, inicio, alvo

def bfs_encontrar_caminho(mapa, inicio, alvo):
    """
    Usa busca em largura (BFS) para achar o menor caminho.
    Retorna uma lista de comandos: ['up', 'left', 'down', ...]
    """
    linhas = len(mapa)
    colunas = len(mapa[0])
    
    # fila armazena: (posicao_atual, caminho_percorrido)
    fila = deque([ (inicio, []) ])
    visitados = set([inicio])
    
    # dicionário: (mudança_linha, mudança_coluna) -> 'comando_texto'
    direcoes = {
        (-1, 0): 'up',    # cima diminui a linha
        (1, 0): 'down',   # baixo aumenta a linha
        (0, -1): 'left',  # esquerda diminui a coluna
        (0, 1): 'right'   # direita aumenta a coluna
    }

    while fila:
        (linha_atual, col_atual), caminho = fila.popleft()

        # chegou no alvo?
        if (linha_atual, col_atual) == alvo:
            return caminho

        # tenta andar para os vizinhos
        for (delta_lin, delta_col), comando in direcoes.items():
            nova_lin, nova_col = linha_atual + delta_lin, col_atual + delta_col

            # verifica se não saiu do mapa
            if 0 <= nova_lin < linhas and 0 <= nova_col < colunas:
                # verifica se não é parede ('b') e se não visitou ainda
                if mapa[nova_lin][nova_col] != 'b' and (nova_lin, nova_col) not in visitados:
                    visitados.add((nova_lin, nova_col))
                    novo_caminho = caminho + [comando]
                    fila.append(((nova_lin, nova_col), novo_caminho))
    
    return [] # retorna lista vazia se não achar nada


class Explorador:
    def __init__(self, largura=29, altura=29):
        self.largura = largura
        self.altura = altura
        # inicializa tudo como '?'
        self.mapa_conhecido = [['?' for _ in range(largura)] for _ in range(altura)]
        
        # posição inicial
        self.pos_atual = (1, 1) 
        self.mapa_conhecido[1][1] = 'f' 
        
        self.visitados = set()
        self.visitados.add(self.pos_atual)
        
        self.pilha = [] 

    def atualizar_mapa(self, sensor_msg):
        lin, col = self.pos_atual
        
        vizinhos = [
            ('up',    lin - 1, col),
            ('down',  lin + 1, col),
            ('left',  lin,     col - 1),
            ('right', lin,     col + 1)
        ]
        
        for direcao, l, c in vizinhos:
            # verifica limites antes de gravar no mapa
            if 0 <= l < self.altura and 0 <= c < self.largura:
                valor_sensor = getattr(sensor_msg, direcao)
                self.mapa_conhecido[l][c] = valor_sensor

    def proximo_passo(self):
        lin, col = self.pos_atual
        
        movimentos = [
            ((-1, 0), 'up'),
            ((0, 1),  'right'),
            ((1, 0),  'down'),
            ((0, -1), 'left')
        ]
        
        # 1. tenta avançar
        for (d_lin, d_col), direcao_txt in movimentos:
            viz_l, viz_c = lin + d_lin, col + d_col
            
            # verifica se o vizinho existe antes de acessar a lista
            dentro_do_mapa = (0 <= viz_l < self.altura) and (0 <= viz_c < self.largura)
            
            if dentro_do_mapa:
                # só entra aqui se a coordenada for válida
                eh_parede = (self.mapa_conhecido[viz_l][viz_c] == 'b')
                foi_visitado = ((viz_l, viz_c) in self.visitados)
                
                if not eh_parede and not foi_visitado:
                    # acha um caminho novo
                    self.pilha.append(self.pos_atual)
                    self.visitados.add((viz_l, viz_c))
                    self.pos_atual = (viz_l, viz_c)
                    return direcao_txt

        # 2. backtracking (voltar)
        if self.pilha:
            anterior_l, anterior_c = self.pilha.pop()
            
            delta_l = anterior_l - lin
            delta_c = anterior_c - col
            
            self.pos_atual = (anterior_l, anterior_c)
            
            if delta_l == -1: return 'up'
            if delta_l == 1:  return 'down'
            if delta_c == -1: return 'left'
            if delta_c == 1:  return 'right'
            
        return None
