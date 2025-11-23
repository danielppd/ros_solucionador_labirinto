import rclpy
import time
from rclpy.node import Node
from cg_interfaces.srv import MoveCmd
from cg_interfaces.msg import RobotSensors

# importa a nova classe Explorador
from .algoritmos import Explorador, bfs_encontrar_caminho

class ControladorRobo(Node):
    def __init__(self):
        super().__init__('controlador_robo')
        
        self.cliente_movimento = self.create_client(MoveCmd, '/move_command')
        
        # assinatura (Subscribe) nos sensores para a Parte 2
        self.subscription = self.create_subscription(
            RobotSensors,
            '/culling_games/robot_sensors',
            self.callback_sensores,
            10
        )
        self.ultimo_sensor = None # guarda a última leitura
        self.explorador = Explorador() # instancia nossa IA

    def callback_sensores(self, msg):
        """chamado automaticamente sempre que o robô manda dados dos sensores"""
        self.ultimo_sensor = msg

    def mover_robo(self, direcao):
        req = MoveCmd.Request()
        req.direction = direcao
        futuro = self.cliente_movimento.call_async(req)
        rclpy.spin_until_future_complete(self, futuro)

    def executar_exploracao(self):
        self.get_logger().info("Iniciando Exploração Sincronizada (Parte 2)...")
        
        # espera inicial para estabilizar
        time.sleep(1.0)
        
        while rclpy.ok():
            # 1. garante leitura nova:
            # limpa a leitura anterior para forçar o recebimento de uma nova
            self.ultimo_sensor = None 
            
            # fica preso aqui até chegar uma mensagem nova do sensor
            while self.ultimo_sensor is None:
                rclpy.spin_once(self, timeout_sec=0.1)
            
            # 2. atualiza mapa (agora temos certeza que o sensor é atual)
            self.explorador.atualizar_mapa(self.ultimo_sensor)
            
            # checagem de vitória
            sensor = self.ultimo_sensor
            if 't' in [sensor.up, sensor.down, sensor.left, sensor.right]:
                self.get_logger().info("!!! ALVO AVISTADO (VIZINHO) !!!")
                # opcional: mover para o alvo para finalizar bonito
                if sensor.up == 't': self.mover_robo('up')
                elif sensor.down == 't': self.mover_robo('down')
                elif sensor.left == 't': self.mover_robo('left')
                elif sensor.right == 't': self.mover_robo('right')
                break
            
            # 3. decide
            proxima_direcao = self.explorador.proximo_passo()
            
            # 4. age e espera
            if proxima_direcao:
                self.get_logger().info(f"Movendo: {proxima_direcao}")
                self.mover_robo(proxima_direcao)
                
                # pausa longa para garantir que o simulador mova o robô fisicamente
                # antes de tentarmos ler o sensor de novo na próxima volta
                time.sleep(0.8) 
            else:
                self.get_logger().info("Exploração finalizada!")
                break
        
        self.validar_mapa()

    def validar_mapa(self):
        """parte final: prova que o mapa gerado serve para navegar"""
        self.get_logger().info("--- VALIDANDO O MAPA GERADO ---")
        
        # pega o mapa que a gente acabou de descobrir
        mapa_final = self.explorador.mapa_conhecido
        
        # encontra onde está o alvo no mapa descoberto
        alvo = None
        for r in range(len(mapa_final)):
            for c in range(len(mapa_final[0])):
                if mapa_final[r][c] == 't':
                    alvo = (r, c)
                    break
        
        if alvo:
            self.get_logger().info(f"Alvo encontrado no mapa mental em: {alvo}")
            # roda o BFS da Parte 1 usando APENAS o mapa que descobrimos
            rota = bfs_encontrar_caminho(mapa_final, (1,1), alvo)
            self.get_logger().info(f"Prova de conceito: Rota otimizada calculada com sucesso: {len(rota)} passos.")
        else:
            self.get_logger().error("Exploramos tudo e não achamos o alvo! (Ou ele não foi marcado)")

def main(args=None):
    rclpy.init(args=args)
    robo = ControladorRobo()
    
    # atenção: agora chamamos a exploração, não a navegação direta
    try:
        robo.executar_exploracao()
    except KeyboardInterrupt:
        pass
    
    robo.destroy_node()
    rclpy.shutdown()
