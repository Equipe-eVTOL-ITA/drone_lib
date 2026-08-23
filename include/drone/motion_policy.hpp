#pragma once

// MotionPolicy — COMO o drone vai de um ponto a outro.
//
// POR QUE ISTO EXISTE
//
// A pergunta que motivou este arquivo: "mudaram a regra e o drone so pode
// girar 90 graus e andar para frente -- onde eu mexo?"
//
// Ate aqui a resposta era: em uns vinte lugares. O deslocamento nascia
// espalhado por `WaypointListState::navigate`, `GoToState::act`,
// `ReturnHomeState::step`, `PrecisionAlignState::act`, os quatro nos de
// navegacao da fase4, os helpers do movement.cpp -- cada um comandando
// posicao ou velocidade em linha reta, do seu jeito. Nao havia UM lugar.
//
// A fase4 ja tinha resolvido isso para si: todo deslocamento dela passa pelo
// `MovimentoAxial`, que separa GIRAR de AVANCAR em duas fases. E este arquivo
// e aquela ideia promovida a politica do workspace inteiro, escolhida por uma
// chave de YAML.
//
//     motion_policy: holonomica   # o de sempre: vai em linha reta
//     motion_policy: axial        # gira parado, so entao avanca
//
// UMA LINHA. E o cenario do dia da competicao deixa de ser um problema.
//
// POR QUE A AXIAL NAO E SO PRECIOSISMO
//
// Girar e transladar ao mesmo tempo varre uma pegada maior do que a do drone
// parado. Num corredor de 0,95 m com um drone de 0,40 m, a diferenca entre
// varrer e nao varrer e a diferenca entre passar e bater. Foi medido na fase4,
// e esta escrito no cabecalho do contexto.hpp de la.
//
// O QUE ESTA CAMADA NAO COBRE, DE PROPOSITO
//
// `PrecisionAlignState` e o `CentralizarNoComodo` da fase4 comandam correcoes
// de centimetros em malha fechada com a camera. Girar antes de cada correcao
// destruiria o alinhamento em vez de protege-lo -- a correcao E lateral, e tem
// de ser. Eles ficam fora, e isso e uma excecao DECLARADA, nao um esquecimento:
// ver `permiteCorrecaoLateral()`.

#include <memory>
#include <string>

#include <Eigen/Eigen>

#include "drone/Drone.hpp"

namespace drone
{

/// Os limites de um deslocamento, juntos para nao virarem cinco argumentos.
struct Limites
{
  /// Deslocamento maximo comandado por tick, em metros.
  ///
  /// A politica holonomica poe o setpoint a esta distancia a frente, na
  /// direcao do destino -- e o que regula a velocidade efetiva, ja que o
  /// comando e de posicao e quem voa e o controlador do PX4. A politica axial
  /// o IGNORA: ela comanda o destino inteiro, como a fase4 faz e tem voado.
  double passo = 0.5;

  /// Distancia horizontal para considerar o ponto atingido, em metros.
  double posicao = 0.10;

  /// Erro de guinada aceito antes de comecar a avancar, em radianos.
  /// So a politica axial o usa.
  double yaw = 0.05;
};

/// Normaliza um angulo para (-pi, pi].
///
/// Existe aqui, e nao so no movement.cpp, porque a politica precisa dele e o
/// `normalizeYawError` de la trabalha em float. Um erro de guinada de 179 para
/// -179 graus e de 2 graus, e nao de 358 -- sem normalizar, a fase de girar
/// nunca converge e o drone fica rodando.
double normalizarAngulo(double a);

/// O estado do drone que uma politica precisa saber. Nada de ROS.
struct Pose
{
  Eigen::Vector3d posicao{0.0, 0.0, 0.0};   ///< mundo FRD
  double yaw = 0.0;                          ///< rad, no referencial da missao
};

/// O setpoint que a politica decidiu, antes de virar mensagem.
struct Comando
{
  Eigen::Vector3d posicao{0.0, 0.0, 0.0};
  double yaw = 0.0;
  bool chegou = false;
};

class MotionPolicy
{
public:
  virtual ~MotionPolicy() = default;

  /// Comeca um deslocamento novo. Chamada no on_enter do estado.
  virtual void iniciar(const Eigen::Vector3d & alvo, double yaw_alvo) = 0;

  /// O QUE comandar, dado onde o drone esta. Sem ROS, sem efeito colateral.
  ///
  /// POR QUE A DECISAO E SEPARADA DA PUBLICACAO
  ///
  /// A garantia que esta camada existe para dar -- "a politica axial nunca
  /// comanda deslocamento lateral" -- so vale se puder ser TESTADA. Com a
  /// decisao embutida num metodo que fala com o `Drone`, testa-la exigiria
  /// subir um rclcpp::Node que gira o proprio executor numa thread, e o
  /// drone_lib nao tinha nenhum teste justamente por isso.
  ///
  /// Aqui a decisao e uma funcao de (pose, alvo, limites) para um setpoint. O
  /// teste passa uma pose, le o comando e confere o invariante. Ver
  /// test/test_motion_policy.cpp.
  virtual Comando decidir(
    const Pose & pose,
    const Eigen::Vector3d & alvo,
    double yaw_alvo,
    const Limites & lim) = 0;

  /// Um tick da FSM (50 ms, 20 Hz). Le o drone, decide e publica.
  /// Devolve true quando chegou.
  ///
  /// `alvo` vem a cada tick, e nao so no iniciar(), porque metade dos estados
  /// recalcula o destino continuamente -- uma espiral de busca, um alvo visto
  /// pela camera. Uma politica que so soubesse do destino no inicio nao
  /// serviria para eles.
  bool irPara(
    const std::shared_ptr<Drone> & drone,
    const Eigen::Vector3d & alvo,
    double yaw_alvo,
    const Limites & lim);

  /// Congela o drone onde ele esta.
  void parar(const std::shared_ptr<Drone> & drone);

  /// Esta politica aceita deslocamento lateral sem girar antes?
  ///
  /// Os estados de alinhamento fino perguntam isto antes de comandar uma
  /// correcao lateral. Uma politica que responde false esta dizendo "este
  /// drone nao anda de lado", e o estado tem de resolver de outro jeito --
  /// em vez de comandar assim mesmo e so descobrir no voo.
  virtual bool permiteCorrecaoLateral() const {return true;}

  virtual const char * nome() const = 0;
};

/// O nome usado quando a missao nao declara `motion_policy`.
///
/// E a holonomica porque e o que todo estado fazia antes desta camada: uma
/// missao que nao muda o YAML tem de voar exatamente como voava.
inline constexpr const char * kPoliticaPadrao = "holonomica";

/// Cria uma politica pelo nome. nullptr = nome desconhecido.
std::unique_ptr<MotionPolicy> criarPolitica(const std::string & nome);

/// Os nomes aceitos, para a mensagem de erro.
std::string politicasDisponiveis();

}  // namespace drone
