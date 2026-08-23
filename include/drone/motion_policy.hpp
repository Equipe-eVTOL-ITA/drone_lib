#pragma once

// MotionPolicy — COMO o drone vai de um ponto a outro, escolhido por YAML.
// Antes disso o deslocamento nascia em ~20 lugares, cada um do seu jeito.

#include <memory>
#include <string>

#include <Eigen/Eigen>

#include "drone/Drone.hpp"

// >>> CONTRATO movimento.politica
// COMO o drone vai de um ponto a outro e uma chave de YAML, e nao codigo:
//
//     motion_policy: holonomica   # padrao -- linha reta, inclusive de lado
//     motion_policy: axial        # gira parado, so entao avanca; nunca de lado
//
// Quem obedece: WaypointListState, GoToState e ReturnHomeState (stdstates), e
// portanto toda missao que os usa. A fase4 ja voava axial por conta propria.
//
// Quem NAO obedece, de proposito: PrecisionAlignState e o CentralizarNoComodo
// da fase4. Sao correcoes de centimetros em malha fechada com a camera, e
// girar antes de cada uma destruiria o alinhamento em vez de proteje-lo. Eles
// perguntam permiteCorrecaoLateral() antes de comandar.
//
// A lista completa de onde nasce um comando de movimento esta em
// docs/CONTRATOS.md, secao "Onde se comanda movimento" -- gerada do codigo.
// <<< CONTRATO

namespace drone
{

/// Os limites de um deslocamento.
struct Limites
{
  /// Distancia a que o setpoint e posto a frente, em metros. E o que regula a
  /// velocidade efetiva. A politica axial o ignora: comanda o destino inteiro.
  double passo = 0.5;
  /// Tolerancia de chegada, em metros.
  double posicao = 0.10;
  /// Erro de guinada aceito antes de avancar, em rad. So a axial o usa.
  double yaw = 0.05;
};

/// Normaliza para (-pi, pi]. Sem isto a fase de girar nunca converge.
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

  /// O QUE comandar, dado onde o drone esta. Sem ROS e sem efeito colateral,
  /// para que o invariante da axial possa ser testado sem subir um Node.
  virtual Comando decidir(
    const Pose & pose,
    const Eigen::Vector3d & alvo,
    double yaw_alvo,
    const Limites & lim) = 0;

  /// Um tick da FSM (20 Hz): le o drone, decide e publica. true = chegou.
  /// `alvo` vem a cada tick porque metade dos estados o recalcula.
  bool irPara(
    const std::shared_ptr<Drone> & drone,
    const Eigen::Vector3d & alvo,
    double yaw_alvo,
    const Limites & lim);

  /// Congela o drone onde ele esta.
  void parar(const std::shared_ptr<Drone> & drone);

  /// false = "este drone nao anda de lado". Os estados de alinhamento fino
  /// perguntam antes de comandar uma correcao lateral.
  virtual bool permiteCorrecaoLateral() const {return true;}

  virtual const char * nome() const = 0;
};

/// Padrao quando a missao nao declara `motion_policy`: o comportamento de
/// antes desta camada, para que quem nao mexer no YAML voe igual.
inline constexpr const char * kPoliticaPadrao = "holonomica";

/// Cria uma politica pelo nome. nullptr = nome desconhecido.
std::unique_ptr<MotionPolicy> criarPolitica(const std::string & nome);

/// Os nomes aceitos, para a mensagem de erro.
std::string politicasDisponiveis();

}  // namespace drone
