#include "drone/motion_policy.hpp"

#include <cmath>
#include <memory>
#include <string>

namespace drone
{

double normalizarAngulo(double a)
{
  while (a > M_PI) {a -= 2.0 * M_PI;}
  while (a <= -M_PI) {a += 2.0 * M_PI;}
  return a;
}

// As duas funcoes abaixo sao as unicas deste arquivo que falam com o Drone;
// tudo o que decide para onde ir esta em decidir(), que nao conhece ROS.

bool MotionPolicy::irPara(
  const std::shared_ptr<Drone> & drone,
  const Eigen::Vector3d & alvo,
  double yaw_alvo,
  const Limites & lim)
{
  if (drone == nullptr) {return false;}

  Pose pose;
  pose.posicao = drone->getLocalPosition();
  pose.yaw = drone->getOrientation()[2];

  const Comando c = this->decidir(pose, alvo, yaw_alvo, lim);

  drone->setLocalPosition(
    static_cast<float>(c.posicao.x()),
    static_cast<float>(c.posicao.y()),
    static_cast<float>(c.posicao.z()),
    static_cast<float>(c.yaw));

  return c.chegou;
}

void MotionPolicy::parar(const std::shared_ptr<Drone> & drone)
{
  if (drone == nullptr) {return;}
  const Eigen::Vector3d p = drone->getLocalPosition();
  drone->setLocalPosition(
    static_cast<float>(p.x()), static_cast<float>(p.y()), static_cast<float>(p.z()),
    static_cast<float>(drone->getOrientation()[2]));
}

namespace
{

// ── Holonomica ──────────────────────────────────────────────────────────────
// Linha reta ate o destino, com a guinada que mandarem -- inclusive de lado.
// Miolo do WaypointListState::navigate(), movido para ca sem mudanca.

class Holonomica : public MotionPolicy
{
public:
  const char * nome() const override {return "holonomica";}

  void iniciar(const Eigen::Vector3d &, double) override {}

  Comando decidir(
    const Pose & pose,
    const Eigen::Vector3d & alvo,
    double yaw_alvo,
    const Limites & lim) override
  {
    Comando c;
    // NaN quer dizer "mantenha o yaw atual". Repassado, o PX4 rejeitaria a
    // mensagem inteira em silencio.
    c.yaw = std::isnan(yaw_alvo) ? pose.yaw : yaw_alvo;

    const Eigen::Vector3d diff = alvo - pose.posicao;

    if (diff.norm() < lim.posicao) {
      c.posicao = alvo;
      c.chegou = true;
      return c;
    }

    // Setpoint `passo` metros a frente, e nao no destino: e a distancia do
    // setpoint que regula a velocidade escolhida pelo PX4.
    const Eigen::Vector3d passo =
      diff.norm() > lim.passo ? diff.normalized() * lim.passo : diff;
    c.posicao = pose.posicao + passo;
    return c;
  }
};

// ── Axial ───────────────────────────────────────────────────────────────────
// Gira parado, so entao avanca. Nunca anda de lado.
//
// Promovido do MovimentoAxial da fase4: girar e transladar ao mesmo tempo
// varre uma pegada maior que a do drone parado, e num corredor de 0,95 m com
// um drone de 0,40 m isso e a diferenca entre passar e bater.

class Axial : public MotionPolicy
{
public:
  const char * nome() const override {return "axial";}

  bool permiteCorrecaoLateral() const override {return false;}

  void iniciar(const Eigen::Vector3d &, double) override
  {
    fase_ = Fase::Girando;
  }

  Comando decidir(
    const Pose & pose,
    const Eigen::Vector3d & alvo,
    double yaw_alvo,
    const Limites & lim) override
  {
    Comando c;
    const double resta = std::hypot(
      alvo.x() - pose.posicao.x(), alvo.y() - pose.posicao.y());

    if (resta < lim.posicao) {
      fase_ = Fase::Pronto;
      c.posicao = alvo;
      c.yaw = std::isnan(yaw_alvo) ? pose.yaw : yaw_alvo;
      c.chegou = true;
      return c;
    }

    // Em FRD a guinada cresce de x para y, entao o rumo e atan2(dy, dx).
    const double rumo = std::atan2(
      alvo.y() - pose.posicao.y(), alvo.x() - pose.posicao.x());
    const double erro_de_rumo = std::abs(normalizarAngulo(rumo - pose.yaw));

    // Reentrada: fora da tolerancia, volta a girar em vez de corrigir de lado.
    // E o que torna a garantia estrutural -- o MovimentoAxial nao a tinha, e
    // dependia de quem o chamava escolher um destino ja alinhado.
    if (fase_ != Fase::Avancando || erro_de_rumo > lim.yaw) {
      if (erro_de_rumo < lim.yaw) {
        fase_ = Fase::Avancando;
      } else {
        fase_ = Fase::Girando;
        // Comandar a posicao ATUAL e a garantia inteira: enquanto o rumo nao
        // converge nao existe setpoint que desloque o drone.
        c.posicao = pose.posicao;
        c.posicao.z() = alvo.z();   // subir/descer nao e andar de lado
        c.yaw = rumo;
        return c;
      }
    }

    // Avanca SOBRE A PROA, projetando o destino nela.
    //
    // Comandar o destino cru nao e equivalente: com tolerancia de 0,05 rad e
    // destino a 5 m, a componente lateral e 5·sen(0,05) = 0,25 m -- medido
    // pelo teste. Projetando, ela e zero por construcao, e a garantia deixa de
    // depender do valor da tolerancia. O erro lateral residual vira erro de
    // rumo conforme o drone se aproxima, e a reentrada acima o manda girar.
    const double proa_x = std::cos(pose.yaw);
    const double proa_y = std::sin(pose.yaw);
    const double avanco =
      (alvo.x() - pose.posicao.x()) * proa_x + (alvo.y() - pose.posicao.y()) * proa_y;

    c.posicao.x() = pose.posicao.x() + proa_x * avanco;
    c.posicao.y() = pose.posicao.y() + proa_y * avanco;
    c.posicao.z() = alvo.z();
    c.yaw = pose.yaw;   // durante o avanco o drone nao gira
    return c;
  }

private:
  enum class Fase { Girando, Avancando, Pronto };
  Fase fase_ = Fase::Girando;
};

}  // namespace

std::unique_ptr<MotionPolicy> criarPolitica(const std::string & nome)
{
  if (nome == "holonomica" || nome == "holonomic") {
    return std::make_unique<Holonomica>();
  }
  if (nome == "axial") {
    return std::make_unique<Axial>();
  }
  return nullptr;
}

std::string politicasDisponiveis()
{
  return "holonomica, axial";
}

}  // namespace drone
