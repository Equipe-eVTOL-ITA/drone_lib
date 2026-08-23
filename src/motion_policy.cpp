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

// ---------------------------------------------------------------------------
// A ponte entre a decisao (pura) e o drone (ROS).
//
// As duas funcoes abaixo sao as UNICAS de todo este arquivo que falam com o
// `Drone`. Tudo o que decide para onde ir esta em `decidir()`, que nao conhece
// ROS e por isso pode ser testado.
// ---------------------------------------------------------------------------
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

// ---------------------------------------------------------------------------
// Holonomica — o comportamento de sempre.
//
// Isto e o miolo do `WaypointListState::navigate()`, movido para ca SEM
// MUDANCA. Extrair em vez de reescrever e o que garante que uma missao sem
// `motion_policy` no YAML voe exatamente como voava: a comparacao nao depende
// de eu ter lido a intencao certa, so de o codigo ser o mesmo.
//
// O drone vai em linha reta ate o destino, com a guinada que lhe mandarem --
// inclusive de lado, se o destino estiver de lado.
// ---------------------------------------------------------------------------
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
    // Um yaw_alvo NaN quer dizer "mantenha o que esta". Passa-lo adiante
    // colocaria NaN no setpoint, e o PX4 rejeita a mensagem inteira em
    // silencio -- o drone simplesmente para de receber comando.
    c.yaw = std::isnan(yaw_alvo) ? pose.yaw : yaw_alvo;

    const Eigen::Vector3d diff = alvo - pose.posicao;

    if (diff.norm() < lim.posicao) {
      c.posicao = alvo;
      c.chegou = true;
      return c;
    }

    // O setpoint vai `passo` metros a frente, na direcao do destino -- e nao
    // no destino. Quem voa e o controlador de posicao do PX4, e a distancia do
    // setpoint e o que regula a velocidade que ele escolhe.
    const Eigen::Vector3d passo =
      diff.norm() > lim.passo ? diff.normalized() * lim.passo : diff;
    c.posicao = pose.posicao + passo;
    return c;
  }
};

// ---------------------------------------------------------------------------
// Axial — gira parado, so entao avanca. Nunca anda de lado.
//
// Promovido do `MovimentoAxial` da fase4, que voa assim desde sempre porque no
// labirinto nao ha escolha: girar e transladar ao mesmo tempo varre uma pegada
// maior que a do drone parado, e num corredor de 0,95 m com um drone de 0,40 m
// a diferenca entre varrer e nao varrer e a diferenca entre passar e bater.
//
// Como o comando e de POSICAO e nao de velocidade, a fase de girar comanda a
// POSICAO ATUAL com a guinada nova -- o drone gira parado -- e so entao a fase
// de avancar muda a posicao.
//
// A GARANTIA, e como ela se sustenta
//
// Na fase de girar, a politica comanda a posicao em que o drone JA ESTA: e
// impossivel produzir deslocamento. Na fase de avancar ela comanda o destino
// com o rumo ate ele -- e so chegou nessa fase depois de ja estar apontada
// para la.
//
// O que a torna estrutural, e nao convencao, e a reentrada: se durante o
// avanco o rumo sair da tolerancia -- porque o destino se moveu, ou porque o
// drone derivou --, ela VOLTA a girar em vez de corrigir de lado. O
// MovimentoAxial da fase4 nao fazia isso; ele dependia de quem o chamava
// escolher um destino alinhado com a guinada, e um chamador distraido produzia
// exatamente o deslocamento lateral que a classe existia para impedir.
// ---------------------------------------------------------------------------
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
      // Chegou: pode assumir a guinada final que o estado pediu, girando
      // parado sobre o destino.
      c.posicao = alvo;
      c.yaw = std::isnan(yaw_alvo) ? pose.yaw : yaw_alvo;
      c.chegou = true;
      return c;
    }

    // O rumo que aponta para o destino. Em FRD, x e para a frente e y para a
    // direita, e a guinada cresce de x para y -- entao e atan2(dy, dx).
    const double rumo = std::atan2(
      alvo.y() - pose.posicao.y(), alvo.x() - pose.posicao.x());

    const double erro_de_rumo = std::abs(normalizarAngulo(rumo - pose.yaw));

    // Reentrada: fora da tolerancia, volta (ou fica) girando. Vale tanto para
    // o primeiro tick quanto para uma deriva no meio do avanco.
    if (fase_ != Fase::Avancando || erro_de_rumo > lim.yaw) {
      if (erro_de_rumo < lim.yaw) {
        fase_ = Fase::Avancando;
      } else {
        fase_ = Fase::Girando;
        // GIRA PARADO: comanda a posicao ATUAL. Este `c.posicao = pose.posicao`
        // e a garantia inteira -- enquanto o rumo nao converge, nao existe
        // setpoint que desloque o drone.
        c.posicao = pose.posicao;
        c.posicao.z() = alvo.z();   // subir/descer nao e andar de lado
        c.yaw = rumo;
        return c;
      }
    }

    // Alinhado: avanca SOBRE A PROA, e nao sobre a reta ate o destino.
    //
    // POR QUE PROJETAR, E NAO COMANDAR O DESTINO
    //
    // Comandar o destino cru parece equivalente -- afinal o rumo ja esta dentro
    // da tolerancia. Nao e, e a diferenca foi medida pelo teste: com tolerancia
    // de 0,05 rad e um destino a 5 m, a componente lateral do deslocamento
    // comandado e 5 · sen(0,05) = 0,25 m. O drone anda um quarto de metro para
    // o lado, dentro das regras, porque "alinhado o bastante" vezes "longe o
    // bastante" da um desvio real.
    //
    // Projetando o destino sobre a reta da proa, a componente lateral e ZERO
    // por construcao -- nao aproximadamente zero. A garantia deixa de depender
    // do valor da tolerancia.
    //
    // O que sobra de erro lateral vira erro de RUMO conforme o drone se
    // aproxima (a mesma distancia lateral subtende um angulo maior de perto), e
    // ai a reentrada acima o manda girar. E assim que ele converge: avanca
    // reto, para, corrige a proa, avanca reto de novo.
    const double proa_x = std::cos(pose.yaw);
    const double proa_y = std::sin(pose.yaw);
    const double avanco =
      (alvo.x() - pose.posicao.x()) * proa_x + (alvo.y() - pose.posicao.y()) * proa_y;

    c.posicao.x() = pose.posicao.x() + proa_x * avanco;
    c.posicao.y() = pose.posicao.y() + proa_y * avanco;
    c.posicao.z() = alvo.z();
    // A guinada comandada e a ATUAL: durante o avanco o drone nao gira. Girar
    // enquanto translada e exatamente o que esta politica existe para impedir.
    c.yaw = pose.yaw;
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
