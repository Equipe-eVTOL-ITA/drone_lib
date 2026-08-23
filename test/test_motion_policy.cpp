// Testes da politica de movimento.
//
// POR QUE ESTE ARQUIVO EXISTE
//
// O drone_lib e o pacote mais critico do workspace -- e a fronteira com o PX4,
// e todo comando que chega ao drone passa por ele -- e nao tinha um teste
// sequer. A razao era boa: a classe `Drone` e um rclcpp::Node que gira o
// proprio executor numa thread, e testar qualquer coisa que a toque exige
// subir ROS.
//
// A MotionPolicy contorna isso separando a DECISAO da PUBLICACAO. O
// `decidir()` e uma funcao de (pose, alvo, limites) para um setpoint, sem ROS
// e sem efeito colateral -- e e nele que mora a garantia que a camada existe
// para dar.
//
// A GARANTIA, dita de uma vez: com a politica axial, o drone nunca recebe um
// comando que o desloque de lado. Se algum dia a regra da competicao mudar
// para "so pode girar 90 graus e andar para frente", e este teste passar, a
// resposta e uma linha de YAML.

#include <cmath>
#include <memory>

#include <gtest/gtest.h>

#include "drone/motion_policy.hpp"

using drone::Comando;
using drone::Limites;
using drone::Pose;
using drone::criarPolitica;
using drone::normalizarAngulo;

namespace
{

/// Quanto o comando desloca o drone PARA O LADO, em metros.
///
/// Projeta o deslocamento comandado no eixo perpendicular a proa ATUAL. E essa
/// a definicao de "andar de lado": sair do lugar numa direcao que nao e aquela
/// para onde o drone aponta.
double deslocamentoLateral(const Pose & pose, const Comando & c)
{
  const double dx = c.posicao.x() - pose.posicao.x();
  const double dy = c.posicao.y() - pose.posicao.y();
  // Em FRD, o eixo lateral (direita) e a proa girada de +90 graus.
  return std::abs(-dx * std::sin(pose.yaw) + dy * std::cos(pose.yaw));
}

Pose emRepouso(double x, double y, double yaw)
{
  Pose p;
  p.posicao = Eigen::Vector3d(x, y, -2.0);
  p.yaw = yaw;
  return p;
}

}  // namespace

// --- o invariante -----------------------------------------------------------

TEST(Axial, NuncaComandaDeslocamentoLateral)
{
  Limites lim;
  lim.posicao = 0.10;
  lim.yaw = 0.05;

  // Varre destinos em todas as direcoes e proas iniciais em todas as direcoes.
  // O caso que mais importa e o pior: destino exatamente a 90 graus da proa --
  // que e onde uma politica holonomica produziria puro deslocamento lateral.
  int amostras = 0;
  for (int i = 0; i < 16; ++i) {
    const double proa = -M_PI + i * (2.0 * M_PI / 16.0);
    for (int j = 0; j < 16; ++j) {
      const double rumo = -M_PI + j * (2.0 * M_PI / 16.0);

      auto politica = criarPolitica("axial");
      ASSERT_NE(politica, nullptr);

      Pose pose = emRepouso(0.0, 0.0, proa);
      const Eigen::Vector3d alvo(5.0 * std::cos(rumo), 5.0 * std::sin(rumo), -2.0);
      politica->iniciar(alvo, std::numeric_limits<double>::quiet_NaN());

      // Simula a convergencia: a cada tick o drone gira um pouco na direcao
      // comandada, como o controlador do PX4 faria.
      bool chegou = false;
      for (int tick = 0; tick < 400; ++tick) {
        const Comando c = politica->decidir(
          pose, alvo, std::numeric_limits<double>::quiet_NaN(), lim);

        EXPECT_LT(deslocamentoLateral(pose, c), 1e-9)
          << "proa=" << proa << " rumo=" << rumo << " tick=" << tick;
        ++amostras;

        if (c.chegou) {chegou = true; break;}

        // O drone persegue o setpoint: gira ate 0,1 rad por tick e so anda
        // depois de apontado.
        const double erro_yaw = normalizarAngulo(c.yaw - pose.yaw);
        pose.yaw = normalizarAngulo(
          pose.yaw + std::copysign(std::min(0.1, std::abs(erro_yaw)), erro_yaw));

        const Eigen::Vector3d rumo_cmd = c.posicao - pose.posicao;
        if (rumo_cmd.head<2>().norm() > 1e-9) {
          pose.posicao += rumo_cmd.normalized() * std::min(0.2, rumo_cmd.norm());
        }
      }

      // A OUTRA METADE DO INVARIANTE.
      //
      // "Nunca anda de lado" e trivial de satisfazer: uma politica que nunca
      // sai do lugar passa. Sem esta assercao, o teste acima daria verde para
      // uma implementacao completamente quebrada -- e teria dado, porque a
      // versao anterior deste laco apenas parava no fim dos ticks sem
      // conferir nada.
      EXPECT_TRUE(chegou)
        << "nao convergiu: proa=" << proa << " rumo=" << rumo
        << " parou a " << (alvo - pose.posicao).head<2>().norm() << " m do alvo";
    }
  }
  EXPECT_GT(amostras, 1000) << "a varredura precisa exercitar de fato";
}

TEST(Axial, AndaParaFrenteAteChegar)
{
  // Convergencia num caso unico e legivel, para quando o teste varrido acima
  // falhar e alguem precisar de um ponto de partida menor.
  auto politica = criarPolitica("axial");
  Limites lim;
  lim.posicao = 0.10;
  lim.yaw = 0.05;

  Pose pose = emRepouso(0.0, 0.0, 0.0);            // proa para o norte
  const Eigen::Vector3d alvo(3.0, 4.0, -2.0);      // 5 m, a 53 graus
  politica->iniciar(alvo, std::numeric_limits<double>::quiet_NaN());

  double percorrido = 0.0;
  bool chegou = false;
  for (int tick = 0; tick < 400 && !chegou; ++tick) {
    const Comando c = politica->decidir(
      pose, alvo, std::numeric_limits<double>::quiet_NaN(), lim);
    chegou = c.chegou;

    const double erro_yaw = normalizarAngulo(c.yaw - pose.yaw);
    pose.yaw = normalizarAngulo(
      pose.yaw + std::copysign(std::min(0.1, std::abs(erro_yaw)), erro_yaw));

    const Eigen::Vector3d d = c.posicao - pose.posicao;
    if (d.head<2>().norm() > 1e-9) {
      const Eigen::Vector3d avanco = d.normalized() * std::min(0.2, d.norm());
      pose.posicao += avanco;
      percorrido += avanco.head<2>().norm();
    }
  }

  EXPECT_TRUE(chegou);
  EXPECT_NEAR((alvo - pose.posicao).head<2>().norm(), 0.0, lim.posicao);
  // O caminho e quase a reta: gira primeiro, depois vai direto. Uma politica
  // que fizesse voltas para chegar tambem "nunca andaria de lado".
  EXPECT_LT(percorrido, 5.0 * 1.15) << "o caminho deveria ser quase a reta de 5 m";
}

TEST(Axial, GiraParadoAntesDeAvancar)
{
  auto politica = criarPolitica("axial");
  Limites lim;

  // Proa para o norte (x), destino a leste (y): 90 graus de giro.
  Pose pose = emRepouso(0.0, 0.0, 0.0);
  const Eigen::Vector3d alvo(0.0, 5.0, -2.0);
  politica->iniciar(alvo, std::numeric_limits<double>::quiet_NaN());

  const Comando c = politica->decidir(
    pose, alvo, std::numeric_limits<double>::quiet_NaN(), lim);

  // A posicao comandada e a atual em x e y: o drone gira sem sair do lugar.
  EXPECT_NEAR(c.posicao.x(), pose.posicao.x(), 1e-9);
  EXPECT_NEAR(c.posicao.y(), pose.posicao.y(), 1e-9);
  // E a guinada comandada aponta para o destino.
  EXPECT_NEAR(c.yaw, M_PI_2, 1e-6);
  EXPECT_FALSE(c.chegou);
}

TEST(Axial, VoltaAGirarSeORumoSaiDaTolerancia)
{
  // E o que torna a garantia estrutural. O MovimentoAxial da fase4 nao fazia
  // isso: uma vez em "avancando", ele comandava o destino aconteca o que
  // acontecer -- e um destino recalculado para o lado virava deslocamento
  // lateral, justamente o que a classe existia para impedir.
  auto politica = criarPolitica("axial");
  Limites lim;
  lim.yaw = 0.05;

  Pose pose = emRepouso(0.0, 0.0, 0.0);
  Eigen::Vector3d alvo(5.0, 0.0, -2.0);
  politica->iniciar(alvo, std::numeric_limits<double>::quiet_NaN());

  // Ja alinhado: entra em avanco e comanda o destino.
  Comando c = politica->decidir(pose, alvo, std::numeric_limits<double>::quiet_NaN(), lim);
  EXPECT_NEAR(c.posicao.x(), 5.0, 1e-9);

  // O destino salta para o lado, sem o drone ter girado.
  alvo = Eigen::Vector3d(0.0, 5.0, -2.0);
  c = politica->decidir(pose, alvo, std::numeric_limits<double>::quiet_NaN(), lim);

  EXPECT_LT(deslocamentoLateral(pose, c), 1e-9) << "deveria girar, nao deslizar";
  EXPECT_NEAR(c.posicao.x(), pose.posicao.x(), 1e-9);
  EXPECT_NEAR(c.yaw, M_PI_2, 1e-6);
}

TEST(Axial, DeclaraQueNaoAceitaCorrecaoLateral)
{
  // Os estados de alinhamento fino perguntam isto antes de comandar uma
  // correcao de centimetros. Responder false e o que os avisa de que este
  // drone nao anda de lado -- em vez de comandar assim mesmo e descobrir no voo.
  EXPECT_FALSE(criarPolitica("axial")->permiteCorrecaoLateral());
  EXPECT_TRUE(criarPolitica("holonomica")->permiteCorrecaoLateral());
}

// --- a politica de sempre ---------------------------------------------------

TEST(Holonomica, ComandaUmPassoNaDirecaoDoDestino)
{
  // Reproduz o que o WaypointListState::navigate() fazia: setpoint a `passo`
  // metros a frente, na direcao do destino, com a guinada congelada.
  auto politica = criarPolitica("holonomica");
  Limites lim;
  lim.passo = 0.5;
  lim.posicao = 0.10;

  Pose pose = emRepouso(0.0, 0.0, 0.0);
  const Eigen::Vector3d alvo(10.0, 0.0, -2.0);

  const Comando c = politica->decidir(pose, alvo, 0.0, lim);
  EXPECT_NEAR(c.posicao.x(), 0.5, 1e-9) << "um passo, e nao o destino inteiro";
  EXPECT_FALSE(c.chegou);
}

TEST(Holonomica, AndaDeLadoQuandoODestinoEstaDeLado)
{
  // Este teste documenta a diferenca entre as duas politicas. Nao e um
  // defeito da holonomica: e o comportamento de sempre, e e por existir que a
  // axial precisa ser uma escolha.
  auto politica = criarPolitica("holonomica");
  Limites lim;
  lim.passo = 0.5;

  Pose pose = emRepouso(0.0, 0.0, 0.0);          // proa para o norte
  const Eigen::Vector3d alvo(0.0, 10.0, -2.0);   // destino a leste

  const Comando c = politica->decidir(pose, alvo, 0.0, lim);
  EXPECT_NEAR(deslocamentoLateral(pose, c), 0.5, 1e-9);
}

TEST(Holonomica, ChegouDentroDaTolerancia)
{
  auto politica = criarPolitica("holonomica");
  Limites lim;
  lim.posicao = 0.10;

  Pose pose = emRepouso(0.0, 0.0, 0.0);
  const Eigen::Vector3d alvo(0.05, 0.0, -2.0);

  EXPECT_TRUE(politica->decidir(pose, alvo, 0.0, lim).chegou);
}

TEST(Politicas, YawNaoPropagaNaN)
{
  // Os estados passam NaN em yaw_alvo para dizer "nao me importo". Repassar
  // isso ao setpoint faria o PX4 rejeitar a mensagem INTEIRA em silencio: o
  // drone simplesmente para de receber comando, sem erro em lugar nenhum.
  Limites lim;
  const Eigen::Vector3d alvo(5.0, 0.0, -2.0);
  const Pose pose = emRepouso(0.0, 0.0, 0.3);
  const double nan = std::numeric_limits<double>::quiet_NaN();

  for (const auto & nome : {"holonomica", "axial"}) {
    auto p = criarPolitica(nome);
    ASSERT_NE(p, nullptr) << nome;
    p->iniciar(alvo, nan);
    const Comando c = p->decidir(pose, alvo, nan, lim);
    EXPECT_FALSE(std::isnan(c.yaw)) << nome;
    EXPECT_FALSE(std::isnan(c.posicao.x())) << nome;
    EXPECT_FALSE(std::isnan(c.posicao.y())) << nome;
    EXPECT_FALSE(std::isnan(c.posicao.z())) << nome;
  }
}

// --- o registro -------------------------------------------------------------

TEST(Politicas, NomeDesconhecidoDevolveNullptr)
{
  // Cair no padrao com um nome errado seria o pior desfecho: o drone voaria de
  // um jeito que ninguem pediu, e o YAML continuaria dizendo outra coisa.
  EXPECT_EQ(criarPolitica("axil"), nullptr);
  EXPECT_EQ(criarPolitica(""), nullptr);
}

TEST(Politicas, OPadraoEAHolonomica)
{
  // Uma missao que nao declara `motion_policy` tem de voar exatamente como
  // voava antes desta camada existir.
  auto p = criarPolitica(drone::kPoliticaPadrao);
  ASSERT_NE(p, nullptr);
  EXPECT_STREQ(p->nome(), "holonomica");
}

TEST(NormalizarAngulo, LevaParaOIntervaloMenosPiAPi)
{
  // Um erro de guinada de 179 para -179 graus e de 2 graus, e nao de 358. Sem
  // normalizar, a fase de girar nunca converge e o drone fica rodando.
  EXPECT_NEAR(normalizarAngulo(3.0 * M_PI), M_PI, 1e-9);
  EXPECT_NEAR(normalizarAngulo(-3.0 * M_PI), M_PI, 1e-9);
  EXPECT_NEAR(normalizarAngulo(0.5), 0.5, 1e-9);

  const double quase_meia_volta = normalizarAngulo(
    (179.0 * M_PI / 180.0) - (-179.0 * M_PI / 180.0));
  EXPECT_NEAR(std::abs(quase_meia_volta), 2.0 * M_PI / 180.0, 1e-9);
}
