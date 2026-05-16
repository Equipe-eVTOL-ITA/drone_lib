#pragma once

#include <Eigen/Eigen>
#include <limits>
#include <memory>
#include "drone/Drone.hpp"

float normalizeYawError(float yaw_error);

// Envia velocity setpoint em FRD, rotacionado pelo yaw atual
void move_local_by_speed(std::shared_ptr<Drone> drone, Eigen::Vector3d direction, float speed);
void move_local_by_speed(std::shared_ptr<Drone> drone, float vx, float vy, float vz);

// Converte velocidade FRD em position setpoint 1 tick (50ms) à frente
void move_local_by_vel_as_position(std::shared_ptr<Drone> drone, float vx, float vy, float vz = 0.0f);

// ── Funções de waypoint ───────────────────────────────────────────────────────

/*
 * Passo constante em direção ao waypoint — sem perfil de velocidade.
 *
 * Semântica idêntica ao antigo move_local_by_waypoint: posiciona o setpoint
 * `step_m` metros à frente na direção do destino a cada tick do FSM.
 * A velocidade efetiva depende do position controller do PX4.
 *
 * Use para: buscas em espiral/arco, seguimento contínuo de trajetória,
 *           alvos recalculados a cada tick (SearchBall, SearchAruco, GoToBall).
 *
 * step_m = 0  →  segura a posição atual (hold).
 * Retorna true quando dentro da tolerância (posição e yaw).
 */
bool move_local_constant_step(
    std::shared_ptr<Drone> drone,
    Eigen::Vector3d        waypoint,
    float                  step_m,
    float                  tolerance  = 0.10f,
    float                  target_yaw = std::numeric_limits<float>::quiet_NaN());

/*
 * Perfil de velocidade trapezoidal para navegação ponto-a-ponto (stateful).
 *
 * Gera um perfil suave de aceleração/cruceiro/desaceleração:
 *
 *   v(m/s)
 *    ↑
 *    │        ___________
 *    │       /           \
 *    │      /  (cruise)   \
 *    │_____/               \____
 *    └──────────────────────────→ t
 *         acc           dec
 *
 * A desaceleração é calculada online pela restrição cinemática:
 *   v_brake = sqrt(2 · a_max · dist_restante)
 * garantindo que o drone sempre pare dentro da tolerância sem ultrapassar o goal.
 *
 * Deve ser instanciado como membro do estado FSM:
 *   - Chame reset() em on_enter() para partir do repouso.
 *   - Chame update() uma vez por tick em act().
 *
 * Use para: GoToBase, GoTo, DescendForShape, navegação ponto-a-ponto.
 */
class TrapezoidalMover {
public:
    TrapezoidalMover() : v_curr_(0.0f) {}

    // Reinicia a partir do repouso — chamar em on_enter()
    void reset() { v_curr_ = 0.0f; }

    // Executa um tick do perfil. Retorna true quando o waypoint foi atingido.
    bool update(std::shared_ptr<Drone> drone,
                Eigen::Vector3d        waypoint,
                float                  v_max,          // m/s
                float                  a_max,          // m/s²
                float                  tolerance  = 0.10f,
                float                  target_yaw = std::numeric_limits<float>::quiet_NaN());

    float currentVelocity() const { return v_curr_; }

private:
    // kDt: período do FSM — usado APENAS para integrar ∆v (filtro trapezoidal)
    static constexpr float kDt = 0.05f;

    // kLookahead: distância do setpoint = v_curr × kLookahead.
    // Deve ser ≈ 1/MPC_XY_P do PX4 (default MPC_XY_P=0.95 → kLookahead≈1.0).
    // Se kLookahead=kDt (0.05), o setpoint ficaria apenas 0.025m à frente a
    // 0.5 m/s, fazendo o PX4 voar a 0.024 m/s em vez de 0.5 m/s.
    static constexpr float kLookahead = 1.0f;

    float v_curr_;
};
