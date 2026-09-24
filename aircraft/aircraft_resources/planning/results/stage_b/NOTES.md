**Como foi rodado.** Cada voo sobe uma pilha nova: ArduCopter SITL (o mesmo `t2_sitl_run.sh` e `t2_biguasim.parm` do T2), BiguaSim (mundo SkyDive/Bridge, Hydrone = perfil DjiMatrice) e um executivo em Python que fala MAVLink direto com o SITL (pymavlink, GUIDED, `SET_POSITION_TARGET_LOCAL_NED`). O executivo usa os mesmos planejadores, mapa, modelo de energia e replanejadores da Etapa A. **Não usa MAVROS nem o `mission_node`.** A percepção de obstáculos continua simulada (pontos de superfície dentro de 12 m da posição do EKF); os obstáculos existem de verdade no BiguaSim como props.

**Só a parte aérea**, 4 a 9 m acima do home, no corredor x = 8…48 m, y ≈ 0 do mundo Bridge. A parte subaquática não foi testada em SITL.

**Métricas** vêm da verdade do BiguaSim (não do EKF). A energia executada usa o empuxo total que a planta aplicou (`Σ k_eta·ω²`, k_eta = 6,64e-5) nas mesmas curvas de potência do planejamento (coeficientes placeholder; o BiguaSim não tem modelo de potência). ΔE = planejada − executada, então **ΔE negativo significa que o voo gastou mais que o plano**.

**Voos:** 12; concluídos sem colisão: 11; folga mínima abaixo do raio do veículo (0,5 m): 1.

* **A*** (4 voos): energia planejada 0.82 Wh, executada 1.00 Wh (ΔE -21 %), comprimento voado 36.5 m (planejado 35.0 m), tempo 15.8 s, folga mínima média 1.52 m, erro lateral máximo médio 0.76 m, erro final médio 0.37 m.
* **RRT*** (8 voos): energia planejada 0.82 Wh, executada 1.07 Wh (ΔE -31 %), comprimento voado 38.5 m (planejado 35.1 m), tempo 17.1 s, folga mínima média 1.47 m, erro lateral máximo médio 0.73 m, erro final médio 0.39 m.
* **K1** (6 voos): 0.0 replanejamentos por voo, tempo 15.5 s, ΔE -19 %, sem replanejamento.
* **K2** (6 voos): 4.3 replanejamentos por voo, tempo 17.8 s, ΔE -36 %, tempo máximo de replanejamento 67 ms.
* **Velocidade:** média 2.28 m/s com picos de 5.1 m/s, acima dos 2 m/s do `WPNAV_SPEED` do T2. O plano usa 2,5 m/s, calibrado no primeiro voo (`stage_b_calibration`, que **não** entra na tabela: só os voos seguintes dizem algo sobre ΔE).
* **Paradas em waypoints (stop-and-go):** 2 nos 12 voos; nos demais o ArduPilot fez fly-by, com queda de velocidade no waypoint.
* **Folga abaixo do raio do veículo:** `b1_poles_rrt_star_K2_s1` (11 replanejamentos, folga mínima 0.44 m). O obstáculo oculto é revelado só em parte (pontos de superfície dentro do raio), o plano passa por trás do que ainda não foi visto e cada nova porção corta o caminho outra vez: houve uma sequência de replanejamentos em poucos segundos com o veículo já dentro da zona de folga, e o GUIDED não para de imediato. Não há evidência de contato físico, só de que a margem de segurança foi violada.

**Rodada anterior (`stage_b_run1_low_floor`, piso de 2 m):** 2 dos 12 voos do RRT\* ficaram ~70 s parados (velocidade média 0,5 m/s, energia 3 a 7 Wh) depois de o plano descer a ~3,5 m do home. O ArduPilot registrou `Yaw Imbalance 38–50 %` e o veículo só seguiu depois. A causa provável é geometria real do Bridge perto do deck (o home fica sobre a ponte), ausente do mapa dos planejadores; não confirmei o objeto. A campanha desta seção usa piso de 4 m.

**Ajustes no ambiente que foram necessários** (detalhes no README): (1) o giro de guinada estava invertido para todos os veículos por uma alteração minha anterior no BiguaSim, o que derrubava o Hydrone; (2) a ponte BiguaSim→ArduPilot manda `[lat, lon, alt]` no campo `position`, que o ArduPilot lê como NED em metros (o runner do Stage B reescreve o campo); (3) o SITL só arma com giroscópio ≥ 216 Hz, então o runner usa 250 ticks/s.
