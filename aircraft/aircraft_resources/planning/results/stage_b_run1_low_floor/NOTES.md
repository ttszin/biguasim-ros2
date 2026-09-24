**Como foi rodado.** Cada voo sobe uma pilha nova: ArduCopter SITL (o mesmo `t2_sitl_run.sh` e `t2_biguasim.parm` do T2), BiguaSim (mundo SkyDive/Bridge, Hydrone = perfil DjiMatrice) e um executivo em Python que fala MAVLink direto com o SITL (pymavlink, GUIDED, `SET_POSITION_TARGET_LOCAL_NED`). O executivo usa exatamente os mesmos planejadores, mapa, modelo de energia e replanejadores da Etapa A. **Não usa MAVROS nem o `mission_node`.** A percepção de obstáculos continua simulada (a partir da posição do EKF); os obstáculos existem de verdade no BiguaSim como props.

**Só a parte aérea**, com altitudes de 2 a 9 m acima do home (0 a 10 m evitam as manobras especiais do `set_reposition`), no corredor x = 8…48 m, y ≈ 0 do mundo Bridge, onde o T2 já voou. A parte subaquática não foi testada em SITL.

**Métricas** vêm da verdade do BiguaSim (não do EKF). A energia executada usa o empuxo total que a planta aplicou (`Σ k_eta·ω²`, k_eta = 6,64e-5) nas mesmas curvas de potência do planejamento; o BiguaSim não tem modelo de potência. Portanto os coeficientes seguem sendo placeholders, mas ΔE agora compara o plano com o empuxo real da planta, e não com o meu seguidor cinemático.

Voos: 12 (12 concluídos sem colisão).
