**Como foi rodado.** Os planejadores foram voados **pelo `mission_node`** (ações `plan_route` e `follow_route`, novas), no stack ROS 2 Humble real (MAVROS + `ardupilot_interface` + `mission`, dentro do `aircraft-image`), contra ArduPilot SITL e BiguaSim (Hydrone, mundo Bridge). Cada voo tem um stack novo e uma missão gerada: `plan_route` → `takeoff` → `follow_route` → `wait` → `land`. Os waypoints saem pelo serviço `SetReposition` (GUIDED, `GlobalPositionTarget`); posição e velocidade entram de `/mavros/local_position/{pose,velocity_local}`. No K5 a nova meta é publicada em `/planner/new_goal` durante o voo. A lógica de voo é a mesma (`PlannedFlight`) do executivo pymavlink da Etapa B.

**Ainda simulado:** a percepção (pontos de superfície dentro do raio, a partir da posição do EKF); os obstáculos existem de verdade no BiguaSim como props. **Só parte aérea**, 4 a 9 m acima do home. Energia executada = empuxo aplicado pela planta nas curvas de potência placeholder.

**Voos:** 2; SUCESSO: 0; COLISAO: 1; ENERGIA_INSUFICIENTE: 1
* **Cenário B3** (2 voos, 0 SUCESSO): folga mín. 0.41 m, tempo 18.8 s, replan. 1.0/voo, ΔE -35 %, erro final 8.15 m.
* **Condição K5** (1 voos, 0 SUCESSO): folga mín. 0.41 m, tempo 18.8 s, replan. 1.0/voo, ΔE -35 %, erro final 8.15 m.
* **Condição K6** (1 voos, 0 SUCESSO): folga mín. nan m, tempo nan s, replan. nan/voo, ΔE +nan %, erro final nan m.
* **Planejador astar** (2 voos, 0 SUCESSO): folga mín. 0.41 m, tempo 18.8 s, replan. 1.0/voo, ΔE -35 %, erro final 8.15 m.
