**Como foi rodado.** Os planejadores foram voados **pelo `mission_node`** (ações `plan_route` e `follow_route`, novas), no stack ROS 2 Humble real (MAVROS + `ardupilot_interface` + `mission`, dentro do `aircraft-image`), contra ArduPilot SITL e BiguaSim (Hydrone, mundo Bridge). Cada voo tem um stack novo e uma missão gerada: `plan_route` → `takeoff` → `follow_route` → `wait` → `land`. Os waypoints saem pelo serviço `SetReposition` (GUIDED, `GlobalPositionTarget`); posição e velocidade entram de `/mavros/local_position/{pose,velocity_local}`. No K5 a nova meta é publicada em `/planner/new_goal` durante o voo. A lógica de voo é a mesma (`PlannedFlight`) do executivo pymavlink da Etapa B.

**Ainda simulado:** a percepção (pontos de superfície dentro do raio, a partir da posição do EKF); os obstáculos existem de verdade no BiguaSim como props. **Só parte aérea**, 4 a 9 m acima do home. Energia executada = empuxo aplicado pela planta nas curvas de potência placeholder.

**Voos:** 1; SUCESSO: 1; 
* **Cenário B1** (1 voos, 1 SUCESSO): folga mín. 1.57 m, tempo 15.8 s, replan. 0.0/voo, ΔE -22 %, erro final 0.44 m.
* **Condição K1** (1 voos, 1 SUCESSO): folga mín. 1.57 m, tempo 15.8 s, replan. 0.0/voo, ΔE -22 %, erro final 0.44 m.
* **Planejador astar** (1 voos, 1 SUCESSO): folga mín. 1.57 m, tempo 15.8 s, replan. 0.0/voo, ΔE -22 %, erro final 0.44 m.
