**Como foi rodado.** Os planejadores foram voados **pelo `mission_node`** (ações `plan_route` e `follow_route`, novas), no stack ROS 2 Humble real (MAVROS + `ardupilot_interface` + `mission`, dentro do `aircraft-image`), contra ArduPilot SITL e BiguaSim (Hydrone, mundo Bridge). Cada voo tem um stack novo e uma missão gerada: `plan_route` → `takeoff` → `follow_route` → `wait` → `land`. Os waypoints saem pelo serviço `SetReposition` (GUIDED, `GlobalPositionTarget`); posição e velocidade entram de `/mavros/local_position/{pose,velocity_local}`. No K5 a nova meta é publicada em `/planner/new_goal` durante o voo. A lógica de voo é a mesma (`PlannedFlight`) do executivo pymavlink da Etapa B.

**Ainda simulado:** a percepção (pontos de superfície dentro do raio, a partir da posição do EKF); os obstáculos existem de verdade no BiguaSim como props. **Só parte aérea**, 4 a 9 m acima do home. Energia executada = empuxo aplicado pela planta nas curvas de potência placeholder.

**Voos:** 33; SUCESSO: 32; ENERGIA_INSUFICIENTE: 1
* **Cenário B1** (4 voos, 4 SUCESSO): folga mín. 1.77 m, tempo 17.2 s, replan. 2.2/voo, ΔE -32 %, erro final 0.39 m.
* **Cenário B2** (4 voos, 4 SUCESSO): folga mín. 1.46 m, tempo 17.1 s, replan. 1.2/voo, ΔE -29 %, erro final 0.40 m.
* **Cenário B3** (13 voos, 12 SUCESSO): folga mín. 1.44 m, tempo 19.7 s, replan. 1.8/voo, ΔE -42 %, erro final 0.40 m.
* **Cenário B4** (6 voos, 6 SUCESSO): folga mín. 1.02 m, tempo 27.1 s, replan. 3.0/voo, ΔE -64 %, erro final 0.38 m.
* **Cenário B5** (6 voos, 6 SUCESSO): folga mín. 1.43 m, tempo 19.7 s, replan. 8.8/voo, ΔE -43 %, erro final 0.40 m.
* **Condição K1** (13 voos, 13 SUCESSO): folga mín. 1.36 m, tempo 20.1 s, replan. 0.0/voo, ΔE -39 %, erro final 0.38 m.
* **Condição K2** (13 voos, 13 SUCESSO): folga mín. 1.46 m, tempo 21.3 s, replan. 7.6/voo, ΔE -50 %, erro final 0.40 m.
* **Condição K3** (3 voos, 3 SUCESSO): folga mín. 1.33 m, tempo 19.4 s, replan. 1.7/voo, ΔE -42 %, erro final 0.40 m.
* **Condição K5** (3 voos, 3 SUCESSO): folga mín. 1.43 m, tempo 19.6 s, replan. 1.0/voo, ΔE -41 %, erro final 0.42 m.
* **Condição K6** (1 voos, 0 SUCESSO): folga mín. nan m, tempo nan s, replan. nan/voo, ΔE +nan %, erro final nan m.
* **Planejador astar** (13 voos, 12 SUCESSO): folga mín. 1.28 m, tempo 20.0 s, replan. 5.8/voo, ΔE -42 %, erro final 0.40 m.
* **Planejador rrt_star** (20 voos, 20 SUCESSO): folga mín. 1.48 m, tempo 20.7 s, replan. 1.9/voo, ΔE -45 %, erro final 0.39 m.
