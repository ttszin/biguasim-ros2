# T8.2 — Planejadores A* e RRT* (mapa híbrido ar–água)

Implementação dos dois planejadores sobre uma infraestrutura comum, mais o harness de testes
da Etapa A do guia (BiguaSim + seguidor de caminho simples). Spec: *T8.2 — Guia de
implementação e testes dos planejadores (A\* e RRT\*) no BiguaSim*.

Tudo é Python (numpy/scipy), sem ROS, rodando no mesmo ambiente do BiguaSim
(`~/venv-ardupilot`, Python 3.12).

## Estrutura

| Arquivo | Papel |
|---|---|
| `config/planner.yaml` | **Todos** os hiperparâmetros e constantes físicas (passo, raio de rewiring, viés, resolução, peso da heurística, margens, μ, curvas de energia, bateria…). Fora do código, para o RSM só reescrever este arquivo (`Config.override`, ou `benchmark.py --set chave=valor`). |
| `medium.py` | Meios pela altura: ar `z>μ`, transição `-μ≤z≤μ`, água `z<-μ`. Regra da travessia vertical. |
| `world.py` | Obstáculos a priori analíticos (cilindros/caixas) + testes exatos segmento×obstáculo (vetorizados). |
| `hybrid_map.py` | **Mapa híbrido**: a priori analítico + camada esparsa de voxels (0,10 m) via `atualizar_ocupacao(celulas, fonte, timestamp)`. Margem por meio, incerteza subaquática (K4), colisão contínua, `gps_to_local`. |
| `energy.py` | Custo por **energia** (J) por meio, custo de transição à parte, `instant_power` para a energia executada. |
| `common.py`, `postprocess.py` | Resultado comum (`PlanResult`, status) e poda por linha de visada com parada em <1 % de ganho. |
| `rrt_star.py` | RRT\*: passo máximo, viés ao objetivo, melhor pai por energia dentro do raio, rewiring, travessia vertical forçada. Árvore **persistente** (`invalidate`, `reroot`, `grow`). |
| `astar.py` | A\* 3D em grade local, 26-conexo, custo = energia, heurística admissível, peso ajustável, **grade deslocada aleatória (A\*ᵣ)**. |
| `replan.py` | Replanejamento contínuo: `CLRRTReplanner` (mantém a árvore, poda ramos cortados) e `DStarLiteReplanner` (reaproveita a busca). |
| `interface.py` | Interface comum: waypoints da missão + mapa + estado + energia → waypoints podados por trecho de meio (AR/TRANSIÇÃO/ÁGUA), energia por trecho e total, status `SUCESSO / SEM_CAMINHO / TIMEOUT / ENERGIA_INSUFICIENTE`. |
| `scenario.py`, `scenarios/c1…c6*.yaml` | Cenários C1–C6 (obstáculos conhecidos, **ocultos**, **móveis**, mudança de missão). |
| `execution.py` | Etapa A: seguidor cinemático + sensores simulados + condições K1–K6 + métricas. |
| `flight_executive.py` | **`PlannedFlight`**: lógica de voo sem E/S (mapa, planejadores, percepção simulada, replanejamento, mudança de meta, energia, chegada em waypoints). Usada pelo `mission_node` e pelo executivo pymavlink. |
| `benchmark.py`, `report.py`, `make_pdf.py` | Campanha da Etapa A (matriz mínima, 30 seeds), relatório/figuras e o PDF com as três etapas. |
| `biguasim_replay.py` | Reproduz a missão dentro do BiguaSim (props + caminho desenhado + veículo). |
| `to_mission.py` | Converte um caminho em YAML de missão do `mission_node` com `go_to_known_gps_waypoint` + `wait_to_reach_waypoint` (rota fixa, sem replanejamento; para replanejar use `plan_route`/`follow_route`). |
| `config/planner.yaml`, `config/planner_sitl.yaml` | Configuração da Etapa A e do veículo real em SITL (velocidade 2,5 m/s, massa 3,8 kg, folga maior). |
| `stage_b/` | Etapa B (pymavlink): `sitl_runner.py` (BiguaSim + props + verdade), `sitl_exec.py`, `run_stage_b.py`, `hover_probe.py`, cenários `scenarios/b1…b5*.yaml`, `BIGUASIM_NOTES.md` (o que foi alterado/achado no BiguaSim). |
| `stage_c/` | Etapa C (`mission_node`): `run_stage_c.py` (orquestrador), `aircraft_stack.sh` (MAVROS + `ardupilot_interface` + `mission` no container). |
| `tests/` | 82 testes (`python3 -m pytest tests -q`, ~1,5 min; passam também dentro da `aircraft-image`, Python 3.10). |

Documentação das ações do `mission_node`: `aircraft/aircraft_ws/src/mission/README.md`. Exemplos de missão: `aircraft/aircraft_resources/missions/planned_route_*.yaml`.
Os logs brutos de cada voo (`results/*/runs/`) **não são versionados** (14 MB só na Etapa C); os CSVs, figuras, notas e o PDF sim, e tudo é reproduzível com os scripts acima.

Frame dos planejadores: `(norte, leste, cima)` em metros, `z = 0` na superfície da água (convenção
do BiguaSim). No BiguaSim: `x = norte`, `y = −leste` (y é "esquerda"), `z = cima`.

## Como rodar

```bash
cd aircraft/aircraft_resources/planning
python3 -m pytest tests -q                                     # testes (~3 min, os do RRT* são lentos)

# campanha completa do guia (30 seeds, K4 a 2/10/20 %, ~15-25 min com 12 núcleos)
python3 benchmark.py --seeds 30 --astar-seeds 3 --jobs 12 --k4-sweep --out results/campaign
python3 report.py results/campaign --convergence               # figuras + report.md

# um cenário no BiguaSim (precisa de GPU/viewport)
python3 biguasim_replay.py --scenario C6 --planner astar --condition K1 --viewport
python3 biguasim_replay.py --trace results/campaign/traces/C4_K1_None_rrt_star_None_0.json --viewport

# varredura de hiperparâmetro (RSM): sobrescreve a config sem tocar no código
python3 benchmark.py --seeds 10 --only C3 --set rrt_star.step=1.0 rrt_star.rewire_radius=4 --out results/rsm_a
```

## Decisões de projeto que valem conhecer

* **Mapa esparso, não denso.** A parte a priori é analítica; só o que os sensores inserem é guardado como voxels
  (dict + KD-tree). A pergunta "matriz de ocupação é obrigatória ou estrutura esparsa é aceitável?" continua
  aberta com a orientação: a interface `atualizar_ocupacao` é a mesma nos dois casos.
* **Colisão contínua, exata** (não amostrada) ao longo de cada segmento, com margem do meio:
  `raio do veículo + folga` (água maior) e, na água, `+ fração × distância ao último ponto com GPS` (K4).
  A inflação de **caixas é cúbica (AABB)**, mais conservadora que a distância euclidiana (~1,7× nos cantos).
* **Travessia da transição só na vertical**, imposta em três lugares: colisão (`violates_vertical_rule`),
  extensão do RRT\* (substitui a extensão por uma vertical) e movimentos do A\*. O seguidor também para antes
  de entrar na banda.
* **Custo de travessia aditivo**: metade de `fixed_wh` em cada plano ±μ cruzado, então continua correto se um waypoint cair dentro da banda.
* **RRT\* nunca prova que não há caminho**: falha de RRT\* é `TIMEOUT`; só o A\* devolve `SEM_CAMINHO`.
* **A\***: `exact_edges=false` valida arestas na rasterização e checa o caminho final de forma exata, refazendo com
  arestas exatas se falhar (todos os caminhos devolvidos são válidos no teste contínuo).
* **Grade deslocada (A\*ᵣ)**: o offset é sorteado por seed, então o A\* é reproduzível; a campanha roda 3 offsets por resolução.
* **Obstáculos móveis (C2)**: a posição do WhiteBoat é compartilhada (T10); o mapa recebe o volume varrido nos próximos
  6 s + folga de latência, senão o conflito só é visto com o barco já em cima da rota.
* **Seguidor (Etapa A)**: segue a *reta* do plano com ponto-cenoura (2 m à frente), freia antes das curvas (velocidade de chegada
  limitada pelo ângulo e pela folga do waypoint, por causa do atraso de velocidade), para antes de trechos que entram na banda, e só
  faz um pequeno ajuste horizontal (≤ 0,3 m/s) dentro dela. Tem proteção de progresso (`TRAVADO` após 20 s sem avançar).
  Esses três ajustes vieram de falhas reais da campanha (colisões em quinas e travamentos na banda), todas do seguidor, não dos planejadores.
* **Escape do RRT\***: se a raiz ficar presa dentro da inflação de um obstáculo móvel, só aceita arestas que se afastem.

## Resultados da campanha (`results/campaign/`, 1152 execuções)

Rode `python3 report.py results/campaign` para as tabelas completas (`report.md`) e figuras. Em resumo, **com energia placeholder**:

* Sucesso: A\* 173/174, RRT\* 868/870 nas execuções que decolam (as outras 108 são K6 = `ENERGIA_INSUFICIENTE` por construção). **Nenhuma colisão.**
  Falhas: 2 TIMEOUT do RRT\* no plano inicial do C6 com 20 % de incerteza e 1 TIMEOUT do A\* (1 m) no C4 com 20 %.
* Energia planejada em K1: RRT\* fica de +0,0 % a +0,3 % do A\* fino em C1/C2/C3, +3,5 a +4,6 % em C4/C5 e **+11,6 % no C6** (com 5 s por trecho).
* Tempo até a 1ª solução: RRT\* 40–500 ms; A\* fino 0,8–6,3 s. O RRT\* gasta o orçamento inteiro porque é anytime.
* Memória de pico (plano inicial): RRT\* 0,2–0,9 MB; A\* cresce com o cubo da resolução (0,4 → ~10 MB nas grades finas destes cenários).
* Replanejamento (média nas execuções que replanejaram), CL-RRT sobre a árvore do plano inicial vs D\* Lite: C3 161 vs 1450 ms, C6 133 vs 512 ms,
  C5 219 vs 492 ms, C2 264 vs 400 ms; **no C1 o D\* Lite ganha (81 vs 277 ms)**.
  Em mudança de missão (K5, C6): CL-RRT 127 ms vs D\* Lite 1889 ms (o D\* Lite reinicia a busca).
* **Correção de um erro meu na primeira versão da campanha:** o RRT\* descartava a árvore do plano inicial e o primeiro replanejamento planejava do zero.
  Corrigido (a árvore é mantida, os ramos cortados são podados e a árvore é reenraizada), e a campanha foi refeita. Planejar do zero era mais rápido no C3
  (42 ms) do que reaproveitar a árvore grande (161 ms): reaproveitar não é de graça.
* Os tempos são de desktop, 1 núcleo por execução, e os cenários são pequenos perto dos 200×200×30 m do guia: **não** extrapolar para a placa.

## Etapa B: BiguaSim + ArduPilot SITL (GUIDED), parte aérea

```bash
python3 stage_b/run_stage_b.py --out results/stage_b --scenarios b1_poles b2_gate --conditions K1 K2 --rrt-seeds 2   # ~1 h, 12 voos
python3 stage_b/run_stage_b.py --out results/stage_b --analyse-only                                                # refaz CSV/figuras
python3 make_pdf.py results/campaign --stage-b results/stage_b --out results/T8.2_resultados.pdf                   # PDF com as duas etapas
```

`stage_b/sitl_runner.py` (lado BiguaSim: props + verdade + empuxo), `stage_b/sitl_exec.py` (executivo MAVLink: planeja, arma, decola, voa, percebe, freia e replaneja),
`stage_b/run_stage_b.py` (uma pilha nova por voo + métricas), `stage_b/hover_probe.py` (sonda de hover para diagnóstico), cenários `stage_b/scenarios/b1_poles.yaml` e `b2_gate.yaml`
(referencial relativo ao home, altitudes 4 a 9 m). Resultado: `results/stage_b/` (`runs_b.csv`, `NOTES.md`, figuras).

Resumo (12 voos): 11 sem colisão, **1 com folga mínima de 0,44 m** (menor que o raio de 0,5 m) num voo do RRT\* em K2 com 11 replanejamentos em sequência (obstáculo oculto revelado só em parte).
Folga mínima média 1,5 m, erro lateral máximo médio 0,75 m, erro final ~0,4 m. **O GUIDED voou a 2,3 m/s de média com picos de ~5 m/s** (acima do `WPNAV_SPEED` de 2 m/s do T2),
então o plano usa 2,5 m/s, calibrado no primeiro voo (`results/stage_b_calibration`, fora da tabela). **A energia executada ficou acima da planejada** (ΔE = -19 % em K1; -36 % em K2, por causa
das paradas e replanejamentos). Detalhes e ressalvas em `results/stage_b/NOTES.md` e no PDF.

Limitações da Etapa B: 12 voos, 2 seeds de RRT\*, 1 resolução de A\*; só K1 e K2; percepção simulada; energia com coeficientes placeholder; corredor pequeno (35 m). A rodada anterior
(`results/stage_b_run1_low_floor`, piso de 2 m) teve 2 voos do RRT\* parados ~70 s por geometria real do Bridge perto do deck, ausente do mapa; por isso o piso passou a 4 m.

### Defeitos do ambiente encontrados (e o que foi feito)

1. **Regressão minha no BiguaSim (corrigida em `~/biguasim`, não commitada):** na sessão do BlueROV2 inverti o giro de guinada em `ardubridge/frame.py::imu_glu_to_frd` para todos os veículos, o que
   derrubava o Hydrone (o hover do T2 já não era estável). Agora é opt-in por perfil (`VehicleProfile.flip_gyro_yaw`, verdadeiro só em BlueROV2/BlueROVHeavy). O BlueBoat também
   ficou afetado entre 30/08 e agora; vale revalidar se houve teste dele nesse período.
2. **Ponte BiguaSim→ArduPilot (NÃO corrigida na biblioteca):** `build_json_state` envia `[lat, lon, alt]` em `"position"`, mas o backend JSON do ArduPilot lê `"position"` como NED em metros relativo ao home
   (latitude/longitude/altitude têm chaves próprias). Lido assim, a posição horizontal não muda e a altitude tem o sinal invertido: o EKF "desce" enquanto o drone sobe e o controlador empurra mais (fuga vertical).
   O `sitl_runner.py` reescreve o campo para o formato esperado. **O runner do T2 (`biguasim_sim_runner.py`) usa a ponte padrão e provavelmente carrega esse defeito**; convém o Kauã/você avaliarem.
3. `depth_to_pressure` fixa 101325 Pa acima da água (o barômetro não acompanha a altitude no ar), e o SITL só arma com giroscópio ≥ 1,8 × `SCHED_LOOP_RATE` = 216 Hz, por isso o runner usa `--ticks 250`.

## Etapa C: os planejadores dentro do `mission_node`, em SITL

Integração ao stack real. Duas ações novas no `aircraft_ws/src/mission/mission/mission_node.py`:

* `plan_route` (antes do `takeoff`): carrega um cenário (`planning/stage_b/scenarios/*.yaml`), planeja com **A\*** ou **RRT\***, confere o orçamento de energia e **recusa a missão**
  (`ENERGIA_INSUFICIENTE`, `SEM_CAMINHO`, `TIMEOUT`) antes de decolar. Parâmetros: `scenario`, `planner`, `condition` (K1/K2/K3/K5/K6), `seed`, `resolution`, `config`
  (padrão `config/planner_sitl.yaml`), `overrides`, `energy_available_wh`, `mission_change`, `result_file`.
* `follow_route` (depois do `takeoff`): laço de 10 Hz com `PlannedFlight` (`planning/flight_executive.py`). Posição/velocidade vêm de `/mavros/local_position/{pose,velocity_local}`
  (ENU, origem = home); os waypoints saem pelo serviço `SetReposition` (GUIDED, o mesmo caminho do `go_to_known_gps_waypoint`). Replaneja quando um obstáculo percebido corta a rota,
  freia (`hold`) enquanto replaneja, faz fly-by ou para no waypoint conforme a curva, e aceita uma **meta nova em `/planner/new_goal`** (`geometry_msgs/Point`: x=norte, y=leste, z=altura, m
  relativos ao home): é o caminho que uma decisão do T11 tomaria. O resultado vai para `result_file` (JSON).

`PlannedFlight` não tem E/S: o `mission_node` (ROS 2, Python 3.10) e o executivo pymavlink da Etapa B (`stage_b/sitl_exec.py`) são adaptadores finos sobre ele. Os testes do núcleo
(`tests/test_flight_executive.py`) rodam também dentro da imagem `aircraft-image` (Python 3.10, scipy 1.8).

```bash
python3 stage_c/run_stage_c.py --out results/stage_c --matrix full        # 33 voos, ~3 h
python3 stage_c/run_stage_c.py --out results/stage_c_smoke --matrix smoke  # 1 voo, para checar o ambiente
```

`stage_c/run_stage_c.py` sobe, por voo: SITL, o runner do BiguaSim (com os obstáculos como props) e o container `aircraft-image` (MAVROS + `ardupilot_interface` + `mission`,
`stage_c/aircraft_stack.sh`). O `mission_node.py` modificado é montado por cima do da imagem (que usa `--symlink-install`), e `aircraft_resources/` é montado para achar o pacote
de planejamento; a imagem não precisa ser reconstruída. A missão é gerada: `plan_route → takeoff → follow_route → wait`. **Não há `land`**: a ação `Land` do `ardupilot_interface`
passa por RTL (sobe, volta e pousa), o que leva minutos no BiguaSim e não é o que se avalia; o veículo fica segurando a meta em GUIDED até o stack ser derrubado.

**Resultados (33 voos, `results/stage_c/`):** 32 voos concluídos (SUCESSO) e 1 recusado por energia (K6, `ENERGIA_INSUFICIENTE`, antes de decolar), **sem colisões**; folga mínima 0,84 m
(A\* 1,28 m e RRT\* 1,48 m em média), erro final ~0,4 m (~0,4 m também nos K5, contra a meta nova). No K5 a meta nova chegou pelo tópico e foi absorvida em 32 ms (A\*) e 9 a 21 ms (RRT\*).
Replanejamento em K2/K3: A\* 137 ms em média (máx. 297) contra RRT\* 31 ms (máx. 69). Voos com replanejamento: RRT\* 3,2 replanejamentos por voo; A\* 3,0 (9,9 com um voo atípico
do B5/K2, com 51). **A energia executada ficou 40 a 50 % acima da planejada** (ΔE de -39 % em K1, -50 % em K2, empuxo real da planta nas curvas placeholder), mais que na Etapa B (-19 %),
porque os cenários novos têm mais curvas e paradas. No B5 (viga) os seis voos passaram por cima da viga a 7,3 a 8,3 m (topo em 6,4 m), verificado na altitude real.

Falhas achadas e corrigidas ao integrar (o que só aparece com o stack real): (1) `plan_route` era reentrante com o RRT\* (4 s de planejamento, temporizador de 1 Hz em executor multi-thread), o que
derrubava o nó; (2) o `ardupilot_interface` recusa `SetReposition` enviado enquanto processa o anterior e o comando recusado se perdia (um voo do B4 ficou 276 s parado); passou a haver entrega
serializada com repetição. **Na campanha final o interface não recusou nenhuma chamada (0), então o mecanismo de repetição nunca foi exercitado ao vivo**; só a serialização foi comprovada.
Os voos K2 anteriores à correção estão arquivados em `results/stage_c_discarded_race`.

Cenários (todos com altitude relativa ao home entre 4 e 9 m): B1 (dois postes, um oculto), B2 (parede com fresta de 5 m), **B3** (campo de postes escalonados, dois ocultos; K5 move a meta),
**B4** (dois blocos deslocados, curva em S, poste oculto entre eles), **B5** (viga a 5,6–6,4 m que obriga a passar **por cima**: evitação vertical).

## Cenários e condições

C1 longo/ar aberto · C2 barco móvel · C3 curto ao redor da boia · C4 entrada na água · C5 subaquático perto da boia · C6 missão completa
(origem → ROI → água → retorno). Condições: K1 mapa conhecido · K2 obstáculo não mapeado (raio 12 m) · K3 atualização contínua (raio 30 m)
· K4 incerteza 2–20 % · K5 mudança de missão · K6 energia reduzida (espera `ENERGIA_INSUFICIENTE`).
Matriz mínima: C1/C5/C6 em todas; C2→K5, C3→K2, C4→K4 (mais K1).

## O que está pronto × o que NÃO está (contra os entregáveis do guia)

| Entregável | Estado |
|---|---|
| Infraestrutura comum (mapa, custo, colisão, interface) com testes unitários | **Pronto** |
| RRT\* integrado | **Pronto**: Etapa A, Etapa B (12 voos, pymavlink) e **Etapa C: dentro do `mission_node`** (33 voos em SITL, stack ROS 2 real), só parte aérea. |
| A\* baseline | **Pronto** (com D\* Lite p/ replanejamento) |
| Scripts de cenários e coleta de métricas, seeds fixas | **Pronto** (`benchmark.py`, seeds 0..29) |
| Planilha de resultados por cenário e condição | **Pronto em CSV** (`runs.csv`, `summary.csv`); não gerei `.xlsx` (sem `openpyxl` instalado) |
| Relatório com escolha, justificativa e limitações | Rascunho automático em `results/*/report.md`; **a escolha final e o texto são seus** |
| Medição de tempo/memória na placa embarcada | **Não feito.** Só desktop, 1 núcleo/execução, tempos também em % do orçamento; `--cpu-slowdown` é só estimativa. |
| Implementação final com o algoritmo escolhido | **Não feito** (depende da escolha) |
| Vídeos das missões | **Não gravei.** `biguasim_replay.py` reproduz cada missão para você gravar a tela. |

### Limitações e placeholders — leia antes de citar números

1. **Coeficientes de energia são PLACEHOLDER** (`config/planner.yaml`): as curvas de 2º grau de Pinheiro et al. (IROS 2024) não estavam
   no guia. Ordem de grandeza plausível (~220 W em hover no ar), mas Wh absolutos não valem; a comparação **A\* × RRT\*** vale.
2. **μ = 0,5 m e margens são placeholders** (μ vem da T7).
3. **ΔE mede desvio de execução, não fidelidade do modelo**: a energia executada usa as mesmas curvas do planejamento (ressalva do guia).
   O BiguaSim **não** foi consultado para empuxo/potência dos propulsores; isso ainda precisa ser verificado e documentado.
4. **Obstáculos são props sintéticos**, não a geometria real do Bridge (ponte/pilares não estão no mapa dos planejadores).
   `biguasim_replay.py --home/--yaw-deg` posiciona o cenário; confira no viewport. O replay do C3 (K1 e K2, A\*) rodou headless no BiguaSim e o agente seguiu a trajetória (erro máx. 0,40 m em 58 passos, lido do LocationSensor). **Não vi a cena**: o tamanho/orientação dos props (`spawn_prop` cilindro/caixa), o desenho do caminho e a posição em relação à ponte não foram inspecionados visualmente, e C1/C2/C4–C6 não foram reproduzidos no motor.
5. **Etapa B (BiguaSim + ArduPilot SITL em GUIDED): parte aérea** (ver seção abaixo). **Etapa D: parte subaquática em SITL** com o BlueROV2/ArduSub
   (`stage_d/`, `stage_d/NOTES.md`): 10 voos, 10 SUCESSO, sem colisão. **Ainda não testado em SITL:** o cruzamento ar↔água (transição) e a
   incerteza de posição subaquática real (K4 segue simulada; o GPS_INPUT do ROV vem da verdade do BiguaSim). A Etapa B fala MAVLink direto
   (pymavlink); a Etapa C passa pelo `mission_node`/MAVROS (só quadricóptero; a versão `sub` do `plan_route` não foi feita).
6. **K4 usa incerteza proporcional à distância ao último ponto com GPS** (waypoint anterior ao primeiro subaquático), aproximação da esfera de incerteza.
7. **Detecção de obstáculos é simulada** (pontos de superfície dentro de um raio); não há LiDAR/sonar reais (T4/T5/T6).
8. **Modelo do seguidor** (cinemático com lag de velocidade e limite de aceleração) é simples de propósito (guia: "seguidor de caminho simples").

## Pendências a confirmar com a equipe (do guia)

Modelo da placa e memória livre com YOLO · matriz de ocupação vs. esparsa · integração BiguaSim×ArduPilot SITL · estimativa de posição subaquática no GUIDED (T5/T6)
· sensores (LiDAR? sonar no WhiteBoat?) · o RSM pertence à 8.2? · altura da zona de transição μ (T7) · coeficientes de energia.

## Vídeos dos voos e gráficos comparativos

* `record_videos.py`: grava voos completos em SITL (ar: Hydrone/ArduCopter, Etapa B; água: BlueROV2/ArduSub, Etapa D) com `--record` nos runners.
  A câmera é um `RGBCamera` do BiguaSim presa ao veículo (o `move_viewport` do BiguaSim desta versão não move o `ViewportCapture`), a 25 Hz, com painel
  (tempo desde o início da rota, altura ou profundidade, velocidade) e a trilha voada desenhada no cenário (`draw_line`). Saída H.264 em
  `results/videos/<nome>.mp4`, cortada do início da rota −3 s até o fim +3 s. Só `videos.csv` e `overview.png` são versionados; os `.mp4` não
  (`.gitignore`), regenere com `python3 record_videos.py [--only nome ...]`. Gravar deixa a simulação ~2× mais lenta (o SITL roda em tempo simulado).
  Convenção da câmera do BiguaSim: rotação `[roll, pitch, yaw]`, pitch POSITIVO olha para baixo.
* `make_comparison_plots.py`: `results/comparison/stage_a_metrics.png` (campanha da Etapa A, 30 seeds) e `sitl_metrics.png` (voos em SITL), mais os CSVs
  com os números. O "tempo até o primeiro caminho" do RRT\* é o de `t_first_solution_ms`; o `t_initial_ms` dele é o orçamento fixo (5 s por trecho), pois ele
  continua refinando.
