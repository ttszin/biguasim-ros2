# BiguaSim ROS2 — BROS Fork

Fork do repositório [hydrone-furg/biguasim-ros2](https://github.com/hydrone-furg/biguasim-ros2) com missões desenvolvidas pelo grupo BROS (Hydrone/FURG) para o desafio T2: transição automática de modo de navegação AERIAL_NAV ↔ AQUATIC_NAV em drone híbrido.

---

## Conteúdo deste repositório

```
biguasim_interfaces/        # Mensagens ROS2 customizadas (original)
biguasim_main/              # Nó ROS2 principal do BiguaSim (original)
missions/
  t2_hybrid_transition/     # Missão T2 — transição aérea/aquática
```

---

## Missão T2 — Transição AERIAL_NAV ↔ AQUATIC_NAV

### O que faz

O drone decola, voa até uma região com água, desce até submergir, aguarda a confirmação do modo `AQUATIC_NAV`, sobe de volta à superfície, aguarda `AERIAL_NAV` e pousa.

A detecção de submersão é feita por um sensor de pressão hidrostática simulado:
- **BiguaSim (recomendado):** `biguasim_bridge_runner.py` lê o `DepthSensor` real do mundo Bridge (SkyDive) e publica `/fcu/external_pressure`.
- **Gazebo/PX4 (referência):** `hydro_sensor_bridge.py` simula o sensor via bounding-box da posição local do drone.

O nó `validador_t2.py` subscreve `/fcu/external_pressure` e publica `/nav_mode` (`AERIAL_NAV` ou `AQUATIC_NAV`) com histerese + debounce de 1,5 s.

### Diagrama de estados

```
IDLE → AERIAL_NAV → TRANSITION_DOWN → AQUATIC_NAV → TRANSITION_UP → LAND
```

### Pipeline ROS2

```
/mavros/local_position/pose   ←─ MAVROS ←─ ArduPilot SITL
        │
        └─► biguasim_bridge_runner.py ──► /fcu/external_pressure
                                                  │
                                          validador_t2.py ──► /nav_mode
                                                                    │
                                                          mission_node.py
```

---

## Pré-requisitos

| Componente | Versão testada |
|---|---|
| Ubuntu | 24.04 LTS |
| ROS 2 | Jazzy |
| BiguaSim | 1.0.0 (pacote SkyDive/Bridge instalado) |
| ArduPilot | latest stable (apenas no Modo ArduPilot) |
| MAVROS | ros-jazzy-mavros (apenas no Modo ArduPilot) |
| aerial-autonomy-stack | [hydrone-furg/aerial-autonomy-stack](https://github.com/hydrone-furg/aerial-autonomy-stack) (apenas no Modo ArduPilot) |

**Variáveis de ambiente opcionais (default entre parênteses):**

```bash
export ARDUPILOT_PATH=~/ardupilot          # diretório raiz do ArduPilot
export STACK_WS=~/aerial-autonomy-stack/aircraft/aircraft_ws  # workspace compilado
```

---

## Modo 1 — Direto (somente BiguaSim, sem ArduPilot)

Modo mais simples, ideal para calibração e validação do sensor de profundidade. Usa controle `cmd_pos_yaw` nativo do BiguaSim, sem necessidade de MAVROS ou ArduPilot.

### Terminal 1 — Runner + sensor de pressão

```bash
cd missions/t2_hybrid_transition

# Missão completa (viewpoint do UE5 oculto):
python3 t2_direct_runner.py

# Com janela do Unreal Engine visível:
python3 t2_direct_runner.py --viewport

# Modo calibração (drone voa até a água e imprime posição + depth a cada segundo):
python3 t2_direct_runner.py --viewport --calibrate --water-x 25 --water-y 0
```

### Terminal 2 — Validador de modo

```bash
cd missions/t2_hybrid_transition
python3 validador_t2.py
```

### Parâmetros do t2_direct_runner.py

| Parâmetro | Default | Descrição |
|---|---|---|
| `--viewport` | off | Abre a janela do Unreal Engine |
| `--spawn X Y Z` | `8 0 13.4` | Posição de spawn no mundo NWU (m) |
| `--cruise-z` | `28.0` | Altitude de cruzeiro no mundo NWU (m) |
| `--water-x` | `25.0` | Coordenada X da água no mundo NWU (m) |
| `--water-y` | `0.0` | Coordenada Y da água no mundo NWU (m) |
| `--descent-z` | `-2.0` | Z alvo abaixo da superfície (m) |
| `--speed` | `3.0` | Multiplicador de ganho de posição XY |
| `--speed-z` | `3.0` | Multiplicador de velocidade vertical |
| `--ticks` | `100` | Ticks por segundo da simulação |
| `--calibrate` | off | Voa até water-xy e imprime posição + depth |

---

## Modo 2 — ArduPilot (pipeline completo)

Pipeline completo com ArduPilot SITL + MAVROS + `ardupilot_interface` + `mission_node`. É o modo exigido pela tarefa T2.

> Abra **5 terminais** na pasta `missions/t2_hybrid_transition/`.

### Terminal 1 — BiguaSim Bridge

Inicia a simulação BiguaSim (mundo Bridge/SkyDive) e publica `/fcu/external_pressure` para o validador:

```bash
python3 biguasim_bridge_runner.py

# Com janela UE5 visível:
python3 biguasim_bridge_runner.py --viewport

# Posição de spawn customizada:
python3 biguasim_bridge_runner.py --viewport --location 8 0 13.4
```

Aguarde a mensagem:
```
BiguaSim pressure bridge ready → /fcu/external_pressure
```

### Terminal 2 — ArduPilot SITL

```bash
bash scripts/t2_sitl_run.sh
```

> Por padrão usa `$ARDUPILOT_PATH/Tools/autotest/t2_params.parm`. Passe o caminho do arquivo como argumento para usar outro arquivo de parâmetros:
> ```bash
> bash scripts/t2_sitl_run.sh /caminho/para/meus_params.parm
> ```

Aguarde o console ArduPilot exibir `EKF3 IMU0 is using GPS` e a altitude estabilizar.

### Terminal 3 — MAVROS

```bash
bash scripts/t2_mavros_run.sh
```

Aguarde:
```
MAVROS connected. Requesting data streams... Streams requested.
```

### Terminal 4 — ardupilot_interface

```bash
bash scripts/t2_ardupilot_interface_run.sh
```

Aguarde a mensagem de conexão do `ardupilot_interface`.

### Terminal 5 — Validador de modo

```bash
python3 validador_t2.py
```

Aguarde:
```
Validator started. Initial mode: AERIAL_NAV
```

### Terminal 6 — Nó de missão

```bash
bash scripts/t2_biguasim_run.sh

# Ou com conops customizado:
bash scripts/t2_biguasim_run.sh /caminho/para/missao.yaml
```

O nó de missão executa a sequência definida em `t2_biguasim_mission.yaml` e vai imprimir o progresso de cada passo.

---

## Missões disponíveis (YAML)

| Arquivo | Descrição |
|---|---|
| `t2_biguasim_mission.yaml` | Missão completa T2 (ArduPilot, mundo Bridge) |
| `t2_hover_test.yaml` | Decola, paira 3 min, pousa — smoke test aéreo |

### Coordenadas de referência (mundo Bridge / BiguaSim)

| Ponto | NWU x | NWU y | NWU z |
|---|---|---|---|
| Spawn / GPS home | 8.0 | 0.0 | 13.4 |
| Acima da água | 25.0 | 0.0 | 28.0 |
| Dentro da água | 25.0 | 0.0 | < 13.4 |

GPS home: lat=-35.36296, lon=149.16393, alt_ell=590.12 m (corresponds to `-L RATBeach` in ArduPilot SITL).

Altitudes na missão YAML são **relativas ao GPS home** (convenção MAVROS). A altitude de cruzeiro `5.0` = mundo z≈18.4 m; a altitude de descida `-15.0` = mundo z≈-1.6 m (≈1.6 m abaixo da superfície da água).

---

## Arquitetura dos nós

### `biguasim_bridge_runner.py`

- Estende `ArduBiguaSimRunner` do pacote `biguasim.ardubridge`
- Lê `DepthSensor` do mundo Bridge a cada tick
- Converte profundidade → pressão hidrostática via `depth_to_pressure()`
- Publica `sensor_msgs/FluidPressure` em `/fcu/external_pressure` (10 Hz)
- Envia o mesmo estado JSON para o ArduPilot SITL (porta 9002 UDP)

### `hydro_sensor_bridge.py`

- Alternativa para simulações com PX4 + Gazebo (sem BiguaSim)
- Subscreve `/DroneN/fmu/out/vehicle_local_position` (PX4 DDS)
- Simula pressão via bounding-box: tank 5×5 m em East=10, North=0
- Publica `/fcu/external_pressure`

### `validador_t2.py`

- Subscreve `/fcu/external_pressure`
- Histerese: entra em AQUATIC_NAV com P > 102 500 Pa (~12 cm de profundidade); sai com P < 101 500 Pa (~1,7 cm)
- Debounce: exige 1,5 s contínuos de leitura consistente antes de trocar o modo
- Publica `std_msgs/String` em `/nav_mode`

### `t2_direct_runner.py`

- Controle de posição via `cmd_pos_yaw` nativo do BiguaSim (sem ArduPilot)
- Máquina de estados: WARMUP → TAKEOFF → FLY_TO_WATER → HOVER → DESCEND → ASCEND → HOVER_EXIT → RETURN_HOME → DONE
- Publica `/fcu/external_pressure` (mesmo tópico do bridge)
- Modo `--calibrate`: voa até a água e imprime posição + depth para ajuste fino das coordenadas

---

## Troubleshooting

**`DepthSensor ausente no agent_state`**
Verifique que o perfil de veículo tem `include_depth_sensor=True` e que o mundo Bridge foi carregado corretamente. O sensor demora alguns frames para aparecer após o spawn.

**AQUATIC_NAV nunca ativado**
Use `--calibrate` para confirmar que o drone realmente está abaixo da superfície. Ajuste `--descent-z` para um valor mais negativo.

**ArduPilot SITL não conecta ao BiguaSim**
Confirme que o bridge está rodando **antes** do SITL e que a porta 9002 UDP está livre: `ss -ulnp | grep 9002`.

**MAVROS RTT too high**
Mensagem suprimida pelo filtro `grep`. O MAVROS funciona normalmente mesmo com alta latência de loopback local.

**EKF não converge**
Aguarde pelo menos 30 s após o SITL iniciar antes de tentar armar. A flag `warmup_frames=500` do bridge garante 2,5 s de frames antes de enviar comandos de motor, mas o EKF pode demorar mais.
