# T8.2: escolha do algoritmo de planejamento (A\* x RRT\*): rascunho

**Estado: rascunho para a equipe decidir.** Os números são dos experimentos deste repositório (Etapas A a E); a decisão final e as pendências abaixo não são minhas. Todas as energias usam coeficientes **placeholder** (`config/planner*.yaml`): compare os algoritmos entre si, não cite Wh absolutos. Os tempos são de desktop, 1 núcleo por execução, não da placa embarcada.

## 1. Recomendação provisória

**RRT\* com custo energético como planejador de produção, mantendo o A\* no repositório como baseline e ferramenta de teste** (é o que o critério de aceite pede para o protótipo não escolhido).

Motivos, todos medidos:

1. **Primeiro caminho bem mais rápido**: 40 a 500 ms no RRT\* contra 125 a 6300 ms no A\* (grade fina), 2 a 13 vezes.
2. **Memória**: mediana de 0,4 MB no RRT\* contra 2,2 MB no A\*, e o A\* cresce com o cubo da resolução (C1: 1,2 MB a 4 m, 9,9 MB a 2 m).
3. **Replanejamento em SITL 3 a 6 vezes mais rápido** (Etapas B, C e D) e pior caso de 1,8 % do orçamento de tempo real contra 12,3 % do A\* na campanha.
4. **Mesma taxa de sucesso, mesmo comprimento de caminho, mesmo número de waypoints.**

O que pesa contra, também medido:

* **Energia planejada maior**: em média +4,6 % (mediana +2,6 %, máximo +22,7 %) sobre o A\* na grade mais fina. Igual em C1 a C3 (ar); **+8,9 % em C4 e +10,9 % em C6** (missões com a transição) e +3,1 % em C5 (subaquático). Os planos do A\* foram mais baratos justamente onde há água, o que confirma o alerta do guia de que a hipótese "RRT\* ganha" pode cair na fase subaquática.
* **Não determinístico**: 30 seeds por caso, e no SITL aéreo um dos 8 voos do RRT\* ficou a 0,44 m de um obstáculo (raio do veículo 0,5 m; sem evidência de contato).
* O RRT\* usa o **orçamento fixo por trecho** (5 s) inteiro para refinar; a energia melhora com o tempo e eu **não** medi quanto (hipótese, não resultado).

## 2. O que foi testado

| Etapa | O quê | Tamanho |
|---|---|---|
| A | BiguaSim com seguidor simples: C1 a C6 × K1 a K6, RRT\* 30 seeds, A\* em 2 resoluções por cenário | 1152 execuções |
| B | Hydrone (ArduCopter GUIDED) em SITL, pymavlink | 12 voos |
| C | Hydrone via `mission_node`/MAVROS | 33 voos |
| D | BlueROV2 (ArduSub GUIDED) em SITL | 10 voos |
| E | Missão híbrida ar → água, dois veículos em sequência (transição modelada, não simulada) | 2 voos (A\*, RRT\*) |
| Vídeos | 1 voo de cada algoritmo em 3 cenários no ar e 3 na água (postes, parede, blocos) + a missão híbrida | 14 vídeos |

Gráficos: `comparison/stage_a_metrics.png` e `comparison/sitl_metrics.png`; tabelas completas em `campaign/report.md`.

## 3. Resultados por critério

### 3.1 Tempo de planejamento (Etapa A, K1, mediana por cenário)

| Cenário | A\* (fina) | RRT\* 1º caminho | RRT\* orçamento |
|---|---|---|---|
| C1 (ar, longo) | 1585 ms (2 m) | 239 ms | 5010 ms |
| C3 (curto, boia) | 763 ms (0,5 m) | 40 ms | 5005 ms |
| C4 (transição) | 2487 ms (1 m) | 84 ms | 5006 ms |
| C5 (água) | 991 ms (1 m) | 106 ms | 5006 ms |
| C6 (missão completa) | 6298 ms (2 m) | 495 ms | 15017 ms (3 trechos) |

O tempo "inicial" do RRT\* é o orçamento fixo, não o tempo que ele precisa: o que importa é o tempo até o primeiro caminho.

### 3.2 Replanejamento (critério de tempo real: planejar o trecho ≤ percorrê-lo)

* Etapa A: nos dois, 100 % dentro do critério em desktop. A\*: mediana 173 ms, máximo 2,67 s (**12,3 %** do orçamento; pior caso em C3 com grade de 0,5 m, mediana 1,4 s). RRT\*: mediana 184 ms, máximo 1,03 s (**1,8 %**).
* Etapa C (voos): média por voo de K2, K3 e K5 (A\* / RRT\*): 43 / 14 ms, 204 / 42 ms e 32 / 15 ms. O RRT\* também replaneja menos vezes por voo (1,9 contra 5,8).
* Etapa D (água): 61 ms contra 10 ms.
* **Não medido**: o critério na Raspberry Pi e na Jetson Nano, com o YOLO rodando. Pendente de hardware.

### 3.3 Memória de pico (Etapa A)

A\*: 0,4 a 10,2 MB conforme resolução (o pior caso, 10,2 MB, é C6 a 2 m). RRT\*: 0,2 a 1,0 MB. Os cenários são pequenos perto dos 200×200×30 m do guia; **não verifiquei o comportamento na escala real**. A extrapolação (memória do A\* crescendo com o cubo) é o argumento do guia, coerente com o que vi, mas não medida.

### 3.4 Robustez

De 1152 execuções: SUCESSO em 1041, ENERGIA_INSUFICIENTE em 108 (todas em K6, resposta esperada) e TIMEOUT em 3: A\* 1 em 192 (C4, K4 20 %, grade de 1 m) e RRT\* 2 em 960 (C6, K4 20 %, seeds 5 e 22). Nenhum SEM_CAMINHO. Incerteza subaquática de 2 a 20 % (K4) aumenta a energia dos dois de modo parecido (C5 a 20 %: A\* 0,50 Wh, RRT\* 0,54 Wh).

### 3.5 Energia e caminho

Comprimento voado igual (mediana −0,1 %). Energia planejada: veja a seção 1. |ΔE| (planejada − executada, com as mesmas curvas do planejamento) vai de 0,01 Wh (C5) a 0,6 Wh conforme o cenário; o RRT\* fica um pouco acima em C2 e C6 (0,60 e 0,59 contra 0,47 e 0,44 do A\* na grade fina) e igual nos demais. Ele mede desvio de execução, não a fidelidade do modelo de consumo.

### 3.6 Execução no autopiloto

* Ar (Etapa B): A\* 4/4 SUCESSO; RRT\* 7/8 SUCESSO e 1 COLISAO (folga 0,44 m). Tempo de missão 15,8 s (A\*) e 17,1 s (RRT\*).
* Via `mission_node` (Etapa C): 32 SUCESSO e 1 recusa por energia (K6), **0 colisões**; folga mínima média 1,28 m (A\*) contra 1,48 m (RRT\*).
* Água (Etapa D): 10/10 SUCESSO, sem colisão.
* Missão híbrida (Etapa E): os dois concluíram os dois trechos sem colisão; o A\* e o RRT\* escolheram colunas de cruzamento em lados opostos da barcaça (leste −6,9 e +7,1), com energia executada total (com a transição modelada) de 1,71 e 1,40 Wh contra 1,25 e 1,27 planejados.

## 4. Onde a hipótese do guia se sustenta

* Sustenta-se para memória, latência e escala: o RRT\* é mais barato e mais rápido.
* **Não se sustenta sem ressalva para a energia nas missões híbridas**: o A\* achou planos 9 a 11 % mais baratos em C4 e C6.
* Água: a literatura favorece o A\* em cenários complexos e com incerteza. Na incerteza máxima (K4 20 %) o A\* teve 1 TIMEOUT (C4) e o RRT\* 2 (C6); no cenário subaquático puro (C5) nenhum dos dois falhou. A robustez ficou praticamente empatada e a diferença ficou na energia.

## 5. Limitações

* **Placa embarcada não medida** (entregável e critério de aceite). Todos os tempos são de desktop.
* **Coeficientes de energia e μ são placeholders**; sem a curva real (Pinheiro et al.) o desempate por energia é frágil. O BiguaSim fornece empuxo dos propulsores, mas não potência.
* Cenários pequenos e sintéticos (props), não a geometria real da ponte; a percepção é simulada por raio; K4 só existe na Etapa A (nos SITL o GPS_INPUT do ROV é a verdade do simulador).
* **Nenhum veículo do BiguaSim faz ar e água em SITL**: a transição é modelada, e a missão híbrida é a costura de dois veículos.
* SITL com poucas repetições (1 a 2 seeds do RRT\*); só a Etapa A tem estatística.
* Hiperparâmetros (passo, raio de rewiring, viés, resolução) foram fixados e não otimizados por RSM; a comparação pode mudar com eles.
* Detalhe do A\*: em descida vertical pura o modelo de energia prefere a diagonal (velocidade vertical de 1 m/s no modelo), o que produz zigue-zague em grade; é um artefato do modelo, não do veículo (`stage_e/NOTES.md`).

## 6. O que falta antes da decisão final

1. Medir na Raspberry Pi e na Jetson Nano com o YOLO ligado, e reportar o tempo como percentual do orçamento (pendente: modelo da placa e memória livre).
2. Trocar os coeficientes placeholder pelos reais e refazer a comparação de energia (é o único critério em que o A\* ganha).
3. Testar o RRT\* com mais tempo de orçamento para ver se a diferença de energia some (hipótese aberta).
4. Repetir os cenários em escala real (200×200×30 m) para verificar a memória do A\*.
5. Definir com o T11 a política de TIMEOUT repetido e confirmar com a equipe as pendências do guia (matriz de ocupação, RSM dentro da 8.2, sensores).
6. Depois da escolha: implementação final com o algoritmo escolhido; o outro fica como ferramenta de teste.

## 7. Como reproduzir

`python3 benchmark.py` (Etapa A); `stage_b/run_stage_b.py`, `stage_c/run_stage_c.py`, `stage_d/run_stage_d.py`, `stage_e/run_stage_e.py` (SITL); `make_comparison_plots.py` (gráficos); `record_videos.py` (vídeos). Detalhes no `README.md` da pasta `planning/`.
