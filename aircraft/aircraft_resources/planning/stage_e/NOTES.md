# Stage E: missão híbrida ar → água em SITL (Hydrone + BlueROV2)

Um único plano híbrido (ar, cruzamento vertical, água) é calculado pelo planejador e **executado por dois veículos em sequência**, cada um na sua pilha SITL:

| Trecho | Executa | Pilha |
|---|---|---|
| AR | Hydrone (ArduCopter, GUIDED), da origem até 2 m acima da água na coluna de cruzamento | Etapa B |
| TRANSIÇÃO (\|z\| ≤ μ) | **não voada**: modelada pelo custo de transição do planejador (0,5 m/s, energia fixa) | - |
| ÁGUA | BlueROV2 (ArduSub, GUIDED), criado na mesma coluna a 2 m de profundidade, até o alvo submerso | Etapa D |

**Por que dois veículos.** Nenhum veículo do BiguaSim faz as duas coisas em SITL hoje: o Hydrone (perfil DjiMatrice) não tem GUIDED subaquático no ArduCopter e o BlueROV2 não voa. Portanto o cruzamento da superfície **não é simulado**: o que a Etapa E prova é a costura (o plano híbrido dividido por meio, a coluna de cruzamento passada de um veículo ao outro, quadros e mapas consistentes, energia por meio) e não a física da transição.

```
python3 stage_e/run_stage_e.py --out results/stage_e --planners astar rrt_star     # plano + Hydrone + BlueROV2 + vídeo
python3 stage_e/run_stage_e.py --out results/stage_e --analyse-only                # reconstrói runs_e.csv
python3 stage_e/run_stage_e.py --out results/stage_e --rejoin-videos               # reconstrói o vídeo emendado
```

`hybrid.py` (plano híbrido, divisão por meio, cenários dos dois trechos), `e1_hybrid.yaml` (cenário), `run_stage_e.py` (orquestrador), `config/planner_hybrid_sitl.yaml` (lado aéreo do `planner_sitl.yaml` + água do ROV a 0,2 m/s). Tudo no referencial da superfície da água com origem em BiguaSim (25, 0, 0); o executivo do Hydrone converte para o referencial do home com `--frame-offset` (o planejador precisa da altura sobre a água para distinguir os meios).

**Cenário E1.** Origem sobre o deck (18,4 m acima da água) e alvo a 4 m de profundidade *sob uma barcaça* que flutua na superfície (norte 6 a 18, calado 1,2 m): a descida direta sobre o alvo está bloqueada, então o cruzamento acontece ao lado dela e o ROV nada por baixo. Há um poste conhecido e um oculto no ar e um conhecido e um oculto na água (K2: os ocultos são descobertos em voo).

## O que a primeira tentativa mostrou

1. **Estrutura da ponte fora do mapa.** No primeiro voo (A\*), a descida diagonal a partir do deck parou "morta" em BiguaSim (15, 0, 12): há geometria real da ponte abaixo do nível do deck perto de x = 15 que nenhum mapa tem. A coluna de água que os testes do T2 usaram é só x ≥ 25. O cenário agora tem um **volume proibido de planejamento** (`nogo`: norte < −0,3 m, até 16,5 m de altura) que não vira prop nem entra nos cenários dos trechos: a rota fica alta até passar da coluna e só então desce. É uma suposição minha, não uma medição da geometria; se a ponte tiver outra forma, o volume muda.
2. **A descida ótima do modelo é diagonal, não vertical.** Com a velocidade vertical de 1 m/s do modelo, descer na diagonal à velocidade de cruzeiro (2,5 m/s) custa menos energia que descer na vertical, e o A\* desenha um zigue-zague de 1 m em uma descida vertical pura. Por isso o trecho aéreo voa os waypoints do próprio plano híbrido (diagonal), terminando na coluna à altura de 2 m, e não um trecho "nível + vertical". É um artefato do modelo de energia, não do veículo.

## Resultado (2026-09-24, K2, 1 voo por algoritmo)

| | A* | RRT* |
|---|---|---|
| Coluna de cruzamento (norte, leste) | (12.32, -6.87) | (13.07, 7.11) |
| Energia planejada AR / TRANSIÇÃO / ÁGUA (Wh) | 0.955 / 0.111 / 0.184 | 0.952 / 0.111 / 0.205 |
| Energia planejada total (Wh) | 1.251 | 1.268 |
| Tempo de planejamento híbrido (ms) | 281.2 | 4002.8 |
| Trecho AR (Hydrone): status | SUCESSO | SUCESSO |
|   comprimento voado (m) / tempo (s) | 42.47 / 24.9 | 40.11 / 19.2 |
|   replanejamentos / folga mínima (m) | 5 / 1.55 | 0 / 1.71 |
|   erro final (m) / energia executada (Wh) | 0.46 / 1.3938 | 0.31 / 1.054 |
| Trecho ÁGUA (BlueROV2): status | SUCESSO | SUCESSO |
|   comprimento voado (m) / tempo (s) | 6.95 / 35.0 | 7.24 / 35.5 |
|   folga mínima (m) / erro final (m) | 1.19 / 0.5 | 1.7 / 0.47 |
|   energia executada (Wh) | 0.2084 | 0.2299 |
| Total executado + transição modelada (Wh) | 1.713 | 1.395 |

* Os dois algoritmos concluíram a missão inteira, sem colisão e com erro final abaixo de 0,5 m em cada trecho.
* A energia executada (empuxo do BiguaSim nas curvas placeholder de cada meio) ficou **acima** da planejada: 1.713 Wh (A\*) e 1.395 Wh (RRT\*) contra 1.251 e 1.268 Wh. No ar isso é o comportamento já visto na Etapa B (o plano assume trajetória ideal). Não investiguei a causa em detalhe.
* O trecho da água é curto (7 m, ~35 s): o planejador escolhe o cruzamento perto do alvo porque nadar custa mais por metro que voar. Isso testa a costura, não uma navegação subaquática longa; para isso, veja a Etapa D.

## Limites

* **Transição não simulada**, dois veículos diferentes, uma mudança de referencial entre os trechos, e só o Hydrone→ROV (o retorno água → ar não existe: o ROV não voa).
* Um cenário, K2 apenas, uma semente do RRT\*; nada estatístico.
* O volume proibido é uma suposição; a geometria real da ponte continua desconhecida.
* Energia com coeficientes placeholder.
* Vídeos emendados em `results/stage_e/videos/` (fora do git): ar com câmera alta, cartão de transição de 4 s, água com câmera de perseguição.
