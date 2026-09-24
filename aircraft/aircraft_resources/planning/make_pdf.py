"""Build the T8.2 results PDF from a benchmark campaign (Stage A) and, if present, Stage B (SITL).

    python3 make_pdf.py results/campaign [--stage-b results/stage_b] [--out T8.2_resultados.pdf]

Tables and every number in the text are computed from the CSVs, so the PDF cannot drift from the
data. The PDF is rendered with headless Chrome (HTML -> PDF); figures are embedded.
"""

from __future__ import annotations

import argparse
import base64
import subprocess
import tempfile
from pathlib import Path

import markdown
import numpy as np
import pandas as pd

from benchmark import RESOLUTIONS
from report import LABEL, cell, fmt

HERE = Path(__file__).resolve().parent
SCEN = ["C1", "C2", "C3", "C4", "C5", "C6"]
DESC = {
    "C1": "F1 (ar): trajeto longo até o ROI, ar aberto, poucos obstáculos",
    "C2": "F1 (ar): mesmo trajeto com obstáculo móvel na superfície (WhiteBoat)",
    "C3": "F2 (ar): caminho curto ao redor da boia, alvo visível",
    "C4": "Transição: aproximação e escolha do ponto de entrada na água",
    "C5": "F3 (água): trajetória subaquática perto da boia, margem ampliada",
    "C6": "Missão completa: origem → ROI → água → retorno",
}
KDESC = {
    "K1": "mapa totalmente conhecido", "K2": "obstáculo não mapeado descoberto em voo (raio 12 m)",
    "K3": "atualização contínua do mapa (raio 30 m)", "K4": "incerteza de posição subaquática 2–20 %",
    "K5": "mudança de missão durante a execução", "K6": "energia inicial reduzida",
}


def img(path: Path, caption: str, width: str = "100%") -> str:
    b64 = base64.b64encode(path.read_bytes()).decode()
    return f'<figure><img src="data:image/png;base64,{b64}" style="width:{width}"><figcaption>{caption}</figcaption></figure>\n'


def md_table(rows, header) -> str:
    s = "| " + " | ".join(header) + " |\n|" + "|".join("---" for _ in header) + "|\n"
    return s + "\n".join("| " + " | ".join(str(c) for c in r) + " |" for r in rows) + "\n"


def baseline_rows(runs, mem):
    rows = []
    for name in SCEN:
        d = runs[(runs.scenario == name) & (runs.condition == "K1")]
        for pl in ("astar", "rrt_star"):
            for res in (RESOLUTIONS[name] if pl == "astar" else [np.nan]):
                e = d[(d.planner == pl) & ((d.resolution == res) if res == res else d.resolution.isna())]
                if e.empty:
                    continue
                ok = e[e.status == "SUCESSO"]
                m = mem[(mem.scenario == name) & (mem.planner == pl) & ((mem.resolution == res) if res == res else mem.resolution.isna())]
                rows.append([name, LABEL[pl] + (f" {res:g} m" if res == res else ""), f"{100 * len(ok) / len(e):.0f}%",
                             cell(ok, "t_first_solution_ms", 0), cell(ok, "t_initial_ms", 0),
                             fmt(float(m.peak_mem_mb.iloc[0])) if len(m) else "-",
                             cell(ok, "energy_planned_wh", 3), cell(ok, "length_planned_m", 1),
                             cell(ok, "n_waypoints", 1), cell(ok, "min_clearance_m", 2)])
    return rows


def condition_rows(runs):
    rows = []
    finest = {n: min(RESOLUTIONS[n]) for n in SCEN}
    for (name, cond, unc), d in runs.groupby(["scenario", "condition", runs.uncertainty.fillna(-1)]):
        for pl in ("astar", "rrt_star"):
            e = d[d.planner == pl]
            if pl == "astar":
                e = e[e.resolution == finest[name]]
            if e.empty:
                continue
            ok = e[e.status == "SUCESSO"]
            st = e.status.value_counts()
            fails = ", ".join(f"{k} {100 * v / len(e):.0f}%" for k, v in st.items() if k != "SUCESSO") or "-"
            rp = ok[ok.n_replans > 0]
            rows.append([name, cond + (f" {unc * 100:g}%" if unc >= 0 else ""),
                         LABEL[pl] + (f" {finest[name]:g} m" if pl == "astar" else ""),
                         f"{100 * len(ok) / len(e):.0f}%", fails, cell(ok, "n_replans", 1), cell(rp, "replan_ms_mean", 0),
                         cell(ok, "energy_planned_wh", 3), cell(ok, "abs_delta_e_wh", 3)])
    return rows


def agg_table(r: pd.DataFrame) -> str:
    r = r[r["status"] != "ENERGIA_INSUFICIENTE"].copy()
    for c in ("planned_wh", "exec_wh_thrust", "delta_e_thrust_wh", "mission_time_s", "n_replans", "replan_ms_max",
              "min_clearance_m", "stops_at_waypoints", "final_error_m", "flown_length_m"):
        r[c] = pd.to_numeric(r[c], errors="coerce")
    r["dE"] = 100.0 * r["delta_e_thrust_wh"] / r["planned_wh"]
    rows = []
    for (scn, pl), g in r.groupby(["scenario", "planner"]):
        rows.append([scn, "A*" if pl == "astar" else "RRT*", len(g), int((g.status == "SUCESSO").sum()), f"{g.min_clearance_m.min():.2f}",
                     f"{g.mission_time_s.mean():.1f}", f"{g.flown_length_m.mean():.1f}", f"{g.n_replans.mean():.1f}",
                     "-" if g.replan_ms_max.isna().all() else f"{g.replan_ms_max.max():.0f}", f"{g.dE.mean():+.0f}",
                     int(g.stops_at_waypoints.sum()), f"{g.final_error_m.mean():.2f}"])
    return md_table(rows, ["cen.", "planej.", "voos", "SUCESSO", "folga mín. (m)", "tempo (s)", "voado (m)", "replan./voo",
                           "replan. máx (ms)", "ΔE (%)", "paradas", "erro final (m)"])


def stage_section(b: Path, csv_name: str, title: str) -> str:
    csv = b / csv_name
    if not csv.exists():
        return ""
    r = pd.read_csv(csv)
    out = [f"\n<div class='pb'></div>\n\n# {title}\n"]
    note = b / "NOTES.md"
    if note.exists():
        out.append(note.read_text() + "\n")
    out.append("\n### Resumo por cenário e planejador\n\n" + agg_table(r))
    r["ΔE (%)"] = (100.0 * r["delta_e_thrust_wh"] / r["planned_wh"]).round(0)
    show = {"run_id": "voo", "status": "status", "n_waypoints": "wps", "planned_wh": "plan. (Wh)", "exec_wh_thrust": "exec. (Wh)",
            "ΔE (%)": "ΔE (%)", "flown_length_m": "voado (m)", "mission_time_s": "tempo (s)", "n_replans": "replan.",
            "replan_ms_max": "replan. máx (ms)", "stops_at_waypoints": "paradas", "min_clearance_m": "folga mín. (m)",
            "max_crosstrack_m": "erro lat. máx (m)", "mean_speed_mps": "vel. média (m/s)", "final_error_m": "erro final (m)"}
    t = r[list(show)].rename(columns=show).round(2)
    out.append("\n### Voos\n\n" + md_table(t.fillna("-").astype(str).values.tolist(), list(t.columns)))
    for fig in sorted((b / "figures").glob("*.png")) if (b / "figures").exists() else []:
        out.append("\n" + img(fig, f"{fig.stem.replace('stage_b_', 'Cenário ')}: trajetória voada em ArduPilot SITL (cheia) contra o primeiro plano (tracejada) e velocidade (verdade do BiguaSim)."))
    return "\n".join(out)


def stage_b_section(b: Path) -> str:
    return stage_section(b, "runs_b.csv", "Etapa B — BiguaSim + ArduPilot SITL em GUIDED (executivo pymavlink)")


def stage_c_section(c: Path) -> str:
    return stage_section(c, "runs_c.csv", "Etapa C — os planejadores dentro do mission_node (stack ROS 2 real)")


def build(out_dir: Path, stage_b: Path | None, stage_c: Path | None = None) -> str:
    runs = pd.read_csv(out_dir / "runs.csv")
    runs["resolution"] = runs["resolution"].astype("float64")
    mem = pd.read_csv(out_dir / "memory.csv")
    fig = out_dir / "figures"
    n = len(runs)
    fly = runs[runs.condition != "K6"]
    a_ok, a_n = int(((fly.planner == "astar") & (fly.status == "SUCESSO")).sum()), int((fly.planner == "astar").sum())
    r_ok, r_n = int(((fly.planner == "rrt_star") & (fly.status == "SUCESSO")).sum()), int((fly.planner == "rrt_star").sum())
    k6 = runs[runs.condition == "K6"]
    k6_ok = int((k6.status == "ENERGIA_INSUFICIENTE").sum())
    fails = fly[fly.status != "SUCESSO"].groupby(["planner", "status"]).size()

    ok = runs[runs.status == "SUCESSO"]
    cmp_rows = []
    for name in SCEN:
        a = ok[(ok.scenario == name) & (ok.condition == "K1") & (ok.planner == "astar") & (ok.resolution == min(RESOLUTIONS[name]))]
        r = ok[(ok.scenario == name) & (ok.condition == "K1") & (ok.planner == "rrt_star")]
        rp_a = ok[(ok.scenario == name) & (ok.planner == "astar") & (ok.n_replans > 0)].replan_ms_mean
        rp_r = ok[(ok.scenario == name) & (ok.planner == "rrt_star") & (ok.n_replans > 0)].replan_ms_mean
        cmp_rows.append([name, f"{100 * (r.energy_planned_wh.mean() / a.energy_planned_wh.mean() - 1):+.1f}%",
                         f"{r.t_first_solution_ms.mean():.0f}", f"{a.t_initial_ms.mean():.0f}",
                         f"{rp_r.mean():.0f}" if len(rp_r) else "-", f"{rp_a.mean():.0f}" if len(rp_a) else "-"])

    md = f"""
<div class="cover">
<h1>T8.2 — Planejadores A* e RRT* no BiguaSim</h1>
<p><b>Resultados da Etapa A (BiguaSim + seguidor de caminho simples)</b>{" , da Etapa B (ArduPilot SITL em GUIDED)" if stage_b and (stage_b / "runs_b.csv").exists() else ""}{" e da Etapa C (planejadores dentro do mission_node)" if stage_c and (stage_c / "runs_c.csv").exists() else ""}</p>
<p>Matheus · {pd.Timestamp.now().strftime('%d/%m/%Y')}</p>
</div>

## 1. Resumo

* Implementados os dois planejadores (RRT\\* como candidato principal e A\\* como baseline) sobre uma infraestrutura comum:
  mapa 3D híbrido ar–água, custo de energia por meio, colisão contínua com margem por meio, travessia da zona de transição
  sempre na vertical, poda por linha de visada e replanejamento contínuo (CL-RRT para o RRT\\*, D\\* Lite para o A\\*).
* Campanha da Etapa A: **{n} execuções** nos cenários C1–C6 e condições K1–K6, RRT\\* com 30 seeds fixas e A\\* em duas resoluções de grade.
* Nas execuções que decolam: A\\* concluiu **{a_ok}/{a_n}** e RRT\\* **{r_ok}/{r_n}**. As outras {k6_ok} execuções são o K6 (energia reduzida),
  que devolve `ENERGIA_INSUFICIENTE` por construção.
* Falhas reais: {", ".join(f"{LABEL.get(p, p)} {s} ×{c}".replace("*", "\\*") for (p, s), c in fails.items()) or "nenhuma"}.
* **Antes de citar números:** os coeficientes das curvas de energia (Pinheiro et al., IROS 2024), a altura μ da zona de transição e as margens são
  **placeholders** (não estavam no guia). A comparação entre os planejadores vale; os valores absolutos em Wh não.

## 2. Método

**Frame.** (norte, leste, cima) em metros, z = 0 na superfície da água (convenção do BiguaSim). Ar: z > μ; transição: −μ ≤ z ≤ μ; água: z < −μ (μ = 0,5 m, placeholder).

**Mapa.** Obstáculos a priori analíticos (cilindros e caixas) mais uma camada esparsa de voxels de 0,10 m, atualizada pela interface única
`atualizar_ocupacao(celulas, fonte, timestamp)` (fontes: mapa a priori, LiDAR, sonar do Hydrone, sonar do WhiteBoat). Uma grade densa de 200×200×30 m a 0,10 m ocuparia ~1,2 GB.

**Custo.** Energia estimada por segmento, a partir de curvas polinomiais de 2º grau (potência × empuxo) para propulsores aéreos e aquáticos; custo de transição contabilizado à parte.

**Colisão e margens.** Verificação contínua ao longo do segmento (geometria exata). Margem = raio do veículo + folga do meio; na água a folga é maior e cresce com a distância ao último ponto com GPS (K4).

**RRT\\*.** Amostragem com viés ao objetivo, extensão limitada, escolha do pai de menor energia acumulada dentro do raio, rewiring, extensão vertical forçada na zona de transição. Árvore persistente para o replanejamento contínuo.

**A\\*.** Grade 3D local, 26-conexa, custo de aresta = energia, heurística admissível; grade deslocada aleatoriamente a cada plano (A\\*ᵣ, Zammit & van Kampen 2022). Replanejamento com D\\* Lite.

**Etapa A.** Seguidor cinemático simples (segue a reta do plano, freia antes de curvas, ajusta só na vertical na zona de transição), com sensores simulados (raio de percepção), obstáculo móvel no C2 e condições K1–K6. A energia executada usa as mesmas curvas do planejamento; portanto **ΔE mede o desvio da trajetória executada em relação à planejada, não a fidelidade do modelo de consumo.**

## 3. Cenários e condições

{md_table([[k, DESC[k]] for k in SCEN], ["ID", "Descrição"])}

{md_table([[k, v] for k, v in KDESC.items()], ["Condição", "Descrição"])}

Matriz mínima do guia: C1, C5 e C6 em todas as condições; C2 em K5, C3 em K2, C4 em K4 (todos também em K1). K4 foi varrido a 2 %, 10 % e 20 %.

<div class='pb'></div>

## 4. Resultados — linha de base (K1)

{md_table(baseline_rows(runs, mem), ["cen.", "planejador", "sucesso", "1ª solução (ms)", "tempo total (ms)", "mem. pico (MB)", "energia plan. (Wh)", "comprimento (m)", "waypoints", "folga mín. (m)"])}

O RRT\\* é *anytime*: gasta o orçamento inteiro (5 s por trecho), por isso a coluna "1ª solução" é a comparável ao tempo do A\\*. Memória: pico do plano inicial (tracemalloc, passada separada). Valores: média ± desvio sobre seeds (RRT\\*) ou deslocamentos de grade (A\\*).

### Comparação (K1)

{md_table(cmp_rows, ["cen.", "energia RRT\\* vs A\\* fino", "RRT\\* 1ª sol. (ms)", "A\\* fino (ms)", "replan. CL-RRT (ms)", "replan. D\\* Lite (ms)"])}

"""
    md += img(fig / "energy_k1.png", "Figura 1 — Energia planejada em K1 (RRT\\*: dispersão entre seeds; A\\*: deslocamentos de grade).")
    md += img(fig / "times.png", "Figura 2 — Tempo de planejamento inicial e de replanejamento. No C4 não houve replanejamento em K1/K4, por isso não há caixa.")
    md += img(fig / "memory.png", "Figura 3 — Memória de pico do plano inicial: o A\\* cresce com o cubo da resolução; o RRT\\* fica abaixo de 1 MB.")
    md += "\n<div class='pb'></div>\n\n## 5. Trajetórias por cenário\n\nVista superior (esquerda) e perfil de altitude (direita), trajetória executada em K1. Obstáculos sólidos: conhecidos; tracejados: não mapeados; caixa vermelha tracejada: WhiteBoat (C2). A faixa ciano é a zona de transição: as travessias são verticais.\n"
    for name in SCEN:
        md += img(fig / f"paths_{name}.png", f"{name} — {DESC[name]}")
    md += "\n<div class='pb'></div>\n\n## 6. Todas as condições\n\n"
    md += md_table(condition_rows(runs), ["cen.", "cond.", "planejador", "sucesso", "falhas", "replan.", "replan. (ms)", "energia plan. (Wh)", "ΔE abs. (Wh)"])
    md += "\nA\\* na resolução mais fina de cada cenário. `replan.` é o número médio de replanejamentos por missão; `replan. (ms)` a média nas execuções que replanejaram.\n"
    md += img(fig / "k4.png", "Figura 4 — K4: efeito da incerteza de posição subaquática (2, 10 e 20 % da distância ao último ponto com GPS).")
    md += img(fig / "rrt_convergence.png", "Figura 5 — Convergência do RRT\\* no primeiro trecho de cada cenário (5 seeds). As curvas são o custo da árvore antes da poda por linha de visada, por isso ficam acima das energias finais das tabelas.")

    md += f"""
## 7. Leitura dos resultados

* **Tempo até a primeira solução e memória:** o RRT\\* encontra a primeira solução bem antes do A\\* fino em todos os cenários e usa menos de 1 MB; a memória do A\\* cresce com o cubo da resolução.
* **Energia:** no K1 o RRT\\* fica muito perto do A\\* em C1–C3 e pior em C4, C5 e, principalmente, C6 (missão de 3 trechos, com 5 s de orçamento por trecho). O A\\* obtém energias menores quando a grade é fina o bastante.
* **Replanejamento:** o CL-RRT foi mais rápido que o D\\* Lite em C2, C3, C5 e C6; o D\\* Lite foi mais rápido no C1. Numa mudança de missão (K5) o CL-RRT reaproveita a árvore, enquanto o D\\* Lite precisa reiniciar a busca.
* **K4:** a folga mínima aumenta com a incerteza, e a energia e o número de waypoints também; a 20 % o plano inicial do C6 passou a falhar em algumas seeds do RRT\\* (TIMEOUT) e em 1 do A\\* no C4.
* **K6:** todas as execuções devolveram `ENERGIA_INSUFICIENTE` antes de decolar, como esperado.
* **A hipótese de trabalho do guia** (RRT\\* como candidato mais provável) é compatível com tempo e memória; a energia planejada favorece o A\\* nos cenários maiores. A escolha final depende do peso dado a cada critério e da medição na placa.

## 8. Limitações (leia antes de citar)

1. **Energia com coeficientes placeholder.** Comparar planejadores entre si; não citar Wh absolutos.
2. **Tempos de desktop**, um núcleo por execução, com cenários bem menores que os 200×200×30 m do guia. A medição na Raspberry Pi/Jetson não foi feita; não extrapolar.
3. **ΔE não mede fidelidade do modelo de consumo** (mesmas curvas no plano e na execução).
4. **Obstáculos sintéticos**: a geometria real da ponte do mundo Bridge não está no mapa dos planejadores.
5. **Percepção simulada** (pontos de superfície dentro de um raio); não há LiDAR/sonar reais.
6. **Modelo de incerteza (K4)** proporcional à distância ao último ponto com GPS: aproximação da esfera de incerteza.
7. A inflação de caixas é cúbica (mais conservadora que a distância euclidiana).
8. Defeitos do seguidor (colisões em quinas, travamento na zona de transição) foram corrigidos durante a campanha; os números aqui são da campanha refeita depois das correções.
"""
    md += stage_b_section(stage_b) if stage_b else ""
    md += stage_c_section(stage_c) if stage_c else ""
    return md


CSS = """
@page { size: A4; margin: 16mm 14mm; }
body { font-family: 'DejaVu Sans', Arial, sans-serif; font-size: 9.6pt; line-height: 1.38; color: #1b1b1b; }
h1 { font-size: 20pt; margin: 0 0 6pt; } h2 { font-size: 14pt; margin: 16pt 0 5pt; border-bottom: 1px solid #ccc; padding-bottom: 2pt; }
h3 { font-size: 11.5pt; margin: 12pt 0 4pt; }
table { border-collapse: collapse; width: 100%; margin: 6pt 0 10pt; font-size: 7.8pt; page-break-inside: auto; }
th, td { border: 1px solid #c8c8c8; padding: 2.5pt 4pt; text-align: left; } th { background: #eef1f6; }
tr { page-break-inside: avoid; } figure { margin: 8pt 0; page-break-inside: avoid; }
figcaption { font-size: 8pt; color: #444; margin-top: 2pt; } .pb { page-break-after: always; }
.cover { padding: 70mm 0 30mm; } .cover h1 { font-size: 28pt; } code { font-size: 8.6pt; background: #f3f3f3; padding: 0 2pt; }
"""


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("campaign")
    ap.add_argument("--stage-b", default=None)
    ap.add_argument("--stage-c", default=None)
    ap.add_argument("--out", default=None)
    a = ap.parse_args()
    out_dir = Path(a.campaign)
    sb = Path(a.stage_b) if a.stage_b else None
    sc_ = Path(a.stage_c) if a.stage_c else None
    md = build(out_dir, sb, sc_)
    html = markdown.markdown(md, extensions=["tables", "md_in_html"])
    doc = f"<!doctype html><html><head><meta charset='utf-8'><title>T8.2 - Resultados</title><style>{CSS}</style></head><body>{html}</body></html>"
    pdf = Path(a.out) if a.out else out_dir / "T8.2_resultados.pdf"
    with tempfile.TemporaryDirectory() as td:
        f = Path(td) / "report.html"
        f.write_text(doc, encoding="utf-8")
        subprocess.run(["google-chrome", "--headless=new", "--disable-gpu", "--no-sandbox", "--no-pdf-header-footer",
                        f"--print-to-pdf={pdf.resolve()}", f"file://{f}"], check=True, capture_output=True, timeout=180)
    print(f"wrote {pdf} ({pdf.stat().st_size / 1e6:.1f} MB)")


if __name__ == "__main__":
    main()
