import yaml
from pathlib import Path

rows = []
for key in ["05m","1m","2m"]:
    p = Path(f"results/stats_{key}.yaml")
    d = yaml.safe_load(p.read_text())
    ap = d.get("analysis_parameters", {})
    td = ap.get("true_distance_m", None)

    # common fields (your YAML already has these kinds of fields)
    mean = d.get("mean_range_m", d.get("mean_m", d.get("mean", None)))
    std  = d.get("std_range_m", d.get("sigma_hit_m", d.get("std_m", d.get("std", None))))
    bias = d.get("bias_m", d.get("bias", None))
    n    = d.get("valid_measurements", d.get("n_valid", d.get("N", None)))
    out  = d.get("outliers", d.get("n_outliers", None))
    rate = d.get("outlier_rate", None)

    rows.append([key, td, mean, std, bias, n, out, rate])

md = []
md.append("| Bag | True dist (m) | Mean (m) | σ_hit / Std (m) | Bias (m) | N valid | Outliers | Outlier rate |")
md.append("|---|---:|---:|---:|---:|---:|---:|---:|")
for r in rows:
    md.append("| {} | {} | {} | {} | {} | {} | {} | {} |".format(*[("" if v is None else v) for v in r]))

Path("results/summary_table.md").write_text("\n".join(md) + "\n")
print("Wrote results/summary_table.md")
