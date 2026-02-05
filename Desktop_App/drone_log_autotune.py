import argparse
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Tuple

import numpy as np
import pandas as pd


REQUIRED_COLUMNS = [
    "t_s",
    "base",
    "takeoff_on",
    "roll_cal_deg",
    "pitch_cal_deg",
    "pwm_lf",
    "pwm_rf",
    "pwm_lb",
    "pwm_rb",
]


@dataclass
class TrimSuggestion:
    cg_roll_bias_pwm: int
    cg_pitch_bias_pwm: int
    diff_lr_target: float
    diff_fb_target: float
    roll_fit: Tuple[float, float]  # (intercept, slope) for roll_cal = a + b*diff_lr
    pitch_fit: Tuple[float, float] # (intercept, slope) for pitch_cal = a + b*diff_fb
    r2_roll: float
    r2_pitch: float
    n_roll: int
    n_pitch: int
    n_total: int


def robust_linreg(x: np.ndarray, y: np.ndarray, iters: int = 6, z: float = 3.0):
    x = np.asarray(x, dtype=float)
    y = np.asarray(y, dtype=float)
    mask = np.isfinite(x) & np.isfinite(y)
    x = x[mask]
    y = y[mask]

    if x.size < 12:
        return None

    keep = np.ones(x.size, dtype=bool)
    beta = None

    for _ in range(iters):
        X = np.column_stack([np.ones(keep.sum()), x[keep]])
        beta = np.linalg.lstsq(X, y[keep], rcond=None)[0]
        resid = y - (beta[0] + beta[1] * x)

        r = resid[keep]
        mad = np.median(np.abs(r - np.median(r))) + 1e-9
        sigma = 1.4826 * mad
        new_keep = np.abs(resid) < z * sigma

        if new_keep.sum() == keep.sum():
            keep = new_keep
            break
        keep = new_keep

    yhat = beta[0] + beta[1] * x
    ss_res = np.sum((y[keep] - yhat[keep]) ** 2)
    ss_tot = np.sum((y[keep] - np.mean(y[keep])) ** 2)
    r2 = 1.0 - ss_res / ss_tot if ss_tot > 0 else float("nan")

    return beta, float(r2), int(keep.sum()), int(x.size)


def load_log(csv_path: Path) -> pd.DataFrame:
    df = pd.read_csv(csv_path)
    missing = [c for c in REQUIRED_COLUMNS if c not in df.columns]
    if missing:
        raise ValueError(f"Brak wymaganych kolumn: {missing}. Dostępne: {list(df.columns)}")

    df = df.copy()
    df["t_s"] = pd.to_numeric(df["t_s"], errors="coerce")
    df = df.dropna(subset=["t_s"]).sort_values("t_s").reset_index(drop=True)
    return df


def pick_hover_segment(df: pd.DataFrame) -> pd.DataFrame:
    """
    Heurystyka: bierzemy tylko fragmenty, gdzie:
    - takeoff_on == 1
    - base > 0
    - base w zakresie 60-90 percentyla (czyli okolice "hover / normalnej pracy")
    - bez skrajnych przechyłów (trim i tak ma centrować okolice 0)
    """
    d = df[(df["takeoff_on"] == 1) & (df["base"] > 0)].copy()
    if d.empty:
        return d

    b = d["base"].astype(float).values
    p60 = np.percentile(b, 60)
    p90 = np.percentile(b, 90)
    seg = d[(d["base"] >= p60) & (d["base"] <= p90)].copy()

    # Odetnij ewidentne outliery (np. po wywrotce) po samych kątach:
    seg = seg[(seg["roll_cal_deg"].abs() <= 25.0) & (seg["pitch_cal_deg"].abs() <= 25.0)]
    return seg


def compute_trim(df_hover: pd.DataFrame) -> Optional[TrimSuggestion]:
    if df_hover is None or df_hover.empty or len(df_hover) < 25:
        return None

    d = df_hover.copy()
    d["diff_lr"] = (d["pwm_lf"] + d["pwm_lb"]) - (d["pwm_rf"] + d["pwm_rb"])
    d["diff_fb"] = (d["pwm_lb"] + d["pwm_rb"]) - (d["pwm_lf"] + d["pwm_rf"])  # rear - front

    roll_fit = robust_linreg(d["diff_lr"].values, d["roll_cal_deg"].values)
    pitch_fit = robust_linreg(d["diff_fb"].values, d["pitch_cal_deg"].values)

    if roll_fit is None or pitch_fit is None:
        return None

    (a_r, b_r), r2r, nr, nt = roll_fit
    (a_p, b_p), r2p, np_, nt2 = pitch_fit

    # Docelowo chcemy roll_cal_deg ~= 0 i pitch_cal_deg ~= 0 w "hover"
    # roll: 0 = a_r + b_r*diff_lr  => diff_lr_target = -a_r/b_r
    # pitch: 0 = a_p + b_p*diff_fb => diff_fb_target = -a_p/b_p
    eps = 1e-6
    if abs(b_r) < eps or abs(b_p) < eps:
        return None

    diff_lr_target = float(-a_r / b_r)
    diff_fb_target = float(-a_p / b_p)

    # Jak to mapuje się na Twoje definicje?
    #
    # W mixerze:
    #   roll_bias dodaje +roll_bias na LF,LB oraz -roll_bias na RF,RB
    #   => diff_lr = (LF+LB) - (RF+RB) zmienia się o +4*roll_bias
    #   => roll_bias_pwm ~= diff_lr_target / 4
    #
    #   pitch_bias dodaje +pitch_bias na LF,RF oraz -pitch_bias na LB,RB
    #   => diff_fb = (LB+RB) - (LF+RF) zmienia się o -4*pitch_bias
    #   => pitch_bias_pwm ~= -diff_fb_target / 4
    #
    cg_roll_bias = int(np.rint(diff_lr_target / 4.0))
    cg_pitch_bias = int(np.rint(-diff_fb_target / 4.0))

    return TrimSuggestion(
        cg_roll_bias_pwm=cg_roll_bias,
        cg_pitch_bias_pwm=cg_pitch_bias,
        diff_lr_target=diff_lr_target,
        diff_fb_target=diff_fb_target,
        roll_fit=(float(a_r), float(b_r)),
        pitch_fit=(float(a_p), float(b_p)),
        r2_roll=float(r2r),
        r2_pitch=float(r2p),
        n_roll=int(nr),
        n_pitch=int(np_),
        n_total=int(len(d)),
    )


def format_c_defines(s: TrimSuggestion) -> str:
    return "\n".join([
        f"#define CG_ROLL_BIAS_PWM    {s.cg_roll_bias_pwm}",
        f"#define CG_PITCH_BIAS_PWM   {s.cg_pitch_bias_pwm}",
        "",
        "// (INFO) targets from log-fit:",
        f"// diff_lr_target (LF+LB - RF-RB) ≈ {s.diff_lr_target:.1f}",
        f"// diff_fb_target (LB+RB - LF-RF) ≈ {s.diff_fb_target:.1f}",
        f"// roll fit:  roll_cal_deg  ≈ {s.roll_fit[0]:.3f} + ({s.roll_fit[1]:.6f}) * diff_lr   (R²={s.r2_roll:.2f}, n={s.n_roll}/{s.n_total})",
        f"// pitch fit: pitch_cal_deg ≈ {s.pitch_fit[0]:.3f} + ({s.pitch_fit[1]:.6f}) * diff_fb   (R²={s.r2_pitch:.2f}, n={s.n_pitch}/{s.n_total})",
    ])


def main():
    ap = argparse.ArgumentParser(
        description="Autotune/Trim z logów drona: estymuje CG_ROLL_BIAS_PWM i CG_PITCH_BIAS_PWM z CSV."
    )
    ap.add_argument("csv", type=str, help="Ścieżka do logu CSV (format jak w Twoich logach).")
    ap.add_argument("--out", type=str, default="", help="Zapisz wynik do pliku tekstowego (opcjonalnie).")
    ap.add_argument("--json", type=str, default="", help="Zapisz wynik do pliku JSON (opcjonalnie).")
    args = ap.parse_args()

    csv_path = Path(args.csv)
    df = load_log(csv_path)
    hover = pick_hover_segment(df)

    if hover.empty or len(hover) < 25:
        print("Nie znalazłem sensownego segmentu 'hover' (takeoff_on==1, base>0).")
        print("Spróbuj log z dłuższym lotem/hoverem albo upewnij się, że pola takeoff_on/base są poprawne.")
        return

    s = compute_trim(hover)
    if s is None:
        print("Nie udało się policzyć trimów (za mało danych albo brak korelacji).")
        return

    report = []
    report.append(f"Plik: {csv_path}")
    report.append(f"Próbki w hover-segmencie: {len(hover)}")
    report.append("")
    report.append("Sugerowane definicje do USER CODE PD:")
    report.append(format_c_defines(s))
    report.append("")
    report.append("Ważne:")
    report.append("- To jest *trim CG* (stały bias), żeby stabilizacja nie musiała cały czas 'walczyć' integratorem.")
    report.append("- Jeśli log nie ma manewrów (NAV_LEFT/RIGHT/FWD/BACK), to nie da się wiarygodnie policzyć KP/KD/KI z samego 'stania' —")
    report.append("  wtedy ten skrypt daje najpewniejszą rzecz, czyli biasy CG.")
    report.append("- Testuj w bezpiecznych warunkach (na uwięzi / z osłonami śmigieł) i zmieniaj wartości małymi krokami.")

    txt = "\n".join(report)
    print(txt)

    if args.out:
        Path(args.out).write_text(txt, encoding="utf-8")

    if args.json:
        payload = {
            "file": str(csv_path),
            "hover_samples": int(len(hover)),
            "cg_roll_bias_pwm": int(s.cg_roll_bias_pwm),
            "cg_pitch_bias_pwm": int(s.cg_pitch_bias_pwm),
            "diff_lr_target": float(s.diff_lr_target),
            "diff_fb_target": float(s.diff_fb_target),
            "roll_fit": {"intercept": float(s.roll_fit[0]), "slope": float(s.roll_fit[1]), "r2": float(s.r2_roll), "n_used": int(s.n_roll), "n_total": int(s.n_total)},
            "pitch_fit": {"intercept": float(s.pitch_fit[0]), "slope": float(s.pitch_fit[1]), "r2": float(s.r2_pitch), "n_used": int(s.n_pitch), "n_total": int(s.n_total)},
        }
        Path(args.json).write_text(json.dumps(payload, indent=2), encoding="utf-8")


if __name__ == "__main__":
    main()
