"""Rotor design calculations (cage induction motor) - callable module.

This module is a refactor of `new rotor last version.py`:
- No interactive `input()` calls
- No terminal menu
- Returns results + a formatted text report suitable for GUI display

Units:
- Lengths are in mm unless explicitly noted.
- Flux in Wb, inductions in Tesla.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple, Any


# ============================================================
# GEOMETRY CONSTRAINTS (same intent as original script)
# ============================================================
EPS_R = 0.01   # mm : impose r1 > r2 + EPS_R
GAP_D = 0.10   # mm : impose d_rb >= r1 + r2 + GAP_D
D_MAX_MARGIN = 0.20  # mm : safety margin for d_rb

MAX_R2_R1_RATIO = 0.85  # r2 ≤ 0.85*r1
MIN_R2_R1_RATIO = 0.30  # r2 ≥ 0.30*r1


# Ratios L/D for different machine types (kept for completeness; GUI uses stator Ls)
L_D_RATIOS = {
    "haute_vitesse": {"min": 0.6, "max": 0.9, "typique": 0.75},
    "standard": {"min": 0.8, "max": 1.2, "typique": 1.0},
    "basse_vitesse": {"min": 1.2, "max": 1.5, "typique": 1.35},
}


# ============================================================
# MATERIAL RESISTIVITY TABLE (Ω·m)
# ============================================================
RESISTIVITY_TABLE: Dict[str, float] = {
    "Cuivre électrolytique": 1.68e-8,
    "Aluminium pur": 2.82e-8,
    "Aluminium moulé (Al-Si)": 3.4e-8,
    "Laiton (Cu-Zn)": 7.0e-8,
    "Bronze (Cu-Sn)": 8.0e-8,
}

CONDUCTIVITY_LEVEL: Dict[str, str] = {
    "Cuivre électrolytique": "Très élevée",
    "Aluminium pur": "Élevée",
    "Aluminium moulé (Al-Si)": "Moyenne",
    "Laiton (Cu-Zn)": "Faible",
    "Bronze (Cu-Sn)": "Faible",
}


# ============================================================
# LIGHTWEIGHT REPORTER (to feed GUI text widgets)
# ============================================================

class TextReporter:
    def __init__(self, width: int = 78):
        self.width = width
        self._lines: List[str] = []

    def banner(self, title: str):
        self._lines.append("\n" + "=" * self.width)
        self._lines.append(title.center(self.width))
        self._lines.append("=" * self.width)

    def section(self, title: str):
        self._lines.append("\n" + "-" * self.width)
        self._lines.append(title)
        self._lines.append("-" * self.width)

    def line(self, text: str = ""):
        self._lines.append(text)

    def show_params(self, params: Dict[str, Any], title: str = "Paramètres"):
        self.section(title)
        for k, v in params.items():
            self._lines.append(f"  - {k}: {v}")

    def show_calc(self, step: str, formula: str, substitution: str, result: str):
        self._lines.append(f"\n{step}")
        self._lines.append(f"  Formule      : {formula}")
        self._lines.append(f"  Substitution : {substitution}")
        self._lines.append(f"  Résultat     : {result}")

    def text(self) -> str:
        return "\n".join(self._lines).strip() + "\n"


# ============================================================
# INPUTS / CONFIG
# ============================================================

@dataclass(frozen=True)
class RotorStatorInputs:
    """Category-1 inputs obtained from stator design."""

    P_pairs: int              # pole pairs
    Ss: int                   # stator slots (called Ns in original rotor script)
    D_mm: float               # stator inner diameter (bore) in mm
    L_mm: float               # machine active length in mm
    Ls_mm: float              # stack length in mm (for bar resistance)
    Li_mm: float              # net iron length in mm (effective magnetic length, used for Wrt/Wry)
    ki: float                 # iron stacking factor (taken from stator calculation)
    Iph_A: float              # phase current
    Nc: int                   # conductors per slot (called Zs in rotor script)
    flux_per_pole_Wb: float   # flux per pole
    B_tooth_T: float          # tooth flux density
    B_yoke_T: float           # yoke/core flux density
    power_kw: float           # rated power
    Kws: float = 0.955        # stator winding factor (used in rotor current transfer)


@dataclass(frozen=True)
class RotorUserConfig:
    """Category-2 inputs configured in the Rotor tab."""

    Nr: int
    nr_method: str  # 'classique' | 'harmonique' | 'custom'

    tol_percent: float
    skew_angle_deg: float

    bar_material: str  # name from RESISTIVITY_TABLE or 'Custom'
    bar_rho_ohm_m: Optional[float] = None

    ring_je_mode: str = 'suggested'  # 'suggested' | 'custom'
    ring_je_custom: Optional[float] = None

    ring_dim_mode: str = 'typical'  # 'typical' | 'manual' | 'auto'
    ring_h_er_mm: Optional[float] = None
    ring_b_er_mm: Optional[float] = None

    ring_material: str = 'Cuivre électrolytique'
    ring_rho_ohm_m: Optional[float] = None


# ============================================================
# Nr recommendation helpers (kept compatible with original)
# ============================================================

def recommend_nr_classique(Ss: int, P_pairs: int) -> List[Tuple[int, float, int]]:
    """Return (Nr, Ss/Nr, Ss-Nr) recommendations."""
    Ns = int(Ss)
    P = int(P_pairs)
    poles = 2 * P
    conditions_a_eviter = {0, P, -P, 2 * P, -2 * P}

    if poles == 2:
        ratios_possibles = [0.7, 0.75, 0.8, 0.85, 0.9, 1.1, 1.15, 1.2, 1.25, 1.3]
    elif poles == 4:
        ratios_possibles = [3 / 4, 4 / 3]
    elif poles == 6:
        ratios_possibles = [3 / 4, 4 / 5]
    else:
        ratios_possibles = [4 / 5, 5 / 6]

    recs: List[Tuple[int, float, int]] = []
    for ratio in ratios_possibles:
        Nr_cand = int(round(Ns / float(ratio)))
        if Nr_cand <= 0:
            continue
        diff = Ns - Nr_cand
        ratio_actuel = Ns / Nr_cand

        if diff in conditions_a_eviter:
            continue
        if poles in (2, 4) and abs(ratio_actuel - 2 / 3) < 0.01:
            continue
        if poles == 2 and abs(ratio_actuel - 1.0) < 0.01:
            continue
        if abs(diff) < 1:
            continue

        recs.append((Nr_cand, float(ratio_actuel), int(diff)))

    # Deduplicate on Nr
    unique: Dict[int, Tuple[int, float, int]] = {}
    for t in recs:
        unique.setdefault(t[0], t)

    return sorted(unique.values(), key=lambda x: x[0])


def recommend_nr_harmonique(Ss: int, P_pairs: int) -> List[Tuple[int, float, int]]:
    """Return (Nr, Ss/Nr, Ss-Nr) recommendations."""
    Ns = int(Ss)
    P = int(P_pairs)
    poles = 2 * P

    valeurs_a_eviter = {Ns, Ns + poles, Ns - poles, Ns + 1, Ns - 1}
    for k in (2, 3):
        valeurs_a_eviter.add(k * Ns)
        if Ns % k == 0:
            valeurs_a_eviter.add(Ns // k)

    min_Nr = max(10, int(Ns * 0.5))
    max_Nr = int(Ns * 1.5)

    recs: List[Tuple[int, float, int]] = []
    for Nr in range(min_Nr, max_Nr + 1):
        if Nr in valeurs_a_eviter:
            continue

        est_multiple = False
        for k in range(1, 10):
            if k * Ns == Nr:
                est_multiple = True
                break
            if Ns % k == 0 and Nr == Ns // k:
                est_multiple = True
                break
        if est_multiple:
            continue

        rapport = Ns / Nr
        if 0.6 <= rapport <= 1.4:
            recs.append((int(Nr), float(rapport), int(Ns - Nr)))

    recs.sort(key=lambda x: abs(x[2]))
    return recs


# ============================================================
# Core rotor calculations (ported)
# ============================================================

def compute_a_Arb_detailed(r1: float, r2: float, alpha: float, h_r0: float, b_r0: float, d_rb: float) -> Tuple[float, Dict[str, float]]:
    angle_r1 = math.pi / 2 + alpha / 2
    angle_r2 = math.pi / 2 - alpha / 2
    cos_half = math.cos(alpha / 2)

    A1 = 0.5 * r1 ** 2 * angle_r1
    A2 = 0.5 * r2 ** 2 * angle_r2
    A3 = h_r0 * b_r0
    A4 = (r1 + r2) * (d_rb - r1 - r2) * cos_half

    total = A1 + A2 + A3 + A4
    return total, {
        "secteur_r1": A1,
        "secteur_r2": A2,
        "rectangle": A3,
        "trapeze": A4,
        "total": total,
    }


def solve_slot_geometry_simple(
    Ab_target: float,
    Nr: int,
    D: float,
    g: float,
    h_r0: float,
    b_r0: float,
    Wrt: float,
    *,
    tol_percent: float = 2.0,
    verbose_report: Optional[TextReporter] = None,
) -> Optional[Tuple[float, float, float, float, Dict[str, float]]]:
    """Return (r1, r2, d_rb, Ab_geom, parts) or None."""

    alpha = 2 * math.pi / Nr
    alpha_deg = alpha * 180 / math.pi
    cos_half = math.cos(alpha / 2)

    d_max = (D / 2) - g - h_r0 - D_MAX_MARGIN

    if verbose_report is not None:
        verbose_report.section("Calcul géométrie encoche avec contraintes")
        verbose_report.line(f"Section cible: {Ab_target:.3f} mm²")
        verbose_report.line(f"Paramètres: α = {alpha:.4f} rad ({alpha_deg:.1f}°), cos(α/2) = {cos_half:.4f}")
        verbose_report.line("Contraintes:")
        verbose_report.line(f"  - r1 > r2 + {EPS_R} mm")
        verbose_report.line(f"  - d_rb ≥ r1 + r2 + {GAP_D} mm")
        verbose_report.line(f"  - d_rb ≤ {d_max:.3f} mm")
        verbose_report.line(f"  - {MIN_R2_R1_RATIO} ≤ r2/r1 ≤ {MAX_R2_R1_RATIO}")

    r1_min = max(0.5, Wrt * 0.15)
    r1_max = min(Wrt * 0.45, d_max * 0.3)
    r1_max = max(r1_min + 0.5, r1_max)

    r1_candidates = [round(r1_min + (r1_max - r1_min) * i / 9, 3) for i in range(10)]
    ratio_candidates = [0.3, 0.4, 0.5, 0.6, 0.7, 0.8]

    best_solution = None
    best_error = float('inf')
    best_ratio = 0.0

    for r1 in r1_candidates:
        for ratio in ratio_candidates:
            r2 = r1 * ratio

            if not (r1 > r2 + EPS_R):
                continue
            if not (MIN_R2_R1_RATIO <= ratio <= MAX_R2_R1_RATIO):
                continue

            d_min = r1 + r2 + GAP_D
            if d_min > d_max:
                continue

            d_test = d_min
            found = False
            for _ in range(50):
                Ab_calc, parts = compute_a_Arb_detailed(r1, r2, alpha, h_r0, b_r0, d_test)
                if Ab_calc >= Ab_target:
                    error = abs(Ab_calc - Ab_target) / Ab_target * 100
                    if error < best_error:
                        best_error = error
                        best_solution = (r1, r2, d_test, Ab_calc, parts)
                        best_ratio = ratio
                        found = True

                    if error <= tol_percent:
                        return best_solution
                    break

                d_test += 0.5
                if d_test > d_max:
                    break

            if found and best_error <= tol_percent * 2:
                break

        if best_solution and best_error <= tol_percent:
            break

    if best_solution:
        if verbose_report is not None:
            r1, r2, d_rb, Ab_calc, _parts = best_solution
            verbose_report.line("\nMeilleure solution trouvée:")
            verbose_report.line(f"  r1 = {r1:.3f} mm, r2 = {r2:.3f} mm (ratio = {best_ratio:.2f})")
            verbose_report.line(f"  d_rb = {d_rb:.3f} mm (d_max = {d_max:.3f} mm)")
            verbose_report.line(f"  Section: {Ab_calc:.3f} mm² (cible: {Ab_target:.3f} mm²)")
            verbose_report.line(f"  Erreur: {best_error:.2f}%")
        return best_solution

    # relaxed attempt
    for r1 in [r1_max, r1_max + 0.5, r1_max + 1.0]:
        for ratio in [0.3, 0.4]:
            r2 = r1 * ratio
            d_test = r1 + r2 + GAP_D
            if d_test > d_max:
                continue
            Ab_calc, parts = compute_a_Arb_detailed(r1, r2, alpha, h_r0, b_r0, d_test)
            error = abs(Ab_calc - Ab_target) / Ab_target * 100
            if error < best_error:
                best_error = error
                best_solution = (r1, r2, d_test, Ab_calc, parts)

    return best_solution


def _select_machine_type_from_power_kw(power_kw: float) -> str:
    """Infer machine category from rated power."""
    if power_kw < 100.0:
        return "petite"
    if power_kw <= 1000.0:
        return "moyenne"
    return "grande"


def _select_bar_current_density_Jb(power_kw: float) -> Tuple[float, str]:
    """Automatic Jb selection based on power."""
    mt = _select_machine_type_from_power_kw(power_kw)
    if mt == "petite":
        return 4.0, mt
    if mt == "moyenne":
        return 3.5, mt
    return 3.0, mt


def _suggest_ring_Je_and_dims(machine_type: str) -> Tuple[float, float, float]:
    """Return (Je, h_er_typ, b_er_typ) as suggested in original."""
    if machine_type == "petite":
        return 6.75, 8.0, 15.0
    if machine_type == "moyenne":
        return 5.75, 12.0, 25.0
    if machine_type == "grande":
        return 5.0, 18.0, 35.0
    return 6.0, 10.0, 20.0


def _rho_from_material(name: str, custom_rho: Optional[float]) -> Tuple[float, str]:
    if name == "Custom":
        if custom_rho is None or custom_rho <= 0:
            raise ValueError("Custom material selected but custom resistivity is missing/invalid")
        return float(custom_rho), "Custom"
    if name not in RESISTIVITY_TABLE:
        raise ValueError(f"Unknown material: {name}")
    return float(RESISTIVITY_TABLE[name]), name


def run_rotor_design(inputs: RotorStatorInputs, config: RotorUserConfig) -> Tuple[Dict[str, Any], str]:
    """Run rotor calculations and return (results, report_text)."""

    rep = TextReporter()
    rep.banner("CALCUL COMPLET ROTOR (GUI)")

    # Unpack stator inputs
    Ns = int(inputs.Ss)
    P = int(inputs.P_pairs)
    Nr = int(config.Nr)

    D = float(inputs.D_mm)
    L = float(inputs.L_mm)
    Lstack = float(inputs.Ls_mm)
    Li_mm = float(getattr(inputs, 'Li_mm', 0.0) or 0.0)
    Ki = float(getattr(inputs, 'ki', 0.95) or 0.95)

    Iph = float(inputs.Iph_A)
    Zs = int(inputs.Nc)

    phi = float(inputs.flux_per_pole_Wb)
    Brt = float(inputs.B_tooth_T)
    Bry = float(inputs.B_yoke_T)

    if Ns <= 0 or P <= 0 or Nr <= 0:
        raise ValueError("Invalid Ns/P/Nr")

    # Constants (same as original), except Kws which should follow the stator design.
    try:
        Kws = float(getattr(inputs, 'Kws', 0.955) or 0.955)
    except Exception:
        Kws = 0.955
    kwr = 1.0
    Zr = 1
    # Ki comes from stator stacking factor

    # fixed geometry params (model)
    h_r0 = 0.5
    b_r0 = 0.8

    # Current density for bars (auto from power)
    Jb, machine_type_bar = _select_bar_current_density_Jb(inputs.power_kw)

    rep.show_params({
        "Ns (stator slots)": Ns,
        "P (pole pairs)": P,
        "Nr (rotor bars)": Nr,
        "D (mm)": f"{D:.3f}",
        "L (mm)": f"{L:.3f}",
        "Lstack (mm)": f"{Lstack:.3f}",
        "Li (mm)": f"{Li_mm:.3f}",
        "Ki (stacking factor)": f"{Ki:.4f}",
        "Iph (A)": f"{Iph:.3f}",
        "Nc/Zs (cond/slot)": Zs,
        "φ (Wb)": phi,
        "Brt (T)": Brt,
        "Bry (T)": Bry,
        "Kws (stator winding factor)": f"{Kws:.6f}",
        "Jb auto (A/mm²)": Jb,
        "Type machine (barres)": machine_type_bar,
        "Nr method": config.nr_method,
    }, "Entrées (stator→rotor + config rotor)")

    # SECTION 2: airgap, Dr, slot pitch (uses L)
    rep.section("Géométrie globale (Lg, Dr, sp2)")
    Lg = 0.2 + 2 * math.sqrt(D * L / 1_000_000)
    g = Lg
    rep.show_calc("Étape G1) Entrefer Lg", "Lg = 0.2 + 2·√(D·L/1e6)", f"Lg = 0.2 + 2·√({D:.3f}·{L:.3f}/1e6)", f"Lg = {Lg:.3f} mm")

    Dr = D - 2 * Lg
    rep.show_calc("Étape G2) Diamètre rotor Dr", "Dr = D - 2·Lg", f"Dr = {D:.3f} - 2·{Lg:.3f}", f"Dr = {Dr:.3f} mm")

    sp2 = math.pi * Dr / Nr
    rep.show_calc("Étape G3) Pas encoches rotor sp2", "sp2 = π·Dr/Nr", f"sp2 = π·{Dr:.3f}/{Nr}", f"sp2 = {sp2:.3f} mm")

    # SECTION 3: electrical (Ir, Ib, Ab)
    rep.section("Électrique (Ir, Ib, Ab_elec)")
    Ir = 0.85 * Iph
    rep.show_calc("Étape E1) Courant rotorique Ir", "Ir = 0.85·Iph", f"Ir = 0.85·{Iph:.3f}", f"Ir = {Ir:.3f} A")

    Ib = Ir * Kws * Ns * Zs / (2 * kwr * Nr * Zr)
    rep.show_calc("Étape E2) Courant de barre Ib", "Ib = Ir·Kws·Ns·Zs / (2·kwr·Nr·Zr)", f"Ib = {Ir:.3f}·{Kws}·{Ns}·{Zs}/(2·{kwr}·{Nr}·{Zr})", f"Ib = {Ib:.3f} A")

    Ab_req = Ib / Jb
    rep.show_calc("Étape E3) Section électrique Ab_elec", "Ab_elec = Ib/Jb", f"Ab_elec = {Ib:.3f}/{Jb:.3f}", f"Ab_elec = {Ab_req:.3f} mm²")

    # SECTION 4: magnetic -> Wrt/Wry (uses L)
    rep.section("Magnétique (fluxrt_max, Wrt, Wry)")
    rep.line("Note: Wrt/Wry use Li (net iron length) instead of L×Ki.")
    fluxrt_max = phi * math.sin(math.pi * P / Nr)
    rep.show_calc("Étape M1) Flux max dent rotor", "φ_rt_max = φ·sin(π·P/Nr)", f"φ_rt_max = {phi:.6f}·sin(π·{P}/{Nr})", f"φ_rt_max = {fluxrt_max:.6f} Wb")

    L_m = L / 1000.0
    Li_m = Li_mm / 1000.0
    Li_used_m = Li_m if Li_m > 0 else (L_m * Ki)
    rep.show_calc(
        "Info) Longueur magnétique utilisée",
        "Li_used = Li",
        f"Li_used = {Li_used_m:.6f} m",
        f"Li_used = {Li_used_m * 1000.0:.3f} mm",
    )

    Wrt = 1000.0 * (fluxrt_max / (Brt * Li_used_m))
    rep.show_calc("Étape M2) Largeur dent rotor Wrt", "Wrt = [φ_rt_max/(Brt·Li)]×1000", f"Wrt = [{fluxrt_max:.6f}/({Brt:.3f}·{Li_used_m:.6f})]×1000", f"Wrt = {Wrt:.3f} mm")

    Wry = 1000.0 * ((phi / 2) / (Bry * Li_used_m))
    rep.show_calc("Étape M3) Largeur culasse rotor Wry", "Wry = [(φ/2)/(Bry·Li)]×1000", f"Wry = [({phi:.6f}/2)/({Bry:.3f}·{Li_used_m:.6f})]×1000", f"Wry = {Wry:.3f} mm")

    # SECTION 5: slot geometry with tolerance
    rep.section("Géométrie encoche avec contraintes")
    best = solve_slot_geometry_simple(
        Ab_target=Ab_req,
        Nr=Nr,
        D=D,
        g=g,
        h_r0=h_r0,
        b_r0=b_r0,
        Wrt=Wrt,
        tol_percent=float(config.tol_percent),
        verbose_report=rep,
    )

    if best is None:
        raise ValueError("Unable to find a valid rotor slot geometry")

    r1, r2, d_rb, Ab_geom, parts = best
    alpha = 2 * math.pi / Nr
    rep.show_params({
        "α (rad)": f"{alpha:.6f}",
        "α (deg)": f"{alpha * 180 / math.pi:.2f}",
        "r1 (mm)": f"{r1:.3f}",
        "r2 (mm)": f"{r2:.3f}",
        "r2/r1": f"{(r2 / r1):.3f}" if r1 else "inf",
        "d_rb (mm)": f"{d_rb:.3f}",
        "Ab_elec (mm²)": f"{Ab_req:.3f}",
        "Ab_geo (mm²)": f"{Ab_geom:.3f}",
        "Erreur (%)": f"{100 * abs(Ab_geom - Ab_req) / Ab_req:.2f}" if Ab_req else "inf",
    }, "Résultat géométrie")

    rep.section("Décomposition Ab_geo")
    rep.line(f"  - Secteur r1 : {parts['secteur_r1']:.3f} mm²")
    rep.line(f"  - Secteur r2 : {parts['secteur_r2']:.3f} mm²")
    rep.line(f"  - Rectangle  : {parts['rectangle']:.3f} mm²")
    rep.line(f"  - Trapèze    : {parts['trapeze']:.3f} mm²")
    rep.line(f"  - TOTAL      : {parts['total']:.3f} mm²")

    # SECTION 6: bar resistance + losses
    rep.banner("RÉSISTANCE DES BARRES ROTORIQUES")
    skew_angle = float(config.skew_angle_deg)

    rho_bar, bar_mat_label = _rho_from_material(config.bar_material, config.bar_rho_ohm_m)
    rho_mm_bar = rho_bar * 1e6  # Ω·mm²/m

    if skew_angle != 0.0:
        Lbar = Lstack / math.cos(math.radians(skew_angle))
        rep.show_calc("Étape B1) Longueur effective de barre", "Lbar = Lstack / cos(θ_skew)", f"Lbar = {Lstack:.3f} / cos({skew_angle:.1f}°)", f"Lbar = {Lbar:.3f} mm")
    else:
        Lbar = Lstack
        rep.show_calc("Étape B1) Longueur de barre", "Lbar = Lstack", f"Lbar = {Lstack:.3f}", f"Lbar = {Lbar:.3f} mm")

    Arb = Ab_geom
    rep.show_calc("Étape B2) Section de la barre Arb", "Arb = Ab_geom", f"Arb = {Arb:.3f}", f"Arb = {Arb:.3f} mm²")

    Rbar = rho_mm_bar * (Lbar / 1000.0) / Arb
    rep.show_calc("Étape B3) Résistance d'une barre Rbar", "Rbar = (ρ×10⁶)×(Lbar/1000)/Arb", f"Rbar = ({rho_bar:.2e}×10⁶)×({Lbar:.3f}/1000)/{Arb:.3f}", f"Rbar = {Rbar:.6e} Ω")

    Rbar_total_parallel = Rbar / Nr
    P_bar = Nr * (Ib ** 2) * Rbar

    # SECTION 7: ring sizing
    rep.banner("DIMENSIONNEMENT DES ANNEAUX DE COURT-CIRCUIT")
    Ie = (Nr * Ib) / (math.pi * P)
    rep.show_calc("Étape A1) Courant d'anneau Ie", "Ie = (Nr·Ib)/(π·P)", f"Ie = ({Nr}·{Ib:.3f})/(π·{P})", f"Ie = {Ie:.3f} A")

    machine_type_anneau = machine_type_bar
    Je_sugg, h_er_typ, b_er_typ = _suggest_ring_Je_and_dims(machine_type_anneau)

    if config.ring_je_mode == 'custom':
        if config.ring_je_custom is None or config.ring_je_custom <= 0:
            raise ValueError("Custom Je selected but value is missing/invalid")
        Je = float(config.ring_je_custom)
    else:
        Je = float(Je_sugg)

    aAeb_req = Ie / Je
    rep.show_calc("Étape A2) Section nécessaire aAeb", "aAeb = Ie/Je", f"aAeb = {Ie:.3f}/{Je:.3f}", f"aAeb = {aAeb_req:.3f} mm²")

    if config.ring_dim_mode == 'manual':
        if config.ring_h_er_mm is None or config.ring_b_er_mm is None:
            raise ValueError("Manual ring dimensions selected but missing h_er/b_er")
        h_er = float(config.ring_h_er_mm)
        b_er = float(config.ring_b_er_mm)
    elif config.ring_dim_mode == 'auto':
        ratio = 2.0
        h_er = math.sqrt(aAeb_req / ratio)
        b_er = ratio * h_er
        h_er = min(max(h_er, 5.0), 25.0)
        b_er = aAeb_req / h_er
        b_er = min(max(b_er, 10.0), 50.0)
        h_er = aAeb_req / b_er
        h_er, b_er = round(h_er, 1), round(b_er, 1)
    else:
        h_er, b_er = float(h_er_typ), float(b_er_typ)

    aAeb_reel = h_er * b_er
    Je_reel = Ie / aAeb_reel if aAeb_reel else float('inf')

    # SECTION 8: ring resistance
    rep.banner("CALCUL DE LA RÉSISTANCE DE L'ANNEAU")
    rho_ring, ring_mat_label = _rho_from_material(config.ring_material, config.ring_rho_ohm_m)
    rho_mm_ring = rho_ring * 1e6

    Der = Dr + 2 * Lg + b_er
    Ler = math.pi * Der
    Aer = aAeb_reel

    Rer = rho_mm_ring * (Ler / 1000.0) / Aer
    Rer_total = Rer / 2.0

    # SECTION 9: totals
    rep.banner("RÉSISTANCE TOTALE ET PERTES ROTORIQUES")
    P_er = 2 * (Ie ** 2) * Rer
    P_rotor_total = P_bar + P_er
    R_rotor_eq = Rbar_total_parallel + Rer_total

    results: Dict[str, Any] = {
        "Ns": Ns,
        "P_pairs": P,
        "Nr": Nr,
        "D_mm": D,
        "L_mm": L,
        "Lstack_mm": Lstack,
        "Lg_mm": Lg,
        "Dr_mm": Dr,
        "sp2_mm": sp2,
        "Iph_A": Iph,
        "Ir_A": Ir,
        "Nc": Zs,
        "phi_Wb": phi,
        "Brt_T": Brt,
        "Bry_T": Bry,
        "Kws": Kws,
        "kwr": kwr,
        "Ki": Ki,
        "Jb_A_per_mm2": Jb,
        "Ib_A": Ib,
        "Ab_req_mm2": Ab_req,
        "fluxrt_max_Wb": fluxrt_max,
        "Wrt_mm": Wrt,
        "Wry_mm": Wry,
        "alpha_rad": alpha,
        "h_r0_mm": h_r0,
        "b_r0_mm": b_r0,
        "r1_mm": r1,
        "r2_mm": r2,
        "d_rb_mm": d_rb,
        "Ab_geom_mm2": Ab_geom,
        "tol_percent": float(config.tol_percent),
        "skew_angle_deg": skew_angle,
        "bar_material": bar_mat_label,
        "bar_rho_ohm_m": rho_bar,
        "Lbar_mm": Lbar,
        "Rbar_ohm": Rbar,
        "Rbar_total_parallel_ohm": Rbar_total_parallel,
        "P_bar_W": P_bar,
        "machine_type_bar": machine_type_bar,
        "Ie_A": Ie,
        "Je_A_per_mm2": Je,
        "h_er_mm": h_er,
        "b_er_mm": b_er,
        "aAeb_req_mm2": aAeb_req,
        "aAeb_reel_mm2": aAeb_reel,
        "Je_reel_A_per_mm2": Je_reel,
        "ring_material": ring_mat_label,
        "ring_rho_ohm_m": rho_ring,
        "Der_mm": Der,
        "Ler_mm": Ler,
        "Aer_mm2": Aer,
        "Rer_ohm": Rer,
        "Rer_total_ohm": Rer_total,
        "P_er_W": P_er,
        "R_rotor_eq_ohm": R_rotor_eq,
        "P_rotor_total_W": P_rotor_total,
    }

    rep.show_params({
        "Matériau barres": bar_mat_label,
        "ρ barres (Ω·m)": f"{rho_bar:.2e}",
        "Rbar (Ω/barre)": f"{Rbar:.6e}",
        "Pertes barres (W)": f"{P_bar:.3f}",
        "Matériau anneaux": ring_mat_label,
        "ρ anneaux (Ω·m)": f"{rho_ring:.2e}",
        "Rer (Ω)": f"{Rer:.6e}",
        "Pertes anneaux (W)": f"{P_er:.3f}",
        "R_rotor_eq (Ω)": f"{R_rotor_eq:.6e}",
        "Pertes totales rotor (W)": f"{P_rotor_total:.3f}",
    }, "Synthèse")

    return results, rep.text()
