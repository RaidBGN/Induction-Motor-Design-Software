"""
╔════════════════════════════════════════════════════════════════════════════╗
║                         STATOR DESIGN MODULE                              ║
║                   Three-Phase Asynchronous Induction Motor                 ║
║                                                                            ║
║  Design calculations based on:                                            ║
║  - Design-AC-machines-2.pdf (Prof. Kamel BOUGHRARA)                       ║
║  - Empirical Tables (abaques-1.pdf)                                       ║
║                                                                            ║
║  Author: Motor Design Tool v2.0                                           ║
║  General Usage: Works for ANY 3-phase async cage motor                     ║
╚════════════════════════════════════════════════════════════════════════════╝
"""

import math
from dataclasses import dataclass
from typing import Dict, Tuple, Optional, List


# ═══════════════════════════════════════════════════════════════════════════
# ELECTRICAL STEEL GRADES (USER-SELECTABLE VIA GUI)
# Stored as plain dicts so they can be serialized/exported easily.
# ═══════════════════════════════════════════════════════════════════════════

STEEL_DATABASE: List[Dict[str, object]] = [
    {
        "Grade": "M800-65A",
        "Thickness_mm": 0.65,
        "Max_Design_B": 1.55,  # T (knee point limit)
        "Loss_W_kg": 8.00,     # @ 1.5 T
        "Application": "Low Cost (Fans/Pumps)",
        "Specific_Density_kg_m3": 7800,
    },
    {
        "Grade": "M600-50A",
        "Thickness_mm": 0.50,
        "Max_Design_B": 1.60,
        "Loss_W_kg": 6.00,
        "Application": "Standard (IE1)",
        "Specific_Density_kg_m3": 7750,
    },
    {
        "Grade": "M400-50A",
        "Thickness_mm": 0.50,
        "Max_Design_B": 1.65,
        "Loss_W_kg": 4.00,
        "Application": "Industrial (IE2/IE3)",
        "Specific_Density_kg_m3": 7700,
    },
    {
        "Grade": "M270-35A",
        "Thickness_mm": 0.35,
        "Max_Design_B": 1.70,
        "Loss_W_kg": 2.70,
        "Application": "High Efficiency (IE3/IE4)",
        "Specific_Density_kg_m3": 7650,
    },
    {
        "Grade": "M235-35A",
        "Thickness_mm": 0.35,
        "Max_Design_B": 1.75,
        "Loss_W_kg": 2.35,
        "Application": "Premium / EV",
        "Specific_Density_kg_m3": 7600,
    }
]


# ═══════════════════════════════════════════════════════════════════════════
# STANDARD WIRE GAUGES (SWG) TABLE
# Used by Forward Design wire selection.
# ═══════════════════════════════════════════════════════════════════════════

SWG_GAUGE: List[int] = [
    -7, -6, -5, -4, -3, -2, 0,
    1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22,
    23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40,
    41, 42, 43, 44, 45, 46, 47, 48, 49, 50
]

SWG_DIAMETER_MM: List[float] = [
    12.7, 11.786, 10.973, 10.16, 9.449, 8.839, 8.23,
    7.62, 7.01, 6.401, 5.893, 5.385, 4.877, 4.47, 4.064, 3.658, 3.251, 2.946, 2.642,
    2.337, 2.032, 1.829, 1.626, 1.422, 1.219, 1.016, 0.914, 0.813, 0.711,
    0.61, 0.559, 0.508, 0.4572, 0.4166, 0.3759, 0.3454, 0.315, 0.2946, 0.2743,
    0.254, 0.2337, 0.2134, 0.193, 0.1727, 0.1524, 0.1321, 0.1219,
    0.1118, 0.1016, 0.0914, 0.0813, 0.0711, 0.061, 0.0508, 0.0406, 0.0305, 0.0254
]


def get_steel_grades() -> List[str]:
    """Return available steel grade names."""
    return [row.get("Grade", "") for row in STEEL_DATABASE if row.get("Grade")]


def get_steel_by_grade(grade: str) -> Optional[Dict[str, object]]:
    """Return the full steel record for a grade (as a shallow copy)."""
    if not grade:
        return None
    for row in STEEL_DATABASE:
        if str(row.get("Grade")) == str(grade):
            return dict(row)
    return None


def get_steel_density_kg_m3(grade: Optional[str]) -> float:
    """Return specific density in kg/m^3 for a steel grade.

    If grade is missing or not found, returns a reasonable default (~7700 kg/m^3).
    """
    default_density = 7700.0
    if not grade:
        return default_density
    row = get_steel_by_grade(str(grade))
    if not row:
        return default_density
    dens = row.get('Specific_Density_kg_m3', None)
    try:
        dens_val = float(dens) if dens is not None else None
    except (TypeError, ValueError):
        dens_val = None
    return dens_val if (dens_val is not None and dens_val > 0) else default_density


def get_steel_loss_w_per_kg(grade: Optional[str]) -> float:
    """Return lamination loss value Pkg in W/kg for a steel grade.

    Uses the Loss_W_kg field from STEEL_DATABASE.
    If grade is missing or not found, returns a conservative default.
    """
    default_loss = 4.0
    if not grade:
        return default_loss
    row = get_steel_by_grade(str(grade))
    if not row:
        return default_loss
    loss = row.get('Loss_W_kg', None)
    try:
        loss_val = float(loss) if loss is not None else None
    except (TypeError, ValueError):
        loss_val = None
    return loss_val if (loss_val is not None and loss_val > 0) else default_loss


def get_stacking_factor(thickness_mm: float) -> float:
    """Estimated stacking factor (iron factor) based on lamination thickness.

    Thinner laminations typically have more insulation layers, reducing the iron factor.
    """
    if thickness_mm >= 0.65:
        return 0.97
    if thickness_mm >= 0.50:
        return 0.95  # Standard for ~0.50 mm grades (e.g., M400-50A)
    if thickness_mm >= 0.35:
        return 0.93  # Standard for ~0.35 mm grades (e.g., M270-35A)
    return 0.90


# ═══════════════════════════════════════════════════════════════════════════
# EMPIRICAL DATA TABLES FROM abaques-1.pdf (Tables 7.1-7.4)
# ═══════════════════════════════════════════════════════════════════════════

class AbaquesTables:
    """
    Empirical data tables for 3-phase asynchronous motors
    from abaques-1.pdf (Table 7.1-7.4)
    """
    
    # TABLE 7.1: Bav (T) and q (A/m) vs Power (kW)
    TABLE_7_1 = {
        1: {'Bav': 0.35, 'q': 16000},
        2: {'Bav': 0.38, 'q': 19000},
        5: {'Bav': 0.42, 'q': 23000},
        10: {'Bav': 0.46, 'q': 25000},
        20: {'Bav': 0.48, 'q': 26000},
        50: {'Bav': 0.50, 'q': 29000},
        100: {'Bav': 0.51, 'q': 31000},
        500: {'Bav': 0.53, 'q': 33000},
    }
    
    # TABLE 7.5: Recommended q (slots/pole/phase) vs Power and Poles
    TABLE_Q_RECOMMENDATIONS = {
        (0.5, 5, 2): (2, 3),
        (0.5, 5, 4): (2, 3),
        (0.5, 5, 6): (2, 2),
        (0.5, 5, 8): (1, 2),
        (5, 50, 2): (3, 4),
        (5, 50, 4): (3, 3),
        (5, 50, 6): (2, 3),
        (5, 50, 8): (2, 2),
        (50, 200, 2): (4, 5),
        (50, 200, 4): (3, 4),
        (50, 200, 6): (3, 3),
        (50, 200, 8): (2, 3),
        (200, 1000, 2): (5, 6),
        (200, 1000, 4): (4, 5),
        (200, 1000, 6): (3, 4),
        (200, 1000, 8): (3, 4),
    }
    
    # Magnetic Induction Limits (Tesla)
    INDUCTION_LIMITS = {
        'B_teeth_max': 1.8,        # Maximum acceptable tooth flux density
        'B_teeth_min': 1.6,        # Minimum acceptable tooth flux density
        'B_teeth_optimal_min': 1.6,  # Optimal range lower bound
        'B_teeth_optimal_max': 1.8,  # Optimal range upper bound
        'B_core_max': 1.6,
        'B_core_min': 1.2,
    }

    # -------------------------------------------------------------------
    # Electrical steel B-H curves (user-provided)
    # B in Tesla, H in A/m
    # -------------------------------------------------------------------
    STEEL_BH_CURVES = {
        'M800-65A': {
            'B': [0.1, 0.5, 1.0, 1.2, 1.4, 1.5, 1.6, 1.65, 1.7, 1.75],
            'H': [35, 65, 115, 170, 290, 400, 600, 850, 1500, 3000],
        },
        'M600-50A': {
            'B': [0.1, 0.5, 1.0, 1.2, 1.4, 1.5, 1.6, 1.65, 1.7, 1.75, 1.8],
            'H': [30, 55, 95, 130, 210, 280, 400, 550, 900, 1800, 4000],
        },
        'M400-50A': {
            'B': [0.1, 0.5, 1.0, 1.2, 1.4, 1.5, 1.6, 1.65, 1.7, 1.75, 1.8],
            'H': [25, 45, 80, 105, 160, 210, 300, 400, 650, 1200, 2500],
        },
        'M270-35A': {
            'B': [0.1, 0.5, 1.0, 1.2, 1.4, 1.5, 1.6, 1.65, 1.7, 1.75, 1.8, 1.85],
            'H': [20, 35, 65, 85, 120, 150, 210, 270, 400, 650, 1200, 2500],
        },
        'M235-35A': {
            'B': [0.1, 0.5, 1.0, 1.2, 1.4, 1.5, 1.6, 1.65, 1.7, 1.75, 1.8, 1.85, 1.9],
            'H': [18, 30, 55, 70, 95, 120, 160, 200, 280, 420, 700, 1300, 3000],
        },
    }

    # -------------------------------------------------------------------
    # Carter coefficient tables (user-provided)
    # -------------------------------------------------------------------
    # Carter coefficient for OPEN slot shape
    # X axis: opening_airgap (unitless ratio as provided)
    CARTER_COEFF_OPEN_SLOT = {
        'opening_airgap': [0.0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0, 4.5, 5.0],
        'carter_coeff':   [0.00, 0.07, 0.14, 0.21, 0.28, 0.34, 0.38, 0.42, 0.45, 0.48, 0.50],
    }

    # Carter coefficient for SEMI-OPEN slot shape
    # X axis: slot_opening_gap (unitless ratio as provided)
    CARTER_COEFF_SEMI_OPEN_SLOT = {
        'slot_opening_gap': [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12],
        'carter_coeff':     [0.00, 0.18, 0.32, 0.45, 0.53, 0.60, 0.66, 0.71, 0.76, 0.80, 0.83, 0.86, 0.89],
    }
    
    # Current Density Recommendations (A/mm²)
    # Current density based on stator inner diameter D (mm)
    # D values in mm
    J_DIAMETER_TABLE_D = [100, 150, 200, 300, 400, 500, 750, 1000]
    # J values in A/mm^2
    J_DIAMETER_TABLE_J = [4, 3.8, 3.6, 3.5, 3.5, 3.5, 3.5, 3.5]
    
    # Slot space factor range
    SLOT_SPACE_FACTOR = (0.25, 0.4)
    
    # Slot aspect ratio range
    SLOT_ASPECT_RATIO = (2.5, 3.5)
    
    # Conductor aspect ratio range (for rectangular conductors)
    CONDUCTOR_ASPECT_RATIO = (2.5, 3.5)

    # -------------------------------------------------------------------
    # Open Slot Method Abaques (user-provided)
    # Phase current ranges (A): I_min[i] <= I < I_max[i]
    # Recommended strip dimensions for rectangular conductors
    # -------------------------------------------------------------------

    OPEN_SLOT_I_MIN_A = [0.0, 20.0, 40.0, 80.0, 150.0]
    OPEN_SLOT_I_MAX_A = [20.0, 40.0, 80.0, 150.0, float('inf')]

    OPEN_SLOT_THICKNESS_MIN_MM = [1.0, 1.5, 1.9, 2.5, 3.2]
    OPEN_SLOT_THICKNESS_MAX_MM = [1.5, 2.0, 2.5, 3.2, 4.0]

    OPEN_SLOT_WIDTH_MIN_MM = [2.5, 4.0, 6.0, 8.0, 10.0]
    OPEN_SLOT_WIDTH_MAX_MM = [5.0, 7.0, 9.0, 11.0, 14.0]

    OPEN_SLOT_RATIO_MIN = [2.5, 2.5, 2.8, 3.0, 3.0]
    OPEN_SLOT_RATIO_MAX = [3.5, 3.5, 3.5, 3.5, 3.5]

    OPEN_SLOT_STANDARD_THICKNESSES_MM = [
        1.0, 1.25, 1.5, 1.6, 1.8, 1.9, 2.0,
        2.24, 2.5, 2.8, 3.15, 3.55, 4.0,
    ]


# ═══════════════════════════════════════════════════════════════════════════
# LAMBDA (L/τ) SELECTION TABLES
# λ is defined as L / τ_pole where τ_pole = πD/P
# ═══════════════════════════════════════════════════════════════════════════

# Recommended L/τ ranges and typical values by pole count (method B)
LAMBDA_BY_POLES: Dict[int, Dict[str, float]] = {
    2: {'min': 0.7, 'max': 1.5, 'typical': 1.1},
    4: {'min': 1.0, 'max': 2.0, 'typical': 1.5},
    6: {'min': 1.2, 'max': 2.5, 'typical': 1.8},
    8: {'min': 1.5, 'max': 3.0, 'typical': 2.2},
}

# Goal-based ranges for L/τ (used when dimension_method='preference_based')
PREFERENCE_LAMBDA_RANGES: Dict[str, Tuple[float, float]] = {
    'minimum_cost': (1.5, 2.0),
    'good_efficiency': (1.4, 1.6),
    'balanced_design': (1.0, 1.1),
}

# Priority order when no perfect overlap exists (highest priority first)
PREFERENCE_PRIORITY: List[str] = [
    'balanced_design',
    'good_efficiency',
    'good_power_factor',
    'minimum_cost',
]

# Peripheral speed limit for standard induction motors
PERIPHERAL_SPEED_LIMIT = 30.0  # m/s


# ═══════════════════════════════════════════════════════════════════════════
# DATA CLASSES FOR STATOR PARAMETERS
# ═══════════════════════════════════════════════════════════════════════════

@dataclass
class MotorSpecifications:
    """Motor input specifications"""
    power_kw: float
    voltage_v: float
    frequency_hz: float
    poles: int
    connection: str = 'star'  # 'star' or 'delta'
    phases: int = 3
    custom_winding_factor: bool = False
    winding_type: str = None  # 'full pitch', 'short pitch (5/6)', 'short pitch (2/3)'
    coil_pitch_percent: float = None  # 100, 83.3, 66.7
    # D/L selection method: 'best_pf', 'lambda_by_poles', 'preference_based', or 'custom_lambda'
    dimension_method: str = 'best_pf'
    # Selected preferences for method C
    preference_flags: Optional[Dict[str, bool]] = None
    # Optional overrides (when provided, take priority over abaques/heuristics)
    target_efficiency: Optional[float] = None
    target_power_factor: Optional[float] = None
    override_lambda_ratio: Optional[float] = None  # lambda = L / tau_pole
    override_Bav: Optional[float] = None
    override_ac: Optional[float] = None  # specific electric loading a_c [A/m]
    override_q: Optional[float] = None  # slots per pole per phase
    # Slot design method selector
    slot_method: str = 'semi_open_slot'

    # Cooling ducts override (passed from GUI)
    # Ncc: number of radial cooling ducts (integer, expected 0..2 in the GUI).
    # When provided, the duct width is kept standard at 10mm.
    override_num_cooling_canals: Optional[int] = None

    # Electrical steel selection (passed from GUI).
    steel_grade: Optional[str] = None
    steel_data: Optional[Dict[str, object]] = None

    # Stator conductor material selection (passed from GUI).
    # Used for: stator resistance (via resistivity) and conductor weight (via density).
    conductor_material: Optional[str] = None
    conductor_rho_ohm_m: Optional[float] = None
    conductor_density_g_cm3: Optional[float] = None


@dataclass
class StatorGeometry:
    """Stator main dimensions"""
    D: float                  # Bore diameter [m]
    L: float                  # Physical stack length [m]
    Ls: float                 # Stack length after removing cooling-duct widths [m]
    Li: float                 # Net iron length (effective magnetic length) [m]  (Li = Ls * ki)
    D_ext: float              # External diameter [m]
    tau_pole: float           # Pole pitch [m]
    flux_per_pole: float      # Flux per pole [Wb]
    h_slot: float             # Slot height [m]
    d_cs: float               # Core depth [m]
    ki: float = 0.95          # Iron factor (stacking factor)
    num_cooling_canals: int = 0  # Number of radial cooling ducts
    canal_width: float = 0.0  # Width of each cooling duct [m]
    validation_info: Dict = None  # Validation information for D and L
    
    def __str__(self):
        return f"""
    ╔══════════════════════════════════════════════╗
    ║         STATOR MAIN DIMENSIONS               ║
    ╠══════════════════════════════════════════════╣
    ║ Bore diameter D         : {self.D*1000:>10.2f} mm  ║
    ║ Stack length L          : {self.L*1000:>10.2f} mm  ║
    ║ Stack length Ls         : {self.Ls*1000:>10.2f} mm  ║
    ║ Net iron length Li      : {self.Li*1000:>10.2f} mm  ║
    ║ External diameter D_ext : {self.D_ext*1000:>10.2f} mm  ║
    ║ Pole pitch τp           : {self.tau_pole*1000:>10.2f} mm  ║
    ║ Flux per pole Φ         : {self.flux_per_pole*1000:>10.4f} mWb ║
    ║ Slot height h_slot      : {self.h_slot*1000:>10.2f} mm  ║
    ║ Core depth d_cs         : {self.d_cs*1000:>10.2f} mm  ║
    ╚══════════════════════════════════════════════╝"""


@dataclass
class StatorSlots:
    """Stator slot design parameters"""
    Ss: int                   # Total number of stator slots
    q: int                    # Slots per pole per phase
    Nc: int                   # Conductors per slot
    b_slot: float             # Slot width [mm]
    h_slot: float             # Slot height [mm]
    A_slot: float             # Slot area [mm²]
    slot_pitch: float         # Slot pitch [m]
    tooth_width_D13: float    # Tooth width at D(1/3) [m]
    B_tooth: float            # Flux density in teeth [T]
    B_tooth_max: float        # Max flux density in teeth [T]
    D_13: float               # Diameter at 1/3 slot height [m]
    
    def __str__(self):
        return f"""
    ╔══════════════════════════════════════════════╗
    ║           STATOR SLOTS DESIGN                ║
    ╠══════════════════════════════════════════════╣
    ║ Total slots Ss          : {self.Ss:>10}           ║
    ║ Slots/pole/phase q      : {self.q:>10}           ║
    ║ Conductors/slot Nc      : {self.Nc:>10}           ║
    ║ Slot width b_slot       : {self.b_slot:>10.2f} mm  ║
    ║ Slot height h_slot      : {self.h_slot:>10.2f} mm  ║
    ║ Slot area A_slot        : {self.A_slot:>10.2f} mm² ║
    ║ D(1/3)                  : {self.D_13*1000:>10.2f} mm  ║
    ║ Tooth width @ D(1/3)    : {self.tooth_width_D13*1000:>10.2f} mm  ║
    ║ B_tooth                 : {self.B_tooth:>10.3f} T   ║
    ║ B_tooth_max             : {self.B_tooth_max:>10.3f} T   ║
    ╚══════════════════════════════════════════════╝"""


@dataclass
class StatorWinding:
    """Stator winding parameters"""
    TPH: int                  # Turns per phase
    Zs: int                   # Total stator conductors
    conductor_type: str       # 'circular' or 'rectangular'
    As: float                 # Conductor section [mm²]
    conductor_dim: Dict       # Diameter (circular) or width/thickness (rectangular)
    L_mean: float             # Mean length of turn [m]
    K_w: float                # Winding factor
    R_s: float                # Stator resistance per phase [Ω]
    copper_losses: float      # Copper losses [W]
    
    def __str__(self):
        dim_str = ""
        if self.conductor_type == 'circular':
            dim_str = f"║ Conductor diameter      : {self.conductor_dim['diameter']:>10.2f} mm  ║"
        else:
            dim_str = f"║ Conductor width         : {self.conductor_dim['width']:>10.2f} mm  ║\n"
            dim_str += f"    ║ Conductor thickness     : {self.conductor_dim['thickness']:>10.2f} mm  ║"
        
        return f"""
    ╔══════════════════════════════════════════════╗
    ║         STATOR WINDING DESIGN                ║
    ╠══════════════════════════════════════════════╣
    ║ Turns per phase TPH     : {self.TPH:>10}           ║
    ║ Total conductors Zs     : {self.Zs:>10}           ║
    ║ Conductor type          : {self.conductor_type:>10}       ║
    ║ Conductor section As    : {self.As:>10.2f} mm² ║
    {dim_str}
    ║ Mean turn length L_mean : {self.L_mean:>10.3f} m   ║
    ║ Winding factor Kw       : {self.K_w:>10.4f}       ║
    ║ Stator R @ 20°C         : {self.R_s:>10.6f} Ω/ph  ║
    ║ Copper losses Pjs       : {self.copper_losses:>10.2f} W   ║
    ╚══════════════════════════════════════════════╝"""


@dataclass
class ElectricalParameters:
    """Electrical parameters"""
    P_elec: float             # Electrical power [kW]
    n_s: float                # Synchronous speed [rpm]
    n_s_rps: float            # Synchronous speed [rps]
    I_s: float                # Phase current [A]
    E_ph: float               # Phase voltage [V]
    cos_phi: float            # Power factor
    efficiency: float         # Efficiency
    
    def __str__(self):
        return f"""
    ╔══════════════════════════════════════════════╗
    ║       ELECTRICAL PARAMETERS                  ║
    ╠══════════════════════════════════════════════╣
    ║ Electrical power P_elec : {self.P_elec:>10.2f} kW  ║
    ║ Synchronous speed n_s   : {self.n_s:>10.2f} rpm ║
    ║ Synchronous speed n_s   : {self.n_s_rps:>10.2f} rps ║
    ║ Phase current I_s       : {self.I_s:>10.2f} A   ║
    ║ Phase voltage E_ph      : {self.E_ph:>10.2f} V   ║
    ║ Power factor cos(φ)     : {self.cos_phi:>10.4f}       ║
    ║ Efficiency η            : {self.efficiency:>10.4f}       ║
    ╚══════════════════════════════════════════════╝"""


# ═══════════════════════════════════════════════════════════════════════════
# INTERPOLATION UTILITIES
# ═══════════════════════════════════════════════════════════════════════════

class Interpolator:
    """Linear interpolation utilities for empirical tables"""
    
    @staticmethod
    def linear_interpolate(x: float, x_values: list, y_values: list) -> float:
        """Linear 1D interpolation"""
        if x <= x_values[0]:
            return y_values[0]
        if x >= x_values[-1]:
            return y_values[-1]
        
        for i in range(len(x_values) - 1):
            if x_values[i] <= x <= x_values[i + 1]:
                x0, x1 = x_values[i], x_values[i + 1]
                y0, y1 = y_values[i], y_values[i + 1]
                return y0 + (y1 - y0) * (x - x0) / (x1 - x0)
        
        return y_values[-1]
    
    @staticmethod
    def get_table_value(power_kw: float, table: Dict, default: float = None) -> float:
        """Get or interpolate value from empirical table"""
        power_list = sorted(table.keys())
        
        if power_kw in table:
            return table[power_kw]
        
        value_list = [table[p] for p in power_list]
        return Interpolator.linear_interpolate(power_kw, power_list, value_list)


# ═══════════════════════════════════════════════════════════════════════════
# MAIN STATOR DESIGN CLASS
# ═══════════════════════════════════════════════════════════════════════════

class StatorDesign:
    """Complete stator design for 3-phase asynchronous motor"""
    
    def __init__(self, specs: MotorSpecifications):
        self.specs = specs
        self.electrical: Optional[ElectricalParameters] = None
        self.geometry: Optional[StatorGeometry] = None
        self.slots: Optional[StatorSlots] = None
        self.winding: Optional[StatorWinding] = None
        self._validation_messages = []
        self.slot_pitch_validation: Optional[Dict] = None
        self.tooth_validation: Optional[Dict] = None
        self._forward_design_targets: Optional[Dict[str, float]] = None

    def _select_forward_design_flux_densities(self) -> Dict[str, float]:
        """Select tooth and yoke flux-density targets for Forward Design.

        Rules:
        - Tooth allowed range: 1.4–2.1 T
        - Yoke allowed range: 1.4–1.7 T
        - Do not exceed selected steel Max_Design_B
        - Yoke value should be smaller than tooth value
        - Tooth: may use the maximum allowable for the steel (within range)
        - Yoke: choose as low as practical (to reduce magnetizing current) while respecting constraints
        """
        steel_data = getattr(self.specs, 'steel_data', None) or {}
        steel_max_b = steel_data.get('Max_Design_B', None)
        try:
            steel_max_b_val = float(steel_max_b) if steel_max_b is not None else None
        except (TypeError, ValueError):
            steel_max_b_val = None

        tooth_min, tooth_max = 1.4, 2.1
        yoke_min, yoke_max = 1.4, 1.7

        if steel_max_b_val is None:
            # Conservative defaults if no steel is provided
            b_tooth = 1.7
            b_yoke = 1.3
        else:
            # Tooth: as high as allowed by steel and tooth range
            b_tooth = min(tooth_max, steel_max_b_val)
            b_tooth = max(0.0, b_tooth)

            # Yoke: low end to reduce magnetizing current, but not above steel and not above yoke range
            b_yoke = min(yoke_min, yoke_max, steel_max_b_val)
            b_yoke = max(0.0, b_yoke)

            # Ensure yoke < tooth
            if b_yoke >= b_tooth:
                b_yoke = max(0.0, b_tooth - 0.05)

        targets = {
            'B_tooth_target': float(b_tooth),
            'B_yoke_target': float(b_yoke),
        }

        if steel_max_b_val is not None:
            targets['steel_max_B'] = float(steel_max_b_val)

        self._forward_design_targets = targets
        return targets

    def _get_yoke_flux_density_for_outer_diameter(self) -> float:
        """Return yoke flux density to use in outer diameter (core) sizing."""
        method = getattr(self.specs, 'slot_method', 'semi_open_slot') or 'semi_open_slot'
        if method in ('semi_open_slot', 'semi_open_slot_new', 'open_slot'):
            targets = self._forward_design_targets or self._select_forward_design_flux_densities()
            return float(targets.get('B_yoke_target', 1.3))
        return 1.3

    def _compute_tooth_width_from_flux(self, Ss: int, B_tooth_T: float) -> float:
        """Compute tooth width using the selected tooth flux density target.

        w_tooth = Phi / (Li * (Ss/P) * B_tooth)
        """
        if self.geometry is None:
            self.calculate_main_dimensions()
        if self.geometry is None:
            return 0.0
        if Ss <= 0:
            return 0.0
        P = self.specs.poles
        Li = float(getattr(self.geometry, 'Li', 0.0) or 0.0)
        if Li <= 0:
            Ls = float(getattr(self.geometry, 'Ls', 0.0) or 0.0)
            ki = float(getattr(self.geometry, 'ki', 1.0) or 1.0)
            Li = (Ls * ki) if (Ls > 0 and ki > 0) else 0.0
        Phi = float(getattr(self.geometry, 'flux_per_pole', 0.0) or 0.0)
        denom = (Li * (float(Ss) / float(P)) * float(B_tooth_T)) if (P > 0 and Li > 0 and B_tooth_T > 0) else 0.0
        return (Phi / denom) if denom > 0 else 0.0

    def _design_slots_unified(self, conductor_mode: str) -> Tuple[StatorWinding, StatorSlots]:
        """Unified slot design for Semi-open and Open slot methods.

        conductor_mode:
          - 'wire'  : circular wire with strands + SWG standardization
          - 'strip' : rectangular strip (open-slot abaque)
        """
        if self.geometry is None:
            self.calculate_main_dimensions()
        if self.electrical is None:
            self.calculate_electrical_parameters()

        TPH, Zs, Nc, Ss, q = self.calculate_turns_and_conductors()
        P = self.specs.poles

        # Ensure flux-density targets are available (steel-based)
        targets = self._forward_design_targets or self._select_forward_design_flux_densities()
        B_tooth = float(targets.get('B_tooth_target', 1.7))

        # Forward step values (phi_max and wst) are used by the slot-dim solver
        if Ss > 0:
            self._compute_forward_design_phi_max_and_wst(Ss)

        # Open-slot next step: derive wst1 and bs at bore (D)
        # (Requested naming: tooth width from maximum flux -> wst1, slot width -> bs)
        if conductor_mode == 'strip' and Ss > 0 and self.geometry is not None:
            if not hasattr(self.geometry, 'validation_info') or self.geometry.validation_info is None:
                self.geometry.validation_info = {}
            geom_val_local = self.geometry.validation_info
            D_m = float(getattr(self.geometry, 'D', 0.0) or 0.0)
            slot_pitch_D_mm = (math.pi * D_m / float(Ss)) * 1000.0 if D_m > 0 else 0.0
            wst1_mm = float(geom_val_local.get('wst_mm', 0.0) or 0.0)
            bs_mm = slot_pitch_D_mm - wst1_mm
            geom_val_local['open_slot_slot_pitch_D_mm'] = slot_pitch_D_mm
            geom_val_local['open_slot_wst1_mm'] = wst1_mm
            geom_val_local['open_slot_bs_mm'] = bs_mm

        # Conductor sizing
        if conductor_mode == 'strip':
            self._compute_open_slot_strip_sizing(Ss=Ss, Nc=Nc)
            conductor_type = 'rectangular'
            geom_val = getattr(self.geometry, 'validation_info', None) or {}
            t_std = float(geom_val.get('strip_thickness_mm_std', 0.0) or 0.0)
            w_std = float(geom_val.get('strip_width_mm_std', 0.0) or 0.0)
            As_mm2 = float(geom_val.get('ac_std_mm2', 0.0) or 0.0)
            conductor_dim = {'width': w_std, 'thickness': t_std}

            # Open-slot next step (new logic): choose Zsw (conductors across width), then compute bs and hs
            if self.geometry is not None:
                if not hasattr(self.geometry, 'validation_info') or self.geometry.validation_info is None:
                    self.geometry.validation_info = {}
                geom_val_local = self.geometry.validation_info

                voltage_v = float(getattr(self.specs, 'voltage_v', 0.0) or 0.0)
                power_kw = float(getattr(self.specs, 'power_kw', 0.0) or 0.0)
                # High-voltage/high-power Open slot variant requested by user:
                #   Apply when V>600 or P>372.85 kW.
                open_slot_hv = (voltage_v > 600.0) or (power_kw > 372.85)


                # -----------------------------------------------------------------
                # HV/Higher-power Open slot logic (requested):
                # - Compute wst1 from phi_max and B_tooth_max (minimum tooth width at bore)
                # - Compute required slot area from strip area and Nc/slot using fill factor = 0.7
                # - Solve bs/hs using A_slot, G_ratio in [3,5], and pitch at bore constraint
                # - Compute wst2 at D+2hs and B_tooth at top of slot
                # -----------------------------------------------------------------
                if open_slot_hv:
                    # Targets and material limits
                    targets_hv = self._forward_design_targets or self._select_forward_design_flux_densities()
                    try:
                        steel_max_B = float(targets_hv.get('steel_max_B', 0.0) or 0.0)
                    except (TypeError, ValueError):
                        steel_max_B = 0.0
                    B_tooth_max = steel_max_B if steel_max_B > 0 else float(targets_hv.get('B_tooth_target', 1.7) or 1.7)

                    # Maximum tooth flux
                    P = int(getattr(self.specs, 'poles', 0) or 0)
                    flux_pole = float(getattr(self.geometry, 'flux_per_pole', 0.0) or 0.0)
                    phi_max = (flux_pole * math.sin(math.pi * float(P) / (2.0 * float(Ss)))) if (Ss > 0 and P > 0 and flux_pole > 0) else 0.0
                    geom_val_local['open_slot_phi_max_used_Wb'] = phi_max

                    # Effective iron length Li
                    Li_used_for_checks = float(getattr(self.geometry, 'Li', 0.0) or 0.0)
                    if Li_used_for_checks <= 0:
                        Ls_fallback = float(getattr(self.geometry, 'Ls', 0.0) or 0.0)
                        ki_fallback = float(getattr(self.geometry, 'ki', 1.0) or 1.0)
                        Li_used_for_checks = (Ls_fallback * ki_fallback) if (Ls_fallback > 0 and ki_fallback > 0) else 0.0
                    geom_val_local['open_slot_Li_used_m'] = Li_used_for_checks

                    # wst1 (minimum tooth width at bore) from B_tooth_max
                    wst1_m = (phi_max / (B_tooth_max * Li_used_for_checks)) if (phi_max > 0 and B_tooth_max > 0 and Li_used_for_checks > 0) else 0.0
                    wst1_mm = wst1_m * 1000.0

                    # slot pitch at bore and pitch-based bs
                    D_m = float(getattr(self.geometry, 'D', 0.0) or 0.0)
                    slot_pitch_D_mm = (math.pi * D_m / float(Ss)) * 1000.0 if (D_m > 0 and Ss > 0) else 0.0
                    bs_pitch_mm = slot_pitch_D_mm - wst1_mm

                    # Required slot area from strip section and conductors per slot (fill factor = 0.7)
                    nc_slot = int(Nc) if Nc is not None else 0
                    A_strip_mm2 = float(w_std) * float(t_std) if (w_std > 0 and t_std > 0) else 0.0
                    fill_factor_slot = 0.7
                    Tac_mm2_hv = A_strip_mm2 * float(max(1, nc_slot))
                    A_slot_mm2_hv = (Tac_mm2_hv / fill_factor_slot) if fill_factor_slot > 0 else 0.0

                    # Solve bs/hs with user-preferred strategy:
                    # - Keep wst1 as the minimum tooth width from B_max (do NOT decrease it).
                    # - If a ratio-limited solution (hs/bs<=5) would require increasing wst1 (pitch_err<=0),
                    #   it is acceptable (B decreases). Otherwise, use pitch-fixed Option A and warn if ratio>5.
                    area_req_mm2 = float(A_slot_mm2_hv)

                    # Candidate 1 (Option A): pitch-fixed + area>=required. Ratio may exceed 5.
                    solver_mode = 'pitch_fixed'
                    bs_pf_mm = max(0.0, float(bs_pitch_mm))
                    hs_pf_mm = 0.0
                    if area_req_mm2 > 0 and bs_pf_mm > 0:
                        hs_pf_mm = max((area_req_mm2 / bs_pf_mm), (3.0 * bs_pf_mm))
                    ratio_pf = (hs_pf_mm / bs_pf_mm) if bs_pf_mm > 0 else float('inf')
                    area_geom_pf_mm2 = bs_pf_mm * hs_pf_mm
                    area_excess_pf_mm2 = area_geom_pf_mm2 - area_req_mm2 if area_req_mm2 > 0 else float('nan')
                    ratio_limit_violated = (ratio_pf > 5.0)

                    # Candidate 2: enforce ratio=5 and exact area, adjust wst1 only if needed.
                    bs_fb_mm = math.sqrt(area_req_mm2 / 5.0) if area_req_mm2 > 0 else 0.0
                    hs_fb_mm = 5.0 * bs_fb_mm
                    pitch_err_fb_mm = (bs_fb_mm + wst1_mm) - slot_pitch_D_mm

                    # Default to Option A.
                    bs_mm_best = bs_pf_mm
                    hs_mm_best = hs_pf_mm
                    pitch_err_mm = 0.0
                    g_ratio = ratio_pf
                    area_geom_mm2 = area_geom_pf_mm2
                    area_excess_mm2 = area_excess_pf_mm2
                    solved_exact = (area_req_mm2 > 0 and bs_pf_mm > 0 and not ratio_limit_violated)

                    # Use fallback only when pitch_err<=0 (means we can increase wst1 to close pitch).
                    if area_req_mm2 > 0 and bs_fb_mm > 0 and pitch_err_fb_mm <= 1e-12:
                        solver_mode = 'fallback_increase_wst1'
                        bs_mm_best = bs_fb_mm
                        hs_mm_best = hs_fb_mm
                        pitch_err_mm = float(pitch_err_fb_mm)
                        g_ratio = 5.0
                        area_geom_mm2 = bs_mm_best * hs_mm_best
                        area_excess_mm2 = area_geom_mm2 - area_req_mm2
                        ratio_limit_violated = False
                        solved_exact = True

                        # Suggested tooth width increase to satisfy pitch exactly
                        wst1_required_mm = float(slot_pitch_D_mm) - float(bs_mm_best)
                        delta_wst1_mm = wst1_required_mm - float(wst1_mm)
                        geom_val_local['open_slot_wst1_required_mm'] = float(wst1_required_mm)
                        geom_val_local['open_slot_wst1_delta_mm'] = float(delta_wst1_mm)

                        # Bore B if tooth width is increased (should be <= B_tooth_max)
                        denom_bore_req = (wst1_required_mm / 1000.0) * Li_used_for_checks
                        B_tooth_bore_if_wst1_req = (phi_max / denom_bore_req) if denom_bore_req > 0 else float('inf')
                        geom_val_local['open_slot_B_tooth_bore_if_wst1_required_T'] = float(B_tooth_bore_if_wst1_req)

                    # Record geometry values
                    geom_val_local['open_slot_mode'] = 'hv_pitch_area_ratio'
                    geom_val_local['open_slot_fill_factor_used'] = float(fill_factor_slot)
                    geom_val_local['open_slot_A_strip_mm2_used'] = float(A_strip_mm2)
                    geom_val_local['open_slot_Nc_per_slot'] = int(nc_slot)
                    geom_val_local['open_slot_Tac_mm2_used'] = float(Tac_mm2_hv)
                    geom_val_local['A_slot_mm2'] = float(A_slot_mm2_hv)
                    geom_val_local['fill_factor'] = float(fill_factor_slot)
                    geom_val_local['Tac_mm2'] = float(Tac_mm2_hv)

                    geom_val_local['open_slot_slot_pitch_D_mm'] = float(slot_pitch_D_mm)
                    geom_val_local['open_slot_wst1_mm'] = float(wst1_mm)
                    geom_val_local['open_slot_bs_mm'] = float(bs_mm_best)
                    geom_val_local['open_slot_hs_mm'] = float(hs_mm_best)
                    geom_val_local['open_slot_ratio_hs_bs'] = float(hs_mm_best / bs_mm_best) if bs_mm_best > 0 else float('inf')
                    geom_val_local['open_slot_pitch_err_mm'] = float(pitch_err_mm)
                    geom_val_local['open_slot_solved_exact'] = float(1.0 if solved_exact else 0.0)
                    geom_val_local['open_slot_solver_mode'] = str(solver_mode)

                    # Area diagnostics
                    geom_val_local['open_slot_A_slot_required_mm2'] = float(area_req_mm2)
                    geom_val_local['open_slot_A_slot_geom_mm2'] = float(area_geom_mm2)
                    geom_val_local['open_slot_area_excess_mm2'] = float(area_excess_mm2) if math.isfinite(area_excess_mm2) else float('nan')
                    geom_val_local['open_slot_ratio_limit_violated'] = float(1.0 if ratio_limit_violated else 0.0)

                    # Flux density at bore (should be ~B_tooth_max by construction when wst1>0)
                    tooth_w_bore_m = (wst1_mm / 1000.0) if wst1_mm > 0 else 0.0
                    denom_bore = tooth_w_bore_m * Li_used_for_checks
                    B_tooth_bore = (phi_max / denom_bore) if denom_bore > 0 else float('inf')
                    geom_val_local['open_slot_B_tooth_bore_T'] = float(B_tooth_bore)
                    geom_val_local['open_slot_steel_max_B_T'] = float(steel_max_B)
                    geom_val_local['open_slot_B_tooth_max_used_T'] = float(B_tooth_max)

                    # Tooth width at top of slot and B_tooth at top
                    hs_m = float(hs_mm_best) / 1000.0 if hs_mm_best > 0 else 0.0

                    # Keep geometry.h_slot consistent with HV Open-slot hs
                    # (Outer diameter already prefers open_slot_hs_mm when slot_method=='open_slot',
                    #  but geometry.h_slot is used elsewhere and should match the slot design result.)
                    if hs_m > 0 and self.geometry is not None:
                        self.geometry.h_slot = hs_m

                    slot_pitch_D_plus_2hs_mm = (math.pi * (D_m + 2.0 * hs_m) / float(Ss)) * 1000.0 if (D_m > 0 and Ss > 0) else 0.0
                    wst2_mm = float(slot_pitch_D_plus_2hs_mm) - float(bs_mm_best)
                    geom_val_local['open_slot_slot_pitch_D_plus_2hs_mm'] = float(slot_pitch_D_plus_2hs_mm)
                    geom_val_local['open_slot_wst2_mm'] = float(wst2_mm)
                    wst2_m = float(wst2_mm) / 1000.0 if wst2_mm > 0 else 0.0
                    denom_top = wst2_m * Li_used_for_checks
                    B_tooth_top_slot = (phi_max / denom_top) if denom_top > 0 else float('inf')
                    geom_val_local['open_slot_B_tooth_top_slot_T'] = float(B_tooth_top_slot)

                else:
                    # -----------------------------------------------------------------
                    # Existing Open-slot packing logic (Zsw/Zsh) for normal machines
                    # -----------------------------------------------------------------
                    # Voltage-based insulation thickness (as requested)
                    if voltage_v <= 1000.0:
                        insulation_t_mm = 3.4
                        insulation_standard = "≤ 1000 V"
                    elif voltage_v <= 3300.0:
                        insulation_t_mm = 4.5
                        insulation_standard = "3300 V"
                    elif voltage_v <= 6600.0:
                        insulation_t_mm = 6.0
                        insulation_standard = "6600 V"
                    else:
                        insulation_t_mm = 8.0
                        insulation_standard = "11000 V"

                    bs_mm = float(geom_val_local.get('open_slot_bs_mm', 0.0) or 0.0)
                    available_width_mm = bs_mm - insulation_t_mm

                    # Packing pitch across width direction uses (t_std + 0.5 mm insulation), per request
                    pitch_width_mm = t_std + 0.5 if t_std > 0 else 0.0
                    n_across_float = (available_width_mm / pitch_width_mm) if pitch_width_mm > 0 else 0.0

                    # Rounding rule: always round down, except when extremely close to next integer
                    rounding_tol = 1e-6
                    n_across_int = int(math.floor(n_across_float + rounding_tol)) if n_across_float > 0 else 0

                    geom_val_local['open_slot_insulationT_mm'] = insulation_t_mm
                    geom_val_local['open_slot_insulation_standard'] = insulation_standard

                    # Choose Zsw based on hs/bs ratio target range [3, 5] (flux-density validation added later)
                    nc_slot = int(Nc) if Nc is not None else 0
                    # Material maximum flux density constraint
                    targets = self._forward_design_targets or self._select_forward_design_flux_densities()
                    try:
                        steel_max_B = float(targets.get('steel_max_B', 0.0) or 0.0)
                    except (TypeError, ValueError):
                        steel_max_B = 0.0
                    enforce_steel_max = steel_max_B > 0

                    P = int(getattr(self.specs, 'poles', 0) or 0)
                    Li_m = float(getattr(self.geometry, 'Li', 0.0) or 0.0)
                    if Li_m <= 0:
                        Ls_m = float(getattr(self.geometry, 'Ls', 0.0) or 0.0)
                        ki = float(getattr(self.geometry, 'ki', 1.0) or 1.0)
                        Li_m = (Ls_m * ki) if (Ls_m > 0 and ki > 0) else 0.0
                    flux_pole = float(getattr(self.geometry, 'flux_per_pole', 0.0) or 0.0)
                    D_mm = float(getattr(self.geometry, 'D', 0.0) or 0.0) * 1000.0

                    # Maximum tooth flux (used for bore/tooth checks)
                    # phi_max = flux_pole * sin(pi * P / (2 * Ss))
                    phi_max = (flux_pole * math.sin(math.pi * float(P) / (2.0 * float(Ss)))) if (Ss > 0 and P > 0 and flux_pole > 0) else 0.0

                    best = None
                    best_check = None
                    any_ok_B_bore = False
                    any_ok_ratio = False
                    any_ok_both = False
                    for zsw in range(1, max(1, nc_slot) + 1):
                        zsh = int(math.ceil(float(nc_slot) / float(zsw))) if nc_slot > 0 else 0
                        bs_mm_candidate = float(zsw) * (t_std + 0.05) + insulation_t_mm
                        hs_mm_candidate = float(zsh) * (w_std + 0.5) + 4.0 + 1.5
                        ratio = (hs_mm_candidate / bs_mm_candidate) if bs_mm_candidate > 0 else float('inf')

                        # Validation at bore (top priority):
                        #   slot_pitch(D) = pi * D / Ss
                        #   w_tooth_bore = slot_pitch(D) - bs
                        #   B_tooth_bore = phi_max / (w_tooth_bore * Li)
                        slot_pitch_D_mm = (math.pi * D_mm / float(Ss)) if (Ss > 0 and D_mm > 0) else 0.0
                        tooth_w_bore_mm = slot_pitch_D_mm - bs_mm_candidate
                        tooth_w_bore_m = (tooth_w_bore_mm / 1000.0) if tooth_w_bore_mm > 0 else 0.0
                        denom_bore = tooth_w_bore_m * Li_m
                        B_tooth_bore = (phi_max / denom_bore) if denom_bore > 0 else float('inf')

                        ok_ratio = (3.0 <= ratio <= 5.0)
                        ok_B = (B_tooth_bore <= steel_max_B) if enforce_steel_max else True

                        any_ok_B_bore = any_ok_B_bore or bool(ok_B)
                        any_ok_ratio = any_ok_ratio or bool(ok_ratio)
                        any_ok_both = any_ok_both or (bool(ok_B) and bool(ok_ratio))

                        # Prefer satisfying BOTH constraints when possible.
                        # Fallback ordering when not possible:
                        #   1) satisfy bore B constraint (ok_B)
                        #   2) satisfy aspect ratio (ok_ratio)
                        #   3) minimize B exceedance, ratio deviation, then smaller hs
                        B_excess = max(0.0, (B_tooth_bore - steel_max_B)) if (enforce_steel_max and math.isfinite(B_tooth_bore)) else 0.0
                        ratio_dev = abs(ratio - 4.0) if math.isfinite(ratio) else 1e9

                        cand = (
                            0.0 if ok_B else 1.0,
                            0.0 if ok_ratio else 1.0,
                            B_excess,
                            ratio_dev,
                            hs_mm_candidate,
                            zsw,
                        )
                        if best is None or cand < best:
                            best = cand
                            best_check = {
                                'Zsw': zsw,
                                'Zsh': zsh,
                                'bs_mm': bs_mm_candidate,
                                'hs_mm': hs_mm_candidate,
                                'ratio_hs_bs': ratio,
                                'slot_pitch_D_mm': slot_pitch_D_mm,
                                'tooth_w_bore_mm': tooth_w_bore_mm,
                                'B_tooth_bore': B_tooth_bore,
                                'phi_max': phi_max,
                                'steel_max_B': steel_max_B,
                                'ok_ratio': ok_ratio,
                                'ok_B_bore': ok_B,
                                'enforce_steel_max': enforce_steel_max,
                            }

                    if not best_check:
                        best_check = {
                            'Zsw': 1,
                            'Zsh': nc_slot,
                            'bs_mm': insulation_t_mm,
                            'hs_mm': float(nc_slot) * (w_std + 0.5) + 5.5,
                            'ratio_hs_bs': float('inf'),
                            'D_1_3_mm': float('nan'),
                            'slot_pitch_1_3_mm': float('nan'),
                            'tooth_w_1_3_mm': float('nan'),
                            'tooth_area_m2': 0.0,
                            'B_tooth_1_3': float('inf'),
                            'Bmax_tooth': float('inf'),
                            'steel_max_B': steel_max_B,
                            'ok_ratio': False,
                            'ok_Bmax': False,
                            'enforce_steel_max': enforce_steel_max,
                        }

                    zsw_best = int(best_check['Zsw'])
                    zsh_best = int(best_check['Zsh'])
                    bs_mm_best = float(best_check['bs_mm'])
                    hs_mm_best = float(best_check['hs_mm'])
                    ratio_best = float(best_check['ratio_hs_bs'])

                    geom_val_local['open_slot_Zsw'] = int(zsw_best)
                    geom_val_local['open_slot_Zsh'] = int(zsh_best)
                    geom_val_local['open_slot_bs_mm'] = float(bs_mm_best)
                    geom_val_local['open_slot_hs_mm'] = float(hs_mm_best)
                    geom_val_local['open_slot_ratio_hs_bs'] = float(ratio_best)
                    geom_val_local['open_slot_Nc_per_slot'] = nc_slot

                    # Store check-phase results for GUI
                    geom_val_local['open_slot_slot_pitch_D_mm'] = best_check.get('slot_pitch_D_mm')
                    geom_val_local['open_slot_wst1_mm'] = best_check.get('tooth_w_bore_mm')
                    geom_val_local['open_slot_B_tooth_bore_T'] = best_check.get('B_tooth_bore')
                    geom_val_local['open_slot_phi_max_used_Wb'] = best_check.get('phi_max')
                    geom_val_local['open_slot_steel_max_B_T'] = best_check.get('steel_max_B')
                    geom_val_local['open_slot_check_ok_ratio'] = 1.0 if best_check.get('ok_ratio') else 0.0
                    geom_val_local['open_slot_check_ok_Bbore'] = 1.0 if best_check.get('ok_B_bore') else 0.0
                    geom_val_local['open_slot_check_enforce_steel_max'] = 1.0 if best_check.get('enforce_steel_max') else 0.0

                    # Expose whether an actually-feasible ("optimal") point exists
                    geom_val_local['open_slot_any_ok_ratio'] = 1.0 if any_ok_ratio else 0.0
                    geom_val_local['open_slot_any_ok_Bbore'] = 1.0 if any_ok_B_bore else 0.0
                    geom_val_local['open_slot_any_ok_both'] = 1.0 if any_ok_both else 0.0

                    # Tooth widths after validation (reporting):
                    # - wst2 comes from slot pitch at D+2hs and constant bs
                    # Checks use Li (effective iron length), not Ls.
                    try:
                        phi_max = float(geom_val_local.get('phi_max', 0.0) or 0.0)
                    except (TypeError, ValueError):
                        phi_max = 0.0

                    # Effective iron length Li (fallback from Ls*ki)
                    Li_used_for_checks = float(getattr(self.geometry, 'Li', 0.0) or 0.0)
                    if Li_used_for_checks <= 0:
                        Ls_fallback = float(getattr(self.geometry, 'Ls', 0.0) or 0.0)
                        ki_fallback = float(getattr(self.geometry, 'ki', 1.0) or 1.0)
                        Li_used_for_checks = (Ls_fallback * ki_fallback) if (Ls_fallback > 0 and ki_fallback > 0) else 0.0
                    geom_val_local['open_slot_Li_used_m'] = Li_used_for_checks

                    # Slot pitch at bore D (for reporting)
                    D_m = float(getattr(self.geometry, 'D', 0.0) or 0.0)
                    slot_pitch_D_mm = (math.pi * D_m / float(Ss)) * 1000.0 if (D_m > 0 and Ss > 0) else 0.0
                    geom_val_local['open_slot_slot_pitch_D_mm'] = slot_pitch_D_mm

                    # Tooth width at top of slot: compute slot pitch at (D + 2*hs), then wst2 = slot_pitch - bs
                    hs_m = float(hs_mm_best) / 1000.0 if hs_mm_best > 0 else 0.0
                    slot_pitch_D_plus_2hs_mm = (math.pi * (D_m + 2.0 * hs_m) / float(Ss)) * 1000.0 if (D_m > 0 and Ss > 0) else 0.0
                    wst2_mm = slot_pitch_D_plus_2hs_mm - float(bs_mm_best)
                    geom_val_local['open_slot_slot_pitch_D_plus_2hs_mm'] = slot_pitch_D_plus_2hs_mm
                    geom_val_local['open_slot_wst2_mm'] = wst2_mm

                    # Check at top of slot: B_tooth_top_slot = flux_max / (wst2 * Li)
                    # flux_max here is phi_max (maximum tooth flux)
                    wst2_m = float(wst2_mm) / 1000.0 if wst2_mm > 0 else 0.0
                    denom_top = wst2_m * Li_used_for_checks
                    B_tooth_top_slot = (phi_max / denom_top) if denom_top > 0 else float('inf')
                    geom_val_local['open_slot_B_tooth_top_slot_T'] = B_tooth_top_slot
        else:
            # wire mode
            # Create a minimal slots placeholder for internal helpers that rely on Ss/Nc.
            if self.slots is None:
                self.slots = StatorSlots(
                    Ss=Ss,
                    q=q,
                    Nc=Nc,
                    b_slot=0.0,
                    h_slot=0.0,
                    A_slot=0.0,
                    slot_pitch=0.0,
                    tooth_width_D13=0.0,
                    B_tooth=0.0,
                    B_tooth_max=0.0,
                    D_13=0.0,
                )
            else:
                self.slots.Ss = Ss
                self.slots.q = q
                self.slots.Nc = Nc

            self._compute_forward_design_wire_sizing()
            conductor_type = 'circular'
            geom_val = getattr(self.geometry, 'validation_info', None) or {}
            As_mm2 = float(geom_val.get('ac_std_mm2', 0.0) or 0.0)
            d_std = float(geom_val.get('strand_diameter_mm_std', 0.0) or 0.0)
            strands = int(float(geom_val.get('number_of_strands', 1.0) or 1.0))
            conductor_dim = {
                'diameter': d_std,
                'strands': strands,
                'oversize_pct': float(geom_val.get('oversize_pct_std', 0.0) or 0.0),
                'area_actual': As_mm2,
            }

            # Low-voltage/low-power Open slot: use random-wound conductor sizing (same as wire mode)
            # but compute (bs, hs) using the same pitch/area/ratio solver as HV.
            slot_method = getattr(self.specs, 'slot_method', 'semi_open_slot') or 'semi_open_slot'
            if slot_method == 'open_slot' and self.geometry is not None and Ss > 0:
                if not hasattr(self.geometry, 'validation_info') or self.geometry.validation_info is None:
                    self.geometry.validation_info = {}
                geom_val_local = self.geometry.validation_info

                # Targets and material limits
                targets_lv = self._forward_design_targets or self._select_forward_design_flux_densities()
                try:
                    steel_max_B = float(targets_lv.get('steel_max_B', 0.0) or 0.0)
                except (TypeError, ValueError):
                    steel_max_B = 0.0
                B_tooth_max = steel_max_B if steel_max_B > 0 else float(targets_lv.get('B_tooth_target', 1.7) or 1.7)

                # Maximum tooth flux
                P_local = int(getattr(self.specs, 'poles', 0) or 0)
                flux_pole = float(getattr(self.geometry, 'flux_per_pole', 0.0) or 0.0)
                phi_max = (flux_pole * math.sin(math.pi * float(P_local) / (2.0 * float(Ss)))) if (Ss > 0 and P_local > 0 and flux_pole > 0) else 0.0
                geom_val_local['open_slot_phi_max_used_Wb'] = phi_max

                # Effective iron length Li
                Li_used_for_checks = float(getattr(self.geometry, 'Li', 0.0) or 0.0)
                if Li_used_for_checks <= 0:
                    Ls_fallback = float(getattr(self.geometry, 'Ls', 0.0) or 0.0)
                    ki_fallback = float(getattr(self.geometry, 'ki', 1.0) or 1.0)
                    Li_used_for_checks = (Ls_fallback * ki_fallback) if (Ls_fallback > 0 and ki_fallback > 0) else 0.0
                geom_val_local['open_slot_Li_used_m'] = Li_used_for_checks

                # wst1 (minimum tooth width at bore) from B_tooth_max
                wst1_m = (phi_max / (B_tooth_max * Li_used_for_checks)) if (phi_max > 0 and B_tooth_max > 0 and Li_used_for_checks > 0) else 0.0
                wst1_mm = wst1_m * 1000.0

                # slot pitch at bore and pitch-based bs
                D_m = float(getattr(self.geometry, 'D', 0.0) or 0.0)
                slot_pitch_D_mm = (math.pi * D_m / float(Ss)) * 1000.0 if (D_m > 0 and Ss > 0) else 0.0
                bs_pitch_mm = slot_pitch_D_mm - wst1_mm

                # Slot area from random-wound copper area and fill factor (wire sizing already uses fill_factor=0.4)
                try:
                    fill_factor_slot = float(geom_val_local.get('fill_factor', 0.4) or 0.4)
                except (TypeError, ValueError):
                    fill_factor_slot = 0.4
                try:
                    Tac_mm2 = float(geom_val_local.get('Tac_mm2', 0.0) or 0.0)
                except (TypeError, ValueError):
                    Tac_mm2 = 0.0
                try:
                    A_slot_mm2_lv = float(geom_val_local.get('A_slot_mm2', 0.0) or 0.0)
                except (TypeError, ValueError):
                    A_slot_mm2_lv = 0.0

                # Solve bs/hs with user-preferred strategy (same as HV):
                # - Keep wst1 as the minimum tooth width from B_max (do NOT decrease it).
                # - If a ratio-limited solution (hs/bs<=5) would require increasing wst1 (pitch_err<=0),
                #   it is acceptable (B decreases). Otherwise, use pitch-fixed Option A and warn if ratio>5.
                area_req_mm2 = float(A_slot_mm2_lv)

                # Candidate 1 (Option A): pitch-fixed + area>=required. Ratio may exceed 5.
                solver_mode = 'pitch_fixed'
                bs_pf_mm = max(0.0, float(bs_pitch_mm))
                hs_pf_mm = 0.0
                if area_req_mm2 > 0 and bs_pf_mm > 0:
                    hs_pf_mm = max((area_req_mm2 / bs_pf_mm), (3.0 * bs_pf_mm))
                ratio_pf = (hs_pf_mm / bs_pf_mm) if bs_pf_mm > 0 else float('inf')
                area_geom_pf_mm2 = bs_pf_mm * hs_pf_mm
                area_excess_pf_mm2 = area_geom_pf_mm2 - area_req_mm2 if area_req_mm2 > 0 else float('nan')
                ratio_limit_violated = (ratio_pf > 5.0)

                # Candidate 2: enforce ratio=5 and exact area, adjust wst1 only if needed.
                bs_fb_mm = math.sqrt(area_req_mm2 / 5.0) if area_req_mm2 > 0 else 0.0
                hs_fb_mm = 5.0 * bs_fb_mm
                pitch_err_fb_mm = (bs_fb_mm + wst1_mm) - slot_pitch_D_mm

                # Default to Option A.
                bs_mm_best = bs_pf_mm
                hs_mm_best = hs_pf_mm
                pitch_err_mm = 0.0
                g_ratio = ratio_pf
                area_geom_mm2 = area_geom_pf_mm2
                area_excess_mm2 = area_excess_pf_mm2
                solved_exact = (area_req_mm2 > 0 and bs_pf_mm > 0 and not ratio_limit_violated)

                # Use fallback only when pitch_err<=0 (means we can increase wst1 to close pitch).
                if area_req_mm2 > 0 and bs_fb_mm > 0 and pitch_err_fb_mm <= 1e-12:
                    solver_mode = 'fallback_increase_wst1'
                    bs_mm_best = bs_fb_mm
                    hs_mm_best = hs_fb_mm
                    pitch_err_mm = float(pitch_err_fb_mm)
                    g_ratio = 5.0
                    area_geom_mm2 = bs_mm_best * hs_mm_best
                    area_excess_mm2 = area_geom_mm2 - area_req_mm2
                    ratio_limit_violated = False
                    solved_exact = True

                    # Suggested tooth width increase to satisfy pitch exactly
                    wst1_required_mm = float(slot_pitch_D_mm) - float(bs_mm_best)
                    delta_wst1_mm = wst1_required_mm - float(wst1_mm)
                    geom_val_local['open_slot_wst1_required_mm'] = float(wst1_required_mm)
                    geom_val_local['open_slot_wst1_delta_mm'] = float(delta_wst1_mm)

                    denom_bore_req = (wst1_required_mm / 1000.0) * Li_used_for_checks
                    B_tooth_bore_if_wst1_req = (phi_max / denom_bore_req) if denom_bore_req > 0 else float('inf')
                    geom_val_local['open_slot_B_tooth_bore_if_wst1_required_T'] = float(B_tooth_bore_if_wst1_req)

                geom_val_local['open_slot_mode'] = 'lv_pitch_area_ratio'
                geom_val_local['open_slot_fill_factor_used'] = float(fill_factor_slot)
                geom_val_local['open_slot_A_conductor_mm2_used'] = float(geom_val_local.get('ac_std_mm2', 0.0) or 0.0)
                geom_val_local['open_slot_Tac_mm2_used'] = float(Tac_mm2)
                geom_val_local['A_slot_mm2'] = float(A_slot_mm2_lv)

                geom_val_local['open_slot_slot_pitch_D_mm'] = float(slot_pitch_D_mm)
                geom_val_local['open_slot_wst1_mm'] = float(wst1_mm)
                geom_val_local['open_slot_bs_mm'] = float(bs_mm_best)
                geom_val_local['open_slot_hs_mm'] = float(hs_mm_best)
                geom_val_local['open_slot_ratio_hs_bs'] = float(hs_mm_best / bs_mm_best) if bs_mm_best > 0 else float('inf')
                geom_val_local['open_slot_pitch_err_mm'] = float(pitch_err_mm)
                geom_val_local['open_slot_solved_exact'] = float(1.0 if solved_exact else 0.0)
                geom_val_local['open_slot_solver_mode'] = str(solver_mode)

                geom_val_local['open_slot_A_slot_required_mm2'] = float(area_req_mm2)
                geom_val_local['open_slot_A_slot_geom_mm2'] = float(area_geom_mm2)
                geom_val_local['open_slot_area_excess_mm2'] = float(area_excess_mm2) if math.isfinite(area_excess_mm2) else float('nan')
                geom_val_local['open_slot_ratio_limit_violated'] = float(1.0 if ratio_limit_violated else 0.0)

                # Flux density at bore
                tooth_w_bore_m = (wst1_mm / 1000.0) if wst1_mm > 0 else 0.0
                denom_bore = tooth_w_bore_m * Li_used_for_checks
                B_tooth_bore = (phi_max / denom_bore) if denom_bore > 0 else float('inf')
                geom_val_local['open_slot_B_tooth_bore_T'] = float(B_tooth_bore)
                geom_val_local['open_slot_steel_max_B_T'] = float(steel_max_B)
                geom_val_local['open_slot_B_tooth_max_used_T'] = float(B_tooth_max)

                # Tooth width at top of slot and B_tooth at top
                hs_m = float(hs_mm_best) / 1000.0 if hs_mm_best > 0 else 0.0
                slot_pitch_D_plus_2hs_mm = (math.pi * (D_m + 2.0 * hs_m) / float(Ss)) * 1000.0 if (D_m > 0 and Ss > 0) else 0.0
                wst2_mm = float(slot_pitch_D_plus_2hs_mm) - float(bs_mm_best)
                geom_val_local['open_slot_slot_pitch_D_plus_2hs_mm'] = float(slot_pitch_D_plus_2hs_mm)
                geom_val_local['open_slot_wst2_mm'] = float(wst2_mm)
                wst2_m = float(wst2_mm) / 1000.0 if wst2_mm > 0 else 0.0
                denom_top = wst2_m * Li_used_for_checks
                B_tooth_top_slot = (phi_max / denom_top) if denom_top > 0 else float('inf')
                geom_val_local['open_slot_B_tooth_top_slot_T'] = float(B_tooth_top_slot)

                # Ensure geometry.h_slot stays aligned with open-slot hs for downstream steps
                if hs_m > 0:
                    self.geometry.h_slot = hs_m

        # Slot area in m^2 for geometry calculation
        geom_val = getattr(self.geometry, 'validation_info', None) or {}
        A_slot_mm2 = float(geom_val.get('A_slot_mm2', 0.0) or 0.0)
        slot_area_m2 = A_slot_mm2 * 1e-6

        # If Open-slot pitch/area/ratio mode computed hs explicitly, reuse it and skip the legacy quadratic model.
        slot_method = getattr(self.specs, 'slot_method', 'semi_open_slot') or 'semi_open_slot'
        open_slot_hs_mm = float(geom_val.get('open_slot_hs_mm', 0.0) or 0.0) if slot_method == 'open_slot' else 0.0
        open_slot_bs_mm = float(geom_val.get('open_slot_bs_mm', 0.0) or 0.0) if slot_method == 'open_slot' else 0.0
        open_slot_mode = str(geom_val.get('open_slot_mode', '')) if slot_method == 'open_slot' else ''

        # Tooth width using target B_tooth (default)
        tooth_width_m = self._compute_tooth_width_from_flux(Ss=Ss, B_tooth_T=B_tooth)

        if slot_method == 'open_slot' and open_slot_hs_mm > 0 and open_slot_bs_mm > 0 and open_slot_mode.endswith('pitch_area_ratio'):
            # Open-slot (LV/HV) pitch/area/ratio geometry: hs and bs are already solved.
            hs_m = open_slot_hs_mm / 1000.0
            D_start_m = float(self.geometry.D)
        else:
            # Slot height from quadratic area model (legacy/semi-open structure)
            D_start_m = float(self.geometry.D) + 0.010  # D + 2*5mm
            a = math.pi / float(Ss)
            b = (math.pi * D_start_m / float(Ss)) - tooth_width_m
            c = -slot_area_m2
            disc = max(0.0, b * b - 4.0 * a * c)
            hs_pos = (-b + math.sqrt(disc)) / (2.0 * a)
            hs_neg = (-b - math.sqrt(disc)) / (2.0 * a)
            hs_m = hs_pos if hs_pos > 0 else hs_neg
            if hs_m <= 0:
                hs_m = abs(hs_pos)

        # Widths and checks
        slot_pitch_mid = math.pi * (float(self.geometry.D) + hs_m) / float(Ss)
        slot_width_mid = slot_pitch_mid - tooth_width_m
        ratio_hs_to_mid_width = (hs_m / slot_width_mid) if slot_width_mid != 0 else float('inf')

        Li_m = float(getattr(self.geometry, 'Li', 0.0) or 0.0)
        if Li_m <= 0:
            Ls_m = float(getattr(self.geometry, 'Ls', 0.0) or 0.0)
            ki = float(getattr(self.geometry, 'ki', 1.0) or 1.0)
            Li_m = (Ls_m * ki) if (Ls_m > 0 and ki > 0) else 0.0
        B_tooth_actual = float(self.geometry.flux_per_pole) / (tooth_width_m * Li_m * (float(Ss) / float(P))) if (tooth_width_m > 0 and Li_m > 0) else 0.0
        B_tooth_max = B_tooth_actual * 1.5

        if slot_method == 'open_slot' and open_slot_hs_mm > 0 and open_slot_bs_mm > 0 and open_slot_mode.endswith('pitch_area_ratio'):
            ws1_m = open_slot_bs_mm / 1000.0
            ws2_m = open_slot_bs_mm / 1000.0
        else:
            ws1_m = (math.pi * D_start_m / float(Ss)) - tooth_width_m
            ws2_m = (math.pi * (float(self.geometry.D) + 2.0 * hs_m) / float(Ss)) - tooth_width_m

        # Store a unified validation snapshot for GUI
        self.tooth_validation = {
            'method': 'semi_open_slot' if conductor_mode != 'strip' else 'open_slot',
            'As_mm2': As_mm2,
            'copper_area_mm2': As_mm2 * float(Nc),
            'slot_area_mm2': A_slot_mm2,
            'tooth_width_m': tooth_width_m,
            'hs_m': hs_m,
            'ws1_m': ws1_m,
            'ws2_m': ws2_m,
            'slot_pitch_mid_m': slot_pitch_mid,
            'slot_width_mid_m': slot_width_mid,
            'ratio_hs_to_mid_width': ratio_hs_to_mid_width,
            'B_tooth': B_tooth_actual,
            'B_tooth_max': B_tooth_max,
            'P': P,
            'Ss': Ss,
            'q': q,
            'Nc': Nc,
            'B_tooth_target_used': B_tooth,
        }

        # Update geometry with h_slot
        self.geometry.h_slot = hs_m

        self.slots = StatorSlots(
            Ss=Ss,
            q=q,
            Nc=Nc,
            b_slot=ws1_m * 1000.0,
            h_slot=hs_m * 1000.0,
            A_slot=A_slot_mm2,
            slot_pitch=slot_pitch_mid,
            tooth_width_D13=tooth_width_m,
            B_tooth=B_tooth_actual,
            B_tooth_max=B_tooth_max,
            D_13=D_start_m,
        )

        # Slot-dimension step (bs1/bs2/hs2) uses wst and A_slot
        if Ss > 0:
            slot_method = getattr(self.specs, 'slot_method', 'semi_open_slot') or 'semi_open_slot'
            if slot_method == 'semi_open_slot_new':
                self._compute_semi_open_slot_new_geometry(Ss)
                try:
                    hs_total_mm = float((getattr(self.geometry, 'validation_info', {}) or {}).get('hs_total_mm', 0.0) or 0.0)
                except (TypeError, ValueError):
                    hs_total_mm = 0.0
                if hs_total_mm > 0:
                    # Keep slots.h_slot consistent with the (hs0+hs1+hs2) definition
                    self.slots.h_slot = float(hs_total_mm)
            else:
                self._compute_forward_design_slot_dimensions(Ss)

        # Mean turn length and resistance
        slot_pitch_D = math.pi * float(self.geometry.D) / float(P)
        L_mean = 2.0 * float(self.geometry.L) + 2.3 * slot_pitch_D + 0.24
        rho_copper = 1.68e-8
        rho_conductor = getattr(self.specs, 'conductor_rho_ohm_m', None)
        try:
            rho_conductor = float(rho_conductor) if rho_conductor is not None else None
        except Exception:
            rho_conductor = None
        rho_used = rho_conductor if (rho_conductor is not None and rho_conductor > 0) else rho_copper
        As_m2 = As_mm2 * 1e-6
        R_s = (rho_used * L_mean * int(round((Nc * Ss) / (2 * self.specs.phases)))) / As_m2 if As_m2 > 0 else float('inf')
        copper_losses = 3.0 * R_s * (float(self.electrical.I_s) ** 2) if math.isfinite(R_s) else float('inf')
        K_w = float(getattr(self, '_K_w', 0.0) or 0.0)
        if K_w <= 0:
            raise ValueError("Invalid K_w (not computed)")

        Zs_final = Nc * Ss
        TPH_final = int(round(Zs_final / (2 * self.specs.phases)))
        self.winding = StatorWinding(
            TPH=TPH_final,
            Zs=Zs_final,
            conductor_type=conductor_type,
            As=As_mm2,
            conductor_dim=conductor_dim,
            L_mean=L_mean,
            K_w=K_w,
            R_s=R_s,
            copper_losses=copper_losses,
        )

        # Conductor weight (material-aware):
        # Wcond = Lmt * Tph * 3 * As * density(g/cm^3) * 10^-3   [kg]
        # - Lmt: mean turn length (L_mean) [m]
        # - Tph: turns per phase (TPH)
        # - As: conductor section [mm^2]
        #   * semi_open_slot: ac(std)
        #   * open_slot: A_strip = w(std) * t(std)
        slot_method = getattr(self.specs, 'slot_method', 'semi_open_slot') or 'semi_open_slot'
        if not hasattr(self.geometry, 'validation_info') or self.geometry.validation_info is None:
            self.geometry.validation_info = {}
        val = self.geometry.validation_info

        As_for_weight_mm2 = 0.0
        As_source = 'unknown'
        if slot_method == 'open_slot':
            # HV/HP open-slot uses rectangular strip; normal open-slot uses random-wound wire (ac_std).
            if conductor_type == 'rectangular':
                As_for_weight_mm2 = float(w_std) * float(t_std) if (w_std > 0 and t_std > 0) else 0.0
                As_source = 'A_strip'
            else:
                As_for_weight_mm2 = float(val.get('ac_std_mm2', As_mm2) or 0.0)
                As_source = 'ac_std'
        else:
            As_for_weight_mm2 = float(val.get('ac_std_mm2', As_mm2) or 0.0)
            As_source = 'ac_std'

        density_default_g_cm3 = 8.9
        density_g_cm3 = getattr(self.specs, 'conductor_density_g_cm3', None)
        try:
            density_g_cm3 = float(density_g_cm3) if density_g_cm3 is not None else None
        except Exception:
            density_g_cm3 = None
        density_used_g_cm3 = density_g_cm3 if (density_g_cm3 is not None and density_g_cm3 > 0) else density_default_g_cm3

        Wcond_kg = float(L_mean) * float(TPH_final) * 3.0 * float(As_for_weight_mm2) * (density_used_g_cm3 * 1e-3)
        val['Wcond_kg'] = Wcond_kg

        # Backward-compatible keys (older GUI versions expect Wcus_kg)
        val['Wcus_kg'] = Wcond_kg
        val['Wcus_As_mm2_used'] = As_for_weight_mm2
        val['Wcus_As_source'] = As_source
        val['Wcus_Lmt_m_used'] = float(L_mean)
        val['Wcus_TPH_used'] = int(TPH_final)

        # Store conductor material metadata for GUI
        val['conductor_material_used'] = getattr(self.specs, 'conductor_material', None)
        val['conductor_density_g_cm3_used'] = float(density_used_g_cm3)
        val['conductor_rho_ohm_m_used'] = float(rho_used)

        # Tooth weight (as requested, SI units):
        # Semi-open slot:
        #   A_tb   = hs2 * wst
        #   A_shoe = wst0*h0 + (h1*(wst0+wst)/2)
        #   A_tooth = A_shoe + A_tb
        # Open slot:
        #   A_tooth = (wst1 + wst2) * hs / 2
        # Weight (both):
        #   Wt = A_tooth[m^2] * Li[m] * Ss * Kgm_3[kg/m^3]
        steel_grade = getattr(self.specs, 'steel_grade', None)
        Kgm_3 = float(get_steel_density_kg_m3(steel_grade))
        Li_m = float(getattr(self.geometry, 'Li', 0.0) or 0.0)
        if Li_m <= 0:
            Ls_m = float(getattr(self.geometry, 'Ls', 0.0) or 0.0)
            ki = float(getattr(self.geometry, 'ki', 1.0) or 1.0)
            Li_m = (Ls_m * ki) if (Ls_m > 0 and ki > 0) else 0.0
        if Li_m <= 0:
            Li_m = float(getattr(self.geometry, 'L', 0.0) or 0.0)
        Ss_used = int(getattr(self.slots, 'Ss', 0) or 0) if self.slots is not None else int(Ss)

        A_tooth_m2 = 0.0
        if slot_method == 'open_slot':
            wst1_mm = float(val.get('open_slot_wst1_mm', 0.0) or 0.0)
            wst2_mm = float(val.get('open_slot_wst2_mm', 0.0) or 0.0)
            hs_mm = float(val.get('open_slot_hs_mm', 0.0) or 0.0)
            A_tooth_m2 = ((wst1_mm / 1000.0) + (wst2_mm / 1000.0)) * (hs_mm / 1000.0) / 2.0 if hs_mm > 0 else 0.0

            val['Wt_method'] = 'open_slot'
            val['Wt_wst1_mm_used'] = wst1_mm
            val['Wt_wst2_mm_used'] = wst2_mm
            val['Wt_hs_mm_used'] = hs_mm
        else:
            hs2_mm = float(val.get('hs2_mm', 0.0) or 0.0)
            wst0_mm = float(val.get('wst0_mm', 0.0) or 0.0)
            h0_mm = float(val.get('slot_dim_hs0_mm', val.get('hs0_mm', 0.0)) or 0.0)
            h1_mm = float(val.get('slot_dim_hs1_mm', val.get('hs1_mm', 0.0)) or 0.0)

            if slot_method == 'semi_open_slot_new':
                # Semi-open (new) slot (requested):
                #   slot_pitch_top_shoe = pi*(D + 2*(h0+h1)) / Ss
                #   w_tooth_top = slot_pitch_top_shoe - bs
                #   A_shoe = wst0*h0 + h1*(wst0 + w_tooth_top)/2
                #   A_tb   = hs2*(wst1 + w_tooth_top)/2
                try:
                    bs_mm = float(val.get('semi_open_new_bs_mm', 0.0) or 0.0)
                except (TypeError, ValueError):
                    bs_mm = 0.0
                try:
                    wst1_mm = float(val.get('semi_open_new_wst1_mm', val.get('wst_mm', 0.0)) or 0.0)
                except (TypeError, ValueError):
                    wst1_mm = 0.0

                try:
                    D_mm_local = float(getattr(self.geometry, 'D', 0.0) or 0.0) * 1000.0
                except Exception:
                    D_mm_local = 0.0
                slot_pitch_top_shoe_mm = (math.pi * (D_mm_local + 2.0 * (h0_mm + h1_mm)) / float(Ss_used)) if (Ss_used > 0 and D_mm_local > 0) else 0.0
                w_tooth_top_mm = (slot_pitch_top_shoe_mm - bs_mm) if (slot_pitch_top_shoe_mm > 0 and bs_mm > 0) else 0.0

                A_shoe_m2 = ((wst0_mm / 1000.0) * (h0_mm / 1000.0)) + ((h1_mm / 1000.0) * ((wst0_mm / 1000.0) + (w_tooth_top_mm / 1000.0)) / 2.0) if (h0_mm > 0 or h1_mm > 0) else 0.0
                A_tb_m2 = ((hs2_mm / 1000.0) * (((wst1_mm / 1000.0) + (w_tooth_top_mm / 1000.0)) / 2.0)) if (hs2_mm > 0 and wst1_mm > 0 and w_tooth_top_mm > 0) else 0.0
                A_tooth_m2 = A_tb_m2 + A_shoe_m2

                val['Wt_method'] = 'semi_open_slot_new'
                val['Wt_hs2_mm_used'] = hs2_mm
                val['Wt_wst0_mm_used'] = wst0_mm
                val['Wt_h0_mm_used'] = h0_mm
                val['Wt_h1_mm_used'] = h1_mm
                val['Wt_wst1_mm_used'] = wst1_mm
                val['Wt_bs_mm_used'] = bs_mm
                val['Wt_slot_pitch_top_shoe_mm_used'] = float(slot_pitch_top_shoe_mm)
                val['Wt_w_tooth_top_mm_used'] = float(w_tooth_top_mm)
                val['Wt_A_tb_m2'] = A_tb_m2
                val['Wt_A_shoe_m2'] = A_shoe_m2
            else:
                wst_mm = float(val.get('slot_dim_wst_mm', val.get('wst_mm', 0.0)) or 0.0)
                A_tb_m2 = (hs2_mm / 1000.0) * (wst_mm / 1000.0) if (hs2_mm > 0 and wst_mm > 0) else 0.0
                A_shoe_m2 = ((wst0_mm / 1000.0) * (h0_mm / 1000.0)) + ((h1_mm / 1000.0) * ((wst0_mm / 1000.0) + (wst_mm / 1000.0)) / 2.0) if (h0_mm > 0 or h1_mm > 0) else 0.0
                A_tooth_m2 = A_tb_m2 + A_shoe_m2

                val['Wt_method'] = 'semi_open_slot'
                val['Wt_hs2_mm_used'] = hs2_mm
                val['Wt_wst_mm_used'] = wst_mm
                val['Wt_wst0_mm_used'] = wst0_mm
                val['Wt_h0_mm_used'] = h0_mm
                val['Wt_h1_mm_used'] = h1_mm
                val['Wt_A_tb_m2'] = A_tb_m2
                val['Wt_A_shoe_m2'] = A_shoe_m2

        Wt_kg = A_tooth_m2 * float(Li_m) * float(Ss_used) * float(Kgm_3) if (A_tooth_m2 > 0 and Li_m > 0 and Ss_used > 0 and Kgm_3 > 0) else 0.0
        val['Wt_A_tooth_m2'] = A_tooth_m2
        # Backward-compatible key name (previously Ls represented net iron length)
        val['Wt_Ls_m_used'] = float(Li_m)
        val['Wt_Li_m_used'] = float(Li_m)
        val['Wt_Ss_used'] = int(Ss_used)
        val['Wt_density_kg_m3_used'] = float(Kgm_3)
        val['Wt_steel_grade_used'] = str(steel_grade) if steel_grade is not None else None
        val['Wt_kg'] = Wt_kg

        return self.winding, self.slots

    def _compute_forward_design_phi_max_and_wst(self, Ss: int) -> Dict[str, float]:
        """Forward Design step: compute phi_max and wst.

        Definitions (as provided, corrected):
        - phi_max = flux_pole * sin(pi * P / (2 * Ss))
        - wst = phi_max / (B_tooth * Li)

        Returns a dict and stores values in geometry.validation_info.
        """
        if self.geometry is None:
            self.calculate_main_dimensions()
        if self.geometry is None:
            return {}

        if Ss <= 0:
            return {}

        P = self.specs.poles
        flux_pole = float(self.geometry.flux_per_pole)
        phi_max = flux_pole * math.sin(math.pi * P / (2.0 * Ss))

        targets = self._forward_design_targets or self._select_forward_design_flux_densities()
        B_tooth = float(targets.get('B_tooth_target', 1.7))
        # Open slot rule update: use the maximum flux density the selected material can support
        slot_method = getattr(self.specs, 'slot_method', 'semi_open_slot') or 'semi_open_slot'
        if slot_method == 'open_slot':
            steel_max_b = targets.get('steel_max_B', None)
            try:
                steel_max_b_val = float(steel_max_b) if steel_max_b is not None else None
            except (TypeError, ValueError):
                steel_max_b_val = None
            if steel_max_b_val is not None and steel_max_b_val > 0:
                B_tooth = steel_max_b_val
        Li = float(getattr(self.geometry, 'Li', 0.0) or 0.0)
        if Li <= 0:
            Ls = float(getattr(self.geometry, 'Ls', 0.0) or 0.0)
            ki = float(getattr(self.geometry, 'ki', 1.0) or 1.0)
            Li = (Ls * ki) if (Ls > 0 and ki > 0) else 0.0
        if Li <= 0:
            Li = float(getattr(self.geometry, 'L', 0.0) or 0.0)

        denom = B_tooth * Li
        wst = (phi_max / denom) if denom > 0 else 0.0

        if not hasattr(self.geometry, 'validation_info') or self.geometry.validation_info is None:
            self.geometry.validation_info = {}
        self.geometry.validation_info['phi_max'] = phi_max
        self.geometry.validation_info['wst_m'] = wst
        self.geometry.validation_info['wst_mm'] = wst * 1000.0
        self.geometry.validation_info['phi_max_Ss'] = Ss
        self.geometry.validation_info['phi_max_P'] = P
        self.geometry.validation_info['phi_max_flux_pole'] = flux_pole
        self.geometry.validation_info['wst_B_tooth_used'] = B_tooth
        # Backward-compatible key name (previously Ls represented net iron length)
        self.geometry.validation_info['wst_Ls_used'] = Li
        self.geometry.validation_info['wst_Li_used'] = Li

        return {
            'phi_max': phi_max,
            'wst_m': wst,
        }

    def _select_swg_diameter(self, required_d_mm: float) -> Dict[str, float]:
        """Pick the smallest SWG diameter that is >= required diameter.

        Rule (as requested): never pick a standard diameter smaller than the calculated diameter.
        """
        if required_d_mm <= 0:
            return {'swg': float('nan'), 'diameter_mm': 0.0}

        candidates = [(g, d) for g, d in zip(SWG_GAUGE, SWG_DIAMETER_MM) if d >= required_d_mm]
        if not candidates:
            # Should be rare (required larger than max available). Use the largest available.
            g, d = SWG_GAUGE[0], SWG_DIAMETER_MM[0]
            return {'swg': float(g), 'diameter_mm': float(d)}

        # Choose the minimum diameter that still meets the >= constraint
        g_best, d_best = min(candidates, key=lambda x: x[1])
        return {'swg': float(g_best), 'diameter_mm': float(d_best)}

    def _compute_forward_design_wire_sizing(self) -> Dict[str, float]:
        """Forward Design step: compute wire area, strands, strand diameter, and SWG selection."""
        if self.geometry is None:
            self.calculate_main_dimensions()
        if self.electrical is None:
            self.calculate_electrical_parameters()
        if self.geometry is None or self.electrical is None:
            return {}

        # Constants (slot geometry assumptions for Forward Design steps)
        # hs0: slot opening height
        # hs1: slot neck / upper slot height (fixed)
        hs0_mm = 1.0
        hs1_mm = 3.0

        # Current density J from abaque based on bore diameter D
        D_mm = float(self.geometry.D) * 1000.0
        J = float(self._get_current_density_from_diameter(D_mm))
        Iph = float(self.electrical.I_s)

        # Copper area per conductor
        ac_mm2 = (Iph / J) if J > 0 else 0.0

        # Determine required strand diameter with max 1.7mm
        MAX_D_MM = 1.7
        number_of_strands = 1
        strand_area_mm2 = ac_mm2
        strand_diameter_mm = math.sqrt((4.0 * strand_area_mm2) / math.pi) if strand_area_mm2 > 0 else 0.0

        safety = 0
        while True:
            safety += 1
            strand_area_mm2 = (ac_mm2 / number_of_strands) if number_of_strands > 0 else 0.0
            strand_diameter_mm = math.sqrt((4.0 * strand_area_mm2) / math.pi) if strand_area_mm2 > 0 else 0.0
            if strand_diameter_mm <= MAX_D_MM:
                break
            number_of_strands += 1
            if number_of_strands > 200 or safety > 400:
                break

        swg_pick = self._select_swg_diameter(strand_diameter_mm)
        d_std_mm = float(swg_pick.get('diameter_mm', 0.0))
        swg = float(swg_pick.get('swg', float('nan')))

        # Slot opening (Forward Design additional step)
        # If no strands are required, wire diameter is the conductor diameter.
        # If strands are required, use strand diameter.
        # In this implementation, d_std_mm is the selected standard diameter for the strand;
        # when number_of_strands == 1, it is also the conductor diameter.
        Ss = int(getattr(self.slots, 'Ss', 0) or 0) if self.slots is not None else 0
        D_mm = float(self.geometry.D) * 1000.0
        slot_pitch_D_mm = (math.pi * D_mm / float(Ss)) if Ss > 0 else 0.0
        bs0_mm = 3.0 * d_std_mm
        wst0_mm = slot_pitch_D_mm - bs0_mm

        # New copper area based on chosen standard strand diameter
        strand_area_std_mm2 = (math.pi * (d_std_mm ** 2) / 4.0) if d_std_mm > 0 else 0.0
        ac_std_mm2 = number_of_strands * strand_area_std_mm2

        # Total copper area in a slot
        Nc = int(getattr(self.slots, 'Nc', 0) or 0) if self.slots is not None else 0
        Tac_mm2 = ac_std_mm2 * Nc

        # Slot area with fill factor (space factor)
        fill_factor = 0.4
        A_slot_mm2 = (Tac_mm2 / fill_factor) if fill_factor > 0 else 0.0

        oversize_pct = ((ac_std_mm2 - ac_mm2) / ac_mm2 * 100.0) if ac_mm2 > 0 else 0.0

        if not hasattr(self.geometry, 'validation_info') or self.geometry.validation_info is None:
            self.geometry.validation_info = {}
        self.geometry.validation_info.update({
            'hs0_mm': hs0_mm,
            # Backward-compat: keep existing key, but hs2_mm here represents fixed hs1=3mm.
            'hs2_mm': hs1_mm,
            'hs1_mm': hs1_mm,
            'Iph_A': Iph,
            'J_A_per_mm2': J,
            'ac_mm2': ac_mm2,
            'number_of_strands': float(number_of_strands),
            'strand_area_mm2': strand_area_mm2,
            'strand_diameter_mm_calc': strand_diameter_mm,
            'strand_diameter_mm_max': MAX_D_MM,
            'swg_selected': swg,
            'strand_diameter_mm_std': d_std_mm,
            # Slot opening and tooth tip width at bore
            'slot_pitch_D_mm': slot_pitch_D_mm,
            'bs0_mm': bs0_mm,
            'wst0_mm': wst0_mm,
            'bs0_diameter_used_mm': d_std_mm,
            'bs0_uses_strand_diameter': float(1.0 if number_of_strands > 1 else 0.0),
            'strand_area_mm2_std': strand_area_std_mm2,
            # Updated areas after selecting standard wire
            'ac_std_mm2': ac_std_mm2,
            'Nc_used_for_Tac': float(Nc),
            'Tac_mm2': Tac_mm2,
            'fill_factor': fill_factor,
            'A_slot_mm2': A_slot_mm2,
            'oversize_pct_std': oversize_pct,
        })

        return {
            'hs0_mm': hs0_mm,
            'hs1_mm': hs1_mm,
            'Iph_A': Iph,
            'J_A_per_mm2': J,
            'ac_mm2': ac_mm2,
            'number_of_strands': float(number_of_strands),
            'strand_diameter_mm_calc': strand_diameter_mm,
            'swg_selected': swg,
            'strand_diameter_mm_std': d_std_mm,
            'slot_pitch_D_mm': slot_pitch_D_mm,
            'bs0_mm': bs0_mm,
            'wst0_mm': wst0_mm,
            'ac_std_mm2': ac_std_mm2,
            'Tac_mm2': Tac_mm2,
            'A_slot_mm2': A_slot_mm2,
        }

    def _compute_forward_design_slot_dimensions(self, Ss: int) -> Dict[str, float]:
        """Forward Design step: compute slot dimensions from provided equations.

        Inputs (per user spec):
        - Ss: number of slots
        - D: bore diameter
        - hs0 (fixed 1mm), hs1 (fixed 3mm)
        - wst: tooth width from forward step (stored as wst_mm)
        - As: required slot area (stored as A_slot_mm2)

        Steps:
        1) bs1 from provided formula at radius (D/2 + hs0 + hs1)
        2) Iterate hs2 to match As using:
           bs2(hs2) from same formula at (D/2 + hs0 + hs1 + hs2)
           Ass = (bs1 + bs2) * hs2 / 2
           Err = |Ass - As| / As
        3) Compute slot pitch at (D + hs1), then bsmean and hs1/bsmean
        """
        if self.geometry is None:
            self.calculate_main_dimensions()
        if self.geometry is None or Ss <= 0:
            return {}

        if not hasattr(self.geometry, 'validation_info') or self.geometry.validation_info is None:
            self.geometry.validation_info = {}
        val = self.geometry.validation_info

        try:
            D_mm = float(self.geometry.D) * 1000.0
        except Exception:
            return {}

        # Pull required inputs from stored Forward Design steps
        hs0_mm = float(val.get('hs0_mm', 1.0) or 1.0)
        hs1_mm = float(val.get('hs1_mm', val.get('hs2_mm', 3.0)) or 3.0)
        wst_mm = float(val.get('wst_mm', 0.0) or 0.0)
        As_mm2 = val.get('A_slot_mm2', None)
        try:
            As_mm2_val = float(As_mm2) if As_mm2 is not None else None
        except (TypeError, ValueError):
            As_mm2_val = None

        theta = math.pi / float(Ss)
        cos_t = math.cos(theta)
        tan_t = math.tan(theta)
        if abs(cos_t) < 1e-12:
            return {}

        # bs1 (mm)
        bs1_mm = 2.0 * (tan_t * ((D_mm / 2.0) + hs0_mm + hs1_mm) - (wst_mm / 2.0) / cos_t)

        # Iteration for hs2 and bs2 to match required slot area As
        hs2_mm = 0.1
        Err = 0.05
        Niter = 0
        bs2_mm = 0.0
        Ass_mm2 = 0.0

        if As_mm2_val is not None and As_mm2_val > 0:
            while (Err > 0.02) and (Niter < 2000):
                bs2_mm = 2.0 * (tan_t * ((D_mm / 2.0) + hs0_mm + hs1_mm + hs2_mm) - (wst_mm / 2.0) / cos_t)
                Ass_mm2 = (bs1_mm + bs2_mm) * hs2_mm / 2.0
                Err = abs(Ass_mm2 - As_mm2_val) / As_mm2_val
                if Err <= 0.02:
                    break
                hs2_mm += 0.1
                Niter += 1
        else:
            Err = float('inf')

        # Slot pitch at (D + hs1) (interpreting the spec literally in mm)
        D_pitch_mm = D_mm + hs1_mm
        slot_pitch_mm = math.pi * D_pitch_mm / float(Ss)
        bsmean_mm = slot_pitch_mm - wst_mm
        hs2_over_bsmean = (hs2_mm / bsmean_mm) if bsmean_mm > 0 else float('inf')

        val.update({
            'slot_dim_Ss': float(Ss),
            'slot_dim_D_mm': D_mm,
            'slot_dim_hs0_mm': hs0_mm,
            'slot_dim_hs1_mm': hs1_mm,
            'slot_dim_wst_mm': wst_mm,
            'bs1_mm': bs1_mm,
            'hs2_mm': hs2_mm,
            'bs2_mm': bs2_mm,
            'Ass_mm2': Ass_mm2,
            'Err_slot_area': Err,
            'Niter_slot_area': float(Niter),
            'slot_pitch_D_plus_hs1_mm': slot_pitch_mm,
            'bsmean_mm': bsmean_mm,
            'hs2_over_bsmean': hs2_over_bsmean,
            # Backward-compat (previously mislabeled as hs1/bsmean)
            'hs1_over_bsmean': hs2_over_bsmean,
        })

        return {
            'bs1_mm': bs1_mm,
            'hs2_mm': hs2_mm,
            'bs2_mm': bs2_mm,
            'Ass_mm2': Ass_mm2,
            'Err_slot_area': Err,
            'Niter_slot_area': float(Niter),
            'slot_pitch_D_plus_hs1_mm': slot_pitch_mm,
            'bsmean_mm': bsmean_mm,
            'hs2_over_bsmean': hs2_over_bsmean,
        }

    def _compute_semi_open_slot_new_geometry(self, Ss: int) -> Dict[str, float]:
        """Semi-open slot (new) geometry solver.

        Solves for (bs, hs2) with constraints:
          (1) A_slot = hs2 * bs
          (2) G = hs2/bs in [3, 5]
          (3) slot_pitch(D_top) = bs + wst1
              where D_top = D + 2*(hs0 + hs1 + hs2)

        Stores results in geometry.validation_info under keys:
          semi_open_new_bs_mm, semi_open_new_hs2_mm, semi_open_new_wst1_mm,
          and also overrides hs0_mm/hs1_mm/hs2_mm/hs_total_mm for downstream steps.
        """
        if self.geometry is None:
            self.calculate_main_dimensions()
        if self.geometry is None or Ss <= 0:
            return {}

        if not hasattr(self.geometry, 'validation_info') or self.geometry.validation_info is None:
            self.geometry.validation_info = {}
        val = self.geometry.validation_info

        # Fixed heights (same assumptions as other Forward Design steps)
        try:
            hs0_mm = float(val.get('hs0_mm', 1.0) or 1.0)
        except (TypeError, ValueError):
            hs0_mm = 1.0
        try:
            hs1_mm = float(val.get('hs1_mm', 3.0) or 3.0)
        except (TypeError, ValueError):
            hs1_mm = 3.0

        # Required slot area
        try:
            A_slot_mm2 = float(val.get('A_slot_mm2', 0.0) or 0.0)
        except (TypeError, ValueError):
            A_slot_mm2 = 0.0
        if A_slot_mm2 <= 0:
            return {}

        # wst1 from Forward step (minimum tooth width), in mm
        try:
            wst1_mm = float(val.get('wst_mm', 0.0) or 0.0)
        except (TypeError, ValueError):
            wst1_mm = 0.0
        if wst1_mm <= 0:
            return {}

        # Pitch model in mm: slot_pitch(D_top) = base_pitch + k*hs2
        try:
            D_mm = float(getattr(self.geometry, 'D', 0.0) or 0.0) * 1000.0
        except Exception:
            D_mm = 0.0
        if D_mm <= 0:
            return {}

        base_pitch_mm = (math.pi * (D_mm + 2.0 * (hs0_mm + hs1_mm)) / float(Ss)) if Ss > 0 else 0.0
        k_pitch = (2.0 * math.pi / float(Ss)) if Ss > 0 else 0.0
        if k_pitch <= 0:
            return {}

        # Root solve for G in [3,5], matching GUI behavior
        def f_of_g(g_ratio: float) -> float:
            if g_ratio <= 0.0:
                return float('nan')
            bs_area = math.sqrt(A_slot_mm2 / g_ratio)
            hs2_local = math.sqrt(A_slot_mm2 * g_ratio)
            pitch_local = base_pitch_mm + (k_pitch * hs2_local)
            bs_pitch = pitch_local - wst1_mm
            return bs_area - bs_pitch

        g_lo = 3.0
        g_hi = 5.0
        f_lo = f_of_g(g_lo)
        f_hi = f_of_g(g_hi)

        solved_exact = False
        g_best = None
        if (not math.isnan(f_lo)) and (not math.isnan(f_hi)):
            if abs(f_lo) < 1e-12:
                g_best = g_lo
                solved_exact = True
            elif abs(f_hi) < 1e-12:
                g_best = g_hi
                solved_exact = True
            elif f_lo * f_hi < 0.0:
                lo = g_lo
                hi = g_hi
                flo = f_lo
                for _ in range(80):
                    mid = 0.5 * (lo + hi)
                    fmid = f_of_g(mid)
                    if math.isnan(fmid):
                        break
                    if abs(fmid) < 1e-12:
                        lo = hi = mid
                        break
                    if flo * fmid < 0.0:
                        hi = mid
                    else:
                        lo = mid
                        flo = fmid
                g_best = 0.5 * (lo + hi)
                solved_exact = True

        if g_best is None:
            best_abs = None
            best_tie = None
            for gi in range(3000, 5001):
                g = gi / 1000.0
                fv = f_of_g(g)
                if math.isnan(fv):
                    continue
                abs_f = abs(fv)
                tie = abs(g - 4.0)
                if best_abs is None or abs_f < best_abs - 1e-12 or (abs(abs_f - best_abs) <= 1e-12 and tie < float(best_tie)):
                    best_abs = abs_f
                    best_tie = tie
                    g_best = g

        g_best = float(g_best)
        bs_before_mm = math.sqrt(A_slot_mm2 / g_best)
        hs2_before_mm = math.sqrt(A_slot_mm2 * g_best)
        pitch_top_before_mm = base_pitch_mm + (k_pitch * hs2_before_mm)
        pitch_check_before_mm = bs_before_mm + wst1_mm
        pitch_err_before_mm = pitch_top_before_mm - pitch_check_before_mm

        tol_pitch_mm = 1e-6

        # Steel limit (if available)
        try:
            steel_max_T = float(val.get('steel_max_B', None))
        except (TypeError, ValueError):
            steel_max_T = None

        # For B check in correction approach
        try:
            phi_max_wb = float(val.get('phi_max', 0.0) or 0.0)
        except (TypeError, ValueError):
            phi_max_wb = 0.0
        Li_used_m = float(val.get('wst_Li_used', val.get('wst_Ls_used', getattr(self.geometry, 'Li', 0.0))) or 0.0)

        def compute_B_tooth(phi_wb: float, Li_m: float, wst_mm_local: float) -> float:
            denom_local = (Li_m * (wst_mm_local / 1000.0))
            return (phi_wb / denom_local) if denom_local > 0 else float('inf')

        # Approach 1
        a1_applicable = (pitch_err_before_mm > tol_pitch_mm)
        a1_wst1_after_mm = (wst1_mm + abs(pitch_err_before_mm)) if a1_applicable else wst1_mm
        a1_pitch_err_mm = pitch_top_before_mm - (bs_before_mm + a1_wst1_after_mm)
        a1_pitch_ok = abs(a1_pitch_err_mm) <= tol_pitch_mm
        a1_B_ok = True
        if steel_max_T is not None and steel_max_T > 0 and phi_max_wb > 0 and Li_used_m > 0 and a1_wst1_after_mm > 0:
            a1_B = compute_B_tooth(phi_max_wb, Li_used_m, a1_wst1_after_mm)
            a1_B_ok = a1_B <= steel_max_T
        a1_success = a1_applicable and a1_pitch_ok and a1_B_ok

        # Approach 2
        a2_wst1_after_mm = wst1_mm
        a2_bs_after_mm = bs_before_mm + pitch_err_before_mm
        if a2_bs_after_mm <= 0:
            a2_bs_after_mm = max(0.001, bs_before_mm)
        hs2_from_area = (A_slot_mm2 / a2_bs_after_mm) if a2_bs_after_mm > 0 else None
        if hs2_from_area is None or hs2_from_area <= 0:
            a2_hs2_after_mm = hs2_before_mm
        else:
            g_from_area = hs2_from_area / a2_bs_after_mm
            if g_from_area < 3.0:
                a2_hs2_after_mm = 3.0 * a2_bs_after_mm
            elif g_from_area > 5.0:
                a2_hs2_after_mm = 5.0 * a2_bs_after_mm
            else:
                a2_hs2_after_mm = hs2_from_area
        a2_pitch_top_mm = base_pitch_mm + (k_pitch * a2_hs2_after_mm)
        a2_pitch_err_mm = a2_pitch_top_mm - (a2_bs_after_mm + a2_wst1_after_mm)

        # Final selection (same as GUI logic)
        final_uses_a1 = bool(solved_exact or a1_success)
        wst1_final_mm = float(wst1_mm)
        if not solved_exact:
            wst1_final_mm = float(a1_wst1_after_mm) if final_uses_a1 else float(a2_wst1_after_mm)
        bs_final_mm = float(bs_before_mm) if final_uses_a1 else float(a2_bs_after_mm)
        hs2_final_mm = float(hs2_before_mm) if final_uses_a1 else float(a2_hs2_after_mm)

        hs_total_mm = float(hs0_mm) + float(hs1_mm) + float(hs2_final_mm)

        val['semi_open_new_solved_exact'] = float(1.0 if solved_exact else 0.0)
        val['semi_open_new_g_ratio'] = float(hs2_final_mm / bs_final_mm) if bs_final_mm > 0 else float('inf')
        val['semi_open_new_bs_mm'] = float(bs_final_mm)
        val['semi_open_new_hs2_mm'] = float(hs2_final_mm)
        val['semi_open_new_wst1_mm'] = float(wst1_final_mm)
        val['semi_open_new_base_pitch_mm'] = float(base_pitch_mm)
        val['semi_open_new_pitch_top_mm'] = float(base_pitch_mm + (k_pitch * hs2_final_mm))
        val['semi_open_new_pitch_err_before_mm'] = float(pitch_err_before_mm)
        val['semi_open_new_pitch_err_after_mm'] = float(a1_pitch_err_mm) if final_uses_a1 else float(a2_pitch_err_mm)

        # Override hs used by downstream tabs
        val['hs0_mm'] = float(hs0_mm)
        val['hs1_mm'] = float(hs1_mm)
        val['hs2_mm'] = float(hs2_final_mm)
        val['hs_total_mm'] = float(hs_total_mm)
        val['hs_total_breakdown_mm'] = {
            'hs0_mm': float(hs0_mm),
            'hs1_mm': float(hs1_mm),
            'hs2_mm': float(hs2_final_mm),
        }
        try:
            self.geometry.h_slot = float(hs_total_mm) / 1000.0
        except Exception:
            pass

        return {
            'bs_final_mm': float(bs_final_mm),
            'hs2_final_mm': float(hs2_final_mm),
            'wst1_final_mm': float(wst1_final_mm),
        }

    def _get_open_slot_recommendations(self, Iph_A: float) -> Dict[str, float]:
        """Lookup Open Slot abaque row and return recommended ranges."""
        rows = range(len(AbaquesTables.OPEN_SLOT_I_MIN_A))
        idx = 0
        for i in rows:
            if AbaquesTables.OPEN_SLOT_I_MIN_A[i] <= Iph_A < AbaquesTables.OPEN_SLOT_I_MAX_A[i]:
                idx = i
                break

        return {
            'row_index': float(idx),
            'I_min_A': float(AbaquesTables.OPEN_SLOT_I_MIN_A[idx]),
            'I_max_A': float(AbaquesTables.OPEN_SLOT_I_MAX_A[idx]),
            'thickness_min_mm': float(AbaquesTables.OPEN_SLOT_THICKNESS_MIN_MM[idx]),
            'thickness_max_mm': float(AbaquesTables.OPEN_SLOT_THICKNESS_MAX_MM[idx]),
            'width_min_mm': float(AbaquesTables.OPEN_SLOT_WIDTH_MIN_MM[idx]),
            'width_max_mm': float(AbaquesTables.OPEN_SLOT_WIDTH_MAX_MM[idx]),
            'ratio_min': float(AbaquesTables.OPEN_SLOT_RATIO_MIN[idx]),
            'ratio_max': float(AbaquesTables.OPEN_SLOT_RATIO_MAX[idx]),
        }

    @staticmethod
    def _round_up_to_half_mm(value_mm: float) -> float:
        """Standardize width upward to the next 0.5 mm step (e.g., 4.2→4.5, 4.8→5.0)."""
        if value_mm <= 0:
            return 0.0
        return math.ceil(value_mm * 2.0) / 2.0

    def _compute_open_slot_strip_sizing(self, Ss: int, Nc: int) -> Dict[str, float]:
        """Open Slot method: rectangular strip sizing from current abaque.

        Steps:
        - ac = Iph / J
        - Choose strip thickness from standard list within recommended [t_min, t_max]
        - Start width from recommended w_min and increase by 0.5 mm until (t_std * width) >= ac
        - ac_std = t_std * width_std
        - Tac = ac_std * Nc
        - A_slot = Tac / fill_factor
        """
        if self.geometry is None:
            self.calculate_main_dimensions()
        if self.electrical is None:
            self.calculate_electrical_parameters()
        if self.geometry is None or self.electrical is None:
            return {}

        # Constants for slot model consistency (same as semi-open workflow)
        hs0_mm = 1.0
        hs1_mm = 3.0
        fill_factor = 0.4

        D_mm = float(self.geometry.D) * 1000.0
        J = float(self._get_current_density_from_diameter(D_mm))
        Iph = float(self.electrical.I_s)

        ac_mm2 = (Iph / J) if J > 0 else 0.0

        rec = self._get_open_slot_recommendations(Iph)
        t_min = float(rec['thickness_min_mm'])
        t_max = float(rec['thickness_max_mm'])

        # Choose thickness from standard list within [t_min, t_max]
        std_ts = [float(x) for x in AbaquesTables.OPEN_SLOT_STANDARD_THICKNESSES_MM]
        candidates_in_range = [t for t in std_ts if (t_min <= t <= t_max)]

        w_min = float(rec['width_min_mm'])
        w_max = float(rec['width_max_mm'])
        width_step_mm = 0.5
        width_start_mm = self._round_up_to_half_mm(w_min)

        # We will try thickness candidates in ascending order (first pick is the smallest in-range).
        # If width iteration cannot reach ac for that thickness, try another in-range thickness.
        if candidates_in_range:
            thickness_choice_note = 'standard_in_range_with_retry'
            thickness_candidates = sorted(candidates_in_range)
        else:
            # Fallback: choose the nearest standard thickness to the middle of the recommended range.
            thickness_choice_note = 'nearest_standard_fallback'
            t_fallback = min(std_ts, key=lambda t: abs(t - ((t_min + t_max) / 2.0))) if std_ts else t_min
            thickness_candidates = [t_fallback]

        # Track best (closest) combo in case none can satisfy within width range.
        best = {
            't': None,
            'w': None,
            'area': -1.0,
            'iter_count': 0,
            'satisfied': False,
            'exceeded_max': False,
        }

        chosen = None
        for t_candidate in thickness_candidates:
            if t_candidate <= 0:
                continue

            w = width_start_mm
            iter_count = 0
            area_mm2 = t_candidate * w

            # Increase width until area >= target or until width exceeds w_max
            while (w <= w_max + 1e-12) and (area_mm2 + 1e-12 < ac_mm2):
                w += width_step_mm
                iter_count += 1
                area_mm2 = t_candidate * w

            satisfied = area_mm2 + 1e-12 >= ac_mm2
            exceeded_max = w > w_max + 1e-12

            if satisfied and not exceeded_max:
                chosen = {
                    't': t_candidate,
                    'w': w,
                    'area': area_mm2,
                    'iter_count': iter_count,
                    'satisfied': True,
                    'exceeded_max': False,
                }
                break

            # Not satisfied within range: clamp to max width for a comparable "closest" candidate.
            area_at_max = t_candidate * w_max
            if area_at_max > best['area']:
                best = {
                    't': t_candidate,
                    'w': w_max,
                    'area': area_at_max,
                    'iter_count': iter_count,
                    'satisfied': False,
                    'exceeded_max': True,
                }

        if chosen is None:
            chosen = best

        t_std = float(chosen['t']) if chosen['t'] is not None else 0.0
        width_std_mm = float(chosen['w']) if chosen['w'] is not None else width_start_mm
        area_mm2 = float(chosen['area']) if chosen['area'] is not None else 0.0
        iter_count = int(chosen.get('iter_count', 0))
        width_satisfied = bool(chosen.get('satisfied', False))
        width_exceeded_max = bool(chosen.get('exceeded_max', False))

        # Reference width purely for reporting
        width_calc_mm = (ac_mm2 / t_std) if t_std > 0 else 0.0

        ac_std_mm2 = area_mm2
        Tac_mm2 = ac_std_mm2 * float(max(1, int(Nc)))
        A_slot_mm2 = (Tac_mm2 / fill_factor) if fill_factor > 0 else 0.0

        ratio_calc = (width_calc_mm / t_std) if t_std > 0 else float('inf')
        ratio_std = (width_std_mm / t_std) if t_std > 0 else float('inf')

        oversize_pct = ((ac_std_mm2 - ac_mm2) / ac_mm2 * 100.0) if ac_mm2 > 0 else 0.0

        # Store for GUI
        if not hasattr(self.geometry, 'validation_info') or self.geometry.validation_info is None:
            self.geometry.validation_info = {}
        self.geometry.validation_info.update({
            'hs0_mm': hs0_mm,
            'hs1_mm': hs1_mm,
            'open_slot_row_index': rec.get('row_index'),
            'open_slot_I_min_A': rec.get('I_min_A'),
            'open_slot_I_max_A': rec.get('I_max_A'),
            'open_slot_thickness_min_mm': t_min,
            'open_slot_thickness_max_mm': t_max,
            'open_slot_width_min_mm': w_min,
            'open_slot_width_max_mm': w_max,
            'open_slot_ratio_min': rec.get('ratio_min'),
            'open_slot_ratio_max': rec.get('ratio_max'),
            'open_slot_standard_thicknesses_mm': list(std_ts),
            'open_slot_thickness_choice_note': thickness_choice_note,
            'open_slot_thickness_candidates_in_range_mm': list(sorted(candidates_in_range)),
            'open_slot_optimal_combo_found': float(1.0 if width_satisfied else 0.0),
            'Iph_A': Iph,
            'J_A_per_mm2': J,
            'ac_mm2': ac_mm2,
            'strip_thickness_mm_std': t_std,
            'strip_width_mm_calc': width_calc_mm,
            'strip_width_mm_std': width_std_mm,
            'strip_width_mm_start': width_start_mm,
            'strip_width_mm_step': width_step_mm,
            'strip_width_iter_count': float(iter_count),
            'strip_width_satisfied': float(1.0 if width_satisfied else 0.0),
            'strip_ratio_calc': ratio_calc,
            'strip_ratio_std': ratio_std,
            'ac_std_mm2': ac_std_mm2,
            'Nc_used_for_Tac': float(max(1, int(Nc))),
            'Tac_mm2': Tac_mm2,
            'fill_factor': fill_factor,
            'A_slot_mm2': A_slot_mm2,
            'oversize_pct_std': oversize_pct,
            'open_slot_width_exceeds_max': float(1.0 if width_exceeded_max else 0.0),
        })

        return {
            'ac_mm2': ac_mm2,
            'thickness_mm_std': t_std,
            'width_mm_calc': width_calc_mm,
            'width_mm_std': width_std_mm,
            'ac_std_mm2': ac_std_mm2,
            'A_slot_mm2': A_slot_mm2,
        }
        
    # ═══════════════════════════════════════════════════════════════════════
    # PHASE 1: EMPIRICAL PARAMETERS FROM ABAQUES
    # ═══════════════════════════════════════════════════════════════════════
    
    def get_empirical_parameters(self) -> Dict:
        """Retrieve empirical parameters from abaques"""
        power_list = sorted(AbaquesTables.TABLE_7_1.keys())
        values_bav = [AbaquesTables.TABLE_7_1[p]['Bav'] for p in power_list]
        values_q = [AbaquesTables.TABLE_7_1[p]['q'] for p in power_list]
        
        bav = Interpolator.linear_interpolate(self.specs.power_kw, power_list, values_bav)
        q_default = Interpolator.linear_interpolate(self.specs.power_kw, power_list, values_q)

        # Power factor and efficiency (formula-based, no tables)
        # Definitions:
        #   P  : rated power in kW (user input)
        #   p  : number of pole pairs = poles / 2
        #   ns : synchronous speed in rpm = 60*f / p
        P_kw = float(self.specs.power_kw)
        poles = int(self.specs.poles)
        pole_pairs = float(poles) / 2.0 if poles > 0 else 0.0
        f_hz = float(self.specs.frequency_hz)

        P_safe = max(P_kw, 1e-9)
        p_safe = max(pole_pairs, 1e-9)
        ns_rpm = (60.0 * f_hz / p_safe) if f_hz > 0 else 0.0
        ns_over_p = max(ns_rpm / p_safe, 1e-9)

        # cos(phi) = 0.84 - 0.04*ln(P) + 0.02*ln(ns/p)
        cos_phi = 0.84 - 0.04 * math.log(P_safe) + 0.02 * math.log(ns_over_p)
        cos_phi = max(0.0, min(0.999, float(cos_phi)))

        # eta = 1 - [0.01 + 0.03*P^(-0.03) + 0.1*ln(p)]
        eta = 1.0 - (0.01 + 0.03 * (P_safe ** (-0.03)) + 0.1 * math.log(p_safe))
        eta = max(0.0, min(0.999, float(eta)))

        # Apply optional user overrides (non-None overrides take priority)
        if getattr(self.specs, 'override_Bav', None) is not None:
            try:
                val = float(self.specs.override_Bav)
                if val > 0:
                    bav = val
            except (TypeError, ValueError):
                pass

        if getattr(self.specs, 'override_ac', None) is not None:
            try:
                val = float(self.specs.override_ac)
                if val > 0:
                    q_default = val
            except (TypeError, ValueError):
                pass

        if getattr(self.specs, 'target_power_factor', None) is not None:
            try:
                val = float(self.specs.target_power_factor)
                if 0 < val <= 1:
                    cos_phi = val
            except (TypeError, ValueError):
                pass

        if getattr(self.specs, 'target_efficiency', None) is not None:
            try:
                val = float(self.specs.target_efficiency)
                if 0 < val <= 1:
                    eta = val
            except (TypeError, ValueError):
                pass
        
        return {
            'B_av': bav,
            'q_default': q_default,
            'eta': eta,
            'cos_phi': cos_phi,
        }
    
    def _determine_q(self) -> int:
        """Determine slots per pole per phase (q)"""
        # Optional override
        override_q = getattr(self.specs, 'override_q', None)
        if override_q is not None:
            try:
                q_val = int(round(float(override_q)))
                if q_val >= 1:
                    return q_val
            except (TypeError, ValueError):
                pass

        power = self.specs.power_kw
        poles = self.specs.poles
        
        for (p_min, p_max, poles_key), (q_min, q_max) in AbaquesTables.TABLE_Q_RECOMMENDATIONS.items():
            if p_min <= power <= p_max and poles == poles_key:
                # If a single q is suggested, use it; if a range is suggested, always take the bigger value.
                return int(q_min) if q_min == q_max else int(q_max)
        
        return 3
    
    def _determine_q_with_validation(self) -> Tuple[int, int, Dict]:
        """
        Determine slots per pole per phase (q) with slot pitch validation.
        
        Slot pitch = pi * D / Ss should be between 1.8 and 2.5 cm (0.018 to 0.025 m)
        
        Returns:
            Tuple of (q, Ss, validation_info)
        """
        validation_info = {
            'initial_q': 0,
            'initial_Ss': 0,
            'initial_slot_pitch_cm': 0,
            'final_q': 0,
            'final_Ss': 0,
            'final_slot_pitch_cm': 0,
            'criteria_met': False,
            'adjustments_made': []
        }
        
        P = self.specs.poles
        m = self.specs.phases
        D = self.geometry.D
        
        # Get initial q from recommendations
        q = self._determine_q()
        validation_info['initial_q'] = q
        
        # Calculate initial Ss and slot pitch
        Ss = P * m * q
        validation_info['initial_Ss'] = Ss
        slot_pitch = math.pi * D / Ss
        slot_pitch_cm = slot_pitch * 100  # Convert to cm
        validation_info['initial_slot_pitch_cm'] = slot_pitch_cm
        
        # From now on, q is taken directly from the recommendation table (no slot-pitch-based adjustment).
        validation_info['criteria_met'] = bool(1.8 <= slot_pitch_cm <= 2.5)
        validation_info['final_q'] = q
        validation_info['final_Ss'] = Ss
        validation_info['final_slot_pitch_cm'] = slot_pitch_cm
        if not validation_info['criteria_met']:
            validation_info['adjustments_made'].append(
                'No q adjustment applied (table-selected q is enforced).'
            )
        return q, Ss, validation_info
    
    def _get_current_density_from_diameter(self, D_mm: float) -> float:
        """Get stator current density based on diameter using linear interpolation/extrapolation
        
        Args:
            D_mm: Stator inner diameter in millimeters
            
        Returns:
            Current density J in A/mm^2
        """
        import numpy as np
        
        # Use numpy for interpolation (with extrapolation)
        J = np.interp(D_mm, AbaquesTables.J_DIAMETER_TABLE_D, AbaquesTables.J_DIAMETER_TABLE_J)
        
        return float(J)

    # ═══════════════════════════════════════════════════════════════════
    # LAMBDA (L/D) SELECTION HELPERS
    # ═══════════════════════════════════════════════════════════════════

    @staticmethod
    def _compute_dimensions_from_lambda(lambda_ratio: float, D2L: float, P: int) -> Tuple[float, float]:
        """Derive D and L from D²L and a chosen lambda where lambda = L / τ"""
        # λ = L / τ = L / (πD/P) => L = λ * π * D / P
        # Substitute into D²L = D² * (λ * π * D / P) = (λ * π / P) * D³
        # Solve for D, then recompute L from D²L to preserve the product exactly.
        D = (D2L * P / (lambda_ratio * math.pi)) ** (1.0 / 3.0)
        L = D2L / (D * D)
        return D, L

    def _select_lambda_by_poles(self) -> Tuple[float, Dict]:
        """Select lambda based on pole-count recommendations (method B)"""
        info = {'method': 'lambda_by_poles'}
        pole_data = LAMBDA_BY_POLES.get(self.specs.poles)
        if pole_data:
            lambda_ratio = pole_data['typical']
            info.update({
                'lambda_ratio': lambda_ratio,
                'range': (pole_data['min'], pole_data['max']),
                'basis': f"poles={self.specs.poles}",
                'note': 'Using typical lambda for pole count'
            })
        else:
            # Fallback for uncommon pole counts: interpolate between nearest known or use balanced default
            lambda_ratio = 1.7
            info.update({
                'lambda_ratio': lambda_ratio,
                'range': (1.2, 2.5),
                'basis': f"poles={self.specs.poles} (fallback)",
                'note': 'Fallback lambda for unsupported pole count'
            })
        return lambda_ratio, info

    @staticmethod
    def _candidate_penalty(value: float, selected_ranges: List[Tuple[float, float]]) -> Tuple[float, float]:
        """Compute (max_violation, sum_violation) for a candidate lambda value"""
        max_violation = 0.0
        sum_violation = 0.0
        for low, high in selected_ranges:
            if low <= value <= high:
                violation = 0.0
            else:
                violation = min(abs(value - low), abs(value - high))
            max_violation = max(max_violation, violation)
            sum_violation += violation
        return max_violation, sum_violation

    def _select_lambda_from_preferences(self, preference_flags: Optional[Dict[str, bool]]) -> Tuple[float, Dict]:
        """Resolve lambda from goal-based preference selection (method C)"""
        info = {'method': 'preference_based'}
        selected = [name for name, checked in (preference_flags or {}).items() if checked]
        if not selected:
            # Default to balanced if nothing is selected
            selected = ['balanced_design']
        # Filter to known ranges to guard against stale flags
        selected = [s for s in selected if s in PREFERENCE_LAMBDA_RANGES]
        if not selected:
            selected = ['balanced_design']
        info['selected_preferences'] = selected
        selected_ranges = [PREFERENCE_LAMBDA_RANGES[name] for name in selected if name in PREFERENCE_LAMBDA_RANGES]
        
        # Attempt direct intersection first
        low_bound = max(r[0] for r in selected_ranges)
        high_bound = min(r[1] for r in selected_ranges)
        if low_bound <= high_bound:
            lambda_ratio = 0.5 * (low_bound + high_bound)
            info.update({
                'lambda_ratio': lambda_ratio,
                'resolution': 'intersection',
                'range': (low_bound, high_bound),
                'note': 'Using midpoint of overlapping preference ranges'
            })
            return lambda_ratio, info
        
        # No common overlap: evaluate candidates to minimize violation
        candidates: List[float] = []
        for r in selected_ranges:
            candidates.extend([r[0], r[1], 0.5 * (r[0] + r[1])])
        # Add balanced midpoint as a stabilizer
        balanced_range = PREFERENCE_LAMBDA_RANGES['balanced_design']
        candidates.append(0.5 * (balanced_range[0] + balanced_range[1]))
        # Deduplicate
        candidates = sorted(set(candidates))
        
        scored: List[Tuple[float, float, float]] = []  # (max_violation, sum_violation, candidate)
        for candidate in candidates:
            max_violation, sum_violation = self._candidate_penalty(candidate, selected_ranges)
            scored.append((max_violation, sum_violation, candidate))
        scored.sort()
        best_candidate = scored[0][2]
        
        # Tie-break: prefer higher-priority preference midpoints when violations equal
        best_candidates = [c for c in scored if c[0] == scored[0][0] and c[1] == scored[0][1]]
        if len(best_candidates) > 1:
            priority_target = None
            for pref in PREFERENCE_PRIORITY:
                if pref in selected:
                    pref_range = PREFERENCE_LAMBDA_RANGES[pref]
                    priority_target = 0.5 * (pref_range[0] + pref_range[1])
                    break
            if priority_target is not None:
                best_candidate = min(
                    [c[2] for c in best_candidates],
                    key=lambda val: abs(val - priority_target)
                )
        lambda_ratio = best_candidate
        info.update({
            'lambda_ratio': lambda_ratio,
            'resolution': 'best_fit',
            'range': None,
            'note': 'Using candidate that minimizes deviation from selected ranges',
            'penalty': {'max': scored[0][0], 'sum': scored[0][1]},
        })
        return lambda_ratio, info

    def _apply_peripheral_speed_limit(self, D: float, D2L: float) -> Tuple[float, float, Dict]:
        """Enforce peripheral speed limit; adjust D to the allowable max and recompute L"""
        info = {}
        v_initial = math.pi * D * self.electrical.n_s_rps
        info['peripheral_speed_initial'] = v_initial
        if v_initial <= PERIPHERAL_SPEED_LIMIT:
            info['peripheral_speed_limited'] = False
            return D, D2L / (D * D), info

        D_limit = PERIPHERAL_SPEED_LIMIT / (math.pi * self.electrical.n_s_rps)
        info['peripheral_speed_limited'] = True
        info['D_speed_limit'] = D_limit

        # Clamp D to the maximum allowable diameter from the speed formula
        D_limited = D_limit
        L_limited = D2L / (D_limited * D_limited)
        return D_limited, L_limited, info
    
    # ═══════════════════════════════════════════════════════════════════════
    # PHASE 2: ELECTRICAL CALCULATIONS
    # ═══════════════════════════════════════════════════════════════════════
    
    def calculate_electrical_parameters(self) -> ElectricalParameters:
        """Calculate electrical parameters"""
        params = self.get_empirical_parameters()
        eta = params['eta']
        cos_phi = params['cos_phi']
        
        # 1. Electrical power: P_elec = P / efficiency [kW]
        P_elec = self.specs.power_kw / eta
        
        # 2. Synchronous speed: ns = 60 * (f / (P/2)) [rpm]
        n_s = 60 * (self.specs.frequency_hz / (self.specs.poles / 2))
        
        # 3. Convert to rps
        n_s_rps = n_s / 60.0
        
        # 4. Phase voltage based on connection
        if self.specs.connection.lower() == 'delta':
            E_ph = self.specs.voltage_v
        else:  # star
            E_ph = self.specs.voltage_v / math.sqrt(3)
        
        # 5. Phase current (line voltage provided):
        # - Star (Y):   I_s = P_elec*1000 / (sqrt(3) * V_line * cos(phi))
        # - Delta (Δ):  I_s = P_elec*1000 / (3 * V_line * cos(phi))
        # Convert P_elec from kW to W
        if self.specs.connection.lower() == 'delta':
            I_s = (P_elec * 1000) / (3.0 * self.specs.voltage_v * cos_phi)
        else:
            I_s = (P_elec * 1000) / (math.sqrt(3) * self.specs.voltage_v * cos_phi)
        
        self.electrical = ElectricalParameters(
            P_elec=P_elec,
            n_s=n_s,
            n_s_rps=n_s_rps,
            I_s=I_s,
            E_ph=E_ph,
            cos_phi=cos_phi,
            efficiency=eta
        )
        
        return self.electrical
    
    @staticmethod
    def _round_to_nearest_10mm(length_m: float) -> float:
        """
        Round a dimension to the nearest 10 mm.
        
        Args:
            length_m: Length in meters
        
        Returns:
            Rounded length in meters (to nearest 10 mm)
        
        Examples:
            0.1627 m (162.7 mm) → 0.160 m (160 mm)
            0.1799 m (179.9 mm) → 0.180 m (180 mm)
            0.1651 m (165.1 mm) → 0.170 m (170 mm)
        """
        length_mm = length_m * 1000  # Convert to mm
        rounded_mm = round(length_mm / 10) * 10  # Round to nearest 10
        return rounded_mm / 1000  # Convert back to m

    @staticmethod
    def _ceil_to_nearest_10mm(length_m: float) -> float:
        """Round a dimension up to the next 10 mm.

        Examples:
            0.1600 m (160.0 mm) → 0.160 m
            0.1601 m (160.1 mm) → 0.170 m
            0.1699 m (169.9 mm) → 0.170 m
        """
        if length_m <= 0:
            return length_m
        length_mm = length_m * 1000.0
        rounded_mm = math.ceil(length_mm / 10.0) * 10.0
        return rounded_mm / 1000.0
    
    def _calculate_iron_factor(self) -> float:
        """
        Calculate iron stacking factor (stacking/iron factor).

        Primary rule: derived from the selected electrical steel lamination thickness.
        Fallback rule: frequency-based default if steel thickness is unavailable.
        
        Returns:
            ki: stacking factor (dimensionless)
        """
        steel_data = getattr(self.specs, 'steel_data', None) or {}
        thickness = steel_data.get('Thickness_mm', None)
        if thickness is not None:
            try:
                return get_stacking_factor(float(thickness))
            except (TypeError, ValueError):
                pass

        # Fallback (legacy behavior)
        if self.specs.frequency_hz > 60:
            return 0.92
        return 0.95

    def _find_standard_wire(self, As_calc: float, max_diameter: float = 1.8) -> Optional[Dict[str, float]]:
        """Select standard strands minimizing oversize (<15%), prioritizing closest area"""
        standard_wires = [
            (0.80, 0.503), (0.85, 0.567), (0.90, 0.636), (0.95, 0.709),
            (1.00, 0.785), (1.06, 0.882), (1.12, 0.985), (1.18, 1.094),
            (1.25, 1.227), (1.32, 1.368), (1.40, 1.539), (1.50, 1.767),
            (1.60, 2.011), (1.70, 2.270), (1.80, 2.545)
        ]

        best = None
        for n_strands in range(1, 21):
            target_strand_area = As_calc / n_strands
            valid_wires = [w for w in standard_wires if w[1] >= target_strand_area and w[0] <= max_diameter]
            if not valid_wires:
                continue
            d_std, a_std = valid_wires[0]
            total_area = n_strands * a_std
            oversize_percentage = ((total_area - As_calc) / As_calc) * 100
            if oversize_percentage < 0:
                continue
            if oversize_percentage < 15:
                if best is None or oversize_percentage < best['oversize'] or (math.isclose(oversize_percentage, best['oversize']) and n_strands < best['strands']):
                    best = {
                        "strands": n_strands,
                        "diameter_mm": d_std,
                        "area_actual": total_area,
                        "oversize": oversize_percentage,
                    }

        return best
    
    def _calculate_cooling_system(self, stack_length_m: float) -> Tuple[int, float]:
        """
        Calculates the number of radial ventilation ducts and their width.

        Standard rule-of-thumb:
        - If stack length is small (<= ~125mm), keep solid (no ducts).
        - For longer stacks, add ducts to keep iron packets reasonably thin.
        - Duct width is usually ~10mm.

        The iterative rule here chooses the smallest number of ducts such that
        the resulting packet width is <= 60mm.

        Args:
            stack_length_m: Stack length in meters

        Returns:
            Tuple of (num_canals, canal_width_m)
        """
        # If the GUI/user provided an override for number of ducts, use it directly.
        override_ncc = getattr(self.specs, 'override_num_cooling_canals', None)
        if override_ncc is not None:
            try:
                ncc_val = int(override_ncc)
            except (TypeError, ValueError):
                ncc_val = None
            if ncc_val is not None:
                # Safety clamp (GUI should already enforce 0..2)
                ncc_val = max(0, min(2, ncc_val))
                return ncc_val, 10.0 / 1000.0

        stack_length_mm = stack_length_m * 1000.0

        # Threshold check (keep small 100–125mm stacks solid for cost/manufacturing)
        if stack_length_mm <= 125.0:
            return 0, 0.0

        DUCT_WIDTH_MM = 10.0

        # Start with 2 packets (1 duct) and increase until packet width is acceptable
        num_packets = 2
        while True:
            num_ducts = num_packets - 1
            total_duct_width = num_ducts * DUCT_WIDTH_MM
            total_iron_length = stack_length_mm - total_duct_width
            if total_iron_length <= 0:
                return 0, 0.0

            packet_width = total_iron_length / num_packets

            if packet_width <= 60.0:
                return num_ducts, DUCT_WIDTH_MM / 1000.0

            num_packets += 1

            # Safety break
            if num_packets > 50:
                return 0, 0.0
    
    def _validate_and_correct_D_L(self, D: float, L: float, P: int, D2L: float) -> Tuple[float, float, float, Dict]:
        """Validate D, L against peripheral speed only and correct if needed"""
        validation_info = {
            'initial_peripheral_speed': math.pi * D * self.electrical.n_s_rps,
            'peripheral_speed_limit': PERIPHERAL_SPEED_LIMIT,
            'peripheral_speed_limited': False,
        }

        if validation_info['initial_peripheral_speed'] <= PERIPHERAL_SPEED_LIMIT:
            tau_pole = math.pi * D / P
            validation_info['peripheral_speed_final'] = validation_info['initial_peripheral_speed']
            return D, L, tau_pole, validation_info

        # Clamp D to the maximum allowable value from the speed formula, then recompute L
        D_limit = PERIPHERAL_SPEED_LIMIT / (math.pi * self.electrical.n_s_rps)
        D = D_limit
        L = D2L / (D * D)
        tau_pole = math.pi * D / P

        validation_info['peripheral_speed_limited'] = True
        validation_info['D_speed_limit'] = D_limit
        validation_info['peripheral_speed_final'] = math.pi * D * self.electrical.n_s_rps

        return D, L, tau_pole, validation_info
    
    # ═══════════════════════════════════════════════════════════════════════
    # PHASE 3: MAIN DIMENSIONS (D and L)
    # ═══════════════════════════════════════════════════════════════════════
    
    def calculate_main_dimensions(self) -> StatorGeometry:
        """Calculate bore diameter D and active length L"""
        if self.electrical is None:
            self.calculate_electrical_parameters()
        
        params = self.get_empirical_parameters()
        B_av = params['B_av']
        q = params['q_default']
        
        # Winding factor K_w is always computed from coil pitch selection (no fixed default).
        q_for_kw = int(self._determine_q())
        if q_for_kw <= 0:
            # Fall back to empirical q (should be valid in normal operation).
            try:
                q_for_kw = int(q)
            except Exception:
                q_for_kw = 0
        if q_for_kw <= 0:
            q_for_kw = 1

        coil_pitch_percent = getattr(self.specs, 'coil_pitch_percent', None)
        try:
            coil_pitch_percent = float(coil_pitch_percent) if coil_pitch_percent is not None else 100.0
        except Exception:
            coil_pitch_percent = 100.0

        # alpha = 180/(q*3) [deg]
        # kd = sin(q*alpha/2) / (q*sin(alpha/2))
        # electrical coil span = (coil_pitch[%]/100) * 180 [deg]
        # kp = sin(electrical_coil_span/2)
        # Kw = kd * kp
        alpha_deg = 180.0 / (q_for_kw * 3.0)
        alpha_rad = math.radians(alpha_deg)
        denom = q_for_kw * math.sin(alpha_rad / 2.0)
        kd = (math.sin(q_for_kw * alpha_rad / 2.0) / denom) if abs(denom) > 1e-12 else 0.0

        electrical_coil_span_deg = (coil_pitch_percent / 100.0) * 180.0
        kp = math.sin(math.radians(electrical_coil_span_deg / 2.0))

        K_w = float(kd * kp)
        
        # Store K_w for later use
        self._K_w = K_w
        
        # Output Coefficient: Co = 11 * Bav * efficiency * cos(phi) * Kw * q * 10^-3
        Co = 11 * B_av * self.electrical.efficiency * self.electrical.cos_phi * K_w * q * 1e-3
        
        # D²L = P / (Co * ns) [m³]
        # P in kW, ns in rps
        D2L = self.specs.power_kw / (Co * self.electrical.n_s_rps)
        
        P = self.specs.poles
        method = getattr(self.specs, 'dimension_method', 'best_pf') or 'best_pf'
        lambda_info: Dict = {}
        D_raw = L_raw = 0.0

        # Highest priority: user-provided lambda override (L/tau)
        lambda_override = getattr(self.specs, 'override_lambda_ratio', None)
        if lambda_override is not None:
            try:
                lambda_override_val = float(lambda_override)
            except (TypeError, ValueError):
                lambda_override_val = None
            if lambda_override_val is not None and lambda_override_val > 0:
                method = 'custom_lambda'
                lambda_ratio = lambda_override_val
                lambda_info = {
                    'method': 'custom_lambda',
                    'lambda_ratio': lambda_ratio,
                    'range': None,
                    'basis': 'user override',
                    'note': 'Using user-specified lambda (L/τ) override'
                }
                D_raw, L_raw = self._compute_dimensions_from_lambda(lambda_ratio, D2L, P)
            else:
                lambda_override_val = None

        if D_raw == 0.0 and L_raw == 0.0 and method == 'lambda_by_poles':
            lambda_ratio, lambda_info = self._select_lambda_by_poles()
            D_raw, L_raw = self._compute_dimensions_from_lambda(lambda_ratio, D2L, P)
        elif D_raw == 0.0 and L_raw == 0.0 and method == 'preference_based':
            lambda_ratio, lambda_info = self._select_lambda_from_preferences(self.specs.preference_flags)
            D_raw, L_raw = self._compute_dimensions_from_lambda(lambda_ratio, D2L, P)
        elif D_raw == 0.0 and L_raw == 0.0:
            # Method A: existing best power factor heuristic
            lambda_info = {'method': 'best_pf', 'note': 'Best power factor heuristic'}
            L_raw = math.sqrt(D2L / (0.018225 * P * P))
            D_raw = 0.135 * P * math.sqrt(L_raw)
            tau_raw = math.pi * D_raw / P if D_raw != 0 else 0
            lambda_ratio = (L_raw / tau_raw) if tau_raw != 0 else None
            lambda_info['lambda_ratio'] = lambda_ratio

        # Enforce peripheral speed limit while maintaining D²L
        D_speed, L_speed, speed_info = self._apply_peripheral_speed_limit(D_raw, D2L)

        # No manufacturability rounding: keep the computed dimensions.
        # Only the peripheral-speed limit may modify D (and then L is recomputed from D²L).
        D_final = D_speed
        L_final = L_speed
        L_unrounded_after_roundD = L_final

        # Pole pitch with final D
        tau_pole = math.pi * D_final / P
        L_over_tau = L_final / tau_pole if tau_pole != 0 else 0
        v_final = math.pi * D_final * self.electrical.n_s_rps

        # Calculate cooling system and iron factor using final L
        ki = self._calculate_iron_factor()
        num_cooling_canals, canal_width_m = self._calculate_cooling_system(L_final)

        # Stack length after removing cooling duct widths (physical iron stack, before stacking factor)
        # Ls = (L - Ncc * Ncm)
        Ls = max(0.0, (L_final - (num_cooling_canals * canal_width_m)))

        # Net iron length (effective magnetic length accounting for stacking factor)
        # Li = Ls * ki
        Li = max(0.0, Ls * ki)

        # Flux per pole: Φ = Bav * π * L * D / P [Wb]
        flux_per_pole = B_av * math.pi * L_final * D_final / P

        # Semi-open/Open slot: select tooth/yoke flux density targets based on steel selection
        forward_targets: Dict[str, float] = {}
        slot_method = getattr(self.specs, 'slot_method', 'semi_open_slot') or 'semi_open_slot'
        if slot_method in ('semi_open_slot', 'semi_open_slot_new', 'open_slot'):
            forward_targets = self._select_forward_design_flux_densities()

        validation_info = {
            'method': method,
            'lambda_info': lambda_info,
            'D2L': D2L,
            'D_raw': D_raw,
            'L_raw': L_raw,
            'D_after_speed': D_speed,
            'L_after_speed': L_speed,
            'L_unrounded_after_roundD': L_unrounded_after_roundD,
            'D_final': D_final,
            'L_final': L_final,
            'tau_pole': tau_pole,
            'L_over_tau': L_over_tau,
            'peripheral_speed_final': v_final,
            'speed_info': speed_info,
            'ki': ki,
            'num_cooling_canals': num_cooling_canals,
            'canal_width': canal_width_m,
            'Ls': Ls,
            'Li': Li,
            # Flux per pole from Bav-based sizing (may be adjusted later after TPH rounding)
            'flux_per_pole_from_Bav': flux_per_pole,
            # Forward design targets (if enabled)
            **({
                'B_tooth_target': forward_targets.get('B_tooth_target'),
                'B_yoke_target': forward_targets.get('B_yoke_target'),
                'steel_max_B': forward_targets.get('steel_max_B'),
            } if forward_targets else {}),
        }

        # Temporary placeholder for h_slot and d_cs (will be calculated later)
        self.geometry = StatorGeometry(
            D=D_final,
            L=L_final,
            Ls=Ls,
            Li=Li,
            D_ext=0,  # Will be calculated after slot design
            tau_pole=tau_pole,
            flux_per_pole=flux_per_pole,
            h_slot=0,
            d_cs=0,
            ki=ki,
            num_cooling_canals=num_cooling_canals,
            canal_width=canal_width_m,
            validation_info=validation_info
        )
        
        return self.geometry

    # ═══════════════════════════════════════════════════════════════════════
    # PHASE 4: WINDING DESIGN (BEFORE SLOT GEOMETRY)
    # ═══════════════════════════════════════════════════════════════════════
    
    def calculate_turns_and_conductors(self) -> Tuple[int, int, int, int]:
        """Calculate turns per phase and conductors"""
        if self.geometry is None:
            self.calculate_main_dimensions()
        
        params = self.get_empirical_parameters()
        
        # Use stored K_w from calculate_main_dimensions
        K_w = float(getattr(self, '_K_w', 0.0) or 0.0)
        if K_w <= 0:
            raise ValueError("Invalid K_w (not computed)")
        
        # Turns per phase: TPH = E_ph / (4.44 * Phi * f * Kw)
        # Use the original Bav-based Phi for this step (before any post-rounding adjustment).
        geom_val = getattr(self.geometry, 'validation_info', None) or {}
        phi_from_bav = geom_val.get('flux_per_pole_from_Bav', self.geometry.flux_per_pole)
        TPH_initial = self.electrical.E_ph / (4.44 * phi_from_bav * self.specs.frequency_hz * K_w)
        # IMPORTANT: Turns must be an integer and cannot be less than the computed value.
        # Always round UP (ceiling): e.g., 81.01 -> 82.
        # Small tolerance avoids ceil(81.0000000001) becoming 82 due to float noise.
        TPH_initial = int(math.ceil(float(TPH_initial) - 1e-12))
        
        # Number of stator slots: Ss = P * m * q
        # q is taken directly from Table 7.5 (TABLE_Q_RECOMMENDATIONS)
        q = self._determine_q()
        P = self.specs.poles
        m = self.specs.phases
        Ss = P * m * q
        
        # Total stator conductors: Zs = TPH * 2 * 3
        Zs_initial = TPH_initial * 2 * 3
        
        # Conductors per slot: Nc = Zs / Ss
        Nc = Zs_initial / Ss
        
        # Check if Nc is integer
        if Nc != int(Nc):
            # Round Nc UP to the next integer (always choose the closest bigger integer)
            # Example: 7.1 -> 8
            Nc = int(math.ceil(Nc))
            
            # Recalculate Zs: Zs = Nc * (Ss / 3)
            Zs = int(Nc * (Ss / 3))
            
            # Recalculate TPH: TPH = Zs / 2
            # Always round UP if Zs/2 is not an integer
            TPH = int(math.ceil(float(Zs) / 2.0 - 1e-12))
        else:
            Nc = int(Nc)
            Zs = Zs_initial
            TPH = TPH_initial
        
        # Slot pitch check removed by design
        self.slot_pitch_validation = None

        # Recalculate flux per pole using the same EMF equation but with the FINAL (integer) TPH.
        # This adjusted Phi is used for all subsequent steps (slots, yoke, etc.).
        # Phi_recalc = E_ph / (4.44 * f * Kw * TPH)
        if TPH > 0 and self.specs.frequency_hz > 0 and K_w > 0:
            phi_recalc = self.electrical.E_ph / (4.44 * self.specs.frequency_hz * K_w * TPH)

            # Persist both values for GUI display/debugging
            if not hasattr(self.geometry, 'validation_info') or self.geometry.validation_info is None:
                self.geometry.validation_info = {}
            geom_val = self.geometry.validation_info
            geom_val['flux_per_pole_from_Bav'] = phi_from_bav
            geom_val['flux_per_pole_recalc'] = phi_recalc

            # Update the geometry value used downstream
            self.geometry.flux_per_pole = phi_recalc

            # --- NEW: Update stack length L and lengths Ls/Li after Phi is updated ---
            # Rule provided: Phi_new / Phi_old = L_new / L_old
            # Use L_old from main dimensions (the one used in old Ls calculation).
            L_old = float(geom_val.get('L_final', getattr(self.geometry, 'L', 0.0)) or 0.0)
            Ls_old = float(geom_val.get('Ls', getattr(self.geometry, 'Ls', 0.0)) or 0.0)
            Li_old = float(geom_val.get('Li', getattr(self.geometry, 'Li', 0.0)) or 0.0)
            phi_old = float(phi_from_bav) if phi_from_bav else 0.0

            if L_old > 0 and phi_old > 0:
                ratio_phi = float(phi_recalc) / phi_old
                L_new_calc = L_old * ratio_phi

                # Recompute ducts and Ls using the same formulas as main dimensions
                ki = float(getattr(self.geometry, 'ki', geom_val.get('ki', 0.95)) or 0.95)
                num_cooling_canals_new, canal_width_m_new = self._calculate_cooling_system(L_new_calc)
                Ls_new = max(0.0, (L_new_calc - (num_cooling_canals_new * canal_width_m_new)))
                Li_new = max(0.0, Ls_new * ki)

                # Update geometry used downstream
                self.geometry.L = L_new_calc
                self.geometry.Ls = Ls_new
                try:
                    self.geometry.Li = Li_new
                except Exception:
                    pass
                try:
                    self.geometry.num_cooling_canals = num_cooling_canals_new
                    self.geometry.canal_width = canal_width_m_new
                except Exception:
                    pass

                # Store old/new values for GUI display
                geom_val.update({
                    'L_old_main_dimensions': L_old,
                    'Ls_old_main_dimensions': Ls_old,
                    'Li_old_main_dimensions': Li_old,
                    'flux_ratio_phi': ratio_phi,
                    'L_new_calc': L_new_calc,
                    'num_cooling_canals_new': num_cooling_canals_new,
                    'canal_width_new': canal_width_m_new,
                    'Ls_new': Ls_new,
                    'Li_new': Li_new,
                })
        
        return TPH, Zs, Nc, Ss, q

    # ═══════════════════════════════════════════════════════════════════════
    # PHASE 5: CONDUCTOR AND SLOT DESIGN
    # ═══════════════════════════════════════════════════════════════════════

    # Legacy/experimental slot methods removed by request.
    # Use design_conductor_and_slots() which routes to _design_slots_unified().

    def design_conductor_and_slots(self):
        """Design conductors and stator slots using selected method"""
        slot_method = getattr(self.specs, 'slot_method', 'semi_open_slot') or 'semi_open_slot'
        if slot_method == 'open_slot':
            # Open-slot has two conductor/geometry modes:
            # - HV/HP: rectangular strip sizing + HV pitch/area/ratio geometry
            # - Normal machines: random-wound wire sizing (same as semi-open/tapered) + pitch/area/ratio geometry
            voltage_v = float(getattr(self.specs, 'voltage_v', 0.0) or 0.0)
            power_kw = float(getattr(self.specs, 'power_kw', 0.0) or 0.0)
            open_slot_hv = (voltage_v > 600.0) or (power_kw > 372.85)
            return self._design_slots_unified(conductor_mode='strip' if open_slot_hv else 'wire')
        # Default: Semi open slot method
        return self._design_slots_unified(conductor_mode='wire')
    
    # ═══════════════════════════════════════════════════════════════════════
    # PHASE 6: STATOR OUTER DIAMETER
    # ═══════════════════════════════════════════════════════════════════════
    
    def calculate_outer_diameter(self):
        """Calculate stator outer diameter"""
        # Flux in stator yoke = 1/2 * flux_per_pole
        flux_yoke = 0.5 * self.geometry.flux_per_pole

        # Flux density in yoke/core (may come from Forward Design)
        B_c = self._get_yoke_flux_density_for_outer_diameter()
        
        # Core area: Ac = flux_yoke / Bc [m²]
        A_c = flux_yoke / B_c
        
        # Core depth: Ac = dcs * Li => dcs = Ac / Li [m]
        Li_m = float(getattr(self.geometry, 'Li', 0.0) or 0.0)
        if Li_m <= 0:
            Ls_m = float(getattr(self.geometry, 'Ls', 0.0) or 0.0)
            ki = float(getattr(self.geometry, 'ki', 1.0) or 1.0)
            Li_m = (Ls_m * ki) if (Ls_m > 0 and ki > 0) else 0.0
        d_cs = A_c / Li_m if Li_m > 0 else 0.0
        
        # Outer diameter: D0 = D + 2 * h_slot + 2 * dcs [m]
        # Slot-height source depends on slot method:
        # - Semi open slot: hs = hs0 + hs1 + hs2
        # - Open slot: use hs from open-slot section (open_slot_hs_mm)
        slot_method = getattr(self.specs, 'slot_method', 'semi_open_slot') or 'semi_open_slot'
        h_slot_used = float(getattr(self.geometry, 'h_slot', 0.0) or 0.0)
        h_slot_source = 'geometry.h_slot'
        if slot_method in ('semi_open_slot', 'semi_open_slot_new', 'open_slot'):
            val = getattr(self.geometry, 'validation_info', None) or {}
            hs0_mm = None
            hs1_mm = None
            hs2_mm = None
            if slot_method == 'open_slot':
                try:
                    open_slot_hs_mm = float(val.get('open_slot_hs_mm', 0.0) or 0.0)
                except (TypeError, ValueError):
                    open_slot_hs_mm = 0.0
                if open_slot_hs_mm > 0:
                    h_slot_used = open_slot_hs_mm / 1000.0
                    h_slot_source = 'open_slot_hs_mm'
            else:
                hs0_mm = val.get('hs0_mm', None)
                hs1_mm = val.get('hs1_mm', None)
                hs2_mm = val.get('hs2_mm', None)
            if slot_method in ('semi_open_slot', 'semi_open_slot_new'):
                try:
                    hs0_mm_val = float(hs0_mm) if hs0_mm is not None else None
                except (TypeError, ValueError):
                    hs0_mm_val = None
                try:
                    hs1_mm_val = float(hs1_mm) if hs1_mm is not None else None
                except (TypeError, ValueError):
                    hs1_mm_val = None
                try:
                    hs2_mm_val = float(hs2_mm) if hs2_mm is not None else None
                except (TypeError, ValueError):
                    hs2_mm_val = None
                if hs0_mm_val is not None and hs1_mm_val is not None and hs2_mm_val is not None:
                    hs_total_mm = hs0_mm_val + hs1_mm_val + hs2_mm_val
                    if hs_total_mm > 0:
                        h_slot_used = hs_total_mm / 1000.0
                        h_slot_source = 'hs_total_mm'
                        # Store breakdown for GUI
                        if not hasattr(self.geometry, 'validation_info') or self.geometry.validation_info is None:
                            self.geometry.validation_info = {}
                        self.geometry.validation_info['hs_total_mm'] = hs_total_mm
                        self.geometry.validation_info['hs_total_breakdown_mm'] = {
                            'hs0_mm': hs0_mm_val,
                            'hs1_mm': hs1_mm_val,
                            'hs2_mm': hs2_mm_val,
                        }

        # Core weight (as requested):
        # D_mean [m] = D + 2*hs + dcs   (mean diameter at mid-yoke)
        # Wc [kg] = pi * D_mean * A_c * Kgm_3
        if not hasattr(self.geometry, 'validation_info') or self.geometry.validation_info is None:
            self.geometry.validation_info = {}
        val_out = self.geometry.validation_info
        steel_grade = getattr(self.specs, 'steel_grade', None)
        Kgm_3 = float(get_steel_density_kg_m3(steel_grade))
        D_mean_m = float(self.geometry.D) + 2.0 * float(h_slot_used) + float(d_cs)
        Wc_kg = math.pi * float(D_mean_m) * float(A_c) * float(Kgm_3) if (D_mean_m > 0 and A_c > 0 and Kgm_3 > 0) else 0.0
        val_out['Wc_kg'] = Wc_kg
        val_out['Wc_D_mean_m'] = float(D_mean_m)
        val_out['Wc_Ac_m2'] = float(A_c)
        val_out['Wc_dcs_m_used'] = float(d_cs)
        val_out['Wc_hs_m_used'] = float(h_slot_used)
        val_out['Wc_hs_source'] = str(h_slot_source)
        val_out['Wc_density_kg_m3_used'] = float(Kgm_3)
        val_out['Wc_steel_grade_used'] = str(steel_grade) if steel_grade is not None else None

        # Iron losses (as requested):
        # Pit = Wt * Pkg   [W]
        # Pic = Wc * Pkg   [W]
        # where Pkg is Loss_W_kg from selected lamination
        Pkg = float(get_steel_loss_w_per_kg(steel_grade))
        Wt_kg = val_out.get('Wt_kg', None)
        if Wt_kg is None:
            # Tooth weight may have been computed earlier in a different step
            Wt_kg = (getattr(self.geometry, 'validation_info', None) or {}).get('Wt_kg', None)
        try:
            Wt_kg_val = float(Wt_kg) if Wt_kg is not None else None
        except (TypeError, ValueError):
            Wt_kg_val = None

        Pit_W = float(Wt_kg_val) * Pkg if (Wt_kg_val is not None and Wt_kg_val > 0 and Pkg > 0) else 0.0
        Pic_W = float(Wc_kg) * Pkg if (Wc_kg > 0 and Pkg > 0) else 0.0
        val_out['iron_loss_Pkg_W_per_kg_used'] = Pkg
        val_out['iron_loss_steel_grade_used'] = str(steel_grade) if steel_grade is not None else None
        val_out['Pit_W'] = Pit_W
        val_out['Pic_W'] = Pic_W

        D_ext_calculated = self.geometry.D + 2 * h_slot_used + 2 * d_cs
        D_ext = self._round_to_nearest_10mm(D_ext_calculated)
        
        # Update geometry and store both values
        self.geometry.D_ext = D_ext
        self.geometry.d_cs = d_cs
        if not hasattr(self.geometry, 'validation_info'):
            self.geometry.validation_info = {}
        self.geometry.validation_info['D_ext_calculated'] = D_ext_calculated
        self.geometry.validation_info['D_ext_rounded'] = D_ext

        # Record which slot height was used for outer diameter
        self.geometry.validation_info['h_slot_used_for_Dext'] = h_slot_used
        self.geometry.validation_info['h_slot_source_for_Dext'] = h_slot_source

        # Store yoke flux density used
        self.geometry.validation_info['B_c_used'] = B_c
        
        method = getattr(self.specs, 'slot_method', 'semi_open_slot') or 'semi_open_slot'
        if method in ('semi_open_slot', 'open_slot'):
            self._validation_messages.append(
                f"✓ Yoke flux density B_yoke = {B_c:.2f} T (steel-based target; base range: 1.4-1.7 T)"
            )
        else:
            self._validation_messages.append(
                f"✓ Core flux density Bc = {B_c:.2f} T (acceptable range: 1.2-1.4 T)"
            )
    
    # ═══════════════════════════════════════════════════════════════════════
    # PUBLIC API METHODS
    # ═══════════════════════════════════════════════════════════════════════
    
    def design_complete(self) -> Dict:
        """Execute complete stator design in sequence"""
        print("╔" + "═" * 82 + "╗")
        print("║" + " " * 82 + "║")
        print("║" + "STATOR DESIGN - THREE-PHASE ASYNCHRONOUS MOTOR".center(82) + "║")
        print("║" + " " * 82 + "║")
        print("╚" + "═" * 82 + "╝")
        print()
        
        # Phase 1: Electrical Parameters
        print("⚡ PHASE 1: ELECTRICAL PARAMETERS")
        print("-" * 82)
        self.calculate_electrical_parameters()
        print(self.electrical)
        print()
        
        # Phase 2: Main Dimensions
        print("📐 PHASE 2: MAIN DIMENSIONS (D, L)")
        print("-" * 82)
        self.calculate_main_dimensions()
        # Print partial geometry (without D_ext)
        print(f"""
    ╔══════════════════════════════════════════════╗
    ║         STATOR MAIN DIMENSIONS (Partial)     ║
    ╠══════════════════════════════════════════════╣
    ║ Bore diameter D         : {self.geometry.D*1000:>10.2f} mm  ║
    ║ Active length L         : {self.geometry.L*1000:>10.2f} mm  ║
    ║ Pole pitch τp           : {self.geometry.tau_pole*1000:>10.2f} mm  ║
    ║ Flux per pole Φ         : {self.geometry.flux_per_pole*1000:>10.4f} mWb ║
    ╚══════════════════════════════════════════════╝""")
        print()
        
        # Phase 3: Winding & Slot Design
        print("🔌 PHASE 3: WINDING & SLOT DESIGN")
        print("-" * 82)
        self.design_conductor_and_slots()
        print(self.winding)
        print()
        print(self.slots)
        print()
        
        # Phase 4: Outer Diameter
        print("🔧 PHASE 4: OUTER DIAMETER CALCULATION")
        print("-" * 82)
        self.calculate_outer_diameter()
        print(f"""
    ╔══════════════════════════════════════════════╗
    ║         STATOR OUTER GEOMETRY                ║
    ╠══════════════════════════════════════════════╣
    ║ External diameter D_ext : {self.geometry.D_ext*1000:>10.2f} mm  ║
    ║ Core depth d_cs         : {self.geometry.d_cs*1000:>10.2f} mm  ║
    ╚══════════════════════════════════════════════╝""")
        print()
        
        # Validation Messages
        print("✅ VALIDATION CHECKS")
        print("-" * 82)
        for msg in self._validation_messages:
            print(msg)
        print()
        
        return {
            'specifications': self.specs,
            'electrical': self.electrical,
            'geometry': self.geometry,
            'slots': self.slots,
            'winding': self.winding,
            'messages': self._validation_messages,
        }
    
    def get_summary(self) -> Dict:
        """Get quick summary of stator design"""
        if self.geometry is None or self.slots is None or self.winding is None:
            return {'error': 'Design not completed. Call design_complete() first.'}
        
        return {
            'Motor': {
                'Power': f"{self.specs.power_kw} kW",
                'Voltage': f"{self.specs.voltage_v} V",
                'Frequency': f"{self.specs.frequency_hz} Hz",
                'Poles': self.specs.poles,
                'Connection': self.specs.connection,
            },
            'Steel': {
                'Grade': (self.specs.steel_data or {}).get('Grade', self.specs.steel_grade or '—'),
                'Thickness': (f"{(self.specs.steel_data or {}).get('Thickness_mm'):.2f} mm" if (self.specs.steel_data or {}).get('Thickness_mm') is not None else '—'),
                'Max Design B': (f"{(self.specs.steel_data or {}).get('Max_Design_B'):.2f} T" if (self.specs.steel_data or {}).get('Max_Design_B') is not None else '—'),
                'Loss @1.5T': (f"{(self.specs.steel_data or {}).get('Loss_W_kg'):.2f} W/kg" if (self.specs.steel_data or {}).get('Loss_W_kg') is not None else '—'),
                'Application': (self.specs.steel_data or {}).get('Application', '—'),
            },
            'Electrical': {
                'P_elec': f"{self.electrical.P_elec:.2f} kW",
                'n_s': f"{self.electrical.n_s:.2f} rpm",
                'I_s': f"{self.electrical.I_s:.2f} A",
                'cos_phi': f"{self.electrical.cos_phi:.4f}",
                'efficiency': f"{self.electrical.efficiency:.4f}",
            },
            'Geometry': {
                'D (bore)': f"{self.geometry.D * 1000:.1f} mm",
                'L (active)': f"{self.geometry.L * 1000:.1f} mm",
                'D_ext (outer)': f"{self.geometry.D_ext * 1000:.1f} mm",
                'Flux per pole': f"{self.geometry.flux_per_pole * 1000:.4f} mWb",
            },
            'Slots': {
                'Ss (total slots)': self.slots.Ss,
                'q (slots/pole/phase)': self.slots.q,
                'Nc (conductors/slot)': self.slots.Nc,
                'Slot width': f"{self.slots.b_slot:.2f} mm",
                'Slot height': f"{self.slots.h_slot:.2f} mm",
                'B_tooth': f"{self.slots.B_tooth:.3f} T",
                'B_tooth_max': f"{self.slots.B_tooth_max:.3f} T",
            },
            'Winding': {
                'TPH (turns/phase)': self.winding.TPH,
                'Zs (total conductors)': self.winding.Zs,
                'Conductor type': self.winding.conductor_type,
                'Conductor section': f"{self.winding.As:.2f} mm²",
                'Conductor material': (self.specs.conductor_material or 'Cuivre électrolytique'),
                'R_s @ 20°C': f"{self.winding.R_s:.6f} Ω",
                'Conductor losses': f"{self.winding.copper_losses:.2f} W",
            },
        }


# ═══════════════════════════════════════════════════════════════════════════
# EXAMPLE USAGE
# ═══════════════════════════════════════════════════════════════════════════

if __name__ == "__main__":
    # Define motor specifications
    specs = MotorSpecifications(
        power_kw=15.0,
        voltage_v=400.0,
        frequency_hz=50.0,
        poles=4,
        connection='star',  # or 'delta'
        cooling_type='IC411'
    )
    
    # Create designer and run complete design
    designer = StatorDesign(specs)
    results = designer.design_complete()
    
    # Get summary
    print("\n" + "="*82)
    print("DESIGN SUMMARY")
    print("="*82)
    summary = designer.get_summary()
    for section, params in summary.items():
        print(f"\n{section}:")
        for key, value in params.items():
            print(f"  {key}: {value}")