"""
================================================================================
                    STATOR DESIGN GUI APPLICATION                          
                   Three-Phase Asynchronous Induction Motor                 
                                                                            
  Interactive design tool with step-by-step equation display                
                                                                            
  Author: Motor Design Tool v2.0 - PyQt6 Version                                          
================================================================================
"""

import sys
import os
import math
import json
from dataclasses import dataclass, asdict
from datetime import datetime
from typing import Dict, List, Tuple, Optional, Any
from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, 
    QGridLayout, QLabel, QLineEdit, QPushButton, QTextEdit, QComboBox,
    QRadioButton, QCheckBox, QTabWidget, QGroupBox, QMessageBox,
    QFileDialog, QStatusBar, QButtonGroup, QSlider, QSizePolicy, QToolButton, QStyle, QSpinBox,
    QStackedWidget,
    QScrollArea,
    QDialog, QTableWidget, QTableWidgetItem, QHeaderView,
    QGraphicsView, QGraphicsScene, QGraphicsPathItem, QGraphicsEllipseItem, QGraphicsLineItem,
    QGraphicsSimpleTextItem
)
from PyQt6.QtCore import Qt, QPointF
import PyQt6.QtGui as QtGui
from PyQt6.QtGui import QFont, QTextCursor, QPainter, QPainterPath, QPen, QBrush, QColor, QTransform, QPalette

# Import the stator design module
# Make sure stator_design_v2.py is in the same directory
try:
    from stator_design_v2 import (
        StatorDesign, 
        MotorSpecifications,
        AbaquesTables,
        Interpolator,
        get_steel_grades,
        get_steel_by_grade,
        get_stacking_factor,
        STEEL_DATABASE,
        SWG_GAUGE,
        SWG_DIAMETER_MM
    )
except ImportError:
    print("Error: stator_design_v2.py must be in the same directory")
    sys.exit(1)


# Import the rotor design module (refactored from terminal script)
try:
    from rotor_design_v1 import RotorStatorInputs, RotorUserConfig, run_rotor_design
except ImportError:
    RotorStatorInputs = None
    RotorUserConfig = None
    run_rotor_design = None


@dataclass
class Stator2DParams:
    D_mm: float
    Dext_mm: float
    Ss: int
    slot_type: str  # 'square' or 'trapez'
    # Square
    bs_mm: Optional[float] = None
    hs_mm: Optional[float] = None
    # Trapez
    bs0_mm: Optional[float] = None
    hs0_mm: Optional[float] = None
    bs1_mm: Optional[float] = None
    hs1_mm: Optional[float] = None
    bs2_mm: Optional[float] = None
    hs2_mm: Optional[float] = None


@dataclass
class Rotor2DParams:
    Dr_mm: float
    Nr: int
    shaft_d_mm: float
    # Teardrop slot geometry
    h0_mm: float
    b0_mm: float
    r1_mm: float
    r2_mm: float
    drb_mm: float


def _stator2d_slot_fillet_radius(p: Stator2DParams) -> float:
    if p.slot_type != 'trapez':
        return 0.0
    bs0 = float(p.bs0_mm or 0.0)
    base = bs0 * 0.12
    return max(0.35, min(1.2, base if base > 0 else 0.6))


def _stator2d_rounded_polygon_path(points: list[tuple[float, float]], radius: float) -> QPainterPath:
    if not points:
        return QPainterPath()
    if radius <= 0 or len(points) < 3:
        path = QPainterPath()
        x0, y0 = points[0]
        path.moveTo(x0, y0)
        for x, y in points[1:]:
            path.lineTo(x, y)
        path.closeSubpath()
        return path

    # Quadratic-rounded corners (same concept as pathFromXYRounded in HTML)
    pts = [{'x': float(x), 'y': float(y)} for x, y in points]
    n = len(pts)

    def _len(vx, vy):
        return math.hypot(vx, vy)

    def _norm(vx, vy):
        L = max(1e-9, _len(vx, vy))
        return vx / L, vy / L

    def _corner(i: int):
        prev = pts[(i - 1 + n) % n]
        cur = pts[i]
        nxt = pts[(i + 1) % n]
        vpx, vpy = prev['x'] - cur['x'], prev['y'] - cur['y']
        vnx, vny = nxt['x'] - cur['x'], nxt['y'] - cur['y']
        lp, ln = _len(vpx, vpy), _len(vnx, vny)
        d = min(radius, lp * 0.45, ln * 0.45)
        upx, upy = _norm(vpx, vpy)
        unx, uny = _norm(vnx, vny)
        sx, sy = cur['x'] + upx * d, cur['y'] + upy * d
        ex, ey = cur['x'] + unx * d, cur['y'] + uny * d
        return (cur['x'], cur['y']), (sx, sy), (ex, ey)

    cur0, start0, _ = _corner(0)
    path = QPainterPath()
    path.moveTo(start0[0], start0[1])
    for i in range(n):
        cur, start, end = _corner(i)
        path.lineTo(start[0], start[1])
        path.quadTo(cur[0], cur[1], end[0], end[1])
    path.closeSubpath()
    return path


def _stator2d_slot_polygon_square(p: Stator2DParams, theta: float) -> list[tuple[float, float]]:
    Rin = float(p.D_mm) / 2.0
    h = float(p.hs_mm or 0.0)
    width = float(p.bs_mm or 0.0)
    r0, r1 = Rin, Rin + h
    half0 = (width / 2.0) / max(1e-9, r0)
    half1 = (width / 2.0) / max(1e-9, r1)
    left0, left1 = theta - half0, theta - half1
    right0, right1 = theta + half0, theta + half1
    return [
        (r0 * math.cos(left0), r0 * math.sin(left0)),
        (r1 * math.cos(left1), r1 * math.sin(left1)),
        (r1 * math.cos(right1), r1 * math.sin(right1)),
        (r0 * math.cos(right0), r0 * math.sin(right0)),
    ]


def _stator2d_slot_width_at_radius_trapez(p: Stator2DParams, r: float, r1: float, r2: float, r3: float) -> float:
    def lerp(a: float, b: float, t: float) -> float:
        t2 = max(0.0, min(1.0, float(t)))
        return a + (b - a) * t2

    bs0 = float(p.bs0_mm or 0.0)
    bs1 = float(p.bs1_mm or 0.0)
    bs2 = float(p.bs2_mm or 0.0)
    if r <= r1:
        return bs0
    if r <= r2:
        return lerp(bs0, bs1, (r - r1) / max(1e-9, (r2 - r1)))
    if r <= r3:
        return lerp(bs1, bs2, (r - r2) / max(1e-9, (r3 - r2)))
    return bs2


def _stator2d_slot_polygon_trapez(p: Stator2DParams, theta: float) -> list[tuple[float, float]]:
    Rin = float(p.D_mm) / 2.0
    r0 = Rin
    r1 = Rin + float(p.hs0_mm or 0.0)
    r2 = Rin + float(p.hs0_mm or 0.0) + float(p.hs1_mm or 0.0)
    r3 = Rin + float(p.hs0_mm or 0.0) + float(p.hs1_mm or 0.0) + float(p.hs2_mm or 0.0)
    radii = [r0, r1, r2, r3]
    widths = [_stator2d_slot_width_at_radius_trapez(p, rr, r1, r2, r3) for rr in radii]
    half = [(w / 2.0) / max(1e-9, radii[i]) for i, w in enumerate(widths)]
    left = [theta - ha for ha in half]
    right = [theta + ha for ha in half]
    pts: list[tuple[float, float]] = []
    for i, rr in enumerate(radii):
        a = left[i]
        pts.append((rr * math.cos(a), rr * math.sin(a)))
    for i in range(len(radii) - 1, -1, -1):
        rr = radii[i]
        a = right[i]
        pts.append((rr * math.cos(a), rr * math.sin(a)))
    return pts


class Rotor2DWidget(QWidget):
    """Rotor 2D cross-section + slot zoom (teardrop slot only)."""

    def __init__(self, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self._params: Optional[Rotor2DParams] = None
        self._selected_slot_index = 0

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)

        controls = QGroupBox("Rotor 2D view")
        controls_layout = QGridLayout(controls)
        self.fit_button = QPushButton("Fit views")
        self.fit_button.clicked.connect(self._fit_views)
        controls_layout.addWidget(self.fit_button, 0, 0)

        self.collected_data_button = QPushButton("Collected data")
        self.collected_data_button.clicked.connect(self._show_collected_data_dialog)
        controls_layout.addWidget(self.collected_data_button, 0, 1)

        layout.addWidget(controls, 0)

        views_row = QWidget()
        views_layout = QHBoxLayout(views_row)
        views_layout.setContentsMargins(0, 0, 0, 0)

        left_col = QVBoxLayout()
        right_col = QVBoxLayout()

        self.cross_scene = QGraphicsScene(self)
        self.cross_view = ZoomPanGraphicsView()
        self.cross_view.setScene(self.cross_scene)
        self.cross_view.setProperty("role", "canvas")
        self.cross_view.setBackgroundBrush(QBrush(Qt.GlobalColor.white))
        self.cross_scene.setBackgroundBrush(QBrush(Qt.GlobalColor.white))
        self.cross_view.setMinimumHeight(360)
        left_col.addWidget(QLabel("Rotor cross-section"), 0)
        left_col.addWidget(self.cross_view, 1)

        self.slot_scene = QGraphicsScene(self)
        self.slot_view = StaticGraphicsView()
        self.slot_view.setScene(self.slot_scene)
        self.slot_view.setProperty("role", "canvas")
        self.slot_view.setBackgroundBrush(QBrush(Qt.GlobalColor.white))
        self.slot_scene.setBackgroundBrush(QBrush(Qt.GlobalColor.white))
        self.slot_view.setMinimumHeight(360)
        right_col.addWidget(QLabel("Rotor slot geometry (zoom)"), 0)
        right_col.addWidget(self.slot_view, 1)

        views_layout.addLayout(left_col, 1)
        views_layout.addLayout(right_col, 1)
        layout.addWidget(views_row, 1)

        self._fit_views()

    def set_params(self, params: Rotor2DParams):
        self._params = params
        self._selected_slot_index = 0
        self._redraw_all()

    def clear_params(self):
        self._params = None
        self._selected_slot_index = 0
        self.cross_scene.clear()
        self.slot_scene.clear()
        self.cross_scene.setSceneRect(-10, -10, 20, 20)
        self.slot_scene.setSceneRect(-10, -10, 20, 20)

    def has_params(self) -> bool:
        return self._params is not None

    def _collected_data_rows(self) -> list[tuple[str, str]]:
        p = self._params
        if p is None:
            return []
        return [
            ("Dr", f"{p.Dr_mm:.3f} mm"),
            ("Shaft diameter", f"{p.shaft_d_mm:.3f} mm"),
            ("Nr", f"{p.Nr}"),
            ("Slot type", "teardrop"),
            ("h0", f"{p.h0_mm:.3f} mm"),
            ("b0", f"{p.b0_mm:.3f} mm"),
            ("r1", f"{p.r1_mm:.3f} mm"),
            ("r2", f"{p.r2_mm:.3f} mm"),
            ("drb", f"{p.drb_mm:.3f} mm"),
        ]

    def _show_collected_data_dialog(self):
        rows = self._collected_data_rows()
        if not rows:
            QMessageBox.information(self, "Collected data", "No rotor data available.")
            return

        dlg = QDialog(self)
        dlg.setWindowTitle("Collected data")
        dlg.setModal(True)
        dlg.setMinimumWidth(520)
        layout = QVBoxLayout(dlg)

        table = QTableWidget(len(rows), 2, dlg)
        table.setHorizontalHeaderLabels(["Parameter", "Value"])
        table.verticalHeader().setVisible(False)
        # Prevent header text from being clipped (especially on scroll/selection with the dark theme).
        try:
            hh = table.horizontalHeader()
            hh.setDefaultAlignment(Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter)
            hh.setMinimumHeight(30)
            hh.setFixedHeight(30)
        except Exception:
            pass
        table.setStyleSheet(
            "QHeaderView::section { padding: 6px 10px; }"
        )
        table.setEditTriggers(QTableWidget.EditTrigger.NoEditTriggers)
        table.setSelectionBehavior(QTableWidget.SelectionBehavior.SelectRows)
        table.setSelectionMode(QTableWidget.SelectionMode.SingleSelection)
        table.horizontalHeader().setStretchLastSection(True)
        table.horizontalHeader().setSectionResizeMode(0, QHeaderView.ResizeMode.ResizeToContents)
        table.horizontalHeader().setSectionResizeMode(1, QHeaderView.ResizeMode.Stretch)
        for r, (k, v) in enumerate(rows):
            table.setItem(r, 0, QTableWidgetItem(str(k)))
            table.setItem(r, 1, QTableWidgetItem(str(v)))
        layout.addWidget(table, 1)

        close_btn = QPushButton("Close")
        close_btn.clicked.connect(dlg.accept)
        layout.addWidget(close_btn, 0)

        dlg.exec()

    def _fit_views(self):
        # Slot zoom view is meant to be stationary; don't let the cross-section fit button
        # affect it.
        if self.cross_view is not None:
            self.cross_view.reset_view()

    def _redraw_all(self):
        self._redraw_cross_section()
        self._redraw_slot_zoom()
        self._fit_views()

    def _on_slot_clicked(self, index: int):
        self._selected_slot_index = int(index)
        self._redraw_cross_section()

    @staticmethod
    def _arc_points_circle_svg(start: tuple[float, float], end: tuple[float, float], r: float, large_arc: bool, sweep_down: bool, steps: int) -> list[tuple[float, float]]:
        """Approximate an SVG circle arc with a polyline.

        The SVG sweep flag is defined in the user coordinate system (y down). We compute the
        arc in a standard math coordinate system (y up) by flipping y; this flips the sweep.
        """
        r = abs(float(r))
        if r <= 1e-9:
            return [end]
        (x1, y1) = start
        (x2, y2) = end

        # To math coords (y up)
        x1u, y1u = float(x1), -float(y1)
        x2u, y2u = float(x2), -float(y2)
        sweep = (not bool(sweep_down))
        large = bool(large_arc)

        dx = (x1u - x2u) / 2.0
        dy = (y1u - y2u) / 2.0
        d2 = dx * dx + dy * dy
        if d2 <= 1e-18:
            return [end]

        # SVG scales radii up if too small.
        if d2 > r * r:
            r = math.sqrt(d2)

        sign = -1.0 if (large == sweep) else 1.0
        num = max(0.0, r * r - d2)
        den = max(1e-18, d2)
        factor = sign * math.sqrt(num / den)
        cxp = factor * dy
        cyp = factor * (-dx)

        mx = (x1u + x2u) / 2.0
        my = (y1u + y2u) / 2.0
        cx = cxp + mx
        cy = cyp + my

        a1 = math.atan2(y1u - cy, x1u - cx)
        a2 = math.atan2(y2u - cy, x2u - cx)
        da = a2 - a1

        if sweep and da < 0:
            da += 2.0 * math.pi
        if (not sweep) and da > 0:
            da -= 2.0 * math.pi

        if large and abs(da) < math.pi:
            da += 2.0 * math.pi if sweep else -2.0 * math.pi
        if (not large) and abs(da) > math.pi:
            da += -2.0 * math.pi if sweep else 2.0 * math.pi

        steps_i = max(6, int(steps))
        pts: list[tuple[float, float]] = []
        for i in range(1, steps_i + 1):
            t = a1 + da * (float(i) / float(steps_i))
            xu = cx + r * math.cos(t)
            yu = cy + r * math.sin(t)
            pts.append((xu, -yu))
        pts[-1] = (float(x2), float(y2))
        return pts

    @staticmethod
    def _teardrop_local_path_from_params(p: Rotor2DParams) -> QPainterPath:
        """Teardrop slot path matching motor_app_fixed.html.

        HTML local coords: x = tangential, y = radial inward from OD (y down).
        Our local coords:  x = radial inward, y = tangential.
        Map: (hx, hy) -> (x=hy, y=hx)
        """

        def nz(v: float) -> float:
            return max(0.0, float(v))

        h0 = nz(p.h0_mm)
        b0 = nz(p.b0_mm)
        r1 = nz(p.r1_mm)
        r2 = nz(p.r2_mm)
        drb = nz(p.drb_mm)

        drb_after = max(0.0, drb)
        drb_centers = max(0.0, drb_after - r1 - r2)
        cy1 = h0 + r1
        cy2 = h0 + r1 + drb_centers
        denom = max(1e-9, drb_centers)
        theta = math.asin(max(-1.0, min(1.0, (r2 - r1) / denom)))

        t1x = r1 * math.cos(theta)
        t1y = -r1 * math.sin(theta)
        t2x = r2 * math.cos(theta)
        t2y = -r2 * math.sin(theta)

        p1R = (t1x, cy1 + t1y)
        p2R = (t2x, cy2 + t2y)
        p1L = (-t1x, cy1 + t1y)
        p2L = (-t2x, cy2 + t2y)

        nTL = (-b0 / 2.0, 0.0)
        nTR = (b0 / 2.0, 0.0)
        nBL = (-b0 / 2.0, h0)
        nBR = (b0 / 2.0, h0)

        poly: list[tuple[float, float]] = [nTL, nTR, nBR]
        if r1 > 1e-9:
            poly.extend(Rotor2DWidget._arc_points_circle_svg(nBR, p1R, r1, large_arc=False, sweep_down=True, steps=18))
        else:
            poly.append(p1R)
        poly.append(p2R)
        if r2 > 1e-9:
            poly.extend(Rotor2DWidget._arc_points_circle_svg(p2R, p2L, r2, large_arc=True, sweep_down=True, steps=28))
        else:
            poly.append(p2L)
        poly.append(p1L)
        if r1 > 1e-9:
            poly.extend(Rotor2DWidget._arc_points_circle_svg(p1L, nBL, r1, large_arc=False, sweep_down=True, steps=18))
        else:
            poly.append(nBL)

        mapped = [(float(hy), float(hx)) for (hx, hy) in poly]
        path = QPainterPath()
        if not mapped:
            return path
        path.moveTo(mapped[0][0], mapped[0][1])
        for x, y in mapped[1:]:
            path.lineTo(x, y)
        path.closeSubpath()
        return path

    def _teardrop_local_path(self) -> QPainterPath:
        p = self._params
        if p is None:
            return QPainterPath()
        return Rotor2DWidget._teardrop_local_path_from_params(p)

    @staticmethod
    def _teardrop_path_at_theta_from_params(p: Rotor2DParams, theta: float) -> QPainterPath:
        R = float(p.Dr_mm) / 2.0
        c = math.cos(theta)
        s = math.sin(theta)
        tr = QTransform(
            -c, -s, 0.0,
            -s,  c, 0.0,
             c * R, s * R, 1.0
        )
        return tr.map(Rotor2DWidget._teardrop_local_path_from_params(p))

    def _teardrop_path_at_theta(self, theta: float) -> QPainterPath:
        p = self._params
        if p is None:
            return QPainterPath()
        return Rotor2DWidget._teardrop_path_at_theta_from_params(p, theta)

    def _redraw_cross_section(self):
        self.cross_scene.clear()
        p = self._params
        if p is None:
            self.cross_scene.setSceneRect(-10, -10, 20, 20)
            return

        R = float(p.Dr_mm) / 2.0
        R_sh = max(0.0, float(p.shaft_d_mm) / 2.0)

        rotor_pen = QPen(QColor(60, 70, 90, 220))
        rotor_pen.setWidthF(1.2)
        rotor_pen.setCosmetic(True)
        rotor_brush = QBrush(QColor(220, 228, 245, 140))

        shaft_pen = QPen(QColor(60, 70, 90, 200))
        shaft_pen.setWidthF(1.0)
        shaft_pen.setCosmetic(True)
        shaft_brush = QBrush(QColor(255, 255, 255, 255))

        outer = QGraphicsEllipseItem(-R, -R, 2 * R, 2 * R)
        outer.setPen(rotor_pen)
        outer.setBrush(rotor_brush)
        self.cross_scene.addItem(outer)

        shaft = QGraphicsEllipseItem(-R_sh, -R_sh, 2 * R_sh, 2 * R_sh)
        shaft.setPen(shaft_pen)
        shaft.setBrush(shaft_brush)
        self.cross_scene.addItem(shaft)

        Nr = max(1, int(p.Nr))
        dtheta = (2.0 * math.pi) / float(Nr)
        for i in range(Nr):
            theta = float(i) * dtheta
            slot_path = self._teardrop_path_at_theta(theta)
            is_sel = (i == int(self._selected_slot_index))
            item = _ClickablePathItem(i, self._on_slot_clicked, slot_path)
            if is_sel:
                sel_pen = QPen(QColor(0, 160, 130, 240))
                sel_pen.setWidthF(1.4)
                sel_pen.setCosmetic(True)
                item.setPen(sel_pen)
                item.setBrush(QBrush(QColor(0, 160, 130, 90)))
            else:
                slot_pen = QPen(QColor(60, 70, 90, 180))
                slot_pen.setWidthF(1.0)
                slot_pen.setCosmetic(True)
                item.setPen(slot_pen)
                item.setBrush(QBrush(QColor(255, 255, 255, 140)))
            self.cross_scene.addItem(item)

        pad = R * 0.18
        self.cross_scene.setSceneRect(-(R + pad), -(R + pad), 2 * (R + pad), 2 * (R + pad))

    def _add_dim_x(self, scene: QGraphicsScene, x1: float, x2: float, y: float, label: str, highlight: bool = False):
        pen = QPen(QColor(0, 160, 130, 235))
        pen.setWidthF(1.0 if not highlight else 1.6)
        pen.setCosmetic(True)
        line = QGraphicsLineItem(x1, y, x2, y)
        line.setPen(pen)
        scene.addItem(line)
        ah = 1.2
        for xx, sgn in ((x1, +1.0), (x2, -1.0)):
            a1 = QGraphicsLineItem(xx, y, xx + sgn * ah, y - ah)
            a1.setPen(pen)
            scene.addItem(a1)
            a2 = QGraphicsLineItem(xx, y, xx + sgn * ah, y + ah)
            a2.setPen(pen)
            scene.addItem(a2)
        t = QGraphicsSimpleTextItem(label)
        t.setBrush(QBrush(QColor(0, 160, 130, 235)))
        text_scale = 0.9 if not highlight else 1.4
        t.setScale(text_scale)
        if highlight:
            t.setFont(QFont("Arial", 12, QFont.Weight.Bold))
        br = t.boundingRect()
        tw = br.width() * text_scale
        t.setPos((x1 + x2) / 2.0 - tw / 2.0, y + 2.0)
        scene.addItem(t)

    def _add_dim_y_abs(self, scene: QGraphicsScene, length: float, x: float, y_center: float, label: str, highlight: bool = False):
        y1 = y_center - length / 2.0
        y2 = y_center + length / 2.0
        pen = QPen(QColor(0, 160, 130, 235))
        pen.setWidthF(1.0 if not highlight else 1.6)
        pen.setCosmetic(True)
        line = QGraphicsLineItem(x, y1, x, y2)
        line.setPen(pen)
        scene.addItem(line)
        ah = 1.2
        for yy, sgn in ((y1, +1.0), (y2, -1.0)):
            a1 = QGraphicsLineItem(x, yy, x - ah, yy + sgn * ah)
            a1.setPen(pen)
            scene.addItem(a1)
            a2 = QGraphicsLineItem(x, yy, x + ah, yy + sgn * ah)
            a2.setPen(pen)
            scene.addItem(a2)
        t = QGraphicsSimpleTextItem(label)
        t.setBrush(QBrush(QColor(0, 160, 130, 235)))
        text_scale = 0.9 if not highlight else 1.4
        t.setScale(text_scale)
        if highlight:
            t.setFont(QFont("Arial", 12, QFont.Weight.Bold))
        br = t.boundingRect()
        tw = br.width() * text_scale
        th = br.height() * text_scale
        pad = 2.5
        tx = (x - tw - pad) if x < 0 else (x + pad)
        t.setPos(tx, y_center - th / 2.0)
        scene.addItem(t)

    def _redraw_slot_zoom(self):
        self.slot_scene.clear()
        p = self._params
        if p is None:
            self.slot_scene.setSceneRect(-10, -10, 20, 20)
            return

        # Fixed vertical schematic (scaled for readability) + dimension callouts.
        # This is a legend-style view; it is not meant to update with every rotor design change.
        scale = 18.0
        h0_mm = 0.500
        b0_mm = 0.800
        r1_mm = 1.269
        r2_mm = 0.508
        drb_mm = 25.877

        p_schem = Rotor2DParams(
            Dr_mm=100.0,
            Nr=1,
            shaft_d_mm=0.0,
            h0_mm=h0_mm * scale,
            b0_mm=b0_mm * scale,
            r1_mm=r1_mm * scale,
            r2_mm=r2_mm * scale,
            drb_mm=drb_mm * scale,
        )

        # Generate the teardrop in its native local coords (depth along +x), then swap axes so depth
        # becomes vertical (+y) and width becomes horizontal (+x).
        base_path = Rotor2DWidget._teardrop_local_path_from_params(p_schem)
        swap = QTransform(0.0, 1.0, 0.0,
                          1.0, 0.0, 0.0,
                          0.0, 0.0, 1.0)
        slot_path = swap.map(base_path)

        slot_item = QGraphicsPathItem(slot_path)
        pen = QPen(QColor(60, 110, 210, 235))
        pen.setWidthF(1.2)
        pen.setCosmetic(True)
        slot_item.setPen(pen)
        slot_item.setBrush(QBrush(QColor(120, 160, 255, 95)))
        self.slot_scene.addItem(slot_item)

        h0 = h0_mm * scale
        b0 = b0_mm * scale
        r1 = r1_mm * scale
        r2 = r2_mm * scale
        drb = drb_mm * scale
        total_depth = h0 + drb
        half_span = max(b0 / 2.0, r1, r2)

        # Dimensions (no numeric values; this is a schematic legend).
        b0_y = -max(30.0, 0.10 * total_depth)
        self._add_dim_x(self.slot_scene, -b0 / 2.0, +b0 / 2.0, b0_y, "b0", highlight=True)

        x_left = -max(95.0, b0 * 1.9)
        x_right = +max(95.0, b0 * 1.9)
        self._add_dim_y_abs(self.slot_scene, h0, x_left, h0 / 2.0, "h0", highlight=True)
        self._add_dim_y_abs(self.slot_scene, drb, x_right, h0 + drb / 2.0, "drb", highlight=True)

        # Radii: show the actual radius construction (center + radius ray) for the two arcs.
        leader_pen = QPen(QColor(0, 160, 130, 235))
        leader_pen.setWidthF(1.2)
        leader_pen.setCosmetic(True)
        leader_brush = QBrush(QColor(0, 160, 130, 235))

        def _radius_annot(text: str, cx: float, cy: float, px: float, py: float, lx: float, ly: float):
            dot = QGraphicsEllipseItem(cx - 1.7, cy - 1.7, 3.4, 3.4)
            dot.setPen(leader_pen)
            dot.setBrush(leader_brush)
            self.slot_scene.addItem(dot)

            ray = QGraphicsLineItem(cx, cy, px, py)
            ray.setPen(leader_pen)
            self.slot_scene.addItem(ray)

            ln = QGraphicsLineItem(px, py, px + lx, py + ly)
            ln.setPen(leader_pen)
            self.slot_scene.addItem(ln)

            t = QGraphicsSimpleTextItem(text)
            t.setBrush(leader_brush)
            t.setScale(1.2)
            t.setFont(QFont("Arial", 11, QFont.Weight.Bold))
            t.setPos(px + lx + 3.0, py + ly - 10.0)
            self.slot_scene.addItem(t)

        # Match the exact arc centers used in _teardrop_local_path_from_params().
        drb_centers = max(0.0, drb - r1 - r2)
        cy1 = h0 + r1
        cy2 = h0 + r1 + drb_centers
        denom = max(1e-9, drb_centers)
        theta = math.asin(max(-1.0, min(1.0, (r2 - r1) / denom))) if drb_centers > 1e-9 else 0.0
        t1x = r1 * math.cos(theta)
        t1y = -r1 * math.sin(theta)
        t2x = r2 * math.cos(theta)
        t2y = -r2 * math.sin(theta)
        p1R_x, p1R_y = (t1x, cy1 + t1y)
        p2R_x, p2R_y = (t2x, cy2 + t2y)

        _radius_annot("r1", 0.0, cy1, p1R_x, p1R_y, max(26.0, half_span * 0.40), 0.0)
        _radius_annot("r2", 0.0, cy2, p2R_x, p2R_y, max(34.0, half_span * 0.55), 0.0)

        # Centerline.
        center_pen = QPen(QColor(60, 70, 90, 150))
        center_pen.setWidthF(1.0)
        center_pen.setCosmetic(True)
        center_pen.setStyle(Qt.PenStyle.DashLine)
        cl = QGraphicsLineItem(0.0, b0_y - 18.0, 0.0, total_depth + 32.0)
        cl.setPen(center_pen)
        self.slot_scene.addItem(cl)

        # Scene rect.
        bounds = slot_path.boundingRect()
        margin_x = max(220.0, bounds.width() * 1.2)
        margin_y = max(160.0, bounds.height() * 0.35)
        self.slot_scene.setSceneRect(bounds.x() - margin_x, bounds.y() - margin_y,
                                     bounds.width() + 2 * margin_x, bounds.height() + 2 * margin_y)

        # Keep slot view auto-framed when it is redrawn.
        if getattr(self, "slot_view", None) is not None:
            self.slot_view.reset_view()

    def _teardrop_local_path(self) -> QPainterPath:
        p = self._params
        if p is None:
            return QPainterPath()
        return Rotor2DWidget._teardrop_local_path_from_params(p)

    @staticmethod
    def _teardrop_path_at_theta_from_params(p: Rotor2DParams, theta: float) -> QPainterPath:
        R = float(p.Dr_mm) / 2.0
        c = math.cos(theta)
        s = math.sin(theta)
        tr = QTransform(
            -c, -s, 0.0,
            -s,  c, 0.0,
             c * R, s * R, 1.0
        )
        # NOTE: Some PyQt6 builds don't expose QPainterPath.transformed(); use QTransform.map instead.
        return tr.map(Rotor2DWidget._teardrop_local_path_from_params(p))

    def _teardrop_path_at_theta(self, theta: float) -> QPainterPath:
        p = self._params
        if p is None:
            return QPainterPath()
        return Rotor2DWidget._teardrop_path_at_theta_from_params(p, theta)

    def _redraw_cross_section(self):
        self.cross_scene.clear()
        p = self._params
        if p is None:
            self.cross_scene.setSceneRect(-10, -10, 20, 20)
            return

        R = float(p.Dr_mm) / 2.0
        R_sh = max(0.0, float(p.shaft_d_mm) / 2.0)

        rotor_pen = QPen(QColor(60, 70, 90, 220))
        rotor_pen.setWidthF(1.2)
        rotor_pen.setCosmetic(True)
        rotor_brush = QBrush(QColor(220, 228, 245, 140))

        shaft_pen = QPen(QColor(60, 70, 90, 200))
        shaft_pen.setWidthF(1.0)
        shaft_pen.setCosmetic(True)
        shaft_brush = QBrush(QColor(255, 255, 255, 255))

        outer = QGraphicsEllipseItem(-R, -R, 2 * R, 2 * R)
        outer.setPen(rotor_pen)
        outer.setBrush(rotor_brush)
        self.cross_scene.addItem(outer)

        shaft = QGraphicsEllipseItem(-R_sh, -R_sh, 2 * R_sh, 2 * R_sh)
        shaft.setPen(shaft_pen)
        shaft.setBrush(shaft_brush)
        self.cross_scene.addItem(shaft)

        Nr = max(1, int(p.Nr))
        dtheta = (2.0 * math.pi) / float(Nr)
        for i in range(Nr):
            theta = float(i) * dtheta
            slot_path = self._teardrop_path_at_theta(theta)
            is_sel = (i == int(self._selected_slot_index))
            item = _ClickablePathItem(i, self._on_slot_clicked, slot_path)
            if is_sel:
                sel_pen = QPen(QColor(0, 160, 130, 240))
                sel_pen.setWidthF(1.4)
                sel_pen.setCosmetic(True)
                item.setPen(sel_pen)
                item.setBrush(QBrush(QColor(0, 160, 130, 90)))
            else:
                slot_pen = QPen(QColor(60, 70, 90, 180))
                slot_pen.setWidthF(1.0)
                slot_pen.setCosmetic(True)
                item.setPen(slot_pen)
                item.setBrush(QBrush(QColor(255, 255, 255, 140)))
            self.cross_scene.addItem(item)

        pad = R * 0.18
        self.cross_scene.setSceneRect(-(R + pad), -(R + pad), 2 * (R + pad), 2 * (R + pad))

    def _add_dim_x(self, scene: QGraphicsScene, x1: float, x2: float, y: float, label: str, highlight: bool = False):
        pen = QPen(QColor(0, 160, 130, 235))
        pen.setWidthF(1.0 if not highlight else 1.6)
        pen.setCosmetic(True)
        line = QGraphicsLineItem(x1, y, x2, y)
        line.setPen(pen)
        scene.addItem(line)
        ah = 1.2
        for ax, ay in ((x1, y), (x2, y)):
            sgn = +1.0 if ax == x1 else -1.0
            a1 = QGraphicsLineItem(ax, ay, ax + sgn * ah, ay - ah)
            a1.setPen(pen)
            scene.addItem(a1)
            a2 = QGraphicsLineItem(ax, ay, ax + sgn * ah, ay + ah)
            a2.setPen(pen)
            scene.addItem(a2)
        t = QGraphicsSimpleTextItem(label)
        t.setBrush(QBrush(QColor(0, 160, 130, 235)))
        text_scale = 0.9 if not highlight else 1.4
        t.setScale(text_scale)
        if highlight:
            t.setFont(QFont("Arial", 12, QFont.Weight.Bold))
        br = t.boundingRect()
        tw = br.width() * text_scale
        t.setPos((x1 + x2) / 2.0 - tw / 2.0, y + 2.0)
        scene.addItem(t)

    def _add_dim_y_abs(self, scene: QGraphicsScene, length: float, x: float, y_center: float, label: str, highlight: bool = False):
        y1 = y_center - length / 2.0
        y2 = y_center + length / 2.0
        pen = QPen(QColor(0, 160, 130, 235))
        pen.setWidthF(1.0 if not highlight else 1.6)
        pen.setCosmetic(True)
        line = QGraphicsLineItem(x, y1, x, y2)
        line.setPen(pen)
        scene.addItem(line)
        ah = 1.2
        for yy, sgn in ((y1, +1.0), (y2, -1.0)):
            a1 = QGraphicsLineItem(x, yy, x - ah, yy + sgn * ah)
            a1.setPen(pen)
            scene.addItem(a1)
            a2 = QGraphicsLineItem(x, yy, x + ah, yy + sgn * ah)
            a2.setPen(pen)
            scene.addItem(a2)
        t = QGraphicsSimpleTextItem(label)
        t.setBrush(QBrush(QColor(0, 160, 130, 235)))
        text_scale = 0.9 if not highlight else 1.4
        t.setScale(text_scale)
        if highlight:
            t.setFont(QFont("Arial", 12, QFont.Weight.Bold))
        br = t.boundingRect()
        tw = br.width() * text_scale
        th = br.height() * text_scale
        pad = 2.5
        tx = (x - tw - pad) if x < 0 else (x + pad)
        t.setPos(tx, y_center - th / 2.0)
        scene.addItem(t)

    def _redraw_slot_zoom(self):
        self.slot_scene.clear()
        p = self._params
        if p is None:
            self.slot_scene.setSceneRect(-10, -10, 20, 20)
            return

        # Fixed vertical schematic (scaled for readability) + dimension callouts.
        scale = 18.0
        h0_mm = 0.500
        b0_mm = 0.800
        r1_mm = 1.269
        r2_mm = 0.508
        drb_mm = 25.877

        p_schem = Rotor2DParams(
            Dr_mm=100.0,
            Nr=1,
            shaft_d_mm=0.0,
            h0_mm=h0_mm * scale,
            b0_mm=b0_mm * scale,
            r1_mm=r1_mm * scale,
            r2_mm=r2_mm * scale,
            drb_mm=drb_mm * scale,
        )

        base_path = Rotor2DWidget._teardrop_local_path_from_params(p_schem)
        swap = QTransform(0.0, 1.0, 0.0,
                          1.0, 0.0, 0.0,
                          0.0, 0.0, 1.0)
        slot_path = swap.map(base_path)

        slot_item = QGraphicsPathItem(slot_path)
        pen = QPen(QColor(60, 110, 210, 235))
        pen.setWidthF(1.2)
        pen.setCosmetic(True)
        slot_item.setPen(pen)
        slot_item.setBrush(QBrush(QColor(120, 160, 255, 95)))
        self.slot_scene.addItem(slot_item)

        h0 = h0_mm * scale
        b0 = b0_mm * scale
        r1 = r1_mm * scale
        r2 = r2_mm * scale
        drb = drb_mm * scale
        total_depth = h0 + drb
        half_span = max(b0 / 2.0, r1, r2)

        # Dimensions (no numeric values; this is a schematic legend)
        b0_y = -max(30.0, 0.10 * total_depth)
        self._add_dim_x(self.slot_scene, -b0 / 2.0, +b0 / 2.0, b0_y, "b0", highlight=True)

        x_left = -max(95.0, b0 * 1.9)
        x_right = +max(95.0, b0 * 1.9)
        self._add_dim_y_abs(self.slot_scene, h0, x_left, h0 / 2.0, "h0", highlight=True)
        self._add_dim_y_abs(self.slot_scene, drb, x_right, h0 + drb / 2.0, "drb", highlight=True)

        # Radii: show the actual radius construction (center + radius ray) for the two arcs.
        leader_pen = QPen(QColor(0, 160, 130, 235))
        leader_pen.setWidthF(1.2)
        leader_pen.setCosmetic(True)
        leader_brush = QBrush(QColor(0, 160, 130, 235))

        def _radius_annot(text: str, cx: float, cy: float, px: float, py: float, lx: float, ly: float):
            dot = QGraphicsEllipseItem(cx - 1.7, cy - 1.7, 3.4, 3.4)
            dot.setPen(leader_pen)
            dot.setBrush(leader_brush)
            self.slot_scene.addItem(dot)

            ray = QGraphicsLineItem(cx, cy, px, py)
            ray.setPen(leader_pen)
            self.slot_scene.addItem(ray)

            ln = QGraphicsLineItem(px, py, px + lx, py + ly)
            ln.setPen(leader_pen)
            self.slot_scene.addItem(ln)

            t = QGraphicsSimpleTextItem(text)
            t.setBrush(leader_brush)
            t.setScale(1.2)
            t.setFont(QFont("Arial", 11, QFont.Weight.Bold))
            t.setPos(px + lx + 3.0, py + ly - 10.0)
            self.slot_scene.addItem(t)

        # Match the exact arc centers used in _teardrop_local_path_from_params().
        drb_centers = max(0.0, drb - r1 - r2)
        cy1 = h0 + r1
        cy2 = h0 + r1 + drb_centers
        denom = max(1e-9, drb_centers)
        theta = math.asin(max(-1.0, min(1.0, (r2 - r1) / denom))) if drb_centers > 1e-9 else 0.0
        t1x = r1 * math.cos(theta)
        t1y = -r1 * math.sin(theta)
        t2x = r2 * math.cos(theta)
        t2y = -r2 * math.sin(theta)
        p1R_x, p1R_y = (t1x, cy1 + t1y)
        p2R_x, p2R_y = (t2x, cy2 + t2y)

        _radius_annot("r1", 0.0, cy1, p1R_x, p1R_y, max(26.0, half_span * 0.40), 0.0)
        _radius_annot("r2", 0.0, cy2, p2R_x, p2R_y, max(34.0, half_span * 0.55), 0.0)

        # Centerline.
        center_pen = QPen(QColor(60, 70, 90, 150))
        center_pen.setWidthF(1.0)
        center_pen.setCosmetic(True)
        center_pen.setStyle(Qt.PenStyle.DashLine)
        cl = QGraphicsLineItem(0.0, b0_y - 18.0, 0.0, total_depth + 32.0)
        cl.setPen(center_pen)
        self.slot_scene.addItem(cl)

        bounds = slot_path.boundingRect()
        margin_x = max(220.0, bounds.width() * 1.2)
        margin_y = max(160.0, bounds.height() * 0.35)
        self.slot_scene.setSceneRect(bounds.x() - margin_x, bounds.y() - margin_y,
                                     bounds.width() + 2 * margin_x, bounds.height() + 2 * margin_y)

        # Keep slot view auto-framed when it is redrawn.
        if getattr(self, "slot_view", None) is not None:
            self.slot_view.reset_view()


class Assembly2DWidget(QWidget):
    """Combined stator + rotor cross-section."""

    def __init__(self, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self._stator: Optional[Stator2DParams] = None
        self._rotor: Optional[Rotor2DParams] = None

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)

        header = QHBoxLayout()
        header.addWidget(QLabel("Assembly (stator + rotor)"), 0)
        header.addStretch(1)
        self.fit_button = QPushButton("Fit")
        self.fit_button.clicked.connect(self._fit_view)
        header.addWidget(self.fit_button, 0)
        layout.addLayout(header)

        self.scene = QGraphicsScene(self)
        self.view = ZoomPanGraphicsView()
        self.view.setScene(self.scene)
        self.view.setProperty("role", "canvas")
        self.view.setBackgroundBrush(QBrush(Qt.GlobalColor.white))
        self.scene.setBackgroundBrush(QBrush(Qt.GlobalColor.white))
        self.view.setMinimumHeight(420)
        layout.addWidget(self.view, 1)

        self._fit_view()

    def set_params(self, stator: Optional[Stator2DParams], rotor: Optional[Rotor2DParams]):
        self._stator = stator
        self._rotor = rotor
        self._redraw()
        self._fit_view()

    def clear_params(self):
        self._stator = None
        self._rotor = None
        self.scene.clear()
        self.scene.setSceneRect(-10, -10, 20, 20)

    def _fit_view(self):
        if self.view is not None:
            self.view.reset_view()

    def _redraw(self):
        self.scene.clear()
        s = self._stator
        r = self._rotor
        if s is None or r is None:
            self.scene.setSceneRect(-10, -10, 20, 20)
            return

        Rout_s = float(s.Dext_mm) / 2.0
        Rin_s = float(s.D_mm) / 2.0
        Rr = float(r.Dr_mm) / 2.0
        Rsh = max(0.0, float(r.shaft_d_mm) / 2.0)

        pen_main = QPen(QColor(60, 70, 90, 220))
        pen_main.setWidthF(1.2)
        pen_main.setCosmetic(True)

        # Stator outer and bore
        outer_s = QGraphicsEllipseItem(-Rout_s, -Rout_s, 2 * Rout_s, 2 * Rout_s)
        outer_s.setPen(pen_main)
        outer_s.setBrush(QBrush(QColor(220, 228, 245, 70)))
        self.scene.addItem(outer_s)

        bore_s = QGraphicsEllipseItem(-Rin_s, -Rin_s, 2 * Rin_s, 2 * Rin_s)
        bore_s.setPen(pen_main)
        bore_s.setBrush(QBrush(QColor(255, 255, 255, 255)))
        self.scene.addItem(bore_s)

        # Stator slots (drawn faintly for context, like the HTML assembly)
        Ss = max(1, int(s.Ss))
        dtheta_s = (2.0 * math.pi) / float(Ss)
        stator_slot_pen = QPen(QColor(60, 70, 90, 140))
        stator_slot_pen.setWidthF(0.9)
        stator_slot_pen.setCosmetic(True)
        stator_slot_brush = QBrush(QColor(255, 255, 255, 40))
        fillet = _stator2d_slot_fillet_radius(s)
        for i in range(Ss):
            th = float(i) * dtheta_s
            if s.slot_type == 'square':
                pts = _stator2d_slot_polygon_square(s, th)
                path = _stator2d_rounded_polygon_path(pts, 0.0)
            else:
                pts = _stator2d_slot_polygon_trapez(s, th)
                path = _stator2d_rounded_polygon_path(pts, fillet)
            it = QGraphicsPathItem(path)
            it.setPen(stator_slot_pen)
            it.setBrush(stator_slot_brush)
            self.scene.addItem(it)

        # Rotor outer and shaft
        rotor_pen = QPen(QColor(60, 110, 210, 235))
        rotor_pen.setWidthF(1.2)
        rotor_pen.setCosmetic(True)
        rotor_item = QGraphicsEllipseItem(-Rr, -Rr, 2 * Rr, 2 * Rr)
        rotor_item.setPen(rotor_pen)
        rotor_item.setBrush(QBrush(QColor(120, 160, 255, 55)))
        self.scene.addItem(rotor_item)

        shaft_item = QGraphicsEllipseItem(-Rsh, -Rsh, 2 * Rsh, 2 * Rsh)
        shaft_item.setPen(pen_main)
        shaft_item.setBrush(QBrush(QColor(255, 255, 255, 255)))
        self.scene.addItem(shaft_item)

        # Rotor slots (faint outlines only, for context)
        Nr = max(1, int(r.Nr))
        dtheta = (2.0 * math.pi) / float(Nr)
        slot_pen = QPen(QColor(60, 70, 90, 110))
        slot_pen.setWidthF(0.9)
        slot_pen.setCosmetic(True)
        for i in range(Nr):
            slot_path = Rotor2DWidget._teardrop_path_at_theta_from_params(r, float(i) * dtheta)
            item = QGraphicsPathItem(slot_path)
            item.setPen(slot_pen)
            item.setBrush(QBrush(QColor(255, 255, 255, 0)))
            self.scene.addItem(item)

        pad = max(Rout_s, Rin_s) * 0.20
        self.scene.setSceneRect(-(Rout_s + pad), -(Rout_s + pad), 2 * (Rout_s + pad), 2 * (Rout_s + pad))


class ZoomPanGraphicsView(QGraphicsView):
    def __init__(self, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing, True)
        self.setViewportUpdateMode(QGraphicsView.ViewportUpdateMode.FullViewportUpdate)
        self.setTransformationAnchor(QGraphicsView.ViewportAnchor.AnchorUnderMouse)
        self.setResizeAnchor(QGraphicsView.ViewportAnchor.AnchorUnderMouse)
        self.setDragMode(QGraphicsView.DragMode.NoDrag)
        self._panning = False
        self._pan_start = None

    def wheelEvent(self, event):
        angle = event.angleDelta().y()
        if angle == 0:
            return
        factor = 1.15 if angle > 0 else 1 / 1.15
        self.scale(factor, factor)

    def mousePressEvent(self, event):
        if event.button() == Qt.MouseButton.LeftButton:
            # If an item handles the click (e.g., slot selection), let it.
            item = self.itemAt(event.pos())
            if item is None:
                self._panning = True
                self._pan_start = event.pos()
                self.setCursor(Qt.CursorShape.ClosedHandCursor)
                event.accept()
                return
        super().mousePressEvent(event)

    def mouseMoveEvent(self, event):
        if self._panning and self._pan_start is not None:
            delta = event.pos() - self._pan_start
            self._pan_start = event.pos()
            self.translate(delta.x() * 0.6, delta.y() * 0.6)
            event.accept()
            return
        super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event):
        if event.button() == Qt.MouseButton.LeftButton and self._panning:
            self._panning = False
            self._pan_start = None
            self.setCursor(Qt.CursorShape.ArrowCursor)
            event.accept()
            return
        super().mouseReleaseEvent(event)

    def reset_view(self):
        self.setTransform(QTransform())
        if self.scene() is not None:
            self.fitInView(self.scene().sceneRect(), Qt.AspectRatioMode.KeepAspectRatio)


class StaticGraphicsView(QGraphicsView):
    """Non-interactive view (no pan/zoom) for schematic diagrams."""

    def __init__(self, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing, True)
        self.setViewportUpdateMode(QGraphicsView.ViewportUpdateMode.FullViewportUpdate)
        self.setTransformationAnchor(QGraphicsView.ViewportAnchor.AnchorViewCenter)
        self.setResizeAnchor(QGraphicsView.ViewportAnchor.AnchorViewCenter)
        self.setDragMode(QGraphicsView.DragMode.NoDrag)
        self.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.setInteractive(False)

    def wheelEvent(self, event):
        event.ignore()

    def reset_view(self):
        self.setTransform(QTransform())
        if self.scene() is not None:
            self.fitInView(self.scene().sceneRect(), Qt.AspectRatioMode.KeepAspectRatio)


class _ClickablePathItem(QGraphicsPathItem):
    def __init__(self, index: int, on_click, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._index = index
        self._on_click = on_click
        self.setAcceptHoverEvents(True)

    def mousePressEvent(self, event):
        if event.button() == Qt.MouseButton.LeftButton:
            try:
                self._on_click(self._index)
            finally:
                event.accept()
            return
        super().mousePressEvent(event)


class Stator2DWidget(QWidget):
    """Stator 2D cross-section + slot zoom (ported from motor_app_fixed.html)."""

    def __init__(self, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self._params: Optional[Stator2DParams] = None
        self._selected_slot_index = 0

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)

        # Controls + collected data
        controls = QGroupBox("Stator 2D view")
        controls_layout = QGridLayout(controls)

        self.fit_button = QPushButton("Fit views")
        self.fit_button.clicked.connect(self._fit_views)
        controls_layout.addWidget(self.fit_button, 0, 0)

        self.collected_data_button = QPushButton("Collected data")
        self.collected_data_button.clicked.connect(self._show_collected_data_dialog)
        controls_layout.addWidget(self.collected_data_button, 0, 1)

        layout.addWidget(controls, 0)

        # Views
        views_row = QWidget()
        views_layout = QHBoxLayout(views_row)
        views_layout.setContentsMargins(0, 0, 0, 0)

        left_col = QVBoxLayout()
        right_col = QVBoxLayout()

        self.cross_scene = QGraphicsScene(self)
        self.cross_view = ZoomPanGraphicsView()
        self.cross_view.setScene(self.cross_scene)
        self.cross_view.setProperty("role", "canvas")
        self.cross_view.setBackgroundBrush(QBrush(Qt.GlobalColor.white))
        self.cross_scene.setBackgroundBrush(QBrush(Qt.GlobalColor.white))
        self.cross_view.setMinimumHeight(360)
        left_col.addWidget(QLabel("Stator cross-section"), 0)
        left_col.addWidget(self.cross_view, 1)

        self.slot_scene = QGraphicsScene(self)
        self.slot_view = StaticGraphicsView()
        self.slot_view.setScene(self.slot_scene)
        self.slot_view.setProperty("role", "canvas")
        self.slot_view.setBackgroundBrush(QBrush(Qt.GlobalColor.white))
        self.slot_scene.setBackgroundBrush(QBrush(Qt.GlobalColor.white))
        self.slot_view.setMinimumHeight(360)
        right_col.addWidget(QLabel("Stator slot geometry (schematic)"), 0)
        right_col.addWidget(self.slot_view, 1)

        views_layout.addLayout(left_col, 1)
        views_layout.addLayout(right_col, 1)
        layout.addWidget(views_row, 1)

        self._fit_views()

    def set_params(self, params: Stator2DParams):
        self._params = params
        self._selected_slot_index = 0
        self._redraw_all()

    def clear_params(self):
        self._params = None
        self._selected_slot_index = 0
        self.cross_scene.clear()
        self.slot_scene.clear()
        self.cross_scene.setSceneRect(-10, -10, 20, 20)
        self.slot_scene.setSceneRect(-10, -10, 20, 20)

    def has_params(self) -> bool:
        return self._params is not None

    def _collected_data_rows(self) -> list[tuple[str, str]]:
        p = self._params
        if p is None:
            return []
        rows: list[tuple[str, str]] = [
            ("D", f"{p.D_mm:.3f} mm"),
            ("D_ext", f"{p.Dext_mm:.3f} mm"),
            ("Ss", f"{p.Ss}"),
            ("Slot type", f"{p.slot_type}"),
        ]
        if p.slot_type == 'square':
            rows.append(("bs", f"{float(p.bs_mm or 0.0):.3f} mm"))
            rows.append(("hs", f"{float(p.hs_mm or 0.0):.3f} mm"))
        else:
            rows.append(("bs0", f"{float(p.bs0_mm or 0.0):.3f} mm"))
            rows.append(("hs0", f"{float(p.hs0_mm or 0.0):.3f} mm"))
            rows.append(("bs1", f"{float(p.bs1_mm or 0.0):.3f} mm"))
            rows.append(("hs1", f"{float(p.hs1_mm or 0.0):.3f} mm"))
            rows.append(("bs2", f"{float(p.bs2_mm or 0.0):.3f} mm"))
            rows.append(("hs2", f"{float(p.hs2_mm or 0.0):.3f} mm"))
        return rows

    def _show_collected_data_dialog(self):
        rows = self._collected_data_rows()
        if not rows:
            QMessageBox.information(self, "Collected data", "No stator data available.")
            return

        dlg = QDialog(self)
        dlg.setWindowTitle("Collected data")
        dlg.setModal(True)
        dlg.setMinimumWidth(520)
        layout = QVBoxLayout(dlg)

        table = QTableWidget(len(rows), 2, dlg)
        table.setHorizontalHeaderLabels(["Parameter", "Value"])
        table.verticalHeader().setVisible(False)
        # Prevent header text from being clipped (especially on scroll/selection with the dark theme).
        try:
            hh = table.horizontalHeader()
            hh.setDefaultAlignment(Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter)
            hh.setMinimumHeight(30)
            hh.setFixedHeight(30)
        except Exception:
            pass
        table.setStyleSheet(
            "QHeaderView::section { padding: 6px 10px; }"
        )
        table.setEditTriggers(QTableWidget.EditTrigger.NoEditTriggers)
        table.setSelectionBehavior(QTableWidget.SelectionBehavior.SelectRows)
        table.setSelectionMode(QTableWidget.SelectionMode.SingleSelection)
        table.horizontalHeader().setStretchLastSection(True)
        table.horizontalHeader().setSectionResizeMode(0, QHeaderView.ResizeMode.ResizeToContents)
        table.horizontalHeader().setSectionResizeMode(1, QHeaderView.ResizeMode.Stretch)
        for r, (k, v) in enumerate(rows):
            table.setItem(r, 0, QTableWidgetItem(str(k)))
            table.setItem(r, 1, QTableWidgetItem(str(v)))
        layout.addWidget(table, 1)

        close_btn = QPushButton("Close")
        close_btn.clicked.connect(dlg.accept)
        layout.addWidget(close_btn, 0)

        dlg.exec()

    def _fit_views(self):
        # Slot schematic view is meant to be stationary; don't let the cross-section fit button
        # affect it.
        self.cross_view.reset_view()

    def _on_slot_clicked(self, index: int):
        p = self._params
        if p is None:
            return
        self._selected_slot_index = max(0, min(int(index), int(p.Ss) - 1))
        self._redraw_cross_section()
        self._redraw_slot_zoom()

    def _redraw_all(self):
        self._redraw_cross_section()
        self._redraw_slot_zoom()
        self._fit_views()

    def _stator_slot_fillet_radius(self) -> float:
        p = self._params
        if p is None:
            return 0.0
        return _stator2d_slot_fillet_radius(p)

    def _rounded_polygon_path(self, points: list[tuple[float, float]], radius: float) -> QPainterPath:
        return _stator2d_rounded_polygon_path(points, radius)

    def _slot_polygon_square(self, theta: float) -> list[tuple[float, float]]:
        p = self._params
        if p is None:
            return []
        return _stator2d_slot_polygon_square(p, theta)

    def _slot_width_at_radius_trapez(self, r: float, r1: float, r2: float, r3: float) -> float:
        p = self._params
        if p is None:
            return 0.0
        return _stator2d_slot_width_at_radius_trapez(p, r, r1, r2, r3)

    def _slot_polygon_trapez(self, theta: float) -> list[tuple[float, float]]:
        p = self._params
        if p is None:
            return []
        return _stator2d_slot_polygon_trapez(p, theta)

    def _redraw_cross_section(self):
        self.cross_scene.clear()
        p = self._params
        if p is None:
            self.cross_scene.setSceneRect(-10, -10, 20, 20)
            return
        Rout = float(p.Dext_mm) / 2.0
        Rin = float(p.D_mm) / 2.0

        # Higher-contrast styling (the initial alpha values were too faint on white).
        steel_pen = QPen(QColor(60, 70, 90, 220))
        steel_pen.setWidthF(1.2)
        steel_pen.setCosmetic(True)
        steel_brush = QBrush(QColor(220, 228, 245, 140))

        bore_pen = QPen(QColor(60, 70, 90, 200))
        bore_pen.setWidthF(1.0)
        bore_pen.setCosmetic(True)
        bore_brush = QBrush(QColor(255, 255, 255, 255))

        outer = QGraphicsEllipseItem(-Rout, -Rout, 2 * Rout, 2 * Rout)
        outer.setPen(steel_pen)
        outer.setBrush(steel_brush)
        self.cross_scene.addItem(outer)

        inner = QGraphicsEllipseItem(-Rin, -Rin, 2 * Rin, 2 * Rin)
        inner.setPen(bore_pen)
        inner.setBrush(bore_brush)
        self.cross_scene.addItem(inner)

        Q = max(1, int(p.Ss))
        dtheta = (2.0 * math.pi) / float(Q)
        for i in range(Q):
            theta = float(i) * dtheta
            if p.slot_type == 'square':
                pts = self._slot_polygon_square(theta)
                slot_path = self._rounded_polygon_path(pts, 0.0)
            else:
                pts = self._slot_polygon_trapez(theta)
                slot_path = self._rounded_polygon_path(pts, self._stator_slot_fillet_radius())

            is_sel = (i == int(self._selected_slot_index))
            item = _ClickablePathItem(i, self._on_slot_clicked, slot_path)
            if is_sel:
                sel_pen = QPen(QColor(0, 160, 130, 240))
                sel_pen.setWidthF(1.4)
                sel_pen.setCosmetic(True)
                item.setPen(sel_pen)
                item.setBrush(QBrush(QColor(0, 160, 130, 90)))
            else:
                slot_pen = QPen(QColor(60, 70, 90, 180))
                slot_pen.setWidthF(1.0)
                slot_pen.setCosmetic(True)
                item.setPen(slot_pen)
                item.setBrush(QBrush(QColor(255, 255, 255, 140)))
            self.cross_scene.addItem(item)

        pad = Rout * 0.18
        self.cross_scene.setSceneRect(-(Rout + pad), -(Rout + pad), 2 * (Rout + pad), 2 * (Rout + pad))

    def _add_dim_x(self, scene: QGraphicsScene, x1: float, x2: float, y: float, label: str, highlight: bool = False):
        pen = QPen(QColor(0, 160, 130, 235))
        pen.setWidthF(1.0 if not highlight else 1.6)
        pen.setCosmetic(True)
        line = QGraphicsLineItem(x1, y, x2, y)
        line.setPen(pen)
        scene.addItem(line)
        # arrowheads
        ah = 1.2
        a1 = QGraphicsLineItem(x1, y, x1 + ah, y - ah)
        a1.setPen(pen)
        scene.addItem(a1)
        a2 = QGraphicsLineItem(x1, y, x1 + ah, y + ah)
        a2.setPen(pen)
        scene.addItem(a2)
        a3 = QGraphicsLineItem(x2, y, x2 - ah, y - ah)
        a3.setPen(pen)
        scene.addItem(a3)
        a4 = QGraphicsLineItem(x2, y, x2 - ah, y + ah)
        a4.setPen(pen)
        scene.addItem(a4)
        t = QGraphicsSimpleTextItem(label)
        t.setBrush(QBrush(QColor(0, 160, 130, 235)))
        text_scale = 0.9 if not highlight else 1.4
        t.setScale(text_scale)
        if highlight:
            t.setFont(QFont("Arial", 12, QFont.Weight.Bold))
        br = t.boundingRect()
        tw = br.width() * text_scale
        # Center label on the dimension line with padding below.
        t.setPos((x1 + x2) / 2.0 - tw / 2.0, y + 2.0)
        scene.addItem(t)

    def _add_dim_y_abs(self, scene: QGraphicsScene, length: float, x: float, y_center: float, label: str, highlight: bool = False):
        y1 = y_center - length / 2.0
        y2 = y_center + length / 2.0
        pen = QPen(QColor(0, 160, 130, 235))
        pen.setWidthF(1.0 if not highlight else 1.6)
        pen.setCosmetic(True)
        line = QGraphicsLineItem(x, y1, x, y2)
        line.setPen(pen)
        scene.addItem(line)
        ah = 1.2
        a1 = QGraphicsLineItem(x, y1, x - ah, y1 + ah)
        a1.setPen(pen)
        scene.addItem(a1)
        a2 = QGraphicsLineItem(x, y1, x + ah, y1 + ah)
        a2.setPen(pen)
        scene.addItem(a2)
        a3 = QGraphicsLineItem(x, y2, x - ah, y2 - ah)
        a3.setPen(pen)
        scene.addItem(a3)
        a4 = QGraphicsLineItem(x, y2, x + ah, y2 - ah)
        a4.setPen(pen)
        scene.addItem(a4)
        t = QGraphicsSimpleTextItem(label)
        t.setBrush(QBrush(QColor(0, 160, 130, 235)))
        text_scale = 0.9 if not highlight else 1.4
        t.setScale(text_scale)
        if highlight:
            t.setFont(QFont("Arial", 12, QFont.Weight.Bold))
        br = t.boundingRect()
        tw = br.width() * text_scale
        th = br.height() * text_scale
        # Place labels outside the dimension column to avoid collisions.
        pad = 2.5
        if x < 0:
            tx = x - tw - pad
        else:
            tx = x + pad
        t.setPos(tx, y_center - th / 2.0)
        scene.addItem(t)

    def _redraw_slot_zoom(self):
        self.slot_scene.clear()
        p = self._params
        if p is None:
            self.slot_scene.setSceneRect(-10, -10, 20, 20)
            return

        # Fixed schematic (no focus dropdown, no show-all toggle). Start with trapezoidal slots.
        if p.slot_type != 'trapez':
            # Square/open slot schematic based on supplied dimensions, scaled for clarity.
            bs_mm, hs_mm = 8.177, 40.883
            # Choose scale so the slot height fits well inside the view.
            target_h_px = 300.0
            scale = max(4.0, min(12.0, target_h_px / max(1e-9, hs_mm)))
            W = bs_mm * scale
            H = hs_mm * scale

            # No smoothing for open slot: keep it strictly rectangular.
            path = QPainterPath()
            path.addRect(-W / 2.0, 0.0, W, H)
            item = QGraphicsPathItem(path)
            pen = QPen(QColor(60, 110, 210, 235))
            pen.setWidthF(1.2)
            pen.setCosmetic(True)
            item.setPen(pen)
            item.setBrush(QBrush(QColor(120, 160, 255, 85)))
            self.slot_scene.addItem(item)

            # Place dimensions close to the slot so the scene stays centered.
            top_dim_y = -max(32.0, 0.09 * H)
            dim_col_x = -W / 2.0 - max(60.0, 0.65 * W)
            self._add_dim_x(self.slot_scene, -W / 2.0, +W / 2.0, top_dim_y, "bs", highlight=True)
            self._add_dim_y_abs(self.slot_scene, H, dim_col_x, H / 2.0, "hs", highlight=True)

            bounds = path.boundingRect()
            margin_x = max(140.0, 1.4 * W)
            margin_y = max(100.0, 0.25 * H)
            self.slot_scene.setSceneRect(bounds.x() - margin_x, bounds.y() - margin_y,
                                         bounds.width() + 2 * margin_x, bounds.height() + 2 * margin_y)

            # Keep slot view auto-framed when it is redrawn.
            if getattr(self, "slot_view", None) is not None:
                self.slot_view.reset_view()
            return

        # Representative trapezoidal slot based on supplied dimensions, scaled for clarity.
        scale = 12.0
        bs0_mm, hs0_mm = 4.878, 1.600
        bs1_mm, hs1_mm = 9.089, 3.000
        bs2_mm, hs2_mm = 13.550, 38.300

        bs0 = bs0_mm * scale
        bs1 = bs1_mm * scale
        bs2 = bs2_mm * scale
        hs0 = hs0_mm * scale
        hs1 = hs1_mm * scale
        hs2 = hs2_mm * scale

        y0 = 0.0
        y1 = y0 + hs0
        y2 = y1 + hs1
        y3 = y2 + hs2

        half_span = max(bs0, bs1, bs2) / 2.0
        r_top = min(hs0 * 0.45, bs0 * 0.18)
        r_bottom = min(hs2 * 0.12, bs2 * 0.18)

        outline = QPainterPath()
        outline.moveTo(-bs0 / 2.0 + r_top, y0)
        outline.lineTo(bs0 / 2.0 - r_top, y0)
        outline.quadTo(QPointF(bs0 / 2.0, y0), QPointF(bs0 / 2.0, y0 + r_top))
        outline.lineTo(bs0 / 2.0, y1)

        ctrl1 = QPointF(bs0 / 2.0 + (bs1 - bs0) * 0.25, (y1 + y2) / 2.0)
        outline.quadTo(ctrl1, QPointF(bs1 / 2.0, y2))

        ctrl2 = QPointF(bs1 / 2.0 + (bs2 - bs1) * 0.22, y2 + hs2 * 0.45)
        outline.quadTo(ctrl2, QPointF(bs2 / 2.0, y3 - r_bottom))
        outline.quadTo(QPointF(bs2 / 2.0, y3), QPointF(bs2 / 2.0 - r_bottom, y3))
        outline.lineTo(-bs2 / 2.0 + r_bottom, y3)
        outline.quadTo(QPointF(-bs2 / 2.0, y3), QPointF(-bs2 / 2.0, y3 - r_bottom))

        ctrl3 = QPointF(-(bs1 / 2.0 + (bs2 - bs1) * 0.22), y2 + hs2 * 0.45)
        outline.quadTo(ctrl3, QPointF(-bs1 / 2.0, y2))
        ctrl4 = QPointF(-(bs0 / 2.0 + (bs1 - bs0) * 0.25), (y1 + y2) / 2.0)
        outline.quadTo(ctrl4, QPointF(-bs0 / 2.0, y1))
        outline.lineTo(-bs0 / 2.0, y0 + r_top)
        outline.quadTo(QPointF(-bs0 / 2.0, y0), QPointF(-bs0 / 2.0 + r_top, y0))
        outline.closeSubpath()

        outline_pen = QPen(QColor(60, 110, 210, 235))
        outline_pen.setWidthF(1.2)
        outline_pen.setCosmetic(True)

        def _band(y_top: float, y_bot: float) -> QPainterPath:
            rect = QPainterPath()
            rect.addRect(-half_span * 2.0, y_top, half_span * 4.0, y_bot - y_top)
            return outline.intersected(rect)

        for (band, brush) in (
            (_band(y0, y1), QBrush(QColor(170, 220, 255, 158))),
            (_band(y1, y2), QBrush(QColor(140, 200, 255, 135))),
            (_band(y2, y3), QBrush(QColor(110, 180, 255, 142))),
        ):
            it = QGraphicsPathItem(band)
            it.setPen(QPen(Qt.PenStyle.NoPen))
            it.setBrush(brush)
            self.slot_scene.addItem(it)

        outline_item = QGraphicsPathItem(outline)
        outline_item.setPen(outline_pen)
        outline_item.setBrush(QBrush(QColor(255, 255, 255, 12)))
        self.slot_scene.addItem(outline_item)

        center_pen = QPen(QColor(60, 70, 90, 150))
        center_pen.setWidthF(1.0)
        center_pen.setCosmetic(True)
        center_pen.setStyle(Qt.PenStyle.DashLine)
        cl = QGraphicsLineItem(0.0, y0 - (r_top + scale * 1.4), 0.0, y3 + (r_bottom + scale * 2.0))
        cl.setPen(center_pen)
        self.slot_scene.addItem(cl)

        top_dim_y = y0 - (r_top + scale * 2.4)
        mid_dim_y = (y1 + y2) / 2.0 + scale * 2.1
        bot_dim_y = y3 + (r_bottom + scale * 2.4)
        self._add_dim_x(self.slot_scene, -bs0 / 2.0, +bs0 / 2.0, top_dim_y, "bs0", highlight=True)
        self._add_dim_x(self.slot_scene, -bs1 / 2.0, +bs1 / 2.0, mid_dim_y, "bs1", highlight=True)
        self._add_dim_x(self.slot_scene, -bs2 / 2.0, +bs2 / 2.0, bot_dim_y, "bs2", highlight=True)

        h0_len = max(0.0, y1 - y0)
        h1_len = max(0.0, y2 - y1)
        h2_len = max(0.0, y3 - y2)
        xh_base = -half_span - scale * 8.6
        col = scale * 3.6
        self._add_dim_y_abs(self.slot_scene, h0_len, xh_base - 0 * col, (y0 + y1) / 2.0, "hs0", highlight=True)
        self._add_dim_y_abs(self.slot_scene, h1_len, xh_base - 1 * col, (y1 + y2) / 2.0, "hs1", highlight=True)
        self._add_dim_y_abs(self.slot_scene, h2_len, xh_base - 2 * col, (y2 + y3) / 2.0, "hs2", highlight=True)

        bounds = outline.boundingRect()
        margin_x = max(scale * 11.0, half_span * 0.9)
        margin_y = max(scale * 18.0, r_bottom + r_top + scale * 6.0)
        self.slot_scene.setSceneRect(bounds.x() - margin_x, bounds.y() - margin_y,
                                     bounds.width() + 2 * margin_x, bounds.height() + 2 * margin_y)

        # Keep slot view auto-framed when it is redrawn.
        if getattr(self, "slot_view", None) is not None:
            self.slot_view.reset_view()


class WelcomePage(QWidget):
    """Landing page shown on startup (New / Load)."""

    def __init__(self, on_new, on_load, on_tables, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.setObjectName("welcomePage")

        root = QVBoxLayout(self)
        root.setContentsMargins(36, 32, 36, 32)
        root.setSpacing(18)

        root.addStretch(1)

        title = QLabel("Induction Motor Design Tool")
        title.setObjectName("welcomeTitle")
        title.setAlignment(Qt.AlignmentFlag.AlignHCenter | Qt.AlignmentFlag.AlignVCenter)
        title.setFont(QFont("Segoe UI", 22, QFont.Weight.Bold))
        root.addWidget(title)

        subtitle = QLabel("Stator + Rotor sizing • 2D views • PDF report")
        subtitle.setObjectName("welcomeSubtitle")
        subtitle.setAlignment(Qt.AlignmentFlag.AlignHCenter | Qt.AlignmentFlag.AlignVCenter)
        subtitle.setFont(QFont("Segoe UI", 11))
        root.addWidget(subtitle)

        cards = QWidget()
        cards_layout = QHBoxLayout(cards)
        cards_layout.setContentsMargins(0, 0, 0, 0)
        cards_layout.setSpacing(16)

        def _card(icon: QtGui.QIcon, heading: str, desc: str, button_text: str, callback):
            card = QWidget()
            card.setObjectName("welcomeCard")
            v = QVBoxLayout(card)
            v.setContentsMargins(18, 18, 18, 18)
            v.setSpacing(10)

            header_row = QHBoxLayout()
            ic = QLabel()
            ic.setPixmap(icon.pixmap(24, 24))
            ic.setObjectName("welcomeIcon")
            header_row.addWidget(ic, 0)

            h = QLabel(heading)
            h.setObjectName("welcomeCardTitle")
            h.setFont(QFont("Segoe UI", 12, QFont.Weight.DemiBold))
            header_row.addWidget(h, 1)
            header_row.addStretch(1)
            v.addLayout(header_row)

            d = QLabel(desc)
            d.setWordWrap(True)
            d.setObjectName("welcomeCardDesc")
            v.addWidget(d)

            v.addStretch(1)

            btn = QPushButton(button_text)
            btn.setProperty("role", "primary")
            btn.clicked.connect(callback)
            v.addWidget(btn)
            return card

        new_icon = self.style().standardIcon(QStyle.StandardPixmap.SP_FileIcon)
        open_icon = self.style().standardIcon(QStyle.StandardPixmap.SP_DialogOpenButton)

        cards_layout.addWidget(
            _card(
                new_icon,
                "Create a new project",
                "Start a new design. You can save it later using File → Save As.",
                "Start New Project",
                on_new,
            ),
            1,
        )
        cards_layout.addWidget(
            _card(
                open_icon,
                "Load an existing project",
                "Open a previously saved project (.motorproj.json) to restore inputs, results, and drawings.",
                "Load Existing File…",
                on_load,
            ),
            1,
        )
        root.addWidget(cards)

        actions = QWidget()
        actions_layout = QHBoxLayout(actions)
        actions_layout.setContentsMargins(0, 0, 0, 0)
        actions_layout.setSpacing(10)

        btn_tables = QPushButton("View Abaques Tables")
        btn_tables.clicked.connect(on_tables)
        actions_layout.addWidget(btn_tables, 0)

        actions_layout.addStretch(1)
        root.addWidget(actions)

        hint = QLabel("Tip: Use File → Save / Save As at any time.")
        hint.setObjectName("welcomeHint")
        hint.setAlignment(Qt.AlignmentFlag.AlignHCenter)
        root.addWidget(hint)

        root.addStretch(2)


class StatorDesignGUI(QMainWindow):
    """Interactive GUI for stator design with equation display"""

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Three-Phase Asynchronous Motor - Stator Design Tool")
        self.setGeometry(100, 100, 1400, 900)

        # Current project path (used by Save). None until New/Load.
        self._current_project_path: Optional[str] = None

        # Project modified flag (used for UX; Save behavior depends on path).
        self._project_dirty: bool = False

        # Captured defaults (used by New Project reset)
        self._initial_ui_state: Optional[dict[str, Any]] = None
        self._initial_tabs_state: Optional[dict[str, Any]] = None

        # ----------------------------
        # Menu bar (File -> New/Load/Save/Save As/Export)
        # ----------------------------
        self._init_menu()

        # Custom fonts (modern default)
        self.title_font = QFont("Segoe UI", 14, QFont.Weight.Bold)
        self.heading_font = QFont("Segoe UI", 11, QFont.Weight.DemiBold)
        self.normal_font = QFont("Segoe UI", 10)
        self.equation_font = QFont("Courier New", 9)

        # Goal-based selection for lambda/heuristic
        self.method_options = ['minimum_cost', 'good_efficiency', 'good_power_factor', 'balanced_design']
        self.preference_ranges = {
            'minimum_cost': (1.5, 2.0),
            'good_efficiency': (1.4, 1.6),
            'balanced_design': (1.0, 1.1),
        }
        # Slot design methods
        self.slot_method_options = [
            ("Trapezoidal semi-open slot", "tapered_slot"),
            ("Semi-open slot", "semi_open_slot_new"),
            ("Open slot", "open_slot"),
        ]
        # Keep a full copy for dynamic gating (Voltage/Power constraints)
        self._slot_method_options_all = list(self.slot_method_options)
        self._slot_method_gated_open_only = False

        # Design results storage
        self.designer = None
        self.results = None

        # Cached stator outputs needed by rotor calculation
        self.stator_to_rotor_inputs = None

        # Rotor results storage (separate from stator export cache)
        self.rotor_results = None

        # UI gating: rotor features locked until stator calculation succeeds
        self.stator_design_done = False

        # Export gating: export is available only after BOTH stator and rotor designs are done
        # (rotor_results must correspond to current stator design).
        self._export_ready = False

        # Electrical steel selection storage
        self.selected_steel_data = None

        # ----------------------------
        # Two/three-page structure (welcome + current GUI + 2D view page)
        # ----------------------------
        self._stacked = QStackedWidget()
        self.setCentralWidget(self._stacked)

        # Page 0: welcome
        self._page_welcome = WelcomePage(
            self.new_project,
            self.load_project,
            self.show_abaques_tables_dialog,
        )
        self._stacked.addWidget(self._page_welcome)

        # Page 1: current GUI
        self._page_main = QWidget()
        main_layout = QHBoxLayout(self._page_main)
        self.create_widgets(main_layout)
        self._stacked.addWidget(self._page_main)

        # Snapshot initial defaults after widgets exist.
        try:
            self._initial_ui_state = self._collect_ui_state()
            self._initial_tabs_state = self._tabs_to_dict()
        except Exception:
            self._initial_ui_state = None
            self._initial_tabs_state = None

        # Page 2: empty placeholder for 2D view (structure only)
        self._page_2d = QWidget()
        page_2d_layout = QVBoxLayout(self._page_2d)
        page_2d_layout.setContentsMargins(12, 12, 12, 12)
        self.back_to_main_button = QPushButton("Go Back")
        self.back_to_main_button.clicked.connect(self.go_back_to_main_page)
        page_2d_layout.addWidget(self.back_to_main_button, 0)

        # Scrollable content (stator -> rotor -> assembly), so user can scroll down.
        self._page_2d_scroll = QScrollArea()
        self._page_2d_scroll.setWidgetResizable(True)
        self._page_2d_content = QWidget()
        self._page_2d_scroll.setWidget(self._page_2d_content)
        content_layout = QVBoxLayout(self._page_2d_content)
        content_layout.setContentsMargins(0, 0, 0, 0)

        self.stator_2d_widget = Stator2DWidget()
        content_layout.addWidget(self.stator_2d_widget, 0)

        self.rotor_2d_group = QGroupBox("Rotor 2D")
        rotor_group_layout = QVBoxLayout(self.rotor_2d_group)
        self.rotor_2d_widget = Rotor2DWidget()
        rotor_group_layout.addWidget(self.rotor_2d_widget, 0)
        content_layout.addWidget(self.rotor_2d_group, 0)

        self.assembly_group = QGroupBox("Assembly")
        assembly_layout = QVBoxLayout(self.assembly_group)
        self.assembly_2d_widget = Assembly2DWidget()
        assembly_layout.addWidget(self.assembly_2d_widget, 0)
        content_layout.addWidget(self.assembly_group, 0)

        # Spacer to keep content naturally sized (no forced compression).
        content_layout.addStretch(1)

        # Rotor + assembly are visible only after rotor calculation.
        self.rotor_2d_group.setVisible(False)
        self.assembly_group.setVisible(False)

        page_2d_layout.addWidget(self._page_2d_scroll, 1)
        self._stacked.addWidget(self._page_2d)

        # Start on welcome page.
        self._stacked.setCurrentWidget(self._page_welcome)

        # Cache of last stator geometry used for 2D view
        self._last_stator_2d_params: Optional[Stator2DParams] = None

        # Cache of last rotor geometry used for 2D view
        self._last_rotor_2d_params: Optional[Rotor2DParams] = None

        # Lock rotor tab/button until stator design is done
        self.set_rotor_ui_enabled(False)

        # Status bar
        self.status_bar = QStatusBar()
        self.setStatusBar(self.status_bar)
        self.status_bar.showMessage("Ready")

        # Initialize export button state
        self.update_export_button_visibility()
        self.update_2d_view_button_visibility()
        self.update_project_button_visibility()
        self._update_window_title()

    def _init_menu(self):
        """Create the top menu bar (File actions)."""
        file_menu = self.menuBar().addMenu("File")

        self.action_new_project = file_menu.addAction("New Project…")
        self.action_new_project.triggered.connect(self.new_project)
        self.action_new_project.setStatusTip("Start a new design (use Save As to create a project file).")

        self.action_load_project = file_menu.addAction("Load Project…")
        self.action_load_project.triggered.connect(self.load_project)
        self.action_load_project.setStatusTip("Load a saved project (.json) to restore results and 2D drawings.")

        file_menu.addSeparator()

        self.action_save = file_menu.addAction("Save")
        self.action_save.triggered.connect(self.save_project)
        self.action_save.setEnabled(False)
        self.action_save.setStatusTip("Save to the current project file.")

        self.action_save_as = file_menu.addAction("Save As…")
        self.action_save_as.triggered.connect(self.save_project_as)
        self.action_save_as.setStatusTip("Save the current work to a new project file.")

        file_menu.addSeparator()

        self.action_export_pdf = file_menu.addAction("Export PDF Report…")
        self.action_export_pdf.triggered.connect(self.export_results)
        self.action_export_pdf.setEnabled(False)
        self.action_export_pdf.setStatusTip("Locked: requires both stator + rotor calculations.")

        # Presets menu (quick-start motor specs)
        presets_menu = self.menuBar().addMenu("Presets")
        self.action_show_presets = presets_menu.addAction("Choose Preset…")
        self.action_show_presets.triggered.connect(self.show_presets_dialog)
        self.action_show_presets.setStatusTip("Apply a predefined motor specification preset.")

    def show_presets_dialog(self):
        """Show a dialog with common motor specification presets and apply the selected one."""

        presets: list[dict[str, Any]] = [
            {
                'key': 'industrial_pump',
                'title': '[ 5.5 kW | 400V Δ | 4-Pole ] - Industrial Pump',
                'description': 'A standard, general-purpose motor for pumps, fans, and conveyors.',
                'power_kw': 5.5,
                'voltage_v': 400.0,
                'frequency_hz': 50.0,
                'poles': 4,
                'connection': 'delta',
                'winding_type': 'short pitch (5/6)',
                'winding_note': 'Short-pitch (5/6) is widely used to reduce harmonics and noise.',
            },
            {
                'key': 'compressor',
                'title': '[ 90 kW | 400V Y | 2-Pole ] - Compressor',
                'description': 'A robust motor for heavy-duty applications like air compressors.',
                'power_kw': 90.0,
                'voltage_v': 400.0,
                'frequency_hz': 50.0,
                'poles': 2,
                'connection': 'star',
                'winding_type': 'short pitch (4/5)',
                'winding_note': 'Common industrial choice: short-pitch (4/5).',
            },
            {
                'key': 'workshop',
                'title': '[ 1.1 kW | 230V Y | 4-Pole ] - Workshop',
                'description': 'A common motor for small workshops (lathes, saws, etc.).',
                'power_kw': 1.1,
                'voltage_v': 230.0,
                'frequency_hz': 50.0,
                'poles': 4,
                'connection': 'star',
                'winding_type': 'full pitch',
                'winding_note': 'Full pitch is simple and common for small motors.',
            },
            {
                'key': 'crusher_mixer',
                'title': '[ 37 kW | 400V Δ | 8-Pole ] - Crusher/Mixer',
                'description': 'A high-torque, slow-speed motor for crushers, mixers, or large blowers.',
                'power_kw': 37.0,
                'voltage_v': 400.0,
                'frequency_hz': 50.0,
                'poles': 8,
                'connection': 'delta',
                'winding_type': 'short pitch (9/10)',
                'winding_note': 'High pole-count motors often benefit from short-pitch (9/10).',
            },
        ]

        dlg = QDialog(self)
        dlg.setWindowTitle("Motor Presets")
        dlg.setModal(True)
        dlg.setMinimumSize(920, 560)

        root = QVBoxLayout(dlg)
        root.setContentsMargins(12, 12, 12, 12)
        root.setSpacing(10)

        title = QLabel("Quick Presets")
        title.setFont(QFont("Segoe UI", 12, QFont.Weight.DemiBold))
        root.addWidget(title, 0)

        subtitle = QLabel("Select a preset below and click Apply. You can fine-tune any value afterwards.")
        subtitle.setStyleSheet("color: gray;")
        root.addWidget(subtitle, 0)

        # Table with presets
        table = QTableWidget(len(presets), 6, dlg)
        table.setHorizontalHeaderLabels(["Preset", "Power", "Voltage", "Connection", "Poles", "Winding (pitch)"])
        table.verticalHeader().setVisible(False)
        table.setSelectionBehavior(QTableWidget.SelectionBehavior.SelectRows)
        table.setSelectionMode(QTableWidget.SelectionMode.SingleSelection)
        table.setEditTriggers(QTableWidget.EditTrigger.NoEditTriggers)
        table.setAlternatingRowColors(True)

        for r, p in enumerate(presets):
            table.setItem(r, 0, QTableWidgetItem(str(p.get('title', ''))))
            table.setItem(r, 1, QTableWidgetItem(f"{float(p.get('power_kw', 0.0)):.3g} kW"))
            table.setItem(r, 2, QTableWidgetItem(f"{float(p.get('voltage_v', 0.0)):.0f} V"))
            conn = 'Δ (Delta)' if p.get('connection') == 'delta' else 'Y (Star)'
            table.setItem(r, 3, QTableWidgetItem(conn))
            table.setItem(r, 4, QTableWidgetItem(str(int(p.get('poles', 0) or 0))))
            table.setItem(r, 5, QTableWidgetItem(str(p.get('winding_type', ''))))

        try:
            header = table.horizontalHeader()
            header.setSectionResizeMode(0, QHeaderView.ResizeMode.Stretch)
            for c in (1, 2, 3, 4, 5):
                header.setSectionResizeMode(c, QHeaderView.ResizeMode.ResizeToContents)
        except Exception:
            pass

        root.addWidget(table, 1)

        details = QTextEdit(dlg)
        details.setReadOnly(True)
        details.setMinimumHeight(140)
        root.addWidget(details, 0)

        rotor_note = QLabel(
            "Rotor note: Presets only change the main motor specification inputs. "
            "Your rotor configuration stays as-is (you can keep it or customize it freely)."
        )
        rotor_note.setWordWrap(True)
        rotor_note.setStyleSheet("color: gray; font-size: 9pt;")
        root.addWidget(rotor_note, 0)

        btn_row = QHBoxLayout()
        btn_row.addStretch(1)
        btn_cancel = QPushButton("Cancel")
        btn_apply = QPushButton("Apply Preset")
        btn_apply.setEnabled(False)
        btn_row.addWidget(btn_cancel, 0)
        btn_row.addWidget(btn_apply, 0)
        root.addLayout(btn_row)

        def _selected_index() -> int:
            try:
                rows = table.selectionModel().selectedRows()
                if rows:
                    return int(rows[0].row())
            except Exception:
                return -1
            return -1

        def _render_details(p: dict[str, Any]):
            conn = "Delta (Δ)" if p.get('connection') == 'delta' else "Star (Y)"
            txt = (
                f"<b>{p.get('title','')}</b><br><br>"
                f"<b>Description:</b> {p.get('description','')}<br><br>"
                f"<b>Specs:</b><br>"
                f"&bull; Power (Pn): {float(p.get('power_kw', 0.0)):.3g} kW<br>"
                f"&bull; Line Voltage (Vn): {float(p.get('voltage_v', 0.0)):.0f} V (3-phase)<br>"
                f"&bull; Frequency (fn): {float(p.get('frequency_hz', 0.0)):.0f} Hz<br>"
                f"&bull; Poles: {int(p.get('poles', 0) or 0)}<br>"
                f"&bull; Stator connection: {conn}<br>"
                f"&bull; Winding pitch: {p.get('winding_type','')}<br><br>"
                f"<b>Winding note:</b> {p.get('winding_note','')}<br>"
            )
            details.setHtml(txt)

        def _on_selection_changed():
            idx = _selected_index()
            if idx < 0 or idx >= len(presets):
                btn_apply.setEnabled(False)
                details.setPlainText("")
                return
            btn_apply.setEnabled(True)
            _render_details(presets[idx])

        def _apply_selected():
            idx = _selected_index()
            if idx < 0 or idx >= len(presets):
                return
            p = presets[idx]
            ok = self.apply_preset(p)
            if ok:
                dlg.accept()

        table.itemSelectionChanged.connect(_on_selection_changed)
        table.itemDoubleClicked.connect(lambda _item: _apply_selected())
        btn_cancel.clicked.connect(dlg.reject)
        btn_apply.clicked.connect(_apply_selected)

        # Pre-select first row for convenience
        try:
            if presets:
                table.selectRow(0)
        except Exception:
            pass

        dlg.exec()

    def apply_preset(self, preset: dict[str, Any]) -> bool:
        """Apply a preset to the main motor specification inputs."""
        if not isinstance(preset, dict):
            return False

        # Ensure we're on the main page so the user immediately sees the changes.
        try:
            if hasattr(self, '_stacked'):
                self._stacked.setCurrentWidget(self._page_main)
        except Exception:
            pass

        try:
            # Fill numeric inputs
            if hasattr(self, 'power_input') and isinstance(self.power_input, QLineEdit):
                self.power_input.setText(str(preset.get('power_kw', '')))
            if hasattr(self, 'voltage_input') and isinstance(self.voltage_input, QLineEdit):
                self.voltage_input.setText(str(preset.get('voltage_v', '')))
            if hasattr(self, 'frequency_input') and isinstance(self.frequency_input, QLineEdit):
                self.frequency_input.setText(str(preset.get('frequency_hz', '')))

            # Poles
            poles = str(int(preset.get('poles', 0) or 0))
            if hasattr(self, 'poles_combo') and isinstance(self.poles_combo, QComboBox) and poles:
                idx = self.poles_combo.findText(poles)
                if idx >= 0:
                    self.poles_combo.setCurrentIndex(idx)
                else:
                    self.poles_combo.setCurrentText(poles)

            # Connection
            conn = str(preset.get('connection', '') or '').strip().lower()
            if conn == 'delta':
                if hasattr(self, 'delta_radio') and isinstance(self.delta_radio, QRadioButton):
                    self.delta_radio.setChecked(True)
            elif conn == 'star':
                if hasattr(self, 'star_radio') and isinstance(self.star_radio, QRadioButton):
                    self.star_radio.setChecked(True)

            # Winding pitch
            wt = str(preset.get('winding_type', '') or '').strip()
            if wt and hasattr(self, 'winding_type_combo') and isinstance(self.winding_type_combo, QComboBox):
                idx = self.winding_type_combo.findText(wt)
                if idx >= 0:
                    self.winding_type_combo.setCurrentIndex(idx)
                else:
                    self.winding_type_combo.setCurrentText(wt)

            # Re-apply dependent UI updates
            try:
                self.update_slot_method_options_based_on_inputs()
            except Exception:
                pass
            try:
                self.update_coil_pitch_options()
            except Exception:
                pass

            # Mark dirty
            self._project_dirty = True

            try:
                self.status_bar.showMessage(f"Preset applied: {preset.get('title','')}")
            except Exception:
                pass
            return True
        except Exception as e:
            QMessageBox.warning(self, "Preset", f"Failed to apply preset:\n{e}")
            return False

    def update_2d_view_button_visibility(self):
        """Enable/disable 2D view button (locked until stator design is available)."""
        enabled = bool(getattr(self, 'stator_design_done', False))
        if hasattr(self, 'show_2d_view_button'):
            self.show_2d_view_button.setVisible(True)
            self.show_2d_view_button.setEnabled(enabled)
            base_text = getattr(self, '_show_2d_view_button_base_text', None) or self.show_2d_view_button.text().replace(" (locked)", "")
            self.show_2d_view_button.setText(base_text if enabled else f"{base_text} (locked)")
            self.show_2d_view_button.setToolTip(
                "" if enabled else "Locked: run a stator calculation (or load a project) to enable the 2D view."
            )

    def update_project_button_visibility(self):
        """Update Save/Save As enablement based on whether a project path is known."""
        has_path = bool(getattr(self, '_current_project_path', None))
        if hasattr(self, 'action_save') and self.action_save is not None:
            self.action_save.setEnabled(has_path)
            self.action_save.setText("Save" if has_path else "Save (select file first)")
        if hasattr(self, 'action_save_as') and self.action_save_as is not None:
            self.action_save_as.setEnabled(True)

    def show_2d_view_page(self):
        """Navigate to the (currently empty) 2D view page."""
        if getattr(self, 'stator_design_done', False):
            self._update_2d_view_from_current_design()
        self._update_2d_view_from_current_rotor()
        if hasattr(self, '_stacked'):
            self._stacked.setCurrentWidget(self._page_2d)

    def go_back_to_main_page(self):
        """Return to the main design/calculation page."""
        if hasattr(self, '_stacked'):
            self._stacked.setCurrentWidget(self._page_main)

    def show_abaques_tables_dialog(self):
        """Display the empirical tables (abaques) used by the design engine."""
        dlg = QDialog(self)
        dlg.setWindowTitle("Abaques Tables")
        dlg.setModal(True)
        dlg.setMinimumSize(980, 640)

        layout = QVBoxLayout(dlg)
        layout.setContentsMargins(12, 12, 12, 12)

        title = QLabel("Empirical Tables (Abaques)")
        title.setFont(QFont("Segoe UI", 12, QFont.Weight.DemiBold))
        layout.addWidget(title)

        tabs = QTabWidget(dlg)
        layout.addWidget(tabs, 1)

        def _make_table(headers: list[str], rows: list[list[str]]) -> QWidget:
            w = QWidget()
            v = QVBoxLayout(w)
            v.setContentsMargins(0, 0, 0, 0)
            table = QTableWidget(len(rows), len(headers), w)
            table.setHorizontalHeaderLabels(headers)
            table.verticalHeader().setVisible(False)
            table.setEditTriggers(QTableWidget.EditTrigger.NoEditTriggers)
            table.setSelectionBehavior(QTableWidget.SelectionBehavior.SelectRows)
            table.setSelectionMode(QTableWidget.SelectionMode.SingleSelection)
            table.horizontalHeader().setStretchLastSection(True)
            for r, row in enumerate(rows):
                for c, cell in enumerate(row):
                    table.setItem(r, c, QTableWidgetItem(str(cell)))
            table.resizeColumnsToContents()
            v.addWidget(table, 1)
            return w

        # Table 7.1: Bav and q vs Power
        t71 = getattr(AbaquesTables, 'TABLE_7_1', {}) or {}
        rows_71: list[list[str]] = []
        try:
            for pkw in sorted(t71.keys(), key=lambda x: float(x)):
                rec = t71.get(pkw, {}) or {}
                rows_71.append([
                    str(pkw),
                    str(rec.get('Bav', '')),
                    str(rec.get('q', '')),
                ])
        except Exception:
            pass
        tabs.addTab(_make_table(["Power (kW)", "Bav (T)", "q (A/m)"], rows_71), "Table 7.1")

        # q recommendations table (power range + poles)
        tq = getattr(AbaquesTables, 'TABLE_Q_RECOMMENDATIONS', {}) or {}
        rows_q: list[list[str]] = []
        try:
            for key in sorted(tq.keys(), key=lambda k: (float(k[0]), float(k[1]), int(k[2]))):
                pmin, pmax, poles = key
                qmin, qmax = tq.get(key, ("", ""))
                rows_q.append([str(pmin), str(pmax), str(poles), str(qmin), str(qmax)])
        except Exception:
            pass
        tabs.addTab(_make_table(["Pmin (kW)", "Pmax (kW)", "Poles", "q min", "q max"], rows_q), "q recommendations")

        # Carter coefficient tables
        co_open = getattr(AbaquesTables, 'CARTER_COEFF_OPEN_SLOT', {}) or {}
        rows_co_open: list[list[str]] = []
        try:
            xs = list(co_open.get('opening_airgap', []) or [])
            ys = list(co_open.get('carter_coeff', []) or [])
            for x, y in zip(xs, ys):
                rows_co_open.append([str(x), str(y)])
        except Exception:
            pass
        tabs.addTab(_make_table(["opening_airgap", "carter_coeff"], rows_co_open), "Carter (open slot)")

        co_semi = getattr(AbaquesTables, 'CARTER_COEFF_SEMI_OPEN_SLOT', {}) or {}
        rows_co_semi: list[list[str]] = []
        try:
            xs = list(co_semi.get('slot_opening_gap', []) or [])
            ys = list(co_semi.get('carter_coeff', []) or [])
            for x, y in zip(xs, ys):
                rows_co_semi.append([str(x), str(y)])
        except Exception:
            pass
        tabs.addTab(_make_table(["slot_opening_gap", "carter_coeff"], rows_co_semi), "Carter (semi-open)")

        # Current density vs diameter table
        d_list = list(getattr(AbaquesTables, 'J_DIAMETER_TABLE_D', []) or [])
        j_list = list(getattr(AbaquesTables, 'J_DIAMETER_TABLE_J', []) or [])
        rows_j: list[list[str]] = [[str(d), str(j)] for d, j in zip(d_list, j_list)]
        tabs.addTab(_make_table(["D (mm)", "J (A/mm²)"], rows_j), "J vs diameter")

        # Lamination materials (steel grades database)
        rows_steel: list[list[str]] = []
        try:
            db = list(STEEL_DATABASE or [])
            for row in db:
                grade = str(row.get('Grade', '') or '')
                th = row.get('Thickness_mm', '')
                maxB = row.get('Max_Design_B', '')
                loss = row.get('Loss_W_kg', '')
                app = row.get('Application', '')
                dens = row.get('Specific_Density_kg_m3', '')
                try:
                    ki = get_stacking_factor(float(th)) if th not in ('', None) else ''
                except Exception:
                    ki = ''
                rows_steel.append([
                    grade,
                    str(th),
                    str(maxB),
                    str(loss),
                    str(ki),
                    str(dens),
                    str(app),
                ])
        except Exception:
            pass
        tabs.addTab(
            _make_table(
                ["Grade", "Thickness (mm)", "Max design B (T)", "Loss (W/kg)", "Stacking factor ki", "Density (kg/m³)", "Application"],
                rows_steel,
            ),
            "Laminations",
        )

        # SWG wire table
        rows_swg: list[list[str]] = []
        try:
            for g, dmm in zip(list(SWG_GAUGE or []), list(SWG_DIAMETER_MM or [])):
                rows_swg.append([str(g), f"{float(dmm):.4f}"])
        except Exception:
            pass
        tabs.addTab(_make_table(["SWG", "Diameter (mm)"], rows_swg), "SWG wire")

        # Rectangular strip (Open slot) abaques
        rows_strip: list[list[str]] = []
        try:
            Imin = list(getattr(AbaquesTables, 'OPEN_SLOT_I_MIN_A', []) or [])
            Imax = list(getattr(AbaquesTables, 'OPEN_SLOT_I_MAX_A', []) or [])
            tmin = list(getattr(AbaquesTables, 'OPEN_SLOT_THICKNESS_MIN_MM', []) or [])
            tmax = list(getattr(AbaquesTables, 'OPEN_SLOT_THICKNESS_MAX_MM', []) or [])
            wmin = list(getattr(AbaquesTables, 'OPEN_SLOT_WIDTH_MIN_MM', []) or [])
            wmax = list(getattr(AbaquesTables, 'OPEN_SLOT_WIDTH_MAX_MM', []) or [])
            rmin = list(getattr(AbaquesTables, 'OPEN_SLOT_RATIO_MIN', []) or [])
            rmax = list(getattr(AbaquesTables, 'OPEN_SLOT_RATIO_MAX', []) or [])
            n = min(len(Imin), len(Imax), len(tmin), len(tmax), len(wmin), len(wmax), len(rmin), len(rmax))
            for i in range(n):
                rows_strip.append([
                    f"{Imin[i]}",
                    f"{Imax[i]}",
                    f"{tmin[i]}",
                    f"{tmax[i]}",
                    f"{wmin[i]}",
                    f"{wmax[i]}",
                    f"{rmin[i]}",
                    f"{rmax[i]}",
                ])
        except Exception:
            pass
        tabs.addTab(
            _make_table(
                [
                    "I min (A)", "I max (A)",
                    "Thickness min (mm)", "Thickness max (mm)",
                    "Width min (mm)", "Width max (mm)",
                    "Ratio min", "Ratio max",
                ],
                rows_strip,
            ),
            "Rectangular strip",
        )

        # Standard thickness list (Open slot)
        rows_std_t: list[list[str]] = []
        try:
            std = list(getattr(AbaquesTables, 'OPEN_SLOT_STANDARD_THICKNESSES_MM', []) or [])
            for v in std:
                rows_std_t.append([f"{float(v):.4f}"])
        except Exception:
            pass
        tabs.addTab(_make_table(["Standard thickness (mm)"], rows_std_t), "Strip standards")

        # B-H curves for steel grades (selectable)
        bh_tab = QWidget()
        bh_layout = QVBoxLayout(bh_tab)
        bh_layout.setContentsMargins(0, 0, 0, 0)
        bh_layout.setSpacing(8)

        bh_top = QWidget()
        bh_top_layout = QHBoxLayout(bh_top)
        bh_top_layout.setContentsMargins(0, 0, 0, 0)
        bh_top_layout.setSpacing(8)
        bh_top_layout.addWidget(QLabel("Steel grade:"), 0)
        bh_grade = QComboBox()
        try:
            grades = sorted(list(getattr(AbaquesTables, 'STEEL_BH_CURVES', {}).keys()))
        except Exception:
            grades = []
        for g in grades:
            bh_grade.addItem(str(g))
        bh_top_layout.addWidget(bh_grade, 0)
        bh_top_layout.addStretch(1)
        bh_layout.addWidget(bh_top, 0)

        bh_table = QTableWidget(0, 2, bh_tab)
        bh_table.setHorizontalHeaderLabels(["B (T)", "H (A/m)"])
        bh_table.verticalHeader().setVisible(False)
        bh_table.setEditTriggers(QTableWidget.EditTrigger.NoEditTriggers)
        bh_table.setSelectionBehavior(QTableWidget.SelectionBehavior.SelectRows)
        bh_table.setSelectionMode(QTableWidget.SelectionMode.SingleSelection)
        bh_table.horizontalHeader().setStretchLastSection(True)
        bh_layout.addWidget(bh_table, 1)

        def _refresh_bh(grade: str):
            curves = getattr(AbaquesTables, 'STEEL_BH_CURVES', {}) or {}
            curve = curves.get(str(grade), {}) or {}
            Bs = list(curve.get('B', []) or [])
            Hs = list(curve.get('H', []) or [])
            n = min(len(Bs), len(Hs))
            bh_table.setRowCount(n)
            for i in range(n):
                bh_table.setItem(i, 0, QTableWidgetItem(str(Bs[i])))
                bh_table.setItem(i, 1, QTableWidgetItem(str(Hs[i])))
            bh_table.resizeColumnsToContents()

        try:
            bh_grade.currentTextChanged.connect(_refresh_bh)
        except Exception:
            pass
        if grades:
            _refresh_bh(grades[0])
        tabs.addTab(bh_tab, "B-H curves")

        close_btn = QPushButton("Close")
        close_btn.clicked.connect(dlg.accept)
        layout.addWidget(close_btn, 0)

        dlg.exec()

    def _update_2d_view_from_current_design(self):
        """Pull the stator 2D inputs from the current stator calculation and update the 2D page."""
        if not getattr(self, 'stator_design_done', False):
            return

        # Allow restoring the 2D page from cached/loaded parameters even when
        # the full stator designer object is not available.
        if self.designer is None:
            params = getattr(self, '_last_stator_2d_params', None)
            if params is None:
                return
            if hasattr(self, 'stator_2d_widget') and self.stator_2d_widget is not None:
                self.stator_2d_widget.set_params(params)
            self._update_2d_view_assembly()
            return

        geom = getattr(self.designer, 'geometry', None)
        slots = getattr(self.designer, 'slots', None)
        specs = getattr(self.designer, 'specs', None)
        if geom is None or slots is None or specs is None:
            return

        D_mm = float(getattr(geom, 'D', 0.0) or 0.0) * 1000.0
        Dext_mm = float(getattr(geom, 'D_ext', 0.0) or 0.0) * 1000.0
        Ss = int(getattr(slots, 'Ss', 0) or 0)
        if D_mm <= 0 or Dext_mm <= 0 or Ss <= 0:
            return

        val_info = geom.validation_info if hasattr(geom, 'validation_info') and geom.validation_info else {}
        slot_method = str(getattr(specs, 'slot_method', '') or '')

        if slot_method == 'open_slot':
            bs_mm = val_info.get('open_slot_bs_mm', None)
            hs_mm = val_info.get('open_slot_hs_mm', None)
            params = Stator2DParams(
                D_mm=D_mm,
                Dext_mm=Dext_mm,
                Ss=Ss,
                slot_type='square',
                bs_mm=float(bs_mm) if bs_mm is not None else 0.0,
                hs_mm=float(hs_mm) if hs_mm is not None else 0.0,
            )
        else:
            bs0_mm = val_info.get('bs0_mm', None)
            bs1_mm = val_info.get('bs1_mm', None)
            bs2_mm = val_info.get('bs2_mm', None)
            hs0_mm = val_info.get('slot_dim_hs0_mm', val_info.get('hs0_mm', None))
            hs1_mm = val_info.get('slot_dim_hs1_mm', val_info.get('hs1_mm', None))
            hs2_mm = val_info.get('hs2_mm', None)
            params = Stator2DParams(
                D_mm=D_mm,
                Dext_mm=Dext_mm,
                Ss=Ss,
                slot_type='trapez',
                bs0_mm=float(bs0_mm) if bs0_mm is not None else 0.0,
                hs0_mm=float(hs0_mm) if hs0_mm is not None else 0.0,
                bs1_mm=float(bs1_mm) if bs1_mm is not None else 0.0,
                hs1_mm=float(hs1_mm) if hs1_mm is not None else 0.0,
                bs2_mm=float(bs2_mm) if bs2_mm is not None else 0.0,
                hs2_mm=float(hs2_mm) if hs2_mm is not None else 0.0,
            )

        self._last_stator_2d_params = params
        if hasattr(self, 'stator_2d_widget') and self.stator_2d_widget is not None:
            self.stator_2d_widget.set_params(params)

        # Assembly may be updated if rotor already exists.
        self._update_2d_view_assembly()

    def _set_rotor_2d_visible(self, visible: bool):
        if hasattr(self, 'rotor_2d_group'):
            self.rotor_2d_group.setVisible(bool(visible))
        if hasattr(self, 'assembly_group'):
            self.assembly_group.setVisible(bool(visible))

    def _update_2d_view_from_current_rotor(self):
        """Pull rotor 2D inputs from the latest rotor calculation and update rotor + assembly drawings."""
        rotor = dict(getattr(self, 'rotor_results', {}) or {})
        if not rotor:
            self._last_rotor_2d_params = None
            if hasattr(self, 'rotor_2d_widget') and self.rotor_2d_widget is not None:
                self.rotor_2d_widget.clear_params()
            if hasattr(self, 'assembly_2d_widget') and self.assembly_2d_widget is not None:
                self.assembly_2d_widget.clear_params()
            self._set_rotor_2d_visible(False)
            return

        try:
            Dr_mm = float(rotor.get('Dr_mm', 0.0) or 0.0)
            Nr = int(rotor.get('Nr', 0) or 0)
            h0_mm = float(rotor.get('h_r0_mm', 0.0) or 0.0)
            b0_mm = float(rotor.get('b_r0_mm', 0.0) or 0.0)
            r1_mm = float(rotor.get('r1_mm', 0.0) or 0.0)
            r2_mm = float(rotor.get('r2_mm', rotor.get('r1_mm', 0.0)) or 0.0)
            drb_mm = float(rotor.get('d_rb_mm', 0.0) or 0.0)
            Wry_mm = float(rotor.get('Wry_mm', 0.0) or 0.0)
        except Exception:
            self._set_rotor_2d_visible(False)
            return

        if Dr_mm <= 0.0 or Nr <= 0:
            self._set_rotor_2d_visible(False)
            return

        shaft_d_mm = max(0.0, Dr_mm - 2.0 * (h0_mm + drb_mm + Wry_mm))
        params = Rotor2DParams(
            Dr_mm=Dr_mm,
            Nr=Nr,
            shaft_d_mm=shaft_d_mm,
            h0_mm=h0_mm,
            b0_mm=b0_mm,
            r1_mm=r1_mm,
            r2_mm=r2_mm,
            drb_mm=drb_mm,
        )

        self._last_rotor_2d_params = params
        if hasattr(self, 'rotor_2d_widget') and self.rotor_2d_widget is not None:
            self.rotor_2d_widget.set_params(params)
        self._set_rotor_2d_visible(True)

        self._update_2d_view_assembly()

    def _update_2d_view_assembly(self):
        if not hasattr(self, 'assembly_2d_widget') or self.assembly_2d_widget is None:
            return
        stator = getattr(self, '_last_stator_2d_params', None)
        rotor = getattr(self, '_last_rotor_2d_params', None)
        if stator is not None and rotor is not None:
            self.assembly_2d_widget.set_params(stator, rotor)
        else:
            self.assembly_2d_widget.clear_params()

    def update_export_button_visibility(self):
        """Show Export button only after stator AND rotor designs are computed."""
        def _has_saved_report_text() -> bool:
            tab_attrs = [
                'tab_empirical',
                'tab_electrical',
                'tab_dimensions',
                'tab_winding',
                'tab_slots',
                'tab_outer',
                'tab_losses',
                'tab_rotor',
                'tab_efficiency',
            ]
            for a in tab_attrs:
                t = getattr(self, a, None)
                if t is None:
                    continue
                try:
                    if (t.toPlainText() or '').strip():
                        return True
                except Exception:
                    continue
            return False

        has_stator_model = bool(self.designer and getattr(self.designer, 'geometry', None) and getattr(self.designer, 'slots', None))
        has_core = bool(getattr(self, 'stator_design_done', False)) and bool(getattr(self, 'rotor_results', None))

        # Two modes:
        # 1) Live model mode: stator model exists and rotor computed.
        # 2) Loaded-project mode: we have saved stator+rotor state and saved report text, even if designer is not rebuilt.
        ready = (has_stator_model and has_core) or (has_core and _has_saved_report_text())
        self._export_ready = ready
        if hasattr(self, 'action_export_pdf') and self.action_export_pdf is not None:
            self.action_export_pdf.setEnabled(ready)
            # Make locked state obvious without clicking.
            self.action_export_pdf.setText("Export PDF Report…" if ready else "Export PDF Report… (locked)")
            self.action_export_pdf.setStatusTip(
                "Export the current results to a PDF report." if ready else "Locked: requires both stator + rotor calculations (or a loaded project with saved results)."
            )

        # Efficiency analysis tab:
        # - In normal workflow, appears only when both stator + rotor are done.
        # - When loading a saved project, we also show it if saved text exists.
        eff_text_present = False
        if hasattr(self, 'tab_efficiency') and self.tab_efficiency is not None:
            try:
                eff_text_present = bool((self.tab_efficiency.toPlainText() or '').strip())
            except Exception:
                eff_text_present = False
        eff_ready = bool(getattr(self, 'stator_design_done', False)) and bool(getattr(self, 'rotor_results', None))
        self._set_efficiency_tab_visible(bool(eff_ready or eff_text_present))

        # Auto-compute only when we have a live designer + rotor (not for loaded display-only projects)
        if has_stator_model and eff_ready:
            self.show_efficiency_analysis()

        # Save button depends only on stator being available.
        self.update_project_button_visibility()

    def _update_window_title(self):
        base = "Three-Phase Asynchronous Motor - Stator Design Tool"
        p = getattr(self, '_current_project_path', None)
        if p:
            name = os.path.basename(str(p))
            self.setWindowTitle(f"{base} — {name}")
        else:
            self.setWindowTitle(base)

    def _build_project_snapshot(self) -> dict[str, Any]:
        # Refresh cached 2D params when possible.
        try:
            self._update_2d_view_from_current_design()
        except Exception:
            pass
        try:
            self._update_2d_view_from_current_rotor()
        except Exception:
            pass

        # Snapshot minimal computed stator summary so loaded-project PDF export can match
        # live export (even when we don't rebuild self.designer).
        stator_summary = None
        try:
            if self.designer is not None and getattr(self.designer, 'geometry', None) is not None and getattr(self.designer, 'slots', None) is not None:
                specs = getattr(self.designer, 'specs', None)
                geom = getattr(self.designer, 'geometry', None)
                slots = getattr(self.designer, 'slots', None)
                if specs is not None and geom is not None and slots is not None:
                    stator_summary = {
                        'specs': {
                            'power_kw': self._jsonify(getattr(specs, 'power_kw', None)),
                            'voltage_v': self._jsonify(getattr(specs, 'voltage_v', None)),
                            'frequency_hz': self._jsonify(getattr(specs, 'frequency_hz', None)),
                            'poles': self._jsonify(getattr(specs, 'poles', None)),
                            'slot_method': self._jsonify(getattr(specs, 'slot_method', None)),
                        },
                        'geometry': {
                            # Stator module uses meters
                            'D': self._jsonify(getattr(geom, 'D', None)),
                            'D_ext': self._jsonify(getattr(geom, 'D_ext', None)),
                            'L': self._jsonify(getattr(geom, 'L', None)),
                            'Ls': self._jsonify(getattr(geom, 'Ls', None)),
                            'validation_info': self._jsonify(getattr(geom, 'validation_info', None)),
                        },
                        'slots': {
                            'Ss': self._jsonify(getattr(slots, 'Ss', None)),
                        },
                    }
        except Exception:
            stator_summary = None

        slot_method_display = ""
        try:
            smw = getattr(self, 'slot_method_edit', None)
            if smw is not None:
                slot_method_display = str(smw.text() or '').strip()
        except Exception:
            slot_method_display = ""

        return {
            'format': 'induction_motor_project',
            'version': 1,
            'saved_at': datetime.now().isoformat(timespec='seconds'),
            'stator_design_done': bool(getattr(self, 'stator_design_done', False)),
            'rotor_results': self._jsonify(getattr(self, 'rotor_results', None)),
            'stator_to_rotor_inputs': self._jsonify(getattr(self, 'stator_to_rotor_inputs', None)),
            'stator_summary': stator_summary,
            'slot_method_ui': str(getattr(self, '_slot_method_ui', '') or ''),
            'slot_method_display': slot_method_display,
            'stator_2d_params': asdict(self._last_stator_2d_params) if getattr(self, '_last_stator_2d_params', None) is not None else None,
            'rotor_2d_params': asdict(self._last_rotor_2d_params) if getattr(self, '_last_rotor_2d_params', None) is not None else None,
            'tabs': self._tabs_to_dict(),
            'ui': self._collect_ui_state(),
        }

    def _save_project_to_path(self, path: str) -> bool:
        if not path:
            return False
        project = self._build_project_snapshot()
        try:
            with open(path, 'w', encoding='utf-8') as f:
                json.dump(project, f, ensure_ascii=False, indent=2)
            try:
                self.status_bar.showMessage(f"Saved: {path}")
            except Exception:
                pass
            return True
        except Exception as e:
            QMessageBox.critical(self, "Save Error", f"Failed to save project:\n{e}")
            return False

    def _reset_to_defaults(self):
        # Clear computed objects/state.
        self.designer = None
        self.results = None
        self.stator_to_rotor_inputs = None
        self.rotor_results = None
        self.stator_design_done = False
        self._export_ready = False
        self._loaded_stator_summary = None

        # Clear cached drawings.
        self._last_stator_2d_params = None
        self._last_rotor_2d_params = None
        try:
            if hasattr(self, 'stator_2d_widget') and self.stator_2d_widget is not None:
                self.stator_2d_widget.clear_params()
        except Exception:
            pass
        try:
            if hasattr(self, 'rotor_2d_widget') and self.rotor_2d_widget is not None:
                self.rotor_2d_widget.clear_params()
        except Exception:
            pass
        try:
            if hasattr(self, 'assembly_2d_widget') and self.assembly_2d_widget is not None:
                self.assembly_2d_widget.clear_params()
        except Exception:
            pass

        # Restore initial UI + tabs.
        if isinstance(self._initial_ui_state, dict):
            self._restore_ui_state(self._initial_ui_state)
        if isinstance(self._initial_tabs_state, dict):
            self._tabs_from_dict(self._initial_tabs_state)

        # Re-apply gating.
        self.set_rotor_ui_enabled(False)
        self.update_2d_view_button_visibility()
        self.update_export_button_visibility()

    def new_project(self) -> bool:
        """Start a new in-memory project (no file dialog). Use Save As to choose a path later."""
        self._reset_to_defaults()
        self._current_project_path = None
        self._project_dirty = False
        self.update_project_button_visibility()
        self._update_window_title()

        try:
            self.status_bar.showMessage("New project started (use File → Save As to create a file)")
        except Exception:
            pass

        if hasattr(self, '_stacked'):
            self._stacked.setCurrentWidget(self._page_main)
        return True

    def save_project(self) -> bool:
        """Save to the current project file.

        This only works after the user has loaded a project or used Save As.
        """
        path = getattr(self, '_current_project_path', None)
        if not path:
            QMessageBox.information(
                self,
                "Save",
                "No project file is selected yet.\n\nUse File → Save As to choose where to save this project.",
            )
            return False
        ok = self._save_project_to_path(str(path))
        if ok:
            self._project_dirty = False
            self.update_project_button_visibility()
        return ok

    def save_project_as(self) -> bool:
        """Save to a new file and update the current project path."""
        path, _ = QFileDialog.getSaveFileName(
            self,
            "Save Project As",
            str(getattr(self, '_current_project_path', '') or ''),
            "Motor project (*.motorproj.json *.json);;All files (*.*)"
        )
        if not path:
            return False
        ok = self._save_project_to_path(path)
        if ok:
            self._current_project_path = path
            self._project_dirty = False
            self.update_project_button_visibility()
            self._update_window_title()
        return ok

    def load_project_from_path(self, path: str) -> bool:
        """Load a saved project snapshot from a path (used by Welcome + File menu)."""
        if not path:
            return False
        try:
            with open(path, 'r', encoding='utf-8') as f:
                project = json.load(f)
        except Exception as e:
            QMessageBox.critical(self, "Load Error", f"Failed to read project file:\n{e}")
            return False

        if not isinstance(project, dict) or project.get('format') != 'induction_motor_project':
            QMessageBox.warning(self, "Invalid File", "This file is not a valid motor project.")
            return False

        # Update current project path
        self._current_project_path = path
        self._project_dirty = False
        self.update_project_button_visibility()
        self._update_window_title()

        # Reset computed objects; project restore is display-oriented.
        self.designer = None
        self.results = None

        # Restore UI inputs first (so dropdown-dependent UI is consistent).
        self._restore_ui_state(project.get('ui', {}))

        # Restore tab content.
        self._tabs_from_dict(project.get('tabs', {}))

        # Restore core state.
        self.stator_design_done = bool(project.get('stator_design_done', False))
        self.rotor_results = project.get('rotor_results', None)
        self.stator_to_rotor_inputs = project.get('stator_to_rotor_inputs', None)

        # Restore computed stator summary (used for PDF export in loaded-project mode)
        self._loaded_stator_summary = project.get('stator_summary', None)
        self._loaded_slot_method_display = str(project.get('slot_method_display', '') or '')
        try:
            sm = str(project.get('slot_method_ui', '') or '').strip()
            if sm:
                self._slot_method_ui = sm
        except Exception:
            pass

        # Restore cached 2D params
        self._last_stator_2d_params = None
        self._last_rotor_2d_params = None

        s2d = project.get('stator_2d_params', None)
        if isinstance(s2d, dict):
            try:
                self._last_stator_2d_params = Stator2DParams(**s2d)
            except Exception:
                self._last_stator_2d_params = None
        else:
            if hasattr(self, 'stator_2d_widget') and self.stator_2d_widget is not None:
                try:
                    self.stator_2d_widget.clear_params()
                except Exception:
                    pass

        r2d = project.get('rotor_2d_params', None)
        if isinstance(r2d, dict):
            try:
                self._last_rotor_2d_params = Rotor2DParams(**r2d)
            except Exception:
                self._last_rotor_2d_params = None

        # Apply gating
        self.set_rotor_ui_enabled(bool(self.stator_design_done))
        self.update_2d_view_button_visibility()

        # Ensure efficiency tab can appear if the project contains it.
        try:
            eff_text = ''
            if hasattr(self, 'tab_efficiency') and self.tab_efficiency is not None:
                eff_text = (self.tab_efficiency.toPlainText() or '')
            if eff_text.strip():
                self._set_efficiency_tab_visible(True)
        except Exception:
            pass

        self.update_export_button_visibility()
        self.update_project_button_visibility()

        # Rebuild rotor Nr options if possible.
        try:
            self.populate_rotor_nr_options()
        except Exception:
            pass

        # Update 2D widgets immediately.
        try:
            self._update_2d_view_from_current_design()
        except Exception:
            pass
        try:
            self._update_2d_view_from_current_rotor()
        except Exception:
            if getattr(self, '_last_rotor_2d_params', None) is not None and hasattr(self, 'rotor_2d_widget'):
                try:
                    self.rotor_2d_widget.set_params(self._last_rotor_2d_params)
                    self._set_rotor_2d_visible(True)
                except Exception:
                    pass
            self._update_2d_view_assembly()

        try:
            self.status_bar.showMessage(f"Project loaded: {os.path.basename(path)}")
        except Exception:
            pass

        # Navigate to main page
        if hasattr(self, '_stacked'):
            self._stacked.setCurrentWidget(self._page_main)

        QMessageBox.information(self, "Loaded", f"Project loaded:\n{path}")
        return True

    def _jsonify(self, obj: Any) -> Any:
        """Convert common Python/Qt/numpy-like values to JSON-safe types."""
        if obj is None:
            return None
        if isinstance(obj, (str, int, float, bool)):
            return obj
        if isinstance(obj, dict):
            out: dict[str, Any] = {}
            for k, v in obj.items():
                out[str(k)] = self._jsonify(v)
            return out
        if isinstance(obj, (list, tuple)):
            return [self._jsonify(v) for v in obj]
        # Best-effort conversion for numpy scalars/etc.
        for attr in ('item',):
            try:
                fn = getattr(obj, attr, None)
                if callable(fn):
                    return self._jsonify(fn())
            except Exception:
                pass
        return str(obj)

    def _collect_ui_state(self) -> dict[str, Any]:
        """Snapshot user inputs so a loaded project can be resumed."""
        state: dict[str, Any] = {}
        for name, widget in (getattr(self, '__dict__', {}) or {}).items():
            try:
                if isinstance(widget, QLineEdit):
                    state[name] = {'type': 'QLineEdit', 'text': widget.text()}
                elif isinstance(widget, QComboBox):
                    state[name] = {
                        'type': 'QComboBox',
                        'currentText': widget.currentText(),
                        'currentData': self._jsonify(widget.currentData()),
                    }
                elif isinstance(widget, QSpinBox):
                    state[name] = {'type': 'QSpinBox', 'value': int(widget.value())}
                elif isinstance(widget, QSlider):
                    state[name] = {'type': 'QSlider', 'value': int(widget.value())}
                elif isinstance(widget, QCheckBox):
                    state[name] = {'type': 'QCheckBox', 'checked': bool(widget.isChecked())}
                elif isinstance(widget, QRadioButton):
                    state[name] = {'type': 'QRadioButton', 'checked': bool(widget.isChecked())}
            except Exception:
                continue
        return state

    def _restore_ui_state(self, state: dict[str, Any]):
        """Restore previously saved input widget states."""
        if not isinstance(state, dict):
            return

        # Dependent inputs: coil pitch display depends on winding type.
        winding_payload = state.get('winding_type_combo') if isinstance(state.get('winding_type_combo'), dict) else None

        for name, payload in state.items():
            if not isinstance(payload, dict):
                continue
            # Handle this after the loop (dependency ordering).
            if name in ('winding_type_combo',):
                continue
            widget = getattr(self, name, None)
            if widget is None:
                continue

            try:
                widget.blockSignals(True)
                wtype = payload.get('type')
                if wtype == 'QLineEdit' and isinstance(widget, QLineEdit):
                    widget.setText(str(payload.get('text', '')))
                elif wtype == 'QSpinBox' and isinstance(widget, QSpinBox):
                    widget.setValue(int(payload.get('value', widget.value())))
                elif wtype == 'QSlider' and isinstance(widget, QSlider):
                    widget.setValue(int(payload.get('value', widget.value())))
                elif wtype == 'QCheckBox' and isinstance(widget, QCheckBox):
                    widget.setChecked(bool(payload.get('checked', widget.isChecked())))
                elif wtype == 'QRadioButton' and isinstance(widget, QRadioButton):
                    widget.setChecked(bool(payload.get('checked', widget.isChecked())))
                elif wtype == 'QComboBox' and isinstance(widget, QComboBox):
                    want_data = payload.get('currentData', None)
                    want_text = str(payload.get('currentText', '') or '')

                    idx = -1
                    if want_data is not None:
                        try:
                            idx = widget.findData(want_data)
                        except Exception:
                            idx = -1
                    if idx < 0 and want_text:
                        try:
                            idx = widget.findText(want_text)
                        except Exception:
                            idx = -1
                    if idx >= 0:
                        widget.setCurrentIndex(idx)
            except Exception:
                pass
            finally:
                try:
                    widget.blockSignals(False)
                except Exception:
                    pass

        # Restore winding type, then rebuild dependent coil pitch display.
        try:
            wt_widget = getattr(self, 'winding_type_combo', None)
            if wt_widget is not None and isinstance(wt_widget, QComboBox) and isinstance(winding_payload, dict):
                wt_widget.blockSignals(True)
                want_data = winding_payload.get('currentData', None)
                want_text = str(winding_payload.get('currentText', '') or '')
                idx = -1
                if want_data is not None:
                    try:
                        idx = wt_widget.findData(want_data)
                    except Exception:
                        idx = -1
                if idx < 0 and want_text:
                    try:
                        idx = wt_widget.findText(want_text)
                    except Exception:
                        idx = -1
                if idx >= 0:
                    wt_widget.setCurrentIndex(idx)
        except Exception:
            pass
        finally:
            try:
                wt_widget.blockSignals(False)
            except Exception:
                pass

        # Rebuild dependent options regardless of signal state.
        try:
            if hasattr(self, 'update_coil_pitch_options'):
                self.update_coil_pitch_options()
        except Exception:
            pass

    def _tabs_to_dict(self) -> dict[str, str]:
        tabs: dict[str, str] = {}
        mapping = {
            'empirical': getattr(self, 'tab_empirical', None),
            'electrical': getattr(self, 'tab_electrical', None),
            'dimensions': getattr(self, 'tab_dimensions', None),
            'winding': getattr(self, 'tab_winding', None),
            'slots': getattr(self, 'tab_slots', None),
            'outer': getattr(self, 'tab_outer', None),
            'losses': getattr(self, 'tab_losses', None),
            'rotor': getattr(self, 'tab_rotor', None),
            'efficiency': getattr(self, 'tab_efficiency', None),
        }
        for key, tab in mapping.items():
            if tab is None:
                continue
            try:
                tabs[key] = tab.toPlainText() or ''
            except Exception:
                tabs[key] = ''
        return tabs

    def _tabs_from_dict(self, tabs: dict[str, Any]):
        if not isinstance(tabs, dict):
            return
        mapping = {
            'empirical': getattr(self, 'tab_empirical', None),
            'electrical': getattr(self, 'tab_electrical', None),
            'dimensions': getattr(self, 'tab_dimensions', None),
            'winding': getattr(self, 'tab_winding', None),
            'slots': getattr(self, 'tab_slots', None),
            'outer': getattr(self, 'tab_outer', None),
            'losses': getattr(self, 'tab_losses', None),
            'rotor': getattr(self, 'tab_rotor', None),
            'efficiency': getattr(self, 'tab_efficiency', None),
        }
        for key, tab in mapping.items():
            if tab is None:
                continue
            try:
                tab.blockSignals(True)
                tab.setPlainText(str(tabs.get(key, '') or ''))
            except Exception:
                pass
            finally:
                try:
                    tab.blockSignals(False)
                except Exception:
                    pass

    def _save_project_legacy_dialog(self):
        """Deprecated legacy entry-point (kept to avoid accidental overrides)."""
        return self.save_project_as()

    def load_project(self):
        """Load a previously saved project snapshot."""
        path, _ = QFileDialog.getOpenFileName(
            self,
            "Load Project",
            "",
            "Motor project (*.motorproj.json *.json);;All files (*.*)"
        )
        if not path:
            return
        return self.load_project_from_path(path)

    def _set_efficiency_tab_visible(self, visible: bool):
        """Add/remove the Efficiency analysis tab in the detailed tabs widget."""
        if not hasattr(self, 'tab_widget') or not hasattr(self, 'tab_efficiency'):
            return
        idx = self.tab_widget.indexOf(self.tab_efficiency)
        if visible:
            if idx < 0:
                self.tab_widget.addTab(self.tab_efficiency, "Efficiency analysis")
        else:
            if idx >= 0:
                self.tab_widget.removeTab(idx)

    def _interp_linear(self, x: list, y: list, xq: float) -> float:
        """Simple 1D linear interpolation with end clamping."""
        if not x or not y or len(x) != len(y):
            raise ValueError("Invalid interpolation data")
        x_vals = [float(v) for v in x]
        y_vals = [float(v) for v in y]
        xq_f = float(xq)
        if xq_f <= x_vals[0]:
            return y_vals[0]
        if xq_f >= x_vals[-1]:
            return y_vals[-1]
        for i in range(len(x_vals) - 1):
            x0, x1 = x_vals[i], x_vals[i + 1]
            if x0 <= xq_f <= x1:
                y0, y1 = y_vals[i], y_vals[i + 1]
                if x1 == x0:
                    return y0
                t = (xq_f - x0) / (x1 - x0)
                return y0 + t * (y1 - y0)
        return y_vals[-1]

    def _get_H_from_steel_BH(self, steel_grade: str, B_tesla: float) -> tuple[str, float]:
        """Return (grade_used, H_A_per_m) using AbaquesTables.STEEL_BH_CURVES."""
        grade = (steel_grade or "").strip()
        curves = getattr(AbaquesTables, 'STEEL_BH_CURVES', {}) or {}
        curve = curves.get(grade)
        grade_used = grade
        if not curve:
            # Fallback to a common default grade
            grade_used = 'M400-50A' if 'M400-50A' in curves else (next(iter(curves.keys()), grade) or grade)
            curve = curves.get(grade_used, None)
        if not curve:
            raise ValueError("No steel B-H curve data available")
        B_list = curve.get('B', [])
        H_list = curve.get('H', [])
        H_val = self._interp_linear(B_list, H_list, float(B_tesla))
        return grade_used, float(H_val)

    def _get_carter_coeff(self, table_kind: str, ratio: float) -> float:
        """Interpolate Carter coefficient from AbaquesTables.

        table_kind:
          - 'open' -> AbaquesTables.CARTER_COEFF_OPEN_SLOT (x: opening_airgap)
          - 'semi_open' -> AbaquesTables.CARTER_COEFF_SEMI_OPEN_SLOT (x: slot_opening_gap)

        ratio is the unitless opening/gap ratio (e.g., bs/Lg).
        """
        tables = getattr(AbaquesTables, '__dict__', {}) or {}
        if table_kind == 'open':
            table = tables.get('CARTER_COEFF_OPEN_SLOT', {}) or {}
            x = table.get('opening_airgap', [])
            y = table.get('carter_coeff', [])
        else:
            table = tables.get('CARTER_COEFF_SEMI_OPEN_SLOT', {}) or {}
            x = table.get('slot_opening_gap', [])
            y = table.get('carter_coeff', [])

        if not x or not y:
            raise ValueError("Carter coefficient tables are missing")
        return float(self._interp_linear(x, y, float(ratio)))

    def show_efficiency_analysis(self):
        """Compute and display ampere-turns for stator/rotor cores and airgap."""
        if not hasattr(self, 'tab_efficiency'):
            return
        if not (self.designer and getattr(self, 'rotor_results', None)):
            # When loading a project, we may have saved text without a live designer.
            # Do not clear that text; just skip auto-compute.
            return

        # Ensure geometry validation info is up-to-date (D_mean is computed there)
        try:
            self.designer.design_conductor_and_slots()
            self.designer.calculate_outer_diameter()
        except Exception:
            pass

        specs = getattr(self.designer, 'specs', None)
        geom = getattr(self.designer, 'geometry', None)
        val_info = geom.validation_info if (geom is not None and hasattr(geom, 'validation_info') and geom.validation_info) else {}
        rotor = dict(getattr(self, 'rotor_results', {}) or {})

        poles = int(getattr(specs, 'poles', 0) or 0)
        if poles <= 0:
            self.append_to_tab(self.tab_efficiency, self.make_header("EFFICIENCY ANALYSIS") + "\nInvalid pole count.\n")
            return

        # B_yoke used for both stator & rotor core (same B_yoke)
        b_yoke = None
        try:
            b_yoke = float(val_info.get('B_yoke_target', None)) if val_info.get('B_yoke_target', None) is not None else None
        except Exception:
            b_yoke = None
        if b_yoke is None:
            targets = getattr(self.designer, '_forward_design_targets', None) or {}
            try:
                b_yoke = float(targets.get('B_yoke_target', None)) if targets.get('B_yoke_target', None) is not None else None
            except Exception:
                b_yoke = None
        if b_yoke is None:
            # Last resort fallback
            b_yoke = 1.3

        steel_grade = (getattr(specs, 'steel_grade', None) or "").strip()
        grade_used, H_A_per_m = self._get_H_from_steel_BH(steel_grade, float(b_yoke))

        # Stator mean diameter: reuse the value computed for core weight (must be in meters)
        D_mean_m = val_info.get('Wc_D_mean_m', None)
        try:
            D_mean_m = float(D_mean_m) if D_mean_m is not None else None
        except Exception:
            D_mean_m = None
        if D_mean_m is None:
            # Fallback: approximate with bore diameter
            try:
                D_mean_m = float(getattr(geom, 'D', 0.0) or 0.0)
            except Exception:
                D_mean_m = 0.0

        # Rotor mean diameter from rotor results (inputs are in mm)
        Dr_mm = rotor.get('Dr_mm', None)
        d_rb_mm = rotor.get('d_rb_mm', None)
        Wry_mm = rotor.get('Wry_mm', None)
        try:
            Dr_mm_f = float(Dr_mm) if Dr_mm is not None else None
            d_rb_mm_f = float(d_rb_mm) if d_rb_mm is not None else None
            Wry_mm_f = float(Wry_mm) if Wry_mm is not None else None
        except Exception:
            Dr_mm_f = d_rb_mm_f = Wry_mm_f = None

        Dr_mean_m = None
        if Dr_mm_f is not None and d_rb_mm_f is not None and Wry_mm_f is not None:
            Dr_mean_mm = Dr_mm_f - 2.0 * d_rb_mm_f - Wry_mm_f
            Dr_mean_m = Dr_mean_mm / 1000.0

        # ------------------------------------------------------------
        # Airgap ampere-turns (Carter correction)
        # ------------------------------------------------------------
        # Base airgap length from rotor (computed consistently with D/L in rotor design)
        Lg_mm = rotor.get('Lg_mm', None)
        try:
            Lg_mm_f = float(Lg_mm) if Lg_mm is not None else None
        except Exception:
            Lg_mm_f = None

        # Stator slot pitch and opening (bore)
        Ss = int(getattr(self.designer.slots, 'Ss', 0) or 0) if getattr(self.designer, 'slots', None) is not None else int(val_info.get('phi_max_Ss', 0) or 0)
        D_bore_m = None
        try:
            D_bore_m = float(getattr(geom, 'D', 0.0) or 0.0)
        except Exception:
            D_bore_m = 0.0
        D_bore_mm = float(D_bore_m) * 1000.0

        slot_method = (getattr(self.designer.specs, 'slot_method', None) or 'semi_open_slot')

        # ------------------------------------------------------------
        # Stator tooth ampere-turns (ATST)
        # ------------------------------------------------------------
        # Uses phi_max (maximum tooth flux) with a varying-tooth-flux profile:
        #   flux(z) = flux_max * (1 - z / (ht + 0.7*dcs))
        # where:
        #   - ht = hs0+hs1+hs2 for semi-open slot; ht = hs for open slot
        #   - dcs is the stator core depth (meters)
        # For each segment [z0,z1]:
        #   flux_mean = (flux(z0) + flux(z1))/2
        #   B_mean = flux_mean / (Li * w_mean)
        #   AT_segment = H(B_mean) * h_segment
        phi_max_tooth_Wb = None
        try:
            if slot_method == 'open_slot':
                phi_max_tooth_Wb = val_info.get('open_slot_phi_max_used_Wb', val_info.get('phi_max', None))
            else:
                phi_max_tooth_Wb = val_info.get('phi_max', None)
            phi_max_tooth_Wb = float(phi_max_tooth_Wb) if phi_max_tooth_Wb is not None else None
        except Exception:
            phi_max_tooth_Wb = None

        Li_tooth_m = None
        try:
            Li_tooth_m = float(val_info.get('Li', getattr(geom, 'Li', None)))
        except Exception:
            Li_tooth_m = None

        dcs_m = None
        try:
            dcs_m = val_info.get('Wc_dcs_m_used', None)
            dcs_m = float(dcs_m) if dcs_m is not None else None
        except Exception:
            dcs_m = None
        if dcs_m is None:
            dcs_m = 0.0

        def flux_at_z(z_m: float, flux_max_wb: float, ht_m: float, dcs_m_local: float) -> float:
            denom = float(ht_m) + 0.7 * float(dcs_m_local)
            if denom <= 0:
                return float(flux_max_wb)
            return float(flux_max_wb) * (1.0 - (float(z_m) / denom))

        ATST = None
        tooth_details = {}
        if phi_max_tooth_Wb is not None and phi_max_tooth_Wb > 0 and Li_tooth_m is not None and Li_tooth_m > 0:
            if slot_method == 'open_slot':
                wst1_mm = val_info.get('Wt_wst1_mm_used', val_info.get('open_slot_wst1_mm', None))
                wst2_mm = val_info.get('Wt_wst2_mm_used', val_info.get('open_slot_wst2_mm', None))
                hs_mm = val_info.get('Wt_hs_mm_used', val_info.get('open_slot_hs_mm', None))
                try:
                    wst1_mm = float(wst1_mm) if wst1_mm is not None else None
                    wst2_mm = float(wst2_mm) if wst2_mm is not None else None
                    hs_mm = float(hs_mm) if hs_mm is not None else None
                except Exception:
                    wst1_mm = wst2_mm = hs_mm = None

                if wst1_mm and wst2_mm and hs_mm and wst1_mm > 0 and wst2_mm > 0 and hs_mm > 0:
                    wst1_m = wst1_mm / 1000.0
                    wst2_m = wst2_mm / 1000.0
                    hs_m = hs_mm / 1000.0

                    ht_m = float(hs_m)
                    denom_m = float(ht_m) + 0.7 * float(dcs_m)
                    phi0 = flux_at_z(0.0, float(phi_max_tooth_Wb), ht_m, float(dcs_m))
                    phi_hs = flux_at_z(float(hs_m), float(phi_max_tooth_Wb), ht_m, float(dcs_m))
                    phi_mean = 0.5 * (float(phi0) + float(phi_hs))

                    w_mean = 0.5 * (float(wst1_m) + float(wst2_m))
                    B_mean = float(phi_mean) / (float(Li_tooth_m) * float(w_mean)) if (Li_tooth_m > 0 and w_mean > 0) else 0.0
                    _, H_mean = self._get_H_from_steel_BH(steel_grade, float(B_mean))
                    ATST = float(H_mean) * float(hs_m)

                    tooth_details = {
                        'kind': 'open',
                        'wst1_mm': wst1_mm,
                        'wst2_mm': wst2_mm,
                        'hs_mm': hs_mm,
                        'dcs_m': float(dcs_m),
                        'ht_m': ht_m,
                        'denom_m': denom_m,
                        'phi_z0_Wb': phi0,
                        'phi_z1_Wb': phi_hs,
                        'phi_mean_Wb': phi_mean,
                        'w_mean_m': w_mean,
                        'B_mean_T': B_mean,
                        'H_mean_Apm': H_mean,
                        'AT': float(ATST),
                    }
            else:
                h0_mm = val_info.get('Wt_h0_mm_used', val_info.get('slot_dim_hs0_mm', val_info.get('hs0_mm', None)))
                h1_mm = val_info.get('Wt_h1_mm_used', val_info.get('slot_dim_hs1_mm', val_info.get('hs1_mm', None)))
                hs2_mm = val_info.get('Wt_hs2_mm_used', val_info.get('semi_open_new_hs2_mm', val_info.get('hs2_mm', None)))
                wst0_mm = val_info.get('Wt_wst0_mm_used', val_info.get('wst0_mm', None))
                # Tooth body width (as requested: wst)
                wst1_mm = val_info.get('Wt_wst1_mm_used', val_info.get('semi_open_new_wst1_mm', val_info.get('wst_mm', None)))
                try:
                    h0_mm = float(h0_mm) if h0_mm is not None else None
                    h1_mm = float(h1_mm) if h1_mm is not None else None
                    hs2_mm = float(hs2_mm) if hs2_mm is not None else None
                    wst0_mm = float(wst0_mm) if wst0_mm is not None else None
                    wst1_mm = float(wst1_mm) if wst1_mm is not None else None
                except Exception:
                    h0_mm = h1_mm = hs2_mm = wst0_mm = wst1_mm = None

                if (
                    h0_mm is not None and h1_mm is not None and hs2_mm is not None
                    and wst0_mm is not None and wst1_mm is not None
                    and h0_mm > 0 and h1_mm > 0 and hs2_mm > 0
                    and wst0_mm > 0 and wst1_mm > 0
                ):
                    w0_m = wst0_mm / 1000.0
                    w2_m = wst1_mm / 1000.0
                    h0_m = h0_mm / 1000.0
                    h1_m = h1_mm / 1000.0
                    h2_m = hs2_mm / 1000.0

                    ht_m = float(h0_m + h1_m + h2_m)
                    denom_m = float(ht_m) + 0.7 * float(dcs_m)

                    # Segment boundaries in meters
                    z0 = 0.0
                    z1 = float(h0_m)
                    z2 = float(h0_m + h1_m)
                    z3 = float(h0_m + h1_m + h2_m)

                    # Flux at boundaries
                    phi0 = flux_at_z(z0, float(phi_max_tooth_Wb), ht_m, float(dcs_m))
                    phi1 = flux_at_z(z1, float(phi_max_tooth_Wb), ht_m, float(dcs_m))
                    phi2 = flux_at_z(z2, float(phi_max_tooth_Wb), ht_m, float(dcs_m))
                    phi3 = flux_at_z(z3, float(phi_max_tooth_Wb), ht_m, float(dcs_m))

                    # Segment mean flux
                    phi_mean_0 = 0.5 * (float(phi0) + float(phi1))
                    phi_mean_1 = 0.5 * (float(phi1) + float(phi2))
                    phi_mean_2 = 0.5 * (float(phi2) + float(phi3))

                    # Segment mean widths (as requested)
                    #  - seg0: uniform -> wst0
                    #  - seg1: non-uniform -> mean(wst0, wst)
                    #  - seg2: uniform -> wst
                    w_mean_0 = float(w0_m)
                    w_mean_1 = 0.5 * (float(w0_m) + float(w2_m))
                    w_mean_2 = float(w2_m)

                    # Mean flux densities
                    B0 = float(phi_mean_0) / (float(Li_tooth_m) * float(w_mean_0)) if (Li_tooth_m > 0 and w_mean_0 > 0) else 0.0
                    B1 = float(phi_mean_1) / (float(Li_tooth_m) * float(w_mean_1)) if (Li_tooth_m > 0 and w_mean_1 > 0) else 0.0
                    B2 = float(phi_mean_2) / (float(Li_tooth_m) * float(w_mean_2)) if (Li_tooth_m > 0 and w_mean_2 > 0) else 0.0

                    _, H0 = self._get_H_from_steel_BH(steel_grade, float(B0))
                    _, H1 = self._get_H_from_steel_BH(steel_grade, float(B1))
                    _, H2 = self._get_H_from_steel_BH(steel_grade, float(B2))

                    AT0 = float(H0) * float(h0_m)
                    AT1 = float(H1) * float(h1_m)
                    AT2 = float(H2) * float(h2_m)

                    ATST = float(AT0 + AT1 + AT2)
                    tooth_details = {
                        'kind': 'semi_open',
                        'hs0_mm': h0_mm,
                        'hs1_mm': h1_mm,
                        'hs2_mm': hs2_mm,
                        'wst0_mm': wst0_mm,
                        'wst1_mm': wst1_mm,
                        'dcs_m': float(dcs_m),
                        'ht_m': ht_m,
                        'denom_m': denom_m,
                        'z0_m': z0,
                        'z1_m': z1,
                        'z2_m': z2,
                        'z3_m': z3,
                        'phi0_Wb': phi0,
                        'phi1_Wb': phi1,
                        'phi2_Wb': phi2,
                        'phi3_Wb': phi3,
                        'phi_mean_0_Wb': phi_mean_0,
                        'phi_mean_1_Wb': phi_mean_1,
                        'phi_mean_2_Wb': phi_mean_2,
                        'w_mean_0_m': w_mean_0,
                        'w_mean_1_m': w_mean_1,
                        'w_mean_2_m': w_mean_2,
                        'B0_T': B0,
                        'B1_T': B1,
                        'B2_T': B2,
                        'H0_Apm': H0,
                        'H1_Apm': H1,
                        'H2_Apm': H2,
                        'AT0': AT0,
                        'AT1': AT1,
                        'AT2': AT2,
                    }

        # ------------------------------------------------------------
        # Rotor tooth ampere-turns (ATRT)
        # ------------------------------------------------------------
        # Uses rotor max tooth flux with a varying-tooth-flux profile:
        #   flux(z) = flux_max * (1 - z / (ht + 0.7*wry))
        # where ht = h_r0 + d_rb (meters) and wry is rotor yoke depth (meters).
        # Segments:
        #   seg0: z=0 -> h_r0, width = w_tooth_bore
        #   seg1: z=h_r0 -> h_r0+r1, width mean = mean(w_tooth_bore, Wrt)
        #   seg2: z=h_r0+r1 -> h_r0+d_rb, width = Wrt, depth = (d_rb - r1)

        ATRT = None
        rotor_tooth_details: Dict[str, float] = {}

        # Inputs from rotor design
        try:
            fluxrt_max_Wb = rotor.get('fluxrt_max_Wb', None)
            fluxrt_max_Wb = float(fluxrt_max_Wb) if fluxrt_max_Wb is not None else None
        except Exception:
            fluxrt_max_Wb = None

        try:
            Nr_rotor_i = int(rotor.get('Nr', 0) or 0)
        except Exception:
            Nr_rotor_i = 0

        try:
            b_r0_mm_f2 = float(rotor.get('b_r0_mm', None)) if rotor.get('b_r0_mm', None) is not None else None
        except Exception:
            b_r0_mm_f2 = None

        try:
            h_r0_mm_f = float(rotor.get('h_r0_mm', None)) if rotor.get('h_r0_mm', None) is not None else None
        except Exception:
            h_r0_mm_f = None

        try:
            r1_mm_f = float(rotor.get('r1_mm', None)) if rotor.get('r1_mm', None) is not None else None
        except Exception:
            r1_mm_f = None

        # d_rb_mm_f is already parsed above from rotor results
        # Rotor tooth width from rotor design: Wrt
        try:
            Wrt_mm_f = float(rotor.get('Wrt_mm', None)) if rotor.get('Wrt_mm', None) is not None else None
        except Exception:
            Wrt_mm_f = None

        def flux_at_z_rotor(z_m: float, flux_max_wb: float, ht_m: float, wry_m: float) -> float:
            denom = float(ht_m) + 0.7 * float(wry_m)
            if denom <= 0:
                return float(flux_max_wb)
            z_use = float(z_m)
            # Clamp z within [0, ht] to avoid negative flux due to bad inputs
            if z_use < 0.0:
                z_use = 0.0
            # Clamp using geometric ht (not denom) so that z never exceeds the physical depth
            ht_geom = float(ht_m)
            if ht_geom > 0.0 and z_use > ht_geom:
                z_use = ht_geom
            return float(flux_max_wb) * (1.0 - (z_use / denom))

        if (
            fluxrt_max_Wb is not None and fluxrt_max_Wb > 0
            and Li_tooth_m is not None and Li_tooth_m > 0
            and Dr_mm_f is not None and Dr_mm_f > 0
            and Nr_rotor_i > 0
            and b_r0_mm_f2 is not None and b_r0_mm_f2 > 0
            and h_r0_mm_f is not None and h_r0_mm_f > 0
            and d_rb_mm_f is not None and d_rb_mm_f > 0
            and r1_mm_f is not None and r1_mm_f >= 0
            and Wrt_mm_f is not None and Wrt_mm_f > 0
        ):
            pitch_rotor_mm = math.pi * float(Dr_mm_f) / float(Nr_rotor_i)
            w_tooth_bore_mm = float(pitch_rotor_mm) - float(b_r0_mm_f2)
            if w_tooth_bore_mm > 0:
                h_r0_m = float(h_r0_mm_f) / 1000.0
                r1_m = float(r1_mm_f) / 1000.0
                d_rb_m = float(d_rb_mm_f) / 1000.0
                ht_m_rt = float(h_r0_m + d_rb_m)

                # Rotor yoke depth (Wry)
                try:
                    wry_mm_f = float(rotor.get('Wry_mm', None)) if rotor.get('Wry_mm', None) is not None else None
                except Exception:
                    wry_mm_f = None
                wry_m = float(wry_mm_f) / 1000.0 if (wry_mm_f is not None and wry_mm_f > 0) else 0.0
                denom_m_rt = float(ht_m_rt) + 0.7 * float(wry_m)

                w_bore_m = float(w_tooth_bore_mm) / 1000.0
                wst_m = float(Wrt_mm_f) / 1000.0

                # z locations
                z0 = 0.0
                z1 = float(h_r0_m)
                z2 = float(h_r0_m + r1_m)
                z3 = float(h_r0_m + d_rb_m)

                # Flux at z locations
                phi0 = flux_at_z_rotor(z0, float(fluxrt_max_Wb), ht_m_rt, float(wry_m))
                phi1 = flux_at_z_rotor(z1, float(fluxrt_max_Wb), ht_m_rt, float(wry_m))
                phi2 = flux_at_z_rotor(z2, float(fluxrt_max_Wb), ht_m_rt, float(wry_m))
                phi3 = flux_at_z_rotor(z3, float(fluxrt_max_Wb), ht_m_rt, float(wry_m))

                # Segment depths
                seg0_h = float(h_r0_m)
                seg1_h = float(r1_m)
                seg2_h = float(max(d_rb_m - r1_m, 0.0))

                # Segment mean flux
                phi_mean_0 = 0.5 * (float(phi0) + float(phi1))
                phi_mean_1 = 0.5 * (float(phi1) + float(phi2))
                phi_mean_2 = 0.5 * (float(phi2) + float(phi3))

                # Segment mean widths
                w_mean_0 = float(w_bore_m)
                w_mean_1 = 0.5 * (float(w_bore_m) + float(wst_m))
                w_mean_2 = float(wst_m)

                # Mean flux densities
                B0 = float(phi_mean_0) / (float(Li_tooth_m) * float(w_mean_0)) if (Li_tooth_m > 0 and w_mean_0 > 0) else 0.0
                B1 = float(phi_mean_1) / (float(Li_tooth_m) * float(w_mean_1)) if (Li_tooth_m > 0 and w_mean_1 > 0) else 0.0
                B2 = float(phi_mean_2) / (float(Li_tooth_m) * float(w_mean_2)) if (Li_tooth_m > 0 and w_mean_2 > 0) else 0.0

                _, H0 = self._get_H_from_steel_BH(steel_grade, float(B0))
                _, H1 = self._get_H_from_steel_BH(steel_grade, float(B1))
                _, H2 = self._get_H_from_steel_BH(steel_grade, float(B2))

                AT0 = float(H0) * float(seg0_h)
                AT1 = float(H1) * float(seg1_h)
                AT2 = float(H2) * float(seg2_h)
                ATRT = float(AT0 + AT1 + AT2)

                rotor_tooth_details = {
                    'kind': 'rotor_3seg',
                    'fluxrt_max_Wb': float(fluxrt_max_Wb),
                    'Nr': float(Nr_rotor_i),
                    'Dr_mm': float(Dr_mm_f),
                    'pitch_rotor_mm': float(pitch_rotor_mm),
                    'b_r0_mm': float(b_r0_mm_f2),
                    'w_tooth_bore_mm': float(w_tooth_bore_mm),
                    'h_r0_mm': float(h_r0_mm_f),
                    'r1_mm': float(r1_mm_f),
                    'd_rb_mm': float(d_rb_mm_f),
                    'Wry_mm': float(wry_mm_f) if wry_mm_f is not None else 0.0,
                    'Wrt_mm': float(Wrt_mm_f),
                    'ht_m': float(ht_m_rt),
                    'denom_m': float(denom_m_rt),
                    'z0_m': float(z0),
                    'z1_m': float(z1),
                    'z2_m': float(z2),
                    'z3_m': float(z3),
                    'phi0_Wb': float(phi0),
                    'phi1_Wb': float(phi1),
                    'phi2_Wb': float(phi2),
                    'phi3_Wb': float(phi3),
                    'phi_mean_0_Wb': float(phi_mean_0),
                    'phi_mean_1_Wb': float(phi_mean_1),
                    'phi_mean_2_Wb': float(phi_mean_2),
                    'w_mean_0_m': float(w_mean_0),
                    'w_mean_1_m': float(w_mean_1),
                    'w_mean_2_m': float(w_mean_2),
                    'B0_T': float(B0),
                    'B1_T': float(B1),
                    'B2_T': float(B2),
                    'H0_Apm': float(H0),
                    'H1_Apm': float(H1),
                    'H2_Apm': float(H2),
                    'seg0_h_m': float(seg0_h),
                    'seg1_h_m': float(seg1_h),
                    'seg2_h_m': float(seg2_h),
                    'AT0': float(AT0),
                    'AT1': float(AT1),
                    'AT2': float(AT2),
                }
        bs_stator_mm = None
        pitch_stator_mm = None
        stator_ratio = None
        kgs = None
        carter_stator = None
        carter_table_used = None

        if Lg_mm_f is not None and Lg_mm_f > 0 and Ss > 0 and D_bore_mm > 0:
            # Prefer stored pitch if available, else compute at bore
            pitch_stator_mm = val_info.get('slot_pitch_D_mm', val_info.get('open_slot_slot_pitch_D_mm', None))
            try:
                pitch_stator_mm = float(pitch_stator_mm) if pitch_stator_mm is not None else (math.pi * D_bore_mm / float(Ss))
            except Exception:
                pitch_stator_mm = (math.pi * D_bore_mm / float(Ss))

            if slot_method == 'open_slot':
                # Open slot: use bs
                bs_stator_mm = val_info.get('open_slot_bs_mm', None)
                try:
                    bs_stator_mm = float(bs_stator_mm) if bs_stator_mm is not None else None
                except Exception:
                    bs_stator_mm = None
                if bs_stator_mm is not None:
                    stator_ratio = float(bs_stator_mm) / float(Lg_mm_f)
                    carter_table_used = 'open'
                    carter_stator = self._get_carter_coeff('open', stator_ratio)
            else:
                # Trapezoidal semi-open slot (backend: semi_open_slot): use bs0
                bs_stator_mm = val_info.get('bs0_mm', None)
                try:
                    bs_stator_mm = float(bs_stator_mm) if bs_stator_mm is not None else None
                except Exception:
                    bs_stator_mm = None
                if bs_stator_mm is not None:
                    stator_ratio = float(bs_stator_mm) / float(Lg_mm_f)
                    carter_table_used = 'semi_open'
                    carter_stator = self._get_carter_coeff('semi_open', stator_ratio)

            if pitch_stator_mm and bs_stator_mm and carter_stator is not None:
                denom = float(pitch_stator_mm) - float(carter_stator) * float(bs_stator_mm)
                if denom > 0:
                    kgs = float(pitch_stator_mm) / denom

        # Rotor Carter factor (semi-open table, using b_r0)
        b_r0_mm = rotor.get('b_r0_mm', None)
        try:
            b_r0_mm_f = float(b_r0_mm) if b_r0_mm is not None else None
        except Exception:
            b_r0_mm_f = None

        Nr_rotor = rotor.get('Nr', None)
        try:
            Nr_rotor_i = int(Nr_rotor) if Nr_rotor is not None else None
        except Exception:
            Nr_rotor_i = None

        kgr = None
        rotor_ratio = None
        carter_rotor = None
        pitch_rotor_mm = None
        if Lg_mm_f is not None and Lg_mm_f > 0 and Dr_mm_f is not None and Dr_mm_f > 0 and Nr_rotor_i and Nr_rotor_i > 0 and b_r0_mm_f is not None and b_r0_mm_f > 0:
            pitch_rotor_mm = math.pi * float(Dr_mm_f) / float(Nr_rotor_i)
            rotor_ratio = float(b_r0_mm_f) / float(Lg_mm_f)
            carter_rotor = self._get_carter_coeff('semi_open', rotor_ratio)
            denom = float(pitch_rotor_mm) - float(carter_rotor) * float(b_r0_mm_f)
            if denom > 0:
                kgr = float(pitch_rotor_mm) / denom

        # Effective airgap and airgap AT
        Lgd_m = None
        if Lg_mm_f is not None and kgs is not None and kgr is not None:
            Lgd_m = (float(Lg_mm_f) / 1000.0) * float(kgs) * float(kgr)

        # Airgap area per pole (use corrected active iron length if available)
        Ls_corr_m = val_info.get('Li', getattr(geom, 'Li', val_info.get('Ls', getattr(geom, 'Ls', getattr(geom, 'L', 0.0)))))
        try:
            Ls_corr_m = float(Ls_corr_m) if Ls_corr_m is not None else 0.0
        except Exception:
            Ls_corr_m = 0.0

        # Flux per pole (prefer corrected/recalculated where available)
        phi_pole_Wb = val_info.get('flux_per_pole_recalc', val_info.get('flux_per_pole_from_Bav', getattr(geom, 'flux_per_pole', 0.0)))
        try:
            phi_pole_Wb = float(phi_pole_Wb) if phi_pole_Wb is not None else 0.0
        except Exception:
            phi_pole_Wb = 0.0

        Agp_m2 = None
        Bg_T = None
        ATg = None
        if poles > 0 and D_bore_m and Ls_corr_m > 0:
            Agp_m2 = math.pi * float(D_bore_m) * float(Ls_corr_m) / float(poles)
            if Agp_m2 > 0 and phi_pole_Wb > 0:
                Bg_T = float(phi_pole_Wb) / float(Agp_m2)
        if Bg_T is not None and Lgd_m is not None:
            # 0.796e6 ≈ 1/μ0 in A/(T·m); factor 1.36 as specified.
            ATg = 0.796e6 * float(Bg_T) * 1.36 * float(Lgd_m)

        # Ampere-turns (P = number of poles)
        ATSC = (math.pi * float(D_mean_m) * float(H_A_per_m)) / (float(poles) * 3.0) if D_mean_m else 0.0
        ATRC = (math.pi * float(Dr_mean_m) * float(H_A_per_m)) / (float(poles) * 3.0) if Dr_mean_m else 0.0

        text = self.make_header("EFFICIENCY ANALYSIS")
        text += "AMPERE-TURNS (CORE + AIRGAP)\n"
        text += self.make_separator()
        text += "1) H from steel B-H curve (interpolated)\n"
        text += self.make_separator()
        text += f"    Steel grade selected: {steel_grade if steel_grade else '—'}\n"
        text += f"    Steel grade used for B-H: {grade_used if grade_used else '—'}\n"
        text += self.format_equation("B_yoke", f"{float(b_yoke):.4f} T")
        text += self.format_equation("H(B_yoke)", f"{float(H_A_per_m):.2f} A/m")

        text += "\n2) Stator core ampere-turns\n"
        text += self.make_separator()
        text += "    Formula: ATSC = pi \u00d7 D_mean \u00d7 H / (P \u00d7 3)\n\n"
        text += self.format_equation("P", f"{int(poles)}")
        text += self.format_equation("D_mean", f"{float(D_mean_m):.6f} m")
        text += self.format_equation("ATSC", f"{float(ATSC):.3f} A\u00b7turn")

        text += "\n2b) Stator tooth ampere-turns\n"
        text += self.make_separator()
        text += "    Flux profile inside tooth:\n"
        text += "      flux(z) = flux_max × (1 - z / (ht + 0.7×dcs))\n"
        text += "    For each segment [z0,z1]:\n"
        text += "      flux_mean = (flux(z0) + flux(z1)) / 2\n"
        text += "      B_mean = flux_mean / (Li × w_mean)\n"
        text += "      AT_segment = H(B_mean) × h_segment\n\n"

        if phi_max_tooth_Wb is None or Li_tooth_m is None:
            text += "    Tooth AT: — (missing phi_max and/or Li)\n"
        elif ATST is None or not tooth_details:
            text += "    Tooth AT: — (insufficient tooth geometry for selected slot type)\n"
            text += self.format_equation("phi_max(tooth)", f"{float(phi_max_tooth_Wb):.9f} Wb")
            text += self.format_equation("Li", f"{float(Li_tooth_m):.6f} m")
        else:
            text += self.format_equation("phi_max(tooth)", f"{float(phi_max_tooth_Wb):.9f} Wb")
            text += self.format_equation("Li", f"{float(Li_tooth_m):.6f} m")
            if tooth_details.get('kind') == 'semi_open':
                text += "\n    Slot type: Trapezoidal semi-open slot\n\n"
                # Geometry + profile parameters
                text += self.format_equation("dcs", f"{float(tooth_details.get('dcs_m', 0.0)):.6f} m")
                text += self.format_equation("ht", f"{float(tooth_details.get('ht_m', 0.0)):.6f} m")
                text += self.format_equation("ht+0.7·dcs", f"{float(tooth_details.get('denom_m', 0.0)):.6f} m")
                text += "\n"
                text += self.format_equation("hs0", f"{float(tooth_details['hs0_mm']):.3f} mm")
                text += self.format_equation("hs1", f"{float(tooth_details['hs1_mm']):.3f} mm")
                text += self.format_equation("hs2", f"{float(tooth_details['hs2_mm']):.3f} mm")
                text += self.format_equation("wst0", f"{float(tooth_details['wst0_mm']):.3f} mm")
                text += self.format_equation("wst", f"{float(tooth_details['wst1_mm']):.3f} mm")

                # Show exact flux(z) inputs used
                try:
                    denom_m = float(tooth_details.get('denom_m', 0.0) or 0.0)
                    z0_m = float(tooth_details.get('z0_m', 0.0) or 0.0)
                    z1_m = float(tooth_details.get('z1_m', 0.0) or 0.0)
                    z2_m = float(tooth_details.get('z2_m', 0.0) or 0.0)
                    z3_m = float(tooth_details.get('z3_m', 0.0) or 0.0)
                except Exception:
                    denom_m = z0_m = z1_m = z2_m = z3_m = 0.0

                text += "\n    flux(z) calculation details:\n\n"
                text += self.format_equation("z0", f"{z0_m:.6f} m")
                text += self.format_equation("z1", f"{z1_m:.6f} m")
                text += self.format_equation("z2", f"{z2_m:.6f} m")
                text += self.format_equation("z3", f"{z3_m:.6f} m")
                text += self.format_equation("denom = ht+0.7·dcs", f"{denom_m:.6f} m")
                text += "\n"
                if denom_m != 0.0:
                    text += f"    flux(z0) = flux_max*(1 - z0/denom) with z0/denom = {z0_m/denom_m:.6g}\n"
                    text += f"    flux(z1) = flux_max*(1 - z1/denom) with z1/denom = {z1_m/denom_m:.6g}\n"
                    text += f"    flux(z2) = flux_max*(1 - z2/denom) with z2/denom = {z2_m/denom_m:.6g}\n"
                    text += f"    flux(z3) = flux_max*(1 - z3/denom) with z3/denom = {z3_m/denom_m:.6g}\n"
                else:
                    text += "    denom is 0 -> ratios set to 0\n"

                # Flux at boundaries
                text += "\n    Flux at segment boundaries (from flux(z)):\n\n"
                text += self.format_equation("flux(z0=0)", f"{float(tooth_details.get('phi0_Wb', 0.0)):.9f} Wb")
                text += self.format_equation("flux(z1=hs0)", f"{float(tooth_details.get('phi1_Wb', 0.0)):.9f} Wb")
                text += self.format_equation("flux(z2=hs0+hs1)", f"{float(tooth_details.get('phi2_Wb', 0.0)):.9f} Wb")
                text += self.format_equation("flux(z3=ht)", f"{float(tooth_details.get('phi3_Wb', 0.0)):.9f} Wb")

                text += "\n"
                text += "    Segment 0 (0 → hs0):\n\n"
                text += self.format_equation("flux_mean(seg0)", f"{float(tooth_details.get('phi_mean_0_Wb', 0.0)):.9f} Wb")
                text += self.format_equation("w_mean(seg0)", f"{float(tooth_details.get('w_mean_0_m', 0.0)):.6f} m")
                text += self.format_equation("B_mean(seg0)", f"{float(tooth_details['B0_T']):.6f} T")
                text += self.format_equation("H(seg0)", f"{float(tooth_details['H0_Apm']):.2f} A/m")
                text += self.format_equation("AT(seg0)", f"{float(tooth_details['AT0']):.3f} A\u00b7turn")
                text += "\n"
                text += "    Segment 1 (hs0 → hs0+hs1):\n\n"
                text += self.format_equation("flux_mean(seg1)", f"{float(tooth_details.get('phi_mean_1_Wb', 0.0)):.9f} Wb")
                text += self.format_equation("w_mean(seg1)", f"{float(tooth_details.get('w_mean_1_m', 0.0)):.6f} m")
                text += self.format_equation("B_mean(seg1)", f"{float(tooth_details['B1_T']):.6f} T")
                text += self.format_equation("H(seg1)", f"{float(tooth_details['H1_Apm']):.2f} A/m")
                text += self.format_equation("AT(seg1)", f"{float(tooth_details['AT1']):.3f} A\u00b7turn")
                text += "\n"
                text += "    Segment 2 (hs0+hs1 → ht):\n\n"
                text += self.format_equation("flux_mean(seg2)", f"{float(tooth_details.get('phi_mean_2_Wb', 0.0)):.9f} Wb")
                text += self.format_equation("w_mean(seg2)", f"{float(tooth_details.get('w_mean_2_m', 0.0)):.6f} m")
                text += self.format_equation("B_mean(seg2)", f"{float(tooth_details['B2_T']):.6f} T")
                text += self.format_equation("H(seg2)", f"{float(tooth_details['H2_Apm']):.2f} A/m")
                text += self.format_equation("AT(seg2)", f"{float(tooth_details['AT2']):.3f} A\u00b7turn")
            else:
                text += "\n    Slot type: Open slot\n\n"
                text += self.format_equation("dcs", f"{float(tooth_details.get('dcs_m', 0.0)):.6f} m")
                text += self.format_equation("ht", f"{float(tooth_details.get('ht_m', 0.0)):.6f} m")
                text += self.format_equation("ht+0.7·dcs", f"{float(tooth_details.get('denom_m', 0.0)):.6f} m")
                text += "\n"
                text += self.format_equation("hs", f"{float(tooth_details['hs_mm']):.3f} mm")
                text += self.format_equation("wst1(bore)", f"{float(tooth_details['wst1_mm']):.3f} mm")
                text += self.format_equation("wst2(top)", f"{float(tooth_details['wst2_mm']):.3f} mm")
                text += "\n"

                # Show exact flux(z) inputs used
                try:
                    denom_m = float(tooth_details.get('denom_m', 0.0) or 0.0)
                    z0_m = 0.0
                    z1_m = float(tooth_details.get('ht_m', 0.0) or 0.0)
                except Exception:
                    denom_m = z0_m = z1_m = 0.0
                text += "    flux(z) calculation details:\n\n"
                text += self.format_equation("z0", f"{z0_m:.6f} m")
                text += self.format_equation("z1", f"{z1_m:.6f} m")
                text += self.format_equation("denom = ht+0.7·dcs", f"{denom_m:.6f} m")
                text += "\n"
                if denom_m != 0.0:
                    text += f"    flux(z0) = flux_max*(1 - z0/denom) with z0/denom = {z0_m/denom_m:.6g}\n"
                    text += f"    flux(z1) = flux_max*(1 - z1/denom) with z1/denom = {z1_m/denom_m:.6g}\n"
                else:
                    text += "    denom is 0 -> ratios set to 0\n"

                text += self.format_equation("flux(z0=0)", f"{float(tooth_details.get('phi_z0_Wb', 0.0)):.9f} Wb")
                text += self.format_equation("flux(z1=hs)", f"{float(tooth_details.get('phi_z1_Wb', 0.0)):.9f} Wb")
                text += self.format_equation("flux_mean", f"{float(tooth_details.get('phi_mean_Wb', 0.0)):.9f} Wb")
                text += self.format_equation("w_mean", f"{float(tooth_details.get('w_mean_m', 0.0)):.6f} m")
                text += "\n"
                text += self.format_equation("B_mean", f"{float(tooth_details.get('B_mean_T', 0.0)):.6f} T")
                text += self.format_equation("H", f"{float(tooth_details.get('H_mean_Apm', 0.0)):.2f} A/m")

            text += "\n"
            text += self.format_equation("ATST", f"{float(ATST):.3f} A\u00b7turn")

        text += "\n3) Rotor core ampere-turns\n"
        text += self.make_separator()
        text += "    Rotor mean diameter: Dr_mean = Dr - 2\u00d7d_rb - Wry\n"
        text += "    Formula: ATRC = pi \u00d7 Dr_mean \u00d7 H / (P \u00d7 3)\n\n"

        if Dr_mean_m is None:
            text += "    Rotor mean diameter not available (missing Dr/d_rb/Wry).\n"
        else:
            text += self.format_equation("Dr", f"{float(Dr_mm_f):.3f} mm")
            text += self.format_equation("d_rb", f"{float(d_rb_mm_f):.3f} mm")
            text += self.format_equation("Wry", f"{float(Wry_mm_f):.3f} mm")
            text += self.format_equation("Dr_mean", f"{float(Dr_mean_m):.6f} m")
            text += self.format_equation("ATRC", f"{float(ATRC):.3f} A\u00b7turn")

        text += "\n3b) Rotor tooth ampere-turns\n"
        text += self.make_separator()
        text += "    Flux profile inside rotor tooth:\n"
        text += "      flux(z) = flux_max × (1 - z / (ht + 0.7×wry))\n"
        text += "      ht = h_r0 + d_rb\n"
        text += "      wry = rotor yoke depth\n"
        text += "    Segments (3):\n"
        text += "      seg0: 0 → h_r0 (width = tooth width at bore)\n"
        text += "      seg1: h_r0 → h_r0+r1 (width mean = mean(w_bore, Wrt))\n"
        text += "      seg2: h_r0+r1 → h_r0+d_rb (width = Wrt, depth = d_rb-r1)\n\n"

        if 'ATRT' not in locals() or ATRT is None or not rotor_tooth_details:
            text += "    Rotor tooth AT: — (insufficient rotor geometry/flux data)\n"
        else:
            # Inputs
            text += self.format_equation("flux_max(rotor tooth)", f"{float(rotor_tooth_details.get('fluxrt_max_Wb', 0.0)):.9f} Wb")
            text += self.format_equation("Li", f"{float(Li_tooth_m):.6f} m")
            text += "\n"
            text += self.format_equation("Nr", f"{int(rotor_tooth_details.get('Nr', 0))}")
            text += self.format_equation("Dr", f"{float(rotor_tooth_details.get('Dr_mm', 0.0)):.3f} mm")
            text += self.format_equation("pitch_rotor(Dr)", f"{float(rotor_tooth_details.get('pitch_rotor_mm', 0.0)):.6f} mm")
            text += self.format_equation("b_r0", f"{float(rotor_tooth_details.get('b_r0_mm', 0.0)):.6f} mm")
            text += self.format_equation("w_tooth_bore", f"{float(rotor_tooth_details.get('w_tooth_bore_mm', 0.0)):.6f} mm")
            text += "\n"
            text += self.format_equation("h_r0", f"{float(rotor_tooth_details.get('h_r0_mm', 0.0)):.6f} mm")
            text += self.format_equation("r1", f"{float(rotor_tooth_details.get('r1_mm', 0.0)):.6f} mm")
            text += self.format_equation("d_rb", f"{float(rotor_tooth_details.get('d_rb_mm', 0.0)):.6f} mm")
            text += self.format_equation("wry", f"{float(rotor_tooth_details.get('Wry_mm', 0.0)):.6f} mm")
            text += self.format_equation("Wrt (wst)", f"{float(rotor_tooth_details.get('Wrt_mm', 0.0)):.6f} mm")
            text += self.format_equation("ht", f"{float(rotor_tooth_details.get('ht_m', 0.0)):.6f} m")
            text += self.format_equation("denom = ht+0.7·wry", f"{float(rotor_tooth_details.get('denom_m', 0.0)):.6f} m")

            # flux(z) details
            try:
                ht_m_rt = float(rotor_tooth_details.get('ht_m', 0.0) or 0.0)
                denom_m_rt = float(rotor_tooth_details.get('denom_m', 0.0) or 0.0)
                z0 = float(rotor_tooth_details.get('z0_m', 0.0) or 0.0)
                z1 = float(rotor_tooth_details.get('z1_m', 0.0) or 0.0)
                z2 = float(rotor_tooth_details.get('z2_m', 0.0) or 0.0)
                z3 = float(rotor_tooth_details.get('z3_m', 0.0) or 0.0)
            except Exception:
                ht_m_rt = denom_m_rt = z0 = z1 = z2 = z3 = 0.0

            text += "\n    flux(z) calculation details:\n\n"
            text += self.format_equation("z0", f"{z0:.6f} m")
            text += self.format_equation("z1", f"{z1:.6f} m")
            text += self.format_equation("z2", f"{z2:.6f} m")
            text += self.format_equation("z3", f"{z3:.6f} m")
            text += self.format_equation("ht", f"{ht_m_rt:.6f} m")
            text += self.format_equation("denom = ht+0.7·wry", f"{denom_m_rt:.6f} m")
            text += "\n"
            if denom_m_rt != 0.0:
                text += f"    flux(z0) = flux_max*(1 - z0/denom) with z0/denom = {z0/denom_m_rt:.6g}\n"
                text += f"    flux(z1) = flux_max*(1 - z1/denom) with z1/denom = {z1/denom_m_rt:.6g}\n"
                text += f"    flux(z2) = flux_max*(1 - z2/denom) with z2/denom = {z2/denom_m_rt:.6g}\n"
                text += f"    flux(z3) = flux_max*(1 - z3/denom) with z3/denom = {z3/denom_m_rt:.6g}\n"
            else:
                text += "    denom is 0 -> ratios set to 0\n"

            # Flux at boundaries
            text += "\n    Flux at segment boundaries (from flux(z)):\n\n"
            text += self.format_equation("flux(z0=0)", f"{float(rotor_tooth_details.get('phi0_Wb', 0.0)):.9f} Wb")
            text += self.format_equation("flux(z1=h_r0)", f"{float(rotor_tooth_details.get('phi1_Wb', 0.0)):.9f} Wb")
            text += self.format_equation("flux(z2=h_r0+r1)", f"{float(rotor_tooth_details.get('phi2_Wb', 0.0)):.9f} Wb")
            text += self.format_equation("flux(z3=h_r0+d_rb)", f"{float(rotor_tooth_details.get('phi3_Wb', 0.0)):.9f} Wb")

            # Segment details
            text += "\n    Segment 0 (0 → h_r0):\n\n"
            text += self.format_equation("flux_mean(seg0)", f"{float(rotor_tooth_details.get('phi_mean_0_Wb', 0.0)):.9f} Wb")
            text += self.format_equation("w_mean(seg0)", f"{float(rotor_tooth_details.get('w_mean_0_m', 0.0)):.6f} m")
            text += self.format_equation("B_mean(seg0)", f"{float(rotor_tooth_details.get('B0_T', 0.0)):.6f} T")
            text += self.format_equation("H(seg0)", f"{float(rotor_tooth_details.get('H0_Apm', 0.0)):.2f} A/m")
            text += self.format_equation("depth(seg0)", f"{float(rotor_tooth_details.get('seg0_h_m', 0.0)):.6f} m")
            text += self.format_equation("AT(seg0)", f"{float(rotor_tooth_details.get('AT0', 0.0)):.3f} A\u00b7turn")

            text += "\n    Segment 1 (h_r0 → h_r0+r1):\n\n"
            text += self.format_equation("flux_mean(seg1)", f"{float(rotor_tooth_details.get('phi_mean_1_Wb', 0.0)):.9f} Wb")
            text += self.format_equation("w_mean(seg1)", f"{float(rotor_tooth_details.get('w_mean_1_m', 0.0)):.6f} m")
            text += self.format_equation("B_mean(seg1)", f"{float(rotor_tooth_details.get('B1_T', 0.0)):.6f} T")
            text += self.format_equation("H(seg1)", f"{float(rotor_tooth_details.get('H1_Apm', 0.0)):.2f} A/m")
            text += self.format_equation("depth(seg1)", f"{float(rotor_tooth_details.get('seg1_h_m', 0.0)):.6f} m")
            text += self.format_equation("AT(seg1)", f"{float(rotor_tooth_details.get('AT1', 0.0)):.3f} A\u00b7turn")

            text += "\n    Segment 2 (h_r0+r1 → h_r0+d_rb):\n\n"
            text += self.format_equation("flux_mean(seg2)", f"{float(rotor_tooth_details.get('phi_mean_2_Wb', 0.0)):.9f} Wb")
            text += self.format_equation("w_mean(seg2)", f"{float(rotor_tooth_details.get('w_mean_2_m', 0.0)):.6f} m")
            text += self.format_equation("B_mean(seg2)", f"{float(rotor_tooth_details.get('B2_T', 0.0)):.6f} T")
            text += self.format_equation("H(seg2)", f"{float(rotor_tooth_details.get('H2_Apm', 0.0)):.2f} A/m")
            text += self.format_equation("depth(seg2)", f"{float(rotor_tooth_details.get('seg2_h_m', 0.0)):.6f} m")
            text += self.format_equation("AT(seg2)", f"{float(rotor_tooth_details.get('AT2', 0.0)):.3f} A\u00b7turn")

            text += "\n"
            text += self.format_equation("ATRT", f"{float(ATRT):.3f} A\u00b7turn")

        text += "\n4) Airgap ampere-turns (Carter-corrected)\n"
        text += self.make_separator()
        text += "    Steps:\n"
        text += "      - Compute Carter factors kgs (stator) and kgr (rotor) from slot opening/airgap ratio and tables\n"
        text += "      - Effective airgap: Lgd = Lg × kgs × kgr\n"
        text += "      - Airgap area per pole: Agp = pi × D × Ls(corr) / P\n"
        text += "      - Airgap flux density: Bg = phi_pole / Agp\n"
        text += "      - Airgap ampere-turns: ATg = 0.796e6 × Bg × 1.36 × Lgd\n\n"

        if Lg_mm_f is None:
            text += "    Airgap length Lg not available (missing rotor Lg_mm).\n"
        else:
            text += self.format_equation("Lg", f"{float(Lg_mm_f):.3f} mm = {float(Lg_mm_f)/1000.0:.6f} m")

        # Stator Carter details
        if kgs is None or carter_stator is None or stator_ratio is None or bs_stator_mm is None or pitch_stator_mm is None:
            text += "\n    Stator Carter factor kgs: — (insufficient slot geometry)\n"
        else:
            slot_shape_label = "Open slot" if slot_method == 'open_slot' else "Trapezoidal semi-open slot"
            table_label = "OPEN slot table" if carter_table_used == 'open' else "SEMI-OPEN slot table"
            bs_label = "bs" if slot_method == 'open_slot' else "bs0"
            text += f"\n    Stator slot shape: {slot_shape_label}\n"
            text += f"    Carter table used: {table_label}\n\n"
            text += self.format_equation(f"slot_pitch_stator(D)", f"{float(pitch_stator_mm):.3f} mm")
            text += self.format_equation(bs_label, f"{float(bs_stator_mm):.3f} mm")
            text += self.format_equation(f"{bs_label}/Lg", f"{float(stator_ratio):.6f}")
            text += self.format_equation("carter_coeff(stator)", f"{float(carter_stator):.4f}")
            text += self.format_equation("kgs", f"{float(kgs):.6f}")

        # Rotor Carter details
        if kgr is None or carter_rotor is None or rotor_ratio is None or b_r0_mm_f is None or pitch_rotor_mm is None:
            text += "\n    Rotor Carter factor kgr: — (insufficient rotor geometry)\n"
        else:
            text += "\n    Rotor Carter table used: SEMI-OPEN slot table\n\n"
            text += self.format_equation("Nr(rotor)", f"{int(Nr_rotor_i)}")
            text += self.format_equation("slot_pitch_rotor(Dr)", f"{float(pitch_rotor_mm):.3f} mm")
            text += self.format_equation("b_r0", f"{float(b_r0_mm_f):.3f} mm")
            text += self.format_equation("b_r0/Lg", f"{float(rotor_ratio):.6f}")
            text += self.format_equation("carter_coeff(rotor)", f"{float(carter_rotor):.4f}")
            text += self.format_equation("kgr", f"{float(kgr):.6f}")

        # Effective airgap
        if Lgd_m is None:
            text += "\n    Effective airgap Lgd: — (need kgs and kgr)\n"
        else:
            text += "\n"
            text += self.format_equation("Lgd", f"{float(Lgd_m):.9f} m")

        # Area, Bg, ATg
        if Agp_m2 is None or Bg_T is None or ATg is None:
            text += "\n    Airgap AT: — (need D, Ls(corr), phi_pole, and Lgd)\n"
        else:
            text += "\n"
            text += self.format_equation("D(bore)", f"{float(D_bore_m):.6f} m")
            text += self.format_equation("Ls(corr)", f"{float(Ls_corr_m):.6f} m")
            text += self.format_equation("phi_pole(corr)", f"{float(phi_pole_Wb):.9f} Wb")
            text += self.format_equation("Agp", f"{float(Agp_m2):.9e} m^2")
            text += self.format_equation("Bg", f"{float(Bg_T):.6f} T")
            text += self.format_equation("ATg", f"{float(ATg):.3f} A\u00b7turn")

        # Total ampere-turns summary
        text += "\n5) Total ampere-turns (summary)\n"
        text += self.make_separator()
        text += "    Formula: AT_total = ATSC + ATST + ATRC + ATRT + ATg\n\n"

        try:
            atsc_v = float(ATSC) if ATSC is not None else 0.0
        except Exception:
            atsc_v = 0.0
        try:
            atst_v = float(ATST) if ATST is not None else 0.0
        except Exception:
            atst_v = 0.0
        try:
            atrc_v = float(ATRC) if ATRC is not None else 0.0
        except Exception:
            atrc_v = 0.0
        try:
            atrt_v = float(ATRT) if ('ATRT' in locals() and ATRT is not None) else 0.0
        except Exception:
            atrt_v = 0.0
        try:
            atg_v = float(ATg) if ATg is not None else 0.0
        except Exception:
            atg_v = 0.0

        AT_total = float(atsc_v + atst_v + atrc_v + atrt_v + atg_v)
        text += self.format_equation("ATSC", f"{atsc_v:.3f} A\u00b7turn")
        text += self.format_equation("ATST", f"{atst_v:.3f} A\u00b7turn")
        text += self.format_equation("ATRC", f"{atrc_v:.3f} A\u00b7turn")
        text += self.format_equation("ATRT", f"{atrt_v:.3f} A\u00b7turn")
        text += self.format_equation("ATg", f"{atg_v:.3f} A\u00b7turn")
        text += "\n"
        text += self.format_equation("AT_total", f"{AT_total:.3f} A\u00b7turn")

        # ------------------------------------------------------------
        # No-load current and losses consolidation
        # ------------------------------------------------------------
        winding = getattr(self.designer, 'winding', None)
        elec = getattr(self.designer, 'electrical', None)

        # Magnetizing current (Im)
        text += "\n6) Magnetizing current (Im)\n"
        text += self.make_separator()
        text += "    Formula: Im = P\u00d7AT_total / (2\u00d71.17\u00d7kw\u00d7Tph)\n\n"

        kw_used = None
        try:
            kw_used = float(getattr(self.designer, '_K_w', None)) if getattr(self.designer, '_K_w', None) is not None else None
        except Exception:
            kw_used = None

        Tph = None
        try:
            Tph = float(getattr(winding, 'TPH', None)) if (winding is not None and getattr(winding, 'TPH', None) is not None) else None
        except Exception:
            Tph = None

        Im_A = None
        denom_im = None
        try:
            denom_im = 2.0 * 1.17 * float(kw_used) * float(Tph) if (kw_used is not None and Tph is not None and kw_used > 0 and Tph > 0) else None
        except Exception:
            denom_im = None
        if denom_im is not None and denom_im > 0:
            Im_A = (float(poles) * float(AT_total)) / float(denom_im)

        text += self.format_equation("P", f"{int(poles)}")
        text += self.format_equation("kw", f"{float(kw_used):.6f}" if kw_used is not None else "—")
        text += self.format_equation("Tph", f"{float(Tph):.0f}" if Tph is not None else "—")
        if denom_im is not None:
            text += self.format_equation("denom = 2\u00d71.17\u00d7kw\u00d7Tph", f"{float(denom_im):.6f}")
        text += "\n"
        text += self.format_equation("Im", f"{float(Im_A):.6f} A" if Im_A is not None else "—")

        # Losses (stator + rotor + friction) and no-load power
        text += "\n7) No-load losses and no-load current\n"
        text += self.make_separator()
        text += "    Collect losses:\n"
        text += "      - Stator copper: P_js\n"
        text += "      - Stator iron: Pit (teeth), Pic (core)\n"
        text += "      - Rotor copper: P_bar (bars), P_er (rings)\n"
        text += "      - Friction: Pfw = 0.01\u00d7P_elec\u00d71000\n"
        text += "      - Rotor iron (added here): Pitr (teeth), Picr (core)\n\n"

        # --- Inputs for losses
        P_js_W = None
        try:
            P_js_W = float(getattr(winding, 'copper_losses', None)) if (winding is not None and getattr(winding, 'copper_losses', None) is not None) else None
        except Exception:
            P_js_W = None

        Pit_W = None
        Pic_W = None
        try:
            Pit_W = float(val_info.get('Pit_W', None)) if val_info.get('Pit_W', None) is not None else None
        except Exception:
            Pit_W = None
        try:
            Pic_W = float(val_info.get('Pic_W', None)) if val_info.get('Pic_W', None) is not None else None
        except Exception:
            Pic_W = None

        P_bar_W = None
        P_er_W = None
        try:
            P_bar_W = float(rotor.get('P_bar_W', None)) if rotor.get('P_bar_W', None) is not None else None
        except Exception:
            P_bar_W = None
        try:
            P_er_W = float(rotor.get('P_er_W', None)) if rotor.get('P_er_W', None) is not None else None
        except Exception:
            P_er_W = None

        # P_elec (kW) from stator electrical results
        P_elec_kW = None
        try:
            P_elec_kW = float(getattr(elec, 'P_elec', None)) if (elec is not None and getattr(elec, 'P_elec', None) is not None) else None
        except Exception:
            P_elec_kW = None
        Pfw_W = (0.01 * float(P_elec_kW) * 1000.0) if (P_elec_kW is not None and P_elec_kW > 0) else None

        # Lamination loss coefficient Pkg and density Kgm_3 (for rotor iron losses)
        Pkg_W_per_kg = None
        try:
            Pkg_W_per_kg = float(val_info.get('iron_loss_Pkg_W_per_kg_used', None)) if val_info.get('iron_loss_Pkg_W_per_kg_used', None) is not None else None
        except Exception:
            Pkg_W_per_kg = None

        Kgm_3 = None
        for k in ('Wc_density_kg_m3_used', 'Wt_density_kg_m3_used', 'Wt_density_kg_m3_used', 'Wc_density_kg_m3_used'):
            if Kgm_3 is not None:
                break
            try:
                if val_info.get(k, None) is not None:
                    Kgm_3 = float(val_info.get(k))
            except Exception:
                Kgm_3 = None
        if Kgm_3 is None:
            # Conservative fallback
            Kgm_3 = 7650.0

        # --- Rotor core weight and iron loss
        Wcr_kg = None
        Picr_W = None
        rotor_core_details = {}
        try:
            # Geometry inputs (mm -> m)
            h_r0_mm_for_w = float(rotor.get('h_r0_mm', None)) if rotor.get('h_r0_mm', None) is not None else None
            d_rb_mm_for_w = float(rotor.get('d_rb_mm', None)) if rotor.get('d_rb_mm', None) is not None else None
            Wry_mm_for_w = float(rotor.get('Wry_mm', None)) if rotor.get('Wry_mm', None) is not None else None
            Dr_mm_for_w = float(rotor.get('Dr_mm', None)) if rotor.get('Dr_mm', None) is not None else None
        except Exception:
            h_r0_mm_for_w = d_rb_mm_for_w = Wry_mm_for_w = Dr_mm_for_w = None

        if (
            Dr_mm_for_w is not None and Dr_mm_for_w > 0
            and h_r0_mm_for_w is not None and h_r0_mm_for_w >= 0
            and d_rb_mm_for_w is not None and d_rb_mm_for_w > 0
            and Wry_mm_for_w is not None and Wry_mm_for_w >= 0
            and b_yoke is not None and float(b_yoke) > 0
            and phi_pole_Wb is not None and float(phi_pole_Wb) > 0
        ):
            D_mean_r_mm = float(Dr_mm_for_w) - 2.0 * (float(h_r0_mm_for_w) + float(d_rb_mm_for_w)) - float(Wry_mm_for_w)
            D_mean_r_m = float(D_mean_r_mm) / 1000.0
            A_c_r_m2 = (float(phi_pole_Wb) / 2.0) / float(b_yoke)
            Wcr_kg = math.pi * float(D_mean_r_m) * float(A_c_r_m2) * float(Kgm_3)
            if Pkg_W_per_kg is not None and Pkg_W_per_kg >= 0:
                Picr_W = float(Wcr_kg) * float(Pkg_W_per_kg)

            rotor_core_details = {
                'D_mean_r_m': float(D_mean_r_m),
                'A_c_r_m2': float(A_c_r_m2),
            }

        # --- Rotor tooth weight and iron loss
        Wtr_kg = None
        Pitr_W = None
        rotor_tooth_weight_details = {}
        try:
            r1_mm_w = float(rotor.get('r1_mm', None)) if rotor.get('r1_mm', None) is not None else None
            r2_mm_w = float(rotor.get('r2_mm', None)) if rotor.get('r2_mm', None) is not None else None
            Wrt_mm_w = float(rotor.get('Wrt_mm', None)) if rotor.get('Wrt_mm', None) is not None else None
            Nr_w = int(rotor.get('Nr', 0) or 0)
        except Exception:
            r1_mm_w = r2_mm_w = Wrt_mm_w = None
            Nr_w = 0

        # For tooth width at bore, reuse computed value if available from AT block; else compute
        w_tooth_bore_mm_w = None
        try:
            if rotor_tooth_details and rotor_tooth_details.get('w_tooth_bore_mm', None) is not None:
                w_tooth_bore_mm_w = float(rotor_tooth_details.get('w_tooth_bore_mm'))
        except Exception:
            w_tooth_bore_mm_w = None
        if w_tooth_bore_mm_w is None:
            try:
                if Dr_mm_for_w is not None and Nr_w > 0 and b_r0_mm_f is not None:
                    w_tooth_bore_mm_w = (math.pi * float(Dr_mm_for_w) / float(Nr_w)) - float(b_r0_mm_f)
            except Exception:
                w_tooth_bore_mm_w = None

        if (
            w_tooth_bore_mm_w is not None and w_tooth_bore_mm_w > 0
            and h_r0_mm_for_w is not None and h_r0_mm_for_w >= 0
            and d_rb_mm_for_w is not None and d_rb_mm_for_w > 0
            and Wrt_mm_w is not None and Wrt_mm_w > 0
            and r1_mm_w is not None and r1_mm_w >= 0
            and r2_mm_w is not None and r2_mm_w >= 0
            and Li_tooth_m is not None and Li_tooth_m > 0
            and Nr_w > 0
        ):
            w_bore_m = float(w_tooth_bore_mm_w) / 1000.0
            h_r0_m_w = float(h_r0_mm_for_w) / 1000.0
            r1_m_w = float(r1_mm_w) / 1000.0
            r2_m_w = float(r2_mm_w) / 1000.0
            d_rb_m_w = float(d_rb_mm_for_w) / 1000.0
            wst_m_w = float(Wrt_mm_w) / 1000.0

            # Segment areas (in the rotor cross-section plane)
            A0 = float(w_bore_m) * float(h_r0_m_w)

            # Transition region: rectangle (with mean width) minus 2 quarter-circles (total half-circle)
            w_mean_1 = 0.5 * (float(w_bore_m) + float(wst_m_w))
            A1_rect = float(w_mean_1) * float(r1_m_w)
            A1_cut = 0.5 * math.pi * float(r1_m_w) * float(r1_m_w)
            A1 = max(float(A1_rect) - float(A1_cut), 0.0)

            # Body region (as requested): wst * (d_rb - r1 - r2)
            body_h = max(float(d_rb_m_w) - float(r1_m_w) - float(r2_m_w), 0.0)
            A2 = float(wst_m_w) * float(body_h)

            A_tooth_m2 = float(A0 + A1 + A2)
            Wtr_kg = float(A_tooth_m2) * float(Li_tooth_m) * float(Nr_w) * float(Kgm_3)
            if Pkg_W_per_kg is not None and Pkg_W_per_kg >= 0:
                Pitr_W = float(Wtr_kg) * float(Pkg_W_per_kg)

            rotor_tooth_weight_details = {
                'w_bore_m': float(w_bore_m),
                'h_r0_m': float(h_r0_m_w),
                'r1_m': float(r1_m_w),
                'r2_m': float(r2_m_w),
                'd_rb_m': float(d_rb_m_w),
                'wst_m': float(wst_m_w),
                'A0_m2': float(A0),
                'A1_rect_m2': float(A1_rect),
                'A1_cut_m2': float(A1_cut),
                'A1_m2': float(A1),
                'body_h_m': float(body_h),
                'A2_m2': float(A2),
                'A_tooth_m2': float(A_tooth_m2),
            }

        # Print loss table
        text += self.format_equation("P_js", f"{float(P_js_W):.3f} W" if P_js_W is not None else "—")
        text += self.format_equation("Pit", f"{float(Pit_W):.3f} W" if Pit_W is not None else "—")
        text += self.format_equation("Pic", f"{float(Pic_W):.3f} W" if Pic_W is not None else "—")
        text += self.format_equation("P_bar", f"{float(P_bar_W):.3f} W" if P_bar_W is not None else "—")
        text += self.format_equation("P_er", f"{float(P_er_W):.3f} W" if P_er_W is not None else "—")
        text += self.format_equation("Pfw", f"{float(Pfw_W):.3f} W" if Pfw_W is not None else "—")

        text += "\n    Rotor iron losses (computed here):\n\n"
        text += self.format_equation("Pkg", f"{float(Pkg_W_per_kg):.3f} W/kg" if Pkg_W_per_kg is not None else "—")
        text += self.format_equation("Kgm_3", f"{float(Kgm_3):.0f} kg/m^3")

        # Rotor core details
        if Wcr_kg is None:
            text += "\n    Rotor core weight/loss: — (missing geometry and/or phi_pole/Bry)\n"
        else:
            text += "\n    Rotor core (yoke) weight:\n\n"
            text += self.format_equation("D_mean_r", f"{float(rotor_core_details.get('D_mean_r_m', 0.0)):.6f} m")
            text += self.format_equation("A_c_r", f"{float(rotor_core_details.get('A_c_r_m2', 0.0)):.9e} m^2")
            text += self.format_equation("Wcr", f"{float(Wcr_kg):.4f} kg")
            text += self.format_equation("Picr", f"{float(Picr_W):.3f} W" if Picr_W is not None else "—")

        # Rotor tooth details
        if Wtr_kg is None or not rotor_tooth_weight_details:
            text += "\n    Rotor tooth weight/loss: — (missing geometry inputs)\n"
        else:
            text += "\n    Rotor teeth weight (3-segment approximation):\n\n"
            text += self.format_equation("Nr", f"{int(Nr_w)}")
            text += self.format_equation("w_bore", f"{float(rotor_tooth_weight_details['w_bore_m']):.6f} m")
            text += self.format_equation("h_r0", f"{float(rotor_tooth_weight_details['h_r0_m']):.6f} m")
            text += self.format_equation("r1", f"{float(rotor_tooth_weight_details['r1_m']):.6f} m")
            text += self.format_equation("r2", f"{float(rotor_tooth_weight_details['r2_m']):.6f} m")
            text += self.format_equation("d_rb", f"{float(rotor_tooth_weight_details['d_rb_m']):.6f} m")
            text += self.format_equation("wst", f"{float(rotor_tooth_weight_details['wst_m']):.6f} m")
            text += "\n"
            text += self.format_equation("A0", f"{float(rotor_tooth_weight_details['A0_m2']):.9e} m^2")
            text += self.format_equation("A1_rect", f"{float(rotor_tooth_weight_details['A1_rect_m2']):.9e} m^2")
            text += self.format_equation("A1_cut(2\u00d7quarter-circles)", f"{float(rotor_tooth_weight_details['A1_cut_m2']):.9e} m^2")
            text += self.format_equation("A1", f"{float(rotor_tooth_weight_details['A1_m2']):.9e} m^2")
            text += self.format_equation("body_h = d_rb-r1-r2", f"{float(rotor_tooth_weight_details['body_h_m']):.6f} m")
            text += self.format_equation("A2", f"{float(rotor_tooth_weight_details['A2_m2']):.9e} m^2")
            text += self.format_equation("A_tooth", f"{float(rotor_tooth_weight_details['A_tooth_m2']):.9e} m^2")
            text += "\n"
            text += self.format_equation("Wtr", f"{float(Wtr_kg):.4f} kg")
            text += self.format_equation("Pitr", f"{float(Pitr_W):.3f} W" if Pitr_W is not None else "—")

        # No-load power and currents
        # User adjustment:
        #   - No-load power excludes copper losses (stator copper + rotor bars/rings).
        #   - The previous sum including copper losses is kept as "total losses".
        P_no_load_W = 0.0
        for v in (Pit_W, Pic_W, Pfw_W, Pitr_W, Picr_W):
            try:
                if v is not None:
                    P_no_load_W += float(v)
            except Exception:
                pass

        total_losses_W = float(P_no_load_W)
        for v in (P_js_W, P_bar_W, P_er_W):
            try:
                if v is not None:
                    total_losses_W += float(v)
            except Exception:
                pass

        V_line = None
        try:
            V_line = float(getattr(specs, 'voltage_v', None)) if getattr(specs, 'voltage_v', None) is not None else None
        except Exception:
            V_line = None

        Iw_A = None
        if V_line is not None and V_line > 0:
            Iw_A = float(P_no_load_W) / (math.sqrt(3.0) * float(V_line))

        I0_A = None
        Pf0 = None
        if Iw_A is not None and Im_A is not None:
            I0_A = math.sqrt(float(Iw_A) * float(Iw_A) + float(Im_A) * float(Im_A))
            if I0_A > 0:
                Pf0 = float(Iw_A) / float(I0_A)

        text += "\n    Totals and no-load current:\n\n"
        text += self.format_equation("Pnl (no-load power, no copper)", f"{float(P_no_load_W):.3f} W = {float(P_no_load_W)/1000.0:.6f} kW")
        text += self.format_equation("Total losses (incl. copper)", f"{float(total_losses_W):.3f} W = {float(total_losses_W)/1000.0:.6f} kW")
        text += self.format_equation("V_line", f"{float(V_line):.3f} V" if V_line is not None else "—")
        text += self.format_equation("Iw = Pnl/(\u221a3\u00b7V_line)", f"{float(Iw_A):.6f} A" if Iw_A is not None else "—")
        text += self.format_equation("I0 = \u221a(Iw^2 + Im^2)", f"{float(I0_A):.6f} A" if I0_A is not None else "—")
        text += self.format_equation("Pf0 = Iw/I0", f"{float(Pf0):.6f}" if Pf0 is not None else "—")

        # Actual efficiency using rated output power P (kW) and total losses (kW)
        text += "\n8) Actual efficiency (from total losses)\n"
        text += self.make_separator()
        text += "    Formula: efficiency = P / (P + total_losses)\n"
        text += "    (P is the user-rated mechanical/electrical output power input, in kW)\n\n"

        P_rated_kW = None
        try:
            P_rated_kW = float(getattr(specs, 'power_kw', None)) if getattr(specs, 'power_kw', None) is not None else None
        except Exception:
            P_rated_kW = None

        eff_pct = None
        if P_rated_kW is not None and P_rated_kW > 0:
            total_losses_kW = float(total_losses_W) / 1000.0
            denom_eff = float(P_rated_kW) + float(total_losses_kW)
            if denom_eff > 0:
                eff_pct = 100.0 * float(P_rated_kW) / denom_eff

        text += self.format_equation("P (rated)", f"{float(P_rated_kW):.6f} kW" if P_rated_kW is not None else "—")
        text += self.format_equation("total_losses", f"{float(total_losses_W)/1000.0:.6f} kW")
        text += self.format_equation("efficiency", f"{float(eff_pct):.3f} %" if eff_pct is not None else "—")

        # Nominal slip estimate from losses
        text += "\n9) Nominal slip (g) from losses\n"
        text += self.make_separator()
        text += "    Formula: g = rotor_copper_losses / (P_in - P_js - P_iron_stator)\n"
        text += "      rotor_copper_losses = P_bar + P_er\n"
        text += "      P_iron_stator = Pit + Pic\n\n"

        P_in_W = (float(P_elec_kW) * 1000.0) if (P_elec_kW is not None and P_elec_kW > 0) else None
        rotor_cu_W = None
        try:
            rotor_cu_W = (float(P_bar_W) if P_bar_W is not None else 0.0) + (float(P_er_W) if P_er_W is not None else 0.0)
        except Exception:
            rotor_cu_W = None

        stator_iron_W = None
        try:
            stator_iron_W = (float(Pit_W) if Pit_W is not None else 0.0) + (float(Pic_W) if Pic_W is not None else 0.0)
        except Exception:
            stator_iron_W = None

        denom_g_W = None
        try:
            if P_in_W is not None:
                denom_g_W = float(P_in_W)
                if P_js_W is not None:
                    denom_g_W -= float(P_js_W)
                if stator_iron_W is not None:
                    denom_g_W -= float(stator_iron_W)
        except Exception:
            denom_g_W = None

        g_pct = None
        if rotor_cu_W is not None and denom_g_W is not None and denom_g_W > 0:
            g_pct = 100.0 * float(rotor_cu_W) / float(denom_g_W)

        text += self.format_equation("P_in", f"{float(P_in_W):.3f} W" if P_in_W is not None else "—")
        text += self.format_equation("rotor_copper_losses (P_bar+P_er)", f"{float(rotor_cu_W):.3f} W" if rotor_cu_W is not None else "—")
        text += self.format_equation("P_js", f"{float(P_js_W):.3f} W" if P_js_W is not None else "—")
        text += self.format_equation("P_iron_stator (Pit+Pic)", f"{float(stator_iron_W):.3f} W" if stator_iron_W is not None else "—")
        text += self.format_equation("denom = P_in-P_js-P_iron_stator", f"{float(denom_g_W):.3f} W" if denom_g_W is not None else "—")
        text += "\n"
        text += self.format_equation("g", f"{float(g_pct):.3f} %" if g_pct is not None else "—")

        # Summary (requested)
        text += "\n10) Summary\n"
        text += self.make_separator()
        text += self.format_equation("Magnetizing current Im", f"{float(Im_A):.6f} A" if Im_A is not None else "—")
        text += self.format_equation("Pnl (no-load power, no copper)", f"{float(P_no_load_W):.3f} W = {float(P_no_load_W)/1000.0:.6f} kW")
        text += self.format_equation("Total losses (incl. copper)", f"{float(total_losses_W):.3f} W = {float(total_losses_W)/1000.0:.6f} kW")
        text += self.format_equation("Iw", f"{float(Iw_A):.6f} A" if Iw_A is not None else "—")
        text += self.format_equation("I0", f"{float(I0_A):.6f} A" if I0_A is not None else "—")
        text += self.format_equation("Pf0", f"{float(Pf0):.6f}" if Pf0 is not None else "—")
        text += self.format_equation("Efficiency", f"{float(eff_pct):.3f} %" if eff_pct is not None else "—")
        text += self.format_equation("Nominal slip g", f"{float(g_pct):.3f} %" if g_pct is not None else "—")

        self.append_to_tab(self.tab_efficiency, text)

    def create_widgets(self, main_layout):
        """Create all GUI widgets"""
        # Left side: a tabbed input area + a fixed actions bar.
        left_panel = QWidget()
        left_panel_layout = QVBoxLayout(left_panel)
        left_panel_layout.setContentsMargins(0, 0, 0, 0)
        left_panel_layout.setSpacing(10)

        # Two tabs (Stator / Rotor). Rotor is disabled until stator is calculated.
        self.design_tab_widget = QTabWidget()
        self.design_tab_widget.setMaximumWidth(370)
        self.design_tab_widget.currentChanged.connect(self.on_design_tab_changed)

        self.stator_tab = QWidget()
        stator_tab_layout = QVBoxLayout(self.stator_tab)
        stator_tab_layout.setContentsMargins(0, 0, 0, 0)

        # Make stator tab scrollable (prevents clipping on small screens / high DPI / maximized).
        stator_scroll = QScrollArea()
        stator_scroll.setWidgetResizable(True)
        stator_scroll.setFrameShape(QScrollArea.Shape.NoFrame)
        stator_scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        stator_scroll_content = QWidget()
        stator_scroll.setWidget(stator_scroll_content)
        stator_scroll_layout = QVBoxLayout(stator_scroll_content)
        stator_scroll_layout.setContentsMargins(0, 0, 0, 0)
        stator_scroll_layout.setSpacing(10)
        stator_tab_layout.addWidget(stator_scroll, 1)

        self.rotor_tab = QWidget()
        rotor_tab_layout = QVBoxLayout(self.rotor_tab)
        rotor_tab_layout.setContentsMargins(0, 0, 0, 0)

        # Make rotor tab scrollable as well.
        rotor_scroll = QScrollArea()
        rotor_scroll.setWidgetResizable(True)
        rotor_scroll.setFrameShape(QScrollArea.Shape.NoFrame)
        rotor_scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAsNeeded)
        rotor_scroll_content = QWidget()
        rotor_scroll.setWidget(rotor_scroll_content)
        rotor_scroll_layout = QVBoxLayout(rotor_scroll_content)
        rotor_scroll_layout.setContentsMargins(0, 0, 0, 0)
        rotor_scroll_layout.setSpacing(10)
        rotor_tab_layout.addWidget(rotor_scroll, 1)

        # Rotor tab content (inputs only; calculations wired later)
        rotor_group = QGroupBox("Rotor Configuration")
        rotor_layout = QGridLayout()
        rotor_group.setLayout(rotor_layout)

        rrow = 0
        rotor_layout.addWidget(QLabel("Rotor slots method:"), rrow, 0)
        self.rotor_nr_method_combo = QComboBox()
        self.rotor_nr_method_combo.addItem("Classique", "classique")
        self.rotor_nr_method_combo.addItem("Harmonique", "harmonique")
        self.rotor_nr_method_combo.currentIndexChanged.connect(self.on_rotor_nr_method_changed)
        rotor_layout.addWidget(self.rotor_nr_method_combo, rrow, 1)

        rrow += 1
        rotor_layout.addWidget(QLabel("Rotor slots (Nr):"), rrow, 0)
        nr_select_widget = QWidget()
        nr_select_layout = QHBoxLayout(nr_select_widget)
        nr_select_layout.setContentsMargins(0, 0, 0, 0)
        self.rotor_nr_combo = QComboBox()
        self.rotor_nr_combo.currentIndexChanged.connect(self.on_rotor_nr_choice_changed)
        self.rotor_nr_custom_input = QLineEdit()
        self.rotor_nr_custom_input.setPlaceholderText("Custom Nr")
        self.rotor_nr_custom_input.setFixedWidth(110)
        self.rotor_nr_custom_input.hide()
        nr_select_layout.addWidget(self.rotor_nr_combo, 1)
        nr_select_layout.addWidget(self.rotor_nr_custom_input, 0)
        rotor_layout.addWidget(nr_select_widget, rrow, 1)

        rrow += 1
        rotor_layout.addWidget(QLabel("Nr range:"), rrow, 0)
        self.rotor_nr_slider = QSlider(Qt.Orientation.Horizontal)
        self.rotor_nr_slider.setMinimum(0)
        self.rotor_nr_slider.setMaximum(0)
        self.rotor_nr_slider.valueChanged.connect(self.on_rotor_nr_slider_changed)
        rotor_layout.addWidget(self.rotor_nr_slider, rrow, 1)

        rrow += 1
        self.rotor_nr_info_label = QLabel("Run stator design to populate Nr options.")
        self.rotor_nr_info_label.setStyleSheet("color: gray; font-size: 9pt;")
        self.rotor_nr_info_label.setWordWrap(True)
        rotor_layout.addWidget(self.rotor_nr_info_label, rrow, 0, 1, 2)

        # Slot geometry tolerance
        rrow += 1
        rotor_layout.addWidget(QLabel("Slot geometry tolerance:"), rrow, 0)
        tol_widget = QWidget()
        tol_layout = QHBoxLayout(tol_widget)
        tol_layout.setContentsMargins(0, 0, 0, 0)
        self.rotor_tol_combo = QComboBox()
        self.rotor_tol_combo.addItem("1%", 1.0)
        self.rotor_tol_combo.addItem("2%", 2.0)
        self.rotor_tol_combo.addItem("3%", 3.0)
        self.rotor_tol_combo.addItem("5%", 5.0)
        self.rotor_tol_combo.addItem("Custom", "custom")
        self.rotor_tol_combo.currentIndexChanged.connect(self.on_rotor_tol_changed)
        self.rotor_tol_custom_input = QLineEdit()
        self.rotor_tol_custom_input.setPlaceholderText("Tolerance %")
        self.rotor_tol_custom_input.setFixedWidth(110)
        self.rotor_tol_custom_input.hide()
        tol_layout.addWidget(self.rotor_tol_combo, 1)
        tol_layout.addWidget(self.rotor_tol_custom_input, 0)
        rotor_layout.addWidget(tol_widget, rrow, 1)

        # Bar skew angle
        rrow += 1
        rotor_layout.addWidget(QLabel("Bar skew angle:"), rrow, 0)
        skew_widget = QWidget()
        skew_layout = QHBoxLayout(skew_widget)
        skew_layout.setContentsMargins(0, 0, 0, 0)
        self.rotor_skew_combo = QComboBox()
        self.rotor_skew_combo.addItem("0°", 0.0)
        self.rotor_skew_combo.addItem("Custom (10–30°)", "custom")
        self.rotor_skew_combo.currentIndexChanged.connect(self.on_rotor_skew_changed)
        self.rotor_skew_custom_input = QLineEdit()
        self.rotor_skew_custom_input.setPlaceholderText("10–30")
        self.rotor_skew_custom_input.setFixedWidth(110)
        self.rotor_skew_custom_input.hide()
        skew_layout.addWidget(self.rotor_skew_combo, 1)
        skew_layout.addWidget(self.rotor_skew_custom_input, 0)
        rotor_layout.addWidget(skew_widget, rrow, 1)

        # Bar material selection
        rrow += 1
        rotor_layout.addWidget(QLabel("Bar material:"), rrow, 0)
        self.rotor_bar_material_combo = QComboBox()
        self.rotor_bar_material_combo.addItems([
            "Cuivre électrolytique",
            "Aluminium pur",
            "Aluminium moulé (Al-Si)",
            "Laiton (Cu-Zn)",
            "Bronze (Cu-Sn)",
            "Custom",
        ])
        self.rotor_bar_material_combo.currentIndexChanged.connect(self.on_rotor_bar_material_changed)
        rotor_layout.addWidget(self.rotor_bar_material_combo, rrow, 1)

        rrow += 1
        self.rotor_bar_material_info = QLabel("")
        self.rotor_bar_material_info.setStyleSheet("color: gray; font-size: 9pt;")
        self.rotor_bar_material_info.setWordWrap(True)
        rotor_layout.addWidget(self.rotor_bar_material_info, rrow, 0, 1, 2)

        rrow += 1
        rotor_layout.addWidget(QLabel("Custom bar resistivity ρ (Ω·m):"), rrow, 0)
        self.rotor_bar_rho_custom_input = QLineEdit()
        self.rotor_bar_rho_custom_input.setPlaceholderText("e.g. 1.68e-8")
        self.rotor_bar_rho_custom_input.hide()
        rotor_layout.addWidget(self.rotor_bar_rho_custom_input, rrow, 1)

        # Ring current density Je
        rrow += 1
        rotor_layout.addWidget(QLabel("Ring current density Je:"), rrow, 0)
        je_widget = QWidget()
        je_layout = QHBoxLayout(je_widget)
        je_layout.setContentsMargins(0, 0, 0, 0)
        self.rotor_ring_je_combo = QComboBox()
        self.rotor_ring_je_combo.addItem("Use suggested Je", "suggested")
        self.rotor_ring_je_combo.addItem("Custom Je", "custom")
        self.rotor_ring_je_combo.currentIndexChanged.connect(self.on_rotor_ring_je_changed)
        self.rotor_ring_je_custom_input = QLineEdit()
        self.rotor_ring_je_custom_input.setPlaceholderText("Je (A/mm²)")
        self.rotor_ring_je_custom_input.setFixedWidth(110)
        self.rotor_ring_je_custom_input.hide()
        je_layout.addWidget(self.rotor_ring_je_combo, 1)
        je_layout.addWidget(self.rotor_ring_je_custom_input, 0)
        rotor_layout.addWidget(je_widget, rrow, 1)

        # Ring dimensions choice
        rrow += 1
        rotor_layout.addWidget(QLabel("Ring dimensions:"), rrow, 0)
        self.rotor_ring_dim_combo = QComboBox()
        self.rotor_ring_dim_combo.addItem("Typiques", "typical")
        self.rotor_ring_dim_combo.addItem("Manual", "manual")
        self.rotor_ring_dim_combo.addItem("Automatique", "auto")
        self.rotor_ring_dim_combo.currentIndexChanged.connect(self.on_rotor_ring_dim_changed)
        rotor_layout.addWidget(self.rotor_ring_dim_combo, rrow, 1)

        rrow += 1
        ring_manual_widget = QWidget()
        ring_manual_layout = QGridLayout(ring_manual_widget)
        ring_manual_layout.setContentsMargins(0, 0, 0, 0)
        ring_manual_layout.addWidget(QLabel("h_er (mm):"), 0, 0)
        self.rotor_ring_h_er_input = QLineEdit()
        self.rotor_ring_h_er_input.setPlaceholderText("h_er")
        ring_manual_layout.addWidget(self.rotor_ring_h_er_input, 0, 1)
        ring_manual_layout.addWidget(QLabel("b_er (mm):"), 1, 0)
        self.rotor_ring_b_er_input = QLineEdit()
        self.rotor_ring_b_er_input.setPlaceholderText("b_er")
        ring_manual_layout.addWidget(self.rotor_ring_b_er_input, 1, 1)
        ring_manual_widget.hide()
        self.rotor_ring_manual_widget = ring_manual_widget
        rotor_layout.addWidget(ring_manual_widget, rrow, 0, 1, 2)

        # Ring material selection
        rrow += 1
        rotor_layout.addWidget(QLabel("Ring material:"), rrow, 0)
        self.rotor_ring_material_combo = QComboBox()
        self.rotor_ring_material_combo.addItems([
            "Cuivre électrolytique",
            "Aluminium pur",
            "Aluminium moulé (Al-Si)",
            "Laiton (Cu-Zn)",
            "Bronze (Cu-Sn)",
            "Custom",
        ])
        self.rotor_ring_material_combo.currentIndexChanged.connect(self.on_rotor_ring_material_changed)
        rotor_layout.addWidget(self.rotor_ring_material_combo, rrow, 1)

        rrow += 1
        self.rotor_ring_material_info = QLabel("")
        self.rotor_ring_material_info.setStyleSheet("color: gray; font-size: 9pt;")
        self.rotor_ring_material_info.setWordWrap(True)
        rotor_layout.addWidget(self.rotor_ring_material_info, rrow, 0, 1, 2)

        rrow += 1
        rotor_layout.addWidget(QLabel("Custom ring resistivity ρ (Ω·m):"), rrow, 0)
        self.rotor_ring_rho_custom_input = QLineEdit()
        self.rotor_ring_rho_custom_input.setPlaceholderText("e.g. 2.82e-8")
        self.rotor_ring_rho_custom_input.hide()
        rotor_layout.addWidget(self.rotor_ring_rho_custom_input, rrow, 1)

        rotor_layout.setRowStretch(rrow + 1, 1)
        rotor_scroll_layout.addWidget(rotor_group)
        rotor_scroll_layout.addStretch(1)

        self.design_tab_widget.addTab(self.stator_tab, "Stator")
        self.design_tab_widget.addTab(self.rotor_tab, "Rotor")

        input_group = QGroupBox("Motor Specifications")
        input_layout = QGridLayout()
        # Keep the input panel compact to avoid horizontal scrolling.
        input_layout.setContentsMargins(0, 0, 0, 0)
        input_layout.setHorizontalSpacing(6)
        input_layout.setVerticalSpacing(6)
        input_group.setLayout(input_layout)
        input_group.setMaximumWidth(350)

        row = 0
        input_layout.addWidget(QLabel("Rated Power (kW):"), row, 0)
        self.power_input = QLineEdit("15.0")
        input_layout.addWidget(self.power_input, row, 1)

        row += 1
        input_layout.addWidget(QLabel("Line Voltage (V):"), row, 0)
        self.voltage_input = QLineEdit("400.0")
        input_layout.addWidget(self.voltage_input, row, 1)

        row += 1
        input_layout.addWidget(QLabel("Frequency (Hz):"), row, 0)
        self.frequency_input = QLineEdit("50.0")
        input_layout.addWidget(self.frequency_input, row, 1)

        row += 1
        input_layout.addWidget(QLabel("Number of Poles:"), row, 0)
        self.poles_combo = QComboBox()
        self.poles_combo.addItems(["2", "4", "6", "8", "10", "12"])
        self.poles_combo.setCurrentText("4")
        input_layout.addWidget(self.poles_combo, row, 1)

        # Cooling ducts (Ncc) - user input (integer 0..2)
        row += 1
        input_layout.addWidget(QLabel("Cooling ducts Ncc (0–2):"), row, 0)
        self.ncc_spinbox = QSpinBox()
        self.ncc_spinbox.setRange(0, 2)
        self.ncc_spinbox.setSingleStep(1)
        self.ncc_spinbox.setValue(0)
        self.ncc_spinbox.setToolTip("Number of radial cooling ducts (integer). Allowed values: 0, 1, 2.")
        input_layout.addWidget(self.ncc_spinbox, row, 1)

        row += 1
        input_layout.addWidget(QLabel("Stator Connection:"), row, 0)
        connection_widget = QWidget()
        connection_layout = QHBoxLayout(connection_widget)
        connection_layout.setContentsMargins(0, 0, 0, 0)
        self.connection_group = QButtonGroup()
        self.star_radio = QRadioButton("Star (Y)")
        self.delta_radio = QRadioButton("Delta (D)")
        self.star_radio.setChecked(True)
        self.connection_group.addButton(self.star_radio)
        self.connection_group.addButton(self.delta_radio)
        connection_layout.addWidget(self.star_radio)
        connection_layout.addWidget(self.delta_radio)
        input_layout.addWidget(connection_widget, row, 1)

        # Electrical steel grade selector
        row += 1
        input_layout.addWidget(QLabel("Electrical Steel Grade:"), row, 0)
        self.steel_combo = QComboBox()
        grades = get_steel_grades()
        self.steel_combo.addItems(grades)
        # Default to a sensible mid-grade if present
        default_grade = "M400-50A"
        if default_grade in grades:
            self.steel_combo.setCurrentText(default_grade)
        self.steel_combo.currentIndexChanged.connect(self.on_steel_changed)
        input_layout.addWidget(self.steel_combo, row, 1)

        row += 1
        self.steel_info_label = QLabel("")
        self.steel_info_label.setWordWrap(True)
        self.steel_info_label.setStyleSheet("color: gray; font-size: 8.5pt;")
        input_layout.addWidget(self.steel_info_label, row, 0, 1, 2)

        # Initialize steel info
        self.on_steel_changed(self.steel_combo.currentIndex())

        # Goal selection
        row += 1
        input_layout.addWidget(QLabel("D/L Selection Method:"), row, 0, 1, 2)
        row += 1
        self.method_combo = QComboBox()
        self.method_combo.addItem("Minimum Cost", "minimum_cost")
        self.method_combo.addItem("Good Efficiency", "good_efficiency")
        self.method_combo.addItem("Good Power Factor", "good_power_factor")
        self.method_combo.addItem("Balanced Design", "balanced_design")
        self.method_combo.currentIndexChanged.connect(self.on_method_changed)
        input_layout.addWidget(self.method_combo, row, 0, 1, 2)

        row += 1
        self.method_label = QLabel("Goal: Minimum Cost")
        self.method_label.setStyleSheet("color: gray; font-size: 9pt;")
        input_layout.addWidget(self.method_label, row, 0, 1, 2)

        row += 1
        self.method_lambda_label = QLabel("")
        self.method_lambda_label.setStyleSheet("color: gray; font-size: 9pt;")
        self.method_lambda_label.setWordWrap(True)
        input_layout.addWidget(self.method_lambda_label, row, 0, 1, 2)

        # (Custom lambda input moved to the "Use custom targets" checkbox section)

        # Slot design method selector
        row += 1
        input_layout.addWidget(QLabel("Slot Design Method:"), row, 0, 1, 2)
        row += 1
        # The slot method is auto-selected (LV vs HV/HP gating). Show it as a read-only text box.
        self.slot_method_edit = QLineEdit()
        self.slot_method_edit.setReadOnly(True)
        self.slot_method_edit.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        # Match the previous dropdown width.
        self.slot_method_edit.setFixedWidth(220)

        # Add an info icon to the right (tooltip explains options)
        self.slot_method_info_button = QToolButton()
        self.slot_method_info_button.setIcon(self.style().standardIcon(QStyle.StandardPixmap.SP_MessageBoxInformation))
        self.slot_method_info_button.setAutoRaise(True)
        self.slot_method_info_button.setCursor(Qt.CursorShape.WhatsThisCursor)
        self.slot_method_info_button.setFocusPolicy(Qt.FocusPolicy.NoFocus)

        slot_method_row = QWidget()
        slot_method_row_layout = QHBoxLayout(slot_method_row)
        slot_method_row_layout.setContentsMargins(0, 0, 0, 0)
        slot_method_row_layout.setSpacing(6)
        slot_method_row_layout.addWidget(self.slot_method_edit)
        slot_method_row_layout.addWidget(self.slot_method_info_button)
        slot_method_row_layout.addStretch(1)
        input_layout.addWidget(slot_method_row, row, 0, 1, 2)

        # Live gating of slot design method options based on Voltage/Power.
        # Condition: if Voltage > 600 V OR Power > 372.85 kW -> only Open slot is allowed.
        self.power_input.textChanged.connect(self.update_slot_method_options_based_on_inputs)
        self.voltage_input.textChanged.connect(self.update_slot_method_options_based_on_inputs)
        # Apply once at startup (based on default values)
        self.update_slot_method_options_based_on_inputs()

        # Preference checkboxes (hidden)
        row += 1
        self.preference_group = QGroupBox("Preferences (Method C)")
        pref_layout = QVBoxLayout()
        self.pref_min_cost = QCheckBox("Minimum Cost (1.5 - 2.0)")
        self.pref_efficiency = QCheckBox("Good Efficiency (1.4 - 1.6)")
        self.pref_pf = QCheckBox("Good Power Factor (1.0 - 1.3)")
        self.pref_balanced = QCheckBox("Balanced Design (1.0 - 1.1)")
        for cb in [self.pref_min_cost, self.pref_efficiency, self.pref_pf, self.pref_balanced]:
            cb.stateChanged.connect(self.on_preference_changed)
        pref_layout.addWidget(self.pref_min_cost)
        pref_layout.addWidget(self.pref_efficiency)
        pref_layout.addWidget(self.pref_pf)
        pref_layout.addWidget(self.pref_balanced)
        self.preference_group.setLayout(pref_layout)
        self.preference_group.hide()
        input_layout.addWidget(self.preference_group, row, 0, 1, 2)
        self.on_method_changed(self.method_combo.currentIndex())

        # Optional custom targets/overrides
        row += 1
        self.custom_targets_checkbox = QCheckBox("Use custom targets/inputs (optional)")
        self.custom_targets_checkbox.stateChanged.connect(self.on_custom_targets_toggled)
        input_layout.addWidget(self.custom_targets_checkbox, row, 0, 1, 2)

        row += 1
        self.custom_targets_group = QGroupBox("Custom Targets / Inputs")
        ct_layout = QGridLayout()
        self.custom_targets_group.setLayout(ct_layout)

        ct_row = 0
        ct_layout.addWidget(QLabel("Target efficiency η (0 = auto):"), ct_row, 0)
        self.target_efficiency_input = QLineEdit("0")
        ct_layout.addWidget(self.target_efficiency_input, ct_row, 1)

        ct_row += 1
        ct_layout.addWidget(QLabel("Target power factor cosφ (0 = auto):"), ct_row, 0)
        self.target_pf_input = QLineEdit("0")
        ct_layout.addWidget(self.target_pf_input, ct_row, 1)

        ct_row += 1
        ct_layout.addWidget(QLabel("Custom L/τ (0 = auto):"), ct_row, 0)
        self.override_lambda_input = QLineEdit("0")
        ct_layout.addWidget(self.override_lambda_input, ct_row, 1)

        ct_row += 1
        ct_layout.addWidget(QLabel("Specific magnetic loading B_av (T) (0 = auto):"), ct_row, 0)
        self.override_bav_input = QLineEdit("0")
        ct_layout.addWidget(self.override_bav_input, ct_row, 1)

        ct_row += 1
        ct_layout.addWidget(QLabel("Specific electric loading a_c (A/m) (0 = auto):"), ct_row, 0)
        self.override_ac_input = QLineEdit("0")
        ct_layout.addWidget(self.override_ac_input, ct_row, 1)

        ct_row += 1
        ct_layout.addWidget(QLabel("Slots per pole per phase q (0 = auto):"), ct_row, 0)
        self.override_q_input = QLineEdit("0")
        ct_layout.addWidget(self.override_q_input, ct_row, 1)

        input_layout.addWidget(self.custom_targets_group, row, 0, 1, 2)
        self.custom_targets_group.hide()

        # Separator
        row += 1
        separator = QWidget()
        separator.setFixedHeight(20)
        input_layout.addWidget(separator, row, 0, 1, 2)

        # Winding factor (K_w) is always computed from coil pitch selection.
        row += 1
        self.winding_type_label = QLabel("Winding Type:")
        self.winding_type_combo = QComboBox()
        # Custom winding factor is now computed (not selected from a lookup table).
        self.winding_type_combo.addItems([
            "full pitch",
            "short pitch (9/10)",
            "short pitch (5/6)",
            "short pitch (4/5)",
            "short pitch (2/3)",
        ])
        self.winding_type_combo.currentTextChanged.connect(self.update_coil_pitch_options)
        input_layout.addWidget(self.winding_type_label, row, 0)
        input_layout.addWidget(self.winding_type_combo, row, 1)

        row += 1
        self.coil_pitch_label = QLabel("Coil Pitch:")
        # Coil pitch is determined by winding type; display as a read-only text box.
        self.coil_pitch_edit = QLineEdit()
        self.coil_pitch_edit.setReadOnly(True)
        self.coil_pitch_edit.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.coil_pitch_edit.setFixedWidth(220)

        input_layout.addWidget(self.coil_pitch_label, row, 0)
        self._coil_pitch_row_container = QWidget()
        _coil_row_layout = QHBoxLayout(self._coil_pitch_row_container)
        _coil_row_layout.setContentsMargins(0, 0, 0, 0)
        _coil_row_layout.setSpacing(6)
        _coil_row_layout.addWidget(self.coil_pitch_edit, 1)
        input_layout.addWidget(self._coil_pitch_row_container, row, 1)

        # Ensure the coil pitch dropdown is populated on first launch.
        self.update_coil_pitch_options()

        row += 1
        self.kw_info_label = QLabel("K_w is computed from coil pitch and q (K_w = k_p × k_d)")
        self.kw_info_label.setStyleSheet("color: gray; font-size: 8pt;")

        # Restore the info icon in the winding-factor section (always visible)
        self.kw_guide_button = QToolButton()
        self.kw_guide_button.setIcon(self.style().standardIcon(QStyle.StandardPixmap.SP_MessageBoxInformation))
        self.kw_guide_button.setAutoRaise(True)
        self.kw_guide_button.setCursor(Qt.CursorShape.PointingHandCursor)
        self.kw_guide_button.setToolTip(
            "<b>COIL PITCH GUIDE:</b><br>"
            "&bull; <b>RECOMMENDED:</b> 5/6 pitch (83.3%) for all AC motors<br>"
            "&bull; <b>Why?</b> Eliminates 5th harmonic, minimal torque loss (3.4%)<br>"
            "&bull; <b>Alternatives:</b><br>"
            "&nbsp;&nbsp;- 9/10 pitch (90%): Slight torque loss, reduces harmonics<br>"
            "&nbsp;&nbsp;- 4/5 pitch (80%): Better harmonic reduction, more torque loss<br>"
            "&nbsp;&nbsp;- Full pitch (100%): Maximum torque, but higher harmonics<br>"
            "&nbsp;&nbsp;- &lt;75% pitch: Avoid - excessive torque loss."
        )
        self.kw_guide_button.clicked.connect(self.show_coil_pitch_guide)

        self._kw_info_row_container = QWidget()
        _kw_info_layout = QHBoxLayout(self._kw_info_row_container)
        _kw_info_layout.setContentsMargins(0, 0, 0, 0)
        _kw_info_layout.setSpacing(6)
        _kw_info_layout.addWidget(self.kw_info_label)
        _kw_info_layout.addStretch(1)
        _kw_info_layout.addWidget(self.kw_guide_button)
        input_layout.addWidget(self._kw_info_row_container, row, 0, 1, 2)

        # Stator conductor material selection (affects resistivity and conductor weight)
        row += 1
        self.stator_conductor_material_label = QLabel("Stator conductor material:")
        input_layout.addWidget(self.stator_conductor_material_label, row, 0)
        self.stator_conductor_material_combo = QComboBox()
        self.stator_conductor_material_combo.addItems([
            "Cuivre électrolytique",
            "Aluminium pur",
        ])
        # Requirement: lock stator conductor material to Copper (user cannot change it).
        # Keep the widget/value so the rest of the program (losses tab, resistivity mapping, etc.)
        # can still read the selected material.
        self.stator_conductor_material_combo.setCurrentText("Cuivre électrolytique")
        self.stator_conductor_material_combo.setEnabled(False)
        self.stator_conductor_material_combo.setToolTip("Fixed to copper (Cuivre électrolytique).")
        self.stator_conductor_material_combo.currentIndexChanged.connect(self.on_stator_conductor_material_changed)
        input_layout.addWidget(self.stator_conductor_material_combo, row, 1)

        row += 1
        self.stator_conductor_material_info = QLabel("")
        self.stator_conductor_material_info.setWordWrap(True)
        self.stator_conductor_material_info.setStyleSheet("color: gray; font-size: 8.5pt;")
        input_layout.addWidget(self.stator_conductor_material_info, row, 0, 1, 2)

        # Hide the material selection UI (still used internally for rho/density & losses)
        self.stator_conductor_material_label.setVisible(False)
        self.stator_conductor_material_combo.setVisible(False)
        self.stator_conductor_material_info.setVisible(False)

        # Initialize conductor material info
        self.on_stator_conductor_material_changed(self.stator_conductor_material_combo.currentIndex())

        # NOTE: action buttons (Calculate/2D) live in a fixed actions bar below the tabs.

        input_layout.setRowStretch(row + 1, 1)
        stator_scroll_layout.addWidget(input_group)
        stator_scroll_layout.addStretch(1)

        left_panel_layout.addWidget(self.design_tab_widget, 1)

        # Fixed actions bar (always visible)
        actions_box = QGroupBox("Actions")
        actions_layout = QVBoxLayout(actions_box)
        actions_layout.setContentsMargins(10, 10, 10, 10)
        actions_layout.setSpacing(8)

        self.calc_stator_button = QPushButton("Calculate Stator Design")
        self.calc_stator_button.setFont(self.heading_font)
        self.calc_stator_button.setProperty("role", "primary")
        self.calc_stator_button.clicked.connect(self.calculate_design)
        actions_layout.addWidget(self.calc_stator_button)

        self.calc_rotor_button = QPushButton("Calculate Rotor Design")
        self.calc_rotor_button.setProperty("role", "primary")
        self.calc_rotor_button.clicked.connect(self.calculate_rotor_design)
        actions_layout.addWidget(self.calc_rotor_button)

        self.show_2d_view_button = QPushButton("Show the 2d view of the machine")
        self._show_2d_view_button_base_text = self.show_2d_view_button.text()
        self.show_2d_view_button.setProperty("role", "primary")
        self.show_2d_view_button.clicked.connect(self.show_2d_view_page)
        self.show_2d_view_button.setEnabled(False)
        actions_layout.addWidget(self.show_2d_view_button)

        left_panel_layout.addWidget(actions_box, 0)

        main_layout.addWidget(left_panel, 0)

        # Right panel
        right_panel = QWidget()
        right_layout = QVBoxLayout(right_panel)
        right_layout.setContentsMargins(0, 0, 0, 0)

        details_group = QGroupBox("Detailed Calculations with Equations")
        details_layout = QVBoxLayout()
        details_group.setLayout(details_layout)
        self.tab_widget = QTabWidget()
        self.tab_empirical = self.create_text_edit_tab()
        self.tab_electrical = self.create_text_edit_tab()
        self.tab_dimensions = self.create_text_edit_tab()
        self.tab_winding = self.create_text_edit_tab()
        self.tab_slots = self.create_text_edit_tab()
        self.tab_outer = self.create_text_edit_tab()
        self.tab_losses = self.create_text_edit_tab()
        self.tab_rotor = self.create_text_edit_tab()
        self.tab_efficiency = self.create_text_edit_tab()
        self.tab_widget.addTab(self.tab_empirical, "Empirical")
        self.tab_widget.addTab(self.tab_electrical, "Electrical")
        self.tab_widget.addTab(self.tab_dimensions, "Main Dimensions")
        self.tab_widget.addTab(self.tab_winding, "Winding")
        self.tab_widget.addTab(self.tab_slots, "Slots")
        self.tab_widget.addTab(self.tab_outer, "Outer Diameter")
        self.tab_widget.addTab(self.tab_losses, "Losses")
        self.tab_widget.addTab(self.tab_rotor, "Rotor")
        # Hidden until both stator + rotor are computed.
        self._set_efficiency_tab_visible(False)
        details_layout.addWidget(self.tab_widget)
        right_layout.addWidget(details_group, 3)

        main_layout.addWidget(right_panel, 1)

        # Initialize rotor UI dynamic states
        self._rotor_nr_candidates = []
        self.on_rotor_tol_changed(self.rotor_tol_combo.currentIndex())
        self.on_rotor_skew_changed(self.rotor_skew_combo.currentIndex())
        self.on_rotor_bar_material_changed(self.rotor_bar_material_combo.currentIndex())
        self.on_rotor_ring_je_changed(self.rotor_ring_je_combo.currentIndex())
        self.on_rotor_ring_dim_changed(self.rotor_ring_dim_combo.currentIndex())
        self.on_rotor_ring_material_changed(self.rotor_ring_material_combo.currentIndex())
        self.on_rotor_nr_method_changed(self.rotor_nr_method_combo.currentIndex())

    def show_coil_pitch_guide(self):
        """Show the coil pitch guide (info icon next to coil pitch selection)."""
        QMessageBox.information(
            self,
            "Coil Pitch Guide",
            "COIL PITCH GUIDE:\n"
            "• RECOMMENDED: 5/6 pitch (83.3%) for all AC motors\n"
            "• Why? Eliminates 5th harmonic, minimal torque loss (3.4%)\n"
            "• Alternatives:\n"
            "  - 9/10 pitch (90%): Slight torque loss, reduces harmonics\n"
            "  - 4/5 pitch (80%): Better harmonic reduction, more torque loss\n"
            "  - Full pitch (100%): Maximum torque, but higher harmonics\n"
            "  - <75% pitch: Avoid - excessive torque loss.\n"
        )

    def update_coil_pitch_options(self):
        """Update coil pitch display based on selected winding type."""
        winding_type = self.winding_type_combo.currentText()

        # Keep the 2nd menu in sync with the winding type.
        # Requested additions:
        # - 4/5 pitch  -> 80%
        # - 9/10 pitch -> 90%
        wt = (winding_type or "").lower()
        if "9/10" in wt:
            coil_pitches = [90]
        elif "4/5" in wt:
            coil_pitches = [80]
        elif "5/6" in wt:
            coil_pitches = [83.3]
        elif "2/3" in wt:
            coil_pitches = [66.7]
        else:
            # full pitch (default)
            coil_pitches = [100]
        cp = float(coil_pitches[0]) if coil_pitches else 100.0
        if hasattr(self, 'coil_pitch_edit') and isinstance(self.coil_pitch_edit, QLineEdit):
            # Show as percentage string.
            self.coil_pitch_edit.setText(f"{cp:g}%")

    def _get_coil_pitch_percent(self) -> float:
        """Return coil pitch percent as a float (e.g., 83.3)."""
        try:
            raw = ""
            if hasattr(self, 'coil_pitch_edit') and isinstance(self.coil_pitch_edit, QLineEdit):
                raw = self.coil_pitch_edit.text()
            raw = (raw or "").replace("%", "").strip()
            return float(raw) if raw else 100.0
        except Exception:
            return 100.0

    def on_method_changed(self, index: int):
        """Handle changes to goal selection dropdown"""
        method = self.method_combo.currentData()
        labels = {
            'minimum_cost': "Goal: Minimum Cost",
            'good_efficiency': "Goal: Good Efficiency",
            'good_power_factor': "Goal: Good Power Factor",
            'balanced_design': "Goal: Balanced Design",
        }
        self.method_label.setText(labels.get(method, "Goal: Minimum Cost"))

        # Display possible L/pole pitch values (lambda = L/τ_pole) for the selected goal
        if hasattr(self, 'method_lambda_label'):
            if method == 'good_power_factor':
                self.method_lambda_label.setText("L/τ (L / pole pitch): heuristic (no predefined range)")
            else:
                lam_range = getattr(self, 'preference_ranges', {}).get(method)
                if lam_range and len(lam_range) == 2:
                    self.method_lambda_label.setText(
                        f"L/τ (L / pole pitch) range: [{lam_range[0]:.2f} - {lam_range[1]:.2f}]"
                    )
                else:
                    self.method_lambda_label.setText("L/τ (L / pole pitch) range: N/A")
        if hasattr(self, 'preference_group'):
            self.preference_group.hide()

    def _set_slot_method_ui(self, method_key: str | None):
        """Set slot method UI key and update the read-only display."""
        self._slot_method_ui = method_key
        label_map = {v: l for (l, v) in (getattr(self, '_slot_method_options_all', None) or [])}
        display = label_map.get(method_key, "")
        if hasattr(self, 'slot_method_edit') and isinstance(self.slot_method_edit, QLineEdit):
            self.slot_method_edit.setText(display)
        self._update_slot_method_info_tooltip()

    def _safe_float_from_lineedit(self, edit: QLineEdit) -> float | None:
        """Parse float from a QLineEdit; return None if invalid/empty."""
        try:
            raw = edit.text().strip()
            if raw == "":
                return None
            return float(raw)
        except Exception:
            return None

    def update_slot_method_options_based_on_inputs(self):
        """Dynamically restrict slot method options based on live user inputs.

                Rules:
                - HV/HP machines (Voltage > 600 V OR Power > 372.85 kW): only Open slot is available,
                    and the dropdown is locked (cannot be opened/changed).
                - LV machines (Voltage <= 600 V AND Power <= 372.85 kW): keep the dropdown available,
                    but disable Semi-open/Open choices so only the trapezoidal semi-open slot can be used.
        """
        # Inputs may be temporarily invalid while typing; treat invalid as 'no restriction'.
        v_line = self._safe_float_from_lineedit(self.voltage_input) if hasattr(self, 'voltage_input') else None
        p_kw = self._safe_float_from_lineedit(self.power_input) if hasattr(self, 'power_input') else None

        is_hv_hp = (v_line is not None and v_line > 600.0) or (p_kw is not None and p_kw > 372.85)

        # Store gating state and update the displayed (read-only) selection.
        self._slot_method_gated_open_only = bool(is_hv_hp)
        target_key = 'open_slot' if is_hv_hp else 'tapered_slot'
        self._set_slot_method_ui(target_key)

    def _update_slot_method_info_tooltip(self):
        """Update the Slot Design Method info tooltip based on LV vs HV/HP gating."""
        if not hasattr(self, 'slot_method_info_button'):
            return

        v_line = self._safe_float_from_lineedit(self.voltage_input) if hasattr(self, 'voltage_input') else None
        p_kw = self._safe_float_from_lineedit(self.power_input) if hasattr(self, 'power_input') else None
        is_hv_hp = (v_line is not None and v_line > 600.0) or (p_kw is not None and p_kw > 372.85)

        if is_hv_hp:
            tooltip = "this is best shape since those machines use form wound rectangular winding."
        else:
            tooltip = (
                "Trapezoidal semi-open slot (LV machines):\n"
                "Reduced harmonics, torque ripple, and noise.\n"
                "Better magnetic performance (higher efficiency), harder to wind.\n\n"
                "This is the optimal slot shape for high efficiency."
            )

        self.slot_method_info_button.setToolTip(tooltip)

    def on_steel_changed(self, index: int):
        """Handle electrical steel grade selection change."""
        grade = self.steel_combo.currentText().strip() if hasattr(self, 'steel_combo') else ""
        data = get_steel_by_grade(grade)
        self.selected_steel_data = data

        if not data:
            self.steel_info_label.setText("No steel data selected")
            return

        thickness = data.get("Thickness_mm")
        max_b = data.get("Max_Design_B")
        loss = data.get("Loss_W_kg")
        app = data.get("Application")
        self.steel_info_label.setText(
            f"Thickness: {thickness:.2f} mm | Max Design B: {max_b:.2f} T | Loss @1.5T: {loss:.2f} W/kg\n"
            f"Application: {app}"
        )

    def on_custom_targets_toggled(self, state: int):
        """Show/hide the custom targets group."""
        checked = state == Qt.CheckState.Checked.value
        if hasattr(self, 'custom_targets_group'):
            self.custom_targets_group.setVisible(checked)

    def get_selected_preferences(self) -> Dict[str, bool]:
        """Collect preference flags for method C"""
        return {
            'minimum_cost': getattr(self, 'pref_min_cost', None) and self.pref_min_cost.isChecked(),
            'good_efficiency': getattr(self, 'pref_efficiency', None) and self.pref_efficiency.isChecked(),
            'good_power_factor': getattr(self, 'pref_pf', None) and self.pref_pf.isChecked(),
            'balanced_design': getattr(self, 'pref_balanced', None) and self.pref_balanced.isChecked(),
        }

    def on_preference_changed(self, state: int):
        """Enforce preference selection rules (currently hidden)"""
        selected = [cb for cb in [self.pref_min_cost, self.pref_efficiency, self.pref_pf, self.pref_balanced] if cb.isChecked()]
        if len(selected) > 2:
            sender = self.sender()
            if sender and sender.isChecked():
                sender.setChecked(False)

    def validate_inputs(self) -> bool:
        """Validate user inputs"""
        try:
            power = float(self.power_input.text())
            voltage = float(self.voltage_input.text())
            frequency = float(self.frequency_input.text())
            poles = int(self.poles_combo.currentText())

            if power <= 0 or voltage <= 0 or frequency <= 0 or poles <= 0:
                QMessageBox.critical(self, "Input Error", "All values must be positive numbers")
                return False

            if poles % 2 != 0:
                QMessageBox.critical(self, "Input Error", "Number of poles must be even")
                return False

            return True

        except ValueError:
            QMessageBox.critical(self, "Input Error", "Please enter valid numeric values")
            return False

    def create_text_edit_tab(self) -> QTextEdit:
        """Create a QTextEdit configured for display"""
        edit = QTextEdit()
        edit.setFont(self.equation_font)
        edit.setReadOnly(True)
        return edit

    def clear_all_tabs(self):
        """Clear text from all tabs"""
        tabs = [self.tab_empirical, self.tab_electrical, self.tab_dimensions, self.tab_winding, self.tab_slots, self.tab_outer, self.tab_losses]
        if hasattr(self, 'tab_rotor'):
            tabs.append(self.tab_rotor)
        if hasattr(self, 'tab_efficiency'):
            tabs.append(self.tab_efficiency)
        for tab in tabs:
            tab.clear()

    
    def make_header(self, text: str, width: int = 80) -> str:
        """Create a text header with borders"""
        line = "=" * width
        return f"{line}\n{text.center(width)}\n{line}\n\n"
    
    def make_separator(self, width: int = 80) -> str:
        """Create a separator line"""
        return "-" * width + "\n"

    def format_equation(self, left: str, right: str, width: int = 55) -> str:
        """Format an equation-style line with aligned result"""
        return f"    {left.ljust(width)} = {right}\n"

    def append_to_tab(self, tab: QTextEdit, text: str):
        """Append formatted text to a QTextEdit tab"""
        tab.clear()
        tab.setPlainText(text)
        tab.moveCursor(QTextCursor.MoveOperation.Start)
    
    def calculate_design(self):
        """Main calculation function"""
        if not self.validate_inputs():
            return
        
        self.status_bar.showMessage("Calculating design...")
        QApplication.processEvents()
        
        # Clear previous results
        self.clear_all_tabs()
        
        try:
            # Get connection type
            connection = "star" if self.star_radio.isChecked() else "delta"
            goal = self.method_combo.currentData()

            # Optional overrides (0 means: use normal/auto selection)
            overrides_enabled = hasattr(self, 'custom_targets_checkbox') and self.custom_targets_checkbox.isChecked()
            target_efficiency = 0.0
            target_pf = 0.0
            override_lambda = 0.0
            override_bav = 0.0
            override_ac = 0.0
            override_q = 0.0

            if overrides_enabled:
                try:
                    target_efficiency = float(self.target_efficiency_input.text())
                    target_pf = float(self.target_pf_input.text())
                    override_lambda = float(self.override_lambda_input.text())
                    override_bav = float(self.override_bav_input.text())
                    override_ac = float(self.override_ac_input.text())
                    override_q = float(self.override_q_input.text())
                except Exception:
                    QMessageBox.critical(self, "Input Error", "Please enter valid numeric values for custom targets/inputs (use 0 to keep auto)")
                    return

                # Basic validation (only when user provided a non-zero override)
                if target_efficiency != 0.0 and not (0.0 < target_efficiency <= 1.0):
                    QMessageBox.critical(self, "Input Error", "Target efficiency η must be between 0 and 1 (use 0 for auto)")
                    return
                if target_pf != 0.0 and not (0.0 < target_pf <= 1.0):
                    QMessageBox.critical(self, "Input Error", "Target power factor cosφ must be between 0 and 1 (use 0 for auto)")
                    return
                if override_lambda != 0.0 and override_lambda <= 0.0:
                    QMessageBox.critical(self, "Input Error", "Custom L/τ must be positive (use 0 for auto)")
                    return
                if override_bav != 0.0 and override_bav <= 0.0:
                    QMessageBox.critical(self, "Input Error", "B_av must be positive (use 0 for auto)")
                    return
                if override_ac != 0.0 and override_ac <= 0.0:
                    QMessageBox.critical(self, "Input Error", "a_c must be positive (use 0 for auto)")
                    return
                if override_q != 0.0 and override_q < 1.0:
                    QMessageBox.critical(self, "Input Error", "q must be >= 1 (use 0 for auto)")
                    return

            # Map goal to backend method and preference flags
            if goal == 'good_power_factor':
                dimension_method = 'best_pf'  # heuristic D relation
                preference_flags = None
            else:
                dimension_method = 'preference_based'
                preference_flags = {
                    'minimum_cost': goal == 'minimum_cost',
                    'good_efficiency': goal == 'good_efficiency',
                    'good_power_factor': False,
                    'balanced_design': goal == 'balanced_design',
                }

            # Stator conductor material (affects rho used in R_s / copper losses, and density used in conductor weight)
            conductor_material = (
                self.stator_conductor_material_combo.currentText().strip()
                if hasattr(self, 'stator_conductor_material_combo') else
                "Cuivre électrolytique"
            )
            conductor_rho_map = {
                "Cuivre électrolytique": 1.68e-8,
                "Aluminium pur": 2.82e-8,
            }
            conductor_density_map_g_cm3 = {
                "Cuivre électrolytique": 8.9,
                "Aluminium pur": 2.7,
            }
            conductor_rho = float(conductor_rho_map.get(conductor_material, 1.68e-8) or 1.68e-8)
            conductor_density_g_cm3 = float(conductor_density_map_g_cm3.get(conductor_material, 8.9) or 8.9)

            # Create motor specifications
            slot_method_backend = getattr(self, '_slot_method_ui', None)
            # Both UI options map to the same backend implementation for now.
            if slot_method_backend in ('tapered_slot',):
                slot_method_backend = 'semi_open_slot'
            specs = MotorSpecifications(
                power_kw=float(self.power_input.text()),
                voltage_v=float(self.voltage_input.text()),
                frequency_hz=float(self.frequency_input.text()),
                poles=int(self.poles_combo.currentText()),
                connection=connection,
                # Always compute K_w from coil pitch selection (no fixed default K_w).
                custom_winding_factor=True,
                winding_type=self.winding_type_combo.currentText(),
                coil_pitch_percent=float(self._get_coil_pitch_percent()),
                dimension_method=dimension_method,
                preference_flags=preference_flags,
                target_efficiency=(target_efficiency if overrides_enabled and target_efficiency != 0.0 else None),
                target_power_factor=(target_pf if overrides_enabled and target_pf != 0.0 else None),
                override_lambda_ratio=(override_lambda if overrides_enabled and override_lambda != 0.0 else None),
                override_Bav=(override_bav if overrides_enabled and override_bav != 0.0 else None),
                override_ac=(override_ac if overrides_enabled and override_ac != 0.0 else None),
                override_q=(override_q if overrides_enabled and override_q != 0.0 else None),
                slot_method=slot_method_backend,
                steel_grade=self.steel_combo.currentText().strip() if hasattr(self, 'steel_combo') else None,
                steel_data=self.selected_steel_data,
                conductor_material=conductor_material,
                conductor_rho_ohm_m=conductor_rho,
                conductor_density_g_cm3=conductor_density_g_cm3,
                override_num_cooling_canals=int(self.ncc_spinbox.value()) if hasattr(self, 'ncc_spinbox') else 0,
            )
            
            # Create designer
            self.designer = StatorDesign(specs)
            
            # Execute design phases with equation display
            self.show_empirical_parameters()
            self.show_electrical_calculations()
            self.show_main_dimensions()
            self.show_winding_design()
            self.show_slot_design()
            self.show_outer_diameter()
            self.show_losses_resistance()
            
            # Display summary
            self.display_summary()
            # Cache summary to allow export without recalculation
            self.results = self.designer.get_summary()

            # Any previous rotor results are now stale (they depend on stator inputs)
            self.rotor_results = None
            if hasattr(self, 'tab_rotor'):
                self.tab_rotor.clear()
            self._update_2d_view_from_current_rotor()

            # Cache stator-derived inputs for rotor tab/calculation
            self.stator_to_rotor_inputs = self.get_rotor_inputs_from_stator()

            # Unlock rotor UI once stator design is successfully completed
            self.stator_design_done = True
            self.set_rotor_ui_enabled(True)

            # 2D view button becomes available after stator design
            self.update_2d_view_button_visibility()

            # Refresh cached 2D view inputs for the newly computed stator
            self._update_2d_view_from_current_design()

            # Export is still not available until rotor design is computed
            self.update_export_button_visibility()
            self.update_project_button_visibility()

            # Populate rotor Nr options now that Ss/P are known and rotor gating is lifted
            if hasattr(self, 'rotor_nr_method_combo'):
                self.on_rotor_nr_method_changed(self.rotor_nr_method_combo.currentIndex())
            
            self.status_bar.showMessage("Design completed successfully!")
            QMessageBox.information(self, "Success", "Stator design completed successfully!")
            
        except Exception as e:
            import traceback
            error_details = traceback.format_exc()
            QMessageBox.critical(self, "Calculation Error", 
                               f"An error occurred:\n{str(e)}\n\nDetails:\n{error_details}")
            self.status_bar.showMessage("Error in calculation")

            # Lock rotor/export if stator calculation failed
            self.stator_design_done = False
            self.set_rotor_ui_enabled(False)
            self.rotor_results = None
            self._update_2d_view_from_current_rotor()
            self.update_export_button_visibility()
            self.update_2d_view_button_visibility()
            self.update_project_button_visibility()

    def get_rotor_inputs_from_stator(self) -> Dict[str, float]:
        """Build the first-category rotor inputs from the completed stator design.

        Rotor script expects (mostly) mm units for lengths and Tesla for inductions.
        """
        if self.designer is None or self.designer.geometry is None or self.designer.electrical is None:
            raise ValueError("Stator design results are not available")

        specs = self.designer.specs
        geom = self.designer.geometry
        elec = self.designer.electrical
        slots = self.designer.slots

        poles_total = int(getattr(specs, 'poles', 0) or 0)
        if poles_total <= 0 or poles_total % 2 != 0:
            raise ValueError("Invalid stator pole count")

        # P in rotor script = pole pairs
        P_pairs = poles_total // 2

        Ss = int(getattr(slots, 'Ss', 0) or 0) if slots is not None else 0
        Nc = int(getattr(slots, 'Nc', 0) or 0) if slots is not None else 0
        if Ss <= 0 or Nc <= 0:
            raise ValueError("Stator slot results not available (Ss/Nc)")

        # Length units: stator module uses meters; rotor script uses mm
        D_mm = float(getattr(geom, 'D', 0.0) or 0.0) * 1000.0
        L_mm = float(getattr(geom, 'L', 0.0) or 0.0) * 1000.0
        Ls_mm = float(getattr(geom, 'Ls', 0.0) or 0.0) * 1000.0
        ki = float(getattr(geom, 'ki', 0.95) or 0.95)

        # Net iron length (effective magnetic length): prefer Li_new if flux correction updated lengths.
        geom_val = geom.validation_info if hasattr(geom, 'validation_info') and geom.validation_info else {}
        Li_m = float(geom_val.get('Li_new', geom_val.get('Li', getattr(geom, 'Li', 0.0))) or 0.0)
        if Li_m <= 0:
            # Fallback consistent with stator definitions
            Ls_m = float(getattr(geom, 'Ls', 0.0) or 0.0)
            Li_m = (Ls_m * ki) if (Ls_m > 0 and ki > 0) else 0.0
        Li_mm = Li_m * 1000.0

        if D_mm <= 0 or L_mm <= 0 or Ls_mm <= 0:
            raise ValueError("Invalid stator geometry (D/L/Ls)")

        # Electrical
        Iph_A = float(getattr(elec, 'I_s', 0.0) or 0.0)
        if Iph_A <= 0:
            raise ValueError("Invalid stator phase current")

        # Magnetic
        flux_per_pole_Wb = float(getattr(geom, 'flux_per_pole', 0.0) or 0.0)
        if flux_per_pole_Wb <= 0:
            raise ValueError("Invalid flux per pole")

        # B_tooth: use computed tooth flux density if available, else fallback to target
        B_tooth_T = 0.0
        if slots is not None:
            try:
                B_tooth_T = float(getattr(slots, 'B_tooth', 0.0) or 0.0)
            except Exception:
                B_tooth_T = 0.0
        if B_tooth_T <= 0:
            targets = getattr(self.designer, '_forward_design_targets', None) or {}
            try:
                B_tooth_T = float(targets.get('B_tooth_target', 0.0) or 0.0)
            except Exception:
                B_tooth_T = 0.0

        # B_core/yoke: use the forward design target if available
        B_yoke_T = 0.0
        targets = getattr(self.designer, '_forward_design_targets', None) or {}
        try:
            B_yoke_T = float(targets.get('B_yoke_target', 0.0) or 0.0)
        except Exception:
            B_yoke_T = 0.0

        power_kw = float(getattr(specs, 'power_kw', 0.0) or 0.0)

        # Stator winding factor passed to rotor script (Kws):
        # Must match the actual K_w used in stator computations.
        try:
            Kws_raw = getattr(self.designer, '_K_w', None)
            Kws = float(Kws_raw) if Kws_raw is not None else 0.0
        except Exception:
            Kws = 0.0

        return {
            # naming aligned to your description
            'P_pairs': P_pairs,
            'Ss': Ss,              # rotor script calls it Ns
            'D_mm': D_mm,
            'L_mm': L_mm,
            'Ls_mm': Ls_mm,
            'Li_mm': Li_mm,
            'ki': ki,
            'Iph_A': Iph_A,
            'Nc': Nc,              # rotor script calls it Zs (conducteurs/encoche)
            'flux_per_pole_Wb': flux_per_pole_Wb,
            'B_tooth_T': B_tooth_T,
            'B_yoke_T': B_yoke_T,
            'power_kw': power_kw,
            'Kws': Kws,
        }

    def _nr_candidates_classique(self, Ss: int, P_pairs: int) -> list:
        """Return list of tuples (Nr, Ss_over_Nr, Ss_minus_Nr) for the 'classique' method."""
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

        recs = []
        for ratio in ratios_possibles:
            try:
                Nr_cand = int(round(Ns / float(ratio)))
            except Exception:
                continue
            if Nr_cand <= 0:
                continue
            diff = Ns - Nr_cand
            ratio_actuel = (Ns / Nr_cand) if Nr_cand else 0.0

            if diff in conditions_a_eviter:
                continue
            if poles in (2, 4) and abs(ratio_actuel - 2 / 3) < 0.01:
                continue
            if poles == 2 and abs(ratio_actuel - 1.0) < 0.01:
                continue
            if abs(diff) < 1:
                continue

            recs.append((Nr_cand, float(ratio_actuel), int(diff)))

        # De-duplicate on Nr
        unique = {}
        for Nr, r, d in recs:
            if Nr not in unique:
                unique[Nr] = (Nr, r, d)
        out = sorted(unique.values(), key=lambda x: x[0])
        return out

    def _nr_candidates_harmonique(self, Ss: int, P_pairs: int) -> list:
        """Return list of tuples (Nr, Ss_over_Nr, Ss_minus_Nr) for the 'harmonique' method."""
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

        recs = []
        for Nr in range(min_Nr, max_Nr + 1):
            if Nr in valeurs_a_eviter:
                continue

            est_multiple = False
            for k in range(1, 10):
                if k * Ns == Nr:
                    est_multiple = True
                    break
                if k != 0 and Ns % k == 0 and Nr == Ns // k:
                    est_multiple = True
                    break
            if est_multiple:
                continue

            rapport = (Ns / Nr) if Nr else 0.0
            if 0.6 <= rapport <= 1.4:
                recs.append((int(Nr), float(rapport), int(Ns - Nr)))

        recs.sort(key=lambda x: abs(x[2]))
        return recs

    def populate_rotor_nr_options(self):
        """Populate rotor Nr dropdown and slider based on current method and stator results."""
        if not hasattr(self, 'rotor_nr_method_combo') or not hasattr(self, 'rotor_nr_combo'):
            return

        if not getattr(self, 'stator_design_done', False) or not self.stator_to_rotor_inputs:
            self._rotor_nr_candidates = []
            self.rotor_nr_combo.blockSignals(True)
            self.rotor_nr_combo.clear()
            self.rotor_nr_combo.addItem("Custom", "custom")
            self.rotor_nr_combo.blockSignals(False)
            self.rotor_nr_slider.setMinimum(0)
            self.rotor_nr_slider.setMaximum(0)
            self.rotor_nr_slider.setValue(0)
            self.rotor_nr_slider.setEnabled(False)
            if hasattr(self, 'rotor_nr_info_label'):
                self.rotor_nr_info_label.setText("Run stator design to populate Nr options.")
            return

        Ss = int(self.stator_to_rotor_inputs.get('Ss', 0) or 0)
        P_pairs = int(self.stator_to_rotor_inputs.get('P_pairs', 0) or 0)
        if Ss <= 0 or P_pairs <= 0:
            return

        method = self.rotor_nr_method_combo.currentData()
        if method == 'harmonique':
            recs = self._nr_candidates_harmonique(Ss=Ss, P_pairs=P_pairs)
        else:
            recs = self._nr_candidates_classique(Ss=Ss, P_pairs=P_pairs)

        # Limit to what the script shows (top 10)
        recs = list(recs[:10])
        candidates = [int(r[0]) for r in recs]
        self._rotor_nr_candidates = candidates

        self.rotor_nr_combo.blockSignals(True)
        self.rotor_nr_combo.clear()
        for Nr, ratio, diff in recs:
            self.rotor_nr_combo.addItem(f"Nr={Nr} | Ss/Nr={ratio:.3f} | Ss-Nr={diff}", int(Nr))
        self.rotor_nr_combo.addItem("Custom", "custom")
        self.rotor_nr_combo.setCurrentIndex(0 if recs else self.rotor_nr_combo.count() - 1)
        self.rotor_nr_combo.blockSignals(False)

        if candidates:
            self.rotor_nr_slider.blockSignals(True)
            self.rotor_nr_slider.setMinimum(0)
            self.rotor_nr_slider.setMaximum(max(0, len(candidates) - 1))
            self.rotor_nr_slider.setValue(0)
            self.rotor_nr_slider.setEnabled(True)
            self.rotor_nr_slider.blockSignals(False)
            if hasattr(self, 'rotor_nr_info_label'):
                self.rotor_nr_info_label.setText(f"{len(candidates)} candidate(s) loaded for method '{method}'.")
        else:
            self.rotor_nr_slider.setMinimum(0)
            self.rotor_nr_slider.setMaximum(0)
            self.rotor_nr_slider.setValue(0)
            self.rotor_nr_slider.setEnabled(False)
            if hasattr(self, 'rotor_nr_info_label'):
                self.rotor_nr_info_label.setText("No Nr candidates found; use Custom.")

        self.on_rotor_nr_choice_changed(self.rotor_nr_combo.currentIndex())

    def set_rotor_ui_enabled(self, enabled: bool):
        """Enable/disable rotor tab + rotor button (locked until stator is done)."""
        if hasattr(self, 'calc_rotor_button'):
            self.calc_rotor_button.setEnabled(bool(enabled))
            self.calc_rotor_button.setToolTip("" if enabled else "Locked: calculate stator first.")
        if hasattr(self, 'design_tab_widget') and hasattr(self, 'rotor_tab'):
            rotor_index = self.design_tab_widget.indexOf(self.rotor_tab)
            if rotor_index != -1:
                self.design_tab_widget.setTabEnabled(rotor_index, bool(enabled))
                self.design_tab_widget.setTabText(rotor_index, "Rotor" if enabled else "Rotor (locked)")
                self.design_tab_widget.setTabToolTip(
                    rotor_index,
                    "" if enabled else "Locked: calculate the stator design first to unlock the rotor tab."
                )

    def on_design_tab_changed(self, index: int):
        """Prevent navigating to rotor tab unless stator design is complete."""
        if not hasattr(self, 'design_tab_widget') or not hasattr(self, 'rotor_tab'):
            return
        rotor_index = self.design_tab_widget.indexOf(self.rotor_tab)
        stator_index = self.design_tab_widget.indexOf(getattr(self, 'stator_tab', None))
        if index == rotor_index and not getattr(self, 'stator_design_done', False):
            # Revert selection and inform user
            if stator_index != -1:
                self.design_tab_widget.setCurrentIndex(stator_index)
            QMessageBox.information(self, "Stator design required", "Please calculate the stator design first to unlock the rotor tab.")

    def calculate_rotor_design(self):
        """Run rotor design using stator-derived inputs + rotor tab config."""
        if not getattr(self, 'stator_design_done', False) or not self.stator_to_rotor_inputs:
            QMessageBox.warning(self, "Stator design required", "Please calculate the stator design first.")
            return
        if run_rotor_design is None or RotorStatorInputs is None or RotorUserConfig is None:
            QMessageBox.critical(self, "Missing module", "rotor_design_v1.py could not be imported.")
            return

        try:
            # Stator winding factor to use in rotor script (Kws)
            Kws = float(self.stator_to_rotor_inputs.get('Kws') or 0.0)
            if Kws <= 0:
                raise ValueError("Invalid Kws (stator K_w not computed)")
            inputs = RotorStatorInputs(
                P_pairs=int(self.stator_to_rotor_inputs['P_pairs']),
                Ss=int(self.stator_to_rotor_inputs['Ss']),
                D_mm=float(self.stator_to_rotor_inputs['D_mm']),
                L_mm=float(self.stator_to_rotor_inputs['L_mm']),
                Ls_mm=float(self.stator_to_rotor_inputs['Ls_mm']),
                Li_mm=float(self.stator_to_rotor_inputs.get('Li_mm', 0.0) or 0.0),
                ki=float(self.stator_to_rotor_inputs.get('ki', 0.95) or 0.95),
                Iph_A=float(self.stator_to_rotor_inputs['Iph_A']),
                Nc=int(self.stator_to_rotor_inputs['Nc']),
                flux_per_pole_Wb=float(self.stator_to_rotor_inputs['flux_per_pole_Wb']),
                B_tooth_T=float(self.stator_to_rotor_inputs['B_tooth_T']),
                B_yoke_T=float(self.stator_to_rotor_inputs['B_yoke_T']),
                power_kw=float(self.stator_to_rotor_inputs.get('power_kw', 0.0) or 0.0),
                Kws=Kws,
            )
        except Exception as e:
            QMessageBox.critical(self, "Rotor inputs", f"Invalid stator-derived rotor inputs: {e}")
            return

        try:
            config = self._build_rotor_user_config_from_ui()
        except Exception as e:
            QMessageBox.critical(self, "Rotor config", str(e))
            return

        self.status_bar.showMessage("Calculating rotor design...")
        QApplication.processEvents()

        try:
            results, report_text = run_rotor_design(inputs, config)
            self.rotor_results = results

            if hasattr(self, 'tab_rotor'):
                self.append_to_tab(self.tab_rotor, report_text)
                # Switch to rotor report tab
                idx = self.tab_widget.indexOf(self.tab_rotor)
                if idx != -1:
                    self.tab_widget.setCurrentIndex(idx)

            self.status_bar.showMessage("Rotor design completed successfully!")
            QMessageBox.information(self, "Success", "Rotor design completed successfully!")

            # Export becomes available once both stator + rotor are ready
            self.update_export_button_visibility()

            # Update 2D rotor + assembly views (only visible after rotor is computed)
            self._update_2d_view_from_current_rotor()

            # Populate efficiency analysis tab (now that both stator + rotor exist)
            if getattr(self, '_export_ready', False):
                self.show_efficiency_analysis()
        except Exception as e:
            import traceback
            error_details = traceback.format_exc()
            QMessageBox.critical(self, "Rotor Calculation Error", f"An error occurred:\n{str(e)}\n\nDetails:\n{error_details}")
            self.status_bar.showMessage("Error in rotor calculation")

            # Keep export locked if rotor calculation failed
            self.rotor_results = None
            self.update_export_button_visibility()

    def _build_rotor_user_config_from_ui(self):
        """Read rotor tab controls and build a RotorUserConfig."""
        # Nr
        nr_method = self.rotor_nr_method_combo.currentData()
        nr_data = self.rotor_nr_combo.currentData()
        if nr_data == 'custom':
            try:
                Nr = int(self.rotor_nr_custom_input.text().strip())
            except Exception:
                raise ValueError("Custom Nr must be an integer")
            if Nr <= 0:
                raise ValueError("Custom Nr must be > 0")
            nr_method_label = 'custom'
        else:
            Nr = int(nr_data)
            nr_method_label = str(nr_method or 'classique')

        # Tolerance
        tol_data = self.rotor_tol_combo.currentData()
        if tol_data == 'custom':
            try:
                tol_percent = float(self.rotor_tol_custom_input.text().strip())
            except Exception:
                raise ValueError("Custom tolerance must be a number")
        else:
            tol_percent = float(tol_data)
        if tol_percent <= 0:
            raise ValueError("Tolerance must be > 0")

        # Skew
        skew_data = self.rotor_skew_combo.currentData()
        if skew_data == 'custom':
            try:
                skew_angle = float(self.rotor_skew_custom_input.text().strip())
            except Exception:
                raise ValueError("Custom skew angle must be a number")
            if not (10.0 <= skew_angle <= 30.0):
                raise ValueError("Custom skew angle must be between 10 and 30 degrees")
        else:
            skew_angle = float(skew_data)

        # Bar material
        bar_material = self.rotor_bar_material_combo.currentText().strip()
        bar_rho = None
        if bar_material == 'Custom':
            try:
                bar_rho = float(self.rotor_bar_rho_custom_input.text().strip())
            except Exception:
                raise ValueError("Custom bar resistivity ρ must be a number")
            if bar_rho <= 0:
                raise ValueError("Custom bar resistivity ρ must be > 0")

        # Ring Je
        ring_je_mode = self.rotor_ring_je_combo.currentData()
        ring_je_custom = None
        if ring_je_mode == 'custom':
            try:
                ring_je_custom = float(self.rotor_ring_je_custom_input.text().strip())
            except Exception:
                raise ValueError("Custom Je must be a number")
            if ring_je_custom <= 0:
                raise ValueError("Custom Je must be > 0")

        # Ring dimensions
        ring_dim_mode = self.rotor_ring_dim_combo.currentData()
        ring_h_er = None
        ring_b_er = None
        if ring_dim_mode == 'manual':
            try:
                ring_h_er = float(self.rotor_ring_h_er_input.text().strip())
                ring_b_er = float(self.rotor_ring_b_er_input.text().strip())
            except Exception:
                raise ValueError("Manual ring dimensions must be numeric")
            if ring_h_er <= 0 or ring_b_er <= 0:
                raise ValueError("Manual ring dimensions must be > 0")

        # Ring material
        ring_material = self.rotor_ring_material_combo.currentText().strip()
        ring_rho = None
        if ring_material == 'Custom':
            try:
                ring_rho = float(self.rotor_ring_rho_custom_input.text().strip())
            except Exception:
                raise ValueError("Custom ring resistivity ρ must be a number")
            if ring_rho <= 0:
                raise ValueError("Custom ring resistivity ρ must be > 0")

        return RotorUserConfig(
            Nr=Nr,
            nr_method=nr_method_label,
            tol_percent=float(tol_percent),
            skew_angle_deg=float(skew_angle),
            bar_material=bar_material,
            bar_rho_ohm_m=bar_rho,
            ring_je_mode=str(ring_je_mode),
            ring_je_custom=ring_je_custom,
            ring_dim_mode=str(ring_dim_mode),
            ring_h_er_mm=ring_h_er,
            ring_b_er_mm=ring_b_er,
            ring_material=ring_material,
            ring_rho_ohm_m=ring_rho,
        )

    # -----------------------------
    # Rotor tab UI handlers (Step 2)
    # -----------------------------

    def on_rotor_nr_method_changed(self, index: int):
        """Update Nr candidates for the selected method (populated in Step 3)."""
        self.populate_rotor_nr_options()

    def on_rotor_nr_choice_changed(self, index: int):
        if not hasattr(self, 'rotor_nr_combo'):
            return
        data = self.rotor_nr_combo.currentData()
        is_custom = (data == 'custom')
        if hasattr(self, 'rotor_nr_custom_input'):
            self.rotor_nr_custom_input.setVisible(is_custom)
        if hasattr(self, 'rotor_nr_slider'):
            self.rotor_nr_slider.setEnabled(not is_custom and len(getattr(self, '_rotor_nr_candidates', [])) > 0)

        # Sync slider when choosing a predefined Nr
        if not is_custom and isinstance(data, int) and hasattr(self, 'rotor_nr_slider'):
            try:
                candidates = list(getattr(self, '_rotor_nr_candidates', []))
                if data in candidates:
                    self.rotor_nr_slider.blockSignals(True)
                    self.rotor_nr_slider.setValue(candidates.index(data))
                    self.rotor_nr_slider.blockSignals(False)
            except Exception:
                pass

    def on_rotor_nr_slider_changed(self, value: int):
        if not hasattr(self, 'rotor_nr_combo'):
            return
        candidates = list(getattr(self, '_rotor_nr_candidates', []))
        if not candidates:
            return
        idx = max(0, min(int(value), len(candidates) - 1))
        target_nr = candidates[idx]
        # Select matching item (skip custom)
        for i in range(self.rotor_nr_combo.count()):
            if self.rotor_nr_combo.itemData(i) == target_nr:
                self.rotor_nr_combo.setCurrentIndex(i)
                break

    def on_rotor_tol_changed(self, index: int):
        if not hasattr(self, 'rotor_tol_combo'):
            return
        is_custom = self.rotor_tol_combo.currentData() == 'custom'
        if hasattr(self, 'rotor_tol_custom_input'):
            self.rotor_tol_custom_input.setVisible(is_custom)

    def on_rotor_skew_changed(self, index: int):
        if not hasattr(self, 'rotor_skew_combo'):
            return
        is_custom = self.rotor_skew_combo.currentData() == 'custom'
        if hasattr(self, 'rotor_skew_custom_input'):
            self.rotor_skew_custom_input.setVisible(is_custom)

    def _material_info_text(self, material_name: str, rho_ohm_m: float) -> str:
        conductivity_levels = {
            "Cuivre électrolytique": "Très élevée",
            "Aluminium pur": "Élevée",
            "Aluminium moulé (Al-Si)": "Moyenne",
            "Laiton (Cu-Zn)": "Faible",
            "Bronze (Cu-Sn)": "Faible",
        }
        level = conductivity_levels.get(material_name, "")
        if rho_ohm_m and rho_ohm_m > 0:
            rho_txt = f"{rho_ohm_m:.3e} Ω·m"
        else:
            rho_txt = "N/A"
        if level:
            return f"ρ = {rho_txt} | Conductivité: {level}"
        return f"ρ = {rho_txt}"

    def _conductor_material_info_text(self, material_name: str, rho_ohm_m: float, density_g_cm3: float) -> str:
        base = self._material_info_text(material_name, rho_ohm_m)
        dens_txt = f"{density_g_cm3:.2f} g/cm³" if density_g_cm3 and density_g_cm3 > 0 else "N/A"
        return f"{base} | Masse volumique: {dens_txt}"

    def on_stator_conductor_material_changed(self, index: int):
        name = self.stator_conductor_material_combo.currentText().strip() if hasattr(self, 'stator_conductor_material_combo') else ""
        # Use the same rotor material table for rho; limited to Cu/Al for stator conductors
        rho_map = {
            "Cuivre électrolytique": 1.68e-8,
            "Aluminium pur": 2.82e-8,
        }
        density_map_g_cm3 = {
            "Cuivre électrolytique": 8.9,
            "Aluminium pur": 2.7,
        }
        rho_val = float(rho_map.get(name, 1.68e-8) or 1.68e-8)
        dens_val = float(density_map_g_cm3.get(name, 8.9) or 8.9)
        if hasattr(self, 'stator_conductor_material_info'):
            self.stator_conductor_material_info.setText(self._conductor_material_info_text(name, rho_val, dens_val))

    def on_rotor_bar_material_changed(self, index: int):
        name = self.rotor_bar_material_combo.currentText().strip() if hasattr(self, 'rotor_bar_material_combo') else ""
        is_custom = (name == "Custom")
        if hasattr(self, 'rotor_bar_rho_custom_input'):
            self.rotor_bar_rho_custom_input.setVisible(is_custom)
        rho_map = {
            "Cuivre électrolytique": 1.68e-8,
            "Aluminium pur": 2.82e-8,
            "Aluminium moulé (Al-Si)": 3.4e-8,
            "Laiton (Cu-Zn)": 7.0e-8,
            "Bronze (Cu-Sn)": 8.0e-8,
        }
        rho_val = 0.0 if is_custom else float(rho_map.get(name, 0.0) or 0.0)
        if hasattr(self, 'rotor_bar_material_info'):
            self.rotor_bar_material_info.setText(self._material_info_text(name, rho_val) if not is_custom else "Custom material: enter ρ (Ω·m)")

    def on_rotor_ring_material_changed(self, index: int):
        name = self.rotor_ring_material_combo.currentText().strip() if hasattr(self, 'rotor_ring_material_combo') else ""
        is_custom = (name == "Custom")
        if hasattr(self, 'rotor_ring_rho_custom_input'):
            self.rotor_ring_rho_custom_input.setVisible(is_custom)
        rho_map = {
            "Cuivre électrolytique": 1.68e-8,
            "Aluminium pur": 2.82e-8,
            "Aluminium moulé (Al-Si)": 3.4e-8,
            "Laiton (Cu-Zn)": 7.0e-8,
            "Bronze (Cu-Sn)": 8.0e-8,
        }
        rho_val = 0.0 if is_custom else float(rho_map.get(name, 0.0) or 0.0)
        if hasattr(self, 'rotor_ring_material_info'):
            self.rotor_ring_material_info.setText(self._material_info_text(name, rho_val) if not is_custom else "Custom material: enter ρ (Ω·m)")

    def on_rotor_ring_je_changed(self, index: int):
        if not hasattr(self, 'rotor_ring_je_combo'):
            return
        is_custom = self.rotor_ring_je_combo.currentData() == 'custom'
        if hasattr(self, 'rotor_ring_je_custom_input'):
            self.rotor_ring_je_custom_input.setVisible(is_custom)

    def on_rotor_ring_dim_changed(self, index: int):
        if not hasattr(self, 'rotor_ring_dim_combo'):
            return
        is_manual = self.rotor_ring_dim_combo.currentData() == 'manual'
        if hasattr(self, 'rotor_ring_manual_widget'):
            self.rotor_ring_manual_widget.setVisible(is_manual)
    
    def show_empirical_parameters(self):
        """Show empirical parameter interpolation with equations"""
        text = self.make_header("EMPIRICAL PARAMETERS FROM ABAQUES TABLES")
        
        params = self.designer.get_empirical_parameters()
        power = self.designer.specs.power_kw
        specs = self.designer.specs
        
        text += f"Based on rated power P = {power:.2f} kW\n\n"

        # User overrides summary (non-zero overrides take priority)
        text += self.make_separator()
        text += "0. User Overrides (if enabled)\n"
        text += self.make_separator()

        overrides = []
        if getattr(specs, 'target_efficiency', None) is not None:
            overrides.append(f"η = {specs.target_efficiency:.4f}")
        if getattr(specs, 'target_power_factor', None) is not None:
            overrides.append(f"cosφ = {specs.target_power_factor:.4f}")
        if getattr(specs, 'override_Bav', None) is not None:
            overrides.append(f"B_av = {specs.override_Bav:.4f} T")
        if getattr(specs, 'override_ac', None) is not None:
            overrides.append(f"a_c = {specs.override_ac:.0f} A/m")
        if getattr(specs, 'override_q', None) is not None:
            overrides.append(f"q (slots/pole/phase) = {specs.override_q:g}")
        if getattr(specs, 'override_lambda_ratio', None) is not None:
            overrides.append(f"L/τ (λ) = {specs.override_lambda_ratio:.3f}")

        if overrides:
            text += "    Active overrides:\n"
            for item in overrides:
                text += f"    - {item}\n"
        else:
            text += "    No user overrides active (all values auto-selected).\n"
        text += "\n"
        
        # B_av interpolation
        text += self.make_separator()
        text += "1. Average Air Gap Flux Density (B_av)\n"
        text += self.make_separator()
        text += "    From Table 7.1 (abaques-1.pdf), interpolating based on power:\n\n"
        text += self.format_equation(
            f"B_av = interpolate(P = {power:.2f} kW, Table_7.1)",
            f"{params['B_av']:.4f} T"
        )
        if getattr(specs, 'override_Bav', None) is not None:
            text += f"    Note: B_av overridden by user → using {specs.override_Bav:.4f} T\n"
        text += "\n"
        
        # q interpolation
        text += self.make_separator()
        text += "2. Specific Electric Loading (q)\n"
        text += self.make_separator()
        text += "    Linear current density from Table 7.1:\n\n"
        text += self.format_equation(
            f"q = interpolate(P = {power:.2f} kW, Table_7.1)",
            f"{params['q_default']:.0f} A/m"
        )
        if getattr(specs, 'override_ac', None) is not None:
            text += f"    Note: a_c overridden by user → using {specs.override_ac:.0f} A/m\n"
        text += "\n"
        
        # Efficiency (formula-based)
        text += self.make_separator()
        text += "3. Efficiency (eta)\n"
        text += self.make_separator()
        text += "    Formula-based (no Table 7.3):\n\n"
        pole_pairs = (self.designer.specs.poles / 2.0) if self.designer.specs.poles else 0.0
        ns_rpm = (60.0 * self.designer.specs.frequency_hz / pole_pairs) if pole_pairs > 0 else 0.0
        text += "    Definitions: P = rated power (kW), p = pole pairs, n_s = 60·f/p (rpm)\n\n"
        text += self.format_equation(
            f"p = {self.designer.specs.poles} / 2",
            f"{pole_pairs:.3f}"
        )
        text += self.format_equation(
            f"n_s = 60 × {self.designer.specs.frequency_hz:.1f} / {pole_pairs:.3f}",
            f"{ns_rpm:.2f} rpm"
        )
        text += self.format_equation(
            f"eta = 1 - [0.01 + 0.03×P^(-0.03) + 0.1×ln(p)]",
            f"{params['eta']:.4f} ({params['eta']*100:.2f}%)"
        )
        if getattr(specs, 'target_efficiency', None) is not None:
            text += f"    Note: η overridden by user → using {specs.target_efficiency:.4f} ({specs.target_efficiency*100:.2f}%)\n"
        text += "\n"
        
        # Power factor (formula-based)
        text += self.make_separator()
        text += "4. Power Factor (cos phi)\n"
        text += self.make_separator()
        text += "    Formula-based (no Table 7.2):\n\n"
        text += self.format_equation(
            f"cos(phi) = 0.84 - 0.04×ln(P) + 0.02×ln(n_s/p)",
            f"{params['cos_phi']:.4f}"
        )
        if getattr(specs, 'target_power_factor', None) is not None:
            text += f"    Note: cosφ overridden by user → using {specs.target_power_factor:.4f}\n"
        text += "\n"
        
        # Slots per pole per phase
        q = self.designer._determine_q()
        text += self.make_separator()
        text += "5. Slots per Pole per Phase (q)\n"
        text += self.make_separator()
        text += "    From Table 7.5 recommendations:\n\n"
        text += self.format_equation(
            f"q = recommended(P = {power:.2f} kW, Poles = {self.designer.specs.poles})",
            f"{q}"
        )
        if getattr(specs, 'override_q', None) is not None:
            text += f"    Note: q overridden by user → using {specs.override_q:g}\n\n"
        else:
            text += "\n"
        
        self.append_to_tab(self.tab_empirical, text)
    
    def show_electrical_calculations(self):
        """Show electrical parameter calculations with equations"""
        text = self.make_header("ELECTRICAL PARAMETERS CALCULATION")
        
        self.designer.calculate_electrical_parameters()
        elec = self.designer.electrical
        params = self.designer.get_empirical_parameters()
        
        # Electrical power
        text += self.make_separator()
        text += "1. Electrical Power (P_elec)\n"
        text += self.make_separator()
        text += "    Input power divided by efficiency:\n\n"
        text += "    Formula: P_elec = P / eta\n\n"
        text += self.format_equation(
            f"P_elec = {self.designer.specs.power_kw:.2f} / {params['eta']:.4f}",
            f"{elec.P_elec:.4f} kW"
        )
        
        # Synchronous speed
        text += self.make_separator()
        text += "2. Synchronous Speed (n_s)\n"
        text += self.make_separator()
        text += "    Rotational speed at synchronous frequency:\n\n"
        text += "    Formula: n_s = 60 x (f / (P/2))  [rpm]\n\n"
        text += self.format_equation(
            f"n_s = 60 x ({self.designer.specs.frequency_hz:.1f} / ({self.designer.specs.poles}/2))",
            f"{elec.n_s:.2f} rpm"
        )
        
        # Convert to rps
        text += "    Converting to revolutions per second:\n\n"
        text += "    Formula: n_s [rps] = n_s [rpm] / 60\n\n"
        text += self.format_equation(
            f"n_s = {elec.n_s:.2f} / 60",
            f"{elec.n_s_rps:.4f} rps"
        )
        
        # Phase voltage
        text += self.make_separator()
        text += "3. Phase Voltage (E_ph)\n"
        text += self.make_separator()
        text += f"    Connection type: {self.designer.specs.connection.upper()}\n\n"
        if self.designer.specs.connection.lower() == 'star':
            text += "    Formula: E_ph = V_line / sqrt(3)  (for star connection)\n\n"
            text += self.format_equation(
                f"E_ph = {self.designer.specs.voltage_v:.1f} / sqrt(3)",
                f"{elec.E_ph:.4f} V"
            )
        else:
            text += "    Formula: E_ph = V_line  (for delta connection)\n\n"
            text += self.format_equation(
                f"E_ph = {self.designer.specs.voltage_v:.1f}",
                f"{elec.E_ph:.4f} V"
            )
        
        # Phase current
        text += self.make_separator()
        text += "4. Phase Current (I_s)\n"
        text += self.make_separator()
        text += "    Three-phase power equation:\n\n"
        if self.designer.specs.connection.lower() == 'delta':
            text += "    Formula (delta): I_s = P_elec × 1000 / (3 × V_line × cos(phi))  [A]\n\n"
            text += self.format_equation(
                f"I_s = {elec.P_elec:.4f} × 1000 / (3 × {self.designer.specs.voltage_v:.1f} × {params['cos_phi']:.4f})",
                f"{elec.I_s:.4f} A"
            )
        else:
            text += "    Formula (star): I_s = P_elec × 1000 / (sqrt(3) × V_line × cos(phi))  [A]\n\n"
            text += self.format_equation(
                f"I_s = {elec.P_elec:.4f} × 1000 / (sqrt(3) × {self.designer.specs.voltage_v:.1f} × {params['cos_phi']:.4f})",
                f"{elec.I_s:.4f} A"
            )
        
        self.append_to_tab(self.tab_electrical, text)
    
    def show_main_dimensions(self):
        """Show main dimension calculations with equations"""
        text = self.make_header("MAIN DIMENSIONS CALCULATION (D, L)")
        
        self.designer.calculate_main_dimensions()
        geom = self.designer.geometry
        elec = self.designer.electrical
        params = self.designer.get_empirical_parameters()
        specs = self.designer.specs
        
        # Output coefficient
        text += self.make_separator()
        text += "1. Output Coefficient (Co)\n"
        text += self.make_separator()
        text += "    Relates electrical output to mechanical dimensions:\n\n"
        text += "    Formula: Co = 11 x B_av x eta x cos phi x K_w x q x 10^-3\n"
        
        # Get actual K_w used (always computed from coil pitch selection)
        K_w_raw = getattr(self.designer, '_K_w', None)
        K_w = float(K_w_raw) if K_w_raw is not None else 0.0

        q_slots_per_pole_per_phase = int(self.designer._determine_q())
        coil_pitch_percent = (
            float(self.designer.specs.coil_pitch_percent)
            if self.designer.specs.coil_pitch_percent is not None
            else 100.0
        )

        text += f"    Where: K_w = {K_w:.4f} (computed from k_p and k_d)\n"
        text += f"           Winding type: {self.designer.specs.winding_type}\n"
        text += f"           Coil pitch: {coil_pitch_percent:g}%\n\n"

        if q_slots_per_pole_per_phase > 0:
            alpha_deg = 180.0 / (q_slots_per_pole_per_phase * 3.0)
            alpha_rad = math.radians(alpha_deg)
            denom = q_slots_per_pole_per_phase * math.sin(alpha_rad / 2.0)
            kd = (math.sin(q_slots_per_pole_per_phase * alpha_rad / 2.0) / denom) if abs(denom) > 1e-12 else 0.0

            electrical_coil_span_deg = (coil_pitch_percent / 100.0) * 180.0
            kp = math.sin(math.radians(electrical_coil_span_deg / 2.0))
            kw = kp * kd

            text += self.format_equation(
                f"alpha = 180 / ({q_slots_per_pole_per_phase} × 3)",
                f"{alpha_deg:.6f} deg"
            )
            text += self.format_equation(
                f"k_d = sin({q_slots_per_pole_per_phase}×alpha/2) / ({q_slots_per_pole_per_phase}×sin(alpha/2))",
                f"{kd:.6f}"
            )
            text += self.format_equation(
                f"electrical_coil_span = ({coil_pitch_percent:g}/100) × 180",
                f"{electrical_coil_span_deg:.6f} deg"
            )
            text += self.format_equation(
                f"k_p = sin({electrical_coil_span_deg:.6f}/2)",
                f"{kp:.6f}"
            )
            text += self.format_equation(
                f"K_w = {kp:.6f} × {kd:.6f}",
                f"{kw:.6f}"
            )
            text += "\n"
        
        Co = 11 * params['B_av'] * params['eta'] * params['cos_phi'] * K_w * params['q_default'] * 1e-3
        text += self.format_equation(
            f"Co = 11 x {params['B_av']:.4f} x {params['eta']:.4f} x {params['cos_phi']:.4f} x {K_w} x {params['q_default']:.0f} x 10^-3",
            f"{Co:.6f}"
        )
        
        # D²L product
        text += self.make_separator()
        text += "2. D^2*L Product\n"
        text += self.make_separator()
        text += "    From output equation: P = Co x D^2 x L x n_s\n\n"
        text += "    Formula: D^2*L = P / (Co x n_s)  [m^3]\n\n"
        D2L = self.designer.specs.power_kw / (Co * elec.n_s_rps)
        text += self.format_equation(
            f"D^2*L = {self.designer.specs.power_kw:.2f} / ({Co:.6f} x {elec.n_s_rps:.4f})",
            f"{D2L:.8f} m^3"
        )
        
        # Method selection and lambda (λ = L / τ)
        text += self.make_separator()
        text += "3. D & L Selection Method\n"
        text += self.make_separator()
        val_info = geom.validation_info if hasattr(geom, 'validation_info') else {}
        lambda_info = val_info.get('lambda_info', {})
        method_key = val_info.get('method', 'best_pf')
        method_names = {
            'best_pf': 'Good power factor (heuristic)',
            'lambda_by_poles': 'Lambda by pole count (method B)',
            'preference_based': 'Goal-based lambda (method C)',
            'custom_lambda': 'L/τ override (custom lambda)'
        }
        text += f"    Selected method: {method_names.get(method_key, method_key)}\n\n"

        if getattr(specs, 'override_lambda_ratio', None) is not None:
            text += f"    Note: L/τ was overridden by user → λ = {specs.override_lambda_ratio:.3f}\n\n"

        if method_key == 'best_pf':
            text += "    Using heuristic for good power factor:\n"
            text += "    D = 0.135 x P x sqrt(L)\n"
            text += "    L = sqrt(D^2*L / (0.018225 x P^2))\n"
            text += "    λ is reported for reference only (derived from D,L,τ).\n\n"
        elif lambda_info.get('lambda_ratio'):
            lam = lambda_info['lambda_ratio']
            lam_range = lambda_info.get('range')
            if lam_range:
                text += f"    Lambda (L/τ) chosen: {lam:.3f} within range [{lam_range[0]:.2f} - {lam_range[1]:.2f}]\n"
            else:
                text += f"    Lambda (L/τ) chosen: {lam:.3f}\n"
            if lambda_info.get('basis'):
                text += f"    Basis: {lambda_info['basis']}\n"
            if lambda_info.get('note'):
                text += f"    Note: {lambda_info['note']}\n"
            if method_key == 'preference_based' and val_info.get('lambda_info', {}).get('resolution'):
                res = lambda_info.get('resolution')
                text += f"    Resolution: {res} (preferences: {', '.join(val_info.get('lambda_info', {}).get('selected_preferences', []))})\n"
            text += "\n"
        else:
            text += "    Using lambda-based selection.\n\n"

        # D and L values
        text += self.make_separator()
        text += "4. Bore Diameter and Stack Length\n"
        text += self.make_separator()
        P = self.designer.specs.poles
        D_raw = val_info.get('D_raw', geom.D)
        L_raw = val_info.get('L_raw', geom.L)
        D_after_speed = val_info.get('D_after_speed', D_raw)
        L_after_speed = val_info.get('L_after_speed', L_raw)
        L_unrounded_after_roundD = val_info.get('L_unrounded_after_roundD', L_after_speed)
        D_final = val_info.get('D_final', geom.D)
        L_final = val_info.get('L_final', geom.L)

        if method_key == 'best_pf':
            text += "    From good power factor heuristic:\n\n"
            text += "    Formula: L^2 = D^2*L / (0.018225 x P^2)\n"
            text += "             D = 0.135 x P x sqrt(L)\n\n"
            text += self.format_equation(
                f"L = sqrt({D2L:.8f} / (0.018225 x {P}^2))",
                f"{L_raw:.6f} m = {L_raw*1000:.2f} mm"
            )
            text += self.format_equation(
                f"D = 0.135 x {P} x sqrt({L_raw:.6f})",
                f"{D_raw:.6f} m = {D_raw*1000:.2f} mm"
            )
        elif lambda_info.get('lambda_ratio'):
            lam = lambda_info['lambda_ratio']
            text += "    From lambda relation (method B/C):\n\n"
            text += "    λ = L / τ, τ = πD/P\n"
            text += "    Formula: D = ((D^2*L * P) / (λ x π))^(1/3)\n"
            text += "             L = D^2*L / D^2\n\n"
            text += self.format_equation(
                f"D = (({D2L:.8f} x {P}) / ({lam:.3f} x pi))^(1/3)",
                f"{D_raw:.6f} m = {D_raw*1000:.2f} mm"
            )
            text += self.format_equation(
                f"L = {D2L:.8f} / {D_raw**2:.6f}",
                f"{L_raw:.6f} m = {L_raw*1000:.2f} mm"
            )
        else:
            text += "    From empirical best power factor relation:\n\n"
            text += "    Formula: L^2 = D^2*L / (0.018225 x P^2)\n"
            text += "             D = 0.135 x P x sqrt(L)\n\n"
            text += self.format_equation(
                f"L = sqrt({D2L:.8f} / (0.018225 x {P}^2))",
                f"{L_raw:.6f} m = {L_raw*1000:.2f} mm"
            )
            text += self.format_equation(
                f"D = 0.135 x {P} x sqrt({L_raw:.6f})",
                f"{D_raw:.6f} m = {D_raw*1000:.2f} mm"
            )

        text += "    Peripheral speed check applied next (limit 30 m/s).\n\n"
        if abs(D_after_speed - D_raw) > 1e-6:
            text += self.format_equation(
                f"D (speed-limited)",
                f"{D_after_speed:.6f} m = {D_after_speed*1000:.2f} mm"
            )
            text += self.format_equation(
                f"L (from D^2*L with limited D)",
                f"{L_after_speed:.6f} m = {L_after_speed*1000:.2f} mm"
            )

        text += "    Note: No manufacturability rounding is applied (only peripheral-speed limiting, if needed).\n\n"
        text += self.format_equation(
            f"D_final",
            f"{D_final:.6f} m = {D_final*1000:.2f} mm"
        )
        text += self.format_equation(
            f"L_final",
            f"{L_final:.6f} m = {L_final*1000:.2f} mm"
        )

        # Pole pitch & peripheral speed check
        text += self.make_separator()
        text += "5. Peripheral Speed Check\n"
        text += self.make_separator()
        tau_pole = val_info.get('tau_pole', math.pi * geom.D / P)
        speed_info = val_info.get('speed_info', {})
        v_initial = speed_info.get('peripheral_speed_initial', math.pi * D_raw * elec.n_s_rps)
        v_final = val_info.get('peripheral_speed_final', math.pi * D_final * elec.n_s_rps)
        text += "    Peripheral speed limit (standard motors): 30 m/s\n\n"
        text += self.format_equation(
            f"V = pi x {D_final:.6f} x {elec.n_s_rps:.4f}",
            f"{v_final:.2f} m/s {'✓' if v_final <= 30.0 else '✗'}"
        )
        if speed_info.get('peripheral_speed_limited'):
            text += f"    Note: D was limited to {speed_info.get('D_speed_limit', D_final):.4f} m to satisfy the 30 m/s cap.\n\n"
        else:
            text += "    Note: Peripheral speed within limit (no reduction applied).\n\n"
        
        # Cooling System and Effective Stack Length
        text += self.make_separator()
        text += "6. Cooling System and Effective Stack Length\n"
        text += self.make_separator()
        
        val_info = geom.validation_info if hasattr(geom, 'validation_info') else {}
        ki = val_info.get('ki', getattr(geom, 'ki', 0.95))
        num_canals = val_info.get('num_cooling_canals', getattr(geom, 'num_cooling_canals', 0))
        canal_width_m = val_info.get('canal_width', getattr(geom, 'canal_width', 0))
        Ls = val_info.get('Ls', getattr(geom, 'Ls', geom.L))
        Li = val_info.get('Li', getattr(geom, 'Li', float(Ls) * float(ki)))
        
        text += "    a) Iron Stacking Factor (ki):\n\n"
        steel_data = getattr(self.designer.specs, 'steel_data', None) or {}
        steel_grade = steel_data.get('Grade', getattr(self.designer.specs, 'steel_grade', None))
        thickness = steel_data.get('Thickness_mm', None)

        if thickness is not None:
            try:
                thickness_val = float(thickness)
                text += f"    Steel grade = {steel_grade if steel_grade else '—'}\n"
                text += f"    Lamination thickness = {thickness_val:.2f} mm\n"
                text += f"    ki = {ki:.2f} (selected from thickness-based stacking factor table)\n\n"
            except (TypeError, ValueError):
                text += f"    Steel grade = {steel_grade if steel_grade else '—'}\n"
                text += f"    ki = {ki:.2f} (steel thickness not numeric; using computed value)\n\n"
        else:
            # Fallback explanation when no steel grade/thickness was provided
            if self.designer.specs.frequency_hz > 60:
                text += f"    Frequency = {self.designer.specs.frequency_hz} Hz > 60 Hz\n"
                text += f"    ki = {ki:.2f} (frequency-based fallback for high frequency)\n\n"
            else:
                text += f"    Frequency = {self.designer.specs.frequency_hz} Hz ≤ 60 Hz\n"
                text += f"    ki = {ki:.2f} (frequency-based fallback for 50/60Hz)\n\n"
        
        text += "    b) Radial Cooling Ducts:\n\n"
        text += f"    Ncc (user input) = {int(num_canals)}\n"
        text += "    Ncm = 10 mm (standard)\n\n"
        
        text += "    c) Stack Length (Ls) after ducts:\n\n"
        text += "    Formula: Ls = (L − Ncc × Ncm)\n\n"
        text += self.format_equation(
            f"Ls = ({geom.L:.6f} - {num_canals} × {canal_width_m:.4f})",
            f"{float(Ls):.6f} m = {float(Ls)*1000:.2f} mm"
        )

        text += "\n    d) Net Iron Length (Li):\n\n"
        text += "    Formula: Li = Ls × ki\n\n"
        text += self.format_equation(
            f"Li = {float(Ls):.6f} × {float(ki):.2f}",
            f"{float(Li):.6f} m = {float(Li)*1000:.2f} mm"
        )
        # Dedicated key for export grouping: prevents Li from being de-duplicated into a later tab.
        text += self.format_equation(
            "Li (Main Dimensions)",
            f"{float(Li)*1000:.2f} mm"
        )
        text += "\n    Note: Li is used for tooth area and core depth calculations\n\n"
        
        # Flux per pole
        text += self.make_separator()
        text += "7. Flux per Pole (Phi)\n"
        text += self.make_separator()
        text += "    Magnetic flux under one pole:\n\n"
        text += "    Formula: Phi = B_av x pi x L x D / P  [Wb]\n\n"
        text += self.format_equation(
            f"Phi = {params['B_av']:.4f} x pi x {geom.L:.6f} x {geom.D:.6f} / {P}",
            f"{val_info.get('flux_per_pole_from_Bav', geom.flux_per_pole):.8f} Wb = {val_info.get('flux_per_pole_from_Bav', geom.flux_per_pole)*1000:.4f} mWb"
        )
        
        self.append_to_tab(self.tab_dimensions, text)
    
    def show_winding_design(self):
        """Show winding design calculations with equations"""
        text = self.make_header("WINDING DESIGN CALCULATION")
        
        TPH, Zs, Nc, Ss, q = self.designer.calculate_turns_and_conductors()
        geom = self.designer.geometry
        elec = self.designer.electrical
        geom_val = geom.validation_info if hasattr(geom, 'validation_info') and geom.validation_info else {}
        phi_from_bav = geom_val.get('flux_per_pole_from_Bav', geom.flux_per_pole)
        phi_recalc = geom_val.get('flux_per_pole_recalc', geom.flux_per_pole)
        
        # Start with slots calculation
        # Number of stator slots (table-selected, used directly)
        text += self.make_separator()
        text += "1. Number of Stator Slots (Ss)\n"
        text += self.make_separator()
        
        # q is taken directly from empirical tables (Table 7.5) and used as-is
        q_selected = q
        P = self.designer.specs.poles
        m = 3
        
        text += "    Step 1: Get q from empirical tables (Table 7.5) and use it directly:\n\n"
        if getattr(self.designer.specs, 'override_q', None) is not None:
            text += f"    Note: q overridden by user → using {self.designer.specs.override_q:g}\n\n"
        text += self.format_equation(
            f"q_initial = recommended(P = {self.designer.specs.power_kw:.2f} kW, Poles = {P})",
            f"{q_selected}"
        )
        
        text += "\n    Step 2: Calculate number of slots:\n\n"
        text += "    Formula: Ss = P x m x q\n"
        text += "    Where: P = poles, m = 3 (phases), q = slots/pole/phase\n\n"
        
        Ss_used = Ss
        text += self.format_equation(
            f"Ss = {P} x {m} x {q_selected}",
            f"{Ss_used} slots"
        )

        # Now show TPH calculation
        text += self.make_separator()
        text += "2. Turns per Phase (TPH) - Calculation\n"
        text += self.make_separator()
        text += "    From EMF equation:\n\n"
        text += "    Formula: TPH = E_ph / (4.44 x Phi x f x K_w)\n"
        
        # Get actual K_w used (always computed from coil pitch selection)
        K_w_raw = getattr(self.designer, '_K_w', None)
        K_w = float(K_w_raw) if K_w_raw is not None else 0.0

        # Do not display K_w derivation here (shown in Main Dimensions only).
        text += "\n"
        
        TPH_calc = elec.E_ph / (4.44 * phi_from_bav * self.designer.specs.frequency_hz * K_w)
        TPH_rounded_up = int(math.ceil(float(TPH_calc) - 1e-12))
        text += self.format_equation(
            f"TPH = {elec.E_ph:.4f} / (4.44 x {phi_from_bav:.8f} x {self.designer.specs.frequency_hz:.1f} x {K_w})",
            f"{TPH_calc:.2f} ~= {TPH_rounded_up} turns (rounded UP)"
        )
        
        # Total conductors (initial)
        text += self.make_separator()
        text += "3. Total Stator Conductors (Zs) - Initial\n"
        text += self.make_separator()
        text += "    Each turn has 2 conductors (go and return), 3 phases:\n\n"
        text += "    Formula: Zs = TPH x 2 x 3\n\n"
        Zs_initial = TPH_rounded_up * 2 * 3
        text += self.format_equation(
            f"Zs = {TPH_rounded_up} x 2 x 3",
            f"{Zs_initial} conductors"
        )
        
        # Conductors per slot
        text += self.make_separator()
        text += "4. Conductors per Slot (Nc)\n"
        text += self.make_separator()
        text += "    Total conductors distributed across slots:\n\n"
        text += "    Formula: Nc = Zs / Ss\n\n"
        Nc_calc = Zs_initial / Ss
        text += self.format_equation(
            f"Nc = {Zs_initial} / {Ss}",
            f"{Nc_calc:.4f}"
        )
        
        # Check if adjustment needed
        if Nc_calc != int(Nc_calc):
            text += "    WARNING: Nc is not an integer! Adjustment required.\n\n"
            text += "    Rounding Nc UP to the next integer (closest bigger integer):\n"
            text += f"    Nc = {int(Nc)}\n\n"
            
            text += "    Recalculating Zs:\n"
            text += "    Formula: Zs = Nc x (Ss / 3)\n\n"
            text += self.format_equation(
                f"Zs = {Nc} x ({Ss} / 3)",
                f"{Zs} conductors"
            )
            
            text += "    Recalculating TPH:\n"
            text += "    Formula: TPH = Zs / 2\n\n"
            text += self.format_equation(
                f"TPH = {Zs} / 2",
                f"{TPH} turns"
            )
        else:
            text += "    OK: Nc is already an integer, no adjustment needed.\n\n"

        # NEW: Recalculate flux per pole using final integer TPH (used in subsequent steps)
        text += self.make_separator()
        text += "5. Recalculated Flux per Pole (Phi) - Used Downstream\n"
        text += self.make_separator()
        text += "    Using the same EMF equation used for TPH, rearranged to solve for Phi:\n\n"
        text += "    Formula: Phi = E_ph / (4.44 x f x K_w x TPH)\n\n"
        phi_calc = elec.E_ph / (4.44 * self.designer.specs.frequency_hz * K_w * TPH)
        text += self.format_equation(
            f"Phi = {elec.E_ph:.4f} / (4.44 x {self.designer.specs.frequency_hz:.1f} x {K_w:.4f} x {TPH})",
            f"{phi_calc:.8f} Wb = {phi_calc*1000:.4f} mWb"
        )
        text += "\n    Note: This recalculated Phi is stored and used for slot/yoke calculations that follow.\n\n"

        # NEW: Update L and Ls after Phi recalculation (used downstream)
        text += self.make_separator()
        text += "6. Updated Stack Length (L) and Net Iron Length (Li) - Used Downstream\n"
        text += self.make_separator()
        text += "    Scaling rule (provided):\n\n"
        text += "    Phi_new / Phi_old = L_new / L_old\n"
        text += "    => L_new = L_old × (Phi_new / Phi_old)\n\n"

        L_old = geom_val.get('L_old_main_dimensions', geom_val.get('L_final', geom.L))
        L_new_calc = geom_val.get('L_new_calc', None)
        L_new_rounded = geom_val.get('L_new_rounded', geom.L)
        ratio_phi = geom_val.get('flux_ratio_phi', None)

        if ratio_phi is None and phi_from_bav and phi_recalc:
            try:
                ratio_phi = float(phi_recalc) / float(phi_from_bav)
            except Exception:
                ratio_phi = None

        if ratio_phi is not None:
            text += self.format_equation(
                f"Phi_new/Phi_old = {float(phi_recalc):.8f} / {float(phi_from_bav):.8f}",
                f"{float(ratio_phi):.6f}"
            )

        if L_new_calc is not None:
            text += self.format_equation(
                f"L_new(calc) = {float(L_old):.6f} × {float(ratio_phi):.6f}",
                f"{float(L_new_calc):.6f} m = {float(L_new_calc)*1000:.2f} mm"
            )

        # L_new_rounded is a derived internal value; keep it out of the displayed report.

        # Updated Ls/Li using same rule as main dimensions
        ki_used = geom_val.get('ki', getattr(geom, 'ki', 0.95))
        ncc_new = geom_val.get('num_cooling_canals_new', getattr(geom, 'num_cooling_canals', 0))
        ncm_new = geom_val.get('canal_width_new', getattr(geom, 'canal_width', 0.0))
        Ls_new = geom_val.get('Ls_new', getattr(geom, 'Ls', geom.L))
        Li_new = geom_val.get('Li_new', getattr(geom, 'Li', float(Ls_new) * float(ki_used)))

        text += "\n    Recompute stack length using the same rule as Main Dimensions:\n\n"
        text += "    Formula: Ls_new = (L_new − Ncc × Ncm)\n\n"
        text += self.format_equation(
            f"Ls_new = ({float(L_new_rounded):.6f} - {int(ncc_new)} × {float(ncm_new):.4f})",
            f"{float(Ls_new):.6f} m = {float(Ls_new)*1000:.2f} mm"
        )

        text += "\n    Net iron length (used downstream):\n\n"
        text += "    Formula: Li_new = Ls_new × ki\n\n"
        text += self.format_equation(
            f"Li_new = {float(Ls_new):.6f} × {float(ki_used):.2f}",
            f"{float(Li_new):.6f} m = {float(Li_new)*1000:.2f} mm"
        )
        text += "\n    Note: L_new and Li_new are the values used in the rest of the program.\n\n"
        
        self.append_to_tab(self.tab_winding, text)
    
    def show_slot_design(self):
        """Show slot design calculations with equations (Semi-open and Open slot methods)."""
        text = self.make_header("SLOT DESIGN AND CONDUCTOR SIZING")

        # Always execute backend sizing so downstream tabs have geometry/slots populated.
        # Display reflects the selected slot method (semi-open or open slot).
        self.designer.design_conductor_and_slots()
        winding = self.designer.winding
        slots = self.designer.slots
        elec = self.designer.electrical
        geom = self.designer.geometry
        tooth_val = getattr(self.designer, 'tooth_validation', {}) or {}
        method = getattr(self.designer.specs, 'slot_method', 'semi_open_slot') or 'semi_open_slot'

        P = self.designer.specs.poles
        Ss = slots.Ss
        geom_val = geom.validation_info if hasattr(geom, 'validation_info') and geom.validation_info else {}
        steel_data = getattr(self.designer.specs, 'steel_data', None) or {}
        grade = steel_data.get('Grade', getattr(self.designer.specs, 'steel_grade', '—'))
        steel_max_b = steel_data.get('Max_Design_B', None)

        # Common values
        b_tooth_target = geom_val.get('B_tooth_target', None)
        b_yoke_target = geom_val.get('B_yoke_target', None)
        phi_max = geom_val.get('phi_max', None)
        wst_m = geom_val.get('wst_m', None)
        flux_pole_used = geom_val.get('phi_max_flux_pole', geom.flux_per_pole)
        Li_used = geom_val.get('wst_Li_used', geom_val.get('wst_Ls_used', getattr(geom, 'Li', getattr(geom, 'Ls', geom.L))))

        ui_method = getattr(self, '_slot_method_ui', None)
        if ui_method == 'tapered_slot':
            semi_open_label = "Trapezoidal semi-open slot"
        else:
            semi_open_label = "Semi-open slot"

        if ui_method == 'semi_open_slot_new':
            # -----------------------------------------------------------------
            # New Semi-open slot method (to be filled progressively)
            # Uses the same upstream targets + conductor sizing as tapered slot.
            # -----------------------------------------------------------------
            semi_open_label = "Semi-open slot"

            # Values commonly provided by backend (it runs semi-open sizing for now)
            b_tooth_target_local = b_tooth_target
            b_yoke_target_local = b_yoke_target
            phi_max_local = phi_max
            flux_pole_used_local = flux_pole_used
            Li_used_local = Li_used

            # Fixed heights (as used in tapered slot)
            hs0_mm = float(geom_val.get('hs0_mm', 1.0) or 1.0)
            hs1_mm = float(geom_val.get('hs1_mm', 3.0) or 3.0)

            # Conductor sizing outputs
            Iph_A = geom_val.get('Iph_A', elec.I_s)
            J = geom_val.get('J_A_per_mm2', None)
            ac_mm2 = geom_val.get('ac_mm2', None)
            strands = geom_val.get('number_of_strands', None)
            d_calc_mm = geom_val.get('strand_diameter_mm_calc', None)
            d_max_mm = geom_val.get('strand_diameter_mm_max', 1.7)
            swg = geom_val.get('swg_selected', None)
            d_std_mm = geom_val.get('strand_diameter_mm_std', None)
            ac_std_mm2 = geom_val.get('ac_std_mm2', None)
            Tac_mm2 = geom_val.get('Tac_mm2', None)
            A_slot_mm2 = geom_val.get('A_slot_mm2', None)
            slot_pitch_D_mm = geom_val.get('slot_pitch_D_mm', None)
            wst0_mm = geom_val.get('wst0_mm', None)

            # Section 1 — same as tapered slot
            text += self.make_separator()
            text += f"1. {semi_open_label}: Tooth & Yoke Flux Density Targets\n"
            text += self.make_separator()
            text += "    Allowable ranges:\n"
            text += "      - Tooth flux density: 1.4 to 2.1 T\n"
            text += "      - Yoke flux density : 1.4 to 1.7 T\n\n"
            text += "    Selection rules:\n"
            text += "      - Do not exceed the selected steel Max_Design_B\n"
            text += "      - Yoke target must be smaller than tooth target\n"
            text += "      - Tooth target: may use the maximum allowed by the steel\n"
            text += "      - Yoke target: choose as low as possible to reduce magnetizing current\n\n"
            text += f"    Selected steel grade: {grade}\n"
            if steel_max_b is not None:
                try:
                    text += f"    Steel Max_Design_B: {float(steel_max_b):.3f} T\n\n"
                except (TypeError, ValueError):
                    text += f"    Steel Max_Design_B: {steel_max_b}\n\n"
            else:
                text += "    Steel Max_Design_B: —\n\n"

            if b_tooth_target_local is not None:
                text += f"    Chosen tooth target B_tooth = {float(b_tooth_target_local):.3f} T\n"
            else:
                text += "    Chosen tooth target B_tooth = —\n"
            if b_yoke_target_local is not None:
                text += f"    Chosen yoke target  B_yoke  = {float(b_yoke_target_local):.3f} T\n\n"
            else:
                text += "    Chosen yoke target  B_yoke  = —\n\n"

            # Section 2 — compute phi_max and wst1
            text += self.make_separator()
            text += f"2. {semi_open_label}: Maximum Tooth Flux and Minimum Tooth Width\n"
            text += self.make_separator()
            text += "    Formula: phi_max = flux_pole × sin(pi × P / (2 × Ss))\n\n"
            if phi_max_local is not None:
                text += self.format_equation(
                    f"phi_max = {float(flux_pole_used_local):.8f} × sin(pi × {int(P)} / (2 × {int(Ss)}))",
                    f"{float(phi_max_local):.8f} Wb = {float(phi_max_local)*1000:.4f} mWb"
                )
            else:
                text += "    phi_max = — (not available)\n"

            text += "\n    Formula: wst1 = phi_max / (B_tooth × Li)\n\n"
            wst1_m = None
            if phi_max_local is not None and b_tooth_target_local is not None and Li_used_local is not None:
                denom = float(b_tooth_target_local) * float(Li_used_local)
                wst1_m = (float(phi_max_local) / denom) if denom > 0 else None
            if wst1_m is not None:
                text += self.format_equation(
                    f"wst1 = {float(phi_max_local):.8f} / ({float(b_tooth_target_local):.3f} × {float(Li_used_local):.6f})",
                    f"{float(wst1_m):.8f} m = {float(wst1_m)*1000:.3f} mm"
                )
            else:
                text += "    wst1 = — (not available)\n"

            # Section 3 — conductor sizing (same as tapered slot)
            text += self.make_separator()
            text += f"3. {semi_open_label}: Copper Area, Strands, and Standard Wire Selection\n"
            text += self.make_separator()
            text += f"    Constants: hs0 = {hs0_mm:.1f} mm, hs1 = {hs1_mm:.1f} mm (fixed)\n\n"
            text += "    Formula: ac = Iph / J\n\n"
            if J is not None and ac_mm2 is not None:
                text += self.format_equation(
                    f"ac = {float(Iph_A):.4f} / {float(J):.4f}",
                    f"{float(ac_mm2):.6f} mm^2"
                )

            text += "\n    Strand selection loop (limit strand diameter ≤ 1.7 mm):\n\n"
            text += "    Start: number_of_strands = 1\n"
            text += "    Each iteration: strand_area = ac / number_of_strands; d = sqrt(4×strand_area/pi)\n"
            text += f"    Stop when d ≤ {float(d_max_mm):.2f} mm.\n\n"
            if strands is not None:
                text += self.format_equation("number_of_strands", f"{int(float(strands))}")
            if d_calc_mm is not None:
                ok = "✓" if float(d_calc_mm) <= float(d_max_mm) else "✗"
                text += self.format_equation("d(calc)", f"{float(d_calc_mm):.4f} mm {ok} (max {float(d_max_mm):.1f} mm)")

            text += "\n    Standard wire selection (SWG):\n\n"
            text += "    Rule: pick the smallest standard diameter that is ≥ d(calc) (never pick smaller).\n\n"
            if swg is not None and d_std_mm is not None:
                text += self.format_equation(
                    "SWG selected",
                    f"SWG {int(float(swg))} → d(std) = {float(d_std_mm):.4f} mm"
                )

            text += "\n    Updated copper areas using selected standard wire:\n\n"
            text += "    a_strand = pi × d(std)^2 / 4\n"
            text += "    ac(std) = number_of_strands × a_strand\n"
            text += "    Tac = ac(std) × Nc\n"
            text += "    A_slot = Tac / k_fill  (k_fill = 0.4)\n\n"
            if d_std_mm is not None:
                a_strand = math.pi * (float(d_std_mm) ** 2) / 4.0
                text += self.format_equation("a_strand", f"{a_strand:.6f} mm^2")
            if ac_std_mm2 is not None:
                text += self.format_equation("ac(std)", f"{float(ac_std_mm2):.6f} mm^2")
            if Tac_mm2 is not None:
                text += self.format_equation("Tac = ac(std) × Nc", f"{float(Tac_mm2):.4f} mm^2")
            if A_slot_mm2 is not None:
                text += self.format_equation("A_slot", f"{float(A_slot_mm2):.4f} mm^2")

            text += "\n    Slot opening at bore:\n\n"
            text += "    slot_pitch(D) = pi × D / Ss\n"
            text += "    bs0 = 3 × d(std)\n"
            text += "    wst0 = slot_pitch(D) − bs0\n\n"
            if slot_pitch_D_mm is not None:
                text += self.format_equation("slot_pitch(D)", f"{float(slot_pitch_D_mm):.3f} mm")
            if d_std_mm is not None:
                text += self.format_equation("bs0 = 3 × d(std)", f"3 × {float(d_std_mm):.4f} = {3.0*float(d_std_mm):.3f} mm")
            if wst0_mm is not None:
                text += self.format_equation("wst0", f"{float(wst0_mm):.3f} mm")

            # Section 4 — new geometry solver
            text += self.make_separator()
            text += f"4. {semi_open_label}: Slot Geometry Optimization (bs, hs2)\n"
            text += self.make_separator()
            text += "    Unknowns: bs (slot width), hs2 (slot height)\n\n"
            text += "    Constraints:\n"
            text += "      (1) A_slot = hs2 × bs\n"
            text += "      (2) G_ratio = hs2 / bs, with 3 ≤ G_ratio ≤ 5\n"
            text += "      (3) slot_pitch(D_top) = bs + wst1\n"
            text += "          where D_top = D + 2×(hs0 + hs1 + hs2)\n\n"

            # Compute pitch model: slot_pitch(D_top) = base + k*hs2
            # base = pi*(D + 2*(hs0+hs1))/Ss ; k = 2*pi/Ss (all in mm)
            D_mm = float(getattr(geom, 'D', 0.0) or 0.0) * 1000.0
            base_pitch_mm = (math.pi * (D_mm + 2.0 * (hs0_mm + hs1_mm)) / float(Ss)) if Ss > 0 else 0.0
            k_pitch = (2.0 * math.pi / float(Ss)) if Ss > 0 else 0.0

            if A_slot_mm2 is None or wst1_m is None or Ss <= 0 or k_pitch <= 0:
                text += "    Not enough data to solve geometry (missing A_slot, wst1, or Ss).\n"
                self.append_to_tab(self.tab_slots, text)
                return

            A_slot_val = float(A_slot_mm2)
            wst1_mm = float(wst1_m) * 1000.0

            text += self.format_equation("A_slot (required)", f"{A_slot_val:.4f} mm^2")
            text += self.format_equation("wst1", f"{wst1_mm:.3f} mm")
            text += self.format_equation("base_pitch", f"pi×(D + 2×(hs0+hs1))/Ss = {base_pitch_mm:.3f} mm")
            text += self.format_equation("k", f"2×pi/Ss = {k_pitch:.6f} (mm pitch per mm hs2)")

            text += "\n    From (1) and (2):\n"
            text += "      bs = sqrt(A_slot / G_ratio)\n"
            text += "      hs2 = sqrt(A_slot × G_ratio)\n\n"
            text += "    From (3):\n"
            text += "      slot_pitch(D_top) = base_pitch + k×hs2\n"
            text += "      bs = slot_pitch(D_top) − wst1\n\n"

            # Exact solve for G_ratio in [3, 5] when possible.
            # Define a single equation in G:
            #   bs_area(G)  = sqrt(A_slot/G)
            #   hs2(G)      = sqrt(A_slot*G)
            #   pitch(G)    = base_pitch + k*hs2(G)
            #   bs_pitch(G) = pitch(G) - wst1
            # Solve f(G) = bs_area(G) - bs_pitch(G) = 0
            def f_of_g(g_ratio: float) -> float:
                if g_ratio <= 0.0 or A_slot_val <= 0.0:
                    return float('nan')
                bs_area_local = math.sqrt(A_slot_val / g_ratio)
                hs2_local = math.sqrt(A_slot_val * g_ratio)
                pitch_local = base_pitch_mm + (k_pitch * hs2_local)
                bs_pitch_local = pitch_local - wst1_mm
                return bs_area_local - bs_pitch_local

            g_lo = 3.0
            g_hi = 5.0
            f_lo = f_of_g(g_lo)
            f_hi = f_of_g(g_hi)

            solved_exact = False
            g_best = None

            # Bisection requires a sign change (or a zero at endpoints).
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
                    fhi = f_hi
                    for _ in range(80):
                        mid = 0.5 * (lo + hi)
                        fmid = f_of_g(mid)
                        if math.isnan(fmid):
                            break
                        if abs(fmid) < 1e-12:
                            lo = hi = mid
                            flo = fhi = fmid
                            break
                        if flo * fmid < 0.0:
                            hi = mid
                            fhi = fmid
                        else:
                            lo = mid
                            flo = fmid
                    g_best = 0.5 * (lo + hi)
                    solved_exact = True

            # Fallback: no exact solution in [3, 5]. Choose the closest feasible ratio.
            # Primary objective: minimize |f(G)|.
            # Secondary objective: prefer ratios closer to 4.0.
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

            # Compute final geometry consistently from area+ratio (constraints 1 and 2 are satisfied by construction)
            g_best = float(g_best)
            bs_mm = math.sqrt(A_slot_val / g_best)
            hs2_mm = math.sqrt(A_slot_val * g_best)
            pitch_top_mm = base_pitch_mm + (k_pitch * hs2_mm)

            # Final checks
            area_check = bs_mm * hs2_mm
            ratio_check = hs2_mm / bs_mm if bs_mm > 0 else float('inf')
            pitch_check = bs_mm + wst1_mm
            pitch_err = pitch_top_mm - pitch_check

            # Keep a snapshot of the pre-correction values for printing (credibility).
            bs_before_mm = bs_mm
            hs2_before_mm = hs2_mm
            wst1_before_mm = wst1_mm
            pitch_top_before_mm = pitch_top_mm
            area_before = area_check
            ratio_before = ratio_check
            pitch_check_before = pitch_check
            pitch_err_before = pitch_err

            # -----------------------------------------------------------------
            # If no exact solution exists, always show Approach 1 first.
            # If Approach 1 fails the B constraint (or is not applicable), then show Approach 2.
            # -----------------------------------------------------------------
            tol_pitch_mm = 1e-6

            # Steel limit (if available)
            try:
                steel_max_T = float(steel_max_b) if steel_max_b is not None else None
            except (TypeError, ValueError):
                steel_max_T = None

            def compute_B_tooth(phi_max_wb: float, Li_m: float, wst1_mm_local: float) -> float:
                denom_local = (Li_m * (wst1_mm_local / 1000.0))
                return (phi_max_wb / denom_local) if denom_local > 0 else float('inf')

            # Approach 1 (always shown first)
            a1_applicable = (pitch_err_before > tol_pitch_mm)
            a1_wst1_after_mm = (wst1_before_mm + abs(pitch_err_before)) if a1_applicable else wst1_before_mm
            a1_bs_after_mm = bs_before_mm
            a1_hs2_after_mm = hs2_before_mm
            a1_pitch_top_mm = pitch_top_before_mm
            a1_area = a1_bs_after_mm * a1_hs2_after_mm
            a1_ratio = a1_hs2_after_mm / a1_bs_after_mm if a1_bs_after_mm > 0 else float('inf')
            a1_pitch_check = a1_bs_after_mm + a1_wst1_after_mm
            a1_pitch_err = a1_pitch_top_mm - a1_pitch_check

            a1_B = None
            a1_B_ok = True
            if phi_max_local is not None and Li_used_local is not None:
                a1_B = compute_B_tooth(float(phi_max_local), float(Li_used_local), float(a1_wst1_after_mm))
                if steel_max_T is not None:
                    a1_B_ok = a1_B <= steel_max_T

            a1_pitch_ok = abs(a1_pitch_err) <= tol_pitch_mm
            a1_success = a1_applicable and a1_pitch_ok and a1_B_ok

            # Approach 2 (only used/shown if Approach 1 fails)
            a2_wst1_after_mm = wst1_before_mm
            a2_bs_after_mm = bs_before_mm + pitch_err_before
            if a2_bs_after_mm <= 0:
                a2_bs_after_mm = max(0.001, bs_before_mm)

            # Update hs2 to keep G in [3,5] (prefer area match if feasible)
            hs2_from_area = (A_slot_val / a2_bs_after_mm) if a2_bs_after_mm > 0 else None
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
            a2_area = a2_bs_after_mm * a2_hs2_after_mm
            a2_ratio = a2_hs2_after_mm / a2_bs_after_mm if a2_bs_after_mm > 0 else float('inf')
            a2_pitch_check = a2_bs_after_mm + a2_wst1_after_mm
            a2_pitch_err = a2_pitch_top_mm - a2_pitch_check

            a2_B = None
            if phi_max_local is not None and Li_used_local is not None:
                a2_B = compute_B_tooth(float(phi_max_local), float(Li_used_local), float(a2_wst1_after_mm))

            text += "    Selected geometry:\n\n"
            if solved_exact:
                text += "    Solve mode: exact (all 3 constraints satisfied within numerical tolerance)\n\n"
            else:
                text += "    Solve mode: closest-feasible (no exact solution in 3 ≤ G_ratio ≤ 5)\n"
                text += "    Notes: A_slot and hs2/bs are satisfied exactly; pitch constraint is minimized.\n\n"

            text += self.format_equation("G_ratio", f"{g_best:.3f}")
            text += self.format_equation("hs2", f"{hs2_before_mm:.3f} mm")
            text += self.format_equation("bs", f"{bs_before_mm:.3f} mm")
            text += "\n    Constraint checks:\n\n"
            text += self.format_equation("A_slot(calc) = hs2×bs", f"{area_before:.3f} mm^2")
            text += self.format_equation("A_slot(target)", f"{A_slot_val:.3f} mm^2")
            text += self.format_equation("hs2/bs", f"{ratio_before:.3f} (target 3–5)")
            text += self.format_equation("slot_pitch(D_top)", f"{pitch_top_before_mm:.3f} mm")
            text += self.format_equation("bs + wst1", f"{pitch_check_before:.3f} mm")
            text += self.format_equation("pitch mismatch (slot_pitch − (bs+wst1))", f"{pitch_err_before:.6f} mm")

            # Append the correction section (always show Approach 1 first; show Approach 2 only if Approach 1 fails)
            if not solved_exact:
                text += "\n"
                text += self.make_separator()
                text += "Correction step (applied after closest-feasible selection)\n"
                text += self.make_separator()

                # -----------------
                # Approach 1
                # -----------------
                text += "    Approach 1: adjust wst1 to match the pitch, then check B ≤ steel limit\n\n"
                if not a1_applicable:
                    text += "    Note: Approach 1 is not applicable because pitch mismatch ≤ 0.\n"
                    text += "    (Increasing wst1 would not reduce the mismatch.)\n\n"

                text += self.format_equation("wst1(before)", f"{wst1_before_mm:.3f} mm")
                text += self.format_equation("wst1(after)", f"{a1_wst1_after_mm:.3f} mm")
                text += self.format_equation("bs(after)", f"{a1_bs_after_mm:.3f} mm")
                text += self.format_equation("hs2(after)", f"{a1_hs2_after_mm:.3f} mm")

                # B calculation and validation (immediately after updated dimensions)
                if a1_B is not None and phi_max_local is not None and Li_used_local is not None:
                    text += "\n    Tooth flux density check at wst1(after):\n\n"
                    text += "    Formula: B_tooth = phi_max / (Li × wst1)\n\n"
                    wst1_m = float(a1_wst1_after_mm) / 1000.0
                    text += self.format_equation(
                        "B_tooth",
                        f"{float(phi_max_local):.8f} / ({float(Li_used_local):.6f} × {wst1_m:.6f}) = {float(a1_B):.4f} T"
                    )
                    if steel_max_T is not None:
                        status = "OK" if a1_B_ok else "Warning"
                        text += self.format_equation("Steel limit", f"{status}: B_tooth ≤ {steel_max_T:.4f} T")

                text += "\n    Updated checks:\n\n"
                text += self.format_equation("A_slot(calc) = hs2×bs", f"{a1_area:.3f} mm^2")
                text += self.format_equation("A_slot(target)", f"{A_slot_val:.3f} mm^2")
                text += self.format_equation("hs2/bs", f"{a1_ratio:.3f} (target 3–5)")
                text += self.format_equation("slot_pitch(D_top)", f"{a1_pitch_top_mm:.3f} mm")
                text += self.format_equation("bs + wst1", f"{a1_pitch_check:.3f} mm")
                text += self.format_equation("pitch mismatch (slot_pitch − (bs+wst1))", f"{a1_pitch_err:.6f} mm")
                if a1_B is not None:
                    if steel_max_T is not None:
                        status = "OK" if a1_B_ok else "Warning"
                        text += self.format_equation("B_tooth(at wst1)", f"{float(a1_B):.4f} T ({status}: ≤ {steel_max_T:.4f} T)")
                    else:
                        text += self.format_equation("B_tooth(at wst1)", f"{float(a1_B):.4f} T")

                # -----------------
                # Approach 2 (only if Approach 1 fails)
                # -----------------
                if not a1_success:
                    text += "\n"
                    text += self.make_separator()
                    text += "    Approach 2: adjust bs by the pitch mismatch, then update hs2 to keep G in [3,5]\n"
                    text += self.make_separator()
                    if a1_applicable and (not a1_B_ok):
                        text += "    Reason: Approach 1 failed the B-tooth steel limit.\n\n"
                    elif not a1_applicable:
                        text += "    Reason: Approach 1 not applicable (pitch mismatch ≤ 0).\n\n"
                    else:
                        text += "    Reason: Approach 1 did not satisfy pitch within tolerance.\n\n"

                    text += self.format_equation("wst1(before)", f"{wst1_before_mm:.3f} mm")
                    text += self.format_equation("wst1(after)", f"{a2_wst1_after_mm:.3f} mm")
                    text += self.format_equation("bs(after)", f"{a2_bs_after_mm:.3f} mm")
                    text += self.format_equation("hs2(after)", f"{a2_hs2_after_mm:.3f} mm")

                    if a2_B is not None and phi_max_local is not None and Li_used_local is not None:
                        text += "\n    Tooth flux density check at wst1(after):\n\n"
                        text += "    Formula: B_tooth = phi_max / (Li × wst1)\n\n"
                        wst1_m = float(a2_wst1_after_mm) / 1000.0
                        text += self.format_equation(
                            "B_tooth",
                            f"{float(phi_max_local):.8f} / ({float(Li_used_local):.6f} × {wst1_m:.6f}) = {float(a2_B):.4f} T"
                        )
                        if steel_max_T is not None:
                            status = "OK" if (a2_B <= steel_max_T) else "Warning"
                            text += self.format_equation("Steel limit", f"{status}: B_tooth ≤ {steel_max_T:.4f} T")

                    text += "\n    Updated checks:\n\n"
                    text += self.format_equation("A_slot(calc) = hs2×bs", f"{a2_area:.3f} mm^2")
                    text += self.format_equation("A_slot(target)", f"{A_slot_val:.3f} mm^2")
                    text += self.format_equation("hs2/bs", f"{a2_ratio:.3f} (target 3–5)")
                    text += self.format_equation("slot_pitch(D_top)", f"{a2_pitch_top_mm:.3f} mm")
                    text += self.format_equation("bs + wst1", f"{a2_pitch_check:.3f} mm")
                    text += self.format_equation("pitch mismatch (slot_pitch − (bs+wst1))", f"{a2_pitch_err:.6f} mm")
                    if a2_B is not None:
                        if steel_max_T is not None:
                            status = "OK" if (a2_B <= steel_max_T) else "Warning"
                            text += self.format_equation("B_tooth(at wst1)", f"{float(a2_B):.4f} T ({status}: ≤ {steel_max_T:.4f} T)")
                        else:
                            text += self.format_equation("B_tooth(at wst1)", f"{float(a2_B):.4f} T")

            # Section 5 — Results summary (requested)
            text += self.make_separator()
            text += f"5. {semi_open_label}: Results\n"
            text += self.make_separator()

            # Compute final values based on accepted approach
            final_uses_a1 = bool(solved_exact or a1_success)
            wst1_final_mm = float(wst1_before_mm)
            if not solved_exact:
                wst1_final_mm = float(a1_wst1_after_mm) if final_uses_a1 else float(a2_wst1_after_mm)
            bs_final_mm = float(bs_before_mm) if final_uses_a1 else float(a2_bs_after_mm)
            hs2_final_mm = float(hs2_before_mm) if final_uses_a1 else float(a2_hs2_after_mm)

            # bs0 and wst0 are derived from the chosen standard wire and slot pitch at bore
            bs0_mm_res = None
            if d_std_mm is not None:
                try:
                    bs0_mm_res = 3.0 * float(d_std_mm)
                except Exception:
                    bs0_mm_res = None
            wst0_mm_res = wst0_mm
            if wst0_mm_res is None and slot_pitch_D_mm is not None and bs0_mm_res is not None:
                try:
                    wst0_mm_res = float(slot_pitch_D_mm) - float(bs0_mm_res)
                except Exception:
                    wst0_mm_res = None

            text += self.format_equation("hs0", f"{float(hs0_mm):.3f} mm")
            text += self.format_equation("hs1", f"{float(hs1_mm):.3f} mm")
            if bs0_mm_res is not None:
                text += self.format_equation("bs0", f"{float(bs0_mm_res):.3f} mm")
            else:
                text += self.format_equation("bs0", "—")
            if wst0_mm_res is not None:
                text += self.format_equation("wst0", f"{float(wst0_mm_res):.3f} mm")
            else:
                text += self.format_equation("wst0", "—")

            text += self.format_equation("wst1(final)", f"{float(wst1_final_mm):.3f} mm")
            text += self.format_equation("bs(final)", f"{float(bs_final_mm):.3f} mm")
            text += self.format_equation("hs2(final)", f"{float(hs2_final_mm):.3f} mm")

            # Persist slot heights for downstream tabs (Outer Diameter uses hs = hs0 + hs1 + hs2)
            # We store the hs2 that corresponds to the accepted approach:
            # - If exact solution exists (or Approach 1 succeeds): keep hs2_before
            # - Otherwise: use Approach 2 hs2
            try:
                hs2_for_outer_mm = float(hs2_before_mm) if (solved_exact or a1_success) else float(a2_hs2_after_mm)
                hs0_for_outer_mm = float(hs0_mm)
                hs1_for_outer_mm = float(hs1_mm)
                hs_total_mm = hs0_for_outer_mm + hs1_for_outer_mm + hs2_for_outer_mm

                if not hasattr(geom, 'validation_info') or geom.validation_info is None:
                    geom.validation_info = {}
                geom.validation_info['hs0_mm'] = hs0_for_outer_mm
                geom.validation_info['hs1_mm'] = hs1_for_outer_mm
                geom.validation_info['hs2_mm'] = hs2_for_outer_mm
                geom.validation_info['hs_total_mm'] = hs_total_mm
                geom.validation_info['hs_total_breakdown_mm'] = {
                    'hs0_mm': hs0_for_outer_mm,
                    'hs1_mm': hs1_for_outer_mm,
                    'hs2_mm': hs2_for_outer_mm,
                }

                # Also update geometry.h_slot (in meters) so any direct use is consistent.
                geom.h_slot = hs_total_mm / 1000.0
            except Exception:
                pass

            self.append_to_tab(self.tab_slots, text)
            return

        # -----------------------------------------------------------------
        # Semi open slot method (previous Forward Design)
        # -----------------------------------------------------------------
        if method == 'semi_open_slot':
            text += self.make_separator()
            text += f"1. {semi_open_label}: Tooth & Yoke Flux Density Targets\n"
            text += self.make_separator()

            text += "    Allowable ranges:\n"
            text += "      - Tooth flux density: 1.4 to 2.1 T\n"
            text += "      - Yoke flux density : 1.4 to 1.7 T\n\n"

            text += "    Selection rules:\n"
            text += "      - Do not exceed the selected steel Max_Design_B\n"
            text += "      - Yoke target must be smaller than tooth target\n"
            text += "      - Tooth target: may use the maximum allowed by the steel\n"
            text += "      - Yoke target: choose as low as possible to reduce magnetizing current\n\n"

            text += f"    Selected steel grade: {grade}\n"
            if steel_max_b is not None:
                try:
                    text += f"    Steel Max_Design_B: {float(steel_max_b):.3f} T\n\n"
                except (TypeError, ValueError):
                    text += f"    Steel Max_Design_B: {steel_max_b}\n\n"
            else:
                text += "    Steel Max_Design_B: —\n\n"

            text += f"    Chosen tooth target B_tooth = {float(b_tooth_target):.3f} T\n" if b_tooth_target is not None else "    Chosen tooth target B_tooth = —\n"
            text += f"    Chosen yoke target  B_yoke  = {float(b_yoke_target):.3f} T\n\n" if b_yoke_target is not None else "    Chosen yoke target  B_yoke  = —\n\n"

            text += self.make_separator()
            text += f"2. {semi_open_label}: Maximum Tooth Flux and Tooth/Slot Width\n"
            text += self.make_separator()
            text += "    Formula: phi_max = flux_pole × sin(pi × P / (2 × Ss))\n\n"
            if phi_max is not None:
                text += self.format_equation(
                    f"phi_max = {float(flux_pole_used):.8f} × sin(pi × {int(P)} / (2 × {int(Ss)}))",
                    f"{float(phi_max):.8f} Wb = {float(phi_max)*1000:.4f} mWb"
                )
            else:
                text += "    phi_max = — (not available)\n"

            text += "\n    Formula: wst = phi_max / (B_tooth × Li)\n\n"
            if wst_m is not None and phi_max is not None and b_tooth_target is not None:
                text += self.format_equation(
                    f"wst = {float(phi_max):.8f} / ({float(b_tooth_target):.3f} × {float(Li_used):.6f})",
                    f"{float(wst_m):.8f} m = {float(wst_m)*1000:.3f} mm"
                )
            else:
                text += "    wst = — (not available)\n"

            text += self.make_separator()
            text += f"3. {semi_open_label}: Copper Area, Strands, and Standard Wire Selection\n"
            text += self.make_separator()

            hs0_mm = geom_val.get('hs0_mm', 1.0)
            hs1_mm = geom_val.get('hs1_mm', 3.0)
            Iph_A = geom_val.get('Iph_A', elec.I_s)
            J = geom_val.get('J_A_per_mm2', None)
            ac_mm2 = geom_val.get('ac_mm2', None)
            strands = geom_val.get('number_of_strands', None)
            d_calc_mm = geom_val.get('strand_diameter_mm_calc', None)
            d_max_mm = geom_val.get('strand_diameter_mm_max', 1.7)
            swg = geom_val.get('swg_selected', None)
            d_std_mm = geom_val.get('strand_diameter_mm_std', None)
            ac_std_mm2 = geom_val.get('ac_std_mm2', None)
            Tac_mm2 = geom_val.get('Tac_mm2', None)
            A_slot_mm2 = geom_val.get('A_slot_mm2', None)
            bs0_mm = geom_val.get('bs0_mm', None)
            wst0_mm = geom_val.get('wst0_mm', None)

            text += f"    Constants: hs0 = {float(hs0_mm):.1f} mm, hs1 = {float(hs1_mm):.1f} mm (fixed)\n\n"
            text += "    Formula: ac = Iph / J\n\n"
            if J is not None and ac_mm2 is not None:
                text += self.format_equation(
                    f"ac = {float(Iph_A):.4f} / {float(J):.4f}",
                    f"{float(ac_mm2):.6f} mm^2"
                )

            # Strand selection loop (detail) + chosen result
            text += "\n    Strand selection loop (limit strand diameter ≤ 1.7 mm):\n\n"
            text += "    Start: number_of_strands = 1\n"
            text += "    Each iteration: strand_area = ac / number_of_strands; d = sqrt(4×strand_area/pi)\n"
            text += f"    Stop when d ≤ {float(d_max_mm):.2f} mm.\n\n"
            if strands is not None:
                text += self.format_equation("number_of_strands", f"{int(float(strands))}")
            if d_calc_mm is not None:
                ok = "✓" if float(d_calc_mm) <= float(d_max_mm) else "✗"
                text += self.format_equation("d(calc)", f"{float(d_calc_mm):.4f} mm {ok} (max {float(d_max_mm):.1f} mm)")

            # Standard wire selection
            text += "\n    Standard wire selection (SWG):\n\n"
            text += "    Rule: pick the smallest standard diameter that is ≥ d(calc) (never pick smaller).\n\n"
            if swg is not None and d_std_mm is not None:
                text += self.format_equation(
                    "SWG selected",
                    f"SWG {int(float(swg))} → d(std) = {float(d_std_mm):.4f} mm"
                )

            # Updated copper areas using d(std)
            text += "\n    Updated copper areas using selected standard wire:\n\n"
            text += "    a_strand = pi × d(std)^2 / 4\n"
            text += "    ac(std) = number_of_strands × a_strand\n"
            text += "    Tac = ac(std) × Nc\n"
            text += "    A_slot = Tac / k_fill  (k_fill = 0.4)\n\n"
            if d_std_mm is not None:
                a_strand = math.pi * (float(d_std_mm) ** 2) / 4.0
                text += self.format_equation("a_strand", f"{a_strand:.6f} mm^2")
            if ac_std_mm2 is not None:
                text += self.format_equation("ac(std)", f"{float(ac_std_mm2):.6f} mm^2")
            if Tac_mm2 is not None:
                text += self.format_equation("Tac = ac(std) × Nc", f"{float(Tac_mm2):.4f} mm^2")
            if A_slot_mm2 is not None:
                text += self.format_equation("A_slot", f"{float(A_slot_mm2):.4f} mm^2")

            # Slot opening at bore
            text += "\n    Slot opening at bore:\n\n"
            text += "    slot_pitch(D) = pi × D / Ss\n"
            text += "    bs0 = 3 × d(std)\n"
            text += "    wst0 = slot_pitch(D) − bs0\n\n"
            slot_pitch_D_mm = geom_val.get('slot_pitch_D_mm', None)
            if slot_pitch_D_mm is not None:
                text += self.format_equation("slot_pitch(D)", f"{float(slot_pitch_D_mm):.3f} mm")
            if d_std_mm is not None:
                text += self.format_equation("bs0 = 3 × d(std)", f"3 × {float(d_std_mm):.4f} = {3.0*float(d_std_mm):.3f} mm")
            if wst0_mm is not None:
                text += self.format_equation("wst0", f"{float(wst0_mm):.3f} mm")

        # -----------------------------------------------------------------
        # Open slot method
        # -----------------------------------------------------------------
        elif method == 'open_slot':
            text += self.make_separator()
            open_slot_mode = geom_val.get('open_slot_mode', '')
            is_pitch_area_ratio_mode = str(open_slot_mode).endswith('pitch_area_ratio')
            is_lv_pitch_area_ratio = (open_slot_mode == 'lv_pitch_area_ratio')
            is_hv_pitch_area_ratio = (open_slot_mode == 'hv_pitch_area_ratio')

            if is_lv_pitch_area_ratio:
                text += "1. Open slot method (LV): Random-wound conductor sizing (same as semi-open slot)\n"
            else:
                text += "1. Open slot method: Rectangular strip conductor sizing\n"
            text += self.make_separator()

            if is_lv_pitch_area_ratio:
                # Random-wound conductor sizing (reuse semi-open logic keys)
                Iph_A = geom_val.get('Iph_A', elec.I_s)
                J = geom_val.get('J_A_per_mm2', None)
                ac_mm2 = geom_val.get('ac_mm2', None)
                n_strands = geom_val.get('number_of_strands', None)
                d_calc = geom_val.get('strand_diameter_mm_calc', None)
                d_std = geom_val.get('strand_diameter_mm_std', None)
                swg = geom_val.get('swg_selected', None)
                ac_std_mm2 = geom_val.get('ac_std_mm2', None)
                Tac_mm2 = geom_val.get('Tac_mm2', None)
                fill_factor = geom_val.get('fill_factor', None)
                A_slot_mm2 = geom_val.get('A_slot_mm2', None)

                text += "    Step 1 — Conductor area:\n\n"
                text += "    Formula: ac = Iph / J\n\n"
                if J is not None and ac_mm2 is not None:
                    text += self.format_equation(
                        f"ac = {float(Iph_A):.4f} / {float(J):.4f}",
                        f"{float(ac_mm2):.6f} mm^2"
                    )
                else:
                    text += "    ac = —\n"

                text += "\n    Step 2 — Stranding + SWG standardization (max strand diameter constraint):\n\n"
                if n_strands is not None:
                    text += self.format_equation("Number of strands", f"{int(float(n_strands))}")
                if d_calc is not None:
                    text += self.format_equation("Strand diameter (calc)", f"{float(d_calc):.4f} mm")
                if swg is not None and d_std is not None:
                    text += self.format_equation("Selected SWG", f"{float(swg):.0f}  (d(std) = {float(d_std):.4f} mm)")
                elif d_std is not None:
                    text += self.format_equation("Selected d(std)", f"{float(d_std):.4f} mm")
                if ac_std_mm2 is not None:
                    text += self.format_equation("ac(std)", f"{float(ac_std_mm2):.6f} mm^2")

                text += "\n    Step 3 — Slot area using fill factor (random-wound):\n\n"
                text += "    Formula: Tac = ac(std) × Nc\n"
                text += "    Formula: A_slot = Tac / k_fill\n\n"
                if Tac_mm2 is not None:
                    text += self.format_equation("Tac", f"{float(Tac_mm2):.4f} mm^2")
                if fill_factor is not None:
                    text += self.format_equation("k_fill", f"{float(fill_factor):.3f}")
                if A_slot_mm2 is not None:
                    text += self.format_equation("A_slot", f"{float(A_slot_mm2):.4f} mm^2")
            else:
                # Rectangular strip sizing (existing)
                Iph_A = geom_val.get('Iph_A', elec.I_s)
                J = geom_val.get('J_A_per_mm2', None)
                ac_mm2 = geom_val.get('ac_mm2', None)

                I_min = geom_val.get('open_slot_I_min_A', None)
                I_max = geom_val.get('open_slot_I_max_A', None)
                t_min = geom_val.get('open_slot_thickness_min_mm', None)
                t_max = geom_val.get('open_slot_thickness_max_mm', None)
                w_min = geom_val.get('open_slot_width_min_mm', None)
                w_max = geom_val.get('open_slot_width_max_mm', None)

                t_std = geom_val.get('strip_thickness_mm_std', None)
                w_start = geom_val.get('strip_width_mm_start', None)
                w_step = geom_val.get('strip_width_mm_step', 0.5)
                w_std = geom_val.get('strip_width_mm_std', None)
                iter_count = geom_val.get('strip_width_iter_count', None)
                width_satisfied = geom_val.get('strip_width_satisfied', None)
                width_exceeds_max = geom_val.get('open_slot_width_exceeds_max', None)

                ac_std_mm2 = geom_val.get('ac_std_mm2', None)
                optimal_found = geom_val.get('open_slot_optimal_combo_found', None)

                text += "    Step 1 — Conductor area (same rule as semi-open slot):\n\n"
                text += "    Formula: ac = Iph / J\n\n"
                if J is not None and ac_mm2 is not None:
                    text += self.format_equation(
                        f"ac = {float(Iph_A):.4f} / {float(J):.4f}",
                        f"{float(ac_mm2):.6f} mm^2"
                    )
                else:
                    text += "    ac = —\n"

                text += "\n    Step 2 — Select thickness range from the Open Slot abaques (by phase current):\n\n"
                if I_min is not None and I_max is not None:
                    imax_str = "inf" if (isinstance(I_max, float) and math.isinf(float(I_max))) else f"{float(I_max):.1f}"
                    text += f"    Abaque current row: {float(I_min):.1f} ≤ Iph < {imax_str} (A)\n\n"
                if t_min is not None and t_max is not None:
                    text += f"    Recommended thickness range: {float(t_min):.2f} to {float(t_max):.2f} mm\n"
                if w_min is not None and w_max is not None:
                    text += f"    Recommended width range    : {float(w_min):.2f} to {float(w_max):.2f} mm\n\n"

                if t_std is not None:
                    text += self.format_equation("Choose a standard thickness within the range", f"t(std) = {float(t_std):.3f} mm")

                text += "\n    Step 3 — Width selection by iteration (0.5 mm steps):\n\n"
                text += "    Rule: start from w_min and increase width by 0.5 mm until:\n"
                text += "      A_strip = w × t(std)  ≥  ac\n\n"
                if w_start is not None:
                    text += self.format_equation("Start width", f"w(start) = {float(w_start):.3f} mm")
                if w_step is not None:
                    text += self.format_equation("Step", f"Δw = {float(w_step):.3f} mm")
                if iter_count is not None:
                    text += self.format_equation("Iterations", f"{int(float(iter_count))}")
                if w_std is not None and t_std is not None:
                    text += self.format_equation("Selected width", f"w(std) = {float(w_std):.3f} mm")
                    text += self.format_equation("A_strip = w(std) × t(std)", f"{float(w_std) * float(t_std):.6f} mm^2")
                if ac_mm2 is not None:
                    text += self.format_equation("Target ac", f"{float(ac_mm2):.6f} mm^2")

                if optimal_found is not None and float(optimal_found) < 0.5:
                    text += "\n    Warning: No optimal thickness/width combination was found within the recommended ranges.\n"
                    if width_exceeds_max is not None and float(width_exceeds_max) > 0.5:
                        text += "    The width was clamped to the recommended maximum for the closest available combination.\n"

            voltage_v = getattr(self.designer.specs, 'voltage_v', None)
            power_kw = getattr(self.designer.specs, 'power_kw', None)
            hv_open_slot = (open_slot_mode == 'hv_pitch_area_ratio')
            try:
                hv_open_slot = hv_open_slot or (float(voltage_v or 0.0) > 600.0) or (float(power_kw or 0.0) > 372.85)
            except (TypeError, ValueError):
                pass

            if is_pitch_area_ratio_mode or hv_open_slot:
                text += self.make_separator()
                title = "2. Open slot method (pitch/area/ratio): Tooth width at bore (wst1) from max flux"
                if is_hv_pitch_area_ratio or hv_open_slot:
                    title = "2. Open slot method (HV): Tooth width at bore (wst1) from max flux"
                elif is_lv_pitch_area_ratio:
                    title = "2. Open slot method (LV): Tooth width at bore (wst1) from max flux"
                text += title + "\n"
                text += self.make_separator()

                slot_pitch_D_mm = geom_val.get('open_slot_slot_pitch_D_mm', None)
                wst1_bore_mm = geom_val.get('open_slot_wst1_mm', None)
                phi_max_used = geom_val.get('open_slot_phi_max_used_Wb', phi_max)
                B_tooth_max_used = geom_val.get('open_slot_B_tooth_max_used_T', geom_val.get('open_slot_steel_max_B_T', None))
                Li_used_m = geom_val.get('open_slot_Li_used_m', None)

                text += "    phi_max = flux_pole × sin(pi × P / (2 × Ss))\n"
                text += "    wst1 = phi_max / (B_tooth(max) × Li)\n\n"
                if phi_max_used is not None:
                    text += self.format_equation("phi_max(used)", f"{float(phi_max_used):.8f} Wb")
                if B_tooth_max_used is not None:
                    text += self.format_equation("B_tooth(max used)", f"{float(B_tooth_max_used):.4f} T")
                if Li_used_m is not None:
                    text += self.format_equation("Li(used)", f"{float(Li_used_m):.6f} m")
                if wst1_bore_mm is not None:
                    text += self.format_equation("wst1", f"{float(wst1_bore_mm):.3f} mm")
                if slot_pitch_D_mm is not None:
                    text += self.format_equation("slot_pitch(D)", f"{float(slot_pitch_D_mm):.3f} mm")

                text += self.make_separator()
                if is_hv_pitch_area_ratio or hv_open_slot:
                    text += "3. Open slot method (HV): Slot area from fill factor (0.7)\n"
                else:
                    text += "3. Open slot method (LV): Slot area from fill factor (0.4)\n"
                text += self.make_separator()

                A_strip_mm2_used = geom_val.get('open_slot_A_strip_mm2_used', None)
                A_cond_mm2_used = geom_val.get('open_slot_A_conductor_mm2_used', None)
                Nc_slot = geom_val.get('open_slot_Nc_per_slot', None)
                Tac_mm2_used = geom_val.get('open_slot_Tac_mm2_used', None)
                fill_factor_used = geom_val.get('open_slot_fill_factor_used', 0.7)
                A_slot_mm2 = geom_val.get('A_slot_mm2', None)

                text += "    Formula: A_slot = (A_strip × Nc_slot) / k_fill\n"
                if is_hv_pitch_area_ratio or hv_open_slot:
                    text += "    Here: k_fill = 0.7\n\n"
                else:
                    text += "    Here: k_fill = 0.4\n\n"

                if A_strip_mm2_used is not None:
                    text += self.format_equation("A_strip", f"{float(A_strip_mm2_used):.6f} mm^2")
                if A_cond_mm2_used is not None:
                    text += self.format_equation("A_conductor (ac_std)", f"{float(A_cond_mm2_used):.6f} mm^2")
                if Nc_slot is not None:
                    text += self.format_equation("Nc per slot", f"{int(float(Nc_slot))}")
                if Tac_mm2_used is not None:
                    text += self.format_equation("Tac = A_strip×Nc", f"{float(Tac_mm2_used):.4f} mm^2")
                if fill_factor_used is not None:
                    text += self.format_equation("k_fill(used)", f"{float(fill_factor_used):.3f}")
                if A_slot_mm2 is not None:
                    text += self.format_equation("A_slot", f"{float(A_slot_mm2):.4f} mm^2")

                text += self.make_separator()
                if is_hv_pitch_area_ratio or hv_open_slot:
                    text += "4. Open slot method (HV): Solve geometry (bs, hs) from pitch/area/ratio\n"
                else:
                    text += "4. Open slot method (LV): Solve geometry (bs, hs) from pitch/area/ratio\n"
                text += self.make_separator()

                bs_mm = geom_val.get('open_slot_bs_mm', None)
                hs_mm = geom_val.get('open_slot_hs_mm', None)
                ratio_hs_bs = geom_val.get('open_slot_ratio_hs_bs', None)
                solved_exact = geom_val.get('open_slot_solved_exact', None)
                solver_mode = geom_val.get('open_slot_solver_mode', None)
                A_slot_req_mm2 = geom_val.get('open_slot_A_slot_required_mm2', geom_val.get('A_slot_mm2', None))
                A_slot_geom_mm2 = geom_val.get('open_slot_A_slot_geom_mm2', None)
                area_excess_mm2 = geom_val.get('open_slot_area_excess_mm2', None)
                pitch_err_mm = geom_val.get('open_slot_pitch_err_mm', None)
                ratio_limit_violated = geom_val.get('open_slot_ratio_limit_violated', None)
                wst1_required_mm = geom_val.get('open_slot_wst1_required_mm', None)
                wst1_delta_mm = geom_val.get('open_slot_wst1_delta_mm', None)
                B_bore_if_wst1_req = geom_val.get('open_slot_B_tooth_bore_if_wst1_required_T', None)

                text += "    Constraints:\n\n"
                text += "      A_slot = bs × hs\n"
                text += "      3 ≤ hs/bs ≤ 5\n"
                text += "      slot_pitch(D) = bs + wst1\n\n"
                if solver_mode is not None:
                    if str(solver_mode) == 'pitch_fixed':
                        text += "    Solve strategy: pitch-fixed (keeps pitch exact)\n\n"
                    elif str(solver_mode) == 'fallback_adjust_bs':
                        text += "    Solve strategy: fallback (adjust bs to satisfy ratio+area; pitch may mismatch)\n\n"
                    else:
                        text += f"    Solve strategy: {solver_mode}\n\n"
                elif solved_exact is not None:
                    mode = "exact" if float(solved_exact) > 0.5 else "closest-feasible"
                    text += f"    Solve mode: {mode}\n\n"
                if bs_mm is not None:
                    text += self.format_equation("bs", f"{float(bs_mm):.3f} mm")
                if hs_mm is not None:
                    text += self.format_equation("hs", f"{float(hs_mm):.3f} mm")
                if ratio_hs_bs is not None:
                    text += self.format_equation("hs/bs", f"{float(ratio_hs_bs):.3f}")

                if solver_mode is not None and str(solver_mode) == 'fallback_increase_wst1':
                    if pitch_err_mm is not None:
                        text += "\n    Pitch mismatch with current wst1 (fallback):\n\n"
                        text += self.format_equation("pitch_err = (bs + wst1) − slot_pitch(D)", f"{float(pitch_err_mm):.4f} mm")
                    text += "\n    Recommendation (feasible because tooth width can be increased):\n\n"
                    if wst1_required_mm is not None:
                        text += self.format_equation("wst1(required) = slot_pitch(D) − bs", f"{float(wst1_required_mm):.3f} mm")
                    if wst1_delta_mm is not None:
                        text += self.format_equation("Increase tooth width by Δwst1", f"{float(wst1_delta_mm):.3f} mm")
                    if B_bore_if_wst1_req is not None:
                        text += self.format_equation("B_tooth_bore(if wst1 increased)", f"{float(B_bore_if_wst1_req):.4f} T")

                text += "\n    Slot area check (geometry must be ≥ required):\n\n"
                if A_slot_req_mm2 is not None:
                    text += self.format_equation("A_slot(required)", f"{float(A_slot_req_mm2):.4f} mm^2")
                if A_slot_geom_mm2 is not None:
                    text += self.format_equation("A_slot(geometry)=bs×hs", f"{float(A_slot_geom_mm2):.4f} mm^2")
                if area_excess_mm2 is not None:
                    text += self.format_equation("A_excess = A_geom − A_req", f"{float(area_excess_mm2):.4f} mm^2")
                if ratio_limit_violated is not None and float(ratio_limit_violated) > 0.5:
                    text += "\n    Warning: hs/bs > 5 (ratio constraint not respected).\n"
                    text += "    Pitch and tooth-flux limit were kept by fixing wst1 and bs from pitch (Option A).\n"
                if solver_mode is not None and str(solver_mode) == 'fallback_increase_wst1':
                    text += "\n    Note: Slot geometry uses hs/bs = 5 and exact area; pitch is satisfied after increasing wst1.\n"

                text += self.make_separator()
                if is_hv_pitch_area_ratio or hv_open_slot:
                    text += "5. Open slot method (HV): Tooth width at top of slot (wst2) and B_tooth(top)\n"
                else:
                    text += "5. Open slot method (LV): Tooth width at top of slot (wst2) and B_tooth(top)\n"
                text += self.make_separator()

                slot_pitch_D_plus_2hs_mm = geom_val.get('open_slot_slot_pitch_D_plus_2hs_mm', None)
                wst2_mm = geom_val.get('open_slot_wst2_mm', None)
                B_tooth_bore_T = geom_val.get('open_slot_B_tooth_bore_T', None)
                B_tooth_top_slot = geom_val.get('open_slot_B_tooth_top_slot_T', None)

                if B_tooth_bore_T is not None:
                    text += self.format_equation("B_tooth_bore", f"{float(B_tooth_bore_T):.4f} T")
                text += "\n"
                text += "    Formula: slot_pitch(D+2hs) = pi × (D + 2×hs) / Ss\n"
                text += "    Formula: wst2 = slot_pitch(D+2hs) − bs\n"
                text += "    Formula: B_tooth_top_slot = phi_max / (wst2 × Li)\n\n"
                if slot_pitch_D_plus_2hs_mm is not None:
                    text += self.format_equation("slot_pitch(D+2hs)", f"{float(slot_pitch_D_plus_2hs_mm):.3f} mm")
                if wst2_mm is not None:
                    text += self.format_equation("wst2", f"{float(wst2_mm):.3f} mm")
                if B_tooth_top_slot is not None:
                    text += self.format_equation("B_tooth_top_slot", f"{float(B_tooth_top_slot):.4f} T")

                text += self.make_separator()
                if is_hv_pitch_area_ratio or hv_open_slot:
                    text += "6. Open slot method (HV): Geometry results\n"
                else:
                    text += "6. Open slot method (LV): Geometry results\n"
                text += self.make_separator()

                if is_hv_pitch_area_ratio or hv_open_slot:
                    text += "    Final geometry values (Open slot HV):\n\n"
                else:
                    text += "    Final geometry values (Open slot LV):\n\n"
                text += self.format_equation("Slot width bs", f"{float(bs_mm):.3f} mm" if bs_mm is not None else "—")
                text += self.format_equation("Slot total height hs", f"{float(hs_mm):.3f} mm" if hs_mm is not None else "—")
                text += self.format_equation("Tooth min width wst1", f"{float(wst1_bore_mm):.3f} mm" if wst1_bore_mm is not None else "—")
                text += self.format_equation("Tooth max width wst2", f"{float(wst2_mm):.3f} mm" if wst2_mm is not None else "—")
            else:
                text += self.make_separator()
                text += "2. Open slot method: Geometry report\n"
                text += self.make_separator()
                text += "    Legacy Open-slot (Zsw/Zsh packing) output was removed.\n"
                text += "    Expected open_slot_mode: lv_pitch_area_ratio or hv_pitch_area_ratio.\n"

                text += "    Final geometry values (Open slot):\n\n"
                if bs_mm is not None:
                    text += self.format_equation("Slot width bs", f"{float(bs_mm):.3f} mm")
                else:
                    text += self.format_equation("Slot width bs", "—")

                if hs_mm is not None:
                    text += self.format_equation("Slot total height hs", f"{float(hs_mm):.3f} mm")
                else:
                    text += self.format_equation("Slot total height hs", "—")

                if wst1_bore_mm is not None:
                    text += self.format_equation("Tooth min width wst1", f"{float(wst1_bore_mm):.3f} mm")
                else:
                    text += self.format_equation("Tooth min width wst1", "—")

                if wst2_mm is not None:
                    text += self.format_equation("Tooth max width wst2", f"{float(wst2_mm):.3f} mm")
                else:
                    text += self.format_equation("Tooth max width wst2", "—")

        else:
            text += "Unsupported slot method selection.\n"

        # Common slot-dimension results (bs1/bs2/hs2) for both methods
        bs1_mm = geom_val.get('bs1_mm', None)
        hs2_mm = geom_val.get('hs2_mm', None)
        bs2_mm = geom_val.get('bs2_mm', None)
        Ass_mm2 = geom_val.get('Ass_mm2', None)
        Err = geom_val.get('Err_slot_area', None)
        bsmean_mm = geom_val.get('bsmean_mm', None)
        ratio_hs2 = geom_val.get('hs2_over_bsmean', None)

        # Ensure hs0/hs1/hs2 are available and consistent for outer diameter when using semi-open/tapered slot.
        # Outer diameter uses: hs = hs0 + hs1 + hs2.
        if method == 'semi_open_slot' and hs2_mm is not None:
            try:
                hs0_for_outer_mm = float(geom_val.get('hs0_mm', 1.0) or 1.0)
                hs1_for_outer_mm = float(geom_val.get('hs1_mm', 3.0) or 3.0)
                hs2_for_outer_mm = float(hs2_mm)
                hs_total_mm = hs0_for_outer_mm + hs1_for_outer_mm + hs2_for_outer_mm
                if not hasattr(geom, 'validation_info') or geom.validation_info is None:
                    geom.validation_info = {}
                geom.validation_info['hs0_mm'] = hs0_for_outer_mm
                geom.validation_info['hs1_mm'] = hs1_for_outer_mm
                geom.validation_info['hs2_mm'] = hs2_for_outer_mm
                geom.validation_info['hs_total_mm'] = hs_total_mm
                geom.validation_info['hs_total_breakdown_mm'] = {
                    'hs0_mm': hs0_for_outer_mm,
                    'hs1_mm': hs1_for_outer_mm,
                    'hs2_mm': hs2_for_outer_mm,
                }
                geom.h_slot = hs_total_mm / 1000.0
            except Exception:
                pass

        if method == 'semi_open_slot' and bs1_mm is not None and hs2_mm is not None and bs2_mm is not None:
            text += self.make_separator()
            text += f"4. {semi_open_label}: Slot dimensioning (bs1, bs2, hs2)\n"
            text += self.make_separator()

            # Add the missing calculation details (same geometry equations used by backend)
            hs0_mm = float(geom_val.get('hs0_mm', 1.0))
            hs1_mm = float(geom_val.get('hs1_mm', 3.0))
            D_mm = float(getattr(geom, 'D', 0.0)) * 1000.0
            wst_mm = float(wst_m) * 1000.0 if wst_m is not None else None
            alpha = math.pi / float(Ss)

            text += "    Formulas (mm):\n\n"
            text += "      bs1 = 2[ tan(pi/Ss)×((D/2)+hs0+hs1) − (wst/2)/cos(pi/Ss) ]\n"
            text += "      bs2 = 2[ tan(pi/Ss)×((D/2)+hs0+hs1+hs2) − (wst/2)/cos(pi/Ss) ]\n"
            text += "      Ass = (bs1 + bs2)×hs2/2\n\n"

            if wst_mm is not None:
                t = math.tan(alpha)
                c = math.cos(alpha)
                term_wst = (wst_mm / 2.0) / c
                bs1_calc = 2.0 * (t * ((D_mm / 2.0) + hs0_mm + hs1_mm) - term_wst)
                bs2_calc = 2.0 * (t * ((D_mm / 2.0) + hs0_mm + hs1_mm + float(hs2_mm)) - term_wst)
                Ass_calc = (float(bs1_mm) + float(bs2_mm)) * float(hs2_mm) / 2.0

                text += self.format_equation(
                    "Inputs",
                    f"D={D_mm:.3f} mm; Ss={int(Ss)}; hs0={hs0_mm:.2f} mm; hs1={hs1_mm:.2f} mm; wst={wst_mm:.3f} mm"
                )
                text += self.format_equation(
                    "bs1(calc)",
                    f"{bs1_calc:.3f} mm"
                )
                text += self.format_equation(
                    "bs2(calc)",
                    f"{bs2_calc:.3f} mm"
                )
                text += self.format_equation(
                    "Ass(calc) = (bs1+bs2)×hs2/2",
                    f"{Ass_calc:.3f} mm^2"
                )
                text += "\n"

            text += self.format_equation("bs1", f"{float(bs1_mm):.3f} mm")
            text += self.format_equation("hs2", f"{float(hs2_mm):.3f} mm")
            text += self.format_equation("bs2", f"{float(bs2_mm):.3f} mm")
            if Ass_mm2 is not None:
                text += self.format_equation("Ass (computed)", f"{float(Ass_mm2):.3f} mm^2")
            if Err is not None:
                text += self.format_equation("Relative error", f"{float(Err)*100:.2f} %")

        # Semi-open slot: provide iteration/check explanation once (after results)
        if method == 'semi_open_slot':
            Niter_slot = geom_val.get('Niter_slot_area', None)
            slot_pitch_plus_mm = geom_val.get('slot_pitch_D_plus_hs1_mm', None)
            ratio_hs2_bsmean = geom_val.get('hs2_over_bsmean', None)
            A_slot_mm2_iter = geom_val.get('A_slot_mm2', None)
            Ass_mm2_iter = geom_val.get('Ass_mm2', None)
            Err_slot = geom_val.get('Err_slot_area', None)
            bs1_mm_iter = geom_val.get('bs1_mm', None)
            bs2_mm_iter = geom_val.get('bs2_mm', None)
            hs2_mm_iter = geom_val.get('hs2_mm', None)

            text += self.make_separator()
            text += f"5. {semi_open_label}: Iteration & Geometry Checks\n"
            text += self.make_separator()
            text += "    How bs2 and hs2 are obtained (slot-area matching loop):\n\n"
            text += "    Goal: find hs2 such that the trapezoidal slot area Ass matches the required slot area A_slot.\n\n"
            text += "    At each iteration k:\n"
            text += "      1) Increase hs2 by a fixed step (Δhs2 = 0.1 mm)\n"
            text += "      2) Recompute bs2 from the slot geometry equation\n"
            text += "      3) Compute Ass = (bs1 + bs2)×hs2/2\n"
            text += "      4) Compute Err = |Ass − A_slot| / A_slot\n"
            text += "    Stop when Err ≤ 2% (or when the iteration limit is reached).\n\n"

            text += "    Convergence values (final iteration results):\n\n"
            if hs2_mm_iter is not None:
                text += self.format_equation("hs2(final)", f"{float(hs2_mm_iter):.3f} mm")
            if bs1_mm_iter is not None:
                text += self.format_equation("bs1(final)", f"{float(bs1_mm_iter):.3f} mm")
            if bs2_mm_iter is not None:
                text += self.format_equation("bs2(final)", f"{float(bs2_mm_iter):.3f} mm")
            if Ass_mm2_iter is not None:
                text += self.format_equation("Ass(final)", f"{float(Ass_mm2_iter):.3f} mm^2")
            if A_slot_mm2_iter is not None:
                text += self.format_equation("A_slot(required)", f"{float(A_slot_mm2_iter):.3f} mm^2")
            if Err_slot is not None:
                text += self.format_equation("Err(final)", f"{float(Err_slot)*100:.2f} %")
            if Niter_slot is not None:
                text += self.format_equation("Niter", f"{int(float(Niter_slot))}")

            # Section 6 — Results summary
            text += self.make_separator()
            text += f"6. {semi_open_label}: Results\n"
            text += self.make_separator()

            # Requested values
            bs0_mm_res = geom_val.get('bs0_mm', None)
            wst0_mm_res = geom_val.get('wst0_mm', None)
            hs0_mm_res = geom_val.get('hs0_mm', 1.0)
            hs1_mm_res = geom_val.get('hs1_mm', 3.0)
            wst_mm_res = (float(wst_m) * 1000.0) if wst_m is not None else None

            if bs0_mm_res is not None:
                text += self.format_equation("bs0", f"{float(bs0_mm_res):.3f} mm")
            else:
                text += self.format_equation("bs0", "—")
            text += self.format_equation("hs0", f"{float(hs0_mm_res):.3f} mm")
            text += self.format_equation("hs1", f"{float(hs1_mm_res):.3f} mm")

            if bs1_mm_iter is not None:
                text += self.format_equation("bs1(final)", f"{float(bs1_mm_iter):.3f} mm")
            else:
                text += self.format_equation("bs1(final)", "—")
            if hs2_mm_iter is not None:
                text += self.format_equation("hs2(final)", f"{float(hs2_mm_iter):.3f} mm")
            else:
                text += self.format_equation("hs2(final)", "—")
            if bs2_mm_iter is not None:
                text += self.format_equation("bs2(final)", f"{float(bs2_mm_iter):.3f} mm")
            else:
                text += self.format_equation("bs2(final)", "—")

            if wst0_mm_res is not None:
                text += self.format_equation("wst0", f"{float(wst0_mm_res):.3f} mm")
            else:
                text += self.format_equation("wst0", "—")
            if wst_mm_res is not None:
                text += self.format_equation("wst", f"{float(wst_mm_res):.3f} mm")
            else:
                text += self.format_equation("wst", "—")

            # Move the hs2/bsmean check here (requested)
            text += "\n    hs2/bsmean check:\n\n"
            text += "      slot_pitch(D+hs1) = pi × (D + hs1) / Ss\n"
            text += "      bsmean = slot_pitch(D+hs1) − wst\n"
            text += "      Ratio = hs2 / bsmean\n\n"
            if slot_pitch_plus_mm is not None:
                text += self.format_equation("slot_pitch(D+hs1)", f"{float(slot_pitch_plus_mm):.3f} mm")
            if bsmean_mm is not None:
                text += self.format_equation("bsmean", f"{float(bsmean_mm):.3f} mm")
            if ratio_hs2_bsmean is not None:
                text += self.format_equation("hs2/bsmean", f"{float(ratio_hs2_bsmean):.3f}")

        self.append_to_tab(self.tab_slots, text)

    def show_outer_diameter(self):
        """Show outer diameter calculation with equations"""
        text = self.make_header("STATOR OUTER DIAMETER CALCULATION")
        
        self.designer.calculate_outer_diameter()
        geom = self.designer.geometry
        
        # Flux in yoke
        text += self.make_separator()
        text += "1. Flux in Stator Yoke (Back Iron)\n"
        text += self.make_separator()
        text += "    Flux splits to both sides of the core:\n\n"
        text += "    Formula: Phi_yoke = Phi / 2  [Wb]\n\n"
        flux_yoke = 0.5 * geom.flux_per_pole
        text += self.format_equation(
            f"Phi_yoke = {geom.flux_per_pole:.8f} / 2",
            f"{flux_yoke:.8f} Wb = {flux_yoke*1000:.4f} mWb"
        )
        
        # Core flux density
        text += self.make_separator()
        text += "2. Core Flux Density (B_c)\n"
        text += self.make_separator()
        val_info = geom.validation_info if hasattr(geom, 'validation_info') else {}
        slot_method = getattr(self.designer.specs, 'slot_method', 'semi_open_slot') or 'semi_open_slot'
        if slot_method in ('semi_open_slot', 'semi_open_slot_new', 'open_slot'):
            steel_data = getattr(self.designer.specs, 'steel_data', None) or {}
            steel_grade = steel_data.get('Grade', getattr(self.designer.specs, 'steel_grade', '—'))
            steel_max = steel_data.get('Max_Design_B', None)
            B_c = val_info.get('B_c_used', val_info.get('B_yoke_target', 1.3))
            text += "    Semi-open/Open slot: yoke flux density is selected from steel grade\n"
            text += "    Base range: 1.4 to 1.7 T (and must be < tooth target)\n"
            if steel_max is not None:
                try:
                    text += f"    Steel: {steel_grade} | Max_Design_B = {float(steel_max):.2f} T\n"
                except (TypeError, ValueError):
                    text += f"    Steel: {steel_grade} | Max_Design_B = {steel_max}\n"
            else:
                text += f"    Steel: {steel_grade}\n"
            text += f"    Using: B_yoke = {B_c:.3f} T\n\n"
        else:
            text += "    Typical range: 1.2 to 1.4 T\n"
            text += "    Using: B_c = 1.3 T\n\n"
            B_c = 1.3
        
        # Core area
        text += self.make_separator()
        text += "3. Core Cross-Sectional Area (A_c)\n"
        text += self.make_separator()
        text += "    From flux equation:\n\n"
        text += "    Formula: A_c = Phi_yoke / B_c  [m^2]\n\n"
        A_c = flux_yoke / B_c
        text += self.format_equation(
            f"A_c = {flux_yoke:.8f} / {B_c}",
            f"{A_c:.8f} m^2"
        )
        
        # Core depth
        text += self.make_separator()
        text += "4. Core Depth (d_cs)\n"
        text += self.make_separator()
        text += "    Radial thickness of back iron:\n\n"
        text += "    Formula: d_cs = A_c / Li  [m]\n"
        text += "    (Using Li = net iron length = Ls × ki)\n\n"
        Li = geom.Li if hasattr(geom, 'Li') else ((geom.Ls * geom.ki) if hasattr(geom, 'Ls') else geom.L)
        text += self.format_equation(
            f"d_cs = {A_c:.8f} / {Li:.6f}",
            f"{geom.d_cs:.6f} m = {geom.d_cs*1000:.2f} mm"
        )
        
        # Outer diameter
        text += self.make_separator()
        text += "5. Stator Outer Diameter (D_ext)\n"
        text += self.make_separator()
        text += "    Total diameter including slots and core:\n\n"
        text += "    Formula: D_ext = D + 2 x h_slot + 2 x d_cs  [m]\n\n"
        
        # Get calculated and rounded values
        val_info = geom.validation_info if hasattr(geom, 'validation_info') else {}
        D_ext_calculated = val_info.get('D_ext_calculated', geom.D_ext)
        D_ext_rounded = val_info.get('D_ext_rounded', geom.D_ext)

        # Slot height used for outer diameter
        slot_method = getattr(self.designer.specs, 'slot_method', 'semi_open_slot') or 'semi_open_slot'
        h_slot_used = val_info.get('h_slot_used_for_Dext', geom.h_slot)
        h_slot_source = val_info.get('h_slot_source_for_Dext', 'geometry.h_slot')
        
        text += self.format_equation(
            f"D_ext = {geom.D:.6f} + 2x{float(h_slot_used):.6f} + 2x{geom.d_cs:.6f}",
            f"{D_ext_calculated:.6f} m = {D_ext_rounded:.6f} m"
        )
        text += self.format_equation(
            f"",
            f"{D_ext_calculated*1000:.2f} mm = {D_ext_rounded*1000:.0f} mm"
        )
        
        if slot_method in ('semi_open_slot', 'semi_open_slot_new', 'open_slot'):
            if slot_method in ('semi_open_slot', 'semi_open_slot_new'):
                hs_total_mm = val_info.get('hs_total_mm', None)
                hs_break = val_info.get('hs_total_breakdown_mm', None)
                if hs_total_mm is not None and isinstance(hs_break, dict):
                    try:
                        text += "\n    Total slot height used in outer diameter\n"
                        text += self.format_equation(
                            "hs = hs0 + hs1 + hs2",
                            f"{float(hs_break.get('hs0_mm', 0.0)):.2f} + {float(hs_break.get('hs1_mm', 0.0)):.2f} + {float(hs_break.get('hs2_mm', 0.0)):.2f} = {float(hs_total_mm):.2f} mm"
                        )
                    except Exception:
                        pass
            else:
                open_slot_hs_mm = val_info.get('open_slot_hs_mm', None)
                if open_slot_hs_mm is not None:
                    try:
                        text += "\n    Total slot height used in outer diameter\n"
                        text += self.format_equation("hs", f"{float(open_slot_hs_mm):.2f} mm")
                    except Exception:
                        pass

            try:
                text += f"    Using h_slot = hs = {float(h_slot_used)*1000:.2f} mm\n"
            except Exception:
                text += "    Using h_slot = hs (total slot height)\n"
            text += f"    (h_slot source: {h_slot_source})\n"
            text += "    OK: Using steel-based yoke flux density target\n"
        else:
            text += "\n    OK: Core flux density within acceptable range (1.2-1.4 T)\n"
        
        self.append_to_tab(self.tab_outer, text)
    
    def show_losses_resistance(self):
        """Show loss and resistance calculations with equations"""
        text = self.make_header("WINDING RESISTANCE AND CONDUCTOR LOSSES")

        # Ensure backend computed winding/slot data (and derived weight values)
        self.designer.design_conductor_and_slots()
        # Ensure outer diameter (and core geometry) is computed for core weight
        self.designer.calculate_outer_diameter()
        
        winding = self.designer.winding
        geom = self.designer.geometry
        elec = self.designer.electrical
        P = self.designer.specs.poles
        
        # Use pole pitch from geometry
        tau_pole = geom.tau_pole
        
        # Mean turn length
        text += self.make_separator()
        text += "1. Mean Turn Length (L_mean)\n"
        text += self.make_separator()
        text += "    Average length of one complete turn:\n\n"
        text += "    Formula: L_mean = 2 x L + 2.3 x tau_pole + 0.24  [m]\n"
        text += "             where tau_pole is the pole pitch (from Main Dimensions)\n\n"
        text += self.format_equation(
            f"L_mean = 2x{geom.L:.6f} + 2.3x{tau_pole:.6f} + 0.24",
            f"{winding.L_mean:.6f} m = {winding.L_mean*1000:.2f} mm"
        )
        
        # Conductor resistivity (material-aware)
        text += self.make_separator()
        text += "2. Conductor Resistivity (ρ)\n"
        text += self.make_separator()
        val_info = geom.validation_info if hasattr(geom, 'validation_info') and geom.validation_info else {}
        rho_used = val_info.get('conductor_rho_ohm_m_used', None)
        material_used = val_info.get('conductor_material_used', None)
        if rho_used is None:
            rho_used = getattr(self.designer.specs, 'conductor_rho_ohm_m', 1.68e-8)
        if material_used is None:
            material_used = getattr(self.designer.specs, 'conductor_material', None)
        try:
            rho_used = float(rho_used)
        except Exception:
            rho_used = 1.68e-8

        text += "    At 20 degrees C (selected material):\n"
        if material_used:
            text += f"    Material: {material_used}\n"
        text += f"    ρ = {rho_used:.3e} Ω·m\n\n"
        
        # Stator resistance
        text += self.make_separator()
        text += "3. Stator Resistance per Phase (R_s)\n"
        text += self.make_separator()
        text += "    DC resistance at 20 degrees C:\n\n"
        text += "    Formula: R_s = (rho x L_mean x TPH) / As  [Ohm]\n"
        text += "             where As is in m^2 (converted from mm^2)\n\n"
        As_m2 = winding.As * 1e-6
        rho = rho_used
        text += self.format_equation(
            f"R_s = ({rho:.2e} x {winding.L_mean:.6f} x {winding.TPH}) / {As_m2:.10f}",
            f"{winding.R_s:.8f} Ohm"
        )
        
        # Conductor losses (I^2R)
        text += self.make_separator()
        text += "4. Conductor Losses in Stator Windings (P_js)\n"
        text += self.make_separator()
        text += "    Total I^2*R losses in all three phases:\n\n"
        text += "    Formula: P_js = 3 x R_s x I_s^2  [W]\n\n"
        text += self.format_equation(
            f"P_js = 3 x {winding.R_s:.8f} x {elec.I_s:.4f}^2",
            f"{winding.copper_losses:.4f} W = {winding.copper_losses/1000:.4f} kW"
        )
        
        # Efficiency impact
        text += self.make_separator()
        text += "5. Conductor Loss Percentage\n"
        text += self.make_separator()
        loss_percentage = (winding.copper_losses / (self.designer.specs.power_kw * 1000)) * 100
        text += f"    Conductor losses as % of rated power:\n"
        text += f"    {loss_percentage:.2f}%\n\n"
        
        if loss_percentage < 3:
            text += "    OK: Conductor losses are low (< 3% of rated power)\n"
        elif loss_percentage < 5:
            text += "    OK: Conductor losses are acceptable (3-5% of rated power)\n"
        else:
            text += "    WARNING: Conductor losses are high (> 5% of rated power)\n"
            text += "    Consider increasing conductor section or reducing current density\n"

        # Conductor weight (material-aware)
        text += self.make_separator()
        text += "6. Conductor Weight (Wcond)\n"
        text += self.make_separator()
        text += "    (As selection depends on slot method)\n\n"

        val_info = geom.validation_info if hasattr(geom, 'validation_info') and geom.validation_info else {}
        slot_method = getattr(self.designer.specs, 'slot_method', 'semi_open_slot') or 'semi_open_slot'
        Wcond_kg = val_info.get('Wcond_kg', None)
        Wcus_kg = val_info.get('Wcus_kg', None)
        As_used_mm2 = val_info.get('Wcus_As_mm2_used', None)
        As_source = val_info.get('Wcus_As_source', None)
        Lmt_used = val_info.get('Wcus_Lmt_m_used', winding.L_mean)
        TPH_used = val_info.get('Wcus_TPH_used', winding.TPH)

        conductor_material_used = val_info.get('conductor_material_used', None)
        conductor_density_g_cm3_used = val_info.get('conductor_density_g_cm3_used', None)
        if conductor_density_g_cm3_used is None:
            # Fallback for older results
            conductor_density_g_cm3_used = 8.9

        if conductor_material_used:
            text += f"    Conductor material: {conductor_material_used}\n"
        text += f"    Density used: {float(conductor_density_g_cm3_used):.2f} g/cm³\n"
        text += f"    Formula: Wcond = Lmt × Tph × 3 × As × {float(conductor_density_g_cm3_used):.2f} × 10^-3   [kg]\n"

        if slot_method == 'open_slot':
            text += "    Open slot method: As = A_strip = w(std) × t(std)\n"
        else:
            ui_method = getattr(self, '_slot_method_ui', None)
            slot_label = "Trapezoidal semi-open slot" if ui_method == 'tapered_slot' else "Semi-open slot"
            text += f"    {slot_label}: As = ac(std)\n"

        if As_source is not None:
            text += self.format_equation("As source", f"{As_source}")
        if As_used_mm2 is not None:
            text += self.format_equation("As", f"{float(As_used_mm2):.6f} mm^2")
        text += self.format_equation("Lmt", f"{float(Lmt_used):.6f} m")
        text += self.format_equation("Tph", f"{int(float(TPH_used))}")

        # Prefer new Wcond, fallback to legacy Wcus
        weight_val = Wcond_kg if Wcond_kg is not None else Wcus_kg
        if weight_val is not None:
            text += "\n"
            text += self.format_equation("Wcond", f"{float(weight_val):.4f} kg")

        # Tooth weight
        text += self.make_separator()
        text += "7. Tooth Weight (Wt)\n"
        text += self.make_separator()
        text += "    (depends on selected slot geometry method)\n\n"

        Wt_kg = val_info.get('Wt_kg', None)
        Wt_method = val_info.get('Wt_method', slot_method)
        A_tooth_m2 = val_info.get('Wt_A_tooth_m2', None)
        Li_m_used = val_info.get('Wt_Li_m_used', val_info.get('Wt_Ls_m_used', None))
        Ss_used = val_info.get('Wt_Ss_used', None)
        Kgm_3_used = val_info.get('Wt_density_kg_m3_used', None)
        steel_grade_used = val_info.get('Wt_steel_grade_used', None)

        if slot_method == 'open_slot':
            text += "    Open slot method:\n"
            text += "    Formula: A_tooth = (wst1 + wst2) × hs / 2\n"
            text += "    Formula: Wt = A_tooth × Li × Ss × Kgm_3   [kg]\n"
            text += "    (A_tooth in m^2, Li in m, Kgm_3 in kg/m^3)\n\n"

            wst1_mm = val_info.get('Wt_wst1_mm_used', val_info.get('open_slot_wst1_mm', None))
            wst2_mm = val_info.get('Wt_wst2_mm_used', val_info.get('open_slot_wst2_mm', None))
            hs_mm = val_info.get('Wt_hs_mm_used', val_info.get('open_slot_hs_mm', None))
            if wst1_mm is not None:
                text += self.format_equation("wst1", f"{float(wst1_mm):.4f} mm")
            if wst2_mm is not None:
                text += self.format_equation("wst2", f"{float(wst2_mm):.4f} mm")
            if hs_mm is not None:
                text += self.format_equation("hs", f"{float(hs_mm):.4f} mm")
        else:
            ui_method = getattr(self, '_slot_method_ui', None)
            slot_label = "Trapezoidal semi-open slot" if ui_method == 'tapered_slot' else "Semi-open slot"
            text += f"    {slot_label}:\n"

            if slot_method == 'semi_open_slot_new':
                text += "    Semi-open slot (new):\n"
                text += "    Formula: slot_pitch(D + 2×(h0+h1)) = pi × (D + 2×(h0+h1)) / Ss\n"
                text += "    Let: w_tooth_top = slot_pitch(D + 2×(h0+h1)) − bs\n"
                text += "    Formula: A_shoe = wst0×h0 + (h1×(wst0 + w_tooth_top)/2)\n"
                text += "    Formula: A_tb = hs2×(wst1 + w_tooth_top)/2\n"
                text += "    Formula: A_tooth = A_shoe + A_tb\n"
                text += "    Formula: Wt = A_tooth × Li × Ss × Kgm_3   [kg]\n"
                text += "    (A_tooth in m^2, Li in m, Kgm_3 in kg/m^3)\n\n"

                hs2_mm = val_info.get('Wt_hs2_mm_used', val_info.get('semi_open_new_hs2_mm', val_info.get('hs2_mm', None)))
                wst0_mm = val_info.get('Wt_wst0_mm_used', val_info.get('wst0_mm', None))
                h0_mm = val_info.get('Wt_h0_mm_used', val_info.get('hs0_mm', None))
                h1_mm = val_info.get('Wt_h1_mm_used', val_info.get('hs1_mm', None))
                wst1_mm = val_info.get('Wt_wst1_mm_used', val_info.get('semi_open_new_wst1_mm', val_info.get('wst_mm', None)))
                bs_mm = val_info.get('Wt_bs_mm_used', val_info.get('semi_open_new_bs_mm', None))
                A_tb_m2 = val_info.get('Wt_A_tb_m2', None)
                A_shoe_m2 = val_info.get('Wt_A_shoe_m2', None)

                # slot_pitch at D + 2*(h0+h1)
                try:
                    D_mm = float(getattr(self.designer.geometry, 'D', 0.0) or 0.0) * 1000.0
                except Exception:
                    D_mm = 0.0
                try:
                    Ss_for_pitch = int(getattr(self.designer.slots, 'Ss', 0) or 0) if self.designer.slots is not None else int(val_info.get('phi_max_Ss', 0) or 0)
                except Exception:
                    Ss_for_pitch = 0
                try:
                    h0_mm_f = float(h0_mm) if h0_mm is not None else 0.0
                except Exception:
                    h0_mm_f = 0.0
                try:
                    h1_mm_f = float(h1_mm) if h1_mm is not None else 0.0
                except Exception:
                    h1_mm_f = 0.0
                slot_pitch_top_shoe_mm = (math.pi * (D_mm + 2.0 * (h0_mm_f + h1_mm_f)) / float(Ss_for_pitch)) if (Ss_for_pitch > 0 and D_mm > 0) else None
                if slot_pitch_top_shoe_mm is not None:
                    text += self.format_equation("slot_pitch(D+2(h0+h1))", f"{float(slot_pitch_top_shoe_mm):.4f} mm")

                if hs2_mm is not None:
                    text += self.format_equation("hs2", f"{float(hs2_mm):.4f} mm")
                if wst0_mm is not None:
                    text += self.format_equation("wst0", f"{float(wst0_mm):.4f} mm")
                if wst1_mm is not None:
                    text += self.format_equation("wst1", f"{float(wst1_mm):.4f} mm")
                if bs_mm is not None:
                    text += self.format_equation("bs", f"{float(bs_mm):.4f} mm")
                if h0_mm is not None:
                    text += self.format_equation("h0", f"{float(h0_mm):.4f} mm")
                if h1_mm is not None:
                    text += self.format_equation("h1", f"{float(h1_mm):.4f} mm")
                if A_tb_m2 is not None:
                    text += self.format_equation("A_tb", f"{float(A_tb_m2):.10f} m^2")
                if A_shoe_m2 is not None:
                    text += self.format_equation("A_shoe", f"{float(A_shoe_m2):.10f} m^2")
            else:
                text += "    Formula: A_tb = hs2 × wst\n"
                text += "    Formula: A_shoe = wst0 × h0 + (h1 × (wst0 + wst) / 2)\n"
                text += "    Formula: A_tooth = A_shoe + A_tb\n"
                text += "    Formula: Wt = A_tooth × Li × Ss × Kgm_3   [kg]\n"
                text += "    (A_tooth in m^2, Li in m, Kgm_3 in kg/m^3)\n\n"

                hs2_mm = val_info.get('Wt_hs2_mm_used', val_info.get('hs2_mm', None))
                wst_mm = val_info.get('Wt_wst_mm_used', val_info.get('slot_dim_wst_mm', val_info.get('wst_mm', None)))
                wst0_mm = val_info.get('Wt_wst0_mm_used', val_info.get('wst0_mm', None))
                h0_mm = val_info.get('Wt_h0_mm_used', val_info.get('slot_dim_hs0_mm', val_info.get('hs0_mm', None)))
                h1_mm = val_info.get('Wt_h1_mm_used', val_info.get('slot_dim_hs1_mm', val_info.get('hs1_mm', None)))
                A_tb_m2 = val_info.get('Wt_A_tb_m2', None)
                A_shoe_m2 = val_info.get('Wt_A_shoe_m2', None)

                if hs2_mm is not None:
                    text += self.format_equation("hs2", f"{float(hs2_mm):.4f} mm")
                if wst_mm is not None:
                    text += self.format_equation("wst", f"{float(wst_mm):.4f} mm")
                if wst0_mm is not None:
                    text += self.format_equation("wst0", f"{float(wst0_mm):.4f} mm")
                if h0_mm is not None:
                    text += self.format_equation("h0", f"{float(h0_mm):.4f} mm")
                if h1_mm is not None:
                    text += self.format_equation("h1", f"{float(h1_mm):.4f} mm")
                if A_tb_m2 is not None:
                    text += self.format_equation("A_tb", f"{float(A_tb_m2):.10f} m^2")
                if A_shoe_m2 is not None:
                    text += self.format_equation("A_shoe", f"{float(A_shoe_m2):.10f} m^2")

        if steel_grade_used is not None:
            text += self.format_equation("Steel grade", f"{steel_grade_used}")
        if Kgm_3_used is not None:
            text += self.format_equation("Kgm_3", f"{float(Kgm_3_used):.0f} kg/m^3")
        if A_tooth_m2 is not None:
            text += self.format_equation("A_tooth", f"{float(A_tooth_m2):.10f} m^2")
        if Li_m_used is not None:
            text += self.format_equation("Li", f"{float(Li_m_used):.6f} m")
        if Ss_used is not None:
            text += self.format_equation("Ss", f"{int(float(Ss_used))}")
        if Wt_method is not None:
            text += self.format_equation("Wt source", f"{Wt_method}")
        if Wt_kg is not None:
            text += "\n"
            text += self.format_equation("Wt", f"{float(Wt_kg):.4f} kg")

        # Core weight
        text += self.make_separator()
        text += "8. Core Weight (Wc)\n"
        text += self.make_separator()
        text += "    Formula: D_mean = D + 2×hs + dcs   [m]\n"
        text += "    Formula: Wc = pi × D_mean × A_c × Kgm_3   [kg]\n"
        text += "    (D_mean in m, A_c in m^2, Kgm_3 in kg/m^3)\n\n"

        Wc_kg = val_info.get('Wc_kg', None)
        D_mean_m = val_info.get('Wc_D_mean_m', None)
        A_c_m2 = val_info.get('Wc_Ac_m2', None)
        dcs_m = val_info.get('Wc_dcs_m_used', None)
        hs_m = val_info.get('Wc_hs_m_used', None)
        hs_src = val_info.get('Wc_hs_source', None)
        Kgm_3_core = val_info.get('Wc_density_kg_m3_used', None)
        steel_grade_core = val_info.get('Wc_steel_grade_used', None)

        if steel_grade_core is not None:
            text += self.format_equation("Steel grade", f"{steel_grade_core}")
        if Kgm_3_core is not None:
            text += self.format_equation("Kgm_3", f"{float(Kgm_3_core):.0f} kg/m^3")
        if hs_m is not None:
            text += self.format_equation("hs", f"{float(hs_m):.6f} m")
        if hs_src is not None:
            text += self.format_equation("hs source", f"{hs_src}")
        if dcs_m is not None:
            text += self.format_equation("dcs", f"{float(dcs_m):.6f} m")
        if D_mean_m is not None:
            text += self.format_equation("D_mean", f"{float(D_mean_m):.6f} m")
        if A_c_m2 is not None:
            text += self.format_equation("A_c", f"{float(A_c_m2):.10f} m^2")
        if Wc_kg is not None:
            text += "\n"
            text += self.format_equation("Wc", f"{float(Wc_kg):.4f} kg")

        # Iron losses in tooth and core
        text += self.make_separator()
        text += "9. Iron Losses (Tooth and Core)\n"
        text += self.make_separator()
        text += "    Tooth iron loss: Pit = Wt × Pkg   [W]\n"
        text += "    Core  iron loss: Pic = Wc × Pkg   [W]\n"
        text += "    (Pkg is the lamination loss value Loss_W_kg from the selected steel grade)\n\n"

        Pkg = val_info.get('iron_loss_Pkg_W_per_kg_used', None)
        iron_grade = val_info.get('iron_loss_steel_grade_used', None)
        Pit_W = val_info.get('Pit_W', None)
        Pic_W = val_info.get('Pic_W', None)
        Wt_val = val_info.get('Wt_kg', None)
        Wc_val = val_info.get('Wc_kg', None)

        if iron_grade is not None:
            text += self.format_equation("Steel grade", f"{iron_grade}")
        if Pkg is not None:
            text += self.format_equation("Pkg", f"{float(Pkg):.3f} W/kg")
        if Wt_val is not None:
            text += self.format_equation("Wt", f"{float(Wt_val):.4f} kg")
        if Pit_W is not None:
            text += self.format_equation("Pit", f"{float(Pit_W):.4f} W")
        if Wc_val is not None:
            text += self.format_equation("Wc", f"{float(Wc_val):.4f} kg")
        if Pic_W is not None:
            text += self.format_equation("Pic", f"{float(Pic_W):.4f} W")
        
        self.append_to_tab(self.tab_losses, text)
    
    def display_summary(self):
        """Display design summary in summary panel"""
        # Summary panel was removed from the UI; keep this method for backward compatibility.
        if not hasattr(self, 'summary_text'):
            return
        summary = self.designer.get_summary()
        
        text = self.make_header("STATOR DESIGN SUMMARY")
        
        for section, params in summary.items():
            text += f"\n{self.make_separator()}"
            text += f"{section.upper()}\n"
            text += f"{self.make_separator()}"
            for key, value in params.items():
                text += f"  {key:.<40} {str(value):>35}\n"
        
        text += "\n" + "=" * 80 + "\n"
        
        # Validation messages
        if self.designer._validation_messages:
            text += "\nVALIDATION MESSAGES:\n"
            text += self.make_separator()
            for msg in self.designer._validation_messages:
                text += f"{msg}\n"
        
        self.summary_text.clear()
        self.summary_text.setPlainText(text)
    
    def export_results(self):
        """Export results to a PDF report."""
        has_live_model = bool(self.designer and getattr(self.designer, 'geometry', None) and getattr(self.designer, 'slots', None))
        if not (getattr(self, 'stator_design_done', False) and getattr(self, 'rotor_results', None)):
            QMessageBox.warning(self, "Rotor Required", "Please calculate BOTH stator and rotor designs (or load a project that contains them) before exporting.")
            return

        pdf_path, _ = QFileDialog.getSaveFileName(
            self,
            "Export PDF Report",
            "",
            "PDF files (*.pdf);;All files (*.*)"
        )

        if not pdf_path:
            return

        try:
            from types import SimpleNamespace

            if has_live_model:
                # Ensure method-dependent stator geometry values exist in validation_info
                self.designer.design_conductor_and_slots()
                self.designer.calculate_outer_diameter()

                geom = self.designer.geometry
                slots = self.designer.slots
                specs = self.designer.specs
                val_info = geom.validation_info if hasattr(geom, 'validation_info') and geom.validation_info else {}
                slot_method = getattr(specs, 'slot_method', 'semi_open_slot') or 'semi_open_slot'
            else:
                # Loaded-project mode: export from saved tab text + rotor_results.
                # We may not have a rebuilt stator designer; use the saved stator summary if present,
                # otherwise fall back to UI + stator_to_rotor_inputs.
                saved = getattr(self, '_loaded_stator_summary', None)
                if not isinstance(saved, dict):
                    saved = {}
                saved_specs = saved.get('specs', {}) if isinstance(saved.get('specs', {}), dict) else {}
                saved_geom = saved.get('geometry', {}) if isinstance(saved.get('geometry', {}), dict) else {}
                saved_slots = saved.get('slots', {}) if isinstance(saved.get('slots', {}), dict) else {}

                s2r = getattr(self, 'stator_to_rotor_inputs', None)
                if not isinstance(s2r, dict):
                    s2r = {}

                def _try_float(v):
                    try:
                        return float(v)
                    except Exception:
                        return None

                def _ui_float(attr_name: str):
                    w = getattr(self, attr_name, None)
                    if w is None:
                        return None
                    try:
                        return _try_float(w.text())
                    except Exception:
                        return None

                # geometry (meters)
                D_m = _try_float(saved_geom.get('D'))
                L_m = _try_float(saved_geom.get('L'))
                Dext_m = _try_float(saved_geom.get('D_ext'))
                if D_m is None:
                    Dmm = _try_float(s2r.get('D_mm'))
                    D_m = (Dmm / 1000.0) if (Dmm is not None) else None
                if L_m is None:
                    Lmm = _try_float(s2r.get('L_mm'))
                    L_m = (Lmm / 1000.0) if (Lmm is not None) else None

                # Prefer 2D cached params for diameters when available (covers older projects)
                s2d = getattr(self, '_last_stator_2d_params', None)
                try:
                    if s2d is not None and getattr(s2d, 'D_mm', None) is not None and D_m is None:
                        D_m = float(s2d.D_mm) / 1000.0
                    if s2d is not None and getattr(s2d, 'Dext_mm', None) is not None and Dext_m is None:
                        Dext_m = float(s2d.Dext_mm) / 1000.0
                    if s2d is not None and getattr(s2d, 'Ss', None) is not None and Ss_val is None:
                        Ss_val = int(s2d.Ss)
                except Exception:
                    pass

                # slots
                Ss_val = saved_slots.get('Ss', None)
                if Ss_val is None:
                    Ss_val = s2r.get('Ss', None)

                # specs
                power_kw = saved_specs.get('power_kw', None)
                if power_kw is None:
                    power_kw = s2r.get('power_kw', None)
                if power_kw is None:
                    power_kw = _ui_float('power_input')
                voltage_v = saved_specs.get('voltage_v', None)
                if voltage_v is None:
                    voltage_v = _ui_float('voltage_input')
                frequency_hz = saved_specs.get('frequency_hz', None)
                if frequency_hz is None:
                    frequency_hz = _ui_float('frequency_input')
                slot_method_saved = saved_specs.get('slot_method', None)

                geom = SimpleNamespace(D=D_m, D_ext=Dext_m, L=L_m, validation_info=saved_geom.get('validation_info', None))
                slots = SimpleNamespace(Ss=Ss_val)
                specs = SimpleNamespace(power_kw=power_kw, voltage_v=voltage_v, frequency_hz=frequency_hz, slot_method=slot_method_saved)

                val_info = saved_geom.get('validation_info', None)
                if not isinstance(val_info, dict):
                    val_info = {}

                # Slot geometry details: if validation_info wasn't saved, rebuild from stator_2d_params.
                try:
                    if (not val_info) and s2d is not None:
                        if getattr(s2d, 'slot_type', None) == 'square':
                            val_info['open_slot_bs_mm'] = self._jsonify(getattr(s2d, 'bs_mm', None))
                            val_info['open_slot_hs_mm'] = self._jsonify(getattr(s2d, 'hs_mm', None))
                        else:
                            val_info['bs0_mm'] = self._jsonify(getattr(s2d, 'bs0_mm', None))
                            val_info['bs1_mm'] = self._jsonify(getattr(s2d, 'bs1_mm', None))
                            val_info['bs2_mm'] = self._jsonify(getattr(s2d, 'bs2_mm', None))
                            val_info['slot_dim_hs0_mm'] = self._jsonify(getattr(s2d, 'hs0_mm', None))
                            val_info['slot_dim_hs1_mm'] = self._jsonify(getattr(s2d, 'hs1_mm', None))
                            val_info['hs2_mm'] = self._jsonify(getattr(s2d, 'hs2_mm', None))
                except Exception:
                    pass

                slot_method = (slot_method_saved or getattr(self, '_slot_method_ui', None) or 'semi_open_slot')

            # Prefer a human-friendly slot method label in the report.
            slot_method_display = ""
            try:
                if has_live_model and hasattr(self, 'slot_method_edit') and self.slot_method_edit is not None:
                    slot_method_display = str(self.slot_method_edit.text() or '').strip()
            except Exception:
                slot_method_display = ""
            if not slot_method_display:
                try:
                    slot_method_display = str(getattr(self, '_loaded_slot_method_display', '') or '').strip()
                except Exception:
                    slot_method_display = ""
            if not slot_method_display:
                # Map internal keys to the same wording used in the UI.
                k = str(slot_method or '').strip().lower()
                if k in ('semi_open_slot', 'semi-open', 'semi_open'):
                    slot_method_display = 'Trapezoidal semi-open slot'
                elif k in ('open_slot', 'open'):
                    slot_method_display = 'Open slot'
                else:
                    slot_method_display = str(slot_method)

            rotor = dict(getattr(self, 'rotor_results', {}) or {})

            # ----------------------------
            # Build PDF content
            # ----------------------------
            import re
            from collections import OrderedDict
            from datetime import datetime

            def _extract_equation_results(text: str) -> "OrderedDict[str, str]":
                """Return last occurrence of each equation 'left = right' in reading order."""
                results: "OrderedDict[str, str]" = OrderedDict()
                eq_re = re.compile(r"^\s{4}(.+?)\s+=\s+(.+?)\s*$")
                for raw in (text or "").splitlines():
                    m = eq_re.match(raw)
                    if not m:
                        continue
                    left = (m.group(1) or "").strip()
                    right = (m.group(2) or "").strip()
                    if not left or not right:
                        continue

                    # Remove internal/config-like entries from the report.
                    l_low = left.lower()
                    r_low = right.lower()
                    if "override" in l_low or "override" in r_low:
                        continue
                    if "no user" in l_low or "no user" in r_low:
                        continue
                    if l_low.startswith("formula") or l_low.startswith("let") or l_low.startswith("note"):
                        continue

                    if left in results:
                        results.pop(left, None)
                    results[left] = right
                return results

            tab_defs = [
                (getattr(self, 'tab_empirical', None), "Empirical"),
                (getattr(self, 'tab_electrical', None), "Electrical"),
                (getattr(self, 'tab_dimensions', None), "Main Dimensions"),
                (getattr(self, 'tab_winding', None), "Winding"),
                (getattr(self, 'tab_slots', None), "Slots"),
                (getattr(self, 'tab_outer', None), "Outer Diameter"),
                (getattr(self, 'tab_losses', None), "Losses"),
                (getattr(self, 'tab_rotor', None), "Rotor"),
                (getattr(self, 'tab_efficiency', None), "Efficiency Analysis"),
            ]

            tab_explainer: dict[str, str] = {
                "Empirical": "Sizing heuristics and empirical coefficients used as a starting point for the design.",
                "Electrical": "Electrical sizing based on the provided ratings (power, voltage, frequency) and selected constraints.",
                "Main Dimensions": "Main geometric dimensions derived from the sizing procedure.",
                "Winding": "Winding layout and winding factor-related results derived from slot/pole selection.",
                "Slots": "Stator slot dimensions and slot-related constraints based on the selected slot type.",
                "Outer Diameter": "Outer diameter calculation and checks derived from the magnetic/electrical constraints.",
                "Losses": "Loss estimates computed from material and current-related parameters.",
                "Rotor": "Rotor sizing and slot parameters computed from the stator design and rotor configuration.",
                "Efficiency Analysis": "Performance indicators derived from losses and output power assumptions.",
            }

            per_tab_results: dict[str, "OrderedDict[str, str]"] = {}
            for tab_obj, tab_title in tab_defs:
                if tab_obj is None:
                    continue
                try:
                    text = tab_obj.toPlainText() or ""
                except Exception:
                    text = ""
                per_tab_results[tab_title] = _extract_equation_results(text)

            def _fmt(v, unit: str = "") -> str:
                if v is None:
                    return "-"
                if isinstance(v, float):
                    s = f"{v:.6g}"
                else:
                    s = str(v)
                return f"{s} {unit}".rstrip()

            # Generate PDF using Qt (no extra dependency)
            # NOTE: In PyQt6, QPdfWriter uses setPageSize(QPageSize), not setPageSizeMM.
            from PyQt6.QtGui import QPdfWriter, QPainter, QFont, QFontMetrics, QPageSize

            writer = QPdfWriter(pdf_path)
            writer.setPageSize(QPageSize(QPageSize.PageSizeId.A4))
            writer.setResolution(144)
            painter = QPainter(writer)
            painter.setRenderHint(QPainter.RenderHint.TextAntialiasing, True)

            paint_device = painter.device()

            margin_px = 54
            page_w = writer.width()
            page_h = writer.height()
            content_w = page_w - 2 * margin_px
            content_h = page_h - 2 * margin_px

            font_title = QFont("Arial", 18, QFont.Weight.Bold)
            font_h1 = QFont("Arial", 13, QFont.Weight.Bold)
            font_h2 = QFont("Arial", 11, QFont.Weight.Bold)
            font_body = QFont("Arial", 10)
            font_mono = QFont("Courier New", 9)
            font_small = QFont("Arial", 8)

            y = margin_px
            page_no = 1

            def _metrics(f: QFont) -> QFontMetrics:
                # IMPORTANT: Use metrics from the PDF paint device.
                # Using screen metrics will underestimate/overestimate text widths and cause
                # wrapping errors (visible as clipped text in the exported PDF).
                return QFontMetrics(f, paint_device)

            def _line_h(f: QFont) -> int:
                return _metrics(f).height() + 3

            def _ensure_space(min_height: int):
                nonlocal y, page_no
                if y + min_height <= (margin_px + content_h):
                    return
                # Finish current page.
                _draw_footer()
                writer.newPage()
                page_no += 1
                y = margin_px

            def _draw_footer():
                nonlocal page_no
                painter.setFont(font_small)
                fm = _metrics(font_small)
                footer_y = margin_px + content_h + fm.ascent() + 6
                painter.drawText(margin_px, footer_y, "Induction Motor Design Report")
                ptxt = f"Page {page_no}"
                w = fm.horizontalAdvance(ptxt)
                painter.drawText(margin_px + content_w - w, footer_y, ptxt)

            def _wrap(text: str, f: QFont, max_w: int) -> list[str]:
                fm = _metrics(f)
                if not text:
                    return [""]

                def _hard_break_word(word: str) -> list[str]:
                    # Split a single long token into chunks that fit.
                    if fm.horizontalAdvance(word) <= max_w:
                        return [word]
                    out_parts: list[str] = []
                    cur = word
                    while cur:
                        lo, hi = 1, len(cur)
                        best = 1
                        while lo <= hi:
                            mid = (lo + hi) // 2
                            if fm.horizontalAdvance(cur[:mid]) <= max_w:
                                best = mid
                                lo = mid + 1
                            else:
                                hi = mid - 1
                        out_parts.append(cur[:best])
                        cur = cur[best:]
                    return out_parts

                out: list[str] = []
                for para in text.split("\n"):
                    if not para.strip():
                        out.append("")
                        continue
                    words = para.split()
                    cur = ""
                    for w in words:
                        # If a single word is longer than the line, hard-break it.
                        if fm.horizontalAdvance(w) > max_w:
                            if cur:
                                out.append(cur)
                                cur = ""
                            out.extend(_hard_break_word(w))
                            continue
                        test = w if not cur else (cur + " " + w)
                        if fm.horizontalAdvance(test) <= max_w:
                            cur = test
                        else:
                            if cur:
                                out.append(cur)
                            cur = w
                    if cur:
                        out.append(cur)
                return out or [""]

            def _wrap_preformatted_line(line: str, f: QFont, max_w: int) -> list[str]:
                """Wrap a single line by characters (preserves leading spaces)."""
                fm = _metrics(f)
                s = "" if line is None else str(line)
                if not s:
                    return [""]
                if fm.horizontalAdvance(s) <= max_w:
                    return [s]
                out_parts: list[str] = []
                cur = s
                while cur:
                    lo, hi = 1, len(cur)
                    best = 1
                    while lo <= hi:
                        mid = (lo + hi) // 2
                        if fm.horizontalAdvance(cur[:mid]) <= max_w:
                            best = mid
                            lo = mid + 1
                        else:
                            hi = mid - 1
                    out_parts.append(cur[:best])
                    cur = cur[best:]
                return out_parts

            def _paragraph(text: str, f: QFont = font_body, top_pad: int = 0, bottom_pad: int = 0):
                nonlocal y
                painter.setFont(f)
                lh = _line_h(f)
                lines = _wrap(text, f, content_w)
                _ensure_space(top_pad + lh * len(lines) + bottom_pad)
                y += top_pad
                for ln in lines:
                    painter.drawText(margin_px, y + _metrics(f).ascent(), ln)
                    y += lh
                y += bottom_pad

            def _heading(text: str, f: QFont, top_pad: int, bottom_pad: int):
                nonlocal y
                painter.setFont(f)
                lh = _line_h(f)
                _ensure_space(top_pad + lh + bottom_pad)
                y += top_pad
                painter.drawText(margin_px, y + _metrics(f).ascent(), text)
                y += lh + bottom_pad

            def _hr(pad: int = 8):
                nonlocal y
                _ensure_space(pad + 6)
                y += pad
                painter.drawLine(margin_px, y, margin_px + content_w, y)
                y += 6

            def _kv_table(rows: list[tuple[str, str]], key_w_ratio: float = 0.55, row_pad: int = 1):
                nonlocal y
                if not rows:
                    return
                painter.setFont(font_body)
                fm = _metrics(font_body)
                lh = fm.height() + 5
                key_w = int(content_w * key_w_ratio)
                val_w = content_w - key_w
                for k, v in rows:
                    k = (k or "").strip()
                    v = (v or "").strip()
                    k_lines = _wrap(k, font_body, key_w)
                    v_lines = _wrap(v, font_body, val_w)
                    n = max(len(k_lines), len(v_lines))
                    _ensure_space(lh * n + row_pad)
                    for i in range(n):
                        kk = k_lines[i] if i < len(k_lines) else ""
                        vv = v_lines[i] if i < len(v_lines) else ""
                        painter.drawText(margin_px, y + fm.ascent(), kk)
                        painter.drawText(margin_px + key_w, y + fm.ascent(), vv)
                        y += lh
                    y += row_pad

            # ----------------------------
            # Report content
            # ----------------------------
            now = datetime.now().strftime("%Y-%m-%d %H:%M")

            _heading("Induction Motor Design Report", font_title, top_pad=0, bottom_pad=2)
            _paragraph(f"Generated on: {now}", font_body, top_pad=0, bottom_pad=8)
            _paragraph(
                "This document summarizes the stator and rotor design results produced by the application. "
                "Values shown are the final computed outputs displayed in the GUI.",
                font_body,
                top_pad=0,
                bottom_pad=6,
            )
            _hr(pad=2)

            # Executive summary (curated, avoids dumping app/config messages)
            summary_rows: list[tuple[str, str]] = []
            summary_rows.append(("Output power", _fmt(getattr(specs, 'power_kw', None), "kW")))
            summary_rows.append(("Line voltage", _fmt(getattr(specs, 'voltage_v', None), "V")))
            summary_rows.append(("Frequency", _fmt(getattr(specs, 'frequency_hz', None), "Hz")))
            summary_rows.append(("Stator slots (Ss)", _fmt(getattr(slots, 'Ss', None), "")))
            summary_rows.append(("Stator bore (D)", _fmt(getattr(geom, 'D', None) * 1000.0 if getattr(geom, 'D', None) is not None else None, "mm")))
            summary_rows.append(("Stator outer diameter (D_ext)", _fmt(getattr(geom, 'D_ext', None) * 1000.0 if getattr(geom, 'D_ext', None) is not None else None, "mm")))
            summary_rows.append(("Machine length (L)", _fmt(getattr(geom, 'L', None) * 1000.0 if getattr(geom, 'L', None) is not None else None, "mm")))
            summary_rows.append(("Rotor bars (Nr)", _fmt(rotor.get('Nr'), "")))
            summary_rows.append(("Rotor outer diameter (Dr)", _fmt(rotor.get('Dr_mm'), "mm")))
            summary_rows.append(("Airgap (Lg)", _fmt(rotor.get('Lg_mm'), "mm")))

            _heading("Executive Summary", font_h1, top_pad=10, bottom_pad=6)
            _kv_table([r for r in summary_rows if r[1] != "-"], key_w_ratio=0.58)

            # Slot method summary (compact)
            slot_rows: list[tuple[str, str]] = [("Slot method", str(slot_method_display))]
            if slot_method == 'open_slot':
                bs_mm = val_info.get('open_slot_bs_mm', None)
                hs_mm = val_info.get('open_slot_hs_mm', None)
                if bs_mm is not None:
                    slot_rows.append(("Open slot width (bs)", _fmt(bs_mm, "mm")))
                if hs_mm is not None:
                    slot_rows.append(("Open slot height (hs)", _fmt(hs_mm, "mm")))
            else:
                bs0_mm = val_info.get('bs0_mm', None)
                bs1_mm = val_info.get('bs1_mm', None)
                bs2_mm = val_info.get('bs2_mm', None)
                hs0_mm = val_info.get('slot_dim_hs0_mm', val_info.get('hs0_mm', None))
                hs1_mm = val_info.get('slot_dim_hs1_mm', val_info.get('hs1_mm', None))
                hs2_mm = val_info.get('hs2_mm', None)
                if bs0_mm is not None:
                    slot_rows.append(("Slot opening (bs0)", _fmt(bs0_mm, "mm")))
                if bs1_mm is not None:
                    slot_rows.append(("Slot width (bs1)", _fmt(bs1_mm, "mm")))
                if bs2_mm is not None:
                    slot_rows.append(("Slot width (bs2)", _fmt(bs2_mm, "mm")))
                if hs0_mm is not None:
                    slot_rows.append(("Slot height (hs0)", _fmt(hs0_mm, "mm")))
                if hs1_mm is not None:
                    slot_rows.append(("Slot height (hs1)", _fmt(hs1_mm, "mm")))
                if hs2_mm is not None:
                    slot_rows.append(("Slot height (hs2)", _fmt(hs2_mm, "mm")))

            _heading("Slot Geometry (Stator)", font_h2, top_pad=10, bottom_pad=4)
            _kv_table(slot_rows, key_w_ratio=0.58)

            # Rotor key results (curated from rotor solver dict)
            rotor_key_rows: list[tuple[str, str]] = []
            rotor_key_rows.extend([
                ("Rotor bars (Nr)", _fmt(rotor.get('Nr'), "")),
                ("Airgap (Lg)", _fmt(rotor.get('Lg_mm'), "mm")),
                ("Rotor outer diameter (Dr)", _fmt(rotor.get('Dr_mm'), "mm")),
                ("Rotor slot pitch (sp2)", _fmt(rotor.get('sp2_mm'), "mm")),
                ("Rotor tooth width (Wrt)", _fmt(rotor.get('Wrt_mm'), "mm")),
                ("Rotor slot radius r1", _fmt(rotor.get('r1_mm'), "mm")),
                ("Rotor slot radius r2", _fmt(rotor.get('r2_mm'), "mm")),
                ("Rotor slot depth (d_rb)", _fmt(rotor.get('d_rb_mm'), "mm")),
                ("Rotor opening height (h_r0)", _fmt(rotor.get('h_r0_mm'), "mm")),
                ("Rotor opening width (b_r0)", _fmt(rotor.get('b_r0_mm'), "mm")),
                ("Bar current (Ib)", _fmt(rotor.get('Ib_A'), "A")),
                ("Bar section (Ab_geo)", _fmt(rotor.get('Ab_geom_mm2'), "mm²")),
                ("Rotor yoke width (Wry)", _fmt(rotor.get('Wry_mm'), "mm")),
                ("Bar length (Lbar)", _fmt(rotor.get('Lbar_mm'), "mm")),
                ("Bar resistance (Rbar)", _fmt(rotor.get('Rbar_ohm'), "Ω")),
                ("Bar losses (P_bar)", _fmt(rotor.get('P_bar_W'), "W")),
                ("Ring dimensions (h_er, b_er)", f"{_fmt(rotor.get('h_er_mm'), 'mm')} ; {_fmt(rotor.get('b_er_mm'), 'mm')}"),
                ("Ring resistance (Rer)", _fmt(rotor.get('Rer_ohm'), "Ω")),
                ("Ring losses (P_er)", _fmt(rotor.get('P_er_W'), "W")),
                ("Rotor equivalent resistance", _fmt(rotor.get('R_rotor_eq_ohm'), "Ω")),
                ("Total rotor losses", _fmt(rotor.get('P_rotor_total_W'), "W")),
            ])
            rotor_key_rows = [r for r in rotor_key_rows if r[1] != "-"]
            if rotor_key_rows:
                _heading("Rotor Key Results", font_h2, top_pad=12, bottom_pad=4)
                _paragraph("Key rotor outputs computed by the rotor design stage.", font_body, top_pad=0, bottom_pad=6)
                _kv_table(rotor_key_rows, key_w_ratio=0.58)

            _hr(pad=8)
            _paragraph(
                "Detailed sections below list the final values shown in each computation tab. "
                "They are grouped by topic to make the report readable.",
                font_body,
                top_pad=0,
                bottom_pad=10,
            )

            # One section per tab
            for tab_obj, tab_title in tab_defs:
                rows = per_tab_results.get(tab_title, None)
                raw_text = ""
                try:
                    raw_text = (tab_obj.toPlainText() or "") if tab_obj is not None else ""
                except Exception:
                    raw_text = ""

                # Fallback: if a tab contains text but equation parsing yields nothing,
                # still include the section in the PDF (important for Rotor details).
                if (not rows) and raw_text.strip():
                    _ensure_space(120)
                    _heading(tab_title, font_h1, top_pad=12, bottom_pad=4)
                    expl = tab_explainer.get(tab_title, "Computed results produced by this tab.")
                    _paragraph(expl, font_body, top_pad=0, bottom_pad=8)
                    painter.setFont(font_mono)
                    fm = _metrics(font_mono)
                    lh = fm.height() + 4
                    for ln in raw_text.splitlines():
                        parts = _wrap_preformatted_line(ln, font_mono, content_w)
                        _ensure_space(lh * max(1, len(parts)))
                        for ptxt in parts:
                            painter.drawText(margin_px, y + fm.ascent(), ptxt)
                            y += lh
                    _hr(pad=6)
                    continue

                if not rows:
                    continue
                _ensure_space(120)
                _heading(tab_title, font_h1, top_pad=12, bottom_pad=4)
                expl = tab_explainer.get(tab_title, "Computed results produced by this tab.")
                _paragraph(expl, font_body, top_pad=0, bottom_pad=8)

                # Render equations as a simple 2-column table.
                painter.setFont(font_mono)
                fm = _metrics(font_mono)
                lh = fm.height() + 4
                key_w = int(content_w * 0.60)
                val_w = content_w - key_w
                for left, right in rows.items():
                    if not left or not right:
                        continue
                    # Extra safety filter
                    ll = left.lower()
                    rr = right.lower()
                    if "override" in ll or "override" in rr or "no user" in ll or "no user" in rr:
                        continue
                    left_lines = _wrap(left, font_mono, key_w)
                    right_lines = _wrap(right, font_mono, val_w)
                    n = max(len(left_lines), len(right_lines))
                    _ensure_space(lh * n + 2)
                    for i in range(n):
                        ltxt = left_lines[i] if i < len(left_lines) else ""
                        rtxt = right_lines[i] if i < len(right_lines) else ""
                        painter.drawText(margin_px, y + fm.ascent(), ltxt)
                        painter.drawText(margin_px + key_w, y + fm.ascent(), rtxt)
                        y += lh
                    y += 2

                _hr(pad=6)

                # If we started a new page, ensure footer gets printed for previous pages.
                # (Footer is drawn at the end of each page via _draw_footer on page breaks)

            # Draw final footer
            _draw_footer()
            painter.end()

            QMessageBox.information(
                self,
                "Success",
                f"Export created:\n{pdf_path}"
            )
        except Exception as e:
            QMessageBox.critical(self, "Export Error", f"Failed to export results:\n{str(e)}")


def main():
    """Main function to run the GUI application"""
    app = QApplication(sys.argv)

    # Modern look: Fusion style + tuned palette + clean stylesheet.
    # (Purely visual; does not change any logic.)
    try:
        app.setStyle("Fusion")
    except Exception:
        pass

    try:
        app.setFont(QFont("Segoe UI", 10))
    except Exception:
        pass

    try:
        pal = QPalette()
        # Dark theme base
        pal.setColor(QPalette.ColorRole.Window, QColor(18, 21, 26))
        pal.setColor(QPalette.ColorRole.Base, QColor(14, 17, 22))
        pal.setColor(QPalette.ColorRole.AlternateBase, QColor(24, 28, 35))
        pal.setColor(QPalette.ColorRole.Text, QColor(231, 234, 240))
        pal.setColor(QPalette.ColorRole.WindowText, QColor(231, 234, 240))
        pal.setColor(QPalette.ColorRole.Button, QColor(24, 28, 35))
        pal.setColor(QPalette.ColorRole.ButtonText, QColor(231, 234, 240))
        pal.setColor(QPalette.ColorRole.ToolTipBase, QColor(231, 234, 240))
        pal.setColor(QPalette.ColorRole.ToolTipText, QColor(18, 21, 26))
        pal.setColor(QPalette.ColorRole.PlaceholderText, QColor(145, 152, 165))
        # Accent / selection
        pal.setColor(QPalette.ColorRole.Highlight, QColor(0, 160, 130))
        pal.setColor(QPalette.ColorRole.HighlightedText, QColor(18, 21, 26))
        app.setPalette(pal)
    except Exception:
        pass

    # QSS: dark theme with teal accent (fits motor/engineering vibe).
    base_dir = os.path.dirname(os.path.abspath(__file__))
    chevron_down = os.path.join(base_dir, "assets", "chevron-down.svg").replace('\\', '/')
    chevron_up = os.path.join(base_dir, "assets", "chevron-up.svg").replace('\\', '/')

    qss = """
        QMainWindow { background: #12151A; }

        /* Default surface */
        QWidget {
            background: #12151A;
            color: #E7EAF0;
        }

        QLabel { color: #E7EAF0; }

        /* Welcome */
        QWidget#welcomePage { background: #0F1216; }
        QLabel#welcomeTitle { color: #EAF2FF; }
        QLabel#welcomeSubtitle { color: #B7C0D1; }
        QLabel#welcomeHint { color: #93A0B8; }
        QWidget#welcomeCard {
            background: #151A22;
            border: 1px solid #2A3342;
            border-radius: 14px;
        }
        QWidget#welcomeCard:hover { border: 1px solid #2F4452; }
        QLabel#welcomeCardTitle { color: #EAF2FF; }
        QLabel#welcomeCardDesc { color: #B7C0D1; }

        /* Checkboxes */
        QCheckBox { spacing: 8px; }
        QCheckBox::indicator {
            width: 16px;
            height: 16px;
            border-radius: 4px;
            border: 1px solid #2A3342;
            background: #0E1116;
        }
        QCheckBox::indicator:hover {
            border: 1px solid #00A082;
        }
        QCheckBox::indicator:checked {
            border: 1px solid #00A082;
            background: #00A082;
        }
        QCheckBox::indicator:disabled {
            border: 1px solid #242B37;
            background: #151A22;
        }

        /* Group boxes */
        QGroupBox {
            background: #151A22;
            border: 1px solid #2A3342;
            border-radius: 10px;
            margin-top: 12px;
            padding: 10px;
        }
        QGroupBox::title {
            subcontrol-origin: margin;
            left: 10px;
            padding: 0 6px;
            color: #E7EAF0;
            font-weight: 600;
        }

        /* Inputs */
        QLineEdit, QComboBox, QSpinBox {
            background: #0E1116;
            border: 1px solid #2A3342;
            border-radius: 8px;
            padding: 6px 8px;
            min-height: 30px;
            selection-background-color: #00A082;
        }
        QLineEdit:focus, QComboBox:focus, QSpinBox:focus {
            border: 1px solid #00A082;
        }

        /* Fix Windows/Fusion corner clipping for combo boxes: style the drop-down subcontrol */
        QComboBox {
            padding-right: 30px; /* reserve space for arrow */
        }
        QComboBox::drop-down {
            subcontrol-origin: padding;
            subcontrol-position: top right;
            width: 28px;
            border-left: 1px solid #2A3342;
            border-top-right-radius: 8px;
            border-bottom-right-radius: 8px;
            background: #0E1116;
        }
        QComboBox::down-arrow {
            image: url(__CHEVRON_DOWN__);
            width: 12px;
            height: 12px;
        }
        QComboBox::down-arrow:on { top: 1px; }

        /* Spinbox arrows (e.g., Cooling ducts) */
        QSpinBox {
            padding-right: 34px; /* room for up/down buttons */
        }
        QSpinBox::up-button {
            subcontrol-origin: padding;
            subcontrol-position: top right;
            width: 28px;
            border-left: 1px solid #2A3342;
            border-top-right-radius: 8px;
            background: #0E1116;
        }
        QSpinBox::down-button {
            subcontrol-origin: padding;
            subcontrol-position: bottom right;
            width: 28px;
            border-left: 1px solid #2A3342;
            border-bottom-right-radius: 8px;
            background: #0E1116;
        }
        QSpinBox::up-arrow {
            image: url(__CHEVRON_UP__);
            width: 12px;
            height: 12px;
        }
        QSpinBox::down-arrow {
            image: url(__CHEVRON_DOWN__);
            width: 12px;
            height: 12px;
        }
        QSpinBox::up-button:hover, QSpinBox::down-button:hover {
            background: #121724;
        }

        QTextEdit {
            background: #0E1116;
            border: 1px solid #2A3342;
            border-radius: 8px;
            padding: 8px;
            min-height: 28px;
            selection-background-color: #00A082;
        }

        QGraphicsView {
            background: #0B0E13;
            border: 1px solid #2A3342;
            border-radius: 10px;
        }

        /* White canvas only for 2D drawing views (not the whole page theme) */
        QGraphicsView[role="canvas"] {
            background: #FFFFFF;
        }

        /* Buttons */
        QPushButton {
            background: #1B2230;
            border: 1px solid #2A3342;
            border-radius: 10px;
            /* Default button: compact (used for Go Back / Fit views / small actions) */
            padding: 6px 12px;
            min-height: 34px;
            font-weight: 600;
        }

        /* Primary action buttons: bigger and clearer */
        QPushButton[role="primary"] {
            background: #00A082;
            border: 1px solid #00A082;
            color: #0F1216;
            padding: 10px 12px;
            min-height: 46px;
        }
        QPushButton[role="primary"]:hover {
            background: #00B092;
            border: 1px solid #00B092;
        }

        QToolButton {
            background: #1B2230;
            border: 1px solid #2A3342;
            border-radius: 10px;
            padding: 6px 8px;
            min-height: 34px;
        }
        QToolButton:hover {
            background: #212B3B;
        }
        QPushButton:hover {
            background: #212B3B;
        }
        QPushButton:pressed {
            background: #1A2230;
        }
        QPushButton:disabled {
            background: #151A22;
            color: #717C90;
            border-color: #242B37;
        }

        /* Tabs */
        QTabWidget::pane {
            border: 1px solid #2A3342;
            border-radius: 10px;
            background: #151A22;
            top: -1px;
        }
        QTabBar::tab {
            background: #12151A;
            border: 1px solid #2A3342;
            border-bottom: none;
            border-top-left-radius: 10px;
            border-top-right-radius: 10px;
            padding: 8px 12px;
            min-height: 32px;
            margin-right: 4px;
            color: #B7C0D1;
        }
        QTabBar::tab:selected {
            background: #151A22;
            color: #E7EAF0;
        }
        QTabBar::tab:disabled {
            background: #12151A;
            color: #717C90;
        }
        QTabBar::tab:!selected:hover {
            background: #1B2230;
        }

        /* Menus */
        QMenuBar {
            background: #12151A;
            border-bottom: 1px solid #2A3342;
        }
        QMenuBar::item {
            padding: 6px 10px;
            background: transparent;
        }
        QMenuBar::item:selected {
            background: #1B2230;
            border-radius: 8px;
        }
        QMenu {
            background: #151A22;
            border: 1px solid #2A3342;
            border-radius: 10px;
            padding: 6px;
        }
        QMenu::item {
            padding: 8px 12px;
            border-radius: 8px;
        }
        QMenu::item:selected {
            background: #1B2230;
        }

        QStatusBar {
            background: #12151A;
            border-top: 1px solid #2A3342;
        }

        QScrollArea {
            background: #12151A;
            border: none;
        }

        /* Scroll bars */
        QScrollBar:vertical {
            background: transparent;
            width: 12px;
            margin: 6px 2px 6px 2px;
        }
        QScrollBar::handle:vertical {
            background: #2A3342;
            min-height: 24px;
            border-radius: 6px;
        }
        QScrollBar::handle:vertical:hover {
            background: #344155;
        }
        QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {
            height: 0px;
        }
        QScrollBar::add-page:vertical, QScrollBar::sub-page:vertical {
            background: transparent;
        }
        """

    qss = qss.replace("__CHEVRON_DOWN__", chevron_down)
    qss = qss.replace("__CHEVRON_UP__", chevron_up)
    app.setStyleSheet(qss)

    window = StatorDesignGUI()
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
