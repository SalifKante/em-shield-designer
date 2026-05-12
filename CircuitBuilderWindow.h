#pragma once

// ============================================================
//  CircuitBuilderWindow.h  —  EMShieldDesigner Phase B
//
//  Architecture overview (matches mainwindow.cpp / Phase A):
//
//  ┌─────────────────────────┐    ┌──────────────────────────┐
//  │  Left panel (vertical)  │    │  AssemblyCanvas (DnD)    │
//  │   • Brand strip         │    │  Source→Ap→Cav→Obs       │
//  │   • Property editor     │    └──────────────────────────┘
//  │   • Elements palette    │    ┌──────────────────────────┐
//  │   • Actions             │    │  QCustomPlot             │
//  │   • Compute / Export    │    │  N Obs.Points → N curves │
//  └─────────────────────────┘    └──────────────────────────┘
//
//  Task 1.2 changes (move toolbar buttons to left panel):
//    [T1.2-A] Removed the horizontal QToolBar that hosted the
//             palette and action buttons.
//    [T1.2-B] Relocated all element-type buttons and action
//             buttons (Arrange / Delete / Clear / Compute /
//             Export CSV) into the left panel below the
//             existing PROPERTIES section.
//    [T1.2-C] Redesigned PaletteButton as a horizontal full-
//             width tile with a 16×16 inline icon on the left
//             and a centred-left text label.  Drag behaviour
//             is preserved exactly.
//    [T1.2-D] All button styling moved to Styles.h and applied
//             via QSS with normal / hover / pressed / disabled
//             states.  Compute is rendered as a primary action
//             (filled accent, bold, taller).
//    [T1.2-E] Switched all C++20-deprecated `[=]` lambdas to
//             explicit `[this]` / `[this, x]` captures to
//             eliminate the 26 implicit-`this` warnings the
//             compiler reported in CircuitBuilderWindow.h.
//    [T1.2-F] Brand strip ("EMShieldBuilder") relocated from
//             the deleted toolbar to the top of the left panel.
//
//  Key design decisions (preserved from before Task 1.2):
//  [D1] Obs.Point (Load) is a SHUNT tap: connects nFrom→0.
//       nodeIdx is NOT incremented after a Load.
//       The circuit continues from the same node.
//       ∴  [Source]→[Ap]→[Cav]→[Obs]→[Ap2]→[Cav2]→[Obs2]
//       produces two SE curves at the two observation nodes.
//  [D2] SE = -20·log₁₀|2·U_obs/V₀|   (Eq. 3.8)
//  [D3] MNASolver topology built ONCE; solve(f) called per
//       frequency (matches validated MNASolver contract).
//  [D4] Source impedance = Z_0 exact (376.730 Ω), not 120π.
//  [D5] QCustomPlot replaces hand-painted SEPlotWidget.
//       Interactive overlay is identical to Phase A.
// ============================================================

#include "qcustomplot.h"   // must precede other Qt headers

#include <QMainWindow>
#include <QWidget>
#include <QFrame>
#include <QGraphicsView>
#include <QGraphicsScene>
#include <QGraphicsItem>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QFormLayout>
#include <QScrollArea>
#include <QLabel>
#include <QPushButton>
#include <QDoubleSpinBox>
#include <QSpinBox>
#include <QLineEdit>
#include <QSplitter>
#include <QStatusBar>
#include <QPainter>
#include <QPainterPath>
#include <QPen>
#include <QBrush>
#include <QColor>
#include <QFont>
#include <QDrag>
#include <QMimeData>
#include <QMouseEvent>
#include <QWheelEvent>
#include <QKeyEvent>
#include <QTimer>
#include <QDropEvent>
#include <QDragEnterEvent>
#include <QDragMoveEvent>
#include <QPropertyAnimation>
#include <QGraphicsDropShadowEffect>
#include <QVector>
#include <QString>
#include <QDebug>
#include <QFileDialog>
#include <QMessageBox>

#include <complex>
#include <cmath>
#include <limits>
#include <memory>
#include <algorithm>
#include <fstream>
#include <iomanip>
#include <Eigen/Dense>

// Physics engine
#include "include/core/PhysicsConstants.h"
#include "include/core/MNASolver.h"
#include "include/core/AP_SlotAperture.h"
#include "include/core/AP_SlotWithCover.h"
#include "include/core/TL_EmptyCavity.h"
#include "include/core/TL_DielectricCavity.h"
#include "include/core/SRC_VoltageSource.h"
#include "include/core/LOAD_Impedance.h"
#include "include/core/CircuitGenerator.h"

using namespace EMCore;

// ============================================================
//  SPICE-inspired light theme tokens
// ============================================================
namespace CBStyle {
inline const QColor BG        { 246, 251, 249 };
inline const QColor SURFACE   { 235, 244, 240 };
inline const QColor SURFACE2  { 220, 235, 229 };
inline const QColor BORDER    { 180, 205, 195 };
inline const QColor BORDER_LT { 210, 228, 222 };
inline const QColor ACCENT    {  14, 100, 200 };
inline const QColor GREEN     {  22, 130,  64 };
inline const QColor ORANGE    { 180,  90,   0 };
inline const QColor RED       { 195,  30,  30 };
inline const QColor TEXT      {  24,  36,  32 };
inline const QColor TEXT_MUTED{  90, 115, 105 };
inline const QColor TEXT_DIM  { 150, 175, 165 };
}
#define CBSTYLE_DECLARED 1

#include "Styles.h"   // [T1.2-D] central QSS + button factory
#include "MessageDialog.h"   // [T1.6] custom error/success dialog

// NOTE: StackLayerPanel.h is included AFTER class CanvasElement is declared,
// further down in this file. See [T1.4-INCLUDE] marker below.

// ============================================================
//  ElementType
// ============================================================
enum class ElementType {
    Source,              // EM plane-wave excitation (series, left-most)
    Aperture,            // Slot aperture wall (SHUNT → ground, Fig.3.7 Branch II)
    ApertureWithCover,   // Aperture + quarter-λ lid (SHUNT → ground, Eq. 3.22)
    EmptyCavity,         // Air-filled waveguide     (series, TL)
    DielectricCavity,    // Dielectric-loaded WG     (series, TL)
    Load                 // Obs.Point shunt tap      (shunt → ground) [D1]
};

// ============================================================
//  ElementParams — one bundle per canvas element
// ============================================================
struct ElementParams {
    ElementType type  { ElementType::EmptyCavity };
    QString     label { "Element" };

    // ── Source ────────────────────────────────────────────
    double E0         { 1.0   };  // V₀ reference [V/m]
    double freqStart  { 1.0   };  // [GHz]
    double freqEnd    { 32.0  };  // [GHz]
    int    freqPoints { 300   };

    // ── Global cross-section (set on Source, used by all elements)
    double a          { 0.050 };  // waveguide width  [m]
    double b_h        { 0.030 };  // waveguide height [m]
    double t_wall     { 0.0015 }; // wall thickness [m] — matches Phase A default 1.5 mm

    // ── Aperture ─────────────────────────────────────────
    double l_slot     { 0.040 };  // [m]
    double w_slot     { 0.002 };  // [m]

    // ── ApertureWithCover ─────────────────────────────────
    double tau_cover  { 0.0005 }; // cover gap [m]

    // ── Cavity ───────────────────────────────────────────
    double L_cavity   { 0.100 };  // depth [m]

    // ── DielectricCavity extra fields ─────────────────────
    double eps_r        { 2.0   };
    double h_dielectric { 0.010 };  // dielectric layer [m] ≤ b

    // ── Load (Obs.Point) — shunt to ground ───────────────
    // [FIX-D1] Default = 1e9 Ω (near-open-circuit voltage probe).
    // Z_L = 377 Ω (old default) acts as matched load killing cavity resonances.
    double ZL_real    { 1.0e9 };
    double ZL_imag    {   0.0 };
};

// ============================================================
//  Schematic-style element painters (Task 1.3)
//
//  Design principles:
//    * Two-tone strokes only — no gradients, no fills with
//      shading, no drop shadows, no isometric pseudo-3D.
//    * Each silhouette is distinct so types are recognisable
//      from shape alone, even without colour.
//    * Selection state is signalled by the caller painting the
//      bounding chrome around the icon; the icon's own stroke
//      width changes from 1 px (idle) to 1.6 px (selected) so
//      the silhouette looks bolder when picked.
//    * The accent colour appears only as a small "type chip"
//      at the top-right corner — a 3-color budget per state
//      is preserved (neutral stroke, transparent body, accent
//      chip).
//    * The same draw functions render correctly at canvas size
//      (~82×72 px) and at button-icon size (16×16 px) because
//      every dimension is proportional to the supplied rect.
// ============================================================
namespace ElementIcon {

// Stroke width for idle vs. selected (selected gets a slightly bolder
// silhouette so it pops without changing shape).
inline qreal strokeWidth(bool sel) { return sel ? 1.6 : 1.0; }

// Type chip — a small filled square in the top-right corner of the icon
// rect that carries the element's accent colour. Acts as a colour key
// that survives even when the icon shape is small.
static void drawTypeChip(QPainter* p, QRectF r, const QColor& accent)
{
    const qreal s = qMax<qreal>(3.0, qMin(r.width(), r.height()) * 0.10);
    const QRectF chip(r.right() - s - 2.0, r.top() + 2.0, s, s);
    p->setPen(Qt::NoPen);
    p->setBrush(accent);
    p->drawRect(chip);
}

// ── Source — circle with sine-wave glyph (signal-source schematic) ──────
static void drawSource(QPainter* p, QRectF r, bool sel = false)
{
    p->save();
    p->setRenderHint(QPainter::Antialiasing, true);

    const QPointF c = r.center();
    const qreal   R = qMin(r.width(), r.height()) * 0.40;

    // Outer circle
    p->setBrush(Qt::NoBrush);
    p->setPen(QPen(sel ? CBStyle::ORANGE : CBStyle::TEXT, strokeWidth(sel)));
    p->drawEllipse(c, R, R);

    // Sine wave inside (one full cycle)
    QPainterPath wave;
    const qreal span = R * 1.4;
    const int   N    = 40;
    for (int i = 0; i <= N; ++i) {
        const qreal t  = qreal(i) / qreal(N);
        const qreal wx = c.x() - span / 2.0 + t * span;
        const qreal wy = c.y() + std::sin(t * 2.0 * M_PI) * R * 0.35;
        if (i == 0) wave.moveTo(wx, wy);
        else        wave.lineTo(wx, wy);
    }
    p->setPen(QPen(CBStyle::TEXT, 1.0));
    p->drawPath(wave);

    drawTypeChip(p, r, CBStyle::ORANGE);
    p->restore();
}

// ── Aperture — vertical front wall with a horizontal slot opening ───────
// Optional `withCover` parameter draws the small cover piece above the
// slot (used for the AP+Cover variant). When true, the type chip is still
// drawn — the cover is the silhouette differentiator, not a colour change.
static void drawAperture(QPainter* p, QRectF r, bool sel = false,
                         bool withCover = false)
{
    p->save();
    p->setRenderHint(QPainter::Antialiasing, true);

    const qreal pad = 4.0;
    const QRectF wall(r.x() + r.width()  * 0.20,
                      r.y() + pad,
                      r.width()  * 0.60,
                      r.height() - 2.0 * pad);

    // Wall outline
    p->setBrush(Qt::NoBrush);
    p->setPen(QPen(sel ? CBStyle::ACCENT : CBStyle::TEXT, strokeWidth(sel)));
    p->drawRect(wall);

    // Horizontal slot at mid-height
    const qreal sw = wall.width()  * 0.55;
    const qreal sh = qMax<qreal>(2.0, wall.height() * 0.10);
    const QRectF slot(wall.left() + (wall.width() - sw) / 2.0,
                      wall.center().y() - sh / 2.0,
                      sw, sh);
    p->setBrush(CBStyle::BG);
    p->setPen(QPen(CBStyle::TEXT, 1.0));
    p->drawRect(slot);

    // Cover piece (AP+Cover variant only)
    if (withCover) {
        const qreal cw = sw + 4.0;
        const qreal ch = qMax<qreal>(2.0, wall.height() * 0.08);
        const QRectF cover(wall.left() + (wall.width() - cw) / 2.0,
                           slot.top() - ch - 1.0,
                           cw, ch);
        p->setBrush(CBStyle::BG);
        p->setPen(QPen(CBStyle::TEXT, 1.0));
        p->drawRect(cover);
    }

    drawTypeChip(p, r, CBStyle::ACCENT);
    p->restore();
}

// ── Cavity — waveguide rectangle with internal propagation arrow.
// Optional `dielectric` parameter adds diagonal hatching in the lower
// half to indicate dielectric fill (Diel.Cav variant).
static void drawCavity(QPainter* p, QRectF r, bool sel = false,
                       const QString& /*lbl*/ = QString(),
                       bool dielectric = false)
{
    p->save();
    p->setRenderHint(QPainter::Antialiasing, true);

    const qreal pad = 4.0;
    const QRectF box(r.x() + pad,
                     r.y() + pad + 4.0,
                     r.width()  - 2.0 * pad,
                     r.height() - 2.0 * pad - 4.0);

    // Waveguide outline
    p->setBrush(Qt::NoBrush);
    p->setPen(QPen(sel ? CBStyle::GREEN : CBStyle::TEXT, strokeWidth(sel)));
    p->drawRect(box);

    // Diagonal hatching in lower half for the dielectric variant.
    if (dielectric) {
        p->setClipRect(QRectF(box.left() + 1.0,
                              box.top() + box.height() * 0.5 + 1.0,
                              box.width() - 2.0,
                              box.height() * 0.5 - 2.0));
        p->setPen(QPen(CBStyle::TEXT, 1.0));
        const qreal step = 4.0;
        // 45-degree lines covering the clipped region.
        for (qreal x = box.left() - box.height();
             x < box.right();
             x += step) {
            p->drawLine(QPointF(x,                box.top() + box.height()),
                        QPointF(x + box.height(), box.top()));
        }
        p->setClipping(false);
    }

    // Horizontal propagation arrow at upper-third height
    const qreal ay  = box.top() + box.height() * 0.30;
    const qreal ax0 = box.left() + 4.0;
    const qreal ax1 = box.right() - 8.0;
    p->setPen(QPen(CBStyle::TEXT, 1.0));
    p->drawLine(QPointF(ax0, ay), QPointF(ax1, ay));
    QPolygonF head;
    head << QPointF(ax1, ay)
         << QPointF(ax1 - 4.0, ay - 3.0)
         << QPointF(ax1 - 4.0, ay + 3.0);
    p->setBrush(CBStyle::TEXT);
    p->setPen(Qt::NoPen);
    p->drawPolygon(head);

    drawTypeChip(p, r, CBStyle::GREEN);
    p->restore();
}

// ── Observation point — circle with cross-hair (probe glyph) ────────────
static void drawObservation(QPainter* p, QRectF r, bool sel = false)
{
    p->save();
    p->setRenderHint(QPainter::Antialiasing, true);

    const QPointF c = r.center();
    const qreal   R = qMin(r.width(), r.height()) * 0.32;

    p->setBrush(Qt::NoBrush);
    p->setPen(QPen(sel ? CBStyle::RED : CBStyle::TEXT, strokeWidth(sel)));
    p->drawEllipse(c, R, R);

    p->setPen(QPen(CBStyle::TEXT, 1.0));
    p->drawLine(QPointF(c.x() - R * 0.9, c.y()),
                QPointF(c.x() + R * 0.9, c.y()));
    p->drawLine(QPointF(c.x(), c.y() - R * 0.9),
                QPointF(c.x(), c.y() + R * 0.9));

    // Centre dot in accent colour — ties the probe to the Obs.Pt type
    p->setPen(Qt::NoPen);
    p->setBrush(CBStyle::RED);
    p->drawEllipse(c, 2.0, 2.0);

    drawTypeChip(p, r, CBStyle::RED);
    p->restore();
}

} // namespace ElementIcon

// ============================================================
//  PaletteButton — full-width draggable left-panel tile
//
//  [T1.2-C] Redesigned for Task 1.2.
//  Old: 76×80 vertical tile (top toolbar)
//  New: full-width horizontal QPushButton subclass with a
//       16×16 painted icon on the left and a text label.
//
//  Drag behaviour is unchanged: a left-button press initiates
//  a QDrag whose mime payload is the integer ElementType.
//  The drop target (AssemblyCanvas) reads it back exactly as
//  before, so no canvas-side code needs to change.
// ============================================================
class PaletteButton : public QPushButton {
    Q_OBJECT
public:
    ElementType type;
    QString     typeName;
    QColor      accent;

    PaletteButton(ElementType t, const QString& n, const QColor& c, QWidget* p=nullptr)
        : QPushButton(p), type(t), typeName(n), accent(c)
    {
        setText("  " + n);                           // leading space gives breathing room from the icon
        setIcon(QIcon(makeIconPixmap(t)));
        setIconSize(QSize(16, 16));
        setMinimumHeight(30);
        setMaximumHeight(30);
        setCursor(Qt::OpenHandCursor);
        setToolTip(QString("Drag %1 onto canvas").arg(n));
        setStyleSheet(EMStyle::elementButtonQSS(c));
        // Buttons must not steal focus from the canvas / property editor
        // when the user clicks them — they only initiate drag.
        setFocusPolicy(Qt::NoFocus);
    }

protected:
    void mousePressEvent(QMouseEvent* ev) override {
        if (ev->button() != Qt::LeftButton) {
            QPushButton::mousePressEvent(ev);
            return;
        }
        // Initiate drag rather than emit clicked(); the canvas listens for
        // the drop event and constructs the CanvasElement there.
        QDrag* drag = new QDrag(this);
        QMimeData* mime = new QMimeData;
        mime->setText(QString::number(static_cast<int>(type)));
        drag->setMimeData(mime);

        // Drag preview pixmap: render the button at its current size so the
        // user gets a visual confirmation of what is being dragged.
        QPixmap pm(size());
        pm.fill(Qt::transparent);
        render(&pm);
        drag->setPixmap(pm);
        drag->setHotSpot(ev->pos());
        drag->exec(Qt::CopyAction);
    }

private:
    // Render the element-type symbol (Source sine-wave / Aperture wall /
    // AP+Cover wall+lid / Cavity box / Diel.Cav box+hatching / Obs.Pt
    // probe) into a 16×16 transparent pixmap by routing to the existing
    // ElementIcon::draw* helpers via the EMStyle::renderIcon16 adapter.
    //
    // drawAperture and drawCavity carry a fourth boolean argument
    // (withCover, dielectric) to differentiate variants. We wrap them in
    // lambdas with the variant flag bound at the call site so the caller
    // sees a clean 3-argument IconPainter signature.
    static QPixmap makeIconPixmap(ElementType t) {
        switch (t) {
        case ElementType::Source:
            return EMStyle::renderIcon16(&ElementIcon::drawSource);
        case ElementType::Aperture:
            return EMStyle::renderIcon16(
                [](QPainter* p, QRectF r, bool s) {
                    ElementIcon::drawAperture(p, r, s, /*withCover*/ false);
                });
        case ElementType::ApertureWithCover:
            return EMStyle::renderIcon16(
                [](QPainter* p, QRectF r, bool s) {
                    ElementIcon::drawAperture(p, r, s, /*withCover*/ true);
                });
        case ElementType::EmptyCavity:
            return EMStyle::renderIcon16(
                [](QPainter* p, QRectF r, bool s) {
                    ElementIcon::drawCavity(p, r, s, QString(), /*dielectric*/ false);
                });
        case ElementType::DielectricCavity:
            return EMStyle::renderIcon16(
                [](QPainter* p, QRectF r, bool s) {
                    ElementIcon::drawCavity(p, r, s, QString(), /*dielectric*/ true);
                });
        case ElementType::Load:
            return EMStyle::renderIcon16(&ElementIcon::drawObservation);
        }
        return QPixmap();
    }
};

// ============================================================
//  CanvasElement — movable QGraphicsItem
// ============================================================
class CanvasElement : public QGraphicsItem {
public:
    ElementParams params;
    static constexpr qreal W=90, H=80;

    explicit CanvasElement(ElementType t, QGraphicsItem* p=nullptr) : QGraphicsItem(p) {
        params.type  = t;
        params.label = defaultLabel(t);
        setFlags(ItemIsMovable|ItemIsSelectable|ItemSendsGeometryChanges);
        setAcceptHoverEvents(true);
    }

    QRectF boundingRect() const override { return {0,0,W,H+20}; }

    void paint(QPainter* p, const QStyleOptionGraphicsItem*, QWidget*) override {
        p->setRenderHint(QPainter::Antialiasing, true);
        const bool   sel = isSelected();
        const QColor ac  = accentColor();
        const bool   hov = isUnderMouse();

        // ── Body chrome ─────────────────────────────────────────────────
        // Three states, no gradients, no shadows:
        //   idle      — neutral border, transparent body
        //   hover     — accent-tinted border (1.2 px), transparent body
        //   selected  — accent border (1.6 px) + faint accent wash (alpha 30)
        if (sel) {
            // Selection chrome: tinted body fill + bold accent border.
            // Drawn slightly outside the icon rect so the silhouette inside
            // never collides with the border.
            p->setPen(Qt::NoPen);
            p->setBrush(QColor(ac.red(), ac.green(), ac.blue(), 30));
            p->drawRoundedRect(QRectF(-2, -2, W + 4, H + 4), 5.0, 5.0);
            p->setBrush(Qt::NoBrush);
            p->setPen(QPen(ac, 1.6));
            p->drawRoundedRect(QRectF(-2, -2, W + 4, H + 4), 5.0, 5.0);
        } else {
            p->setBrush(Qt::NoBrush);
            p->setPen(QPen(hov ? ac : CBStyle::BORDER, hov ? 1.2 : 0.8));
            p->drawRoundedRect(QRectF(0, 0, W, H), 4.0, 4.0);
        }

        // ── Element silhouette ─────────────────────────────────────────
        const QRectF ic(4, 4, W - 8, H - 8);
        switch (params.type) {
        case ElementType::Source:
            ElementIcon::drawSource(p, ic, sel);
            break;
        case ElementType::Aperture:
            ElementIcon::drawAperture(p, ic, sel, /*withCover*/ false);
            break;
        case ElementType::ApertureWithCover:
            ElementIcon::drawAperture(p, ic, sel, /*withCover*/ true);
            break;
        case ElementType::EmptyCavity:
            ElementIcon::drawCavity(p, ic, sel, QString(), /*dielectric*/ false);
            break;
        case ElementType::DielectricCavity:
            ElementIcon::drawCavity(p, ic, sel, QString(), /*dielectric*/ true);
            break;
        case ElementType::Load:
            ElementIcon::drawObservation(p, ic, sel);
            break;
        }

        // ── Label underneath ───────────────────────────────────────────
        // Label always uses the type accent colour so it doubles as a
        // colour key when the icon is far from the eye.
        QFont f("Courier New", 7);
        p->setFont(f);
        p->setPen(QPen(ac, 1.0));
        p->drawText(QRectF(0, H + 2, W, 16), Qt::AlignCenter, params.label);
    }

    QColor accentColor() const {
        switch(params.type){
        case ElementType::Source:            return CBStyle::ORANGE;
        case ElementType::Aperture:
        case ElementType::ApertureWithCover: return CBStyle::ACCENT;
        case ElementType::EmptyCavity:
        case ElementType::DielectricCavity:  return CBStyle::GREEN;
        case ElementType::Load:              return CBStyle::RED;
        } return CBStyle::TEXT;
    }

    static QString defaultLabel(ElementType t) {
        switch(t){
        case ElementType::Source:            return "E₀ Source";
        case ElementType::Aperture:          return "Aperture";
        case ElementType::ApertureWithCover: return "AP+Cover";
        case ElementType::EmptyCavity:       return "Cavity";
        case ElementType::DielectricCavity:  return "Diel. Cav.";
        case ElementType::Load:              return "Obs. Point";
        } return "Element";
    }

protected:
    void hoverEnterEvent(QGraphicsSceneHoverEvent*) override { update(); }
    void hoverLeaveEvent(QGraphicsSceneHoverEvent*) override { update(); }
    QVariant itemChange(GraphicsItemChange ch, const QVariant& v) override {
        if(ch==ItemPositionHasChanged && scene()) scene()->update();
        return QGraphicsItem::itemChange(ch,v);
    }
};

// ============================================================
//  ConnectionWire — Bezier wire between adjacent elements
// ============================================================
class ConnectionWire : public QGraphicsItem {
public:
    CanvasElement* src; CanvasElement* dst;
    ConnectionWire(CanvasElement* s, CanvasElement* d, QGraphicsItem* p=nullptr)
        : QGraphicsItem(p), src(s), dst(d)
    { setZValue(-1); setFlag(ItemStacksBehindParent); }

    QRectF boundingRect() const override {
        auto a=src->pos(), b=dst->pos();
        return { qMin(a.x(),b.x())-10, qMin(a.y(),b.y())-10,
                qAbs(a.x()-b.x())+CanvasElement::W+20,
                qAbs(a.y()-b.y())+CanvasElement::H+20 };
    }
    void paint(QPainter* p, const QStyleOptionGraphicsItem*, QWidget*) override {
        p->setRenderHint(QPainter::Antialiasing, true);

        // Anchor points: right edge of source, left edge of destination,
        // both at vertical centre of their elements.
        const QPointF a = src->pos() + QPointF(CanvasElement::W, CanvasElement::H / 2.0);
        const QPointF b = dst->pos() + QPointF(0.0,               CanvasElement::H / 2.0);

        // Reserve a small head zone so the arrowhead does not overlap the
        // destination's body chrome.
        constexpr qreal kHead     = 8.0;   // head length
        constexpr qreal kHeadHalf = 4.0;   // half-width of triangle base
        const QPointF tip  = b;
        const QPointF tail = (b.x() > a.x())
                                 ? QPointF(b.x() - kHead, b.y())
                                 : QPointF(b.x() + kHead, b.y());

        // Path: straight line if endpoints are vertically aligned, otherwise
        // a smooth horizontal-tangent cubic so the line enters/leaves both
        // elements perpendicular to their right/left edges.
        QPainterPath path;
        path.moveTo(a);
        if (qAbs(a.y() - tail.y()) < 1.5) {
            path.lineTo(tail);
        } else {
            const qreal mx = (a.x() + tail.x()) * 0.5;
            path.cubicTo(QPointF(mx, a.y()),
                         QPointF(mx, tail.y()),
                         tail);
        }

        // Solid muted line — single neutral colour regardless of selection.
        // Connections express topology, not selection state; tying the wire
        // colour to selection would make the canvas flash visually busy when
        // multiple elements are selected.
        p->setBrush(Qt::NoBrush);
        p->setPen(QPen(CBStyle::TEXT_MUTED, 1.5, Qt::SolidLine,
                       Qt::RoundCap, Qt::RoundJoin));
        p->drawPath(path);

        // Filled triangle arrowhead — direction taken from the tail of the
        // path so it points correctly whether the path is straight or
        // curved. We approximate the local angle by sampling the path
        // slightly before the tail.
        qreal angle = 0.0;
        const qreal len = path.length();
        if (len > 1.0) {
            const QPointF justBefore =
                path.pointAtPercent(qMax<qreal>(0.0, (len - 1.0) / len));
            angle = std::atan2(tail.y() - justBefore.y(),
                               tail.x() - justBefore.x());
        } else {
            angle = std::atan2(tail.y() - a.y(), tail.x() - a.x());
        }
        const QPointF baseCentre(tip.x() - kHead * std::cos(angle),
                                 tip.y() - kHead * std::sin(angle));
        const QPointF perp(-std::sin(angle) * kHeadHalf,
                           std::cos(angle) * kHeadHalf);
        QPolygonF head;
        head << tip
             << (baseCentre + perp)
             << (baseCentre - perp);
        p->setPen(Qt::NoPen);
        p->setBrush(CBStyle::TEXT_MUTED);
        p->drawPolygon(head);
    }
};

// ============================================================
//  AssemblyCanvas — QGraphicsView that accepts drops
// ============================================================
class AssemblyCanvas : public QGraphicsView {
    Q_OBJECT
public:
    QVector<CanvasElement*>  elements;
    QVector<ConnectionWire*> wires;

    explicit AssemblyCanvas(QWidget* p=nullptr) : QGraphicsView(p) {
        auto* sc = new QGraphicsScene(this);
        sc->setSceneRect(0,0,1600,600); sc->setBackgroundBrush(Qt::NoBrush); setScene(sc);
        setAcceptDrops(true);
        setRenderHints(QPainter::Antialiasing|QPainter::SmoothPixmapTransform);
        setDragMode(RubberBandDrag);
        setStyleSheet(QString("background:rgb(%1,%2,%3);border:none;")
                          .arg(CBStyle::BG.red()).arg(CBStyle::BG.green()).arg(CBStyle::BG.blue()));
        setHorizontalScrollBarPolicy(Qt::ScrollBarAsNeeded);
        setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);

        // [T1.5] Zoom support setup.
        //
        // setTransformationAnchor(AnchorUnderMouse) makes QGraphicsView::scale()
        // pivot around the cursor position rather than the view centre. This is
        // what gives the wheel-zoom its "pinch-around-cursor" feel.
        //
        // Focus policy is StrongFocus so that key events (Ctrl+0) are delivered
        // once the user has interacted with the canvas at least once.
        setTransformationAnchor(QGraphicsView::AnchorUnderMouse);
        setResizeAnchor(QGraphicsView::AnchorViewCenter);
        setFocusPolicy(Qt::StrongFocus);

        // Zoom indicator overlay — a frameless QLabel parented to the viewport
        // so it floats above the scene without participating in the scene's
        // coordinate system. Positioned in resizeEvent() to stay anchored
        // bottom-right whenever the view itself resizes.
        zoomIndicator_ = new QLabel(viewport());
        zoomIndicator_->setObjectName("ZoomIndicator");
        zoomIndicator_->setStyleSheet(QString(
                                          "QLabel#ZoomIndicator{"
                                          "background:%1;"
                                          "color:white;"
                                          "font-family:'Courier New',monospace;"
                                          "font-size:11px;"
                                          "font-weight:bold;"
                                          "padding:4px 10px;"
                                          "border:1px solid %2;"
                                          "border-radius:4px;"
                                          "}"
                                          )
                                          .arg(EMStyle::rgba(CBStyle::TEXT, 220))
                                          .arg(EMStyle::rgb(CBStyle::TEXT)));
        zoomIndicator_->setAlignment(Qt::AlignCenter);
        zoomIndicator_->setText(QStringLiteral("Zoom: 100%"));
        zoomIndicator_->adjustSize();
        zoomIndicator_->hide();

        // Auto-hide timer — restarted on every zoom action; fires once.
        zoomIndicatorTimer_ = new QTimer(this);
        zoomIndicatorTimer_->setSingleShot(true);
        zoomIndicatorTimer_->setInterval(2000);   // 2 seconds per spec
        connect(zoomIndicatorTimer_, &QTimer::timeout, this,
                [this]{ if (zoomIndicator_) zoomIndicator_->hide(); });
    }

signals:
    void elementSelected(CanvasElement* el);
    void elementDropped(ElementType type, QPointF pos);
    void circuitChanged();

protected:
    void drawBackground(QPainter* p, const QRectF& r) override {
        p->fillRect(r, CBStyle::BG);
        p->setPen(QPen(CBStyle::BORDER_LT,1.5));
        const int s=32;
        int x0=int(r.left())/s*s, y0=int(r.top())/s*s;
        for(int x=x0; x<r.right(); x+=s)
            for(int y=y0; y<r.bottom(); y+=s)
                p->drawPoint(x,y);
    }
    void dragEnterEvent(QDragEnterEvent* e) override { if(e->mimeData()->hasText()) e->acceptProposedAction(); }
    void dragMoveEvent(QDragMoveEvent*   e) override { if(e->mimeData()->hasText()) e->acceptProposedAction(); }
    void dropEvent(QDropEvent* e) override {
        bool ok=false; int t=e->mimeData()->text().toInt(&ok); if(!ok) return;
        emit elementDropped(static_cast<ElementType>(t), mapToScene(e->position().toPoint()));
        e->acceptProposedAction();
    }
    void mousePressEvent(QMouseEvent* e) override {
        QGraphicsView::mousePressEvent(e);
        emit elementSelected(dynamic_cast<CanvasElement*>(itemAt(e->pos())));
    }

    // ─── [T1.5] Mouse-wheel zoom (50%–300%, pivots on cursor) ───────────
    //
    // Scroll up  (angleDelta().y() > 0) → zoom in
    // Scroll down (angleDelta().y() < 0) → zoom out
    //
    // The zoom factor is applied incrementally via scale() rather than by
    // resetting the transform. This lets QGraphicsView's AnchorUnderMouse
    // mode handle the cursor-pivot math automatically.
    //
    // Bypassing QGraphicsView::wheelEvent() prevents the default behaviour
    // (vertical scroll), which would compete with the zoom.
    void wheelEvent(QWheelEvent* e) override {
        constexpr qreal kStep = 1.15;        // ~15% per notch — feels natural
        constexpr qreal kMin  = 0.50;        // 50% lower bound
        constexpr qreal kMax  = 3.00;        // 300% upper bound

        const int delta = e->angleDelta().y();
        if (delta == 0) {
            QGraphicsView::wheelEvent(e);
            return;
        }

        const qreal factor = (delta > 0) ? kStep : (1.0 / kStep);
        const qreal target = qBound(kMin, zoomFactor_ * factor, kMax);

        // If the clamp eliminates the change (we were already at the bound
        // and tried to push past it), don't apply anything — visually nothing
        // happens but the indicator still flashes to confirm the input.
        if (qFuzzyCompare(target, zoomFactor_)) {
            showZoomIndicator();
            e->accept();
            return;
        }

        const qreal applied = target / zoomFactor_;
        scale(applied, applied);
        zoomFactor_ = target;

        showZoomIndicator();
        e->accept();
    }

    // ─── [T1.5] Ctrl+0 resets zoom to 100% ──────────────────────────────
    //
    // Resets the entire view transform (which is exactly the accumulated
    // scale, since we never apply any other transform). After reset, also
    // recenter on the scene contents so the user isn't left looking at
    // empty space if they had zoomed deep into a corner.
    void keyPressEvent(QKeyEvent* e) override {
        if (e->key() == Qt::Key_0 && (e->modifiers() & Qt::ControlModifier)) {
            resetTransform();
            zoomFactor_ = 1.0;
            // If there are elements, gently re-centre so the user lands on
            // something visible. If empty, leave the view at the origin.
            if (!elements.isEmpty()) {
                centerOn(scene()->itemsBoundingRect().center());
            }
            showZoomIndicator();
            e->accept();
            return;
        }
        QGraphicsView::keyPressEvent(e);
    }

    // [T1.5] Keep the zoom indicator anchored at bottom-right when the
    // viewport resizes (window resize, splitter drag, etc.).
    void resizeEvent(QResizeEvent* e) override {
        QGraphicsView::resizeEvent(e);
        repositionZoomIndicator();
    }

private:
    // [T1.5] Update the indicator text, move it to its anchor position,
    // make it visible, and (re)start the 2-second auto-hide timer.
    void showZoomIndicator() {
        if (!zoomIndicator_) return;
        zoomIndicator_->setText(
            QStringLiteral("Zoom: %1%").arg(int(std::round(zoomFactor_ * 100.0))));
        zoomIndicator_->adjustSize();
        repositionZoomIndicator();
        zoomIndicator_->raise();
        zoomIndicator_->show();
        zoomIndicatorTimer_->start();   // resets the countdown
    }

    void repositionZoomIndicator() {
        if (!zoomIndicator_ || !viewport()) return;
        const int margin = 12;
        const int x = viewport()->width()  - zoomIndicator_->width()  - margin;
        const int y = viewport()->height() - zoomIndicator_->height() - margin;
        zoomIndicator_->move(qMax(0, x), qMax(0, y));
    }

    qreal   zoomFactor_         {1.0};
    QLabel* zoomIndicator_      {nullptr};
    QTimer* zoomIndicatorTimer_ {nullptr};

public slots:
    CanvasElement* addElement(ElementType type, QPointF pos) {
        auto* el = new CanvasElement(type); el->setPos(pos);
        scene()->addItem(el); elements.append(el);
        if(elements.size()>1){
            auto* w = new ConnectionWire(elements[elements.size()-2], el);
            scene()->addItem(w); wires.append(w);
        }
        emit circuitChanged(); return el;
    }
    void removeSelected() {
        for(auto* it : scene()->selectedItems()){
            auto* el = dynamic_cast<CanvasElement*>(it); if(!el) continue;
            wires.erase(std::remove_if(wires.begin(),wires.end(),[&](ConnectionWire* w){
                            if(w->src==el||w->dst==el){ scene()->removeItem(w); delete w; return true; }
                            return false;
                        }),wires.end());
            elements.removeOne(el); scene()->removeItem(el); delete el;
        }
        emit circuitChanged();
    }
    void clearAll() {
        for(auto* w:wires){ scene()->removeItem(w); delete w; }
        for(auto* e:elements){ scene()->removeItem(e); delete e; }
        wires.clear(); elements.clear(); emit circuitChanged();
    }
    void autoArrange() {
        for(int i=0;i<elements.size();++i) elements[i]->setPos(60+i*130, 200);
        for(auto* w:wires){ scene()->removeItem(w); delete w; } wires.clear();
        for(int i=0;i<elements.size()-1;++i){
            auto* w=new ConnectionWire(elements[i],elements[i+1]);
            scene()->addItem(w); wires.append(w);
        }
        scene()->update();
        // [T1.4] Auto-arrange may renumber elements left-to-right.
        // Emit circuitChanged so the StackLayerPanel rebuilds with the
        // new ordering. Symmetric with addElement / removeSelected /
        // clearAll, which all emit circuitChanged on structural change.
        emit circuitChanged();
    }
};

// [T1.4-INCLUDE]
// StackLayerPanel.h depends on CanvasElement, ElementType, ElementParams,
// and ElementIcon — all declared above. It MUST be included after those
// declarations are visible. See StackLayerPanel.h for the matching guard.
#include "StackLayerPanel.h"

// ============================================================
//  CLocaleDoubleSpinBox
//
//  Subclass of QDoubleSpinBox that ALWAYS uses the C locale
//  (decimal point = '.') regardless of the system locale.
//
//  Root cause of the t_wall / f_start revert bug on Russian/European
//  Windows: Qt's QDoubleSpinBox calls QLocale::toDouble() in both
//  validate() and valueFromText(), which uses ',' as the decimal
//  separator on these systems. Even after setLocale(QLocale::c()),
//  Qt 5.x on some Windows builds still falls back to the system
//  locale in the internal validator. This subclass overrides both
//  methods to guarantee correct parsing.
//
//  Fix: override validate() to replace ',' with '.' before passing
//  to the base class, and override valueFromText() to parse with
//  QString::toDouble() which always uses '.'.
// ============================================================
class CLocaleDoubleSpinBox : public QDoubleSpinBox {
public:
    explicit CLocaleDoubleSpinBox(QWidget* parent = nullptr)
        : QDoubleSpinBox(parent)
    {
        setLocale(QLocale::c());
    }

    QValidator::State validate(QString& input, int& pos) const override {
        input.replace(',', '.');
        return QDoubleSpinBox::validate(input, pos);
    }

    double valueFromText(const QString& text) const override {
        QString t = text;
        t.replace(',', '.');
        t.remove(' ');
        bool ok = false;
        const double v = t.toDouble(&ok);
        return ok ? v : minimum();
    }

    QString textFromValue(double value) const override {
        return QLocale::c().toString(value, 'f', decimals());
    }
};

// ============================================================
//  BuilderPropertyPanel — context-sensitive parameter editor
//
//  [T1.2-E] All [=] lambdas in addDouble / addInt / addLineEdit
//  changed to explicit [this, fn] captures (the user-supplied
//  callback is captured by value, currentElement_ is read via
//  this).  This eliminates the C++20 implicit-this warning.
// ============================================================
class BuilderPropertyPanel : public QWidget {
    Q_OBJECT
public:
    explicit BuilderPropertyPanel(QWidget* p=nullptr) : QWidget(p) {
        // [T1.2-B] Width hints are now advisory: the enclosing splitter
        // handle lets the user resize the panel freely.  The minimum is
        // kept so the panel stays usable when collapsed close to the
        // brand strip.
        setMinimumWidth(220);
        setStyleSheet(QString("background:rgb(%1,%2,%3);color:rgb(%4,%5,%6);")
                          .arg(CBStyle::SURFACE.red()).arg(CBStyle::SURFACE.green()).arg(CBStyle::SURFACE.blue())
                          .arg(CBStyle::TEXT.red()).arg(CBStyle::TEXT.green()).arg(CBStyle::TEXT.blue()));
        auto* l=new QVBoxLayout(this); l->setContentsMargins(10,10,10,8); l->setSpacing(4);

        headerLabel_=new QLabel("PROPERTIES",this);
        headerLabel_->setStyleSheet(dimLabel());
        l->addWidget(headerLabel_);

        typeLabel_=new QLabel("No element selected",this);
        typeLabel_->setStyleSheet(mutedLabel());
        l->addWidget(typeLabel_);

        scrollArea_=new QScrollArea(this); scrollArea_->setWidgetResizable(true);
        scrollArea_->setStyleSheet("border:none;background:transparent;");
        formWidget_=new QWidget; formLayout_=new QFormLayout(formWidget_);
        formLayout_->setContentsMargins(0,6,0,6); formLayout_->setSpacing(6);
        formLayout_->setLabelAlignment(Qt::AlignRight);
        scrollArea_->setWidget(formWidget_); l->addWidget(scrollArea_);

        // [FIX-B4] Correct 5-element circuit matching Phase A (Fig. 3.7)
        placeholderLabel_=new QLabel(
            "  CORRECT CIRCUIT\n"
            "  (matches Phase A):\n\n"
            "  1. [Source]\n"
            "       a, b, t_wall,\n"
            "       f_start, f_end\n"
            "     ↓  (series, 0→1)\n"
            "  2. [Aperture]\n"
            "       l_slot, w_slot\n"
            "     ↓  (SHUNT 1→0)\n"
            "  3. [Cavity]  L = p\n"
            "       e.g. 0.150 m\n"
            "     ↓  (series 1→2)\n"
            "  4. [Obs.Pt]  ← SE_P1\n"
            "     ↓  (SHUNT 2→0)\n"
            "  5. [Cavity]  L = d-p\n"
            "       e.g. 0.150 m\n"
            "     ↓  (series 2→3)\n"
            "  ╚═ Back-wall short\n"
            "     auto-added at node 3\n\n"
            "  Use Arrange after\n"
            "  dropping elements.\n\n"
            "  Click element to\n"
            "  edit its params.",this);
        placeholderLabel_->setStyleSheet(dimLabel()+" font-size:10px;");
        placeholderLabel_->setAlignment(Qt::AlignLeft);
        l->addWidget(placeholderLabel_);
        // No final stretch: the enclosing left-panel layout owns the stretch
        // policy now (see CircuitBuilderWindow::buildLeftPanel).
        showPlaceholder(true);
    }

signals:
    void paramsChanged(CanvasElement* el);

public slots:
    void showElement(CanvasElement* el) {
        // [BUGFIX-PARAMS-RESET] Guard the entire form rebuild.
        // m_loading_ stays true from the moment we change currentElement_
        // through the construction of every spinbox in the new form.
        // All spurious valueChanged() emissions caused by widget
        // construction (setValue with a non-zero default) and widget
        // destruction (clearForm -> ~QDoubleSpinBox) are observed by the
        // lambdas which see the flag and return early without writing.
        m_loading_ = true;
        currentElement_=el; clearForm();
        if(!el){
            showPlaceholder(true);
            typeLabel_->setText("No element selected");
            m_loading_ = false;
            return;
        }
        showPlaceholder(false);
        QColor col=el->accentColor();
        typeLabel_->setText(typeToStr(el->params.type));
        typeLabel_->setStyleSheet(QString("font-family:'Courier New';font-size:11px;font-weight:bold;"
                                          "color:rgb(%1,%2,%3);").arg(col.red()).arg(col.green()).arg(col.blue()));

        // [T1.2-E] Explicit [this] captures throughout. The fn callbacks
        // are captured by value (default behaviour for std::function).
        addLineEdit("Label:", el->params.label,
                    [this](const QString& v){ currentElement_->params.label = v; currentElement_->update(); });

        switch(el->params.type){
        case ElementType::Source:
            addDouble("E₀ [V/m]:",     el->params.E0,         0.01,  1000,  0.1,
                      [this](double v){ currentElement_->params.E0 = v; });
            // [FIX-B1] min=0.0001 GHz = 100 kHz — allows Phase A's 1 MHz start
            addDouble("f start [GHz]:", el->params.freqStart, 0.0001, 100, 0.001,
                      [this](double v){ currentElement_->params.freqStart = v; });
            addDouble("f end [GHz]:",  el->params.freqEnd,    0.1,    100, 1.0,
                      [this](double v){ currentElement_->params.freqEnd = v; });
            addInt   ("Points:",       el->params.freqPoints, 10,    2000, 50,
                   [this](int v)   { currentElement_->params.freqPoints = v; });
            addSep("Cross-section (shared by all):");
            addDouble("a [m]:",        el->params.a,          0.001,  1.0, 0.005,
                      [this](double v){ currentElement_->params.a = v; });
            addDouble("b [m]:",        el->params.b_h,        0.001,  1.0, 0.005,
                      [this](double v){ currentElement_->params.b_h = v; });
            addDouble("t_wall [m]:",   el->params.t_wall,     0.0001, 0.1, 0.0001,
                      [this](double v){ currentElement_->params.t_wall = v; });
            break;
        case ElementType::Aperture:
            addDouble("l_slot [m]:", el->params.l_slot, 0.001,  1.0, 0.002,
                      [this](double v){ currentElement_->params.l_slot = v; });
            addDouble("w_slot [m]:", el->params.w_slot, 0.0001, 0.1, 0.0005,
                      [this](double v){ currentElement_->params.w_slot = v; });
            break;
        case ElementType::ApertureWithCover:
            addDouble("l_slot [m]:", el->params.l_slot,    0.001,  1.0,  0.002,
                      [this](double v){ currentElement_->params.l_slot = v; });
            addDouble("w_slot [m]:", el->params.w_slot,    0.0001, 0.1,  0.0005,
                      [this](double v){ currentElement_->params.w_slot = v; });
            addDouble("τ gap [m]:",  el->params.tau_cover, 0.0001, 0.01, 0.0001,
                      [this](double v){ currentElement_->params.tau_cover = v; });
            break;
        case ElementType::EmptyCavity:
            addDouble("L [m]:", el->params.L_cavity, 0.001, 2.0, 0.01,
                      [this](double v){ currentElement_->params.L_cavity = v; });
            break;
        case ElementType::DielectricCavity:
            addDouble("L [m]:",      el->params.L_cavity,     0.001,  2.0,   0.01,
                      [this](double v){ currentElement_->params.L_cavity = v; });
            addDouble("h_diel [m]:", el->params.h_dielectric, 0.0001, 1.0,   0.001,
                      [this](double v){ currentElement_->params.h_dielectric = v; });
            addDouble("ε_r:",        el->params.eps_r,         1.0,   100.0, 0.5,
                      [this](double v){ currentElement_->params.eps_r = v; });
            break;
        case ElementType::Load:
            addInfo("SHUNT observation tap.\n"
                    "SE at this node (Eq. 3.8):\n"
                    "SE=-20·log₁₀|2U/V₀|\n\n"
                    "Z_L >> Z₀ = non-loading.\n"
                    "Default: 1e9 Ω (correct).\n"
                    "WARNING: 377Ω = matched\n"
                    "load → kills resonances!\n\n"
                    "CORRECT CIRCUIT ORDER:\n"
                    "Source → Aperture\n"
                    "→ Cavity (L = p)\n"
                    "→ Obs.Pt (Z=1e9)\n"
                    "→ Cavity (L = d-p)\n"
                    "Back-wall short added\n"
                    "automatically.");
            // [FIX-D4] max=1e10 covers the 1e9 default; step=1e6 for navigation
            addDouble("Z_L real [Ω]:", el->params.ZL_real,  0.001,    1.0e10, 1.0e6,
                      [this](double v){ currentElement_->params.ZL_real = v; });
            addDouble("Z_L imag [Ω]:", el->params.ZL_imag, -1.0e9,    1.0e9,  1.0e3,
                      [this](double v){ currentElement_->params.ZL_imag = v; });
            break;
        }

        // [BUGFIX-PARAMS-RESET] Form is fully built; allow user edits to
        // flow through to currentElement_->params from this point on.
        m_loading_ = false;
    }

private:
    QLabel* headerLabel_; QLabel* typeLabel_; QLabel* placeholderLabel_;
    QScrollArea* scrollArea_; QWidget* formWidget_; QFormLayout* formLayout_;
    CanvasElement* currentElement_{nullptr};

    // [BUGFIX-PARAMS-RESET]
    // Guard flag set to true while showElement() is rebuilding the form.
    // Each user-callback lambda checks this flag and early-returns if it
    // is true. Without this guard, the spurious valueChanged() emissions
    // that Qt fires during widget construction (setValue() with a
    // non-default value) and during widget destruction (clearForm() ->
    // removeRow() -> ~QDoubleSpinBox()) reach the lambdas and write into
    // `currentElement_->params`, which by step 1 of showElement() already
    // points at the NEXT element being shown. The result is that every
    // time the user clicks a different element, that element's params
    // are clobbered with stale values from the previously-displayed form.
    bool m_loading_{false};

    QString dimLabel()   const {
        return QString("color:rgb(%1,%2,%3);font-family:'Courier New';font-size:10px;letter-spacing:1px;")
        .arg(CBStyle::TEXT_DIM.red()).arg(CBStyle::TEXT_DIM.green()).arg(CBStyle::TEXT_DIM.blue());
    }
    QString mutedLabel() const {
        return QString("color:rgb(%1,%2,%3);font-family:'Courier New';font-size:11px;")
        .arg(CBStyle::TEXT_MUTED.red()).arg(CBStyle::TEXT_MUTED.green()).arg(CBStyle::TEXT_MUTED.blue());
    }
    QString typeToStr(ElementType t) {
        switch(t){
        case ElementType::Source:            return "SOURCE";
        case ElementType::Aperture:          return "APERTURE (shunt)";
        case ElementType::ApertureWithCover: return "APERTURE + COVER (shunt)";
        case ElementType::EmptyCavity:       return "EMPTY CAVITY";
        case ElementType::DielectricCavity:  return "DIELECTRIC CAVITY";
        case ElementType::Load:              return "OBS. POINT (shunt)";
        } return "ELEMENT";
    }
    void clearForm(){
        // [BUGFIX-PARAMS-RESET]
        // Disconnect all signals from each widget BEFORE removing it.
        // QFormLayout::removeRow() calls deleteLater() on the contained
        // widgets, and during their destruction Qt fires one final
        // valueChanged() / textChanged() that would otherwise reach the
        // lambdas connected in addDouble() / addInt() / addLineEdit().
        // Although the m_loading_ guard already protects against a stale
        // write, disconnecting here is belt-and-braces and also avoids
        // the cost of dispatching those signals at all.
        for(int row = 0; row < formLayout_->rowCount(); ++row){
            for(int role = 0; role < 2; ++role){
                if(auto* item = formLayout_->itemAt(row, static_cast<QFormLayout::ItemRole>(role))){
                    if(QWidget* w = item->widget()){
                        QObject::disconnect(w, nullptr, this, nullptr);
                    }
                }
            }
        }
        while(formLayout_->rowCount() > 0) formLayout_->removeRow(0);
    }
    void showPlaceholder(bool s){ scrollArea_->setVisible(!s); placeholderLabel_->setVisible(s); }

    QString spinSS(QColor col) {
        // [T2.1a] Forwarded to EMStyle::spinSS — single source of truth.
        // Body kept as a forwarder so existing addDouble/addInt/addLineEdit
        // call sites compile unchanged.
        return EMStyle::spinSS(col);
    }
    QString lblSS() {
        // [T2.1a] Forwarded to EMStyle::lblSS — single source of truth.
        return EMStyle::lblSS();
    }

    // [T1.2-E] addDouble / addInt / addLineEdit:
    // captures changed from [=] to explicit [this, fn] to satisfy
    // C++20's "implicit capture of this via [=] is deprecated" warning.
    //
    // [BUGFIX-PARAMS-RESET] Each lambda checks m_loading_ and returns
    // early if the form is still being populated. This blocks the
    // setValue()-induced valueChanged() emissions during construction
    // and the ~QDoubleSpinBox()-induced final emission during destruction
    // from writing to currentElement_->params (which by then points at
    // the next element, not the one whose data is being loaded).
    //
    // [BUGFIX-DECIMALS-ROUNDING] CRITICAL ORDER OF setup() CALLS:
    //   1. setDecimals(6)  ← MUST be first
    //   2. setRange(mn, mx)
    //   3. setValue(v)
    //   4. setSingleStep(step)
    //
    // QDoubleSpinBox::setValue() rounds the supplied value to the precision
    // currently set by decimals(). On a fresh QDoubleSpinBox, decimals()
    // defaults to 2. Calling setValue(0.005) before setDecimals(6) silently
    // rounds 0.005 → 0.01 (the nearest two-decimal value). Subsequently
    // calling setDecimals(6) does NOT recover the original precision —
    // it only re-formats the already-rounded internal value as "0.010000".
    //
    // Symptom: any user-entered value with more than two decimals of
    // precision (e.g. typing 0.005, 0.003, 0.0015) silently mutates to
    // its 2-decimal rounded form the next time the property panel is
    // rebuilt for that element.
    //
    // The diagnostic log made this unambiguous:
    //   addDouble incoming v=0.005, min=0.001, max=2, step=0.01
    //   after setValue+setDecimals: spinbox.value()=0.01 text="0.010000"
    //
    // Fix: call setDecimals(6) BEFORE setValue() so the value is rounded
    // to 6 decimals (i.e., not rounded at all for our use case).
    void addDouble(const QString& l, double v, double mn, double mx, double step,
                   std::function<void(double)> fn)
    {
        auto* s = new CLocaleDoubleSpinBox(formWidget_);
        s->setDecimals(6);                           // [BUGFIX-DECIMALS-ROUNDING] FIRST
        s->setRange(mn, mx);
        s->setValue(v);                              // value preserved at 6-decimal precision
        s->setSingleStep(step);
        QColor c = currentElement_ ? currentElement_->accentColor() : CBStyle::ACCENT;
        s->setStyleSheet(spinSS(c));
        connect(s, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
                [this, fn](double val){
                    if (m_loading_) return;
                    fn(val);
                    emit paramsChanged(currentElement_);
                });
        auto* ll = new QLabel(l, formWidget_); ll->setStyleSheet(lblSS());
        formLayout_->addRow(ll, s);
    }
    void addInt(const QString& l, int v, int mn, int mx, int step,
                std::function<void(int)> fn)
    {
        auto* s = new QSpinBox(formWidget_);
        s->setLocale(QLocale::c());
        s->setRange(mn, mx); s->setValue(v); s->setSingleStep(step);
        QColor c = currentElement_ ? currentElement_->accentColor() : CBStyle::ACCENT;
        s->setStyleSheet(spinSS(c));
        connect(s, QOverload<int>::of(&QSpinBox::valueChanged), this,
                [this, fn](int val){
                    if (m_loading_) return;
                    fn(val);
                    emit paramsChanged(currentElement_);
                });
        auto* ll = new QLabel(l, formWidget_); ll->setStyleSheet(lblSS());
        formLayout_->addRow(ll, s);
    }
    void addLineEdit(const QString& l, const QString& v,
                     std::function<void(const QString&)> fn)
    {
        auto* e = new QLineEdit(formWidget_); e->setText(v);
        QColor c = currentElement_ ? currentElement_->accentColor() : CBStyle::ACCENT;
        e->setStyleSheet(spinSS(c));
        connect(e, &QLineEdit::textChanged, this,
                [this, fn](const QString& val){
                    if (m_loading_) return;
                    fn(val);
                    emit paramsChanged(currentElement_);
                });
        auto* ll = new QLabel(l, formWidget_); ll->setStyleSheet(lblSS());
        formLayout_->addRow(ll, e);
    }
    void addSep(const QString& title) {
        auto* ll=new QLabel(title,formWidget_);
        ll->setStyleSheet(QString("color:rgb(%1,%2,%3);font-family:'Courier New';font-size:9px;"
                                  "letter-spacing:1px;margin-top:6px;border-top:1px solid rgb(%4,%5,%6);")
                              .arg(CBStyle::TEXT_DIM.red()).arg(CBStyle::TEXT_DIM.green()).arg(CBStyle::TEXT_DIM.blue())
                              .arg(CBStyle::BORDER.red()).arg(CBStyle::BORDER.green()).arg(CBStyle::BORDER.blue()));
        formLayout_->addRow(ll);
    }
    void addInfo(const QString& text) {
        auto* ll=new QLabel(text,formWidget_); ll->setWordWrap(true);
        ll->setStyleSheet(QString("color:rgb(%1,%2,%3);background:rgb(%4,%5,%6);"
                                  "border:1px solid rgb(%7,%8,%9);border-radius:4px;"
                                  "font-family:'Courier New';font-size:9px;padding:5px 6px;margin-bottom:4px;")
                              .arg(CBStyle::TEXT_MUTED.red()).arg(CBStyle::TEXT_MUTED.green()).arg(CBStyle::TEXT_MUTED.blue())
                              .arg(CBStyle::SURFACE2.red()).arg(CBStyle::SURFACE2.green()).arg(CBStyle::SURFACE2.blue())
                              .arg(CBStyle::BORDER.red()).arg(CBStyle::BORDER.green()).arg(CBStyle::BORDER.blue()));
        formLayout_->addRow(ll);
    }
};

// ============================================================
//  CircuitBuilderWindow — Phase B main window
// ============================================================
class CircuitBuilderWindow : public QMainWindow {
    Q_OBJECT

public:
    explicit CircuitBuilderWindow(QWidget* p=nullptr) : QMainWindow(p) {
        setWindowTitle("Circuit Builder — EMShieldDesigner");
        setMinimumSize(1200,720); resize(1440,860);
        applyGlobalStyle(); buildUI(); connectSignals();
    }

private:
    // ── Widgets ──────────────────────────────────────────────────
    AssemblyCanvas*       canvas_     {nullptr};
    BuilderPropertyPanel* propPanel_  {nullptr};
    StackLayerPanel*      stackPanel_ {nullptr};   // [T1.4] right-side layer stack
    QCustomPlot*          m_plot      {nullptr};
    QLabel*               statusLbl_  {nullptr};

    // [T1.2-B] Action buttons stored as members so connectSignals()
    // can wire them up. Element palette buttons do not need member
    // storage — their drag behaviour is fully encapsulated inside
    // PaletteButton::mousePressEvent().
    QPushButton* btnArrange_ {nullptr};
    QPushButton* btnDelete_  {nullptr};
    QPushButton* btnClear_   {nullptr};
    QPushButton* btnCompute_ {nullptr};
    QPushButton* btnExport_  {nullptr};

    // [T1.5b] Topology validity indicator — red/green dot + brief text
    // shown directly above the COMPUTE button. Driven by
    // refreshValidityIndicator() which re-runs validateCircuitCore() on
    // every circuitChanged signal. The Compute button itself remains
    // clickable in all states; clicking it on an invalid circuit raises
    // a full-text dialog. The row's tooltip carries the same full text.
    QFrame*  validityDot_   {nullptr};
    QLabel*  validityLabel_ {nullptr};
    QWidget* validityRow_   {nullptr};

    // ── Computed result store (export + interactive readout) ─────
    QVector<double>          m_freqs;
    QVector<QVector<double>> m_SE_data;
    QVector<QString>         m_labels;

    // ── Interactive overlay pointers ─────────────────────────────
    QCPItemLine*            m_crosshairLine {nullptr};
    QCPItemText*            m_readoutLabel  {nullptr};
    QVector<QCPItemTracer*> m_tracers;

    // ── Curve colour palette (same as Phase A / mainwindow) ──────
    static const QVector<QColor>& palette() {
        static const QVector<QColor> p = {
                                          { 37,  99, 235}, {220,  38,  38}, { 22, 163,  74},
                                          {217, 119,   6}, {147,  51, 234}, { 14, 165, 233},
                                          {244,  63,  94}, { 34, 197,  94},
                                          };
        return p;
    }

    // ============================================================
    //  Style
    //
    //  Note: the QToolBar / QStatusBar entries previously in this
    //  string are kept harmless; the toolbar no longer exists in
    //  Task 1.2 but the rule is inert when no QToolBar is present.
    // ============================================================
    void applyGlobalStyle() {
        setStyleSheet(QString(R"(
            QMainWindow,QWidget{background:rgb(%1,%2,%3);color:rgb(%4,%5,%6);font-family:'Courier New',monospace;}
            QStatusBar{background:rgb(%7,%8,%9);border-top:1px solid rgb(%10,%11,%12);color:rgb(%13,%14,%15);font-size:11px;}
            QSplitter::handle{background:rgb(%10,%11,%12);width:1px;height:1px;}
        )")
                          .arg(CBStyle::BG.red())    .arg(CBStyle::BG.green())    .arg(CBStyle::BG.blue())
                          .arg(CBStyle::TEXT.red())  .arg(CBStyle::TEXT.green())  .arg(CBStyle::TEXT.blue())
                          .arg(CBStyle::SURFACE.red()).arg(CBStyle::SURFACE.green()).arg(CBStyle::SURFACE.blue())
                          .arg(CBStyle::BORDER.red()) .arg(CBStyle::BORDER.green()) .arg(CBStyle::BORDER.blue())
                          .arg(CBStyle::TEXT_MUTED.red()).arg(CBStyle::TEXT_MUTED.green()).arg(CBStyle::TEXT_MUTED.blue()));
    }

    // ============================================================
    //  buildLeftPanel — [T1.2-B] new helper
    //
    //  Composes the full vertical left-side panel:
    //    1. Brand strip          (relocated from old toolbar)
    //    2. Property editor      (existing BuilderPropertyPanel)
    //    3. ELEMENTS section     (6 draggable PaletteButtons)
    //    4. ACTIONS section      (Arrange, Delete, Clear)
    //    5. Compute (primary)
    //    6. Export CSV
    //
    //  Each section is separated by a thin divider. The property
    //  editor expands to fill remaining vertical space; everything
    //  below it is fixed-size.
    // ============================================================
    QWidget* buildLeftPanel() {
        auto* panel = new QWidget;
        panel->setObjectName("LeftPanel");
        panel->setStyleSheet(QString("QWidget#LeftPanel{background:%1;}")
                                 .arg(EMStyle::rgb(CBStyle::SURFACE)));

        auto* root = new QVBoxLayout(panel);
        root->setContentsMargins(0, 0, 0, 0);
        root->setSpacing(0);

        // ── 1. Brand strip ────────────────────────────────────────
        auto* brand = new QLabel(EMStyle::brandStripText(), panel);
        brand->setTextFormat(Qt::RichText);
        brand->setStyleSheet(EMStyle::brandStripQSS());
        brand->setAlignment(Qt::AlignCenter);
        root->addWidget(brand);

        // ── 2. Property editor (existing) ─────────────────────────
        propPanel_ = new BuilderPropertyPanel(panel);
        // Property editor expands to take remaining vertical room.
        propPanel_->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Expanding);
        root->addWidget(propPanel_, /*stretch*/ 1);

        // ── 3. ELEMENTS section ───────────────────────────────────
        auto* elementsSection = new QWidget(panel);
        elementsSection->setStyleSheet(QString("background:%1;")
                                           .arg(EMStyle::rgb(CBStyle::SURFACE)));
        auto* elementsLayout = new QVBoxLayout(elementsSection);
        elementsLayout->setContentsMargins(10, 4, 10, 8);
        elementsLayout->setSpacing(4);

        auto* elementsHeader = new QLabel("ELEMENTS", elementsSection);
        elementsHeader->setStyleSheet(EMStyle::sectionHeaderQSS());
        elementsLayout->addWidget(elementsHeader);

        struct PSpec { ElementType t; QString n; QColor c; };
        const QVector<PSpec> palette = {
                                        { ElementType::Source,            "Source",   CBStyle::ORANGE },
                                        { ElementType::Aperture,          "Aperture", CBStyle::ACCENT },
                                        { ElementType::ApertureWithCover, "AP+Cover", CBStyle::ACCENT },
                                        { ElementType::EmptyCavity,       "Cavity",   CBStyle::GREEN  },
                                        { ElementType::DielectricCavity,  "Diel.Cav", CBStyle::GREEN  },
                                        { ElementType::Load,              "Obs.Pt",   CBStyle::RED    },
                                        };
        for (const auto& s : palette) {
            auto* btn = new PaletteButton(s.t, s.n, s.c, elementsSection);
            elementsLayout->addWidget(btn);
        }
        root->addWidget(elementsSection);

        // ── 4. ACTIONS section ────────────────────────────────────
        auto* actionsSection = new QWidget(panel);
        actionsSection->setStyleSheet(QString("background:%1;")
                                          .arg(EMStyle::rgb(CBStyle::SURFACE)));
        auto* actionsLayout = new QVBoxLayout(actionsSection);
        actionsLayout->setContentsMargins(10, 4, 10, 4);
        actionsLayout->setSpacing(4);

        auto* actionsHeader = new QLabel("ACTIONS", actionsSection);
        actionsHeader->setStyleSheet(EMStyle::sectionHeaderQSS());
        actionsLayout->addWidget(actionsHeader);

        btnArrange_ = makeActionButton("Arrange",    EMStyle::AccentRole::Neutral, actionsSection);
        btnDelete_  = makeActionButton("Delete",     EMStyle::AccentRole::Danger,  actionsSection);
        btnClear_   = makeActionButton("Clear",      EMStyle::AccentRole::Source,  actionsSection);
        actionsLayout->addWidget(btnArrange_);
        actionsLayout->addWidget(btnDelete_);
        actionsLayout->addWidget(btnClear_);
        root->addWidget(actionsSection);

        // ── 5+6. PRIMARY section (Compute + Export) ──────────────
        auto* primarySection = new QWidget(panel);
        primarySection->setStyleSheet(QString("background:%1;border-top:1px solid %2;")
                                          .arg(EMStyle::rgb(CBStyle::SURFACE))
                                          .arg(EMStyle::rgb(CBStyle::BORDER_LT)));
        auto* primaryLayout = new QVBoxLayout(primarySection);
        primaryLayout->setContentsMargins(10, 8, 10, 12);
        primaryLayout->setSpacing(6);

        // [T1.5b] Validity indicator row.
        // Sits just above the COMPUTE button so its colour gives the user
        // a one-glance read of whether Compute will succeed. The row is a
        // thin horizontal strip:
        //
        //     [● 12px]   <brief status text>
        //
        // Tooltip on the whole row carries the long-form explanation.
        validityRow_ = new QWidget(primarySection);
        validityRow_->setObjectName("ValidityRow");
        validityRow_->setStyleSheet(
            "QWidget#ValidityRow{background:transparent;}");
        auto* validityLayout = new QHBoxLayout(validityRow_);
        validityLayout->setContentsMargins(2, 2, 2, 2);
        validityLayout->setSpacing(8);

        validityDot_ = new QFrame(validityRow_);
        validityDot_->setFrameShape(QFrame::NoFrame);
        // Initial style is set by refreshValidityIndicator() but we provide
        // a visible default so the row is never blank during construction.
        validityDot_->setStyleSheet(QString(
                                        "QFrame{"
                                        "background:%1;"
                                        "border-radius:6px;"
                                        "min-width:12px;max-width:12px;"
                                        "min-height:12px;max-height:12px;"
                                        "}"
                                        ).arg(EMStyle::rgb(CBStyle::RED)));
        validityLayout->addWidget(validityDot_);

        validityLabel_ = new QLabel(QStringLiteral("Empty canvas"), validityRow_);
        validityLabel_->setStyleSheet(QString(
                                          "QLabel{"
                                          "color:%1;"
                                          "background:transparent;"
                                          "font-family:'Courier New',monospace;"
                                          "font-size:10px;"
                                          "font-weight:bold;"
                                          "letter-spacing:1px;"
                                          "}"
                                          ).arg(EMStyle::rgb(CBStyle::RED)));
        validityLayout->addWidget(validityLabel_, /*stretch*/ 1);

        primaryLayout->addWidget(validityRow_);

        btnCompute_ = new QPushButton("COMPUTE", primarySection);
        btnCompute_->setMinimumHeight(38);
        btnCompute_->setStyleSheet(EMStyle::primaryButtonQSS(EMStyle::accentFor(EMStyle::AccentRole::Primary)));
        btnCompute_->setCursor(Qt::PointingHandCursor);
        primaryLayout->addWidget(btnCompute_);

        btnExport_ = makeActionButton("Export CSV", EMStyle::AccentRole::Neutral, primarySection);
        primaryLayout->addWidget(btnExport_);

        root->addWidget(primarySection);

        return panel;
    }

    // Helper: instantiate a non-primary action button with the
    // standard QSS, height, cursor, and focus policy.
    QPushButton* makeActionButton(const QString& text,
                                  EMStyle::AccentRole role,
                                  QWidget* parent)
    {
        auto* btn = new QPushButton(text, parent);
        btn->setMinimumHeight(28);
        btn->setStyleSheet(EMStyle::actionButtonQSS(EMStyle::accentFor(role)));
        btn->setCursor(Qt::PointingHandCursor);
        return btn;
    }

    // ============================================================
    //  UI construction
    //
    //  [T1.2-A] Toolbar removed. The window's central widget is now
    //  a horizontal QSplitter whose left side is the assembled left
    //  panel and whose right side is the (canvas / plot) vertical
    //  splitter — same as before.
    // ============================================================
    void buildUI() {
        auto* mainSpl = new QSplitter(Qt::Horizontal, this);

        // [T1.2-B] left side
        QWidget* leftPanel = buildLeftPanel();
        mainSpl->addWidget(leftPanel);

        // Right side: vertical split (top section / plot)
        auto* rightSpl = new QSplitter(Qt::Vertical, mainSpl);

        // [T1.4] Top section is itself a horizontal splitter:
        //   left  ~65% : circuit canvas
        //   right ~35% : read-only StackLayerPanel mirroring canvas elements
        //
        // The stack panel listens for canvas signals (circuitChanged,
        // elementSelected) and the property panel's paramsChanged signal.
        // Wiring is in connectSignals() further below.
        auto* topSpl = new QSplitter(Qt::Horizontal, rightSpl);
        canvas_ = new AssemblyCanvas(topSpl);
        topSpl->addWidget(canvas_);

        stackPanel_ = new StackLayerPanel(topSpl);
        // Floor on the stack panel so it stays usable when the user drags
        // the inner splitter; canvas takes priority for extra width.
        stackPanel_->setMinimumWidth(220);
        topSpl->addWidget(stackPanel_);
        topSpl->setSizes({650, 350});                    // 65 / 35 default
        topSpl->setStretchFactor(0, 1);                  // canvas absorbs growth
        topSpl->setStretchFactor(1, 0);                  // stack panel stays compact
        topSpl->setHandleWidth(2);
        topSpl->setCollapsible(0, false);
        topSpl->setCollapsible(1, false);

        rightSpl->addWidget(topSpl);

        m_plot = new QCustomPlot(rightSpl);
        setupPlot();
        rightSpl->addWidget(m_plot);
        rightSpl->setSizes({400, 320});

        mainSpl->addWidget(rightSpl);
        // Initial sizing: left panel ~260 px, right side fills the rest.
        // The user can drag the splitter handle to resize per the Task 1.2
        // design decision.
        mainSpl->setSizes({260, 1180});
        mainSpl->setStretchFactor(0, 0);   // left panel: don't auto-stretch
        mainSpl->setStretchFactor(1, 1);   // right side: absorbs window growth
        mainSpl->setHandleWidth(2);
        // Prevent the user from collapsing the left panel below usability.
        leftPanel->setMinimumWidth(220);
        setCentralWidget(mainSpl);

        // ── Status bar ────────────────────────────────────────────
        statusLbl_ = new QLabel(
            "Ready  —  correct order: "
            "[Source]→[Aperture]→[Cavity(p)]→[Obs.Pt]→[Cavity(d-p)]  |  "
            "Last Cavity auto-terminates to ground  |  Compute");
        statusLbl_->setStyleSheet(QString("color:rgb(%1,%2,%3);")
                                      .arg(CBStyle::TEXT_MUTED.red()).arg(CBStyle::TEXT_MUTED.green()).arg(CBStyle::TEXT_MUTED.blue()));
        statusBar()->addWidget(statusLbl_);
        statusBar()->setSizeGripEnabled(false);
    }

    // ============================================================
    //  setupPlot — identical configuration to Phase A mainwindow
    // ============================================================
    void setupPlot() {
        m_plot->plotLayout()->insertRow(0);
        auto* title = new QCPTextElement(m_plot,
                                         "Circuit Builder — Shielding Effectiveness",
                                         QFont("sans-serif", 11, QFont::Bold));
        m_plot->plotLayout()->addElement(0, 0, title);

        m_plot->xAxis->setLabel("Frequency [GHz]");
        m_plot->yAxis->setLabel("SE [dB]");
        m_plot->xAxis->setRange(0.0, 32.0);
        m_plot->yAxis->setRange(0.0, 120.0);

        m_plot->xAxis->grid()->setSubGridVisible(true);
        m_plot->yAxis->grid()->setSubGridVisible(true);
        QPen gp(QColor(200,200,200)); gp.setStyle(Qt::DashLine);
        m_plot->xAxis->grid()->setPen(gp);
        m_plot->yAxis->grid()->setPen(gp);
        QPen sg(QColor(230,230,230)); sg.setStyle(Qt::DotLine);
        m_plot->xAxis->grid()->setSubGridPen(sg);
        m_plot->yAxis->grid()->setSubGridPen(sg);

        m_plot->legend->setVisible(true);
        m_plot->legend->setFont(QFont("sans-serif",9));
        m_plot->legend->setBrush(QBrush(QColor(255,255,255,200)));
        m_plot->axisRect()->insetLayout()->setInsetAlignment(0, Qt::AlignTop|Qt::AlignRight);

        m_plot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom |
                                QCP::iSelectPlottables | QCP::iSelectLegend);

        // Click-to-read overlay (matches Phase A onPlotClicked)
        connect(m_plot, &QCustomPlot::mousePress,
                this,   &CircuitBuilderWindow::onPlotClicked);
    }

    // ============================================================
    //  connectSignals
    //
    //  [T1.2-E] Lambda captures use [this] explicitly throughout.
    // ============================================================
    void connectSignals() {
        // Action buttons
        connect(btnArrange_, &QPushButton::clicked, this,
                [this]{ canvas_->autoArrange(); });
        connect(btnDelete_,  &QPushButton::clicked, this,
                [this]{ canvas_->removeSelected(); });
        connect(btnClear_,   &QPushButton::clicked, this,
                [this]{
                    canvas_->clearAll();
                    clearPlot();
                    setStatus("Canvas cleared — drag elements to build a new circuit.",
                              CBStyle::TEXT_MUTED);
                });
        connect(btnCompute_, &QPushButton::clicked, this,
                &CircuitBuilderWindow::runCompute);
        connect(btnExport_,  &QPushButton::clicked, this,
                &CircuitBuilderWindow::onExportCSV);

        // Canvas <-> property panel
        connect(canvas_, &AssemblyCanvas::elementDropped, this,
                [this](ElementType t, QPointF pos){
                    auto* el = canvas_->addElement(t, pos);
                    propPanel_->showElement(el);
                    setStatus(
                        QString("Added: %1  |  Arrange left→right: "
                                "Source → Aperture → Cavity → Obs.Pt").arg(el->params.label),
                        CBStyle::ACCENT);
                });
        connect(canvas_, &AssemblyCanvas::elementSelected, this,
                [this](CanvasElement* el){
                    propPanel_->showElement(el);
                    // [T1.4] Mirror canvas selection on the layer stack.
                    stackPanel_->setSelected(el);
                });
        connect(canvas_, &AssemblyCanvas::circuitChanged, this,
                [this]{
                    clearPlot();
                    // [T1.4] Any structural change to the canvas (add /
                    // remove / clear / arrange) rebuilds the stack from
                    // canvas_->elements, sorted left-to-right by X.
                    stackPanel_->rebuild(canvas_->elements);
                    // [T1.5b] Re-evaluate topology and recolour the
                    // validity dot + status text. Cheap O(N) walk; fine
                    // to call on every structural change.
                    refreshValidityIndicator();
                });
        connect(propPanel_, &BuilderPropertyPanel::paramsChanged, this,
                [this](CanvasElement* el){
                    if (el) el->update();
                    // [T1.4] Live-update only the affected card; no rebuild.
                    if (el) stackPanel_->refreshElement(el);
                });

        // [T1.5b] Initial paint of the validity indicator. Without this
        // call the row would stay at its construction-time default ("Empty
        // canvas" / red) until the user does *something* to the canvas.
        // That happens to be correct for an empty startup canvas, but
        // calling here makes startup state robust if the canvas ever ships
        // with pre-populated elements (e.g. file load in a future task).
        refreshValidityIndicator();
    }

    // ============================================================
    //  [T1.5b] Circuit topology validation
    //
    //  The MNA solver assumes a strict alternating chain:
    //
    //      Source -> ( Aperture -> Cavity )+ -> Obs.Pt
    //
    //  where Aperture is either an AP_SlotAperture or an
    //  AP_SlotWithCover, and Cavity is either a TL_EmptyCavity or
    //  a TL_DielectricCavity. The chain must be sorted by X
    //  position. Any deviation produces incorrect SE values or
    //  a degenerate MNA system.
    //
    //  Validation runs in two places:
    //    1. runCompute() — fails loudly with a QMessageBox.
    //    2. refreshValidityIndicator() — drives the dot/text row
    //       above the COMPUTE button (live status).
    //
    //  Single source of truth: validateCircuitCore() returns an
    //  error code + offending index. Two thin formatters translate
    //  the code into either a brief one-line message (for the
    //  indicator) or a full multi-sentence message (for the
    //  dialog and tooltip).
    // ============================================================

    enum class ValidationCode {
        Ok,
        EmptyCanvas,            // 0 elements
        SourceMissing,          // no Source at all
        SourceMultiple,         // >1 Source
        SourceNotFirst,         // Source not at leftmost X position
        ObsMissing,             // no Obs.Pt at all
        ObsMultiple,            // >1 Obs.Pt
        ObsNotLast,             // Obs.Pt not at rightmost X position
        ApertureMissing,        // no Aperture/AP+Cover at all
        CavityMissing,          // no Cavity/Diel.Cav at all
        UnbalancedSections,     // |#apertures - #cavities| != 0
        PatternViolation        // alternation broken at some position
    };

    struct ValidationResult {
        ValidationCode code   {ValidationCode::Ok};
        int            offset {-1};   // offset into the X-sorted sequence,
        // or -1 if not relevant for this code
    };

    // Helper predicates — kept inline + static so any member can use them.
    static bool isSource   (ElementType t) { return t == ElementType::Source; }
    static bool isObs      (ElementType t) { return t == ElementType::Load; }
    static bool isAperture (ElementType t) { return t == ElementType::Aperture
                                                   || t == ElementType::ApertureWithCover; }
    static bool isCavity   (ElementType t) { return t == ElementType::EmptyCavity
                                                 || t == ElementType::DielectricCavity; }

    // ─── Core validator ────────────────────────────────────────
    //
    // Steps, in priority order:
    //   1. empty canvas
    //   2. count of Sources (must == 1) and Obs.Pts (must == 1)
    //   3. presence of >=1 aperture, >=1 cavity
    //   4. balance #apertures == #cavities
    //   5. Source is leftmost, Obs.Pt is rightmost
    //   6. middle of the chain alternates Aperture, Cavity, Aperture,
    //      Cavity, ..., ending on Cavity just before the Obs.Pt.
    //
    // Reads from canvas_->elements; sorts a local copy. Pure function
    // w.r.t. the canvas (does not mutate state).
    ValidationResult validateCircuitCore() const {
        if (!canvas_ || canvas_->elements.isEmpty()) {
            return {ValidationCode::EmptyCanvas, -1};
        }

        // Sort by X position — same comparator as runCompute() and
        // StackLayerPanel::rebuild() — so all three views agree on order.
        QVector<CanvasElement*> ordered = canvas_->elements;
        std::sort(ordered.begin(), ordered.end(),
                  [](CanvasElement* a, CanvasElement* b){
                      return a->pos().x() < b->pos().x();
                  });

        const int n = ordered.size();

        // Count types
        int nSrc = 0, nObs = 0, nAp = 0, nCav = 0;
        for (auto* el : ordered) {
            const ElementType t = el->params.type;
            if (isSource(t))   ++nSrc;
            if (isObs(t))      ++nObs;
            if (isAperture(t)) ++nAp;
            if (isCavity(t))   ++nCav;
        }

        if (nSrc == 0)                     return {ValidationCode::SourceMissing,    -1};
        // [T1.5b-FIX] Order of "missing" checks matches the user's natural
        // left-to-right build order: Source → Aperture → Cavity → Obs.Pt.
        // Originally Obs.Pt-missing was checked before Aperture/Cavity-missing,
        // which made the indicator say "Obs.Pt missing" even when the user had
        // only just started by placing a Source. The user couldn't see the
        // intermediate hints ("Aperture missing", "Cavity missing") because
        // the early return skipped them. Reordering yields a natural
        // breadcrumb trail: each new dropped element advances the indicator
        // to the next missing piece.
        if (nAp  == 0)                     return {ValidationCode::ApertureMissing,  -1};
        if (nCav == 0)                     return {ValidationCode::CavityMissing,    -1};
        if (nObs == 0)                     return {ValidationCode::ObsMissing,       -1};
        // "Multiple of X" checks come AFTER all the "missing" checks because
        // having extras is only meaningful once the minimum set is present.
        // A user with two Sources but no Aperture has a more fundamental
        // problem ("Aperture missing") that we report first.
        if (nSrc >  1)                     return {ValidationCode::SourceMultiple,   -1};
        if (nObs >  1)                     return {ValidationCode::ObsMultiple,      -1};
        if (nAp != nCav)                   return {ValidationCode::UnbalancedSections, -1};

        // Position checks
        if (!isSource(ordered.front()->params.type))
            return {ValidationCode::SourceNotFirst, 0};
        if (!isObs(ordered.back()->params.type))
            return {ValidationCode::ObsNotLast, n - 1};

        // Pattern check on the middle: indices 1..n-2 must be
        //   Aperture, Cavity, Aperture, Cavity, ...
        // i.e. element at offset (1 + 2k) is Aperture, at (1 + 2k + 1) is Cavity.
        for (int i = 1; i < n - 1; ++i) {
            const ElementType t = ordered[i]->params.type;
            const int k = i - 1;          // 0-based middle index
            const bool wantAperture = (k % 2 == 0);
            const bool ok = wantAperture ? isAperture(t) : isCavity(t);
            if (!ok) {
                return {ValidationCode::PatternViolation, i};
            }
        }

        return {ValidationCode::Ok, -1};
    }

    // ─── Brief message — for the status row dot label ──────────
    static QString validationBrief(ValidationCode c) {
        switch (c) {
        case ValidationCode::Ok:                  return QStringLiteral("Circuit valid");
        case ValidationCode::EmptyCanvas:         return QStringLiteral("Empty canvas");
        case ValidationCode::SourceMissing:       return QStringLiteral("Source missing");
        case ValidationCode::SourceMultiple:      return QStringLiteral("Multiple Sources");
        case ValidationCode::SourceNotFirst:      return QStringLiteral("Source must be first");
        case ValidationCode::ObsMissing:          return QStringLiteral("Obs.Pt missing");
        case ValidationCode::ObsMultiple:         return QStringLiteral("Multiple Obs.Pts");
        case ValidationCode::ObsNotLast:          return QStringLiteral("Obs.Pt must be last");
        case ValidationCode::ApertureMissing:     return QStringLiteral("Aperture missing");
        case ValidationCode::CavityMissing:       return QStringLiteral("Cavity missing");
        case ValidationCode::UnbalancedSections:  return QStringLiteral("Unbalanced sections");
        case ValidationCode::PatternViolation:    return QStringLiteral("Invalid order");
        }
        return QString();
    }

    // ─── Full message — for the QMessageBox and tooltip ────────
    QString validationFull(ValidationResult r) const {
        switch (r.code) {
        case ValidationCode::Ok:
            return QStringLiteral("Circuit topology is valid.");
        case ValidationCode::EmptyCanvas:
            return QStringLiteral(
                "The canvas is empty.\n\n"
                "Drop elements onto the canvas to build a circuit. "
                "Minimum legal circuit is:\n"
                "    Source -> Aperture -> Cavity -> Obs.Pt");
        case ValidationCode::SourceMissing:
            return QStringLiteral(
                "No Source element found.\n\n"
                "Every circuit needs exactly one Source to provide the "
                "excitation voltage V0. Drag a Source element onto the canvas.");
        case ValidationCode::SourceMultiple:
            return QStringLiteral(
                "Multiple Source elements found.\n\n"
                "Only one Source is allowed per circuit. Delete the extras "
                "so a single excitation V0 drives the chain.");
        case ValidationCode::SourceNotFirst:
            return QStringLiteral(
                "Source is not the leftmost element.\n\n"
                "The Source must be placed at the leftmost X position because "
                "the equivalent circuit is read left-to-right starting from V0. "
                "Move it to the left of all other elements, or click Arrange.");
        case ValidationCode::ObsMissing:
            return QStringLiteral(
                "No Obs.Pt element found.\n\n"
                "Every circuit needs exactly one Obs.Pt where the shielding "
                "effectiveness SE = -20 log10|2 U2 / V0| is measured. "
                "Drag an Obs.Pt element onto the canvas.");
        case ValidationCode::ObsMultiple:
            return QStringLiteral(
                "Multiple Obs.Pt elements found.\n\n"
                "Only one Obs.Pt is allowed per circuit. Delete the extras.");
        case ValidationCode::ObsNotLast:
            return QStringLiteral(
                "Obs.Pt is not the rightmost element.\n\n"
                "The Obs.Pt must be placed at the rightmost X position so the "
                "back-wall short-circuit termination can be appended after it. "
                "Move it to the right of all other elements, or click Arrange.");
        case ValidationCode::ApertureMissing:
            return QStringLiteral(
                "No Aperture element found.\n\n"
                "The circuit needs at least one Aperture (or AP+Cover) - the "
                "coupling element from the external field through the front "
                "wall into the cavity.");
        case ValidationCode::CavityMissing:
            return QStringLiteral(
                "No Cavity element found.\n\n"
                "The circuit needs at least one Cavity (or Diel.Cav) section "
                "behind the aperture to define the waveguide region of the "
                "enclosure interior.");
        case ValidationCode::UnbalancedSections:
            return QStringLiteral(
                "Each Aperture must be paired with a Cavity.\n\n"
                "The strict-alternation rule requires the same number of "
                "Apertures and Cavities (one per section). Currently they "
                "do not match - add or remove elements until the counts "
                "are equal.");
        case ValidationCode::PatternViolation:
            return QStringLiteral(
                "Element order is invalid.\n\n"
                "After the Source, the chain must alternate "
                "Aperture -> Cavity -> Aperture -> Cavity -> ... and end on a "
                "Cavity just before the Obs.Pt. Reorder the elements (drag, "
                "or click Arrange) so the pattern is followed.");
        }
        return QString();
    }

    // ─── Indicator update — called from connectSignals lambdas ─
    //
    // Re-evaluates topology and updates the dot colour, the brief text
    // beside the dot, and the tooltip (full message) on the row.
    // Cheap; safe to call on every circuitChanged.
    void refreshValidityIndicator() {
        if (!validityDot_ || !validityLabel_) return;
        const ValidationResult r = validateCircuitCore();
        const bool ok = (r.code == ValidationCode::Ok);
        const QColor dotColor = ok ? CBStyle::GREEN : CBStyle::RED;

        // Dot is a tiny QFrame with rounded-corner background — colour swap
        // via stylesheet is cheaper than QPainter and lets QSS govern radius.
        validityDot_->setStyleSheet(QString(
                                        "QFrame{"
                                        "background:%1;"
                                        "border-radius:6px;"
                                        "min-width:12px;max-width:12px;"
                                        "min-height:12px;max-height:12px;"
                                        "}"
                                        ).arg(EMStyle::rgb(dotColor)));

        const QString brief = validationBrief(r.code);
        validityLabel_->setText(brief);
        validityLabel_->setStyleSheet(QString(
                                          "QLabel{"
                                          "color:%1;"
                                          "background:transparent;"
                                          "font-family:'Courier New',monospace;"
                                          "font-size:10px;"
                                          "font-weight:bold;"
                                          "letter-spacing:1px;"
                                          "}"
                                          ).arg(EMStyle::rgb(dotColor)));

        // Tooltip on the whole row carries the long-form explanation so the
        // user can find the "why" without clicking anything.
        const QString full = validationFull(r);
        if (validityRow_) validityRow_->setToolTip(full);
    }

    // ============================================================
    //  runCompute
    //
    //  Circuit model (left-to-right X position defines order):
    //
    //  1-section example:
    //    Source(0→1, series)
    //    Aperture(1→2, series)
    //    Cavity(2→3, series TL)
    //    Obs.Pt(3→0, shunt) ← records node 3 → SE curve P1
    //
    //  2-section example:
    //    Source(0→1) → Ap1(1→2) → Cav1(2→3) → Obs(3→0, P1)
    //    → Ap2(3→4) → Cav2(4→5) → Obs(5→0, P2)
    //
    //  Key rule [D1]: after a Load (shunt), nodeIdx is NOT
    //  incremented. The circuit continues from the same node.
    // ============================================================
    void runCompute()
    {
        // [T1.5b] Single-source validation gate. Must run before any
        // sorting, MNA assembly, or sweep — a malformed circuit yields
        // either a degenerate matrix (Eigen exception) or physically
        // meaningless SE numbers, both of which are unacceptable.
        const ValidationResult vr = validateCircuitCore();
        if (vr.code != ValidationCode::Ok) {
            const QString brief = validationBrief(vr.code);
            const QString full  = validationFull(vr);
            // [T1.6] Custom branded dialog replaces the default QMessageBox.
            // The full message body carries multi-line guidance from the
            // Task 1.5b validation message bank (validationFull()).
            MessageDialog::error(this,
                                 QStringLiteral("Circuit topology error"),
                                 full);
            setStatus(QString("Cannot compute — %1.").arg(brief), CBStyle::RED);
            return;
        }

        auto& els = canvas_->elements;

        // Sort elements by X position to define circuit order. Validation
        // above already confirmed: Source at front, Obs.Pt at back, the
        // middle alternates Aperture/Cavity, equal counts on both sides.
        QVector<CanvasElement*> ordered = els;
        std::sort(ordered.begin(), ordered.end(),
                  [](CanvasElement* a, CanvasElement* b){ return a->pos().x() < b->pos().x(); });

        // Advisory: if the circuit ends with a Cavity (not an Obs.Pt), warn the user
        // that the right-wall short-circuit termination will be added automatically.
        const bool endsWithCavity = (ordered.last()->params.type == ElementType::EmptyCavity ||
                                     ordered.last()->params.type == ElementType::DielectricCavity);
        (void)endsWithCavity;  // used only for status message below

        // Extract sweep parameters from the Source element
        const ElementParams& sp = ordered.first()->params;
        const double a_g  = sp.a;
        const double b_g  = sp.b_h;
        const double t_g  = sp.t_wall;
        const double fMin = sp.freqStart * 1e9;
        const double fMax = sp.freqEnd   * 1e9;
        const int    Np   = sp.freqPoints;
        const double V0   = (sp.E0 > 0.0) ? sp.E0 : 1.0;

        // ── Build MNA topology ONCE (outside frequency loop) [D3] ──
        MNASolver solver;
        int nodeIdx  = 0;
        int branchId = 0;
        QVector<int>     obsNodes;   // 1-based node indices at each Obs.Pt
        QVector<QString> obsLabels;
        int obsCount = 0;

        for(auto* el : ordered)
        {
            const ElementParams& ep = el->params;
            const int nFrom = nodeIdx;
            const int bid   = branchId++;

            switch(ep.type)
            {
            // Series source: 0 → 1
            // Z_0 exact = 376.730 Ω from PhysicsConstants.h [D4]
            case ElementType::Source:
                solver.addBranch(std::make_shared<SRC_VoltageSource>(
                    nFrom, nodeIdx+1, bid,
                    Complex(ep.E0, 0.0), Z_0));
                ++nodeIdx;
                break;

            // ── Slot Aperture — SHUNT branch (Fig. 3.7 Branch II) ──────
            case ElementType::Aperture:
                solver.addBranch(std::make_shared<AP_SlotAperture>(
                    nFrom, 0, bid,
                    a_g, b_g, ep.l_slot, ep.w_slot, t_g));
                break;

            // ── Aperture With Cover — also SHUNT ────────────────────────
            case ElementType::ApertureWithCover:
                solver.addBranch(std::make_shared<AP_SlotWithCover>(
                    nFrom, 0, bid,
                    a_g, b_g, ep.l_slot, ep.w_slot, t_g, ep.tau_cover));
                break;

            // Series cavity (air-filled TL)
            case ElementType::EmptyCavity:
                solver.addBranch(std::make_shared<TL_EmptyCavity>(
                    nFrom, nodeIdx+1, bid,
                    a_g, b_g, ep.L_cavity));
                ++nodeIdx;
                break;

            // Series cavity (dielectric-loaded TL)
            case ElementType::DielectricCavity:
            {
                double h_safe = std::clamp(ep.h_dielectric, 1e-6, b_g*0.999);
                solver.addBranch(std::make_shared<TL_DielectricCavity>(
                    nFrom, nodeIdx+1, bid,
                    a_g, b_g, ep.L_cavity, h_safe, ep.eps_r));
                ++nodeIdx;
                break;
            }

            // SHUNT Obs.Point: nFrom → 0 [D1]
            case ElementType::Load:
                solver.addBranch(std::make_shared<LOAD_Impedance>(
                    nFrom, 0, bid,
                    Complex(ep.ZL_real, ep.ZL_imag)));
                ++obsCount;
                obsNodes.append(nFrom);
                obsLabels.append(ep.label.isEmpty()
                                     ? QString("P%1").arg(obsCount)
                                     : ep.label);
                break;
            }
        }

        if(obsNodes.isEmpty()){
            setStatus("No observation nodes built (internal error).",
                      CBStyle::RED);
            return;
        }

        // ── Auto short-circuit termination ────────────────────────────────────────
        {
            auto lastSeries = std::find_if(
                ordered.rbegin(), ordered.rend(),
                [](CanvasElement* e){ return e->params.type != ElementType::Load; });

            const bool isCavityTerminated =
                (lastSeries != ordered.rend()) &&
                ((*lastSeries)->params.type == ElementType::EmptyCavity ||
                 (*lastSeries)->params.type == ElementType::DielectricCavity);

            const bool obsAtFinalNode = obsNodes.contains(nodeIdx);

            if(isCavityTerminated && !obsAtFinalNode) {
                solver.addBranch(std::make_shared<LOAD_Impedance>(
                    nodeIdx, 0, branchId++,
                    Complex(1e-4, 0.0)));
            }
        }

        // ── Frequency sweep — solver.solve(f) only [D3] ────────────
        QVector<double>          freqsGHz;
        QVector<QVector<double>> SE_curves(obsNodes.size());
        freqsGHz.reserve(Np);
        for(auto& c : SE_curves) c.reserve(Np);

        for(int i = 0; i < Np; ++i)
        {
            const double f = fMin + (fMax - fMin) *
                                        (Np > 1 ? double(i)/double(Np-1) : 0.0);
            freqsGHz.append(f / 1e9);

            try {
                const Eigen::VectorXcd V = solver.solve(f);

                for(int oi = 0; oi < obsNodes.size(); ++oi){
                    const int vidx = obsNodes[oi] - 1;   // 0-based
                    if(vidx < 0 || vidx >= static_cast<int>(V.size())){
                        SE_curves[oi].append(std::numeric_limits<double>::quiet_NaN());
                        continue;
                    }
                    // [D2] Eq.(3.8): SE = -20·log₁₀|2·U_obs / V₀|
                    const double U2  = std::abs(V(vidx));
                    const double arg = (U2 > 1e-30) ? (2.0*U2/V0) : 1e-30;
                    SE_curves[oi].append(-20.0 * std::log10(arg));
                }
            }
            catch(const std::exception& ex){
                for(auto& c : SE_curves)
                    c.append(std::numeric_limits<double>::quiet_NaN());
                qDebug() << "MNA exception at f=" << f/1e9 << "GHz:" << ex.what();
            }
        }

        // Store for export and interactive readout
        m_freqs   = freqsGHz;
        m_SE_data = SE_curves;
        m_labels  = obsLabels;

        plotResults();

        // Build status line identical in format to Phase A
        QVector<double> valid;
        for(const auto& c : SE_curves)
            for(double v : c) if(std::isfinite(v)) valid.append(v);
        const double seMin = valid.isEmpty() ? 0.0 : *std::min_element(valid.begin(),valid.end());
        const double seMax = valid.isEmpty() ? 0.0 : *std::max_element(valid.begin(),valid.end());
        setStatus(
            QString("OK  %1 pts · %2 curve(s) · SE: %3…%4 dB · %5–%6 GHz")
                .arg(Np).arg(obsNodes.size())
                .arg(seMin,0,'f',1).arg(seMax,0,'f',1)
                .arg(sp.freqStart,0,'f',1).arg(sp.freqEnd,0,'f',1),
            CBStyle::GREEN);
    }

    // ============================================================
    //  plotResults — QCustomPlot rendering matching Phase A exactly
    // ============================================================
    void plotResults()
    {
        clearPlot();

        double ymin =  std::numeric_limits<double>::infinity();
        double ymax = -std::numeric_limits<double>::infinity();

        for(int c = 0; c < m_SE_data.size(); ++c){
            QCPGraph* graph = m_plot->addGraph();
            graph->setData(m_freqs, m_SE_data[c]);
            graph->setName(m_labels.value(c, QString("P%1").arg(c+1)));
            QPen pen(palette()[c % palette().size()]);
            pen.setWidth(2); graph->setPen(pen);
            graph->setSelectable(QCP::stWhole);
            for(double v : m_SE_data[c])
                if(std::isfinite(v)){ ymin=std::min(ymin,v); ymax=std::max(ymax,v); }
        }

        // Auto-range
        if(!m_freqs.isEmpty())
            m_plot->xAxis->setRange(m_freqs.first(), m_freqs.last());
        if(std::isfinite(ymin) && std::isfinite(ymax)){
            const double yr = std::max(ymax-ymin, 1.0);
            m_plot->yAxis->setRange(ymin-0.1*yr, ymax+0.1*yr);
        }

        // 0 dB reference line
        auto* zl = new QCPItemStraightLine(m_plot);
        zl->point1->setCoords(0,0); zl->point2->setCoords(1,0);
        QPen zp(QColor(128,128,128,150)); zp.setStyle(Qt::DashLine); zl->setPen(zp);

        setupInteractiveItems();
        m_plot->replot();
    }

    void clearPlot() {
        m_plot->clearPlottables();
        m_plot->clearItems();
        m_crosshairLine = nullptr;
        m_readoutLabel  = nullptr;
        m_tracers.clear();
        m_plot->replot();
    }

    // ============================================================
    //  setupInteractiveItems — crosshair + tracers + readout label
    //  Mirrors Phase A MainWindow::setupPlotInteractiveItems()
    // ============================================================
    void setupInteractiveItems()
    {
        // Vertical crosshair
        m_crosshairLine = new QCPItemLine(m_plot);
        QPen cp(QColor(80,80,80,200)); cp.setStyle(Qt::DashLine); cp.setWidth(1);
        m_crosshairLine->setPen(cp);
        m_crosshairLine->start->setType(QCPItemPosition::ptPlotCoords);
        m_crosshairLine->end->setType(QCPItemPosition::ptPlotCoords);
        m_crosshairLine->setVisible(false);

        // Readout text box
        m_readoutLabel = new QCPItemText(m_plot);
        m_readoutLabel->setPositionAlignment(Qt::AlignLeft|Qt::AlignTop);
        m_readoutLabel->position->setType(QCPItemPosition::ptPlotCoords);
        m_readoutLabel->setFont(QFont("Courier New",9));
        m_readoutLabel->setColor(Qt::black);
        m_readoutLabel->setPadding(QMargins(6,4,6,4));
        m_readoutLabel->setBrush(QBrush(QColor(255,255,240,230)));
        QPen bp(QColor(100,100,100)); bp.setWidth(1); m_readoutLabel->setPen(bp);
        m_readoutLabel->setVisible(false);

        // Per-curve tracers (snap dots)
        m_tracers.clear();
        for(int c = 0; c < m_plot->graphCount(); ++c){
            auto* tr = new QCPItemTracer(m_plot);
            tr->setGraph(m_plot->graph(c));
            tr->setStyle(QCPItemTracer::tsCircle); tr->setSize(8);
            QPen tp(palette()[c % palette().size()]); tp.setWidth(2);
            tr->setPen(tp);
            tr->setBrush(QBrush(palette()[c % palette().size()]));
            tr->setInterpolating(true); tr->setVisible(false);
            m_tracers.append(tr);
        }
    }

    // ============================================================
    //  onPlotClicked — interactive SE readout
    //  Logic mirrors Phase A MainWindow::onPlotClicked() exactly.
    // ============================================================
    void onPlotClicked(QMouseEvent* event)
    {
        if(m_freqs.isEmpty() || m_SE_data.isEmpty()) return;
        if(event->button() != Qt::LeftButton) return;

        const double clickedGHz = m_plot->xAxis->pixelToCoord(event->pos().x());

        // Find nearest frequency index
        int idx=0; double minDist=std::abs(m_freqs[0]-clickedGHz);
        for(int i=1; i<m_freqs.size(); ++i){
            double d=std::abs(m_freqs[i]-clickedGHz);
            if(d<minDist){ minDist=d; idx=i; }
        }
        const double f_GHz = m_freqs[idx];

        // Crosshair
        if(m_crosshairLine){
            const double yLo = m_plot->yAxis->range().lower - 1.0;
            const double yHi = m_plot->yAxis->range().upper + 1.0;
            m_crosshairLine->start->setCoords(f_GHz, yLo);
            m_crosshairLine->end->setCoords(f_GHz, yHi);
            m_crosshairLine->setVisible(true);
        }

        // Tracers
        for(auto* tr : m_tracers){ tr->setGraphKey(f_GHz); tr->setVisible(true); }

        // Readout text — one line per curve
        QString text = QString("f = %1 GHz\n").arg(f_GHz, 0, 'f', 4);
        for(int c=0; c<m_SE_data.size(); ++c){
            const double se  = m_SE_data[c].value(idx, std::numeric_limits<double>::quiet_NaN());
            const QString lb = m_labels.value(c, QString("P%1").arg(c+1));
            text += std::isfinite(se)
                        ? QString("%1 : %2 dB\n").arg(lb,-10).arg(se,8,'f',2)
                        : QString("%1 :   ∞ dB\n").arg(lb,-10);
        }
        text = text.trimmed();

        if(m_readoutLabel){
            const double xRange  = m_plot->xAxis->range().size();
            const double xOff    = m_plot->xAxis->range().lower;
            const double yTop    = m_plot->yAxis->range().upper;
            const double yRange  = m_plot->yAxis->range().size();
            const bool nearRight = (clickedGHz - xOff) > 0.60 * xRange;
            m_readoutLabel->setPositionAlignment(
                (nearRight ? Qt::AlignRight : Qt::AlignLeft) | Qt::AlignTop);
            m_readoutLabel->position->setCoords(f_GHz, yTop - 0.04*yRange);
            m_readoutLabel->setText(text);
            m_readoutLabel->setVisible(true);
        }
        m_plot->replot();
    }

    // ============================================================
    //  onExportCSV — same format as Phase A mainwindow
    //
    //  [T1.6] Three user-facing dialogs:
    //    1. No-data error  : if the user clicks Export before clicking
    //                        Compute, there is nothing to write.
    //    2. File-write err : the chosen path could not be opened.
    //    3. Success        : confirms the file path so the user can
    //                        find the saved CSV.
    // ============================================================
    void onExportCSV()
    {
        if(m_freqs.isEmpty()){
            MessageDialog::error(
                this,
                QStringLiteral("No data to export"),
                QStringLiteral(
                    "There is no computed SE data to export.\n\n"
                    "Click COMPUTE first to populate the plot, then "
                    "use Export CSV to save the results."));
            return;
        }
        const QString fn = QFileDialog::getSaveFileName(
            this, "Export CSV", "SE_builder.csv", "CSV (*.csv)");
        if(fn.isEmpty()) return;

        std::ofstream f(fn.toStdString());
        if(!f.is_open()){
            MessageDialog::error(
                this,
                QStringLiteral("Cannot write file"),
                QString("The selected file could not be opened for writing:\n\n"
                        "%1\n\n"
                        "Check that the destination folder exists and that "
                        "the file is not currently open in another program.").arg(fn));
            return;
        }
        f << "f_GHz";
        for(const auto& l : m_labels) f << ",SE_" << l.toStdString() << "_dB";
        f << "\n" << std::fixed;
        for(int i=0; i<m_freqs.size(); ++i){
            f << std::setprecision(9) << m_freqs[i];
            for(const auto& c : m_SE_data)
                f << "," << std::setprecision(6) << (i<c.size() ? c[i] : 0.0);
            f << "\n";
        }
        f.close();
        setStatus("Exported: " + fn, CBStyle::GREEN);

        // [T1.6] Success popup confirming the saved file path. The status
        // bar already shows "Exported: <path>" in green; the dialog gives
        // a stronger acknowledgement and presents the path in copy-able
        // form for users who want to navigate to the file.
        MessageDialog::success(
            this,
            QStringLiteral("Export complete"),
            QString("CSV saved successfully to:\n\n%1").arg(fn));
    }

    void setStatus(const QString& msg, QColor col=CBStyle::TEXT_MUTED){
        statusLbl_->setText(msg);
        statusLbl_->setStyleSheet(QString("color:rgb(%1,%2,%3);")
                                      .arg(col.red()).arg(col.green()).arg(col.blue()));
    }
};
