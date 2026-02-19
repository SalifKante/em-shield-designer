#pragma once

// ============================================================
//  CircuitBuilderWindow.h
//  Qt/C++ — EMShieldDesigner  (Phase B)
//
//  Integrates with:
//    include/core/MNASolver.h
//    include/core/AP_SlotAperture.h
//    include/core/AP_SlotWithCover.h
//    include/core/TL_EmptyCavity.h
//    include/core/TL_DielectricCavity.h
//    include/core/SRC_VoltageSource.h
//    include/core/LOAD_Impedance.h
//    include/core/PhysicsConstants.h
//    include/core/CircuitGenerator.h
//    SectionItem.h
//    PropertyPanel.h
// ============================================================

#include <QMainWindow>
#include <QWidget>
#include <QToolBar>
#include <QDockWidget>
#include <QGraphicsView>
#include <QGraphicsScene>
#include <QGraphicsItem>
#include <QGraphicsRectItem>
#include <QGraphicsEllipseItem>
#include <QGraphicsLineItem>
#include <QGraphicsPathItem>
#include <QGraphicsTextItem>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QFormLayout>
#include <QScrollArea>
#include <QLabel>
#include <QPushButton>
#include <QDoubleSpinBox>
#include <QSpinBox>
#include <QLineEdit>
#include <QComboBox>
#include <QGroupBox>
#include <QStatusBar>
#include <QSplitter>
#include <QPainter>
#include <QPainterPath>
#include <QPen>
#include <QBrush>
#include <QColor>
#include <QFont>
#include <QDrag>
#include <QMimeData>
#include <QMouseEvent>
#include <QDropEvent>
#include <QDragEnterEvent>
#include <QDragMoveEvent>
#include <QMessageBox>
#include <QFileDialog>
#include <QAction>
#include <QTimer>
#include <QPropertyAnimation>
#include <QGraphicsDropShadowEffect>
#include <QLinearGradient>
#include <QRadialGradient>
#include <QVector>
#include <QMap>
#include <QString>
#include <QStringList>
#include <QDebug>
#include <complex>
#include <cmath>
#include <vector>
#include <memory>
#include <algorithm>
#include <Eigen/Dense>

// Pull in your physics engine
#include "include/core/PhysicsConstants.h"
#include "include/core/MNASolver.h"
#include "include/core/AP_SlotAperture.h"
#include "include/core/AP_SlotWithCover.h"
#include "include/core/TL_EmptyCavity.h"
#include "include/core/TL_DielectricCavity.h"
#include "include/core/SRC_VoltageSource.h"
#include "include/core/LOAD_Impedance.h"
#include "include/core/CircuitGenerator.h"

// All physics classes live in the EMCore namespace
using namespace EMCore;

// ============================================================
// Design tokens  (SPICE-inspired light theme)
// ============================================================
namespace CBStyle {
// Backgrounds — based on #f6fbf9 (mint-white), layered surfaces
inline const QColor BG        { 246, 251, 249 };   // #f6fbf9  canvas / window base
inline const QColor SURFACE   { 235, 244, 240 };   // #ebf4f0  panels, toolbar
inline const QColor SURFACE2  { 220, 235, 229 };   // #dceee5  hover states

// Borders — soft grey-green, like LTspice grid lines
inline const QColor BORDER    { 180, 205, 195 };   // #b4cdc3
inline const QColor BORDER_LT { 210, 228, 222 };   // #d2e4de  dot-grid dots

// Accent colours — SPICE-inspired: bold on light background
inline const QColor ACCENT    {  14, 100, 200 };   // #0e64c8  blue   — aperture
inline const QColor GREEN     {  22, 130,  64 };   // #168240  green  — cavity
inline const QColor ORANGE    { 180,  90,   0 };   // #b45a00  amber  — source
inline const QColor RED       { 195,  30,  30 };   // #c31e1e  red    — load

// Text — dark on light background
inline const QColor TEXT      {  24,  36,  32 };   // #182420  near-black
inline const QColor TEXT_MUTED{  90, 115, 105 };   // #5a7369
inline const QColor TEXT_DIM  { 150, 175, 165 };   // #96afa5

// Helpers
static QString css(const QColor& fg, const QColor& bg=BG,
                   const QColor& border=BORDER, int radius=6)
{
    return QString(
               "color: rgb(%1,%2,%3);"
               "background: rgb(%4,%5,%6);"
               "border: 1px solid rgb(%7,%8,%9);"
               "border-radius: %10px;"
               ).arg(fg.red()).arg(fg.green()).arg(fg.blue())
        .arg(bg.red()).arg(bg.green()).arg(bg.blue())
        .arg(border.red()).arg(border.green()).arg(border.blue())
        .arg(radius);
}
}

// ============================================================
// ElementType — maps palette items to MNA branch templates
// ============================================================
enum class ElementType {
    Source,       // SRC_VoltageSource   — leftmost, exactly one
    Aperture,     // AP_SlotAperture     — shielding wall with slot
    ApertureWithCover, // AP_SlotWithCover — wall with removable cover
    EmptyCavity,  // TL_EmptyCavity      — air-filled rectangular cavity
    DielectricCavity, // TL_DielectricCavity — dielectric-filled cavity
    Load          // LOAD_Impedance      — rightmost, exactly one
};

// ============================================================
// ElementParams — parameter bundle (one per canvas element)
// ============================================================
struct ElementParams {
    ElementType type { ElementType::EmptyCavity };
    QString     label { "Element" };

    // ── Source ────────────────────────────────────────────
    double E0          { 1.0 };         // excitation field [V/m]
    double freqStart   { 1.0 };         // [GHz]
    double freqEnd     { 32.0 };        // [GHz]
    int    freqPoints  { 300 };

    // ── Global cross-section (shared by all elements) ─────
    //   (set in Property Panel when Source is selected;
    //    propagated automatically to MNA solver)
    double a           { 0.050 };       // waveguide width  [m]
    double b_h         { 0.030 };       // waveguide height [m]
    double t_wall      { 0.001 };       // wall thickness   [m]

    // ── Aperture (slot) ───────────────────────────────────
    double l_slot      { 0.040 };       // slot length [m]
    double w_slot      { 0.002 };       // slot width  [m]

    // ── Aperture with cover ───────────────────────────────
    double tau_cover   { 0.0005 };      // cover gap [m]

    // ── Cavity ───────────────────────────────────────────
    double L_cavity      { 0.100 };     // cavity depth [m]

    // ── DielectricCavity only ─────────────────────────
    // TL_DielectricCavity(node_from, node_to, branch_id, a, b, L, h, eps_r)
    double eps_r         { 2.0  };      // relative permittivity of dielectric
    double h_dielectric  { 0.010 };     // dielectric layer thickness [m] (h ≤ b)

    // ── Load ─────────────────────────────────────────────
    double ZL_real     { 377.0 };       // [Ohm]
    double ZL_imag     { 0.0   };       // [Ohm]
};

// ============================================================
//  3‑D icon drawing helpers
//  Every element is drawn purely with QPainter — no images.
// ============================================================
namespace ElementIcon {

// Draw isometric-ish aperture wall (H-shape cross-section with slot)
static void drawAperture(QPainter* p, QRectF r, bool selected=false)
{
    p->save();
    QColor accent = selected ? CBStyle::ACCENT : QColor(14, 100, 200);
    QColor topFace  = selected ? QColor(190, 215, 245) : QColor(210, 228, 248);
    QColor frontFace= selected ? QColor(155, 195, 240) : QColor(180, 210, 240);
    QColor sideFace = selected ? QColor(120, 165, 220) : QColor(145, 185, 225);
    QPen pen(selected ? CBStyle::ACCENT : CBStyle::BORDER, selected?1.5:0.8);
    p->setRenderHint(QPainter::Antialiasing);

    double w=r.width(), h=r.height();
    double ox=r.x(), oy=r.y();
    double depth=w*0.12;

    // Front face
    QRectF front(ox, oy+depth, w-depth, h-depth);
    p->setPen(pen); p->setBrush(frontFace);
    p->drawRect(front);

    // Top face
    QPolygonF top;
    top << QPointF(ox, oy+depth) << QPointF(ox+depth, oy)
        << QPointF(ox+w, oy)     << QPointF(ox+w-depth, oy+depth);
    p->setBrush(topFace); p->drawPolygon(top);

    // Right side face
    QPolygonF right;
    right << QPointF(ox+w-depth, oy+depth) << QPointF(ox+w, oy)
          << QPointF(ox+w, oy+h-depth)     << QPointF(ox+w-depth, oy+h);
    p->setBrush(sideFace); p->drawPolygon(right);

    // Slot cut-out (white gap)
    double slotX = front.left()  + front.width()*0.35;
    double slotW = front.width() * 0.30;
    double slotY = front.top()   + front.height()*0.25;
    double slotH = front.height()* 0.50;
    p->setPen(Qt::NoPen); p->setBrush(CBStyle::BG);
    p->drawRect(QRectF(slotX, slotY, slotW, slotH));

    // Slot outline
    p->setPen(QPen(selected ? CBStyle::ACCENT : CBStyle::ORANGE, 1.4));
    p->setBrush(Qt::NoBrush);
    p->drawRect(QRectF(slotX, slotY, slotW, slotH));

    // "H" label
    p->setPen(QPen(CBStyle::ACCENT, 1));
    QFont f("Courier",7); f.setBold(true); p->setFont(f);
    p->drawText(QPointF(ox+3, oy+depth+front.height()*0.6), "H");
    p->restore();
}

// Draw 3-D cavity box (rectangular waveguide section)
static void drawCavity(QPainter* p, QRectF r, bool selected=false, const QString& lbl="CAV")
{
    p->save();
    QColor topC  = selected ? QColor(185, 230, 200) : QColor(205, 238, 215);
    QColor frontC= selected ? QColor(155, 210, 175) : QColor(175, 220, 190);
    QColor sideC = selected ? QColor(125, 190, 150) : QColor(145, 200, 165);
    QPen pen(selected ? CBStyle::GREEN : CBStyle::BORDER, selected?1.5:0.8);
    p->setRenderHint(QPainter::Antialiasing);

    double w=r.width(), h=r.height();
    double ox=r.x(), oy=r.y();
    double dx=w*0.18, dy=h*0.22;

    // Top face
    QPolygonF top;
    top << QPointF(ox,    oy+dy) << QPointF(ox+dx,  oy)
        << QPointF(ox+w,  oy)    << QPointF(ox+w-dx, oy+dy);
    p->setPen(pen); p->setBrush(topC); p->drawPolygon(top);

    // Front face
    QRectF front(ox, oy+dy, w-dx, h-dy);
    p->setBrush(frontC); p->drawRect(front);

    // Right side face
    QPolygonF side;
    side << QPointF(ox+w-dx, oy+dy) << QPointF(ox+w, oy)
         << QPointF(ox+w,    oy+h-dy) << QPointF(ox+w-dx, oy+h);
    p->setBrush(sideC); p->drawPolygon(side);

    // Arrow indicating wave propagation
    double arrowY = oy+dy + (h-dy)*0.55;
    double ax0=ox+8, ax1=ox+w-dx-12;
    p->setPen(QPen(CBStyle::GREEN, 1.5));
    p->drawLine(QPointF(ax0,arrowY),QPointF(ax1,arrowY));
    QPolygonF arr;
    arr << QPointF(ax1,arrowY-4) << QPointF(ax1+8,arrowY) << QPointF(ax1,arrowY+4);
    p->setBrush(CBStyle::GREEN); p->setPen(Qt::NoPen); p->drawPolygon(arr);

    // Label
    p->setPen(QPen(CBStyle::GREEN, 1));
    QFont f("Courier",7); p->setFont(f);
    p->drawText(QPointF(ox+6, arrowY-5), lbl);
    p->restore();
}

// Draw source (EM wave emitter, concentric rings + arrows)
static void drawSource(QPainter* p, QRectF r, bool selected=false)
{
    p->save();
    p->setRenderHint(QPainter::Antialiasing);
    QPointF c=r.center();
    double rad=qMin(r.width(),r.height())*0.38;
    QPen pen(selected ? CBStyle::ORANGE : QColor(160, 90, 10), selected?1.5:1.0);

    // Outer rings
    for(int i=3;i>=1;--i){
        double rr=rad*i/3.0;
        p->setPen(QPen(CBStyle::ORANGE.lighter(100 + i*20), 0.8, Qt::DashLine));
        p->setBrush(Qt::NoBrush);
        p->drawEllipse(c, rr, rr);
    }
    // Center dot
    p->setBrush(CBStyle::ORANGE); p->setPen(Qt::NoPen);
    p->drawEllipse(c, rad*0.25, rad*0.25);

    // Radiation arrows (6 directions)
    for(int deg=0;deg<360;deg+=60){
        double rad0=deg*M_PI/180.0;
        QPointF from(c.x()+rad*0.9*cos(rad0), c.y()+rad*0.9*sin(rad0));
        QPointF to  (c.x()+rad*1.4*cos(rad0), c.y()+rad*1.4*sin(rad0));
        p->setPen(QPen(CBStyle::ORANGE, 1, Qt::SolidLine,Qt::RoundCap));
        p->drawLine(from,to);
    }
    // "E" label
    p->setPen(QPen(CBStyle::ORANGE,1));
    QFont f("Courier",9); f.setBold(true); p->setFont(f);
    p->drawText(QRectF(c.x()-5,c.y()-7,12,14), Qt::AlignCenter, "E");
    p->restore();
}

// Draw observation point / load (crosshair + concentric)
static void drawObservation(QPainter* p, QRectF r, bool selected=false)
{
    p->save();
    p->setRenderHint(QPainter::Antialiasing);
    QPointF c=r.center();
    double rad=qMin(r.width(),r.height())*0.36;

    p->setBrush(Qt::NoBrush);
    for(int i=1;i<=2;++i){
        p->setPen(QPen(CBStyle::RED.lighter(100 + i*30), selected?1.2:0.8));
        p->drawEllipse(c, rad*i/2.0, rad*i/2.0);
    }
    // Crosshair
    p->setPen(QPen(CBStyle::RED, 0.8, Qt::SolidLine, Qt::RoundCap));
    p->drawLine(QPointF(c.x()-rad*1.1,c.y()), QPointF(c.x()+rad*1.1,c.y()));
    p->drawLine(QPointF(c.x(),c.y()-rad*1.1), QPointF(c.x(),c.y()+rad*1.1));
    // Center
    p->setBrush(CBStyle::RED); p->setPen(Qt::NoPen);
    p->drawEllipse(c, 3.5, 3.5);
    // Label
    p->setPen(QPen(CBStyle::RED,1));
    QFont f("Courier",7); p->setFont(f);
    p->drawText(QPointF(c.x()+rad*0.4, c.y()-rad*0.3), "Z\u2097");
    p->restore();
}

} // namespace ElementIcon

// ============================================================
//  PaletteButton — a draggable toolbar button for one element type
// ============================================================
class PaletteButton : public QWidget {
    Q_OBJECT
public:
    ElementType type;
    QString     typeName;
    QColor      accent;

    PaletteButton(ElementType t, const QString& name, const QColor& col, QWidget* parent=nullptr)
        : QWidget(parent), type(t), typeName(name), accent(col)
    {
        setFixedSize(76, 80);
        setToolTip(QString("Drag %1 onto the canvas").arg(name));
        setCursor(Qt::OpenHandCursor);
    }

protected:
    void paintEvent(QPaintEvent*) override
    {
        QPainter p(this);
        p.setRenderHint(QPainter::Antialiasing);

        bool hov = underMouse();
        QColor bg  = hov ? CBStyle::SURFACE2 : CBStyle::SURFACE;
        QColor brd = hov ? accent : CBStyle::BORDER;

        // Card background
        p.setBrush(bg);
        p.setPen(QPen(brd, hov?1.5:1.0));
        p.drawRoundedRect(rect().adjusted(1,1,-1,-1), 6, 6);

        // Icon area
        QRectF iconR(6, 4, 64, 56);
        drawIcon(&p, iconR);

        // Label
        p.setPen(accent);
        QFont f("Courier New", 7); f.setBold(false); p.setFont(f);
        p.drawText(QRect(0, 62, width(), 14), Qt::AlignCenter, typeName.toUpper());
    }

    void drawIcon(QPainter* p, QRectF r)
    {
        switch(type){
        case ElementType::Source:              ElementIcon::drawSource(p,r); break;
        case ElementType::Aperture:
        case ElementType::ApertureWithCover:   ElementIcon::drawAperture(p,r); break;
        case ElementType::EmptyCavity:
        case ElementType::DielectricCavity:    ElementIcon::drawCavity(p,r); break;
        case ElementType::Load:                ElementIcon::drawObservation(p,r); break;
        }
    }

    void mousePressEvent(QMouseEvent* ev) override
    {
        if(ev->button()!=Qt::LeftButton) return;
        // Package element type in drag MIME
        QDrag*     drag = new QDrag(this);
        QMimeData* mime = new QMimeData;
        mime->setText(QString::number((int)type));
        drag->setMimeData(mime);

        // Drag pixmap preview
        QPixmap pm(size());
        pm.fill(Qt::transparent);
        render(&pm);
        drag->setPixmap(pm);
        drag->setHotSpot(ev->pos());

        drag->exec(Qt::CopyAction);
    }

    void enterEvent(QEnterEvent*) override { update(); }
    void leaveEvent(QEvent*)      override { update(); }
};

// ============================================================
//  CanvasElement — a movable QGraphicsItem on the assembly canvas
// ============================================================
class CanvasElement : public QGraphicsItem {
public:
    ElementParams params;
    bool          selected_ { false };
    bool          hovered_  { false };

    static constexpr qreal W = 90;   // item width
    static constexpr qreal H = 80;   // item height

    explicit CanvasElement(ElementType type, QGraphicsItem* parent=nullptr)
        : QGraphicsItem(parent)
    {
        params.type = type;
        params.label = defaultLabel(type);
        setFlags(QGraphicsItem::ItemIsMovable |
                 QGraphicsItem::ItemIsSelectable |
                 QGraphicsItem::ItemSendsGeometryChanges);
        setAcceptHoverEvents(true);
    }

    QRectF boundingRect() const override { return QRectF(0,0,W,H+20); }

    void paint(QPainter* p, const QStyleOptionGraphicsItem*, QWidget*) override
    {
        bool sel = isSelected();
        QColor accentFor = accentColor();

        // Selection / hover glow
        if(sel){
            p->setPen(Qt::NoPen);
            p->setBrush(QColor(accentFor.red(),accentFor.green(),accentFor.blue(),35));
            p->drawRoundedRect(QRectF(-2,-2,W+4,H+4),6,6);
            p->setPen(QPen(accentFor,2.0));
            p->setBrush(Qt::NoBrush);
            p->drawRoundedRect(QRectF(-2,-2,W+4,H+4),6,6);
        } else {
            // Subtle card shadow on light background
            p->setPen(QPen(CBStyle::BORDER, 0.8));
            p->setBrush(QColor(255,255,255,180));
            p->drawRoundedRect(QRectF(0,0,W,H),4,4);
        }

        // Draw 3-D icon
        QRectF iconR(4,4,W-8,H-8);
        switch(params.type){
        case ElementType::Source:           ElementIcon::drawSource(p,iconR,sel);      break;
        case ElementType::Aperture:
        case ElementType::ApertureWithCover:ElementIcon::drawAperture(p,iconR,sel);    break;
        case ElementType::EmptyCavity:
        case ElementType::DielectricCavity: ElementIcon::drawCavity(p,iconR,sel,
                                    params.label.left(4));                  break;
        case ElementType::Load:             ElementIcon::drawObservation(p,iconR,sel); break;
        }

        // Label below
        p->setPen(QPen(accentFor,1));
        QFont f("Courier New",7); p->setFont(f);
        p->drawText(QRectF(0,H+2,W,16), Qt::AlignCenter, params.label);
    }

    QColor accentColor() const {
        switch(params.type){
        case ElementType::Source:              return CBStyle::ORANGE;
        case ElementType::Aperture:
        case ElementType::ApertureWithCover:   return CBStyle::ACCENT;
        case ElementType::EmptyCavity:
        case ElementType::DielectricCavity:    return CBStyle::GREEN;
        case ElementType::Load:                return CBStyle::RED;
        }
        return CBStyle::TEXT;
    }

    QString defaultLabel(ElementType t){
        switch(t){
        case ElementType::Source:              return "E₀ Source";
        case ElementType::Aperture:            return "Aperture";
        case ElementType::ApertureWithCover:   return "AP+Cover";
        case ElementType::EmptyCavity:         return "Cavity";
        case ElementType::DielectricCavity:    return "Diel. Cav.";
        case ElementType::Load:                return "Obs. Point";
        }
        return "Element";
    }

protected:
    void hoverEnterEvent(QGraphicsSceneHoverEvent*) override { hovered_=true;  update(); }
    void hoverLeaveEvent(QGraphicsSceneHoverEvent*) override { hovered_=false; update(); }

    QVariant itemChange(GraphicsItemChange ch, const QVariant& v) override {
        if(ch==ItemPositionHasChanged && scene())
            scene()->update();
        return QGraphicsItem::itemChange(ch,v);
    }
};

// ============================================================
//  ConnectionWire — draws dashed line between two CanvasElements
// ============================================================
class ConnectionWire : public QGraphicsItem {
public:
    CanvasElement* src;
    CanvasElement* dst;

    ConnectionWire(CanvasElement* s, CanvasElement* d, QGraphicsItem* parent=nullptr)
        : QGraphicsItem(parent), src(s), dst(d)
    {
        setZValue(-1);
        setFlag(QGraphicsItem::ItemStacksBehindParent);
    }

    QRectF boundingRect() const override {
        auto a=src->pos(), b=dst->pos();
        return QRectF(qMin(a.x(),b.x())-10, qMin(a.y(),b.y())-10,
                      qAbs(a.x()-b.x())+CanvasElement::W+20,
                      qAbs(a.y()-b.y())+CanvasElement::H+20);
    }

    void paint(QPainter* p, const QStyleOptionGraphicsItem*, QWidget*) override {
        QPointF a = src->pos() + QPointF(CanvasElement::W, CanvasElement::H/2.0);
        QPointF b = dst->pos() + QPointF(0,               CanvasElement::H/2.0);
        QPainterPath path;
        double mx=(a.x()+b.x())/2.0;
        path.moveTo(a);
        path.cubicTo(QPointF(mx,a.y()), QPointF(mx,b.y()), b);

        p->setPen(QPen(CBStyle::ACCENT, 1.5, Qt::DashLine));
        p->setBrush(Qt::NoBrush);
        p->drawPath(path);

        // Connector dots
        p->setBrush(CBStyle::ACCENT); p->setPen(Qt::NoPen);
        p->drawEllipse(a, 4,4);
        p->drawEllipse(b, 4,4);
    }
};

// ============================================================
//  AssemblyCanvas — the QGraphicsView that accepts drops
// ============================================================
class AssemblyCanvas : public QGraphicsView {
    Q_OBJECT
public:
    QVector<CanvasElement*> elements;
    QVector<ConnectionWire*> wires;

    explicit AssemblyCanvas(QWidget* parent=nullptr)
        : QGraphicsView(parent)
    {
        QGraphicsScene* sc = new QGraphicsScene(this);
        sc->setSceneRect(0,0,1600,600);
        sc->setBackgroundBrush(Qt::NoBrush);
        setScene(sc);

        setAcceptDrops(true);
        setRenderHints(QPainter::Antialiasing|QPainter::SmoothPixmapTransform);
        setDragMode(QGraphicsView::RubberBandDrag);
        setStyleSheet(QString("background: rgb(%1,%2,%3); border:none;")
                          .arg(CBStyle::BG.red())
                          .arg(CBStyle::BG.green())
                          .arg(CBStyle::BG.blue()));
        setHorizontalScrollBarPolicy(Qt::ScrollBarAsNeeded);
        setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);

        // Dot-grid is drawn in drawBackground
    }

signals:
    void elementSelected(CanvasElement* el);
    void elementDropped(ElementType type, QPointF scenePos);
    void circuitChanged();

protected:
    void drawBackground(QPainter* p, const QRectF& rect) override {
        // Solid background
        p->fillRect(rect, CBStyle::BG);

        // Dot grid (32 px spacing)
        p->setPen(QPen(CBStyle::BORDER_LT, 1.5));
        const int step=32;
        int x0=int(rect.left())/step*step;
        int y0=int(rect.top()) /step*step;
        for(int x=x0; x<rect.right(); x+=step)
            for(int y=y0; y<rect.bottom(); y+=step)
                p->drawPoint(x,y);
    }

    void dragEnterEvent(QDragEnterEvent* ev) override {
        if(ev->mimeData()->hasText()) ev->acceptProposedAction();
    }
    void dragMoveEvent(QDragMoveEvent* ev) override {
        if(ev->mimeData()->hasText()) ev->acceptProposedAction();
    }
    void dropEvent(QDropEvent* ev) override {
        bool ok=false;
        int t = ev->mimeData()->text().toInt(&ok);
        if(!ok) return;
        QPointF sp = mapToScene(ev->position().toPoint());
        emit elementDropped(static_cast<ElementType>(t), sp);
        ev->acceptProposedAction();
    }

    void mousePressEvent(QMouseEvent* ev) override {
        QGraphicsView::mousePressEvent(ev);
        QGraphicsItem* it = itemAt(ev->pos());
        CanvasElement* el = dynamic_cast<CanvasElement*>(it);
        emit elementSelected(el);
    }

public slots:
    // Add element at scene position
    CanvasElement* addElement(ElementType type, QPointF pos)
    {
        auto* el = new CanvasElement(type);
        el->setPos(pos);
        scene()->addItem(el);
        elements.append(el);

        // Auto-connect to previous element
        if(elements.size()>1){
            auto* wire = new ConnectionWire(elements[elements.size()-2], el);
            scene()->addItem(wire);
            wires.append(wire);
        }
        emit circuitChanged();
        return el;
    }

    void removeSelected()
    {
        for(auto* it : scene()->selectedItems()){
            auto* el = dynamic_cast<CanvasElement*>(it);
            if(!el) continue;
            // Remove associated wires
            wires.erase(std::remove_if(wires.begin(), wires.end(),
                                       [&](ConnectionWire* w){
                                           if(w->src==el||w->dst==el){ scene()->removeItem(w); delete w; return true; }
                                           return false;
                                       }), wires.end());
            elements.removeOne(el);
            scene()->removeItem(el);
            delete el;
        }
        emit circuitChanged();
    }

    void clearAll()
    {
        for(auto* w : wires){ scene()->removeItem(w); delete w; }
        for(auto* e : elements){ scene()->removeItem(e); delete e; }
        wires.clear(); elements.clear();
        emit circuitChanged();
    }

    // Auto-arrange elements in a horizontal row
    void autoArrange()
    {
        const double startX=60, startY=200, spacingX=130;
        for(int i=0;i<elements.size();++i)
            elements[i]->setPos(startX + i*spacingX, startY);
        // Rebuild wires
        for(auto* w:wires){ scene()->removeItem(w); delete w; }
        wires.clear();
        for(int i=0;i<elements.size()-1;++i){
            auto* w=new ConnectionWire(elements[i],elements[i+1]);
            scene()->addItem(w);
            wires.append(w);
        }
        scene()->update();
    }
};

// ============================================================
//  PropertyPanel — left dock, context-sensitive parameter editor
// ============================================================
class BuilderPropertyPanel : public QWidget {
    Q_OBJECT
public:
    explicit BuilderPropertyPanel(QWidget* parent=nullptr) : QWidget(parent)
    {
        setMinimumWidth(220);
        setMaximumWidth(260);
        setStyleSheet(QString("background: rgb(%1,%2,%3); color: rgb(%4,%5,%6);")
                          .arg(CBStyle::SURFACE.red()).arg(CBStyle::SURFACE.green()).arg(CBStyle::SURFACE.blue())
                          .arg(CBStyle::TEXT.red()).arg(CBStyle::TEXT.green()).arg(CBStyle::TEXT.blue()));

        auto* layout = new QVBoxLayout(this);
        layout->setContentsMargins(10,10,10,8);
        layout->setSpacing(4);

        // Header
        headerLabel_ = new QLabel("PROPERTIES", this);
        headerLabel_->setStyleSheet(QString("color: rgb(%1,%2,%3); font-family:'Courier New'; font-size:10px; letter-spacing:2px;")
                                        .arg(CBStyle::TEXT_DIM.red()).arg(CBStyle::TEXT_DIM.green()).arg(CBStyle::TEXT_DIM.blue()));
        layout->addWidget(headerLabel_);

        typeLabel_ = new QLabel("No element selected", this);
        typeLabel_->setStyleSheet(QString("font-family:'Courier New'; font-size:11px; color:rgb(%1,%2,%3);")
                                      .arg(CBStyle::TEXT_MUTED.red()).arg(CBStyle::TEXT_MUTED.green()).arg(CBStyle::TEXT_MUTED.blue()));
        layout->addWidget(typeLabel_);

        // Scroll area for form
        scrollArea_ = new QScrollArea(this);
        scrollArea_->setWidgetResizable(true);
        scrollArea_->setStyleSheet("border:none; background:transparent;");
        formWidget_ = new QWidget;
        formLayout_ = new QFormLayout(formWidget_);
        formLayout_->setContentsMargins(0,6,0,6);
        formLayout_->setSpacing(6);
        formLayout_->setLabelAlignment(Qt::AlignRight);
        scrollArea_->setWidget(formWidget_);
        layout->addWidget(scrollArea_);

        // Placeholder
        placeholderLabel_ = new QLabel(
            "←  Drag elements\n"
            "    onto the canvas\n\n"
            "    Click an element\n"
            "    to edit its\n"
            "    parameters here.",
            this);
        placeholderLabel_->setStyleSheet(
            QString("color: rgb(%1,%2,%3); font-family:'Courier New'; font-size:11px;")
                .arg(CBStyle::TEXT_DIM.red()).arg(CBStyle::TEXT_DIM.green()).arg(CBStyle::TEXT_DIM.blue()));
        placeholderLabel_->setAlignment(Qt::AlignCenter);
        layout->addWidget(placeholderLabel_);
        layout->addStretch();

        showPlaceholder(true);
    }

signals:
    void paramsChanged(CanvasElement* el);

public slots:
    void showElement(CanvasElement* el)
    {
        currentElement_ = el;
        clearForm();

        if(!el){
            showPlaceholder(true);
            typeLabel_->setText("No element selected");
            return;
        }
        showPlaceholder(false);

        QColor col = el->accentColor();
        typeLabel_->setText(typeToString(el->params.type));
        typeLabel_->setStyleSheet(QString("font-family:'Courier New'; font-size:11px; color:rgb(%1,%2,%3);")
                                      .arg(col.red()).arg(col.green()).arg(col.blue()));

        // Label field (always present)
        addLineEdit("Label:", el->params.label, [=](const QString& v){
            currentElement_->params.label = v;
            currentElement_->update();
        });

        // Type-specific fields
        switch(el->params.type){
        case ElementType::Source:
            addDoubleField("E₀ [V/m]:", el->params.E0, 0.01, 1000, 0.1,
                           [=](double v){ currentElement_->params.E0=v; });
            addDoubleField("f start [GHz]:", el->params.freqStart, 0.1, 100, 0.5,
                           [=](double v){ currentElement_->params.freqStart=v; });
            addDoubleField("f end [GHz]:", el->params.freqEnd, 0.1, 100, 1.0,
                           [=](double v){ currentElement_->params.freqEnd=v; });
            addIntField("Points:", el->params.freqPoints, 10, 2000, 50,
                        [=](int v){ currentElement_->params.freqPoints=v; });
            addSeparator("Cross-section (global):");
            addDoubleField("a [m]:", el->params.a, 0.001, 1.0, 0.005,
                           [=](double v){ currentElement_->params.a=v; });
            addDoubleField("b [m]:", el->params.b_h, 0.001, 1.0, 0.005,
                           [=](double v){ currentElement_->params.b_h=v; });
            addDoubleField("t wall [m]:", el->params.t_wall, 0.0001, 0.1, 0.0005,
                           [=](double v){ currentElement_->params.t_wall=v; });
            break;

        case ElementType::Aperture:
            addDoubleField("l_slot [m]:", el->params.l_slot, 0.001, 1.0, 0.002,
                           [=](double v){ currentElement_->params.l_slot=v; });
            addDoubleField("w_slot [m]:", el->params.w_slot, 0.0001, 0.1, 0.0005,
                           [=](double v){ currentElement_->params.w_slot=v; });
            break;

        case ElementType::ApertureWithCover:
            addDoubleField("l_slot [m]:", el->params.l_slot, 0.001, 1.0, 0.002,
                           [=](double v){ currentElement_->params.l_slot=v; });
            addDoubleField("w_slot [m]:", el->params.w_slot, 0.0001, 0.1, 0.0005,
                           [=](double v){ currentElement_->params.w_slot=v; });
            addDoubleField("τ cover [m]:", el->params.tau_cover, 0.0001, 0.01, 0.0001,
                           [=](double v){ currentElement_->params.tau_cover=v; });
            break;

        case ElementType::EmptyCavity:
            addDoubleField("L [m]:", el->params.L_cavity, 0.001, 2.0, 0.01,
                           [=](double v){ currentElement_->params.L_cavity=v; });
            break;

        case ElementType::DielectricCavity:
            addDoubleField("L [m]:", el->params.L_cavity, 0.001, 2.0, 0.01,
                           [=](double v){ currentElement_->params.L_cavity=v; });
            addDoubleField("h diel. [m]:", el->params.h_dielectric, 0.0001, 1.0, 0.001,
                           [=](double v){ currentElement_->params.h_dielectric=v; });
            addDoubleField("ε_r:", el->params.eps_r, 1.0, 100.0, 0.5,
                           [=](double v){ currentElement_->params.eps_r=v; });
            break;

        case ElementType::Load:
            addInfoLabel(
                "Observation point: SE is measured\n"
                "at this node as 20·log₁₀(E₀/|V|).\n"
                "Z_L models the measurement port\n"
                "impedance (377 Ω = free space).");
            addDoubleField("Z_L real [Ω]:", el->params.ZL_real, 1.0, 1e6, 10,
                           [=](double v){ currentElement_->params.ZL_real=v; });
            addDoubleField("Z_L imag [Ω]:", el->params.ZL_imag, -1e6, 1e6, 10,
                           [=](double v){ currentElement_->params.ZL_imag=v; });
            break;
        }
    }

private:
    QLabel*      headerLabel_;
    QLabel*      typeLabel_;
    QLabel*      placeholderLabel_;
    QScrollArea* scrollArea_;
    QWidget*     formWidget_;
    QFormLayout* formLayout_;
    CanvasElement* currentElement_ { nullptr };

    QString typeToString(ElementType t){
        switch(t){
        case ElementType::Source:              return "SOURCE";
        case ElementType::Aperture:            return "APERTURE";
        case ElementType::ApertureWithCover:   return "APERTURE + COVER";
        case ElementType::EmptyCavity:         return "EMPTY CAVITY";
        case ElementType::DielectricCavity:    return "DIELECTRIC CAVITY";
        case ElementType::Load:                return "OBS. POINT  (Z_L)";
        }
        return "ELEMENT";
    }

    void clearForm(){
        while(formLayout_->rowCount()>0)
            formLayout_->removeRow(0);
    }

    void showPlaceholder(bool show){
        scrollArea_->setVisible(!show);
        placeholderLabel_->setVisible(show);
    }

    QString spinStyle(QColor col){
        return QString(
                   "QDoubleSpinBox,QSpinBox,QLineEdit {"
                   "  background:rgb(%1,%2,%3);"
                   "  color:rgb(%4,%5,%6);"
                   "  border:1px solid rgb(%7,%8,%9);"
                   "  border-radius:4px;"
                   "  padding:3px 6px;"
                   "  font-family:'Courier New'; font-size:11px;"
                   "}"
                   "QDoubleSpinBox:focus,QSpinBox:focus,QLineEdit:focus {"
                   "  border:1px solid rgb(%10,%11,%12);"
                   "}"
                   ).arg(CBStyle::BG.red()).arg(CBStyle::BG.green()).arg(CBStyle::BG.blue())
            .arg(CBStyle::TEXT.red()).arg(CBStyle::TEXT.green()).arg(CBStyle::TEXT.blue())
            .arg(CBStyle::BORDER.red()).arg(CBStyle::BORDER.green()).arg(CBStyle::BORDER.blue())
            .arg(col.red()).arg(col.green()).arg(col.blue());
    }

    QString labelStyle(){
        return QString("color:rgb(%1,%2,%3); font-family:'Courier New'; font-size:10px;")
        .arg(CBStyle::TEXT_MUTED.red()).arg(CBStyle::TEXT_MUTED.green()).arg(CBStyle::TEXT_MUTED.blue());
    }

    // ── Field factory helpers ──────────────────────────────────
    void addDoubleField(const QString& lbl, double val, double mn, double mx, double step,
                        std::function<void(double)> onChange)
    {
        auto* spin = new QDoubleSpinBox(formWidget_);
        spin->setRange(mn,mx); spin->setValue(val); spin->setSingleStep(step);
        spin->setDecimals(6);
        QColor col = currentElement_ ? currentElement_->accentColor() : CBStyle::ACCENT;
        spin->setStyleSheet(spinStyle(col));
        connect(spin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [=](double v){
            onChange(v);
            emit paramsChanged(currentElement_);
        });
        auto* lLabel=new QLabel(lbl,formWidget_); lLabel->setStyleSheet(labelStyle());
        formLayout_->addRow(lLabel, spin);
    }

    void addIntField(const QString& lbl, int val, int mn, int mx, int step,
                     std::function<void(int)> onChange)
    {
        auto* spin = new QSpinBox(formWidget_);
        spin->setRange(mn,mx); spin->setValue(val); spin->setSingleStep(step);
        QColor col = currentElement_ ? currentElement_->accentColor() : CBStyle::ACCENT;
        spin->setStyleSheet(spinStyle(col));
        connect(spin, QOverload<int>::of(&QSpinBox::valueChanged), this, [=](int v){
            onChange(v);
            emit paramsChanged(currentElement_);
        });
        auto* lLabel=new QLabel(lbl,formWidget_); lLabel->setStyleSheet(labelStyle());
        formLayout_->addRow(lLabel, spin);
    }

    void addLineEdit(const QString& lbl, const QString& val,
                     std::function<void(const QString&)> onChange)
    {
        auto* edit = new QLineEdit(formWidget_);
        edit->setText(val);
        QColor col = currentElement_ ? currentElement_->accentColor() : CBStyle::ACCENT;
        edit->setStyleSheet(spinStyle(col));
        connect(edit, &QLineEdit::textChanged, this, [=](const QString& v){
            onChange(v);
            emit paramsChanged(currentElement_);
        });
        auto* lLabel=new QLabel(lbl,formWidget_); lLabel->setStyleSheet(labelStyle());
        formLayout_->addRow(lLabel, edit);
    }

    void addSeparator(const QString& title)
    {
        auto* lbl=new QLabel(title, formWidget_);
        lbl->setStyleSheet(QString(
                               "color:rgb(%1,%2,%3);"
                               "font-family:'Courier New'; font-size:9px;"
                               "letter-spacing:1px;"
                               "margin-top:6px; border-top:1px solid rgb(%4,%5,%6);")
                               .arg(CBStyle::TEXT_DIM.red()).arg(CBStyle::TEXT_DIM.green()).arg(CBStyle::TEXT_DIM.blue())
                               .arg(CBStyle::BORDER.red()).arg(CBStyle::BORDER.green()).arg(CBStyle::BORDER.blue()));
        formLayout_->addRow(lbl);
    }

    void addInfoLabel(const QString& text)
    {
        auto* lbl = new QLabel(text, formWidget_);
        lbl->setWordWrap(true);
        lbl->setStyleSheet(QString(
                               "color: rgb(%1,%2,%3);"
                               "background: rgb(%4,%5,%6);"
                               "border: 1px solid rgb(%7,%8,%9);"
                               "border-radius: 4px;"
                               "font-family: 'Courier New'; font-size: 9px;"
                               "padding: 5px 6px;"
                               "margin-bottom: 4px;")
                               .arg(CBStyle::TEXT_MUTED.red()).arg(CBStyle::TEXT_MUTED.green()).arg(CBStyle::TEXT_MUTED.blue())
                               .arg(CBStyle::SURFACE2.red()).arg(CBStyle::SURFACE2.green()).arg(CBStyle::SURFACE2.blue())
                               .arg(CBStyle::BORDER.red()).arg(CBStyle::BORDER.green()).arg(CBStyle::BORDER.blue()));
        formLayout_->addRow(lbl);
    }
};

// ============================================================
//  SEPlotWidget — simple canvas-rendered shielding effectiveness plot
// ============================================================
class SEPlotWidget : public QWidget {
    Q_OBJECT
public:
    struct Curve { QVector<double> freqGHz; QVector<double> SE_dB; QString label; QColor color; };

    explicit SEPlotWidget(QWidget* parent=nullptr) : QWidget(parent) {
        setMinimumHeight(200);
        setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
        setStyleSheet(QString("background:rgb(%1,%2,%3); border-top:1px solid rgb(%4,%5,%6);")
                          .arg(CBStyle::SURFACE.red()).arg(CBStyle::SURFACE.green()).arg(CBStyle::SURFACE.blue())
                          .arg(CBStyle::BORDER.red()).arg(CBStyle::BORDER.green()).arg(CBStyle::BORDER.blue()));
    }

    void setCurves(const QVector<Curve>& c){ curves_=c; update(); }
    void clear(){ curves_.clear(); update(); }

protected:
    void paintEvent(QPaintEvent*) override
    {
        QPainter p(this);
        p.setRenderHint(QPainter::Antialiasing);
        QRect R=rect();
        p.fillRect(R, CBStyle::SURFACE);

        if(curves_.isEmpty()){
            p.setPen(CBStyle::TEXT_DIM);
            QFont f("Courier New",11); p.setFont(f);
            p.drawText(R, Qt::AlignCenter, "Run ⚡ Compute to see the SE curve");
            return;
        }

        // Margins
        int ml=55, mr=20, mt=28, mb=40;
        QRect plot(ml, mt, R.width()-ml-mr, R.height()-mt-mb);

        // Find data range
        double fMin=1e18,fMax=-1e18,sMin=1e18,sMax=-1e18;
        for(auto& c:curves_){
            for(double f:c.freqGHz){ fMin=qMin(fMin,f); fMax=qMax(fMax,f); }
            for(double s:c.SE_dB)  { sMin=qMin(sMin,s); sMax=qMax(sMax,s); }
        }
        sMin=qFloor(sMin/10.0)*10-5;
        sMax=qCeil (sMax/10.0)*10+5;

        auto toX=[&](double f){ return plot.left()+(f-fMin)/(fMax-fMin+1e-30)*plot.width(); };
        auto toY=[&](double s){ return plot.bottom()-(s-sMin)/(sMax-sMin+1e-30)*plot.height(); };

        // Grid
        // White plot area
        p.fillRect(plot, QColor(255,255,255));
        p.setPen(QPen(CBStyle::BORDER, 1));
        p.setBrush(Qt::NoBrush);
        p.drawRect(plot);

        QFont gf("Courier New",8); p.setFont(gf);
        for(double s=qCeil(sMin/10.0)*10; s<=sMax; s+=10){
            int y=toY(s);
            p.setPen(QPen(CBStyle::BORDER_LT, 0.7)); p.drawLine(plot.left(),y,plot.right(),y);
            p.setPen(CBStyle::TEXT_MUTED);
            p.drawText(QRect(0,y-8,ml-4,16), Qt::AlignRight|Qt::AlignVCenter, QString::number((int)s));
        }
        for(double f=0;f<=40;f+=8){
            if(f<fMin||f>fMax) continue;
            int x=toX(f);
            p.setPen(QPen(CBStyle::BORDER_LT, 0.7)); p.drawLine(x,plot.top(),x,plot.bottom());
            p.setPen(CBStyle::TEXT_MUTED);
            p.drawText(QRect(x-20,plot.bottom()+4,40,16), Qt::AlignCenter,
                       QString("%1 GHz").arg((int)f));
        }

        // Curves
        for(auto& c:curves_){
            if(c.freqGHz.isEmpty()) continue;
            QPainterPath path;
            for(int i=0;i<c.freqGHz.size();++i){
                QPointF pt(toX(c.freqGHz[i]), toY(c.SE_dB[i]));
                i==0 ? path.moveTo(pt) : path.lineTo(pt);
            }
            // Glow
            p.setPen(QPen(QColor(c.color.red(),c.color.green(),c.color.blue(),50),4));
            p.drawPath(path);
            // Main curve
            p.setPen(QPen(c.color,2));
            p.drawPath(path);
        }

        // Axes labels
        p.setPen(CBStyle::TEXT_MUTED); gf.setPointSize(9); p.setFont(gf);
        p.drawText(QRect(0,0,R.width(),mt), Qt::AlignCenter, "Shielding Effectiveness  SE [dB]");
        p.save(); p.translate(12,R.height()/2); p.rotate(-90);
        p.drawText(QRect(-60,-8,120,16), Qt::AlignCenter, "SE [dB]"); p.restore();
        p.drawText(QRect(0,R.height()-16,R.width(),16), Qt::AlignCenter, "Frequency [GHz]");
    }

private:
    QVector<Curve> curves_;
};

// ============================================================
//  CircuitBuilderWindow — the main Phase-B window
// ============================================================
class CircuitBuilderWindow : public QMainWindow {
    Q_OBJECT
public:
    explicit CircuitBuilderWindow(QWidget* parent=nullptr) : QMainWindow(parent)
    {
        setWindowTitle("Circuit Builder — EMShieldDesigner");
        setMinimumSize(1200, 720);
        resize(1400, 800);
        applyGlobalStyle();
        buildUI();
        connectSignals();
    }

private:
    // ── Widgets ──────────────────────────────────────────────
    AssemblyCanvas*       canvas_    { nullptr };
    BuilderPropertyPanel* propPanel_ { nullptr };
    SEPlotWidget*         seplot_    { nullptr };
    QLabel*               statusLbl_ { nullptr };

    // ── UI builder ───────────────────────────────────────────
    void applyGlobalStyle()
    {
        setStyleSheet(QString(R"(
            QMainWindow, QWidget {
                background: rgb(%1,%2,%3);
                color: rgb(%4,%5,%6);
                font-family: 'Courier New', monospace;
            }
            QToolBar {
                background: rgb(%7,%8,%9);
                border-bottom: 1px solid rgb(%10,%11,%12);
                spacing: 6px;
                padding: 4px 8px;
            }
            QStatusBar {
                background: rgb(%7,%8,%9);
                border-top: 1px solid rgb(%10,%11,%12);
                color: rgb(%13,%14,%15);
                font-size: 11px;
            }
            QPushButton {
                border-radius: 5px;
                padding: 5px 14px;
                font-family: 'Courier New';
                font-size: 11px;
            }
            QPushButton:disabled { opacity: 0.4; }
            QSplitter::handle { background: rgb(%10,%11,%12); width:1px; height:1px; }
        )")
                          .arg(CBStyle::BG.red())    .arg(CBStyle::BG.green())    .arg(CBStyle::BG.blue())
                          .arg(CBStyle::TEXT.red())  .arg(CBStyle::TEXT.green())  .arg(CBStyle::TEXT.blue())
                          .arg(CBStyle::SURFACE.red()).arg(CBStyle::SURFACE.green()).arg(CBStyle::SURFACE.blue())
                          .arg(CBStyle::BORDER.red()) .arg(CBStyle::BORDER.green()) .arg(CBStyle::BORDER.blue())
                          .arg(CBStyle::TEXT_MUTED.red()).arg(CBStyle::TEXT_MUTED.green()).arg(CBStyle::TEXT_MUTED.blue()));
    }

    void buildUI()
    {
        // ── Top toolbar: title + palette + actions ─────────
        QToolBar* toolbar = addToolBar("Palette");
        toolbar->setMovable(false);
        toolbar->setIconSize(QSize(40,40));

        // App title in toolbar
        auto* titleLbl = new QLabel(QString("  EM<span style='color:rgb(%1,%2,%3)'>Shield</span>"
                                            "<b style='color:rgb(%4,%5,%6)'>Builder</b>  ")
                                        .arg(CBStyle::TEXT_MUTED.red()).arg(CBStyle::TEXT_MUTED.green()).arg(CBStyle::TEXT_MUTED.blue())
                                        .arg(CBStyle::ACCENT.red()).arg(CBStyle::ACCENT.green()).arg(CBStyle::ACCENT.blue()),
                                    toolbar);
        titleLbl->setTextFormat(Qt::RichText);
        titleLbl->setStyleSheet("font-size:13px; letter-spacing:1px;");
        toolbar->addWidget(titleLbl);

        // Separator
        auto* sep1=new QFrame(toolbar);
        sep1->setFrameShape(QFrame::VLine);
        sep1->setStyleSheet(QString("background:rgb(%1,%2,%3); max-width:1px; margin:4px 6px;")
                                .arg(CBStyle::BORDER.red()).arg(CBStyle::BORDER.green()).arg(CBStyle::BORDER.blue()));
        toolbar->addWidget(sep1);

        // Palette buttons
        struct PaletteSpec { ElementType type; QString name; QColor col; };
        QVector<PaletteSpec> palette = {
                                        { ElementType::Source,              "Source",     CBStyle::ORANGE },
                                        { ElementType::Aperture,            "Aperture",   CBStyle::ACCENT },
                                        { ElementType::ApertureWithCover,   "AP+Cover",   CBStyle::ACCENT },
                                        { ElementType::EmptyCavity,         "Cavity",     CBStyle::GREEN  },
                                        { ElementType::DielectricCavity,    "Diel.Cav",   CBStyle::GREEN  },
                                        { ElementType::Load,                "Obs.Point",  CBStyle::RED    },
                                        };
        for(auto& spec : palette){
            auto* btn = new PaletteButton(spec.type, spec.name, spec.col, toolbar);
            toolbar->addWidget(btn);
        }

        // Flexible spacer
        auto* spacer=new QWidget(toolbar); spacer->setSizePolicy(QSizePolicy::Expanding,QSizePolicy::Preferred);
        toolbar->addWidget(spacer);

        // Action buttons (right side of toolbar)
        auto* btnArrange = makeToolBtn("⇌ Arrange",  CBStyle::TEXT_MUTED);
        auto* btnDelete  = makeToolBtn("✕ Delete",   CBStyle::RED);
        auto* btnClear   = makeToolBtn("⊘ Clear",    CBStyle::ORANGE);
        auto* btnCompute = makeToolBtn("⚡ Compute",  CBStyle::GREEN, true);

        toolbar->addWidget(btnArrange);
        toolbar->addWidget(btnDelete);
        toolbar->addWidget(btnClear);
        toolbar->addWidget(btnCompute);

        connect(btnArrange, &QPushButton::clicked, this, [this]{ canvas_->autoArrange(); });
        connect(btnDelete,  &QPushButton::clicked, this, [this]{ canvas_->removeSelected(); });
        connect(btnClear,   &QPushButton::clicked, this, [this]{
            canvas_->clearAll(); seplot_->clear(); setStatus("Canvas cleared.", CBStyle::TEXT_MUTED); });
        connect(btnCompute, &QPushButton::clicked, this, &CircuitBuilderWindow::runCompute);

        // ── Central area: horizontal splitter ──────────────
        auto* splitter = new QSplitter(Qt::Horizontal, this);

        // Left: property panel
        propPanel_ = new BuilderPropertyPanel(splitter);
        splitter->addWidget(propPanel_);

        // Right: canvas + plot (vertical splitter)
        auto* rightSplit = new QSplitter(Qt::Vertical, splitter);

        canvas_ = new AssemblyCanvas(rightSplit);
        rightSplit->addWidget(canvas_);

        seplot_ = new SEPlotWidget(rightSplit);
        rightSplit->addWidget(seplot_);
        rightSplit->setSizes({440, 220});

        splitter->addWidget(rightSplit);
        splitter->setSizes({230, 1170});
        splitter->setHandleWidth(2);

        setCentralWidget(splitter);

        // ── Status bar ─────────────────────────────────────
        statusLbl_ = new QLabel("Ready  —  drag elements from the palette, then ⚡ Compute");
        statusLbl_->setStyleSheet(QString("color:rgb(%1,%2,%3);")
                                      .arg(CBStyle::TEXT_MUTED.red()).arg(CBStyle::TEXT_MUTED.green()).arg(CBStyle::TEXT_MUTED.blue()));
        statusBar()->addWidget(statusLbl_);
        statusBar()->setSizeGripEnabled(false);
    }

    QPushButton* makeToolBtn(const QString& text, QColor col, bool bold=false)
    {
        auto* btn = new QPushButton(text);
        btn->setStyleSheet(QString(
                               "QPushButton {"
                               "  color:rgb(%1,%2,%3);"
                               "  background: rgb(%4,%5,%6);"
                               "  border: 1px solid rgb(%1,%2,%3);"
                               "  font-weight: %7;"
                               "  padding: 5px 14px;"
                               "  border-radius: 5px;"
                               "}"
                               "QPushButton:hover {"
                               "  background: rgba(%1,%2,%3,25);"
                               "  border: 1.5px solid rgb(%1,%2,%3);"
                               "}"
                               "QPushButton:pressed { background: rgba(%1,%2,%3,45); }"
                               ).arg(col.red()).arg(col.green()).arg(col.blue())
                               .arg(CBStyle::SURFACE.red()).arg(CBStyle::SURFACE.green()).arg(CBStyle::SURFACE.blue())
                               .arg(bold?"700":"400"));
        return btn;
    }

    void connectSignals()
    {
        connect(canvas_, &AssemblyCanvas::elementDropped, this,
                [this](ElementType type, QPointF pos){
                    auto* el = canvas_->addElement(type, pos);
                    propPanel_->showElement(el);
                    setStatus(QString("Added: %1").arg(el->params.label), CBStyle::ACCENT);
                });

        connect(canvas_, &AssemblyCanvas::elementSelected, this,
                [this](CanvasElement* el){ propPanel_->showElement(el); });

        connect(canvas_, &AssemblyCanvas::circuitChanged, this,
                [this](){ seplot_->clear(); });

        connect(propPanel_, &BuilderPropertyPanel::paramsChanged, this,
                [this](CanvasElement* el){ if(el) el->update(); });
    }

    // ── MNA computation ──────────────────────────────────────
    void runCompute()
    {
        auto& els = canvas_->elements;
        if(els.size() < 2){
            setStatus("Need at least Source + one branch element + Load.", CBStyle::RED);
            return;
        }

        // Find source & load elements
        CanvasElement* srcEl  = nullptr;
        CanvasElement* loadEl = nullptr;
        for(auto* e : els){
            if(e->params.type == ElementType::Source && !srcEl)  srcEl  = e;
            if(e->params.type == ElementType::Load   && !loadEl) loadEl = e;
        }
        if(!srcEl){  setStatus("No Source element found on canvas.", CBStyle::RED); return; }
        if(!loadEl){ setStatus("No Load element found on canvas.",   CBStyle::RED); return; }

        // Sort elements left-to-right by X position — defines circuit order
        QVector<CanvasElement*> ordered = els;
        std::sort(ordered.begin(), ordered.end(),
                  [](CanvasElement* a, CanvasElement* b){ return a->pos().x() < b->pos().x(); });

        // ── Extract global cross-section params from Source element ──────
        const ElementParams& sp = srcEl->params;
        const double a_g  = sp.a;          // waveguide width  [m]
        const double b_g  = sp.b_h;        // waveguide height [m]
        const double t_g  = sp.t_wall;     // wall thickness   [m]
        const double fMin = sp.freqStart * 1e9;
        const double fMax = sp.freqEnd   * 1e9;
        const int    Np   = sp.freqPoints;

        // ── Assign node numbers: each branch gets (nodeIndex, nodeIndex+1) ─
        // Source:  node 0 → node 1
        // Branch1: node 1 → node 2
        // ...
        // Load:    node N-1 → node 0 (ground)
        // Total independent nodes = ordered.size() - 1
        // Last node of Load connects back to ground (node 0).
        // MNASolver counts nodes automatically from branch node indices.
        const int numBranches = ordered.size();

        QVector<double> freqsGHz;
        QVector<double> SE_dB;
        freqsGHz.reserve(Np);
        SE_dB.reserve(Np);

        // ── Source voltage magnitude for SE reference ──────────────────
        // SE = 20·log10( |V_source| / |V_load_node| )
        // V_source = sp.E0 (the incident field amplitude set on Source element)
        const double V_source_mag = (sp.E0 > 0.0) ? sp.E0 : 1.0;

        // ── Frequency sweep ────────────────────────────────────────────
        for(int i = 0; i < Np; ++i)
        {
            double f = fMin + (fMax - fMin) * i / (Np > 1 ? Np - 1 : 1);
            freqsGHz.append(f / 1e9);

            // ── Build MNA solver for this frequency ────────────────────
            // MNASolver API:
            //   solver.addBranch(shared_ptr<BranchTemplate>)
            //   Eigen::VectorXcd V = solver.solve(f_Hz)
            //   V[k-1] = voltage at node k  (node 0 = ground is implicit)
            MNASolver solver;

            int nodeIdx = 0;  // current node counter

            for(auto* el : ordered)
            {
                const ElementParams& p = el->params;
                int nFrom = nodeIdx;
                int nTo   = nodeIdx + 1;
                // Load connects back to ground
                if(el->params.type == ElementType::Load)
                    nTo = 0;

                int branchId = nodeIdx;  // unique ID per branch

                switch(el->params.type)
                {
                // ── Voltage Source ─────────────────────────────────
                // SRC_VoltageSource(node_from, node_to, branch_id,
                //                   V_incident, Z_source)
                case ElementType::Source:
                    solver.addBranch(std::make_shared<SRC_VoltageSource>(
                        nFrom, nTo, branchId,
                        Complex(p.E0, 0.0),   // incident voltage phasor
                        120.0 * M_PI          // source impedance = Z0 = 377 Ω
                        ));
                    break;

                // ── Slot Aperture ──────────────────────────────────
                // AP_SlotAperture(node_from, node_to, branch_id,
                //                 a, b, l_slot, w_slot, t_wall)
                case ElementType::Aperture:
                    solver.addBranch(std::make_shared<AP_SlotAperture>(
                        nFrom, nTo, branchId,
                        a_g, b_g,
                        p.l_slot, p.w_slot, t_g
                        ));
                    break;

                // ── Aperture With Cover ────────────────────────────
                // AP_SlotWithCover(node_from, node_to, branch_id,
                //                  a, b, l_slot, w_slot, t_wall, tau_cover)
                case ElementType::ApertureWithCover:
                    solver.addBranch(std::make_shared<AP_SlotWithCover>(
                        nFrom, nTo, branchId,
                        a_g, b_g,
                        p.l_slot, p.w_slot, t_g, p.tau_cover
                        ));
                    break;

                // ── Empty Cavity ───────────────────────────────────
                // TL_EmptyCavity(node_from, node_to, branch_id,
                //                a, b, L)
                case ElementType::EmptyCavity:
                    solver.addBranch(std::make_shared<TL_EmptyCavity>(
                        nFrom, nTo, branchId,
                        a_g, b_g, p.L_cavity
                        ));
                    break;

                // ── Dielectric Cavity ──────────────────────────────
                // TL_DielectricCavity(node_from, node_to, branch_id,
                //                     a, b, L, h_dielectric, eps_r)
                case ElementType::DielectricCavity:
                {
                    // Guard: h_dielectric must be <= b
                    double h_safe = std::min(p.h_dielectric, b_g * 0.999);
                    h_safe = std::max(h_safe, 1e-6);
                    solver.addBranch(std::make_shared<TL_DielectricCavity>(
                        nFrom, nTo, branchId,
                        a_g, b_g, p.L_cavity,
                        h_safe, p.eps_r
                        ));
                    break;
                }

                // ── Load Impedance ─────────────────────────────────
                // LOAD_Impedance(node_from, node_to, branch_id, Z_load)
                case ElementType::Load:
                    solver.addBranch(std::make_shared<LOAD_Impedance>(
                        nFrom, nTo, branchId,
                        Complex(p.ZL_real, p.ZL_imag)
                        ));
                    break;
                }

                // Advance node index for every branch except Load
                // (Load terminates back at ground, so no new node)
                if(el->params.type != ElementType::Load)
                    ++nodeIdx;
            }

            // ── Solve: returns node voltage vector V[0..N-1] ──────────
            // Node indices in V are 1-based internally but vector is 0-based:
            //   V(0) = voltage at node 1
            //   V(1) = voltage at node 2  ...
            //   V(nodeIdx-1) = voltage at last active node (before Load)
            try {
                Eigen::VectorXcd V = solver.solve(f);

                // ── Compute SE ─────────────────────────────────────────
                // The last active node (nodeIdx - 1) is the observation node
                // just before the Load. Its index in V is (nodeIdx - 1) - 1
                // because node 0 (ground) is implicit.
                // V has size = max_node_index = nodeIdx
                // Node k maps to V(k-1).
                // Last active node index = nodeIdx  →  V(nodeIdx - 1)
                int obsNodeIdx = nodeIdx - 1;  // 0-based index in V
                if(obsNodeIdx < 0 || obsNodeIdx >= (int)V.size()){
                    SE_dB.append(0.0);
                    continue;
                }

                double V_obs = std::abs(V(obsNodeIdx));
                if(V_obs < 1e-30) V_obs = 1e-30;  // avoid log(0)

                double se = 20.0 * std::log10(V_source_mag / V_obs);
                SE_dB.append(se);
            }
            catch(const std::exception& ex){
                // Solver can throw if circuit is degenerate at this frequency
                SE_dB.append(0.0);
                qDebug() << "MNA solver exception at f=" << f/1e9 << "GHz:" << ex.what();
            }
        }

        // ── Display result ─────────────────────────────────────────────
        SEPlotWidget::Curve curve;
        curve.freqGHz = freqsGHz;
        curve.SE_dB   = SE_dB;
        curve.label   = "SE (user circuit)";
        curve.color   = CBStyle::ACCENT;
        seplot_->setCurves({curve});

        double seMin = *std::min_element(SE_dB.begin(), SE_dB.end());
        double seMax = *std::max_element(SE_dB.begin(), SE_dB.end());
        setStatus(
            QString("✓  Computed %1 pts  ·  SE range: %2 … %3 dB  ·  %4–%5 GHz")
                .arg(Np)
                .arg(seMin, 0, 'f', 1).arg(seMax, 0, 'f', 1)
                .arg(sp.freqStart, 0, 'f', 1).arg(sp.freqEnd, 0, 'f', 1),
            CBStyle::GREEN);
    }

    void setStatus(const QString& msg, QColor col=CBStyle::TEXT_MUTED)
    {
        statusLbl_->setText(msg);
        statusLbl_->setStyleSheet(QString("color:rgb(%1,%2,%3);")
                                      .arg(col.red()).arg(col.green()).arg(col.blue()));
    }
};
