#pragma once

// ============================================================
//  CircuitBuilderWindow.h  —  EMShieldDesigner Phase B
//
//  Architecture overview (matches mainwindow.cpp / Phase A):
//
//  ┌─────────────┐    ┌────────────────────────────────────┐
//  │ BuilderProp │    │  AssemblyCanvas (drag-and-drop)    │
//  │   Panel     │    │  Elements ordered left → right:    │
//  │             │    │  [Source]→[Aperture]→[Cavity]→[Obs]│
//  └─────────────┘    └────────────────────────────────────┘
//                     ┌────────────────────────────────────┐
//                     │  QCustomPlot  (matches Phase A)    │
//                     │  N Obs.Points → N SE curves        │
//                     │  Click-to-read interactive overlay │
//                     └────────────────────────────────────┘
//
//  Key design decisions:
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
#include <QToolBar>
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
//  Pure-QPainter 3-D element icons
// ============================================================
namespace ElementIcon {

static void drawAperture(QPainter* p, QRectF r, bool sel=false)
{
    p->save();
    p->setRenderHint(QPainter::Antialiasing);
    QColor front = sel ? QColor(155,195,240) : QColor(180,210,240);
    QColor topF  = sel ? QColor(190,215,245) : QColor(210,228,248);
    QColor side  = sel ? QColor(120,165,220) : QColor(145,185,225);
    QPen pen(sel ? CBStyle::ACCENT : CBStyle::BORDER, sel?1.5:0.8);
    double w=r.width(),h=r.height(),ox=r.x(),oy=r.y(),d=w*0.12;
    QRectF fr(ox,oy+d,w-d,h-d);
    p->setPen(pen); p->setBrush(front); p->drawRect(fr);
    QPolygonF tp; tp<<QPointF(ox,oy+d)<<QPointF(ox+d,oy)<<QPointF(ox+w,oy)<<QPointF(ox+w-d,oy+d);
    p->setBrush(topF); p->drawPolygon(tp);
    QPolygonF rt; rt<<QPointF(ox+w-d,oy+d)<<QPointF(ox+w,oy)<<QPointF(ox+w,oy+h-d)<<QPointF(ox+w-d,oy+h);
    p->setBrush(side); p->drawPolygon(rt);
    double sx=fr.left()+fr.width()*0.35, sw=fr.width()*0.30;
    double sy=fr.top()+fr.height()*0.25, sh=fr.height()*0.50;
    p->setPen(Qt::NoPen); p->setBrush(CBStyle::BG); p->drawRect(QRectF(sx,sy,sw,sh));
    p->setPen(QPen(sel?CBStyle::ACCENT:CBStyle::ORANGE,1.4)); p->setBrush(Qt::NoBrush);
    p->drawRect(QRectF(sx,sy,sw,sh));
    p->setPen(QPen(CBStyle::ACCENT,1));
    QFont f("Courier",7); f.setBold(true); p->setFont(f);
    p->drawText(QPointF(ox+3,oy+d+fr.height()*0.6),"H");
    p->restore();
}

static void drawCavity(QPainter* p, QRectF r, bool sel=false, const QString& lbl="CAV")
{
    p->save(); p->setRenderHint(QPainter::Antialiasing);
    QColor tc=sel?QColor(185,230,200):QColor(205,238,215);
    QColor fc=sel?QColor(155,210,175):QColor(175,220,190);
    QColor sc=sel?QColor(125,190,150):QColor(145,200,165);
    QPen pen(sel?CBStyle::GREEN:CBStyle::BORDER,sel?1.5:0.8);
    double w=r.width(),h=r.height(),ox=r.x(),oy=r.y(),dx=w*0.18,dy=h*0.22;
    QPolygonF tp; tp<<QPointF(ox,oy+dy)<<QPointF(ox+dx,oy)<<QPointF(ox+w,oy)<<QPointF(ox+w-dx,oy+dy);
    p->setPen(pen); p->setBrush(tc); p->drawPolygon(tp);
    QRectF fr(ox,oy+dy,w-dx,h-dy); p->setBrush(fc); p->drawRect(fr);
    QPolygonF sd; sd<<QPointF(ox+w-dx,oy+dy)<<QPointF(ox+w,oy)<<QPointF(ox+w,oy+h-dy)<<QPointF(ox+w-dx,oy+h);
    p->setBrush(sc); p->drawPolygon(sd);
    double ay=oy+dy+(h-dy)*0.55, ax0=ox+8, ax1=ox+w-dx-12;
    p->setPen(QPen(CBStyle::GREEN,1.5)); p->drawLine(QPointF(ax0,ay),QPointF(ax1,ay));
    QPolygonF arr; arr<<QPointF(ax1,ay-4)<<QPointF(ax1+8,ay)<<QPointF(ax1,ay+4);
    p->setBrush(CBStyle::GREEN); p->setPen(Qt::NoPen); p->drawPolygon(arr);
    p->setPen(QPen(CBStyle::GREEN,1)); QFont f("Courier",7); p->setFont(f);
    p->drawText(QPointF(ox+6,ay-5),lbl);
    p->restore();
}

static void drawSource(QPainter* p, QRectF r, bool sel=false)
{
    p->save(); p->setRenderHint(QPainter::Antialiasing);
    QPointF c=r.center(); double rad=qMin(r.width(),r.height())*0.38;
    for(int i=3;i>=1;--i){
        p->setPen(QPen(CBStyle::ORANGE.lighter(100+i*20),0.8,Qt::DashLine));
        p->setBrush(Qt::NoBrush); p->drawEllipse(c,rad*i/3.0,rad*i/3.0);
    }
    p->setBrush(CBStyle::ORANGE); p->setPen(Qt::NoPen); p->drawEllipse(c,rad*0.25,rad*0.25);
    for(int deg=0;deg<360;deg+=60){
        double a0=deg*M_PI/180.0;
        p->setPen(QPen(CBStyle::ORANGE,1,Qt::SolidLine,Qt::RoundCap));
        p->drawLine(QPointF(c.x()+rad*0.9*cos(a0),c.y()+rad*0.9*sin(a0)),
                    QPointF(c.x()+rad*1.4*cos(a0),c.y()+rad*1.4*sin(a0)));
    }
    p->setPen(QPen(CBStyle::ORANGE,1)); QFont f("Courier",9); f.setBold(true); p->setFont(f);
    p->drawText(QRectF(c.x()-5,c.y()-7,12,14),Qt::AlignCenter,"E");
    p->restore();
}

static void drawObservation(QPainter* p, QRectF r, bool sel=false)
{
    p->save(); p->setRenderHint(QPainter::Antialiasing);
    QPointF c=r.center(); double rad=qMin(r.width(),r.height())*0.36;
    p->setBrush(Qt::NoBrush);
    for(int i=1;i<=2;++i){
        p->setPen(QPen(CBStyle::RED.lighter(100+i*30),sel?1.2:0.8));
        p->drawEllipse(c,rad*i/2.0,rad*i/2.0);
    }
    p->setPen(QPen(CBStyle::RED,0.8,Qt::SolidLine,Qt::RoundCap));
    p->drawLine(QPointF(c.x()-rad*1.1,c.y()),QPointF(c.x()+rad*1.1,c.y()));
    p->drawLine(QPointF(c.x(),c.y()-rad*1.1),QPointF(c.x(),c.y()+rad*1.1));
    p->setBrush(CBStyle::RED); p->setPen(Qt::NoPen); p->drawEllipse(c,3.5,3.5);
    p->setPen(QPen(CBStyle::RED,1)); QFont f("Courier",7); p->setFont(f);
    p->drawText(QPointF(c.x()+rad*0.4,c.y()-rad*0.3),"Obs");
    p->restore();
}

} // namespace ElementIcon

// ============================================================
//  PaletteButton — draggable toolbar tile
// ============================================================
class PaletteButton : public QWidget {
    Q_OBJECT
public:
    ElementType type; QString typeName; QColor accent;
    PaletteButton(ElementType t, const QString& n, const QColor& c, QWidget* p=nullptr)
        : QWidget(p), type(t), typeName(n), accent(c)
    { setFixedSize(76,80); setToolTip(QString("Drag %1 onto canvas").arg(n)); setCursor(Qt::OpenHandCursor); }
protected:
    void paintEvent(QPaintEvent*) override {
        QPainter p(this); p.setRenderHint(QPainter::Antialiasing);
        bool hov=underMouse();
        p.setBrush(hov?CBStyle::SURFACE2:CBStyle::SURFACE);
        p.setPen(QPen(hov?accent:CBStyle::BORDER,hov?1.5:1.0));
        p.drawRoundedRect(rect().adjusted(1,1,-1,-1),6,6);
        QRectF ic(6,4,64,56);
        switch(type){
        case ElementType::Source:            ElementIcon::drawSource(&p,ic); break;
        case ElementType::Aperture:
        case ElementType::ApertureWithCover: ElementIcon::drawAperture(&p,ic); break;
        case ElementType::EmptyCavity:
        case ElementType::DielectricCavity:  ElementIcon::drawCavity(&p,ic); break;
        case ElementType::Load:              ElementIcon::drawObservation(&p,ic); break;
        }
        p.setPen(accent); QFont f("Courier New",7); p.setFont(f);
        p.drawText(QRect(0,62,width(),14),Qt::AlignCenter,typeName.toUpper());
    }
    void mousePressEvent(QMouseEvent* ev) override {
        if(ev->button()!=Qt::LeftButton) return;
        QDrag* drag=new QDrag(this); QMimeData* mime=new QMimeData;
        mime->setText(QString::number((int)type)); drag->setMimeData(mime);
        QPixmap pm(size()); pm.fill(Qt::transparent); render(&pm);
        drag->setPixmap(pm); drag->setHotSpot(ev->pos()); drag->exec(Qt::CopyAction);
    }
    void enterEvent(QEnterEvent*) override { update(); }
    void leaveEvent(QEvent*)      override { update(); }
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
        bool sel=isSelected(); QColor ac=accentColor();
        if(sel){
            p->setPen(Qt::NoPen);
            p->setBrush(QColor(ac.red(),ac.green(),ac.blue(),35));
            p->drawRoundedRect(QRectF(-2,-2,W+4,H+4),6,6);
            p->setPen(QPen(ac,2)); p->setBrush(Qt::NoBrush);
            p->drawRoundedRect(QRectF(-2,-2,W+4,H+4),6,6);
        } else {
            p->setPen(QPen(CBStyle::BORDER,0.8)); p->setBrush(QColor(255,255,255,180));
            p->drawRoundedRect(QRectF(0,0,W,H),4,4);
        }
        QRectF ic(4,4,W-8,H-8);
        switch(params.type){
        case ElementType::Source:            ElementIcon::drawSource(p,ic,sel); break;
        case ElementType::Aperture:
        case ElementType::ApertureWithCover: ElementIcon::drawAperture(p,ic,sel); break;
        case ElementType::EmptyCavity:
        case ElementType::DielectricCavity:  ElementIcon::drawCavity(p,ic,sel,params.label.left(4)); break;
        case ElementType::Load:              ElementIcon::drawObservation(p,ic,sel); break;
        }
        p->setPen(QPen(ac,1)); QFont f("Courier New",7); p->setFont(f);
        p->drawText(QRectF(0,H+2,W,16),Qt::AlignCenter,params.label);
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
        QPointF a=src->pos()+QPointF(CanvasElement::W, CanvasElement::H/2.0);
        QPointF b=dst->pos()+QPointF(0,               CanvasElement::H/2.0);
        double mx=(a.x()+b.x())/2.0;
        QPainterPath path; path.moveTo(a);
        path.cubicTo({mx,a.y()},{mx,b.y()},b);
        p->setPen(QPen(CBStyle::ACCENT,1.5,Qt::DashLine)); p->setBrush(Qt::NoBrush);
        p->drawPath(path);
        p->setBrush(CBStyle::ACCENT); p->setPen(Qt::NoPen);
        p->drawEllipse(a,4,4); p->drawEllipse(b,4,4);
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
    }
};

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
        // Belt-and-suspenders: also set the locale on the object
        setLocale(QLocale::c());
    }

    // Override validate: replace comma with period so that users can
    // type either separator. Base class then validates the corrected string.
    QValidator::State validate(QString& input, int& pos) const override {
        // Replace any comma (European decimal separator) with period
        input.replace(',', '.');
        return QDoubleSpinBox::validate(input, pos);
    }

    // Override valueFromText: parse using the C locale (always '.')
    double valueFromText(const QString& text) const override {
        QString t = text;
        t.replace(',', '.');     // accept either separator
        t.remove(' ');           // strip any thousands separator
        bool ok = false;
        const double v = t.toDouble(&ok);
        return ok ? v : minimum();
    }

    // Override textFromValue: always display with '.' as separator
    QString textFromValue(double value) const override {
        return QLocale::c().toString(value, 'f', decimals());
    }
};

// ============================================================
//  BuilderPropertyPanel — context-sensitive parameter editor
// ============================================================
class BuilderPropertyPanel : public QWidget {
    Q_OBJECT
public:
    explicit BuilderPropertyPanel(QWidget* p=nullptr) : QWidget(p) {
        setMinimumWidth(220); setMaximumWidth(260);
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
            "  Use ⇌ Arrange after\n"
            "  dropping elements.\n\n"
            "  Click element to\n"
            "  edit its params.",this);
        placeholderLabel_->setStyleSheet(dimLabel()+" font-size:10px;");
        placeholderLabel_->setAlignment(Qt::AlignLeft);
        l->addWidget(placeholderLabel_); l->addStretch();
        showPlaceholder(true);
    }

signals:
    void paramsChanged(CanvasElement* el);

public slots:
    void showElement(CanvasElement* el) {
        currentElement_=el; clearForm();
        if(!el){ showPlaceholder(true); typeLabel_->setText("No element selected"); return; }
        showPlaceholder(false);
        QColor col=el->accentColor();
        typeLabel_->setText(typeToStr(el->params.type));
        typeLabel_->setStyleSheet(QString("font-family:'Courier New';font-size:11px;font-weight:bold;"
                                          "color:rgb(%1,%2,%3);").arg(col.red()).arg(col.green()).arg(col.blue()));
        addLineEdit("Label:",el->params.label,[=](const QString& v){currentElement_->params.label=v;currentElement_->update();});
        switch(el->params.type){
        case ElementType::Source:
            addDouble("E₀ [V/m]:",    el->params.E0,        0.01,1000,0.1,  [=](double v){currentElement_->params.E0=v;});
            // [FIX-B1] min=0.0001 GHz = 100 kHz — allows Phase A's 1 MHz start
            addDouble("f start [GHz]:",el->params.freqStart, 0.0001,100, 0.001,  [=](double v){currentElement_->params.freqStart=v;});
            addDouble("f end [GHz]:",  el->params.freqEnd,   0.1,100, 1.0,  [=](double v){currentElement_->params.freqEnd=v;});
            addInt   ("Points:",       el->params.freqPoints,10, 2000,50,    [=](int v)   {currentElement_->params.freqPoints=v;});
            addSep("Cross-section (shared by all):");
            addDouble("a [m]:",       el->params.a,      0.001,1.0,0.005,   [=](double v){currentElement_->params.a=v;});
            addDouble("b [m]:",       el->params.b_h,    0.001,1.0,0.005,   [=](double v){currentElement_->params.b_h=v;});
            // step=0.0001 m (0.1mm) gives fine control; min=0.0001 m (0.1mm)
            addDouble("t_wall [m]:",  el->params.t_wall, 0.0001,0.1,0.0001, [=](double v){currentElement_->params.t_wall=v;});
            break;
        case ElementType::Aperture:
            addDouble("l_slot [m]:",el->params.l_slot,0.001,1.0,0.002,  [=](double v){currentElement_->params.l_slot=v;});
            addDouble("w_slot [m]:",el->params.w_slot,0.0001,0.1,0.0005,[=](double v){currentElement_->params.w_slot=v;});
            break;
        case ElementType::ApertureWithCover:
            addDouble("l_slot [m]:", el->params.l_slot,    0.001,1.0,0.002,   [=](double v){currentElement_->params.l_slot=v;});
            addDouble("w_slot [m]:", el->params.w_slot,    0.0001,0.1,0.0005, [=](double v){currentElement_->params.w_slot=v;});
            addDouble("τ gap [m]:",  el->params.tau_cover,0.0001,0.01,0.0001, [=](double v){currentElement_->params.tau_cover=v;});
            break;
        case ElementType::EmptyCavity:
            addDouble("L [m]:",el->params.L_cavity,0.001,2.0,0.01,[=](double v){currentElement_->params.L_cavity=v;});
            break;
        case ElementType::DielectricCavity:
            addDouble("L [m]:",      el->params.L_cavity,    0.001,2.0,0.01,   [=](double v){currentElement_->params.L_cavity=v;});
            addDouble("h_diel [m]:", el->params.h_dielectric,0.0001,1.0,0.001, [=](double v){currentElement_->params.h_dielectric=v;});
            addDouble("ε_r:",        el->params.eps_r,        1.0,100.0,0.5,   [=](double v){currentElement_->params.eps_r=v;});
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
            addDouble("Z_L real [Ω]:",el->params.ZL_real, 0.001,1.0e10,1.0e6,
                      [=](double v){currentElement_->params.ZL_real=v;});
            addDouble("Z_L imag [Ω]:",el->params.ZL_imag,-1.0e9,1.0e9,1.0e3,
                      [=](double v){currentElement_->params.ZL_imag=v;});
            break;
        }
    }

private:
    QLabel* headerLabel_; QLabel* typeLabel_; QLabel* placeholderLabel_;
    QScrollArea* scrollArea_; QWidget* formWidget_; QFormLayout* formLayout_;
    CanvasElement* currentElement_{nullptr};

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
    void clearForm(){ while(formLayout_->rowCount()>0) formLayout_->removeRow(0); }
    void showPlaceholder(bool s){ scrollArea_->setVisible(!s); placeholderLabel_->setVisible(s); }

    QString spinSS(QColor col) {
        return QString("QDoubleSpinBox,QSpinBox,QLineEdit{"
                       "background:rgb(%1,%2,%3);color:rgb(%4,%5,%6);"
                       "border:1px solid rgb(%7,%8,%9);border-radius:4px;padding:3px 6px;"
                       "font-family:'Courier New';font-size:11px;}"
                       "QDoubleSpinBox:focus,QSpinBox:focus,QLineEdit:focus{"
                       "border:1px solid rgb(%10,%11,%12);}")
            .arg(CBStyle::BG.red()).arg(CBStyle::BG.green()).arg(CBStyle::BG.blue())
            .arg(CBStyle::TEXT.red()).arg(CBStyle::TEXT.green()).arg(CBStyle::TEXT.blue())
            .arg(CBStyle::BORDER.red()).arg(CBStyle::BORDER.green()).arg(CBStyle::BORDER.blue())
            .arg(col.red()).arg(col.green()).arg(col.blue());
    }
    QString lblSS() {
        return QString("color:rgb(%1,%2,%3);font-family:'Courier New';font-size:10px;")
        .arg(CBStyle::TEXT_MUTED.red()).arg(CBStyle::TEXT_MUTED.green()).arg(CBStyle::TEXT_MUTED.blue());
    }
    void addDouble(const QString& l, double v, double mn, double mx, double step, std::function<void(double)> fn) {
        // [FIX-B1] CLocaleDoubleSpinBox overrides validate()/valueFromText()/textFromValue()
        // to guarantee '.' decimal separator on all platforms, including Russian/European
        // Windows where Qt's default validator silently rejects typed '.' values.
        auto* s = new CLocaleDoubleSpinBox(formWidget_);
        s->setRange(mn,mx); s->setValue(v); s->setSingleStep(step); s->setDecimals(6);
        QColor c=currentElement_?currentElement_->accentColor():CBStyle::ACCENT; s->setStyleSheet(spinSS(c));
        connect(s,QOverload<double>::of(&QDoubleSpinBox::valueChanged),this,[=](double val){fn(val);emit paramsChanged(currentElement_);});
        auto* ll=new QLabel(l,formWidget_); ll->setStyleSheet(lblSS()); formLayout_->addRow(ll,s);
    }
    void addInt(const QString& l, int v, int mn, int mx, int step, std::function<void(int)> fn) {
        auto* s=new QSpinBox(formWidget_);
        s->setLocale(QLocale::c());   // [FIX-B1] consistent locale
        s->setRange(mn,mx); s->setValue(v); s->setSingleStep(step);
        QColor c=currentElement_?currentElement_->accentColor():CBStyle::ACCENT; s->setStyleSheet(spinSS(c));
        connect(s,QOverload<int>::of(&QSpinBox::valueChanged),this,[=](int val){fn(val);emit paramsChanged(currentElement_);});
        auto* ll=new QLabel(l,formWidget_); ll->setStyleSheet(lblSS()); formLayout_->addRow(ll,s);
    }
    void addLineEdit(const QString& l, const QString& v, std::function<void(const QString&)> fn) {
        auto* e=new QLineEdit(formWidget_); e->setText(v);
        QColor c=currentElement_?currentElement_->accentColor():CBStyle::ACCENT; e->setStyleSheet(spinSS(c));
        connect(e,&QLineEdit::textChanged,this,[=](const QString& val){fn(val);emit paramsChanged(currentElement_);});
        auto* ll=new QLabel(l,formWidget_); ll->setStyleSheet(lblSS()); formLayout_->addRow(ll,e);
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
    AssemblyCanvas*       canvas_   {nullptr};
    BuilderPropertyPanel* propPanel_{nullptr};
    QCustomPlot*          m_plot    {nullptr};
    QLabel*               statusLbl_{nullptr};

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
    // ============================================================
    void applyGlobalStyle() {
        setStyleSheet(QString(R"(
            QMainWindow,QWidget{background:rgb(%1,%2,%3);color:rgb(%4,%5,%6);font-family:'Courier New',monospace;}
            QToolBar{background:rgb(%7,%8,%9);border-bottom:1px solid rgb(%10,%11,%12);spacing:6px;padding:4px 8px;}
            QStatusBar{background:rgb(%7,%8,%9);border-top:1px solid rgb(%10,%11,%12);color:rgb(%13,%14,%15);font-size:11px;}
            QPushButton{border-radius:5px;padding:5px 14px;font-family:'Courier New';font-size:11px;}
            QPushButton:disabled{opacity:0.4;}
            QSplitter::handle{background:rgb(%10,%11,%12);width:1px;height:1px;}
        )")
                          .arg(CBStyle::BG.red())    .arg(CBStyle::BG.green())    .arg(CBStyle::BG.blue())
                          .arg(CBStyle::TEXT.red())  .arg(CBStyle::TEXT.green())  .arg(CBStyle::TEXT.blue())
                          .arg(CBStyle::SURFACE.red()).arg(CBStyle::SURFACE.green()).arg(CBStyle::SURFACE.blue())
                          .arg(CBStyle::BORDER.red()) .arg(CBStyle::BORDER.green()) .arg(CBStyle::BORDER.blue())
                          .arg(CBStyle::TEXT_MUTED.red()).arg(CBStyle::TEXT_MUTED.green()).arg(CBStyle::TEXT_MUTED.blue()));
    }

    // ============================================================
    //  UI construction
    // ============================================================
    void buildUI() {
        // ── Toolbar ──────────────────────────────────────────────
        QToolBar* tb = addToolBar("Palette");
        tb->setMovable(false); tb->setIconSize({40,40});

        auto* titleLbl = new QLabel(
            QString("  EM<span style='color:rgb(%1,%2,%3)'>Shield</span>"
                    "<b style='color:rgb(%4,%5,%6)'>Builder</b>  ")
                .arg(CBStyle::TEXT_MUTED.red()).arg(CBStyle::TEXT_MUTED.green()).arg(CBStyle::TEXT_MUTED.blue())
                .arg(CBStyle::ACCENT.red())    .arg(CBStyle::ACCENT.green())    .arg(CBStyle::ACCENT.blue()), tb);
        titleLbl->setTextFormat(Qt::RichText);
        titleLbl->setStyleSheet("font-size:13px;letter-spacing:1px;");
        tb->addWidget(titleLbl);

        // Separator
        auto* vsep = new QFrame(tb); vsep->setFrameShape(QFrame::VLine);
        vsep->setStyleSheet(QString("background:rgb(%1,%2,%3);max-width:1px;margin:4px 6px;")
                                .arg(CBStyle::BORDER.red()).arg(CBStyle::BORDER.green()).arg(CBStyle::BORDER.blue()));
        tb->addWidget(vsep);

        // Palette buttons
        struct PS { ElementType t; QString n; QColor c; };
        for(auto& s : QVector<PS>{
                                   { ElementType::Source,            "Source",   CBStyle::ORANGE },
                                   { ElementType::Aperture,          "Aperture", CBStyle::ACCENT },
                                   { ElementType::ApertureWithCover, "AP+Cover", CBStyle::ACCENT },
                                   { ElementType::EmptyCavity,       "Cavity",   CBStyle::GREEN  },
                                   { ElementType::DielectricCavity,  "Diel.Cav", CBStyle::GREEN  },
                                   { ElementType::Load,              "Obs.Pt",   CBStyle::RED    }})
            tb->addWidget(new PaletteButton(s.t, s.n, s.c, tb));

        auto* sp = new QWidget(tb); sp->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
        tb->addWidget(sp);

        // Action buttons
        auto* bArr = makeToolBtn("⇌ Arrange",   CBStyle::TEXT_MUTED);
        auto* bDel = makeToolBtn("✕ Delete",    CBStyle::RED);
        auto* bClr = makeToolBtn("⊘ Clear",     CBStyle::ORANGE);
        auto* bCmp = makeToolBtn("⚡ Compute",   CBStyle::GREEN, true);
        auto* bExp = makeToolBtn("⬇ Export CSV", CBStyle::TEXT_MUTED);
        tb->addWidget(bArr); tb->addWidget(bDel); tb->addWidget(bClr);
        tb->addWidget(bCmp); tb->addWidget(bExp);
        connect(bArr, &QPushButton::clicked, this, [this]{ canvas_->autoArrange(); });
        connect(bDel, &QPushButton::clicked, this, [this]{ canvas_->removeSelected(); });
        connect(bClr, &QPushButton::clicked, this, [this]{
            canvas_->clearAll();
            clearPlot();
            setStatus("Canvas cleared — drag elements to build a new circuit.", CBStyle::TEXT_MUTED);
        });
        connect(bCmp, &QPushButton::clicked, this, &CircuitBuilderWindow::runCompute);
        connect(bExp, &QPushButton::clicked, this, &CircuitBuilderWindow::onExportCSV);

        // ── Central layout: splitter ──────────────────────────────
        auto* mainSpl = new QSplitter(Qt::Horizontal, this);

        propPanel_ = new BuilderPropertyPanel(mainSpl);
        mainSpl->addWidget(propPanel_);

        auto* rightSpl = new QSplitter(Qt::Vertical, mainSpl);
        canvas_ = new AssemblyCanvas(rightSpl);
        rightSpl->addWidget(canvas_);

        // QCustomPlot — same setup as Phase A (mainwindow.cpp)
        m_plot = new QCustomPlot(rightSpl);
        setupPlot();
        rightSpl->addWidget(m_plot);
        rightSpl->setSizes({400, 320});

        mainSpl->addWidget(rightSpl);
        mainSpl->setSizes({245, 1195});
        mainSpl->setHandleWidth(2);
        setCentralWidget(mainSpl);

        // ── Status bar ────────────────────────────────────────────
        statusLbl_ = new QLabel(
            "Ready  —  correct order: "
            "[Source]→[Aperture]→[Cavity(p)]→[Obs.Pt]→[Cavity(d-p)]  |  "
            "Last Cavity auto-terminates to ground  |  ⚡ Compute");
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

    QPushButton* makeToolBtn(const QString& text, QColor col, bool bold=false) {
        auto* b = new QPushButton(text);
        b->setStyleSheet(QString(
                             "QPushButton{color:rgb(%1,%2,%3);background:rgb(%4,%5,%6);"
                             "border:1px solid rgb(%1,%2,%3);font-weight:%7;padding:5px 14px;border-radius:5px;}"
                             "QPushButton:hover{background:rgba(%1,%2,%3,25);border:1.5px solid rgb(%1,%2,%3);}"
                             "QPushButton:pressed{background:rgba(%1,%2,%3,45);}")
                             .arg(col.red()).arg(col.green()).arg(col.blue())
                             .arg(CBStyle::SURFACE.red()).arg(CBStyle::SURFACE.green()).arg(CBStyle::SURFACE.blue())
                             .arg(bold?"700":"400"));
        return b;
    }

    void connectSignals() {
        connect(canvas_, &AssemblyCanvas::elementDropped, this,
                [this](ElementType t, QPointF pos){
                    auto* el = canvas_->addElement(t, pos);
                    propPanel_->showElement(el);
                    setStatus(
                        QString("Added: %1  |  Arrange left→right: "
                                "Source → Aperture → Cavity → Obs.Pt").arg(el->params.label),
                        CBStyle::ACCENT);
                });
        connect(canvas_, &AssemblyCanvas::elementSelected,  this,
                [this](CanvasElement* el){ propPanel_->showElement(el); });
        connect(canvas_, &AssemblyCanvas::circuitChanged,   this,
                [this]{ clearPlot(); });
        connect(propPanel_, &BuilderPropertyPanel::paramsChanged, this,
                [this](CanvasElement* el){ if(el) el->update(); });
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
        auto& els = canvas_->elements;
        if(els.size() < 3){
            setStatus("❌  Minimum circuit: Source → Aperture → Cavity → Obs.Pt", CBStyle::RED);
            return;
        }

        // Sort elements by X position to define circuit order
        QVector<CanvasElement*> ordered = els;
        std::sort(ordered.begin(), ordered.end(),
                  [](CanvasElement* a, CanvasElement* b){ return a->pos().x() < b->pos().x(); });

        // Validate: leftmost must be Source
        if(ordered.first()->params.type != ElementType::Source){
            setStatus("❌  Leftmost element must be Source.  "
                      "Use ⇌ Arrange then drag Source to the left.", CBStyle::RED);
            return;
        }

        // Validate: at least one Obs.Pt exists
        const bool hasObs = std::any_of(ordered.begin(), ordered.end(),
                                        [](CanvasElement* e){ return e->params.type==ElementType::Load; });
        if(!hasObs){
            setStatus("❌  Add at least one Obs.Pt element to the circuit.", CBStyle::RED);
            return;
        }

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
            // Z_ap connects from node nFrom to ground (node 0).
            // nodeIdx NOT incremented: the main circuit path continues
            // from the SAME node after the aperture shunt.
            // This matches Eq. 3.13 and the A-matrix column 2 of Eq. 3.10:
            //   A[0][1] = +1  (only appears in row of node nFrom → ground).
            case ElementType::Aperture:
                solver.addBranch(std::make_shared<AP_SlotAperture>(
                    nFrom, 0, bid,     // [FIX-B3] nTo=0 (ground), was nodeIdx+1
                    a_g, b_g, ep.l_slot, ep.w_slot, t_g));
                // [FIX-B3] no ++nodeIdx — circuit path continues from nFrom
                break;

            // ── Aperture With Cover — also SHUNT ────────────────────────
            case ElementType::ApertureWithCover:
                solver.addBranch(std::make_shared<AP_SlotWithCover>(
                    nFrom, 0, bid,     // [FIX-B3] nTo=0 (ground)
                    a_g, b_g, ep.l_slot, ep.w_slot, t_g, ep.tau_cover));
                // [FIX-B3] no ++nodeIdx
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
            // nodeIdx NOT incremented — circuit continues from nFrom
            case ElementType::Load:
                solver.addBranch(std::make_shared<LOAD_Impedance>(
                    nFrom, 0, bid,
                    Complex(ep.ZL_real, ep.ZL_imag)));
                ++obsCount;   // [FIX-D2] increment BEFORE building label
                obsNodes.append(nFrom);
                obsLabels.append(ep.label.isEmpty()
                                     ? QString("P%1").arg(obsCount)
                                     : ep.label);
                break;
            }
        }

        if(obsNodes.isEmpty()){
            setStatus("❌  No observation nodes built (internal error).", CBStyle::RED);
            return;
        }

        // ── Auto short-circuit termination ────────────────────────────────────────
        //
        // The closed metallic back wall = short circuit (V = 0).
        // We add a near-short (Z = 1e-4 Ω) at the final series node.
        //
        // [FIX-D3] CRITICAL: do NOT add the short if nodeIdx is already
        //  an observation node.  Adding a 1e-4 Ω shunt at the SAME node
        //  as an Obs.Pt shunts V_obs ≈ 0 → SE → +∞ (the 110–172 dB symptom).
        //
        //  This collision only occurs in the 4-element circuit
        //  (Source→Aperture→Cavity→Obs.Pt).  In the correct 5-element
        //  circuit (…→Obs.Pt→Cavity_dp) nodeIdx points one node PAST
        //  the observation node, so there is no collision.
        {
            auto lastSeries = std::find_if(
                ordered.rbegin(), ordered.rend(),
                [](CanvasElement* e){ return e->params.type != ElementType::Load; });

            const bool isCavityTerminated =
                (lastSeries != ordered.rend()) &&
                ((*lastSeries)->params.type == ElementType::EmptyCavity ||
                 (*lastSeries)->params.type == ElementType::DielectricCavity);

            // [FIX-D3] guard: only add short when nodeIdx is not an obs node
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
            QString("✓  %1 pts · %2 curve(s) · SE: %3…%4 dB · %5–%6 GHz")
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
    // ============================================================
    void onExportCSV()
    {
        if(m_freqs.isEmpty()){
            QMessageBox::warning(this, "No Data", "Run ⚡ Compute first.");
            return;
        }
        const QString fn = QFileDialog::getSaveFileName(
            this, "Export CSV", "SE_builder.csv", "CSV (*.csv)");
        if(fn.isEmpty()) return;

        std::ofstream f(fn.toStdString());
        if(!f.is_open()){
            QMessageBox::critical(this, "Error", "Cannot open file for writing.");
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
    }

    void setStatus(const QString& msg, QColor col=CBStyle::TEXT_MUTED){
        statusLbl_->setText(msg);
        statusLbl_->setStyleSheet(QString("color:rgb(%1,%2,%3);")
                                      .arg(col.red()).arg(col.green()).arg(col.blue()));
    }
};
