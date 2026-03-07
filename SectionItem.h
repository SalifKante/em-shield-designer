#ifndef SECTIONITEM_H
#define SECTIONITEM_H

#include <QGraphicsItem>
#include <QPainter>
#include <QGraphicsSceneMouseEvent>
#include <QStyleOptionGraphicsItem>
#include <QString>
#include <cmath>

// ============================================================================
// SECTION ITEM DATA
// ============================================================================
// Plain data mirror of EMCore::SectionConfig, expressed in GUI-friendly units
// (millimetres, dimensionless permittivities).
//
// CircuitGenerator / mainwindow.cpp converts from mm to metres before
// building the EnclosureConfig.
//
// Field correspondence with SectionConfig (include/core/CircuitGenerator.h):
//
//   depth_mm          ↔  depth           [mm → m]
//   obs_position_mm   ↔  obs_position    [mm → m]   must be in (0, depth_mm)
//   has_observation   ↔  has_observation
//   section_width_a_mm↔  section_width_a [mm → m]   ≤ 0 → use global a
//   aperture_l_mm     ↔  aperture_l      [mm → m]
//   aperture_w_mm     ↔  aperture_w      [mm → m]
//   has_cover         ↔  has_cover
//   cover_gap_mm      ↔  cover_gap       [mm → m]
//   cover_eps_r       ↔  cover_eps_r                 1.0 = air (Eq. 3.22)
//                                                     >1.0 = dielectric (Eq. 3.24)
//   has_dielectric    ↔  has_dielectric
//   dielectric_h_mm   ↔  dielectric_h    [mm → m]
//   dielectric_er     ↔  dielectric_er               must be ≥ 1.0

struct SectionItemData {

    // --- Cavity ---
    double depth_mm           = 300.0;   ///< Section depth [mm]
    double obs_position_mm    = 150.0;   ///< Observation point offset [mm]
    ///< Must be strictly inside (0, depth_mm)
    bool   has_observation    = true;

    // --- Per-section width override (STAR_BRANCH topology) ---
    // If ≤ 0, the global EnclosureConfig::a is used.
    // Set a positive value for STAR_BRANCH sub-chambers that differ in width
    // from the spine (Figure 3.11 — sections 2, 3 may have a₂ ≠ a₁).
    double section_width_a_mm = -1.0;    ///< Override enclosure broad dim [mm]

    // --- Aperture ---
    double aperture_l_mm      = 80.0;   ///< Aperture width  l [mm]
    double aperture_w_mm      = 80.0;   ///< Aperture height w [mm]

    // --- Cover (Eqs. 3.22 / 3.24) ---
    bool   has_cover          = false;
    double cover_gap_mm       = 1.0;    ///< Gap thickness τ [mm]
    double cover_eps_r        = 1.0;    ///< Gap filler εᵣ
    ///< 1.0 → air gap  (Eq. 3.22)
    ///< >1.0 → dielectric gap (Eq. 3.24)

    // --- Dielectric fill (Eqs. 3.19 – 3.21) ---
    bool   has_dielectric     = false;
    double dielectric_h_mm    = 0.0;    ///< Dielectric layer thickness [mm]
    double dielectric_er      = 1.0;    ///< Relative permittivity (≥ 1.0)
};

// ============================================================================
// SECTION ITEM — QGraphicsItem for the circuit canvas
// ============================================================================

class SectionItem : public QGraphicsItem
{
public:
    static constexpr double BOX_WIDTH      = 120.0;
    static constexpr double BOX_HEIGHT     =  80.0;
    static constexpr double SPACING        =   8.0;
    static constexpr double APERTURE_WIDTH =   6.0;

    explicit SectionItem(int sectionIndex, QGraphicsItem* parent = nullptr)
        : QGraphicsItem(parent)
        , m_index(sectionIndex)
    {
        setFlag(QGraphicsItem::ItemIsSelectable, true);
        setFlag(QGraphicsItem::ItemSendsGeometryChanges, true);
        setAcceptHoverEvents(true);
        updateTooltip();
    }

    // -----------------------------------------------------------------------
    // Data access
    // -----------------------------------------------------------------------
    int  sectionIndex()  const { return m_index; }
    void setSectionIndex(int idx) { m_index = idx; updateTooltip(); update(); }

    const SectionItemData& data() const { return m_data; }
    void setData(const SectionItemData& d) { m_data = d; updateTooltip(); update(); }

    // -----------------------------------------------------------------------
    // QGraphicsItem interface
    // -----------------------------------------------------------------------
    QRectF boundingRect() const override {
        return QRectF(-2, -2, BOX_WIDTH + APERTURE_WIDTH + 4,
                      BOX_HEIGHT + 4);
    }

    void paint(QPainter* painter,
               const QStyleOptionGraphicsItem* option,
               QWidget* /*widget*/) override
    {
        const bool selected = option->state & QStyle::State_Selected;
        const bool hovered  = option->state & QStyle::State_MouseOver;

        painter->setRenderHint(QPainter::Antialiasing, true);

        // ---- Aperture slot (left edge) ------------------------------------
        const double ap_h = BOX_HEIGHT * 0.6;
        const double ap_y = (BOX_HEIGHT - ap_h) / 2.0;

        QColor apColor = m_data.has_cover
                             ? QColor(180, 80, 80)
                             : QColor(80, 80, 180);
        painter->setPen(Qt::NoPen);
        painter->setBrush(apColor);
        painter->drawRect(QRectF(0.0, ap_y, APERTURE_WIDTH, ap_h));

        painter->setPen(Qt::white);
        painter->setFont(QFont("sans-serif", 6));
        painter->drawText(QRectF(0.0, ap_y, APERTURE_WIDTH, ap_h),
                          Qt::AlignCenter,
                          m_data.has_cover ? "C" : "A");

        // ---- Cavity box ---------------------------------------------------
        const QRectF boxRect(APERTURE_WIDTH, 0.0, BOX_WIDTH, BOX_HEIGHT);

        if (m_data.has_dielectric) {
            QLinearGradient grad(APERTURE_WIDTH, 0.0,
                                 APERTURE_WIDTH, BOX_HEIGHT);
            double ratio = std::min(
                m_data.dielectric_h_mm / std::max(1.0, m_data.depth_mm), 1.0);
            grad.setColorAt(0.0,            QColor(240, 245, 255));
            grad.setColorAt(1.0 - ratio,    QColor(240, 245, 255));
            grad.setColorAt(
                std::min(1.0 - ratio + 0.01, 1.0), QColor(200, 220, 255));
            grad.setColorAt(1.0,            QColor(180, 200, 240));
            painter->setBrush(grad);
        } else {
            painter->setBrush(hovered
                                  ? QColor(245, 248, 255)
                                  : QColor(250, 252, 255));
        }

        QPen borderPen;
        if (selected)      borderPen = QPen(QColor( 37,  99, 235), 2.5);
        else if (hovered)  borderPen = QPen(QColor(100, 140, 200), 1.5);
        else               borderPen = QPen(QColor(120, 140, 170), 1.2);
        painter->setPen(borderPen);
        painter->drawRoundedRect(boxRect, 4, 4);

        // ---- Section label ------------------------------------------------
        painter->setPen(QColor(40, 50, 70));
        painter->setFont(QFont("sans-serif", 10, QFont::Bold));
        painter->drawText(boxRect.adjusted(0, 6, 0, 0),
                          Qt::AlignHCenter | Qt::AlignTop,
                          QString("S%1").arg(m_index + 1));

        // ---- Depth text ---------------------------------------------------
        painter->setPen(QColor(80, 100, 130));
        painter->setFont(QFont("sans-serif", 7));
        painter->drawText(boxRect.adjusted(4, 0, -4, -4),
                          Qt::AlignHCenter | Qt::AlignBottom,
                          QString("%1 mm").arg(m_data.depth_mm, 0, 'f', 0));

        // ---- Aperture dimensions ------------------------------------------
        painter->drawText(boxRect.adjusted(4, 22, -4, 0),
                          Qt::AlignHCenter | Qt::AlignTop,
                          QString("Ap: %1×%2")
                              .arg(m_data.aperture_l_mm, 0, 'f', 0)
                              .arg(m_data.aperture_w_mm, 0, 'f', 0));

        // ---- Observation dot ----------------------------------------------
        if (m_data.has_observation) {
            double obs_ratio = m_data.obs_position_mm
                               / std::max(1.0, m_data.depth_mm);
            obs_ratio = std::clamp(obs_ratio, 0.05, 0.95);
            const double obs_x = APERTURE_WIDTH + obs_ratio * BOX_WIDTH;
            const double obs_y = BOX_HEIGHT / 2.0;

            painter->setPen(Qt::NoPen);
            painter->setBrush(QColor(220, 38, 38));
            painter->drawEllipse(QPointF(obs_x, obs_y), 5.0, 5.0);

            painter->setPen(Qt::white);
            painter->setFont(QFont("sans-serif", 6, QFont::Bold));
            painter->drawText(QRectF(obs_x - 5, obs_y - 5, 10, 10),
                              Qt::AlignCenter,
                              QString("P%1").arg(m_index + 1));
        }

        // ---- Dielectric εr label ------------------------------------------
        if (m_data.has_dielectric) {
            painter->setPen(QColor(100, 120, 180));
            painter->setFont(QFont("sans-serif", 6));
            painter->drawText(boxRect.adjusted(4, 36, -4, 0),
                              Qt::AlignHCenter | Qt::AlignTop,
                              QString("εr=%1")
                                  .arg(m_data.dielectric_er, 0, 'f', 1));
        }

        // ---- Cover εr label (when dielectric gap is active) ---------------
        if (m_data.has_cover && m_data.cover_eps_r > 1.0 + 1e-9) {
            painter->setPen(QColor(160, 60, 60));
            painter->setFont(QFont("sans-serif", 6));
            // Draw small label on the aperture slot
            painter->drawText(QRectF(0.0, ap_y - 10, APERTURE_WIDTH + 20, 10),
                              Qt::AlignLeft | Qt::AlignVCenter,
                              QString("ε%1").arg(m_data.cover_eps_r, 0, 'f', 1));
        }

        // ---- Selection glow -----------------------------------------------
        if (selected) {
            painter->setPen(Qt::NoPen);
            painter->setBrush(QColor(37, 99, 235, 20));
            painter->drawRoundedRect(boxRect.adjusted(-3, -3, 3, 3), 6, 6);
        }
    }

    QPainterPath shape() const override {
        QPainterPath path;
        path.addRect(QRectF(0, 0, APERTURE_WIDTH + BOX_WIDTH, BOX_HEIGHT));
        return path;
    }

protected:
    void hoverEnterEvent(QGraphicsSceneHoverEvent* e) override
    { update(); QGraphicsItem::hoverEnterEvent(e); }
    void hoverLeaveEvent(QGraphicsSceneHoverEvent* e) override
    { update(); QGraphicsItem::hoverLeaveEvent(e); }

private:
    void updateTooltip() {
        QString tip =
            QString("Section %1\n"
                    "Depth:    %2 mm\n"
                    "Obs pos:  %3 mm\n"
                    "Aperture: %4 × %5 mm")
                .arg(m_index + 1)
                .arg(m_data.depth_mm,        0, 'f', 1)
                .arg(m_data.obs_position_mm, 0, 'f', 1)
                .arg(m_data.aperture_l_mm,   0, 'f', 1)
                .arg(m_data.aperture_w_mm,   0, 'f', 1);

        if (m_data.has_cover) {
            tip += QString("\nCover gap: %1 mm")
            .arg(m_data.cover_gap_mm, 0, 'f', 2);
            if (m_data.cover_eps_r > 1.0 + 1e-9)
                tip += QString("  εr=%1 (Eq.3.24)")
                           .arg(m_data.cover_eps_r, 0, 'f', 2);
            else
                tip += "  (air, Eq.3.22)";
        }
        if (m_data.has_dielectric)
            tip += QString("\nDielectric: εr=%1, h=%2 mm")
                       .arg(m_data.dielectric_er,  0, 'f', 2)
                       .arg(m_data.dielectric_h_mm, 0, 'f', 1);
        if (m_data.section_width_a_mm > 0.0)
            tip += QString("\nWidth override: a=%1 mm")
                       .arg(m_data.section_width_a_mm, 0, 'f', 1);

        setToolTip(tip);
    }

    int             m_index;
    SectionItemData m_data;
};

#endif // SECTIONITEM_H
