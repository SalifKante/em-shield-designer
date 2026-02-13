#ifndef SECTIONITEM_H
#define SECTIONITEM_H

#include <QGraphicsItem>
#include <QPainter>
#include <QGraphicsSceneMouseEvent>
#include <QStyleOptionGraphicsItem>
#include <QString>
#include <cmath>

// Forward-declare to avoid pulling in the full engine header in the .h
// The actual SectionConfig struct is used by value in a simplified mirror here.
// CircuitCanvas will handle the conversion to EMCore::SectionConfig.

struct SectionItemData {
    double depth_mm        = 300.0;
    double obs_position_mm = 150.0;
    bool   has_observation = true;

    double aperture_l_mm   = 80.0;
    double aperture_w_mm   = 80.0;

    bool   has_cover       = false;
    double cover_gap_mm    = 1.0;

    bool   has_dielectric  = false;
    double dielectric_h_mm = 0.0;
    double dielectric_er   = 1.0;
};

class SectionItem : public QGraphicsItem
{
public:
    // Visual constants
    static constexpr double BOX_WIDTH  = 120.0;   // px
    static constexpr double BOX_HEIGHT = 80.0;    // px
    static constexpr double SPACING    = 8.0;     // px between sections
    static constexpr double APERTURE_WIDTH = 6.0; // px for aperture slot drawing

    SectionItem(int sectionIndex, QGraphicsItem* parent = nullptr)
        : QGraphicsItem(parent)
        , m_index(sectionIndex)
    {
        setFlag(QGraphicsItem::ItemIsSelectable, true);
        setFlag(QGraphicsItem::ItemSendsGeometryChanges, true);
        setAcceptHoverEvents(true);
        updateTooltip();
    }

    // --- Data access ---
    int sectionIndex() const { return m_index; }
    void setSectionIndex(int idx) { m_index = idx; updateTooltip(); update(); }

    const SectionItemData& data() const { return m_data; }
    void setData(const SectionItemData& d) { m_data = d; updateTooltip(); update(); }

    // --- QGraphicsItem interface ---
    QRectF boundingRect() const override {
        double w = BOX_WIDTH + APERTURE_WIDTH + 4;
        double h = BOX_HEIGHT + 4;
        return QRectF(-2, -2, w, h);
    }

    void paint(QPainter* painter, const QStyleOptionGraphicsItem* option,
               QWidget* /*widget*/) override
    {
        bool selected = (option->state & QStyle::State_Selected);
        bool hovered  = (option->state & QStyle::State_MouseOver);

        painter->setRenderHint(QPainter::Antialiasing, true);

        // --- Aperture slot (left edge) ---
        double ap_x = 0;
        double ap_h = BOX_HEIGHT * 0.6;  // Visual aperture height
        double ap_y = (BOX_HEIGHT - ap_h) / 2.0;

        QColor apColor = m_data.has_cover ? QColor(180, 80, 80) : QColor(80, 80, 180);
        painter->setPen(Qt::NoPen);
        painter->setBrush(apColor);
        painter->drawRect(QRectF(ap_x, ap_y, APERTURE_WIDTH, ap_h));

        // Aperture label
        painter->setPen(Qt::white);
        QFont smallFont("sans-serif", 6);
        painter->setFont(smallFont);
        painter->drawText(QRectF(ap_x, ap_y, APERTURE_WIDTH, ap_h),
                          Qt::AlignCenter, m_data.has_cover ? "C" : "A");

        // --- Main box (cavity) ---
        double box_x = APERTURE_WIDTH;
        QRectF boxRect(box_x, 0, BOX_WIDTH, BOX_HEIGHT);

        // Fill
        QColor fillColor;
        if (m_data.has_dielectric) {
            // Gradient for dielectric
            QLinearGradient grad(box_x, 0, box_x, BOX_HEIGHT);
            double ratio = m_data.dielectric_h_mm /
                           std::max(1.0, m_data.depth_mm);  // Approximate visual
            ratio = std::min(ratio, 1.0);
            grad.setColorAt(0.0, QColor(240, 245, 255));
            grad.setColorAt(1.0 - ratio, QColor(240, 245, 255));
            grad.setColorAt(1.0 - ratio + 0.01, QColor(200, 220, 255));
            grad.setColorAt(1.0, QColor(180, 200, 240));
            painter->setBrush(grad);
        } else {
            fillColor = hovered ? QColor(245, 248, 255) : QColor(250, 252, 255);
            painter->setBrush(fillColor);
        }

        // Border
        QPen borderPen;
        if (selected) {
            borderPen = QPen(QColor(37, 99, 235), 2.5);
        } else if (hovered) {
            borderPen = QPen(QColor(100, 140, 200), 1.5);
        } else {
            borderPen = QPen(QColor(120, 140, 170), 1.2);
        }
        painter->setPen(borderPen);
        painter->drawRoundedRect(boxRect, 4, 4);

        // --- Section label ---
        painter->setPen(QColor(40, 50, 70));
        QFont labelFont("sans-serif", 10, QFont::Bold);
        painter->setFont(labelFont);
        QString label = QString("S%1").arg(m_index + 1);
        painter->drawText(boxRect.adjusted(0, 6, 0, 0),
                          Qt::AlignHCenter | Qt::AlignTop, label);

        // --- Dimensions text ---
        QFont dimFont("sans-serif", 7);
        painter->setFont(dimFont);
        painter->setPen(QColor(80, 100, 130));

        QString dimText = QString("%1 mm").arg(m_data.depth_mm, 0, 'f', 0);
        painter->drawText(boxRect.adjusted(4, 0, -4, -4),
                          Qt::AlignHCenter | Qt::AlignBottom, dimText);

        // --- Aperture dimensions ---
        QString apText = QString("Ap: %1x%2")
                             .arg(m_data.aperture_l_mm, 0, 'f', 0)
                             .arg(m_data.aperture_w_mm, 0, 'f', 0);
        painter->drawText(boxRect.adjusted(4, 22, -4, 0),
                          Qt::AlignHCenter | Qt::AlignTop, apText);

        // --- Observation point (red dot) ---
        if (m_data.has_observation) {
            // Position observation dot proportionally within the box
            double obs_ratio = m_data.obs_position_mm /
                               std::max(1.0, m_data.depth_mm);
            obs_ratio = std::min(std::max(obs_ratio, 0.05), 0.95);
            double obs_x = box_x + obs_ratio * BOX_WIDTH;
            double obs_y = BOX_HEIGHT / 2.0;

            // Dot
            painter->setPen(Qt::NoPen);
            painter->setBrush(QColor(220, 38, 38));
            painter->drawEllipse(QPointF(obs_x, obs_y), 5, 5);

            // P label
            painter->setPen(Qt::white);
            QFont pFont("sans-serif", 6, QFont::Bold);
            painter->setFont(pFont);
            painter->drawText(QRectF(obs_x - 5, obs_y - 5, 10, 10),
                              Qt::AlignCenter,
                              QString("P%1").arg(m_index + 1));
        }

        // --- Dielectric indicator ---
        if (m_data.has_dielectric) {
            painter->setPen(QColor(100, 120, 180));
            QFont dFont("sans-serif", 6);
            painter->setFont(dFont);
            QString dText = QString("εr=%1").arg(m_data.dielectric_er, 0, 'f', 1);
            painter->drawText(boxRect.adjusted(4, 36, -4, 0),
                              Qt::AlignHCenter | Qt::AlignTop, dText);
        }

        // --- Selection highlight glow ---
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
    void hoverEnterEvent(QGraphicsSceneHoverEvent* event) override {
        update();
        QGraphicsItem::hoverEnterEvent(event);
    }

    void hoverLeaveEvent(QGraphicsSceneHoverEvent* event) override {
        update();
        QGraphicsItem::hoverLeaveEvent(event);
    }

private:
    void updateTooltip() {
        QString tip = QString("Section %1\nDepth: %2 mm\nAperture: %3x%4 mm\nObserver: %5 mm")
        .arg(m_index + 1)
            .arg(m_data.depth_mm, 0, 'f', 1)
            .arg(m_data.aperture_l_mm, 0, 'f', 1)
            .arg(m_data.aperture_w_mm, 0, 'f', 1)
            .arg(m_data.obs_position_mm, 0, 'f', 1);
        if (m_data.has_cover)
            tip += QString("\nCover gap: %1 mm").arg(m_data.cover_gap_mm, 0, 'f', 2);
        if (m_data.has_dielectric)
            tip += QString("\nDielectric: εr=%1, h=%2 mm")
                       .arg(m_data.dielectric_er, 0, 'f', 1)
                       .arg(m_data.dielectric_h_mm, 0, 'f', 1);
        setToolTip(tip);
    }

    int             m_index;
    SectionItemData m_data;
};

#endif // SECTIONITEM_H
