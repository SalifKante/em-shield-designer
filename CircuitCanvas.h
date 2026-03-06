#ifndef CIRCUITCANVAS_H
#define CIRCUITCANVAS_H

// ============================================================
//  CircuitCanvas.h
//  Qt/C++ — EMShieldDesigner  (Quick Simulation mode)
//
//  Corrections vs. original:
//    [C1] Ground symbol redrawn as standard horizontal-line stack
//         (IEC 60617 / IEEE Std 315-1975 ground glyph).
//         Original drew three vertical bars — incorrect geometry.
//    [C2] Node label computation is topology-aware.
//         CASCADE  → labels follow 2i+1 / 2(i+1) (original formula, correct).
//         STAR_BRANCH → labels follow the non-linear node assignment
//         of Fig. 3.11b: section 1 at nodes 1,2; section 2 at 4,5;
//         section 3 at 6,7 (inter-section apertures occupy nodes 3 and 5).
//         A setTopology() accessor allows MainWindow to push the
//         active TopologyType to the canvas whenever it changes.
// ============================================================

#include "SectionItem.h"
#include "include/core/CircuitGenerator.h"    // TopologyType enum

#include <QGraphicsView>
#include <QGraphicsScene>
#include <QGraphicsTextItem>
#include <QGraphicsLineItem>
#include <QGraphicsEllipseItem>
#include <QKeyEvent>
#include <QWheelEvent>
#include <QScrollBar>
#include <QVector>

using EMCore::TopologyType;

class CircuitCanvas : public QGraphicsView
{
    Q_OBJECT

public:
    explicit CircuitCanvas(QWidget* parent = nullptr)
        : QGraphicsView(parent)
        , m_topology(TopologyType::CASCADE)   // default matches MainWindow default
    {
        m_scene = new QGraphicsScene(this);
        setScene(m_scene);

        setRenderHint(QPainter::Antialiasing, true);
        setDragMode(QGraphicsView::RubberBandDrag);
        setViewportUpdateMode(QGraphicsView::SmartViewportUpdate);
        setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
        setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
        setBackgroundBrush(QColor(248, 250, 252));

        connect(m_scene, &QGraphicsScene::selectionChanged,
                this, &CircuitCanvas::onSelectionChanged);
    }

    // ── Public API ────────────────────────────────────────────────────

    // [C2] Called by MainWindow whenever the topology combo changes.
    void setTopology(TopologyType t)
    {
        if(m_topology == t) return;
        m_topology = t;
        if(!m_sections.isEmpty()) layoutSections();  // rebuild labels immediately
    }
    TopologyType topology() const { return m_topology; }

    // ── Section management ───────────────────────────────────────────

    void addSection(const SectionItemData& data = SectionItemData())
    {
        int idx = m_sections.size();
        SectionItem* item = new SectionItem(idx);
        item->setData(data);
        m_scene->addItem(item);
        m_sections.append(item);
        layoutSections();
        emit sectionCountChanged(m_sections.size());
    }

    void removeSection(int index)
    {
        if(index < 0 || index >= m_sections.size()) return;
        if(m_sections.size() <= 1) return;   // keep at least 1 section

        SectionItem* item = m_sections[index];
        m_scene->removeItem(item);
        m_sections.removeAt(index);
        delete item;

        for(int i = 0; i < m_sections.size(); ++i)
            m_sections[i]->setSectionIndex(i);

        layoutSections();
        emit sectionCountChanged(m_sections.size());
        emit selectionChanged(-1);
    }

    void removeSelectedSection()
    {
        QList<QGraphicsItem*> sel = m_scene->selectedItems();
        if(sel.isEmpty()) return;
        SectionItem* item = dynamic_cast<SectionItem*>(sel.first());
        if(item) removeSection(item->sectionIndex());
    }

    int          sectionCount()     const { return m_sections.size(); }
    SectionItem* sectionAt(int i)   const { return (i>=0&&i<m_sections.size())?m_sections[i]:nullptr; }

    int selectedIndex() const
    {
        QList<QGraphicsItem*> sel = m_scene->selectedItems();
        if(sel.isEmpty()) return -1;
        SectionItem* item = dynamic_cast<SectionItem*>(sel.first());
        return item ? item->sectionIndex() : -1;
    }

    void selectSection(int index)
    {
        m_scene->clearSelection();
        if(index >= 0 && index < m_sections.size())
            m_sections[index]->setSelected(true);
    }

    QVector<SectionItemData> allSectionData() const
    {
        QVector<SectionItemData> result;
        result.reserve(m_sections.size());
        for(const auto* item : m_sections) result.append(item->data());
        return result;
    }

    void loadPreset(const QVector<SectionItemData>& sections)
    {
        clear();
        for(const auto& sec : sections) addSection(sec);
        if(!m_sections.isEmpty()) selectSection(0);
    }

    void clear()
    {
        for(auto* item : m_sections){ m_scene->removeItem(item); delete item; }
        m_sections.clear();
        clearDecorations();
        emit sectionCountChanged(0);
    }

signals:
    void selectionChanged(int sectionIndex);
    void sectionCountChanged(int count);

protected:
    void keyPressEvent(QKeyEvent* event) override
    {
        if(event->key()==Qt::Key_Delete || event->key()==Qt::Key_Backspace){
            removeSelectedSection(); event->accept();
        } else {
            QGraphicsView::keyPressEvent(event);
        }
    }

    void wheelEvent(QWheelEvent* event) override
    {
        if(event->modifiers() & Qt::ControlModifier){
            double factor = event->angleDelta().y() > 0 ? 1.15 : 1.0/1.15;
            scale(factor, factor); event->accept();
        } else {
            QGraphicsView::wheelEvent(event);
        }
    }

    void resizeEvent(QResizeEvent* event) override
    {
        QGraphicsView::resizeEvent(event);
        fitContent();
    }

private slots:
    void onSelectionChanged()
    {
        QList<QGraphicsItem*> sel = m_scene->selectedItems();
        if(sel.isEmpty()){ emit selectionChanged(-1); return; }
        SectionItem* item = dynamic_cast<SectionItem*>(sel.first());
        if(item) emit selectionChanged(item->sectionIndex());
    }

private:
    // ── Layout helpers ───────────────────────────────────────────────

    void clearDecorations()
    {
        for(auto* item : m_decorations){ m_scene->removeItem(item); delete item; }
        m_decorations.clear();
    }

    // [C2] Compute boundary node numbers that bracket section i,
    //      depending on the active topology.
    //
    //      CASCADE (Fig. 3.10):
    //        each section contributes 2 nodes (entry after aperture, obs).
    //        entryNode(i) = 2i+1,   obsNode(i) = 2i+2.
    //
    //      STAR_BRANCH (Fig. 3.11):
    //        Section 1 spans nodes 1,2.  Aperture Z²_ap sits at node 3.
    //        Section 2 spans nodes 4,5 (obs at 4, end at 5 not shown).
    //        Aperture Z³_ap sits at node 5.
    //        Section 3 spans nodes 6,7.
    //        Pattern: section 0 → entry=1, obs=2
    //                 section i>0 → entry = 2 + 2i, obs = 3 + 2i
    //        (Derived from the 10-branch, 6-node Y-matrix in Eq. 3.26.)
    struct NodeLabels { int entry; int obs; };
    NodeLabels nodeLabelsFor(int i) const
    {
        if(m_topology == TopologyType::CASCADE){
            // Nodes: ..., 2i+1 (entry), 2i+2 (obs), ...
            return { 2*i + 1, 2*(i+1) };
        } else {
            // STAR_BRANCH: section 0 → {1, 2}
            //              section i → {2 + 2i, 3 + 2i}
            if(i == 0) return { 1, 2 };
            return { 2 + 2*i, 3 + 2*i };
        }
    }

    void layoutSections()
    {
        clearDecorations();
        if(m_sections.isEmpty()) return;

        double x = 0.0, y = 0.0;
        const double wireY = y + SectionItem::BOX_HEIGHT / 2.0;

        // ── Source symbol ────────────────────────────────────────────
        const double srcR = 14.0;
        auto* srcCircle = m_scene->addEllipse(
            x - srcR, wireY - srcR, 2*srcR, 2*srcR,
            QPen(QColor(22, 163, 74), 2),
            QBrush(QColor(240, 253, 244)));
        m_decorations.append(srcCircle);

        auto* srcLabel = m_scene->addText("V₀", QFont("sans-serif", 8, QFont::Bold));
        srcLabel->setDefaultTextColor(QColor(22, 163, 74));
        srcLabel->setPos(x - 9.0, wireY - 9.0);
        m_decorations.append(srcLabel);

        const double wireStartX = x + srcR;
        const double wireEndX   = x + srcR + 20.0;
        auto* wire = m_scene->addLine(wireStartX, wireY, wireEndX, wireY,
                                      QPen(QColor(60, 70, 90), 1.5));
        m_decorations.append(wire);
        x = wireEndX + 2.0;

        // ── Section items ────────────────────────────────────────────
        for(int i = 0; i < m_sections.size(); ++i){
            m_sections[i]->setPos(x, y);
            x += SectionItem::BOX_WIDTH + SectionItem::APERTURE_WIDTH + SectionItem::SPACING;

            if(i < m_sections.size() - 1){
                double wx1 = x - SectionItem::SPACING;
                auto* conn = m_scene->addLine(wx1, wireY, x, wireY,
                                              QPen(QColor(60, 70, 90), 1.5));
                m_decorations.append(conn);
            }
        }

        // ── [C1] Ground symbol — standard IEC horizontal-line stack ──
        //
        //   Corrected geometry:
        //     Line 0: full width  (halfLen = 10)   — thickest
        //     Line 1: medium      (halfLen =  7)
        //     Line 2: narrow      (halfLen =  4)   — thinnest
        //   Lines are spaced 5 px apart, vertically centred on gndY.
        //   Each line is horizontal: (gx - halfLen, gy) → (gx + halfLen, gy).
        //
        const double gndX = x - SectionItem::SPACING + 6.0;
        const double gndY = wireY;

        // Short connecting wire
        auto* gndWire = m_scene->addLine(gndX, gndY, gndX + 16.0, gndY,
                                         QPen(QColor(60, 70, 90), 1.5));
        m_decorations.append(gndWire);

        const double gcx = gndX + 18.0;   // centre-x of ground stack
        for(int gi = 0; gi < 3; ++gi){
            const double halfLen = 10.0 - gi * 3.0;          // 10, 7, 4
            const double lineY   = gndY + gi * 5.0;           // stack downward
            const double penW    = 2.0  - gi * 0.5;           // 2.0, 1.5, 1.0
            auto* gline = m_scene->addLine(
                gcx - halfLen, lineY,     // left endpoint
                gcx + halfLen, lineY,     // right endpoint  ← horizontal
                QPen(QColor(60, 70, 90), penW));
            m_decorations.append(gline);
        }

        // ── Node labels ──────────────────────────────────────────────
        QFont nf("sans-serif", 7);
        const double nodeY = y - 14.0;

        // Ground node label (N0) at source
        auto* n0 = m_scene->addText("N0", nf);
        n0->setDefaultTextColor(QColor(120, 120, 120));
        n0->setPos(x - SectionItem::SPACING - 14.0, nodeY);
        m_decorations.append(n0);

        // [C2] Per-section labels using topology-aware nodeLabelsFor()
        for(int i = 0; i < m_sections.size(); ++i){
            const QPointF pos = m_sections[i]->pos();
            const auto    nl  = nodeLabelsFor(i);

            // Entry node (after aperture)
            auto* nEntry = m_scene->addText(QString("N%1").arg(nl.entry), nf);
            nEntry->setDefaultTextColor(QColor(120, 120, 120));
            nEntry->setPos(pos.x() + SectionItem::APERTURE_WIDTH - 4.0, nodeY);
            m_decorations.append(nEntry);

            // Observation node (inside cavity box)
            if(m_sections[i]->data().has_observation){
                const SectionItemData& sd   = m_sections[i]->data();
                double obs_ratio = sd.obs_position_mm / std::max(1.0, sd.depth_mm);
                obs_ratio = std::min(std::max(obs_ratio, 0.05), 0.95);
                const double obsX = pos.x() + SectionItem::APERTURE_WIDTH
                                    + obs_ratio * SectionItem::BOX_WIDTH;
                auto* obsLabel = m_scene->addText(QString("N%1").arg(nl.obs), nf);
                obsLabel->setDefaultTextColor(QColor(220, 38, 38));
                obsLabel->setPos(obsX - 8.0, y + SectionItem::BOX_HEIGHT + 2.0);
                m_decorations.append(obsLabel);
            }
        }

        fitContent();
    }

    void fitContent()
    {
        if(m_sections.isEmpty()) return;
        QRectF bounds = m_scene->itemsBoundingRect().adjusted(-30, -30, 30, 30);
        m_scene->setSceneRect(bounds);
        fitInView(bounds, Qt::KeepAspectRatio);
    }

    // ── Members ──────────────────────────────────────────────────────
    QGraphicsScene*          m_scene;
    QVector<SectionItem*>    m_sections;
    QVector<QGraphicsItem*>  m_decorations;
    TopologyType             m_topology;
};

#endif // CIRCUITCANVAS_H
