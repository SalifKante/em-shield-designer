#ifndef CIRCUITCANVAS_H
#define CIRCUITCANVAS_H

#include "SectionItem.h"

#include <QGraphicsView>
#include <QGraphicsScene>
#include <QGraphicsTextItem>
#include <QGraphicsLineItem>
#include <QGraphicsEllipseItem>
#include <QKeyEvent>
#include <QWheelEvent>
#include <QScrollBar>
#include <QVector>

class CircuitCanvas : public QGraphicsView
{
    Q_OBJECT

public:
    explicit CircuitCanvas(QWidget* parent = nullptr)
        : QGraphicsView(parent)
    {
        m_scene = new QGraphicsScene(this);
        setScene(m_scene);

        // View settings
        setRenderHint(QPainter::Antialiasing, true);
        setDragMode(QGraphicsView::RubberBandDrag);
        setViewportUpdateMode(QGraphicsView::SmartViewportUpdate);
        setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
        setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);

        // Background
        setBackgroundBrush(QColor(248, 250, 252));

        // Selection tracking
        connect(m_scene, &QGraphicsScene::selectionChanged,
                this, &CircuitCanvas::onSelectionChanged);
    }

    // --- Section management ---

    void addSection(const SectionItemData& data = SectionItemData()) {
        int idx = m_sections.size();
        SectionItem* item = new SectionItem(idx);
        item->setData(data);
        m_scene->addItem(item);
        m_sections.append(item);
        layoutSections();
        emit sectionCountChanged(m_sections.size());
    }

    void removeSection(int index) {
        if (index < 0 || index >= m_sections.size()) return;
        if (m_sections.size() <= 1) return;  // Keep at least 1 section

        SectionItem* item = m_sections[index];
        m_scene->removeItem(item);
        m_sections.removeAt(index);
        delete item;

        // Reindex remaining sections
        for (int i = 0; i < m_sections.size(); ++i) {
            m_sections[i]->setSectionIndex(i);
        }

        layoutSections();
        emit sectionCountChanged(m_sections.size());
        emit selectionChanged(-1);  // Clear property panel
    }

    void removeSelectedSection() {
        QList<QGraphicsItem*> sel = m_scene->selectedItems();
        if (sel.isEmpty()) return;

        SectionItem* item = dynamic_cast<SectionItem*>(sel.first());
        if (item) {
            removeSection(item->sectionIndex());
        }
    }

    int sectionCount() const { return m_sections.size(); }

    SectionItem* sectionAt(int index) const {
        if (index >= 0 && index < m_sections.size())
            return m_sections[index];
        return nullptr;
    }

    int selectedIndex() const {
        QList<QGraphicsItem*> sel = m_scene->selectedItems();
        if (sel.isEmpty()) return -1;
        SectionItem* item = dynamic_cast<SectionItem*>(sel.first());
        return item ? item->sectionIndex() : -1;
    }

    void selectSection(int index) {
        m_scene->clearSelection();
        if (index >= 0 && index < m_sections.size()) {
            m_sections[index]->setSelected(true);
        }
    }

    // Get all section data for building EnclosureConfig
    QVector<SectionItemData> allSectionData() const {
        QVector<SectionItemData> result;
        result.reserve(m_sections.size());
        for (const auto* item : m_sections) {
            result.append(item->data());
        }
        return result;
    }

    // Load preset configuration
    void loadPreset(const QVector<SectionItemData>& sections) {
        clear();
        for (const auto& sec : sections) {
            addSection(sec);
        }
        if (!m_sections.isEmpty()) {
            selectSection(0);
        }
    }

    void clear() {
        for (auto* item : m_sections) {
            m_scene->removeItem(item);
            delete item;
        }
        m_sections.clear();
        clearDecorations();
        emit sectionCountChanged(0);
    }

signals:
    void selectionChanged(int sectionIndex);
    void sectionCountChanged(int count);

protected:
    void keyPressEvent(QKeyEvent* event) override {
        if (event->key() == Qt::Key_Delete || event->key() == Qt::Key_Backspace) {
            removeSelectedSection();
            event->accept();
        } else {
            QGraphicsView::keyPressEvent(event);
        }
    }

    void wheelEvent(QWheelEvent* event) override {
        // Zoom with Ctrl+wheel
        if (event->modifiers() & Qt::ControlModifier) {
            double factor = event->angleDelta().y() > 0 ? 1.15 : 1.0 / 1.15;
            scale(factor, factor);
            event->accept();
        } else {
            QGraphicsView::wheelEvent(event);
        }
    }

    void resizeEvent(QResizeEvent* event) override {
        QGraphicsView::resizeEvent(event);
        fitContent();
    }

private slots:
    void onSelectionChanged() {
        QList<QGraphicsItem*> sel = m_scene->selectedItems();
        if (sel.isEmpty()) {
            emit selectionChanged(-1);
        } else {
            SectionItem* item = dynamic_cast<SectionItem*>(sel.first());
            if (item) {
                emit selectionChanged(item->sectionIndex());
            }
        }
    }

private:
    void clearDecorations() {
        for (auto* item : m_decorations) {
            m_scene->removeItem(item);
            delete item;
        }
        m_decorations.clear();
    }

    void layoutSections() {
        clearDecorations();

        if (m_sections.isEmpty()) return;

        double x = 0;
        double y = 0;

        // --- Source symbol (left side) ---
        double srcX = x;

        // Source circle
        double srcR = 14;
        auto* srcCircle = m_scene->addEllipse(
            srcX - srcR, y + SectionItem::BOX_HEIGHT/2.0 - srcR,
            2*srcR, 2*srcR,
            QPen(QColor(22, 163, 74), 2),
            QBrush(QColor(240, 253, 244))
            );
        m_decorations.append(srcCircle);

        // "V" label inside source
        auto* srcLabel = m_scene->addText("V₀", QFont("sans-serif", 8, QFont::Bold));
        srcLabel->setDefaultTextColor(QColor(22, 163, 74));
        srcLabel->setPos(srcX - 9, y + SectionItem::BOX_HEIGHT/2.0 - 9);
        m_decorations.append(srcLabel);

        // Wire from source to first section
        double wireStartX = srcX + srcR;
        double wireEndX = srcX + srcR + 20;
        double wireY = y + SectionItem::BOX_HEIGHT / 2.0;
        auto* wire = m_scene->addLine(wireStartX, wireY, wireEndX, wireY,
                                      QPen(QColor(60, 70, 90), 1.5));
        m_decorations.append(wire);

        x = wireEndX + 2;

        // --- Place section items ---
        for (int i = 0; i < m_sections.size(); ++i) {
            m_sections[i]->setPos(x, y);
            x += SectionItem::BOX_WIDTH + SectionItem::APERTURE_WIDTH + SectionItem::SPACING;

            // Connection wire between sections (except after last)
            if (i < m_sections.size() - 1) {
                double wireX1 = x - SectionItem::SPACING;
                double wireX2 = x;
                auto* conn = m_scene->addLine(wireX1, wireY, wireX2, wireY,
                                              QPen(QColor(60, 70, 90), 1.5));
                m_decorations.append(conn);
            }
        }

        // --- Ground symbol (right side) after last section ---
        double gndX = x - SectionItem::SPACING + 6;
        double gndY = wireY;

        // Short horizontal wire to ground
        auto* gndWire = m_scene->addLine(gndX, gndY, gndX + 16, gndY,
                                         QPen(QColor(60, 70, 90), 1.5));
        m_decorations.append(gndWire);

        // Ground symbol (3 horizontal lines getting shorter)
        double gx = gndX + 18;
        for (int i = 0; i < 3; ++i) {
            double halfLen = 10 - i * 3;
            auto* gline = m_scene->addLine(
                gx, gndY - halfLen, gx, gndY + halfLen,
                QPen(QColor(60, 70, 90), 2.0 - i * 0.5)
                );
            m_decorations.append(gline);
            gx += 4;
        }

        // --- Node labels ---
        QFont nodeFont("sans-serif", 7);
        double nodeY = y - 14;

        // Node 0 at source
        auto* n0 = m_scene->addText("N0", nodeFont);
        n0->setDefaultTextColor(QColor(120, 120, 120));
        n0->setPos(srcX - 8, nodeY);
        m_decorations.append(n0);

        // Node labels at section boundaries
        for (int i = 0; i < m_sections.size(); ++i) {
            QPointF pos = m_sections[i]->pos();

            // Node at start of section (after aperture)
            int nodeNum = 2 * i + 1;
            auto* nLabel = m_scene->addText(QString("N%1").arg(nodeNum), nodeFont);
            nLabel->setDefaultTextColor(QColor(120, 120, 120));
            nLabel->setPos(pos.x() + SectionItem::APERTURE_WIDTH - 4, nodeY);
            m_decorations.append(nLabel);

            // Observation node
            if (m_sections[i]->data().has_observation) {
                int obsNode = 2 * (i + 1);
                double obs_ratio = m_sections[i]->data().obs_position_mm /
                                   std::max(1.0, m_sections[i]->data().depth_mm);
                obs_ratio = std::min(std::max(obs_ratio, 0.05), 0.95);
                double obsX = pos.x() + SectionItem::APERTURE_WIDTH +
                              obs_ratio * SectionItem::BOX_WIDTH;
                auto* obsLabel = m_scene->addText(QString("N%1").arg(obsNode), nodeFont);
                obsLabel->setDefaultTextColor(QColor(220, 38, 38));
                obsLabel->setPos(obsX - 8, y + SectionItem::BOX_HEIGHT + 2);
                m_decorations.append(obsLabel);
            }
        }

        fitContent();
    }

    void fitContent() {
        if (m_sections.isEmpty()) return;
        QRectF bounds = m_scene->itemsBoundingRect().adjusted(-30, -30, 30, 30);
        m_scene->setSceneRect(bounds);
        fitInView(bounds, Qt::KeepAspectRatio);
    }

    QGraphicsScene*       m_scene;
    QVector<SectionItem*> m_sections;
    QVector<QGraphicsItem*> m_decorations;  // Source, wires, ground symbols
};

#endif // CIRCUITCANVAS_H
