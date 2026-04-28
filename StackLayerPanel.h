#pragma once

// ============================================================================
//  StackLayerPanel.h — Equivalent-circuit layer stack viewer (Task 1.4)
//
//  Right-side panel that mirrors the circuit canvas as a read-only vertical
//  stack of cards. One card per CanvasElement currently on the canvas, sorted
//  left-to-right by X position so the layer order matches the circuit
//  topology used by runCompute().
//
//  Architecture:
//
//      StackLayerPanel  (QWidget)
//      ├── Sticky header:  "EQUIVALENT CIRCUIT LAYERS"
//      ├── QScrollArea
//      │   └── inner content widget
//      │       └── QVBoxLayout cardsLayout_
//      │           ├── StackLayerCard #1
//      │           ├── StackLayerCard #2
//      │           ├── ...
//      │           └── stretch
//      └── Empty-state label  (shown when cards_ is empty)
//
//  Sync rules (driven by signals from AssemblyCanvas):
//      circuitChanged    → rebuild(canvas->elements)
//      paramsChanged(el) → refreshElement(el)
//      elementSelected   → setSelected(el)
//
//  Strict no-edit invariant: the panel never mutates element params. It only
//  reads from element->params and updates its own display. The property
//  editor on the left panel remains the sole writer of element data.
// ============================================================================

#include "Styles.h"   // EMStyle helpers (accentFor, renderIcon16, rgb)

#include <QFrame>
#include <QHBoxLayout>
#include <QIcon>
#include <QLabel>
#include <QList>
#include <QPainter>
#include <QPushButton>
#include <QScrollArea>
#include <QString>
#include <QVBoxLayout>
#include <QVector>
#include <QWidget>

#include <algorithm>

// Forward-declare CanvasElement / ElementType / ElementParams: their full
// definitions live in CircuitBuilderWindow.h. This header is only included
// AFTER CircuitBuilderWindow.h has declared them, identically to how
// Styles.h is structured — see CBSTYLE_DECLARED guard.
#ifndef CBSTYLE_DECLARED
#  error "StackLayerPanel.h requires CBStyle / CanvasElement to be visible. \
Include this file after CircuitBuilderWindow.h's CBStyle namespace and \
CanvasElement class are declared."
#endif


    // ============================================================================
    //  StackLayerCard — single row representing one CanvasElement
    //
    //  Visual anatomy (cards have a fixed structure regardless of element type;
    //  the parameter rows underneath the header line vary by type):
    //
    //   ┌──────────────────────────────────────┐
    //   │▌ [icon] #1  E₀ Source                │   ← header line
    //   │▌                                     │
    //   │▌  E₀ = 1.000 V/m                     │   ← per-element parameter rows
    //   │▌  f: 0.001–40.000 GHz                │
    //   │▌  a×b = 0.010×0.040 m                │
    //   └──────────────────────────────────────┘
    //   ↑
    //   4-px accent stripe (border-left) coloured by element type:
    //     Source = orange, Aperture/AP+Cover = blue,
    //     Cavity/Diel.Cav = green, Obs.Pt = red.
    //
    //  States:
    //    idle      — surface background, BORDER_LT outline, accent stripe
    //    selected  — surface2 background, accent outline, accent stripe is
    //                 darkened to make it pop
    // ============================================================================

    class StackLayerCard : public QFrame {
    Q_OBJECT
public:
    StackLayerCard(CanvasElement* element, int layerNumber, QWidget* parent = nullptr)
        : QFrame(parent)
        , element_(element)
        , layerNumber_(layerNumber)
    {
        setFrameShape(QFrame::NoFrame);
        setObjectName("StackLayerCard");

        auto* outer = new QVBoxLayout(this);
        outer->setContentsMargins(10, 8, 10, 8);
        outer->setSpacing(4);

        // ── Header row: layer #, type icon, element label ────────────
        auto* headerRow = new QHBoxLayout;
        headerRow->setContentsMargins(0, 0, 0, 0);
        headerRow->setSpacing(8);

        layerLabel_ = new QLabel(QString("#%1").arg(layerNumber_), this);
        layerLabel_->setStyleSheet(layerNumberQSS());
        layerLabel_->setFixedWidth(28);
        headerRow->addWidget(layerLabel_);

        iconLabel_ = new QLabel(this);
        iconLabel_->setFixedSize(16, 16);
        iconLabel_->setPixmap(makeIconPixmap(element_->params.type));
        iconLabel_->setStyleSheet("background:transparent;");
        headerRow->addWidget(iconLabel_);

        titleLabel_ = new QLabel(this);
        titleLabel_->setStyleSheet(titleQSS());
        titleLabel_->setTextInteractionFlags(Qt::NoTextInteraction);
        headerRow->addWidget(titleLabel_, /*stretch*/ 1);

        outer->addLayout(headerRow);

        // ── Parameter list ───────────────────────────────────────────
        paramsLabel_ = new QLabel(this);
        paramsLabel_->setStyleSheet(paramsQSS());
        paramsLabel_->setWordWrap(true);
        paramsLabel_->setTextInteractionFlags(Qt::NoTextInteraction);
        outer->addWidget(paramsLabel_);

        // Initial population
        applyCardStyleSheet(/*selected*/ false);
        refreshContent();
    }

    // ── Public API ───────────────────────────────────────────────────

    CanvasElement* element() const { return element_; }

    int layerNumber() const { return layerNumber_; }

    void setLayerNumber(int n) {
        if (layerNumber_ == n) return;
        layerNumber_ = n;
        layerLabel_->setText(QString("#%1").arg(n));
    }

    void setSelected(bool sel) {
        if (selected_ == sel) return;
        selected_ = sel;
        applyCardStyleSheet(sel);
    }

    // Pull current values out of element_->params and rebuild the visible
    // text. Cheap — called whenever the property panel emits paramsChanged.
    void refreshContent() {
        if (!element_) return;
        const ElementParams& p = element_->params;

        titleLabel_->setText(displayTypeName(p.type) + "  —  " + p.label);
        paramsLabel_->setText(buildParamText(p));
    }

private:
    // ── Member widgets / state ───────────────────────────────────────
    CanvasElement* element_ {nullptr};
    int            layerNumber_ {0};
    bool           selected_    {false};

    QLabel* layerLabel_  {nullptr};
    QLabel* iconLabel_   {nullptr};
    QLabel* titleLabel_  {nullptr};
    QLabel* paramsLabel_ {nullptr};

    // ── QSS builders ─────────────────────────────────────────────────

    QColor accent() const {
        // Element-type accent matches the canvas selection chrome and the
        // left-panel button stripe colours.
        switch (element_->params.type) {
        case ElementType::Source:            return CBStyle::ORANGE;
        case ElementType::Aperture:
        case ElementType::ApertureWithCover: return CBStyle::ACCENT;
        case ElementType::EmptyCavity:
        case ElementType::DielectricCavity:  return CBStyle::GREEN;
        case ElementType::Load:              return CBStyle::RED;
        }
        return CBStyle::TEXT_MUTED;
    }

    void applyCardStyleSheet(bool sel) {
        const QColor a = accent();
        const QColor bg     = sel ? CBStyle::SURFACE2 : CBStyle::SURFACE;
        const QColor border = sel ? a                 : CBStyle::BORDER_LT;
        const qreal  borderW = sel ? 1.4 : 1.0;
        // The accent stripe stays the same colour in both states to avoid
        // a "moving target" feeling; only the surrounding border + bg
        // intensifies on selection. The layer-number badge and title bold
        // also pick up the accent more strongly when selected.
        setStyleSheet(QString(
                          "QFrame#StackLayerCard{"
                          "background:%1;"
                          "border:%2px solid %3;"
                          "border-left:4px solid %4;"
                          "border-radius:4px;"
                          "}"
                          )
                          .arg(EMStyle::rgb(bg))
                          .arg(borderW, 0, 'f', 1)
                          .arg(EMStyle::rgb(border))
                          .arg(EMStyle::rgb(a)));

        // Refresh sub-label colours — selection bumps the title weight.
        titleLabel_->setStyleSheet(sel ? titleSelectedQSS() : titleQSS());
        layerLabel_->setStyleSheet(sel ? layerNumberSelectedQSS() : layerNumberQSS());
    }

    QString layerNumberQSS() const {
        return QString(
                   "QLabel{"
                   "color:%1;"
                   "background:transparent;"
                   "font-family:'Courier New',monospace;"
                   "font-size:11px;"
                   "font-weight:bold;"
                   "letter-spacing:1px;"
                   "}"
                   ).arg(EMStyle::rgb(CBStyle::TEXT_MUTED));
    }

    QString layerNumberSelectedQSS() const {
        return QString(
                   "QLabel{"
                   "color:%1;"
                   "background:transparent;"
                   "font-family:'Courier New',monospace;"
                   "font-size:11px;"
                   "font-weight:bold;"
                   "letter-spacing:1px;"
                   "}"
                   ).arg(EMStyle::rgb(accent()));
    }

    QString titleQSS() const {
        return QString(
                   "QLabel{"
                   "color:%1;"
                   "background:transparent;"
                   "font-family:'Courier New',monospace;"
                   "font-size:11px;"
                   "font-weight:600;"
                   "}"
                   ).arg(EMStyle::rgb(CBStyle::TEXT));
    }

    QString titleSelectedQSS() const {
        return QString(
                   "QLabel{"
                   "color:%1;"
                   "background:transparent;"
                   "font-family:'Courier New',monospace;"
                   "font-size:11px;"
                   "font-weight:bold;"
                   "}"
                   ).arg(EMStyle::rgb(accent()));
    }

    QString paramsQSS() const {
        return QString(
                   "QLabel{"
                   "color:%1;"
                   "background:transparent;"
                   "font-family:'Courier New',monospace;"
                   "font-size:10px;"
                   "padding-left:36px;"        // align under title (after #N + icon column)
                   "}"
                   ).arg(EMStyle::rgb(CBStyle::TEXT_MUTED));
    }

    // ── Icon pixmap factory — uses Task 1.3 schematic painters ───────

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

    static QString displayTypeName(ElementType t) {
        switch (t) {
        case ElementType::Source:            return QStringLiteral("SOURCE");
        case ElementType::Aperture:          return QStringLiteral("APERTURE");
        case ElementType::ApertureWithCover: return QStringLiteral("AP+COVER");
        case ElementType::EmptyCavity:       return QStringLiteral("CAVITY");
        case ElementType::DielectricCavity:  return QStringLiteral("DIEL.CAV");
        case ElementType::Load:              return QStringLiteral("OBS.PT");
        }
        return QStringLiteral("ELEMENT");
    }

    // ── Parameter text formatter ─────────────────────────────────────
    //
    // Per-type list of the most relevant parameters. Numeric formatting
    // mirrors the property panel: 6-decimal fixed for small physical
    // dimensions, scientific notation for impedances that span orders
    // of magnitude (Z_L can be 1 mΩ to 10 GΩ).

    static QString fmt(double v, int decimals = 6) {
        return QString::number(v, 'f', decimals);
    }

    static QString fmtImp(double v) {
        // Use scientific notation when magnitude >= 1e4 or < 1e-2 to keep
        // the line short; otherwise fixed-point with 3 decimals.
        const double a = std::abs(v);
        if (a == 0.0)                  return QStringLiteral("0");
        if (a >= 1.0e4 || a < 1.0e-2)  return QString::number(v, 'e', 2);
        return QString::number(v, 'f', 3);
    }

    static QString buildParamText(const ElementParams& p) {
        switch (p.type) {
        case ElementType::Source:
            return QString(
                       "E\u2080 = %1 V/m\n"
                       "f: %2 \u2013 %3 GHz   (%4 pts)\n"
                       "a\u00d7b = %5\u00d7%6 m\n"
                       "t_wall = %7 m"
                       )
                .arg(fmt(p.E0, 3))
                .arg(fmt(p.freqStart, 4))
                .arg(fmt(p.freqEnd, 3))
                .arg(p.freqPoints)
                .arg(fmt(p.a, 4))
                .arg(fmt(p.b_h, 4))
                .arg(fmt(p.t_wall, 6));

        case ElementType::Aperture:
            return QString("l\u00d7w = %1\u00d7%2 m")
                .arg(fmt(p.l_slot, 4))
                .arg(fmt(p.w_slot, 4));

        case ElementType::ApertureWithCover:
            return QString(
                       "l\u00d7w = %1\u00d7%2 m\n"
                       "\u03c4 gap = %3 m"
                       )
                .arg(fmt(p.l_slot, 4))
                .arg(fmt(p.w_slot, 4))
                .arg(fmt(p.tau_cover, 6));

        case ElementType::EmptyCavity:
            return QString("L = %1 m").arg(fmt(p.L_cavity, 4));

        case ElementType::DielectricCavity:
            return QString(
                       "L = %1 m\n"
                       "h_diel = %2 m\n"
                       "\u03b5\u1d63 = %3"
                       )
                .arg(fmt(p.L_cavity, 4))
                .arg(fmt(p.h_dielectric, 5))
                .arg(fmt(p.eps_r, 2));

        case ElementType::Load:
            // Show Z_L in instrument shorthand, e.g. "Z_L = 1.00e+09 + j0 \u03a9"
            if (p.ZL_imag == 0.0) {
                return QString("Z_L = %1 \u03a9").arg(fmtImp(p.ZL_real));
            }
            return QString("Z_L = %1 %2 j%3 \u03a9")
                .arg(fmtImp(p.ZL_real))
                .arg(p.ZL_imag >= 0.0 ? QStringLiteral("+") : QStringLiteral("\u2212"))
                .arg(fmtImp(std::abs(p.ZL_imag)));
        }
        return QString();
    }
};


// ============================================================================
//  StackLayerPanel — read-only right-side panel hosting one card per element
// ============================================================================

class StackLayerPanel : public QWidget {
    Q_OBJECT
public:
    explicit StackLayerPanel(QWidget* parent = nullptr) : QWidget(parent) {
        setObjectName("StackLayerPanel");
        setStyleSheet(QString("QWidget#StackLayerPanel{background:%1;}")
                          .arg(EMStyle::rgb(CBStyle::BG)));

        auto* root = new QVBoxLayout(this);
        root->setContentsMargins(0, 0, 0, 0);
        root->setSpacing(0);

        // ── Header (sticky) ───────────────────────────────────────────
        auto* header = new QLabel(QStringLiteral("EQUIVALENT CIRCUIT LAYERS"), this);
        header->setStyleSheet(QString(
                                  "QLabel{"
                                  "background:%1;"
                                  "color:%2;"
                                  "font-family:'Courier New',monospace;"
                                  "font-size:10px;"
                                  "font-weight:bold;"
                                  "letter-spacing:2px;"
                                  "padding:10px 12px;"
                                  "border-bottom:1px solid %3;"
                                  "}"
                                  )
                                  .arg(EMStyle::rgb(CBStyle::SURFACE2))
                                  .arg(EMStyle::rgb(CBStyle::TEXT))
                                  .arg(EMStyle::rgb(CBStyle::BORDER)));
        header->setAlignment(Qt::AlignLeft | Qt::AlignVCenter);
        root->addWidget(header);

        // ── Scroll area + cards container ────────────────────────────
        scrollArea_ = new QScrollArea(this);
        scrollArea_->setWidgetResizable(true);
        scrollArea_->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
        scrollArea_->setStyleSheet(QString(
                                       "QScrollArea{background:%1;border:none;}"
                                       "QScrollBar:vertical{background:%1;width:10px;margin:0;}"
                                       "QScrollBar::handle:vertical{background:%2;border-radius:4px;min-height:24px;}"
                                       "QScrollBar::handle:vertical:hover{background:%3;}"
                                       "QScrollBar::add-line:vertical,QScrollBar::sub-line:vertical{height:0;}"
                                       )
                                       .arg(EMStyle::rgb(CBStyle::BG))
                                       .arg(EMStyle::rgb(CBStyle::BORDER))
                                       .arg(EMStyle::rgb(CBStyle::TEXT_MUTED)));

        contentWidget_ = new QWidget(scrollArea_);
        contentWidget_->setStyleSheet(QString("background:%1;")
                                          .arg(EMStyle::rgb(CBStyle::BG)));
        cardsLayout_ = new QVBoxLayout(contentWidget_);
        cardsLayout_->setContentsMargins(10, 10, 10, 10);
        cardsLayout_->setSpacing(8);
        cardsLayout_->addStretch(1);   // push cards to the top
        scrollArea_->setWidget(contentWidget_);
        root->addWidget(scrollArea_, /*stretch*/ 1);

        // ── Empty-state label ────────────────────────────────────────
        emptyLabel_ = new QLabel(
            QStringLiteral(
                "No elements yet.\n\n"
                "Drop elements onto the canvas\n"
                "to see them stack here."),
            contentWidget_);
        emptyLabel_->setAlignment(Qt::AlignCenter);
        emptyLabel_->setStyleSheet(QString(
                                       "QLabel{"
                                       "background:transparent;"
                                       "color:%1;"
                                       "font-family:'Courier New',monospace;"
                                       "font-size:10px;"
                                       "padding:30px 12px;"
                                       "}"
                                       ).arg(EMStyle::rgb(CBStyle::TEXT_DIM)));
        // Insert empty-state ABOVE the stretch so it sits at the top.
        cardsLayout_->insertWidget(0, emptyLabel_);
    }

    // ── Public API ──────────────────────────────────────────────────

    // Rebuild the stack from the current set of canvas elements.
    // Cards are sorted left-to-right by X position so the stack mirrors
    // the circuit topology used by runCompute().
    void rebuild(const QVector<CanvasElement*>& elements) {
        clearCards();

        if (elements.isEmpty()) {
            emptyLabel_->setVisible(true);
            return;
        }
        emptyLabel_->setVisible(false);

        // Sort by X position; same comparator as runCompute()
        QVector<CanvasElement*> ordered = elements;
        std::sort(ordered.begin(), ordered.end(),
                  [](CanvasElement* a, CanvasElement* b){
                      return a->pos().x() < b->pos().x();
                  });

        for (int i = 0; i < ordered.size(); ++i) {
            auto* card = new StackLayerCard(ordered[i], i + 1, contentWidget_);
            cards_.append(card);
            // Insert before the trailing stretch so cards stack from the top.
            cardsLayout_->insertWidget(cardsLayout_->count() - 1, card);
        }
        // Re-apply current selection (if the previously-selected element
        // is still present) — keeps highlight stable across rebuilds.
        if (selectedElement_) {
            applySelectionHighlight();
        }
    }

    // Update the visible parameter text on whichever card maps to `el`.
    // Called from CircuitBuilderWindow when the property panel emits
    // paramsChanged. No-op if the element has no card (e.g. the stack
    // hasn't been rebuilt yet after a drop).
    void refreshElement(CanvasElement* el) {
        if (!el) return;
        for (auto* card : cards_) {
            if (card->element() == el) {
                card->refreshContent();
                return;
            }
        }
    }

    // One-way selection sync from canvas → panel.  Highlight the card
    // matching `el`, un-highlight all others. `el == nullptr` clears
    // every highlight.
    void setSelected(CanvasElement* el) {
        selectedElement_ = el;
        applySelectionHighlight();
    }

private:
    // ── Helpers ─────────────────────────────────────────────────────

    void clearCards() {
        for (auto* card : cards_) {
            cardsLayout_->removeWidget(card);
            card->deleteLater();
        }
        cards_.clear();
    }

    void applySelectionHighlight() {
        for (auto* card : cards_) {
            card->setSelected(card->element() == selectedElement_);
        }
    }

    // ── Members ─────────────────────────────────────────────────────
    QScrollArea*           scrollArea_      {nullptr};
    QWidget*               contentWidget_   {nullptr};
    QVBoxLayout*           cardsLayout_     {nullptr};
    QLabel*                emptyLabel_      {nullptr};
    QList<StackLayerCard*> cards_;
    CanvasElement*         selectedElement_ {nullptr};
};
