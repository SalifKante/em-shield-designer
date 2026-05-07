#pragma once

// ============================================================================
//  MessageDialog.h — branded error / success modal dialog (Task 1.6)
//
//  Drop-in replacement for QMessageBox::warning / QMessageBox::critical /
//  QMessageBox::information with the visual language established by the rest
//  of the application:
//      * frameless rounded card (no native title-bar chrome)
//      * 4 px coloured left accent stripe (red for error, green for success)
//      * QPainter-drawn type icon (warning triangle / checkmark) — no
//        external image assets, no emoji, consistent with the Task 1.3
//        schematic palette
//      * 150 ms opacity fade-in on show
//      * click-outside-card dismisses; drag from anywhere repositions
//      * one action button (Close / OK)
//
//  Usage:
//      MessageDialog::error  (parent, "Title", "Body text");
//      MessageDialog::success(parent, "Title", "Body text");
//
//  Both static helpers block until the dialog is dismissed, mirroring
//  QMessageBox::exec semantics, so existing QMessageBox call sites can be
//  swapped one-for-one without touching surrounding control flow.
// ============================================================================

#include "Styles.h"   // CBStyle palette + EMStyle helpers

#include <QApplication>
#include <QColor>
#include <QDialog>
#include <QFont>
#include <QFontMetrics>
#include <QGraphicsOpacityEffect>
#include <QHBoxLayout>
#include <QLabel>
#include <QMouseEvent>
#include <QPaintEvent>
#include <QPainter>
#include <QPainterPath>
#include <QPropertyAnimation>
#include <QPushButton>
#include <QScreen>
#include <QShowEvent>
#include <QString>
#include <QVBoxLayout>
#include <QWidget>

// Guard: this header relies on CBStyle/EMStyle being declared, which happens
// inside CircuitBuilderWindow.h. If you ever include MessageDialog.h in a
// translation unit that does NOT have CircuitBuilderWindow.h's CBStyle in
// scope, the guard fires and you get a clear error.
#ifndef CBSTYLE_DECLARED
#  error "MessageDialog.h requires the CBStyle palette. Include this header \
after CircuitBuilderWindow.h, or otherwise ensure CBSTYLE_DECLARED is defined."
#endif


// ============================================================================
//  MessageDialog
// ============================================================================
class MessageDialog : public QDialog {
    Q_OBJECT
public:
    enum class Kind { Error, Success };

    // ─── Static helpers (the public API) ────────────────────────
    static void error(QWidget* parent, const QString& title, const QString& body) {
        MessageDialog dlg(parent, Kind::Error, title, body);
        dlg.exec();
    }
    static void success(QWidget* parent, const QString& title, const QString& body) {
        MessageDialog dlg(parent, Kind::Success, title, body);
        dlg.exec();
    }

    // ─── Construction ───────────────────────────────────────────
    MessageDialog(QWidget* parent, Kind kind, const QString& title, const QString& body)
        : QDialog(parent), kind_(kind)
    {
        setWindowFlags(Qt::Dialog
                       | Qt::FramelessWindowHint
                       | Qt::NoDropShadowWindowHint);
        setAttribute(Qt::WA_TranslucentBackground, true);
        setModal(true);

        // The visible card sits inside the dialog's own geometry. A small
        // outer margin is kept (kOuterMargin_ pixels) so the card has room
        // to be visually separate from the dialog's edge — but no
        // QGraphicsDropShadowEffect is applied: stacking it with the
        // dialog-level QGraphicsOpacityEffect (used for fade-in) causes
        // Qt's effect renderer to crash with "A paint device can only be
        // painted by one painter at a time." The card stands out via its
        // accent left stripe, dark border, and modal grab on the parent.
        const int margin = kOuterMargin_;
        auto* outer = new QVBoxLayout(this);
        outer->setContentsMargins(margin, margin, margin, margin);
        outer->setSpacing(0);

        card_ = new QFrame(this);
        card_->setObjectName("MessageDialogCard");
        outer->addWidget(card_);

        buildCardContents(title, body);

        // Fade-in on show. The opacity effect lives on the dialog top-level
        // widget. Since we no longer use any other QGraphicsEffect anywhere
        // in this widget tree, this is now safe.
        opacityEffect_ = new QGraphicsOpacityEffect(this);
        opacityEffect_->setOpacity(0.0);
        setGraphicsEffect(opacityEffect_);
        fadeAnim_ = new QPropertyAnimation(opacityEffect_, "opacity", this);
        fadeAnim_->setDuration(150);
        fadeAnim_->setStartValue(0.0);
        fadeAnim_->setEndValue(1.0);
    }

protected:
    // ─── Layout / first-show centering + fade-in trigger ────────
    void showEvent(QShowEvent* e) override {
        QDialog::showEvent(e);
        adjustSize();
        if (auto* p = parentWidget()) {
            const QRect pr = p->frameGeometry();
            move(pr.center().x() - width()  / 2,
                 pr.center().y() - height() / 2);
        } else if (auto* scr = screen()) {
            const QRect g = scr->availableGeometry();
            move(g.center().x() - width()  / 2,
                 g.center().y() - height() / 2);
        }
        if (fadeAnim_) fadeAnim_->start();
    }

    // ─── Click-outside-card dismisses ───────────────────────────
    //
    // The dialog widget includes the shadow margin around the card. A click
    // landing on the shadow margin (visually outside the card body) should
    // dismiss the dialog. We test the point against card_->geometry() —
    // anything outside the card rect is "outside".
    //
    // Drags begin INSIDE the card and reposition the whole dialog window.
    void mousePressEvent(QMouseEvent* e) override {
        if (e->button() == Qt::LeftButton) {
            const QPoint pos = e->position().toPoint();
            if (!card_->geometry().contains(pos)) {
                // Outside the card body → dismiss.
                reject();
                return;
            }
            // Inside the card — start a drag.
            dragging_     = true;
            dragOrigin_   = e->globalPosition().toPoint() - frameGeometry().topLeft();
            e->accept();
            return;
        }
        QDialog::mousePressEvent(e);
    }
    void mouseMoveEvent(QMouseEvent* e) override {
        if (dragging_ && (e->buttons() & Qt::LeftButton)) {
            move(e->globalPosition().toPoint() - dragOrigin_);
            e->accept();
            return;
        }
        QDialog::mouseMoveEvent(e);
    }
    void mouseReleaseEvent(QMouseEvent* e) override {
        if (e->button() == Qt::LeftButton) dragging_ = false;
        QDialog::mouseReleaseEvent(e);
    }

private:
    // ─── Per-kind colour / glyph helpers ────────────────────────
    QColor accentColor() const {
        return (kind_ == Kind::Error) ? CBStyle::RED : CBStyle::GREEN;
    }
    QString primaryButtonText() const {
        return (kind_ == Kind::Error) ? QStringLiteral("Close")
                                      : QStringLiteral("OK");
    }

    // Build the card's child widgets (icon, title, body, button).
    void buildCardContents(const QString& title, const QString& body) {
        // Card-level QSS: the actual rounded shape + accent stripe are drawn
        // in IconArea::paintEvent / Card::paintEvent below. The QSS sets the
        // fill colour for the QFrame's automatic background draw so child
        // widgets see a consistent backdrop.
        card_->setStyleSheet(QString(
            "QFrame#MessageDialogCard{"
                "background:%1;"
                "border:1px solid %2;"
                "border-left:4px solid %3;"
                "border-radius:8px;"
            "}"
        )
        .arg(EMStyle::rgb(CBStyle::SURFACE))
        .arg(EMStyle::rgb(CBStyle::BORDER))
        .arg(EMStyle::rgb(accentColor())));

        auto* cardLayout = new QVBoxLayout(card_);
        cardLayout->setContentsMargins(20, 18, 20, 16);
        cardLayout->setSpacing(12);

        // ── Header row: type icon + title text ────────────────
        auto* headerRow = new QHBoxLayout;
        headerRow->setContentsMargins(0, 0, 0, 0);
        headerRow->setSpacing(12);

        auto* iconArea = new IconWidget(kind_, card_);
        iconArea->setFixedSize(32, 32);
        headerRow->addWidget(iconArea, 0, Qt::AlignTop);

        auto* titleLabel = new QLabel(title, card_);
        titleLabel->setStyleSheet(QString(
            "QLabel{"
            "color:%1;"
            "background:transparent;"
            "font-family:'Segoe UI','Courier New',sans-serif;"
            "font-size:13px;"
            "font-weight:bold;"
            "letter-spacing:0.5px;"
            "}"
        ).arg(EMStyle::rgb(accentColor())));
        titleLabel->setWordWrap(false);
        headerRow->addWidget(titleLabel, /*stretch*/ 1);

        cardLayout->addLayout(headerRow);

        // ── Body text ────────────────────────────────────────
        auto* bodyLabel = new QLabel(body, card_);
        bodyLabel->setStyleSheet(QString(
            "QLabel{"
            "color:%1;"
            "background:transparent;"
            "font-family:'Segoe UI','Courier New',sans-serif;"
            "font-size:11px;"
            "line-height:140%;"
            "}"
        ).arg(EMStyle::rgb(CBStyle::TEXT)));
        bodyLabel->setWordWrap(true);
        bodyLabel->setMinimumWidth(kBodyMinWidth_);
        bodyLabel->setMaximumWidth(kBodyMaxWidth_);
        bodyLabel->setTextInteractionFlags(
            Qt::TextSelectableByMouse | Qt::TextSelectableByKeyboard);
        cardLayout->addWidget(bodyLabel);

        // ── Action button row ────────────────────────────────
        auto* buttonRow = new QHBoxLayout;
        buttonRow->setContentsMargins(0, 4, 0, 0);
        buttonRow->setSpacing(8);
        buttonRow->addStretch(1);

        auto* btn = new QPushButton(primaryButtonText(), card_);
        btn->setMinimumWidth(96);
        btn->setMinimumHeight(30);
        btn->setCursor(Qt::PointingHandCursor);
        btn->setStyleSheet(buttonQSS());
        btn->setDefault(true);
        btn->setAutoDefault(true);
        // Pressing the button accepts the dialog (close + return Accepted).
        connect(btn, &QPushButton::clicked, this, &QDialog::accept);
        buttonRow->addWidget(btn);

        cardLayout->addLayout(buttonRow);
    }

    // QSS for the action button — distinct from the action-row buttons in
    // CircuitBuilderWindow's left panel; uses a solid accent fill so it
    // pulls focus as the affirmative dismissal control.
    QString buttonQSS() const {
        const QColor a = accentColor();
        return QString(
            "QPushButton{"
                "background:%1;"
                "color:white;"
                "border:none;"
                "border-radius:4px;"
                "padding:6px 14px;"
                "font-family:'Segoe UI','Courier New',sans-serif;"
                "font-size:11px;"
                "font-weight:bold;"
                "letter-spacing:1px;"
            "}"
            "QPushButton:hover{background:%2;}"
            "QPushButton:pressed{background:%3;}"
            "QPushButton:focus{outline:none;}"
        )
        .arg(EMStyle::rgb(a))
        .arg(EMStyle::rgb(a.darker(110)))
        .arg(EMStyle::rgb(a.darker(125)));
    }

    // ─── IconWidget: paints either a warning triangle (Error) or
    //                a checkmark inside a circle (Success) ─────
    class IconWidget : public QWidget {
    public:
        IconWidget(Kind kind, QWidget* parent = nullptr)
            : QWidget(parent), kind_(kind) {
            setAttribute(Qt::WA_TranslucentBackground);
        }
    protected:
        void paintEvent(QPaintEvent*) override {
            QPainter p(this);
            p.setRenderHint(QPainter::Antialiasing, true);
            const QRectF r = rect().adjusted(2, 2, -2, -2);
            const QColor accent = (kind_ == Kind::Error)
                                      ? CBStyle::RED
                                      : CBStyle::GREEN;
            if (kind_ == Kind::Error) {
                drawWarningTriangle(p, r, accent);
            } else {
                drawCheckmarkCircle(p, r, accent);
            }
        }
    private:
        // Triangle with rounded corners + a vertical exclamation stroke
        // and a base dot. Single-colour fill, no gradients.
        static void drawWarningTriangle(QPainter& p, QRectF r, const QColor& accent) {
            QPainterPath tri;
            const qreal cx = r.center().x();
            const qreal top = r.top();
            const qreal bot = r.bottom();
            const qreal left = r.left();
            const qreal right = r.right();
            tri.moveTo(cx, top);
            tri.lineTo(right, bot);
            tri.lineTo(left, bot);
            tri.closeSubpath();
            p.setPen(Qt::NoPen);
            p.setBrush(accent);
            p.drawPath(tri);
            // Exclamation
            p.setPen(QPen(Qt::white, 2.4, Qt::SolidLine, Qt::RoundCap));
            const qreal h = bot - top;
            p.drawLine(QPointF(cx, top + h * 0.40),
                       QPointF(cx, top + h * 0.70));
            p.setBrush(Qt::white);
            p.setPen(Qt::NoPen);
            p.drawEllipse(QPointF(cx, top + h * 0.83), 1.6, 1.6);
        }
        // Filled circle + white checkmark stroke.
        static void drawCheckmarkCircle(QPainter& p, QRectF r, const QColor& accent) {
            p.setPen(Qt::NoPen);
            p.setBrush(accent);
            p.drawEllipse(r);
            p.setPen(QPen(Qt::white, 2.6, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
            p.setBrush(Qt::NoBrush);
            QPainterPath check;
            const qreal w = r.width(), h = r.height();
            const QPointF a(r.left() + w * 0.28, r.top() + h * 0.52);
            const QPointF b(r.left() + w * 0.45, r.top() + h * 0.68);
            const QPointF c(r.left() + w * 0.74, r.top() + h * 0.36);
            check.moveTo(a);
            check.lineTo(b);
            check.lineTo(c);
            p.drawPath(check);
        }
        Kind kind_;
    };

    // ─── Members / constants ────────────────────────────────────
    Kind                     kind_;
    QFrame*                  card_           {nullptr};
    QGraphicsOpacityEffect*  opacityEffect_  {nullptr};
    QPropertyAnimation*      fadeAnim_       {nullptr};
    bool                     dragging_       {false};
    QPoint                   dragOrigin_;

    static constexpr int kOuterMargin_  = 8;
    static constexpr int kBodyMinWidth_ = 320;
    static constexpr int kBodyMaxWidth_ = 480;
};
