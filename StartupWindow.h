#ifndef STARTUPWINDOW_H
#define STARTUPWINDOW_H

#include <QWidget>
#include <QPushButton>
#include <QLabel>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QFrame>
#include <QPainter>
#include <QPainterPath>
#include <QFont>
#include <QGraphicsDropShadowEffect>
#include <QPropertyAnimation>

#include <cmath>

// Phase B: Circuit Builder window
#include "CircuitBuilderWindow.h"

// ============================================================================
//  [T2.1a] IconBadge — small painted glyph that replaces the emoji string
//
//  Two kinds are supported: QuickSim (a stylised lightning bolt) and
//  CircuitBuilder (a wrench glyph). Both are drawn with pure QPainter
//  primitives — no font glyphs, no external assets, no emoji.
// ============================================================================
class IconBadge : public QWidget
{
public:
    enum Kind { QuickSim, CircuitBuilder };

    IconBadge(Kind kind, const QColor& accent, QWidget* parent = nullptr)
        : QWidget(parent), m_kind(kind), m_accent(accent)
    {
        setFixedSize(46, 46);
        setAttribute(Qt::WA_TransparentForMouseEvents);
        setAttribute(Qt::WA_TranslucentBackground);
    }

protected:
    void paintEvent(QPaintEvent*) override
    {
        QPainter p(this);
        p.setRenderHint(QPainter::Antialiasing, true);
        if (m_kind == QuickSim) drawLightning(p);
        else                    drawWrench  (p);
    }

private:
    // Lightning bolt — stylised, vertical, slightly slanted
    void drawLightning(QPainter& p)
    {
        const QRectF r = rect().adjusted(8, 6, -8, -6);
        QPainterPath path;
        path.moveTo(r.left() + r.width() * 0.65, r.top());
        path.lineTo(r.left() + r.width() * 0.15, r.top() + r.height() * 0.55);
        path.lineTo(r.left() + r.width() * 0.45, r.top() + r.height() * 0.55);
        path.lineTo(r.left() + r.width() * 0.30, r.bottom());
        path.lineTo(r.left() + r.width() * 0.85, r.top() + r.height() * 0.40);
        path.lineTo(r.left() + r.width() * 0.55, r.top() + r.height() * 0.40);
        path.closeSubpath();
        p.setPen(Qt::NoPen);
        p.setBrush(m_accent);
        p.drawPath(path);
    }

    // Wrench — head + handle, drawn as two overlapping shapes
    void drawWrench(QPainter& p)
    {
        const QRectF r = rect().adjusted(6, 6, -6, -6);
        const qreal w = r.width();
        const qreal h = r.height();

        // Handle: rounded rect tilted 45 degrees, from lower-left to mid-upper-right
        p.save();
        p.translate(r.center());
        p.rotate(-45.0);
        QRectF handle(-w * 0.42, -w * 0.10, w * 0.85, w * 0.20);
        p.setPen(Qt::NoPen);
        p.setBrush(m_accent);
        p.drawRoundedRect(handle, 4.0, 4.0);
        p.restore();

        // Open-jaw head at the upper-right end of the handle
        const QPointF headCenter(
            r.left() + w * 0.74,
            r.top()  + h * 0.26);
        const qreal headR = w * 0.22;

        // Outer circle (head silhouette)
        p.setBrush(m_accent);
        p.drawEllipse(headCenter, headR, headR);

        // Inner cut-out — opens at top-right so the wrench is recognisable
        p.setBrush(palette().window().color());      // use the button's bg colour
        QPainterPath jaw;
        jaw.moveTo(headCenter.x() - headR * 0.45, headCenter.y() - headR * 0.45);
        jaw.lineTo(headCenter.x() + headR * 0.85, headCenter.y() - headR * 0.85);
        jaw.lineTo(headCenter.x() + headR * 0.85, headCenter.y() + headR * 0.15);
        jaw.closeSubpath();
        p.drawPath(jaw);

        // Inner hub (small circle in the centre of the head)
        p.setBrush(palette().window().color());
        p.drawEllipse(headCenter, headR * 0.35, headR * 0.35);
    }

    Kind   m_kind;
    QColor m_accent;
};


// ============================================================================
//  StartupWindow
// ============================================================================
class StartupWindow : public QWidget
{
    Q_OBJECT

public:
    explicit StartupWindow(QWidget* parent = nullptr)
        : QWidget(parent)
    {
        setWindowTitle("EMShieldDesigner");
        resize(780, 520);
        setMinimumSize(600, 400);
        setWindowFlags(Qt::Window |
                       Qt::WindowMinimizeButtonHint |
                       Qt::WindowMaximizeButtonHint |
                       Qt::WindowCloseButtonHint);
        setupUI();
        playEntryAnimation();
    }

signals:
    void quickSimulationClicked();
    void startupClosed();

protected:
    void closeEvent(QCloseEvent* event) override
    {
        emit startupClosed();
        QWidget::closeEvent(event);
    }

    void paintEvent(QPaintEvent*) override
    {
        QPainter p(this);
        p.setRenderHint(QPainter::Antialiasing, true);

        // Background gradient
        QLinearGradient bgGrad(0, 0, width(), height());
        bgGrad.setColorAt(0.0, QColor(237, 242, 244));
        bgGrad.setColorAt(0.5, QColor(243, 247, 248));
        bgGrad.setColorAt(1.0, QColor(234, 239, 241));
        p.fillRect(rect(), bgGrad);

        // Subtle dot grid
        p.setPen(QPen(QColor(200, 210, 218, 40), 0.5));
        const int gridSize = 40;
        for (int gx = 0; gx < width();  gx += gridSize)
            p.drawLine(gx, 0, gx, height());
        for (int gy = 0; gy < height(); gy += gridSize)
            p.drawLine(0, gy, width(), gy);

        drawEMWaves(p);

        // Top + bottom accent bar
        QLinearGradient accentGrad(0, 0, width(), 0);
        accentGrad.setColorAt(0.0, QColor(37,  99, 235,   0));
        accentGrad.setColorAt(0.3, QColor(37,  99, 235, 140));
        accentGrad.setColorAt(0.7, QColor(59, 130, 246, 140));
        accentGrad.setColorAt(1.0, QColor(37,  99, 235,   0));
        p.setPen(Qt::NoPen);
        p.setBrush(accentGrad);
        p.drawRect(0, 0,          width(), 3);
        p.drawRect(0, height()-2, width(), 2);
    }

private slots:
    void onCircuitBuilderClicked()
    {
        if (m_builderWindow) {
            m_builderWindow->raise();
            m_builderWindow->activateWindow();
            return;
        }

        m_builderWindow = new CircuitBuilderWindow(nullptr);

        connect(m_builderWindow, &QObject::destroyed, this, [this]() {
            m_builderWindow = nullptr;
            show();
            setWindowOpacity(0.0);
            auto* fadeIn = new QPropertyAnimation(this, "windowOpacity");
            fadeIn->setDuration(250);
            fadeIn->setStartValue(0.0);
            fadeIn->setEndValue(1.0);
            fadeIn->setEasingCurve(QEasingCurve::OutCubic);
            fadeIn->start(QAbstractAnimation::DeleteWhenStopped);
        });

        m_builderWindow->setAttribute(Qt::WA_DeleteOnClose);
        m_builderWindow->show();
        hide();
    }

private:
    CircuitBuilderWindow* m_builderWindow { nullptr };

    void drawEMWaves(QPainter& p)
    {
        p.save();
        for (int wave = 0; wave < 3; ++wave) {
            QPainterPath path;
            const double amplitude = 15.0 + wave * 8.0;
            const double yBase     = 130.0 + wave * 25.0;
            const int    alpha     = 30 - wave * 6;
            path.moveTo(0, yBase);
            for (int wx = 0; wx <= width(); wx += 2) {
                double wy = yBase + amplitude *
                                        std::sin(wx * 0.015 + wave * 1.2);
                path.lineTo(wx, wy);
            }
            p.setPen(QPen(QColor(59, 130, 246, alpha), 1.0));
            p.setBrush(Qt::NoBrush);
            p.drawPath(path);
        }
        drawShieldIcon(p, width() - 90.0, height() - 110.0, 60.0);
        p.restore();
    }

    void drawShieldIcon(QPainter& p, double cx, double cy, double size)
    {
        QPainterPath shield;
        const double sw = size * 0.5;
        const double sh = size * 0.65;
        shield.moveTo(cx, cy - sh);
        shield.cubicTo(cx + sw, cy - sh * 0.7,
                       cx + sw, cy + sh * 0.1, cx, cy + sh);
        shield.cubicTo(cx - sw, cy + sh * 0.1,
                       cx - sw, cy - sh * 0.7, cx, cy - sh);
        p.setPen(QPen(QColor(59, 130, 246, 35), 1.5));
        p.setBrush(QColor(59, 130, 246, 12));
        p.drawPath(shield);
        p.setPen(QColor(59, 130, 246, 45));
        p.setFont(QFont("Georgia", 14, QFont::Bold));
        p.drawText(QRectF(cx - sw, cy - sh * 0.3, 2*sw, sh * 0.6),
                   Qt::AlignCenter, "SE");
    }

    // ── UI ───────────────────────────────────────────────────────────────
    void setupUI()
    {
        QVBoxLayout* mainLayout = new QVBoxLayout(this);
        mainLayout->setContentsMargins(50, 45, 50, 40);
        mainLayout->setSpacing(0);

        m_lblTitle = new QLabel("EMShieldDesigner");
        m_lblTitle->setAlignment(Qt::AlignCenter);
        m_lblTitle->setStyleSheet(
            "QLabel {"
            "  color: #1e293b;"
            "  font-size: 32px;"
            "  font-weight: 700;"
            "  letter-spacing: 2px;"
            "  font-family: 'Segoe UI', 'Calibri', sans-serif;"
            "}");

        m_lblSubtitle = new QLabel(
            "Shielding Effectiveness Analyzer\n"
            "Using Equivalent Circuit Method and Nodal Analysis");
        m_lblSubtitle->setAlignment(Qt::AlignCenter);
        m_lblSubtitle->setStyleSheet(
            "QLabel {"
            "  color: #64748b;"
            "  font-size: 13px;"
            "  font-weight: 400;"
            "  font-family: 'Segoe UI', 'Calibri', sans-serif;"
            "}");

        QFrame* divider = new QFrame;
        divider->setFixedHeight(1);
        divider->setStyleSheet("background-color: rgba(100, 116, 139, 0.2);");

        mainLayout->addStretch(2);
        mainLayout->addWidget(m_lblTitle);
        mainLayout->addSpacing(10);
        mainLayout->addWidget(m_lblSubtitle);
        mainLayout->addSpacing(20);
        mainLayout->addWidget(divider);
        mainLayout->addStretch(2);

        // Buttons
        QHBoxLayout* btnLayout = new QHBoxLayout;
        btnLayout->setSpacing(30);

        m_btnQuick = createModeButton(
            "Quick Simulation",
            "Preset configurations with\ninteractive parameter control",
            QColor(37, 99, 235),
            IconBadge::QuickSim);
        connect(m_btnQuick, &QPushButton::clicked,
                this, &StartupWindow::quickSimulationClicked);

        m_btnBuilder = createModeButton(
            "Circuit Builder",
            "Drag & drop elements to build\ncustom equivalent circuits",
            QColor(22, 163, 74),
            IconBadge::CircuitBuilder);
        connect(m_btnBuilder, &QPushButton::clicked,
                this, &StartupWindow::onCircuitBuilderClicked);

        btnLayout->addWidget(m_btnQuick);
        btnLayout->addWidget(m_btnBuilder);
        mainLayout->addLayout(btnLayout);
        mainLayout->addStretch(1);

        QLabel* lblFooter = new QLabel("Electromagnetic Compatibility Research Tool");
        lblFooter->setAlignment(Qt::AlignCenter);
        lblFooter->setStyleSheet(
            "QLabel {"
            "  color: #94a3b8;"
            "  font-size: 10px;"
            "  font-family: 'Segoe UI', sans-serif;"
            "}");
        mainLayout->addWidget(lblFooter);
    }

    // [T2.1a] createModeButton: the `icon` string parameter has been
    // replaced by an IconBadge::Kind, so the mode glyph is painted by
    // QPainter instead of being rendered as an emoji character. The
    // rest of the button styling is unchanged from the original.
    QPushButton* createModeButton(const QString&    title,
                                  const QString&    description,
                                  const QColor&     accent,
                                  IconBadge::Kind   iconKind)
    {
        QPushButton* btn = new QPushButton;
        btn->setFixedSize(300, 150);
        btn->setCursor(Qt::PointingHandCursor);

        QVBoxLayout* bLayout = new QVBoxLayout(btn);
        bLayout->setContentsMargins(16, 12, 16, 16);
        bLayout->setSpacing(4);

        // Icon row — centred IconBadge instead of an emoji label
        auto* iconRow = new QHBoxLayout;
        iconRow->setContentsMargins(0, 0, 0, 0);
        iconRow->addStretch(1);
        auto* badge = new IconBadge(iconKind, accent, btn);
        iconRow->addWidget(badge);
        iconRow->addStretch(1);
        bLayout->addLayout(iconRow);

        auto makeLabel = [](const QString& text, const QString& style,
                            QWidget* parent) -> QLabel* {
            QLabel* lbl = new QLabel(text, parent);
            lbl->setAlignment(Qt::AlignCenter);
            lbl->setStyleSheet(style);
            lbl->setAttribute(Qt::WA_TransparentForMouseEvents);
            return lbl;
        };

        bLayout->addWidget(makeLabel(title,
                                     "font-size: 16px; font-weight: 700; color: #1e293b;"
                                     " font-family: 'Segoe UI', sans-serif; background: transparent;", btn));
        bLayout->addWidget(makeLabel(description,
                                     "font-size: 11px; color: #64748b;"
                                     " font-family: 'Segoe UI', sans-serif; background: transparent;", btn));

        const int r = accent.red(), g = accent.green(), b = accent.blue();
        btn->setStyleSheet(QString(
                               "QPushButton {"
                               "  background-color: rgba(255,255,255,0.7);"
                               "  border: 1.5px solid rgba(%1,%2,%3,0.25);"
                               "  border-radius: 12px;"
                               "}"
                               "QPushButton:hover {"
                               "  background-color: rgba(255,255,255,0.9);"
                               "  border: 1.5px solid rgba(%1,%2,%3,0.5);"
                               "}"
                               "QPushButton:pressed {"
                               "  background-color: rgba(%1,%2,%3,0.08);"
                               "  border: 1.5px solid rgba(%1,%2,%3,0.7);"
                               "}").arg(r).arg(g).arg(b));

        QGraphicsDropShadowEffect* shadow = new QGraphicsDropShadowEffect;
        shadow->setBlurRadius(20);
        shadow->setOffset(0, 4);
        shadow->setColor(QColor(0, 0, 0, 60));
        btn->setGraphicsEffect(shadow);

        return btn;
    }

    void playEntryAnimation()
    {
        setWindowOpacity(0.0);
        auto* fadeIn = new QPropertyAnimation(this, "windowOpacity");
        fadeIn->setDuration(400);
        fadeIn->setStartValue(0.0);
        fadeIn->setEndValue(1.0);
        fadeIn->setEasingCurve(QEasingCurve::OutCubic);
        fadeIn->start(QAbstractAnimation::DeleteWhenStopped);
    }

    // Widgets
    QLabel*      m_lblTitle    { nullptr };
    QLabel*      m_lblSubtitle { nullptr };
    QPushButton* m_btnQuick    { nullptr };
    QPushButton* m_btnBuilder  { nullptr };
};

#endif // STARTUPWINDOW_H
