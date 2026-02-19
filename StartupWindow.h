#ifndef STARTUPWINDOW_H
#define STARTUPWINDOW_H

#include <QWidget>
#include <QPushButton>
#include <QLabel>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QPainter>
#include <QPainterPath>
#include <QFont>
#include <QGraphicsDropShadowEffect>
#include <QPropertyAnimation>
#include <QSequentialAnimationGroup>
#include <QGraphicsOpacityEffect>
#include <QTimer>

// ── Phase B: Circuit Builder window ──────────────────────────
#include "CircuitBuilderWindow.h"

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
        setWindowFlags(Qt::Window | Qt::WindowMinimizeButtonHint |
                       Qt::WindowMaximizeButtonHint | Qt::WindowCloseButtonHint);

        setupUI();
        playEntryAnimation();
    }

signals:
    void quickSimulationClicked();
    // circuitBuilderClicked is no longer a signal —
    // the launch logic lives entirely in the slot below.

protected:
    void paintEvent(QPaintEvent*) override {
        QPainter p(this);
        p.setRenderHint(QPainter::Antialiasing, true);

        // Background gradient (light grey)
        QLinearGradient bgGrad(0, 0, width(), height());
        bgGrad.setColorAt(0.0, QColor(237, 242, 244));
        bgGrad.setColorAt(0.5, QColor(243, 247, 248));
        bgGrad.setColorAt(1.0, QColor(234, 239, 241));
        p.fillRect(rect(), bgGrad);

        // Subtle grid pattern
        p.setPen(QPen(QColor(200, 210, 218, 40), 0.5));
        int gridSize = 40;
        for (int x = 0; x < width(); x += gridSize)
            p.drawLine(x, 0, x, height());
        for (int y = 0; y < height(); y += gridSize)
            p.drawLine(0, y, width(), y);

        drawEMWaves(p);

        // Top accent line
        QLinearGradient accentGrad(0, 0, width(), 0);
        accentGrad.setColorAt(0.0, QColor(37, 99, 235, 0));
        accentGrad.setColorAt(0.3, QColor(37, 99, 235, 140));
        accentGrad.setColorAt(0.7, QColor(59, 130, 246, 140));
        accentGrad.setColorAt(1.0, QColor(37, 99, 235, 0));
        p.setPen(Qt::NoPen);
        p.setBrush(accentGrad);
        p.drawRect(0, 0, width(), 3);
        p.drawRect(0, height() - 2, width(), 2);
    }

private slots:
    // ── Launches CircuitBuilderWindow, hides (not destroys) StartupWindow ──
    void onCircuitBuilderClicked()
    {
        // Guard: only one instance at a time
        if (m_builderWindow) {
            m_builderWindow->raise();
            m_builderWindow->activateWindow();
            return;
        }

        m_builderWindow = new CircuitBuilderWindow(nullptr);

        // When the builder closes, show the startup window again
        // so the user can switch modes without restarting the app.
        connect(m_builderWindow, &QObject::destroyed, this, [this]() {
            m_builderWindow = nullptr;
            this->show();
            // Brief fade-back-in
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
        this->hide();   // hide (not close) so we can return to it
    }

private:
    // ── Stored pointer so we can guard against double-launch ──
    CircuitBuilderWindow* m_builderWindow { nullptr };

    void drawEMWaves(QPainter& p) {
        p.save();
        for (int wave = 0; wave < 3; ++wave) {
            QPainterPath path;
            double amplitude = 15 + wave * 8;
            double yOffset   = 130 + wave * 25;
            int    alpha     = 30 - wave * 6;
            path.moveTo(0, yOffset);
            for (int x = 0; x <= width(); x += 2) {
                double y = yOffset + amplitude * std::sin(x * 0.015 + wave * 1.2);
                path.lineTo(x, y);
            }
            p.setPen(QPen(QColor(59, 130, 246, alpha), 1.0));
            p.setBrush(Qt::NoBrush);
            p.drawPath(path);
        }
        drawShieldIcon(p, width() - 90, height() - 110, 60);
        p.restore();
    }

    void drawShieldIcon(QPainter& p, double cx, double cy, double size) {
        QPainterPath shield;
        double w = size * 0.5, h = size * 0.65;
        shield.moveTo(cx, cy - h);
        shield.cubicTo(cx + w, cy - h * 0.7, cx + w, cy + h * 0.1, cx, cy + h);
        shield.cubicTo(cx - w, cy + h * 0.1, cx - w, cy - h * 0.7, cx, cy - h);
        p.setPen(QPen(QColor(59, 130, 246, 35), 1.5));
        p.setBrush(QColor(59, 130, 246, 12));
        p.drawPath(shield);
        p.setPen(QColor(59, 130, 246, 45));
        p.setFont(QFont("Georgia", 14, QFont::Bold));
        p.drawText(QRectF(cx - w, cy - h * 0.3, 2 * w, h * 0.6), Qt::AlignCenter, "SE");
    }

    void setupUI() {
        QVBoxLayout* mainLayout = new QVBoxLayout(this);
        mainLayout->setContentsMargins(50, 45, 50, 40);
        mainLayout->setSpacing(0);

        // ── Title ────────────────────────────────────────────
        m_lblTitle = new QLabel("EMShieldDesigner");
        m_lblTitle->setAlignment(Qt::AlignCenter);
        m_lblTitle->setStyleSheet(
            "QLabel {"
            "  color: #1e293b;"
            "  font-size: 32px;"
            "  font-weight: 700;"
            "  letter-spacing: 2px;"
            "  font-family: 'Segoe UI', 'Calibri', sans-serif;"
            "}"
            );

        m_lblSubtitle = new QLabel(
            "Shielding Effectiveness Analyzer\n"
            "Using Equivalent Circuit Method and Nodal Analysis"
            );
        m_lblSubtitle->setAlignment(Qt::AlignCenter);
        m_lblSubtitle->setStyleSheet(
            "QLabel {"
            "  color: #64748b;"
            "  font-size: 13px;"
            "  font-weight: 400;"
            "  line-height: 1.6;"
            "  font-family: 'Segoe UI', 'Calibri', sans-serif;"
            "}"
            );

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

        // ── Buttons ───────────────────────────────────────────
        QHBoxLayout* btnLayout = new QHBoxLayout;
        btnLayout->setSpacing(30);

        // Quick Simulation (unchanged — wired via signal in main.cpp)
        m_btnQuick = createModeButton(
            "Quick Simulation",
            "Preset configurations with\n"
            "interactive parameter control",
            QColor(37, 99, 235),
            "⚡"
            );
        connect(m_btnQuick, &QPushButton::clicked,
                this, &StartupWindow::quickSimulationClicked);

        // Circuit Builder (wired directly to slot — no signal needed)
        m_btnBuilder = createModeButton(
            "Circuit Builder",
            "Drag & drop elements to build\n"
            "custom equivalent circuits",
            QColor(22, 163, 74),
            "🔧"
            );
        connect(m_btnBuilder, &QPushButton::clicked,
                this, &StartupWindow::onCircuitBuilderClicked);

        btnLayout->addWidget(m_btnQuick);
        btnLayout->addWidget(m_btnBuilder);
        mainLayout->addLayout(btnLayout);
        mainLayout->addStretch(1);

        // ── Footer ────────────────────────────────────────────
        QLabel* lblFooter = new QLabel("Electromagnetic Compatibility Research Tool");
        lblFooter->setAlignment(Qt::AlignCenter);
        lblFooter->setStyleSheet(
            "QLabel {"
            "  color: #94a3b8;"
            "  font-size: 10px;"
            "  font-family: 'Segoe UI', sans-serif;"
            "}"
            );
        mainLayout->addWidget(lblFooter);
    }

    QPushButton* createModeButton(const QString& title, const QString& description,
                                  const QColor& accent, const QString& icon)
    {
        QPushButton* btn = new QPushButton;
        btn->setFixedSize(300, 150);
        btn->setCursor(Qt::PointingHandCursor);

        QVBoxLayout* btnLayout = new QVBoxLayout(btn);
        btnLayout->setContentsMargins(16, 16, 16, 16);

        QLabel* lblIcon = new QLabel(icon);
        lblIcon->setAlignment(Qt::AlignCenter);
        lblIcon->setStyleSheet("font-size: 30px; background: transparent;");
        lblIcon->setAttribute(Qt::WA_TransparentForMouseEvents);

        QLabel* lblTitle = new QLabel(title);
        lblTitle->setAlignment(Qt::AlignCenter);
        lblTitle->setStyleSheet(
            "font-size: 16px; font-weight: 700; color: #1e293b;"
            " font-family: 'Segoe UI', sans-serif; background: transparent;"
            );
        lblTitle->setAttribute(Qt::WA_TransparentForMouseEvents);

        QLabel* lblDesc = new QLabel(description);
        lblDesc->setAlignment(Qt::AlignCenter);
        lblDesc->setStyleSheet(
            "font-size: 11px; color: #64748b;"
            " font-family: 'Segoe UI', sans-serif; background: transparent;"
            );
        lblDesc->setAttribute(Qt::WA_TransparentForMouseEvents);

        btnLayout->addWidget(lblIcon);
        btnLayout->addWidget(lblTitle);
        btnLayout->addWidget(lblDesc);

        int r = accent.red(), g = accent.green(), b = accent.blue();
        btn->setStyleSheet(QString(
                               "QPushButton {"
                               "  background-color: rgba(255, 255, 255, 0.7);"
                               "  border: 1.5px solid rgba(%1, %2, %3, 0.25);"
                               "  border-radius: 12px;"
                               "}"
                               "QPushButton:hover {"
                               "  background-color: rgba(255, 255, 255, 0.9);"
                               "  border: 1.5px solid rgba(%1, %2, %3, 0.5);"
                               "}"
                               "QPushButton:pressed {"
                               "  background-color: rgba(%1, %2, %3, 0.08);"
                               "  border: 1.5px solid rgba(%1, %2, %3, 0.7);"
                               "}"
                               ).arg(r).arg(g).arg(b));

        QGraphicsDropShadowEffect* shadow = new QGraphicsDropShadowEffect;
        shadow->setBlurRadius(20);
        shadow->setOffset(0, 4);
        shadow->setColor(QColor(0, 0, 0, 60));
        btn->setGraphicsEffect(shadow);

        return btn;
    }

    void playEntryAnimation() {
        setWindowOpacity(0.0);
        auto* fadeIn = new QPropertyAnimation(this, "windowOpacity");
        fadeIn->setDuration(400);
        fadeIn->setStartValue(0.0);
        fadeIn->setEndValue(1.0);
        fadeIn->setEasingCurve(QEasingCurve::OutCubic);
        fadeIn->start(QAbstractAnimation::DeleteWhenStopped);
    }

    // Widget pointers
    QLabel*               m_lblTitle    { nullptr };
    QLabel*               m_lblSubtitle { nullptr };
    QPushButton*          m_btnQuick    { nullptr };
    QPushButton*          m_btnBuilder  { nullptr };
};

#endif // STARTUPWINDOW_H
