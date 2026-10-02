#ifndef STARTUPWINDOW_H
#define STARTUPWINDOW_H

#include <QApplication>
#include <QCoreApplication>
#include <QEvent>
#include <QFrame>
#include <QHBoxLayout>
#include <QLabel>
#include <QPainter>
#include <QPushButton>
#include <QSettings>
#include <QTranslator>
#include <QVariant>
#include <QVBoxLayout>
#include <QWidget>

// Circuit Builder window (opened from this launcher)
#include "CircuitBuilderWindow.h"

// ============================================================================
//  StartupWindow — mode launcher (Direction A, October 2026)
//
//  Plain launcher: product name and one line of description, a language
//  switch, two full-width mode buttons and a version line. No painted
//  backgrounds, badges, shadows or entry animations.
// ============================================================================
class StartupWindow : public QWidget
{
    Q_OBJECT

public:
    explicit StartupWindow(QWidget* parent = nullptr)
        : QWidget(parent)
    {
        setWindowTitle("EMShieldDesigner");
        resize(640, 420);
        setMinimumSize(560, 380);
        setWindowFlags(Qt::Window |
                       Qt::WindowMinimizeButtonHint |
                       Qt::WindowMaximizeButtonHint |
                       Qt::WindowCloseButtonHint);
        setupUI();
        // [P3c.1-fix] Adopt the launch-time translator main.cpp installed (when
        // launched in a non-source language) so setLanguage() can remove it on
        // a switch back to English. Null when launched in English.
        m_translator = qobject_cast<QTranslator*>(
            qApp->property("emshield_translator").value<QObject*>());
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
        p.fillRect(rect(), CBStyle::SURFACE);
    }

    // [P3c.1] Qt posts LanguageChange to every top-level widget when a
    // translator is installed/removed. Catch it and re-translate our own text.
    void changeEvent(QEvent* e) override
    {
        if (e->type() == QEvent::LanguageChange)
            retranslateUi();
        QWidget::changeEvent(e);
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
        });

        m_builderWindow->setAttribute(Qt::WA_DeleteOnClose);
        m_builderWindow->show();
        hide();
    }

private:
    CircuitBuilderWindow* m_builderWindow { nullptr };

    // ── UI ───────────────────────────────────────────────────────────────
    void setupUI()
    {
        auto* mainLayout = new QVBoxLayout(this);
        mainLayout->setContentsMargins(36, 28, 36, 24);
        mainLayout->setSpacing(0);

        // Header: product name + language switch
        auto* headRow = new QHBoxLayout;
        headRow->setContentsMargins(0, 0, 0, 0);
        m_lblTitle = new QLabel(QStringLiteral("EMShieldDesigner"), this);
        m_lblTitle->setStyleSheet(QString(
            "QLabel{color:%1;font-family:'Segoe UI';font-size:22px;font-weight:600;"
            "background:transparent;}").arg(EMStyle::rgb(CBStyle::TEXT)));
        headRow->addWidget(m_lblTitle, 0, Qt::AlignBottom);
        headRow->addStretch(1);

        m_btnLangEn = new QPushButton(QStringLiteral("English"), this);
        m_btnLangRu = new QPushButton(QStringLiteral("Русский"), this);
        for (QPushButton* b : { m_btnLangEn, m_btnLangRu }) {
            b->setCursor(Qt::PointingHandCursor);
            b->setFixedHeight(26);
        }
        connect(m_btnLangEn, &QPushButton::clicked, this,
                [this]{ setLanguage(QStringLiteral("en")); });
        connect(m_btnLangRu, &QPushButton::clicked, this,
                [this]{ setLanguage(QStringLiteral("ru")); });
        auto* langRow = new QHBoxLayout;
        langRow->setContentsMargins(0, 0, 0, 0);
        langRow->setSpacing(0);
        langRow->addWidget(m_btnLangEn);
        langRow->addWidget(m_btnLangRu);
        headRow->addLayout(langRow);
        mainLayout->addLayout(headRow);

        mainLayout->addSpacing(6);
        m_lblSubtitle = new QLabel(this);   // text set in retranslateUi()
        m_lblSubtitle->setWordWrap(true);
        m_lblSubtitle->setStyleSheet(QString(
            "QLabel{color:%1;font-family:'Segoe UI';font-size:13px;background:transparent;}")
            .arg(EMStyle::rgb(CBStyle::TEXT_MUTED)));
        mainLayout->addWidget(m_lblSubtitle);

        mainLayout->addSpacing(22);
        auto* divider = new QFrame(this);
        divider->setFixedHeight(1);
        divider->setStyleSheet(QString("QFrame{background:%1;border:none;}")
                                   .arg(EMStyle::rgb(CBStyle::BORDER_LT)));
        mainLayout->addWidget(divider);
        mainLayout->addSpacing(22);

        // Mode buttons
        m_btnQuick = createModeButton(m_lblQuickTitle, m_lblQuickDesc);
        connect(m_btnQuick, &QPushButton::clicked,
                this, &StartupWindow::quickSimulationClicked);
        m_btnBuilder = createModeButton(m_lblBuilderTitle, m_lblBuilderDesc);
        connect(m_btnBuilder, &QPushButton::clicked,
                this, &StartupWindow::onCircuitBuilderClicked);
        mainLayout->addWidget(m_btnQuick);
        mainLayout->addSpacing(10);
        mainLayout->addWidget(m_btnBuilder);
        mainLayout->addStretch(1);

        m_lblFooter = new QLabel(this);     // text set in retranslateUi()
        m_lblFooter->setStyleSheet(QString(
            "QLabel{color:%1;font-family:'Segoe UI';font-size:11px;background:transparent;}")
            .arg(EMStyle::rgb(CBStyle::TEXT_DIM)));
        mainLayout->addWidget(m_lblFooter);

        retranslateUi();

        // [P3c] Reflect the persisted language in the switch's initial state.
        QSettings settings;
        applyLangStyles(settings.value(QStringLiteral("language"),
                                       QStringLiteral("en")).toString());
    }

    // Full-width mode button: title and one-line description, left-aligned.
    // Title/description labels are returned so retranslateUi() can refill them.
    QPushButton* createModeButton(QLabel*& outTitle, QLabel*& outDesc)
    {
        auto* btn = new QPushButton(this);
        btn->setCursor(Qt::PointingHandCursor);
        btn->setMinimumHeight(68);

        auto* layout = new QVBoxLayout(btn);
        layout->setContentsMargins(18, 12, 18, 12);
        layout->setSpacing(3);

        auto makeLabel = [btn](const QString& style) {
            auto* lbl = new QLabel(btn);
            lbl->setStyleSheet(style);
            lbl->setAttribute(Qt::WA_TransparentForMouseEvents);
            return lbl;
        };
        outTitle = makeLabel(QString(
            "QLabel{color:%1;font-family:'Segoe UI';font-size:14px;font-weight:600;"
            "background:transparent;}").arg(EMStyle::rgb(CBStyle::TEXT)));
        outDesc = makeLabel(QString(
            "QLabel{color:%1;font-family:'Segoe UI';font-size:12px;background:transparent;}")
            .arg(EMStyle::rgb(CBStyle::TEXT_MUTED)));
        layout->addWidget(outTitle);
        layout->addWidget(outDesc);

        btn->setStyleSheet(QString(
            "QPushButton{background:%1;border:1px solid %2;border-radius:6px;text-align:left;}"
            "QPushButton:hover{border:1px solid %3;background:%4;}"
            "QPushButton:pressed{background:%5;border:1px solid %3;}"
            "QPushButton:focus{outline:none;border:1px solid %3;}")
            .arg(EMStyle::rgb(CBStyle::BG))
            .arg(EMStyle::rgb(CBStyle::BORDER_LT))
            .arg(EMStyle::rgb(CBStyle::ACCENT))
            .arg(EMStyle::rgb(CBStyle::BG))
            .arg(EMStyle::rgb(CBStyle::SURFACE2)));
        return btn;
    }

    // ── [P3c] Language switch ────────────────────────────────────────────
    // A click SAVES the choice and installs/removes the translator so the
    // next window (Quick Simulation / Circuit Builder) opens in that language.
    // This window is retranslated live via changeEvent()/retranslateUi().
    // [P3c.1-fix] Works in both directions from any launch state: the
    // launch-time translator from main.cpp is adopted into m_translator in the
    // constructor (via the "emshield_translator" qApp property).
    void applyLangStyles(const QString& cur)
    {
        auto styleFor = [](bool selected, bool left) -> QString {
            const QString radius = left
                ? QStringLiteral("border-top-left-radius:4px;border-bottom-left-radius:4px;")
                : QStringLiteral("border-top-right-radius:4px;border-bottom-right-radius:4px;"
                                 "border-left:none;");
            return QString(
                "QPushButton{background:%1;color:%2;border:1px solid %3;%4"
                "padding:0px 12px;font-family:'Segoe UI';font-size:12px;%5}"
                "QPushButton:hover{color:%6;}")
                .arg(EMStyle::rgb(selected ? CBStyle::SURFACE2 : CBStyle::BG))
                .arg(EMStyle::rgb(selected ? CBStyle::ACCENT : CBStyle::TEXT_MUTED))
                .arg(EMStyle::rgb(CBStyle::BORDER))
                .arg(radius)
                .arg(selected ? QStringLiteral("font-weight:600;") : QString())
                .arg(EMStyle::rgb(CBStyle::ACCENT));
        };
        m_btnLangEn->setStyleSheet(styleFor(cur == QStringLiteral("en"), true));
        m_btnLangRu->setStyleSheet(styleFor(cur == QStringLiteral("ru"), false));
    }

    void setLanguage(const QString& code)
    {
        QSettings settings;
        const QString current = settings.value(QStringLiteral("language"),
                                               QStringLiteral("en")).toString();
        if (code == current) return;   // already selected

        settings.setValue(QStringLiteral("language"), code);

        if (code == QStringLiteral("ru")) {
            if (!m_translator) {
                m_translator = new QTranslator(this);
                if (!m_translator->load(QStringLiteral("em-shield-designer_ru"),
                                        QStringLiteral(":/i18n")))
                    qWarning("StartupWindow: failed to load em-shield-designer_ru.qm");
            }
            qApp->installTranslator(m_translator);   // safe to call repeatedly
            qApp->setProperty("emshield_translator",
                              QVariant::fromValue<QObject*>(m_translator));
        } else {
            if (m_translator) qApp->removeTranslator(m_translator);
            qApp->setProperty("emshield_translator", QVariant());
        }

        applyLangStyles(code);
    }

    // [P3c.1] Reset every translatable label to the current language. The
    // product name and the native language names stay fixed.
    void retranslateUi()
    {
        if (!m_lblSubtitle || !m_lblFooter) return;   // guard before setupUI
        m_lblSubtitle->setText(tr(
            "Shielding effectiveness of metallic enclosures by the equivalent-circuit "
            "method and modified nodal analysis"));
        m_lblFooter->setText(tr("Version %1 · TUSUR")
                                 .arg(QCoreApplication::applicationVersion()));
        if (m_lblQuickTitle)   m_lblQuickTitle->setText(tr("Quick Simulation"));
        if (m_lblQuickDesc)    m_lblQuickDesc->setText(
            tr("Enclosures defined by parameters or presets, with one or more sections"));
        if (m_lblBuilderTitle) m_lblBuilderTitle->setText(tr("Circuit Builder"));
        if (m_lblBuilderDesc)  m_lblBuilderDesc->setText(
            tr("Equivalent circuits assembled from elements, for non-standard configurations"));
    }

    // Widgets
    QLabel*      m_lblTitle    { nullptr };
    QLabel*      m_lblSubtitle { nullptr };
    QPushButton* m_btnQuick    { nullptr };
    QPushButton* m_btnBuilder  { nullptr };

    // [P3c] Language switch
    QPushButton* m_btnLangEn   { nullptr };
    QPushButton* m_btnLangRu   { nullptr };
    QTranslator* m_translator  { nullptr };

    // [P3c.1] Retranslatable labels (footer + the four mode-button labels).
    QLabel* m_lblFooter        { nullptr };
    QLabel* m_lblQuickTitle    { nullptr };
    QLabel* m_lblQuickDesc     { nullptr };
    QLabel* m_lblBuilderTitle  { nullptr };
    QLabel* m_lblBuilderDesc   { nullptr };
};

#endif // STARTUPWINDOW_H
