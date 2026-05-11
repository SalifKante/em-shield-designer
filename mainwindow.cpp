// ============================================================================
//  mainwindow.cpp — Quick Simulation window (Window 1)
//
//  [T2.1a] UI redesign — see header for full task summary.
// ============================================================================

#include "mainwindow.h"
#include "qcustomplot.h"
#include "CircuitCanvas.h"
#include "PropertyPanel.h"
#include "SectionItem.h"

// Core engine
#include "include/core/CircuitGenerator.h"

// Unified design tokens + helpers (Window 2 has been using these since
// Task 1.2; we now share them with Window 1).
#include "CircuitBuilderWindow.h"   // brings in CBStyle palette + CBSTYLE_DECLARED guard
#include "Styles.h"                 // EMStyle helpers (spinSS, lblSS, comboSS, ...)
#include "MessageDialog.h"          // [T1.6] custom error/success dialog

#include <QApplication>
#include <QMouseEvent>
#include <QSplitter>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QDoubleSpinBox>
#include <QSpinBox>
#include <QComboBox>
#include <QPushButton>
#include <QScrollArea>
#include <QStatusBar>
#include <QFileDialog>
#include <QElapsedTimer>
#include <QFrame>
#include <QIcon>

#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <string>

using namespace EMCore;

// ============================================================================
//  Constants used in this translation unit
// ============================================================================
namespace {
// Width of the left-panel column. Matches the Circuit Builder convention.
constexpr int kLeftPanelMinWidth = 260;
constexpr int kLeftPanelMaxWidth = 320;
} // anonymous namespace


// ============================================================================
//  CONSTRUCTOR / DESTRUCTOR
// ============================================================================

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
{
    setAttribute(Qt::WA_QuitOnClose, false);
    setWindowTitle("Quick Simulation — EMShieldDesigner");
    setWindowIcon(QIcon(":/icons/emshield_256.png"));
    resize(1400, 800);

    // Window-level background — keeps the application palette unified.
    setStyleSheet(QString(
                      "QMainWindow{background:%1;}"
                      "QStatusBar{background:%2;color:%3;"
                      "font-family:'Courier New';font-size:10px;border-top:1px solid %4;}"
                      )
                      .arg(EMStyle::rgb(CBStyle::BG))
                      .arg(EMStyle::rgb(CBStyle::SURFACE))
                      .arg(EMStyle::rgb(CBStyle::TEXT_MUTED))
                      .arg(EMStyle::rgb(CBStyle::BORDER_LT)));

    setupUI();
    setupStatusBar();

    // Load default preset (3-section)
    m_cboPreset->setCurrentIndex(2);
    loadPreset(3);

    // Run initial analysis so user sees a non-empty plot on first open.
    onComputeClicked();
}

MainWindow::~MainWindow() {}


// ============================================================================
//  UI SETUP
// ============================================================================

void MainWindow::setupUI()
{
    m_splitter = new QSplitter(Qt::Horizontal, this);
    m_splitter->setHandleWidth(2);
    setCentralWidget(m_splitter);

    // ── Left side: brand strip + section stack + primary buttons ─────────
    m_leftPanel = buildLeftPanel();
    m_splitter->addWidget(m_leftPanel);

    // ── Right side: vertical split with canvas on top, plot below ────────
    m_canvas = new CircuitCanvas;
    m_canvas->setMinimumHeight(120);

    setupPlot();

    auto* rightSpl = new QSplitter(Qt::Vertical);
    rightSpl->setHandleWidth(2);
    rightSpl->addWidget(m_canvas);
    rightSpl->addWidget(m_plot);
    rightSpl->setStretchFactor(0, 1);
    rightSpl->setStretchFactor(1, 3);
    rightSpl->setSizes({200, 580});
    rightSpl->setCollapsible(0, false);
    rightSpl->setCollapsible(1, false);

    m_splitter->addWidget(rightSpl);
    m_splitter->setStretchFactor(0, 0);
    m_splitter->setStretchFactor(1, 1);
    m_splitter->setSizes({kLeftPanelMinWidth + 20, 1130});

    // ── Signal wiring ────────────────────────────────────────────────────
    connect(m_canvas, &CircuitCanvas::selectionChanged,
            this,     &MainWindow::onCanvasSelectionChanged);
    connect(m_canvas, &CircuitCanvas::sectionCountChanged,
            this,     &MainWindow::onSectionCountChanged);

    connect(m_propertyPanel, &PropertyPanel::dataChanged,
            this,             &MainWindow::onPropertyChanged);
}

QWidget* MainWindow::buildLeftPanel()
{
    // ── Container ────────────────────────────────────────────────────────
    auto* panel = new QWidget;
    panel->setObjectName("QuickSimLeftPanel");
    panel->setMinimumWidth(kLeftPanelMinWidth);
    panel->setMaximumWidth(kLeftPanelMaxWidth);
    panel->setStyleSheet(QString(
                             "QWidget#QuickSimLeftPanel{"
                             "background:%1;"
                             "border-right:1px solid %2;}"
                             )
                             .arg(EMStyle::rgb(CBStyle::SURFACE))
                             .arg(EMStyle::rgb(CBStyle::BORDER)));

    auto* root = new QVBoxLayout(panel);
    root->setContentsMargins(0, 0, 0, 0);
    root->setSpacing(0);

    // ── 1. Brand strip ───────────────────────────────────────────────────
    auto* brand = new QLabel(EMStyle::brandStripText("QuickSim"), panel);
    brand->setStyleSheet(EMStyle::brandStripQSS());
    brand->setAlignment(Qt::AlignLeft | Qt::AlignVCenter);
    brand->setTextFormat(Qt::RichText);
    root->addWidget(brand);

    // ── Scrollable column for the section stack ──────────────────────────
    auto* scroll = new QScrollArea(panel);
    scroll->setWidgetResizable(true);
    scroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    scroll->setStyleSheet(QString(
                              "QScrollArea{background:%1;border:none;}"
                              "QScrollBar:vertical{background:%1;width:10px;margin:0;}"
                              "QScrollBar::handle:vertical{background:%2;border-radius:4px;min-height:24px;}"
                              "QScrollBar::handle:vertical:hover{background:%3;}"
                              "QScrollBar::add-line:vertical,QScrollBar::sub-line:vertical{height:0;}"
                              )
                              .arg(EMStyle::rgb(CBStyle::SURFACE))
                              .arg(EMStyle::rgb(CBStyle::BORDER))
                              .arg(EMStyle::rgb(CBStyle::TEXT_MUTED)));

    auto* scrollContent = new QWidget;
    scrollContent->setStyleSheet(QString("background:%1;")
                                     .arg(EMStyle::rgb(CBStyle::SURFACE)));
    auto* scrollLayout = new QVBoxLayout(scrollContent);
    scrollLayout->setContentsMargins(10, 8, 10, 8);
    scrollLayout->setSpacing(10);

    // ── Helper: section header label ─────────────────────────────────────
    auto makeSectionHeader = [&scrollLayout, panel](const QString& text) {
        auto* lbl = new QLabel(text, panel);
        lbl->setStyleSheet(EMStyle::sectionHeaderQSS());
        scrollLayout->addWidget(lbl);
    };

    // ── Helper: spin-row builder (label + spin in a single row) ──────────
    //
    // Returns the QDoubleSpinBox so the caller can store the pointer.
    auto makeSpinRow = [&scrollLayout, panel](
                           const QString& label,
                           double v, double mn, double mx,
                           double step, int decimals,
                           const QString& suffix,
                           QColor focusAccent) -> QDoubleSpinBox*
    {
        auto* row = new QWidget(panel);
        row->setStyleSheet("background:transparent;");
        auto* lay = new QHBoxLayout(row);
        lay->setContentsMargins(0, 0, 0, 0);
        lay->setSpacing(8);

        auto* lbl = new QLabel(label, row);
        lbl->setStyleSheet(EMStyle::lblSS());
        lbl->setMinimumWidth(80);
        lay->addWidget(lbl);

        auto* spin = new QDoubleSpinBox(row);
        spin->setLocale(QLocale::c());
        spin->setDecimals(decimals);   // [BUGFIX-DECIMALS-ROUNDING] — set FIRST
        spin->setRange(mn, mx);
        spin->setValue(v);
        spin->setSingleStep(step);
        if (!suffix.isEmpty()) spin->setSuffix(suffix);
        spin->setStyleSheet(EMStyle::spinSS(focusAccent));
        lay->addWidget(spin);

        scrollLayout->addWidget(row);
        return spin;
    };

    auto makeIntSpinRow = [&scrollLayout, panel](
                              const QString& label,
                              int v, int mn, int mx, int step,
                              QColor focusAccent) -> QSpinBox*
    {
        auto* row = new QWidget(panel);
        row->setStyleSheet("background:transparent;");
        auto* lay = new QHBoxLayout(row);
        lay->setContentsMargins(0, 0, 0, 0);
        lay->setSpacing(8);

        auto* lbl = new QLabel(label, row);
        lbl->setStyleSheet(EMStyle::lblSS());
        lbl->setMinimumWidth(80);
        lay->addWidget(lbl);

        auto* spin = new QSpinBox(row);
        spin->setLocale(QLocale::c());
        spin->setRange(mn, mx);
        spin->setValue(v);
        spin->setSingleStep(step);
        spin->setStyleSheet(EMStyle::spinSS(focusAccent));
        lay->addWidget(spin);

        scrollLayout->addWidget(row);
        return spin;
    };

    // ── Helper: combo-row builder ────────────────────────────────────────
    auto makeComboRow = [&scrollLayout, panel](
                            const QString& label,
                            const QStringList& items,
                            QColor accent) -> QComboBox*
    {
        auto* row = new QWidget(panel);
        row->setStyleSheet("background:transparent;");
        auto* lay = new QHBoxLayout(row);
        lay->setContentsMargins(0, 0, 0, 0);
        lay->setSpacing(8);

        auto* lbl = new QLabel(label, row);
        lbl->setStyleSheet(EMStyle::lblSS());
        lbl->setMinimumWidth(80);
        lay->addWidget(lbl);

        auto* combo = new QComboBox(row);
        combo->addItems(items);
        combo->setStyleSheet(EMStyle::comboSS(accent));
        lay->addWidget(combo, /*stretch*/ 1);

        scrollLayout->addWidget(row);
        return combo;
    };

    // ── 2. PRESETS section ───────────────────────────────────────────────
    makeSectionHeader(QStringLiteral("PRESETS"));

    m_cboPreset = makeComboRow(
        QStringLiteral("Config:"),
        {"1-Section (baseline)",
         "2-Section identical",
         "3-Section identical",
         "2-Section different",
         "5-Section cascade",
         "Custom (edit below)"},
        EMStyle::accentFor(EMStyle::AccentRole::Primary));
    m_cboPreset->setItemData(0, 1);
    m_cboPreset->setItemData(1, 2);
    m_cboPreset->setItemData(2, 3);
    m_cboPreset->setItemData(3, 4);
    m_cboPreset->setItemData(4, 5);
    m_cboPreset->setItemData(5, 0);
    connect(m_cboPreset, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &MainWindow::onPresetChanged);

    // ── 3. ENCLOSURE section ─────────────────────────────────────────────
    makeSectionHeader(QStringLiteral("ENCLOSURE"));

    const QColor accentEnc = EMStyle::accentFor(EMStyle::AccentRole::Primary);
    m_spinA = makeSpinRow(QStringLiteral("a [mm]:"),     300.0,  10.0, 2000.0, 1.0, 1,
                          QStringLiteral(" mm"), accentEnc);
    m_spinB = makeSpinRow(QStringLiteral("b [mm]:"),     120.0,   1.0, 2000.0, 1.0, 1,
                          QStringLiteral(" mm"), accentEnc);
    m_spinT = makeSpinRow(QStringLiteral("t [mm]:"),       1.5,   0.1,   50.0, 0.1, 2,
                          QStringLiteral(" mm"), accentEnc);

    m_cboTopology = makeComboRow(
        QStringLiteral("Topology:"),
        {"Cascade  (Fig. 3.10)", "Star-branch (Fig. 3.11)"},
        accentEnc);
    m_cboTopology->setToolTip(
        "CASCADE:     sections connected serially in depth.\n"
        "STAR_BRANCH: section 1 is the spine; sections 2..N branch\n"
        "             laterally from the spine output junction.");
    connect(m_cboTopology, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &MainWindow::onTopologyChanged);

    // ── 4. FREQUENCY section ─────────────────────────────────────────────
    makeSectionHeader(QStringLiteral("FREQUENCY SWEEP"));

    m_spinFstart = makeSpinRow(QStringLiteral("Start:"),    1.0,   0.1,  5000.0, 1.0, 1,
                               QStringLiteral(" MHz"), accentEnc);
    m_spinFstop  = makeSpinRow(QStringLiteral("Stop:"),  2000.0,  10.0, 40000.0, 100.0, 1,
                              QStringLiteral(" MHz"), accentEnc);
    m_spinPoints = makeIntSpinRow(QStringLiteral("Points:"), 200, 2, 5000, 10, accentEnc);

    // ── 5. SECTION PROPERTIES section (PropertyPanel goes here) ─────────
    makeSectionHeader(QStringLiteral("SECTION PROPERTIES"));

    m_propertyPanel = new PropertyPanel(scrollContent);
    scrollLayout->addWidget(m_propertyPanel);

    // ── 6. ACTIONS section ───────────────────────────────────────────────
    makeSectionHeader(QStringLiteral("ACTIONS"));

    m_btnAddSection = new QPushButton(QStringLiteral("Add Section"), scrollContent);
    m_btnAddSection->setStyleSheet(
        EMStyle::actionButtonQSS(EMStyle::accentFor(EMStyle::AccentRole::Neutral)));
    m_btnAddSection->setCursor(Qt::PointingHandCursor);
    m_btnAddSection->setShortcut(QKeySequence("Ctrl+N"));
    m_btnAddSection->setToolTip(QStringLiteral("Add a new section (Ctrl+N)"));
    connect(m_btnAddSection, &QPushButton::clicked, this, &MainWindow::onAddSection);
    scrollLayout->addWidget(m_btnAddSection);

    m_btnRemoveSection = new QPushButton(QStringLiteral("Remove Section"), scrollContent);
    m_btnRemoveSection->setStyleSheet(
        EMStyle::actionButtonQSS(EMStyle::accentFor(EMStyle::AccentRole::Neutral)));
    m_btnRemoveSection->setCursor(Qt::PointingHandCursor);
    m_btnRemoveSection->setShortcut(QKeySequence("Delete"));
    m_btnRemoveSection->setToolTip(QStringLiteral("Remove selected section (Delete)"));
    connect(m_btnRemoveSection, &QPushButton::clicked, this, &MainWindow::onRemoveSection);
    scrollLayout->addWidget(m_btnRemoveSection);

    scrollLayout->addStretch(1);

    scroll->setWidget(scrollContent);
    root->addWidget(scroll, /*stretch*/ 1);

    // ── 7. PRIMARY action row (Compute + Export, always pinned to bottom) ─
    auto* primary = new QWidget(panel);
    primary->setStyleSheet(QString(
                               "background:%1;border-top:1px solid %2;"
                               )
                               .arg(EMStyle::rgb(CBStyle::SURFACE))
                               .arg(EMStyle::rgb(CBStyle::BORDER_LT)));

    auto* primaryLayout = new QVBoxLayout(primary);
    primaryLayout->setContentsMargins(10, 8, 10, 12);
    primaryLayout->setSpacing(6);

    // [T2.1a] Validity indicator row — same pattern as Window 2 (Task 1.5b).
    m_validityRow = new QWidget(primary);
    m_validityRow->setObjectName("ValidityRow");
    m_validityRow->setStyleSheet(
        "QWidget#ValidityRow{background:transparent;}");
    auto* validityLayout = new QHBoxLayout(m_validityRow);
    validityLayout->setContentsMargins(2, 2, 2, 2);
    validityLayout->setSpacing(8);

    m_validityDot = new QFrame(m_validityRow);
    m_validityDot->setFrameShape(QFrame::NoFrame);
    validityLayout->addWidget(m_validityDot);

    m_validityLabel = new QLabel(QStringLiteral("Checking..."), m_validityRow);
    validityLayout->addWidget(m_validityLabel, /*stretch*/ 1);

    primaryLayout->addWidget(m_validityRow);

    m_btnCompute = new QPushButton(QStringLiteral("COMPUTE"), primary);
    m_btnCompute->setMinimumHeight(38);
    m_btnCompute->setStyleSheet(
        EMStyle::primaryButtonQSS(EMStyle::accentFor(EMStyle::AccentRole::Primary)));
    m_btnCompute->setCursor(Qt::PointingHandCursor);
    m_btnCompute->setShortcut(QKeySequence("Ctrl+R"));
    m_btnCompute->setToolTip(QStringLiteral("Run shielding analysis (Ctrl+R)"));
    connect(m_btnCompute, &QPushButton::clicked, this, &MainWindow::onComputeClicked);
    primaryLayout->addWidget(m_btnCompute);

    m_btnExport = new QPushButton(QStringLiteral("Export CSV"), primary);
    m_btnExport->setStyleSheet(
        EMStyle::actionButtonQSS(EMStyle::accentFor(EMStyle::AccentRole::Neutral)));
    m_btnExport->setCursor(Qt::PointingHandCursor);
    m_btnExport->setShortcut(QKeySequence("Ctrl+S"));
    m_btnExport->setToolTip(QStringLiteral("Export results to CSV (Ctrl+S)"));
    connect(m_btnExport, &QPushButton::clicked, this, &MainWindow::onExportCSV);
    primaryLayout->addWidget(m_btnExport);

    root->addWidget(primary);

    // ── Live validity re-evaluation: any parameter change triggers refresh
    //    via the central spin/combo signals. PropertyPanel and CircuitCanvas
    //    are wired in setupUI() to also trigger refresh through their slots.
    auto refresh = [this]{ refreshValidityIndicator(); };
    connect(m_spinA,      QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, refresh);
    connect(m_spinB,      QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, refresh);
    connect(m_spinT,      QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, refresh);
    connect(m_spinFstart, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, refresh);
    connect(m_spinFstop,  QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, refresh);
    connect(m_spinPoints, QOverload<int>::of   (&QSpinBox::valueChanged),       this, refresh);

    return panel;
}

void MainWindow::setupPlot()
{
    m_plot = new QCustomPlot;

    // [T2.1a] Plot styling aligned with Window 2 (Circuit Builder).
    m_plot->setBackground(QBrush(CBStyle::BG));

    // Title
    m_plot->plotLayout()->insertRow(0);
    auto* title = new QCPTextElement(
        m_plot,
        QStringLiteral("Quick Simulation — Shielding Effectiveness"),
        QFont("Segoe UI", 11, QFont::Bold));
    title->setTextColor(CBStyle::TEXT);
    m_plot->plotLayout()->addElement(0, 0, title);

    // Axes
    m_plot->xAxis->setLabel(QStringLiteral("Frequency [GHz]"));
    m_plot->yAxis->setLabel(QStringLiteral("SE [dB]"));
    m_plot->xAxis->setLabelFont(QFont("Segoe UI", 9));
    m_plot->yAxis->setLabelFont(QFont("Segoe UI", 9));
    m_plot->xAxis->setTickLabelFont(QFont("Segoe UI", 8));
    m_plot->yAxis->setTickLabelFont(QFont("Segoe UI", 8));
    m_plot->xAxis->setLabelColor(CBStyle::TEXT);
    m_plot->yAxis->setLabelColor(CBStyle::TEXT);
    m_plot->xAxis->setTickLabelColor(CBStyle::TEXT_MUTED);
    m_plot->yAxis->setTickLabelColor(CBStyle::TEXT_MUTED);
    m_plot->xAxis->setBasePen(QPen(CBStyle::BORDER));
    m_plot->yAxis->setBasePen(QPen(CBStyle::BORDER));
    m_plot->xAxis->setTickPen(QPen(CBStyle::BORDER));
    m_plot->yAxis->setTickPen(QPen(CBStyle::BORDER));
    m_plot->xAxis->setSubTickPen(QPen(CBStyle::BORDER_LT));
    m_plot->yAxis->setSubTickPen(QPen(CBStyle::BORDER_LT));
    m_plot->xAxis->setRange(0.0, 2.0);
    m_plot->yAxis->setRange(-40.0, 100.0);

    // Grid
    m_plot->xAxis->grid()->setSubGridVisible(true);
    m_plot->yAxis->grid()->setSubGridVisible(true);
    QPen gridPen(CBStyle::BORDER_LT);
    gridPen.setStyle(Qt::DashLine);
    m_plot->xAxis->grid()->setPen(gridPen);
    m_plot->yAxis->grid()->setPen(gridPen);
    QPen subGridPen(CBStyle::BORDER_LT);
    subGridPen.setStyle(Qt::DotLine);
    m_plot->xAxis->grid()->setSubGridPen(subGridPen);
    m_plot->yAxis->grid()->setSubGridPen(subGridPen);

    // Legend
    m_plot->legend->setVisible(true);
    m_plot->legend->setFont(QFont("Segoe UI", 9));
    m_plot->legend->setBrush(QBrush(QColor(255, 255, 255, 220)));
    m_plot->legend->setBorderPen(QPen(CBStyle::BORDER));
    m_plot->axisRect()->insetLayout()->setInsetAlignment(
        0, Qt::AlignTop | Qt::AlignRight);

    // Interactions
    m_plot->setInteractions(QCP::iRangeDrag    | QCP::iRangeZoom |
                            QCP::iSelectPlottables | QCP::iSelectLegend);

    connect(m_plot, &QCustomPlot::mousePress,
            this,   &MainWindow::onPlotClicked);
}

void MainWindow::setupStatusBar()
{
    m_lblStatus = new QLabel(QStringLiteral("Ready"));
    m_lblStatus->setStyleSheet(QString("color:%1;background:transparent;")
                                   .arg(EMStyle::rgb(CBStyle::TEXT_MUTED)));
    statusBar()->addWidget(m_lblStatus, 1);
}


// ============================================================================
//  STATUS BAR HELPER
// ============================================================================

void MainWindow::setStatus(const QString& msg, const QColor& col)
{
    m_lblStatus->setText(msg);
    m_lblStatus->setStyleSheet(QString("color:rgb(%1,%2,%3);background:transparent;"
                                       "font-family:'Courier New';font-size:10px;")
                                   .arg(col.red()).arg(col.green()).arg(col.blue()));
}


// ============================================================================
//  [T2.1a] VALIDITY INDICATOR
// ============================================================================
//
// We evaluate two layers of correctness:
//   1. enclosure-level: are global a/b/t valid? is the frequency span sensible?
//   2. config-level:    EnclosureConfig::isValid (combines enclosure + sections)
//
// EnclosureConfig::isValid (declared in CircuitGenerator.h) does both — it
// already checks a, b, t, source impedance, section count, and per-section
// fields. So a single call answers everything.

namespace {
EnclosureConfig snapshotConfigFromUI(const QDoubleSpinBox* spinA,
                                     const QDoubleSpinBox* spinB,
                                     const QDoubleSpinBox* spinT,
                                     const QComboBox*      cboTopology,
                                     CircuitCanvas*        canvas)
{
    EnclosureConfig cfg;
    cfg.a = spinA->value() / 1000.0;
    cfg.b = spinB->value() / 1000.0;
    cfg.t = spinT->value() / 1000.0;
    cfg.topology = (cboTopology->currentIndex() == 1)
                       ? TopologyType::STAR_BRANCH
                       : TopologyType::CASCADE;

    const QVector<SectionItemData> sd = canvas->allSectionData();
    cfg.sections.reserve(sd.size());
    for (const auto& s : sd) {
        SectionConfig sec;
        sec.depth           = s.depth_mm        / 1000.0;
        sec.obs_position    = s.obs_position_mm / 1000.0;
        sec.has_observation = s.has_observation;
        sec.section_width_a = (s.section_width_a_mm > 0.0)
                                  ? s.section_width_a_mm / 1000.0
                                  : -1.0;
        sec.aperture_l      = s.aperture_l_mm   / 1000.0;
        sec.aperture_w      = s.aperture_w_mm   / 1000.0;
        sec.has_cover       = s.has_cover;
        sec.cover_gap       = s.cover_gap_mm    / 1000.0;
        sec.cover_eps_r     = s.cover_eps_r;
        sec.has_dielectric  = s.has_dielectric;
        sec.dielectric_h    = s.dielectric_h_mm / 1000.0;
        sec.dielectric_er   = s.dielectric_er;
        cfg.sections.push_back(sec);
    }
    return cfg;
}
} // namespace


void MainWindow::setValidityState(bool ok, const QString& brief, const QString& full)
{
    const QColor dotColor = ok ? CBStyle::GREEN : CBStyle::RED;
    m_validityDot->setStyleSheet(QString(
                                     "QFrame{"
                                     "background:%1;border-radius:6px;"
                                     "min-width:12px;max-width:12px;"
                                     "min-height:12px;max-height:12px;}"
                                     ).arg(EMStyle::rgb(dotColor)));

    m_validityLabel->setText(brief);
    m_validityLabel->setStyleSheet(QString(
                                       "QLabel{"
                                       "color:%1;background:transparent;"
                                       "font-family:'Courier New';font-size:10px;font-weight:bold;"
                                       "letter-spacing:1px;}"
                                       ).arg(EMStyle::rgb(dotColor)));

    m_validityRow->setToolTip(full);
}


void MainWindow::refreshValidityIndicator()
{
    if (!m_validityDot || !m_validityLabel) return;

    // Frequency span check happens before EnclosureConfig::isValid because
    // isValid doesn't know about the frequency sweep. f_stop must exceed
    // f_start, and the user-set number of points must be at least 2 (the
    // sweep linspace formula needs n>=2 for both endpoints to be included).
    const double fstart = m_spinFstart->value();
    const double fstop  = m_spinFstop->value();
    const int    npts   = m_spinPoints->value();
    if (fstop <= fstart) {
        setValidityState(false,
                         QStringLiteral("Bad frequency span"),
                         QStringLiteral("Frequency stop must be greater than frequency start.\n\n"
                                        "Increase the Stop value or decrease the Start value."));
        return;
    }
    if (npts < 2) {
        setValidityState(false,
                         QStringLiteral("Too few points"),
                         QStringLiteral("Frequency sweep needs at least 2 points so both endpoints "
                                        "are included in the linspace."));
        return;
    }

    // Config-level validation via the engine. This catches:
    //   - a, b, t out of range
    //   - no sections at all
    //   - STAR_BRANCH with fewer than 2 sections
    //   - per-section issues (obs_position outside (0, depth), aperture
    //     dimensions, cover/dielectric parameters)
    const EnclosureConfig cfg = snapshotConfigFromUI(
        m_spinA, m_spinB, m_spinT, m_cboTopology, m_canvas);

    std::string err;
    if (!cfg.isValid(err)) {
        const QString errQ = QString::fromStdString(err);

        // Extract a short, leading phrase for the indicator row.
        QString brief = errQ;
        const int colon = brief.indexOf(':');
        if (colon > 0 && colon < 30) brief.truncate(colon);
        if (brief.length() > 32) {
            brief = brief.left(29) + QStringLiteral("...");
        }

        const QString full = QStringLiteral(
                                 "Configuration is not valid for analysis:\n\n%1\n\n"
                                 "Fix the highlighted condition, then COMPUTE will be safe to run.")
                                 .arg(errQ);
        setValidityState(false, brief, full);
        return;
    }

    setValidityState(true,
                     QStringLiteral("Circuit valid"),
                     QStringLiteral("Configuration is valid. Click COMPUTE to run the sweep."));
}


// ============================================================================
//  PRESET HANDLING
// ============================================================================

void MainWindow::onPresetChanged(int index)
{
    const int presetId = m_cboPreset->itemData(index).toInt();
    if (presetId > 0) loadPreset(presetId);
}

void MainWindow::loadPreset(int presetId)
{
    QVector<SectionItemData> sections;
    SectionItemData sec;

    switch (presetId) {
    case 1:
        sec.depth_mm = 300; sec.obs_position_mm = 150;
        sec.aperture_l_mm = 80; sec.aperture_w_mm = 80;
        sections = { sec };
        break;
    case 2:
        sec.depth_mm = 300; sec.obs_position_mm = 150;
        sec.aperture_l_mm = 80; sec.aperture_w_mm = 80;
        sections = { sec, sec };
        break;
    case 3:
        sec.depth_mm = 300; sec.obs_position_mm = 150;
        sec.aperture_l_mm = 80; sec.aperture_w_mm = 80;
        sections = { sec, sec, sec };
        break;
    case 4: {
        SectionItemData s1;
        s1.depth_mm = 300; s1.obs_position_mm = 150;
        s1.aperture_l_mm = 80; s1.aperture_w_mm = 80;
        SectionItemData s2;
        s2.depth_mm = 200; s2.obs_position_mm = 100;
        s2.aperture_l_mm = 50; s2.aperture_w_mm = 50;
        sections = { s1, s2 };
        break;
    }
    case 5:
        sec.depth_mm = 200; sec.obs_position_mm = 100;
        sec.aperture_l_mm = 60; sec.aperture_w_mm = 60;
        sections.fill(sec, 5);
        break;
    default:
        return;
    }

    m_canvas->loadPreset(sections);
    refreshValidityIndicator();
}


// ============================================================================
//  TOPOLOGY CHANGE
// ============================================================================

void MainWindow::onTopologyChanged(int index)
{
    // Forward to the canvas so it can recolour / relabel nodes per Fig 3.10
    // vs Fig 3.11. CircuitCanvas::setTopology re-runs layoutSections().
    const TopologyType t = (index == 1)
                               ? TopologyType::STAR_BRANCH
                               : TopologyType::CASCADE;
    m_canvas->setTopology(t);
    refreshValidityIndicator();
}


// ============================================================================
//  SECTION ADD / REMOVE
// ============================================================================

void MainWindow::onAddSection()
{
    SectionItemData newSec;
    newSec.depth_mm        = 300;
    newSec.obs_position_mm = 150;
    newSec.aperture_l_mm   = 80;
    newSec.aperture_w_mm   = 80;

    m_canvas->addSection(newSec);
    m_canvas->selectSection(m_canvas->sectionCount() - 1);

    m_cboPreset->blockSignals(true);
    m_cboPreset->setCurrentIndex(m_cboPreset->count() - 1);
    m_cboPreset->blockSignals(false);

    setStatus(QString("Added section %1 — click Compute to update")
                  .arg(m_canvas->sectionCount()),
              CBStyle::TEXT_MUTED);
}

void MainWindow::onRemoveSection()
{
    if (m_canvas->sectionCount() <= 1) {
        setStatus(QStringLiteral("Cannot remove the last section"), CBStyle::RED);
        return;
    }
    m_canvas->removeSelectedSection();

    m_cboPreset->blockSignals(true);
    m_cboPreset->setCurrentIndex(m_cboPreset->count() - 1);
    m_cboPreset->blockSignals(false);

    setStatus(QString("%1 sections remaining — click Compute to update")
                  .arg(m_canvas->sectionCount()),
              CBStyle::TEXT_MUTED);
}


// ============================================================================
//  CANVAS / PROPERTY PANEL EVENT HANDLERS
// ============================================================================

void MainWindow::onCanvasSelectionChanged(int sectionIndex)
{
    if (sectionIndex >= 0) {
        SectionItem* item = m_canvas->sectionAt(sectionIndex);
        if (item) m_propertyPanel->loadSection(sectionIndex, item->data());
    } else {
        m_propertyPanel->clearSelection();
    }
}

void MainWindow::onPropertyChanged(int sectionIndex, const SectionItemData& data)
{
    SectionItem* item = m_canvas->sectionAt(sectionIndex);
    if (item) {
        item->setData(data);
        m_cboPreset->blockSignals(true);
        m_cboPreset->setCurrentIndex(m_cboPreset->count() - 1);
        m_cboPreset->blockSignals(false);
    }
    refreshValidityIndicator();
}

void MainWindow::onSectionCountChanged(int /*count*/)
{
    refreshValidityIndicator();
}


// ============================================================================
//  EXPORT
// ============================================================================

void MainWindow::onExportCSV()
{
    if (m_freqs.isEmpty()) {
        MessageDialog::error(this,
                             QStringLiteral("No data to export"),
                             QStringLiteral(
                                 "There is no computed SE data to export.\n\n"
                                 "Click COMPUTE first to populate the plot, then "
                                 "use Export CSV to save the results."));
        return;
    }

    const QString filename = QFileDialog::getSaveFileName(
        this, QStringLiteral("Export CSV"),
        QStringLiteral("SE_results.csv"),
        QStringLiteral("CSV Files (*.csv)"));
    if (filename.isEmpty()) return;

    std::ofstream file(filename.toStdString());
    if (!file.is_open()) {
        MessageDialog::error(this,
                             QStringLiteral("Cannot write file"),
                             QString("The selected file could not be opened for writing:\n\n"
                                     "%1\n\n"
                                     "Check that the destination folder exists and that "
                                     "the file is not currently open in another program.").arg(filename));
        return;
    }

    // Header row
    file << "f_GHz";
    for (const auto& lbl : m_labels)
        file << ",SE_" << lbl.toStdString() << "_dB";
    file << "\n";

    // Data rows
    file << std::fixed;
    for (int i = 0; i < m_freqs.size(); ++i) {
        file << std::setprecision(9) << m_freqs[i];
        for (int c = 0; c < m_SE_data.size(); ++c)
            file << "," << std::setprecision(6) << m_SE_data[c][i];
        file << "\n";
    }
    file.close();

    setStatus(QStringLiteral("Exported: ") + filename, CBStyle::GREEN);

    MessageDialog::success(this,
                           QStringLiteral("Export complete"),
                           QString("CSV saved successfully to:\n\n%1").arg(filename));
}


// ============================================================================
//  ANALYSIS ENGINE
// ============================================================================

void MainWindow::onComputeClicked() { runAnalysis(); }

void MainWindow::clearPlot()
{
    m_plot->clearPlottables();
    m_plot->clearItems();
    m_crosshairLine = nullptr;
    m_readoutLabel  = nullptr;
    m_tracers.clear();
    m_plot->replot();
}

void MainWindow::runAnalysis()
{
    QElapsedTimer timer;
    timer.start();

    // 1. Read enclosure parameters from UI
    const double a       = m_spinA->value()      / 1000.0;
    const double b       = m_spinB->value()      / 1000.0;
    const double t       = m_spinT->value()      / 1000.0;
    const double f_start = m_spinFstart->value() * 1e6;
    const double f_stop  = m_spinFstop->value()  * 1e6;
    const int    n_pts   = m_spinPoints->value();

    // 2. Build EnclosureConfig from canvas sections
    EnclosureConfig cfg;
    cfg.a = a;
    cfg.b = b;
    cfg.t = t;
    cfg.topology = (m_cboTopology->currentIndex() == 1)
                       ? TopologyType::STAR_BRANCH
                       : TopologyType::CASCADE;

    const QVector<SectionItemData> sectionData = m_canvas->allSectionData();
    if (sectionData.isEmpty()) {
        MessageDialog::error(this,
                             QStringLiteral("No sections"),
                             QStringLiteral("The canvas has no sections.\n\n"
                                            "Use Add Section or load a preset, then click COMPUTE."));
        setStatus(QStringLiteral("No sections defined"), CBStyle::RED);
        return;
    }

    for (const auto& sd : sectionData) {
        SectionConfig sec;
        sec.depth           = sd.depth_mm          / 1000.0;
        sec.obs_position    = sd.obs_position_mm   / 1000.0;
        sec.has_observation = sd.has_observation;
        sec.section_width_a = (sd.section_width_a_mm > 0.0)
                                  ? sd.section_width_a_mm / 1000.0
                                  : -1.0;
        sec.aperture_l      = sd.aperture_l_mm     / 1000.0;
        sec.aperture_w      = sd.aperture_w_mm     / 1000.0;
        sec.has_cover       = sd.has_cover;
        sec.cover_gap       = sd.cover_gap_mm      / 1000.0;
        sec.cover_eps_r     = sd.cover_eps_r;
        sec.has_dielectric  = sd.has_dielectric;
        sec.dielectric_h    = sd.dielectric_h_mm   / 1000.0;
        sec.dielectric_er   = sd.dielectric_er;
        cfg.sections.push_back(sec);
    }

    // 3. Validate configuration
    std::string error_msg;
    if (!cfg.isValid(error_msg)) {
        MessageDialog::error(this,
                             QStringLiteral("Invalid configuration"),
                             QString::fromStdString(error_msg));
        setStatus(QStringLiteral("Cannot compute — invalid configuration"), CBStyle::RED);
        return;
    }

    setStatus(QStringLiteral("Computing..."), CBStyle::TEXT_MUTED);
    QApplication::processEvents();

    // 4. Generate circuit
    MNASolver solver;
    std::vector<ObservationPoint> obs_points;
    try {
        obs_points = CircuitGenerator::generate(cfg, solver, false);
    } catch (const std::exception& e) {
        MessageDialog::error(this,
                             QStringLiteral("Circuit generation failed"),
                             QString::fromStdString(e.what()));
        setStatus(QStringLiteral("Circuit generation failed"), CBStyle::RED);
        return;
    }

    if (obs_points.empty()) {
        MessageDialog::error(this,
                             QStringLiteral("No observation points"),
                             QStringLiteral(
                                 "No sections have 'Has observation point' enabled.\n\n"
                                 "Enable at least one section's observation in the SECTION "
                                 "PROPERTIES panel, then COMPUTE again."));
        setStatus(QStringLiteral("No observation points"), CBStyle::RED);
        return;
    }

    // 5. Frequency sweep — inclusive linspace
    const double fstep = (n_pts > 1)
                             ? (f_stop - f_start) / static_cast<double>(n_pts - 1)
                             : 0.0;

    QVector<double>          freqs_GHz;
    QVector<QVector<double>> SE_curves(static_cast<int>(obs_points.size()));
    QVector<QString>         labels;

    freqs_GHz.reserve(n_pts);
    for (auto& c : SE_curves) c.reserve(n_pts);
    for (const auto& op : obs_points)
        labels.append(QString::fromStdString(op.label));

    for (int i = 0; i < n_pts; ++i) {
        const double f = f_start + i * fstep;
        freqs_GHz.append(f / 1e9);

        try {
            const auto SE_vals =
                CircuitGenerator::computeSE(solver, obs_points, cfg, f);
            for (std::size_t oi = 0; oi < obs_points.size(); ++oi)
                SE_curves[static_cast<int>(oi)].append(SE_vals[oi]);
        } catch (const std::exception& e) {
            std::cerr << "[MainWindow] sweep error at f="
                      << f / 1e9 << " GHz: " << e.what() << "\n";
            for (auto& c : SE_curves)
                c.append(std::numeric_limits<double>::quiet_NaN());
        }
    }

    const qint64 elapsed = timer.elapsed();

    // 6. Store results
    m_freqs   = freqs_GHz;
    m_SE_data = SE_curves;
    m_labels  = labels;

    // 7. Plot
    plotResults(freqs_GHz, SE_curves, labels);

    // 8. Status bar
    setStatus(
        QString("%1-section %2 | %3 branches, %4 nodes, %5 obs pts | "
                "%6 points in %7 ms")
            .arg(static_cast<int>(cfg.sections.size()))
            .arg(cfg.topology == TopologyType::STAR_BRANCH ? "STAR_BRANCH" : "CASCADE")
            .arg(solver.getNumBranches())
            .arg(solver.getNumNodes())
            .arg(static_cast<int>(obs_points.size()))
            .arg(n_pts)
            .arg(elapsed),
        CBStyle::GREEN);
}


// ============================================================================
//  PLOTTING
// ============================================================================

void MainWindow::plotResults(const QVector<double>&          freqs_GHz,
                             const QVector<QVector<double>>& SE_curves,
                             const QVector<QString>&          labels)
{
    m_plot->clearPlottables();
    m_plot->clearItems();
    m_crosshairLine = nullptr;
    m_readoutLabel  = nullptr;
    m_tracers.clear();

    // Curve colours — same palette as Window 2 plus a few extras for
    // multi-observation circuits.
    static const QVector<QColor> colors = {
        CBStyle::ACCENT,                // #0E64C8 blue
        CBStyle::RED,                   // #C31E1E
        CBStyle::GREEN,                 // #168240
        CBStyle::ORANGE,                // #B45A00
        QColor(147,  51, 234),
        QColor( 14, 165, 233),
        QColor(244,  63,  94),
        QColor( 34, 197,  94),
    };

    double ymin =  std::numeric_limits<double>::infinity();
    double ymax = -std::numeric_limits<double>::infinity();

    for (int c = 0; c < SE_curves.size(); ++c) {
        QCPGraph* graph = m_plot->addGraph();
        graph->setData(freqs_GHz, SE_curves[c]);
        graph->setName(labels.value(c, QString("P%1").arg(c + 1)));
        QPen pen(colors[c % colors.size()]);
        pen.setWidth(2);
        graph->setPen(pen);
        graph->setSelectable(QCP::stWhole);

        for (double v : SE_curves[c]) {
            if (std::isfinite(v)) {
                ymin = std::min(ymin, v);
                ymax = std::max(ymax, v);
            }
        }
    }

    if (!freqs_GHz.isEmpty())
        m_plot->xAxis->setRange(freqs_GHz.first(), freqs_GHz.last());
    if (std::isfinite(ymin) && std::isfinite(ymax)) {
        const double yrange = std::max(ymax - ymin, 1.0);
        m_plot->yAxis->setRange(ymin - 0.1 * yrange, ymax + 0.1 * yrange);
    }

    // 0 dB reference line
    auto* zeroLine = new QCPItemStraightLine(m_plot);
    zeroLine->point1->setCoords(0.0, 0.0);
    zeroLine->point2->setCoords(1.0, 0.0);
    QPen zeroPen(CBStyle::BORDER);
    zeroPen.setStyle(Qt::DashLine);
    zeroLine->setPen(zeroPen);

    setupPlotInteractiveItems();

    m_plot->replot();
}

void MainWindow::setupPlotInteractiveItems()
{
    // Vertical crosshair
    m_crosshairLine = new QCPItemLine(m_plot);
    QPen chPen(CBStyle::TEXT_MUTED);
    chPen.setStyle(Qt::DashLine);
    chPen.setWidth(1);
    m_crosshairLine->setPen(chPen);
    m_crosshairLine->start->setType(QCPItemPosition::ptPlotCoords);
    m_crosshairLine->end->setType(QCPItemPosition::ptPlotCoords);
    m_crosshairLine->setVisible(false);

    // SE readout label
    m_readoutLabel = new QCPItemText(m_plot);
    m_readoutLabel->setPositionAlignment(Qt::AlignLeft | Qt::AlignTop);
    m_readoutLabel->position->setType(QCPItemPosition::ptPlotCoords);
    m_readoutLabel->setFont(QFont("Courier New", 9));
    m_readoutLabel->setColor(CBStyle::TEXT);
    m_readoutLabel->setPadding(QMargins(6, 4, 6, 4));
    m_readoutLabel->setBrush(QBrush(QColor(255, 255, 240, 230)));
    QPen borderPen(CBStyle::BORDER);
    borderPen.setWidth(1);
    m_readoutLabel->setPen(borderPen);
    m_readoutLabel->setVisible(false);

    // Per-curve tracers
    m_tracers.clear();
    static const QVector<QColor> colors = {
        CBStyle::ACCENT, CBStyle::RED, CBStyle::GREEN, CBStyle::ORANGE,
        QColor(147,  51, 234), QColor( 14, 165, 233),
        QColor(244,  63,  94), QColor( 34, 197,  94),
    };
    for (int c = 0; c < m_plot->graphCount(); ++c) {
        auto* tracer = new QCPItemTracer(m_plot);
        tracer->setGraph(m_plot->graph(c));
        tracer->setStyle(QCPItemTracer::tsCircle);
        tracer->setSize(8);
        QPen tPen(colors[c % colors.size()]);
        tPen.setWidth(2);
        tracer->setPen(tPen);
        tracer->setBrush(QBrush(colors[c % colors.size()]));
        tracer->setVisible(false);
        tracer->setInterpolating(true);
        m_tracers.append(tracer);
    }
}


// ============================================================================
//  PLOT CLICK HANDLER (unchanged from prior version)
// ============================================================================

void MainWindow::onPlotClicked(QMouseEvent* event)
{
    if (m_freqs.isEmpty() || m_SE_data.isEmpty()) return;
    if (event->button() != Qt::LeftButton) return;

    const double clickedGHz = m_plot->xAxis->pixelToCoord(event->pos().x());

    int    idx     = 0;
    double minDist = std::abs(m_freqs[0] - clickedGHz);
    for (int i = 1; i < m_freqs.size(); ++i) {
        const double dist = std::abs(m_freqs[i] - clickedGHz);
        if (dist < minDist) { minDist = dist; idx = i; }
    }
    const double f_GHz = m_freqs[idx];

    if (m_crosshairLine) {
        const double yLo = m_plot->yAxis->range().lower - 1.0;
        const double yHi = m_plot->yAxis->range().upper + 1.0;
        m_crosshairLine->start->setCoords(f_GHz, yLo);
        m_crosshairLine->end->setCoords(f_GHz, yHi);
        m_crosshairLine->setVisible(true);
    }

    for (auto* tracer : m_tracers) {
        tracer->setGraphKey(f_GHz);
        tracer->setVisible(true);
    }

    QString text = QString("f = %1 GHz\n").arg(f_GHz, 0, 'f', 4);
    for (int c = 0; c < m_SE_data.size(); ++c) {
        const double se = m_SE_data[c].value(idx,
                                             std::numeric_limits<double>::quiet_NaN());
        const QString lbl = m_labels.value(c, QString("P%1").arg(c + 1));
        if (std::isfinite(se)) {
            text += QString("%1 : %2 dB\n").arg(lbl, -4).arg(se, 8, 'f', 2);
        } else {
            text += QString("%1 :   inf dB\n").arg(lbl, -4);
        }
    }
    text = text.trimmed();

    if (m_readoutLabel) {
        const double xRange = m_plot->xAxis->range().size();
        const double xOff   = m_plot->xAxis->range().lower;
        const double yTop   = m_plot->yAxis->range().upper;
        const double yRange = m_plot->yAxis->range().size();
        const bool nearRight = (clickedGHz - xOff) > 0.60 * xRange;
        m_readoutLabel->setPositionAlignment(
            (nearRight ? Qt::AlignRight : Qt::AlignLeft) | Qt::AlignTop);
        m_readoutLabel->position->setCoords(f_GHz, yTop - 0.04 * yRange);
        m_readoutLabel->setText(text);
        m_readoutLabel->setVisible(true);
    }

    m_plot->replot();
}
