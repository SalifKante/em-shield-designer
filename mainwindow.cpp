#include "mainwindow.h"
#include "qcustomplot.h"
#include "CircuitCanvas.h"
#include "PropertyPanel.h"
#include "SectionItem.h"

// Core engine
#include "include/core/CircuitGenerator.h"

#include <QApplication>
#include <QMouseEvent>
#include <QToolBar>
#include <QAction>
#include <QSplitter>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGroupBox>
#include <QLabel>
#include <QDoubleSpinBox>
#include <QSpinBox>
#include <QComboBox>
#include <QPushButton>
#include <QStatusBar>
#include <QFileDialog>
#include <QMessageBox>
#include <QElapsedTimer>
#include <QFrame>

#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>

using namespace EMCore;

// ============================================================================
// CONSTRUCTOR / DESTRUCTOR
// ============================================================================

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
{
    setWindowTitle("EMShieldDesigner — Shielding Effectiveness Analyser");
    resize(1400, 750);

    setupUI();
    setupToolbar();
    setupStatusBar();

    // Load default preset (3-section)
    m_cboPreset->setCurrentIndex(2);
    loadPreset(3);

    // Run initial analysis
    onComputeClicked();
}

MainWindow::~MainWindow() {}

// ============================================================================
// UI SETUP
// ============================================================================

void MainWindow::setupUI()
{
    m_splitter = new QSplitter(Qt::Horizontal, this);
    setCentralWidget(m_splitter);

    // =========================================================
    // LEFT PANEL: enclosure controls + property panel
    // =========================================================
    m_leftPanel = new QWidget;
    QVBoxLayout* leftLayout = new QVBoxLayout(m_leftPanel);
    leftLayout->setContentsMargins(6, 6, 6, 6);
    leftLayout->setSpacing(8);

    setupControlPanel();

    leftLayout->addWidget(m_grpPresets);
    leftLayout->addWidget(m_grpEnclosure);
    leftLayout->addWidget(m_grpFrequency);

    QFrame* sep = new QFrame;
    sep->setFrameShape(QFrame::HLine);
    sep->setFrameShadow(QFrame::Sunken);
    leftLayout->addWidget(sep);

    m_propertyPanel = new PropertyPanel;
    leftLayout->addWidget(m_propertyPanel);

    QPushButton* btnCompute = new QPushButton("Compute SE");
    btnCompute->setMinimumHeight(38);
    btnCompute->setStyleSheet(
        "QPushButton {"
        "  background-color: #2563eb; color: white;"
        "  font-weight: bold; font-size: 13px;"
        "  border-radius: 6px; border: none; }"
        "QPushButton:hover   { background-color: #1d4ed8; }"
        "QPushButton:pressed { background-color: #1e40af; }");
    connect(btnCompute, &QPushButton::clicked, this, &MainWindow::onComputeClicked);
    leftLayout->addWidget(btnCompute);

    m_leftPanel->setMaximumWidth(280);

    // =========================================================
    // CENTER: circuit canvas
    // =========================================================
    m_canvas = new CircuitCanvas;
    m_canvas->setMinimumHeight(120);

    // =========================================================
    // RIGHT: SE plot
    // =========================================================
    setupPlot();

    QSplitter* rightSplitter = new QSplitter(Qt::Vertical);
    rightSplitter->addWidget(m_canvas);
    rightSplitter->addWidget(m_plot);
    rightSplitter->setStretchFactor(0, 1);
    rightSplitter->setStretchFactor(1, 3);
    rightSplitter->setSizes({180, 520});

    m_splitter->addWidget(m_leftPanel);
    m_splitter->addWidget(rightSplitter);
    m_splitter->setStretchFactor(0, 0);
    m_splitter->setStretchFactor(1, 1);
    m_splitter->setSizes({270, 1130});

    // =========================================================
    // SIGNAL CONNECTIONS
    // =========================================================
    connect(m_canvas, &CircuitCanvas::selectionChanged,
            this,     &MainWindow::onCanvasSelectionChanged);

    connect(m_propertyPanel, &PropertyPanel::dataChanged,
            this,             &MainWindow::onPropertyChanged);
}

void MainWindow::setupControlPanel()
{
    // ---- Presets ----
    m_grpPresets = new QGroupBox("Preset");
    QVBoxLayout* presetLayout = new QVBoxLayout;

    m_cboPreset = new QComboBox;
    m_cboPreset->addItem("1-Section (baseline)",  1);
    m_cboPreset->addItem("2-Section identical",   2);
    m_cboPreset->addItem("3-Section identical",   3);
    m_cboPreset->addItem("2-Section different",   4);
    m_cboPreset->addItem("5-Section cascade",     5);
    m_cboPreset->addItem("Custom (edit below)",   0);
    connect(m_cboPreset, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &MainWindow::onPresetChanged);
    presetLayout->addWidget(m_cboPreset);
    m_grpPresets->setLayout(presetLayout);

    // ---- Enclosure ----
    m_grpEnclosure = new QGroupBox("Enclosure");
    QVBoxLayout* encLayout = new QVBoxLayout;

    // Lambda to add a labelled spin-box row
    auto addSpinRow = [](QVBoxLayout* layout, const QString& label,
                         QDoubleSpinBox*& spin, double value,
                         double mn, double mx, const QString& suffix) {
        QHBoxLayout* row = new QHBoxLayout;
        QLabel* lbl = new QLabel(label);
        lbl->setMinimumWidth(90);
        spin = new QDoubleSpinBox;
        spin->setRange(mn, mx);
        spin->setValue(value);
        spin->setSuffix(suffix);
        spin->setDecimals(1);
        row->addWidget(lbl);
        row->addWidget(spin);
        layout->addLayout(row);
    };

    addSpinRow(encLayout, "Width (a):",  m_spinA, 300.0, 10.0, 2000.0, " mm");
    addSpinRow(encLayout, "Height (b):", m_spinB, 120.0, 10.0, 2000.0, " mm");
    addSpinRow(encLayout, "Wall (t):",   m_spinT,   1.5,  0.1,   50.0, " mm");

    // ---- Topology selector ----
    {
        QHBoxLayout* topoRow = new QHBoxLayout;
        QLabel* lblTopo = new QLabel("Topology:");
        lblTopo->setMinimumWidth(90);
        m_cboTopology = new QComboBox;
        m_cboTopology->addItem("Cascade  (Fig. 3.10)", QVariant::fromValue(int(0)));
        m_cboTopology->addItem("Star-branch (Fig. 3.11)", QVariant::fromValue(int(1)));
        m_cboTopology->setToolTip(
            "CASCADE:     sections connected serially in depth.\n"
            "STAR_BRANCH: section 1 is the spine; sections 2…N branch\n"
            "             laterally from the spine output junction.");
        topoRow->addWidget(lblTopo);
        topoRow->addWidget(m_cboTopology);
        encLayout->addLayout(topoRow);
    }

    m_grpEnclosure->setLayout(encLayout);

    // ---- Frequency ----
    m_grpFrequency = new QGroupBox("Frequency Sweep");
    QVBoxLayout* freqLayout = new QVBoxLayout;

    addSpinRow(freqLayout, "Start:", m_spinFstart,    1.0,   0.1, 5000.0, " MHz");
    addSpinRow(freqLayout, "Stop:",  m_spinFstop,  2000.0,  10.0,32000.0, " MHz");

    {
        QHBoxLayout* ptsRow = new QHBoxLayout;
        QLabel* lblPts = new QLabel("Points:");
        lblPts->setMinimumWidth(90);
        m_spinPoints = new QSpinBox;
        m_spinPoints->setRange(2, 5000);  // Minimum 2 so endpoints are both included
        m_spinPoints->setValue(200);
        ptsRow->addWidget(lblPts);
        ptsRow->addWidget(m_spinPoints);
        freqLayout->addLayout(ptsRow);
    }

    m_grpFrequency->setLayout(freqLayout);
}

void MainWindow::setupPlot()
{
    m_plot = new QCustomPlot;

    // Title
    m_plot->plotLayout()->insertRow(0);
    auto* title = new QCPTextElement(
        m_plot, "Shielding Effectiveness vs Frequency",
        QFont("sans-serif", 12, QFont::Bold));
    m_plot->plotLayout()->addElement(0, 0, title);

    // Axes
    m_plot->xAxis->setLabel("Frequency [GHz]");
    m_plot->yAxis->setLabel("SE [dB]");
    m_plot->xAxis->setRange(0.0, 2.0);
    m_plot->yAxis->setRange(-40.0, 100.0);

    // Grid
    m_plot->xAxis->grid()->setSubGridVisible(true);
    m_plot->yAxis->grid()->setSubGridVisible(true);
    QPen gridPen(QColor(200, 200, 200));
    gridPen.setStyle(Qt::DashLine);
    m_plot->xAxis->grid()->setPen(gridPen);
    m_plot->yAxis->grid()->setPen(gridPen);
    QPen subGridPen(QColor(230, 230, 230));
    subGridPen.setStyle(Qt::DotLine);
    m_plot->xAxis->grid()->setSubGridPen(subGridPen);
    m_plot->yAxis->grid()->setSubGridPen(subGridPen);

    // Legend
    m_plot->legend->setVisible(true);
    m_plot->legend->setFont(QFont("sans-serif", 9));
    m_plot->legend->setBrush(QBrush(QColor(255, 255, 255, 200)));
    m_plot->axisRect()->insetLayout()->setInsetAlignment(
        0, Qt::AlignTop | Qt::AlignRight);

    // Interactions: drag, zoom, select — plus mouse-click readout
    m_plot->setInteractions(QCP::iRangeDrag    | QCP::iRangeZoom |
                            QCP::iSelectPlottables | QCP::iSelectLegend);

    // Connect click signal for interactive SE readout
    connect(m_plot, &QCustomPlot::mousePress,
            this,   &MainWindow::onPlotClicked);
}

void MainWindow::setupToolbar()
{
    m_toolbar = addToolBar("Main");
    m_toolbar->setMovable(false);
    m_toolbar->setIconSize(QSize(20, 20));

    m_actAddSection = m_toolbar->addAction("+ Section");
    m_actAddSection->setShortcut(QKeySequence("Ctrl+N"));
    m_actAddSection->setToolTip("Add a new section (Ctrl+N)");
    connect(m_actAddSection, &QAction::triggered, this, &MainWindow::onAddSection);

    m_actRemoveSection = m_toolbar->addAction("- Section");
    m_actRemoveSection->setShortcut(QKeySequence("Delete"));
    m_actRemoveSection->setToolTip("Remove selected section (Delete)");
    connect(m_actRemoveSection, &QAction::triggered, this, &MainWindow::onRemoveSection);

    m_toolbar->addSeparator();

    m_actCompute = m_toolbar->addAction("Compute");
    m_actCompute->setShortcut(QKeySequence("Ctrl+R"));
    m_actCompute->setToolTip("Run shielding analysis (Ctrl+R)");
    connect(m_actCompute, &QAction::triggered, this, &MainWindow::onComputeClicked);

    m_toolbar->addSeparator();

    m_actExport = m_toolbar->addAction("Export CSV");
    m_actExport->setShortcut(QKeySequence("Ctrl+S"));
    m_actExport->setToolTip("Export results to CSV (Ctrl+S)");
    connect(m_actExport, &QAction::triggered, this, &MainWindow::onExportCSV);
}

void MainWindow::setupStatusBar()
{
    m_lblStatus = new QLabel("Ready");
    statusBar()->addWidget(m_lblStatus, 1);
}

// ============================================================================
// PRESET HANDLING
// ============================================================================

void MainWindow::onPresetChanged(int index)
{
    int presetId = m_cboPreset->itemData(index).toInt();
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
}

// ============================================================================
// SECTION ADD / REMOVE
// ============================================================================

void MainWindow::onAddSection()
{
    SectionItemData newSec;
    newSec.depth_mm = 300;
    newSec.obs_position_mm = 150;
    newSec.aperture_l_mm = 80;
    newSec.aperture_w_mm = 80;

    m_canvas->addSection(newSec);
    m_canvas->selectSection(m_canvas->sectionCount() - 1);

    m_cboPreset->blockSignals(true);
    m_cboPreset->setCurrentIndex(m_cboPreset->count() - 1);
    m_cboPreset->blockSignals(false);

    m_lblStatus->setText(
        QString("Added section %1 — click Compute to update")
            .arg(m_canvas->sectionCount()));
}

void MainWindow::onRemoveSection()
{
    if (m_canvas->sectionCount() <= 1) {
        m_lblStatus->setText("Cannot remove the last section");
        return;
    }
    m_canvas->removeSelectedSection();

    m_cboPreset->blockSignals(true);
    m_cboPreset->setCurrentIndex(m_cboPreset->count() - 1);
    m_cboPreset->blockSignals(false);

    m_lblStatus->setText(
        QString("%1 sections remaining — click Compute to update")
            .arg(m_canvas->sectionCount()));
}

// ============================================================================
// CANVAS ↔ PROPERTY PANEL
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
}

// ============================================================================
// EXPORT
// ============================================================================

void MainWindow::onExportCSV()
{
    if (m_freqs.isEmpty()) {
        QMessageBox::warning(this, "No Data", "Run an analysis first.");
        return;
    }

    QString filename = QFileDialog::getSaveFileName(
        this, "Export CSV", "SE_results.csv", "CSV Files (*.csv)");
    if (filename.isEmpty()) return;

    std::ofstream file(filename.toStdString());
    if (!file.is_open()) {
        QMessageBox::critical(this, "Error", "Could not open file for writing.");
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

    m_lblStatus->setText("Exported: " + filename);
}

// ============================================================================
// ANALYSIS ENGINE
// ============================================================================

void MainWindow::onComputeClicked() { runAnalysis(); }

void MainWindow::runAnalysis()
{
    QElapsedTimer timer;
    timer.start();

    // -----------------------------------------------------------------------
    // 1. Read enclosure parameters from UI
    // -----------------------------------------------------------------------
    const double a       = m_spinA->value()      / 1000.0;   // mm → m
    const double b       = m_spinB->value()      / 1000.0;
    const double t       = m_spinT->value()      / 1000.0;
    const double f_start = m_spinFstart->value() * 1e6;      // MHz → Hz
    const double f_stop  = m_spinFstop->value()  * 1e6;
    const int    n_pts   = m_spinPoints->value();

    // -----------------------------------------------------------------------
    // 2. Build EnclosureConfig from canvas sections
    // -----------------------------------------------------------------------
    EnclosureConfig cfg;
    cfg.a = a;
    cfg.b = b;
    cfg.t = t;

    // Topology from selector
    cfg.topology = (m_cboTopology->currentIndex() == 1)
                       ? TopologyType::STAR_BRANCH
                       : TopologyType::CASCADE;

    const QVector<SectionItemData> sectionData = m_canvas->allSectionData();
    if (sectionData.isEmpty()) {
        m_lblStatus->setText("No sections defined");
        return;
    }

    for (const auto& sd : sectionData) {
        SectionConfig sec;
        sec.depth           = sd.depth_mm          / 1000.0;
        sec.obs_position    = sd.obs_position_mm   / 1000.0;
        sec.has_observation = sd.has_observation;
        sec.aperture_l      = sd.aperture_l_mm     / 1000.0;
        sec.aperture_w      = sd.aperture_w_mm     / 1000.0;
        sec.has_cover       = sd.has_cover;
        sec.cover_gap       = sd.cover_gap_mm      / 1000.0;
        sec.cover_eps_r     = sd.cover_eps_r;         // Eq. (3.24) dielectric gap
        sec.has_dielectric  = sd.has_dielectric;
        sec.dielectric_h    = sd.dielectric_h_mm   / 1000.0;
        sec.dielectric_er   = sd.dielectric_er;
        cfg.sections.push_back(sec);
    }

    // -----------------------------------------------------------------------
    // 3. Validate configuration
    // -----------------------------------------------------------------------
    std::string error_msg;
    if (!cfg.isValid(error_msg)) {
        QMessageBox::warning(this, "Invalid Configuration",
                             QString::fromStdString(error_msg));
        return;
    }

    m_lblStatus->setText("Computing…");
    QApplication::processEvents();

    // -----------------------------------------------------------------------
    // 4. Generate circuit
    // -----------------------------------------------------------------------
    MNASolver solver;
    std::vector<ObservationPoint> obs_points;
    try {
        obs_points = CircuitGenerator::generate(cfg, solver, false);
    } catch (const std::exception& e) {
        QMessageBox::critical(this, "Circuit Error",
                              "Circuit generation failed:\n" + QString::fromStdString(e.what()));
        return;
    }

    if (obs_points.empty()) {
        QMessageBox::warning(this, "No Observation Points",
                             "No sections have 'has_observation' enabled.");
        return;
    }

    // -----------------------------------------------------------------------
    // 5. Frequency sweep — inclusive linspace: f_start … f_stop (n_pts points)
    //
    //    Using (n_pts - 1) as the divisor ensures both f_start and f_stop
    //    are exactly included in the sweep.  With the old formula
    //    fstep = (f_stop - f_start) / n_pts, the last point fell
    //    one step short of f_stop.
    // -----------------------------------------------------------------------
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

        // Wrap each frequency point — catches near-resonance singularities
        // without aborting the entire sweep.
        try {
            // CORRECTED: cfg is passed as required by Eq. (3.8) so that
            // V₀ = cfg.V_source is used in SE = −20·log₁₀(|2·U_obs/V₀|).
            const auto SE_vals =
                CircuitGenerator::computeSE(solver, obs_points, cfg, f);

            for (std::size_t oi = 0; oi < obs_points.size(); ++oi)
                SE_curves[static_cast<int>(oi)].append(SE_vals[oi]);

        } catch (const std::exception& e) {
            // Frequency point failed (e.g. singular matrix at resonance).
            // Insert NaN so QCustomPlot produces a gap rather than a spike.
            std::cerr << "[MainWindow] sweep error at f="
                      << f / 1e9 << " GHz: " << e.what() << "\n";
            for (auto& c : SE_curves)
                c.append(std::numeric_limits<double>::quiet_NaN());
        }
    }

    const qint64 elapsed = timer.elapsed();

    // -----------------------------------------------------------------------
    // 6. Store results for export and interactive readout
    // -----------------------------------------------------------------------
    m_freqs   = freqs_GHz;
    m_SE_data = SE_curves;
    m_labels  = labels;

    // -----------------------------------------------------------------------
    // 7. Plot
    // -----------------------------------------------------------------------
    plotResults(freqs_GHz, SE_curves, labels);

    // -----------------------------------------------------------------------
    // 8. Status bar — use solver's actual node count (correct for both
    //    CASCADE and STAR_BRANCH topologies)
    // -----------------------------------------------------------------------
    m_lblStatus->setText(
        QString("%1-section %2 | %3 branches, %4 nodes, %5 obs pts | "
                "%6 points in %7 ms")
            .arg(static_cast<int>(cfg.sections.size()))
            .arg(cfg.topology == TopologyType::STAR_BRANCH ? "STAR_BRANCH" : "CASCADE")
            .arg(solver.getNumBranches())
            .arg(solver.getNumNodes())
            .arg(static_cast<int>(obs_points.size()))
            .arg(n_pts)
            .arg(elapsed));
}

// ============================================================================
// PLOTTING
// ============================================================================

void MainWindow::plotResults(const QVector<double>&          freqs_GHz,
                             const QVector<QVector<double>>& SE_curves,
                             const QVector<QString>&          labels)
{
    m_plot->clearPlottables();
    m_plot->clearItems();

    // Reset overlay pointers (clearItems() deleted them)
    m_crosshairLine = nullptr;
    m_readoutLabel  = nullptr;
    m_tracers.clear();

    static const QVector<QColor> colors = {
        { 37,  99, 235},   // Blue
        {220,  38,  38},   // Red
        { 22, 163,  74},   // Green
        {217, 119,   6},   // Amber
        {147,  51, 234},   // Purple
        { 14, 165, 233},   // Sky
        {244,  63,  94},   // Rose
        { 34, 197,  94},   // Emerald
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

        // NaN values produce automatic gaps in QCustomPlot
        for (double v : SE_curves[c]) {
            if (std::isfinite(v)) {
                ymin = std::min(ymin, v);
                ymax = std::max(ymax, v);
            }
        }
    }

    // Auto-range axes
    if (!freqs_GHz.isEmpty()) {
        m_plot->xAxis->setRange(freqs_GHz.first(), freqs_GHz.last());
    }
    if (std::isfinite(ymin) && std::isfinite(ymax)) {
        const double yrange = std::max(ymax - ymin, 1.0);
        m_plot->yAxis->setRange(ymin - 0.1 * yrange, ymax + 0.1 * yrange);
    }

    // 0 dB reference line
    auto* zeroLine = new QCPItemStraightLine(m_plot);
    zeroLine->point1->setCoords(0.0, 0.0);
    zeroLine->point2->setCoords(1.0, 0.0);
    QPen zeroPen(QColor(128, 128, 128, 150));
    zeroPen.setStyle(Qt::DashLine);
    zeroLine->setPen(zeroPen);

    // Create interactive overlay items (crosshair, tracers, readout)
    setupPlotInteractiveItems();

    m_plot->replot();
}

// ============================================================================
// INTERACTIVE PLOT OVERLAY ITEMS
// ============================================================================
// Called at the end of every plotResults() to create the overlay items that
// support the click-to-read-SE feature.  All items start invisible; they
// become visible on the first user click.

void MainWindow::setupPlotInteractiveItems()
{
    // -----------------------------------------------------------------------
    // Vertical crosshair line
    // -----------------------------------------------------------------------
    m_crosshairLine = new QCPItemLine(m_plot);
    QPen chPen(QColor(80, 80, 80, 200));
    chPen.setStyle(Qt::DashLine);
    chPen.setWidth(1);
    m_crosshairLine->setPen(chPen);
    // Use axis-coordinate anchors so the line stretches to plot edges
    m_crosshairLine->start->setType(QCPItemPosition::ptPlotCoords);
    m_crosshairLine->end->setType(QCPItemPosition::ptPlotCoords);
    m_crosshairLine->setVisible(false);

    // -----------------------------------------------------------------------
    // SE readout label (styled text box)
    // -----------------------------------------------------------------------
    m_readoutLabel = new QCPItemText(m_plot);
    m_readoutLabel->setPositionAlignment(Qt::AlignLeft | Qt::AlignTop);
    m_readoutLabel->position->setType(QCPItemPosition::ptPlotCoords);
    m_readoutLabel->setFont(QFont("Courier New", 9));
    m_readoutLabel->setColor(Qt::black);
    m_readoutLabel->setPadding(QMargins(6, 4, 6, 4));
    m_readoutLabel->setBrush(QBrush(QColor(255, 255, 240, 230)));  // Pale yellow
    QPen borderPen(QColor(100, 100, 100));
    borderPen.setWidth(1);
    m_readoutLabel->setPen(borderPen);
    m_readoutLabel->setVisible(false);

    // -----------------------------------------------------------------------
    // Per-curve tracers (filled circles that snap to data points)
    // -----------------------------------------------------------------------
    m_tracers.clear();
    static const QVector<QColor> colors = {
                                           { 37,  99, 235}, {220,  38,  38}, { 22, 163,  74},
                                           {217, 119,   6}, {147,  51, 234}, { 14, 165, 233},
                                           {244,  63,  94}, { 34, 197,  94},
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
        tracer->setInterpolating(true);   // Linear interpolation between data pts

        m_tracers.append(tracer);
    }
}

// ============================================================================
// INTERACTIVE PLOT CLICK HANDLER
// ============================================================================
// Triggered by QCustomPlot::mousePress.
// 1. Converts the pixel x-position to a frequency in GHz.
// 2. Finds the nearest frequency index in m_freqs.
// 3. Updates the crosshair, per-curve tracers, and the readout label.

void MainWindow::onPlotClicked(QMouseEvent* event)
{
    // Ignore if no data has been computed yet
    if (m_freqs.isEmpty() || m_SE_data.isEmpty()) return;

    // Ignore right-click (reserved for QCustomPlot context menu / drag)
    if (event->button() != Qt::LeftButton) return;

    // Convert pixel x → axis coordinate [GHz]
    const double clickedGHz =
        m_plot->xAxis->pixelToCoord(event->pos().x());

    // Find nearest index in the frequency array
    int    idx     = 0;
    double minDist = std::abs(m_freqs[0] - clickedGHz);
    for (int i = 1; i < m_freqs.size(); ++i) {
        const double dist = std::abs(m_freqs[i] - clickedGHz);
        if (dist < minDist) { minDist = dist; idx = i; }
    }
    const double f_GHz = m_freqs[idx];

    // -----------------------------------------------------------------------
    // Update crosshair: vertical line at f_GHz spanning the full y-range
    // -----------------------------------------------------------------------
    if (m_crosshairLine) {
        const double yLo = m_plot->yAxis->range().lower - 1.0;
        const double yHi = m_plot->yAxis->range().upper + 1.0;
        m_crosshairLine->start->setCoords(f_GHz, yLo);
        m_crosshairLine->end->setCoords(f_GHz, yHi);
        m_crosshairLine->setVisible(true);
    }

    // -----------------------------------------------------------------------
    // Update tracers: snap each dot to the data point at idx
    // -----------------------------------------------------------------------
    for (auto* tracer : m_tracers) {
        tracer->setGraphKey(f_GHz);
        tracer->setVisible(true);
    }

    // -----------------------------------------------------------------------
    // Build readout text
    //   Line 1: "f = X.XXXX GHz"
    //   Lines:  "P1 :  XX.XX dB"  (one per curve; NaN → "  N/A   ")
    // -----------------------------------------------------------------------
    QString text = QString("f = %1 GHz\n").arg(f_GHz, 0, 'f', 4);
    for (int c = 0; c < m_SE_data.size(); ++c) {
        const double se = m_SE_data[c].value(idx,
                                             std::numeric_limits<double>::quiet_NaN());
        const QString lbl = m_labels.value(c, QString("P%1").arg(c + 1));
        if (std::isfinite(se)) {
            text += QString("%1 : %2 dB\n")
            .arg(lbl, -4)
                .arg(se, 8, 'f', 2);
        } else {
            // Perfect shield or missing data
            text += QString("%1 :   ∞ dB\n").arg(lbl, -4);
        }
    }
    text = text.trimmed();

    // -----------------------------------------------------------------------
    // Position readout label — avoid running off the right or top edge
    // -----------------------------------------------------------------------
    if (m_readoutLabel) {
        const double xRange  = m_plot->xAxis->range().size();
        const double xOff    = m_plot->xAxis->range().lower;
        const double yTop    = m_plot->yAxis->range().upper;
        const double yRange  = m_plot->yAxis->range().size();

        // Flip to left alignment when click is in the right 40% of the plot
        const bool nearRight = (clickedGHz - xOff) > 0.60 * xRange;
        m_readoutLabel->setPositionAlignment(
            (nearRight ? Qt::AlignRight : Qt::AlignLeft) | Qt::AlignTop);

        // Place label near top of plot, slightly inset from axis edge
        m_readoutLabel->position->setCoords(f_GHz, yTop - 0.04 * yRange);
        m_readoutLabel->setText(text);
        m_readoutLabel->setVisible(true);
    }

    m_plot->replot();
}
