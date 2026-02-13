#include "mainwindow.h"
#include "qcustomplot.h"
#include "CircuitCanvas.h"
#include "PropertyPanel.h"

// Core engine includes
#include "include/core/CircuitGenerator.h"

#include <QApplication>
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
#include <QScrollArea>

#include <cmath>
#include <fstream>
#include <iomanip>

using namespace EMCore;

// ============================================================================
// CONSTRUCTOR / DESTRUCTOR
// ============================================================================

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
{
    setWindowTitle("EMShieldDesigner - Shielding Effectiveness Analyzer");
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
    // Main splitter: [Left Panel | Canvas | Plot]
    m_splitter = new QSplitter(Qt::Horizontal, this);
    setCentralWidget(m_splitter);

    // =============================================
    // LEFT PANEL: Controls + Property Panel
    // =============================================
    m_leftPanel = new QWidget;
    QVBoxLayout* leftLayout = new QVBoxLayout(m_leftPanel);
    leftLayout->setContentsMargins(6, 6, 6, 6);
    leftLayout->setSpacing(8);

    setupControlPanel();

    // Scrollable area for left panel content
    leftLayout->addWidget(m_grpPresets);
    leftLayout->addWidget(m_grpEnclosure);
    leftLayout->addWidget(m_grpFrequency);

    // Separator
    QFrame* sep = new QFrame;
    sep->setFrameShape(QFrame::HLine);
    sep->setFrameShadow(QFrame::Sunken);
    leftLayout->addWidget(sep);

    // Property panel
    m_propertyPanel = new PropertyPanel;
    leftLayout->addWidget(m_propertyPanel);

    // Compute button at bottom
    QPushButton* btnCompute = new QPushButton("Compute SE");
    btnCompute->setMinimumHeight(38);
    btnCompute->setStyleSheet(
        "QPushButton {"
        "  background-color: #2563eb; color: white;"
        "  font-weight: bold; font-size: 13px;"
        "  border-radius: 6px; border: none;"
        "}"
        "QPushButton:hover { background-color: #1d4ed8; }"
        "QPushButton:pressed { background-color: #1e40af; }"
        );
    connect(btnCompute, &QPushButton::clicked, this, &MainWindow::onComputeClicked);
    leftLayout->addWidget(btnCompute);

    m_leftPanel->setLayout(leftLayout);
    m_leftPanel->setMaximumWidth(280);

    // =============================================
    // CENTER: Circuit Canvas
    // =============================================
    m_canvas = new CircuitCanvas;
    m_canvas->setMinimumHeight(120);

    // =============================================
    // RIGHT: SE Plot
    // =============================================
    setupPlot();

    // --- Vertical splitter for canvas (top) and plot (bottom) ---
    QSplitter* rightSplitter = new QSplitter(Qt::Vertical);
    rightSplitter->addWidget(m_canvas);
    rightSplitter->addWidget(m_plot);
    rightSplitter->setStretchFactor(0, 1);  // Canvas: smaller
    rightSplitter->setStretchFactor(1, 3);  // Plot: larger
    rightSplitter->setSizes({180, 520});

    // Add to main splitter
    m_splitter->addWidget(m_leftPanel);
    m_splitter->addWidget(rightSplitter);
    m_splitter->setStretchFactor(0, 0);
    m_splitter->setStretchFactor(1, 1);
    m_splitter->setSizes({270, 1130});

    // =============================================
    // SIGNAL CONNECTIONS
    // =============================================
    connect(m_canvas, &CircuitCanvas::selectionChanged,
            this, &MainWindow::onCanvasSelectionChanged);

    connect(m_propertyPanel, &PropertyPanel::dataChanged,
            this, &MainWindow::onPropertyChanged);
}

void MainWindow::setupControlPanel()
{
    // === Presets ===
    m_grpPresets = new QGroupBox("Preset");
    QVBoxLayout* presetLayout = new QVBoxLayout;

    m_cboPreset = new QComboBox;
    m_cboPreset->addItem("1-Section (baseline)",          1);
    m_cboPreset->addItem("2-Section identical",           2);
    m_cboPreset->addItem("3-Section identical",           3);
    m_cboPreset->addItem("2-Section different",           4);
    m_cboPreset->addItem("5-Section cascade",             5);
    m_cboPreset->addItem("Custom (edit below)",           0);
    connect(m_cboPreset, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &MainWindow::onPresetChanged);

    presetLayout->addWidget(m_cboPreset);
    m_grpPresets->setLayout(presetLayout);

    // === Enclosure ===
    m_grpEnclosure = new QGroupBox("Enclosure");
    QVBoxLayout* encLayout = new QVBoxLayout;

    auto addSpinRow = [](QVBoxLayout* layout, const QString& label,
                         QDoubleSpinBox*& spin, double value,
                         double min, double max, const QString& suffix) {
        QHBoxLayout* row = new QHBoxLayout;
        QLabel* lbl = new QLabel(label);
        lbl->setMinimumWidth(90);
        spin = new QDoubleSpinBox;
        spin->setRange(min, max);
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

    m_grpEnclosure->setLayout(encLayout);

    // === Frequency ===
    m_grpFrequency = new QGroupBox("Frequency");
    QVBoxLayout* freqLayout = new QVBoxLayout;

    addSpinRow(freqLayout, "Start:", m_spinFstart,    1.0,   0.1,  5000.0, " MHz");
    addSpinRow(freqLayout, "Stop:",  m_spinFstop,  2000.0,  10.0, 32000.0, " MHz");

    QHBoxLayout* ptsRow = new QHBoxLayout;
    QLabel* lblPts = new QLabel("Points:");
    lblPts->setMinimumWidth(90);
    m_spinPoints = new QSpinBox;
    m_spinPoints->setRange(50, 5000);
    m_spinPoints->setValue(200);
    ptsRow->addWidget(lblPts);
    ptsRow->addWidget(m_spinPoints);
    freqLayout->addLayout(ptsRow);

    m_grpFrequency->setLayout(freqLayout);
}

void MainWindow::setupPlot()
{
    m_plot = new QCustomPlot;

    // Title
    m_plot->plotLayout()->insertRow(0);
    QCPTextElement* title = new QCPTextElement(m_plot,
                                               "Shielding Effectiveness vs Frequency", QFont("sans-serif", 12, QFont::Bold));
    m_plot->plotLayout()->addElement(0, 0, title);

    // Axes
    m_plot->xAxis->setLabel("Frequency [GHz]");
    m_plot->yAxis->setLabel("SE [dB]");
    m_plot->xAxis->setRange(0, 2.0);
    m_plot->yAxis->setRange(-40, 100);

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
    m_plot->axisRect()->insetLayout()->setInsetAlignment(0, Qt::AlignTop | Qt::AlignRight);

    // Interactions
    m_plot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom |
                            QCP::iSelectPlottables | QCP::iSelectLegend);
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
    if (presetId > 0) {
        loadPreset(presetId);
    }
}

void MainWindow::loadPreset(int presetId)
{
    QVector<SectionItemData> sections;
    SectionItemData sec;

    switch (presetId) {
    case 1: // 1-section
        sec.depth_mm = 300; sec.obs_position_mm = 150;
        sec.aperture_l_mm = 80; sec.aperture_w_mm = 80;
        sections = {sec};
        break;

    case 2: // 2-section identical
        sec.depth_mm = 300; sec.obs_position_mm = 150;
        sec.aperture_l_mm = 80; sec.aperture_w_mm = 80;
        sections = {sec, sec};
        break;

    case 3: // 3-section identical
        sec.depth_mm = 300; sec.obs_position_mm = 150;
        sec.aperture_l_mm = 80; sec.aperture_w_mm = 80;
        sections = {sec, sec, sec};
        break;

    case 4: { // 2-section different
        SectionItemData sec1;
        sec1.depth_mm = 300; sec1.obs_position_mm = 150;
        sec1.aperture_l_mm = 80; sec1.aperture_w_mm = 80;

        SectionItemData sec2;
        sec2.depth_mm = 200; sec2.obs_position_mm = 100;
        sec2.aperture_l_mm = 50; sec2.aperture_w_mm = 50;

        sections = {sec1, sec2};
        break;
    }
    case 5: // 5-section
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
    // Add a section with default parameters
    SectionItemData newSec;
    newSec.depth_mm = 300;
    newSec.obs_position_mm = 150;
    newSec.aperture_l_mm = 80;
    newSec.aperture_w_mm = 80;

    m_canvas->addSection(newSec);

    // Select the new section
    m_canvas->selectSection(m_canvas->sectionCount() - 1);

    // Switch to "Custom" preset
    m_cboPreset->blockSignals(true);
    m_cboPreset->setCurrentIndex(m_cboPreset->count() - 1);  // "Custom"
    m_cboPreset->blockSignals(false);

    m_lblStatus->setText(QString("Added section %1 - click Compute to update")
                             .arg(m_canvas->sectionCount()));
}

void MainWindow::onRemoveSection()
{
    if (m_canvas->sectionCount() <= 1) {
        m_lblStatus->setText("Cannot remove last section");
        return;
    }

    m_canvas->removeSelectedSection();

    // Switch to "Custom" preset
    m_cboPreset->blockSignals(true);
    m_cboPreset->setCurrentIndex(m_cboPreset->count() - 1);
    m_cboPreset->blockSignals(false);

    m_lblStatus->setText(QString("%1 sections remaining - click Compute to update")
                             .arg(m_canvas->sectionCount()));
}

// ============================================================================
// CANVAS <-> PROPERTY PANEL COMMUNICATION
// ============================================================================

void MainWindow::onCanvasSelectionChanged(int sectionIndex)
{
    if (sectionIndex >= 0) {
        SectionItem* item = m_canvas->sectionAt(sectionIndex);
        if (item) {
            m_propertyPanel->loadSection(sectionIndex, item->data());
        }
    } else {
        m_propertyPanel->clearSelection();
    }
}

void MainWindow::onPropertyChanged(int sectionIndex, const SectionItemData& data)
{
    SectionItem* item = m_canvas->sectionAt(sectionIndex);
    if (item) {
        item->setData(data);

        // Switch to "Custom" preset
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

    QString filename = QFileDialog::getSaveFileName(this, "Export CSV",
                                                    "SE_results.csv", "CSV Files (*.csv)");
    if (filename.isEmpty()) return;

    std::ofstream file(filename.toStdString());
    if (!file.is_open()) {
        QMessageBox::critical(this, "Error", "Could not open file for writing.");
        return;
    }

    file << "f_GHz";
    for (const auto& label : m_labels)
        file << ",SE_" << label.toStdString() << "_dB";
    file << "\n";

    file << std::fixed << std::setprecision(9);
    for (int i = 0; i < m_freqs.size(); ++i) {
        file << m_freqs[i];
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

void MainWindow::onComputeClicked()
{
    runAnalysis();
}

void MainWindow::runAnalysis()
{
    QElapsedTimer timer;
    timer.start();

    // Read enclosure parameters from UI
    double a = m_spinA->value() / 1000.0;
    double b = m_spinB->value() / 1000.0;
    double t = m_spinT->value() / 1000.0;
    double f_start = m_spinFstart->value() * 1e6;
    double f_stop  = m_spinFstop->value()  * 1e6;
    int    num_points = m_spinPoints->value();

    // Build EnclosureConfig from canvas sections
    EnclosureConfig cfg;
    cfg.a = a;
    cfg.b = b;
    cfg.t = t;

    QVector<SectionItemData> sectionData = m_canvas->allSectionData();
    if (sectionData.isEmpty()) {
        m_lblStatus->setText("No sections defined");
        return;
    }

    for (const auto& sd : sectionData) {
        SectionConfig sec;
        sec.depth          = sd.depth_mm / 1000.0;
        sec.obs_position   = sd.obs_position_mm / 1000.0;
        sec.has_observation = sd.has_observation;
        sec.aperture_l     = sd.aperture_l_mm / 1000.0;
        sec.aperture_w     = sd.aperture_w_mm / 1000.0;
        sec.has_cover      = sd.has_cover;
        sec.cover_gap      = sd.cover_gap_mm / 1000.0;
        sec.has_dielectric  = sd.has_dielectric;
        sec.dielectric_h   = sd.dielectric_h_mm / 1000.0;
        sec.dielectric_er  = sd.dielectric_er;
        cfg.sections.push_back(sec);
    }

    // Validate
    std::string error_msg;
    if (!cfg.isValid(error_msg)) {
        QMessageBox::warning(this, "Invalid Configuration",
                             QString::fromStdString(error_msg));
        return;
    }

    m_lblStatus->setText("Computing...");
    QApplication::processEvents();

    // Generate circuit
    MNASolver solver;
    std::vector<ObservationPoint> obs_points;
    try {
        obs_points = CircuitGenerator::generate(cfg, solver, false);
    } catch (const std::exception& e) {
        QMessageBox::critical(this, "Error",
                              "Circuit generation failed: " + QString::fromStdString(e.what()));
        return;
    }

    // Frequency sweep
    double fstep = (f_stop - f_start) / num_points;

    QVector<double> freqs_GHz;
    freqs_GHz.reserve(num_points);

    QVector<QVector<double>> SE_curves(static_cast<int>(obs_points.size()));
    for (auto& curve : SE_curves)
        curve.reserve(num_points);

    QVector<QString> labels;
    for (const auto& op : obs_points)
        labels.append(QString::fromStdString(op.label));

    for (int i = 0; i < num_points; ++i) {
        double f = f_start + i * fstep;
        freqs_GHz.append(f / 1e9);

        auto SE_vals = CircuitGenerator::computeSE(solver, obs_points, f);

        for (size_t oi = 0; oi < obs_points.size(); ++oi)
            SE_curves[static_cast<int>(oi)].append(SE_vals[oi]);
    }

    qint64 elapsed = timer.elapsed();

    // Store for export
    m_freqs   = freqs_GHz;
    m_SE_data = SE_curves;
    m_labels  = labels;

    // Plot
    plotResults(freqs_GHz, SE_curves, labels);

    // Status
    int N = static_cast<int>(cfg.sections.size());
    m_lblStatus->setText(
        QString("%1 sections | %2 branches, %3 nodes, %4 obs pts | %5 points in %6 ms")
            .arg(N)
            .arg(3*N + 1)
            .arg(2*N)
            .arg(static_cast<int>(obs_points.size()))
            .arg(num_points)
            .arg(elapsed)
        );
}

// ============================================================================
// PLOTTING
// ============================================================================

void MainWindow::plotResults(const QVector<double>& freqs_GHz,
                             const QVector<QVector<double>>& SE_curves,
                             const QVector<QString>& labels)
{
    m_plot->clearPlottables();
    m_plot->clearItems();

    QVector<QColor> colors = {
        QColor(37, 99, 235),     // Blue
        QColor(220, 38, 38),     // Red
        QColor(22, 163, 74),     // Green
        QColor(217, 119, 6),     // Amber
        QColor(147, 51, 234),    // Purple
        QColor(14, 165, 233),    // Sky
        QColor(244, 63, 94),     // Rose
        QColor(34, 197, 94),     // Emerald
    };

    double ymin = 1e9, ymax = -1e9;

    for (int c = 0; c < SE_curves.size(); ++c) {
        QCPGraph* graph = m_plot->addGraph();
        graph->setData(freqs_GHz, SE_curves[c]);
        graph->setName(labels[c]);

        QPen pen(colors[c % colors.size()]);
        pen.setWidth(2);
        graph->setPen(pen);
        graph->setSelectable(QCP::stWhole);

        for (double v : SE_curves[c]) {
            if (std::isfinite(v)) {
                if (v < ymin) ymin = v;
                if (v > ymax) ymax = v;
            }
        }
    }

    m_plot->xAxis->setRange(freqs_GHz.first(), freqs_GHz.last());
    double yrange = ymax - ymin;
    if (yrange < 1.0) yrange = 1.0;
    m_plot->yAxis->setRange(ymin - 0.1 * yrange, ymax + 0.1 * yrange);

    // 0 dB reference line
    QCPItemStraightLine* zeroLine = new QCPItemStraightLine(m_plot);
    zeroLine->point1->setCoords(0, 0);
    zeroLine->point2->setCoords(1, 0);
    QPen zeroPen(QColor(128, 128, 128, 150));
    zeroPen.setStyle(Qt::DashLine);
    zeroLine->setPen(zeroPen);

    m_plot->replot();
}
