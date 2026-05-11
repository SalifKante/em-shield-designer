#ifndef MAINWINDOW_H
#define MAINWINDOW_H

// ============================================================================
//  mainwindow.h — Quick Simulation window  (Window 1)
//
//  [T2.1a] UI redesign to match Window 2 (Circuit Builder):
//      * QToolBar removed; Add/Remove/Compute/Export moved into the left
//        panel as styled buttons
//      * EMShieldQuickSim brand strip at the top of the left panel
//      * Section headers using EMStyle::sectionHeaderQSS
//      * Spin boxes / combo boxes styled via Styles.h helpers
//      * Live validity indicator (red/green dot + brief text + tooltip)
//        wired to EnclosureConfig::isValid + CircuitCanvas signals
//      * QMessageBox calls replaced with MessageDialog (error/success)
//      * Plot styled to match Window 2's plot
//      * Window title pattern aligned with Circuit Builder
//
//  Functional behaviour preserved:
//      * runAnalysis() pipeline unchanged
//      * Preset handling, topology selector, CSV format unchanged
//      * Interactive plot readout (crosshair, tracers, label) unchanged
// ============================================================================

#include <QMainWindow>
#include <QVector>
#include <QString>

// Forward declarations — Qt widgets
class QCustomPlot;
class QCPItemLine;
class QCPItemText;
class QCPItemTracer;
class QLabel;
class QSplitter;
class QDoubleSpinBox;
class QSpinBox;
class QFrame;
class QComboBox;
class QPushButton;
class QWidget;

// Forward declarations — application classes
class CircuitCanvas;
class PropertyPanel;
struct SectionItemData;

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow() override;

private slots:
    // --- Computation ---
    void onComputeClicked();
    void onExportCSV();

    // --- Preset / topology ---
    void onPresetChanged(int index);
    void onTopologyChanged(int index);

    // --- Section management ---
    void onAddSection();
    void onRemoveSection();

    // --- Canvas <-> property panel ---
    void onCanvasSelectionChanged(int sectionIndex);
    void onPropertyChanged(int sectionIndex, const SectionItemData& data);
    void onSectionCountChanged(int count);

    // --- Interactive plot readout ---
    // Called when the user clicks on the plot area. Displays a vertical
    // crosshair, per-curve markers, and a readout box showing SE [dB] for
    // every observation point at the clicked frequency.
    void onPlotClicked(QMouseEvent* event);

private:
    // -----------------------------------------------------------------------
    // Setup helpers
    // -----------------------------------------------------------------------
    void setupUI();
    void setupPlot();

    // [T2.1a] Build the left panel from scratch, matching the Window 2
    // pattern: brand strip, scrollable section stack, primary button row
    // at the bottom. Returns a QWidget owning the entire panel.
    QWidget* buildLeftPanel();

    // [T2.1a] Status bar carries the post-compute summary as before.
    void setupStatusBar();

    // -----------------------------------------------------------------------
    // Analysis pipeline
    // -----------------------------------------------------------------------
    void loadPreset(int presetId);
    void runAnalysis();

    // -----------------------------------------------------------------------
    // Plot helpers
    // -----------------------------------------------------------------------
    void plotResults(const QVector<double>&          freqs_GHz,
                     const QVector<QVector<double>>& SE_curves,
                     const QVector<QString>&          labels);
    void setupPlotInteractiveItems();
    void clearPlot();

    // -----------------------------------------------------------------------
    // [T2.1a] Live validity indicator — same pattern as Window 2 (Task 1.5b)
    //
    // EnclosureConfig::isValid is cheap (bounds checks only), so we
    // re-evaluate on every relevant change:
    //   - canvas section count changes (add/remove)
    //   - property panel emits dataChanged
    //   - any enclosure / frequency spin box value changes
    //   - topology selector changes
    // -----------------------------------------------------------------------
    void refreshValidityIndicator();

    // [T2.1a] Tiny helper that sets the colour + text + tooltip of the
    // validity row in one call. Avoids duplication across the call sites.
    void setValidityState(bool ok, const QString& brief, const QString& full);

    // -----------------------------------------------------------------------
    // [T2.1a] Status-bar text setter — same API as Window 2.
    // -----------------------------------------------------------------------
    void setStatus(const QString& msg, const class QColor& col);

    // -----------------------------------------------------------------------
    // UI components
    // -----------------------------------------------------------------------

    // Plot
    QCustomPlot*    m_plot         = nullptr;
    QSplitter*      m_splitter     = nullptr;

    // Circuit canvas & property panel
    CircuitCanvas*  m_canvas       = nullptr;
    PropertyPanel*  m_propertyPanel= nullptr;

    // Left control panel (built in buildLeftPanel)
    QWidget*        m_leftPanel    = nullptr;

    // Form widgets (still QDoubleSpinBox / QSpinBox / QComboBox — only
    // their styling and parenting changes from the old version)
    QDoubleSpinBox* m_spinA        = nullptr;   ///< Cavity width  a [mm]
    QDoubleSpinBox* m_spinB        = nullptr;   ///< Cavity height b [mm]
    QDoubleSpinBox* m_spinT        = nullptr;   ///< Wall thickness t [mm]
    QComboBox*      m_cboTopology  = nullptr;   ///< CASCADE / STAR_BRANCH selector

    QDoubleSpinBox* m_spinFstart   = nullptr;   ///< Start frequency [MHz]
    QDoubleSpinBox* m_spinFstop    = nullptr;   ///< Stop  frequency [MHz]
    QSpinBox*       m_spinPoints   = nullptr;   ///< Number of frequency points

    QComboBox*      m_cboPreset    = nullptr;

    // Action buttons (replaces the old toolbar)
    QPushButton*    m_btnAddSection    = nullptr;
    QPushButton*    m_btnRemoveSection = nullptr;
    QPushButton*    m_btnCompute       = nullptr;
    QPushButton*    m_btnExport        = nullptr;

    // [T2.1a] Validity indicator row above the COMPUTE button
    QWidget*        m_validityRow   = nullptr;
    QFrame*         m_validityDot   = nullptr;
    QLabel*         m_validityLabel = nullptr;

    // Status bar
    QLabel*         m_lblStatus    = nullptr;

    // -----------------------------------------------------------------------
    // Interactive plot overlay items
    // (owned by QCustomPlot; pointers become invalid after clearItems())
    // -----------------------------------------------------------------------
    QCPItemLine*           m_crosshairLine  = nullptr;
    QCPItemText*           m_readoutLabel   = nullptr;
    QVector<QCPItemTracer*> m_tracers;

    // -----------------------------------------------------------------------
    // Result data (stored for CSV export and interactive readout)
    // -----------------------------------------------------------------------
    QVector<double>          m_freqs;
    QVector<QVector<double>> m_SE_data;
    QVector<QString>         m_labels;
};

#endif // MAINWINDOW_H
