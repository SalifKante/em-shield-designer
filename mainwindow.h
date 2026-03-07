#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QVector>
#include <QString>

// Forward declarations — Qt widgets
class QCustomPlot;
class QCPItemLine;
class QCPItemText;
class QCPItemTracer;
class QAction;
class QToolBar;
class QLabel;
class QSplitter;
class QDoubleSpinBox;
class QSpinBox;
class QGroupBox;
class QComboBox;

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

    // --- Section management ---
    void onAddSection();
    void onRemoveSection();

    // --- Canvas ↔ property panel ---
    void onCanvasSelectionChanged(int sectionIndex);
    void onPropertyChanged(int sectionIndex, const SectionItemData& data);

    // --- Interactive plot readout ---
    // Called when the user clicks on the plot area.
    // Displays a vertical crosshair, per-curve markers, and a readout box
    // showing SE [dB] for every observation point at the clicked frequency.
    void onPlotClicked(QMouseEvent* event);

private:
    // -----------------------------------------------------------------------
    // Setup helpers
    // -----------------------------------------------------------------------
    void setupUI();
    void setupToolbar();
    void setupPlot();
    void setupControlPanel();
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

    // Creates / recreates interactive overlay items (crosshair, tracers,
    // readout label) after every plotResults() call.
    void setupPlotInteractiveItems();

    // -----------------------------------------------------------------------
    // UI components
    // -----------------------------------------------------------------------

    // Plot
    QCustomPlot*    m_plot         = nullptr;
    QSplitter*      m_splitter     = nullptr;

    // Circuit canvas & property panel
    CircuitCanvas*  m_canvas       = nullptr;
    PropertyPanel*  m_propertyPanel= nullptr;

    // Toolbar actions
    QToolBar*       m_toolbar      = nullptr;
    QAction*        m_actCompute   = nullptr;
    QAction*        m_actExport    = nullptr;
    QAction*        m_actAddSection    = nullptr;
    QAction*        m_actRemoveSection = nullptr;

    // Left control panel
    QWidget*        m_leftPanel    = nullptr;

    // Enclosure group
    QGroupBox*      m_grpEnclosure = nullptr;
    QDoubleSpinBox* m_spinA        = nullptr;   ///< Cavity width  a [mm]
    QDoubleSpinBox* m_spinB        = nullptr;   ///< Cavity height b [mm]
    QDoubleSpinBox* m_spinT        = nullptr;   ///< Wall thickness t [mm]
    QComboBox*      m_cboTopology  = nullptr;   ///< CASCADE / STAR_BRANCH selector

    // Frequency group
    QGroupBox*      m_grpFrequency = nullptr;
    QDoubleSpinBox* m_spinFstart   = nullptr;   ///< Start frequency [MHz]
    QDoubleSpinBox* m_spinFstop    = nullptr;   ///< Stop  frequency [MHz]
    QSpinBox*       m_spinPoints   = nullptr;   ///< Number of frequency points

    // Presets group
    QGroupBox*      m_grpPresets   = nullptr;
    QComboBox*      m_cboPreset    = nullptr;

    // Status bar
    QLabel*         m_lblStatus    = nullptr;

    // -----------------------------------------------------------------------
    // Interactive plot overlay items
    // (owned by QCustomPlot; pointers become invalid after clearItems())
    // -----------------------------------------------------------------------
    QCPItemLine*           m_crosshairLine  = nullptr;  ///< Vertical frequency cursor
    QCPItemText*           m_readoutLabel   = nullptr;  ///< SE value readout box
    QVector<QCPItemTracer*> m_tracers;                  ///< One dot per SE curve

    // -----------------------------------------------------------------------
    // Result data (stored for CSV export and interactive readout)
    // -----------------------------------------------------------------------
    QVector<double>          m_freqs;    ///< Frequency axis [GHz]
    QVector<QVector<double>> m_SE_data; ///< SE_data[curve][freq_index] [dB]
    QVector<QString>         m_labels;  ///< Curve labels ("P1", "P2", …)
};

#endif // MAINWINDOW_H
