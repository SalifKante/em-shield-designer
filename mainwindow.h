#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QVector>
#include <QString>

// Forward declarations
class QCustomPlot;
class QAction;
class QToolBar;
class QStatusBar;
class QLabel;
class QSplitter;
class QDoubleSpinBox;
class QSpinBox;
class QGroupBox;
class QVBoxLayout;
class QComboBox;
class CircuitCanvas;
class PropertyPanel;

struct SectionItemData;

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void onComputeClicked();
    void onExportCSV();
    void onPresetChanged(int index);
    void onAddSection();
    void onRemoveSection();
    void onCanvasSelectionChanged(int sectionIndex);
    void onPropertyChanged(int sectionIndex, const SectionItemData& data);

private:
    void setupUI();
    void setupToolbar();
    void setupPlot();
    void setupControlPanel();
    void setupStatusBar();
    void loadPreset(int presetId);

    void runAnalysis();
    void plotResults(const QVector<double>& freqs_GHz,
                     const QVector<QVector<double>>& SE_curves,
                     const QVector<QString>& labels);

    // --- UI Components ---
    QCustomPlot*    m_plot;
    QSplitter*      m_splitter;

    // Circuit canvas
    CircuitCanvas*  m_canvas;

    // Property panel
    PropertyPanel*  m_propertyPanel;

    // Toolbar
    QToolBar*       m_toolbar;
    QAction*        m_actCompute;
    QAction*        m_actExport;
    QAction*        m_actAddSection;
    QAction*        m_actRemoveSection;

    // Control panel widgets
    QWidget*        m_leftPanel;
    QGroupBox*      m_grpEnclosure;
    QDoubleSpinBox* m_spinA;
    QDoubleSpinBox* m_spinB;
    QDoubleSpinBox* m_spinT;

    QGroupBox*      m_grpFrequency;
    QDoubleSpinBox* m_spinFstart;
    QDoubleSpinBox* m_spinFstop;
    QSpinBox*       m_spinPoints;

    QGroupBox*      m_grpPresets;
    QComboBox*      m_cboPreset;

    // Status bar
    QLabel*         m_lblStatus;

    // --- Data ---
    QVector<double>          m_freqs;
    QVector<QVector<double>> m_SE_data;
    QVector<QString>         m_labels;
};

#endif // MAINWINDOW_H
