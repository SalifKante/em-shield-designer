#ifndef PROPERTYPANEL_H
#define PROPERTYPANEL_H

#include "SectionItem.h"

#include <QWidget>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGroupBox>
#include <QLabel>
#include <QDoubleSpinBox>
#include <QCheckBox>
#include <QFrame>

// ============================================================================
// PROPERTY PANEL
// ============================================================================
// Displays and edits parameters for the currently selected SectionItem.
//
// All fields mirror SectionItemData exactly (see SectionItem.h).
// loadSection() populates every widget without emitting dataChanged
// (via m_blockSignals guard).
// Any user edit triggers emitDataChanged() → dataChanged() signal.
//
// Widget ↔ SectionItemData field map
// ───────────────────────────────────────────────────────────────────────────
// m_spinDepth          ↔  depth_mm
// m_spinObsPos         ↔  obs_position_mm   min=0.1 mm (strictly positive)
// m_chkObservation     ↔  has_observation
// m_spinWidthA         ↔  section_width_a_mm  (-1 = use global a)
// m_spinApL            ↔  aperture_l_mm
// m_spinApW            ↔  aperture_w_mm
// m_chkCover           ↔  has_cover
// m_spinCoverGap       ↔  cover_gap_mm
// m_spinCoverEpsR      ↔  cover_eps_r        1.0=air Eq.(3.22), >1.0 Eq.(3.24)
// m_chkDielectric      ↔  has_dielectric
// m_spinDielH          ↔  dielectric_h_mm
// m_spinDielEr         ↔  dielectric_er      min=1.0
// ============================================================================

class PropertyPanel : public QWidget
{
    Q_OBJECT

public:
    explicit PropertyPanel(QWidget* parent = nullptr)
        : QWidget(parent)
    {
        setupUI();
        setEnabled(false);   // Disabled until a section is selected
    }

    // -----------------------------------------------------------------------
    // Load data from a section item — does NOT emit dataChanged
    // -----------------------------------------------------------------------
    void loadSection(int index, const SectionItemData& data)
    {
        m_blockSignals = true;
        m_currentIndex = index;

        m_lblTitle->setText(QString("Section %1 Properties").arg(index + 1));

        // Cavity
        m_spinDepth->setValue(data.depth_mm);
        m_spinObsPos->setValue(data.obs_position_mm);
        m_chkObservation->setChecked(data.has_observation);
        m_spinObsPos->setEnabled(data.has_observation);

        // Per-section width override (STAR_BRANCH)
        // -1 is stored as 0 in the spinbox (sentinel → "use global a")
        m_spinWidthA->setValue(
            data.section_width_a_mm > 0.0 ? data.section_width_a_mm : 0.0);

        // Aperture
        m_spinApL->setValue(data.aperture_l_mm);
        m_spinApW->setValue(data.aperture_w_mm);

        // Cover
        m_chkCover->setChecked(data.has_cover);
        m_spinCoverGap->setValue(data.cover_gap_mm);
        m_spinCoverGap->setEnabled(data.has_cover);
        m_spinCoverEpsR->setValue(data.cover_eps_r);
        m_spinCoverEpsR->setEnabled(data.has_cover);

        // Dielectric
        m_chkDielectric->setChecked(data.has_dielectric);
        m_spinDielH->setValue(data.dielectric_h_mm);
        m_spinDielEr->setValue(data.dielectric_er);
        m_spinDielH->setEnabled(data.has_dielectric);
        m_spinDielEr->setEnabled(data.has_dielectric);

        setEnabled(true);
        m_blockSignals = false;
    }

    void clearSelection()
    {
        m_currentIndex = -1;
        m_lblTitle->setText("No Section Selected");
        setEnabled(false);
    }

    int currentIndex() const { return m_currentIndex; }

signals:
    void dataChanged(int sectionIndex, const SectionItemData& data);

private:

    // -----------------------------------------------------------------------
    // UI construction
    // -----------------------------------------------------------------------
    void setupUI()
    {
        QVBoxLayout* mainLayout = new QVBoxLayout(this);
        mainLayout->setContentsMargins(0, 0, 0, 0);
        mainLayout->setSpacing(8);

        // Title
        m_lblTitle = new QLabel("No Section Selected");
        m_lblTitle->setStyleSheet(
            "font-weight: bold; font-size: 12px; color: #1e3a5f;");
        mainLayout->addWidget(m_lblTitle);

        // ---- Cavity ----
        QGroupBox* grpCavity = new QGroupBox("Cavity");
        QVBoxLayout* cavLayout = new QVBoxLayout;
        cavLayout->setSpacing(4);

        addSpinRow(cavLayout, "Depth:", m_spinDepth,
                   10.0, 2000.0, 300.0, " mm", 1);

        // obs_position minimum is 0.1 mm (strictly positive) because
        // SectionConfig::isValid() rejects obs_position == 0 — it would
        // produce a zero-length TL segment rejected by TL_EmptyCavity.
        addSpinRow(cavLayout, "Obs position:", m_spinObsPos,
                   0.1, 2000.0, 150.0, " mm", 1);

        m_chkObservation = new QCheckBox("Has observation point");
        m_chkObservation->setChecked(true);
        cavLayout->addWidget(m_chkObservation);

        grpCavity->setLayout(cavLayout);
        mainLayout->addWidget(grpCavity);

        // ---- Width override (STAR_BRANCH) ----
        // 0.0 is the sentinel for "use global a" (maps to section_width_a_mm = -1)
        QGroupBox* grpWidth = new QGroupBox("Width Override (STAR_BRANCH)");
        QVBoxLayout* widthLayout = new QVBoxLayout;
        widthLayout->setSpacing(4);

        QLabel* lblWidthHint = new QLabel(
            "Set > 0 to override the global enclosure\n"
            "width for this section (Fig. 3.11).\n"
            "0 = use global a.");
        lblWidthHint->setStyleSheet("color: #555; font-size: 9px;");
        lblWidthHint->setWordWrap(true);
        widthLayout->addWidget(lblWidthHint);

        addSpinRow(widthLayout, "Width (a):", m_spinWidthA,
                   0.0, 2000.0, 0.0, " mm", 1);

        grpWidth->setLayout(widthLayout);
        mainLayout->addWidget(grpWidth);

        // ---- Aperture ----
        QGroupBox* grpAperture = new QGroupBox("Aperture");
        QVBoxLayout* apLayout = new QVBoxLayout;
        apLayout->setSpacing(4);

        addSpinRow(apLayout, "Width (l):",  m_spinApL,
                   1.0, 500.0, 80.0, " mm", 1);
        addSpinRow(apLayout, "Height (w):", m_spinApW,
                   1.0, 500.0, 80.0, " mm", 1);

        grpAperture->setLayout(apLayout);
        mainLayout->addWidget(grpAperture);

        // ---- Cover ----
        QGroupBox* grpCover = new QGroupBox("Aperture Cover");
        QVBoxLayout* covLayout = new QVBoxLayout;
        covLayout->setSpacing(4);

        m_chkCover = new QCheckBox("Enable cover");
        covLayout->addWidget(m_chkCover);

        addSpinRow(covLayout, "Gap (τ):", m_spinCoverGap,
                   0.01, 50.0, 1.0, " mm", 2);
        m_spinCoverGap->setEnabled(false);

        // cover_eps_r: 1.0 → air gap Eq.(3.22); > 1.0 → dielectric Eq.(3.24)
        addSpinRow(covLayout, "Gap εr:", m_spinCoverEpsR,
                   1.0, 100.0, 1.0, "", 2);
        m_spinCoverEpsR->setEnabled(false);
        m_spinCoverEpsR->setToolTip(
            "Relative permittivity of the cover gap filler.\n"
            "1.0 = air gap  → uses Eq. (3.22).\n"
            "> 1.0 = dielectric filler → uses Eq. (3.24).");

        // Equation hint label — updates dynamically
        m_lblCoverEq = new QLabel("Eq. (3.22) — air gap");
        m_lblCoverEq->setStyleSheet("color: #666; font-size: 9px;");
        m_lblCoverEq->setEnabled(false);
        covLayout->addWidget(m_lblCoverEq);

        grpCover->setLayout(covLayout);
        mainLayout->addWidget(grpCover);

        // ---- Dielectric fill ----
        QGroupBox* grpDiel = new QGroupBox("Dielectric Fill");
        QVBoxLayout* dielLayout = new QVBoxLayout;
        dielLayout->setSpacing(4);

        m_chkDielectric = new QCheckBox("Enable dielectric");
        dielLayout->addWidget(m_chkDielectric);

        addSpinRow(dielLayout, "Height (h):", m_spinDielH,
                   0.1, 500.0, 60.0, " mm", 1);
        // dielectric_er minimum is 1.0, matching SectionConfig::isValid()
        addSpinRow(dielLayout, "εr:", m_spinDielEr,
                   1.0, 100.0, 4.4, "", 2);
        m_spinDielH->setEnabled(false);
        m_spinDielEr->setEnabled(false);

        grpDiel->setLayout(dielLayout);
        mainLayout->addWidget(grpDiel);

        mainLayout->addStretch();

        // ---------------------------------------------------------------
        // Connect all widget signals → emitDataChanged
        // ---------------------------------------------------------------
        auto emitChange = [this]() { emitDataChanged(); };

        connect(m_spinDepth,    QOverload<double>::of(&QDoubleSpinBox::valueChanged),
                this, emitChange);
        connect(m_spinObsPos,   QOverload<double>::of(&QDoubleSpinBox::valueChanged),
                this, emitChange);
        connect(m_spinWidthA,   QOverload<double>::of(&QDoubleSpinBox::valueChanged),
                this, emitChange);
        connect(m_spinApL,      QOverload<double>::of(&QDoubleSpinBox::valueChanged),
                this, emitChange);
        connect(m_spinApW,      QOverload<double>::of(&QDoubleSpinBox::valueChanged),
                this, emitChange);
        connect(m_spinCoverGap, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
                this, emitChange);
        connect(m_spinDielH,    QOverload<double>::of(&QDoubleSpinBox::valueChanged),
                this, emitChange);
        connect(m_spinDielEr,   QOverload<double>::of(&QDoubleSpinBox::valueChanged),
                this, emitChange);

        // cover_eps_r spinbox: emit data AND update the equation hint label
        connect(m_spinCoverEpsR,
                QOverload<double>::of(&QDoubleSpinBox::valueChanged),
                this, [this](double val) {
                    updateCoverEqLabel(val);
                    emitDataChanged();
                });

        connect(m_chkObservation, &QCheckBox::toggled, this,
                [this](bool checked) {
                    m_spinObsPos->setEnabled(checked);
                    emitDataChanged();
                });

        connect(m_chkCover, &QCheckBox::toggled, this,
                [this](bool checked) {
                    m_spinCoverGap->setEnabled(checked);
                    m_spinCoverEpsR->setEnabled(checked);
                    m_lblCoverEq->setEnabled(checked);
                    emitDataChanged();
                });

        connect(m_chkDielectric, &QCheckBox::toggled, this,
                [this](bool checked) {
                    m_spinDielH->setEnabled(checked);
                    m_spinDielEr->setEnabled(checked);
                    emitDataChanged();
                });
    }

    // -----------------------------------------------------------------------
    // Helper: add a labelled QDoubleSpinBox row to a QVBoxLayout
    // -----------------------------------------------------------------------
    void addSpinRow(QVBoxLayout*     layout,
                    const QString&   label,
                    QDoubleSpinBox*& spin,
                    double           minVal,
                    double           maxVal,
                    double           defaultVal,
                    const QString&   suffix,
                    int              decimals)
    {
        QHBoxLayout* row = new QHBoxLayout;
        QLabel* lbl = new QLabel(label);
        lbl->setMinimumWidth(90);
        spin = new QDoubleSpinBox;
        spin->setRange(minVal, maxVal);
        spin->setValue(defaultVal);
        spin->setDecimals(decimals);
        if (!suffix.isEmpty()) spin->setSuffix(suffix);
        row->addWidget(lbl);
        row->addWidget(spin);
        layout->addLayout(row);
    }

    // -----------------------------------------------------------------------
    // Helper: update the cover equation hint label
    // -----------------------------------------------------------------------
    void updateCoverEqLabel(double eps_r)
    {
        if (!m_lblCoverEq) return;
        if (eps_r > 1.0 + 1e-9)
            m_lblCoverEq->setText(
                QString("Eq. (3.24) — dielectric  εr = %1").arg(eps_r, 0, 'f', 2));
        else
            m_lblCoverEq->setText("Eq. (3.22) — air gap");
    }

    // -----------------------------------------------------------------------
    // Build SectionItemData from current widget state and emit dataChanged
    // -----------------------------------------------------------------------
    void emitDataChanged()
    {
        if (m_blockSignals || m_currentIndex < 0) return;

        SectionItemData data;

        // Cavity
        data.depth_mm          = m_spinDepth->value();
        data.obs_position_mm   = m_spinObsPos->value();
        data.has_observation   = m_chkObservation->isChecked();

        // Width override: spinbox value 0.0 → sentinel -1.0 (use global a)
        data.section_width_a_mm =
            (m_spinWidthA->value() > 0.0) ? m_spinWidthA->value() : -1.0;

        // Aperture
        data.aperture_l_mm     = m_spinApL->value();
        data.aperture_w_mm     = m_spinApW->value();

        // Cover
        data.has_cover         = m_chkCover->isChecked();
        data.cover_gap_mm      = m_spinCoverGap->value();
        data.cover_eps_r       = m_spinCoverEpsR->value();  // ← Eq.(3.24) binding

        // Dielectric
        data.has_dielectric    = m_chkDielectric->isChecked();
        data.dielectric_h_mm   = m_spinDielH->value();
        data.dielectric_er     = m_spinDielEr->value();

        emit dataChanged(m_currentIndex, data);
    }

    // -----------------------------------------------------------------------
    // Widget members
    // -----------------------------------------------------------------------

    QLabel* m_lblTitle = nullptr;

    // Cavity
    QDoubleSpinBox* m_spinDepth       = nullptr;
    QDoubleSpinBox* m_spinObsPos      = nullptr;   ///< min = 0.1 mm (strictly positive)
    QCheckBox*      m_chkObservation  = nullptr;

    // Width override
    QDoubleSpinBox* m_spinWidthA      = nullptr;   ///< 0 = use global a

    // Aperture
    QDoubleSpinBox* m_spinApL         = nullptr;
    QDoubleSpinBox* m_spinApW         = nullptr;

    // Cover
    QCheckBox*      m_chkCover        = nullptr;
    QDoubleSpinBox* m_spinCoverGap    = nullptr;
    QDoubleSpinBox* m_spinCoverEpsR   = nullptr;   ///< 1.0=Eq.3.22, >1.0=Eq.3.24
    QLabel*         m_lblCoverEq      = nullptr;   ///< Dynamic equation hint

    // Dielectric
    QCheckBox*      m_chkDielectric   = nullptr;
    QDoubleSpinBox* m_spinDielH       = nullptr;
    QDoubleSpinBox* m_spinDielEr      = nullptr;   ///< min = 1.0

    // State
    int  m_currentIndex = -1;
    bool m_blockSignals = false;
};

#endif // PROPERTYPANEL_H
