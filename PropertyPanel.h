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

class PropertyPanel : public QWidget
{
    Q_OBJECT

public:
    explicit PropertyPanel(QWidget* parent = nullptr)
        : QWidget(parent)
    {
        setupUI();
        setEnabled(false);  // Disabled until a section is selected
    }

    // Load data from a section item
    void loadSection(int index, const SectionItemData& data) {
        m_blockSignals = true;
        m_currentIndex = index;

        m_lblTitle->setText(QString("Section %1 Properties").arg(index + 1));

        m_spinDepth->setValue(data.depth_mm);
        m_spinObsPos->setValue(data.obs_position_mm);
        m_chkObservation->setChecked(data.has_observation);
        m_spinObsPos->setEnabled(data.has_observation);

        m_spinApL->setValue(data.aperture_l_mm);
        m_spinApW->setValue(data.aperture_w_mm);

        m_chkCover->setChecked(data.has_cover);
        m_spinCoverGap->setValue(data.cover_gap_mm);
        m_spinCoverGap->setEnabled(data.has_cover);

        m_chkDielectric->setChecked(data.has_dielectric);
        m_spinDielH->setValue(data.dielectric_h_mm);
        m_spinDielEr->setValue(data.dielectric_er);
        m_spinDielH->setEnabled(data.has_dielectric);
        m_spinDielEr->setEnabled(data.has_dielectric);

        setEnabled(true);
        m_blockSignals = false;
    }

    void clearSelection() {
        m_currentIndex = -1;
        m_lblTitle->setText("No Section Selected");
        setEnabled(false);
    }

    int currentIndex() const { return m_currentIndex; }

signals:
    void dataChanged(int sectionIndex, const SectionItemData& data);

private:
    void setupUI() {
        QVBoxLayout* mainLayout = new QVBoxLayout(this);
        mainLayout->setContentsMargins(0, 0, 0, 0);
        mainLayout->setSpacing(8);

        // Title
        m_lblTitle = new QLabel("No Section Selected");
        m_lblTitle->setStyleSheet("font-weight: bold; font-size: 12px; color: #1e3a5f;");
        mainLayout->addWidget(m_lblTitle);

        // --- Cavity group ---
        QGroupBox* grpCavity = new QGroupBox("Cavity");
        QVBoxLayout* cavLayout = new QVBoxLayout;
        cavLayout->setSpacing(4);

        addSpinRow(cavLayout, "Depth:", m_spinDepth, 10.0, 2000.0, 300.0, " mm", 1);
        addSpinRow(cavLayout, "Obs position:", m_spinObsPos, 0.0, 2000.0, 150.0, " mm", 1);

        m_chkObservation = new QCheckBox("Has observation point");
        m_chkObservation->setChecked(true);
        cavLayout->addWidget(m_chkObservation);

        grpCavity->setLayout(cavLayout);
        mainLayout->addWidget(grpCavity);

        // --- Aperture group ---
        QGroupBox* grpAperture = new QGroupBox("Aperture");
        QVBoxLayout* apLayout = new QVBoxLayout;
        apLayout->setSpacing(4);

        addSpinRow(apLayout, "Width (l):", m_spinApL, 1.0, 500.0, 80.0, " mm", 1);
        addSpinRow(apLayout, "Height (w):", m_spinApW, 1.0, 500.0, 80.0, " mm", 1);

        grpAperture->setLayout(apLayout);
        mainLayout->addWidget(grpAperture);

        // --- Cover group ---
        QGroupBox* grpCover = new QGroupBox("Aperture Cover");
        QVBoxLayout* covLayout = new QVBoxLayout;
        covLayout->setSpacing(4);

        m_chkCover = new QCheckBox("Enable cover");
        covLayout->addWidget(m_chkCover);

        addSpinRow(covLayout, "Gap (τ):", m_spinCoverGap, 0.01, 50.0, 1.0, " mm", 2);
        m_spinCoverGap->setEnabled(false);

        grpCover->setLayout(covLayout);
        mainLayout->addWidget(grpCover);

        // --- Dielectric group ---
        QGroupBox* grpDiel = new QGroupBox("Dielectric Fill");
        QVBoxLayout* dielLayout = new QVBoxLayout;
        dielLayout->setSpacing(4);

        m_chkDielectric = new QCheckBox("Enable dielectric");
        dielLayout->addWidget(m_chkDielectric);

        addSpinRow(dielLayout, "Height (h):", m_spinDielH, 0.1, 500.0, 60.0, " mm", 1);
        addSpinRow(dielLayout, "εr:", m_spinDielEr, 1.0, 100.0, 4.4, "", 2);
        m_spinDielH->setEnabled(false);
        m_spinDielEr->setEnabled(false);

        grpDiel->setLayout(dielLayout);
        mainLayout->addWidget(grpDiel);

        mainLayout->addStretch();

        // --- Connect signals ---
        auto emitChange = [this]() { emitDataChanged(); };

        connect(m_spinDepth,  QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, emitChange);
        connect(m_spinObsPos, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, emitChange);
        connect(m_spinApL,    QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, emitChange);
        connect(m_spinApW,    QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, emitChange);
        connect(m_spinCoverGap,  QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, emitChange);
        connect(m_spinDielH,  QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, emitChange);
        connect(m_spinDielEr, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, emitChange);

        connect(m_chkObservation, &QCheckBox::toggled, this, [this](bool checked) {
            m_spinObsPos->setEnabled(checked);
            emitDataChanged();
        });
        connect(m_chkCover, &QCheckBox::toggled, this, [this](bool checked) {
            m_spinCoverGap->setEnabled(checked);
            emitDataChanged();
        });
        connect(m_chkDielectric, &QCheckBox::toggled, this, [this](bool checked) {
            m_spinDielH->setEnabled(checked);
            m_spinDielEr->setEnabled(checked);
            emitDataChanged();
        });
    }

    void addSpinRow(QVBoxLayout* layout, const QString& label,
                    QDoubleSpinBox*& spin, double min, double max,
                    double value, const QString& suffix, int decimals)
    {
        QHBoxLayout* row = new QHBoxLayout;
        QLabel* lbl = new QLabel(label);
        lbl->setMinimumWidth(90);
        spin = new QDoubleSpinBox;
        spin->setRange(min, max);
        spin->setValue(value);
        spin->setDecimals(decimals);
        if (!suffix.isEmpty()) spin->setSuffix(suffix);
        row->addWidget(lbl);
        row->addWidget(spin);
        layout->addLayout(row);
    }

    void emitDataChanged() {
        if (m_blockSignals || m_currentIndex < 0) return;

        SectionItemData data;
        data.depth_mm        = m_spinDepth->value();
        data.obs_position_mm = m_spinObsPos->value();
        data.has_observation = m_chkObservation->isChecked();
        data.aperture_l_mm   = m_spinApL->value();
        data.aperture_w_mm   = m_spinApW->value();
        data.has_cover       = m_chkCover->isChecked();
        data.cover_gap_mm    = m_spinCoverGap->value();
        data.has_dielectric  = m_chkDielectric->isChecked();
        data.dielectric_h_mm = m_spinDielH->value();
        data.dielectric_er   = m_spinDielEr->value();

        emit dataChanged(m_currentIndex, data);
    }

    // Widgets
    QLabel*         m_lblTitle;

    QDoubleSpinBox* m_spinDepth;
    QDoubleSpinBox* m_spinObsPos;
    QCheckBox*      m_chkObservation;

    QDoubleSpinBox* m_spinApL;
    QDoubleSpinBox* m_spinApW;

    QCheckBox*      m_chkCover;
    QDoubleSpinBox* m_spinCoverGap;

    QCheckBox*      m_chkDielectric;
    QDoubleSpinBox* m_spinDielH;
    QDoubleSpinBox* m_spinDielEr;

    int  m_currentIndex = -1;
    bool m_blockSignals = false;
};

#endif // PROPERTYPANEL_H
