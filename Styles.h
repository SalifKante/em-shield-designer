#pragma once

// ============================================================================
//  Styles.h — Centralised QSS stylesheets and button factory
//  Created in Task 1.2 (move toolbar buttons to left panel).
//
//  Why a header rather than an external .qss file:
//    * The codebase already builds all QSS at runtime by interpolating colour
//      constants from the CBStyle namespace (see CircuitBuilderWindow.h
//      applyGlobalStyle(), spinSS(), makeToolBtn() etc.).  An external .qss
//      file cannot reference CBStyle colours without duplication.
//    * Header-based styles ship inside the .exe — zero deployment files,
//      zero runtime file-not-found risk, zero windeployqt configuration.
//    * Reusable from both CircuitBuilderWindow (Task 1.2) and the future
//      Quick-Simulation redesign (Task 2.1).
//
//  All public identifiers live in namespace EMStyle to avoid clashes with the
//  existing CBStyle (colour tokens) and ElementIcon (paint helpers) namespaces.
// ============================================================================

#include <QColor>
#include <QHBoxLayout>
#include <QIcon>
#include <QLabel>
#include <QPainter>
#include <QPaintEvent>
#include <QPixmap>
#include <QPushButton>
#include <QSize>
#include <QString>
#include <QToolButton>
#include <QWidget>
#include <functional>

// CBStyle is defined in CircuitBuilderWindow.h. This header MUST be included
// from a translation unit where CBStyle's inline constants are already
// visible. CircuitBuilderWindow.h enforces this by defining the macro
// CBSTYLE_DECLARED immediately after the CBStyle namespace and before its
// `#include "Styles.h"` directive. The static_assert below catches accidental
// stand-alone inclusion at compile time.
#ifndef CBSTYLE_DECLARED
#  error "Styles.h requires CBStyle to be visible. Include CircuitBuilderWindow.h instead, which sets CBSTYLE_DECLARED before including this header."
#endif

namespace EMStyle {

// ----------------------------------------------------------------------------
//  Colour-string helpers
// ----------------------------------------------------------------------------

inline QString rgb(const QColor& c) {
    return QString("rgb(%1,%2,%3)").arg(c.red()).arg(c.green()).arg(c.blue());
}

inline QString rgba(const QColor& c, int alpha) {
    return QString("rgba(%1,%2,%3,%4)")
    .arg(c.red()).arg(c.green()).arg(c.blue()).arg(alpha);
}

// ----------------------------------------------------------------------------
//  Element-type accent colour
//
//  Encapsulates the colour-coding convention already used throughout the
//  CircuitBuilderWindow code (Source = orange, Aperture/AP+Cover = accent
//  blue, Cavity/Diel.Cav = green, Obs.Pt = red).  A single source of truth
//  prevents drift between the canvas element borders, the palette buttons,
//  and the property-panel form fields.
// ----------------------------------------------------------------------------

enum class AccentRole {
    Source,
    Aperture,
    Cavity,
    Observation,
    Neutral,
    Primary,    // green — used for the Compute button
    Danger      // red   — used for the Delete button
};

inline QColor accentFor(AccentRole r) {
    switch (r) {
    case AccentRole::Source:      return CBStyle::ORANGE;
    case AccentRole::Aperture:    return CBStyle::ACCENT;
    case AccentRole::Cavity:      return CBStyle::GREEN;
    case AccentRole::Observation: return CBStyle::RED;
    case AccentRole::Primary:     return CBStyle::GREEN;
    case AccentRole::Danger:      return CBStyle::RED;
    case AccentRole::Neutral:
    default:                      return CBStyle::TEXT_MUTED;
    }
}

// ----------------------------------------------------------------------------
//  Section header label QSS — "ELEMENTS", "ACTIONS" etc.
// ----------------------------------------------------------------------------

inline QString sectionHeaderQSS() {
    return QString(
               "QLabel{"
               "color:%1;"
               "font-family:'Courier New',monospace;"
               "font-size:10px;"
               "font-weight:bold;"
               "letter-spacing:2px;"
               "padding:6px 0px 4px 0px;"
               "border-top:1px solid %2;"
               "}"
               ).arg(rgb(CBStyle::TEXT_DIM)).arg(rgb(CBStyle::BORDER_LT));
}

// First section header (no top border, since it follows the brand strip).
inline QString sectionHeaderFirstQSS() {
    return QString(
               "QLabel{"
               "color:%1;"
               "font-family:'Courier New',monospace;"
               "font-size:10px;"
               "font-weight:bold;"
               "letter-spacing:2px;"
               "padding:6px 0px 4px 0px;"
               "}"
               ).arg(rgb(CBStyle::TEXT_DIM));
}

// ----------------------------------------------------------------------------
//  Brand strip QSS — relocated from the deleted toolbar to the top of the
//  left panel.  Renders the EM<Shield><Builder> wordmark.
// ----------------------------------------------------------------------------

inline QString brandStripQSS() {
    return QString(
               "QLabel{"
               "background:%1;"
               "color:%2;"
               "font-family:'Courier New',monospace;"
               "font-size:13px;"
               "letter-spacing:1px;"
               "padding:10px 8px;"
               "border-bottom:1px solid %3;"
               "}"
               ).arg(rgb(CBStyle::SURFACE2))
        .arg(rgb(CBStyle::TEXT))
        .arg(rgb(CBStyle::BORDER));
}

inline QString brandStripText() {
    return QString(
               "&nbsp;EM<span style='color:%1'>Shield</span>"
               "<b style='color:%2'>Builder</b>"
               ).arg(rgb(CBStyle::TEXT_MUTED)).arg(rgb(CBStyle::ACCENT));
}

// ----------------------------------------------------------------------------
//  Element button QSS — horizontal icon-on-left + text tile.
//
//  Visual specification:
//    * Full panel width, fixed 30 px height
//    * Left edge: 4 px coloured stripe matching the element's accent role
//    * Idle:      surface background, muted text
//    * Hover:     surface2 background, accent border 1 px
//    * Pressed:   accent-tinted background, accent border 1.5 px
//    * Disabled:  reduced opacity, no border
//    * Focus:     same as hover (avoids double-outline noise)
//
//  The accent stripe is implemented via border-left so the same QSS rule
//  works at any panel width without QPainter overrides.
// ----------------------------------------------------------------------------

inline QString elementButtonQSS(const QColor& accent) {
    return QString(
               "QPushButton{"
               "text-align:left;"
               "padding:0px 8px 0px 12px;"
               "background:%1;"
               "color:%2;"
               "border:1px solid %3;"
               "border-left:4px solid %4;"
               "border-radius:4px;"
               "font-family:'Courier New',monospace;"
               "font-size:11px;"
               "font-weight:500;"
               "}"
               "QPushButton:hover{"
               "background:%5;"
               "color:%4;"
               "border:1px solid %4;"
               "border-left:4px solid %4;"
               "}"
               "QPushButton:pressed{"
               "background:%6;"
               "color:%4;"
               "border:1.5px solid %4;"
               "border-left:4px solid %4;"
               "}"
               "QPushButton:disabled{"
               "background:%1;"
               "color:%7;"
               "border:1px solid %3;"
               "border-left:4px solid %3;"
               "}"
               "QPushButton:focus{"
               "outline:none;"
               "}"
               ).arg(rgb(CBStyle::SURFACE))    // %1 normal background
        .arg(rgb(CBStyle::TEXT))       // %2 normal text
        .arg(rgb(CBStyle::BORDER_LT))  // %3 normal border
        .arg(rgb(accent))              // %4 accent (stripe + hover/pressed border)
        .arg(rgb(CBStyle::SURFACE2))   // %5 hover background
        .arg(rgba(accent, 35))         // %6 pressed background tint
        .arg(rgb(CBStyle::TEXT_DIM));  // %7 disabled text
}

// ----------------------------------------------------------------------------
//  Action button QSS — Arrange, Delete, Clear, Export CSV.
//
//  Same anatomy as the element button but without the accent stripe.
//  The accent colour is used only for hover/pressed feedback.
// ----------------------------------------------------------------------------

inline QString actionButtonQSS(const QColor& accent) {
    return QString(
               "QPushButton{"
               "text-align:center;"
               "padding:0px 12px;"
               "background:%1;"
               "color:%2;"
               "border:1px solid %3;"
               "border-radius:4px;"
               "font-family:'Courier New',monospace;"
               "font-size:11px;"
               "}"
               "QPushButton:hover{"
               "background:%4;"
               "color:%5;"
               "border:1px solid %5;"
               "}"
               "QPushButton:pressed{"
               "background:%6;"
               "color:%5;"
               "border:1.5px solid %5;"
               "}"
               "QPushButton:disabled{"
               "color:%7;"
               "border:1px solid %3;"
               "}"
               "QPushButton:focus{"
               "outline:none;"
               "}"
               ).arg(rgb(CBStyle::SURFACE))    // %1
        .arg(rgb(CBStyle::TEXT))       // %2
        .arg(rgb(CBStyle::BORDER_LT))  // %3
        .arg(rgb(CBStyle::SURFACE2))   // %4 hover bg
        .arg(rgb(accent))              // %5 hover/pressed border + text
        .arg(rgba(accent, 30))         // %6 pressed bg
        .arg(rgb(CBStyle::TEXT_DIM));  // %7 disabled
}

// ----------------------------------------------------------------------------
//  Primary button QSS — Compute.
//
//  Solid filled accent background, white bold text, taller than other buttons
//  to mark it as the dominant call to action in the panel.
// ----------------------------------------------------------------------------

inline QString primaryButtonQSS(const QColor& accent) {
    const QColor accentDark = accent.darker(118);
    const QColor accentLight = accent.lighter(108);
    return QString(
               "QPushButton{"
               "background:%1;"
               "color:white;"
               "border:1px solid %2;"
               "border-radius:5px;"
               "font-family:'Courier New',monospace;"
               "font-size:12px;"
               "font-weight:bold;"
               "letter-spacing:1px;"
               "padding:0px 12px;"
               "}"
               "QPushButton:hover{"
               "background:%3;"
               "border:1px solid %2;"
               "}"
               "QPushButton:pressed{"
               "background:%2;"
               "}"
               "QPushButton:disabled{"
               "background:%4;"
               "color:%5;"
               "border:1px solid %6;"
               "}"
               "QPushButton:focus{"
               "outline:none;"
               "}"
               ).arg(rgb(accent))             // %1 normal background
        .arg(rgb(accentDark))         // %2 normal border / pressed bg
        .arg(rgb(accentLight))        // %3 hover bg
        .arg(rgb(CBStyle::SURFACE))   // %4 disabled bg
        .arg(rgb(CBStyle::TEXT_DIM))  // %5 disabled text
        .arg(rgb(CBStyle::BORDER));   // %6 disabled border
}

// ----------------------------------------------------------------------------
//  Horizontal divider QSS — separates panel sections.
// ----------------------------------------------------------------------------

inline QString dividerQSS() {
    return QString(
               "QFrame{"
               "background:%1;"
               "max-height:1px;"
               "min-height:1px;"
               "margin:4px 0px;"
               "border:none;"
               "}"
               ).arg(rgb(CBStyle::BORDER_LT));
}

// ----------------------------------------------------------------------------
//  Icon pixmap factory — renders a 16×16 element icon by wrapping the
//  existing ElementIcon::draw* QPainter helpers.
//
//  The painter functions live in CircuitBuilderWindow.h's ElementIcon
//  namespace; the caller passes one of them in.
// ----------------------------------------------------------------------------

using IconPainter = std::function<void(QPainter*, QRectF, bool)>;

inline QPixmap renderIcon16(IconPainter painter) {
    QPixmap pm(16, 16);
    pm.fill(Qt::transparent);
    QPainter p(&pm);
    p.setRenderHint(QPainter::Antialiasing, true);
    p.setRenderHint(QPainter::SmoothPixmapTransform, true);
    painter(&p, QRectF(0, 0, 16, 16), false);
    p.end();
    return pm;
}

} // namespace EMStyle
