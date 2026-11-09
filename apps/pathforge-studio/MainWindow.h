#pragma once

#include <QMainWindow>
#include <QMap>
#include <QString>
#include <optional>
#include <pathforge/geometry/track_data.hpp>
#include <pathforge/geometry/validation.hpp>
#include <vector>

class QComboBox;
class QDoubleSpinBox;
class QFormLayout;
class QLabel;
class QListWidget;
class QSpinBox;
class TrackViewport;

class MainWindow : public QMainWindow {
  Q_OBJECT
 public:
  MainWindow();

  // File operations without dialogs (also used by tests). Both return an empty string on success and an error
  // message otherwise. A failed open leaves the current track, view and diagnostics untouched.
  QString openTrackFile(const QString& path);
  QString saveTrackFile(const QString& path, bool overwrite);
  const pathforge::geometry::TrackData& currentTrack() const { return current_; }
  const pathforge::geometry::ValidationReport& currentReport() const { return report_; }

 private slots:
  void generateTrack();
  void presetChanged();
  void openTrack();
  void saveTrack();
  void saveTrackAs();
  void issueSelected(int row);

 private:
  QWidget* buildSidebar();
  QWidget* buildDiagnostics();
  void setDiagnostic(const QString& key, const QString& value);
  // Validates and shows `data`. On failure (data cannot form a track) nothing changes and *error is set.
  bool display(const pathforge::geometry::TrackData& data, const QString& file,
               std::optional<double> referenceLength, QString* error);
  void updateTitle();

  TrackViewport* viewport_ = nullptr;
  QComboBox* preset_ = nullptr;
  QDoubleSpinBox* radius_ = nullptr;
  QDoubleSpinBox* length_ = nullptr;
  QDoubleSpinBox* width_ = nullptr;
  QDoubleSpinBox* amplitude_ = nullptr;
  QDoubleSpinBox* trackWidth_ = nullptr;
  QFormLayout* form_ = nullptr;
  QSpinBox* points_ = nullptr;
  QLabel* cursorLabel_ = nullptr;
  QMap<QString, QLabel*> diag_;
  QListWidget* issueList_ = nullptr;
  pathforge::geometry::TrackData current_;
  pathforge::geometry::ValidationReport report_;
  QString filePath_;
};
