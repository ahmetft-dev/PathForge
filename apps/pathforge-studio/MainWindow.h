#pragma once

#include <QMainWindow>
#include <QMap>
#include <QMessageBox>
#include <QString>
#include <cstdint>
#include <functional>
#include <tuple>

class EditorSession;
class QAction;
class QCheckBox;
class QComboBox;
class QDoubleSpinBox;
class QFormLayout;
class QGroupBox;
class QPushButton;
class QLabel;
class QLineEdit;
class QListWidget;
class QSpinBox;
class TrackViewport;

class MainWindow : public QMainWindow {
  Q_OBJECT
 public:
  MainWindow();

  // File operations without dialogs (used by the dialog slots and by tests). Both return an empty string on
  // success and an error message otherwise. A failed open leaves the current document untouched.
  QString openTrackFile(const QString& path);
  QString saveTrackFile(const QString& path, bool overwrite);

  // Unsaved-changes workflow. Returns true when it is safe to replace/close the document: it is unmodified,
  // was saved, or the user chose to discard. The prompt (Save / Discard / Cancel) can be replaced for tests.
  bool maybeSave();
  void setUnsavedPrompt(std::function<QMessageBox::StandardButton()> prompt) { prompt_ = std::move(prompt); }

  EditorSession* session() const { return session_; }
  TrackViewport* viewport() const { return viewport_; }
  QAction* addPointsAction() const { return addPointsAction_; }

 protected:
  void closeEvent(QCloseEvent* event) override;

 private slots:
  void generatePreset();
  void presetChanged();
  void newTrack();
  void openTrack();
  bool saveTrack();
  bool saveTrackAs();
  void deleteSelected();
  void issueSelected(int row);

  void onGeometryChanged();
  void onValidationChanged();
  void onDocumentReplaced();

 private:
  QWidget* buildSidebar();
  QWidget* buildDiagnostics();
  void setDiagnostic(const QString& key, const QString& value);
  void refreshViewport();
  void syncDocumentWidgets();
  void syncPointPanel();
  void applyDocumentSettingsFromWidgets();
  void updateTitle();

  using ViewKey = std::tuple<std::uint64_t, std::uint64_t, std::uint64_t>;
  ViewKey lastView_{~0ull, 0, 0};
  EditorSession* session_ = nullptr;
  TrackViewport* viewport_ = nullptr;

  // Presets
  QComboBox* preset_ = nullptr;
  QDoubleSpinBox* radius_ = nullptr;
  QDoubleSpinBox* length_ = nullptr;
  QDoubleSpinBox* width_ = nullptr;
  QDoubleSpinBox* amplitude_ = nullptr;
  QDoubleSpinBox* trackWidth_ = nullptr;
  QFormLayout* form_ = nullptr;
  QSpinBox* points_ = nullptr;

  // Document settings
  QLineEdit* nameEdit_ = nullptr;
  QCheckBox* closedCheck_ = nullptr;
  QComboBox* interpolation_ = nullptr;
  QDoubleSpinBox* spacing_ = nullptr;
  QDoubleSpinBox* defaultLeft_ = nullptr;
  QDoubleSpinBox* defaultRight_ = nullptr;

  // Selected-point properties
  QGroupBox* pointBox_ = nullptr;
  QLabel* pointTitle_ = nullptr;
  QDoubleSpinBox* pointX_ = nullptr;
  QDoubleSpinBox* pointY_ = nullptr;
  QDoubleSpinBox* pointLeft_ = nullptr;
  QDoubleSpinBox* pointRight_ = nullptr;
  QPushButton* applyAllWidths_ = nullptr;

  QAction* addPointsAction_ = nullptr;
  QAction* closedAction_ = nullptr;
  QAction* deleteAction_ = nullptr;

  QAction* undoAction_ = nullptr;
  QAction* redoAction_ = nullptr;
  std::function<QMessageBox::StandardButton()> prompt_;

  QLabel* cursorLabel_ = nullptr;
  QMap<QString, QLabel*> diag_;
  QListWidget* issueList_ = nullptr;
};
