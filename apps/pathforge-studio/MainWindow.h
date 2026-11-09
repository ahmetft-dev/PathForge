#pragma once

#include <QMainWindow>
#include <QMap>
#include <QString>

class QComboBox;
class QDoubleSpinBox;
class QFormLayout;
class QLabel;
class QSpinBox;
class TrackViewport;

class MainWindow : public QMainWindow {
  Q_OBJECT
 public:
  MainWindow();

 private slots:
  void generateTrack();
  void presetChanged();

 private:
  QWidget* buildSidebar();
  QWidget* buildDiagnostics();
  void setDiagnostic(const QString& key, const QString& value);

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
};
