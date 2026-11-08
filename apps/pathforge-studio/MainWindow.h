#pragma once

#include <QMainWindow>

class QComboBox;
class QDoubleSpinBox;
class QLabel;
class QSpinBox;
class TrackViewport;

class MainWindow : public QMainWindow {
  Q_OBJECT
 public:
  MainWindow();

 private slots:
  void generateTrack();

 private:
  QWidget* buildSidebar();
  QWidget* buildDiagnostics();
  void setDiagnostic(const QString& key, const QString& value);

  TrackViewport* viewport_ = nullptr;
  QComboBox* preset_ = nullptr;
  QDoubleSpinBox* radius_ = nullptr;
  QSpinBox* points_ = nullptr;
  QLabel* cursorLabel_ = nullptr;
  QMap<QString, QLabel*> diag_;
};
