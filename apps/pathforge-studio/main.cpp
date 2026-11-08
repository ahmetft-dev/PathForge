#include <QApplication>
#include <QCommandLineParser>
#include <QPalette>
#include <QStyleFactory>
#include <QTimer>

#include "MainWindow.h"

namespace {
void applyDarkTheme(QApplication& app) {
  app.setStyle(QStyleFactory::create("Fusion"));
  QPalette p;
  const QColor bg(0x1b, 0x1e, 0x23), base(0x14, 0x16, 0x1a), text(0xd8, 0xdc, 0xe2), accent(0x4f, 0xc3, 0xf7);
  p.setColor(QPalette::Window, bg);
  p.setColor(QPalette::WindowText, text);
  p.setColor(QPalette::Base, base);
  p.setColor(QPalette::AlternateBase, bg);
  p.setColor(QPalette::Text, text);
  p.setColor(QPalette::Button, QColor(0x26, 0x2a, 0x31));
  p.setColor(QPalette::ButtonText, text);
  p.setColor(QPalette::ToolTipBase, base);
  p.setColor(QPalette::ToolTipText, text);
  p.setColor(QPalette::Highlight, accent);
  p.setColor(QPalette::HighlightedText, QColor(0x10, 0x12, 0x15));
  p.setColor(QPalette::Disabled, QPalette::Text, QColor(0x70, 0x76, 0x80));
  p.setColor(QPalette::Disabled, QPalette::ButtonText, QColor(0x70, 0x76, 0x80));
  app.setPalette(p);
}
}  // namespace

int main(int argc, char** argv) {
  QApplication app(argc, argv);
  app.setApplicationName("PathForge Studio");
  applyDarkTheme(app);

  QCommandLineParser parser;
  parser.addHelpOption();
  QCommandLineOption grab("screenshot", "Render the main window to a PNG file and exit (smoke test).",
                          "file");
  parser.addOption(grab);
  parser.process(app);

  MainWindow w;
  w.show();
  if (parser.isSet(grab)) {
    const QString file = parser.value(grab);
    QTimer::singleShot(300, [&w, &app, file] {
      const bool ok = w.grab().save(file);
      app.exit(ok ? 0 : 1);
    });
  }
  return app.exec();
}
