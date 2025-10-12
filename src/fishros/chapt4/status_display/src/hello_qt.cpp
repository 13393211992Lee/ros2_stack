#include <QApplication>
#include  <QLabel>
#include <QString>

int main(int argc, char* argv[]){
    QApplication app(argc, argv);
    QLabel* label = new QLabel();
    QString message2 = QString::fromStdString("Hello, QT! My name is cici");
    label->setText(message2);
    label->show();
    app.exec(); //执行应用 阻塞代码
    return 0;
}