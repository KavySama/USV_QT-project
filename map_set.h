#ifndef MAP_SET_H
#define MAP_SET_H

#include <QWidget>
#include <QVector>

namespace Ui {
class map_set;
}

class map_set : public QWidget
{
    Q_OBJECT

public:
    explicit map_set(QWidget *parent = nullptr);
    ~map_set();
signals:
    void rov_location(double,double);
public slots:

private slots:
    void on_button_save_clicked();
    void on_button_clear_clicked();
    void doProcessRov_ll(QString rov_lat,QString rov_lon);


private:
    Ui::map_set *ui;
    QVector<double> llToFloat(QString lat,QString lon);
    QVector<double> ned_calculate(double latitude,double longitude);

};

#endif // MAP_SET_H
