#ifndef OBJECT_H_
#define OBJECT_H_

#include <iostream>
#include <QString>
#include <QStringList>
#include <QMatrix4x4>
#include <QVector>

typedef std::vector<QMatrix4x4> Transforms;

struct EvalBppResults {
    qreal bpp_used_space_;  // qreal is double
    qreal bpp_packed_mass_;
    QVector3D bpp_com_;
};

struct Node
{
    QString name;
    QString status;
    QString ip;
    QString command;
    std::vector<QString> node_dependencies;
};

struct Module
{
    std::vector<Node> nodes;
    int status;
    QString name;
};

#endif /* OBJECT_H_ */
