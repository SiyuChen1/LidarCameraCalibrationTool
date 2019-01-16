#ifndef XML_READ_AND_WRITE_H
#define XML_READ_AND_WRITE_H

#include <QXmlStreamReader>
#include <QFile>
#include <QDebug>
#include <QXmlStreamWriter>
#include <QMessageBox>

namespace XML{
  class XMLWrite: public QObject
  {
      Q_OBJECT
  public:
      XMLWrite();
      ~XMLWrite();
      void write(const QString& file_path_name_,const QString& topic_name_);
  };
}

#endif