#include "xml_read_and_write.h"


namespace XML
{
XMLWrite::XMLWrite(){
}

XMLWrite::~XMLWrite(){

}

void XMLWrite::write(const QString& file_path_name_ , const QString& topic_name_){

    QFile file(file_path_name_);
    if(!file.open(QFile::ReadWrite | QFile::Text)){
        QMessageBox::information(NULL, QString("launch file"), QString("open error!"));
    }
    QXmlStreamWriter xml(&file);

    xml.setAutoFormatting(true);
    xml.writeStartDocument();

    xml.writeStartElement("launch");
    
    // start node
    xml.writeStartElement("node");
    xml.writeAttribute("pkg", "calibration");
    xml.writeAttribute("type", "remove_ground");
    xml.writeAttribute("name", "remove_ground");

    // start param
    xml.writeStartElement("param");
    xml.writeAttribute("name","topic_name");
    xml.writeAttribute("type","string");
    xml.writeAttribute("value",topic_name_);

    // end param
    xml.writeEndElement();

    // end node
    xml.writeEndElement();

    // end launch
    xml.writeEndElement();

    xml.writeEndDocument();
}

}

        