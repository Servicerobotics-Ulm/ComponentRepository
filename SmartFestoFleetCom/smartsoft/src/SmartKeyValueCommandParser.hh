#ifndef _SmartKeyValueCommandParser_H_
#define _SmartKeyValueCommandParser_H_

#include <QtCore>

class SmartKeyValueCommandParser
{
public:
    static bool parse(QString message,QString& command,QMap<QString,QString>& keyValueMap)
    {
		//qDebug() << "Message:" << message;
		int pos=0;
		if(message.startsWith(QChar('"')))
		{
			pos=1;
			message.replace("\\","");
		}
		
        QRegExp commandExp("\\w+");
        int commandIndex = commandExp.indexIn(message,pos);
		//qDebug() << "commandIndex " << commandIndex;

        if(pos==commandIndex)
        {
            command = commandExp.cap();
            //qDebug() << "Command at " << commandIndex << " length:" << commandExp.matchedLength() << " command:" << command;

            if(message.contains(QChar(':')))
            {
                //search keys
                QRegExp keyExp("\\s*\\w+:");
                pos=commandIndex+command.length()+1;
                while((pos = keyExp.indexIn(message,pos)) != -1)
                {
                    //qDebug() << message.mid(pos);

                    QString key = keyExp.cap();
                    key.remove(":");
                    key = key.trimmed();

                    pos+=keyExp.matchedLength();

                    QString value;

                    if(message.at(pos) == QChar('"') )
                    {
                        QRegExp stringValueExp("\"[^\"]*\"");
                        int valueIndex=stringValueExp.indexIn(message,pos);
                        if(pos == valueIndex)
                        {
                            value=stringValueExp.cap();
                            value.remove("\"");
                            pos+=stringValueExp.matchedLength();
                        }
                    }
                    else
                    {
                        QRegExp valueExp("\\S+");
                        int valueIndex=valueExp.indexIn(message,pos);
                        if( pos == valueIndex)
                        {
                            //qDebug() << "Simple value : " <<  valueExp.cap();
                            value=valueExp.cap();
                            pos+=valueExp.matchedLength();
                        }
                    }


                    if(value.isEmpty())
                    {
                        qDebug() << "Error parsing value for key:" << key;
                        return false;
                    }

                    qDebug() << key << " : " << value;

                    keyValueMap[key]=value;
                }

                return true;
            }
            else
            {
                int pos=commandIndex+command.length()+1;

                QString value;
                int keyIndex=0;
                QString key;
                bool openingQuoteFound=false;
                while(pos<message.length())
                {
                    key = QString("arg%1").arg(keyIndex);
                    QChar ch = message.at(pos++);

                    if(QChar('"') == ch)
                    {
                        if(openingQuoteFound)
                        {
                            openingQuoteFound = false;
                            keyValueMap[key]=value;
                            ++keyIndex;
                            value.clear();
                        }
                        else
                        {
                            openingQuoteFound=true;
                        }
                    }
                    else if(ch.isSpace())
                    {
                        if(openingQuoteFound)
                        {
                            value+=ch;
                        }
                        else
                        {
                            if(false==value.isEmpty())
                            {
                                keyValueMap[key]=value;
                                ++keyIndex;
                                value.clear();
                            }
                        }
                    }
                    else
                    {
                        value+=ch;
                    }
                }

                if(false==value.isEmpty())
                {
                    keyValueMap[key]=value;
                }

                return true;
            }
        }

        return false;
    }
};

#endif //_SmartKeyValueCommandParser_H_
