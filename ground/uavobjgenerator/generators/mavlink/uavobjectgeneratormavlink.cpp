/**
 ******************************************************************************
 *
 * @file       UAVObjectGeneratorMAVLink.cpp
 * @author     The OpenPilot Team, http://www.openpilot.org Copyright (C) 2010.
 * @brief      produce flight code for uavobjects
 *
 * @see        The GNU Public License (GPL) Version 3
 *
 *****************************************************************************/
/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#include <QDebug>
#include "uavobjectgeneratormavlink.h"

using namespace std;

#define MAVLINK_FILE_NAME "mavlinkadapter"

bool UAVObjectGeneratorMAVLink::generate(UAVObjectParser* parser,QString templatepath,QString outputpath) {

    fieldTypeStrC << "int8_t" << "int16_t" << "int32_t" <<"uint8_t"
            <<"uint16_t" << "uint32_t" << "float" << "uint8_t";

    globalParamIndex = 0;

    QString flightObjHeaders, flightObjAdapterHeaders, objFileNames;//, objNames;

    // Index function calls
    QString getParamIndexByNameLines, getParamByIndexLines, setParamByIndexLines, getParamNameByIndexLines,
            getParamCountLines, getParamByNameLines, setParamByNameLines, writeParametersToStorageLines,
            readParametersFromStorage;

    flightCodePath = QDir( templatepath + QString("flight/UAVObjects"));
    flightOutputPath = QDir( outputpath + QString("flight") );
    flightOutputPath.mkpath(flightOutputPath.absolutePath());

    flightListIncludeTemplate = readFile( flightCodePath.absoluteFilePath("inc/uavobjectmavlinksettingslisttemplate.h") );
    flightListCodeTemplate = readFile( flightCodePath.absoluteFilePath("uavobjectmavlinksettingslisttemplate.c") );
    flightIncludeTemplate = readFile( flightCodePath.absoluteFilePath("inc/uavobjectmavlinktemplate.h") );
    flightCodeTemplate = readFile( flightCodePath.absoluteFilePath("uavobjectmavlinktemplate.c") );
    flightMakeTemplate = readFile( flightCodePath.absoluteFilePath("MAVLinkMakefiletemplate.inc") );

    if ( flightListCodeTemplate.isNull() || flightListIncludeTemplate.isNull() || flightIncludeTemplate.isNull()) {
            cerr << "Error: Could not open flight template files." << endl;
            return false;
        }

    for (int objidx = 0; objidx < parser->getNumObjects(); ++objidx) {
        ObjectInfo* info=parser->getObjectByIndex(objidx);
        if (info->isSettings)
        {
            bool result = process_object(info);
            if (!result) return false;

            flightObjHeaders.append(QString("#include \"%1.h\"\n").arg(info->namelc));
            flightObjAdapterHeaders.append(QString("#include \"%1.h\"\n").arg(info->namelc + MAVLINK_FILE_NAME));
            getParamIndexByNameLines.append(QString("\t#ifdef UAVOBJMAVLINKADAPTER_ENABLE_%2mavlinkadapter\n\tret = get%1ParamIndexByName(name); \n\tif (ret != -1) return ret;\n\t#endif\n").arg(info->name).arg(info->namelc));
            getParamByIndexLines.append(QString("\t#ifdef UAVOBJMAVLINKADAPTER_ENABLE_%2mavlinkadapter\n\tret = get%1ParamByIndex(index, param);\n\tif (ret == MAVLINK_RET_VAL_PARAM_SUCCESS) return ret;\n\t#endif\n").arg(info->name).arg(info->namelc));
            // .append(QString("").arg(info->name));
            setParamByIndexLines.append(QString("\t#ifdef UAVOBJMAVLINKADAPTER_ENABLE_%2mavlinkadapter\n\tret = set%1ParamByIndex(index, param);\n\tif (ret == MAVLINK_RET_VAL_PARAM_SUCCESS) return ret;\n\t#endif\n").arg(info->name).arg(info->namelc));
            getParamNameByIndexLines.append(QString("\t#ifdef UAVOBJMAVLINKADAPTER_ENABLE_%2mavlinkadapter\n\tret = (char*)get%1ParamNameByIndex(index);\n\tif (ret != 0) return ret;\n\t#endif\n").arg(info->name).arg(info->namelc));
            getParamCountLines.append(QString("\t#ifdef UAVOBJMAVLINKADAPTER_ENABLE_%2mavlinkadapter\n\tcount += get%1ParamCount();\n#endif\n").arg(info->name).arg(info->namelc));
            getParamByNameLines.append(QString("\t#ifdef UAVOBJMAVLINKADAPTER_ENABLE_%2mavlinkadapter\n\tret = get%1ParamByIndex(index, param);\n\tif (ret == MAVLINK_RET_VAL_PARAM_SUCCESS) return ret; // Else continue with other sub-sections\n\t#endif\n").arg(info->name).arg(info->namelc));
            setParamByNameLines.append(QString("\t#ifdef UAVOBJMAVLINKADAPTER_ENABLE_%2mavlinkadapter\n\tret = set%1ParamByIndex(index, param);\n\tif (ret == MAVLINK_RET_VAL_PARAM_SUCCESS) return ret; // Else continue with other sub-sections\n\t#endif\n").arg(info->name).arg(info->namelc));
            writeParametersToStorageLines.append(QString("\t#ifdef UAVOBJMAVLINKADAPTER_ENABLE_%2mavlinkadapter\n\thandle = %1Handle();\n\tres = UAVObjSave(handle, 0);\n\tif (res != 0) success = MAVLINK_RET_VAL_PARAM_WRITE_ERROR;\n\t#endif\n").arg(info->name).arg(info->namelc));
            readParametersFromStorage.append(QString("\t#ifdef UAVOBJMAVLINKADAPTER_ENABLE_%2mavlinkadapter\n\thandle = %1Handle();\n\tres = UAVObjLoad(handle, 0);\n\tif (res != 0) success = MAVLINK_RET_VAL_PARAM_READ_ERROR;\n\t#endif\n").arg(info->name).arg(info->namelc));
            objFileNames.append(" " + info->namelc + MAVLINK_FILE_NAME);
        }

    }

    // Write main adapter code file
    flightListCodeTemplate.replace( QString("$(SETTINGSHEADERS)"), flightObjHeaders);
    flightListCodeTemplate.replace( QString("$(ADAPTERHEADERS)"), flightObjAdapterHeaders);

    flightListCodeTemplate.replace( QString("$(GETPARAMINDEXBYNAMELINES)"), getParamIndexByNameLines);
    flightListCodeTemplate.replace( QString("$(GETPARAMBYINDEXLINES)"), getParamByIndexLines);
    flightListCodeTemplate.replace( QString("$(SETPARAMBYINDEXLINES)"), setParamByIndexLines);
    flightListCodeTemplate.replace( QString("$(GETPARAMNAMEBYINDEXLINES)"), getParamNameByIndexLines);
    flightListCodeTemplate.replace( QString("$(GETPARAMCOUNTLINES)"), getParamCountLines);
    flightListCodeTemplate.replace( QString("$(GETPARAMBYNAMELINES)"), getParamByNameLines);
    flightListCodeTemplate.replace( QString("$(SETPARAMBYNAMELINES)"), setParamByNameLines);
    flightListCodeTemplate.replace( QString("$(WRITEPARAMETERSTOSTORAGELINES)"), writeParametersToStorageLines);
    flightListCodeTemplate.replace( QString("$(READPARAMETERSFROMSTORAGELINES)"), readParametersFromStorage);

    bool res = writeFileIfDiffrent( flightOutputPath.absolutePath() + "/uavobjectmavlinksettings.c",
                                    flightListCodeTemplate );
    if (!res) {
        cout << "Error: Could not write flight adapter code file" << endl;
        return false;
    }

    writeFileIfDiffrent( flightOutputPath.absolutePath() + "/uavobjectmavlinksettings.h",
                         flightListIncludeTemplate );
    if (!res) {
        cout << "Error: Could not write flight adapter header file" << endl;
        return false;
    }

    // Write the flight object Makefile
    flightMakeTemplate.replace( QString("$(UAVOBJFILENAMES)"), objFileNames);
    //flightMakeTemplate.replace( QString("$(UAVOBJNAMES)"), objNames);
    res = writeFileIfDiffrent( flightOutputPath.absolutePath() + "/MAVLinkMakefile.inc",
                     flightMakeTemplate );
    if (!res) {
        cout << "Error: Could not write flight Makefile" << endl;
        return false;
    }

    return true; // if we come here everything should be fine
}


QString compactName(const QString& input)
{
    QString name;

    name = input.toLower();

    const int sizeLimit = 1;

    name.replace("settings", "");

    // Replace known words with abbreviations
    if (name.length() > sizeLimit)
    {
        name.replace("gyroscope", "gyro");
        name.replace("accelerometer", "acc");
        name.replace("accel", "acc");
        name.replace("magnetometer", "mag");
        name.replace("distance", "dist");
        name.replace("ailerons", "ail");
        name.replace("altitude", "alt");
        name.replace("waypoint", "wp");
        name.replace("throttle", "thr");
        name.replace("elevator", "elev");
        name.replace("rudder", "rud");
        name.replace("error", "err");
        name.replace("version", "ver");
        name.replace("message", "msg");
        name.replace("count", "cnt");
        name.replace("value", "val");
        name.replace("source", "src");
        name.replace("index", "idx");
        name.replace("type", "typ");
        name.replace("mode", "mod");
        name.replace("battery", "bat");
        name.replace("voltage", "V");
        name.replace("sensor", "sens");
        name.replace("flight", "");
        name.replace("threshold", "thre");
        name.replace("calibrations", "cal");
        name.replace("calibration", "cal");
        name.replace("capacity", "cap");
        name.replace("location", "loc");
        name.replace("attitude", "att");
        name.replace("camera", "cam");
        name.replace("guidance", "guid");
        name.replace("stabilization", "stab");
        name.replace("vector", "vect");
        name.replace("vertical", "vert");
        name.replace("horizontal", "horz");
        name.replace("rotation", "rot");
        name.replace("mixer", "mix");
        name.replace("maximum", "max");
        name.replace("manual", "man");
        name.replace("control", "ctrl");
        name.replace("channel", "ch");
        name.replace("roll", "r");
        name.replace("pitch", "p");
        name.replace("yaw", "y");
        name.replace("thrust", "t");
        name.replace("curve", "crv");
        name.replace("actuator", "act");
        name.replace("rate", "rat");
        name.replace("optional", "opt");
        name.replace("module", "mod");
        name.replace("input", "in");
        name.replace("range", "R");
        name.replace("update", "upd");
        name.replace("limit", "lim");
        name.replace("accessory", "acces");
        name.replace("position", "pos");
        name.replace("neutral", "neut");
        name.replace("latitude", "lat");
        name.replace("longitude", "lon");
        name.replace("leveling", "lev");
        name.replace("output", "out");
        name.replace("armed", "arm");
        name.replace("timeout", "tout");
        name.replace("integral", "int");
        name.replace("warning", "warn");
        name.replace("alarm", "alrm");
        name.replace("factor", "fact");
        name.replace("corrected", "corr");
        name.replace("current", "curr");
        name.replace("config", "cfg");
        name.replace("system", "sys");

        // Catch a few particular inputs
        name.replace("mix_mix", "mix_m");
        name.replace("ahrscal", "cal");
        name.replace("guid", "g");
        name.replace("tempcompfact", "tcompf");

    }

//    // Check if sub-part is still exceeding N chars
//    if (name.length() > sizeLimit)
//    {
//        name.replace("a", "");
//        name.replace("e", "");
//        name.replace("i", "");
//        name.replace("o", "");
//        name.replace("u", "");
//    }

    return name;
}


/**
 * Generate the Flight object files
**/
bool UAVObjectGeneratorMAVLink::process_object(ObjectInfo* info)
{
    if (info == NULL)
        return false;

    // Gracefully ignore objects that are not settings
    if (!info->isSettings)
        return true;

    // Prepare output strings
    QString outInclude = flightIncludeTemplate;
    QString outCode = flightCodeTemplate;

    // Replace common tags
    replaceCommonTags(outInclude, info);
    replaceCommonTags(outCode, info);

    // Replace the $(DATAFIELDS) tag
    QString type;
    QString fields;

    QString fieldBaseName;

    if (info->compactname.length() != 0) {
        fieldBaseName = info->compactname+ "_";
    } else {
        fieldBaseName = info->name.toLower()+ "_";
    }

    unsigned int localParamIndex = 0;
    unsigned int localParamOffset = globalParamIndex;
    unsigned int localParamCount = 0;

    QString getParamNameByIndexLines, getParamByIndexLines, setParamByIndexLines;

    for (int n = 0; n < info->fields.length(); ++n)
    {
        // Determine type
        type = fieldTypeStrC[info->fields[n]->type];
        QString mavlinkType;
        QString structAccess;

        if (type == "float" || type == "double")
        {
            mavlinkType = "MAVLINK_TYPE_FLOAT";
            structAccess = "param->param_float";
        }
        else if (type == "uint8_t" || type == "uint16_t" || type == "uint32_t")
        {
            mavlinkType = "MAVLINK_TYPE_UINT32_T";
            structAccess = "param->param_uint32";
        }
        else if (type == "int8_t" || type == "int16_t" || type == "int32_t")
        {
            mavlinkType = "MAVLINK_TYPE_INT32_T";
            structAccess = "param->param_int32";
        }
        else
        {
            mavlinkType = "MAVLINK_TYPE_FLOAT";
            structAccess = "param->param_float";
        }

        QString mavlinkFieldName = QString(info->fields[n]->name);

        if (info->fields[n]->compactname.length() != 0) {
            // Take the user-supplied input if any
            mavlinkFieldName = info->fields[n]->compactname;
        }

        for (int i = 0; i < info->fields[n]->numElements; ++i)
        {
            localParamCount++;
            QString fieldName = fieldBaseName;
            QString objAccess;
            if (info->fields[n]->numElements > 1)
            {
                objAccess = QString("%1[%2]").arg(info->fields[n]->name).arg(i);
                fieldName = fieldName.toUpper();
                fieldName += QString("%1_%2").arg(mavlinkFieldName).arg(QString(info->fields[n]->elementNames[i]));
            }
            else
            {
                fieldName = fieldName.toUpper();
                fieldName += mavlinkFieldName;
                objAccess = info->fields[n]->name;
            }

            // Compress only if not user-supplied
            if (info->fields[n]->compactname.length() == 0) {
                fieldName = compactName(fieldName);
            }

            int fieldNameLengthMaximum = 18;


            // Check if the elementnames part should be compressed
            if (fieldName.length() > fieldNameLengthMaximum && info->fields[n]->compactname.length() != 0 && info->fields[n]->numElements > 1)
            {
                QStringList parts = fieldName.split("_");
                parts.replace(2, compactName(parts.at(2)));
                fieldName = parts.join("_");
            }


            if (fieldName.length() <= fieldNameLengthMaximum)
            {
                //qDebug() << "\t" << fieldName << type;
            }
            else
            {
                cout << endl << "ERROR:\t\tThe field <" << fieldName.toStdString() << "> is " << fieldName.length() << " chars long, but the name limit is " << fieldNameLengthMaximum << " characters." << endl;
                cout << "\t\tPlease edit in the XML file " << info->filename.toStdString() << " the main <object> tag AND the field <" << info->fields[n]->name.toStdString() << ">" << endl;
                cout << "\t\tand add to both tags the attribute: compactname=\"<name>\", where <name> is not more than " << fieldNameLengthMaximum << " characters long." << endl << endl;
                return false;
            }

            getParamNameByIndexLines += QString("\tcase %1:\n\t\treturn \"%2\";\n\tbreak;\n").arg(globalParamIndex).arg(fieldName);
            getParamByIndexLines +=     QString("	\t\tcase %1:\n\t\t{\n\t\t%2 = settings.%3;\n\t\tparam->type = %4;\n\t\t}\n\t\tbreak;\n").arg(globalParamIndex).arg(structAccess).arg(objAccess).arg(mavlinkType);
            setParamByIndexLines +=     QString("\t\tcase %1:\n\t\t{\n\t\t\tif (param->type == %2)\n\t\t\t{\n\t\t\t\tsettings.%3 = %4;\n\t\t\t}\n\t\telse\n\t\t{\n\t\t\treturn MAVLINK_RET_VAL_PARAM_TYPE_MISMATCH;\n\t\t}\n\t\t}\n\t\tbreak;\n").arg(globalParamIndex).arg(mavlinkType).arg(objAccess).arg(structAccess);
            globalParamIndex++;
            localParamIndex++;
        }
    }

    outCode.replace(QString("$(GLOBALTOTALFIELDOFFSET)"), QString("%1").arg(0));//localParamOffset));
    //outInclude.replace(QString("$(DATAFIELDS)"), fields);
    outCode.replace(QString("$(LOCALFIELDCOUNT)"), QString("%1").arg(localParamIndex));

    // Actual lines
    outCode.replace(QString("$(GETPARAMNAMEBYINDEXLINES)"), getParamNameByIndexLines);
    outCode.replace(QString("$(GETPARAMBYINDEXLINES)"), getParamByIndexLines);
    outCode.replace(QString("$(SETPARAMBYINDEXLINES)"), setParamByIndexLines);

    outCode.replace(QString("$(PARAMCOUNT)"), QString("%1").arg(localParamCount));

    // Write the flight code
    bool res = writeFileIfDiffrent( flightOutputPath.absolutePath() + "/" + info->namelc + MAVLINK_FILE_NAME + ".c", outCode );
    if (!res) {
        cout << "Error: Could not write flight code files" << endl;
        return false;
    }

    res = writeFileIfDiffrent( flightOutputPath.absolutePath() + "/" + info->namelc + MAVLINK_FILE_NAME + ".h", outInclude );

    if (!res) {
        cout << "Error: Could not write flight include files" << endl;
        return false;
    } else {
        //cout << "SUCCESS:\tWrote file: " << flightOutputPath.absolutePath().toStdString() + "/" + info->namelc.toStdString() + MAVLINK_FILE_NAME + ".h" << endl;
    }

    return true;
}


