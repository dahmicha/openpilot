/**
 ******************************************************************************
 *
 * @file       uavobjectgeneratorflight.h
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

#ifndef UAVOBJECTGENERATORMAVLINK_H
#define UAVOBJECTGENERATORMAVLINK_H

#include "../generator_common.h"

class UAVObjectGeneratorMAVLink
{
public:
    bool generate(UAVObjectParser* gen,QString templatepath,QString outputpath);
    QStringList fieldTypeStrC;
    QString flightIncludeTemplate, flightCodeTemplate, flightListCodeTemplate, flightListIncludeTemplate, flightMakeTemplate;
    QDir flightCodePath;
    QDir flightOutputPath;

protected:
    unsigned int globalParamIndex;

private:
    bool process_object(ObjectInfo* info);

};

#endif

