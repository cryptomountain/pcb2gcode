/*
 * This file is part of pcb2gcode.
 * 
 * Copyright (C) 2015 Nicola Corna <nicola@corna.info>
 *
 * pcb2gcode is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * pcb2gcode is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with pcb2gcode.  If not, see <http://www.gnu.org/licenses/>.
 */
 
#include "tile.hpp"

#include <iostream>

#include <boost/format.hpp>
using boost::format;

Tiling::Tiling( TileInfo tileInfo, double cfactor ) :
    tileInfo( tileInfo ), cfactor( cfactor ),
    initialXOffsetVar(0), initialYOffsetVar(0)
{

}

void Tiling::header( std::ofstream &of )
{
    if( tileInfo.enabled )
    {
        switch( tileInfo.software )
        {
            case LINUXCNC:
                of << "\no" << tileInfo.tileVar << " sub ( Main subroutine )\n\n";
                break;
            
            case MACH3:
            case MACH4:
                tileSequence( of );
                of << gCodeEnd << "\nO" << tileInfo.tileVar << " ( Main subroutine )\n\n";
                break;
            case CHILI:
            case CUSTOM:
                break;
        
        }
    }
}

void Tiling::footer( std::ofstream &of )
{
    if( tileInfo.enabled )
    {
        switch( tileInfo.software )
        {
            case LINUXCNC:
                of << "\no" << tileInfo.tileVar << " endsub\n\n";
                tileSequence( of );
                of << gCodeEnd;
                break;
            
            case MACH3:
            case MACH4:
                of << "\nM99\n\n";
                break;
            case CHILI:
            case CUSTOM:
                break;
        }
    }
    
    if( !tileInfo.enabled || tileInfo.software == CUSTOM )
        of << gCodeEnd;
}

void Tiling::tileSequence( std::ofstream &of )
{
    const char *callSub[] = { "o%1$d call", "M98 P%1$d", "M98 P%1$d" };
    const char *setX0[] = { "G92 X[#5420-[%1$f]]", "G00 X%1$f\nG92 X0", "G00 X%1$f\nG92 X0" };
    const char *setY0[] = { "G92 Y[#5421-[%1$f]]", "G00 Y%1$f\nG92 Y0", "G00 Y%1$f\nG92 Y0" };

    for( unsigned int i = 0; i < tileInfo.tileY; i++ )
    {
        of << ( format( callSub[tileInfo.software] ) % tileInfo.tileVar ) << "\n";
        for( unsigned int j = 0; j < tileInfo.tileX - 1; j++ )
        {
            of << ( format( setX0[tileInfo.software] ) %
                  ( i % 2 == 0 ? tileInfo.boardWidth * cfactor : -tileInfo.boardWidth * cfactor ) ) << "\n";
            of << ( format( callSub[tileInfo.software] ) % tileInfo.tileVar ) << "\n";
        }
        if( i < tileInfo.tileY - 1 )
           of << str( format( setY0[tileInfo.software] ) % ( tileInfo.boardHeight * cfactor ) ) << "\n";
    }

    of << ( format( setY0[tileInfo.software] ) % ( -tileInfo.boardHeight * cfactor * ( tileInfo.tileY - 1 ) ) ) << "\n";
    if( tileInfo.tileY % 2 )
        of << ( format( setX0[tileInfo.software] ) % ( -tileInfo.boardWidth * cfactor * ( tileInfo.tileX - 1 ) ) ) << "\n";
}

Tiling::TileInfo Tiling::generateTileInfo( const boost::program_options::variables_map& options,
                                           uniqueCodes &ocodes, double boardHeight, double boardWidth )
{
    TileInfo tileInfo;
    
    tileInfo.enabled = options["tile-x"].as<int>() > 1 || options["tile-y"].as<int>() > 1;
    tileInfo.tileX = options["tile-x"].as<int>();
    tileInfo.tileY = options["tile-y"].as<int>();
    tileInfo.boardHeight = boardHeight;
    tileInfo.boardWidth = boardWidth;
    tileInfo.tileVar = ocodes.getUniqueCode();

    if( !options.count("software") ||
        boost::iequals( options["software"].as<string>(), "custom" ) )
        tileInfo.software = CUSTOM;
    else if( boost::iequals( options["software"].as<string>(), "linuxcnc" ) )
        tileInfo.software = LINUXCNC;
    else if ( boost::iequals( options["software"].as<string>(), "mach3" ) )
        tileInfo.software = MACH3;
    else if ( boost::iequals( options["software"].as<string>(), "mach4" ) )
        tileInfo.software = MACH4;
    else
    {
        tileInfo.software = CUSTOM;
        std::cerr << "Tiling: unknown software!\n";
    }

    if( tileInfo.software == CUSTOM )
    {
        tileInfo.forXNum = tileInfo.tileX;
        tileInfo.forYNum = tileInfo.tileY;
    }
    else
    {
        tileInfo.forXNum = 1;
        tileInfo.forYNum = 1;
    }
    
    return tileInfo;
}

