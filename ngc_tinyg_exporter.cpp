#include "ngc_exporter.hpp"
#include <boost/foreach.hpp>
#include <boost/algorithm/string.hpp>
#include <iostream>
using std::cerr;
using std::ios_base;
using std::left;

#include <iomanip>

#include <glibmm/miscutils.h>
using Glib::build_filename;

#include <boost/format.hpp>
using boost::format;

void NGC_Exporter::export_layer_tinyg(shared_ptr<Layer> layer, string of_name)
{
    string layername = layer->get_name();
    shared_ptr<RoutingMill> mill = layer->get_manufacturer();
    bool bSvgOnce = TRUE;
    bool bAutolevelNow;
    vector<shared_ptr<icoords> > toolpaths = layer->get_toolpaths();
    vector<unsigned int> bridges;
    vector<unsigned int>::const_iterator currentBridge;
    static const unsigned int repeatVar = ocodes.getUniqueCode();

    double xoffsetTot;
    double yoffsetTot;
    Tiling tiling( tileInfo, cfactor );

    tiling.setGCodeEnd(
         "G00 Z" + str( format("%.3f") % ( mill->zchange * cfactor ) ) +
        " ( retract )\n\n" + postamble + "M5 ( Spindle off. )\nM9 ( Coolant off. )\n"
        "M2 ( Program end. )\n\n" );

    tiling.initialXOffsetVar = globalVars.getUniqueCode();
    tiling.initialYOffsetVar = globalVars.getUniqueCode();

    // open output file
    std::ofstream of;
    of.open(of_name.c_str());

    // write header to .ngc file
    BOOST_FOREACH( string s, header )
    {
        of << "( " << s << " )\n";
    }

    if( ( bFrontAutoleveller && layername == "front" ) ||
        ( bBackAutoleveller && layername == "back" ) )
        bAutolevelNow = true;
    else
        bAutolevelNow = false;

    if( bAutolevelNow || ( tileInfo.enabled && tileInfo.software != CUSTOM ) )
        of << "( TinyG Gcode for " << getSoftwareString(tileInfo.software) << " )\n";
    else
        of << "( Software-independent Gcode, TinyG model )\n";

    of.setf(ios_base::fixed);      //write floating-point values in fixed-point notation
    of.precision(5);              //Set floating-point decimal precision

    of << "\n" << preamble;       //insert external preamble

    if (bMetricoutput)
    {
        of << "G94 ( Millimeters per minute feed rate. )\n"
           << "G21 ( Units == Millimeters. )\n\n";
    }
    else
    {
        of << "G94 ( Inches per minute feed rate. )\n"
           << "G20 ( Units == INCHES. )\n\n";
    }

    of << "G90 ( Absolute coordinates. )\n"
       << "G64 P" << g64 << " ( set maximum deviation from commanded toolpath )\n"
       << "F" << mill->feed * cfactor << " ( Feedrate. )\n\n";

    if( bAutolevelNow )
    {
        if( !leveller->prepareWorkarea( toolpaths ) )
        {
            std::cerr << "Required number of probe points (" << leveller->requiredProbePoints() <<
                      ") exceeds the maximum number (" << leveller->maxProbePoints() << "). "
                      "Reduce either al-x or al-y." << std::endl;
            exit(EXIT_FAILURE);
        }

        leveller->header( of );
    }

    of << "F" << mill->feed * cfactor << " ( Feedrate. )\n"
       << "M3 S" << left << mill->speed << " ( RPM spindle speed. )\n";

    tiling.header( of );

    //SVG EXPORTER
    if (bDoSVG)
    {
        //choose a color
        svgexpo->set_rand_color();
    }

    for( unsigned int i = 0; i < tileInfo.forYNum; i++ )
    {
        yoffsetTot = yoffset - i * tileInfo.boardHeight;

        for( unsigned int j = 0; j < tileInfo.forXNum; j++ )
        {
            xoffsetTot = xoffset - ( i % 2 ? tileInfo.forXNum - j - 1 : j ) * tileInfo.boardWidth;

            if( tileInfo.enabled && tileInfo.software == CUSTOM )
                of << "( Piece #" << j + 1 + i * tileInfo.forXNum << ", position [" << j << ";" << i << "] )\n\n";

            // contours
            BOOST_FOREACH( shared_ptr<icoords> path, toolpaths )
            {
                // retract, move to the starting point of the next contour
                of << "G00 Z" << mill->zsafe * cfactor << " ( retract )\n\n";
                of << "X" << ( path->begin()->first - xoffsetTot ) * cfactor << " Y"
                   << ( path->begin()->second - yoffsetTot ) * cfactor << " ( rapid move to begin. )\n";

                //SVG EXPORTER
                if (bDoSVG)
                {
                    svgexpo->move_to(path->begin()->first, path->begin()->second);
                    bSvgOnce = TRUE;
                }

                /* if we're cutting, perhaps do it in multiple steps, but do isolations just once.
                 * i know this is partially repetitive, but this way it's easier to read
                 */
                shared_ptr<Cutter> cutter = boost::dynamic_pointer_cast<Cutter>(mill);

                if (cutter && cutter->do_steps)
                {

                    //--------------------------------------------------------------------
                    //cutting (outline)

                    double z_step = cutter->stepsize;
                    double z = mill->zwork + z_step * abs(int(mill->zwork / z_step));

                    if( bBridges )
                        if( i == 0 && j == 0 )  //Compute the bridges only the 1st time
                            bridges = layer->get_bridges( path );

                    while (z >= mill->zwork)
                    {
                        of << "G01 Z" << z * cfactor << " F" << mill->vertfeed * cfactor << " ( plunge. )\n";
                        of << "F" << mill->feed * cfactor << "\n";

                        icoords::iterator iter = path->begin();
                        icoords::iterator last = path->end();      // initializing to quick & dirty sentinel value
                        icoords::iterator peek;

                        if (bBridges)
                            currentBridge = bridges.begin();

                        while (iter != path->end())
                        {
                            peek = iter + 1;

                            if (mill->optimise //Already optimised (also includes the bridge case)
                                    || last == path->end()  //First
                                    || peek == path->end()   //Last
                                    || !aligned(last, iter, peek) )      //Not aligned
                            {
                                of << "X" << ( iter->first - xoffsetTot ) * cfactor << " Y"
                                   << ( iter->second - yoffsetTot ) * cfactor << endl;
                                if (bDoSVG)
                                {
                                    if (bSvgOnce)
                                        svgexpo->line_to(iter->first, iter->second);
                                }

                                if( bBridges && currentBridge != bridges.end() )
                                {
                                    double bridges_depth = cutter->bridges_height >= 0 ?
                                        cutter->bridges_height : cutter->bridges_height * z / mill->zwork;

                                    if( *currentBridge == iter - path->begin() )
                                        of << "Z" << bridges_depth * cfactor << endl;
                                    else if( *currentBridge == last - path->begin() )
                                    {
                                        of << "Z" << z * cfactor << " F" << cutter->vertfeed * cfactor << endl;
                                        of << "F" << cutter->feed * cfactor;
                                        ++currentBridge;
                                    }
                                }
                            }

                            last = iter;
                            ++iter;
                        }
                        //SVG EXPORTER
                        if (bDoSVG)
                        {
                            svgexpo->close_path();
                            bSvgOnce = FALSE;
                        }
                        z -= z_step;
                    }
                }
                else
                {
                    //--------------------------------------------------------------------
                    // isolating (front/backside)
                    of << "F" << mill->vertfeed * cfactor << endl;

                    if( bAutolevelNow )
                    {
                        leveller->setLastChainPoint( icoordpair( ( path->begin()->first - xoffsetTot ) * cfactor,
                                                     ( path->begin()->second - yoffsetTot ) * cfactor ) );
                        of << leveller->g01Corrected( icoordpair( ( path->begin()->first - xoffsetTot ) * cfactor,
                                                      ( path->begin()->second - yoffsetTot ) * cfactor ) );
                    }
                    else
                        of << "G01 Z" << mill->zwork * cfactor << "\n";

                    of << "F" << mill->feed * cfactor << endl;

                    icoords::iterator iter = path->begin();
                    icoords::iterator last = path->end();      // initializing to quick & dirty sentinel value
                    icoords::iterator peek;

                    while (iter != path->end())
                    {
                        peek = iter + 1;
                        if (mill->optimise //When simplifypath is performed, no further optimisation is required
                                || last == path->end()  //First
                                || peek == path->end()   //Last
                                || !aligned(last, iter, peek) )      //Not aligned
                        {
                            /* no need to check for "they are on one axis but iter is outside of last and peek"
                             because that's impossible from how they are generated
                            TODO: TinyG won't do block commands, needs G00 on all move lines*/
                            if( bAutolevelNow )
                                of << leveller->addChainPoint( icoordpair( ( iter->first - xoffsetTot ) * cfactor,
                                                                           ( iter->second - yoffsetTot ) * cfactor ) );
                            else
                                of << "X" << ( iter->first - xoffsetTot ) * cfactor << " Y"
                                   << ( iter->second - yoffsetTot ) * cfactor << endl;
                            //SVG EXPORTER
                            if (bDoSVG)
                                if (bSvgOnce)
                                    svgexpo->line_to(iter->first, iter->second);
                        }

                        last = iter;
                        ++iter;
                    }
                    //SVG EXPORTER
                    if (bDoSVG)
                    {
                        svgexpo->close_path();
                        bSvgOnce = FALSE;
                    }
                }
            }
        }
    }

    tiling.footer( of );

    if( bAutolevelNow )
    {
        leveller->footer( of );
    }

    of.close();

    //SVG EXPORTER
    if (bDoSVG)
    {
        svgexpo->stroke();
    }
}
