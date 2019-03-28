/*
===============================================================================

  FILE:  lasexample.cpp

  CONTENTS:

	This source code serves as an example how you can easily use LASlib to
	write your own processing tools or how to import from and export to the
	LAS format or - its compressed, but identical twin - the LAZ format.

  PROGRAMMERS:

	martin.isenburg@rapidlasso.com  -  http://rapidlasso.com

  COPYRIGHT:

	(c) 2007-2014, martin isenburg, rapidlasso - fast tools to catch reality

	This is free software; you can redistribute and/or modify it under the
	terms of the GNU Lesser General Licence as published by the Free Software
	Foundation. See the LICENSE.txt file for more information.

	This software is distributed WITHOUT ANY WARRANTY and without even the
	implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

  CHANGE HISTORY:

	3 January 2011 -- created while too homesick to go to Salzburg with Silke

===============================================================================
*/

#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <cstring>
#include <iostream>

#include "lasreader.hpp"
#include "laswriter.hpp"

static double taketime()
{
	return (double)(clock()) / CLOCKS_PER_SEC;
}

int main()
{
	double start_time = 0.0;
	int i = 0;

	LASreadOpener lasreadopener;
	LASwriteOpener laswriteopener;

	lasreadopener.set_file_name("france.laz");
	laswriteopener.set_file_name("out.laz");

	start_time = taketime();

	// check input & output
	if (!lasreadopener.active() || !laswriteopener.active())
	{
		fprintf(stderr, "ERROR: no input or output specified\n");
	}

	// open lasreader
	LASreader* lasreader = lasreadopener.open();
	if (lasreader == 0)
	{
		fprintf(stderr, "ERROR: could not open lasreader\n");
	}
	
	// open laswriter
	LASwriter* laswriter = laswriteopener.open(&lasreader->header);
	if (laswriter == 0)
	{
		fprintf(stderr, "ERROR: could not open laswriter\n");
	}

	//fprintf(stderr, "reading %I64d points from '%s' and writing them modified to '%s'.\n", lasreader->npoints, lasreadopener.get_file_name(), laswriteopener.get_file_name());

	// loop over points and modify them

	// where there is a point to read
	while (lasreader->read_point())
	{
		i++;
		// modify the point
		//lasreader->point.set_point_source_ID(1020);
		//lasreader->point.set_user_data(42);
		//if (lasreader->point.get_classification() == 12) lasreader->point.set_classification(1);
		
		/*lasreader->point.set_Z(lasreader->point.get_Z() + 100);
		lasreader->point.set_X(lasreader->point.get_X() + 100);
		lasreader->point.set_Y(lasreader->point.get_Y() + 100);*/

		LASpoint point;
		point.init(&lasreader->header, lasreader->header.point_data_format, lasreader->header.point_data_record_length, 0);
		
		//point.set_X(lasreader->point.get_X() +0.1);
		//point.set_Y(lasreader->point.get_Y() + 0.1);
		//point.set_Z(i + 0.1);
		
		// write the modified point
		laswriter->write_point(&point);
		// add it to the inventory
		laswriter->update_inventory(&point);
		//if (i == 1000) break;
	}

	laswriter->update_header(&lasreader->header, TRUE);

	I64 total_bytes = laswriter->close();
	delete laswriter;

	//fprintf(stderr, "total time: %g sec %I64d bytes for %I64d points\n", taketime() - start_time, total_bytes, lasreader->p_count);

	lasreader->close();
	delete lasreader;
	// std::cin.get();
	return 0;
}
