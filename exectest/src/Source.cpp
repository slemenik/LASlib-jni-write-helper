/*
===============================================================================

  FILE:  las2las.cpp

  CONTENTS:

	This tool reads and writes LIDAR data in the LAS format and is typically
	used to modify the contents of a LAS file. Examples are keeping or dropping
	those points lying within a certain region (-keep_xy, -drop_x_above, ...),
	or points with a certain elevation (-keep_z, -drop_z_below, -drop_z_above)
	eliminating points that are the second return (-drop_return 2), that have a
	scan angle above a certain threshold (-drop_scan_angle_above 5), or that are
	below a certain intensity (-drop_intensity_below 15).
	Another typical use may be to extract only first (-first_only) returns or
	only last returns (-last_only). Extracting the first return is actually the
	same as eliminating all others (e.g. -keep_return 2 -keep_return 3, ...).

  PROGRAMMERS:

	martin.isenburg@rapidlasso.com  -  http://rapidlasso.com

  COPYRIGHT:

	(c) 2007-2017, martin isenburg, rapidlasso - fast tools to catch reality

	This is free software; you can redistribute and/or modify it under the
	terms of the GNU Lesser General Licence as published by the Free Software
	Foundation. See the LICENSE.txt file for more information.

	This software is distributed WITHOUT ANY WARRANTY and without even the
	implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

  CHANGE HISTORY:

	30 November 2017 -- set OGC WKT with '-set_ogc_wkt "PROJCS[\"WGS84\",GEOGCS[\"GCS_ ..."
	10 October 2017 -- allow piped input *and* output if no filter or coordinate change
	14 July 2017 -- fixed missing 'comma' in compound (COMPD_CS) OGC WKT string
	23 October 2016 -- OGC WKT string stores COMPD_CS for projection + vertical
	22 October 2016 -- new '-set_ogc_wkt_in_evlr' store to EVLR instead of VLR
	 1 January 2016 -- option '-set_ogc_wkt' to store CRS as OGC WKT string
	 3 May 2015 -- improved up-conversion via '-set_version 1.4 -point_type 6'
	 5 July 2012 -- added option to '-remove_original_vlr'
	 6 May 2012 -- added option to '-remove_tiling_vlr'
	 5 January 2012 -- added option to clip points to the bounding box
	17 May 2011 -- enabling batch processing with wildcards or multiple file names
	13 May 2011 -- moved indexing, filtering, transforming into LASreader
	18 April 2011 -- can set projection tags or reproject horizontally
	26 January 2011 -- added LAStransform for simply manipulations of points
	21 January 2011 -- added LASreadOpener and reading of multiple LAS files
	 3 January 2011 -- added -reoffset & -rescale + -keep_circle via LASfilter
	10 January 2010 -- added -subseq for selecting a [start, end] interval
	10 June 2009 -- added -scale_rgb_down and -scale_rgb_up to scale rgb values
	12 March 2009 -- updated to ask for input if started without arguments
	 9 March 2009 -- added ability to remove user defined headers or vlrs
	17 September 2008 -- updated to deal with LAS format version 1.2
	17 September 2008 -- dropping or keeping in double precision and based on z
	10 July 2007 -- created after talking with Linda about the H1B process

===============================================================================
*/

#include <time.h>
#include <stdlib.h>
#include <string>
#include <float.h>

#include "lasreader.hpp"
#include "laswriter.hpp"
#include "lastransform.hpp"
#include "bytestreamout_file.hpp"
#include "bytestreamin_file.hpp"

static void usage(bool error = false, bool wait = false)
{
	fprintf(stderr, "usage:\n");
	fprintf(stderr, "las2las -i *.las -utm 13N\n");
	fprintf(stderr, "las2las -i *.laz -first_only -olaz\n");
	fprintf(stderr, "las2las -i *.las -drop_return 4 5 -olaz\n");
	fprintf(stderr, "las2las -latlong -target_utm 12T -i in.las -o out.las\n");
	fprintf(stderr, "las2las -i in.laz -target_epsg 2972 -o out.laz\n");
	fprintf(stderr, "las2las -point_type 0 -lof file_list.txt -merged -o out.las\n");
	fprintf(stderr, "las2las -remove_vlr 2 -scale_rgb_up -i in.las -o out.las\n");
	fprintf(stderr, "las2las -i in.las -keep_xy 630000 4834500 630500 4835000 -keep_z 10 100 -o out.las\n");
	fprintf(stderr, "las2las -i in.txt -iparse xyzit -keep_circle 630200 4834750 100 -oparse xyzit -o out.txt\n");
	fprintf(stderr, "las2las -i in.las -remove_padding -keep_scan_angle -15 15 -o out.las\n");
	fprintf(stderr, "las2las -i in.las -rescale 0.01 0.01 0.01 -reoffset 0 300000 0 -o out.las\n");
	fprintf(stderr, "las2las -i in.las -set_version 1.2 -keep_gpstime 46.5 47.5 -o out.las\n");
	fprintf(stderr, "las2las -i in.las -drop_intensity_below 10 -olaz -stdout > out.laz\n");
	fprintf(stderr, "las2las -i in.las -last_only -drop_gpstime_below 46.75 -otxt -oparse xyzt -stdout > out.txt\n");
	fprintf(stderr, "las2las -i in.las -remove_all_vlrs -keep_class 2 3 4 -olas -stdout > out.las\n");
	fprintf(stderr, "las2las -h\n");
	if (wait)
	{
		fprintf(stderr, "<press ENTER>\n");
		getc(stdin);
	}
	exit(error);
}

static void byebye(bool error = false, bool wait = false)
{
	if (wait)
	{
		fprintf(stderr, "<press ENTER>\n");
		getc(stdin);
	}
	exit(error);
}

static double taketime()
{
	return (double)(clock()) / CLOCKS_PER_SEC;
}

static bool save_vlrs_to_file(const LASheader* header)
{
	U32 i;
	FILE* file = fopen("vlrs.vlr", "wb");
	if (file == 0)
	{
		return false;
	}
	ByteStreamOut* out;
	if (IS_LITTLE_ENDIAN())
		out = new ByteStreamOutFileLE(file);
	else
		out = new ByteStreamOutFileBE(file);
	// write number of VLRs
	if (!out->put32bitsLE((U8*)&(header->number_of_variable_length_records)))
	{
		fprintf(stderr, "ERROR: writing header->number_of_variable_length_records\n");
		return false;
	}
	// loop over VLRs
	for (i = 0; i < header->number_of_variable_length_records; i++)
	{
		if (!out->put16bitsLE((U8*)&(header->vlrs[i].reserved)))
		{
			fprintf(stderr, "ERROR: writing header->vlrs[%d].reserved\n", i);
			return false;
		}
		if (!out->putBytes((U8*)header->vlrs[i].user_id, 16))
		{
			fprintf(stderr, "ERROR: writing header->vlrs[%d].user_id\n", i);
			return false;
		}
		if (!out->put16bitsLE((U8*)&(header->vlrs[i].record_id)))
		{
			fprintf(stderr, "ERROR: writing header->vlrs[%d].record_id\n", i);
			return false;
		}
		if (!out->put16bitsLE((U8*)&(header->vlrs[i].record_length_after_header)))
		{
			fprintf(stderr, "ERROR: writing header->vlrs[%d].record_length_after_header\n", i);
			return false;
		}
		if (!out->putBytes((U8*)header->vlrs[i].description, 32))
		{
			fprintf(stderr, "ERROR: writing header->vlrs[%d].description\n", i);
			return false;
		}

		// write the data following the header of the variable length record

		if (header->vlrs[i].record_length_after_header)
		{
			if (header->vlrs[i].data)
			{
				if (!out->putBytes((U8*)header->vlrs[i].data, header->vlrs[i].record_length_after_header))
				{
					fprintf(stderr, "ERROR: writing %d bytes of data from header->vlrs[%d].data\n", header->vlrs[i].record_length_after_header, i);
					return false;
				}
			}
			else
			{
				fprintf(stderr, "ERROR: there should be %d bytes of data in header->vlrs[%d].data\n", header->vlrs[i].record_length_after_header, i);
				return false;
			}
		}
	}
	delete out;
	fclose(file);
	return true;
}

static bool load_vlrs_from_file(LASheader* header)
{
	U32 i;
	FILE* file = fopen("vlrs.vlr", "rb");
	if (file == 0)
	{
		return false;
	}
	ByteStreamIn* in;
	if (IS_LITTLE_ENDIAN())
		in = new ByteStreamInFileLE(file);
	else
		in = new ByteStreamInFileBE(file);
	// read number of VLRs
	U32 number_of_variable_length_records;
	try { in->get32bitsLE((U8*)&number_of_variable_length_records); }
	catch (...)
	{
		fprintf(stderr, "ERROR: reading number_of_variable_length_records\n");
		return false;
	}
	// loop over VLRs
	LASvlr vlr;
	for (i = 0; i < number_of_variable_length_records; i++)
	{
		try { in->get16bitsLE((U8*)&(vlr.reserved)); }
		catch (...)
		{
			fprintf(stderr, "ERROR: reading vlr.reserved\n");
			return false;
		}
		try { in->getBytes((U8*)vlr.user_id, 16); }
		catch (...)
		{
			fprintf(stderr, "ERROR: reading vlr.user_id\n");
			return false;
		}
		try { in->get16bitsLE((U8*)&(vlr.record_id)); }
		catch (...)
		{
			fprintf(stderr, "ERROR: reading vlr.record_id\n");
			return false;
		}
		try { in->get16bitsLE((U8*)&(vlr.record_length_after_header)); }
		catch (...)
		{
			fprintf(stderr, "ERROR: reading vlr.record_length_after_header\n");
			return false;
		}
		try { in->getBytes((U8*)vlr.description, 32); }
		catch (...)
		{
			fprintf(stderr, "ERROR: reading vlr.description\n");
			return false;
		}

		// write the data following the header of the variable length record

		if (vlr.record_length_after_header)
		{
			vlr.data = new U8[vlr.record_length_after_header];
			try { in->getBytes((U8*)vlr.data, vlr.record_length_after_header); }
			catch (...)
			{
				fprintf(stderr, "ERROR: reading %d bytes into vlr.data\n", vlr.record_length_after_header);
				return false;
			}
		}
		else
		{
			vlr.data = 0;
		}
		header->add_vlr(vlr.user_id, vlr.record_id, vlr.record_length_after_header, vlr.data, TRUE, vlr.description);
	}
	delete in;
	fclose(file);
	return true;
}

// for point type conversions
const U8 convert_point_type_from_to[11][11] =
{
  {  0,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1 },
  {  0,  0,  1,  1,  1,  1,  1,  1,  1,  1,  1 },
  {  0,  1,  0,  1,  1,  1,  1,  1,  1,  1,  1 },
  {  0,  0,  1,  0,  1,  1,  1,  1,  1,  1,  1 },
  {  0,  0,  1,  1,  0,  1,  1,  1,  1,  1,  1 },
  {  0,  0,  1,  0,  1,  0,  1,  1,  1,  1,  1 },
  {  1,  1,  1,  1,  1,  1,  0,  1,  1,  1,  1 },
  {  1,  1,  1,  1,  1,  1,  1,  0,  1,  1,  1 },
  {  1,  1,  1,  1,  1,  1,  1,  1,  0,  1,  1 },
  {  1,  1,  1,  1,  1,  1,  1,  1,  1,  0,  1 },
  {  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  0 },
};



char* constToChar(const char* str) {
	// const char* str = "-keep_xy";
	char *cstr = new char[strlen(str) + 1];
	strcpy(cstr, str);
	return cstr;
}

void test(char* a);
void test2(int argc, char *argv[]);

int main(int argc, char *argv[])
{

	//test(argv[0]);
	test2(argc, argv);
	return 0;
	bool verbose = false;
	bool very_verbose = false;
	bool force = false;
	// fixed header changes 
	int set_version_major = -1;
	int set_version_minor = -1;
	int set_point_data_format = -1;
	int set_point_data_record_length = -1;
	int set_global_encoding_gps_bit = -1;
	int set_lastiling_buffer_flag = -1;
	// variable header changes
	bool set_ogc_wkt = false;
	bool set_ogc_wkt_in_evlr = false;
	CHAR* set_ogc_wkt_string = 0;
	bool remove_header_padding = false;
	bool remove_all_variable_length_records = false;
	int remove_variable_length_record = -1;
	int remove_variable_length_record_from = -1;
	int remove_variable_length_record_to = -1;
	bool remove_all_extended_variable_length_records = false;
	int remove_extended_variable_length_record = -1;
	int remove_extended_variable_length_record_from = -1;
	int remove_extended_variable_length_record_to = -1;
	bool move_evlrs_to_vlrs = false;
	bool save_vlrs = false;
	bool load_vlrs = false;
	int set_attribute_scales = 0;
	int set_attribute_scale_index[5] = { -1, -1, -1, -1, -1 };
	double set_attribute_scale_scale[5] = { 1.0, 1.0, 1.0, 1.0, 1.0 };
	int set_attribute_offsets = 0;
	int set_attribute_offset_index[5] = { -1, -1, -1, -1, -1 };
	double set_attribute_offset_offset[5] = { 0.0, 0.0, 0.0, 0.0, 0.0 };
	int unset_attribute_scales = 0;
	int unset_attribute_scale_index[5] = { -1, -1, -1, -1, -1 };
	int unset_attribute_offsets = 0;
	int unset_attribute_offset_index[5] = { -1, -1, -1, -1, -1 };
	bool remove_tiling_vlr = false;
	bool remove_original_vlr = false;
	bool remove_empty_files = true;
	// extract a subsequence
	I64 subsequence_start = 0;
	I64 subsequence_stop = I64_MAX;
	// fix files with corrupt points
	bool clip_to_bounding_box = false;
	double start_time = 0;

	LASreadOpener lasreadopener;
	LASwriteOpener laswriteopener;

	//-i 462_100_grad.laz -o out.laz -keep_xy 462258 100584 462352 100596
	int argcFake = 6;
	char* argvFake[6] = { 
		argv[0], 
		constToChar("-keep_xy"),
		constToChar("462258"), //minx
		constToChar("100584"), //miny
		constToChar("462352"), //maxx
		constToChar("100596") // maxy
	};

	if (!lasreadopener.parse(argc, argv)) {
		byebye(true);
	}
	if (!laswriteopener.parse(argc, argv)) {
		byebye(true);
	}
	
	//lasreadopener.set_file_name("462_100_grad.laz");
	//laswriteopener.set_file_name("out.laz");

	// check input

	if (!lasreadopener.active())
	{
		fprintf(stderr, "ERROR: no input specified\n");
		usage(true, argc == 1);
	}

	// possibly loop over multiple input files

	while (lasreadopener.active())
	{
		if (verbose) start_time = taketime();

		// open lasreader

		LASreader* lasreader = lasreadopener.open();

		if (lasreader == 0)
		{
			fprintf(stderr, "ERROR: could not open lasreader\n");
			usage(true, argc == 1);
		}

		// store the inventory for the header

		LASinventory lasinventory;

		// the point we write sometimes needs to be copied

		LASpoint* point = 0;

		// prepare the header for output


		// reproject or just set the projection?

		LASquantizer* reproject_quantizer = 0;
		LASquantizer* saved_quantizer = 0;
		bool set_projection_in_header = false;


		// check output

		// prepare the header for the surviving points

		/*strncpy(lasreader->header.system_identifier, "LAStools (c) by rapidlasso GmbH", 32);
		lasreader->header.system_identifier[31] = '\0';
		char temp[64];
		sprintf(temp, "las2las (version %d)", LAS_TOOLS_VERSION);
		strncpy(lasreader->header.generating_software, temp, 32);
		lasreader->header.generating_software[31] = '\0';
*/
		// open laswriter

		LASwriter* laswriter = laswriteopener.open(&lasreader->header);

		if (laswriter == 0)
		{
			fprintf(stderr, "ERROR: could not open laswriter\n");
			byebye(true, argc == 1);
		}


		// loop over points

		int a = 0;
		int b = 0;
		int c = 0;
		int d = 0;
		int e = 0;
		int f = 0;
		int g = 0;

		
		while (lasreader->read_point())
		{
			a++;
			laswriter->write_point(&lasreader->point);
			// without extra pass we need inventory of surviving points
			laswriter->update_inventory(&lasreader->point);
		}


		// without the extra pass we need to fix the header now
		b = a;
		
			if (reproject_quantizer) lasreader->header = *reproject_quantizer;
			laswriter->update_header(&lasreader->header, TRUE);
			if (verbose) { fprintf(stderr, "total time: %g sec. written %u surviving points to '%s'.\n", taketime() - start_time, (U32)laswriter->p_count, laswriteopener.get_file_name()); }
		
		

		laswriter->close();
		// delete empty output files
		if (remove_empty_files && (laswriter->npoints == 0) && laswriteopener.get_file_name())
		{
			remove(laswriteopener.get_file_name());
			if (verbose) fprintf(stderr, "removing empty output file '%s'\n", laswriteopener.get_file_name());
		}
		delete laswriter;

		lasreader->close();
		delete lasreader;

		if (reproject_quantizer) {
			delete reproject_quantizer;
		}

		laswriteopener.set_file_name(0);
	}

	// byebye(false, argc == 1);

	return 0;
}

static char* doubleToChar(double d) {
	char *c = new char[10];
	sprintf(c, "%lf", d);
	return c;
}

LASreader* lasreader;
LASwriter* laswriter;

const char* init(const char* inputFileName, const char* outputFileName, int argc = NULL, char** argv = NULL) {


	LASreadOpener lasreadopener;
	if (argv != NULL) {
		if (!lasreadopener.parse(argc, argv)) {
			return "ERROR: lasreadopener.parse() \n";
		}
	}
	lasreadopener.set_file_name(inputFileName);
	if (!lasreadopener.active())
	{
		return "ERROR: no input specified\n";
	}
	lasreader = lasreadopener.open();
	if (lasreader == 0)
	{
		return "ERROR: could not open lasreader\n";
	}

	if (outputFileName != NULL) {
		LASwriteOpener laswriteopener;
		laswriteopener.set_file_name(outputFileName);
		if (!laswriteopener.active())
		{
			return "ERROR: no output specified\n";
		}
		// open lasreader

		// open laswriter
		laswriter = laswriteopener.open(&lasreader->header);
		if (laswriter == 0)
		{
			return "ERROR: could not open laswriter\n";
		}
	}
	//char returnValue[100];
	//sprintf(returnValue, "reading %I64d points from '%s' and writing them modified to '%s'.\n", lasreader->npoints, lasreadopener.get_file_name(), laswriteopener.get_file_name());
	return "init";
}

const char* after() {
	if (laswriter) {
		laswriter->update_header(&lasreader->header, TRUE);

		I64 total_bytes = laswriter->close();
		delete laswriter;
	}

	//const I64 count = lasreader->p_count;
	lasreader->close();
	delete lasreader;

	//double time = taketime() - start_time;
	//char returnValue[100];
	//sprintf(returnValue, "total time: %f sec %I64d bytes for %I64d points.\n", time, total_bytes, count);
	return "end writing";
}

void test2(int argc, char *argv[]) {
	
	const char *nativeStringInputFileName = "462_100_grad.laz";
	const char *nativeStringTempFileName = "temp.laz";

	//env->CallVoidMethod(obj, methodprintStringId, env->NewStringUTF("Java_com_slemenik_lidar_reconstruction_jni_JniLibraryHelpers_getMinMaxHeight"));

	//-keep_circle 630000 4850000 100
	char* argv2[5] = {
		//constToChar("C://\0"),//argv[0],
		constToChar("-keep_xy"),//constToChar("-keep_circle"),
		doubleToChar(462288.5555237896),
		doubleToChar(100587.71330607303),
		doubleToChar(462298.63557309966),
		doubleToChar(100602.37311110702),
	};

	//env->CallVoidMethod(obj, methodprintStringId, env->NewStringUTF("argv napolnjeni"));
	const char* message = init(nativeStringInputFileName, nativeStringTempFileName, argc, argv);
	int i = 0;
	while (lasreader->read_point())
	{
		// write the modified point
		laswriter->write_point(&lasreader->point);
		// add it to the inventory
		laswriter->update_inventory(&lasreader->point);

		i++;
	}
	// return createPoints(minHeight, maxHeight, x, y); //or closestPoint.get_x, closestPoint.get_y
	// return "Tocke ustvarjene.";
	after();

	return;
	//return JNIEXPORT jint JNICALL();
}

void test(char* a) {
	double maxHeight = 0.0;
	double minHeight = DBL_MAX;
	
	double x = 462298.36577557964;
	double y = 100588.93338568103;
	double radius = 0.1;
	const char* inputFileName = "462_100_grad.laz";

	//-keep_circle 630000 4850000 100
	char* argv[5] = {
		a,//argv[0],
		constToChar("-keep_circle"),//keep_circle
		constToChar("462298.36577557964"),//doubleToChar(x),
		constToChar("100588.93338568103"),//doubleToChar(y),
		constToChar("0.1")//doubleToChar(radius)
	};

	//env->CallVoidMethod(obj, methodprintStringId, env->NewStringUTF("argv napolnjeni"));
	const char* message = init(inputFileName, NULL, 5, argv);
	//env->CallVoidMethod(obj, methodprintStringId, env->NewStringUTF(message));
	LASpoint closestPoint;
	closestPoint.init(&lasreader->header, lasreader->header.point_data_format, lasreader->header.point_data_record_length, 0);
	while (lasreader->read_point())
	{
		//env->CallVoidMethod(obj, methodprintStringId, env->NewStringUTF("beremo tocke"));
		double lasX = lasreader->point.get_x();
		double lasY = lasreader->point.get_y();
		double lasZ = lasreader->point.get_z();
		//env->CallVoidMethod(obj, methodId, lasX);
		//env->CallVoidMethod(obj, methodId, lasY);
		//env->CallVoidMethod(obj, methodId, lasZ);

		//env->CallVoidMethod(obj, methodprintStringId, env->NewStringUTF("racunamo razdalje"));

		//env->CallVoidMethod(obj, methodprintStringId, env->NewStringUTF("primerjamo visine"));
		if (lasZ > maxHeight) maxHeight = lasZ;
		if (lasZ < minHeight) minHeight = lasZ;

		//env->CallVoidMethod(obj, methodprintStringId, env->NewStringUTF("ali tocka obstaja"));

	}
	// return createPoints(minHeight, maxHeight, x, y); //or closestPoint.get_x, closestPoint.get_y
	// return "Tocke ustvarjene.";
	after();
	double arr[4] = { minHeight, maxHeight, closestPoint.get_x(), closestPoint.get_y() };

	char ada = 'd';
}