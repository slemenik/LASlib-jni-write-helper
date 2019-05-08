

#include <stdlib.h>
#include <string.h>

#include "lasreader.hpp"
#include "laswriter.hpp"
#include "lastransform.hpp"

static char* constToChar(const char* str) {
	// const char* str = "-keep_xy";
	char *cstr = new char[strlen(str) + 1];
	strcpy(cstr, str);
	return cstr;
}

static char* doubleToChar(double d) {
	char *c = new char[10];
	sprintf(c, "%lf", d);
	return c;
}

int main(int argc, char *argv[])
{
	LASreadOpener lasreadopener;
	LASwriteOpener laswriteopener;
	
	/* -i 
	C:\Users\Matej\IdeaProjects\lidar-buildings-mountains-reconstruction\462_100_grad.laz 
	-o 
	C:\Users\Matej\Dropbox\Faks\magistrska\temp\LAStools\bin\out.laz 
	-keep_xy 
	462356.5542241777 
	100650.86422597301 
	462384.7146828237 
	100678.16336077302*/

	//char* argv2[6] = {
	//	constToChar("dummy"),//argv[0],
	//	//constToChar("-i"),
	//	//constToChar("C:\\Users\\Matej\\IdeaProjects\\lidar-buildings-mountains-reconstruction\\462_100_grad.laz"),
	//	//constToChar("-o"),
	//	//constToChar("C:\\Users\\Matej\\Dropbox\\Faks\\magistrska\\temp\\LAStools\\bin\\out.laz"),
	//	constToChar("-keep_xy"),//constToChar("-keep_circle"),
	//	doubleToChar(462256.5542241777),
	//	doubleToChar(100610.86422597301),
	//	doubleToChar(462384.7146828237),
	//	doubleToChar(100678.16336077302),
	//};
	//int argc1 = 6;

	
	//if (!lasreadopener.parse(argc1, argv2)) return -1;
		//if (!laswriteopener.parse(argc1, argv2)) return -1;
	
	lasreadopener.set_file_name("C:\\Users\\Matej\\Dropbox\\Faks\\magistrska\\laz\\410_137_triglav.laz");
	laswriteopener.set_file_name("C:\\Users\\Matej\\Dropbox\\Faks\\magistrska\\laz\\out.laz");

	// check input

	if (!lasreadopener.active())
	{
		fprintf(stderr, "ERROR: no input specified\n");
	}

	// open lasreader

	LASreader* lasreader = lasreadopener.open();

	if (lasreader == 0)
	{
		fprintf(stderr, "ERROR: could not open lasreader\n");
	}

	LASwriter* laswriter = laswriteopener.open(&lasreader->header);

	if (laswriter == 0)
	{
		fprintf(stderr, "ERROR: could not open laswriter\n");
	}

	LASpoint point;
	LASheader header = lasreader->header;
	point.init(&header, lasreader->header.point_data_format, lasreader->header.point_data_record_length, &header);


	while (lasreader->read_point())
	{
		point = lasreader->point;
		point.set_x(0.1);
		laswriter->write_point(&point);
		laswriter->update_inventory(&point);
	}

	laswriter->update_header(&lasreader->header, TRUE);
		
	laswriter->close();
	delete laswriter;
	lasreader->close();
	delete lasreader;

		


	return 0;
}
