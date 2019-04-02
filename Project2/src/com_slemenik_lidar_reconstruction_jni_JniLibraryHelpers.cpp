#include "com_slemenik_lidar_reconstruction_jni_JniLibraryHelpers.h"

#include <stdio.h>
#include <string>
#include <iostream>
#include <time.h>

#include "lasreader.hpp"
#include "laswriter.hpp"

LASreader* lasreader;
LASwriter* laswriter;
double start_time = 0.0;

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

static double distanceCalculate(double x1, double y1, double x2, double y2)
{
	double x = x1 - x2; //calculating number to square in next step
	double y = y1 - y2;
	double dist;

	dist = pow(x, 2) + pow(y, 2);       //calculating Euclidean distance
	dist = sqrt(dist);

	return dist;
}

// static void createPoints(double minZ, double maxZ, double x, double y) {
//	//        System.out.println("Ustvari tocke od visine " + minZ + " do " + maxZ);
//	double currentZ = minZ + CREATED_POINTS_SPACING; //we set first Z above minZ, avoiding duplicates points on same level
//	double *newPoints[3];
//	while (currentZ < maxZ) {
//		//            Main.count++;
//		
//		points2Insert.add(new double[] {x, y, currentZ});
//		
//
//		//            System.out.println(new Coordinate(x, y, currentZ));
//		currentZ += CREATED_POINTS_SPACING;
//	}
//}

//static double* getMinMaxHeight(double x, double y, double radius, const char* inputFileName ) {
//	//        System.out.println("Ustvari tocke na koordinati " + c);
//	double maxHeight = 0.0;
//	double minHeight = DBL_MAX;
//
//	//-keep_circle 630000 4850000 100
//	char* argv[6] = {
//		//constToChar("-tempTODO"),//argv[0],
//		constToChar("-keep_circle"),
//		doubleToChar(x),
//		doubleToChar(y),
//		doubleToChar(radius),
//	};
//
//	init(inputFileName, NULL, argv);
//	LASpoint closestPoint;
//	while (lasreader->read_point())
//	{
//		//laswriter->write_point(&lasreader->point);
//		
//		double lasX = lasreader->point.get_x();
//		double lasY = lasreader->point.get_y();
//		double lasZ = lasreader->point.get_z();
//
//		double distance = distanceCalculate(lasX, lasY, x, y);
//
//		if (lasZ > maxHeight) maxHeight = lasZ;
//		if (lasZ < minHeight) minHeight = lasZ;
//		
//		if (closestPoint.get_x()) {
//			double minDistance = distanceCalculate(closestPoint.get_x(), closestPoint.get_y(), x, y);
//			if (distance < minDistance) { 
//				//if current distance is smaller than distance from closestPoint and shp(x,y)
//				//we set the current point coordinates as being the closest
//				closestPoint.set_x(lasX);
//				closestPoint.set_y(lasX);
//				closestPoint.set_z(lasX);
//			}
//		}
//		else {//first iteration, does not exist yet, so we initialize it
//			closestPoint.init(&lasreader->header, lasreader->header.point_data_format, lasreader->header.point_data_record_length, 0);
//			closestPoint.set_x(lasX);
//			closestPoint.set_y(lasX);
//			closestPoint.set_z(lasX);
//		}
//	}
//	// return createPoints(minHeight, maxHeight, x, y); //or closestPoint.get_x, closestPoint.get_y
//	// return "Tocke ustvarjene.";
//	after();
//	return new double[4]{ minHeight, maxHeight, closestPoint.get_x(), closestPoint.get_y() };
//}

int write_point(const F64 x, const F64 y, const F64 z) {

	LASpoint point;
	point.init(&lasreader->header, lasreader->header.point_data_format, lasreader->header.point_data_record_length, 0);

	point.set_x(x);   
	point.set_y(y);
	point.set_z(z);

	// write the modified point
	BOOL result = laswriter->write_point(&point);
	// add it to the inventory
	laswriter->update_inventory(&point);

	return result;
}

JNIEXPORT jint JNICALL Java_com_slemenik_lidar_reconstruction_jni_JniLibraryHelpers_writeJNIPointList
(JNIEnv * env, jobject obj, jobjectArray pointsArray, jstring inputFileName, jstring outputFileName)
{
	int len = env->GetArrayLength(pointsArray);
	jclass className = env->GetObjectClass(obj);
	jmethodID methodId = env->GetMethodID(className, "printDouble", "(D)V");
	jmethodID methodprintStringId = env->GetMethodID(className, "printString", "(Ljava/lang/String;)V");
	
	const char *nativeStringInputFileName = env->GetStringUTFChars(inputFileName, 0);
	const char *nativeStringOutputName = env->GetStringUTFChars(outputFileName, 0);

	const char* message;
	message = init(nativeStringInputFileName, nativeStringOutputName);
	env->CallVoidMethod(obj, methodprintStringId, env->NewStringUTF(message));
	env->ReleaseStringUTFChars(inputFileName, nativeStringInputFileName);
	env->ReleaseStringUTFChars(outputFileName, nativeStringOutputName);

	//foreach point
	for (int i = 0; i < len; ++i) {
		jdoubleArray oneDim = (jdoubleArray)env->GetObjectArrayElement(pointsArray, i);
		jdouble *point = env->GetDoubleArrayElements(oneDim, 0);
		jdouble x = point[0];
		jdouble y = point[1];
		jdouble z = point[2];

		//write point
		int result = write_point(x, y, z);
		if (result == 0) {
			message = "Failed to write point %d, %d, %d", x, y, z;
			env->CallVoidMethod(obj, methodprintStringId, env->NewStringUTF(message));
		}

		/*env->CallVoidMethod(obj, methodId, x);
		env->CallVoidMethod(obj, methodId, y);
		env->CallVoidMethod(obj, methodId, z);*/

		env->ReleaseDoubleArrayElements(oneDim, point, JNI_ABORT);
		env->DeleteLocalRef(oneDim);
	}
	message = after();
	env->CallVoidMethod(obj, methodprintStringId, env->NewStringUTF(message));
	return len;
}

JNIEXPORT void JNICALL Java_com_slemenik_lidar_reconstruction_jni_JniLibraryHelpers_writeJNIPoint(JNIEnv * env, jobject obj, jdouble x, jdouble y, jdouble z)
{

	jclass className = env->GetObjectClass(obj);
	jmethodID mid = env->GetMethodID(className, "primt", "(Ljava/lang/String;)V");
	jmethodID middouble = env->GetMethodID(className, "test3", "(D)V");
	if (mid != NULL) {
		jstring msg = env->NewStringUTF("notri");//a.c_str()
		env->CallVoidMethod(obj, mid, msg);
		env->CallVoidMethod(obj, middouble, x);

		std::string a = "456";
		//        const char* ptrs = std::to_string(x).c_str();
		//        char* ptr = (char*)(&a);
		//        printf("a");
		//        fflush(stdout);
		msg = env->NewStringUTF("pol");//a.c_str()
		env->CallVoidMethod(obj, mid, msg);

		msg = env->NewStringUTF(a.c_str());//a.c_str()
		env->CallVoidMethod(obj, mid, msg);
	}
	//    return;
}

JNIEXPORT jdoubleArray JNICALL Java_com_slemenik_lidar_reconstruction_jni_JniLibraryHelpers_getJNIMinMaxHeight(JNIEnv * env, jobject obj, jdouble x, jdouble y, jdouble radius, jstring inputFileName)
{
	jclass className = env->GetObjectClass(obj);
	jmethodID methodId = env->GetMethodID(className, "printDouble", "(D)V");
	jmethodID methodprintStringId = env->GetMethodID(className, "printString", "(Ljava/lang/String;)V");
	const char *nativeStringInputFileName = env->GetStringUTFChars(inputFileName, 0);

	//env->CallVoidMethod(obj, methodprintStringId, env->NewStringUTF("Java_com_slemenik_lidar_reconstruction_jni_JniLibraryHelpers_getMinMaxHeight"));

	double maxHeight = 0.0;
	double minHeight = DBL_MAX;

	//-keep_circle 630000 4850000 100
	char* argv[5] = {
		constToChar("dummy"),//argv[0],
		constToChar("-keep_circle"),
		doubleToChar(x),
		doubleToChar(y),
		doubleToChar(radius),
	};
	LASreadOpener lasreadopener;
	if (!lasreadopener.parse(5, argv)) return NULL;

	lasreadopener.set_file_name(nativeStringInputFileName);
	LASreader* lasreader = lasreadopener.open();

	LASpoint closestPoint;
	closestPoint.init(&lasreader->header, lasreader->header.point_data_format, lasreader->header.point_data_record_length, 0);
	int i = 0;
	while (lasreader->read_point())
	{
		double lasX = lasreader->point.get_x();
		double lasY = lasreader->point.get_y();
		double lasZ = lasreader->point.get_z();

		double distance = distanceCalculate(lasX, lasY, x, y);

		if (lasZ > maxHeight) maxHeight = lasZ;
		if (lasZ < minHeight) minHeight = lasZ;

		//env->CallVoidMethod(obj, methodprintStringId, env->NewStringUTF("ali tocka obstaja"));
		if (closestPoint.get_x()) {
			//env->CallVoidMethod(obj, methodprintStringId, env->NewStringUTF("da"));
			double minDistance = distanceCalculate(closestPoint.get_x(), closestPoint.get_y(), x, y);
			if (distance < minDistance) {
				//if current distance is smaller than distance from closestPoint and shp(x,y)
				//we set the current point coordinates as being the closest
				closestPoint.set_x(lasX);
				closestPoint.set_y(lasX);
				closestPoint.set_z(lasX);
			}
		}
		else {//first iteration, does not exist yet, so we initialize it
			//env->CallVoidMethod(obj, methodprintStringId, env->NewStringUTF("ne"));
			closestPoint.set_x(lasX);
			closestPoint.set_y(lasX);
			closestPoint.set_z(lasX);
		}
		i++;
	}
	lasreader->close();
	delete lasreader;

	double arr[4] = { minHeight, maxHeight, closestPoint.get_x(), closestPoint.get_y() };
	
	std::string s = std::to_string(i);
	char const *pchar = s.c_str();
	env->CallVoidMethod(obj, methodprintStringId, env->NewStringUTF(pchar));

	env->ReleaseStringUTFChars(inputFileName, nativeStringInputFileName);
	jdoubleArray result = env->NewDoubleArray(4);

	env->SetDoubleArrayRegion(result, 0, 4, arr);
	return result;
	//return JNIEXPORT jdoubleArray JNICALL();
}

JNIEXPORT jint JNICALL Java_com_slemenik_lidar_reconstruction_jni_JniLibraryHelpers_createTempLaz
(JNIEnv * env, jobject obj, jdouble minX, jdouble minY, jdouble maxX, jdouble maxY, jstring tempFileName, jstring inputFileName)
{
	jclass className = env->GetObjectClass(obj);
	jmethodID methodId = env->GetMethodID(className, "printDouble", "(D)V");
	jmethodID methodprintStringId = env->GetMethodID(className, "printString", "(Ljava/lang/String;)V");
	const char *nativeStringInputFileName = env->GetStringUTFChars(inputFileName, 0);
	const char *nativeStringTempFileName = env->GetStringUTFChars(tempFileName, 0);

	LASreadOpener lasreadopener;
	LASwriteOpener laswriteopener;

	//-keep_circle 630000 4850000 100
	char* argv[6] = {
		constToChar("dummy"),//argv[0],
		constToChar("-keep_xy"),//constToChar("-keep_circle"),
		doubleToChar(minX),
		doubleToChar(minY),
		doubleToChar(maxX),
		doubleToChar(maxY),
	};

	if (!lasreadopener.parse(6, argv)) return -1;

	lasreadopener.set_file_name(nativeStringInputFileName);
	laswriteopener.set_file_name(nativeStringTempFileName);

	LASreader* lasreader = lasreadopener.open();
	LASwriter* laswriter = laswriteopener.open(&lasreader->header);

	int i = 0;
	while (lasreader->read_point())
	{
		laswriter->write_point(&lasreader->point);
		laswriter->update_inventory(&lasreader->point);
		i++;
	}
	laswriter->update_header(&lasreader->header, TRUE);

	laswriter->close();
	delete laswriter;
	lasreader->close();
	delete lasreader;

	env->ReleaseStringUTFChars(tempFileName, nativeStringTempFileName);
	env->ReleaseStringUTFChars(inputFileName, nativeStringInputFileName);
	
	return i;
	//return JNIEXPORT jint JNICALL();
}
