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

static double taketime()
{
	return (double)(clock()) / CLOCKS_PER_SEC;
}

const char* init() {

	LASreadOpener lasreadopener;
	LASwriteOpener laswriteopener;

	lasreadopener.set_file_name("france.laz");
	laswriteopener.set_file_name("out.laz");

	start_time = taketime();

	// check input & output
	if (!lasreadopener.active() || !laswriteopener.active())
	{
		return "ERROR: no input or output specified\n";
	}
	// open lasreader
	lasreader = lasreadopener.open();
	if (lasreader == 0)
	{
		return "ERROR: could not open lasreader\n";
	}
	// open laswriter
	laswriter = laswriteopener.open(&lasreader->header);
	if (laswriter == 0)
	{
		return "ERROR: could not open laswriter\n";
	}
	char returnValue[100];
	sprintf(returnValue, "reading %I64d points from '%s' and writing them modified to '%s'.\n", lasreader->npoints, lasreadopener.get_file_name(), laswriteopener.get_file_name());

	return returnValue;
}

const char* after() {
	laswriter->update_header(&lasreader->header, TRUE);

	I64 total_bytes = laswriter->close();
	delete laswriter;
	const I64 count = lasreader->p_count;

	lasreader->close();
	delete lasreader;

	double time = taketime() - start_time;
	char returnValue[100];
	//sprintf(returnValue, "total time: %f sec %I64d bytes for %I64d points.\n", time, total_bytes, count);
	sprintf(returnValue, "end writing");
	return returnValue;
}

int write_point(const F64 x, const F64 y, const F64 z) {

	LASpoint point;
	point.init(&lasreader->header, lasreader->header.point_data_format, lasreader->header.point_data_record_length, 0);
	/*point.set_X(lasreader->point.get_X() + 0.1);
	point.set_Y(lasreader->point.get_Y() + 0.1);
	point.set_Z(i + 0.1);*/

	const F64 a = x;
	point.set_X(point.quantizer->get_X(x)); // brez quantizer ne sprejme F64, ampak     
	point.set_Y(point.quantizer->get_Y(y)); // I32, zato naredimo tu isto, kot se   
	point.set_Z(point.quantizer->get_Z(z)); // zgodi v ozadju, ce bi dali set (F64)

	// write the modified point
	BOOL result = laswriter->write_point(&point);
	// add it to the inventory
	laswriter->update_inventory(&point);

	return result;
}

JNIEXPORT jint JNICALL Java_com_slemenik_lidar_reconstruction_jni_JniLibraryHelpers_writeJNIPointList(JNIEnv * env, jobject obj, jobjectArray pointsArray)
{
	int len = env->GetArrayLength(pointsArray);
	jclass className = env->GetObjectClass(obj);
	jmethodID methodId = env->GetMethodID(className, "printDouble", "(D)V");
	jmethodID methodprintStringId = env->GetMethodID(className, "printString", "(Ljava/lang/String;)V");
	const char* message;

	message = init();
	env->CallVoidMethod(obj, methodprintStringId, env->NewStringUTF(message));
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
