/* DO NOT EDIT THIS FILE - it is machine generated */
#include "jni.h"
/* Header for class com_slemenik_lidar_reconstruction_jni_JniLibraryHelpers */

#ifndef _Included_com_slemenik_lidar_reconstruction_jni_JniLibraryHelpers
#define _Included_com_slemenik_lidar_reconstruction_jni_JniLibraryHelpers
#ifdef __cplusplus
extern "C" {
#endif
	/*
 * Class:     com_slemenik_lidar_reconstruction_jni_JniLibraryHelpers
 * Method:    writeJNIPointList
 * Signature: ([[DLjava/lang/String;Ljava/lang/String;I)I
 */
	JNIEXPORT jint JNICALL Java_com_slemenik_lidar_reconstruction_jni_JniLibraryHelpers_writeJNIPointList
	(JNIEnv * env, jobject obj, jobjectArray pointsArray, jstring inputFileName, jstring outputFileName, jint classification);

	/*
	 * Class:     com_slemenik_lidar_reconstruction_jni_JniLibraryHelpers
	 * Method:    writeJNIPoint
	 * Signature: (DDD)V
	 */
	JNIEXPORT void JNICALL Java_com_slemenik_lidar_reconstruction_jni_JniLibraryHelpers_writeJNIPoint
	(JNIEnv * env, jobject obj, jdouble x, jdouble y, jdouble z);

	/*
	 * Class:     com_slemenik_lidar_reconstruction_jni_JniLibraryHelpers
	 * Method:    getMinMaxHeight
	 * Signature: (DDDLjava/lang/String;)[D
	 */
	JNIEXPORT jdoubleArray JNICALL Java_com_slemenik_lidar_reconstruction_jni_JniLibraryHelpers_getJNIMinMaxHeight
	(JNIEnv *env, jobject obj, jdouble x , jdouble y, jdouble radius, jstring inputFileName);

	/*
	* Class:     com_slemenik_lidar_reconstruction_jni_JniLibraryHelpers
	* Method : getJNIMinMaxHeightBBox
	* Signature : (DDDLjava / lang / String; DDDD)[D
	*/
	JNIEXPORT jdoubleArray JNICALL Java_com_slemenik_lidar_reconstruction_jni_JniLibraryHelpers_getJNIMinMaxHeightBBox
	(JNIEnv *env, jobject obj, jdouble x, jdouble y, jdouble radius, jstring inputFileName, jdouble bbox1, jdouble bbox2, jdouble bbox3, jdouble bbox4);

	/*
	 * Class:     com_slemenik_lidar_reconstruction_jni_JniLibraryHelpers
	 * Method:    createTempLaz
	 * Signature: (DDDDLjava/lang/String;)I
	 */
	JNIEXPORT jint JNICALL Java_com_slemenik_lidar_reconstruction_jni_JniLibraryHelpers_createTempLaz
	(JNIEnv *env, jobject obj, jdouble minX, jdouble minY, jdouble maxX, jdouble maxY, jstring tempFileName, jstring inputFileName);

	/*
	 * Class:     com_slemenik_lidar_reconstruction_jni_JniLibraryHelpers
	 * Method:    getJNIPointArray
	 * Signature: (Ljava/lang/String;)[[D
	 */
	JNIEXPORT jobjectArray JNICALL Java_com_slemenik_lidar_reconstruction_jni_JniLibraryHelpers_getJNIPointArray
	(JNIEnv *env, jobject obj, jstring inputFileName);

	/*
	 * Class:     com_slemenik_lidar_reconstruction_jni_JniLibraryHelpers
	 * Method:    getJNIHeaderInfo
	 * Signature: (Ljava/lang/String;)[D
	 */
	JNIEXPORT jdoubleArray JNICALL Java_com_slemenik_lidar_reconstruction_jni_JniLibraryHelpers_getJNIHeaderInfo
	(JNIEnv *env, jobject obj, jstring inputFileName);

	/*
	 * Class:     com_slemenik_lidar_reconstruction_jni_JniLibraryHelpers
	 * Method:    getJNIPointArrayRange
	 * Signature: (Ljava/lang/String;DD)[[D
	 */
	JNIEXPORT jobjectArray JNICALL Java_com_slemenik_lidar_reconstruction_jni_JniLibraryHelpers_getJNIPointArrayRange
	(JNIEnv *env, jobject obj, jstring inputFileName, jdouble minX, jdouble maxX);

	/*
	* Class:     com_slemenik_lidar_reconstruction_jni_JniLibraryHelpers
	* Method:    writeJNIPointListWithClassification
	* Signature: ([[DLjava/lang/String;Ljava/lang/String;)I
	*/
	JNIEXPORT jint JNICALL Java_com_slemenik_lidar_reconstruction_jni_JniLibraryHelpers_writeJNIPointListWithClassification
	(JNIEnv *env, jobject obj, jobjectArray pointsArray, jstring inputFileName, jstring outputFileName);

	/*
	 * Class:     com_slemenik_lidar_reconstruction_jni_JniLibraryHelpers
	 * Method:    getJNIPointArrayParams
	 * Signature: (Ljava/lang/String;[Ljava/lang/String;)[[D
	 */
	JNIEXPORT jobjectArray JNICALL Java_com_slemenik_lidar_reconstruction_jni_JniLibraryHelpers_getJNIPointArrayParams
	(JNIEnv *env, jobject obj, jstring inputFileName, jobjectArray params);

#ifdef __cplusplus
}
#endif
#endif
