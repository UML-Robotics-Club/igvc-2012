#ifndef LL_UTM_H
#define LL_UTM_H

#include <math.h>

extern "C" {

// Define Proto's
struct lluStruct LLtoUTM(int ReferenceEllipsoid, double Lat, double Long, int zone);
struct ullStruct UTMtoLL(int referenceEllipsoid, double northing, double easting, int zone, char utmLetter);
char UTMLetterDesignator(double Lat);

};

// Define Structs
typedef struct lluStruct {
  int zone;
  char UTMLetter;
  double UTMEasting;
  double UTMNorthing;
} lluType;

typedef struct ullStruct {
  double latitude;
  double longitude;
  int hemisphere;
} ullType;

#endif
