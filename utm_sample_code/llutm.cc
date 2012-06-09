#include <stdio.h>
#include "llutm.hh"

// Define constants
#define deg2rad (M_PI / 180.0)
#define rad2deg (180.0 / M_PI)
#define k0 0.9996
#define equatorialRadius 1    // Modified
#define eccentricitySquared 2 // Modified

/* 
 Id, Equatorial Radius, Square of Eccentricity	
 first once is a placeholder only, To allow array indices to match id numbers 
*/
double ellipsoid [24][3] = {
	{ -1, 0, 0},                 // Placeholder
	{ 1, 6377563, 0.00667054},   // Airy
	{ 2, 6378160, 0.006694542},  // Australian National
	{ 3, 6377397, 0.006674372},  // Bessel 1841
	{ 4, 6377484, 0.006674372},  // Bessel 1841 (Nambia} 
	{ 5, 6378206, 0.006768658},  // Clarke 1866
	{ 6, 6378249, 0.006803511},  // Clarke 1880
	{ 7, 6377276, 0.006637847},  // Everest
	{ 8, 6378166, 0.006693422},  // Fischer 1960 (Mercury} 
	{ 9, 6378150, 0.006693422},  // Fischer 1968
	{ 10, 6378160, 0.006694605}, // GRS 1967
	{ 11, 6378137, 0.00669438},  // GRS 1980
	{ 12, 6378200, 0.006693422}, // Helmert 1906
	{ 13, 6378270, 0.00672267},  // Hough
	{ 14, 6378388, 0.00672267},  // International
	{ 15, 6378245, 0.006693422}, // Krassovsky
	{ 16, 6377340, 0.00667054},  // Modified Airy
	{ 17, 6377304, 0.006637847}, // Modified Everest
	{ 18, 6378155, 0.006693422}, // Modified Fischer 1960
	{ 19, 6378160, 0.006694542}, // South American 1969
	{ 20, 6378165, 0.006693422}, // WGS 60
	{ 21, 6378145, 0.006694542}, // WGS 66
	{ 22, 6378135, 0.006694318}, // WGS-72
	{ 23, 6378137, 0.00669438}   // WGS-84
};

#ifdef LLUTM_HAS_MAIN
void main() {
  lluType temp;
  temp = LLtoUTM(23,12,10,0);
  printf("LLtoUTM - %d, %c, %f, %f \n",temp.zone, temp.UTMLetter, temp.UTMEasting, temp.UTMNorthing); 

  ullType temp2;
  temp2 = UTMtoLL(1,150000,100,32,'P');
  printf("UTMtoLL - %f, %f %c",temp2.latitude, temp2.longitude, temp2.hemisphere); 
  return;
}
#endif

lluType LLtoUTM(int referenceEllipsoid, double latitude, double longitude, int zone){
/*converts lat/long to UTM coords.  Equations from USGS Bulletin 1532 
East Longitudes are positive, West longitudes are negative. 
North latitudes are positive, South latitudes are negative
Lat and Long are in decimal degrees*/

  // Local Defines
  int longOrigin;
  double a, eccSquared, longTemp, latRad, longRad, longOriginRad, 
	eccPrimeSquared, N, T, C, A, M, UTMEasting, UTMNorthing;
  lluType results;

  a = ellipsoid[referenceEllipsoid][equatorialRadius];
  eccSquared = ellipsoid[referenceEllipsoid][eccentricitySquared];

  //Make sure the longitude is between -180.00 .. 179.9
  longTemp = (longitude+180)-(int)((longitude+180)/360)*360-180; //-180.00 .. 179.9
  latRad = latitude * deg2rad;
  longRad = longTemp * deg2rad;

  // Determines zone if 0
  if (zone == 0){
    zone = (int)((longTemp + 180)/6) + 1;
  }

  // Checks zone within boundary
  if (latitude >= 56.0 && latitude < 64.0 && longTemp >= 3.0 && longTemp < 12.0)
    zone = 32;

  // Special zones for Svalbard
  if (latitude >= 72.0 && latitude < 84.0) {
    if (longTemp >= 0.0  && longTemp <  9.0) zone = 31;
    else if (longTemp >= 9.0  && longTemp < 21.0) zone = 33;
    else if (longTemp >= 21.0 && longTemp < 33.0) zone = 35;
    else if (longTemp >= 33.0 && longTemp < 42.0) zone = 37;
  }

  longOrigin = (zone - 1) * 6 - 180 + 3; // +3 puts origin in middle of zone
  longOriginRad = longOrigin * deg2rad;

  eccPrimeSquared = (eccSquared)/(1-eccSquared);
  N = a/sqrt(1-eccSquared * sin(latRad)* sin(latRad));
  T = tan(latRad) * tan(latRad);
  C = eccPrimeSquared * cos(latRad) * cos(latRad);
  A = cos(latRad) * (longRad-longOriginRad);

  M = a*((1
    - eccSquared/4
    - 3*eccSquared*eccSquared/64
    - 5*eccSquared*eccSquared*eccSquared/256)*latRad 
    - (3*eccSquared/8
    + 3*eccSquared*eccSquared/32
    + 45*eccSquared*eccSquared*eccSquared/1024)*sin(2*latRad)
    + (15*eccSquared*eccSquared/256 + 45*eccSquared*eccSquared*eccSquared/1024)*sin(4*latRad) 
    - (35*eccSquared*eccSquared*eccSquared/3072)*sin(6*latRad));
    
  UTMEasting = (k0*N*(A+(1-T+C)*A*A*A/6
    + (5-18*T+T*T+72*C-58*eccPrimeSquared)*A*A*A*A*A/120) 
    + 500000.0);

  UTMNorthing = (k0*(M+N*tan(latRad)*(A*A/2+(5-T+9*C+4*C*C)*A*A*A*A/24
    + (61-58*T+T*T+600*C-330*eccPrimeSquared)
    *A*A*A*A*A*A/720)));

    if (latitude < 0)
        UTMNorthing = UTMNorthing + 10000000.0; // 10000000 meter offset for southern hemisphere

  // Populate Struct
  results.zone = zone;
  results.UTMLetter = UTMLetterDesignator(latitude);
  results.UTMEasting = UTMEasting;
  results.UTMNorthing = UTMNorthing;
  
  return results;
}

ullType UTMtoLL(int referenceEllipsoid, double northing, double easting, int zone, char utmLetter){
/*converts UTM coords to lat/long.  Equations from USGS Bulletin 1532 
East Longitudes are positive, West longitudes are negative. 
North latitudes are positive, South latitudes are negative
Lat and Long are in decimal degrees.*/  

  // Local Defines
  int longOrigin, hemisphere;
  double a, eccSquared, e1, x, y, eccPrimeSquared, m, mu, phi1Rad, phi1, n1, t1,c1,r1,d, lat, longitude;
  ullType results;
  
  a = ellipsoid[referenceEllipsoid][equatorialRadius];
  eccSquared = ellipsoid[referenceEllipsoid][eccentricitySquared];
  e1 = (1-sqrt(1-eccSquared))/(1+sqrt(1-eccSquared));
  //NorthernHemisphere; //1 for northern hemisphere, 0 for southern

  x = easting - 500000.0; // remove 500,000 meter offset for longitude
  y = northing;


  if (utmLetter >= 'N')
    hemisphere = 1;  // point is in northern hemisphere
  else {
    hemisphere = 0;  // point is in southern hemisphere
    y -= 10000000.0;         // remove 10,000,000 meter offset used for southern hemisphere
  }
  longOrigin = (zone - 1) * 6 - 180 + 3;  // +3 puts origin in middle of zone

  eccPrimeSquared = (eccSquared)/(1-eccSquared);


  m = y / k0;
  mu = m/(a*(1-eccSquared/4-3*eccSquared*eccSquared/64-5*eccSquared*eccSquared*eccSquared/256));
  phi1Rad = (mu + (3*e1/2-27*e1*e1*e1/32)*sin(2*mu) 
               + (21*e1*e1/16-55*e1*e1*e1*e1/32)*sin(4*mu)
               +(151*e1*e1*e1/96)*sin(6*mu));
  phi1 = phi1Rad * rad2deg;
  n1 = a/sqrt(1-eccSquared * sin(phi1Rad) * sin(phi1Rad));
  t1 = tan(phi1Rad) * tan(phi1Rad);
  c1 = eccPrimeSquared * cos(phi1Rad) * cos(phi1Rad);
  r1 = a*(1-eccSquared)/pow(1-eccSquared*sin(phi1Rad) * sin(phi1Rad), 1.5);
  d = x/(n1*k0);

  lat = phi1Rad - (n1*tan(phi1Rad)/r1)*(d*d/2-(5+3*t1+10*c1-4*c1*c1-9*eccPrimeSquared)*d*d*d*d/24
                                          +(61+90*t1+298*c1+45*t1*t1-252*eccPrimeSquared-3*c1*c1)*d*d*d*d*d*d/720);
  lat = lat * rad2deg;

  longitude = (d-(1+2*t1+c1)*d*d*d/6+(5-2*c1+28*t1-3*c1*c1+8*eccPrimeSquared+24*t1*t1)
            *d*d*d*d*d/120)/cos(phi1Rad);
  longitude = longOrigin + longitude * rad2deg;

  // Populate Results
  results.latitude = lat;
  results.longitude = longitude;
  results.hemisphere = hemisphere;  
  return results;
}

char UTMLetterDesignator(double lat){
/*This routine determines the correct UTM letter designator for the given latitude
returns 'Z' if latitude is outside the UTM limits of 84N to 80S */

    if ( lat <= 84 && lat >= 72 ) return 'X';
    else if ( lat < 72 && lat >= 64 ) return 'W';
    else if ( lat < 64 && lat >= 56 ) return 'V';
    else if ( lat < 56 && lat >= 48 ) return 'U';
    else if ( lat < 48 && lat >= 40 ) return 'T';
    else if ( lat < 40 && lat >= 32 ) return 'S';
    else if ( lat < 32 && lat >= 24 ) return 'R';
    else if ( lat < 24 && lat >= 16 ) return 'Q';
    else if ( lat < 16 && lat >= 8 ) return 'P';
    else if ( lat < 8  && lat >= 0 ) return 'N';
    else if ( lat < 0  && lat >= -8 ) return 'M';
    else if ( lat < -8 && lat >= -16 ) return 'L';
    else if ( lat < -16 && lat >= -24 ) return 'K';
    else if ( lat < -24 && lat >= -32) return 'J';
    else if ( lat < -32 && lat >= -40) return 'H';
    else if ( lat < -40 && lat >= -48) return 'G';
    else if ( lat < -48 && lat >= -56) return 'F';
    else if ( lat < -56 && lat >= -64) return 'E';
    else if ( lat < -64 && lat >= -72) return 'D';
    else if ( lat < -72 && lat >= -80) return 'C';
    else  return 'Z';	// if the Latitude is outside the UTM limits
}
