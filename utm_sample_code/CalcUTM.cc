
#include "CalcUTM.hh"
#include "llutm.hh"

const int E = 23;
char utm_letter = 'T';
int  utm_zone   = 19;

double utm_north(double lat, double lon) {
    lluType rv;
    rv = LLtoUTM(E, lat, lon, 0);
    utm_letter = rv.UTMLetter;
    utm_zone = rv.zone;
    return rv.UTMNorthing;
}

double utm_east(double lat, double lon) {
    lluType rv;
    rv = LLtoUTM(E, lat, lon, 0);
    utm_letter = rv.UTMLetter;
    utm_zone = rv.zone;
    return rv.UTMEasting;    
}

double utm_to_lat(double utm_n, double utm_e) {
    ullType rv;
    rv = UTMtoLL(E, utm_n, utm_e, utm_zone, utm_letter);
    return rv.latitude;
}

double utm_to_lon(double utm_n, double utm_e) {
    ullType rv;
    rv = UTMtoLL(E, utm_n, utm_e, utm_zone, utm_letter);
    return rv.longitude;        
}
