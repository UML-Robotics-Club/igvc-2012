/* -*- c++ -*- */
#ifndef CALC_UTM_HH
#define CALC_UTM_HH

double utm_north(double lat, double lon);
double utm_east(double lat, double lon);

double utm_to_lon(double utm_n, double utm_e);
double utm_to_lat(double utm_n, double utm_e);

#endif 
