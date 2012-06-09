
#include <stdio.h>
#include "CalcUTM.hh"

int
main(int argc, char* argv[])
{
    double lat = 42.678555;
    double lon = -83.195592;

    double utm_n = utm_north(lat, lon);
    double utm_e = utm_east(lat, lon);

    printf("UTM for %.06f, %.06f is %.02f, %.02f\n",
        lat, lon, utm_e, utm_n);

    return 0;
}
