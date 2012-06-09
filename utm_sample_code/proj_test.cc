
#include <math.h>
#include <stdio.h>
#include <proj_api.h>

double
dtor(double deg)
{
    return M_PI * (deg / 180.0);
}

int
main(int argc, char* argv[])
{
    projPJ ll  = pj_init_plus("+proj=latlong +datum=WGS84");
    projPJ utm = pj_init_plus("+proj=utm +datum=WGS84 +zone=17");

    double lat = 42.678555;
    double lon = -83.195592;

    double utm_n = dtor(lat);
    double utm_e = dtor(lon);
    double zero  = 0.0;

    int rv = pj_transform(ll, utm, 1, 0, &utm_e, &utm_n, &zero);
    if (rv)
        printf("Error code = %d : %s\n", rv, pj_strerrno(rv));

    printf("UTM for %.06f, %.06f is %.02f, %.02f\n",
        lat, lon, utm_e, utm_n);

    pj_free(ll);
    pj_free(utm);
    return 0;
}
