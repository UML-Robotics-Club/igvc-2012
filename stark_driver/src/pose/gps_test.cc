
#include <iostream>
using std::cout;
using std::cerr;
using std::endl;

#include <iomanip>

#include "GpsProxy.hh"

int
main(int argc, char* argv[])
{
    GpsProxy gps;
 
    cout << std::fixed;
    cout << std::setprecision(2);
   
    while (1) {
        gps.update();
        gps_pos_t pos = gps.position();
        cout << "Pos: " << pos.utm_e << " " << pos.utm_n << endl;        
    }

    return 0;
}
