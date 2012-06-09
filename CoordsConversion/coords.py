import pyproj

# Indexes for lat
dLat = 0;
mLat = 1;
sLat = 2;

# Indexes for lon
dLon = 3;
mLon = 4;
sLon = 5;

def llToUTM(x,y):
    x,y = pyproj.transform(ll,utm,y,x)
    gps.write(str(x) + " " + str(y) +"\n")

# Conversion from dms to degrees
def conv(line):
    lat = int(line[dLat])+int(line[mLat])/60.0+float(line[sLat])/3600.0
    lon =  int(line[dLon])+int(line[mLon])/60.0+float(line[sLon])/3600.0
    lon *= -1.0
    #gps.write(str(lat) + "," + str(lon) + "\n");
    llToUTM(lat,lon)

# Open input/output files
f=open('coords.txt');
gps = open('gps.csv', 'w');

# Setup proj for right location
utm = pyproj.Proj(proj='utm', zone=17, datum='WGS84')
ll = pyproj.Proj(proj='latlon', datum='WGS84')

# Convert each line
for line in f:
    conv(line.split( ));

#Close the output file
gps.close();


