# Indexes for lat
dLat = 0;
mLat = 1;
sLat = 2;

# Indexes for lon
dLon = 3;
mLon = 4;
sLon = 5;

# Conversion from dms to degrees
def conv(line):
    lat = int(line[dLat])+int(line[mLat])/60.0+float(line[sLat])/3600.0
    lon =  int(line[dLon])+int(line[mLon])/60.0+float(line[sLon])/3600.0
    gps.write(str(lat*-1) + "," + str(lon) + "\n");
    

# Open input/output files
f=open('coords.txt');
gps = open('gps.csv', 'w');

# Convert each line
for line in f:
    conv(line.split( ));

#Close the output file
gps.close();
