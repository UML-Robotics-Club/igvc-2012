

all: alex_test proj_test

alex_test: CalcUTM.cc CalcUTM.hh llutm.cc llutm.hh alex_test.cc
	g++ -o alex_test CalcUTM.cc llutm.cc alex_test.cc

proj_test: proj_test.cc
	g++ -o proj_test proj_test.cc -lproj

clean:
	rm -f alex_test *.o *~ proj_test
