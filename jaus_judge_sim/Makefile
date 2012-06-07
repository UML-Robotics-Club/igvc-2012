
CC     = g++
DEBUG     = -ggdb -DDEBUG
OPTIONS   = -fno-strict-aliasing
DEFINES   = $(DEBUG)
LIB       = -L/usr/local/lib/active/ -lcxutils -ljauscore -ljausmobility -lpthread
FLAGS     = -Wall -I/usr/local/include/active/ $(OPTIONS) $(DEFINES)
#OBJS      = main_stub.o
OBJS_JUDGE= judge.o CommandConsole.o

#all:
#	make main_stub; make judge;
#
#main_stub: $(OBJS)
#	$(CC) -o main_stub $(OBJS) $(LIB)

judge: $(OBJS_JUDGE)
	$(CC) -o judge $(OBJS_JUDGE) $(LIB)

%.o: %.cpp
	$(CC) -c -g $(FLAGS) $<

clean:
	rm -f *.o *~ judge

