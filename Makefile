koradcontrol : koradcontrol.cpp
	clang++ koradcontrol.cpp -lboost_program_options -lboost_system -pthread -Wall -o koradcontrol --static -I/usr/local/include -L/usr/local/lib

clean:
	rm koradcontrol
