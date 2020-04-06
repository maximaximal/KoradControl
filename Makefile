koradcontrol : koradcontrol.cpp
	clang++ koradcontrol.cpp -lboost_program_options -pthread -Wall -o koradcontrol

clean:
	rm koradcontrol
