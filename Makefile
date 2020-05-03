koradcontrol : koradcontrol.cpp
	clang++ koradcontrol.cpp -lboost_program_options -lboost_system -pthread -Wall -o koradcontrol --static

clean:
	rm koradcontrol
