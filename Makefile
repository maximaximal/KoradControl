koradcontrol : koradcontrol.cpp
	clang++ koradcontrol.cpp -lboost_program_options -pthread -Wall -o koradcontrol --static

clean:
	rm koradcontrol
