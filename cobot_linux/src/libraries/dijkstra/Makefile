all: dijkstra
	
dijkstra: dgraph.cpp dgraph_factory.cpp dijkstra.cpp radixheap.cpp run_dijkstra.cpp ../../shared/util/timer.cc dgraph_factory.h dgraph.h dijkstra.h heap.h radixheap.h ../../shared/util/timer.h
	g++ -o dijkstra -g -O3 dgraph.cpp dgraph_factory.cpp dijkstra.cpp radixheap.cpp run_dijkstra.cpp ../../shared/util/timer.cc -lrt -ltcmalloc -I../../shared/util
