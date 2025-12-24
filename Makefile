all:
	gcc main.c genetic.c graph.c multi.c visualize.c -o rescue \
	    -lglut -lGL -lGLU -pthread -Wall -lm

run: all
	./rescue

clean:
	rm -f rescue

