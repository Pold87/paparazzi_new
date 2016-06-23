CC=gcc
CFLAGS=-I.
DEPS = particle_filter.h random_number_gen.h

all: particle_filter_sift #particle_filter_example

%.o: %.c $(DEPS)
	$(CC) -c -o $@ $< $(CFLAGS)

particle_filter_example: particle_filter_example.o
	gcc -g -o particle_filter_example particle_filter.c particle_filter_example.o random_number_gen.c $(DEPS) -I. -lm

particle_filter_sift: particle_filter_sift.o
	gcc -g -o particle_filter_sift particle_filter.c particle_filter_sift.o random_number_gen.c -I. -lm


run:
	./particle_filter

clean: 
	rm particle_filer_sift *.o
