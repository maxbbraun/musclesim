CC = g++
DEBUG = -g
CFLAGS = -Wall -O4
INCLUDE = -I./
LIBRARY = -L./
STD_OPTIONS = -lm

YURAGI_FILES  = AdaptiveAttractorSelection.cpp MSYuragi.cpp MSActivity.cpp CYuragi.cpp CActivity.cpp GMixture.cpp
ALL_CPP_FILES = $(YURAGI_FILES)

ALL_OBJS = $(ALL_CPP_FILES:.cpp=.o)

.SUFFIXES: .cpp .o
.cpp.o:
	$(CC) $(DEBUG) $(CFLAGS) $(INCLUDE) -c $*.cpp

all: $(ALL_OBJS)

clean:
	rm *.o *~

depend:
	makedepend -- $(CFLAGS) $(INCLUDE) $(LIBRARY) --$(ALL_CPP_FILES)

