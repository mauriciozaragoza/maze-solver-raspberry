program_NAME := maze
program_C_SRCS := $(wildcard *.c)
program_CXX_SRCS := $(wildcard *.cpp)
program_C_OBJS := ${program_C_SRCS:.c=.o}
program_CXX_OBJS := ${program_CXX_SRCS:.cpp=.o}
program_OBJS := $(program_C_OBJS) $(program_CXX_OBJS)
program_INCLUDE_DIRS :=
program_LIBRARY_DIRS :=
program_LIBRARIES := 

CPPFLAGS += $(foreach includedir,$(program_INCLUDE_DIRS),-I$(includedir)) `pkg-config --cflags opencv` -std=c++0x 
LDFLAGS += $(foreach librarydir,$(program_LIBRARY_DIRS),-L$(librarydir)) 
LDFLAGS += $(foreach library,$(program_LIBRARIES),-l$(library))

.PHONY: all clean distclean

all: $(program_NAME)

$(program_NAME): $(program_OBJS)
		$(LINK.cc) $(program_OBJS) `pkg-config --libs opencv` -o $(program_NAME)

clean:
		@- $(RM) $(program_NAME)
		@- $(RM) $(program_OBJS)

distclean: clean