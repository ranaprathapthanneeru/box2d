.SUFFIXES: .cpp .hpp

# Programs
SHELL 	= bash
CC     	= g++ -pg
LD	= ld
RM 	= rm
ECHO	= /bin/echo
CAT	= cat
PRINTF	= printf
SED	= sed
DOXYGEN = doxygen
PDFLATEX=pdflatex
BIBTEX=bibtex
GPROF=gprof
######################################
# Project Name (generate executable with this name)
TARGET = cs251_base

# Project Paths
PROJECT_ROOT=./
EXTERNAL_ROOT=$(PROJECT_ROOT)/external
SRCDIR = $(PROJECT_ROOT)/src
OBJDIR = $(PROJECT_ROOT)/obj
BINDIR = $(PROJECT_ROOT)/bin
DOCDIR = $(PROJECT_ROOT)/doc
BOX2DROOT=$(PROJECT_ROOT)/external/src
INBOXROOT=$(BOX2DROOT)/Box2D 
REPORTROOT=$(PROJECT_ROOT)/report
PYTH_TXT_DOT=$(PROJECT_ROOT)/gprof2dot.py
RUN=./
# Library Paths
BOX2D_ROOT=$(EXTERNAL_ROOT)
GLUI_ROOT=/usr
GL_ROOT=/usr/include/

#Libraries
LIBS = -lBox2D -lglui -lglut -lGLU -lGL

# Compiler and Linker flags
CPPFLAGS =-O3 -Wall -fno-strict-aliasing
CPPFLAGS+=-I $(BOX2D_ROOT)/include -I $(GLUI_ROOT)/include
LDFLAGS+=-L $(BOX2D_ROOT)/lib -L $(GLUI_ROOT)/lib

######################################

NO_COLOR=\e[0m
OK_COLOR=\e[1;32m
ERR_COLOR=\e[1;31m
WARN_COLOR=\e[1;33m
MESG_COLOR=\e[1;34m
FILE_COLOR=\e[1;37m

OK_STRING="[OK]"
ERR_STRING="[ERRORS]"
WARN_STRING="[WARNINGS]"
OK_FMT="${OK_COLOR}%30s\n${NO_COLOR}"
ERR_FMT="${ERR_COLOR}%30s\n${NO_COLOR}"
WARN_FMT="${WARN_COLOR}%30s\n${NO_COLOR}"
######################################

SRCS := $(wildcard $(SRCDIR)/*.cpp)
INCS := $(wildcard $(SRCDIR)/*.hpp)
OBJS := $(SRCS:$(SRCDIR)/%.cpp=$(OBJDIR)/%.o)


.PHONY: all setup b2dsetupgen codeDoc report clean distclean

all: setup b2dsetupgen $(BINDIR)/$(TARGET)
newall:setup b2dsetup $(BINDIR)/$(TARGET) 
setup:
	@$(ECHO) "Setting up compilation..."
	@mkdir -p obj
	@mkdir -p bin

b2dsetupgen:                  
	@if test ! -d $(INBOXROOT) ; then \
	cd $(BOX2DROOT) && tar -xzvf Box2D.tgz; \
	fi;
	@cd $(INBOXROOT) && mkdir -p build251 && cd build251 && cmake ../ && make && make install

$(BINDIR)/$(TARGET): $(OBJS)
	@$(PRINTF) "$(MESG_COLOR)Building executable:$(NO_COLOR) $(FILE_COLOR) %16s$(NO_COLOR)" "$(notdir $@)"
	@$(CC) -o $@ $(LDFLAGS) $(OBJS) $(LIBS) 2> temp.log || touch temp.err
	@if test -e temp.err; \
	then $(PRINTF) $(ERR_FMT) $(ERR_STRING) && $(CAT) temp.log; \
	elif test -s temp.log; \
	then $(PRINTF) $(WARN_FMT) $(WARN_STRING) && $(CAT) temp.log; \
	else $(PRINTF) $(OK_FMT) $(OK_STRING); \
	fi;
	@$(RM) -f temp.log temp.err

-include -include $(OBJS:.o=.d)

$(OBJS): $(OBJDIR)/%.o : $(SRCDIR)/%.cpp
	@$(PRINTF) "$(MESG_COLOR)Compiling: $(NO_COLOR) $(FILE_COLOR) %25s$(NO_COLOR)" "$(notdir $<)"
	@$(CC) $(CPPFLAGS) -c $< -o $@ -MD 2> temp.log || touch temp.err
	@if test -e temp.err; \
	then $(PRINTF) $(ERR_FMT) $(ERR_STRING) && $(CAT) temp.log; \
	elif test -s temp.log; \
	then $(PRINTF) $(WARN_FMT) $(WARN_STRING) && $(CAT) temp.log; \
	else printf "${OK_COLOR}%30s\n${NO_COLOR}" "[OK]"; \
	fi;
	@$(RM) -f temp.log temp.err

b2dsetup:                  
	@if test ! -d $(INBOXROOT) ; then \
	cd $(BOX2DROOT) && tar -xzvf Box2D.tgz; \
	fi;
	@cd $(INBOXROOT) && mkdir -p build251 && cd build251 && cmake -DCMAKE_BUILD_TYPE=Release ../ && make && make install

codeDoc:
	@$(ECHO) -n "Generating Doxygen Documentation ...  "
	@$(RM) -rf doc/html
	@$(DOXYGEN) $(DOCDIR)/Doxyfile 2 > /dev/null
	@$(ECHO) "Done"

report:
	cd $(REPORTROOT) && $(PDFLATEX) project.tex && $(BIBTEX) project.aux && $(PDFLATEX) project.tex && $(PDFLATEX) project.tex

clean:
	@$(ECHO) -n "Cleaning up..."
	@$(RM) -rf $(OBJDIR) *~ $(DEPS) $(SRCDIR)/*~
	@$(ECHO) "Done"

distclean: clean
	@$(RM) -rf $(BINDIR) $(INBOXROOT) $(DOCDIR)/html

profile: distclean all
	cd $(BINDIR) && $(RUN)cs251_base && $(GPROF) cs251_base gmon.out > profiling_data.txt
	cp gprof2dot.py $(BINDIR) 
	cd $(BINDIR) && python3 gprof2dot.py < profiling_data.txt > profiling_data.dot && dot -Tpng profiling_data.dot -o OutputFile.png  

profilerelease:
	cd $(BINDIR) && $(RUN)cs251_base && $(GPROF) cs251_base gmon.out > prof_data_rel.txt
	cp gprof2dot.py $(BINDIR)
	cd $(BINDIR) && python3 gprof2dot.py < prof_data_rel.txt > prof_data_rel.dot && dot -Tpng prof_data_rel.dot -o OutputFile_rel.png
release: distclean newall profilerelease 
