CXX=g++

CSVLIBDIR=libcsv-3.0.3
CSVLIB=libcsv.a

INCDIR=./include
SRCDIR=./src
BINDIR=./bin
OBJDIR=./obj
TESTBINDIR=./testbin

CXXFLAGS = --std=c++14 -I $(INCDIR) -I $(CSVLIBDIR)
TESTLDFlAGS= -lgtest -lgtest_main -lpthread
XMLLDFLAGS= -lexpat

CSVOBJS=$(OBJDIR)/CSVReader.o
XMLOBJS=$(OBJDIR)/XMLReader.o
ROUTEROBJ=$(OBJDIR)/MapRouter.o 


CSVTEST=testcsv
XMLTEST=testxml
ROUTERTEST=testrouter

all: $(CSVLIBDIR)/.libs/$(CSVLIB) directories RUNTESTS

RUNTESTS: RUNCSVTEST RUNXMLTEST RUNROUTERTEST

RUNCSVTEST: $(TESTBINDIR)/$(CSVTEST)
	$(TESTBINDIR)/$(CSVTEST)

RUNXMLTEST: $(TESTBINDIR)/$(XMLTEST)
	$(TESTBINDIR)/$(XMLTEST)

RUNROUTERTEST: $(TESTBINDIR)/$(ROUTERTEST)
	$(TESTBINDIR)/$(ROUTERTEST)

$(CSVLIBDIR)/.libs/$(CSVLIB): $(CSVLIBDIR)/Makefile
	cd $(CSVLIBDIR); make ; cd ..

$(CSVLIBDIR)/Makefile:
	cd $(CSVLIBDIR); ./configure ; cd ..

$(TESTBINDIR)/$(ROUTERTEST): $(OBJDIR)/testrouter.o $(ROUTEROBJ) $(CSVOBJS) $(XMLOBJS) $(CSVLIBDIR)/.libs/$(CSVLIB)
	$(CXX) $(CXXFLAGS) $(OBJDIR)/testrouter.o $(ROUTEROBJ) $(CSVOBJS) $(XMLOBJS) $(CSVLIBDIR)/.libs/$(CSVLIB) -o $(TESTBINDIR)/$(ROUTERTEST) $(TESTLDFlAGS) $(XMLLDFLAGS)

$(OBJDIR)/testrouter.o: $(SRCDIR)/testrouter.cpp $(INCDIR)/MapRouter.h
	$(CXX) $(CXXFLAGS) $(SRCDIR)/testrouter.cpp -c -o $(OBJDIR)/testrouter.o

$(ROUTEROBJ): $(SRCDIR)/MapRouter.cpp $(INCDIR)/MapRouter.h $(INCDIR)/XMLReader.h $(INCDIR)/CSVReader.h $(INCDIR)/XMLEntity.h
	$(CXX) $(CXXFLAGS) $(SRCDIR)/MapRouter.cpp -c -o $(ROUTEROBJ)

$(TESTBINDIR)/$(CSVTEST): $(OBJDIR)/testcsv.o $(CSVOBJS) $(CSVLIBDIR)/.libs/$(CSVLIB)
	$(CXX) $(CXXFLAGS) $(OBJDIR)/testcsv.o $(CSVOBJS) $(CSVLIBDIR)/.libs/$(CSVLIB) -o $(TESTBINDIR)/$(CSVTEST) $(TESTLDFlAGS)

$(OBJDIR)/testcsv.o: $(SRCDIR)/testcsv.cpp $(INCDIR)/CSVReader.h
	$(CXX) $(CXXFLAGS) $(SRCDIR)/testcsv.cpp -c -o $(OBJDIR)/testcsv.o

$(OBJDIR)/CSVReader.o: $(SRCDIR)/CSVReader.cpp $(INCDIR)/CSVReader.h
	$(CXX) $(CXXFLAGS) $(SRCDIR)/CSVReader.cpp -c -o $(OBJDIR)/CSVReader.o

$(TESTBINDIR)/$(XMLTEST): $(OBJDIR)/testxml.o $(XMLOBJS)
	$(CXX) $(CXXFLAGS) $(OBJDIR)/testxml.o $(XMLOBJS) -o $(TESTBINDIR)/$(XMLTEST) $(TESTLDFlAGS) $(XMLLDFLAGS)

$(OBJDIR)/testxml.o: $(SRCDIR)/testxml.cpp $(INCDIR)/XMLReader.h
	$(CXX) $(CXXFLAGS) $(SRCDIR)/testxml.cpp -c -o $(OBJDIR)/testxml.o

$(OBJDIR)/XMLReader.o: $(SRCDIR)/XMLReader.cpp $(INCDIR)/XMLReader.h
	$(CXX) $(CXXFLAGS) $(SRCDIR)/XMLReader.cpp -c -o $(OBJDIR)/XMLReader.o

$(OBJDIR)/CSVWriter.o: $(SRCDIR)/CSVWriter.cpp $(INCDIR)/CSVWriter.h
	$(CXX) $(CXXFLAGS) $(SRCDIR)/CSVWriter.cpp -c -o $(OBJDIR)/CSVWriter.o

directories: $(BINDIR) $(OBJDIR) $(TESTBINDIR)

$(BINDIR):
	mkdir -p $(BINDIR)

$(OBJDIR):
	mkdir -p $(OBJDIR)
	
$(TESTBINDIR):
	mkdir -p $(TESTBINDIR)
	
clean:
	cd $(CSVLIBDIR); make clean ; cd ..
	rm -f $(CSVLIBDIR)/Makefile
	rm -f $(OBJDIR)/*.o
	rm -f $(BINDIR)/*
	rm -f $(TESTBINDIR)/*
