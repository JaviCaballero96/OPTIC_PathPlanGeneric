#include "globals.h"
#include "ptree.h"
#include <FlexLexer.h>
#include "instantiation.h"
#include "SimpleEval.h"
#include "DebugWriteController.h"
#include "typecheck.h"
#include "TIM.h"
#include "FuncAnalysis.h"
#include "FFSolver.h"
#include <assert.h>
#include <algorithm>
#include <sstream>
#include <fstream>
#include <list>
#include "ptree.h"
#include "numericanalysis.h"
#include "temporalanalysis.h"
#include "PreferenceHandler.h"
#include <algorithm>

using namespace TIM;
using namespace Inst;
using namespace VAL;
using namespace std;
using namespace Planner;

using std::cerr;
using std::ostringstream;
using std::endl;
using std::ifstream;

class actionAnalysis {
public :
	string name;
	list<string> arguments;
	list<string> argumentType;

	bool isMetricDependent;

	actionAnalysis(string AName): name(AName), isMetricDependent(false)
		{}

};

class predicateAnalysis {
public :
	string name;
	list<string> arguments;
	list<string> argumentType;

	predicateAnalysis(string AName): name(AName)
		{}

};

class functionAnalysis {
public :
	string name;
	list<string> arguments;
	list<string> argumentType;

	functionAnalysis(string AName): name(AName)
		{}

};

class domainAnalysis
{
public:

	string domainRoute;
	string problemRoute;

    list<actionAnalysis*> actionList;
    list<predicateAnalysis*> predicateList;
    list<functionAnalysis*> functionList;

	void readDomainActions();
	void readDomainPredicates();
	void readDomainFunctions();

};



extern domainAnalysis DomainAnalysis;


