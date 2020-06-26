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

class predicateAnalysis {
public :
	string name;
	list<string> arguments;
	list<string> argumentType;

	//Values, only used in goal
	list<string> argumentValue;
	//Negated, goal and actions
	bool negated;

	predicateAnalysis(string AName): name(AName), negated(false)
		{}

	predicateAnalysis(predicateAnalysis* predicate): negated(false)
	{
		this->name = predicate->name;
		list<string>::iterator strIt = predicate->arguments.begin();
		for(; strIt != predicate->arguments.end(); strIt++)
		{
			this->arguments.push_back(*strIt);
		}

		strIt = predicate->argumentType.begin();
		for(; strIt != predicate->argumentType.end(); strIt++)
		{
			this->argumentType.push_back(*strIt);
		}
	}

};

class functionAnalysis {
public :
	string name;
	list<string> arguments;
	list<string> argumentType;

	//Values, only used in metrics
	list<string> argumentValue;
	//Increase or decrease, actions
	bool increase;

	functionAnalysis(): name(""), increase(true)
		{}

	functionAnalysis(string AName): name(AName), increase(true)
		{}

	functionAnalysis(functionAnalysis *function)
	{
		this->name = function->name;
		this->increase = function->increase;

		list<string>::iterator strIt = function->arguments.begin();
		for(; strIt != function->arguments.end(); strIt++)
		{
			this->arguments.push_back(*strIt);
		}

		strIt = function->argumentType.begin();
		for(; strIt != function->argumentType.end(); strIt++)
		{
			this->argumentType.push_back(*strIt);
		}
	}
};

class funcOperation {
public :
	functionAnalysis* function;
	list<functionAnalysis*> operators;
	list<double> weight;

	bool increase;
};

class actionAnalysis {
public :
	string name;
	list<string> arguments;
	list<string> argumentType;

	bool isMetricDependent;

	list<predicateAnalysis*> precondPred;
	list<functionAnalysis*> precondFunc;

	list<predicateAnalysis*> effectsPred;
	list<funcOperation*> effectsFuncOp;

	actionAnalysis(string AName): name(AName), isMetricDependent(false)
		{}

};

class objectTypeAnalysis {
public :
	string name;
	list<string> instances;
};

class metricAnalysis {
public :
	list<functionAnalysis*> functions;
	list<double> weight;
	bool minimise;
};

class goalAnalysis {
public :
	list<predicateAnalysis*> predicates;
};

class domainAnalysis
{
public:

	string domainRoute;
	string problemRoute;

    list<actionAnalysis*> actionList;
    list<predicateAnalysis*> predicateList;
    list<functionAnalysis*> functionList;
    list<objectTypeAnalysis*> objectList;
    metricAnalysis metric;
    goalAnalysis goal;

    //Planning info functions
    bool isMetricDependent(string action);

    //Read info functions
	void readDomainActions();
	void readDomainPredicates();
	void readDomainFunctions();
	void readProblemObjects();
	void readProblemMetric();
	void readProblemGoal();

	//Analysis functions
	void findMetricDependentActions();

private:
	functionAnalysis* findFunction(string argName);
	void readDomainActPrecond(ifstream* domainStream);
	void readDomainActEffects(ifstream* domainStream);

};



extern domainAnalysis DomainAnalysis;


