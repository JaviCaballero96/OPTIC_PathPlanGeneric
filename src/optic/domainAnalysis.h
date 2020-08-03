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

	//Position Predicate
	bool isPositionPredicate;

	predicateAnalysis(string AName): name(AName), negated(false), isPositionPredicate(false)
		{}

	predicateAnalysis(predicateAnalysis* predicate):
		negated(predicate->negated), isPositionPredicate(predicate->isPositionPredicate)
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

	list<predicateAnalysis*> precondPred;
	list<functionAnalysis*> precondFunc;

	list<predicateAnalysis*> effectsPred;
	list<funcOperation*> effectsFuncOp;

	// Boolean that indicates if this action will be used to optimize the metric
	bool isMetricDependent;
	// Boolean that indicates if this metric depèndent action aims to be optimized by its parameters
	bool isMetricOptimizer;

	// Boolean that indicates if this action will set a goal predicate
	bool isGoalAction;
	list<int> indexPredGoal;
	list<bool> isGoalArgument;
	int nActionsRequired;

	// Boolean that indicates if this action will set a goal predicate that blocks metric actions
	bool isFinalStateGoalAction;
	// Boolean that indicates if this action sets a predicate that must precede a goal action
	bool isRequiredGoalAction;
	list<string> goalsPreceded;
	list<int> nReqActGoalPreceded;

	// Boolean that indicates if this action sets a predicate
	// that allows a metric dependent action to execute
	bool isRequiredMetricAction;


	// Boolean that indicates if this action modifies or sets
	// a metric value, which can or not affect the metric
	bool isMetricFunctionSetterAction;
	bool isMetricFunctionModifierAction;
	bool isChangingActiveMetric;
	list<int> effectModifierIndex;
	int nOptimizationsPossible;
	int nOptimizationDone;

	//Movement action
	bool isMovementAction;

	actionAnalysis(string AName): name(AName), isMetricDependent(false), isMetricOptimizer(false), isGoalAction(false),
			isFinalStateGoalAction(false), nActionsRequired(0), isRequiredGoalAction(false), isRequiredMetricAction(false),
			isMovementAction(false), isMetricFunctionSetterAction(false), isMetricFunctionModifierAction(false),
			isChangingActiveMetric(false), nOptimizationsPossible(0), nOptimizationDone(0)
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

	bool timeMetricActive;

	list<string> agentRestrictions;
	list<double> agentRestrictionsValue;
	list<double> agentRestrictionsValuePlanning;
	double totalTimeMetric;
};

class goalAnalysis {
public :
	int nGoals, nFinalStateGoals;
	list<predicateAnalysis*> predicates;
};

class instanceAnalysis {
public :
	bool PredFunc;

	predicateAnalysis* predicate;

	functionAnalysis* function;
	double value;
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
    list<instanceAnalysis*> instancesList;
    goalAnalysis goal;
    metricAnalysis metric;
    double maxMetricEstimate;
    double minMetricEstimate;
    double metricNormalizer;


    //Planning info functions
    actionAnalysis* getAction(string action);
    bool isMetricDependent(string action);
    bool isGoalAction(string action);
    bool isMovementAction(string fullAction);
    void resetActionsState();
    void storeAgentMetricValue(string function, double value);
    bool isUnkownUseAction(actionAnalysis action);

    //Read info functions
	void readDomainActions();
	void readDomainPredicates();
	void readDomainFunctions();
	void readProblemObjects();
	void readProblemMetric();
	void readProblemGoal();
	void readProblemInit();

	//Analysis functions
	void analyseActions();
	void findMetricDependentActions();
	void findGoalActions();
	void analyseGoalActions();
	void analyseGoalTypes();
	void findPrecondGoalActions();
	void findPrecondMetricActions();
	void findPositionPredicate();
	void findMovementAction();
	void findMetricOptimizerActions();
	void calculatenOptimizationsPossible();
	void calculatenMaxMetricEstimate();
	void findMetricRestrictions();

private:
	functionAnalysis* findFunction(string argName);
	void readDomainActPrecond(ifstream* domainStream);
	void readDomainActEffects(ifstream* domainStream);

};



extern domainAnalysis DomainAnalysis;


