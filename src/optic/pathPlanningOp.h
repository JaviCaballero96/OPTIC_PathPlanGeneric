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

#define PATHPLANDEPTH 5;

using namespace TIM;
using namespace Inst;
using namespace VAL;
using namespace std;
using namespace Planner;

using std::cerr;
using std::ostringstream;
using std::endl;
using std::ifstream;

class pathObj;
class DijkstraPath;

class Position{
	public:
		string x;
		string y;

		list<pathObj*> pathsFromHere;
		list<DijkstraPath*> shortPaths;

		double risk;

		Position():risk(0){}

		Position(const Position& pos);

		list<pathObj*>::const_iterator getPathBegin() const
		{
			return pathsFromHere.begin();
		}

		list<pathObj*>::const_iterator getPathEnd() const
		{
			return pathsFromHere.end();
		}

		list<DijkstraPath*>::const_iterator getShortPathBegin() const
		{
			return shortPaths.begin();
		}

		list<DijkstraPath*>::const_iterator getShortPathEnd() const
		{
			return shortPaths.end();
		}
};

class DijkstraPath{
	public:
		float cost;

		pathObj* path;
		list<Position*> fullPath;

		DijkstraPath() {}
		DijkstraPath(const DijkstraPath& dPath);
		DijkstraPath(DijkstraPath* dPath, pathObj* path);

		list<Position*>::const_iterator getFullPathBegin() const
		{
			return fullPath.begin();
		}

		list<Position*>::const_iterator getFullPathEnd() const
		{
			return fullPath.end();
		}

		pathObj* getPath()
		{
			return path;
		}

};

class pathObj
{
	private:
		Position origin;
		Position goal;
		float cost;

	public:
		pathObj(): cost(-1)
		{
		}

		pathObj(const pathObj& path)
		{
			cost = path.getCost();
			origin.x = path.getOrigin().x;
			origin.y = path.getOrigin().y;
			goal.x = path.getGoal().x;
			goal.y = path.getGoal().y;
		}

		pathObj(Position ori, Position go, float cst): cost(cst)
		{
			origin.x = ori.x;
			origin.y = ori.y;
			goal.x = go.x;
			goal.y = go.y;
		}

		pathObj(string orStr, string goStr): cost(-1)
		{
			origin.x = orStr.substr(orStr.find("c")+1,orStr.find("_")-1);
			origin.y = orStr.substr(orStr.find("_")+1,orStr.length());
			goal.x = goStr.substr(goStr.find("c")+1,goStr.find("_")-1);
			goal.y = goStr.substr(goStr.find("_")+1,goStr.length());
		}

		pathObj(string orStr, string goStr, float cst): cost(cst)
		{
			origin.x = orStr.substr(orStr.find("c")+1,orStr.find("_")-1);
			origin.y = orStr.substr(orStr.find("_")+1,orStr.length());
			goal.x = goStr.substr(goStr.find("c")+1,goStr.find("_")-1);
			goal.y = goStr.substr(goStr.find("_")+1,goStr.length());
		}

		Position getOrigin() const
		{
			return origin;
		}

		Position getGoal() const
		{
			return goal;
	    }

		float getCost() const
		{
			return cost;
		}

		void setCost(float cst)
		{
			cost = cst;
		}

};

class Agent
{
public:
	//Agent Name
	string name;

	//Origin and list of objectives
	Position* problemOrigin;
	list<Position*> problemGoal;

	//Position
	string atPosition;
	string initAtPosition;

	//Metric booleans
	bool distMetricDependent;
	bool riskMetricDependent;
	bool batteryMetricDependent;

	Agent(string nameArg): name(nameArg),distMetricDependent(false),
			riskMetricDependent(false), batteryMetricDependent(false){}
};

class pathPlanningOp
{
    private:
	    list <pathObj*> allPossiblePaths;
	    list <Position> positionList;
	    list <DijkstraPath*> allShortPaths;

	    list<Agent*> agents;

	    Position problemOrigin;
	    list<Position*>  commonProblemGoal;

	    //POSITIONS
	    void calculateShortPaths(list<Position>::iterator pos, bool debug);

	    DijkstraPath* performDijkstra(list<Position>::iterator ori,
	    		list<Position>::iterator go, bool debug);

	    bool isAShortPath(instantiatedOp* instOp, bool debug);

	    void searchOrigin(stringstream & effectStream);

	    double normalizeCost(double gCost, double pathCost,
	    		double riskCost, double batteryCost);

	public:

	    bool riskMetricActive;
	    bool distMetricActive;
	    bool batteryMetricActive;

	    double maxRisk;
	    double maxDist;
	    double maxBattery;

	    //ROUTES
		void createPathPlanRoutes(OpStore::iterator,OpStore::iterator,
				PNEStore::iterator,PNEStore::iterator);

		void storeOriginGoalsAgents(VAL::goal & p, VAL::effect_lists & e);

		void storeMetric(VAL::metric_spec* metric);

		void printAllRoutes();

		bool exists(string orStr, string goStr);

		pathObj* find(string orStr, string goStr);

		void calculateAllShortPaths();

		void printAllShortPaths();

		bool isValid(instantiatedOp* instOp);

		DijkstraPath* findShortPath(string origin, string goal);

		//POSITIONS
		void createPositionList();

		void printAllPositions();

		void storeDirectPathsPerPos();

		void printDirectPathsPerPos();

		double calculateCost(list<ActionSegment >::iterator actItr,
				const MinimalState & theState, bool distTermActive,double distCost,
				double gCost,string planString);

};

extern pathPlanningOp pathPlan;

