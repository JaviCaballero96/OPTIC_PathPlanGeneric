#include "pathPlanningOp.h"
#include "domainAnalysis.h"

#ifdef STOCHASTICDURATIONS
#include "StochasticDurations.h"
#endif


using namespace TIM;
using namespace Inst;
using namespace VAL;
using namespace std;

using std::cerr;
using std::ostringstream;
using std::endl;
using std::ifstream;

pathPlanningOp pathPlan;

void pathPlanningOp::createPathPlanRoutes(OpStore::iterator opBeginIt,
		OpStore::iterator opEndIt, PNEStore::iterator pneBeginIt,
		PNEStore::iterator pneEndIt)
{
	bool debug = false;

    this->maxRisk = 0;
    this->maxDist = 0;

	OpStore::iterator opIt;
	PNEStore::iterator pneIt;
	std::stringstream ss;


	if(debug) cout << "Instanciated Operations:" << endl;
	for (opIt = opBeginIt; opIt != opEndIt; ++opIt) {
			string opStr;
			ss << *(*opIt);
			opStr = ss.str();
			if(debug) cout << "   " << opStr  << endl;
	        //cout << opStr.substr(1,opStr.find(" ")) << endl;
	        if(strcmp(opStr.substr(1,opStr.find(" ")-1).c_str(), "robotbase_goingto") == 0)
	        {
	        	if(debug) cout << "      PathPlanning Op: ";
	        	do
	        	{
	        		opStr = opStr.substr(opStr.find(" ")+1, opStr.length());
	        	}
	        	while(strcmp(opStr.substr(0, 1).c_str(),"c"));
	        	string strOr = opStr.substr(0, opStr.find(" "));
	        	opStr = opStr.substr(opStr.find(" ")+1, opStr.length());
	        	string strGo = opStr.substr(0, opStr.find(" "));
	        	if(strGo.find(")") != string::npos)
	        	{
	        	    strGo = strGo.substr(0, strGo.length()-1);
	        	}
	        	if(debug) cout << "from " << strOr << " to " << strGo << endl;
	        	if(!exists(strOr,strGo))
	        	{
	        		if(debug) cout << "      New object" << endl;
	        		allPossiblePaths.push_back(new pathObj(strOr, strGo));
	        	}else
	        	{
	        		if(debug) cout << "      Already exists" << endl;
	        	}
	        }
	        ss.str(string());
	};

	if(debug) cout << "PNEs:" << endl;
	for (pneIt = pneBeginIt; pneIt != pneEndIt; ++pneIt) {
		string pneStr;
		PNE realPNE = *(*pneIt);
		float cost = EFT(realPNE.getHead())->getInitial(realPNE.begin(), realPNE.end()).second;
		ss << *(*pneIt);
		pneStr = ss.str();
		if(debug) cout << "   " << pneStr  << endl;
        if(strcmp(pneStr.substr(1,pneStr.find(" ")-1).c_str(), "distance_to_move") == 0)
        {
        	if(debug) cout << "      PathPlanning PNE: ";
        	do
        	{
        		pneStr = pneStr.substr(pneStr.find(" ")+1, pneStr.length());
        	}
        	while(strcmp(pneStr.substr(0, 1).c_str(),"c"));
        	string strOr = pneStr.substr(0, pneStr.find(" "));
        	pneStr = pneStr.substr(pneStr.find(" ")+1, pneStr.length());
        	string strGo = pneStr.substr(0, pneStr.find(" "));
        	if(strGo.find(")") != string::npos)
        	{
        		strGo = strGo.substr(0, strGo.length()-1);
        	}
        	if(debug) cout << "from " << strOr << " to " << strGo << " -- cost: " << cost << endl;
        	if(!exists(strOr,strGo))
        	{
        		if(debug) cout << "      New object" << endl;
        		allPossiblePaths.push_back(new pathObj(strOr, strGo, cost));
				if(this->maxDist < cost)
				{
					this->maxDist = cost;
				}
        	}else
        	{
        		if(debug) cout << "      Already exists, setting up cost." << cost << endl;
        		pathObj* FoundPNE = find(strOr, strGo);
				if(this->maxDist < cost)
				{
					this->maxDist = cost;
				}
        		if(FoundPNE != NULL) FoundPNE->setCost(cost);
        	}

        }
        ss.str(string());
	};
}

void pathPlanningOp::storeOriginGoalsAgents(VAL::goal & g,
		VAL::effect_lists & e)
{
	//Store Agents

	//Store Origin
	stringstream originStrStream;
	stringstream originStrStreamAdd;
	stringstream originStrStreamAssign;

	streambuf * old2 = cout.rdbuf(originStrStreamAdd.rdbuf());
	cout << e.add_effects << endl;
	cout.rdbuf(old2);
	searchOrigin(originStrStreamAdd);
	originStrStream.str("");
	old2 = cout.rdbuf(originStrStreamAssign.rdbuf());
	cout << e.assign_effects << endl;
	cout.rdbuf(old2);
	searchOrigin(originStrStreamAssign);
	originStrStream.str("");
	old2 = cout.rdbuf(originStrStream.rdbuf());
	cout << e.cond_assign_effects << endl;
	cout.rdbuf(old2);
	searchOrigin(originStrStream);
	originStrStream.str("");
	old2 = cout.rdbuf(originStrStream.rdbuf());
	cout << e.cond_effects << endl;
	cout.rdbuf(old2);
	searchOrigin(originStrStream);
	originStrStream.str("");
	old2 = cout.rdbuf(originStrStream.rdbuf());
	cout << e.del_effects << endl;
	cout.rdbuf(old2);
	searchOrigin(originStrStream);
	originStrStream.str("");
	old2 = cout.rdbuf(originStrStream.rdbuf());
	cout << e.forall_effects << endl;
	cout.rdbuf(old2);
	searchOrigin(originStrStream);
	originStrStream.str("");
	old2 = cout.rdbuf(originStrStream.rdbuf());
	cout << e.timed_effects << endl;
	cout.rdbuf(old2);
	searchOrigin(originStrStream);
	originStrStream.str("");

	//Store Goal
	//cout << g << endl;
	stringstream goalStrStream;
	string line, agentArgument = "", goal;

	streambuf * old = cout.rdbuf(goalStrStream.rdbuf());
	g.write(goalStrStream);
	cout.rdbuf( old );

	//Read goal structure and extract goal loc
	bool storeGoal = false;
    while (getline(goalStrStream, line)) {
        //cout << line << endl;
		if(line.find("(prop)") != string::npos)
		{
			//cout << endl << "New goal: ";
			getline(goalStrStream, line);
			getline(goalStrStream, line);
			getline(goalStrStream, line);
			line.erase(remove_if(line.begin(), line.end(), ::isspace), line.end());
			line = line.substr(line.find(":")+1,line.length());
			//cout << line;
			if(line.find("robotbase_at") != string::npos ||
				    line.find("visited") != string::npos ||
				    line.find("cameras_picture") != string::npos ||
					line.find("communication_transmittedp") != string::npos)
			{
				storeGoal = true;
				goal = line;
			}else
			{
				storeGoal = false;
				goal = "";
			}
		}

		if(line.find("(symbol)") != string::npos)
		{
			string arg;
			getline(goalStrStream, line);
			line.erase(remove_if(line.begin(), line.end(), ::isspace), line.end());
			line = line.substr(line.find(":")+1,line.find("[")-5);
			//cout << " " << line << "(";
			arg = line;
			while(getline(goalStrStream, line))
			{
				if(line.find("type:") != string::npos)
				{
					if(line.find("NULL") != string::npos)
					{
						//cout << ") ";
						break;
					}
					getline(goalStrStream, line);
					getline(goalStrStream, line);
					line.erase(remove_if(line.begin(), line.end(), ::isspace), line.end());
					line = line.substr(line.find(":")+1,line.find("[")-5);
					//cout << "/" << line;
					string type = line;
					if(type.find("agent") != string::npos)
					{
						agentArgument = arg;
					}
					if(arg.find("c") != string::npos &&
						arg.find("_") != string::npos &&
						type.find("loc") != string::npos &&
						storeGoal)
					{
						storeGoal = false;
						if(agentArgument == "")
						{
							Position* aux1 = new Position();
							aux1->x = arg.substr(arg.find("c")+1,arg.find("_")-1);
							aux1->y = arg.substr(arg.find("_")+1,arg.length());
							commonProblemGoal.push_back(aux1);
						}else
						{
							list<Agent*>::iterator agentIt = agents.begin();
							for(;agentIt != agents.end();agentIt++)
							{
								if((*agentIt)->name == agentArgument)
								{
									Position* posAux = new Position();
									posAux->x = arg.substr(arg.find("c")+1,arg.find("_")-1);
									posAux->y = arg.substr(arg.find("_")+1,arg.length());
									(*agentIt)->problemGoal.push_back(posAux);
								}
							}
							agentArgument = "";
						}
					}

				}
			}
		}
    }
    //cout << endl;
}

void pathPlanningOp::storeMetric(VAL::metric_spec* metric)
{
	//cout << *metric << endl;

	stringstream metricStrStream;
	string line;

	streambuf * old = cout.rdbuf(metricStrStream.rdbuf());
	metric->write(metricStrStream);
	cout.rdbuf( old );

	//cout << "Metric: " << endl;

	this->riskMetricActive = false;
	this->distMetricActive = false;
	this->batteryMetricActive= false;

	while (getline(metricStrStream, line))
	{
		if(line.find("func_term") != string::npos)
		{
			getline(metricStrStream, line);
			getline(metricStrStream, line);
			getline(metricStrStream, line);

			line.erase(remove_if(line.begin(), line.end(), ::isspace), line.end());
			line = line.substr(line.find(":")+1,line.length());
			string argName = line;
			string agentName;

			//cout << "   " << argName;

			getline(metricStrStream, line);
			getline(metricStrStream, line);
			getline(metricStrStream, line);

			if(line.find("(symbol)") != string::npos)
			{
				string arg;
				getline(metricStrStream, line);
				line.erase(remove_if(line.begin(), line.end(), ::isspace), line.end());
				line = line.substr(line.find(":")+1,line.find("[")-5);
				//cout << " " << line << "(";
				arg = line;
				while(getline(metricStrStream, line))
				{
					if(line.find("type:") != string::npos)
					{
						if(line.find("NULL") != string::npos)
						{
							//cout << ") " << endl;
							break;
						}
						getline(metricStrStream, line);
						getline(metricStrStream, line);
						line.erase(remove_if(line.begin(), line.end(), ::isspace), line.end());
						line = line.substr(line.find(":")+1,line.find("[")-5);
						//cout << "/" << line;
						string type = line;
						if(type.find("agent") != string::npos)
						{
							if(argName == "dist"){
								this->distMetricActive = true;
								list<Agent*>::iterator agentIt = agents.begin();
								for(; agentIt != agents.end(); agentIt++)
								{
									if((*agentIt)->name == arg)
									{
										(*agentIt)->distMetricDependent = true;
									}
								}
							}
							else if(argName == "risk"){
								this->riskMetricActive = true;
								list<Agent*>::iterator agentIt = agents.begin();
								for(; agentIt != agents.end(); agentIt++)
								{
									if((*agentIt)->name == arg)
									{
										(*agentIt)->riskMetricDependent = true;
									}
								}
							}else if(argName == "battery"){
								this->batteryMetricActive = true;
								list<Agent*>::iterator agentIt = agents.begin();
								for(; agentIt != agents.end(); agentIt++)
								{
									if((*agentIt)->name == arg)
									{
										(*agentIt)->batteryMetricDependent = true;
									}
								}
							}
						}
					}
				}
			}
		}
	}
}

void pathPlanningOp::searchOrigin(stringstream & effectStream)
{
	string line, agentArgument = "", locArgument = "";
	bool storeAgent = false;
	bool storeRisk = false;
	bool storeGoal = false;
    while (getline(effectStream, line)) {
        //cout << line << endl;
		if(line.find("(prop)") != string::npos || line.find("(func_term)") != string::npos)
		{
			//cout << endl << "New statement: ";
			getline(effectStream, line);
			getline(effectStream, line);
			getline(effectStream, line);
			line.erase(remove_if(line.begin(), line.end(), ::isspace), line.end());
			line = line.substr(line.find(":")+1,line.length());
			//cout << line;
			if(line.find("free") != string::npos)
			{
				storeAgent = true;
			}else
			{
				storeAgent = false;
			}
			if(line.find("robotbase_at") != string::npos)
			{
				storeGoal = true;
			}else
			{
				storeGoal = false;
			}
			if(line.find("posrisk") != string::npos)
			{
				storeRisk = true;
			}else
			{
				storeRisk = false;
			}
		}

		if(line.find("(symbol)") != string::npos)
		{
			string arg;
			getline(effectStream, line);
			line.erase(remove_if(line.begin(), line.end(), ::isspace), line.end());
			line = line.substr(line.find(":")+1,line.find("[")-5);
			//cout << " " << line << "(";
			arg = line;
			bool keep = true;
			while(getline(effectStream, line) && keep)
			{
				if(line.find("type:") != string::npos)
				{
					if(line.find("NULL") != string::npos)
					{
						//cout << ") ";
						break;
					}
					getline(effectStream, line);
					getline(effectStream, line);
					line.erase(remove_if(line.begin(), line.end(), ::isspace), line.end());
					line = line.substr(line.find(":")+1,line.find("[")-5);
					//cout << "/" << line;
					string type = line;
					if(type.find("agent") != string::npos &&
						storeAgent)
					{
						bool alreadyExists = false;
						list<Agent*>::iterator agentIt = agents.begin();
						for(;agentIt != agents.end();agentIt++)
						{
							if((*agentIt)->name == arg)
							{
								alreadyExists = true;
							}
						}
						if(!alreadyExists)
						{
							agents.push_back(new Agent(arg));
						}
					}else if(type.find("agent") != string::npos)
					{
						agentArgument = arg;
					}
					if(type.find("loc") != string::npos)
					{
						locArgument = arg;
					}
					if(arg.find("c") != string::npos &&
							arg.find("_") != string::npos &&
							type.find("loc") != string::npos &&
							storeGoal)
					{
						storeGoal = false;
						if(agentArgument == "")
						{
							problemOrigin.x = arg.substr(arg.find("c")+1,arg.find("_")-1);
							problemOrigin.y = arg.substr(arg.find("_")+1,arg.length());
						}else
						{
							list<Agent*>::iterator agentIt = agents.begin();
							for(;agentIt != agents.end();agentIt++)
							{
								if((*agentIt)->name == agentArgument)
								{
									Position* posAux = new Position();
									posAux->x = arg.substr(arg.find("c")+1,arg.find("_")-1);
									posAux->y = arg.substr(arg.find("_")+1,arg.length());
									(*agentIt)->problemOrigin = posAux;
									(*agentIt)->initAtPosition = arg;
								}
							}
							agentArgument = "";
							locArgument = "";
						}

					}
					else if(arg.find("c") != string::npos &&
						arg.find("_") != string::npos &&
						type.find("loc") != string::npos &&
						storeRisk)
					{
						storeRisk = false;
						while(getline(effectStream, line) && keep)
						{
							if(line.find("val") != string::npos)
							{
								line.erase(remove_if(line.begin(), line.end(), ::isspace), line.end());
								line = line.substr(line.find(":")+1,line.length());

								list<Position>::iterator posIt = positionList.begin();
								for(;posIt != positionList.end() && keep; posIt++)
			          			{
									string argX = locArgument.substr(locArgument.find("c")+1,locArgument.find("_")-1);
									string argY = locArgument.substr(locArgument.find("_")+1,locArgument.length());
									if(posIt->x == argX && posIt->y == argY)
									{
										posIt->risk = atof(line.c_str());
										if(this->maxRisk < atof(line.c_str()))
										{
											this->maxRisk = atof(line.c_str());
										}
										agentArgument = "";
										locArgument = "";
										keep = false;
									}
								}
							}
						}
					}
				}
			}
		}
    }
    //cout << endl;
}

bool pathPlanningOp::exists(string ori, string go)
{
	bool exists = false;
	list<pathObj*> :: iterator pathIt;

	for(pathIt = allPossiblePaths.begin();
			pathIt != allPossiblePaths.end(); pathIt++)
	{
		string origin = "c" + (*pathIt)->getOrigin().x  + "_" + (*pathIt)->getOrigin().y;
		string goal = "c" + (*pathIt)->getGoal().x  + "_" + (*pathIt)->getGoal().y;

		if(ori == origin && go == goal)
		{
			exists = true;
			break;
		}

	}
	return exists;
}

pathObj* pathPlanningOp::find(string ori, string go)
{
	list<pathObj*> :: iterator pathIt;

	for(pathIt = allPossiblePaths.begin();
			pathIt != allPossiblePaths.end(); pathIt++)
	{
		string origin = "c" + (*pathIt)->getOrigin().x  + "_" + (*pathIt)->getOrigin().y;
		string goal = "c" + (*pathIt)->getGoal().x  + "_" + (*pathIt)->getGoal().y;

		if(ori == origin && go == goal)
		{
			return (*pathIt);
		}

	}
	return NULL;
}

void pathPlanningOp::printAllRoutes()
{
	list<pathObj*> :: iterator pathIt;
	int i = 1;
	for(pathIt = allPossiblePaths.begin();
				pathIt != allPossiblePaths.end(); pathIt++)
	{
		/*cout << i << "- From (" << (*pathIt)->getOrigin().x << "," << (*pathIt)->getOrigin().y <<
				") to (" << (*pathIt)->getGoal().x << "," << (*pathIt)->getGoal().y <<
				") -- cost: " << (*pathIt)->getCost() << endl;*/
		i++;
	}
}

void pathPlanningOp::createPositionList()
{
	list<pathObj*> :: iterator pathIt;

	for(pathIt = allPossiblePaths.begin();
				pathIt != allPossiblePaths.end(); pathIt++)
	{
		bool exists = false;

		list<Position> :: iterator posIt;
		for(posIt = positionList.begin();
				posIt != positionList.end(); posIt++)
		{
			if((*pathIt)->getOrigin().x == posIt->x && (*pathIt)->getOrigin().y == posIt->y)
			{
				exists = true;
			}
		}

		if(!exists)
		{
			Position pos;
			pos.x = (*pathIt)->getOrigin().x;
			pos.y = (*pathIt)->getOrigin().y;
			positionList.push_back(pos);
		}

		exists = false;
		for(posIt = positionList.begin();
				posIt != positionList.end(); posIt++)
		{
			if((*pathIt)->getGoal().x == posIt->x && (*pathIt)->getGoal().y == posIt->y)
			{
				exists = true;
			}
		}

		if(!exists)
		{
			Position pos;
			pos.x = (*pathIt)->getGoal().x;
			pos.y = (*pathIt)->getGoal().y;
			positionList.push_back(pos);
		}
	}
}

void pathPlanningOp::printAllPositions()
{
	cout << "Positions: " << endl;
	list<Position> :: iterator posIt;
	for(posIt = positionList.begin();
			posIt != positionList.end(); posIt++)
	{
		cout << "   (" << posIt->x << "," << posIt->y << ")" << endl;
	}
}

void pathPlanningOp::storeDirectPathsPerPos()
{
	list<Position> :: iterator posIt1, posIt2;
	for(posIt1 = positionList.begin();
			posIt1 != positionList.end(); posIt1++)
	{
		string ori = "c" + posIt1->x  + "_" + posIt1->y;

		for(posIt2 = positionList.begin();
					posIt2 != positionList.end(); posIt2++)
		{
			string goal = "c" + posIt2->x  + "_" + posIt2->y;
			if(exists(ori,goal))
			{
				pathObj* path = new pathObj(*(find(ori,goal)));
				float cost = path->getCost();
				if(cost > 0)
					posIt1->pathsFromHere.push_back(path);
			}
		}
	}
}

void pathPlanningOp::printDirectPathsPerPos()
{
	list<Position> :: iterator posIt;
	for(posIt = positionList.begin();
			posIt != positionList.end(); posIt++)
	{
		cout << "Position: (" << posIt->x << "," << posIt->y << "):" << endl;

		list<pathObj*> :: iterator pathIt;
		for(pathIt = posIt->pathsFromHere.begin();
			pathIt != posIt->pathsFromHere.end(); pathIt++)
		{
			cout << "    (" << (*pathIt)->getOrigin().x << "," << (*pathIt)->getOrigin().y << ") --> ("
					<<"(" << (*pathIt)->getGoal().x << "," << (*pathIt)->getGoal().y << ")" <<
					" : " << (*pathIt)->getCost() << endl;
		}
	}
}

void pathPlanningOp::calculateAllShortPaths()
{
	bool debug = false;

	list<Position> :: iterator posIt;
		for(posIt = positionList.begin();
				posIt != positionList.end(); posIt++)
	{
			calculateShortPaths(posIt, debug);
	}
}

void pathPlanningOp::calculateShortPaths(list<Position>::iterator origin, bool debug)
{
	list<Position> :: iterator posIt;
			for(posIt = positionList.begin();
					posIt != positionList.end(); posIt++)
	{
		DijkstraPath* fullPath = performDijkstra(origin, posIt, debug);

		if (debug) cout << "Shortest path found from (" << fullPath->path->getOrigin().x <<
				"," << fullPath->path->getOrigin().y << ") to (" << fullPath->path->getGoal().x <<
				"," << fullPath->path->getGoal().y << ") costs: " << fullPath->cost << endl;
		if(fullPath->cost > 0)
		{
			allShortPaths.push_back(new DijkstraPath(*fullPath));
		}
	}
}

DijkstraPath* pathPlanningOp::performDijkstra(list<Position>::iterator ori, list<Position>::iterator go, bool debug)
{
	int depth = 0 ,maxDepth = 30;

	DijkstraPath* fullPath = new DijkstraPath();;
	list<pathObj> alreadyVisited;
	list<DijkstraPath*> nodes;
	list<pathObj*>::iterator pathIt;
	list<DijkstraPath*>::iterator dijksIt;
	list<Position>::iterator posIt;

	if(ori->x == go->x && ori->y == go->y)
	{
		DijkstraPath* newNode = new DijkstraPath();
		newNode->cost = -1;
		newNode->path = new pathObj(*ori,*go,-1);
		return newNode;
	}

	for(pathIt = ori->pathsFromHere.begin();
			pathIt != ori->pathsFromHere.end(); pathIt++)
	{
		DijkstraPath* newNode = new DijkstraPath();
		newNode->fullPath.push_back(new Position((*pathIt)->getOrigin()));
		newNode->fullPath.push_back(new Position((*pathIt)->getGoal()));
		newNode->path = new pathObj(**pathIt);
		newNode->cost = (*pathIt)->getCost();
		nodes.push_back(newNode);
	}

	while(true) //Search
	{
		if(depth > maxDepth)
		{
			DijkstraPath* newNode = new DijkstraPath();
			newNode->cost = -1;
			newNode->path = new pathObj(*ori,*go,-1);
			return newNode;
		}

		//Find less cost path from nodes list

		float minCost=99999;
		DijkstraPath* nodeToExpand;
		for(dijksIt = nodes.begin();
				dijksIt != nodes.end(); dijksIt++)
		{
			if((*dijksIt)->cost < minCost)
			{
				minCost = (*dijksIt)->cost;
				nodeToExpand = *dijksIt;
			}
		}

		if(nodeToExpand->getPath()->getGoal().x == go->x &&
				nodeToExpand->getPath()->getGoal().y == go->y)
		{
			//End of search
			return nodeToExpand;
		}

		for(posIt = positionList.begin(); posIt != positionList.end();
				posIt++)
		{
			if(posIt->x == nodeToExpand->path->getGoal().x &&
					posIt->y == nodeToExpand->path->getGoal().y)
				break;
		}

		for(pathIt = posIt->pathsFromHere.begin();
				pathIt != posIt->pathsFromHere.end(); pathIt++)
		{
			if(nodeToExpand->path->getOrigin().x != (*pathIt)->getGoal().x ||
					nodeToExpand->path->getOrigin().y != (*pathIt)->getGoal().y)
			nodes.push_back(new DijkstraPath(nodeToExpand,*pathIt));
		}

		nodes.remove(nodeToExpand);

		depth++;
	}

	return fullPath;
}

void pathPlanningOp::printAllShortPaths()
{
	list<DijkstraPath*>::iterator dPathIt;

	for(dPathIt = allShortPaths.begin();
			dPathIt != allShortPaths.end(); dPathIt++)
	{
		list<Position*>::iterator posIt;

		cout << "From (" << (*dPathIt)->path->getOrigin().x <<
			"," << (*dPathIt)->path->getOrigin().y << ") to (" << (*dPathIt)->path->getGoal().x <<
			"," << (*dPathIt)->path->getGoal().y << ") costs: " << (*dPathIt)->cost << endl;

		cout << "  Route:";
		for(posIt = (*dPathIt)->fullPath.begin();
				posIt != (*dPathIt)->fullPath.end(); posIt++)
		{
			cout << " (" << (*posIt)->x << "," << (*posIt)->y << ")";
		}
		cout << endl;
	}
}

DijkstraPath* pathPlanningOp::findShortPath(string ori, string go)
{
	list<DijkstraPath*> :: iterator dPathIt;

	if(ori == go)
	{
		DijkstraPath *dPathAux = new DijkstraPath();
		dPathAux->cost = 0;
		return dPathAux;
	}
	for(dPathIt = allShortPaths.begin();
			dPathIt != allShortPaths.end(); dPathIt++)
	{
		string origin = "c" + (*dPathIt)->getPath()->getOrigin().x  + "_" + (*dPathIt)->getPath()->getOrigin().y;
		string goal = "c" + (*dPathIt)->getPath()->getGoal().x  + "_" + (*dPathIt)->getPath()->getGoal().y;

		if(ori == origin && go == goal)
		{
			return (*dPathIt);
		}

	}
	return NULL;
}

bool pathPlanningOp::isValid(instantiatedOp* instOp)
{
	bool debug = false;
	stringstream ss;

	ss << *instOp;
	if (debug) cout << "Checking if " << ss.str()
			<< " is a path planning operation." << endl;

	if(ss.str().substr(1,ss.str().find(" ")-1) == "robotbase_goingto")
	{
		if (debug) cout << "	It is, checking if it is a short path." << endl;
		return isAShortPath(instOp, debug);
	}else
	{
		if (debug) cout << "	It is not, returning." << endl;
		return true;
	}

}

bool pathPlanningOp::isAShortPath(instantiatedOp* instOp, bool debug)
{
	bool isShort = false;
	stringstream ss;
	ss << *instOp;
	string instOpS = ss.str();
	list<DijkstraPath*>::iterator dPathIt;

	do
	{
		instOpS = instOpS.substr(instOpS.find(" ")+1, instOpS.length());
	}
	while(strcmp(instOpS.substr(0, 1).c_str(),"c"));

	string origin = instOpS.substr(0, instOpS.find(" "));
	instOpS = instOpS.substr(instOpS.find(" ")+1, instOpS.length());
	string goal = instOpS.substr(0, instOpS.find(" "));

	if(debug) cout << "		(" << origin << "," << goal << endl;

	string xOri = origin.substr(1,origin.find("_")-1);
	string yOri = origin.substr(origin.find("_")+1,origin.length());
	string xGo = goal.substr(1,goal.find("_")-1);
	string yGo = goal.substr(goal.find("_")+1,goal.length());
	if (yGo.substr(yGo.length()-1, yGo.length()) == ")")
		yGo = yGo.substr(0,yGo.length()-1);

	for(dPathIt = allShortPaths.begin();
			dPathIt != allShortPaths.end(); dPathIt++)
	{
		if((*dPathIt)->path->getOrigin().x == xOri &&
		   (*dPathIt)->path->getOrigin().y == yOri &&
		   (*dPathIt)->path->getGoal().x == xGo &&
		   (*dPathIt)->path->getGoal().y == yGo
		   )
		{
			if((*dPathIt)->fullPath.size() <= 2)
			{
				if(debug) cout << "		Short path, keeping it" << endl;
				isShort = true;
				break;
			}
		}
	}

	if(!isShort)
		if(debug) cout << "		It is not a short path" << endl;

	return isShort;
}

Position::Position(const Position& pos): x(pos.x), y(pos.y), risk(0)
{
			list<pathObj*>::const_iterator pathIt;
			list<DijkstraPath*>::const_iterator dPathIt;

			for(pathIt = pos.getPathBegin();
					pathIt != pos.getPathEnd(); pathIt++)
			{
				this->pathsFromHere.push_back(new pathObj(**pathIt));
			}

			for(dPathIt = pos.getShortPathBegin();
					dPathIt != pos.getShortPathEnd(); dPathIt++)
			{
				this->shortPaths.push_back(new DijkstraPath(**dPathIt));
			}
}

DijkstraPath::DijkstraPath(const DijkstraPath& dPath)
{
	list<Position*>::const_iterator posIt;
	cost = dPath.cost;

	this->path = new pathObj(*(dPath.path));

	for(posIt = dPath.getFullPathBegin();
		posIt != dPath.getFullPathEnd(); posIt++)
	{
		this->fullPath.push_back(new Position(**posIt));
	}
}

DijkstraPath::DijkstraPath(DijkstraPath* dPath, pathObj* path)
{
	list<Position*>::const_iterator posIt;

	this->path = new pathObj(*(dPath->path));

	for(posIt = dPath->getFullPathBegin();
		posIt != dPath->getFullPathEnd(); posIt++)
	{
		this->fullPath.push_back(new Position(**posIt));
	}
	this->fullPath.push_back(new Position(path->getGoal()));

	this->path = new pathObj(dPath->path->getOrigin(),path->getGoal(),-1);

	this->cost = dPath->cost + path->getCost();
}

double pathPlanningOp::calculateCost(list<ActionSegment >::iterator actItr,
		const MinimalState & theState, bool distTermActive,double distCost,
		double gCost,string planString)
{
	stringstream auxStream;
	string actionString;
	string strOr, strGo, line, agentName="";
	list<string> alreadyVisited;
	double costOpticDistanceFree = 0, minCost = 99999;
	double riskCost = 0, pathCost= 0, batteryCost = 0;
	bool riskActionDependent = false;
	string goalChoosed;
	Agent *agent;
	istringstream planStream(planString);

	//cout << "Considering action: " << *(actItr->first) << endl;
	//cout << "In the state: " << endl;

	//Reset agents state
	list<Agent*>::iterator agentIt = agents.begin();
	for(;agentIt != agents.end(); agentIt++)
	{
		(*agentIt)->atPosition = (*agentIt)->initAtPosition;
	}
	//Read current plan

	while(getline(planStream, line))
	{
		//cout << line << endl;
		string action;
		if(DomainAnalysis.isMovementAction(line)
				&& line.find("end") != string::npos)
		{
			Agent* actionAgent = NULL;
			action = line;
	    	do
	    	{
	    		string argument = action.substr(0, action.find(" "));
	    		action = action.substr(action.find(" ")+1, action.length());

	    		agentIt = agents.begin();
				for(;agentIt != agents.end() && actionAgent == NULL; agentIt++)
				{
					if(argument.find((*agentIt)->name) != string::npos)
					{
						actionAgent = *agentIt;
						break;
					}
				}
	    	}
	    	while(strcmp(action.substr(0, 1).c_str(),"c"));

	    	string strOrigin = action.substr(0, action.find(" "));
	    	action = action.substr(action.find(" ")+1, action.length());
	    	string strGoal = action.substr(0, action.find(" "));

	    	if(strGoal.find(")") != string::npos)
	    	{
	    		strGoal = strGoal.substr(0, strGoal.find(")"));
	    	}

	    	actionAgent->atPosition = strGoal;
	    	if(!(std::find(alreadyVisited.begin(), alreadyVisited.end(), strGoal) != alreadyVisited.end()))
	    	{
	    		alreadyVisited.push_back(strGoal);
	    	}
	    }
	}

	auxStream << *(actItr->first);
	actionString = auxStream.str();
	auxStream.str("");

	//If the action is movement
	if(DomainAnalysis.isMovementAction(actionString))
	{
		//Distance metric

		//cout << "Considering applying a pathplanning op" <<  endl;
		riskActionDependent = true;

    	do
    	{
    		string argument = actionString.substr(0, actionString.find(" "));
    		actionString = actionString.substr(actionString.find(" ")+1, actionString.length());
    		list<Agent*>::iterator agentIt;
    		agentIt = agents.begin();
    		for(;agentIt != agents.end() && agentName == ""; agentIt++)
    		{
    			if(argument.find((*agentIt)->name) != string::npos)
    			{
    				agentName = (*agentIt)->name;
    				agent = (*agentIt);
    			}
    		}
    	}
    	while(strcmp(actionString.substr(0, 1).c_str(),"c"));

    	strOr = actionString.substr(0, actionString.find(" "));
    	actionString = actionString.substr(actionString.find(" ")+1, actionString.length());
    	string strGo = actionString.substr(0, actionString.find(" "));
    	if(strGo.find(")") != string::npos)
    	{
    	    strGo = strGo.substr(0, strGo.length()-1);
    	}

    	//cout << "from " << strOr << " to " << strGo << endl;

    	pathObj *directPath = find(strOr, strGo);
    	if(directPath == NULL)
    	{
    		cout << "Error obtaining direct path" << endl;
    		return -1;
    	}

    	//cout << "Optic cost = " << gCost << " and distance = " <<  directPath->getCost() << endl;
    	if(agent->distMetricDependent && gCost != 0)
    	{
    		costOpticDistanceFree = gCost - directPath->getCost();
    	}else{
    		costOpticDistanceFree = gCost;
    	}

    	//Iterate over agent dependent objectives
    	int nAlreadyVisited = 0;
    	list<Position*>::iterator posIt = agent->problemGoal.begin();
    	for(; posIt != agent->problemGoal.end(); posIt++)
    	{
    		string strProblemGoal = "c" + (*posIt)->x + "_" + (*posIt)->y;
        	if(std::find(alreadyVisited.begin(), alreadyVisited.end(), strGo) != alreadyVisited.end())
        	{
        		//cout << "Skyping" << strProblemGoal << " goal, already visited." << endl;
        		nAlreadyVisited++;
        		continue;
        	}

			DijkstraPath *dijksPath = findShortPath(strGo,strProblemGoal);
			if(dijksPath == NULL)
			{
				cout << "Error obtaining short path" << endl;
				return -1;
			}

			if(dijksPath->cost < minCost)
			{
				goalChoosed = strProblemGoal;
				minCost = dijksPath->cost;
			}
    	}

    	//Iterate over common objectives
    	posIt = commonProblemGoal.begin();
    	for(; posIt != commonProblemGoal.end(); posIt++)
    	{
    		string strProblemGoal = "c" + (*posIt)->x + "_" + (*posIt)->y;
        	if(std::find(alreadyVisited.begin(), alreadyVisited.end(), strProblemGoal) != alreadyVisited.end())
        	{
        		//cout << "Skyping " << strProblemGoal << " goal, already visited." << endl;
        		nAlreadyVisited++;
        		continue;
        	}

			DijkstraPath *dijksPath = findShortPath(strGo,strProblemGoal);
			if(dijksPath == NULL)
			{
				cout << "Error obtaining short path" << endl;
				return -1;
			}

			if(dijksPath->cost < minCost)
			{
				goalChoosed = strProblemGoal;
				minCost = dijksPath->cost;
			}
    	}

    	//cout << "Distance from proposed new position to goal " << goalChoosed << " = " << minCost << endl;

    	if(nAlreadyVisited >= (agent->problemGoal.size() + commonProblemGoal.size()))
    	{
    		gCost = costOpticDistanceFree + directPath->getCost();
    		pathCost = directPath->getCost();
    	}
    	else if(agent->distMetricDependent && this->distMetricActive)
    	{
    		gCost = costOpticDistanceFree + minCost + directPath->getCost() / 2;
    		pathCost = minCost + directPath->getCost() / 2;
    	}else if(this->distMetricActive)
    	{
    		gCost = costOpticDistanceFree + minCost;
    		pathCost = minCost;
    	}

    	// Risk metric
    	if(agent->riskMetricDependent && riskActionDependent)
    	{
    		//Risk associated to the new position
	    	list<Position>::iterator possIt = positionList.begin();
	    	for(; possIt != positionList.end(); possIt++)
	    	{
	    		string strPos = "c" + possIt->x + "_" + possIt->y;
	    		if(strPos == strGo)
	    		{
	    			riskCost = possIt->risk;
	    			break;
	    		}
	    	}
	    	gCost = gCost - riskCost;

    	}

    	if(this->riskMetricActive && riskActionDependent){

	    	//Risk associated to distance between agents
	    	list<Agent*>::iterator agentIt = agents.begin();
			for(;agentIt != agents.end(); agentIt++)
			{
				if((*agentIt)->name == agent->name)
					continue;

				pathObj *directPath = NULL;
				if((*agentIt)->atPosition != "")
				{
					directPath = find((*agentIt)->atPosition, strGo);
				}else
				{
					string strOriAgent = "c" + (*agentIt)->problemOrigin->x + "_" + (*agentIt)->problemOrigin->y;
					directPath = find(strOriAgent, strGo);
				}

				if(directPath == NULL)
				{
					riskCost += 0;
				}
				else if(directPath->getCost() < 10)
				{
					riskCost += 0;
				}
			}

    	}

    	gCost += riskCost;

    	//cout << "PathPlan Operation New Cost is = " << gCost << endl;

	}

	return gCost;
    //return normalizeCost(gCost, pathCost, riskCost, batteryCost);
}

double pathPlanningOp::normalizeCost(double gCost, double pathCost,
		double riskCost, double batteryCost)
{
	double maxCost = 0;
	int activeMetrics = 0;

	if(this->distMetricActive)
	{
		maxCost = this->maxDist + this->maxDist / 2;
		pathCost = ((pathCost * 0.3333) / maxCost);
		activeMetrics++;
	}
	else
	{
		pathCost = 0;
	}
	if(this->batteryMetricActive)
	{
		maxCost = this->maxBattery * this->maxDist;
		batteryCost = ((batteryCost * 0.3333) / maxCost);
		activeMetrics++;
	}
	else
	{
		batteryCost = 0;
	}
	if(this->riskMetricActive)
	{
		maxCost = this->maxRisk;
		riskCost = ((riskCost * 0.3333) / maxCost);
		activeMetrics++;
	}else
	{
		riskCost = 0;
	}

	switch(activeMetrics)
	{
		case 1:
			gCost = (riskCost + pathCost + batteryCost)/0.3333;
			break;
		case 2:
			gCost = (riskCost + pathCost + batteryCost)/0.6667;
			break;
	}

	return gCost;
}
