#include "domainAnalysis.h"

#ifdef STOCHASTICDURATIONS
#include "StochasticDurations.h"
#endif

#include <fstream>

using namespace TIM;
using namespace Inst;
using namespace VAL;
using namespace std;

using std::cerr;
using std::ostringstream;
using std::endl;
using std::ifstream;

domainAnalysis DomainAnalysis;

void domainAnalysis::readDomainActions()
{
	std:ifstream domainStream((this->domainRoute).c_str());

	string line;
	while (getline(domainStream, line))
	{
		if(line.find(":durative-action") != string::npos &&
				line.find(":requirements") == string::npos)
		{
			istringstream lineStream(line);
			string actionName;
			while(lineStream >> actionName)
			{
				if(actionName.find(":") == string::npos)
				{
					transform(actionName.begin(), actionName.end(), actionName.begin(), ::tolower);
					actionAnalysis* action = new actionAnalysis(actionName);
					this->actionList.push_back(action);
				}
			}
			while (getline(domainStream, line))
			{
				if(line.find(":parameters") != string::npos)
				{
					istringstream lineStream2(line);
					string word;
					string parameter = "", parameterType = "";
					bool storedLast = true;
					while(lineStream2 >> word)
					{
						if(word.find("?") != string::npos &&
								word.find(":") == string::npos)
						{
							if(!storedLast)
							{
								actionAnalysis* actIt = actionList.back();
								actIt->arguments.push_back(parameter);
								actIt->argumentType.push_back(parameterType);
								storedLast = true;
							}
							parameter = word.substr(word.find("?") + 1, word.length());
							if(parameter.find(")") != string::npos)
							{
								parameter = parameter.substr(0,parameter.find(")"));
							}
							transform(parameter.begin(), parameter.end(), parameter.begin(), ::tolower);
							storedLast = false;
						}else if(word.find("-") != string::npos &&
								word.find(":") == string::npos)
						{
							lineStream2 >> word;
							parameterType = word;
							if(parameterType.find(")") != string::npos)
							{
								parameterType = parameterType.substr(0,parameterType.find(")"));
							}
							transform(parameterType.begin(), parameterType.end(), parameterType.begin(), ::tolower);
							actionAnalysis* actIt = actionList.back();
							actIt->arguments.push_back(parameter);
							actIt->argumentType.push_back(parameterType);
							storedLast = true;
						}
					}
					actionAnalysis* actIt = actionList.back();
					list<string>::iterator strIt = actIt->argumentType.begin();
					list<string>::iterator strIt2 = actIt->argumentType.begin();
					strIt2++;
					for(; strIt != actIt->argumentType.end(); strIt++)
					{
						if((*strIt) == "")
						{
							(*strIt) = (*strIt2);
						}
						strIt2++;
					}


					this->readDomainActPrecond(&domainStream);
					this->readDomainActEffects(&domainStream);

					break;
				}
			}
		}
	}
}

void domainAnalysis::readDomainPredicates()
{
	std:ifstream domainStream((this->domainRoute).c_str());

	string line;
	bool alreadyFound = false;
	while (getline(domainStream, line) && !alreadyFound)
	{
		if(line.find(":predicates") != string::npos)
		{
			alreadyFound = true;
			while (getline(domainStream, line))
			{
				istringstream lineStream(line);
				string word;
				string parameter = "", parameterType = "";
				bool storedLast = true;

				lineStream >> word;

				if(word == ")")
				{
					break;
				}
				while(word.find("(") != string::npos)
				{
					word = word.substr(word.find("(") + 1, word.length());
				}
				while(word.find(")") != string::npos)
				{
					word = word.substr(0, word.find(")"));
				}

				transform(word.begin(), word.end(), word.begin(), ::tolower);
				predicateAnalysis* pred = new predicateAnalysis(word);
				this->predicateList.push_back(pred);

				while(lineStream >> word)
				{
					if(word.find("?") != string::npos &&
							word.find(":") == string::npos)
					{
						if(!storedLast)
						{
							predicateAnalysis* actIt = predicateList.back();
							actIt->arguments.push_back(parameter);
							actIt->argumentType.push_back(parameterType);
							storedLast = true;
						}
						parameter = word.substr(word.find("?") + 1, word.length());
						while(parameter.find("(") != string::npos)
						{
							parameter = parameter.substr(parameter.find("(") + 1, parameter.length());
						}
						while(parameter.find(")") != string::npos)
						{
							parameter = parameter.substr(0, parameter.find(")"));
						}
						transform(parameter.begin(), parameter.end(), parameter.begin(), ::tolower);
						storedLast = false;
					}else if(word.find("-") != string::npos &&
							word.find(":") == string::npos)
					{
						lineStream >> word;
						while(word.find("(") != string::npos)
						{
							word = word.substr(word.find("(") + 1, word.length());
						}
						while(word.find(")") != string::npos)
						{
							word = word.substr(0, word.find(")"));
						}
						parameterType = word;
						if(parameterType.find(")") != string::npos)
						{
							parameterType = parameterType.substr(0,parameterType.find(")"));
						}
						transform(parameterType.begin(), parameterType.end(), parameterType.begin(), ::tolower);
						predicateAnalysis* actIt = predicateList.back();
						actIt->arguments.push_back(parameter);
						actIt->argumentType.push_back(parameterType);
						storedLast = true;
					}
				}

				predicateAnalysis* actIt = predicateList.back();
				list<string>::iterator strIt = actIt->argumentType.begin();
				list<string>::iterator strIt2 = actIt->argumentType.begin();
				strIt2++;
				for(; strIt != actIt->argumentType.end(); strIt++)
				{
					if((*strIt) == "")
					{
						(*strIt) = (*strIt2);
					}
					strIt2++;
				}
			}
		}
	}
}

void domainAnalysis::readDomainFunctions()
{
	std:ifstream domainStream((this->domainRoute).c_str());

	string line;
	while (getline(domainStream, line))
	{
		if(line.find(":functions") != string::npos)
		{
			while (getline(domainStream, line))
			{
				istringstream lineStream(line);
				string word;
				string parameter = "", parameterType = "";
				bool storedLast = true;

				lineStream >> word;
				if(word == ")")
				{
					break;
				}
				while(word.find("(") != string::npos)
				{
					word = word.substr(word.find("(") + 1, word.length());
				}
				while(word.find(")") != string::npos)
				{
					word = word.substr(0, word.find(")"));
				}
				transform(word.begin(), word.end(), word.begin(), ::tolower);
				functionAnalysis* pred = new functionAnalysis(word);
				this->functionList.push_back(pred);

				while(lineStream >> word)
				{
					while(word.find("(") != string::npos)
					{
						word = word.substr(word.find("(") + 1, word.length());
					}
					while(word.find(")") != string::npos)
					{
						word = word.substr(0, word.find(")"));
					}

					if(word.find("?") != string::npos &&
							word.find(":") == string::npos)
					{
						if(!storedLast)
						{
							functionAnalysis* actIt = functionList.back();
							actIt->arguments.push_back(parameter);
							actIt->argumentType.push_back(parameterType);
							storedLast = true;
						}
						parameter = word.substr(word.find("?") + 1, word.length());
						while(parameter.find("(") != string::npos)
						{
							parameter = parameter.substr(parameter.find("(") + 1, parameter.length());
						}
						while(parameter.find(")") != string::npos)
						{
							parameter = parameter.substr(0, parameter.find(")"));
						}
						transform(parameter.begin(), parameter.end(), parameter.begin(), ::tolower);
						storedLast = false;
					}else if(word.find("-") != string::npos &&
							word.find(":") == string::npos)
					{
						lineStream >> word;
						while(word.find("(") != string::npos)
						{
							word = word.substr(word.find("(") + 1, word.length());
						}
						while(word.find(")") != string::npos)
						{
							word = word.substr(0, word.find(")"));
						}
						parameterType = word;
						transform(parameterType.begin(), parameterType.end(), parameterType.begin(), ::tolower);
						functionAnalysis* actIt = functionList.back();
						actIt->arguments.push_back(parameter);
						actIt->argumentType.push_back(parameterType);
						storedLast = true;
					}
				}

				functionAnalysis* actIt = functionList.back();
				list<string>::iterator strIt = actIt->argumentType.begin();
				list<string>::iterator strIt2 = actIt->argumentType.begin();
				strIt2++;
				for(; strIt != actIt->argumentType.end(); strIt++)
				{
					if((*strIt) == "")
					{
						(*strIt) = (*strIt2);
					}
					strIt2++;
				}
			}
		}
	}
}

void domainAnalysis::readProblemObjects()
{
	std:ifstream domainStream((this->problemRoute).c_str());

	string line;
	while (getline(domainStream, line))
	{
		if(line.find(":objects") != string::npos)
		{
			while (getline(domainStream, line))
			{
				if(line == ")")
				{
					break;
				}
				string word;
				istringstream lineStream(line);
				objectTypeAnalysis* object = new objectTypeAnalysis();
				while(lineStream >> word)
				{
					if(word != "-")
					{
						transform(word.begin(), word.end(), word.begin(), ::tolower);
						object->instances.push_back(word);
					}else
					{
						lineStream >> word;
						transform(word.begin(), word.end(), word.begin(), ::tolower);
						object->name = word;
						this->objectList.push_back(object);
					}
				}
			}
		}
	}
}

void domainAnalysis::readProblemMetric()
{
	std:ifstream domainStream((this->problemRoute).c_str());
	this->metric.timeMetricActive = false;
	this->metric.totalTimeMetric = 0;

	string line;
	while (getline(domainStream, line))
	{
		if(line.find(":metric") != string::npos)
		{
			string word;
			bool negative = false;
			int n = 0;
			istringstream lineStream(line);
			while(lineStream >> word)
			{

				while(word.find("(") != string::npos)
				{
					word = word.substr(word.find("(") + 1, word.length());
				}
				while(word.find(")") != string::npos)
				{
					word = word.substr(0, word.find(")"));
				}

				if(word == "minimize")
				{
					this->metric.minimise = true;
				}else if(word == "maximize")
				{
					this->metric.minimise = false;
				}

				if(word == "-" || word=="/")
				{
					negative = true;
					n = 0;
				}

				transform(word.begin(), word.end(), word.begin(), ::tolower);
				functionAnalysis* function = this->findFunction(word);
				if(word == "total-time")
				{
					this->metric.timeMetricActive = true;
				}
				if(function != NULL)
				{
					int arguments = function->arguments.size();
					functionAnalysis* functionVal = new functionAnalysis(function);

					for(int i = 0; i < arguments; i++)
					{
						lineStream >> word;
						while(word.find("(") != string::npos)
						{
							word = word.substr(word.find("(") + 1, word.length());
						}
						while(word.find(")") != string::npos)
						{
							word = word.substr(0, word.find(")"));
						}
						transform(word.begin(), word.end(), word.begin(), ::tolower);
						functionVal->argumentValue.push_back(word);
					}
					if(n == 1 && negative == true)
					{
						negative = false;
						this->metric.weight.push_back((double) -1);
					}else
					{
						this->metric.weight.push_back((double) 1);
					}
					n++;

					this->metric.functions.push_back(functionVal);
				}
			}
		}
	}
}

void domainAnalysis::readProblemGoal()
{
	std:ifstream domainStream((this->problemRoute).c_str());

	string line;
	this->goal.nGoals = 0;
	this->goal.nFinalStateGoals = 0;
	while (getline(domainStream, line))
	{
		if(line.find(":goal") != string::npos)
		{
			while (getline(domainStream, line))
			{
				transform(line.begin(), line.end(), line.begin(), ::tolower);

				string word;
				istringstream lineStream(line);
				lineStream >> word;

				if(word == ")")
				{
					break;
				}

				while(word.find("(") != string::npos)
				{
					word = word.substr(word.find("(") + 1, word.length());
				}
				while(word.find(")") != string::npos)
				{
					word = word.substr(0, word.find(")"));
				}

				bool negated = false;
				int arguments = 0;
				if(word == "not")
				{
					negated = true;
					lineStream >> word;
					while(word.find("(") != string::npos)
					{
						word = word.substr(word.find("(") + 1, word.length());
					}
					while(word.find(")") != string::npos)
					{
						word = word.substr(0, word.find(")"));
					}
				}

				list<predicateAnalysis*>::iterator predIt = predicateList.begin();
				for(; predIt != predicateList.end(); predIt++)
				{
					if(word == (*predIt)->name)
					{
						arguments = (*predIt)->arguments.size();
						break;
					}
				}
				predicateAnalysis* predicate = new predicateAnalysis(*predIt);
				predicate->negated = negated;
				for(int i = 0; i<arguments; i++)
				{
					lineStream >> word;
					while(word.find("(") != string::npos)
					{
						word = word.substr(word.find("(") + 1, word.length());
					}
					while(word.find(")") != string::npos)
					{
						word = word.substr(0, word.find(")"));
					}

					predicate->argumentValue.push_back(word);

					i++;
				}

				this->goal.predicates.push_back(predicate);
				this->goal.nGoals++;
			}

			break;
		}
	}
}

void domainAnalysis::readProblemInit()
{
	std:ifstream domainStream((this->problemRoute).c_str());

	string line;
	while (getline(domainStream, line))
	{
		if(line.find(":init") != string::npos)
		{
			while (getline(domainStream, line))
			{
				while(line.find(";") != string::npos)
				{
					getline(domainStream, line);
				}
				transform(line.begin(), line.end(), line.begin(), ::tolower);

				string word;
				istringstream lineStream(line);
				lineStream >> word;

				if(word == ")")
				{
					break;
				}

				while(word.find("(") != string::npos)
				{
					word = word.substr(word.find("(") + 1, word.length());
				}
				while(word.find(")") != string::npos)
				{
					word = word.substr(0, word.find(")"));
				}

				instanceAnalysis* instance = new instanceAnalysis();
				if(word == "=")
				{
					lineStream >> word;
					while(word.find("(") != string::npos)
					{
						word = word.substr(word.find("(") + 1, word.length());
					}
					while(word.find(")") != string::npos)
					{
						word = word.substr(0, word.find(")"));
					}

					list<functionAnalysis*>::iterator funcIt = functionList.begin();
					int arguments = 0;
					for(; funcIt != functionList.end(); funcIt++)
					{
						if(word == (*funcIt)->name)
						{
							arguments = (*funcIt)->arguments.size();
							break;
						}
					}
					functionAnalysis* function = new functionAnalysis(*funcIt);
					list<string>::iterator strIt = (*funcIt)->arguments.begin();
					for(int i = 0; i < arguments; i++)
					{
						lineStream >> word;
						while(word.find("(") != string::npos)
						{
							word = word.substr(word.find("(") + 1, word.length());
						}
						while(word.find(")") != string::npos)
						{
							word = word.substr(0, word.find(")"));
						}

						function->argumentValue.push_back(word);
					}

					lineStream >> word;
					while(word.find("(") != string::npos)
					{
						word = word.substr(word.find("(") + 1, word.length());
					}
					while(word.find(")") != string::npos)
					{
						word = word.substr(0, word.find(")"));
					}

					instance->PredFunc = false;
					instance->function = function;
					instance->value = atof(word.c_str());

				}else
				{
					list<predicateAnalysis*>::iterator predIt = predicateList.begin();
					int arguments = 0;
					for(; predIt != predicateList.end(); predIt++)
					{
						if(word == (*predIt)->name)
						{
							arguments = (*predIt)->arguments.size();
							break;
						}
					}
					predicateAnalysis* predicate = new predicateAnalysis(*predIt);
					list<string>::iterator strIt = (*predIt)->arguments.begin();
					for(int i = 0; i < arguments; i++)
					{
						lineStream >> word;

						while(word.find("(") != string::npos)
						{
							word = word.substr(word.find("(") + 1, word.length());
						}
						while(word.find(")") != string::npos)
						{
							word = word.substr(0, word.find(")"));
						}

						predicate->argumentValue.push_back(word);
					}

					instance->PredFunc = true;
					instance->predicate = predicate;
				}

				this->instancesList.push_back(instance);
			}
		}
	}
}

//ANALYSY FUNCTIONS

void domainAnalysis::findMetricDependentActions()
{
	//Iterate over actions and predicates in metric, searching for effects
	//functions in actions that affect functions in metric
	list<functionAnalysis*>::iterator funcIt = this->metric.functions.begin();
	for(; funcIt != this->metric.functions.end(); funcIt++)
	{
		list<actionAnalysis*>::iterator actIt = this->actionList.begin();
		for(; actIt != this->actionList.end(); actIt++)
		{
			list<funcOperation*>::iterator funcOpIt = (*actIt)->effectsFuncOp.begin();
			for(; funcOpIt != (*actIt)->effectsFuncOp.end(); funcOpIt++)
			{
				if((*funcOpIt)->function->name == (*funcIt)->name &&
						!((*actIt)->isMetricDependent))
				{
					if((*funcOpIt)->function->argumentType.size() ==
							(*funcIt)->argumentType.size())
					{
						list<string>::iterator strIt1 = (*funcOpIt)->function->argumentType.begin();
						list<string>::iterator strIt2 = (*funcIt)->argumentType.begin();
						bool argumentsEqual = true;
						for(; strIt1 != (*funcOpIt)->function->argumentType.end(); strIt1++)
						{
							if(*strIt1 != *strIt2)
							{
								argumentsEqual = false;
							}
							strIt2++;
						}

						if(argumentsEqual)
						{
							(*actIt)->isMetricDependent = true;

							bool optimizer = false;
							list<functionAnalysis*>::iterator funcIt2 = (*funcOpIt)->operators.begin();
							for(; funcIt2 != (*funcOpIt)->operators.end(); funcIt2++)
							{
								if((*funcIt2)->arguments.size() > 0)
								{
									list<string>::iterator strIt1 = (*funcIt2)->argumentType.begin();
									list<string>::iterator strIt2 = (*actIt)->argumentType.begin();

									int nCoincidences = 0;
									for(; strIt1 != (*funcIt2)->argumentType.end(); strIt1++)
									{
										nCoincidences = 0;
										for(; strIt2 != (*actIt)->argumentType.end(); strIt2++)
										{
											if(*strIt1 == *strIt2)
											{
												nCoincidences++;
											}
										}

										if(nCoincidences >= 1)
										{
											optimizer = true;
										}
									}

								}
							}

							(*actIt)->isMetricOptimizer = optimizer;
						}
					}
				}
			}
		}
	}
}

void domainAnalysis::findGoalActions()
{
	//Search for actions whose effects are predicates in the goal
	list<predicateAnalysis*>::iterator predIt = this->goal.predicates.begin();
	for(; predIt != this->goal.predicates.end(); predIt++)
	{
	    list<actionAnalysis*>::iterator actIt = actionList.begin();
	    for(; actIt != actionList.end(); actIt++)
	    {
	    	int index = 0;
	    	list<predicateAnalysis*>::iterator effectIt = (*actIt)->effectsPred.begin();
	    	for(; effectIt != (*actIt)->effectsPred.end(); effectIt++)
			{
		    	if((*predIt)->name == (*effectIt)->name &&
		    			(*predIt)->negated == (*effectIt)->negated &&
		    			!((*actIt)->isGoalAction))
		    	{
					if((*predIt)->argumentType.size() ==
							(*effectIt)->argumentType.size())
					{
						list<string>::iterator strIt1 = (*predIt)->argumentType.begin();
						list<string>::iterator strIt2 = (*effectIt)->argumentType.begin();
						bool argumentsEqual = true;
						for(; strIt1 != (*predIt)->argumentType.end(); strIt1++)
						{
							if(*strIt1 != *strIt2)
							{
								argumentsEqual = false;
							}
							strIt2++;
						}

						if(argumentsEqual)
						{
							(*actIt)->isGoalAction = true;
							(*actIt)->indexPredGoal.push_back(index);

							list<string>::iterator strIt3 = (*actIt)->argumentType.begin();
							for(; strIt3 != (*actIt)->argumentType.end(); strIt3++)
							{
								bool argExists = false;
								list<string>::iterator strIt4 = (*effectIt)->argumentType.begin();
								for(; strIt4 != (*effectIt)->argumentType.end(); strIt4++)
								{
									if((*strIt3) == (*strIt4))
									{
										argExists = true;
									}
								}
								(*actIt)->isGoalArgument.push_back(argExists);
							}
						}
					}
		    	}
		    	index++;
			}
	    }
	}
}

void domainAnalysis::analyseGoalActions()
{
	//This fuction tries to find which objectives have to be set
	//during the execution and which should wait until the end to be set.

	list<actionAnalysis*>::iterator actIt = this->actionList.begin();
	for(; actIt != this->actionList.end(); actIt++)
	{
		if((*actIt)->isGoalAction)
		{
			int index = 0;
			list<predicateAnalysis*>::iterator efActIt = (*actIt)->effectsPred.begin();
			for(; efActIt != (*actIt)->effectsPred.end(); efActIt++)
			{
				list<actionAnalysis*>::iterator actIt2 = this->actionList.begin();
				for(; actIt2 != this->actionList.end(); actIt2++)
					if(!((*actIt2)->isMetricOptimizer))
					{
						list<predicateAnalysis*>::iterator precondActIt = (*actIt2)->precondPred.begin();
						for(; precondActIt != (*actIt2)->precondPred.end(); precondActIt++)
						{
							if((*efActIt)->name == (*precondActIt)->name &&
									(*efActIt)->negated == (*precondActIt)->negated &&
									(*efActIt)->arguments.size() == (*precondActIt)->arguments.size())
							{
								list<string>::iterator strIt1 = (*efActIt)->argumentType.begin();
								list<string>::iterator strIt2 = (*precondActIt)->argumentType.begin();
								bool argumentsEqual = true;
								for(; strIt1 != (*efActIt)->argumentType.end(); strIt1++)
								{
									if(*strIt1 != *strIt2)
									{
										argumentsEqual = false;
									}
									strIt2++;
								}

								if(argumentsEqual)
								{
									(*actIt)->isFinalStateGoalAction = true;
								}
							}
						}
					}

				index++;
			}
		}
	}

}

void domainAnalysis::analyseGoalTypes()
{
	//Iterate over action effects, try to find changes in parameters and delete not useful conditions
    list<actionAnalysis*>::iterator actIt = actionList.begin();
    for(; actIt != actionList.end(); actIt++)
    {
		if((*actIt)->isGoalAction)
		{
			list<predicateAnalysis*>::iterator efActIt = (*actIt)->effectsPred.begin();
			for(; efActIt != (*actIt)->effectsPred.end(); efActIt++)
			{
				list<predicateAnalysis*>::iterator goalPredIt = this->goal.predicates.begin();
				for(; goalPredIt != this->goal.predicates.end(); goalPredIt++)
				{
					if((*efActIt)->name == (*goalPredIt)->name &&
							(*efActIt)->negated == (*goalPredIt)->negated &&
							(*efActIt)->arguments.size() == (*goalPredIt)->arguments.size())
					{
						list<string>::iterator strIt1 = (*efActIt)->argumentType.begin();
						list<string>::iterator strIt2 = (*goalPredIt)->argumentType.begin();
						bool argumentsEqual = true;
						for(; strIt1 != (*efActIt)->argumentType.end(); strIt1++)
						{
							if(*strIt1 != *strIt2)
							{
								argumentsEqual = false;
							}
							strIt2++;
						}

						if(argumentsEqual)
						{
							if((*actIt)->isFinalStateGoalAction)
							{
								this->goal.nFinalStateGoals++;
							}
						}
					}
				}
			}
		}
    }

}

void domainAnalysis::analyseActions()
{
	//Iterate over action effects, try to find changes in parameters and delete not useful conditions
    list<actionAnalysis*>::iterator actIt = actionList.begin();
    for(; actIt != actionList.end(); actIt++)
    {
    	list<int> deletionList;
    	int index = 0;

    	list<predicateAnalysis*>::iterator effectIt = (*actIt)->effectsPred.begin();
    	for(; effectIt != (*actIt)->effectsPred.end(); effectIt++)
		{
    		//For each predicate effect, search for predicates with the same name and arguments
    		int index2 = 0;
        	list<predicateAnalysis*>::iterator effectIt2 = (*actIt)->effectsPred.begin();
        	for(; effectIt2 != (*actIt)->effectsPred.end(); effectIt2++)
    		{
        		cout << (*effectIt)->name << "|" << index << (*effectIt2)->name << "|" << index2 << endl;
        		if(index != index2 &&
        				(*effectIt)->name == (*effectIt2)->name &&
						!(std::find(deletionList.begin(), deletionList.end(), index) != deletionList.end()) &&
						!(std::find(deletionList.begin(), deletionList.end(), index2) != deletionList.end()))
        		{
        			if((*effectIt)->argumentType.size() ==
        										(*effectIt2)->argumentType.size())
					{
						list<string>::iterator strIt1 = (*effectIt)->argumentType.begin();
						list<string>::iterator strIt2 = (*effectIt2)->argumentType.begin();
						bool argumentsEqual = true;
						for(; strIt1 != (*effectIt)->argumentType.end(); strIt1++)
						{
							if(*strIt1 != *strIt2)
							{
								argumentsEqual = false;
							}
							strIt2++;
						}

						if(argumentsEqual)
						{
							cout << (*effectIt)->name << " effect in " << (*actIt)->name << ":" << endl;

							//We now know the same predicate is affected twice in the action.
							//This can mean that it changes some variable state,
							//it adds some variable state or it's internal control (not useful).
							list<string>::iterator strIt1 = (*effectIt)->arguments.begin();
							list<string>::iterator strIt2 = (*effectIt2)->arguments.begin();
							list<string>::iterator strIt = (*effectIt2)->argumentType.begin();
							bool argumentsEqual2 = true;
							for(; strIt1 != (*effectIt)->arguments.end(); strIt1++)
							{
								if(*strIt1 != *strIt2)
								{
									cout << "	Argument " << (*strIt) << " changes." << endl;
									argumentsEqual2 = false;
								}else
								{
									cout << "	Argument " << (*strIt) << " is the same." << endl;
								}

								strIt2++;
								strIt++;
							}

							if(argumentsEqual2)
							{
								cout << "	They will be deleted." << endl;
								//If the arguments are the same, then these effects can be deleted.
								//Since they do not express information aboun the domain.
								deletionList.push_back(index);
								deletionList.push_back(index2);
							}
						}
					}
        		}
        		index2++;
    		}
        	index++;
		}

    	deletionList.sort();
    	list<int>::reverse_iterator delIt = deletionList.rbegin();
    	for(; delIt != deletionList.rend(); delIt++)
    	{
    		list<predicateAnalysis*>::iterator predIt = (*actIt)->effectsPred.begin();
    	    std::advance (predIt,*delIt);
    		(*actIt)->effectsPred.erase(predIt);
    	}
    }
}

void domainAnalysis::findPrecondGoalActions()
{
	//Iterate over action effects, try to find changes in parameters and delete not useful conditions
    list<actionAnalysis*>::iterator actIt = actionList.begin();
    for(; actIt != actionList.end(); actIt++)
    {
		if((*actIt)->isGoalAction && !((*actIt)->isFinalStateGoalAction))
		{
		    list<actionAnalysis*>::iterator actIt2 = actionList.begin();
		    for(; actIt2 != actionList.end(); actIt2++)
		    {
				if(!((*actIt2)->isMetricOptimizer))
		    	{
					list<predicateAnalysis*>::iterator precondIt = (*actIt)->precondPred.begin();
					for(; precondIt != (*actIt)->precondPred.end(); precondIt++)
					{
						list<predicateAnalysis*>::iterator effectIt = (*actIt2)->effectsPred.begin();
					    for(; effectIt != (*actIt2)->effectsPred.end(); effectIt++)
					    {
					    	if((*effectIt)->name == (*precondIt)->name &&
					    		(*effectIt)->arguments.size() == (*precondIt)->arguments.size())
					    	{
					    		list<string>::iterator strIt1 = (*effectIt)->argumentType.begin();
								list<string>::iterator strIt2 = (*precondIt)->argumentType.begin();
								bool argumentsEqual = true;
								for(; strIt1 != (*effectIt)->argumentType.end(); strIt1++)
								{
									if(*strIt1 != *strIt2)
									{
										argumentsEqual = false;
									}
									strIt2++;
								}

								if(argumentsEqual)
								{
									(*actIt2)->isRequiredGoalAction = true;
									(*actIt)->nActionsRequired = (*actIt)->nActionsRequired + 1;
									(*actIt2)->goalsPreceded.push_back((*actIt)->name);
								}
					    	}
					    }
					}
			    }
		    }
		}
    }

    actIt = actionList.begin();
	for(; actIt != actionList.end(); actIt++)
	{
		if((*actIt)->isRequiredGoalAction)
		{
			list<string>::iterator strIt = (*actIt)->goalsPreceded.begin();
			for(; strIt != (*actIt)->goalsPreceded.end(); strIt++)
			{
			    list<actionAnalysis*>::iterator actIt2 = actionList.begin();
				for(; actIt2 != actionList.end(); actIt2++)
				{
					if((*strIt) == (*actIt2)->name)
					{
						(*actIt)->nReqActGoalPreceded.push_back((*actIt2)->nActionsRequired);
					}
				}
			}
		}
	}
}

void domainAnalysis::findPossibleUsefullActions()
{
    list<actionAnalysis*>::iterator actIt = actionList.begin();
    for(; actIt != actionList.end(); actIt++)
    {
		if((*actIt)->isRequiredMetricAction || (*actIt)->isRequiredGoalAction)
		{
			//cout << (*actIt)->name << endl;
		    list<actionAnalysis*>::iterator actIt2 = actionList.begin();
		    for(; actIt2 != actionList.end(); actIt2++)
		    {
				if(!((*actIt2)->isRequiredMetricAction) && !((*actIt2)->isRequiredGoalAction) && !((*actIt2)->isGoalAction)
					&& !((*actIt2)->isChangingActiveMetric) && !((*actIt2)->isMetricDependent) && !((*actIt2)->isMetricOptimizer)
					&& !((*actIt2)->isFunctionLimitedSolver))
				{
					//cout << (*actIt2)->name << endl;
					list<predicateAnalysis*>::iterator precondIt = (*actIt)->precondPred.begin();
					for(; precondIt != (*actIt)->precondPred.end(); precondIt++)
					{
						list<predicateAnalysis*>::iterator effectIt = (*actIt2)->effectsPred.begin();
					    for(; effectIt != (*actIt2)->effectsPred.end(); effectIt++)
					    {
					    	if((*effectIt)->name == (*precondIt)->name &&
					    		(*effectIt)->arguments.size() == (*precondIt)->arguments.size())
							{
								list<string>::iterator strIt1 = (*effectIt)->argumentType.begin();
								list<string>::iterator strIt2 = (*precondIt)->argumentType.begin();
								bool argumentsEqual = true;
								for(; strIt1 != (*effectIt)->argumentType.end(); strIt1++)
								{
									if(*strIt1 != *strIt2)
									{
										argumentsEqual = false;
									}
									strIt2++;
								}

								if(argumentsEqual)
								{
									(*actIt2)->isPossiblyUseful = true;
								}
							}
					    }
					}
				}
		    }
		}
    }
}

void domainAnalysis::findPrecondMetricActions()
{
    list<actionAnalysis*>::iterator actIt = actionList.begin();
    for(; actIt != actionList.end(); actIt++)
    {
		if((*actIt)->isMetricOptimizer)
		{
		    list<actionAnalysis*>::iterator actIt2 = actionList.begin();
		    for(; actIt2 != actionList.end(); actIt2++)
		    {
				if(!((*actIt2)->isMetricOptimizer) &&
					!((*actIt2)->isGoalAction) &&
				    !((*actIt2)->isRequiredGoalAction))
				{
					list<predicateAnalysis*>::iterator precondIt = (*actIt)->precondPred.begin();
					for(; precondIt != (*actIt)->precondPred.end(); precondIt++)
					{
						list<predicateAnalysis*>::iterator effectIt = (*actIt2)->effectsPred.begin();
					    for(; effectIt != (*actIt2)->effectsPred.end(); effectIt++)
					    {
					    	if((*effectIt)->name == (*precondIt)->name &&
					    		(*effectIt)->arguments.size() == (*precondIt)->arguments.size())
							{
								list<string>::iterator strIt1 = (*effectIt)->argumentType.begin();
								list<string>::iterator strIt2 = (*precondIt)->argumentType.begin();
								bool argumentsEqual = true;
								for(; strIt1 != (*effectIt)->argumentType.end(); strIt1++)
								{
									if(*strIt1 != *strIt2)
									{
										argumentsEqual = false;
									}
									strIt2++;
								}

								if(argumentsEqual)
								{
									(*actIt2)->isRequiredMetricAction = true;
								}
							}
					    }
					}
				}
		    }
		}
    }
}

void domainAnalysis::findFunctionLimitedSolver()
{
	list<actionAnalysis*>::iterator actIt = actionList.begin();
	for(; actIt != actionList.end(); actIt++)
	{
		if((*actIt)->isMetricDependent || (*actIt)->isGoalAction || (*actIt)->isFinalStateGoalAction
				|| (*actIt)->isRequiredMetricAction || (*actIt)->isMetricOptimizer)
		{
			list<funcComparison*>::iterator funComIt = (*actIt)->precondFunc.begin();
			for(; funComIt != (*actIt)->precondFunc.end(); funComIt++)
			{
				if((*funComIt)->greater)
				{
					list<actionAnalysis*>::iterator actIt2 = actionList.begin();
					for(; actIt2 != actionList.end(); actIt2++)
					{
						list<funcOperation*>::iterator funcOpIt = (*actIt2)->effectsFuncOp.begin();
						for(; funcOpIt != (*actIt2)->effectsFuncOp.end(); funcOpIt++)
						{
							if((*funcOpIt)->assign)
							{
								if((*funcOpIt)->function->name == (*funComIt)->function->name)
								{
									(*actIt2)->isFunctionLimitedSolver = true;
									(*actIt)->isFunctionLimited = true;
								}
							}
						}
					}
				}
			}
		}
	}
}

void domainAnalysis::findPrecondFunctionLimitedSolver()
{
	list<actionAnalysis*>::iterator actIt = actionList.begin();
	for(; actIt != actionList.end(); actIt++)
	{
		if((*actIt)->isFunctionLimitedSolver)
		{
		    list<actionAnalysis*>::iterator actIt2 = actionList.begin();
		    for(; actIt2 != actionList.end(); actIt2++)
		    {
		    	if(!((*actIt2)->isMetricOptimizer))
		    	{
					list<predicateAnalysis*>::iterator precondIt = (*actIt)->precondPred.begin();
					for(; precondIt != (*actIt)->precondPred.end(); precondIt++)
					{
						list<predicateAnalysis*>::iterator effectIt = (*actIt2)->effectsPred.begin();
						for(; effectIt != (*actIt2)->effectsPred.end(); effectIt++)
						{
							if((*effectIt)->name == (*precondIt)->name &&
								(*effectIt)->arguments.size() == (*precondIt)->arguments.size())
							{
								list<string>::iterator strIt1 = (*effectIt)->argumentType.begin();
								list<string>::iterator strIt2 = (*precondIt)->argumentType.begin();
								bool argumentsEqual = true;
								for(; strIt1 != (*effectIt)->argumentType.end(); strIt1++)
								{
									if(*strIt1 != *strIt2)
									{
										argumentsEqual = false;
									}
									strIt2++;
								}

								if(argumentsEqual && ((*effectIt)->negated == (*precondIt)->negated))
								{
									(*actIt2)->isFunctionLimitedSolverPrecond = true;
								}
							}
						}
					}
				}
		    }
		}
	}
}

void domainAnalysis::findPositionPredicate()
{
	string locString, agentString;

    list<predicateAnalysis*>::iterator predIt = predicateList.begin();
    for(; predIt != predicateList.end(); predIt++)
    {
    	if((*predIt)->argumentType.size() == 2)
    	{
    		bool locType = false, agentType = false;
        	list<string>::iterator strIt = (*predIt)->argumentType.begin();
            for(; strIt != (*predIt)->argumentType.end(); strIt++)
            {
            	if ((*strIt) == "sdfsfsdfs")
            	{
            		locType = true;
            	}
            	else if ((*strIt) == "dsghscxvx")
            	{
            		agentType = true;
            	}
            }

            if(locType && agentType)
            {
            	(*predIt)->isPositionPredicate = true;
            }
    	}
    }
}

void domainAnalysis::findMovementAction()
{
	list<actionAnalysis*>::iterator actIt = actionList.begin();
	for(; actIt != actionList.end(); actIt++)
	{
		bool negated = false, notNegated = false;
		list<predicateAnalysis*>::iterator effectIt = (*actIt)->effectsPred.begin();
		for(; effectIt != (*actIt)->effectsPred.end(); effectIt++)
		{
			if((*effectIt)->isPositionPredicate)
			{
				if((*effectIt)->negated)
				{
					negated = true;
				}
				else {
					notNegated = true;
				}
			}
		}

        if(negated && notNegated)
        {
        	(*actIt)->isMovementAction = true;
        }

	}
}

void domainAnalysis::findMetricOptimizerActions()
{
	//This function tries to find actions that modify the value of the metric in the future
	// by setting or modifying the state of the domain.

	list<actionAnalysis*>::iterator actIt = actionList.begin();
	for(; actIt != actionList.end(); actIt++)
	{
		if((*actIt)->isMetricOptimizer)
		{
			list<funcOperation*>::iterator funcOpIt = (*actIt)->effectsFuncOp.begin();
			for(; funcOpIt != (*actIt)->effectsFuncOp.end(); funcOpIt++)
			{
				bool isMetricFunction = false;
				list<functionAnalysis*>::iterator metricFuncIt = this->metric.functions.begin();
				for(; metricFuncIt != this->metric.functions.end(); metricFuncIt++)
				{
					//Check if this function belongs to the metric

					if((*funcOpIt)->function->name == (*metricFuncIt)->name &&
							(*funcOpIt)->function->arguments.size() == (*metricFuncIt)->arguments.size())
					{
						list<string>::iterator strIt1 = (*funcOpIt)->function->argumentType.begin();
						list<string>::iterator strIt2 = (*metricFuncIt)->argumentType.begin();
						bool argumentsEqual = true;
						for(; strIt1 != (*funcOpIt)->function->argumentType.end(); strIt1++)
						{
							if(*strIt1 != *strIt2)
							{
								argumentsEqual = false;
							}
							strIt2++;
						}

						if(argumentsEqual)
						{
							isMetricFunction = true;
						}
					}

					//For each operator of the FunctionOp list, search for actions that changes or set the state.

					list<functionAnalysis*>::iterator functionIt = (*funcOpIt)->operators.begin();
					for(; functionIt != (*funcOpIt)->operators.end(); functionIt++)
					{
						//First, search for predicates that set the state of the function value
					    list<predicateAnalysis*>::iterator predIt = predicateList.begin();
						for(; predIt != predicateList.end(); predIt++)
						{
							bool existsAllArguments = true;
							if((*functionIt)->argumentType.size() <= (*predIt)->argumentType.size())
							{
								list<string>::iterator strIt1 = (*functionIt)->argumentType.begin();
								for(; strIt1 != (*functionIt)->argumentType.end(); strIt1++)
								{
									if(!(std::find((*predIt)->argumentType.begin(),
											(*predIt)->argumentType.end(), (*strIt1)) != (*predIt)->argumentType.end()))
									{
											existsAllArguments = false;
									}

								}

								if(existsAllArguments)
								{
									/*cout << "Found possible connection: predicate " << (*predIt)->name <<
											" and function " << (*functionIt)->name << endl;*/

									//Try to find an action that sets or changes this predicate
									// 1) Set
									list<actionAnalysis*>::iterator actIt2 = actionList.begin();
									for(; actIt2 != actionList.end(); actIt2++)
									{
										if(!((*actIt2)->isMetricOptimizer) &&
												!((*actIt2)->isGoalAction) &&
												(*actIt2)->isRequiredMetricAction)
										{

											bool exists = false;
											bool existsNegated = false;
											int index = 0;

											//Check effects
											list<predicateAnalysis*>::iterator predIt2 =  (*actIt2)->effectsPred.begin();
											for(; predIt2 != (*actIt2)->effectsPred.end(); predIt2++)
											{
												list<string>::iterator strIt1 = (*predIt)->argumentType.begin();
												list<string>::iterator strIt2 = (*predIt2)->argumentType.begin();
												bool argumentsEqual = true;
												for(; strIt1 != (*predIt)->argumentType.end(); strIt1++)
												{
													if(*strIt1 != *strIt2)
													{
														argumentsEqual = false;
													}
													strIt2++;
												}

												if(argumentsEqual)
												{

													if((*predIt2)->negated)
													{
														existsNegated = true;
													}else
													{
														exists = true;
														if(!(std::find((*actIt2)->effectModifierIndex.begin(),
																(*actIt2)->effectModifierIndex.end(), index) !=
																		(*actIt2)->effectModifierIndex.end()))
														{
															(*actIt2)->effectModifierIndex.push_back(index);
														}
													}
												}

												index++;
											}

											if(exists && existsNegated)
											{
												(*actIt2)->isMetricFunctionModifierAction = true;
												(*actIt2)-> isChangingActiveMetric = isMetricFunction;
												/*cout << "Action " << (*actIt2)->name << " modifies the state of function "
														<< (*functionIt)->name << endl;*/
											}else if(exists)
											{
												(*actIt2)->isMetricFunctionSetterAction = true;
												(*actIt2)-> isChangingActiveMetric = isMetricFunction;
												/*cout << "Action " << (*actIt2)->name << " sets the state of function "
														<< (*functionIt)->name << endl;*/
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
	}
}

void domainAnalysis::calculatenOptimizationsPossible()
{
	list<actionAnalysis*>::iterator actIt = actionList.begin();
	for(; actIt != actionList.end(); actIt++)
	{
		if((*actIt)->isChangingActiveMetric)
		{
			int index = 0;
			list<predicateAnalysis*>::iterator effectIt = (*actIt)->effectsPred.begin();
			for(; effectIt != (*actIt)->effectsPred.end(); effectIt++)
			{
				if(std::find((*actIt)->effectModifierIndex.begin(),
						(*actIt)->effectModifierIndex.end(), index) !=
								(*actIt)->effectModifierIndex.end())
				{
					int nOptions = 1;

					list<string>::iterator strIt1 = (*effectIt)->argumentType.begin();
					for(; strIt1 != (*effectIt)->argumentType.end(); strIt1++)
					{
						list<objectTypeAnalysis*>::iterator objIt2 = this->objectList.begin();
						for(; objIt2 != this->objectList.end(); objIt2++)
						{
							if((*strIt1) == (*objIt2)->name)
							{
								nOptions = nOptions * (*objIt2)->instances.size();
							}
						}
					}

					nOptions = nOptions - (*effectIt)->argumentType.size();
					if(nOptions == 0) nOptions = 1;

					if((*actIt)->nOptimizationsPossible < nOptions)
					{
						(*actIt)->nOptimizationsPossible = nOptions;
					}
				}
				index++;
			}
		}
	}
}

void domainAnalysis::calculatenMaxMetricEstimate()
{
	this->maxMetricEstimate = 0;

	list<functionAnalysis*>::iterator funcIt = this->metric.functions.begin();
	for(; funcIt != this->metric.functions.end(); funcIt++)
	{
		list<actionAnalysis*>::iterator actIt = this->actionList.begin();
		for(; actIt != this->actionList.end(); actIt++)
		{
			if((*actIt)->isMetricOptimizer)
			{
				list<funcOperation*>::iterator funcOpIt = (*actIt)->effectsFuncOp.begin();
				for(; funcOpIt != (*actIt)->effectsFuncOp.end(); funcOpIt++)
				{
					if((*funcOpIt)->function->name == (*funcIt)->name)
					{
						bool argumentsEqual = true;
						list<string>::iterator strIt1 = (*funcOpIt)->function->argumentType.begin();
						list<string>::iterator strIt2 = (*funcIt)->argumentType.begin();
						for(; strIt1 != (*funcOpIt)->function->argumentType.end(); strIt1++)
						{
							if(*strIt1 != *strIt2)
							{
								argumentsEqual = false;
							}
							strIt2++;
						}

						if(argumentsEqual)
						{
							double maxValue = 0;
							double minValue = 0;
							double maxMetricValue = 1;
							double minMetricValue = 1;

							list<functionAnalysis*>::iterator funcIt2 = (*funcOpIt)->operators.begin();
							list<double>::iterator doubleIt = (*funcOpIt)->weight.begin();
							for(; funcIt2 != (*funcOpIt)->operators.end(); funcIt2++)
							{
								if((*doubleIt) == 1)
								{
									maxValue = -999999;
									minValue = 999999;
								}else
								{
									maxValue = 999999;
									minValue = -999999;
								}
								list<instanceAnalysis*>::iterator insIt = this->instancesList.begin();
								for(; insIt != this->instancesList.end(); insIt++)
								{
									if(!((*insIt)->PredFunc))
									{
										if((*insIt)->function->name == (*funcIt2)->name)
										{
											bool argumentsEqual = true;
											list<string>::iterator strIt1 = (*insIt)->function->argumentType.begin();
											list<string>::iterator strIt2 = (*funcIt2)->argumentType.begin();
											for(; strIt1 != (*insIt)->function->argumentType.end(); strIt1++)
											{
												if(*strIt1 != *strIt2)
												{
													argumentsEqual = false;
												}
												strIt2++;
											}

											if(argumentsEqual)
											{
												if((*doubleIt) == 1)
												{
													if(maxValue < (*insIt)->value)
													{
														maxValue = (*insIt)->value;
													}
													if(minValue > (*insIt)->value)
													{
														minValue = (*insIt)->value;
													}
												}else
												{
													if(maxValue > 1 / (*insIt)->value)
													{
														maxValue = 1 / (*insIt)->value;
													}
													if(minValue < 1 / (*insIt)->value)
													{
														minValue = 1 / (*insIt)->value;
													}
												}
											}
										}
									}
								}

								maxMetricValue = maxMetricValue * maxValue;
								minMetricValue = minMetricValue * minValue;
								if(!((*funcOpIt)->increase))
								{
									maxMetricValue = -maxMetricValue;
									minMetricValue = -minMetricValue;
								}
								doubleIt++;
							}

							this->maxMetricEstimate = this->maxMetricEstimate + maxMetricValue;
							this->minMetricEstimate = this->minMetricEstimate + minMetricValue;
						}
					}
				}
			}
		}
	}

	this->metricNormalizer = 1;
	while((this->maxMetricEstimate / this->metricNormalizer) > 1)
	{
		this->metricNormalizer = this->metricNormalizer * 10;
	}

	while((this->maxMetricEstimate / this->metricNormalizer) < 0.1)
	{
		this->metricNormalizer = this->metricNormalizer / 10;
	}
}

void domainAnalysis::findMetricRestrictions()
{
	list<instanceAnalysis*>::iterator insIt = this->instancesList.begin();
	for(; insIt != this->instancesList.end(); insIt++)
	{
		if(!((*insIt)->PredFunc))
		{
			if((*insIt)->function->name == "metricpercentage")
			{
				list<string>::iterator strIt = (*insIt)->function->argumentValue.begin();
				this->metric.agentRestrictions.push_back(*strIt);
				this->metric.agentRestrictionsValue.push_back((*insIt)->value);
				this->metric.agentRestrictionsValuePlanning.push_back(0);
			}
		}
	}
}
//PLANNING INFO FUNCTIONS
actionAnalysis* domainAnalysis::getAction(string action)
{
	list<actionAnalysis*>::iterator actIt = actionList.begin();
	for(; actIt != actionList.end(); actIt++)
	{
		if((*actIt)->name == action)
		{
			return (*actIt);
		}
	}

	return NULL;
}

bool domainAnalysis::isMetricDependent(string action)
{
	list<actionAnalysis*>::iterator actIt = actionList.begin();
	for(; actIt != actionList.end(); actIt++)
	{
		if((*actIt)->name == action)
		{
			return (*actIt)->isMetricDependent;
		}
	}

	return false;
}

bool domainAnalysis::isGoalAction(string action)
{
	list<actionAnalysis*>::iterator actIt = actionList.begin();
	for(; actIt != actionList.end(); actIt++)
	{
		if((*actIt)->name == action)
		{
			return (*actIt)->isGoalAction;
		}
	}

	return false;
}

bool domainAnalysis::isMovementAction(string fullAction)
{
	istringstream lineStream(fullAction);
	string word;
	while(lineStream >> word)
	{
		while(word.find("(") != string::npos)
		{
			word = word.substr(word.find("(") + 1, word.length());
		}
		while(word.find(")") != string::npos)
		{
			word = word.substr(0, word.find(")"));
		}

		list<actionAnalysis*>::iterator actIt = actionList.begin();
		for(; actIt != actionList.end(); actIt++)
		{
			if((*actIt)->name == word)
			{
				return (*actIt)->isMovementAction;
			}
		}
	}

	return false;
}

void domainAnalysis::resetActionsState()
{
	list<actionAnalysis*>::iterator actIt = actionList.begin();
	for(; actIt != actionList.end(); actIt++)
	{
		if((*actIt)->isChangingActiveMetric)
		{
			(*actIt)->nOptimizationDone = 0;
		}
	}
}

void domainAnalysis::storeAgentMetricValue(string function, double value)
{

	if(function == "total-time")
	{
		this->metric.totalTimeMetric = value;
	}

	list<functionAnalysis*>::iterator funcIt = this->metric.functions.begin();
	for(; funcIt != this->metric.functions.end(); funcIt++)
	{
		istringstream lineStream(function);
		string word;
		while(lineStream >> word)
		{
			while(word.find("(") != string::npos)
			{
				word = word.substr(word.find("(") + 1, word.length());
			}
			while(word.find(")") != string::npos)
			{
				word = word.substr(0, word.find(")"));
			}

			if(word == (*funcIt)->name)
			{
				list<string>::iterator strIt = (*funcIt)->argumentType.begin();
				list<string>::iterator strIt3 = (*funcIt)->argumentValue.begin();
				for(; strIt != (*funcIt)->argumentType.end(); strIt++, strIt3++)
				{
					lineStream >> word;
					while(word.find("(") != string::npos)
					{
						word = word.substr(word.find("(") + 1, word.length());
					}
					while(word.find(")") != string::npos)
					{
						word = word.substr(0, word.find(")"));
					}

					if(*strIt == "agent" && *strIt3 == word)
					{
						list<string>::iterator strIt2 = this->metric.agentRestrictions.begin();
						list<double>::iterator douIt2 = this->metric.agentRestrictionsValuePlanning.begin();
						for(; strIt2 != this->metric.agentRestrictions.end(); strIt2++, douIt2++)
						{
							if(word == *strIt2)
							{
								*douIt2 = value;
							}
						}
					}
				}
			}
		}
	}
}

bool domainAnalysis::isUnkownUseAction(actionAnalysis action)
{
	if( !(action.isMetricDependent) && !(action.isMetricOptimizer) && !(action.isGoalAction) &&
			!(action.isFinalStateGoalAction ) && action.nActionsRequired == 0 && !(action.isRequiredGoalAction) &&
			!(action.isRequiredMetricAction) && !(action.isMovementAction) && !(action.isMetricFunctionSetterAction) &&
			!(action.isMetricFunctionModifierAction) && !(action.isChangingActiveMetric) &&
			!(action.isFunctionLimited) && !(action.isFunctionLimitedSolver) &&
			action.nOptimizationsPossible == 0 && action.nOptimizationDone == 0)
	{
		return true;
	}
	else
	{
		return false;
	}

}

//PRIVATE FUNCTIONS

functionAnalysis* domainAnalysis::findFunction(string argName)
{
	list<functionAnalysis*>::iterator funcIt = this->functionList.begin();
	for(; funcIt != this->functionList.end(); funcIt++)
	{
		if((*funcIt)->name == argName)
		{
			return (*funcIt);
		}
	}

	return NULL;

}

void domainAnalysis::readDomainActPrecond(ifstream *domainStream)
{
	string line;
	string oriLine;
	while (getline(*domainStream, line))
	{
		//cout << line << endl;
		transform(line.begin(), line.end(), line.begin(), ::tolower);
		oriLine = line;
		line.erase(remove_if(line.begin(), line.end(), ::isspace), line.end());

		list<predicateAnalysis*>::iterator predIt = predicateList.begin();
		for(; predIt != predicateList.end(); predIt++)
		{
			string startStr= "atstart", endStr = "atend";
			if(line.find(startStr) != string::npos)
			{
				line = line.erase(line.find(startStr), startStr.length());
			}
			if(line.find(endStr) != string::npos)
			{
				line = line.erase(line.find(endStr), endStr.length());
			}

			if(line.find((*predIt)->name) != string::npos &&
					line.find("duration") == string::npos)
			{
				actionAnalysis* actIt = actionList.back();
				predicateAnalysis *predAux = new predicateAnalysis(*predIt);

				if(line.find("not") != string::npos)
				{
					predAux->negated = true;
				}
				actIt->precondPred.push_back(predAux);
				break;
			}
		}

		line = oriLine;
		list<functionAnalysis*>::iterator funcIt = functionList.begin();
		for(; funcIt != functionList.end(); funcIt++)
		{
			if(line.find((*funcIt)->name) != string::npos &&
					line.find("duration") == string::npos)
			{

				string word;
				bool negative = false;
				int n = 0;
				funcComparison* funcCom = new funcComparison();

				istringstream lineStream(line);
				while(lineStream >> word)
				{
					while(word.find("(") != string::npos)
					{
						word = word.substr(word.find("(") + 1, word.length());
					}
					while(word.find(")") != string::npos)
					{
						word = word.substr(0, word.find(")"));
					}

					if(word == "<")
					{
						funcCom->greater = false;
						funcCom->equal = false;
						break;
					}else if(word == ">")
					{
						funcCom->greater = true;
						funcCom->equal = false;
						break;
					}else if(word == "=")
					{
						funcCom->greater = false;
						funcCom->equal = true;
						break;
					}
				}

				lineStream >> word;
				while(word.find("(") != string::npos)
				{
					word = word.substr(word.find("(") + 1, word.length());
				}
				while(word.find(")") != string::npos)
				{
					word = word.substr(0, word.find(")"));
				}

				functionAnalysis* function = this->findFunction(word);
				if(function != NULL)
				{
					funcCom->function = new functionAnalysis(*function);
					int arguments = function->arguments.size();

					for(int i = 0; i < arguments; i++)
					{
						lineStream >> word;
						while(word.find("(") != string::npos)
						{
							word = word.substr(word.find("(") + 1, word.length());
						}
						while(word.find(")") != string::npos)
						{
							word = word.substr(0, word.find(")"));
						}
						funcCom->function->argumentValue.push_back(word);
					}
				}

				while(lineStream >> word)
				{
					while(word.find("(") != string::npos)
					{
						word = word.substr(word.find("(") + 1, word.length());
					}
					while(word.find(")") != string::npos)
					{
						word = word.substr(0, word.find(")"));
					}

					if(word == "-" || word=="/")
					{
						negative = true;
						n = 0;
					}

					functionAnalysis* function = this->findFunction(word);
					if(function != NULL)
					{
						int arguments = function->arguments.size();
						functionAnalysis* functionVal = new functionAnalysis(function);

						for(int i = 0; i < arguments; i++)
						{
							lineStream >> word;
							while(word.find("(") != string::npos)
							{
								word = word.substr(word.find("(") + 1, word.length());
							}
							while(word.find(")") != string::npos)
							{
								word = word.substr(0, word.find(")"));
							}
							functionVal->argumentValue.push_back(word);
						}
						if(n == 1 && negative == true)
						{
							negative = false;
							funcCom->weight.push_back((double) -1);
						}else
						{
							funcCom->weight.push_back((double) 1);
						}
						n++;

						funcCom->operators.push_back(functionVal);
					}
				}

				actionAnalysis* actIt = actionList.back();
				actIt->precondFunc.push_back(funcCom);
				break;

			}
		}

		line.erase(remove_if(line.begin(), line.end(), ::isspace), line.end());
		if(line == ")")
		{
			break;
		}
	}
}

void domainAnalysis::readDomainActEffects(ifstream *domainStream)
{
	string line;
	while (getline(*domainStream, line))
	{
		transform(line.begin(), line.end(), line.begin(), ::tolower);

		list<predicateAnalysis*>::iterator predIt = predicateList.begin();
		for(; predIt != predicateList.end(); predIt++)
		{

			string startStr= "at start", endStr = "at end";
			if(line.find(startStr) != string::npos)
			{
				line = line.erase(line.find(startStr), startStr.length());
			}
			if(line.find(endStr) != string::npos)
			{
				line = line.erase(line.find(endStr), endStr.length());
			}

			if(line.find((*predIt)->name) != string::npos &&
					line.find("duration") == string::npos)
			{
				//line.erase(remove_if(line.begin(), line.end(), ::isspace), line.end());
				actionAnalysis* actIt = actionList.back();
				predicateAnalysis *predAux = new predicateAnalysis(*predIt);

				if(line.find("not") != string::npos)
				{
					predAux->negated = true;
				}

				line = line.substr(line.find((*predIt)->name) + (*predIt)->name.length(), line.length());

				list<string>::iterator strIt = predAux->arguments.begin();
				istringstream lineStream(line);
				string word;
				while(lineStream >> word)
				{
					while(word.find("(") != string::npos)
					{
						word = word.substr(word.find("(") + 1, word.length());
					}
					while(word.find(")") != string::npos)
					{
						word = word.substr(0, word.find(")"));
					}

					(*strIt) = word.substr(1,word.length());
					strIt++;
				}


				actIt->effectsPred.push_back(predAux);
				break;
			}
		}

		list<functionAnalysis*>::iterator funcIt = functionList.begin();
		for(; funcIt != functionList.end(); funcIt++)
		{
			if(line.find((*funcIt)->name) != string::npos &&
					line.find("duration") == string::npos)
			{
				string word;
				bool negative = false;
				int n = 0;
				funcOperation* funcOp = new funcOperation();

				istringstream lineStream(line);
				while(lineStream >> word)
				{
					while(word.find("(") != string::npos)
					{
						word = word.substr(word.find("(") + 1, word.length());
					}
					while(word.find(")") != string::npos)
					{
						word = word.substr(0, word.find(")"));
					}

					if(word == "increase")
					{
						funcOp->increase = true;
						funcOp->assign = false;
						break;
					}else if(word == "decrease")
					{
						funcOp->increase = false;
						funcOp->assign = false;
						break;
					}else if(word == "assign")
					{
						funcOp->increase = false;
						funcOp->assign = true;
						break;
					}
				}

				lineStream >> word;
				while(word.find("(") != string::npos)
				{
					word = word.substr(word.find("(") + 1, word.length());
				}
				while(word.find(")") != string::npos)
				{
					word = word.substr(0, word.find(")"));
				}

				functionAnalysis* function = this->findFunction(word);
				if(function != NULL)
				{
					funcOp->function = new functionAnalysis(*function);
					int arguments = function->arguments.size();

					for(int i = 0; i < arguments; i++)
					{
						lineStream >> word;
						while(word.find("(") != string::npos)
						{
							word = word.substr(word.find("(") + 1, word.length());
						}
						while(word.find(")") != string::npos)
						{
							word = word.substr(0, word.find(")"));
						}
						funcOp->function->argumentValue.push_back(word);
					}
				}

				while(lineStream >> word)
				{
					while(word.find("(") != string::npos)
					{
						word = word.substr(word.find("(") + 1, word.length());
					}
					while(word.find(")") != string::npos)
					{
						word = word.substr(0, word.find(")"));
					}

					if(word == "-" || word=="/")
					{
						negative = true;
						n = 0;
					}

					functionAnalysis* function = this->findFunction(word);
					if(function != NULL)
					{
						int arguments = function->arguments.size();
						functionAnalysis* functionVal = new functionAnalysis(function);

						for(int i = 0; i < arguments; i++)
						{
							lineStream >> word;
							while(word.find("(") != string::npos)
							{
								word = word.substr(word.find("(") + 1, word.length());
							}
							while(word.find(")") != string::npos)
							{
								word = word.substr(0, word.find(")"));
							}
							functionVal->argumentValue.push_back(word);
						}
						if(n == 1 && negative == true)
						{
							negative = false;
							funcOp->weight.push_back((double) -1);
						}else
						{
							funcOp->weight.push_back((double) 1);
						}
						n++;

						funcOp->operators.push_back(functionVal);
					}
				}

				actionAnalysis* actIt = actionList.back();
				actIt->effectsFuncOp.push_back(funcOp);
				break;
			}
		}

		line.erase(remove_if(line.begin(), line.end(), ::isspace), line.end());
		if(line == ")")
		{
			break;
		}
	}
}
