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
	while (getline(domainStream, line))
	{
		if(line.find(":predicates") != string::npos)
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
			}

			break;
		}
	}
}

//ANALYSY FUNCTIONS

void domainAnalysis::findMetricDependentActions()
{

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
						}

						if(argumentsEqual)
						{
							(*actIt)->isMetricDependent = true;
						}
					}

				}
			}
		}
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
	while (getline(*domainStream, line))
	{
		line.erase(remove_if(line.begin(), line.end(), ::isspace), line.end());
		transform(line.begin(), line.end(), line.begin(), ::tolower);

		list<predicateAnalysis*>::iterator predIt = predicateList.begin();
		for(; predIt != predicateList.end(); predIt++)
		{
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

		list<functionAnalysis*>::iterator funcIt = functionList.begin();
		for(; funcIt != functionList.end(); funcIt++)
		{
			if(line.find((*funcIt)->name) != string::npos &&
					line.find("duration") == string::npos)
			{

			}
		}

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
			if(line.find((*predIt)->name) != string::npos &&
					line.find("duration") == string::npos &&
					line.find("at end") != string::npos)
			{
				line.erase(remove_if(line.begin(), line.end(), ::isspace), line.end());
				actionAnalysis* actIt = actionList.back();
				predicateAnalysis *predAux = new predicateAnalysis(*predIt);

				if(line.find("not") != string::npos)
				{
					predAux->negated = true;
				}
				actIt->effectsPred.push_back(predAux);
				break;
			}
		}

		list<functionAnalysis*>::iterator funcIt = functionList.begin();
		for(; funcIt != functionList.end(); funcIt++)
		{
			if(line.find((*funcIt)->name) != string::npos &&
					line.find("duration") == string::npos &&
					line.find("at end") != string::npos)
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
						break;
					}else if(word == "decrease")
					{
						funcOp->increase = false;
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


	list<functionAnalysis*>::iterator funcIt = functionList.begin();
	for(; funcIt != functionList.end(); funcIt++)
	{
		if(line.find((*funcIt)->name) != string::npos)
		{
			actionAnalysis* actIt = actionList.back();
			functionAnalysis *funcAux = new functionAnalysis(*funcIt);

			actIt->precondFunc.push_back(funcAux);
		}
	}
}
