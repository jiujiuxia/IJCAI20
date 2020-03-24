// Solve Vehicle Routing Problem with Time Windows using ALNS

#include<vector>
#include<iostream>
#include<set>
#include<cmath>

#define INF (double) 1000000000000000
#define ep (double) 0.00000001
#define FILENAME "solomon-100/In/rc205.txt"

using namespace std;


double insertTime;
double removeTime;


// Parameters for LNS
int maxIteration = 25000;
int improvementLimit = 2000;
vector<double> scoreWeight = {33, 9, 13};
vector<double> relateWeight = {9, 3, 2, 5};
double reactionFactor = 0.1;
double worstRandomDegree = 3;
double ShawRandomDegree = 6;
double coolingRate = 0.99975;
double removeControl = 0.4;
double tempControl = 0.05;
double noiseControl = 0.025;


struct request;
struct solution;



vector<double> objWeight = {1, 0, 100000};
int initialVehicleNumber ,vehicleNumber, requestNumber;
double vehicleSpeed, vehicleCapacity;
vector<request> requestList;
double maxDist;
double maxAmount;
vector<vector<double>> distances;
int stage1Iteration;


solution LNS(solution& s, int stage);
int requestRemove(solution& s, int removeNumber);
int requestInsert(solution& s, int useNoise);
bool accept(solution& sNew, solution& s, double& temp);
void updateAdaptiveWeight(vector<double>& weight, vector<double>& score, vector<int>& nb, double reactionFactor);
int rouletteSelection(vector<double> weight);
void randomRemoval(solution& s, int removeNumber);
void worstRemoval(solution& s, int removeNumber, double worstRandomDegree);
void ShawRemoval(solution& s, int removeNumber, double ShawRandomDegree);
void regretInsert(solution& s, int k, int useNoise);
double calcTemp(double objValue);
void getInput();
double calcRouteCost(vector<int>& route);
double generateNoise();
double insertingCost(solution& s, int node, vector<int>& route, int& pos);
double removeCost(int node, vector<int>& route);
void resetAdaptiveWeight();


// Variable controlling Adaptive Weights
vector<double> removalWeight = {1, 1, 1};
vector<double> insertingWeight = {1, 1, 1, 1, 1};
vector<double> noiseWeight = {1, 1};
vector<double> removalScore = {0, 0, 0};
vector<double> insertingScore = {0, 0, 0, 0, 0};
vector<double> noiseScore = {0, 0};
vector<int> removalCount = {0, 0, 0};
vector<int> insertingCount = {0, 0, 0, 0, 0};
vector<int> noiseCount = {0, 0};
set<unsigned long long> visitedList;


// Customer information
struct request {
	int id;
	double xPos, yPos;
	double amount;
	double lowerTime, upperTime;
	double serviceTime;
	double dist(request& other) {
		return distances[id][other.id];
	}
};



// Solution class
struct solution {
	vector<vector<int>> vehicleRoute;
	vector<pair<int, int>> route;
	vector<bool> visited;
	vector<double> visitedTime, startTime, waitingTime;
	vector<double> maxDelay;
	double maxTime = 0;

	solution() {
		vehicleRoute.resize(vehicleNumber);
		visited.resize(requestNumber);
		visitedTime.resize(requestNumber);
		startTime.resize(requestNumber);
		waitingTime.resize(requestNumber);
		route.resize(requestNumber);
		maxDelay.resize(requestNumber);
		maxDelay[0] = INF;
		for (int i = 0; i < vehicleNumber; ++i) {
			vehicleRoute[i] = {0, 0};

		}
		visited[0] = true;
	}


    // Number of used vehical
	int usedvehicle() {
		int ret = 0;
		for (int i = 0; i < vehicleNumber; ++i) {
			if (vehicleRoute[i].size() > 2) ++ret;
		}
		return ret;
	}

	// Total Objective value of the solution
	double objFunction() {
		double ret = 0;
		for (int i = 0; i < vehicleNumber; ++i) {
			ret += calcRouteCost(vehicleRoute[i]);
		}
		for (int i = 1; i < requestNumber; ++i) {
			if (!visited[i]) ret += objWeight[2];
		}
		return ret;
	}

	// Calculate Obj Value to calculate starting temperature
	double tempObjFunction() {
		double ret = 0;
		for (int i = 0; i < vehicleNumber; ++i) {
			ret += calcRouteCost(vehicleRoute[i]);
		}
		return ret;
	}

	unsigned long long getHash() {
		unsigned long long ret = 0;
		sort(vehicleRoute.begin(), vehicleRoute.end());
		for (int i = 0; i < vehicleNumber; ++i) {
			for (int j = 1; j < vehicleRoute[i].size()-1; ++j) {
				ret = ret * 1009 + vehicleRoute[i][j];
			}
		}
		return ret;
	}

	// Insert node to route at pos
	void insert(int node, int route, int pos) {
		vehicleRoute[route].insert(vehicleRoute[route].begin()+pos, node);
		visited[node] = true;
	}

	// Calculate time visiting each node
	void findVisitedTime(int r) {
		visitedTime[0] = 0;
		double time = 0;
		for (int j = 1; j < vehicleRoute[r].size()-1; ++j) {
			time += distances[vehicleRoute[r][j]][vehicleRoute[r][j-1]] / vehicleSpeed;
			visitedTime[vehicleRoute[r][j]] = time;
			startTime[vehicleRoute[r][j]] = max(time, requestList[vehicleRoute[r][j]].lowerTime);
			time = startTime[vehicleRoute[r][j]];
			waitingTime[vehicleRoute[r][j]] = max(0., time - visitedTime[vehicleRoute[r][j]]);
			maxTime = max(maxTime, time);
			time += requestList[vehicleRoute[r][j]].serviceTime;
		}
	}

	void print() {
		for (int i = 0; i < vehicleNumber; ++i) {
			cout << "Route " << i << ": ";
			for (int j = 0; j < vehicleRoute[i].size(); ++j) {
				cout << vehicleRoute[i][j] << " ";
			}
			cout << endl;
		}
		cout << "Request Bank : ";
		for (int i = 0; i < requestNumber; ++i)
			if (!visited[i]) cout << i << " ";
		cout << endl;
	}

	// Calculate relateness of 2 nodes in the solution
	double relatedness(request& a, request& b) {
		double ret = relateWeight[0] * (a.dist(b)) / maxDist;
		ret += relateWeight[1] * (abs(visitedTime[a.id] - visitedTime[b.id])) / maxTime;
		ret += relateWeight[2] * abs(a.amount - b.amount) / maxAmount;
		return ret;
	}

	// Remove all node from removeList to request bank
	void remove(vector<int>& removeList) {
		for (int i = 0; i < removeList.size(); ++i) {
			int pickupNode = removeList[i];
			visited[pickupNode] = false;
		}
		for (int i = 0; i < vehicleNumber; ++i) {
			vector<int> newRoute;
			for (int j = 0; j < vehicleRoute[i].size(); ++j) {
				if (visited[vehicleRoute[i][j]]) {
					newRoute.push_back(vehicleRoute[i][j]);
				}
			}
			vehicleRoute[i] = newRoute;
		}
	}

	// Remove vehicles from solution
	void resize(int k) {
		for (int i = k; i < vehicleRoute.size(); ++i) {
			for (int j = 1; j < vehicleRoute[i].size()-1; ++j) {
				visited[vehicleRoute[i][j]] = false;

			}
		}
		vehicleRoute.resize(k);
	}

	// Number of unvisited nodes
	int requestBankSize() {
		int ret = 0;
		for (int i = 0; i < requestNumber; ++i)
			if (!visited[i]) ++ret;
		return ret;
	}

	// Find position of node i in solution
	void findRoute(int i) {
		for (int j = 1; j < vehicleRoute[i].size()-1; ++j) {
			route[vehicleRoute[i][j]] = {i, j};
		}
	}

	// Calculate max delay of each node
	void calcMaxDelay(int r) {
		vector<int> route = vehicleRoute[r];
		int p = route[route.size()-1];
		maxDelay[p] = requestList[p].upperTime - startTime[p];
		for (int i = route.size()-2; i > 0; --i) {
			int p = route[i];
			maxDelay[p] = min(maxDelay[route[i+1]] + waitingTime[route[i+1]],
			requestList[p].upperTime - startTime[p]);

		}
	}

};



int main() {
	cout << fixed;
	srand(time(NULL));
	getInput();
	clock_t start = clock();
	pair<int, double> minSol = {1000, INF}, sumSol = {0, 0};
	for (int i = 0; i < 10; ++i) {
		cout << "Test run #" << i << endl;
		cout << "Stage 1" << endl;
		// Stage 1 : Minimize number of vehicles used
		tempControl = 0.35;
		coolingRate = 0.9999;
		clock_t _start = clock();
		stage1Iteration = 0;
		vehicleNumber = initialVehicleNumber;
		resetAdaptiveWeight();
		visitedList.clear();

		solution s;
		int minvehicle;
		regretInsert(s, 1, 0);
		vehicleNumber = s.usedvehicle();
		s.resize(vehicleNumber);
		solution sTemp = s;
		while (s.requestBankSize() == 0) {
			minvehicle = s.usedvehicle();
			cout << "Min vehicle Used : " << minvehicle << "  Best Solution : " << s.objFunction()  << endl;
			vehicleNumber = minvehicle-1;
			s.resize(vehicleNumber);
			s = LNS(s,1);
			if (s.requestBankSize() == 0) sTemp = s;
		}
		vehicleNumber = minvehicle;


		// Stage 2 : Minimize the cost
		resetAdaptiveWeight();
		cout << "Stage 2" << endl;
		tempControl = 0.05;
		coolingRate = 0.99975;
		s = LNS(sTemp, 2);
		minvehicle = s.usedvehicle();
		double bestSol = s.objFunction();
		cout << "Min vehicle Used : " << minvehicle << "  Best Solution : " << bestSol << endl;
		cout << "Time " << (double) (clock() - _start) / CLOCKS_PER_SEC << endl;
		minSol = min(minSol, {minvehicle, bestSol});
		sumSol.first += minvehicle;
		sumSol.second += bestSol;
		cout << "*************************************\n";
	}
	cout << FILENAME << endl;
	cout << (double) sumSol.second / 10 << " " << (double) sumSol.first / 10 << endl;
	cout << minSol.second << " " << minSol.first << endl;
	cout << (double) (clock() - start) / CLOCKS_PER_SEC / 10 << endl;
}

// Large neighborhood search with initial solution s;
solution LNS(solution& s, int stage) {
	solution sBest = s;
	double temp = calcTemp(s.tempObjFunction());
	int nbIteration = 0;
	if (stage == 1) nbIteration = stage1Iteration;
	int lastImprovement = 0;
	do {
		if (stage == 1) ++stage1Iteration;
		int removeNumber;
		do {
			removeNumber = rand() % min(100, int (removeControl * (requestNumber)));
		} while (removeNumber < 4) ;
		++nbIteration;
		++lastImprovement;
		solution sNew = s;
		if (sNew.requestBankSize() < sBest.requestBankSize()) lastImprovement = 0;
		int useNoise = rouletteSelection(noiseWeight);
		

		// Remove and insert
		int removeHeuristic = requestRemove(sNew, removeNumber);
		int insertHeuristic = requestInsert(sNew, useNoise);
		

		// Calculate Score for each heuristic
		++removalCount[removeHeuristic];
		++insertingCount[insertHeuristic];
		++noiseCount[useNoise];
		if (accept(sNew, s, temp)) {
			unsigned long long hashNb = sNew.getHash();
			if (visitedList.find(hashNb) == visitedList.end()) {
				visitedList.insert(hashNb);
				if (sNew.objFunction() < sBest.objFunction()) {
					sBest = sNew;
					removalScore[removeHeuristic] += scoreWeight[0];
					insertingScore[insertHeuristic] += scoreWeight[0];
					noiseScore[useNoise] += scoreWeight[0];
				}
				else if (sNew.objFunction()  < s.objFunction()) {
					removalScore[removeHeuristic] += scoreWeight[1];
					insertingScore[insertHeuristic] += scoreWeight[1];
					noiseScore[useNoise] += scoreWeight[1];
				}
				else {
					removalScore[removeHeuristic] += scoreWeight[2];
					insertingScore[insertHeuristic] += scoreWeight[2];
					noiseScore[useNoise] += scoreWeight[2];
				}
			}
			s = sNew;
		}
		temp = temp * coolingRate;
		if (nbIteration % 100 == 0) {
			updateAdaptiveWeight(removalWeight, removalScore, removalCount, reactionFactor);
			updateAdaptiveWeight(insertingWeight, insertingScore, insertingCount, reactionFactor);
			updateAdaptiveWeight(noiseWeight, noiseScore, noiseCount, reactionFactor);
		}


		// Stop Stage 1 if no improvement
		if (stage == 1) {
			if (sBest.requestBankSize() == 0) return sBest;
			if (sBest.requestBankSize() >= 5 && lastImprovement > improvementLimit) return sBest;
		}

	} while (nbIteration < maxIteration);
	return sBest;
}

// Calculate min cost to insert node to route
double insertingCost(solution& s, int node, vector<int>& route, int& pos) {
	double totalAmount = 0;
	for (int i = 1; i < route.size()-1; ++i) {
		totalAmount += requestList[route[i]].amount;
	}
	if (totalAmount + requestList[node].amount > vehicleCapacity) {
		return INF;
	}
	double minRouteCost = INF;
	for (int i = 1; i < route.size(); ++i) {
		double cost = distances[node][route[i-1]] + distances[node][route[i]] - distances[route[i]][route[i-1]];
		double arrivalTime = s.startTime[route[i-1]] + distances[node][route[i-1]] + requestList[route[i-1]].serviceTime;
		double waitTime = max(0., requestList[node].lowerTime - arrivalTime);
		double delay = cost + waitTime + requestList[node].serviceTime;
		bool ok = false;
		if (arrivalTime <= requestList[node].upperTime)
			if (delay <= s.waitingTime[route[i]] + s.maxDelay[route[i]] + ep) {
				ok = true;
				if (cost < minRouteCost){
					minRouteCost = cost;
					pos = i;
				}
			}
	}
	return minRouteCost;
}

// Calculate cost to remove node from route
double removeCost(int node, vector<int>& route) {
	double oldCost = calcRouteCost(route);
	vector<int> newRoute = route;
	newRoute.erase(newRoute.begin()+node);
	double cost = calcRouteCost(newRoute);
	return oldCost - cost;
}


// Calculate cost of 1 route
double calcRouteCost(vector<int>& route) {
	double sumDist = 0, sumTime = 0;
	double currentCapacity = 0;
	for (int i = 1; i < route.size(); ++i) {
		currentCapacity += requestList[route[i]].amount;
		if (currentCapacity > vehicleCapacity) return INF;
		sumDist += requestList[route[i]].dist(requestList[route[i-1]]);
		sumTime = sumTime + requestList[route[i]].dist(requestList[route[i-1]]) / vehicleSpeed;
		sumTime = max(sumTime, requestList[route[i]].lowerTime);
		if (sumTime > requestList[route[i]].upperTime) return INF;
		sumTime += requestList[route[i]].serviceTime;
	}
	return objWeight[0] * sumDist + objWeight[1] * sumTime;
}


// Remove requests from solution and return used heuristic
int requestRemove(solution& s, int removeNumber) {
	int removeHeuristic =  rouletteSelection(removalWeight);
	if (removeHeuristic == 0) randomRemoval(s, removeNumber);
	else if (removeHeuristic == 1) worstRemoval(s, removeNumber, worstRandomDegree);
	else ShawRemoval(s, removeNumber, ShawRandomDegree);
	return removeHeuristic;
}

// Insert unvisited requests from solution and return used heuristic
int requestInsert(solution& s, int useNoise) {
	int insertHeuristic =  rouletteSelection(insertingWeight);
	if (insertHeuristic == 0) regretInsert(s, 1, useNoise); // Basic greedy insert
	else if (insertHeuristic == 1) regretInsert(s, 2, useNoise);
	else if (insertHeuristic == 2) regretInsert(s, 3, useNoise);
	else if (insertHeuristic == 3) regretInsert(s, 4, useNoise);
	else regretInsert(s, s.requestBankSize(), useNoise); // Regret-m insert
	return insertHeuristic;

}

// Check if the new Solution is Accepted
bool accept(solution& sNew, solution& s, double &temp) {
	double obj = s.objFunction();
	double objNew = sNew.objFunction();
	if (objNew <= obj) return true;
	double probability = exp((obj - objNew) / temp);
	return ((double) rand() / RAND_MAX <= probability);
}

// Update Weights of each heuristic after a segment
void updateAdaptiveWeight(vector<double>& weight, vector<double>& score, vector<int>& nb, double reactionFactor) {
	for (int i = 0; i < weight.size(); ++i) {
		weight[i] = weight[i] * (1 - reactionFactor) + reactionFactor * (score[i] / max(nb[i], 1));
		score[i] = 0;
		nb[i] = 0;
	}
}


// Select a number using Roulette Wheel Selection
int rouletteSelection(vector<double> weight) {
	vector<double> sumWeight(weight.size());
	double randomNumber = (double) rand() / RAND_MAX;
	sumWeight[0] = weight[0];
	for (int i = 1; i < weight.size(); ++i) {
		sumWeight[i] = sumWeight[i-1] + weight[i];
	}
	randomNumber = randomNumber * sumWeight[weight.size()-1];
	for (int i = 0; i < weight.size(); ++i) {
		if (randomNumber < sumWeight[i]) return i;
	}
	return weight.size()-1;
}


// Remove a number of random Customer
void randomRemoval(solution& s, int removeNumber) {
	vector<int> removeList;
	for (int i = 1; i < requestNumber; ++i) {
		if (s.visited[i] && requestList[i].amount > 0) removeList.push_back(i);
	}
	random_shuffle(removeList.begin(), removeList.end());
	if (removeList.size() > removeNumber) removeList.resize(removeNumber);

	s.remove(removeList);

}


// Remove Customer base on Cost
void worstRemoval(solution& s, int removeNumber, double worstRandomDegree) {
	for (int i = 0; i < vehicleNumber; ++i)
		s.findRoute(i);
	vector<double> C(requestNumber);
	for (int i = 1; i < requestNumber; ++i) {
		if (s.visited[i] && requestList[i].amount > 0) {
			int removeRoute = s.route[i].first;
			int pos = s.route[i].second;
			C[i] = removeCost(pos, s.vehicleRoute[removeRoute]);
		}
	}
	while (removeNumber > 0) {
		vector<pair<double,int>> heap;

		for (int i = 1; i < requestNumber; ++i) {
			if (s.visited[i] && requestList[i].amount > 0) {
				heap.push_back({C[i], i});
			}
		}
		if (heap.empty()) break;
		make_heap(heap.begin(), heap.end());
		double y = 1;
		do {
			y = (double) rand() / (RAND_MAX);
		} while (y >= 1);
		y = pow(y, worstRandomDegree);
		int toRemove = int (y * heap.size());
		int removeRequest;
		for (int i = 0; i <= toRemove; ++i) {
			removeRequest = heap.front().second;
			pop_heap(heap.begin(), heap.end()) ;heap.pop_back();
		}
		int removeRoute = s.route[removeRequest].first;
		int node = s.route[removeRequest].second;
		s.visited[removeRequest] = false;
		s.vehicleRoute[removeRoute].erase(s.vehicleRoute[removeRoute].begin()+node);
		--removeNumber;
		s.findRoute(removeRoute);
		for (int i = 1; i < s.vehicleRoute[removeRoute].size()-1; ++i) {
			if (requestList[i].amount > 0) {
				int node = s.vehicleRoute[removeRoute][i];
				int pos = s.route[node].second;
				C[i] = removeCost(pos, s.vehicleRoute[removeRoute]);
			}
		}
	}
}

// Remove Node base on relateness
void ShawRemoval(solution& s, int removeNumber, double ShawRandomDegree) {
	vector<int> allRequest, removeList;
	for (int i = 0; i < vehicleNumber; ++i)
		s.findVisitedTime(i);
	vector<vector<double>> relate(requestNumber);
	for (int i = 0; i < requestNumber; ++i) {
		if (s.visited[i] && (requestList[i].amount > 0)) {
			allRequest.push_back(i);
			relate[i].resize(requestNumber);
			for (int j = 0; j < requestNumber; ++j) {
				if (s.visited[j] && (requestList[j].amount > 0)) {
					relate[i][j] = s.relatedness(requestList[i],requestList[j]);
				}
			}
		}
	}
	int r = rand() % allRequest.size();

	removeList.push_back(allRequest[r]);
	allRequest.erase(allRequest.begin() + r);

	while (removeList.size() < removeNumber  && allRequest.size() > 0) {
		r = rand() % removeList.size();
		vector<pair<double,int>> heap;
		for (int i = 0; i < allRequest.size(); ++i) {
			heap.push_back({-relate[allRequest[i]][removeList[r]], i});
		}
		make_heap(heap.begin(), heap.end());
		double y;
		do {
			y = (double) rand() / (RAND_MAX);
		} while (y >= 1);
		y = pow(y, ShawRandomDegree);
		int toRemove = int (y * allRequest.size());
		int removePos;
		for (int i = 0; i <= toRemove; ++i) {
			removePos = heap.front().second;
			pop_heap(heap.begin(), heap.end()); heap.pop_back();
		}
		removeList.push_back(allRequest[removePos]);
		allRequest.erase(allRequest.begin()+removePos);
	}
	s.remove(removeList);
}


void regretInsert(solution& s, int k, int useNoise) {
	vector<vector<double>> C(requestNumber);
	vector<vector<int>> pos(requestNumber);
	for (int i = 0; i < vehicleNumber; ++i) {
		s.findVisitedTime(i);
		s.calcMaxDelay(i);
	}
	for (int i = 1; i < requestNumber; ++i)
		if (!s.visited[i] && (requestList[i].amount > 0)) {
			C[i].resize(vehicleNumber);
			pos[i].resize(vehicleNumber);
		}
	for (int i = 1; i < requestNumber; ++i) {
		if (!s.visited[i] && requestList[i].amount > 0) {
			for (int j = 0; j < vehicleNumber; ++j) {
				C[i][j] = insertingCost(s, i, s.vehicleRoute[j], pos[i][j]);
			}
		}
	}
	while (true) {
		// Choose Node and Route to insert
		int minPossible = k;
		double maxRegret = 0;
		int insertRequest, insertRoute;
		double insertCost = INF;
		int insertPos;
		for (int i = 1; i < requestNumber; ++i) {
			if (!s.visited[i] && requestList[i].amount > 0) {
				vector<pair<double,int>> heap;
				for (int j = 0; j < vehicleNumber; ++j) {
					double cost = C[i][j];
					if (cost < INF) {
						cost += useNoise * generateNoise();
					}
					heap.push_back({-cost, -j});
				}
				make_heap(heap.begin(), heap.end());
				double minCost = -heap.front().first;
				if (minCost == INF) continue;
				int possibleRoute = 0;
				if (minCost < INF) ++possibleRoute;
				int minRoute = -heap.front().second;
				pop_heap(heap.begin(), heap.end()); heap.pop_back();
				double kCost = minCost;
				int kRoute;
				for (int z = 1; z < k; ++z) {
					if (heap.empty()) break;
					kCost = -heap.front().first;
					kRoute = -heap.front().second;
					if (kCost < INF) ++possibleRoute;
					pop_heap(heap.begin(), heap.end()); heap.pop_back();
				}
				double regretCost = kCost - minCost;
				if (possibleRoute < minPossible 
					|| (possibleRoute == minPossible && (regretCost > maxRegret 
					|| (regretCost == maxRegret && minCost < insertCost)))) {
					minPossible = possibleRoute;
					maxRegret = regretCost;
					insertRequest = i;
					insertRoute = minRoute;
					insertPos = pos[i][minRoute];
					insertCost = minCost;
				}
			}
		}
		if (insertCost == INF) return;

		// Insert node and recalculate Insert Cost
		s.insert(insertRequest, insertRoute, insertPos);
		C[insertRequest].resize(0);
		pos[insertRequest].resize(0);
		s.findVisitedTime(insertRoute);
		s.calcMaxDelay(insertRoute);
		for (int i = 1; i < requestNumber; ++i)
			if (!s.visited[i] && (requestList[i].amount > 0) && C[i][insertRoute] < INF) {
				C[i][insertRoute] = insertingCost(s, i, s.vehicleRoute[insertRoute], pos[i][insertRoute]);
			}
		
	}
}


// Calculate Starting Temperature
double calcTemp(double objValue)  {
	return objValue * tempControl / log(2);
}

// Read and process input
void getInput() {
	freopen(FILENAME, "r", stdin);
	string st;
	for (int i = 0; i < 4; ++i) {getline(cin, st);}
	cin >> initialVehicleNumber >> vehicleCapacity;
	for (int i = 0; i < 4; ++i) getline(cin, st);
	vehicleSpeed = max(vehicleSpeed, 1.0);
	while (!feof(stdin)) {
		double x, y, lower, upper, amount, service;
		int id;
		cin >> id;
		cin >> x >> y >> amount >> lower >> upper >> service;
		++requestNumber;
		requestList.push_back({id, x, y, amount, lower, upper, service});
		maxAmount = max(maxAmount, amount);
	}
		--requestNumber;
		requestList.resize(requestNumber);
	distances.resize(requestNumber);
	for (int i = 0; i < requestNumber; ++i) {
		distances[i].resize(requestNumber);
		for (int j = 0; j < requestNumber; ++j)  {
			distances[i][j] = sqrt((requestList[i].xPos - requestList[j].xPos) * (requestList[i].xPos - requestList[j].xPos) + (requestList[i].yPos - requestList[j].yPos) * (requestList[i].yPos - requestList[j].yPos));
			maxDist = max(maxDist, requestList[i].dist(requestList[j]));
		}
	}
	fclose(stdin);
}


// Add noise factor
double generateNoise() {
	return (((double) rand() / RAND_MAX) - 0.5) * (noiseControl * maxDist) * 2;
}


void resetAdaptiveWeight() {
	removalWeight = {1, 1, 1};
	insertingWeight = {1, 1, 1, 1, 1};
	noiseWeight = {1, 1};
	removalScore = {0, 0, 0};
	insertingScore = {0, 0, 0, 0, 0};
	noiseScore = {0, 0};
	removalCount = {0, 0, 0};
	insertingCount = {0, 0, 0, 0, 0};
	noiseCount = {0, 0};
}