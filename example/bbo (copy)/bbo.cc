#include <iostream>
#include <queue>
#include <tuple>
#include <vector>
#include "bbo.h"
#include "libcargo.h"

using namespace cargo;
using namespace std;
vec_t<int> sortFunc(const vec_t<long> &cost){
	vec_t<int> idx(cost.size());
	//put 1,2,3 to size of idx vector in idx
	iota(idx.begin(), idx.end(), 0);

	// sort indexes based on comparing values in v
	sort(idx.begin(), idx.end(),[&cost](size_t i1, size_t i2) {return cost[i1] < cost[i2];});
	return idx;

}
BBO::BBO(const std::string& name) : RSAlgorithm(name, false), grid_(10) {
	this->batch_time() = 60;
	PopulationSize=20;
	NumberOfElites = 2;
	MutationProbability = 0.04;
	GenerationLimit=50;
	for (int i = 1; i <= PopulationSize; ++i) {
		mu.push_back((PopulationSize + 1 - i) / (double)(PopulationSize + 1)); // emigration rate
		lambda.push_back( 1 - mu[i-1]); // immigration rate
	}
	this->gen.seed(rd());

}



void BBO::handle_vehicle(const Vehicle& vehl) {
	this->grid_.insert(vehl);

}

void BBO::listen(bool skip_assigned, bool skip_delayed) {
	this->grid_.clear();
	RSAlgorithm::listen(skip_assigned, skip_delayed);
}

void BBO::bbo_init(){

	dict<VehlId, vec_t<Customer>> Temp; //A temporary map to be used below
	dict<CustId, VehlId> temp1;



		//1- filling CandidateList for each rider


	//we need to generate some population by using any algorithm or randomly
	for (int indx = 0; indx < PopulationSize; ++indx) {
		solutions.push_back(Temp);
		assignedRider.push_back(temp1);

		Grid local_grid(this->grid_);  // make a deep copy
		for (const Customer& cust : this->customers()) {
			CandidateList[cust.id()] = local_grid.within(pickup_range(cust), cust.orig());
			//fill cust_llokup
			cust_lookup[cust.id()]=cust;
			// TODO: Add to local vehicle lookup (we need it during bbo) I am not sure ????
			for (auto &cand : CandidateList[cust.id()])
				this->vehicle_lookup[cand->id()] = cand;
		}
		problemDimension=CandidateList.size();

		for(auto & cust : CandidateList){
			bool initial=false;
			vec_t<MutableVehicleSptr> candidates = cust.second;
			while (!candidates.empty() && !initial) {
				auto k = candidates.begin();
				std::uniform_int_distribution<> m(0, candidates.size()-1);
				std::advance(k, m(this->gen));
				MutableVehicleSptr cand = *k;
				candidates.erase(k);

				if (cand->schedule().data().size() < 8) {
					sop_insert(*cand, cust_lookup[cust.first], sch, rte);

					if (chkcap(cand->capacity(), sch) && chktw(sch, rte)) {
						cand->set_sch(sch);  										// update grid version of the candidate
						cand->set_rte(rte);

						//cout<<"index= "<<indx<<"   "<<solutions.size()<<"  "<<cand->id()<<endl;
						//cout<<endl;
						//cout<<cust.first<<"  ";
						//cout<<cust_lookup[cust.first].id()<<endl;
						if (solutions[indx].count(cand->id()) == 0) {
							vec_t<Customer> assignments = {};

							solutions[indx][cand->id()] = assignments;
						}
						solutions[indx].at(cand->id()).push_back(cust_lookup[cust.first]);  		// update our copy of the candidate
						assignedRider[indx][cust.first]=cand->id();

						initial = true;
					}
					else {
					// print << "      skipping due to infeasible" << std::endl;
					}
				}
				else {
				// print << "      skipping due to sched_max" << std::endl;
				}
			}
			if(!initial){
				cout<<"Rider "<<cust.first<<"left unassigned"<<endl;
				unassignedRider.push_back(cust.first);
			}
		}

		costUpdate(indx);

	}
	solution_show();
}

void BBO::bbo_body(){

	for (int Generation = 0; Generation< GenerationLimit; Generation++){

		// Save the best solutions and costs in the elite arrays
		vec_t<dict<VehlId, vec_t<Customer>>> EliteSolutions;
		vec_t<long> EliteCosts;
	    for (int i = 0; i < NumberOfElites; ++i){
	    	EliteSolutions.push_back(solutions[sortIndx[i]]);
	    	EliteCosts.push_back(solutionsCosts[sortIndx[i]]);
		}
		//dict<VehlId, vec_t<Customer>> Temp; //A temporary map to be used below
	    vec_t<dict<VehlId, vec_t<Customer>>> tempSolutions = solutions;

	    //Use migration rates to decide how much information to share between solutions

	    for (int k = 0; k < PopulationSize; ++k) {

			for(auto &r : assignedRider[sortIndx[k]]){
				cout<<k<<"= " <<r.first<<" "<<r.second<<endl;
				//Should we immigrate?

				if( rand() < lambda[k] ){

	                //Yes - Pick a solution from which to emigrate (roulette wheel selection)
	                float RandomNum = rand() * std::accumulate(mu.begin(), mu.end(), 0.0);
	                float Select = mu[0];
	                int SelectIndex = 0;
	                cout<<"Randnum:"<<RandomNum<<"  "<<std::accumulate(mu.begin(), mu.end(), 0.0);
	                while( (RandomNum > Select) && (SelectIndex < PopulationSize) ){
	                    SelectIndex ++;
	                    Select += mu[SelectIndex];
	                }
					cout<<"index:"<<SelectIndex<<endl;
	                copyRider(SelectIndex, k, r);
	                cout<<"~~~"<<endl;
	                if (solutions[sortIndx[SelectIndex]].count(r.second) == 0) {
	                	cout<<"AAA"<<endl;
						vec_t<Customer> assignments = {};
						solutions[sortIndx[SelectIndex]][r.second] = assignments;

					}


	                cout<<"DDD"<<endl;

	                tempSolutions[sortIndx[k]][r.second] = solutions[sortIndx[SelectIndex]].at(r.second);	// this is the migration step
	        	}
	            else
	            {
	            	//tempSolutions[sortIndx[k]].at(r.second) = solutions[sortIndx[k]].at(r.second);
	            }
	        }
		}
		//TODO: bbo_mutation();
		solutions=tempSolutions;
		for(int i=0; i< solutions.size(); i++)
			costUpdate(i);
		sortIndx=sortFunc(solutionsCosts);

		for (int i = 0; i < NumberOfElites; i++){ 									// replace the worst individuals with the previous generation's elites{
		        solutions[sortIndx[PopulationSize-i]] = EliteSolutions[sortIndx[i]];
		        solutionsCosts[sortIndx[PopulationSize-i]] = EliteCosts[sortIndx[i]];
		}
		sortIndx=sortFunc(solutionsCosts);
		minimuxCostPerGeneration.push_back(solutionsCosts[sortIndx[0]]);
		cout<<"Minimum of the generation "<< Generation<< "is: " <<minimuxCostPerGeneration[Generation];

	}

}
void BBO::bbo_mutation(){

	for (int k = 0; k < PopulationSize; ++k)
		for (int ParameterIndex = 0;  ParameterIndex < problemDimension; ParameterIndex++)
			if ((rd()%1) < MutationProbability)
				//tempSolutions(k, ParameterIndex) = MinDomain + (MaxDomain - MinDomain) * rand();
				int a;


}

void BBO::copyRider(int popIndxSrc, int popIndxDest, std::pair<CustId, VehlId> r){

	//1:: how many riders to be copied
	//TODO: maybe rand() be better than this way
	int ridersCount;
	if( mu[popIndxSrc] > (2/3.0) )
		ridersCount=3;
	else if( mu[popIndxSrc] > (1/3.0) )
		ridersCount=2;
	else
		ridersCount=1;

	//2: Assignment

	//3: feasibility check

	//4 final step is :
	//tempSolutions[sortIndx[k]].at(r.second) =
}
void BBO::match() {

	this->reset_workspace();

	bbo_init();

	//1- update cost
	sortIndx=sortFunc(solutionsCosts);

	bbo_body();
	for (const Customer& cust : this->customers()) {
		//vec_t<MutableVehicleSptr> candidates = this->candidates_list[cust.id()] = local_grid.within(pickup_range(cust), cust.orig());

	}

	//Last step: commit to database
}

void BBO::costUpdate(int indx){
	MutableVehicle k;
	if(solutionsCosts.size()<=indx)
		solutionsCosts.push_back(0);
	for(auto &i : solutions[indx]){
		k = *(vehicle_lookup[i.first]);
		solutionsCosts[indx] += k.route().cost();
		//cout<<i.first<<"   "<<k.id();
		//cout<<k.route().cost();
	}
	for(auto &i : unassignedRider){
		Customer &c=cust_lookup[i];

		solutionsCosts[indx] += get_shortest_path(c.orig(),c.dest());
	}

}

void BBO::solution_show(){
	int indx=0;
	for(auto & i :assignedRider){
		cout<<endl<<"Solution:"<<indx++<<" ";
		for(auto &j : i){
			cout<<j.first<<"->"<<j.second<<", ";
		}
	}
}

void BBO::reset_workspace() {
	EliteSolutions= {};
	sortIndx= {};
	EliteCosts= {};
	solutions= {};
	solutionsCosts= {};
	assignedRider={};
	unassignedRider={};
	this->best_cost = InfInt;
	this->sch = best_sch = {};
	this->rte = best_rte = {};
	this->best_vehl = nullptr;
	this->matched = false;
}

