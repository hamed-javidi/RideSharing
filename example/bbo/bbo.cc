#include <iostream>
#include <queue>
#include <tuple>
#include <vector>
#include "bbo.h"
#include "libcargo/distance.h"
#include "libcargo.h"
#include <cstdint>
#include <stdlib.h>
using namespace cargo;



template<class T, class U>
bool compare_shared_ptr(const std::shared_ptr<T>&a,const std::shared_ptr<U>&b)
{
  if(a == b)
	  return true;

  return false;
}

template < typename T>
std::pair<bool, int > findInVector(const std::vector<T>  & vecOfElements, const T  & element)
{
	std::pair<bool, int > result;
	auto it = std::find_if(vecOfElements.begin(), vecOfElements.end(), [&element](const T & val){
		if (val.id() ==  element.id())
			return true;
		return false;
	});

	if (it != vecOfElements.end())
	{
		result.second = distance(vecOfElements.begin(), it);
		result.first = true;
	}
	else
	{
		result.first = false;
		result.second = -1;
	}

	return result;
}

void BBO::doSort(){
	print<<std::endl<<"SORTING..."<<std::endl;
	if(solutions.size() <=1)
		return;
	vec_t<int> idx(solutionsCosts.size());
	const auto cost=solutionsCosts;
	//put 1,2,3 to size of idx vector in idx
	iota(idx.begin(), idx.end(), 0);

	// sort indexes based on comparing values in v
	sort(idx.begin(), idx.end(),[&cost](size_t i1, size_t i2) {return cost[i1] < cost[i2];});

	auto tcost=solutionsCosts;
	auto tsol=solutions;
	auto assignt=assignedRider;
	auto unassignt=unassignedRider;
	auto tlookupv=lookupVehicle;
	auto tcandlist=CandidateList;
	auto tgrid=local_grid;
	int j=0;
	if(!(solutionsCosts.size() == solutions.size() && solutions.size() == assignedRider.size() &&  solutions.size() ==  unassignedRider.size() && solutions.size()== CandidateList.size())){
		print<<std::endl<<"CRITICAL ERROR SIZING"<<std::endl;
		print<<solutionsCosts.size()<<std::endl;
		print<<solutions.size()<<std::endl;
		print<<assignedRider.size()<<std::endl;
		print<<unassignedRider.size()<<std::endl;
		print<<lookupVehicle.size()<<std::endl;
		print<<CandidateList.size()<<std::endl;

	}
	for(auto &i : idx){
		local_grid[j]=tgrid[i];
	    solutionsCosts[j]=tcost[i];
		solutions[j]=tsol[i];
		assignedRider[j]=assignt[i];
		unassignedRider[j]=unassignt[i];
		lookupVehicle[j]=tlookupv[i];
		CandidateList[j]=tcandlist[i];
		j++;
	}

}
bool BBO::checkSCH(int solutionIdx, MutableVehicleSptr const & r){
	int sum=0;
	for(auto const &i : assignedRider[solutionIdx])
		if(i.second == r)
			sum++;
		else if(i.second->id() == r->id()){
			print<<"	Consistency Error";
			throw;
		}

	if((r->schedule().data().size() != solutions[solutionIdx][r].size()) || (r->schedule().data().size() != sum)){
		print << "	CHK--OK"<<std::endl;
		return 0;
	}
	return 1;
}
bool BBO::checkSCHTemp(int solutionIdx, MutableVehicleSptr const & r, vec_t<dict<Customer, MutableVehicleSptr>>  &tempAssignedRider, vec_t<dict<MutableVehicleSptr, vec_t<Customer>>> &tempSolutions){
	int sum=0;
	for(auto const &i : tempAssignedRider[solutionIdx])
		if(i.second == r)
			sum++;
		else if(i.second->id() == r->id()){
			print<<"	Consistency Error CHK-T";
			throw;
		}

	if((r->schedule().data().size() != tempSolutions[solutionIdx][r].size()) || (r->schedule().data().size() != sum)){
		print <<"	CHK--OK"<<std::endl;
		return 0;
	}
	return 1;
}
BBO::BBO(const std::string& name) : RSAlgorithm(name, false), grid_(10){

	best_cost=INT32_MAX;
	matched=false;
	debugMode=true;
	problemDimension=0;
	this->batch_time() = 30;

	for (uint8_t i = 1; i <= PopulationSize; ++i) {
		mu.push_back((PopulationSize + 1 - i) / (double)(PopulationSize + 1)); // emigration rate
		lambda.push_back( 1 - mu[i-1]); // immigration rate
	}
	//this->gen.seed(rd());
	//TODO: comment the below line and uncomment the above line
	this->gen.seed(1);
	print<<"Solution size: "<<solutions.size()<<std::endl;
	unassignedRider.reserve(PopulationSize);
	assignedRider.reserve(PopulationSize);
	EliteAssignedRider.reserve(NumberOfElites);
	solutions.reserve(PopulationSize);
	solutionsCosts.reserve(PopulationSize);
	lookupVehicle.reserve(PopulationSize);
	CandidateList.reserve(PopulationSize);
	EliteSolutions.reserve(NumberOfElites);
	EliteCosts.reserve(NumberOfElites);
	minimumCostPerGeneration.reserve(GenerationLimit);
}



void BBO::handle_vehicle(const Vehicle& vehl) {
	this->grid_.insert(vehl);

}

void BBO::listen(bool skip_assigned, bool skip_delayed) {
	this->grid_.clear();
	RSAlgorithm::listen(skip_assigned, skip_delayed);
}

void BBO::bbo_init(){
	if(debugMode)
		print<<std::endl<<"Initializing...";

	//1- filling CandidateList for each rider
	//we need to generate some population by using any algorithm or randomly
	for (int indx = 0; indx < PopulationSize; ++indx) {
		local_grid.push_back(this->grid_);  // make a deep copy
		for (const Customer& cust : this->customers()) {
			vec_t<MutableVehicleSptr> candidates=local_grid[indx].within(pickup_range(cust), cust.orig());
			if(indx >= CandidateList.size())
				CandidateList.push_back({{cust,candidates}});
			else
				CandidateList[indx][cust] = candidates;
			// TODO: Add to local vehicle lookup (we need it during bbo) it needs to be improved. It does an inefficiently work
			for (const MutableVehicleSptr cand : candidates){
				if(indx >= lookupVehicle.size())
					lookupVehicle.push_back({{cand->id(), cand}});
				else
					lookupVehicle[indx][cand->id()] = cand;
			}
			bool initial=false;
			while (!candidates.empty() && !initial) {
				auto k = candidates.begin();
				std::uniform_int_distribution<> m(0, candidates.size()-1);
				std::advance(k, m(this->gen));
				MutableVehicleSptr cand = *k;
				candidates.erase(k);
				if (cand->schedule().data().size() < 8) {
					sop_insert(*cand, cust, sch, rte);
					if (chkcap(cand->capacity(), sch) && chktw(sch, rte)) {
						if(checkVehlStopsDuplication(cand,cust)){
							print<<"Sch Duplication"<<std::endl;
							throw;
						}
						cand->set_sch(sch);  										// update grid version of the candidate
						cand->set_rte(rte);
						cand->reset_lvn();
						if(indx >= solutions.size())
							solutions.push_back({{cand,{cust}}});
						else
							solutions[indx][cand].push_back(cust);
						if(indx >= assignedRider.size())
							assignedRider.push_back({{cust,cand}});
						else if(assignedRider[indx].count(cust) == 0)
							assignedRider[indx][cust]=cand;
						else{
							print<<"	Error in inserting a record to assignRider list: current cust is exist in the list"<<std::endl;
							throw;
						}
						if(debugMode)
							print << "	Rider "<<cust.id()<<"is assigned to driver "<<cand->id()<<std::endl;
						initial = true;
						checkSCH(indx, cand);
					}
					else {
//						if(debugMode)
//							print << "      skipping due to infeasible" << std::endl;
					}
				}
				else {
//					if(debugMode)
//						print << "      skipping due to sched_max" << std::endl;
				}
//				if (this->timeout(this->timeout_0))
//					break;
			}
			if(!initial){
				if(debugMode)
					print << "	Rider " << cust.id() << "left unassigned" << std::endl;
				if(indx >= unassignedRider.size())
					unassignedRider.push_back({cust});
				else
					unassignedRider[indx].push_back(cust);
			}
		}

		costUpdate(indx);
		if(debugMode)
			print << "	Solution " << indx << ", Assigned Rider: " << assignedRider[indx].size() << ", Unassigned: " <<unassignedRider[indx].size() << std::endl;
	}

	//problemDimension=CandidateList[0].size();
}

void BBO::bbo_body(){
	// Save the best solutions and costs in the elite arrays
//	dict<MutableVehicleSptr, vec_t<Customer>> Temp;
//	dict<Customer, MutableVehicleSptr> temp1;
//	dict<VehlId, MutableVehicleSptr> temp3;

	for (int Generation = 0; Generation< GenerationLimit; Generation++){

		// Save the best solutions and costs in the elite arrays

		for (int i = 0; i < NumberOfElites; ++i){
			local_grid.push_back(local_grid[i]);
//			lookupVehicle.push_back(temp3);
//			EliteSolutions.push_back(Temp);
//			EliteAssignedRider.push_back(temp1);

			for(auto const & record : solutions[i]){
				auto newCand = local_grid[PopulationSize+i].select(record.first->id());
				if(i >= EliteSolutions.size())
					EliteSolutions.push_back({{newCand,{record.second}}});
				else
					EliteSolutions[i][newCand]=record.second;
			}
			for(auto const & record : assignedRider[i]){
				auto newCand = local_grid[PopulationSize+i].select(record.second->id());
				if(i >= EliteAssignedRider.size())
					EliteAssignedRider.push_back({{record.first,newCand}});
				else
					EliteAssignedRider[i][record.first]=newCand;
			}
			for(auto const & record : lookupVehicle[i]){
				auto newCand = local_grid[PopulationSize+i].select(record.first);
				if(i >= (lookupVehicle.size()-PopulationSize)) //lookupVehicle is used for storing Solutions and Elites
					lookupVehicle.push_back({{record.first,newCand}});
				else
					lookupVehicle[PopulationSize+i][record.first]=newCand;
			}
			EliteCosts.push_back(solutionsCosts[i]);

		}

	    auto tempSolutions = solutions;
	    auto tempAssignedRider = assignedRider;
	    auto tempUnassignedRider=unassignedRider;

	    //Use migration rates to decide how much information to share between solutions

	    for (int k = 0; k < PopulationSize; ++k) {

			for(auto const &r : assignedRider[k]){
				//print<<k<<"= " <<r.first.id()<<" "<<r.second->id()<<std::endl;
				//Should we immigrate?

				if( rand() < lambda[k] ){

	                //Yes - Pick a solution from which to emigrate (roulette wheel selection)
	                float RandomNum = rand() * std::accumulate(mu.begin(), mu.end(), 0.0);
	                float Select = mu[0];
	                int SelectIndex = 0;

	                while( (RandomNum > Select) && (SelectIndex < PopulationSize) ){
	                    SelectIndex ++;
	                    Select += mu[SelectIndex];
	                }


	                if(SelectIndex!=k){
	                	if(debugMode)
	                		print<<std::endl<<"Population "<<k<<"= " <<"Rider "<<r.first.id()<<" -> Driver "<<r.second->id()<<", Transfer to Population "<<SelectIndex<<std::endl;
	                	if(!migrate_vehicle_based(SelectIndex, k, r.second, tempSolutions,tempAssignedRider, tempUnassignedRider))
	                		print<<"migrate is not done!"<<std::endl;
//	                	if(!checkVehlStopsWithRiders(tempSolutions)){
//	                		print<<"NOT MATCH"<<std::endl;
//	                		throw;
//	                	}
	                	//solution_show();
	                }

//	                if (solutions[SelectIndex].count(r.second) == 0) {
//	                	//print<<"AAA"<<std::endl;
//						vec_t<Customer> assignments = {};
//						solutions[SelectIndex][r.second] = assignments;
//					}
//
//	                tempSolutions[k][r.second] = solutions[SelectIndex].at(r.second);	// this is the migration step
	        	}
	            else
	            {
	            	//tempSolutions[sortIndx[k]].at(r.second) = solutions[sortIndx[k]].at(r.second);
	            }
	        }
		}
		//TODO: bbo_mutation();
		solutions=tempSolutions;
		assignedRider=tempAssignedRider;
		unassignedRider=tempUnassignedRider;

		for(uint8_t i=0; i< solutions.size(); i++)
			costUpdate(i);

		doSort();
		//insertElits();
		//Clear ELITES to be ready to use in the next generation
		for (int i = 0; i < NumberOfElites; ++i){
			local_grid.pop_back();
			lookupVehicle.pop_back();
		}
		EliteCosts.clear();
		EliteSolutions.clear();
		EliteAssignedRider.clear();

		//doSort();
//		if(!checkVehlStopsWithRiders(solutions)){
//			print<<"NOT MATCH"<<std::endl;
//			throw;
//		}
		minimumCostPerGeneration.push_back(solutionsCosts[0]);
		print<<"Minimum of the generation "<< Generation<< " is: " <<minimumCostPerGeneration[Generation]<<std::endl;
		if(debugMode){
			solution_show();
			print<<std::endl;
		}

	}

}
bool BBO::checkVehlStopsWithRiders(const vec_t<dict<MutableVehicleSptr, vec_t<Customer>>> & sol){
	int solCnt=0;

	for(auto const & i: sol){
		for(auto const & rec : i){
			if((rec.first->schedule().size()-2) != rec.second.size()*2){
				print<<"Solution: "<<solCnt<<" , Vehl id: "<<rec.first->id()<<std::endl;
				return 0;
			}
		}
		solCnt++;
	}
	return 1;
}
bool BBO::checkVehlStopsDuplication(const MutableVehicleSptr &vehl, const Customer & cust){
	for(auto const & i : vehl->schedule().data())
		if(i.owner()==cust.id()){
			print<<"Rider "<<cust.id()<<" already exist in the vehl  "<<vehl->id()<<std::endl;
			return 1;
		}

	return 0;
}
void BBO::end() {

  RSAlgorithm::end();
}
void BBO::insertElits(){
	for (uint8_t i = 0; i < NumberOfElites; i++){
		solutions[PopulationSize-1-i].clear();
		solutions[PopulationSize-1-i]=EliteSolutions[i];
		lookupVehicle[PopulationSize-1-i]=lookupVehicle[PopulationSize+i];
		assignedRider[PopulationSize-1-i]=assignedRider[PopulationSize+i];
		solutionsCosts[PopulationSize-1-i] = EliteCosts[i];
	}
	/*
	try{
		for (uint8_t i = 0; i < NumberOfElites; i++){
			solutions[PopulationSize-1-i].clear();

			for(auto const &vSrc : EliteSolutions[i]){
				MutableVehicleSptr & vDest=lookupVehicle[PopulationSize-1-i].at(vSrc.first->id());
				solutions[PopulationSize-1-i][vDest]=vSrc.second;
				for(auto const &r:vSrc.second){
					assignedRider[PopulationSize-1-i][r]=vDest;
				}
			}

			solutionsCosts[PopulationSize-1-i] = EliteCosts[i];

		}
	}
	catch(const out_of_range &e){
			cerr<<"Exception at Elites access" << e.what() << std::endl;
	}
	*/
}

void BBO::bbo_mutation(){

	for (uint8_t k = 0; k < PopulationSize; ++k)
		for (int ParameterIndex = 0;  ParameterIndex < problemDimension; ParameterIndex++)
			if ((rd()%1) < MutationProbability)
				//tempSolutions(k, ParameterIndex) = MinDomain + (MaxDomain - MinDomain) * rand();
				;


}

bool BBO::migrate_vehicle_based(int popIndxDest, int popIndxSrc,
									const MutableVehicleSptr &r,
									vec_t<dict<MutableVehicleSptr, vec_t<Customer>>> & tempSolutions,
									vec_t<dict<Customer, MutableVehicleSptr>> &tempAssignedRider,
									vec_t<vec_t<Customer>> &tempUnassignedRider){
	//In case of ROLL BACK migration
	if(debugMode){
		print << "	step 1"<<std::endl;
		print<<"	Migration function is started:"<<std::endl;
	}
	vec_t<dict<MutableVehicleSptr, vec_t<Customer>>> t_tempSolution;
	vec_t<dict<Customer, MutableVehicleSptr>> t_tempAssignedRider;
	vec_t<vec_t<Customer>> t_tempUnassignedRider;
	t_tempSolution=tempSolutions;
	t_tempAssignedRider=tempAssignedRider;
	t_tempUnassignedRider=tempUnassignedRider;
	if(debugMode)
		print<<"	S:"<<popIndxSrc<<" D:"<<popIndxDest<<" Vehl:"<<r->id()<<std::endl;
	//create a new vehicle based on selected vehicle if it does not exist in the destination

	vec_t<Customer> copyRiders={};
	MutableVehicleSptr const copyVehl=lookupVehicle[popIndxDest].at(r->id());
	vec_t<Customer> alreadyAsssignRider={},reassignRiders={},toBeMigratedRiders={};

	if(t_tempSolution[popIndxSrc].count(r))
		for(auto const &i: t_tempSolution[popIndxSrc][r])
			copyRiders.push_back(i);

	// 1- Does the	selected rider's vehicle exist in the target solution?
	if(t_tempSolution[popIndxDest].count(copyVehl)==0){

		//1.NO: Insert the selected vehicle without its riders as an emigrate feature to the target solution.
		if(debugMode)
			print <<"	Vehicle id "<<copyVehl->id()<<" is copied in the population "<<popIndxDest<<std::endl;
		t_tempSolution[popIndxDest][copyVehl]={};
	}

	//1-Yes: Choose some riders from the selected vehicle as emigrate riders
	int xx=copyRiders.size();
	if(xx == 0){
		print <<popIndxSrc<<", "<<popIndxDest<<", "<<r->id()<<std::endl;
		throw;
	}

	//TODO uncomment the below line to have a random number of selected riders
	//ridersRandomlySelector(popIndxSrc, r, copyRiders);
	if(debugMode)
		print <<xx<<" out of "<<copyRiders.size()<<" rider is selected"<<std::endl;

	//fill reassign rider and AlreadyAssignRider
	for(auto const &i: t_tempSolution[popIndxDest][copyVehl]){
		bool found=0;
		for(auto const &j : copyRiders){
			if(i.id() == j.id()){
				alreadyAsssignRider.push_back(i);
				found=1;
				break;
			}
		}
		if(!found)
			reassignRiders.push_back(i);
	}

	for(auto const &i : copyRiders){
		std::pair<bool, int> result =findInVector<Customer>(alreadyAsssignRider,i);
		print <<result.first<<"  "<<result.second<<std::endl;
		if(!result.first)
			toBeMigratedRiders.push_back(i);
	}

	if(debugMode){
		print <<"	copyRiders: ";
				for(auto const &i : copyRiders){
					print <<i.id()<<", ";
				}
		print <<"	AlreadyAssigndRiders: ";
		for(auto const &i : alreadyAsssignRider){
			print <<i.id()<<", ";
		}
		print <<std::endl<<"	ReassignRiders: ";
		for(auto const &i : reassignRiders){
			print <<i.id()<<", ";
		}
		print <<std::endl<<"	toBeMigratedRiders: ";
		for(auto const &i : toBeMigratedRiders){
			print <<i.id()<<", ";
		}
		print <<std::endl;
		print <<"	step 2"<<std::endl;
	}
	//2. Has any of the copy riders been assigned to some vehicles else in the target solution?

	//2.YES, deassign them from those vehicles in the target solution
	for(auto const &i: toBeMigratedRiders){
		MutableVehicleSptr cand=nullptr;
		if(t_tempAssignedRider[popIndxDest].count(i) > 0)
			cand=t_tempAssignedRider[popIndxDest][i];
		if(cand == nullptr)  //rider i has not been assigned in destination
			continue;
		if(t_tempSolution[popIndxDest].count(cand)){
			t_tempSolution[popIndxDest][cand].erase(std::remove(t_tempSolution[popIndxDest][cand].begin(), t_tempSolution[popIndxDest][cand].end(), i), t_tempSolution[popIndxDest][cand].end());
			if(t_tempSolution[popIndxDest][cand].size() == 0 ){
				t_tempSolution[popIndxDest].erase(cand);
				print <<"	Driver "<<cand->id()<<"is removed from solution "<<popIndxDest<<std::endl;
			}
			std::vector<Stop>  sch_after_rem = cand->schedule().data();
			std::vector<Wayp> rte_after_rem;
			opdel(sch_after_rem, i.id());
			route_through(sch_after_rem, rte_after_rem);

			// Add the traveled distance to rte_after_rem <<<<<
			for (Wayp& wp : rte_after_rem)
				wp.first += cand->route().dist_at(cand->idx_last_visited_node());

			cand->set_sch(sch_after_rem);
			cand->set_rte(rte_after_rem);
			cand->reset_lvn();
			t_tempAssignedRider[popIndxDest].erase(i);
			if(checkVehlStopsDuplication(cand,i)){
				print<<"	Sch Duplication"<<std::endl;
				throw;
			}
			checkSCHTemp(popIndxDest,cand, t_tempAssignedRider, t_tempSolution);
			if(debugMode)
				print <<"	Rider "<<i.id()<<"is deassigned from "<<cand->id()<<std::endl;
		}
		else
			print <<"	Error "<<i.id()<<std::endl;
	}
	if(debugMode)
		print<<"	step 3"<<std::endl;
	// 3. Does some riders else already assigned to the selected vehicle in the target solution?
	//3. YES: Deassign them and put them to a waiting list to reassign later

	for(const auto &i: reassignRiders){
		//MutableVehicleSptr cand=t_tempAssignedRider[popIndxDest][i];
		if(t_tempSolution[popIndxDest].count(copyVehl)){
			t_tempSolution[popIndxDest][copyVehl].erase(std::remove(t_tempSolution[popIndxDest][copyVehl].begin(), t_tempSolution[popIndxDest][copyVehl].end(), i), t_tempSolution[popIndxDest][copyVehl].end());

			//update vehicle's route and schedule
			std::vector<Stop>  sch_after_rem = copyVehl->schedule().data();
			std::vector<Wayp> rte_after_rem;
			opdel(sch_after_rem, i.id());
			route_through(sch_after_rem, rte_after_rem);

			// Add the traveled distance to rte_after_rem <<<<<
			for (Wayp& wp : rte_after_rem)
				wp.first += copyVehl->route().dist_at(copyVehl->idx_last_visited_node());

			copyVehl->set_sch(sch_after_rem);
			copyVehl->set_rte(rte_after_rem);
			copyVehl->reset_lvn();
			t_tempAssignedRider[popIndxDest].erase(i);
			if(checkVehlStopsDuplication(copyVehl,i)){
				print<<"	Sch Duplication"<<std::endl;
				throw;
			}
			checkSCHTemp(popIndxDest,copyVehl, t_tempAssignedRider, t_tempSolution);
			if(debugMode)
				print <<"	Rider "<<i.id()<<"is deassigned from "<<copyVehl->id()<<std::endl;
		}
	}
	if(debugMode)
		print <<"	step 3-2"<<std::endl;
	//3. No: Assign the selected riders to the selected vehicle in the target solution
	for(const auto &i: toBeMigratedRiders){
		sop_insert(copyVehl, i, sch, rte);
		if(checkVehlStopsDuplication(copyVehl,i)){
			print<<"Sch Duplication"<<std::endl;
			throw;
		}
		copyVehl->set_sch(sch);
		copyVehl->set_rte(rte);
		copyVehl->reset_lvn();
		t_tempAssignedRider[popIndxDest][i]=copyVehl;
		t_tempSolution[popIndxDest][copyVehl].push_back(i);
		checkSCHTemp(popIndxDest,copyVehl, t_tempAssignedRider, t_tempSolution);
		if(debugMode)
			print <<"	Rider "<<i.id()<<"is assigned to "<<copyVehl->id()<<std::endl;
	}
	if(debugMode)
		print<<"	step 4"<<std::endl;
	//4. Is the waiting list empty?

	//4. NO: Reassign them to vehicles which the best candidate in the target solution
	for(auto const &i: reassignRiders)
		if(!Greedy_Assignment(popIndxDest,i, t_tempSolution, t_tempAssignedRider,t_tempUnassignedRider)){
			//exit without saving migration results
			//return 0;
			;
	}
	if(debugMode)
		print <<"	step saving"<<std::endl;
	tempSolutions=t_tempSolution;
	tempAssignedRider=t_tempAssignedRider;
	tempUnassignedRider=t_tempUnassignedRider;
	return 1;
}

bool BBO::Greedy_Assignment(int solIndex,const Customer &cust,
								vec_t<dict<MutableVehicleSptr, vec_t<Customer>>> & tempSolutions,
								vec_t<dict<Customer, MutableVehicleSptr>> &tempAssignedRider,
								vec_t<vec_t<Customer>> &tempUnassignedRider){
	bool match=false;
	this->best_cost = InfInt;
	this->best_sch.clear();
	this->best_rte .clear();
	for (auto const & cand : CandidateList[solIndex].at(cust)) {
		// Speed heuristic: try only if vehicle's current schedule has < 8 customer stops
		if (cand->schedule().data().size() < 8) {
			DistInt curCost= cand->route().cost();
			DistInt newcost = sop_insert(*cand, cust, sch, rte);
			DistInt cost = newcost - curCost; //Actually we have computed overhead cost

			if (cost < this->best_cost) {
				if (chkcap(cand->capacity(), sch) && chktw(sch, rte)) {
					this->best_vehl = cand;
					this->best_sch = sch;
					this->best_rte = rte;
					this->best_cost = cost;
					match=true;
				}
			}
		}

	}
	if(!match){
		if(debugMode)
			print <<"	Rider "<<cust.id()<<"left unassigned"<<std::endl;
		tempUnassignedRider[solIndex].push_back(cust);

		return 0;
	}
	else{
		if(checkVehlStopsDuplication(best_vehl,cust)){
			print<<"Sch Duplication"<<std::endl;
			throw;
		}
		best_vehl->set_sch(best_sch);  										// update grid version of the candidate
		best_vehl->set_rte(best_rte);
		best_vehl->reset_lvn();

		tempSolutions[solIndex][best_vehl].push_back(cust);  		// update our copy of the candidate
		tempAssignedRider[solIndex][cust]=best_vehl;
		checkSCHTemp(solIndex,best_vehl, tempAssignedRider, tempSolutions);
		if(debugMode)
			print <<"	Rider "<<cust.id()<<" Assign to driver : "<<best_vehl->id()<<" with cost "<<best_cost<< " using Greedy"<<std::endl;
	}
	return 1;
}

void BBO::ridersRandomlySelector(int indx, MutableVehicleSptr vehl, vec_t<Customer> & riders){
	//1:: how many riders are feasible to be copied
	int ridersCount=riders.size();
	if(ridersCount > (vehl->capacity()-vehl->queued()))
		ridersCount = vehl->capacity()-vehl->queued();
	if(ridersCount==0){
		riders={};
		return;
	}

	switch(ridersCount){
		case 3:
			if( mu[indx] <= 0.66  && mu[indx] > 0.33)
				ridersCount=2;
			else if( mu[indx] <= 0.33 )
				ridersCount=1;
			break;

		case 2:
			if( mu[indx] <= 0.5 )
				ridersCount=1;
			break;
		// in case 1 nothing
	}
	randXofY(ridersCount,riders);

}
void BBO::randXofY(int ridersCount, vec_t<Customer> & riders){
	while(riders.size()>ridersCount){
		riders.erase(riders.begin()+(int)(rand()*riders.size()));
	}

}
void BBO::match() {

	this->reset_workspace();

	print<<"Batch #: "<<(++(this->batchCounter))<<", requests: "<<this->customers().size()<< std::endl;
	bbo_init();
	if(debugMode){
		solution_show();
		int sum[PopulationSize]={0};
		if(debugMode){
			for(int i=0;i<PopulationSize;i++){
				for(const auto & j:solutions[i])
					if(compare_shared_ptr(j.first, lookupVehicle[i].at(j.first->id())))
						sum[i]++;
				print<<sum[i]<<std::endl;
			}
		}
	}
	doSort();
	if(debugMode)
		solution_show();
	this->init_cost=solutionsCosts[0];
	bbo_body();
	this->avgCostImprovement+= 1-(this->init_cost / solutionsCosts[0]);
	commit();
	print << "Average improvement: "<<this->avgCostImprovement/this->batchCounter<<std::endl;
	//Last step: commit to database
}
void BBO::commit() {
  for (const auto& kv : solutions[0]) {
    MutableVehicleSptr cand = kv.first;
    vec_t<CustId> cadd = {};
    vec_t<CustId> cdel = {};
    for (const Customer& cust : kv.second)
    	cadd.push_back(cust.id());
    if (this->assign(cadd, cdel, cand->route().data(), cand->schedule().data(), *cand)) {
       for (const CustId& cid : cadd)
         print << "Matched " << cid << " with " << cand->id() << std::endl;
    }  else {
       for (const CustId& cid : cadd)
         print(MessageType::Warning) << "Rejected due to sync " << cid << " with " << cand->id() << std::endl;
     }
    checkSCH(0, cand);
  }
}
void BBO::costUpdate(int indx){

	if(solutionsCosts.size() <= (unsigned)indx)
		solutionsCosts.push_back(0);
	solutionsCosts[indx]=0;
	for(const auto &i : solutions[indx]){
		solutionsCosts[indx] += i.first->route().cost();
	}
	for(const auto &i : unassignedRider[indx]){
		solutionsCosts[indx] += 2 * get_shortest_path(i.orig(),i.dest());			//2x cost penalty for unassigned riders
	}

}

uint16_t dictSize(dict<MutableVehicleSptr, vec_t<Customer>> d){
	uint16_t sum=0;
	for(auto i : d)
		for(auto j : i.second)
			sum++;
	return sum;
}


void BBO::solution_show(){
//	int indx=0;
//	for(auto & i :assignedRider){
//		print<<std::endl<<"Cost="<<solutionsCosts[indx]<<", ";
//		print<<"Solution:"<<indx++<<" ";
//		for(auto &j : i){
//			print<<j.first.id()<<"->"<<j.second->id()<<", ";
//		}
//
//	}
//	print<<std::endl;


		for(int i=0;i<PopulationSize;i++){

			print<<std::endl<<"Cost="<<solutionsCosts[i]<<", ";
			print<<"Solution:"<<i<<" Drivers: "<<solutions[i].size()<<", Riders: "<<dictSize(solutions[i])<<"\t";
			for(const auto & j : solutions[i]){
				print<<j.first->id()<<"->";
				for(const auto k : j.second){

					print<<k.id()<<(compare_shared_ptr(j.first, lookupVehicle[i].at(j.first->id()))?"-O-, ":"-X-, ");
				}

			}

		}
		print<<std::endl;
}



void BBO::reset_workspace() {
	EliteSolutions.clear();
	EliteAssignedRider.clear();
	EliteCosts={};
	solutions.clear();
	solutionsCosts={};
	assignedRider.clear();
	unassignedRider.clear();
	CandidateList.clear();
	lookupVehicle.clear();
	local_grid={};
	minimumCostPerGeneration={};
	mu={};
	lambda={};
	this->best_cost = InfInt;
	this->sch = best_sch = {};
	this->rte = best_rte = {};
	this->best_vehl = nullptr;
	this->matched = false;
}

