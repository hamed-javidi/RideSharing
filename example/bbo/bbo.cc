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
	std::cout<<std::endl<<"SORTING..."<<std::endl;
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
	if(!(solutionsCosts.size() == solutions.size() && solutions.size() == assignedRider.size() &&  solutions.size() ==  unassignedRider.size() && solutions.size()== lookupVehicle.size()-NumberOfElites && solutions.size()== CandidateList.size())){
		std::cout<<std::endl<<"CRITICAL ERROR SIZING"<<std::endl;
		std::cout<<solutionsCosts.size()<<std::endl;
		std::cout<<solutions.size()<<std::endl;
		std::cout<<assignedRider.size()<<std::endl;
		std::cout<<unassignedRider.size()<<std::endl;
		std::cout<<lookupVehicle.size()<<std::endl;
		std::cout<<CandidateList.size()<<std::endl;

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
			std::cout<<"Consistency Error";
			throw;
		}

	if((r->schedule().data().size() != solutions[solutionIdx][r].size()) || (r->schedule().data().size() != sum)){
		std::cout << std::endl<<"CHK--OK"<<std::endl;
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
			std::cout<<"Consistency Error CHK-T";
			throw;
		}

	if((r->schedule().data().size() != tempSolutions[solutionIdx][r].size()) || (r->schedule().data().size() != sum)){
		std::cout << std::endl<<"CHK--OK"<<std::endl;
		return 0;
	}
	return 1;
}
BBO::BBO(const std::string& name) : RSAlgorithm(name, false), grid_(10){

	best_cost=INT32_MAX;
	matched=false;
	debugMode=true;
	problemDimension=0;
	this->batch_time() = 60;

	for (uint8_t i = 1; i <= PopulationSize; ++i) {
		mu.push_back((PopulationSize + 1 - i) / (double)(PopulationSize + 1)); // emigration rate
		lambda.push_back( 1 - mu[i-1]); // immigration rate
	}
	//this->gen.seed(rd());
	//TODO: comment the below line and uncomment the above line
	this->gen.seed(1);
	std::cout<<"Solution size: "<<solutions.size()<<std::endl;
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
	std::cout<<std::endl<<"Initializing...";
	std::cout<<std::endl<<"Solution size::: "<<solutions.size()<<std::endl;
//	dict<MutableVehicleSptr, vec_t<Customer>> Temp; //A temporary map to be used below
//	dict<Customer, MutableVehicleSptr> temp1;
//	dict<Customer, vec_t<MutableVehicleSptr>> temp2;
//	dict<VehlId, MutableVehicleSptr> temp3;
//	vec_t<Customer> temp4;
		//1- filling CandidateList for each rider


	//we need to generate some population by using any algorithm or randomly
	for (int indx = 0; indx < PopulationSize; ++indx) {

//		solutions.push_back(Temp);
//		assignedRider.push_back(temp1);
//		CandidateList.push_back(temp2);
//		lookupVehicle.push_back(temp3);
//		unassignedRider.push_back(temp4);
		local_grid.push_back(this->grid_);  // make a deep copy
		std::cout<<indx<<",";
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

			std::cout<<"Exit";
			bool initial=false;
			while (!candidates.empty() && !initial) {
				auto k = candidates.begin();
				std::uniform_int_distribution<> m(0, candidates.size()-1);
				std::advance(k, m(this->gen));
				MutableVehicleSptr cand = *k;
				candidates.erase(k);
				std::cout<<"erase";

				if (cand->schedule().data().size() < 8) {
					sop_insert(*cand, cust, sch, rte);
					std::cout<<"sop ";
					if (chkcap(cand->capacity(), sch) && chktw(sch, rte)) {
						if(checkVehlStopsDuplication(cand,cust)){
							std::cout<<"Sch Duplication"<<std::endl;
							throw;
						}

						cand->set_sch(sch);  										// update grid version of the candidate
						cand->set_rte(rte);
						cand->reset_lvn();
						std::cout<<"chkcap + ";
						if(indx >= solutions.size())
							solutions.push_back({{cand,{cust}}});
						else
							solutions[indx][cand].push_back(cust);
						if(indx >= assignedRider.size())
							assignedRider.push_back({{cust,cand}});
						else if(assignedRider[indx].count(cust) == 0)
							assignedRider[indx][cust]=cand;
						else{
							std::cout<<std::endl<<"Error in inserting a record to assignRider list: current cust is exist in the list"<<std::endl;
							throw;
						}
						if(debugMode)
							print << "		End of Init function"<<std::endl;
						initial = true;
						checkSCH(indx, cand);
					}
					else {
						if(debugMode)
							print << "      skipping due to infeasible" << std::endl;
					}
				}
				else {
					if(debugMode)
						print << "      skipping due to sched_max" << std::endl;
				}
			}
			if(!initial){
				if(debugMode)
					print << "		Rider " << cust.id() << "left unassigned" << std::endl;
				if(indx >= unassignedRider.size())
					unassignedRider.push_back({cust});
				else
					unassignedRider[indx].push_back(cust);
			}
		}

		costUpdate(indx);
		if(debugMode)
			print << "		Solution " << indx << ", Assigned Rider: " << assignedRider[indx].size() << ", Unassigned: " <<unassignedRider[indx].size() << std::endl;
	}

	problemDimension=CandidateList[0].size();
//	if(!checkVehlStopsWithRiders(solutions)){
//		std::cout<<"NOT MATCH"<<std::endl;
//		throw;
//	}
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
				//std::cout<<k<<"= " <<r.first.id()<<" "<<r.second->id()<<std::endl;
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
	                		std::cout<<std::endl<<"Population "<<k<<"= " <<"Rider "<<r.first.id()<<" -> Driver "<<r.second->id()<<", Transfer to Population "<<SelectIndex<<std::endl;
	                	if(!migrate_vehicle_based(SelectIndex, k, r.second, tempSolutions,tempAssignedRider, tempUnassignedRider))
	                		std::cout<<"migrate is not done!"<<std::endl;
//	                	if(!checkVehlStopsWithRiders(tempSolutions)){
//	                		std::cout<<"NOT MATCH"<<std::endl;
//	                		throw;
//	                	}
	                	//solution_show();
	                }

//	                if (solutions[SelectIndex].count(r.second) == 0) {
//	                	//std::cout<<"AAA"<<std::endl;
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
//			std::cout<<"NOT MATCH"<<std::endl;
//			throw;
//		}
		minimumCostPerGeneration.push_back(solutionsCosts[0]);
		std::cout<<"Minimum of the generation "<< Generation<< " is: " <<minimumCostPerGeneration[Generation]<<std::endl;
		if(debugMode){
			solution_show();
			std::cout<<std::endl;
		}

	}

}
bool BBO::checkVehlStopsWithRiders(const vec_t<dict<MutableVehicleSptr, vec_t<Customer>>> & sol){
	int solCnt=0;

	for(auto const & i: sol){
		for(auto const & rec : i){
			if((rec.first->schedule().size()-2) != rec.second.size()*2){
				std::cout<<"Solution: "<<solCnt<<" , Vehl id: "<<rec.first->id()<<std::endl;
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
			std::cout<<"Rider "<<cust.id()<<" already exist in the vehl  "<<vehl->id()<<std::endl;
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
	std::cout<<std::endl<<"	Migration function is started:"<<std::endl;
	auto t_tempSolution=tempSolutions;
	auto t_tempAssignedRider=tempAssignedRider;
	auto t_tempUnassignedRider=tempUnassignedRider;

	//create a new vehicle based on selected vehicle if it does not exist in the destination

	vec_t<Customer> copyRiders={};
	MutableVehicleSptr const copyVehl=lookupVehicle[popIndxDest].at(r->id());
	vec_t<Customer> alreadyAsssignRider={},reassignRiders={},toBeMigratedRiders={};



	if(t_tempSolution[popIndxSrc].count(r))
		for(auto const &i: t_tempSolution[popIndxSrc][r])
			copyRiders.push_back(i);
	print << "	step 1"<<std::endl;
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

	for(auto const &i : alreadyAsssignRider){
		std::pair<bool, int> result =findInVector<Customer>(copyRiders,i);
		if(!result.first)
			toBeMigratedRiders.push_back(copyRiders[result.second]);
	}

	if(debugMode){
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
	}
	//2. Has any of the copy riders been assigned to some vehicles else in the target solution?
	print <<"	step 2"<<std::endl;
	//2.YES, deassign them from those vehicles in the target solution
	for(auto const &i: toBeMigratedRiders){
		MutableVehicleSptr cand=nullptr;
		if(t_tempAssignedRider[popIndxDest].count(i) > 0)
			cand=t_tempAssignedRider[popIndxDest][i];
		if(cand == nullptr)  //rider i has not been assigned in destination
			continue;

		std::cout<<"S:"<<popIndxSrc<<" D:"<<popIndxDest<<" Vehl:"<<r->id()<<" Rider:"<<i.id()<<std::endl;
		if(t_tempSolution[popIndxDest].count(cand)){
			for(auto j=(t_tempSolution[popIndxDest][cand]).begin();j!=(t_tempSolution[popIndxDest][cand]).end(); j++){
				std::cout<<"j="<<j->id()<<std::endl;
				if(j->id() == i.id()){
					t_tempSolution[popIndxDest][cand].erase(j);
					//update vehicle's route and schedule

					std::vector<Stop>  sch_after_rem = cand->schedule().data();
					std::vector<Wayp> rte_after_rem;
					opdel(sch_after_rem, j->id());
					route_through(sch_after_rem, rte_after_rem);

					// Add the traveled distance to rte_after_rem <<<<<
					for (Wayp& wp : rte_after_rem)
						wp.first += cand->route().dist_at(cand->idx_last_visited_node());

					cand->set_sch(sch_after_rem);
					cand->set_rte(rte_after_rem);
					cand->reset_lvn();
					t_tempAssignedRider[popIndxDest].erase(*j);
					if(checkVehlStopsDuplication(cand,*j)){
						std::cout<<"Sch Duplication"<<std::endl;
						throw;
					}
					checkSCHTemp(popIndxDest,cand, t_tempAssignedRider, t_tempSolution);
					break;
				}
			}
		}
		else
			std::cout<<"Rider "<<i.id()<<"is already unassigned in target solution";
	}
	std::cout<<"step 3"<<std::endl;
	// 3. Does some riders else already assigned to the selected vehicle in the target solution?
	//3. YES: Deassign them and put them to a waiting list to reassign later

	for(const auto &i: reassignRiders){
		//MutableVehicleSptr cand=t_tempAssignedRider[popIndxDest][i];
		if(t_tempSolution[popIndxDest].count(copyVehl)){

			for(auto j=(t_tempSolution[popIndxDest][copyVehl]).begin();j!=(t_tempSolution[popIndxDest][copyVehl]).end(); j++){
				if(j->id() == i.id()){
					t_tempSolution[popIndxDest][copyVehl].erase(j);
					//update vehicle's route and schedule
					std::vector<Stop>  sch_after_rem = copyVehl->schedule().data();
					std::vector<Wayp> rte_after_rem;
					opdel(sch_after_rem, j->id());
					route_through(sch_after_rem, rte_after_rem);

					// Add the traveled distance to rte_after_rem <<<<<
					for (Wayp& wp : rte_after_rem)
						wp.first += copyVehl->route().dist_at(copyVehl->idx_last_visited_node());

					copyVehl->set_sch(sch_after_rem);
					copyVehl->set_rte(rte_after_rem);
					copyVehl->reset_lvn();
					t_tempAssignedRider[popIndxDest].erase(*j);
					if(checkVehlStopsDuplication(copyVehl,*j)){
						std::cout<<"Sch Duplication"<<std::endl;
						throw;
					}
					checkSCHTemp(popIndxDest,copyVehl, t_tempAssignedRider, t_tempSolution);
					break;
				}
			}
		}
	}

	print <<"	step 3-2"<<std::endl;
	//3. No: Assign the selected riders to the selected vehicle in the target solution
	for(const auto &i: copyRiders){
		auto it = find_if(alreadyAsssignRider.begin(),alreadyAsssignRider.end(), [&i](const Customer& obj) {return obj.id() == i.id();});

		if (it != alreadyAsssignRider.end())
			continue;
		else{
			sop_insert(copyVehl, i, sch, rte);
			if(checkVehlStopsDuplication(copyVehl,i)){
				std::cout<<"Sch Duplication"<<std::endl;
				throw;
			}
			copyVehl->set_sch(sch);
			copyVehl->set_rte(rte);
			copyVehl->reset_lvn();
			t_tempAssignedRider[popIndxDest][i]=copyVehl;
			t_tempSolution[popIndxDest][copyVehl].push_back(i);
			checkSCHTemp(popIndxDest,copyVehl, t_tempAssignedRider, t_tempSolution);
		}
	}
	std::cout<<"step 4"<<std::endl;
	//4. Is the waiting list empty?
	if(reassignRiders.size() != 0){

		//4. NO: Reassign them to vehicles which the best candidate in the target solution
		for(auto const &i: reassignRiders)
			if(!Greedy_Assignment(popIndxDest,i, t_tempSolution, t_tempAssignedRider,t_tempUnassignedRider)){
				//exit without saving migration results
				//return 0;
				;
			}
	}

	//4. YES: Delete empty vehicles	from target solution
	for(auto i= t_tempSolution[popIndxDest].begin(); i!=t_tempSolution[popIndxDest].end(); )
		if(i->second.size() == 0 )
			i = t_tempSolution[popIndxDest].erase(i);
		else if(i->first->schedule().data().size()<=1){
			std::cout<<"Something wrong  "<<i->first->schedule().data().size()<<std::endl;
			throw;
		}
		else
			++i;
	std::cout<<"step saving"<<std::endl;
	//exit with saving migration results
	tempSolutions=t_tempSolution;
	tempAssignedRider=t_tempAssignedRider;
	tempUnassignedRider=t_tempUnassignedRider;
	std::cout<<std::endl<<"exit migration"<<std::endl;
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
	for (const MutableVehicleSptr& cand : CandidateList[solIndex].at(cust)) {
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
		std::cout<<"Rider "<<cust.id()<<"left unassigned"<<std::endl;
		tempUnassignedRider[solIndex].push_back(cust);

		return 0;
	}
	else{
		if(checkVehlStopsDuplication(best_vehl,cust)){
			std::cout<<"Sch Duplication"<<std::endl;
			throw;
		}
		best_vehl->set_sch(best_sch);  										// update grid version of the candidate
		best_vehl->set_rte(best_rte);
		best_vehl->reset_lvn();

		tempSolutions[solIndex][best_vehl].push_back(cust);  		// update our copy of the candidate
		tempAssignedRider[solIndex][cust]=best_vehl;
		checkSCHTemp(solIndex,best_vehl, tempAssignedRider, tempSolutions);
		std::cout<<"Rider "<<cust.id()<<" reassign to driver : "<<best_vehl->id()<<"with cost "<<best_cost<<std::endl;
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

	std::cout<<"Batch #: "<<(++(this->batchCounter))<<", requests: "<<this->customers().size()<< std::endl;
	bbo_init();
	if(debugMode){
		solution_show();
		int sum[PopulationSize]={0};
		if(debugMode){
			for(int i=0;i<PopulationSize;i++){
				for(const auto & j:solutions[i])
					if(compare_shared_ptr(j.first, lookupVehicle[i].at(j.first->id())))
						sum[i]++;
				std::cout<<sum[i]<<std::endl;
			}
		}
	}
	doSort();
	if(debugMode)
		solution_show();

	bbo_body();
	commit();

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
//		std::cout<<std::endl<<"Cost="<<solutionsCosts[indx]<<", ";
//		std::cout<<"Solution:"<<indx++<<" ";
//		for(auto &j : i){
//			std::cout<<j.first.id()<<"->"<<j.second->id()<<", ";
//		}
//
//	}
//	std::cout<<std::endl;


		for(int i=0;i<PopulationSize;i++){

			std::cout<<std::endl<<"Cost="<<solutionsCosts[i]<<", ";
			std::cout<<"Solution:"<<i<<" Drivers: "<<solutions[i].size()<<", Riders: "<<dictSize(solutions[i])<<"\t";
			for(const auto & j : solutions[i]){
				std::cout<<j.first->id()<<"->";
				for(const auto k : j.second){

					std::cout<<k.id()<<(compare_shared_ptr(j.first, lookupVehicle[i].at(j.first->id()))?"-O-, ":"-X-, ");
				}

			}

		}
		std::cout<<std::endl;
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

