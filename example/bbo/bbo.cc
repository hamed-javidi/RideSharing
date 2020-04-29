#include <stdlib.h>

#include <iostream>
#include <queue>
#include <tuple>
#include <vector>
#include <cstdint>
#include <cstdlib>      // std::rand, std::srand

#include "libcargo.h"
#include "bbo.h"
#include "libcargo/distance.h"

#include "matplotlib-cpp-master/matplotlibcpp.h"

using namespace cargo;
using namespace dlib;


#define _USE_MATH_DEFINES
namespace plt = matplotlibcpp;

BBO::BBO(const std::string& name) : RSAlgorithm(name, false), grid_(10){

	//features configuration
	best_cost=INT32_MAX;
	NumberOfElites= maxNumberOfElites;
	PopulationSize=maxPopulationSize;
	matched=false;

	problemDimension=0;
	this->batch_time() = 30;

	for (uint8_t i = 1; i <= PopulationSize; ++i) {
		mu.push_back((PopulationSize + 1 - i) / (double)(PopulationSize + 1)); // emigration rate
		lambda.push_back( 1 - mu[i-1]); // immigration rate
	}
	//this->gen.seed(rd());
	//TODO: comment the below line and uncomment the above line
	this->gen.seed(1);


	assignedRider.reserve(PopulationSize);
	unassignedRider.reserve(PopulationSize);
	solutions.reserve(PopulationSize);
	solutionsCosts.reserve(PopulationSize);
	lookupVehicle.reserve(PopulationSize);
	CandidateList.reserve(PopulationSize);

	elitesAssignedRider.reserve(NumberOfElites);
	elitesUnassignedRider.reserve(NumberOfElites);
	elitesSolutions.reserve(NumberOfElites);
	elitesCosts.reserve(NumberOfElites);
	elitesLookupVehicle.reserve(NumberOfElites);
	elitesCandidateList.reserve(NumberOfElites);

	minimumCostPerGeneration.reserve(GenerationLimit);
}



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
	sortedIdx.resize(solutionsCosts.size());
	const auto cost=solutionsCosts;
	//put 1,2,3 to size of idx vector in idx
	iota(sortedIdx.begin(), sortedIdx.end(), 0);
	// sort indexes based on comparing values in v
	sort(sortedIdx.begin(), sortedIdx.end(),[&cost](size_t i1, size_t i2) {return cost[i1] < cost[i2];});

	auto const  tcost=solutionsCosts;
	auto const  tsol=solutions;
	auto const  assignt=assignedRider;
	auto const  unassignt=unassignedRider;
	auto const  tlookupv=lookupVehicle;
	auto const  tcandlist=CandidateList;
	//vec_t<Grid> tgrid=new vec_t<Grid>(local_grid,a);
//	vec_t<Grid> tgrid(solutions.size());
//	int m=0;
//	for(auto const &k : local_grid){
//		tgrid[m].x_dim_ = k.x_dim_;
//		tgrid[m].y_dim_ = k.y_dim_;
//		tgrid[m].n_     = k.n_;
//		tgrid[m].data_.resize(k.data_.size());
//		for (size_t i = 0; i < k.data_.size(); ++i) {
//			tgrid[m].data_[i].resize(k.data_.at(i).size());
//			for (size_t j = 0; j < k.data_.at(i).size(); ++j) {
//				tgrid[m].data_.at(i)[j] = k.data_.at(i).at(j);
//			}
//		}
//	}

	//vec_t<Grid> tgrid(1.2,8);
	int j=0;
	if(!(solutionsCosts.size() == solutions.size() && solutions.size() == assignedRider.size() &&  solutions.size() ==  unassignedRider.size() && solutions.size()== CandidateList.size()&&  solutions.size()== lookupVehicle.size())){
		print<<std::endl<<"CRITICAL ERROR SIZING"<<std::endl;
		print<<solutionsCosts.size()<<std::endl;
		print<<solutions.size()<<std::endl;
		print<<assignedRider.size()<<std::endl;
		print<<unassignedRider.size()<<std::endl;
		print<<lookupVehicle.size()<<std::endl;
		print<<CandidateList.size()<<std::endl;
		//print<<local_grid.size()<<std::endl;
	}

	for(auto &i : sortedIdx){
//		local_grid[j]=tgrid[i];
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
bool BBO::checkVehlStopsWithRiders(const vec_t<dict<MutableVehicleSptr, vec_t<Customer>>> & sol){
	int solCnt=0;
	int sum;
	for(auto const & solIdx: sol){
		for(auto const & obj : solIdx)
			for(auto const &rider : obj.second){
				sum=0;
				for(auto const stop: obj.first->schedule().data())
					if(stop.owner()==rider.id())
						sum++;
				if(sum!=2){
					print<<"Solution: "<<solCnt<<", Vehl "<<obj.first->id()<<"stops are not match with rider "<<rider.id()<<std::endl;
					return 0;
				}
			}
		solCnt++;
	}

	return 1;
}
bool BBO::checkVehileConsistencyInAllStructures(	const dict<MutableVehicleSptr, vec_t<Customer>> & solutions,
													const dict<Customer, MutableVehicleSptr> & assignedRider,
													const dict<VehlId,MutableVehicleSptr> & lookupVehl,
													const dict<Customer, vec_t<MutableVehicleSptr>> & CandidateList	){
	for(auto const &cand : solutions)
		if(cand.first != lookupVehl.at(cand.first->id())){
		  std::cout<<"Vehl's pointer in solutions "<<cand.first<<" not match with "<<lookupVehl.at(cand.first->id())<<std::endl;
		  return 0;
		}
	for(auto const &cand : assignedRider)
		if(cand.second != lookupVehl.at(cand.second->id())){
		  std::cout<<"Vehl's pointer in solutions "<<cand.second<<" not match with "<<lookupVehl.at(cand.second->id())<<std::endl;
		  return 0;
		}
	for(auto const &obj : CandidateList)
		for(auto const & cand : obj.second)
			if(cand != lookupVehl.at(cand->id())){
			  std::cout<<"Vehl's pointer in solutions "<<cand<<" not match with "<<lookupVehl.at(cand->id())<<std::endl;
			  return 0;
			}
	return  1;
}
bool BBO::checkSCHTemp(int solutionIdx, MutableVehicleSptr const & r, vec_t<dict<Customer, MutableVehicleSptr>> const  &tempAssignedRider, vec_t<dict<MutableVehicleSptr, vec_t<Customer>>> const &tempSolutions){
	int sum=0;
	for(auto const &i : tempAssignedRider[solutionIdx])
		if(i.second == r)
			sum++;
		else if(i.second->id() == r->id()){
			print<<"	Consistency Error CHK-T";
			throw;
		}
	if(tempSolutions[solutionIdx].count(r)!=0)
		if(r->schedule().data().size()-2 != tempSolutions[solutionIdx].at(r).size()*2){
			print <<"	CHK--NOT OK"<<std::endl;
			throw;
		}
	if((r->schedule().data().size()-2 != sum*2)){
		print <<"	CHK--NOT OK"<<std::endl;
		throw;
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
//bool BBO::vehlPointerDuplication(){
//	for(auto const & i : vehl->schedule().data())
//		if(i.owner()==cust.id()){
//			print<<"Rider "<<cust.id()<<" already exist in the vehl  "<<vehl->id()<<std::endl;
//			return 1;
//		}
//
//	return 0;
//}
void BBO::selectElites(){
	//NumberOfElites=maxNumberOfElites;
	for (size_t i = 0; i < (size_t)NumberOfElites; ++i){
//			if(i>0)
//				if(solutionsCosts[i]==solutionsCosts[i-1]){
//					NumberOfElites--;
//					break;
//				}

		//elites_grid.push_back(local_grid[sortedIdx[i]]);//
//		if(elites_grid[i].get_size()==0)
//			print<< "elites_grid: "<<i<<" is null++++++++++++++++++++++"<<std::endl;
		for(auto const & record : lookupVehicle[i]){
			MutableVehicle newCand = *record.second;
			auto newCandSptr = std::make_shared<MutableVehicle>(newCand);
			if(i >= (elitesLookupVehicle.size()))
				elitesLookupVehicle.push_back({{record.first,newCandSptr}});
			else
				elitesLookupVehicle[i][record.first]=newCandSptr;
		}
		for(auto const & record : solutions[i]){
			auto newCandSptr = elitesLookupVehicle[i].at(record.first->id());
			if(i >= elitesSolutions.size())
				elitesSolutions.push_back({{newCandSptr,record.second}});
			else
				elitesSolutions[i][newCandSptr]=record.second;
			if(debugMode)
				if(!checkVehlStopsWithRiders(elitesSolutions)){
					print<<"NOT MATCH"<<std::endl;
					throw;
				}
		}
		for(auto const & record : assignedRider[i]){
			auto newCandSptr = elitesLookupVehicle[i].at(record.second->id());
			if(i >= elitesAssignedRider.size())
				elitesAssignedRider.push_back({{record.first,newCandSptr}});
			else
				elitesAssignedRider[i][record.first]=newCandSptr;
		}

		for(auto const & record : CandidateList[i]){
			vec_t<MutableVehicleSptr> cands;
			for(auto const & cand : record.second){
				auto newCand = elitesLookupVehicle[i].at(cand->id());
				cands.push_back(newCand);
			}
			if(i >= elitesCandidateList.size())
				elitesCandidateList.push_back({{record.first,cands}});
			else
				elitesCandidateList[i][record.first]=cands;
		}

		elitesUnassignedRider.push_back(unassignedRider[i]);
		elitesCosts.push_back(solutionsCosts[i]);
	}
}
void BBO::insertElits(){
	for (uint8_t i = 0; i < NumberOfElites; i++){
		solutions.pop_back();
		lookupVehicle.pop_back();
		assignedRider.pop_back();
		unassignedRider.pop_back();
		solutionsCosts.pop_back();
//		local_grid.pop_back();
		CandidateList.pop_back();
	}
	for (uint8_t i = 0; i < NumberOfElites; i++){
		solutions.push_back(elitesSolutions[i]);
		lookupVehicle.push_back(elitesLookupVehicle[i]);
		assignedRider.push_back(elitesAssignedRider[i]);
		unassignedRider.push_back(elitesUnassignedRider[i]);
		solutionsCosts.push_back(elitesCosts[i]);
//		local_grid.push_back(elites_grid[i]);
		CandidateList.push_back(elitesCandidateList[i]);
	}

//		solutions[PopulationSize-1-i]=		elitesSolutions[i];
//		lookupVehicle[PopulationSize-1-i]=	elitesLookupVehicle[i];
//		assignedRider[PopulationSize-1-i]=	elitesAssignedRider[i];
//		unassignedRider[PopulationSize-1-i]=elitesUnassignedRider[i];
//		solutionsCosts[PopulationSize-1-i]= elitesCosts[i];
//		local_grid[PopulationSize-1-i]=		elites_grid[i];
//		CandidateList[PopulationSize-1-i]=	elitesCandidateList[i];


}

void BBO::bbo_mutation(){

//	for (uint8_t k = 0; k < PopulationSize; ++k)
//		for (int ParameterIndex = 0;  ParameterIndex < problemDimension; ParameterIndex++)
//			if ((rd()%1) < MutationProbability)
//				//tempSolutions(k, ParameterIndex) = MinDomain + (MaxDomain - MinDomain) * rand();
//				;


}
bool BBO::PutMatchedHistory(const MutableVehicleSptr cand, const Customer & cust, vec_t<Stop> & sch, vec_t<Wayp> & rte, DistInt & cost, const bool is_matched){
	vec_t<TripId> stops={};
	for(auto i : cand->schedule().data())
		stops.push_back(i.owner());
	if(is_matched == true)
		matchHist[std::make_tuple(cust,cand)] = std::make_tuple(stops, is_matched, sch, rte, cost);
	else{
		sch={};	rte={}; cost=0;
		matchHist[std::make_tuple(cust,cand)] = std::make_tuple(stops, is_matched, sch, rte, cost);
	}
	return true; //no error
}
MutableVehicleSptr BBO::GetMatchedHistory(const MutableVehicleSptr cand, const Customer & cust, vec_t<Stop> & sch, vec_t<Wayp> & rte, DistInt & cost, bool & is_matched){
	auto itr = matchHist.find(std::make_tuple(cust,cand));
	if(matchHist.end() != itr){
		vec_t<TripId> stops={};
		for(auto i : cand->schedule().data())
			stops.push_back(i.owner());
		if((std::get<0>(itr->second) == stops) && (std::get<1>(itr->second)== true)){
			sch = std::get<2>(itr->second);
			rte = std::get<3>(itr->second);
			cost = std::get<4>(itr->second);
			is_matched=true;
			return std::get<1>(itr->first); //record is found
		}
		else if((std::get<0>(itr->second)==stops) && (std::get<1>(itr->second)== false)){
			is_matched=false;
			return std::get<1>(itr->first);  //record is found
		}

	}
	return nullptr; // record is not found
}
bool BBO::UpdateMatchedStructures(const uint8_t indx, const MutableVehicleSptr cand, const Customer & cust){
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
		return false;
	}
	if(debugMode)
		print << "	Rider "<<cust.id()<<"is assigned to driver "<<cand->id()<<std::endl;
	return true;
}
bool BBO::UpdateUnmatchedStructures(const uint8_t indx, const Customer & cust){
	if(indx >= unassignedRider.size())
		unassignedRider.push_back({cust});
	else
		unassignedRider[indx].push_back(cust);
	if(debugMode)
		print << "	Rider " << cust.id() << "left unassigned" << std::endl;

	return true;
}
MutableVehicleSptr BBO::initUsingGreedy(const Customer cust, const vec_t<MutableVehicleSptr> &candidates){

	MutableVehicleSptr best_vehl=nullptr;
	DistInt best_cost = InfInt;
	vec_t<Stop> sch,best_sch = {};
	vec_t<Wayp> rte,best_rte = {};

	for (auto const cand : candidates) {
		bool is_matched;
		DistInt cur_cost = cand->route().cost();
		DistInt cost=0, new_cost=0;
		if(GetMatchedHistory(cand, cust, sch, rte, new_cost, is_matched) != nullptr){
			if(is_matched){
				cost = new_cost - cur_cost;
				if (cost < best_cost) {
					//no need to check capacity and time window
					best_vehl = cand;
					best_sch = sch;
					best_rte = rte;
					best_cost = cost;
				}
			}
		}
		else{
			if (cand->schedule().data().size() < 8) {
				new_cost = sop_insert(*cand, cust, sch, rte);
				cost = new_cost - cur_cost;
				if (chkcap(cand->capacity(), sch) && chktw(sch, rte)) {
					PutMatchedHistory(cand, cust, sch, rte, new_cost, true);
					if (cost < best_cost) {
						best_vehl = cand;
						best_sch = sch;
						best_rte = rte;
						best_cost = cost;
					}
				}
				else
					PutMatchedHistory(cand,cust, sch,rte, new_cost, false);
			}
		}
	}
	if(best_vehl != nullptr){
		best_vehl->set_sch(best_sch);
		best_vehl->set_rte(best_rte);
		best_vehl->reset_lvn();
	}
	return best_vehl;
}
MutableVehicleSptr BBO::initRandomely(const Customer cust, vec_t<MutableVehicleSptr> &candidates){
	vec_t<Stop> sch = {};
	vec_t<Wayp> rte = {};
	DistInt cost=0;
	for (auto cand : candidates) {
		bool is_matched;
		if(GetMatchedHistory(cand, cust, sch, rte, cost, is_matched) != nullptr){
			if(is_matched){
				if(debugMode)
					if(checkVehlStopsDuplication(cand,cust)){
						print<<"Sch Duplication"<<std::endl;
						throw;
					}
				cand->set_sch(sch);
				cand->set_rte(rte);
				cand->reset_lvn();

				return cand;
			}
		}
		else{
			if (cand->schedule().data().size() < 8) {
				sch={}; rte={};
				cost = sop_insert(*cand, cust, sch, rte);
				if (chkcap(cand->capacity(), sch) && chktw(sch, rte)) {
					PutMatchedHistory(cand,cust, sch, rte, cost, true);
					if(debugMode)
						if(checkVehlStopsDuplication(cand,cust)){
							print<<"Sch Duplication"<<std::endl;
							throw;
						}
					cand->set_sch(sch);
					cand->set_rte(rte);
					cand->reset_lvn();

					return cand;//
				}
				else {
					sch={}; rte={}; cost=0;
					PutMatchedHistory(cand, cust, sch, rte, cost, false);
					if(debugMode)
						print << "      skipping due to infeasible" << std::endl;
				}
			}

		}
	}
	return nullptr;
}
bool BBO::Greedy_Assignment(	int 												solIndex,
								const Customer 										& cust,
								vec_t <dict<MutableVehicleSptr, vec_t<Customer>>> 	& temp_solutions,
								vec_t <dict<Customer, MutableVehicleSptr>> 			& temp_assignedRider,
								vec_t <vec_t<Customer>> 							& temp_unassignedRider,
								rollBackHistory 									& rollback_hist
								){
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
		temp_unassignedRider[solIndex].push_back(cust);

		return 0;
	}
	else{
		if(debugMode)
			if(checkVehlStopsDuplication(best_vehl,cust)){
				print<<"Sch Duplication"<<std::endl;
				throw;
			}
		if(rollback_hist.count(best_vehl) == 0)
			rollback_hist[best_vehl] = std::make_tuple(best_vehl -> schedule().data(), best_vehl -> route().data());

		best_vehl->set_sch(best_sch);  										// update grid version of the candidate
		best_vehl->set_rte(best_rte);
		best_vehl->reset_lvn();

		temp_solutions[solIndex][best_vehl].push_back(cust);  		// update our copy of the candidate
		temp_assignedRider[solIndex][cust]=best_vehl;
//		checkSCHTemp(solIndex,best_vehl, tempAssignedRider, tempSolutions);
		if(debugMode)
			print <<"	Rider "<<cust.id()<<" Assign to driver : "<<temp_assignedRider[solIndex][cust]->id()<<" with cost "<<best_cost<< " using Greedy"<<std::endl;
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
    //checkSCH(0, cand);
  }
}
void BBO::solutionsCostUpdate(){
	for(uint8_t indx=0; indx< solutions.size(); indx++){
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
}

//uint16_t dictSize(dict<MutableVehicleSptr, vec_t<Customer>> d){
//	uint16_t sum=0;
//	for(auto i : d)
//		for(auto j : i.second)
//			sum++;
//	return sum;
//}


void BBO::solution_show(){
	for(int i=0;i<PopulationSize;i++){

		print<<"	Cost = "<<solutionsCosts[i]<<", ";
		print<<"	Solution: "<<i<<" Drivers: "<<solutions[i].size()<<", Riders: "<<solutions[i].size()<<"\t";
		for(const auto & j : solutions[i]){
			print<<j.first->id()<<"->";
			if( j.second.size())
				for(const auto k : j.second){

					print<<k.id()<<", ";//<<(compare_shared_ptr(j.first, lookupVehicle[i].at(j.first->id()))?"-O-, ":"-X-, ");
				}
			else
				print<<"adsfgfsfdgdfs"<<std::endl;
			print<<"\t";
		}
		print<<std::endl;
	}

}



bool BBO::migrate_vehicle_based(int popIndxDest, int popIndxSrc,
									const MutableVehicleSptr r,
									vec_t<dict<MutableVehicleSptr, vec_t<Customer>>> & tempSolutions,
									vec_t<dict<Customer, MutableVehicleSptr>> &tempAssignedRider,
									vec_t<vec_t<Customer>> &tempUnassignedRider){
	//In case of ROLL BACK migration
	bool commitChanges=1;
	rollBackHistory rollBackHist;
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
	rollBackHist[copyVehl]=std::make_tuple(copyVehl->schedule().data(),copyVehl->route().data());
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
	if(debugMode)
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
		if(debugMode)
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
			if(rollBackHist.count(cand)==0)
				rollBackHist[cand]=std::make_tuple(cand->schedule().data(),cand->route().data());
			//else
				//this cand exist in the hist
			t_tempSolution[popIndxDest][cand].erase(std::remove(t_tempSolution[popIndxDest][cand].begin(), t_tempSolution[popIndxDest][cand].end(), i), t_tempSolution[popIndxDest][cand].end());
			if(t_tempSolution[popIndxDest][cand].size() == 0 ){
				t_tempSolution[popIndxDest].erase(cand);
				if(debugMode)
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
			if(debugMode)
				if(checkVehlStopsDuplication(cand,i)){
					print<<"	Sch Duplication"<<std::endl;
					throw;
				}
//			checkSCHTemp(popIndxDest,cand, t_tempAssignedRider, t_tempSolution);
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
//			checkSCHTemp(popIndxDest,copyVehl, t_tempAssignedRider, t_tempSolution);
			if(debugMode)
				print <<"	Rider "<<i.id()<<"is deassigned from "<<copyVehl->id()<<std::endl;
		}
	}
	if(debugMode)
		print <<"	step 3-2"<<std::endl;
	//3. No: Assign the selected riders to the selected vehicle in the target solution
	for(const auto &i: toBeMigratedRiders){
		sop_insert(copyVehl, i, sch, rte);
		if(debugMode)
			if(checkVehlStopsDuplication(copyVehl,i)){
				print<<"Sch Duplication"<<std::endl;
				throw;
			}
		copyVehl->set_sch(sch);
		copyVehl->set_rte(rte);
		copyVehl->reset_lvn();
		t_tempAssignedRider[popIndxDest][i]=copyVehl;
		t_tempSolution[popIndxDest][copyVehl].push_back(i);
//		checkSCHTemp(popIndxDest,copyVehl, t_tempAssignedRider, t_tempSolution);
		if(debugMode)
			print <<"	Rider "<<i.id()<<"is assigned to "<<copyVehl->id()<<std::endl;
	}
	if(debugMode)
		print<<"	step 4"<<std::endl;
	//4. Is the waiting list empty?

	//4. NO: Reassign them to vehicles which the best candidate in the target solution
	for(auto const &i: reassignRiders)
		if(!Greedy_Assignment(popIndxDest,i, t_tempSolution, t_tempAssignedRider,t_tempUnassignedRider, rollBackHist)){
			if(rollBack){
				if(debugMode)
					print<<"	roll back becuase of rider "<<i.id()<<" is unmatched!"<<std::endl;
				commitChanges=0;
			}
		}

//	//4. YES: Delete empty vehicles	from target solution
//	for(auto i= t_tempSolution[popIndxDest].begin(); i!=t_tempSolution[popIndxDest].end(); )
//		if(i->second.size() == 0 )
//			i = t_tempSolution[popIndxDest].erase(i);
//		else if(i->first->schedule().data().size()<=1){
//			std::cout<<"Something wrong  "<<i->first->schedule().data().size()<<std::endl;
//			throw;
//		}
//		else
//			++i;

	if(commitChanges){
		if(debugMode)
			print <<"	Migration is done!"<<std::endl;
		tempSolutions=t_tempSolution;
		tempAssignedRider=t_tempAssignedRider;
		tempUnassignedRider=t_tempUnassignedRider;
		return 1;
	}
	else{ //roll back steps
		for(auto const record : rollBackHist){
			std::get<0>(record.first)->set_sch(std::get<0>(record.second));
			std::get<0>(record.first)->set_rte(std::get<1>(record.second));
			std::get<0>(record.first)->reset_lvn();
		}
	}
	return 0;
}

void BBO::bbo_init(){
	matchHist.clear();
	if(debugMode)
		print<<std::endl<<"Initializing...";
	bool hybridIter = false;
	int hybridIterNum;
	if(hybridInit)
		hybridIterNum = PopulationSize - int(PopulationSize * (hybridInitPercent / 100.0));
	for (int indx = 0; indx < PopulationSize; ++indx)
			local_grid.push_back(this->grid_);  // make a deep copy
	//1- filling CandidateList for each rider
	//we need to generate some population by using any algorithm or randomly

	std::srand ( unsigned ( std::time(0) ) );
	for (uint8_t indx = 0; indx < PopulationSize; ++indx) {
		if(indx >= hybridIterNum)
			hybridIter = true;
		auto customers = this->customers();
		std::random_shuffle ( customers.begin(), customers.end() );
		for (const auto & cust : customers ) {
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
			bool matched=false;
			MutableVehicleSptr cand;

			if(hybridIter == 0){
				std::random_shuffle ( candidates.begin(), candidates.end() );
				cand = initRandomely(cust, candidates);
				if (cand!=nullptr){
					matched = true;
				}
			}
			else{//hybrid init mode
				cand = initUsingGreedy(cust, candidates);
				if (cand!=nullptr){
					matched = true;
				}
			}

			if(!matched)
				UpdateUnmatchedStructures(indx, cust);
			else
				UpdateMatchedStructures(indx, cand, cust);
		}
		if(debugMode){
			print << "	Solution " << (int)indx << ", Assigned Rider: " << assignedRider[indx].size() << ", Unassigned: " <<unassignedRider[indx].size() << std::endl;
			checkVehileConsistencyInAllStructures(solutions[indx], assignedRider[indx], lookupVehicle[indx], CandidateList[indx]);
		}
	}
	if(debugMode)
		if(!checkVehlStopsWithRiders(solutions)){
			print<<"NOT MATCH"<<std::endl;
			throw;
		}
	PopulationSize=solutions.size();
//	if(PopulationSize<maxPopulationSize){
//		print<<"NUMBER OF POP"<<std::endl;
//
//	}
	solutionsCostUpdate();

}

void BBO::bbo_body(){
	if(debugMode)
		for (uint8_t i = 0; i < PopulationSize; ++i)
			checkVehileConsistencyInAllStructures(solutions[i], assignedRider[i], lookupVehicle[i], CandidateList[i]);

	if(PopulationSize==0)
		return;
	mu={};
	lambda={};
	for (uint8_t i = 1; i <= PopulationSize; ++i) {
		mu.push_back((PopulationSize + 1 - i) / (double)(PopulationSize + 1)); // emigration rate
		lambda.push_back( 1 - mu[i-1]); // immigration rate
	}

	for (int Generation = 0; Generation< GenerationLimit; Generation++){
		// Save the best solutions and costs in the elite arrays
		selectElites();
		if(debugMode)
			if(!checkVehlStopsWithRiders(solutions)){
				print<<"NOT MATCH"<<std::endl;
				throw;
			}
	    auto tempSolutions = solutions;
	    auto tempAssignedRider = assignedRider;
	    auto tempUnassignedRider=unassignedRider;
	    //auto tempLocal_grid=local_grid;
	    //Use migration rates to decide how much information to share between solutions
	    for (int k = 0; k < PopulationSize; ++k) {
			for(auto const &r : tempAssignedRider[k]){
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
	                		print<<std::endl<<"Population "<<SelectIndex<<"= " <<"Rider "<<r.first.id()<<" -> Driver "<<r.second->id()<<", Transfer to Population "<<k<<std::endl;
	                	if(!migrate_vehicle_based(k, SelectIndex, r.second, tempSolutions,tempAssignedRider, tempUnassignedRider))
	                		if(debugMode)
	                			print<<"migrate is not done!"<<std::endl;
	                	if(debugMode){
							if(!checkVehlStopsWithRiders(tempSolutions)){
								print<<"NOT MATCH"<<std::endl;
								throw;
							}
							if(debugMode)
								checkVehileConsistencyInAllStructures(tempSolutions[k], tempAssignedRider[k], lookupVehicle[k], CandidateList[k]);
	                	}
	                }
	        	}
	        }
		}
		//TODO: bbo_mutation();
		solutions=tempSolutions;
		assignedRider=tempAssignedRider;
		unassignedRider=tempUnassignedRider;
		if(debugMode)
			if(!checkVehlStopsWithRiders(solutions)){
				print<<"NOT MATCH"<<std::endl;
				throw;
			}
		solutionsCostUpdate();
		doSort();
		if(debugMode)
			for (uint8_t i = 0; i < PopulationSize; ++i)
				checkVehileConsistencyInAllStructures(solutions[i], assignedRider[i], lookupVehicle[i], CandidateList[i]);
		insertElits();
		if(debugMode){
			for (uint8_t i = 0; i < PopulationSize; ++i)
				checkVehileConsistencyInAllStructures(solutions[i], assignedRider[i], lookupVehicle[i], CandidateList[i]);
			if(!checkVehlStopsWithRiders(solutions)){
				print<<"NOT MATCH"<<std::endl;
				throw;
			}
		}
		doSort();
		//Clear ELITES to be ready to use in the next generation
		elitesSolutions.clear();
		elitesCosts={};
		elitesAssignedRider.clear();
		elitesUnassignedRider.clear();
		elitesCandidateList.clear();
		elitesLookupVehicle.clear();
//		elites_grid={};


		if(debugMode){
			solution_show();
			print<<std::endl;
		}
		minimumCostPerGeneration.push_back(solutionsCosts[0]);
		print<<"Minimum of the generation "<< Generation<< " is: " <<minimumCostPerGeneration[Generation]<<std::endl;
		if(debugMode)
			for (uint8_t i = 0; i < PopulationSize; ++i)
				checkVehileConsistencyInAllStructures(solutions[i], assignedRider[i], lookupVehicle[i], CandidateList[i]);
	}
}
void BBO::DriverClustering(vec_t<Point> data, int k){
	if(data.size()==0)
		return;
	 std::vector<sample_type> samples;
	 std::vector<sample_type> initial_centers;

	kcentroid<kernel_type> kc(kernel_type(0.1),0.01, 8);
	kkmeans<kernel_type> test(kc);
	test.set_number_of_centers(k);
	sample_type m;
	for(auto i : data){
		m(0)=i.lat;
		m(1)= i.lng;
		samples.push_back(m);
	}
	pick_initial_centers(k, initial_centers, samples, test.get_kernel());
	test.train(samples,initial_centers);
	for(auto i : samples)
		print << test(i) << ", ";
	print<<std::endl;
	//hg
}
void BBO::match() {

	this->reset_workspace();
	DriverClustering(driver_pool,25);
	print<<"Batch #: "<<(++(this->batchCounter))<<", requests: "<<this->customers().size()<< std::endl;
	bbo_init();
//	if(debugMode){
//		solution_show();
//		int sum[PopulationSize]={0};
//		if(debugMode){
//			for(int i=0;i<PopulationSize;i++){
//				for(const auto & j:solutions[i])
//					if(compare_shared_ptr(j.first, lookupVehicle[i].at(j.first->id())))
//						sum[i]++;
//				print<<sum[i]<<std::endl;
//			}
//		}
//	}
	if(debugMode)
		for (uint8_t i = 0; i < PopulationSize; ++i)
			checkVehileConsistencyInAllStructures(solutions[i], assignedRider[i], lookupVehicle[i], CandidateList[i]);
	doSort();
	if(debugMode){
		for (uint8_t i = 0; i < PopulationSize; ++i)
			checkVehileConsistencyInAllStructures(solutions[i], assignedRider[i], lookupVehicle[i], CandidateList[i]);

		solution_show();
	}
	init_cost=solutionsCosts[0];
	bbo_body();
	this->avgCostImprovement+= 1-((double)solutionsCosts[0]/init_cost);
	commit();
	print << "Average improvement: "<<this->avgCostImprovement/this->batchCounter<<std::endl;
	//Last step: commit to database
}
void BBO::handle_vehicle(const Vehicle& vehl) {
	this->grid_.insert(vehl);
	if(clustering_mode)
		driver_pool.push_back(Cargo::node2pt(vehl.route().node_at(vehl.idx_last_visited_node())));

}

void BBO::listen(bool skip_assigned, bool skip_delayed) {
	this->grid_.clear();
	//Clustring part
	driver_pool.clear();

	RSAlgorithm::listen(skip_assigned, skip_delayed);
}

void BBO::end() {

  RSAlgorithm::end();
}
void BBO::reset_workspace() {
	elitesSolutions.clear();
	elitesCosts={};
	elitesAssignedRider.clear();
	elitesUnassignedRider.clear();
	elitesCandidateList.clear();
	elitesLookupVehicle.clear();
//	elites_grid={};
	PopulationSize=maxPopulationSize;



	solutions.clear();
	solutionsCosts={};
	assignedRider.clear();
	unassignedRider.clear();
	CandidateList.clear();
	lookupVehicle.clear();
	local_grid={};

	minimumCostPerGeneration={};
	this->best_cost = InfInt;
	this->sch = best_sch = {};
	this->rte = best_rte = {};
	this->best_vehl = nullptr;
	this->matched = false;

}

