#include <iostream>
#include <queue>
#include <tuple>
#include <vector>
#include <fstream>
#include <chrono>
#include <thread>
#include <algorithm>    // std::sort
#include <string>
#include <iterator>
#include <memory> /* shared_ptr */
#include <mutex> /* lock_guard */
#include <utility>


#include "tshare.h"
#include "libcargo.h"

#include "libcargo/cargo.h" /* Cargo::gtree() */
#include "libcargo/classes.h"
#include "libcargo/debug.h"
#include "libcargo/distance.h"
#include "libcargo/functions.h"
#include "libcargo/types.h"

#include "gtree/gtree.h"

using namespace cargo;

//!!!!!grid_ and grid_sise must be the same!!!

auto cmp = [](MutableVehicleSptr  left,  MutableVehicleSptr  right) {
  return left->queued() > right->queued();
};



Tshare::Tshare(const std::string& name) : RSAlgorithm(name, false), grid_size(10), grid_(10) {
	sort_now=false;
	//calculate distances between each two grid cells
	double x=grid_.x_dim_/2;
	double y=grid_.y_dim_/2;
	error_=0;

	//calculating center point of each cell
	for (int i = 0; i < grid_size; ++i) {
		//distance_matrix.push_back ( std::vector<int>() );
		for (int j = 0; j < grid_size; ++j) {
			Point z;
			z.lng = Cargo::bbox().lower_left.lng + x + (i*grid_.x_dim_);
			z.lat = Cargo::bbox().lower_left.lat + y + (j*grid_.y_dim_);
			center[i][j]= z;
			printf("%lf,%lf\n",center[i][j].lng,center[i][j].lat);
			//find the nearest node to center point of each cell thru all nodes
			double temp,min=999999.0;
			NodeId min_nodeid;
			for (auto n : Cargo::nodes_) {
				//std::cout<<"n"<<n.second.lat<<"   ,"<<n.second.lng<<"  ,"<<center[i][j].lat-n.second.lat<<std::endl;
				temp=sqrt(pow(center[i][j].lat - n.second.lat,2.0) + pow(center[i][j].lng - n.second.lng,2.0));
				//std::cout<<temp<<"   ,";

				if (temp<min){
					min=temp;
					min_nodeid=n.first;
				}
			}
			centernode[i][j]=min_nodeid;
			//distance_matrix[i].push_back (min_nodeid);
		}
	}


	for (int i = 0; i < grid_size; ++i) {
		for (int j = 0; j < grid_size; ++j) {
			distance_matrix.push_back ( std::vector<int>() );
			for (int m = 0; m < grid_size; ++m) {
				for (int n = 0; n < grid_size; ++n) {
					distance_matrix[i*grid_size+j].push_back (get_shortest_path(centernode[i][j],centernode[m][n]));
				}
			}
		}
	}

	for (int i = 0; i < grid_size; ++i) {
		gridcell.push_back(std::vector<cell>());
		for (int j = 0; j < grid_size; ++j) {
			cell c;
			c.id=i*grid_size+j;
			for (int m = 0; m < grid_size; ++m) {
				for (int n = 0; n <  grid_size; ++n) {
					int index= m* grid_size + n;
					if ( index != c.id){
						c.spatial.insert(std::pair<int, int>(index , distance_matrix[c.id][index]));
						c.temporal.insert(std::pair<int, int>(index , (int(distance_matrix[c.id][index] / Cargo::vspeed()))));

					}
				}
			}
			//need to be sorted
			//sort(c.spatial.begin(), c.spatial.end());
			//sort(c.temporal.begin(), c.temporal.end());
			gridcell[i].push_back(c);
		}
	}
	this->batch_time() = 1;



}

void Tshare::handle_customer(const Customer& cust) {
	this->reset_workspace();
	//this->candidates = this->grid_.within(pickup_range(cust), cust.orig());
	this->candidates=dualside_matching(cust);
//	if (candidates.empty())
//		std::cout<<std::endl<<Cargo::now()<<'\t'<<"id="<<cust.id()<<'\t'<<"cell="<<node2gridcell(cust.orig())<<'\t'<<"Candidate is empty!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<std::endl;
	//std::priority_queue<MutableVehicleSptr, vec_t< MutableVehicleSptr>, decltype(cmp)>my_q(cmp);
	DistInt cost_min = InfInt;
	for (const auto& cand : this->candidates) {

		DistInt cost_old = cand->route().cost();
		DistInt cost_new = sop_insert(cand, cust, sch, rte);
		DistInt cost = cost_new - cost_old;

		if (cost < cost_min && insertion_check(cust, cand) && chkcap(cand->capacity(), sch)) {
			cost_min = cost;
			this->best_vehl = cand;
			this->best_sch = sch;
			this->best_rte = rte;
			this->matched = true;
		}
		if (this->timeout(this->timeout_0))
			break;
	}
	if (this->matched) {

		print << "Matched " << cust.id() << " with " << this->best_vehl->id() << std::endl;
		this->assign({cust.id()}, {}, this->best_rte, this->best_sch, *(this->best_vehl));
	}

}

void Tshare::handle_vehicle(const Vehicle& vehl) {

	//to call fun1 to convert taxi to common vehicle for M5Knon
//	long dist= (Cargo::gtree().search(vehl.orig(),vehl.dest()))/ Cargo::vspeed();
//	vlate.push_back(dist*2);
//	if(vehl.id()>200 && vehl.id()<=3800){
//		if (vehl.id()>vearly.size() && vehl.id()%3==0)
//			vearly.push_back(vearly.back() +1);
//		else
//			vearly.push_back(vearly.back());
//	}
//	else if(vehl.id()>3800 && vehl.id()<=5000)
//		if (vehl.id()>vearly.size() && vehl.id()%2==0)
//			vearly.push_back(vearly.back() +1);
//		else
//			vearly.push_back(vearly.back());
//	else if(vehl.id()<=200 && vearly.size()<=200)
//		vearly.push_back(0);


	insert_gridcell_taxies(vehl);
	sort_now=true;


	if(vehl.id()==5000)
		//fun1();
		for (int i = 0; i < 10; ++i) {
			std::cout<<i<<": ";
				for (int j = 0; j < 10; ++j) {
					std::cout<<gridcell[i][j].taxies.size()<<", ";
				}
				std::cout<<std::endl;
			}

	//Show table of vehicles origin
	/*
	if (vehl.id()==1000){
		std::ofstream out("table.txt");
		for (int i = 0; i < 10; i++) {
			for (int j = 0; j < 10; j++) {
				out <<gridcell[i][j].taxies.size() << "    " ;
				std::cout << std::setw(4) << std::right << gridcell[i][j].taxies.size();
			}
			out<< '\n' << std::endl;
		}
	}
*/
}
//void Tshare::fun1(){
//	TripId tid;
//	OrigId oid;
//	DestId did;
//	Load q;
//	ErlyTime early;
//	LateTime late;
//	std::string _;
//
//	std::ifstream ifs("/home/hamed/Desktop/Cargo-master/Cargo_benchmark/problem/rs-bj5non-m5k-c3-d6-s10-x1.0.instance");
//	std::ofstream if2("/home/hamed/Desktop/Cargo-master/Cargo_benchmark/problem/rs-bj5non-m5k-c3-d6-s10-x1.0-updated.instance");
//	ifs >> _ >> _ >> _ >> _ >> _ >> _ >> _;
//	ifs >> _;
//	std::getline(ifs, _);
//	while (ifs >> tid >> oid >> did >> q >> early >> late) {
//	  if( q == -3)
//		  if2 << tid << '\t' << oid << '\t' << did << '\t' << q << '\t' << early << '\t' << vlate[tid-1] + early << '\n';
//	  else
//		  if2 << tid << '\t' << oid << '\t' << did << '\t' << q << '\t' << early << '\t' << late << '\n';
//	}
//	ifs.close();
//	if2.close();
//}

void Tshare::insert_gridcell_taxies(const Vehicle& vehl){
	//insert current location in current cell
	int cur_cell=node2gridcell(vehl.last_visited_node());
	int x=cur_cell/grid_size;
	int y= cur_cell%grid_size;

	gridcell[x][y].taxies.push_back(std::make_pair(0,std::make_shared<MutableVehicle>(vehl)));


	/*//for debugging:
	std::ofstream ofs;
	ofs.open ("/home/hamed/Desktop/Cargo-master/Cargo_benchmark/handle_vehicle.txt", std::ofstream::out | std::ofstream::app);
	ofs<< vehl.id()<<'\t'<<Cargo::now()<<'\t'<<gridnum <<'\t'<<vehl.last_visited_node()<<'\n';
	ofs.close();*/


	int t;
	for (auto i = vehl.idx_last_visited_node(); i < vehl.route().data().size()-1; ++i) {
		int cellnum=node2gridcell(vehl.route().data()[i].second);
		x=cellnum/grid_size;
		y=cellnum%grid_size;
		if(cur_cell==cellnum){
			t=0;
		}
		else{
			t=gridcell[cur_cell/grid_size][cur_cell%grid_size].temporal.at(cellnum);
		}
		bool found=false;
		for(auto i :gridcell[x][y].taxies){
			if(i.second->id()==vehl.id()){
				found=true;
				break;
			}
		}
		if(!found)
			gridcell[x][y].taxies.push_back(std::make_pair(t,std::make_shared<MutableVehicle>(vehl)));
	}
}


void Tshare::clear_gridcell_taxies(){
	std::cout<<std::endl<<"CLEAR TAXIES **** Time: "<<Cargo::now()<<std::endl;
	int i,j;
	for (i = 0; i < grid_size; ++i) {
		for (j = 0; j < grid_size; ++j) {
			gridcell[i][j].taxies.clear();
		}
	}
}

void Tshare::listen(bool skip_assigned, bool skip_delayed) {
  //this->grid_.clear();
  clear_gridcell_taxies();

  RSAlgorithm::listen(skip_assigned, skip_delayed);
}

void Tshare::reset_workspace() {
	if(sort_now){
		for (int i = 0; i < grid_size; ++i) {
			for (int j = 0; j < grid_size; ++j) {
				std::sort(gridcell[i][j].taxies.begin(), gridcell[i][j].taxies.end(), sortfunction);
			}
		}
		sort_now=false;
	}
	 //clear_gridcell_taxies();
	this->best_cost = InfInt;
	this->sch = best_sch = {};
	this->rte = best_rte = {};
	this->best_vehl = nullptr;
	this->candidates = {};
	this->matched = false;
}

vec_t<MutableVehicleSptr> Tshare::dualside_matching(const Customer& cust){
	int go=node2gridcell(cust.orig());
	int gd=node2gridcell(cust.dest());
	vec_t<MutableVehicleSptr> So=Check_pickup_time_window(go, gd , cust.late());
	vec_t<MutableVehicleSptr> Sd= Check_dropoff_time_window(gd , cust.late());
	vec_t<MutableVehicleSptr> S= set_intersection_MutableVehicleSptr(So,Sd);

	if(!S.empty())
		return S;
	//line 7 of paper's algorithm
	vec_t <int> lo,ld;
	int x=go/grid_size;
	int y= go%grid_size;
	cell cc=gridcell[x][y];

	for(auto const& gi : cc.temporal)
		if(((int)Cargo::now() + gridcell[gi.first/grid_size][gi.first%grid_size].temporal[go]) <= cust.late() - gridcell[x][y].temporal[gd])
			lo.push_back(gi.first);
	//line 11
	x=gd/grid_size;
	y= gd%grid_size;
	cc=gridcell[x][y];
	for(auto const& gj : cc.temporal)
			if(((int)(Cargo::now()) + gridcell[gj.first/grid_size][gj.first%grid_size].temporal[gd]) <= cust.late())
				ld.push_back(gj.first);

	bool stopo=false,stopd=false;
	int indexo=0,indexd=0;

	while (S.empty() && (!stopo or !stopd)){

		if (lo.size()>indexo)
		{
			vec_t<MutableVehicleSptr> temp=Check_pickup_time_window(lo[indexo], ld[indexd],cust.late()-Cargo::now()- gridcell[lo[indexo]/grid_size][lo[indexo]%grid_size].temporal[go]);
			So.insert( So.end(), temp.begin(), temp.end());
			indexo++;
		}
		else
			stopo=true;

		if (ld.size()>indexd)
		{
			vec_t<MutableVehicleSptr> temp=Check_dropoff_time_window(ld[indexd],cust.late()-Cargo::now()-gridcell[ld[indexd]/grid_size][ld[indexd]%grid_size].temporal[gd]);
			Sd.insert( Sd.end(), temp.begin(), temp.end());
			indexd++;
			//std::cout<<indexd<<std::endl;
		}
		else
			stopd=true;


	}
	S=set_intersection_MutableVehicleSptr(So,Sd);

	if(!S.empty())
			return S;
	if(!So.empty()){
		for(auto i: So)
			if (i->queued()==0){
				S.push_back(So[0]);
			}
	}

	return S;
}

long Tshare::lazyshortestpath(const NodeId orig, const NodeId dest){
	//1- check the cache to find the past calculation
	if (Cargo::scexist(orig,dest))
		return Cargo::scget(orig, dest)/Cargo::vspeed();

	//2- triangle equation
	const NodeId origcell=node2gridcell(orig);
	const NodeId destcell=node2gridcell(dest);
	return (distance_matrix[origcell][destcell] - get_shortest_path(orig,origcell) - get_shortest_path(dest,destcell))/Cargo::vspeed();

}

int Tshare::node2gridcell(const NodeId node){
	Point pt=Cargo::node2pt(node);
	double temp,min=999999.0;
	int gridnum;
	for (int i = 0; i < grid_size; ++i) {
		for (int j = 0; j < grid_size; ++j) {
			temp=sqrt(pow(center[i][j].lat - pt.lat,2.0) + pow(center[i][j].lng - pt.lng,2.0));
			if (temp<min){
				min=temp;
				gridnum=gridcell[i][j].id;
			}
		}
	}
	return gridnum;
}


vec_t<MutableVehicleSptr>  Tshare::Check_pickup_time_window(int orig_cell_id, int dest_cell_id, const LateTime cust_latetime){
	//Taxies will be in customer's origin cell before the late pick up time
	int xo= orig_cell_id/grid_size;
	int yo= orig_cell_id%grid_size;
	vec_t<MutableVehicleSptr> out;
	for(auto i : Tshare::gridcell[xo][yo].taxies){
		int late_pickup_time=cust_latetime-gridcell[xo][yo].temporal[dest_cell_id];
		if(i.first+Cargo::now() <= late_pickup_time)
			out.push_back(i.second);
	}
	return out;
}
vec_t<MutableVehicleSptr>  Tshare::Check_dropoff_time_window(int cellid, const LateTime cust_latetime){
	//Taxies will be in customer's destination cell before the late drop off time
	int x=cellid/grid_size;
	int y= cellid%grid_size;
	vec_t<MutableVehicleSptr> out;
	for(auto i : Tshare::gridcell[x][y].taxies)
		if(i.first+Cargo::now()<cust_latetime)
			out.push_back(i.second);
	return out;
}
vec_t<MutableVehicleSptr> set_intersection_MutableVehicleSptr(vec_t<MutableVehicleSptr> a, vec_t<MutableVehicleSptr> b){
	vec_t<MutableVehicleSptr> out;
	for(auto i: a)
		for(auto j: b)
			if(i==j)
				out.push_back(j);
	return out;
}

//No need to use insertion chech because sop_insert do the same procedure

bool Tshare::insertion_check(const Customer& cust, const MutableVehicleSptr& V){


	//long travel_time = Cargo::gtree().search(cust.orig(), cust.dest())/Cargo::vspeed();
	long travel_time=gridcell[node2gridcell(cust.orig())/grid_size][node2gridcell(cust.orig())%grid_size].temporal[node2gridcell(cust.dest())];
	long cust_p_late=cust.late()-travel_time;
//	if (cust_p_late<0){
//		std::cout<<std::endl<<"Error"<<++error_<<" cust.late"<<cust.late()<<" travel_time:"<<travel_time<<" orig_cell:"<< node2gridcell(cust.orig())<<" dest_cell:"<<node2gridcell(cust.dest())<<" GRIDCELL_temporal:"<< gridcell[node2gridcell(cust.orig())/grid_size][node2gridcell(cust.orig())%grid_size].temporal[node2gridcell(cust.dest())]<<std::endl;
//		//std::cout<<"cust_p_late"<<cust_p_late<<std::endl;
//		return false;
//	}
	long x=Cargo::now() + lazyshortestpath(V->last_visited_node(),cust.orig())/Cargo::vspeed();
//	if ( x> cust_p_late){
//		std::cout<<cust.id()<<'\tcell='<<node2gridcell(cust.orig())<<'\t'<<"Could not be picked up at "<<cust_p_late<<" < "<< x <<"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<std::endl;
//		return false;
//	}

	if(!chktw(sch, rte)){
		//std::cout<<cust.id()<<'\tcell='<<node2gridcell(cust.orig())<<'\t'<<"CHKTW FALSE!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<std::endl;
		return false;

	}
	return true;
}
bool sortfunction (std::pair<int , MutableVehicleSptr> i,std::pair<int , MutableVehicleSptr> j) {
	return (i.first < j.first);
}




