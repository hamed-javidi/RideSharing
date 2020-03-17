#include <iostream>
#include <queue>
#include <tuple>
#include <vector>

#include "greedy.h"
#include "libcargo.h"

using namespace cargo;

Greedy::Greedy(const std::string& name) : RSAlgorithm(name, false), grid_(10) {
  this->batch_time() = 30;

}

void Greedy::handle_customer(const Customer& cust) {
	//Hamed begin// to save riders locations
	/*
	int x_orig=grid_.hash_x(Cargo::node2pt(cust.orig()));
	int y_orig=grid_.hash_y(Cargo::node2pt(cust.orig()));
	int x_dest=grid_.hash_x(Cargo::node2pt(cust.dest()));
	int y_dest=grid_.hash_y(Cargo::node2pt(cust.dest()));
	this->customer_orig_cell_count[x_orig][y_orig]++;
	this->customer_dest_cell_count[x_dest][y_dest]++;
	*/
	this->reset_workspace();
	this->candidates = this->grid_.within(pickup_range(cust), cust.orig());

	for (const MutableVehicleSptr& cand : this->candidates) {
		// Speed heuristic: try only if vehicle's current schedule has < 8 customer stops
		if (cand->schedule().data().size() < 10) {
			DistInt cost = sop_insert(*cand, cust, sch, rte, Cargo::gtree()) - cand->route().cost();
			if (cost < this->best_cost) {
				if (chkcap(cand->capacity(), sch) && chktw(sch, rte)) {
					this->best_vehl = cand;
					this->best_sch = sch;
					this->best_rte = rte;
					this->best_cost = cost;
				}
			}
		}
		if (this->timeout(this->timeout_0))
		  break;
	}
  if (this->best_vehl != nullptr)
    this->matched = true;

  if (this->matched) {
    print << "Matched " << cust.id() << " with " << this->best_vehl->id() << std::endl;
    this->assign(
      {cust.id()}, {}, this->best_rte, this->best_sch, *(this->best_vehl));
  }


}

void Greedy::handle_vehicle(const Vehicle& vehl) {
  this->grid_.insert(vehl);
  //Hamed begin// to save riders locations
  	/*
  int x_orig=grid_.hash_x(Cargo::node2pt(vehl.orig()));
  int y_orig=grid_.hash_y(Cargo::node2pt(vehl.orig()));
  int x_dest=grid_.hash_x(Cargo::node2pt(vehl.dest()));
  int y_dest=grid_.hash_y(Cargo::node2pt(vehl.dest()));
  this->driver_orig_cell_count[x_orig][y_orig]++;
  this->driver_dest_cell_count[x_dest][y_dest]++;
	*/
}

void Greedy::listen(bool skip_assigned, bool skip_delayed) {
  this->grid_.clear();
  RSAlgorithm::listen(skip_assigned, skip_delayed);
}

void Greedy::reset_workspace() {
  this->best_cost = InfInt;
  this->sch = best_sch = {};
  this->rte = best_rte = {};
  this->best_vehl = nullptr;
  this->candidates = {};
  this->matched = false;
}

