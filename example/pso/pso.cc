#include <iostream>
#include <queue>
#include <tuple>
#include <vector>
#include "pso.h"
#include "libcargo.h"

using namespace cargo;

PSO::PSO(const std::string& name) : RSAlgorithm(name, false), grid_(10) {
  this->batch_time() = 30;

}



void PSO::handle_vehicle(const Vehicle& vehl) {
  this->grid_.insert(vehl);

}

void PSO::listen(bool skip_assigned, bool skip_delayed) {
  this->grid_.clear();
  RSAlgorithm::listen(skip_assigned, skip_delayed);
}



void PSO::match() {
	for (const Customer& cust : this->customers()) {
		//vec_t<MutableVehicleSptr> candidates = this->candidates_list[cust.id()] = local_grid.within(pickup_range(cust), cust.orig());

	}

    // Add to local vehicle lookup (we need it during pso_body)
    for (const MutableVehicleSptr cand : candidates)
      this->vehicle_lookup[cand->id()] = cand;

}

void PSO::reset_workspace() {
  this->best_cost = InfInt;
  this->sch = best_sch = {};
  this->rte = best_rte = {};
  this->best_vehl = nullptr;
  this->candidates = {};
  this->matched = false;
}

