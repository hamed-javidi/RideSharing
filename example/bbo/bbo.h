#include "libcargo.h"
#include <memory>
#include <set>
#include <unordered_map>
#include <utility>
#include <random>
#include <numeric>
#include <algorithm>

using namespace cargo;





struct Driver{
			int capacity;		// capacity = real capacity - already fixed rides
			vec_t<int> rider;  //temporary riders
		};

class BBO : public RSAlgorithm {
	public:
		BBO(const std::string &);

		/* My overrides */
		virtual void match();
		virtual void handle_vehicle(const Vehicle &);
		virtual void listen(bool skip_assigned = true, bool skip_delayed = true);
		virtual void end();

	private:
		bool debugMode;
		vec_t<int> sortedIdx;
		std::random_device rd;
		Grid grid_;
		vec_t<Grid> local_grid;
//		vec_t<Grid> elites_grid;
		std::mt19937 gen;
		//BBO variables
		int NumberOfElites;
		int PopulationSize;
		const float MutationProbability= 0.04;
		const int GenerationLimit=10;
		const int maxNumberOfElites=2;
		const int maxPopulationSize=20;
		int problemDimension;
		vec_t <float> mu;
		vec_t <float> lambda;

		vec_t<dict<Customer, MutableVehicleSptr>> assignedRider ;
		vec_t<dict<Customer, MutableVehicleSptr>> elitesAssignedRider;

		vec_t<dict<Customer, vec_t<MutableVehicleSptr>>> CandidateList;
		vec_t<dict<Customer, vec_t<MutableVehicleSptr>>> elitesCandidateList;
		vec_t<vec_t<Customer>> unassignedRider;
		vec_t<vec_t<Customer>> elitesUnassignedRider;
		vec_t<dict<MutableVehicleSptr, vec_t<Customer>>> solutions;
		vec_t<dict<MutableVehicleSptr, vec_t<Customer>>> elitesSolutions;
		vec_t<dict<VehlId, MutableVehicleSptr>> lookupVehicle;
		vec_t<dict<VehlId, MutableVehicleSptr>> elitesLookupVehicle;
		vec_t<long> solutionsCosts;
		vec_t<long> elitesCosts;
		vec_t<long> minimumCostPerGeneration;


		//dict<CustId, Customer> cust_lookup;



		DistInt get_cost();
		bool migrate_vehicle_based(int popIndxSrc, int popIndxDest, const MutableVehicleSptr &, vec_t<dict<MutableVehicleSptr, vec_t<Customer>>> &, vec_t<dict<Customer, MutableVehicleSptr>> & tempAssignedRider, vec_t<vec_t<Customer>> &tempUnassignedRider);
		void bbo_init();
		void insertElits();
		void bbo_body();
		void bbo_mutation();
		bool checkSCH(int solutionIdx, MutableVehicleSptr const & r);
		bool checkSCHTemp(int solutionIdx, MutableVehicleSptr const & r, vec_t<dict<Customer, MutableVehicleSptr>> const &tempAssignedRider, vec_t<dict<MutableVehicleSptr, vec_t<Customer>>> const  &tempSolutions);
		void solutionsCostUpdate();
		void solution_show();
		bool Greedy_Assignment(int,const Customer &, vec_t<dict<MutableVehicleSptr, vec_t<Customer>>> & tempSolutions, vec_t<dict<Customer, MutableVehicleSptr>> &tempAssignedRider, vec_t<vec_t<Customer>> &tempUnassignedRider);
		void ridersRandomlySelector(int indx, MutableVehicleSptr vehl, vec_t<Customer> & riders);
		void randXofY(int ridersCount, vec_t<Customer> & riders);
		void doSort();
		void commit();
		bool checkVehlStopsWithRiders(const vec_t<dict<MutableVehicleSptr, vec_t<Customer>>> & sol);
		bool checkVehlStopsDuplication(const MutableVehicleSptr &vehl, const Customer & cust);
		void selectElites();
		bool checkVehileConsistencyInAllStructures(	const dict<MutableVehicleSptr, vec_t<Customer>> & solutions,
															const dict<Customer, MutableVehicleSptr> & assignedRider,
															const dict<VehlId,MutableVehicleSptr> & lookupVehl,
															const dict<Customer, vec_t<MutableVehicleSptr>> & CandidateList	);
		//vec_t<Customer>::iterator searchRider(Customer const &a, vec_t<Customer> const &b);
		/* Workspace variables */
		DistInt best_cost;
		vec_t<Stop> sch, best_sch;
		vec_t<Wayp> rte, best_rte;
		MutableVehicleSptr best_vehl;

		bool matched;
		tick_t timeout_0;

		long init_cost;

		void reset_workspace();
		float rand(){
			return rd()/float(rd.max());
		}
};


