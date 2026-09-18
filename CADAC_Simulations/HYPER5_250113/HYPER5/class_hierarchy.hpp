///////////////////////////////////////////////////////////////////////////////
//FILE: 'class_hierarchy.hpp'
//Declares the derived classes 'Round3', 'Hyper', 'Target', and 'Satellite'
// of the base class 'Cadac'
// and the global class 'Vehicles'
//
//030627 Created by Peter H Zipfel
//060511 Updated to latest CADAC++ standards, PZi
//060522 Inclusion of 'Satellite' object, PZi
//240121 Updated for HYPER5, PZi
///////////////////////////////////////////////////////////////////////////////

#ifndef cadac_class_hierarchy__HPP
#define cadac_class_hierarchy__HPP

#include "global_header.hpp"

using namespace std;

///////////////////////////////////////////////////////////////////////////////
//Abstract base class: Cadac
//
//011128 Created by Peter H Zipfel
//030627 Adapted to CRUISE simulation, PZi
//240121 Updated for HYPER5, PZi
///////////////////////////////////////////////////////////////////////////////
class Cadac
{
private:
	char name[CHARN]={}; //vehicle object name
		
protected:

	//module-variable array of class 'Round3'
	//first 10 locations are reserved for executive variables
	Variable *round3=nullptr;

	//array of module-variables as defined in class 'Hyper'
	//first 10 locations are reserved for executive variables
	Variable *hyper=nullptr;

	//Array of module-variables as defined in class 'Target'
	//first 10 locations are reserved for executive variables
	Variable *target=nullptr;

	//Array of module-variables as defined in class 'Satellite'
	//first  10 locations are  reserved for executive variables
	Variable *satellite=nullptr;


public:
	//flag indicating an 'event' has occured
	bool event_epoch=false;

	//time elapsed in event 
	double event_time=0; //event_time

	///////////////////////////////////////////////////////////////////////////
	//Constructor of class 'Cadac'
	//
	//010703 Created by Peter H Zipfel
	///////////////////////////////////////////////////////////////////////////
	Cadac(){};

	///////////////////////////////////////////////////////////////////////////
	//Destructor of class 'Cadac'
	//
	//010703 Created by Peter H Zipfel
	///////////////////////////////////////////////////////////////////////////
	virtual~Cadac(){};

	///////////////////////////////////////////////////////////////////////////
	//Setting vehicle object name
	//
	//010703 Created by Peter H Zipfel
	///////////////////////////////////////////////////////////////////////////
	void set_name(const char *vehicle_name) {strcpy(name,vehicle_name);}

	///////////////////////////////////////////////////////////////////////////
	//Getting vehicle object name
	//
	//010703 Created by Peter H Zipfel
	///////////////////////////////////////////////////////////////////////////
	const char *get_vname() {return name;}

	//////////////////////////executive functions /////////////////////////////
	virtual void sizing_arrays()=0;
	virtual void vehicle_array()=0;
	virtual void scrn_array()=0;
	virtual void plot_array()=0;
	virtual void scrn_banner()=0;
	virtual void tabout_banner(ofstream &ftabout,const char *title)=0;
	virtual void tabout_data(ofstream &ftabout)=0;
	virtual void vehicle_data(fstream &input)=0;
	virtual void read_tables(const char *file_name,Datadeck &datatable)=0;
	virtual void scrn_index_arrays()=0;
	virtual void scrn_data()=0;
	virtual void plot_banner(ofstream &fplot,const char *title)=0;
	virtual void plot_index_arrays()=0;
	virtual void plot_data(ofstream &fplot,bool merge)=0;
	virtual void event(const char *options)=0;
	virtual void document(ostream &fdoc,const char *title,Document *doc_hyper5)=0;
	virtual void com_index_arrays()=0;
	virtual Packet loading_packet_init(int num_hyper,int num_target,int num_satellite)=0;
	virtual Packet loading_packet(int num_hyper,int num_target,int num_satellite)=0;

	//module functions -MOD
	virtual void def_environment()=0;
	virtual void init_environment(double sim_time,double int_step)=0;
	virtual void environment(double sim_time,double event_time,double &int_step,double &out_fact)=0;
	virtual void def_aerodynamics()=0;
	virtual void aerodynamics()=0; 
	virtual void def_propulsion()=0;
	virtual void init_propulsion()=0;
	virtual void propulsion(double int_step)=0;
	virtual void def_forces()=0;
	virtual void forces()=0;
	virtual void def_newton()=0;
	virtual void init_newton()=0;
	virtual void newton(double int_step)=0;
	virtual void def_control()=0;
	virtual void control(double int_step)=0;
	virtual void def_guidance()=0;
	virtual void guidance()=0;
	virtual void def_targeting()=0;
	virtual void targeting(Packet *combus,int vehicle_slot,int num_vehicles
		,int num_target,int num_satellite)=0;
	virtual void def_seeker()=0;
	virtual void seeker(Packet *combus,int vehicle_slot,int num_vehicles,int num_target)=0;
	virtual void def_intercept()=0;
	virtual void intercept(Packet *combus,int vehicle_slot,double int_step,const char *title)=0;
};

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
//Derived class: Round3
//
//First derived class in the 'Cadac' hierarchy
//Models atmosphere, gravitational acceleration and equations of motions
//Contains modules: environment and Newton's law
//
//011128 Created by Peter H Zipfel
//030627 Adapted to CRUISE simulation, PZi
///////////////////////////////////////////////////////////////////////////////
class Round3:public Cadac
{
protected:			
	//Indicator array pointing to the module-variable which are to 
	//be written to the screen
	int *round3_scrn_ind=nullptr; int round3_scrn_count=0;

	//Indicator array pointing to the module-variable which are to 
	//be written to the 'ploti.asc' files
	int *round3_plot_ind=nullptr; int round3_plot_count=0;

	//Indicator array pointing to the module-variable which are to 
	//be written to 'combus' 'packets'
	int *round3_com_ind=nullptr; int round3_com_count=0;
public:
	Round3();
	virtual~Round3(){};

	//executive functions
	virtual void sizing_arrays()=0;
	virtual void vehicle_array()=0;
	virtual void scrn_array()=0;
	virtual void plot_array()=0;
	virtual void scrn_banner()=0;
	virtual void tabout_banner(ofstream &ftabout,const char *title)=0;
	virtual void tabout_data(ofstream &ftabout)=0;
	virtual void vehicle_data(fstream &input)=0;
	virtual void read_tables(const char *file_name,Datadeck &datatable)=0;
	virtual void scrn_index_arrays()=0;
	virtual void scrn_data()=0;
	virtual void plot_banner(ofstream &fplot,const char *title)=0;
	virtual void plot_index_arrays()=0;
	virtual void plot_data(ofstream &fplot,bool merge)=0;
	virtual void event(const char *options)=0;
	virtual void document(ostream &fdoc,const char *title,Document *doc_hyper5)=0;
	virtual void com_index_arrays()=0;
	virtual Packet loading_packet_init(int num_hyper,int num_target,int num_satellite)=0;
	virtual Packet loading_packet(int num_hyper,int num_target,int num_satellite)=0;

	//module functions
	virtual void def_aerodynamics()=0;
	virtual void aerodynamics()=0; 
	virtual void def_propulsion()=0;
	virtual void init_propulsion()=0;
	virtual void propulsion(double int_step)=0;
	virtual void def_forces()=0;
	virtual void forces()=0;
	virtual void def_control()=0;
	virtual void control(double int_step)=0;
	virtual void def_guidance()=0;
	virtual void guidance()=0;
	virtual void def_targeting()=0;
	virtual void targeting(Packet *combus,int vehicle_slot,int num_vehicles
		,int num_target,int num_satellite)=0;
	virtual void def_seeker()=0;
	virtual void seeker(Packet *combus,int vehicle_slot,int num_vehicles,int num_target)=0;
	virtual void def_intercept()=0;
	virtual void intercept(Packet *combus,int vehicle_slot,double int_step,const char *title)=0;

	//virtual functions to be defined in this class
	virtual void def_environment();
	virtual void init_environment(double sim_time,double int_step);
	virtual void environment(double sim_time,double event_time,double &int_step,double &out_fact);
	virtual void def_newton();
	virtual void init_newton();
	virtual void newton(double int_step);
};

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
//Derived class: Hyper
//
//Second level of derived class of the 'Cadac' hierarchy
//Models aerodynamics, propulsion, guidance and control 
//Contains Modules: aerodynamics, propulsion, forces, control, guidance 
//
//011128 Created by Peter H Zipfel
//030627 Adapted to CRUISE simulation, PZi
///////////////////////////////////////////////////////////////////////////////

class Hyper:public Round3
{
protected:
	//name of HYPER5 vehicle object
	char hyper5_name[CHARL]={};

	//event list of 'Event' object pointers and actual number of events 
	Event *event_ptr_list[NEVENT]={}; int nevent=0;
	//total number of envents for a vehicle object
	int event_total=0;

	//compacted array of all module-variables of vehicle object 'Hyper'
	Variable *hyper5=nullptr; int nhyper5=0;

	//screen output array of module-variables of vehicle object 'Hyper'
	Variable *scrn_hyper5=nullptr; int nscrn_hyper5=0;

	//plot output array of module-variables of vehicle object 'Hyper'
	Variable *plot_hyper5=nullptr; int nplot_hyper5=0;

	//communications output array of module-variables of vehicle object 'Hyper'
	Variable *com_hyper5=nullptr; int ncom_hyper5=0;

	//packet of data for each hyper vehicle
	Packet packet;

	//indicator array pointing to the module-variable which are to 
	// be written to the screen
	int *hyper_scrn_ind=nullptr; int hyper_scrn_count=0;

	//indicator array pointing to the module-variable which are to 
	// be written to the 'ploti.asc' files
	int *hyper_plot_ind=nullptr; int hyper_plot_count=0;

	//indicator array pointing to the module-variable which are to 
	// be written to 'combus' 'packets'
	int *hyper_com_ind=nullptr; int hyper_com_count=0;

	//array of ground distances of 'Missile' object from all 'Target' objects
	double *grnd_range=nullptr;

	//array of all satellites with indicator whether visible form 'this' missile
	// and first target to assure that satellite can provide targeting data to missile 
	Targeting *visibility=nullptr;

	//declaring Table pointer as temporary storage of a single table
	Table *table=nullptr;
	//	declaring Datadeck 'aerotable' that stores all aerodynamic tables
	Datadeck aerotable;
	//	declaring Datadeck 'proptable' that stores all propulsion tables
	Datadeck proptable;

public:
	Hyper(){};
	Hyper(Module *module_list,int num_modules,int num_target,int num_satellite);
	virtual~Hyper();

	//executive functions
	virtual void sizing_arrays();
	virtual void vehicle_array();
	virtual void scrn_array();
	virtual void plot_array();
	virtual void scrn_banner();
	virtual void tabout_banner(ofstream &ftabout,const char *title);
	virtual void tabout_data(ofstream &ftabout);
	virtual void vehicle_data(fstream &input);
	virtual void read_tables(const char *file_name,Datadeck &datatable);
	virtual void scrn_index_arrays();
	virtual void scrn_data();
	virtual void plot_banner(ofstream &fplot,const char *title);
	virtual void plot_index_arrays();
	virtual void plot_data(ofstream &fplot,bool merge);
	virtual void event(const char *options);
	virtual void document(ostream &fdoc,const char *title,Document *doc_hyper5);
	virtual void com_index_arrays();
	virtual Packet loading_packet_init(int num_hyper,int num_target,int num_satellite);
	virtual Packet loading_packet(int num_hyper,int num_target,int num_satellite);

	//module functions
	virtual void def_aerodynamics();
	virtual void aerodynamics(); 
	virtual void def_propulsion();
	virtual void init_propulsion();
	virtual void propulsion(double int_step);
	virtual void def_forces();
	virtual void forces();
	virtual void def_control();
	virtual void control(double int_step);
	virtual void def_guidance();
	virtual void guidance();
	virtual void def_targeting();
	virtual void targeting(Packet *combus,int vehicle_slot,int num_vehicles
		,int num_target,int num_satellite);
	virtual void def_seeker();
	virtual void seeker(Packet *combus,int vehicle_slot,int num_vehicles,int num_target);
	virtual void def_intercept();
	virtual void intercept(Packet *combus,int vehicle_slot,double int_step,const char *title);

	//functions of control module
	double control_heading(double psivgcx);
	double control_flightpath(double thtvgcx,double phimv);
	double control_bank(double phicx,double int_step);
	double control_load(double ancomx,double int_step);
	double control_lateral(double alcomx);
	double control_altitude(double altcom,double phimvx);

	//functions of guidance module
	Matrix guidance_line();
	Matrix guidance_pronav();
	Matrix guidance_point();
	double guidance_arc();

	//functions of seeker module 
	void seeker_grnd_ranges(Packet *combus,int num_vehicles);

	//functions of targeting module
	void targeting_satellite(Packet *combus,int num_vehicles);
	void targeting_grnd_ranges(Packet *combus,int num_vehicles);
  };
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
//Derived class: Target
//
//Second level of derived class of the 'Cadac' hierarchy, branching from 'Round3'
//Models target accelerations
//Contains Module 'forces' and 'intercept'
//
//010205 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

class Target:public Round3
{
protected:
	//name of TARGET3 vehicle object
	char target3_name[CHARL]={};

	//Communications output array of module-variables of vehicle object 'Target'
	Variable *com_target3=nullptr;int ncom_target3=0;

	//Packet of data for each target vehicle
	Packet packet;

	//Indicator array pointing to the module-variable which are to 
	//be written to 'combus' 'packets'
	int *target_com_ind=nullptr; int target_com_count=0;
public:
	Target();
	Target(Module *module_list,int num_modules);
	virtual~Target();

	//executive functions dummy returns
	virtual void vehicle_array(){};
	virtual void scrn_array(){};
	virtual void plot_array(){};
	virtual void scrn_banner(){};
	virtual void tabout_banner(ofstream &ftabout,const char *title){};
	virtual void tabout_data(ofstream &ftabout){};
	virtual void scrn_index_arrays(){};
	virtual void scrn_data(){};
	virtual void plot_banner(ofstream &fplot,const char *title){};
	virtual void plot_index_arrays(){};
	virtual void plot_data(ofstream &fplot,bool merge){};
	virtual void event(const char *options){};
	virtual void doc_input(fstream &input){};

	//executive functions active
	virtual void sizing_arrays();
	virtual void vehicle_data(fstream &input);
	virtual void read_tables(const char *file_name,Datadeck &datatable){};
	virtual void com_index_arrays();
	virtual void document(ostream &fdoc,const char *title,Document *doc_vehicle);
	virtual Packet loading_packet_init(int num_hyper,int num_target,int num_satellite);
	virtual Packet loading_packet(int num_hyper,int num_target,int num_satellite);

	//module function dummy returns
	virtual void def_aerodynamics(){};
	virtual void aerodynamics(){}; 
	virtual void def_propulsion(){};
	virtual void init_propulsion(){};
	virtual void propulsion(double int_step){};
	virtual void def_control(){};
	virtual void control(double int_step){};
	virtual void def_guidance(){};
	virtual void guidance(){};
	virtual void def_targeting(){};
	virtual void targeting(Packet *combus,int vehicle_slot,int num_vehicles
		,int num_target,int num_satellite){};
	virtual void def_seeker(){};
	virtual void seeker(Packet *combus,int vehicle_slot,int num_vehicles,int num_target){};

	//module functions active
	virtual void def_forces();
	virtual void forces();
	virtual void def_intercept();
	virtual void intercept(Packet *combus,int vehicle_slot,double int_step,const char *title);
};
///////////////////////////////////////////////////////////////////////////////
//Derived class: Satellite
//
//Second level of derived class of the 'Cadac' hierarchy, branching from 'Round3'
//Contains Module 'forces' and 'seeker'
//
//010810 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

class Satellite:public Round3
{
protected:
	//name of SATELLITE3 vehicle object
	char satellite3_name[CHARL]={};

	//Communications output array of module-variables of vehicle object 'Satellite'
	Variable *com_satellite3=nullptr;int ncom_satellite3=0;

	//Packet of data for each satellite vehicle
	Packet packet;

	//Indicator array pointing to the module-variable which are to 
	//be written to 'combus' 'packets'
	int *satellite_com_ind=nullptr; int satellite_com_count=0;

public:
	Satellite();
	Satellite(Module *module_list,int num_modules);
	virtual~Satellite();

	//executive functions dummy returns
	virtual void vehicle_array(){};
	virtual void scrn_array(){};
	virtual void plot_array(){};
	virtual void scrn_banner(){};
	virtual void tabout_banner(ofstream &ftabout,const char *title){};
	virtual void tabout_data(ofstream &ftabout){};
	virtual void scrn_index_arrays(){};
	virtual void scrn_data(){};
	virtual void plot_banner(ofstream &fplot,const char *title){};
	virtual void plot_index_arrays(){};
	virtual void plot_data(ofstream &fplot,bool merge){};
	virtual void event(const char *options){};
	virtual void doc_input(fstream &input){};

	//executive functions active
	virtual void sizing_arrays();
	virtual void vehicle_data(fstream &input);
	virtual void read_tables(const char *file_name,Datadeck &datatable){};
	virtual void com_index_arrays(); 
	virtual void document(ostream &fdoc,const char *title,Document *doc_vehicle);
	virtual Packet loading_packet_init(int num_hyper,int num_target,int num_satellite); 
	virtual Packet loading_packet(int num_hyper,int num_target,int num_satellite); 

	//module function dummy returns
	virtual void def_aerodynamics(){};
	virtual void aerodynamics(){}; 
	virtual void def_propulsion(){};
	virtual void init_propulsion(){};
	virtual void propulsion(double int_step){};
	virtual void def_control(){};
	virtual void control(double int_step){};
	virtual void def_guidance(){};
	virtual void guidance(){};
	virtual void def_targeting(){};
	virtual void targeting(Packet *combus,int vehicle_slot,int num_vehicles
		,int num_target,int num_satellite){};
	virtual void def_intercept(){};
	virtual void intercept(Packet *combus,int vehicle_slot,double int_step,const char *title){};
	virtual void def_seeker(){};
	virtual void seeker(Packet *combus,int vehicle_slot,int num_vehicles,int num_target){};

	//module functions active
	virtual void def_forces();
	virtual void forces();
};

///////////////////////////////////////////////////////////////////////////////
////////////////////////// Global class 'Vehicle'//////////////////////////////
///////////// must be located after 'Cadac' hierarchy in this file ////////////
///////////////////////////////////////////////////////////////////////////////
//Class 'Vehicle'
//
//Global class for typifying the array of vehicle pointers
//
//010629 Created by Peter Zipfel
///////////////////////////////////////////////////////////////////////////////
class Vehicle
{
private:
	int capacity=0;	//max number of vehicles permitted in vehicle list
	int howmany=0;	//actual number of vehicles in vehicle list 
	Cadac **vehicle_ptr=nullptr; //'vehicle_ptr' is the pointer to an array of pointers of type 'Cadac'
public:
	Vehicle(int number=1);	//constructor, setting capacity, allocating dynamic memory
	virtual ~Vehicle();	//destructor, de-allocating dynamic memory
	void add_vehicle(Cadac &ptr);	//adding vehicle to list
	Cadac *operator[](int position);	//[] operator returns vehicle pointer
	int size();	//returning 'howmany' vehicles are stored in vehicle list
};

#endif