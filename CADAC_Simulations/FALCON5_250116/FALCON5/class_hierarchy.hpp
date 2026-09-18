///////////////////////////////////////////////////////////////////////////////
//FILE: class_hierarchy.hpp
//
//Contains the classes of the hierarchy of base class 'Cadac'
//
//011128 Created by Peter H Zipfel
//081010 Modified for GENSIM6, PZi
//130724 Building AIM5, PZi
//231113 Building FALCON5, PZi 
///////////////////////////////////////////////////////////////////////////////
#define _CRT_SECURE_NO_DEPRECATE
#ifndef cadac_class_hierarchy__HPP
#define cadac_class_hierarchy__HPP

#include "global_header.hpp"

using namespace std;

///////////////////////////////////////////////////////////////////////////////
//Abstract base class: Cadac
//
//011128 Created by Peter H Zipfel
//070411 Included Aircraft class, PZi
//130724 Building AIM5, PZi
//231113 Modifing for FALCON5, PZi
///////////////////////////////////////////////////////////////////////////////
class Cadac
{
private:
	char name[CHARN]={}; //vehicle object name
		
protected:

	//module-variable array of class 'Flat3'
	Variable *flat3=nullptr;

	//Array of module-variables as defined in class 'Plane'
	Variable *plane=nullptr;

	

public:
	//flag indicating an 'event' has occured
	bool event_epoch=false;

	//time elapsed in event 
	double event_time=0;

	virtual~Cadac(){};

	///////////////////////////////////////////////////////////////////////////
	//Constructor of class 'Cadac'
	//
	//010703 Created by Peter H Zipfel
	///////////////////////////////////////////////////////////////////////////
	Cadac(){}

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
	char *get_vname() {return name;}

	//////////////////////////executive functions /////////////////////////////
	virtual void sizing_arrays()=0;
	virtual void vehicle_array()=0;
	virtual void scrn_array()=0;
	virtual void plot_array()=0;
	virtual void scrn_banner()=0;
	virtual void tabout_banner(ofstream &ftabout,char *title)=0;
	virtual void tabout_data(ofstream &ftabout)=0;
	virtual void vehicle_data(fstream &input)=0;
	virtual void read_tables(char *file_name,Datadeck &datatable)=0;
	virtual void scrn_index_arrays()=0;
	virtual void scrn_data()=0;
	virtual void plot_banner(ofstream &fplot,char *title)=0;
	virtual void plot_index_arrays()=0;
	virtual void plot_data(ofstream &fplot,bool merge)=0;
	virtual void event(char *options)=0;
	virtual void document(ostream &fdoc,const char *run_title,Document *doc_vehicle)=0;

	//module functions -MOD
	virtual void def_environment()=0;
	virtual void environment()=0;
	virtual void def_kinematics()=0;
	virtual void init_kinematics(double sim_time)=0;
	virtual void kinematics(double sim_time,double event_time)=0;
	virtual void def_newton()=0;
	virtual void init_newton()=0;
	virtual void newton(double int_step)=0;
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
	virtual void def_intercept()=0;
	virtual void intercept()=0;
};
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
//Derived class: Flat3
//
//First derived class in the 'Cadac' hierarchy
//Models atmosphere, gravitational acceleration and equations of motions
//Contains modules: environment, kinematics and Newton's law
//
//010116 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////
class Flat3:public Cadac
{
protected:			

	//Indicator array pointing to the module-variable which are to 
	//be written to the screen
	int *flat3_scrn_ind=nullptr; int flat3_scrn_count=0;

	//Indicator array pointing to the module-variable which are to 
	//be written to the 'ploti.asc' files
	int *flat3_plot_ind=nullptr; int flat3_plot_count=0;

public:
	Flat3();
	virtual~Flat3(){};

	//executive functions 
	virtual void sizing_arrays()=0;
	virtual void vehicle_array()=0;
	virtual void scrn_array()=0;
	virtual void plot_array()=0;
	virtual void scrn_banner()=0;
	virtual void tabout_banner(ofstream &ftabout,char *title)=0;
	virtual void tabout_data(ofstream &ftabout)=0;
	virtual void vehicle_data(fstream &input)=0;
	virtual void read_tables(char *file_name,Datadeck &datatable)=0;
	virtual void scrn_index_arrays()=0;
	virtual void scrn_data()=0;
	virtual void plot_banner(ofstream &fplot,char *title)=0;
	virtual void plot_index_arrays()=0;
	virtual void plot_data(ofstream &fplot,bool merge)=0;
	virtual void event(char *options)=0;
	virtual void document(ostream &fdoc,const char *title,Document *doc_vehicle)=0;

	//module functions -MOD
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
	virtual void def_intercept()=0;
	virtual void intercept()=0;

	//virtual functions to be declared in this class
	virtual void def_environment();
	virtual void environment();
	virtual void def_kinematics();
	virtual void init_kinematics(double sim_time);
	virtual void kinematics(double sim_time,double event_time);
	virtual void def_newton();
	virtual void init_newton();
	virtual void newton(double int_step);
};
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
//Derived class:Plane
//
//Second level of derived class of the 'Cadac' hierarchy, branching from 'Flat3'
//Models Plane movements
//Contains Module 'forces'
//
//010205 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////
class Plane:public Flat3
{
protected:
	//name of FALCON5 vehicle object
	char plane_name[CHARL]={};

	//Event list of 'Event' object pointers and actual number of events 
	Event *event_ptr_list[NEVENT]={};int nevent=0;

	//total number of envents for a vehicle object
	int event_total=0;

	//Compacted array of all module-variables of vehicle object 'Plane'
	Variable *plane5=nullptr;int nplane5=0;

	//Screen output array of module-variables of vehicle object 'Plane'
	Variable *scrn_plane5=nullptr;int nscrn_plane=0;

	//Plot output array of module-variables of vehicle object 'Plane'
	Variable *plot_plane5=nullptr;int nplot_plane=0;

	//indicator array pointing to the module-variable which are to 
	//be written to the screen
	int *plane_scrn_ind=nullptr; int plane_scrn_count=0;

	//indicator array pointing to the module-variable which are to 
	//be written to the 'ploti.asc' files
	int *plane_plot_ind=nullptr; int plane_plot_count=0;

	//declaring Table pointer as temporary storage of a single table
	Table *table=nullptr;
	//declaring Datadeck 'aerotable' that stores all aero tables
	Datadeck aerotable;
	//declaring Datadeck 'proptable' that stores all aero tables
	Datadeck proptable;

public:
	Plane(){};
	Plane(Module *module_list,int num_modules);
	virtual~Plane();

	//executive functions 
	virtual void sizing_arrays();
	virtual void vehicle_array();
	virtual void scrn_array();
	virtual void plot_array();
	virtual void scrn_banner();
	virtual void tabout_banner(ofstream &ftabout,char *title);
	virtual void tabout_data(ofstream &ftabout);
	virtual void vehicle_data(fstream &input);
	virtual void read_tables(char *file_name,Datadeck &datatable);
	virtual void scrn_index_arrays();
	virtual void scrn_data();
	virtual void plot_banner(ofstream &fplot,char *title);
	virtual void plot_index_arrays();
	virtual void plot_data(ofstream &fplot,bool merge);
	virtual void event(char *options);
	virtual void document(ostream &fdoc,const char *title,Document *doc_vehicle);

	//module functions active
	virtual void def_aerodynamics();
	virtual void aerodynamics(); 
	virtual void def_propulsion();
	virtual void init_propulsion();
	virtual void propulsion(double int_step);
	virtual void def_control();
	virtual void control(double int_step);
	virtual void def_guidance();
	virtual void guidance();
	virtual void def_forces();
	virtual void forces();
	virtual void def_intercept();
	virtual void intercept();

	//functions of control module
	double control_heading(double psivlcx);
	double control_flightpath(double thtvgcx,double phimv);
	double control_bank(double phicx,double int_step);
	double control_load(double ancomx,double int_step);
	double control_lateral(double alcomx);
	double control_altitude(double altcom,double phimvx);
	//functions of guidance module
	Matrix guidance_line();
	Matrix guidance_point();

};

///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
////////////////////////// Global class 'Vehicle'//////////////////////////////
///////////// must be located after 'Cadac' hierarchy in this file (why?)//////
///////////////////////////////////////////////////////////////////////////////
//Class 'Vehicle'
//
//Global class for typifying the array of vehicle pointers
//
//010629 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////
class Vehicle
{
private:
	int capacity=0;	//max number of vehicles permitted in vehicle list
	int howmany=0;	//actual number of vehicles in vehicle list
	//'vehicle_ptr' is the pointer to an array of pointers of type 'Cadac' 
	Cadac **vehicle_ptr=nullptr;
public:
	Vehicle(int number=1);	//constructor, setting capacity, allocating dynamic memory
	virtual ~Vehicle();	//destructor, de-allocating dynamic memory
	void add_vehicle(Cadac &ptr);	//adding vehicle to list
	Cadac *operator[](int position);	//[] operator returns vehicle pointer
	int size();	//returning 'howmany' vehicles are stored in vehicle list
};

#endif