///////////////////////////////////////////////////////////////////////////////
//FILE: 'class_hierarchy.hpp'
//
//Contains the classes of the hierarchy of base class 'Cadac'
//
//030627 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

#define _CRT_SECURE_NO_DEPRECATE
#ifndef cadac_class_hierarchy__HPP
#define cadac_class_hierarchy__HPP

#include "global_header.hpp"

///////////////////////////////////////////////////////////////////////////////
//Abstract base class: Cadac
//
//011128 Created by Peter H Zipfel
//030627 Adapted to PLANE6 simulation, PZi
///////////////////////////////////////////////////////////////////////////////
class Cadac
{
private:
	char name[CHARN]=""; //vehicle object name
		
protected:

	//module-variable array of class 'Flat6'
	//first 10 locations reserved for time and 'com' variables
	Variable *flat6=nullptr;

	//array of module-variables as defined in class 'Plane'
	//first ten locations are reserved for 'com' variables
	Variable *plane=nullptr;

public:
	//flag indicating an 'event' has occured
	bool event_epoch=false;

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
	virtual void document(ostream &fdoc,const char *title,Document *doc_plane6)=0;
	virtual void com_index_arrays()=0;
	virtual Packet loading_packet_init(int num_plane)=0;
	virtual Packet loading_packet(int num_plane)=0;

	//module functions -MOD
	virtual void def_environment()=0;
	virtual void environment(double int_step)=0;
	virtual void def_kinematics()=0;
	virtual void init_kinematics()=0;
	virtual void kinematics(double int_step)=0;
	virtual void def_newton()=0;
	virtual void init_newton()=0;
	virtual void newton(double sim_time,double int_step)=0;
	virtual void def_euler()=0;
	virtual void init_euler()=0;
	virtual void euler(double int_step)=0;
	virtual void def_aerodynamics()=0;
	virtual void init_aerodynamics()=0;
	virtual void aerodynamics()=0; 
	virtual void def_propulsion()=0;
	virtual void propulsion(double int_step)=0;
	virtual void def_forces()=0;
	virtual void forces()=0;
	virtual void def_actuator()=0;
	virtual void actuator(double int_step)=0;
	virtual void def_control()=0;
	virtual void control(double int_step)=0;
	virtual void def_guidance()=0;
	virtual void guidance()=0;
};

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
//Derived class: Flat6
//
//First derived class in the 'Cadac' hierarchy
//Models atmosphere, gravitaional acceleration and equations of motions
//Contains modules: environment and Newton's law
//
//011128 Created by Peter H Zipfel
//030627 Adapted to PLANE6 simulation, PZi
///////////////////////////////////////////////////////////////////////////////
class Flat6:public Cadac
{
protected:			
	//Indicator array pointing to the module-variable which are to 
	//be written to the screen
	int *flat6_scrn_ind=nullptr; int flat6_scrn_count=0;

	//Indicator array pointing to the module-variable which are to 
	//be written to the 'ploti.asc' files
	int *flat6_plot_ind=nullptr; int flat6_plot_count=0;

	//Indicator array pointing to the module-variable which are to 
	//be written to 'combus' 'packets'
	int *flat6_com_ind=nullptr; int flat6_com_count=0;
public:
	Flat6();
	virtual~Flat6(){};

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
	virtual void document(ostream &fdoc,const char *title,Document *doc_plane6)=0;
	virtual void com_index_arrays()=0;
	virtual Packet loading_packet_init(int num_plane)=0;
	virtual Packet loading_packet(int num_plane)=0;

	//module functions -MOD
	virtual void def_aerodynamics()=0;
	virtual void init_aerodynamics()=0;
	virtual void aerodynamics()=0; 
	virtual void def_propulsion()=0;
	virtual void propulsion(double int_step)=0;
	virtual void def_forces()=0;
	virtual void forces()=0;
	virtual void def_actuator()=0;
	virtual void actuator(double int_step)=0;
	virtual void def_control()=0;
	virtual void control(double int_step)=0;
	virtual void def_guidance()=0;
	virtual void guidance()=0;

	//virtual functions to be declared in this class
	virtual void def_kinematics();
	virtual void init_kinematics();
	virtual void kinematics(double int_step);
	virtual void def_euler();
	virtual void init_euler();
	virtual void euler(double int_step);
	virtual void def_environment();
	virtual void environment(double int_step);
	virtual void def_newton();
	virtual void init_newton();
	virtual void newton(double sim_time,double int_step);

};

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
//Derived class: Plane
//
//Second level of derived class of the 'Cadac' hierarchy
//Models aerodynamics, propulsion, guidance and control 
//Contains Modules: aerodynamics, propulsion, forces, control, guidance 
//
//011128 Created by Peter H Zipfel
//030627 Adapted to PLANE6 simulation, PZi
///////////////////////////////////////////////////////////////////////////////

class Plane:public Flat6
{
protected:
	//name of PLANE6 vehicle object
	char plane6_name[CHARL]="";

	//event list of 'Event' object pointers and actual number of events 
	Event *event_ptr_list[NEVENT];int nevent=0;
	//total number of envents for a vehicle object
	int event_total=0;

	//compacted array of all module-variables of vehicle object 'Plane'
	Variable *plane6=nullptr;int nplane6=0;

	//screen ouput array of module-variables of vehicle object 'Plane'
	Variable *scrn_plane6=nullptr;int nscrn_plane6=0;

	//plot ouput array of module-variables of vehicle object 'Plane'
	Variable *plot_plane6=nullptr;int nplot_plane6=0;

	//communications ouput array of module-variables of vehicle object 'Plane'
	Variable *com_plane6=nullptr;int ncom_plane6=0;

	//packet of data for each plane vehicle
	Packet packet;

	//indicator array pointing to the module-variable which are to 
	//be written to the screen
	int *plane_scrn_ind=nullptr; int plane_scrn_count=0;

	//indicator array pointing to the module-variable which are to 
	//be written to the 'ploti.asc' files
	int *plane_plot_ind=nullptr; int plane_plot_count=0;

	//indicator array pointing to the module-variable which are to 
	//be written to 'combus' 'packets'
	int *plane_com_ind=nullptr; int plane_com_count=0;

	//declaring Table pointer as temporary storage of a single table
	Table *table=nullptr;
	//	declaring Datadeck 'aerotable' that stores all aero tables
	Datadeck aerotable;
	//	declaring Datadeck 'proptable' that stores all aero tables
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
	virtual void document(ostream &fdoc,const char *title,Document *doc_plane6);
	virtual void com_index_arrays();
	virtual Packet loading_packet_init(int num_plane);
	virtual Packet loading_packet(int num_plane);

	//module functions -MOD
	virtual void def_aerodynamics();
	virtual void init_aerodynamics();
	virtual void aerodynamics(); 
	virtual void def_propulsion();
	virtual void propulsion(double int_step);
	virtual void def_forces();
	virtual void forces();
	virtual void def_actuator();
	virtual void actuator(double int_step);
	virtual void def_control();
	virtual void control(double int_step);
	virtual void def_guidance();
	virtual void guidance();

	//functions in respective modules
	void aerodynamics_der();
	double propulsion_thrust(double throttle,double int_step);
	Matrix actuator_scnd(Matrix ACTCX, double int_step);
	double control_roll(double phicomx);
	double control_roll_rate(double pcomx);
	double control_yaw_rate(double rcomx);
	double control_pitch_rate(double qcomx);
	double control_lateral_accel(double alcomx);
	double control_normal_accel(double ancomx,double int_step);
	double control_gamma(double thtvlcomx);
	double control_heading(double psivlcomx);
	double control_altitude(double altcom);
	void control_roll();
	void control_rate();
	void control_accel(double int_step);
	Matrix guidance_line(Matrix SWBL,double psiflx,double thtflx);
  };

///////////////////////////////////////////////////////////////////////////////
////////////////////////// Global class 'Vehicle'//////////////////////////////
///////////// must be located after 'Cadac' hierarchy in this file (why?)//////
///////////////////////////////////////////////////////////////////////////////
//Class 'Vehicle'
//
// Global class for typifying the array of vehicle pointers
//
//010629 Created by Peter Zipfel
//030627 Adapted to PLANE6 simulation, PZi
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