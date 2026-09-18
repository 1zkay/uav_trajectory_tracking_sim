///////////////////////////////////////////////////////////////////////////////
//FILE: 'plane_modules.cpp'
//
//Contains all modules of class 'Plane'
//						aerodynamics()	plane[30-39]
//						propulsion()    plane[10-29]
//						control()       plane[40-79]
//						guidance()		plane[80-99]
//						forces()		flat3[10]
//						intercept()     plane[100-110]
//
//070412 Created by Peter H Zipfel
//130725 Building AIM5, PZi
//231114 Building FALCON5 PZi
///////////////////////////////////////////////////////////////////////////////

#include "class_hierarchy.hpp"

using namespace std;

///////////////////////////////////////////////////////////////////////////////
//Definition of aerodynamic module-variables 
//Member function of class 'Plane'
//Module-variable locations are assigned to plane[30-39]
// 
//Defining and initializing module-variables
//
//001226 Created by Peter H Zipfel
//231114 Building FALCON5 PZi
///////////////////////////////////////////////////////////////////////////////
void Plane::def_aerodynamics()
{
	//Definition of module-variables
	plane[30].init("cl",0,"Lift coefficient - ND","aerodynamics","out","");
	plane[31].init("cd",0,"Drag coefficient - ND","aerodynamics","out","");
	plane[32].init("cl_ov_cd",0,"Lift-over-drag ratio - ND","aerodynamics","diag","scrn,plot");
	plane[33].init("area",27.87,"Aerodynamic reference area - m^2","aerodynamics","data","");
	plane[34].init("mac","int",0,"Mean aerodynamic chord flag - ND","aerodynamics","data","");
	plane[35].init("cla",0,"Lift slope derivative - 1/deg","aerodynamics","out","scrn,plot");
}	
//$$$//////////////////////////////////////////////////////////////////////////
//Aerodynamic module
//Member function of class 'Plane'
// Area = 27.87 m^2 
//
//001023 Created by Peter Zipfel
//001227 Upgraded to module-variable arrays, PZi
//231122 Building FALCON5 PZi
///////////////////////////////////////////////////////////////////////////////
void Plane::aerodynamics()
{
	//local variables
	double cd(0);
	double cl(0);
	double cla(0);
	double cl_ov_cd(0);
	
	//localizing module-variables
	//input data
	int mac=plane[34].integer();
	//input from other modules
	double time=flat3[0].real();
	double mach=flat3[14].real();
	double alphax=plane[51].real();
	if(time>0.5)
		{double start=time; }
	//-------------------------------------------------------------------------
	if(mac==30){
		cl=aerotable.look_up("cl_30MAC_vs_mach_alphax",mach,alphax);
		cd=aerotable.look_up("cd_30MAC_vs_mach_alphax",mach,alphax);
		double clp=aerotable.look_up("cl_30MAC_vs_mach_alphax",mach,alphax+2);
		double cln=aerotable.look_up("cl_30MAC_vs_mach_alphax",mach,alphax-2);
		cla=(clp-cln)/4;
	}else if(mac==35){
		cl=aerotable.look_up("cl_35MAC_vs_mach_alphax",mach,alphax);
		cd=aerotable.look_up("cd_35MAC_vs_mach_alphax",mach,alphax);
		double clp=aerotable.look_up("cl_35MAC_vs_mach_alphax",mach,alphax+2);
		double cln=aerotable.look_up("cl_35MAC_vs_mach_alphax",mach,alphax-2);
		cla=(clp-cln)/4;
	}else if(mac==40){
		cl=aerotable.look_up("cl_40MAC_vs_mach_alphax",mach,alphax);
		cd=aerotable.look_up("cd_40MAC_vs_mach_alphax",mach,alphax);
		double clp=aerotable.look_up("cl_40MAC_vs_mach_alphax",mach,alphax+2);
		double cln=aerotable.look_up("cl_40MAC_vs_mach_alphax",mach,alphax-2);
		cla=(clp-cln)/4;
	}
	cl_ov_cd=cl/cd;
	//-------------------------------------------------------------------------
	//loading module-variables
	//output to other modules
	plane[30].gets(cl);
	plane[31].gets(cd);
	plane[35].gets(cla);
	//diagnostics
	plane[32].gets(cl_ov_cd);
}	
///////////////////////////////////////////////////////////////////////////////
//Definition of propulsion module-variables
//Member function of class 'Plane'
//Module-variable locations are assigned to plane[10-29]
//
// Initializing the module-variables
// Initializing the state variable derivatives to zero 
//
//State derivatives: 
//		treqd = Derivative of thrust req'd - N/s
//		fmassed = Derivative of fuel mass - kg/s
//		
//001103 Created by Peter Zipfel
//231120 Building FALCON5 PZi
///////////////////////////////////////////////////////////////////////////////

void Plane::def_propulsion()
{
	//Definition of module-variables
	plane[10].init("mprop","int",0,"Mode switch - ND","propulsion","data/diag","scrn,plot");
	plane[12].init("fidle",0,"Idle thrust - N","propulsion","diag","scrn,plot");
	plane[13].init("thrust_com",0,"Commanded thrust - N","propulsion","data","");
	plane[14].init("thrust",0,"Turbojet thrust - N","propulsion","out","scrn,plot");
	plane[15].init("treqd",0,"Derivative of thrust req'd - N/s","propulsion","state","");
	plane[16].init("treq",0,"Thrust req'd - N/s","propulsion","state","");
	plane[17].init("fmassed",0,"Derivative of fuel mass expended - kg/s","propulsion","state","");
	plane[18].init("fmasse",0,"Fuel mass expended - kg","propulsion","state","scrn,plot");
	plane[19].init("fuelmass",0,"Fuel mass in vehicle - kg","propulsion","save","");
	plane[20].init("mach_com",0,"Commanded Mach number - ND","propulsion","data","");
	plane[21].init("gfthm",0,"Gain of Mach hold loop - N","propulsion","data","");
	plane[22].init("tfth",0,"Time constant of Mach hold loop - s","propulsion","data","");
	plane[23].init("mass",0,"Vehicle mass - kg","propulsion","out","scrn,plot");
	plane[24].init("tav",0,"Thrust available - N","propulsion","diag","scrn,plot");
	plane[25].init("mass_init",0,"Initial vehicle mass - kg","propulsion","data","");
	plane[26].init("fuel_init",0,"Initial fuel - kg","propulsion","data","");
	plane[27].init("ff",0,"Fuel flow - kg/s","propulsion","dia","scrn");
}

///////////////////////////////////////////////////////////////////////////////
//Initialization of propulsion module
//Member function of class 'Plane'
// Initializing mass
//		
//001103 Created by Peter Zipfel
//231120 Building FALCON5 PZi
///////////////////////////////////////////////////////////////////////////////
void Plane::init_propulsion()
{
	//local module-variable
	double mass(0);

	//localizing module-variables
	//input data
	double mass_init=plane[25].real();
	//-------------------------------------------------------------------------
	//initializing mass
	mass=mass_init;
	//-------------------------------------------------------------------------
	//loading module variables
	plane[23].gets(mass);
}

//$$$//////////////////////////////////////////////////////////////////////////
//Propulsion module
//Member function of class Plane
// Generates thrust of FALCON5 vehicle
//
//High efficiency turbojet
//Gross mass of vehicle (full fuel) 1000 kg
//Max fuel mass 150 kg
//This module performs the following functions:
//
//(1) Provides the thrust available and fuel flow tables
//(2) Provides the idle thrust and fuel flow tables
//(3) Provides the c.g. location as a function of plane missile mass, variable is 'mac'
//
//mprop=0 No thrusting (input)
//		1 Commanded thrust (input), use thrust_com        
//		2 Idle thrusting (input)
//		3 Max thrusting (input)
//		4 Required thrust determined by Mach hold loop (input)
//		5 Required thrust < idle thrust (diagnostic)
//		6 Required thrust > max thrust (diagnostic)
//		
//001023 Created by Peter Zipfel
//231120 Building FALCON5 PZi
//001227 Upgraded to module-variable arrays, PZi
//231120 Building FALCON5 PZi
///////////////////////////////////////////////////////////////////////////////
void Plane::propulsion(double int_step)
{
	//local variables
	double tcom(0);
	double epsmch(0);
	double treqss(0);
	double treqb(0);
	double treqd_new(0);
	double fmassed_new(0);
	double ff(0);
	double treqs(0);

	//local module-variables
	double cg(0);
	double fidle(0);
	double thrust(0);
	double tav(0);

	//localizing module-variables
	//input data
	int mprop=plane[10].integer();
	double thrust_com=plane[13].real();
	double mach_com=plane[20].real();
	double gfthm=plane[21].real();
	double tfth=plane[22].real();
	double mass_init=plane[25].real();
	double fuel_init=plane[26].real();
	//input from other modules
	double pdynmc=flat3[13].real();
	double mach=flat3[14].real();
	double alt=flat3[36].real();
	//state variables
	double treqd=plane[15].real(); 
	double treq=plane[16].real();
	double fmassed=plane[17].real();
	double fmasse=plane[18].real();
	//restore saved values
	double fuelmass=plane[19].real();
	//input from other modules
	double mass=plane[23].real();
	double cd=plane[31].real();
	double area=plane[33].real();
	double alphax=plane[51].real();
	//-------------------------------------------------------------------------	
	//No thrust, just calculate c.g. location
	if(mprop==0)
	{
		thrust=0;
		ff=0;
		return;
	}
	//Look-up idle thrust
	fidle=proptable.look_up("fidle_vs_alt_mach",alt,mach); 
	//Look-up max thrust available
	tav=proptable.look_up("tav_vs_alt_mach",alt,mach); 

	//Branch to the desired thrusting mode
	//commanded thrust
	if(mprop==1)
	{
		thrust=thrust_com;
		//Fuel flow from S.L. to 3048 m
		ff=proptable.look_up("ff_vs_thrust_alt_mach",thrust,alt,mach); 
		//Initialize state variable 'treq' (thrust required)
		treq=thrust_com;
	}
	//idle thrusting
	else if(mprop==2)
	{	thrust=fidle;
		//Idle fuel flow from S.L. to 12000 m
		ff=proptable.look_up("iff_vs_alt",alt);
	}
	//max thrusting
	else if(mprop==3)
	{	thrust=tav;
		//Fuel flow from S.L. to 12000 m
		ff=proptable.look_up("ff_vs_thrust_alt_mach",thrust,alt,mach); 
	}
	//thrust determined by mach-hold loop
	else if(mprop>3)
	{
		//Calculate thrust required in stability axis
		mprop=4;
		treqs=cd*pdynmc*area;
		//Mach hold loop
		epsmch=mach_com-mach;
		tcom=epsmch*gfthm+treqs;
		treqd_new=(tcom-2.*treq)/tfth;
		treq=integrate(treqd_new,treqd,treq,int_step);
		treqd=treqd_new;
		treqss=treq;
		//Thrust in body axis
		treqb=treqss/cos(alphax*RAD);
		//Thrust limiting
		if(treqb<fidle) {mprop=5;treqb=fidle;};
		if(treqb>tav) {mprop=6;treqb=tav;};
		thrust=treqb;
		//Fuel flow from S.L. to 3048 m 
		ff=proptable.look_up("ff_vs_thrust_alt_mach",thrust,alt,mach); 
	}
	//Current vehicle mass obtained through integrating over fuel mass expended 
	fmassed_new=ff;
	fmasse=integrate(fmassed_new,fmassed,fmasse,int_step);
	fmassed=fmassed_new;
	mass=mass_init-fmasse;
	fuelmass=fuel_init-fmasse;

	//setting thrust to zero when fuel expended
	if(fuelmass<=0) thrust=0;
	//-------------------------------------------------------------------------
	//loading module-variables
	//state variables
	plane[15].gets(treqd); 
	plane[16].gets(treq);
	plane[17].gets(fmassed);
	plane[18].gets(fmasse);
	//saving variables
	plane[19].gets(fuelmass);
	//output to other modules
	plane[10].gets(mprop);
	plane[14].gets(thrust);
	plane[23].gets(mass);
	//diagnostics
	plane[12].gets(fidle);
	plane[24].gets(tav);
	plane[27].gets(ff);
}	
//////////////////////////////////////////////////////////////////////////////////
// Control Module
// Definition of control module-variables
// Member function of class 'Plane'
// Module-variable locations are assigned to plane[40-79]
//		
//001228 Created by Peter H Zipfel
//231122 Building FALCON5 PZi
///////////////////////////////////////////////////////////////////////////////
void Plane::def_control()
{
	//definition of module-variables
	plane[40].init("mcontrol","int",0,"Mode switch - ND","control","data","scrn");
	plane[41].init("psivlcx",0,"Commanded heading angle - deg","control","data","plot");
	plane[42].init("thtvgcx",0,"Commanded flight path angle - deg","control","data","plot");
	plane[43].init("alphacx",0,"Commanded angle of attack - deg","control","data","");
	plane[44].init("phimvcx",0,"Commanded bank angle - deg","control","data","");
	plane[45].init("TBV",0,0,0,0,0,0,0,0,0,"TM of body wrt velocity coord. - ND","control","out","");
	plane[47].init("gain_thtvg",0,"Flight path angle hold control gain - g/deg","control","data","");
	plane[48].init("gain_psivg",0,"Heading angle hold control gain - ND","control","data","");
	plane[49].init("anx",0,"Normal load factor - g","control","diag","scrn,plot");
	plane[50].init("avx",0,"Vertical load factor - g","control","diag","scrn,plot");
	plane[51].init("alphax",0,"Angle of attack - deg","control","out","scrn,plot");
	plane[52].init("phimvx",0,"Bank angle - deg","control","out","scrn,plot");
	plane[53].init("phicx",0,"Bank angle command- deg","control","data","scrn,plot");
	plane[54].init("phix",0,"Bank angle state - deg","control","state","plot");
	plane[55].init("phixd",0,"Bank angle state derivative - deg/sec","control","state","");
	plane[56].init("philimx",0,"Bank angle command limiter - deg","control","data","");
	plane[57].init("tphi",0,"Time constant of bank angle response - sec","control","data","");
	plane[58].init("anposlimx",0,"Positive load factor limiter - g's","control","data","");
	plane[59].init("anneglimx",0,"Negative load factor limiter - g's","control","data","");
	plane[60].init("gacp",0,"Root locus gain of accel loop - rad/s^2","control","data","");
	plane[61].init("ta",0,"Ratio of prop/integral gains. If>0, P-I engaged","control","data","");
	plane[62].init("xi",0,"Integral feedback - rad/s","control","state","");
	plane[63].init("xid",0,"Integral feedback derivative - rad/s^2","control","state","");
	plane[64].init("qq",0,"Pitch rate - rad/s","control","diag","plot");
	plane[65].init("tip",0,"Incidence lag time constant - s","control","diag","plot");
	plane[66].init("alp",0,"Angle of attack - rad","control","state","plot");
	plane[67].init("alpd",0,"Angle of attack derivative - rad/s","control","state","");
	plane[68].init("alpposlimx",0,"Angle of attack positive limiter - deg","control","data","");
	plane[69].init("alpneglimx",0,"Angle of attack negative limiter - deg","control","data","");
	plane[70].init("ancomx",0,"Load factor command - g's","control","data","plot");
	plane[71].init("alcomx",0,"Lateral acceleration command - g's","control","data","plot");
	plane[72].init("allimx",0,"Lateral acceleration limiter - g's","control","data","");
	plane[73].init("gcp",0,"Lateral roll gain - rad","control","data","");
	plane[74].init("alx",0,"Lateral acceleration - g's","control","diag","plot");
	plane[75].init("altdlim",0,"Altitude rate limiter - m/s","control","data","");
	plane[76].init("gh",0,"Altitude gain - g/m","control","data","");
	plane[77].init("gv",0,"Altitude rate gain - g/(m/s)","control","data","");
	plane[78].init("altd",0,"Altitude rate  - m/s","control","diag","plot");
	plane[79].init("altcom",0,"Altitude command  - m","control","data","plot");
}
//$$$////////////////////////////////////////////////////////////////////////// 
//Control module
//Member function of class 'Plane' 
//
//mcontrol = 00: No control, phimv=0, alpha=0
//			 01: Flight path angle control, no heading control, thtvgcx (diagnostic)
//			 10: Heading and alpha control, psivlcx, alphacx (diagnostic)
//			*11: Heading and flight path angle control, thtvgcx and psivlcx
//			 03: Bank angle controller, phicx (diagnostics)
//			 04: Accleration control in load-factor plane, ancomx, with mguidance 03  
//			 40: Accleration control in lateral (horizontal) plane, alcomx 
//			 44: Accleration control in lateral and load-factor plane alcomx, ancomx, with mguidance 33, mguidance 46 
//			 06: Altitude hold control with inner acceleration autopilot, altcom (diagnostics)
//			*16: Heading and altitude hold controller, psivlcx, altcom			 			  
//			 46: Lateral acceleration and altitude controller, alcomx, altcom, with mguidance 30, mguidance 40
//			*36: Bank angle control and altitude hold (used with mguidance=70), phicx, altcom (diagnostics)

//001228 Upgraded to module-variable arrays, PZi
//010126 Added acceleration and altitude controllers, PZi
//231122 Building FALCON5 PZi
/////////////////////////////////////////////////////////////////////////////////	
void Plane::control(double int_step)
{
	//local module-variables
	double phimvx(0);
	double alphax(0);
	Matrix TBV(3,3);
	Matrix TVG(3,3);

	//localizing module-variables
	//input data
	int mcontrol=plane[40].integer();
	double thtvgcx=plane[42].real();
	double psivlcx=plane[41].real();
	double alphacx=plane[43].real();
	double ancomx=plane[70].real();
	double alcomx=plane[71].real();
	double altcom=plane[79].real();
	//restore saved values
	double phicx=plane[53].real();
	//input from other modules
	//-------------------------------------------------------------------------
	//No control
	if (mcontrol==0)
	{
		phimvx=0;
		alphax=0;
	}
	//Flight path angle control
	if (mcontrol==1)
	{
		phimvx=0;
		alphax=control_flightpath(thtvgcx,phimvx);
	}
	//Heading and alpha control
	if (mcontrol==10)
	{
		phicx=control_heading(psivlcx);
		phimvx=control_bank(phicx,int_step);
		alphax=alphacx;
	}
	//heading and flight path angle control
	if (mcontrol==11)
	{
		phicx=control_heading(psivlcx);
		phimvx=control_bank(phicx,int_step);
		alphax=control_flightpath(thtvgcx,phimvx);
	}
	//bank angle and alpha control
	if (mcontrol==3)
	{
		phimvx=control_bank(phicx,int_step);
		alphax=alphacx;
	}
	//load factor control
	if (mcontrol==4)
	{
		alphax=control_load(ancomx,int_step);
	}
	//lateral acceleration control
	if (mcontrol==40)
	{
		phicx=control_lateral(alcomx);
		phimvx=control_bank(phicx,int_step);
	}
	//acceleration control in both planes
	if (mcontrol==44)
	{
		phicx=control_lateral(alcomx);
		phimvx=control_bank(phicx,int_step);
		alphax=control_load(ancomx,int_step);
	}

	//altitude control
	if (mcontrol==6)
	{
		ancomx=control_altitude(altcom,phimvx);
		alphax=control_load(ancomx,int_step);
	}
	//heading and altitude
	if (mcontrol==16)
	{
		phicx=control_heading(psivlcx);
		phimvx=control_bank(phicx,int_step);

		ancomx=control_altitude(altcom,phimvx);
		alphax=control_load(ancomx,int_step);
	}
	//lateral acceleration and altitude control
	if (mcontrol==46)
	{
		phicx=control_lateral(alcomx);
		phimvx=control_bank(phicx,int_step);

		ancomx=control_altitude(altcom,phimvx);
		alphax=control_load(ancomx,int_step);
	}
	//lateral bank control and altitude control
	if (mcontrol==36)
	{
		phimvx=control_bank(phicx,int_step);

		ancomx=control_altitude(altcom,phimvx);
		alphax=control_load(ancomx,int_step);
	}
	//calculating TBV ;
	TBV=cadtbv(phimvx*RAD,alphax*RAD);
	//-------------------------------------------------------------------------
	//loading module-variables
	//saving variables
	plane[53].gets(phicx);
	//output to other modules
	plane[45].gets_mat(TBV);
	plane[51].gets(alphax);
	plane[52].gets(phimvx);
	plane[70].gets(ancomx);
}

///////////////////////////////////////////////////////////////////////////////
//Heading angle control
//
//return output: phimvx, bank angle - deg 
//
//000725 Created by Michael Horvath
//001228 Upgraded to module-variable arrays, PZi
///////////////////////////////////////////////////////////////////////////////

double Plane::control_heading(double psivlcx)
{
	//local variables
	double phimvx(0);
	double psivgx_comp(0);
	double sign_psivgx(0);
	
	//localizing module-variables
	//input data
	double gain_psivg=plane[48].real();
	//input from other modules
	double psivlx=flat3[29].real();
	//-------------------------------------------------------------------------
	//elininating the singulatiry at psivlx=+-180 deg by giving special
	//treatment to the heading control of +-45 deg from south
	if(fabs(psivlcx)<=135)
		psivgx_comp=psivlx;
	else
	{
		if(psivlx*psivlcx>=0)
			psivgx_comp=psivlx;
		else
		{
			if(psivlx>=0)sign_psivgx=1;
			else sign_psivgx=-1;
			psivgx_comp=360-psivlx*sign_psivgx;
		}
	}
	phimvx=gain_psivg*(psivlcx-psivgx_comp);
	return phimvx;
}

///////////////////////////////////////////////////////////////////////////////
//Flight path angle control
//
//return output: alphax, angle of attack - deg
//
//000725 Created by Michael Horvath
//001228 Upgraded to module-variable arrays, PZi
///////////////////////////////////////////////////////////////////////////////

double Plane::control_flightpath(double thtvgcx,double phimvx)
{

	//local module-variables
	double anx(0);
	double avx(0);
	double alphax(0);
	
	//localizing module-variables
	//input data
	double gain_thtvg=plane[47].real();
	double alpposlimx=plane[68].real();
	double alpneglimx=plane[69].real();
	//input from other modules
	double pdynmc=flat3[13].real();
	double thtvl=flat3[35].real();
	double grav=flat3[11].real();
	double mass=plane[23].real();
	double area=plane[33].real();
	double cla=plane[35].real();
	//-------------------------------------------------------------------------
	//vertical and normal loadfactors
	avx=gain_thtvg*(thtvgcx*RAD-thtvl);
	anx=((avx)/(cos(phimvx*RAD)));
	alphax=((anx*mass*grav)/(pdynmc*area*cla));

	//limiting angle of attack
	if(alphax>alpposlimx) alphax=alpposlimx;
	if(alphax<alpneglimx) alphax=alpneglimx;

	//loading module-variables
	plane[49].gets(anx);
	plane[50].gets(avx);

	return alphax;
}
///////////////////////////////////////////////////////////////////////////////
//Bank angle controller
//
//return output: phix, bank angle - deg
//
//010126 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

double Plane::control_bank(double phicx,double int_step)
{
	//local variables
	double phixd_new(0);
	
	//localizing module-variables
	//input data
	double philimx=plane[56].real();
	double tphi=plane[57].real();
	//state variables
	double phix=plane[54].real();
	double phixd=plane[55].real();
	//-------------------------------------------------------------------------	
	//limiting bank angle command
	if(phicx>philimx) phicx=philimx;
	if(phicx<-philimx) phicx=-philimx;

	//bank angle lag
	phixd_new=(phicx-phix)/tphi;
	phix=integrate(phixd_new,phixd,phix,int_step);
	phixd=phixd_new;
	//-------------------------------------------------------------------------
	//loading module-variables
	//state variables
	plane[54].gets(phix);
	plane[55].gets(phixd);

	return phix;
}
///////////////////////////////////////////////////////////////////////////////
//Load factor controller
//
//return output: alpx, angle of attack - deg
//
//010129 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

double Plane::control_load(double ancomx,double int_step)
{
	//local variables
	Matrix FSPB(3,1);
	Matrix TBV(3,3);
	double alpha(0);
	double phimv(0);
	double fspb3(0);
	double eanx(0);
	double gr(0);
	double gi(0);
	double xid_new(0);
	double alpd_new(0);
	double alpx(0);

	//local module-variables
	double anx(0);
	double tip(0);
	double qq(0);
	
	//localizing module-variables
	//input data
	double anposlimx=plane[58].real();
	double anneglimx=plane[59].real();
	double phimvx=plane[52].real();
	double gacp=plane[60].real();
	double ta=plane[61].real();
	double alphax=plane[51].real();
	double alpposlimx=plane[68].real();
	double alpneglimx=plane[69].real();
	//input from other modules
	Matrix FSPV=flat3[10].vec();
	double grav=flat3[11].real();
	//state variables
	double xi=plane[62].real();
	double xid=plane[63].real();
	double alp=plane[66].real();
	double alpd=plane[67].real();
	//input from other modules
	double mass=plane[23].real();
	double dvbe=flat3[25].real();
	double pdynmc=flat3[13].real();
	double thrust=plane[14].real();
	double area=plane[33].real();
	double cla=plane[35].real();
	//-------------------------------------------------------------------------
	//tranforming specific force to body coordinates
	alpha=alphax*RAD;
	phimv=phimvx*RAD;
	TBV=cadtbv(phimv,alpha);
	FSPB=TBV*FSPV;

	//limiting load factor command
	if(ancomx>anposlimx) ancomx=anposlimx;
	if(ancomx<anneglimx) ancomx=anneglimx;

	//load factor feedback
	fspb3=FSPB.get_loc(2,0);
	anx=-fspb3/grav;

	//error signal
	eanx=ancomx-anx;

	//incidence lag time constant
	tip=dvbe*mass/(pdynmc*area*cla/RAD+thrust);

	//integral path
	if(ta>0)
	{
		gr=gacp*tip/dvbe;
		gi=gr/ta;
		xid_new=gi*eanx;
		xi=integrate(xid_new,xid,xi,int_step);
		xid=xid_new;
	}else xi=0;

	//proportional path with integral signal(if present)
	qq=gr*eanx+xi;

	//incidence lag dynamics
	alpd_new=qq-alp/tip;
	alp=integrate(alpd_new,alpd,alp,int_step);
	alpd=alpd_new;

	alpx=alp*DEG;

	//limiting angle of attack
	if(alpx>alpposlimx) alpx=alpposlimx;
	if(alpx<alpneglimx) alpx=alpneglimx;
	//-------------------------------------------------------------------------
	//loading module-variables
	//state variables
	plane[62].gets(xi);
	plane[63].gets(xid);
	plane[66].gets(alp);
	plane[67].gets(alpd);
	//diagnostics
	plane[49].gets(anx);
	plane[64].gets(qq);
	plane[65].gets(tip);

	return alpx;
}
///////////////////////////////////////////////////////////////////////////////
//Lateral acceleration controller
//
//return output: phicx, bank command - deg
//
//010130 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

double Plane::control_lateral(double alcomx)
{
	//local variables
	Matrix FSPB(3,1);
	Matrix TBV(3,3);
	double alpha(0);
	double phimv(0);
	double fspb3(0);
	double anx(0);
	double sign(0);
	double phic(0);
	double phicx(0);
	double fspv2(0);

	//local module-variables
	double alx(0);
	
	//localizing module-variables
	//input data
	double allimx=plane[72].real();
	double phimvx=plane[52].real();
	double alphax=plane[51].real();
	double gcp=plane[73].real();
	//input from other modules
	Matrix FSPV=flat3[10].vec();
	double grav=flat3[11].real();
	//-------------------------------------------------------------------------
	//tranforming specific force to body coordinates
	alpha=alphax*RAD;
	phimv=phimvx*RAD;
	TBV=cadtbv(phimv,alpha);
	FSPB=TBV*FSPV;
	//normal load factor
	fspb3=FSPB.get_loc(2,0);
	anx=-fspb3/grav;

	//limiting lateral load factor command
	if(alcomx>allimx) alcomx=allimx;
	if(alcomx<-allimx) alcomx=-allimx;

	//commanded bank angle
	if(anx>=0)sign=1;
	else sign=-1;
	phic=gcp*sign/(fabs(anx)+.001)*alcomx;
	phicx=phic*DEG;

	//diagnostic: lateral acceleration achieved
	fspv2=FSPV.get_loc(1,0);
	alx=fspv2/grav;
	//-------------------------------------------------------------------------
	//loading module-variables
	plane[74].gets(alx);

	return phicx;
}

///////////////////////////////////////////////////////////////////////////////
//Altitude controller
//
//return output: ancomx, load factor command - g's
//
//010131 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

double Plane::control_altitude(double altcom,double phimvx)
{
	//local variables
	double ealt(0);
	double ancomx(0);

	//local module-variables
	double altd(0);
	
	//localizing module-variables
	//input data
	double anposlimx=plane[58].real();
	double anneglimx=plane[59].real();
	double altdlim=plane[75].real();
	double gh=plane[76].real();
	double gv=plane[77].real();
	//input from other modules
	double alt=flat3[36].real();
	double grav=flat3[11].real();
	Matrix VBEL=flat3[27].vec();
	//-------------------------------------------------------------------------
	//altitude error
	ealt=gh*(altcom-alt);

	//limiting altitude rate
	if(ealt>altdlim) ealt=altdlim;
	if(ealt<-altdlim) ealt=-altdlim;

	//altitude rate feedback
	altd=-VBEL.get_loc(2,0);

	//load factor command
	ancomx=(gv*(ealt-altd)/grav+1)*(1/cos(phimvx*RAD));

	//limiting load factor command
	if(ancomx>anposlimx) ancomx=anposlimx;
	if(ancomx<anneglimx) ancomx=anneglimx;
	//-------------------------------------------------------------------------
	//loading module-variables
	//diagnsotics
	plane[78].gets(altd);

	return ancomx;
}

///////////////////////////////////////////////////////////////////////////////
//Definition of guidance module-variables
//Member function of class 'Plane'
//Module-variable locations are assigned to plane[80-99]
//		
//010209 Created by Peter H Zipfel
//231129 Building FALCON5 PZi
///////////////////////////////////////////////////////////////////////////////
void Plane::def_guidance()
{
	//definition of module-variables
	plane[80].init("mguidance","int",0,"Switch for guidance options - ND","guidance","data","scrn");
	plane[82].init("line_gain",0,"Line guidance gain - 1/s","guidance","data","");
	plane[83].init("nl_gain_fact",1,"Nonlinear gain factor - ND","guidance","data","");
	plane[84].init("decrement",0,"distance decrement - m","guidance","data","");
	plane[85].init("swel1",0,"Waypoint north - m","guidance","data","");
	plane[86].init("swel2",0,"Waypoint east - m","guidance","data","");
	plane[87].init("swel3",0,"Waypoint down - m","guidance","data","");
	plane[88].init("psiflx",0,"Heading line-of-attack angle - deg","guidance","data","");
	plane[89].init("thtflx",0,"Pitch line-of-attack angle - deg","guidance","data","");
	plane[90].init("point_gain",0,"Point guidance gain - 1/s","guidance","data","");
	plane[91].init("wp_sltrange",999999,"Range to waypoint - m","guidance","dia","");
	plane[92].init("nl_gain",0,"Nonlinear gain - rad","guidance","dia","");
	plane[93].init("VBEO",0,0,0,"Vehicle velocity in LOS coordinats - m/s","guidance","dia","");
	plane[94].init("VBEF",0,0,0,"Vehicle velocity in LOA coordinats - m/s","guidance","dia","");
	plane[95].init("wp_grdrange",999999,"Ground range to waypoint - m","guidance","dia","scrn,plot");
	plane[96].init("SWBL",0,0,0,"Vehicle wrt waypoint/target in geo coor - m","guidance","out","");
	plane[97].init("rad_min",0,"Minimum arc radius, calculated - m","guidance","dia","");
	plane[98].init("write","int",0,"True flag for writing miss to console - ND","guidance","save","scrn,plot");
	plane[99].init("wp_flag","int",0,"=1:closing on target; =-1:fleeting; =0:outside - ND","guidance","dia","plot,scrn");

}
//$$$//////////////////////////////////////////////////////////////////////////  
//Guidance module
//Member function of class 'Plane'
//
//mguidance:
//			= 30 line-guidance lateral, with mcontrol 46 
//			= 03 line-guidance in pitch 
//			= 33 line-guidance lateral and in pitch, with mcontrol=44
//			= 40 point-guidance lateral, with mcontrol 46
//			= 43 point-guidance lateral, line-guidance in pitch, with mcontrol=44
// 
//010209 Created by Peter H Zipfel
//010820 Added point guidance capability, PZi
//010823 New waypoint arc guidance, PZi
//231129 Building FALCON5 PZi
///////////////////////////////////////////////////////////////////////////////	
void Plane::guidance()
{
	//local variables
	Matrix APNB(3,1);
	Matrix ALGV(3,1);
	Matrix APGV(3,1);

	//local module-variables
	double ancomx(0);
	double alcomx(0);

	//localizing module-variables
	//input data
	int mguidance=plane[80].integer();
	//input from other modules
	double grav=flat3[11].real();
	double phicx=plane[53].real();
	double anposlimx=plane[58].real();
	double anneglimx=plane[59].real();
	double allimx=plane[72].real();
	//-------------------------------------------------------------------------
	//returning if no guidance
	if(mguidance==0)
	{
		alcomx=0;
		ancomx=0;
		return;
	}
	//lateral line guidance
	if(mguidance==30)
	{
		ALGV=guidance_line();
		alcomx=ALGV.get_loc(1,0)/grav;
	}
	//pitch line guidance
	if(mguidance==3)
	{
		ALGV=guidance_line();
		alcomx=0;
		ancomx=-ALGV.get_loc(2,0)/grav;
	}
	//line guidance lateral and pitch
	if(mguidance==33)
	{
		ALGV=guidance_line();
		alcomx=ALGV.get_loc(1,0)/grav;
		ancomx=-ALGV.get_loc(2,0)/grav;
	}
	//point guidance lateral and line guidance pitch
	if(mguidance==43)
	{
		ALGV=guidance_line();
		APGV=guidance_point();
		alcomx=APGV.get_loc(1,0)/grav;
		ancomx=-ALGV.get_loc(2,0)/grav;
	}
	//point guidance lateral
	if(mguidance==40)
	{
		APGV=guidance_point();
		alcomx=APGV.get_loc(1,0)/grav;
	}

	//limiting normal load factor command
	if(ancomx>anposlimx) ancomx=anposlimx;
	if(ancomx<anneglimx) ancomx=anneglimx;

	//limiting lateral load factor command
	if(alcomx>allimx) alcomx=allimx;
	if(alcomx<-allimx) alcomx=-allimx;
	//-------------------------------------------------------------------------
	//loading module-variables
	//output to other modules
	plane[53].gets(phicx);
	plane[70].gets(ancomx);
	plane[71].gets(alcomx);
}
///////////////////////////////////////////////////////////////////////////////
//Guidance to a line
//Against stationary waypoints or targets
//Waypoint are provided as: 'swel1', 'swel2', 'swel3'
//applicable to mguidance:
//			= 30 line-guidance lateral 
//			= 03 line-guidance in pitch 
//			= 33 line-guidance lateral and in pitch
//return output:
//	 ALGV, acceleration demanded by line guidance in velocity coord. - m/s^2
//where:
//		alcomx=ALGV(2)/grav, lateral acceleration command - g's
//		ancomx=-ALGV(3)/grav, normal acceleration command - g's
//
//010405 Created by Peter H Zipfel
//010904 Upgraded to Waypoint guidance, PZi
//231129 Building FALCON5 PZi
////////////////////////////////////////////////////////////////////////////////

Matrix Plane::guidance_line()
{
	//local variables
	Matrix ALGV(3,1);
	Matrix TFL(3,3);
	Matrix SWEL(3,1);
	Matrix TOL(3,3);
	Matrix POLAR(3,1);
	Matrix VH(3,1);
	Matrix SH(3,1);
	double psiol(0);
	double thtol(0);
	double swbl1(0);
	double swbl2(0);
	double vbeo2(0);
	double vbeo3(0);
	double vbef2(0);
	double vbef3(0);
	double algv1(0);
	double algv2(0);
	double algv3(0);
	double vbel1(0);
	double vbel2(0);
	double dvbe(0);
	double rad_min(0);

	//local module-variables
	double wp_sltrange(0);
	double wp_grdrange(0);
	Matrix VBEO(3,1);
	Matrix VBEF(3,1);
	Matrix SWBL(3,1);
	double nl_gain(0);
	int wp_flag(0);

	//localizing module-variables
	//input data
	double line_gain=plane[82].real();
	double nl_gain_fact=plane[83].real();
	double decrement=plane[84].real();
	double swel1=plane[85].real();
	double swel2=plane[86].real();
	double swel3=plane[87].real();
	double psiflx=plane[88].real();
	double thtflx=plane[89].real();
	int write=plane[98].integer();

	//input from other modules
	double time=flat3[0].real(); 
	double grav=flat3[11].real();
	double thtvlx=flat3[30].real();
	Matrix VBEL=flat3[27].vec();
	Matrix SBEL=flat3[26].vec();
	double philimx=plane[56].real();
	//-------------------------------------------------------------------------
	//TM of LOA wrt local axes
	TFL=mat2tr(psiflx*RAD,thtflx*RAD);

	//waypoint W wrt refrence point E from input
	SWEL.build_vec3(swel1,swel2,swel3);

	//waypoint wrt aircraft displacement in local coord 
	SWBL=SWEL-SBEL;

	//building TM of LOS wrt local axes; also getting range-to-go to waypoint
	POLAR=SWBL.pol_from_cart();
	wp_sltrange=POLAR.get_loc(0,0);
	psiol=POLAR.get_loc(1,0);
	thtol=POLAR.get_loc(2,0);
	TOL=mat2tr(psiol,thtol);

	//ground range to waypoint 
	swbl1=SWBL.get_loc(0,0);
	swbl2=SWBL.get_loc(1,0);
	wp_grdrange=sqrt(swbl1*swbl1+swbl2*swbl2);

	//converting local aircraft velocity to LOS and LOA coordinates
	VBEO=TOL*VBEL;
	vbeo2=VBEO.get_loc(1,0);
	vbeo3=VBEO.get_loc(2,0);

	VBEF=TFL*VBEL;
	vbef2=VBEF.get_loc(1,0);
	vbef3=VBEF.get_loc(2,0);

	//nonlinear gain
	nl_gain=nl_gain_fact*(1-exp(-wp_sltrange/decrement));

	//line guidance steering law
	algv1=grav*sin(thtvlx*RAD);
	algv2=line_gain*(-vbeo2+nl_gain*vbef2);
	algv3=line_gain*(-vbeo3+nl_gain*vbef3)-grav*cos(thtvlx*RAD);

	//packing accelerations int vector
	ALGV.assign_loc(0,0,algv1);
	ALGV.assign_loc(1,0,algv2);
	ALGV.assign_loc(2,0,algv3);

	//setting way point flag (if within 2 times turning radius): closing (+1) or fleeting (-1)
	dvbe=VBEL.absolute();
	rad_min=dvbe*dvbe/(grav*tan(philimx*RAD));
	if(wp_grdrange<2*rad_min)
	{
		//projection of displacement vector into horizontal plane, SH
		SH.assign_loc(0,0,swbl1);
		SH.assign_loc(1,0,swbl2);
		SH.assign_loc(2,0,0);
		//projection of velocity vector into horizontal plane, VH
		vbel1=VBEL.get_loc(0,0);
		vbel2=VBEL.get_loc(1,0);
		VH.assign_loc(0,0,vbel1);
		VH.assign_loc(1,0,vbel2);
		VH.assign_loc(2,0,0);
		//setting flag eihter to +1 or -1
		wp_flag=sign(VH^SH);
		if(wp_flag==1)write=1;
	}
	else wp_flag=0;
		
	//-------------------------------------------------------------------------
	//loading module-variables
	//to other modules
	plane[98].gets(write);
	//diagnostics
	plane[91].gets(wp_sltrange);
	plane[92].gets(nl_gain);
	plane[93].gets_vec(VBEO);
	plane[94].gets_vec(VBEF);
	plane[95].gets(wp_grdrange);
	plane[96].gets_vec(SWBL);
	plane[97].gets(rad_min);
	plane[99].gets(wp_flag);

	return ALGV;
}

///////////////////////////////////////////////////////////////////////////////
//Guidance to a point
//special case of line guidance
//Against waypoints
//Waypoint are provided as: 'swel1', 'swel2', 'swel3'
//applicable to mguidance:
//			= 40 point-guidance lateral 
//			= 04 point-guidance in pitch 
//			= 44 point-guidance lateral and in pitch
//return output:
//	 ALGV, acceleration demanded by point guidance in velocity coord. - m/s^2
//where:
//		alcomx=ALGV(2)/grav, lateral acceleration command - g's
//		ancomx=-ALGV(3)/grav, normal acceleration command - g's
//
//010816 Created by Peter H Zipfel
//010830 Upgraded to Waypoint guidance, PZi
//231129 Building FALCON5 PZi
///////////////////////////////////////////////////////////////////////////////

Matrix Plane::guidance_point()
{
	//local variables
	Matrix APGV(3,1);
	Matrix TFL(3,3);
	Matrix SWEL(3,1);
	Matrix TOL(3,3);
	Matrix POLAR(3,1);
	Matrix VH(3,1);
	Matrix SH(3,1);
	double psiol(0);
	double thtol(0);
	double vbel1(0);
	double vbel2(0);
	double dvbe(0);
	double swbl1(0);
	double swbl2(0);
	double vbeo2(0);
	double vbeo3(0);
	double apgv1(0);
	double apgv2(0);
	double apgv3(0);

	//local module-variables
	double wp_sltrange(0);
	double wp_grdrange(0);
	double rad_min(0);
	int wp_flag(0);
	Matrix VBEO(3,1);
	Matrix SWBL(3,1);

	//localizing module-variables
	//input data
	double swel1=plane[85].real();
	double swel2=plane[86].real();
	double swel3=plane[87].real();
	double point_gain=plane[90].real();
	int write=plane[98].integer();

	//input from other modules
	double time=flat3[0].real(); 
	double grav=flat3[11].real();
	double thtvlx=flat3[30].real();
	Matrix VBEL=flat3[27].vec();
	Matrix SBEL=flat3[26].vec();
	double philimx=plane[56].real();
	//-------------------------------------------------------------------------

	//waypoint W wrt refrence point E from input
	SWEL.build_vec3(swel1,swel2,swel3);

	//waypoint wrt aircraft displacement in local coord 
	SWBL=SWEL-SBEL;

	//building TM of LOS wrt local axes; also getting range-to-go to waypoint
	POLAR=SWBL.pol_from_cart();
	wp_sltrange=POLAR.get_loc(0,0);
	psiol=POLAR.get_loc(1,0);
	thtol=POLAR.get_loc(2,0);
	TOL=mat2tr(psiol,thtol);

	//ground range to waypoint 
	swbl1=SWBL.get_loc(0,0);
	swbl2=SWBL.get_loc(1,0);
	wp_grdrange=sqrt(swbl1*swbl1+swbl2*swbl2);

	//converting local aircraft velocity to LOS and LOA coordinates
	VBEO=TOL*VBEL;
	vbeo2=VBEO.get_loc(1,0);
	vbeo3=VBEO.get_loc(2,0);

	//point guidance steering law
	apgv1=grav*sin(thtvlx*RAD);
	apgv2=point_gain*(-vbeo2);
	apgv3=point_gain*(-vbeo3)-grav*cos(thtvlx*RAD);

	//packing accelerations int vector
	APGV.assign_loc(0,0,apgv1);
	APGV.assign_loc(1,0,apgv2);
	APGV.assign_loc(2,0,apgv3);

	//setting way point flag (if within 2 times turning radius): closing (+1) or fleeting (-1)
	dvbe=VBEL.absolute();
	rad_min=dvbe*dvbe/(grav*tan(philimx*RAD));
	if(wp_grdrange<2*rad_min)
	{
		//projection of displacement vector into horizontal plane, SH
		SH.assign_loc(0,0,swbl1);
		SH.assign_loc(1,0,swbl2);
		SH.assign_loc(2,0,0);
		//projection of velocity vector into horizontal plane, VH
		vbel1=VBEL.get_loc(0,0);
		vbel2=VBEL.get_loc(1,0);
		VH.assign_loc(0,0,vbel1);
		VH.assign_loc(1,0,vbel2);
		VH.assign_loc(2,0,0);
		//setting flag eihter to +1 or -1
		wp_flag=sign(VH^SH);
		if(wp_flag==1)write=1;
	}
	else wp_flag=0;
		
	//-------------------------------------------------------------------------
	//loading module-variables
	//to other modules
	plane[98].gets(write);
	//diagnostics
	plane[91].gets(wp_sltrange);
	plane[93].gets_vec(VBEO);
	plane[95].gets(wp_grdrange);
	plane[96].gets_vec(SWBL);
	plane[97].gets(rad_min);
	plane[99].gets(wp_flag);

	return APGV;
}

///////////////////////////////////////////////////////////////////
//Definition of force module-variables
// Member function of class 'Plane'
//
//Note that FSPV is entered into the flat3[10] array because it is needed
// for the newton module, which is a member of the 'Flat3' class
//		
//001129 Created by Peter Zipfel
///////////////////////////////////////////////////////////////////////////////
void Plane::def_forces()
{
	//Definition of module-variables
	flat3[10].init("FSPV",0,0,0,"Specific force in V-coord - m/s^2","forces","out","plot");
}
//$$$//////////////////////////////////////////////////////////////////////////
//Force Module 
//Member function of class 'Plane' 
//Calulates the total force acting on the vehicle
//
//000623 Created by Michael Chiaramonte
//000724 Function calls have been removed, Michael Horvath
//001227 Upgraded to module-variable arrays, PZi
//231122 Building FALCON5 PZi
///////////////////////////////////////////////////////////////////////////////
void Plane::forces()
{
	//localizing module-variables
	//input from other modules
	double pdynmc=flat3[13].real();
	Matrix FSPV=flat3[10].vec();
	double cl=plane[30].real();
	double cd=plane[31].real();
	double area=plane[33].real();
	double thrust=plane[14].real();
	double mass=plane[23].real();
	double alphax=plane[51].real();
	double phimvx=plane[52].real();
	//-------------------------------------------------------------------------
	double phimv=phimvx*RAD;
	double alpha=alphax*RAD;

	double fspv1=(-pdynmc*area*cd+thrust*cos(alpha))/mass; 
	double fspv2=sin(phimv)*(pdynmc*area*cl+thrust*sin(alpha))/mass;
	double fspv3=-cos(phimv)*(pdynmc*area*cl+thrust*sin(alpha))/mass;
	
	FSPV.assign_loc(0,0,fspv1);
	FSPV.assign_loc(1,0,fspv2);
	FSPV.assign_loc(2,0,fspv3);
	//-------------------------------------------------------------------------
	//loading module-variables
	//output to other modules
	flat3[10].gets_vec(FSPV);
}

///////////////////////////////////////////////////////////////////////////////
//Definition of intercept module-variables
//Member function of class 'Plane'
//Module-variable locations are assigned to plane[100-110]
//		
//010328 Created by Peter H Zipfel
//231129 Building FALCON5 PZi
///////////////////////////////////////////////////////////////////////////////
void Plane::def_intercept()
{
	//definition of module-variables 
	plane[100].init("stop_run","int",0,"if 1 stops run - ND","intercept","data","");
}
//$$$//////////////////////////////////////////////////////////////////////////
//Intercept module
//Member function of class 'Plane'
//Determining closest approach of aircraft to way points
//
//010328 Created by Peter H Zipfel
//231129 Building FALCON5 PZi
///////////////////////////////////////////////////////////////////////////////
void Plane::intercept()
{
	//local variables
	double swbl1(0);
	double swbl2(0);
	double swbl3(0);
	double dwbh(0);

	//input from this module
	int stop_run=plane[100].integer();
 
	//input from other modules
	double time=flat3[0].real();
	double psivlx=flat3[29].real();
	double thtvlx=flat3[30].real();
	int mguidance=plane[80].integer();
	Matrix SWBL=plane[96].vec();
	int wp_flag=plane[99].integer();
	int write=plane[98].integer();

	//-------------------------------------------------------------------------
	//Waypoint horizontal miss distance 
	if(write)
	{
		if(mguidance==30||mguidance==40)
		{ 
		//waypoint has been crossed
		if(wp_flag==-1)
			{
				swbl1=SWBL.get_loc(0,0);
				swbl2=SWBL.get_loc(1,0);
				dwbh=sqrt(swbl1*swbl1+swbl2*swbl2);

				//writing miss information to console
				cout<<"\n"<<" *** Aircraft overflies waypoint at time = "<<time<<" sec *** \n";
				cout<<" miss north = "<<swbl1<<" m, miss east = "<<swbl2<<" m, heading = "<<psivlx<<" deg from north \n";
				cout<<" horizontal miss distance  = "<<dwbh<<" m\n";
				write=0;
				if(stop_run==1) exit(1);
			}
		}
	//Terminal line guidance in pitch and yaw
	if(mguidance==33)
		{ 
		if(wp_flag==-1)
			{
				swbl1=SWBL.get_loc(0,0);
				swbl2=SWBL.get_loc(1,0);
				swbl3=SWBL.get_loc(2,0);

				dwbh=sqrt(swbl1*swbl1+swbl2*swbl2+swbl3*swbl3);
			
			//Line guidance: simple miss distance calculation at last integration, no interpolation
				//writing miss information to console
				cout<<"\n"<<" *** Aircraft intercepts PI at time = "<<time<<" sec *** \n";
				cout<<" miss north = "<<swbl1<<" m, miss east = "<<swbl2<<" m, miss down = "<<swbl2<<" m \n";
				cout<<" heading = "<<psivlx<<" deg from north, flight path angle ="<<thtvlx<<" deg (neg down) \n";
				cout<<" absolute miss distance = "<<dwbh<<" m\n";
				write=0;
				if(stop_run==1) exit(1);
			}
		}		
	}
	//-------------------------------------------------------------------------
	// save value
	plane[98].gets(write);
}

