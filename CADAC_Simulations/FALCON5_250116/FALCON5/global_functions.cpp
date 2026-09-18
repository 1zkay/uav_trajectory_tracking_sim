///////////////////////////////////////////////////////////////////////////////
//FILE: 'global_functions.cpp'
//
//Contains the global functions for the PLANE simulation.
//
//011129 Created by Peter H Zipfel
//030110 Included aircraft object, PZi
//030319 Upgraded to SM Item32, PZi
//130724 Building AIM5, PZi
//231113 Building FALCON5, PZi 
///////////////////////////////////////////////////////////////////////////////

#include "class_hierarchy.hpp"
#include <vector>
#include <sstream>
using namespace std;

///////////////////////////////////////////////////////////////////////////////
//Acquiring simulation title and option line from the input file 'input.asc'.
//Printing of title banner to screen
//
//Parameter output: *title, *options
//
//Parameter input: &nmc
//
//011128 Created by Peter H Zipfel
//020919 Added 'document_input()', PZi
///////////////////////////////////////////////////////////////////////////////

void acquire_title_options(fstream &input, char *run_title, char *options)
{ 
	char read[CHARN]={};
	char line_clear[CHARL]={};
	bool title_absent=true;
	int n(0);
	
	//read until 'OPTIONS' or if not encountered within 50 lines print error message
	do
	{
		n++;
		input>>read;
		if(ispunct(read[0])) input.getline(line_clear,CHARL,'\n');//throwaway line
		if (!strcmp(read,"TITLE")) // true if 'TITLE' is read
		{
			input.getline(run_title,CHARL,'\n');
			cout<<"\n"<<run_title<<"   "<< __DATE__ <<" "<< __TIME__ <<"\n";
			title_absent=false;
		}
	}while((strcmp(read,"OPTIONS"))&&(n<50));
	input.getline(options,CHARL,'\n');
	if(title_absent)
	{
		strcpy(run_title,"*** No Title found ***");
		cout<<"\n"<<run_title<<"   "<< __DATE__ <<" "<< __TIME__ <<"\n";

	}
	if(n==50) {cerr<<"*** Error: OPTIONS must be before MODULES*** \n";system("pause");exit(1);}
}

////////////////////////////////////////////////////////////////////////////////
//Acquiring the number of modules from the input file. 
//
//Parameter output: &num, number of modules, (call-by-reference)
//					&file_prt, pointer to first module in 'input.asc' 
//
//011128 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

void number_modules(fstream &input,int &num)
{
	char temp[CHARN]={};
	char line_clear[CHARL]={};
	num=0;
	int file_ptr=0;

	input>>temp;
	if (!strcmp(temp,"MODULES"))
	{
		input.getline(line_clear,CHARL,'\n');
		file_ptr=int(input.tellg());
		do
		{
			input>>temp;
			input.getline(line_clear,CHARL,'\n');
			num++;
		}while(strcmp(temp,"END"));
		num=num-1;
	}
	else
		cout<<"*** 'MODULES' must follow 'OPTIONS' line without comment lines *** \n";

	//reset file pointer to first module name
	input.seekg(file_ptr);
}
///////////////////////////////////////////////////////////////////
//Acquiring the ordering of the modules from the input file 'input.asc'.
//
//Argument output: *module_list, list of module names in the sequence of 'input.asc' 
//
//011128 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

void order_modules(fstream &input,int &num,Module *module_list)
{	
	string temp="";
	char module_type[CHARL]={};
	char line_clear[CHARL]={};


	for (int i=0;i<num;i++)		
	{
		input>>temp;
		module_list[i].name=temp;

		//reading the type of module functions present
		input.getline(module_type,CHARL,'\n');

		//initializing first
		module_list[i].definition="0";
		module_list[i].initialization="0";
		module_list[i].execution="0";
		module_list[i].termination="0";

		//loading the structure data
		if(strstr(module_type,"def"))module_list[i].definition="def";
		if(strstr(module_type,"init"))module_list[i].initialization="init";
		if(strstr(module_type,"exec"))module_list[i].execution="exec";
		if(strstr(module_type,"term"))module_list[i].termination="term";
	}
	input.getline(line_clear,CHARL,'\n');
}
///////////////////////////////////////////////////////////////////////////////
//Acquiring timing parameters
//
//Parameter output: plot_step, scrn_step, int_step
//
//010330 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////
void acquire_timing(fstream &input,double &plot_step,double &scrn_step,double &int_step)
{
	char temp[CHARN]={};
	char line_clear[CHARL]={};
	plot_step=0;
	scrn_step=0;
	int_step=0;

	input>>temp;
	if (!strcmp(temp,"TIMING"))
	{
		input.getline(line_clear,CHARL,'\n');
		do
		{
			input>>temp;
			if(!strcmp(temp,"plot_step"))input>>plot_step;
			if(!strcmp(temp,"scrn_step"))input>>scrn_step;
			if(!strcmp(temp,"int_step"))input>>int_step;
			input.getline(line_clear,CHARL,'\n');

		}while(strcmp(temp,"END"));
	}
	else
		cout<<"*** 'TIMING' must follow 'MODULES; NO blank lines between MODULES...END' ***\n";
}

///////////////////////////////////////////////////////////////////////////////
//Acquiring the number of vehicle objects from the input file 'input.asc'
//
//Parameter output: &num_vehicle, number of vehicles, (call-by-reference)
//					&num_plane, number of planes					
//
//010330 Created by Peter H Zipfel
//020920 Added check for illigal '=' signs and missing numerical entries, PZi
//070411 Added Aircraft, PZi
///////////////////////////////////////////////////////////////////////////////

void number_objects(fstream &input,int &num_vehicles,int &num_plane)
{
	char read[CHARN]={};
	char line_clear[CHARL]={};
	num_vehicles=0;
	num_plane=0;
	int file_ptr=0;
	char comment[3]="//";

	//reading number of total vehicle objects	
	input>>read;
	if (!strcmp(read,"VEHICLES")){
		input>>num_vehicles;		
		input.getline(line_clear,CHARL,'\n');
	}
	else
		cout<<"*** 'VEHICLES' must follow 'TIMING' ***\n";

	//searching for # of Plane objects
	//saving file pointer position
	file_ptr=int(input.tellg());

	do{
		input>>read;
		input.getline(line_clear,CHARL,'\n');
		if (!strcmp(read,"PLANE")) num_plane++;

	}while((num_plane)<num_vehicles);

	//resetting file pointer position
	input.seekg(file_ptr);

	//flagging illigal '=' signs
	int icount=0;
	while(!input.eof()){
		input>>read;
		if(strstr(read,comment)||!strcmp(read,"IF")){
			input.getline(line_clear,CHARL,'\n');
		}
		else{
			if(strstr(read,"=")){
				input.getline(line_clear,CHARL,'\n');
				icount++;
			}
		}
	}	

	//resetting file pointer position
	input.clear();
	input.seekg(file_ptr);

	//flagging missing numerical entries
	int vcount=0;
	while(!input.eof()){
		input>>read;
		if(strstr(read,comment)||isupper(read[0]))
			input.getline(line_clear,CHARL,'\n');
		else{
			input>>read;
			if(strstr(read,comment)){
				input.getline(line_clear,CHARL,'\n');
				vcount++;
			}
			else
				input.getline(line_clear,CHARL,'\n');
		}
	}
	if(icount){cout<<" *** Error: "<<icount<<" illigal '=' sign(s) found  in 'input.asc' ***\n";}
	if(vcount){cout<<" *** Error: "<<vcount<<" missing numerical value(s) in 'input.asc' ***\n";}
	if(icount||vcount){system("pause");exit(1);}

	//resetting file pointer position
	input.clear();
	input.seekg(file_ptr);
}

///////////////////////////////////////////////////////////////////////////////
//Allocating dynamic memory for an object by defining the appropriate pointer
//  
//Parameter output: *obj, type-of-vehicle pointer
//Arguments of object: module_list, num_modules, num_plane to be passed 
// to the constructorof 'Plane'
//Return output: type, type-of-vehicle as defined in 'input.asc'
//
//011128 Created by Peter H Zipfel
//070411 Added Aircraft, PZi
//231113 Deleted Secondary Aircraft, PZi
///////////////////////////////////////////////////////////////////////////////

Cadac *set_obj_type(fstream &input,Module *module_list,int num_modules,int num_plane)				   
{
	char line_clear[CHARL]={};
	char temp[CHARN]={};
	Cadac *obj=nullptr;
	int file_ptr=0;

	//diagnostic: file pointer
	file_ptr=int(input.tellg());

	//bypassing comment lines
	do{
		input>>temp;
		if(ispunct(temp[0]))
			input.getline(line_clear,CHARL,'\n');
	}while(ispunct(temp[0]));

	if (!strcmp(temp,"PLANE"))
	{
		//the pointer 'obj' is allocated the type 'Plane' 
		obj=new Plane(module_list,num_modules);
		if(obj==0){cerr<<"*** Error:'obj' allocation failed *** \n";system("pause");exit(1);}
		obj->set_name("PLANE");
	}
	return obj;
}

///////////////////////////////////////////////////////////////////////////////
//Acquiring the simulation ending time from 'input.asc'
//
//011128 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

double acquire_endtime(fstream &input)
{	
	double num(0);
	char read[CHARN]={};
	char line_clear[CHARL]={};
	int file_ptr=0;

	//resetting file pointer to beginning
	file_ptr=int(input.tellg()); //note: for test only
	input.seekg(ios::beg);
	file_ptr=int(input.tellg()); //note: for test only
	do
	{
		file_ptr=int(input.tellg()); //note: for test only

		input>>read;
		if(strcmp(read,"ENDTIME"))
			input.getline(line_clear,CHARL);
	}while(strcmp(read,"ENDTIME"));

	input>>num;

	return num;
}

///////////////////////////////////////////////////////////////////////////////
//Merging the 'ploti.asc' files onto 'plot.asc' 
//Compatible witch CADAC Studio plotting
//
//010117 Created by Peter H Zipfel
//011022 Changed treatment of plot_file_list;'num_vehicles'replaced by'num_missile',PZi
//020304 Correction in first while loop, PZi
///////////////////////////////////////////////////////////////////////////////
void merge_plot_files(string *plot_file_list,int num_plane, char *title)
{	
	char line_clear[CHARL]={};
	char line[CHARL]={};
	char buff[CHARN]={};
	int num_labels;
	int num_label_lines;
	int strip_lines;
	ifstream *file_istream_list;
	bool first_time=true;
	const char *name;
	int i(0);

	//Allocating memory for 'ploti.asc' file streams
	file_istream_list=new ifstream[num_plane];
	if(file_istream_list==0)
		{cerr<<"*** Error: file_istream_list[] allocation failed *** \n";system("pause");exit(1);}

	ofstream fmerge("plot.asc");

	for(i=0;i<num_plane;i++)
	{
		name=plot_file_list[i].c_str(); //conversion from 'string' to ' char' type
		file_istream_list[i].open(name);
	}

	//determining number of lines to be stripped of ploti.asc, i=1,2,3...
	ifstream fplot1("plot1.asc");
	fplot1.getline(line_clear,CHARL,'\n');
	fplot1>>buff;
	fplot1>>buff;
	fplot1>>num_labels;
	num_label_lines=num_labels/5;
	if(num_labels%5>0) num_label_lines=num_label_lines+1;
	strip_lines=num_label_lines+2;

	//copying first file 'plot1.asc' onto 'plot.asc'
	while(!file_istream_list[0].eof())
	{
		file_istream_list[0].getline(line,CHARL,'\n');
			if(first_time)
			{
				first_time=false;
				fmerge<<"1"<<title<<"  "<< __DATE__ <<" "<< __TIME__;
			}
			else
				//copy only non-blank lines
				if(file_istream_list[0].gcount()) fmerge<<'\n'<<line;
	}
	//copying remaining files onto 'plot.asc'
	for(i=1;i<num_plane;i++)
	{
		//discarding header lines
		int n=0;
		for(n=0;n<strip_lines;n++)
			file_istream_list[i].getline(line_clear,CHARL,'\n');

			//copying data lines
		while(!file_istream_list[i].eof())
		{
			file_istream_list[i].getline(line,CHARL,'\n');
			//copy only non-blank lines
			if(file_istream_list[i].gcount()) fmerge<<line<<'\n';
		}
	}
	//closing files and deallocating memory
	fmerge.close();
	for(i=0;i<num_plane;i++) file_istream_list[i].close();
	delete [] file_istream_list;
}

///////////////////////////////////////////////////////////////////////////////
///////////// Definition of Member functions of class 'Variable' //////////////
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
//Initialization of module-variables of type 'real'
//
// private class member output: name, rval, def, mod, role, out
//
//001128 Created by Peter H Zipfel
//020909 Added error code handling, PZi
///////////////////////////////////////////////////////////////////////////////
void Variable::init(const char *na,double rv,const char *de,const char *mo,const char *ro,const char *ou)
{

	if(!strcmp(name,"empty")==0) error[0]='*'; //if not 'empty', slot is illigally occupied
	strcpy(name,na);
	rval=rv;
	strcpy(def,de);
	strcpy(mod,mo);
	strcpy(role,ro);
	strcpy(out,ou);
}
///////////////////////////////////////////////////////////////////////////////
//Initialization of module-variables of type 'int'
//
// privat class member output: name, type, ival, def, mod, role, out
//
//001128 Created by Peter H Zipfel
//020909 Added error code handling, PZi
///////////////////////////////////////////////////////////////////////////////
void Variable::init(const char *na,const char *ty,int iv,const char *de,const char *mo,const char *ro,const char *ou)
{
	if(!strcmp(name,"empty")==0) error[0]='*';
	strcpy(name,na);
	strcpy(type,ty);
	ival=iv;
	strcpy(def,de);
	strcpy(mod,mo);
	strcpy(role,ro);
	strcpy(out,ou);
}
///////////////////////////////////////////////////////////////////////////////
//Initialization of module-variables of type 'Matrix' for 3x1 vectors
//
//private class member output: name, VEC, def, mod, role, out
//
//001128 Created by Peter H Zipfel
//020909 Added error code handling, PZi
///////////////////////////////////////////////////////////////////////////////
void Variable::init(const char *na,double v1,double v2,double v3,const char *de,const char *mo,const char *ro,const char *ou)
{
	double *pbody;
	pbody=VEC.get_pbody();
	*pbody=v1;
	*(pbody+1)=v2;
	*(pbody+2)=v3;

	if(!strcmp(name,"empty")==0) error[0]='*';
	strcpy(name,na);
	strcpy(def,de);
	strcpy(mod,mo);
	strcpy(role,ro);
	strcpy(out,ou);
}

///////////////////////////////////////////////////////////////////////////////
//Initialization of module-variables of type 'Matrix' for 3x3 matrices
//
//private class member output: name, MAT, def, mod, role, out
//
//001226 Created by Peter H Zipfel
//020104 Corrected element assigment errors, PZi
//020909 Added error code handling, PZi
///////////////////////////////////////////////////////////////////////////////

void Variable::init(const char *na,double v11,double v12,double v13,double v21,double v22,double v23,
					double v31,double v32,double v33,const char *de,const char *mo,const char *ro,const char *ou)
{
	double *pbody;
	pbody=MAT.get_pbody();
	*pbody=v11;
	*(pbody+1)=v12;
	*(pbody+2)=v13;
	*(pbody+3)=v21;
	*(pbody+4)=v22;
	*(pbody+5)=v23;
	*(pbody+6)=v31;
	*(pbody+7)=v32;
	*(pbody+8)=v33;

	if(!strcmp(name,"empty")==0) error[0]='*';
	strcpy(name,na);
	strcpy(def,de);
	strcpy(mod,mo);
	strcpy(role,ro);
	strcpy(out,ou);
}



///////////////////////////////////////////////////////////////////////////////
//Documenting 'input.asc' with module-variable definitions
// occurs if flag 'y_doc' is set
//
//020912 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////
void document_input(Document *doc_plane)
{
	 char buffl[CHARL]={};*buffl=NULL;
	 char buffn[CHARN]={};*buffn=NULL;
	 char buffo[CHARN]={};*buffo=NULL;
	 char numerical[CHARN]={};*numerical=NULL;
	 char line_clear[CHARL]={};*line_clear=NULL;
	bool ident=false;
	bool def_found=false;

	//opening existing input.asc file
	fstream input1("input.asc");
	if(!input1){cout<<" *** Error: cannot open 'input1.asc' file *** \n";system("pause");exit(1);}

	//opening new copy file
	fstream fcopy("input_copy.asc");
	if(!fcopy){cout<<" *** Error: cannot open 'input_copy.asc' file *** \n";system("pause");exit(1);}

	//copying 'input.asc' to 'input_copy.asc'
	do{
		input1.getline(buffl,CHARL,'\n');
		fcopy<<buffl<<'\n';
	}while(!input1.eof());

	//clear EOF flag in 'input.asc' file
	input1.clear();
	input1.close();

	//creating new output stream to file 'input.asc' and destroying all previous data
	ofstream input;
	input.open("input.asc",ios::out|ios::trunc);

	//reset file pointers to beginning
	fcopy.seekp(ios::beg);

	//copying from 'fcopy' stream to 'input' stream until 'VEHICLES' is reached
	do{
		fcopy.getline(buffl,CHARL,'\n');
		input<<buffl<<'\n';

	}while(!strstr(buffl,"VEHICLES"));

	//reading until STOP of input_copy file
	input.setf(ios::left);
	do{
		fcopy>>buffo;;//buffo gets PLANE, ENDTIME STOP and //comments


		if(!strcmp(buffo,"PLANE")){
			input<<'\t'<<buffo;		//writes PLANE line
			fcopy.getline(line_clear,CHARL,'\n');
			input<<line_clear<<endl;

			//inside PLANE loop until 'END' is reached
			do{
				fcopy>>buffn;
				//inserting whole line starting with key word IF
				if(!strcmp(buffn,"IF")){
					input<<"\t\t\t"<<buffn;
					fcopy.getline(line_clear,CHARL,'\n');
					input<<line_clear<<endl;
					//set ident
					ident=true;
				}
				//inserting whole line starting with key word ENDIF
				else if(!strcmp(buffn,"ENDIF")){
					input<<"\t\t\t"<<buffn;
					fcopy.getline(line_clear,CHARL,'\n');
					input<<line_clear<<endl;
					//set ident
					ident=false;
				}
				//reading and writing comment lines
				else if(ispunct(buffn[0])){
					//reading and writing comment lines
					if(ident)
						input<<"\t\t\t"<<buffn;
					else
						input<<"\t\t"<<buffn;
					fcopy.getline(line_clear,CHARL,'\n');
					input<<line_clear<<endl;
				}
				//inserting whole line starting with certain key words
				else if(!strcmp(buffn,"AERO_DECK")||!strcmp(buffn,"PROP_DECK")){
					input<<"\t\t\t"<<buffn;
					fcopy.getline(line_clear,CHARL,'\n');
					input<<line_clear<<endl;
				}
				//inserting 'END' with only one tab
				else if(!strcmp(buffn,"END")){
					input<<'\t'<<buffn;
					fcopy.getline(line_clear,CHARL,'\n');
					input<<line_clear<<endl;
				}
				else{
					//inserting module-variable name
					if(ident)
						input<<"\t\t\t\t"<<buffn;
					else
						input<<"\t\t\t"<<buffn;

					//jumping over numerical value
					fcopy>>numerical;
					input<<"  "<<numerical;

					//getting defintion for module-variable stored in 'buffn'
					def_found=false; 
					for(int k=0;k<(NFLAT3+NPLANE);k++){
						if(!strcmp(doc_plane[k].get_name(),"end_array"))break;
						if(!strcmp(doc_plane[k].get_name(),buffn)){
							fcopy.getline(line_clear,CHARL,'\n'); 							
							input<<"    //";
							if(!strcmp(doc_plane[k].get_type(),"int"))input<<"'int' ";
							input<<doc_plane[k].get_def();
							input<<"  module ";
							input<<doc_plane[k].get_mod();
							input<<endl;
							def_found=true;
						}
					}
					//if 'name' has no defintion clear line
					if(!def_found){
						fcopy.getline(line_clear,CHARL,'\n');
						input<<"   //*** <<< Check spelling"<<endl;;
					}
				}
			}while(strcmp(buffn,"END"));
		}//end of PLANE has been reached

		//inserting last three key words
		else{
			input<<buffo;
			fcopy.getline(line_clear,CHARL,'\n');
 			input<<line_clear<<endl;
		}
		*buffl=NULL;
		*buffn=NULL;
		*numerical=NULL;
		*line_clear=NULL;
	}while(strcmp(buffo,"STOP"));

	*buffo=NULL;
	input.clear();
	fcopy.clear();
	input.close();
	fcopy.close();  
}
///////////////////////////////////////////////////////////////////////////////
//Write output files 'plot' and 'traj' in 'comma separated variable' format
// with OPTION flag 'y_csv' set
//
//120703 Created by Keri Baily
//////////////////////////////////////////////////////////////////////////////
void parse_plot_traj_csv(string *files, int num_plane, bool merge, string type)
{
	for(int i=0; i<num_plane; i++){

		vector<string> text;
		ifstream ifs(files[i].c_str());
		string temp;
		int vars(0);

		while(getline(ifs,temp))
			text.push_back(temp);

		ifs.close();

		int num=i+1;
		char cnum[3];
		_itoa(num,cnum,10);
		string n(cnum);
		string file;
		ofstream csv_file;
		if(!merge){
			file="plot"+n+".csv";
			csv_file.open(file.c_str(),ios::trunc);
		}
		else{
			if(type=="plot")
				file="plot.csv";
			if(i==0){
				csv_file.open(file.c_str(),ios::trunc);
			}
			else
				csv_file.open(file.c_str(),ios::app);
		}
		vector<string>::iterator k;
		int j=0;
		int l=0;
		for(k=text.begin(); k!=text.end(); k++){
			if(j==0)
				csv_file<<*k<<endl;
			else if(j==1){
				vector<int> nums;

				istringstream iss((*k));
				do{
					string sub;
					iss >> sub;
					nums.push_back(atoi(sub.c_str()));
				} while(iss);

				vars = nums.at(2);
				csv_file<<*k<<endl;
			}
			else{
				const char * pch=strtok(const_cast<char*>((*k).c_str())," ");
				while(pch != NULL){
					csv_file<<pch<<",";
					pch=strtok(NULL," ");
				}
				if(((l-1)*5)/vars>=1){
					csv_file<<"\n";
					l=1;
				}
			}
			j++;
			l++;
		}
		csv_file.close();
	}	
	return;
}
