/*
 * evaluarPF.cpp

 *
 *  Created on: 17/06/2013
 *      Author: jplata
 */


#include "FiltroParticulas.h"
#include <mrpt/utils/CTicTac.h>


using namespace mrpt::gui;
using namespace mrpt::utils;

double calcularError(CPose2D prediccion,vector<CPose2D> trayectoria);



int main( int argc, const char* argv[] )
{

	// Leer parametros de entrada

	if(argc != 2){
		cout << "Uso: EvaluarTracking numero_trayectoria" << endl;
		exit(1);
	}

	// Trayectoria a evaluar
	int trayectoria=atoi(argv[1]);


	CDisplayWindowPlots winPlot("Filtro de Particulas");
	winPlot.hold_on();



	vector<CPose2D> t_p;
	vector<double> t_x,t_y,x,y,errores,time;
	CTicTac timer;

	double px,py;
	CPose2D persona;


	FILE *input,*output,*trayect;


	bool filtroInicializado;

	char nombre[100];

	// Leemos trayectoria
	//sprintf(nombre,"/home/jplata/Eclipse/MedidasPiernas/17Julio/trayectorias_bk/trayectoria%i.dat",trayectoria);
	sprintf(nombre,"/home/jplata/Eclipse/MedidasPiernas/17Julio/trayect_interp/t%i_interp.dat",trayectoria);
	trayect=fopen(nombre,"r");

	while(!feof(trayect)){

		fscanf(trayect,"x:%lf,y:%lf\n",&px,&py);
		t_x.push_back(px);
		t_y.push_back(py);
		t_p.push_back(CPose2D(px,py,0));

	}
	fclose(trayect);

	sprintf(nombre,"/home/jplata/Eclipse/MedidasPiernas/17Julio/T%i/Errores.dat",trayectoria);
	// Fichero para almacenar errores
	//output=fopen(nombre,"w");

	// Repetir para distinto numero de particulas

	//int particulas[]={50,100,150,200,250};
	int particulas[]={200};

	for(int j=0;j < sizeof(particulas)/sizeof(int); j++){

		errores.clear();

		int fin=24;

		if(trayectoria == 7){
			fin=20;
		}


		// Comprobamos error del tracker con las trayectorias almacenadas

		for(int i=0;i<fin;i++){

			sprintf(nombre,"/home/jplata/Eclipse/MedidasPiernas/17Julio/T%i/t%i_%i.dat",trayectoria,trayectoria,i);

			// Crear filtro particulas
			filtroInicializado=false;
			Tracker seguidor;

			// Comprobar existencia del archivo
			input=fopen(nombre,"r");

			// Si es de 100ms cambio la estadistica
			if(i >=18 || trayectoria == 7){
				seguidor.setGaussianParams(0.07,0.03,1);
			}



			while(!feof(input)){

				timer.Tic();

				// Obtengo posicion de la persona
				fscanf(input,"x:%lf,y:%lf\n",&px,&py);
				persona=CPose2D(px,py,0);

				winPlot.clear();
				winPlot.plot(t_x,t_y,".r1");



				// Filtro de Partículas
				if(!filtroInicializado){
					// Filtro no inicializado aún
					// Compruebo si se ha detectado persona
					if(px != -5){
						seguidor.inicializar(particulas[j],persona);
						filtroInicializado=true;
						seguidor.drawParticles(&winPlot);
						x.clear();
						y.clear();
						x.push_back(px);
						y.push_back(py);
						winPlot.plot(x,y,".c4");
					}

				}
				else{
					CPose2D objetivo;
					// Filtro ya inicializado
					if(px == -5){
						// No se ha detectado persona
						objetivo=seguidor.obtenerPosicionEstimada(CPose2D(),false);
					}
					else{
						objetivo=seguidor.obtenerPosicionEstimada(persona,true);
						x.clear();
						y.clear();
						x.push_back(px);
						y.push_back(py);
						winPlot.plot(x,y,".c4");
					}

					//fprintf(output,"%f,",calcularError(objetivo,t_p));
					errores.push_back(calcularError(objetivo,t_p));

					seguidor.drawParticles(&winPlot);
					x.clear();
					y.clear();
					x.push_back(objetivo.x());
					y.push_back(objetivo.y());

					winPlot.plot(x,y,".k8");

					time.push_back(timer.Tac()*1000);
				}
			}

			//fprintf(output,"\n");
		}


		double media=0;
		// Calcular error medio
		for(int i=0;i < errores.size(); i++){
			media+=errores[i];
		}
		media=media/errores.size();


		double std=0;
		for(int i=0;i < errores.size(); i++){
			std+=square(errores[i]-media);
		}
		std=sqrt(std/(errores.size()-1));

		double t_medio=0;
		// Calcular error medio
		for(int i=0;i < time.size(); i++){
			t_medio+=time[i];
		}
		t_medio=t_medio/time.size();



		cout << "Particulas: " << particulas[j] << endl;
		cout << "Error medio: " << media << endl;
		cout << "Std error: " << std << endl;
		cout << "Tiempo medio: " << t_medio << endl;

		//fprintf(output,"%i,%0.5f,%0.5f\n",particulas[j],media,std);

	}

	//fclose(output);


	cout << "Presione cualquier tecla para terminar:" << endl;

	mrpt::system::os::getch();

}




double calcularError(CPose2D prediccion,vector<CPose2D> trayectoria){

	if( trayectoria.empty()){

		cout << "Vector trayectoria vacio" << endl;
		exit(0);
	}

	double error=prediccion.distanceTo(trayectoria[0]);
	double d;

	for(int i=1;i < trayectoria.size(); i++){
		d=prediccion.distanceTo(trayectoria[i]);

		if(d < error){
			error=d;
		}
	}

	return error;
}


