/*
 * testSVM.cpp

 *
 *  Created on: 17/06/2013
 *      Author: jplata
 */


#include "Detector.h"
#include "svm.h"
#include "FiltroParticulas.h"
#include <mrpt/utils/CTicTac.h>

#define Malloc(type,n) (type *)malloc((n)*sizeof(type))

using namespace mrpt::gui;



int main( int argc, const char* argv[] )
{

	vector<double> mx,my,px,py;

	CDisplayWindowPlots medidasPlot("Medidas");
	CDisplayWindowPlots clusterPlot("Cluster");
	CDisplayWindowPlots piernasPlot("Piernas");
	CDisplayWindowPlots personasPlot("Personas");

	vector<CPose2D>* puntos;
	CTicTac timer;

	Detector detector;

	Tracker seguidor;

	bool filtroInicializado=false;



	// Cargar modelo SVM
	struct svm_model *model=svm_load_model("svm_model");
	struct svm_node *instancia;
	double target;

	// datos de cada instancia, 4 en total: 3 carcateristicas más -1 indicando fin
	instancia=Malloc(struct svm_node,4);


	medidasPlot.hold_on();
	double limits[] = {0,2,-2,2};
	vector<double> limites (limits, limits + 4);



	for(int i=0;i<24;i++){

		timer.Tic();

		char nombre[100];

		sprintf(nombre,"/home/jplata/Eclipse/MedidasPiernas/12Julio/200ms/raw_laser%i.dat",i);
		cout << "Fichero:  " << nombre << endl;

		// Comprobar existencia del archivo
		FILE* file=fopen(nombre,"r");

		if(!file){
			cout << "¡¡¡¡Archivo no encontrado!!! Continuar con el siguiente" << endl;
			continue;
		}


		detector.abrirFichero(nombre,false);

		cout << "Tiempo apertura fichero (ms): " << timer.Tac()*1000 << endl;

		// Medidas
		puntos=detector.getPuntos();


		Eigen::MatrixXf rectas=detector.eliminarRectas(30,181);


		vector<Cluster> piernas=detector.clusterizar(0.1,3);;

		cout << "Tiempo hasta hough y clusterizar (ms): " << timer.Tac()*1000 << endl;

		mx.clear();
		my.clear();



		for(unsigned int i=0;i<puntos->size();i++){
			mx.push_back(puntos->at(i).x());
			my.push_back(puntos->at(i).y());
		}

		medidasPlot.clear();
		string fileName(nombre);
		medidasPlot.setWindowTitle("Medidas - " + fileName.substr(fileName.find_last_of("/")+1));
		medidasPlot.plot(mx,my,".b2");

		for(int j=0;j < rectas.rows();j++){

			Grafico::dibujarLinea(&medidasPlot,rectas(j,0),rectas(j,1),limites);

		}

		cout << "Tiempo hasta dibujar lineas (ms): " << timer.Tac()*1000 << endl;


		clusterPlot.clear();
		clusterPlot.setWindowTitle("Cluster - " + fileName.substr(fileName.find_last_of("/")+1));
		clusterPlot.hold_on();

		piernasPlot.clear();
		piernasPlot.setWindowTitle("Piernas - " + fileName.substr(fileName.find_last_of("/")+1));
		piernasPlot.hold_on();

		personasPlot.clear();
		personasPlot.setWindowTitle("Personas - " + fileName.substr(fileName.find_last_of("/")+1));
		personasPlot.hold_on();



		// Obtengo puntos clusters
		string formato[2];
		formato[0]=".r2";
		formato[1]=".b2";

		for(int j=0;j < piernas.size();j++){


			puntos=piernas[j].getPuntos();
			px.clear();
			py.clear();

			for(unsigned int k=0;k<puntos->size();k++){
				px.push_back(puntos->at(k).x());
				py.push_back(puntos->at(k).y());
			}
			clusterPlot.plot(px,py,formato[j%2]);

			// Determinar si es pierna o no
			instancia[0].index=1;
			instancia[1].index=2;
			instancia[2].index=3;
			instancia[3].index=-1;

			instancia[0].value=piernas[j].getContorno();
			instancia[1].value=piernas[j].getAncho();
			instancia[2].value=piernas[j].getProfundidad();

			target=svm_predict(model,instancia);

			if(target==1){
				// El clasificador SVM lo reconoce como pierna
				piernasPlot.plot(px,py,formato[j%2]);
			}
			else{
				// No es una pierna, lo elimino del vector
				piernas.erase(piernas.begin()+j);
				j--;
			}

		}

		cout << "Tiempo hasta clasificar piernas (ms): " << timer.Tac()*1000 << endl;

		vector<CPose2D> personas=detector.buscarPersonas(piernas);
		cout << "Personas detectadas: " << personas.size() << endl;

		cout << "Tiempo hasta buscar personas (ms): " << timer.Tac()*1000 << endl;

		detector.printClusters(piernas);
		px.clear();
		py.clear();
		for(int k=0;k < personas.size(); k++){
			px.push_back(personas[k].x());
			py.push_back(personas[k].y());
		}
		piernasPlot.plot(px,py,".c4");
		personasPlot.plot(px,py,".r4");


		// Filtro de Partículas
		if(!filtroInicializado){
			// Filtro no inicializado aún
			// Compruebo si se ha detectado persona
			if(!personas.empty()){
				seguidor.inicializar(50,personas[0]);
				filtroInicializado=true;
				seguidor.drawParticles(&personasPlot);
			}

		}
		else{
			CPose2D objetivo;
			// Filtro ya inicializado
			if(!personas.empty()){
				// No se ha detectado persona
				objetivo=seguidor.obtenerPosicionEstimada(CPose2D(),false);
			}
			else{
				objetivo=seguidor.obtenerPosicionEstimada(personas[0],true);
			}
			seguidor.drawParticles(&personasPlot);
			px.clear();
			py.clear();
			px.push_back(objetivo.x());
			py.push_back(objetivo.y());

			personasPlot.plot(px,py,".k8");
		}




		medidasPlot.axis(-0.5,3,-3,3);
		clusterPlot.axis(-0.5,3,-3,3);
		piernasPlot.axis(-0.5,3,-3,3);
		personasPlot.axis(-0.5,3,-3,3);



		cout << "Presione cualquier tecla para pasar a la siguiente muestra" << endl;

		mrpt::system::os::getch();

	}


	//print_vector("%f\t",ancho);



	//plot1.axis(-0.1,0.5,-0.1,0.5);
	//plot2.axis(-0.1,0.5,-0.1,0.5);


	cout << "Presione cualquier tecla para terminar:" << endl;

	mrpt::system::os::getch();


}





