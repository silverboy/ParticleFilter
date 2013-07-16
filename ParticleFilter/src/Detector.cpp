/*
 * Detector.cpp
 *
 *  Created on: 02/05/2013
 *      Author: jplata
 */

#include "Detector.h"

using namespace mrpt::poses;
using namespace mrpt::utils;

Cluster::Cluster(){
	num_puntos=0;
}


Cluster::Cluster(double contorno, double ancho,double profundidad):contorno(contorno),ancho(ancho),profundidad(profundidad){

}

void Cluster::annadirPunto(CPose2D punto){
	puntos.push_back(punto);
	num_puntos++;
}


void Cluster::clear(){
	puntos.clear();
	num_puntos=0;
}

int Cluster::getNumPuntos(){
	return num_puntos;
}

vector<CPose2D>* Cluster::getPuntos(){
	return &puntos;
}

void Cluster::print(){
	cout << "Numero de puntos: " << num_puntos << endl;
	cout << "Contorno: " << contorno << "\tAncho: " << ancho << "\tProfundidad: " << profundidad
			<<"\tCentro: ( " << centro.x() << " , " << centro.y() << " )" <<  endl;

	for(unsigned int i=0;i < puntos.size();i++){
		printf("X:%f\t Y:%f\n",puntos[i].x(),puntos[i].y());
	}

}

void Cluster::calcularAtributos(){

	// Contorno
	contorno=0;

	// Linea que une el primer y ultimo punto
	TLine2D linea(TPoint2D(puntos[0]),TPoint2D(puntos[getNumPuntos()-1]));

	double d(0),x,y;

	x=puntos.back().x();
	y=puntos.back().y();


	for(int i=0; i < puntos.size()-1; i++){

		contorno+=puntos[i].distanceTo(puntos[i+1]);
		x+=puntos[i].x();
		y+=puntos[i].y();

		if(i > 0){
			if(linea.distance(TPoint2D(puntos[i])) > d){
				d=linea.distance(TPoint2D(puntos[i]));
			}
		}
	}

	x=x/getNumPuntos();
	y=y/getNumPuntos();

	centro=CPose2D(x,y,0);

	// Profundidad
	profundidad=d;

	// Ancho
	ancho=puntos[0].distanceTo(puntos[getNumPuntos()-1]);

}

double Cluster::getAncho(){
	return ancho;
}

double Cluster::getProfundidad(){
	return profundidad;
}

double Cluster::getContorno(){
	return contorno;
}

CPose2D Cluster::getCentro(){
	return centro;
}


Detector::Detector() {
	// TODO Auto-generated constructor stub
	ASSERT_FILE_EXISTS_("../CONFIG_Measure.ini");

	CConfigFile config("../CONFIG_Measure.ini");

	max_dist=config.read_double("MEASURE_CONFIG","SAVE_MEASURE_MAX_DISTANCE",10,false);

}

Detector::Detector(double max_dist):max_dist(max_dist){
}

void Detector::setDistancias(vector<double> d){
	distancias=d;
}

void Detector::setPuntos(vector<CPose2D> p){
	puntos=p;
}

vector<CPose2D>* Detector::getPuntos(){
	return &puntos;
}

void Detector::abrirFichero(char* filename,bool filtrar_distancia){

	FILE* file=fopen(filename,"r");

	int i;
	double d,ang,x,y;

	distancias.clear();
	angulos.clear();
	puntos.clear();


	// Si se indica filtrar distancia solo se almacenan aquellas medidas cuya
	// distancia sea inferior a max_dist
	while(!feof(file)){
		fscanf(file,"i:%d\tAngulo:%lf\t Distancia:%lf\t X:%lf\t Y:%lf\n",&i,&ang,&d,&x,&y);
		if(!filtrar_distancia || d < max_dist){
			distancias.push_back(d);
			angulos.push_back(ang);
			puntos.push_back(CPose2D(x,y,DEG2RAD(ang)));
		}

	}

	fclose(file);

}



vector<Cluster> Detector::clusterizar(float umbral,int min_puntos){

	if(distancias.empty()){
		cout << "No hay datos cargados, llame antes a abrirFichero" << endl;
		return;
	}

	vector<Cluster> piernas;

	Cluster grupo;

	for(int i=0;i < distancias.size()- min_puntos + 1;i++){

		if(distancias[i] < max_dist){


			//Inicio cluster en el punto concreto
			grupo.annadirPunto(puntos[i]);


			// Avanzo por los siguientes puntos
			while(i < distancias.size()-1
					&& distancias[i+1] < max_dist
					&& puntos[i].distanceTo(puntos[i+1]) < umbral){
				// Añadir punto a cluster
				grupo.annadirPunto(puntos[i+1]);
				i++;
			}

			// Cluster terminado, compruebo si tiene el minimo de puntos
			if(grupo.getNumPuntos() >= min_puntos){
				grupo.calcularAtributos();
				piernas.push_back(grupo);
			}

			grupo.clear();

		}


	}

	return piernas;

}


void Detector::printClusters(vector<Cluster> piernas){

	cout << "Clusteres detectados: " << piernas.size() << endl;

	for(int i=0;i < piernas.size();i++){
		piernas[i].print();
	}

}

Eigen::MatrixXf Detector::eliminarRectas(int Np, int Ntheta){
	// Aplicamos transformada de Hough anotando
	// las rectas constituidas por un punto y el siguiente

	// p y theta indican cuantos intervalos de cuantizacion
	// para cada uno

	// Matriz para puntuar
	Eigen::MatrixXi scores(Np,Ntheta);
	Eigen::MatrixXf rectas(0,2);
	scores.setZero();
	double p,theta,A,B,C;
	int fila,columna;

	// Comienzo a puntuar

	// Puntos siguientes a considerar
	int siguientes=2;


	for(int i=0; i < puntos.size()-1; i++ ){

		int fin=min((int)puntos.size(),i+siguientes+1);

		for(int j=i+1;j<fin;j++){

			if(distancias[i] < max_dist && distancias [j] < max_dist){

				// Si ambos puntos se encuentran dentro del
				// rango de distancias considero la recta que los une

				TLine2D auxiliar(TPoint2D(puntos[i]),TPoint2D(puntos[j]));
				A=auxiliar.coefs[0];
				B=auxiliar.coefs[1];
				C=auxiliar.coefs[2];

				// Si C no es negativo invierto el signo de todos
				if(C > 0){
					A=-A;
					B=-B;
					C=-C;
				}

				theta=atan2(B,A);
				p=auxiliar.distance(TPoint2D(0,0));

				fila=floor(p*Np/max_dist);
				columna=floor(theta*Ntheta/(2*M_PI) + (double)Ntheta/2 );


				if(fila >= Np){
					cout << "Error en fila en transformada Hough" << endl;
					exit(0);
				}

				if(columna >= Ntheta){
					cout << "Error en columna en transformada Hough" << endl;
					exit(0);
				}

				scores(fila,columna)++;

			}
		} // Fin for j
	}//Fin for i


	for(int i=0;i < scores.rows();i++){
		for(int j=0;j < scores.cols();j++){

			if(scores(i,j) > 10){

				rectas.conservativeResize(rectas.rows()+1,Eigen::NoChange);
				p=i*max_dist/Np + max_dist/(2*Np);
				theta=j*2*M_PI/Ntheta - M_PI + M_PI/Ntheta;
				rectas(rectas.rows()-1,0)=p;
				rectas(rectas.rows()-1,1)=theta;

				// Filtro puntos pertenecientes a la recta
				filtrarPuntosEnRecta(TLine2D(cos(theta),sin(theta),-p),0.1);

			}
		}
	}




	return rectas;

}

/**
 * Funcion que busca personas y devuelve su posicion
 */
vector<CPose2D> Detector::buscarPersonas(vector<Cluster> piernas){

	vector<CPose2D> personas;

	// Iteramos en el vector Cluster
	for(int i=0; i < (signed int)piernas.size()-1 ; i++){
		// Seleccionado el primer elemento busco otro que se encuentre a una distancia razonable
		for(int j=i+1;j < piernas.size();j++){

			if(piernas[i].getCentro().distanceTo(piernas[j].getCentro()) < 1){
				// Considero que esos dos pùntos forman una persona
				CPose2D punto=piernas[i].getCentro()+piernas[j].getCentro();
				punto.x(punto.x()/2);
				punto.y(punto.y()/2);
				personas.push_back(punto);

				// Elimino ambos elementos del vector y rompo el bucle for
				piernas.erase(piernas.begin()+j);
				piernas.erase(piernas.begin()+i);
				i--;
				break;
			}
		}

	}
	return personas;

}


/**
 *
 * Funcion que establece a max_dist aquellos puntos que se encuentran a una distancia inferior
 *  a la indicada como argumento a la recta pasada como argumento
 *
 *
 */



void Detector::filtrarPuntosEnRecta(TLine2D recta,double distancia){

	for(int i=0;i < puntos.size(); i++){

		if(distancias[i] < max_dist && recta.distance(TPoint2D(puntos[i])) < distancia ){
			distancias[i]=max_dist;

		}

	}
}

void Detector::filtrarDatos(){
	// Eliminamos los datos 75 y 76 que son incorrectos en las medidas de entrenamiento desde el perfil 524 en adelante
	distancias.erase(distancias.begin()+75);
	distancias.erase(distancias.begin()+75);
	angulos.erase(angulos.begin()+75);
	angulos.erase(angulos.begin()+75);
	puntos.erase(puntos.begin()+75);
	puntos.erase(puntos.begin()+75);

}

void Grafico::dibujarLinea(CDisplayWindowPlots *winplot,  double p, double theta,vector<double> limites){

	double A(cos(theta)),B(sin(theta)),C(-p);

	vector<double> x(2);
	vector<double> y(2);

	if(limites.size() != 4){
		cout << limites.size() << endl;
		cout << "Error en dibujarLinea: limites erroneos, debe contener [xmin,xma,ymin,ymax]" << endl;
		exit(0);
	}



	if(abs(theta) < 3*M_PI/4 && abs(theta) > M_PI/4 ){

		// La linea es más horizontal que vertical

		//Caso especial: linea completamente horizontal
		if(abs(theta) == M_PI/2){
			x[0]=limites[0];
			x[1]=limites[1];
			y[0]=-C/B;
			y[1]=-C/B;
		}
		else{
			x[0]=limites[0];
			x[1]=limites[1];
			y[0]=(-A*limites[0]-C)/B;
			y[1]=(-A*limites[1]-C)/B;
		}
	}
	else{
		// La linea es más vertical que horizontal

		//Caso especial: linea completamente vertical
		if(abs(theta) == M_PI || theta == 0 ){
			x[0]=-C/A;
			x[1]=-C/A;
			y[0]=limites[2];
			y[1]=limites[3];
		}
		else{
			y[0]=limites[2];
			y[1]=limites[3];
			x[0]=(-B*limites[2]-C)/A;
			x[1]=(-B*limites[3]-C)/A;
		}
	}

	winplot->plot(x,y,"-g");

}

