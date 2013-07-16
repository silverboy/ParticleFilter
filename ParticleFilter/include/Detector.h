/*
 * Detector.h
 *
 *  Created on: 02/05/2013
 *      Author: jplata
 */

#ifndef DETECTOR_H_
#define DETECTOR_H_

//#include <math.h>
#include <stdio.h>
#include <vector>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/gui/CDisplayWindowPlots.h>
#include <mrpt/utils/CConfigFile.h>
#include <mrpt/system/filesystem.h>
#include <Eigen/StdVector>



using namespace std;
using namespace mrpt::poses;
using namespace mrpt::gui;

EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(CPose2D);

class Cluster {
public:
	Cluster();
	Cluster(double contorno, double ancho,double profundidad);
	void annadirPunto(CPose2D);
	void clear();
	int getNumPuntos();
	vector<CPose2D>* getPuntos();
	void print();
	void calcularAtributos();
	double getAncho();
	double getProfundidad();
	double getContorno();
	CPose2D getCentro();

private:
	int num_puntos;
	vector<CPose2D> puntos;
	double contorno;
	double ancho;
	double profundidad;
	CPose2D centro;
};

EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Cluster);




class Detector {
public:
	Detector();
	Detector(double max_dist);
	void setDistancias(vector<double> d);
	void setPuntos(vector<CPose2D> p);
	vector<CPose2D>* getPuntos();
	void abrirFichero(char* filename,bool filtrar_distancia);
	vector<Cluster> clusterizar(float umbral,int min_puntos);
	vector<CPose2D> buscarPersonas(vector<Cluster> piernas);

	void printClusters(vector<Cluster> piernas);
	void filtrarDatos();

	//Function
	Eigen::MatrixXf eliminarRectas(int Np,int Ntheta);
	void filtrarPuntosEnRecta(TLine2D recta,double distancia);


private:
	// Fields
	double max_dist;
	vector<double> distancias;
	vector<double> angulos;
	vector<CPose2D> puntos;


};

class Grafico{
public:
	static void dibujarLinea(CDisplayWindowPlots *plot, double p, double theta,vector<double> limites);

};

#endif /* DETECTOR_H_ */
