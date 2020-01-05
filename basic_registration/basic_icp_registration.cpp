#include "stdafx.h"
#include <stdio.h>
#include "Matrix.h"
#include "ANN.h"
#include <windows.h>


typedef float *TabFloat;

int main(int argc, char* argv[])
{
	// initialisation des fichiers
	FILE *in1 = fopen("../../visu/3D.samples/foot/foot.pts.neu", "r"); // to rename files
	FILE *in2 = fopen("../../visu/3D.samples/foot/foot_perturbed.pts.neu", "r");

	FILE *out1 = fopen("../../visu/3D.samples/foot/result.neu", "w");
	FILE *out2 = fopen("../../visu/3D.samples/foot/error.txt", "w");

	FILE *out3 = fopen("../../visu/3D.samples/foot/transformation.txt", "w");
	FILE *out4 = fopen("../../visu/3D.samples/foot/tempsdexecution.txt", "w");
	// ******************************************************************
	/* Variables : compteurs*/
	int i, j, iteration = 0;
	float x, y, z;
	int n = 3, NbrPt1 = 0, NbrPt2 = 0;

	/* Variables de temps d'ex¨¦cution*/
	LARGE_INTEGER start, end, freq;
	double elapsed;

	/*tableaux de points*/
	TabFloat *Point1, *Point2, *Voisin;
	//-- Nos variables --
	//transformation T(R,t)
	Matrix R(n, n, false); // Matrice de rotation 
	Vector t(3); //vecteur de translation 

	//Calcul de la SVD {H.ComputeSVD(U,W,V);}
	Matrix H(n, n, false); //Matrice 	
	Matrix U(3, 3, false);
	Matrix W(3, 3, false);
	Matrix V(3, 3, false);

	int NbrPtC = 0; // nombre des points des fichiers
	int  NbrPtV = 0; //nombre des points des fichiers
	float seuil = 500; // le seuil ¨¤ utilis¨¦ par ICP 
	float erreur = 1000; // l'¨¦cart type moyen (~1000) valeur par d¨¦faut
	float max; //utilis¨¦ pour la distance maximale entre 2 points (distance cart¨¦sienne)
	float min; //utilis¨¦e pour la distance minimale entre 2 points (distance cart¨¦sienne)
	int k; // compteur des it¨¦ration d'ICP
	int l; // indices utilisable pour la programmation
	float* PointG1 = new float[3]; //centre de gravit¨¦ nuage 1
	float* PointG2 = new float[3]; //centre de gravit¨¦ nuage 2
	//=========================structure of kd-tree=======================================(start)
	ANNkd_tree* kd_tree;
	int num = 50;                 // number of nearest neighbors
	int dim = 3;               // dimension
	ANNpointArray dataPts;     // data points
	ANNpoint queryPt;          // query point
	ANNidxArray nnIdx;         // near neighbor indices
	ANNdistArray dists;        // near neighbor distances
	double eps = 0;
	//=========================structure of kd-tree=======================================(end)

	// *********************** INITIALISATION *******************************************
	//--Init : centroide
	for (i = 0; i < 3; i++)
	{
		PointG1[i] = 0.;
		PointG2[i] = 0.;
	}
	//-- Init : nombre points : lecture pour comptage des points 
	while (fscanf(in1, "%f %f %f", &x, &y, &z) != EOF)
		NbrPt1++;
	while (fscanf(in2, "%f %f %f", &x, &y, &z) != EOF)
		NbrPt2++;
	//-- Init allocation memoire :  allocations des tableaux des deux ensembles de points
	Point1 = new TabFloat[NbrPt1];
	for (i = 0; i < NbrPt1; i++)
		Point1[i] = new float[3];

	Point2 = new TabFloat[NbrPt2];
	for (i = 0; i < NbrPt2; i++) {
		Point2[i] = new float[3];
	}

	queryPt = annAllocPt(dim); // allocate query point
	dataPts = annAllocPts(NbrPt2, dim); // allocate data points
	nnIdx = new ANNidx[num]; // allocate near neigh indices
	dists = new ANNdist[num]; // allocate near neighbor dists
	
	



	//Init : retour au d¨¦but du fichier
	rewind(in1);
	rewind(in2);

	//Init :  lecture pour la mise m¨¦moire	
	for (i = 0; i < NbrPt1; i++) {
		fscanf(in1, "%f %f %f", &Point1[i][0], &Point1[i][1], &Point1[i][2]);
		queryPt[0] = (ANNcoord)Point1[1][0];
		queryPt[1] = (ANNcoord)Point1[1][1];
		queryPt[2] = (ANNcoord)Point1[1][2];
	}


	for (i = 0; i < NbrPt2; i++) {
		fscanf(in2, "%f %f %f", &Point2[i][0], &Point2[i][1], &Point2[i][2]);
		dataPts[i][0] = (ANNcoord)Point2[i][0];
		dataPts[i][1] = (ANNcoord)Point2[i][1];
		dataPts[i][2] = (ANNcoord)Point2[i][2];

		//cout << "kd-tree point cord:" << i << ": " << dataPts[i][0] << "point-cloud point cord" << Point2[i][0] << endl;
		//cout << "kd-tree point cord:" << i << ": " << dataPts[i][1] << "point-cloud point cord" << Point2[i][1] << endl;
		//cout << "kd-tree point cord:" << i << ": " << dataPts[i][2] << "point-cloud point cord" << Point2[i][2] << endl;
		//cout << "============================================================================================" << endl;
	}
	//Init :  tableau du plus proche voisin
	Voisin = new TabFloat[NbrPt1];
	for (i = 0; i < NbrPt1; i++)
	{
		Voisin[i] = new float[2];
		Voisin[i][1] = (float)i;
	}
	NbrPtC = NbrPt2;
	k = 0, l = 0;
	

	// ************************** ITERATION ICP ****************************************	
	// Boucle d'iteration selon l'erreur ou/et le nombre d'it¨¦ration
	while ((erreur > 0.1) && (k < 100))
	{
		//=================================CREAT KD-TREE=====================================

		kd_tree = new ANNkd_tree(dataPts, NbrPt2, dim);


		//kd_tree->annkFRSearch(queryPt, seuil, 100, nnIdx, dists, 0.0);
		//for (int i = 0; i < num; i++) {			// print summary
		//	dists[num-i-1] = sqrt(dists[num - i - 1]);			// unsquare distance
		//	cout << "\t" << i << "\t" << nnIdx[num - i - 1] << "\t" << dists[num - i - 1] << "\n";
		//}

		//kd_tree->Print(true,);

		//=================================CREAT KD-TREE=====================================



		/* D¨¦but du temps d'ex¨¦cution de l'it¨¦ration*/
		QueryPerformanceFrequency(&freq);
		/* Lancement de la mesure     */
		QueryPerformanceCounter(&start);
		/* Affichage du compteur K    */
		printf("******* iteration %d *******\n", k); k++;
		//*************************************************************************************
		//*** STEP 0 : Calcul du seuil en fonction de la distance max oubien fixation en dur
		//*************************************************************************************

			/*max = PT_PT_DIST(Point1[0],Point2[0]);
			for (i=0; i<NbrPt1; i++)
				{
				for (j=1; j<NbrPt2; j++)
					{
					if (max< PT_PT_DIST(Point1[i],Point2[j]))
							max= PT_PT_DIST(Point1[i],Point2[j]);
					}
				}
			seuil=max/20;*/

		//seuil = 500;
		

		//*************************************************************************************
		//*** STEP 1 : Recherche des plus proches voisins :
		//*************************************************************************************
	  
		
		for (i = 0; i < NbrPt1; i++)
		{
			//int count = 0;
			min = PT_PT_DIST(Point1[i], Point2[0]);
			Voisin[i][1] = 0;
			for (j = 1; j < NbrPt2; j++)
			{
				if (min > PT_PT_DIST(Point1[i], Point2[j]))
				{
					min = PT_PT_DIST(Point1[i], Point2[j]);
					Voisin[i][1] = (float)j;
					//count++;
				}
			}
			if (min > seuil)
				Voisin[i][1] = -1;
			else
				Voisin[i][0] = min;
			//cout <<Voisin[i][1] <<endl;
			//cout << count << endl;
		}
		
		/*
		for (i = 0; i < NbrPt1; i++) {
			min = PT_PT_DIST(Point1[i], Point2[0]);
			Voisin[i][1] = -1;
			queryPt[0] = (ANNcoord)Point1[i][0];
			queryPt[1] = (ANNcoord)Point1[i][1];
			queryPt[2] = (ANNcoord)Point1[i][2];
			//kd_tree->annkFRSearch(queryPt, seuil ,num, nnIdx, dists, 0.0);
			kd_tree->annkSearch(queryPt, num, nnIdx, dists, 0.0);
			for (int j = 0; j < num; j++) {
				dists[num - j - 1] = sqrt(dists[num - j - 1]);
				//cout << "\t" << i << "\t" << nnIdx[i] << "\t" << dists[i] << "\n";
				if (min > dists[num - j - 1]) {
					min = dists[num - j - 1];
					Voisin[i][1] = (float)nnIdx[num - j - 1];
				}
			}
			//if (min > seuil)
				//Voisin[i][1] = -1;
			//else
				Voisin[i][0] = min;

			//cout << Voisin[i][1] <<endl;
		}*/
		

		//*************************************************************************************
		//*** STEP 2: CALCUL DE LA TRANSFORMEE :
		//*************************************************************************************
		//-- STEP 2.1 : Recherche des centres de masse :
		Vector pm1(3);
		Vector pm2(3);
		int N = 0;
		Vector somme1(3);
		Vector somme2(3);
		for (i = 0; i < NbrPt1; i++)
		{
			if (Voisin[i][1] != -1)
			{
				N++;
				somme1.operator +=(Point1[i]);
				l = (int)Voisin[i][1];
				somme2.operator +=(Point2[l]);
			}
		}
		//cout << "Step 2.1 somme1.x:" << somme1[0] << "  somme2.x:" << somme2[0] << endl;
		//cout << "Step 2.1 somme1.y:" << somme1[1] << "  somme2.y:" << somme2[1] << endl;
		//cout << "Step 2.1 somme1.z:" << somme1[2] << "  somme2.z:" << somme2[2] << endl;
		cout << "N:" << N << endl;
		pm1 = somme1 / N;
		pm2 = somme2 / N;


		Vector q1(3);
		Vector q2(3);
		//-- STEP 2.2 : Calcul de H
		H.SetToZero();
		for (i = 0; i < NbrPt1; i++)
		{
			if (Voisin[i][1] != -1)
			{
				q1[0] = Point1[i][0] - pm1[0];
				q1[1] = Point1[i][1] - pm1[1];
				q1[2] = Point1[i][2] - pm1[2];
				l = (int)Voisin[i][1];
				q2[0] = Point2[l][0] - pm2[0];
				q2[1] = Point2[l][1] - pm2[1];
				q2[2] = Point2[l][2] - pm2[2];
				H = H + abT(q2, q1);
			}
		}
		//-- STEP 2.3 : Estimation de la transformation rigide (R,t)
		H.ComputeSVD(U, W, V);
		R = V * Transpose(U);
		Vector t(3);
		t = pm1 - R * pm2;
		//*************************************************************************************
		//*** STEP 3: APPLICATION DE LA TRANSFORMEE :
		//*************************************************************************************
		//--STEP 3.1 Application de la transformation :	
		Vector tampon(3);
		Vector tempPoint2(3);

		for (i = 0; i < NbrPt2; i++)
		{
			tempPoint2[0] = Point2[i][0];
			tempPoint2[1] = Point2[i][1];
			tempPoint2[2] = Point2[i][2];

			tampon = R * tempPoint2;

			for (j = 0; j < 3; j++) {
				Point2[i][j] = tampon[j] + t[j];
				dataPts[i][j] = tampon[j] + t[j];
			}
		}

		//--STEP 3.2 fin de temps d'ex¨¦cution 
		/* Arret de la mesure         */
		QueryPerformanceCounter(&end);
		/* Conversion en millsecondes */
		elapsed = (1000.0 * (end.QuadPart - start.QuadPart)) / freq.QuadPart;
		printf("Tps ex¨¦cution it¨¦raration : %d : %.0f millisecondes entre start et end.\n", k, elapsed);
		fprintf(out4, "%.0f\n", elapsed);
		//*************************************************************************************
		//*** STEP 4: CALCUL DE L'ERREUR ET AFFICHAGE DES RESULTATS :
		//*************************************************************************************
		erreur = 0;
		for (i = 0; i < NbrPt1; i++)
		{
			if (Voisin[i][1] != -1)
			{
				l = (int)Voisin[i][1];
				erreur = erreur + SQR(PT_PT_DIST(Point1[i], Point2[l]));
			}
		}
		erreur = erreur / N;
		cout << erreur << endl;
		fprintf(out2, "%f \n", erreur);
		fprintf(out3, "It¨¦ration %d\nT : %f %f %f\n", k, t[0], t[1], t[2]);
		fprintf(out3, "R : \n");

		for (i = 0; i < 3; i++)
			fprintf(out3, "%f %f %f\n\n", R(i, 0), R(i, 1), R(i, 2));
	}//fin boucle while
	// ******************************************************************
	// sauvegarde du fichier
	printf("Sauvegarde du fichier\n");
	for (i = 0; i < NbrPt2; i++)
		fprintf(out1, "%f %f %f\n", Point2[i][0], Point2[i][1], Point2[i][2]);

	printf("Sauvegarde du fichier\n");

	fclose(in1);
	fclose(in2);
	fclose(out1);
	fclose(out2);
	fclose(out3);
	fclose(out4);
	return 0;
}