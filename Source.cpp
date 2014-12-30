#include "stdafx.h"
#include <opencv2/imgproc/imgproc.hpp>  
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>  
#include <opencv2/opencv.hpp>
#include <time.h>
#include <iostream>

using namespace std;
using namespace cv;

#define _CRT_SECURE_NO_DEPRECATE

#define B(image,x,y) ((uchar*)(image->imageData + image->widthStep*(y)))[(x)*3]		//B
#define G(image,x,y) ((uchar*)(image->imageData + image->widthStep*(y)))[(x)*3+1]	//G
#define R(image,x,y) ((uchar*)(image->imageData + image->widthStep*(y)))[(x)*3+2]	//R
#define S(image,x,y) ((uchar*)(image->imageData + image->widthStep*(y)))[(x)]	
#define  FRAME_INTERVAL 10  
#define  T 40    //Tf
#define Re 30     //

#define R_BIN      8  // Histogram RED 
#define G_BIN      8  // Histogram GREEN 
#define B_BIN      8  // Histogram BLUE  

#define R_SHIFT    5  /* R,G,,B left shift values */
#define G_SHIFT    5  /* that corresponds to the above*/
#define B_SHIFT    5  /* log2( 256/8 ) as changing values */
#define IA 16807
#define IM 2147483647
#define AM (1.0/IM)
#define IQ 127773
#define IR 2836
#define MASK 123459876

const string ID_CAM_GO_PRO = "http://10.5.5.9:8080/live/amba.m3u8";
const int FRAME_WIDTH = 640;
const int FRAME_HEIGHT = 480;
# define ALPHA_COEFFICIENT      0.3     // refresh rate of target model bw 0.1  - 0.3 

//constants for calculating Z
const float ACTUAL_HEIGHT_IN_CM = 30;
const float STICKER_LENGTH_IN_CM = 7.2; 
const float STICKER_BREADTH_IN_CM = 4.0; 
const float STICKER_AREA_IN_CM2 = 28.8; 

typedef struct __SpaceState {  
	int xCoor;              
	int yCoor;              
	float v_xt;           // velocity in x direction 
	float v_yt;           // velocity in y direction 
	int Hxt;              // height in x direction 
	int Hyt;              // height in y direction 
	float at_dot;         // Scaling speed
} SPACESTATE;

bool isProgramPaused = false;
bool shouldTrackObject = false;
IplImage *curframe = NULL; 
IplImage *imgBackground = NULL;
IplImage *imgForeground = NULL;
IplImage *imgTrack = NULL;
unsigned char *img;//convert iplimg to char* for easier calculation
int xin, yin; //input coordinates centre of tracking
int xout, yout; //output coordinates of centre of tracking
int width, height; //of Windows
int widthInput,heightInput; 
int widthOutput,heightOutput; 

long ran_seed = 802163120; /* for random */
float DELTA_T = (float)0.05;    /* Frame rate，can be 30，25，15，10 etc */
int POSITION_DISTURB = 20; //15     /* disturbance of position   */
float VELOCITY_DISTURB = 40.0;  /* disturbance of velocity */
float SCALE_DISTURB = 0.0;      /* disturbance of windows width and height */
float SCALE_CHANGE_D = (float)0.001;   /* disturbance of scaling speed changes */

int NUM_PARTICLE = 2000;       // For particle filter   
float * modelHist = NULL; // Model of historgram 
SPACESTATE * states = NULL;  // array for states 
float * weights = NULL;   // weights of particles 
int nbin;                 // values for historgram 
float weightThreshold = (float)0.90; // weight threshold   
float MAX_WEIGHT_THRESHOLD = (float)0.0001;  // Maximum weight threshold，used to determine if target is lost

bool bSelectObject = false;
Point origin;
Rect selection;

# define SIGMA2       0.02           /* 2*sigma^2, here sigma = 0.1 */

/*
set random seed with the parameter specified or the system time.
*/
long setSeed( long setvalue ){
	if ( setvalue != 0 ) {
		ran_seed = setvalue;
	} else {
		ran_seed = time(NULL);
	}
	return ran_seed;
}

/*
Calculate the distribution of colour histogram within an image
Input parameters:
int x0, y0：           The center of the specified image region
int Wx, Hy：           the width and heighht of the target image
unsigned char * image：image data，scanning from left to right，up to down，
Color format：RGB, RGB, ...
(Or：YUV, YUV, ...)
int W, H：             Image width and height
Output parameters：
float * colorHist：    Color histogram，colour index：
i = r * G_BIN * B_BIN + g * B_BIN + b arrangement
int bins：             The values of the colour histogram R_BIN*G_BIN*B_BIN（It is 8x8x8=512 here）
*/
void calColorHistogram( int x0, int y0, int Wx, int Hy, unsigned char * image, int W, int H, float * colorHist, int bins ){
	// Assign 0 to the values of the histogram​
	for (int i = 0; i < bins; i++ ){    
		colorHist[i] = 0.0;
	}
	/* Consider the special scenario where x0, y0 are outside the image or Wx<=0, Hy<=0. 
	Set the colour histogram to be 0 in such cases. */
	if ( ( x0 < 0 ) || (x0 >= W) || ( y0 < 0 ) || ( y0 >= H ) || ( Wx <= 0 ) || ( Hy <= 0 ) ) {
		return;
	}
	// the coordinates of the upper left corner of the specified image area
	int x_begin = x0 - Wx;               /* Calculate the actual width and area starting point */
	int y_begin = y0 - Hy;
	if ( x_begin < 0 ) {
		x_begin = 0;
	}
	if ( y_begin < 0 ) {
		y_begin = 0;
	}
	int x_end = x0 + Wx;
	int y_end = y0 + Hy;
	if ( x_end >= W ) {
		x_end = W - 1;
	}
	if ( y_end >= H ) {
		y_end = H - 1;
	}
	int a2 = Wx * Wx + Hy * Hy;          // Calculate the radius squared kernel function a ^ 2 
	float f = 0.0;                         // Normalization factor 
	for (int y = y_begin; y <= y_end; y++ ){
		for (int x = x_begin; x <= x_end; x++ ){
			int r = image[(y * W + x)* 3] >> R_SHIFT;   // Computing the histogram values 
			int g = image[(y * W + x )* 3+1] >> G_SHIFT; // shift values are according to R G B values 
			int b = image[(y * W + x ) * 3+2] >> B_SHIFT;
			int index = r * G_BIN * B_BIN + g * B_BIN + b;
			float r2 = (float)(((y-y0) * (y-y0) + (x-x0) * (x-x0)) *1.0/ a2); // calculate radius squared r^2 
			float k = 1 - r2;   // kernel value k(r) = 1-r^2, |r| < 1; other values k(r) = 0 
			f = f + k;
			colorHist[index] = colorHist[index] + k;  // Calculate the nuclear density weighted color histogram 
		}
	}
	for ( int i = 0; i < bins; i++ ) {    // Normalize the Histogram 
		colorHist[i] = colorHist[i]/f;
	}
	return;
}

/*
Returns Bhattacharyya coefficient -a measure of the amount of overlap between two statistical samples or populations
Input parameters：
float * p, * q：      two colour histogram density estimation
int bins：            histogram values
*/
float calBhattacharyya(float * p, float * q, int bins){
	float rho = 0.0;
	for (int i = 0; i < bins; i++ ){
		rho = (float)(rho + sqrt( p[i]*q[i] ));
	}
	return rho ;
}

float calWeightedPi(float rho){
	float pi_n, d2;
	d2 = 1 - rho;
	pi_n = (float)(exp( - d2/SIGMA2 ));
	return pi_n;
}

/*
Generating a pseudo-random number between [0, 1] using Park and Miller method
*/
float ran0(long *idum){
	long k = (*idum) / IQ;            /* simple bit patterns for idum.                 */
	*idum = IA*( *idum - k * IQ) - IR * k;  /* Compute idum = (IA*idum) % IM without over- */
	if (*idum < 0) {
		*idum += IM;  /* flows by Schrage’s method.               */
	}
	float ans = AM * (*idum);          /* Convert idum to a floating result.            */
	return ans;
}

//Return a random floating number between 0, 1
float rand0_1(){
	return ran0(&ran_seed);
}

//Return a random value from a Gaussian distrbution x ~ N(u, sigma)
float randGaussian( float u, float sigma){
	float x1, x2, v1, v2, y;
	float s = 100.0;
	/*
	Use the screening method (Box-Mulles method) to produce a random number from a 
	normal distribution N~(0,1) 	
	1. Produce random variables X1, X2 between 0 and 1
	2. Calculate 
	V1= 2 * X1 - 1, 
	V2= 2 * X2 - 1, 
	s = V1^2 + V2^2
	3. If s <= 1, go to step 4 , otherwise return to step 1
	4. Calculate A = (-2ln(s)/s) ^ (1/2), y1 = V1 * A, y2 = V2 * A
	y1, y2 follows N ~ (0,1) random variable
	*/
	while (s > 1.0){
		x1 = rand0_1();
		x2 = rand0_1();
		v1 = 2 * x1 - 1;
		v2 = 2 * x2 - 1;
		s = v1 * v1 + v2 * v2;
	}
	y = (float)(sqrt( -2.0 * log(s)/s ) * v1);
	/*
	According to the formula
	z = (y - u)/ sqrt(sigma)
	normalize y variable into a standard normal distrbution Z ~ N(u, sigma)
	*/
	return( y - u )/ sqrt(sigma);	
}

/*
Inialize the system
int x0, y0：        initial coordinates
int Wx, Hy：        target width and height
unsigned char * img：	Image in RGB
int W, H：          image width and height
*/
int initialize(int x0, int y0, int Wx, int Hy, unsigned char * img, int W, int H ){
	float random[7];
	setSeed(0); /* for random */
	states = new SPACESTATE [NUM_PARTICLE]; // assign memory for statespace array 
	if (states == NULL) {
		return -2;
	}
	weights = new float [NUM_PARTICLE];     // assign memory for weight array
	if (weights == NULL) {
		return -3;	
	}
	nbin = R_BIN * G_BIN * B_BIN; /* Determine the values of histogram */
	modelHist = new float [nbin]; /* assign memory for histogram */
	if ( modelHist == NULL ) {
		return -1;
	}
	//Calculation of model target histogram
	calColorHistogram(x0, y0, Wx, Hy, img, W, H, modelHist, nbin);
	// Initialize particle state with x0, y0, 1, 1, Wx, Hy, 0.1	taking center of Normal Distribution to be N(0, 0.4)
	states[0].xCoor = x0;
	states[0].yCoor = y0;
	states[0].v_xt = (float) 0.0; 
	states[0].v_yt = (float) 0.0; 
	states[0].Hxt = Wx;
	states[0].Hyt = Hy;
	states[0].at_dot = (float) 0.0; 
	weights[0] = (float)(1.0/ NUM_PARTICLE); 
	for ( int i = 1; i < NUM_PARTICLE; i++ ) {
		for ( int j = 0; j < 7; j++ ) {
			random[j] = randGaussian( 0, (float)0.6 ); //Produce seven random Gaussian numbers 
		}
		states[i].xCoor = (int)( states[0].xCoor + random[0] * Wx );
		states[i].yCoor = (int)( states[0].yCoor + random[1] * Hy );
		states[i].v_xt = (float)( states[0].v_xt + random[2] * VELOCITY_DISTURB );
		states[i].v_yt = (float)( states[0].v_yt + random[3] * VELOCITY_DISTURB );
		states[i].Hxt = (int)( states[0].Hxt + random[4] * SCALE_DISTURB );
		states[i].Hyt = (int)( states[0].Hyt + random[5] * SCALE_DISTURB );
		states[i].at_dot = (float)( states[0].at_dot + random[6] * SCALE_CHANGE_D );
		// average weight is 1/N, because all particles  have equal probability
		weights[i] = (float)(1.0 / NUM_PARTICLE);
	}
	return 1;
}

/*
Calculating normalized cumulative probability c'_i
Modifies cumulatedWeight： an array with N+1 cumulated weights，
*/
void normalizeCumulatedWeight( float * weightedProb, float * cumulatedWeight, int size ){
	for ( int i = 0; i < size + 1; i++ ) 
		cumulatedWeight[i] = 0;
	for ( int i = 0; i < size; i++ )
		cumulatedWeight[i+1] = cumulatedWeight[i] + weightedProb[i];
	for ( int i = 0; i < size + 1; i++ )
		cumulatedWeight[i] = cumulatedWeight[i]/ cumulatedWeight[size];
	return;
}

/*
Use binary search to find smallest index j where nCumuWeight[j] <= fixedRandomNum
*/
int binarySearch(float fixedRandomNum, float * nCumuWeight, int size ){
	int leftIndex, rightIndex, middleIndex;
	leftIndex = 0; 	
	rightIndex = size-1;   
	while (rightIndex >= leftIndex){
		middleIndex = (leftIndex  + rightIndex)/2;
		if ( fixedRandomNum >= nCumuWeight[middleIndex] && fixedRandomNum < nCumuWeight[middleIndex+1]){
			return( middleIndex );
		}
		if ( fixedRandomNum < nCumuWeight[middleIndex] ) {
			rightIndex = middleIndex - 1;
		} else {
			leftIndex = middleIndex + 1;
		}
	}
	return 0;
}

/*
Redo sampleImportance
Input parameters:
float * c：          corresponding sample weight array pi (n) 
int N：              Weight array, number of elements in resampling index of array 
Puts indexes of resamples into array called resampleIndex
*/
void sampleImportance( float * c, int * resampleIndex, int N ){
	float rand_num, *cumulatedWeight;
	cumulatedWeight = new float [N + 1]; // assign memory for the cumulated weight array with size N+1 
	normalizeCumulatedWeight(c, cumulatedWeight, N); // calculate the cumulated weight 
	for (int i = 0; i < N; i++ ) {
		rand_num = rand0_1();     
		int j = binarySearch(rand_num, cumulatedWeight, N + 1); /* Find the smallest index j with value <=rnum */
		if (j == N) {
			j--;
		}
		resampleIndex[i] = j;	/* insert resampling index values */		
	}
	delete[] cumulatedWeight;
	return;	
}

/*
Reselect N new samples from N input samples based on their weights
Put values back into originalSampleSet
*/
void reselectParticles( SPACESTATE *originalSampleSet, float *originalWeights, int numSamples ){
	SPACESTATE *tempState = new SPACESTATE[numSamples];
	int *reselectIndex = new int[numSamples];
	sampleImportance( originalWeights, reselectIndex, numSamples ); // Resample according to weights 
	for (int i = 0; i < numSamples; i++ ){
		tempState[i] = originalSampleSet[reselectIndex[i]];
	}
	for (int i = 0; i < numSamples; i++ ){
		originalSampleSet[i] = tempState[i];
	}
	delete[] tempState;
	delete[] reselectIndex;
	return;
}

/*
Propagation: Predicts the system state values according to the system state equation.
The system equation is S(t) = A S(t-1) + W(t-1)
where W(t-1) stands for the Gaussian noise.
Puts predicted state into the state array after update.
*/
void propagate(SPACESTATE *state, int numStates ){
	float randomNum[7];
	/* Update every state within state[i] */
	for (int i = 0; i < numStates; i++ ){ /* Add a mean 0 random Gaussian noise */
		for (int j = 0; j < 7; j++ ) {
			randomNum[j] = randGaussian( 0, (float)0.6 ); /*Produce seven random Gaussian numbers */
		}
		state[i].xCoor = (int)(state[i].xCoor + state[i].v_xt * DELTA_T + randomNum[0] * state[i].Hxt + 0.5);
		state[i].yCoor = (int)(state[i].yCoor + state[i].v_yt * DELTA_T + randomNum[1] * state[i].Hyt + 0.5);
		state[i].v_xt = (float)(state[i].v_xt + randomNum[2] * VELOCITY_DISTURB);
		state[i].v_yt = (float)(state[i].v_yt + randomNum[3] * VELOCITY_DISTURB);
		state[i].Hxt = (int)(state[i].Hxt + state[i].Hxt * state[i].at_dot + randomNum[4] * SCALE_DISTURB + 0.5);
		state[i].Hyt = (int)(state[i].Hyt + state[i].Hyt * state[i].at_dot + randomNum[5] * SCALE_DISTURB + 0.5);
		state[i].at_dot = (float)(state[i].at_dot + randomNum[6] * SCALE_CHANGE_D);
		//cvCircle(imgTrack, Point(state[i].xCoor, state[i].yCoor), 3 , CV_RGB(255, 255, 0), 1, 4, 3 ); //to draw particle
	}
	return;
}

/*
Observe: Using the state set St of each sample and the observation from the histogram, 
update the estimation to get new weighted probabilities. Modifies the weight array directly
*/
void observe( SPACESTATE *state, float *weight, int size, unsigned char *image, int widthImg, int heightImg, float *objectHist, int hbins ){
	float *colorHist = new float[hbins];
	for (int i = 0; i < size; i++ ) {
		// 1. Calculates the distribution of colour histogram 
		calColorHistogram(state[i].xCoor, state[i].yCoor, state[i].Hxt, state[i].Hyt,
			image, widthImg, heightImg, colorHist, hbins );
		// 2. Calculates the Bhattacharyya coefficient 
		float rho = calBhattacharyya(colorHist, objectHist, hbins );
		// 3. Calculates each weight values based on the Bhattachharyya coefficient
		weight[i] = calWeightedPi(rho);	
	}
	delete[] colorHist;
	return;	
}

/*
Estimate (according to the weights) the values of the state to use as a tracking output 
*/
void estimate( SPACESTATE * state, float * weight, int size, SPACESTATE & EstState ){
	float at_dot, Hxt, Hyt, v_xt, v_yt, xCoor, yCoor, weight_sum;
	at_dot = 0;
	Hxt = 0; 
	Hyt = 0;
	v_xt = 0;	
	v_yt = 0;
	xCoor = 0;  	
	yCoor = 0;
	weight_sum = 0;
	// Summing
	for ( int i = 0; i < size; i++ ){ 
		at_dot += state[i].at_dot * weight[i];
		Hxt += state[i].Hxt * weight[i];
		Hyt += state[i].Hyt * weight[i];
		v_xt += state[i].v_xt * weight[i];
		v_yt += state[i].v_yt * weight[i];
		xCoor += state[i].xCoor * weight[i];
		yCoor += state[i].yCoor * weight[i];
		weight_sum += weight[i];
	}
	// Averaging 
	if ( weight_sum <= 0 ) {
		weight_sum = 1; /* avoid division by 0*/
	}
	EstState.at_dot = at_dot / weight_sum;
	EstState.Hxt = (int)(Hxt / weight_sum + 0.5 );
	EstState.Hyt = (int)(Hyt / weight_sum + 0.5 );
	EstState.v_xt = v_xt / weight_sum;
	EstState.v_yt = v_yt / weight_sum;
	EstState.xCoor = (int)(xCoor / weight_sum + 0.5 );
	EstState.yCoor = (int)(yCoor / weight_sum + 0.5 );
	return;
}

/************************************************************
updateModel
Input parameters：
SPACESTATE EstState：   state estimate
float * targetHist：    target Histogram
int bins：              value of Histogram
float PiT：             threshold（weight threshold）
unsigned char * img：   image in RGB
int width, height:		image width and height 
Output：
float * targetHist：    updated target histogram
************************************************************/

int updateModel( SPACESTATE EstState, float * targetHist, int bins, float PiT, unsigned char * img, int widthImg, int heightImg ) {
	float * estHist, bha, Pi_E;
	int rvalue = -1;
	estHist = new float [bins];
	// 1.Calculate the estimated value of the target histogram 
	calColorHistogram( EstState.xCoor, EstState.yCoor, EstState.Hxt, 
		EstState.Hyt, img, widthImg, heightImg, estHist, bins );
	// 2.calculate the Bhattacharyya coefficient 
	bha  = calBhattacharyya( estHist, targetHist, bins );
	// 3.Calculate the probability weights */
	Pi_E = calWeightedPi( bha );

	if ( Pi_E > PiT ) {
		for ( int i = 0; i < bins; i++ ){
			targetHist[i] = (float)((1.0 - ALPHA_COEFFICIENT) * targetHist[i] + ALPHA_COEFFICIENT * estHist[i]);
		}
		rvalue = 1;
	}
	delete[] estHist;
	return rvalue ;
}

/*
Function to clear memory when program exits
*/
void clearAll(){
	if (modelHist != NULL) {
		delete [] modelHist;
	}
	if (states != NULL) {
		delete [] states;
	}
	if (weights != NULL) {
		delete [] weights;
	}
	return;
}

int trackColorParticle( unsigned char *image, int widthImg, int heightImg, int &xCoor, int &yCoor, int &Wx_h, int &Hy_h, float &max_weight ) {
	SPACESTATE EstimatedState;
	reselectParticles( states, weights, NUM_PARTICLE );
	/* Sampling state equation to predict states variables or changes */
	propagate( states, NUM_PARTICLE);
	/* Observe and update the state and values */
	observe( states, weights, NUM_PARTICLE, image, widthImg, heightImg, modelHist, nbin );
	/* Estimate the position from the states values */
	estimate( states, weights, NUM_PARTICLE, EstimatedState );
	xCoor = EstimatedState.xCoor;
	yCoor = EstimatedState.yCoor;
	Wx_h = EstimatedState.Hxt;
	Hy_h = EstimatedState.Hyt;
	// update the model 
	updateModel( EstimatedState, modelHist, nbin, weightThreshold, image, widthImg, heightImg );
	// calculate the maximum weight 
	max_weight = weights[0];
	for ( int i = 1; i < NUM_PARTICLE; i++ ){
		max_weight = max_weight > weights[i] ?  max_weight: weights[i];
	}
	// Test for validity
	if ( xCoor < 0 || yCoor < 0 || xCoor >= widthImg || yCoor >= heightImg || Wx_h <= 0 || Hy_h <= 0 ) {
		return -1 ;
	} else {
		return 1 ;		
	}
}

string intToString(int number){
	std::stringstream ss;
	ss << number;
	return ss.str();
}

//Method to find the Z coordinates.
float findZCoor(float areaOfImageInPx){
	float scale = STICKER_LENGTH_IN_CM/ STICKER_BREADTH_IN_CM;
	float focalLength = sqrt (1196 / scale) / STICKER_LENGTH_IN_CM * ACTUAL_HEIGHT_IN_CM;
	if (areaOfImageInPx == 0){
		return 0;
	}
	float zCoor = focalLength / sqrt(areaOfImageInPx/scale) * STICKER_LENGTH_IN_CM;
	return zCoor;
}

//Converts an IplImage to img
void iplToImge(IplImage* src, int w,int h){
	for ( int j = 0; j < h; j++ ) { // Turn into a positive image
		for ( int i = 0; i < w; i++ ) {
			img[ (j * w + i ) * 3 ] = R(src,i,j);
			img[ (j * w + i ) * 3 + 1 ] = G(src,i,j);
			img[ (j * w + i ) * 3 + 2 ] = B(src,i,j);
		}
	}
}

//Allow user to select area of interest
void mouseHandler( int event, int x, int y, int flags, void* param){
	int centerX, centerY;
	if (bSelectObject){
		selection.x = MIN(x, origin.x);
		selection.y = MIN(y, origin.y);
		selection.width = abs(x - origin.x);
		selection.height = abs(y - origin.y);
	}
	switch(event){
	case CV_EVENT_LBUTTONDOWN:
		origin  = Point(x, y);
		selection = Rect(x, y, 0, 0);
		bSelectObject = true;
		isProgramPaused = false;
		break;
	case CV_EVENT_LBUTTONUP:
		bSelectObject = false;
		centerX = selection.x + selection.width/ 2.0;
		centerY = selection.y + selection.height/ 2.0;
		widthInput = selection.width / 2;
		heightInput = selection.height / 2;
		initialize(centerX, centerY, widthInput, heightInput, img, width, height );
		shouldTrackObject = true;
		break;
	}
}

//use some of the openCV drawing functions to draw crosshairs on our tracked image
void drawObject(int x, int y , int z){
	Scalar colorText = Scalar(0, 255, 0);
	cvCircle(imgTrack, Point(x, y), 20 , CV_RGB(0, 255, 255), 2, 8, 0 );
}

int main(int argc, char *argv[]){
	VideoCapture capture;
	if (argc == 1 || (argc == 2 && strlen(argv[1]) == 1 && isdigit(argv[1][0]))){
		capture.open(ID_CAM_GO_PRO);
		//capture.open(0);
	} else if( argc == 2 ){
		//capture = cvCaptureFromAVI( /*argv[1]*/"../13.avi");
		capture.open(0);
	}
	IplImage* frame[FRAME_INTERVAL]; //to store frames
	float rho_v;//represents similarity level
	float max_weight;
	for (int i = 0; i < FRAME_INTERVAL; i++){
		frame[i] = NULL;
	}
	Mat cameraFeed;
	int row, col;
	int star = 0;

	capture.set(CV_CAP_PROP_FRAME_WIDTH, FRAME_WIDTH);
	capture.set(CV_CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT);

	Mat distortion_coeff, camera_matrix;
	FileStorage fs("out_camera_data.xml", FileStorage::READ);
	fs["Distortion_Coefficients"] >> distortion_coeff;
	fs["Camera_Matrix"] >> camera_matrix;
	fs.release();
	Mat empty, newCameraMatrix, map1, map2, undistorted;
	capture.read(cameraFeed);
	Size imageSize = cameraFeed.size();
	initUndistortRectifyMap(camera_matrix, distortion_coeff, empty, newCameraMatrix, imageSize, CV_32FC1, map1, map2);

	while (1){
		Mat image(Size(FRAME_WIDTH, FRAME_HEIGHT), CV_8UC3);
		capture.read(cameraFeed);
		Mat temp = cameraFeed.clone();
		remap(temp, undistorted, map1, map2, INTER_LINEAR, 0, 0);
		curframe = new IplImage(undistorted);
		if (!star) {
			imgTrack = cvCreateImage(cvGetSize(curframe), IPL_DEPTH_8U, 3);
			row = curframe -> height;
			col = curframe -> width;
			width = curframe -> width;
			height = curframe -> height; 
			img = new unsigned char [width * height * 3];
			star = 1;
		}
		imgTrack = cvCloneImage(curframe);
		iplToImge(imgTrack, width, height);
		if (shouldTrackObject){
			//Tracking one frame
			rho_v = trackColorParticle( img, width, height, xout, yout, widthOutput, heightOutput, max_weight );
			/* Draw a blue box around the new position */
			//if ( rho_v > 0 && max_weight > 0.0001 ) { /* determines if target is lost */
			if (rho_v > 0 ){
				Point seed = Point(xout, yout);
				int loDiff = 50, upDiff = 55; //by trial and error
				int connectivity = 4;
				int isColor = true;
				bool useMask = false;
				int newMaskVal = 255;
				int ffillMode = 1;
				int lo = ffillMode == 0 ? 0 : loDiff;
				int up = ffillMode == 0 ? 0 : upDiff;
				int flags = connectivity + (newMaskVal << 8) + (ffillMode == 1 ? CV_FLOODFILL_FIXED_RANGE : 0);
				int b = (unsigned)theRNG() & 255;
				int g = (unsigned)theRNG() & 255;
				int r = (unsigned)theRNG() & 255;
				Rect ccomp;
				Scalar newVal = isColor ? Scalar(b, g, r) : Scalar(r*0.299 + g*0.587 + b*0.114);
				Mat dst = imgTrack;
				int area = floodFill(dst, seed, newVal, &ccomp, Scalar(lo, lo, lo), Scalar(up, up, up), flags);
				
				int zout = findZCoor(area);

				//Drawing rectangle based on stored xout and yout values.
				//We make the bounding rectangle bigger than actual size to do processing on it.
				int xTopLeft = xout - widthOutput;
				int xTopRight = xout + widthOutput;
				int yTopLeft =  yout - heightOutput;
				int yTopRight = yout + heightOutput;
				cvRectangle(imgTrack, cvPoint(xTopLeft, yTopLeft), cvPoint(xTopRight, yTopRight), cvScalar(255,0,0), 2, 8, 0);
				cv::Rect const mask(xTopLeft, yTopLeft, 2 * widthOutput, 2 * heightOutput);
				
				//cout << area << " pixels were repainted\n";

				drawObject(xout, yout, 0);
				CvFont font;
				double hScale=1.0, vScale=1.0;
				int    lineWidth=2;
				cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX|CV_FONT_ITALIC, hScale, vScale, 0, lineWidth);
				std::string s =  std::to_string(xout) + "," + std::to_string(yout) + "," + std::to_string(zout);
				char* c = new char[s.length() + 1];
				strcpy_s(c, 11, s.c_str());
				cvPutText (imgTrack, c , cvPoint(30, 30), &font, cvScalar(255,255,0));
				xin = xout; //for next frame
				yin = yout;
				widthInput = widthOutput; 
				heightInput = heightOutput;
			}
			//else
			//{
			//	cout<<"target lost"<<endl;
			//}

		}
		cvNamedWindow("Original Video", 1);
		cvNamedWindow("Tracking", 1);
		cvShowImage("Original Video",curframe);
		cvShowImage("Tracking", imgTrack);
		cvSetMouseCallback("Original Video", mouseHandler, 0);
		char c = cvWaitKey(10);
		switch (c) {
			//Press ESC to exit program
		case 27:
			return 0;
			//case 'm':
			//case 'M':
			//	shouldUseMorphOps = !shouldUseMorphOps;
			//	if (shouldUseMorphOps) {
			//		std::cout << MSG_MORPHING_ENABLED << std::endl;
			//	} else {
			//		std::cout << MSG_MORPHING_DISABLED << std::endl;
			//	}
			//	break;
			//Press P to pause or resume the code
		case 'p': 
		case 'P':
			isProgramPaused = !isProgramPaused;
			if (isProgramPaused == true){ 
				std::cout << "Paused" << std::endl;
				while (isProgramPaused == true){
					//stay in this loop forever until unpaused or exited.
					char c = cvWaitKey();
					if (c == 112){
						isProgramPaused = false;
						std::cout << "Resumed" << std::endl;
						break;
					}
					if (c == 27){
						return 0 ;
					}
				}
			}
		case 't': 
		case 'T':
			shouldTrackObject = !shouldTrackObject;
			if (shouldTrackObject){
				std::cout << "Tracking enabled" << std::endl;
			} else {
				std::cout << "Tracking disabled" << std::endl;
			}
			break;
		default:
			continue;
		}
	}
	cvReleaseImage(&imgBackground);
	cvReleaseImage(&imgForeground);
	cvDestroyAllWindows();
	clearAll();
	return 0;
}